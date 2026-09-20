#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <execution>
#include <filesystem>
#include <fstream>

#include "common/log.h"
#include "common/options.h"
#include "core/lightning_math.hpp"
#include "laser_mapping.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

namespace {
/// 单调时钟当前时刻（秒）。时间戳（雷达时间）会跳变，不能用来算耗时差。
double NowSteadySec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

LaserMapping *LaserMapping::obs_model_this_ = nullptr;

bool LaserMapping::Init(const std::string &config_yaml) {
    LLOG_INFO(logging::kLio, "init laser mapping from {}", config_yaml);
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ikdtree_ = std::make_shared<KDTreeType>();
    ikdtree_->set_downsample_param(filter_size_map_min_);

    // esekfom init
    obs_model_this_ = this;
    double epsi[23];
    std::fill(epsi, epsi + 23, 0.001);
    kf_.init_dyn_share(fastlio::get_f, fastlio::df_dx, fastlio::df_dw, &LaserMapping::ObsModelFastLio,
                       fasterlio::NUM_MAX_ITERATIONS, epsi);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_scan;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        fasterlio::NUM_MAX_ITERATIONS = yaml["fasterlio"]["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = yaml["fasterlio"]["esti_plane_threshold"].as<float>();

        filter_size_scan = yaml["fasterlio"]["filter_size_scan"].as<float>();
        filter_size_map_min_ = yaml["fasterlio"]["filter_size_map"].as<float>();
        /// 关键帧点云的存储分辨率，缺省 0（存原始扫描，保持旧行为）
        kf_cloud_res_ = yaml["fasterlio"]["kf_cloud_res"] ? yaml["fasterlio"]["kf_cloud_res"].as<float>() : 0.0f;
        keep_first_imu_estimation_ = yaml["fasterlio"]["keep_first_imu_estimation"].as<bool>();
        gyr_cov = yaml["fasterlio"]["gyr_cov"].as<float>();
        acc_cov = yaml["fasterlio"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["fasterlio"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["fasterlio"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
        lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

        extrinT_ = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();

        use_aa_ = yaml["fasterlio"]["use_aa"].as<bool>();
        if (yaml["fasterlio"]["cube_len"]) {
            cube_len_ = yaml["fasterlio"]["cube_len"].as<double>();
        }

        skip_lidar_num_ = yaml["fasterlio"]["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        float height_max = yaml["roi"]["height_max"].as<float>();
        float height_min = yaml["roi"]["height_min"].as<float>();

        preprocess_->SetHeightROI(height_max, height_min);

        options_.kf_dis_th_ = yaml["fasterlio"]["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = yaml["fasterlio"]["kf_angle_th"].as<double>() * M_PI / 180.0;
        options_.enable_icp_part_ = yaml["fasterlio"]["enable_icp_part"].as<bool>();
        options_.min_pts = yaml["fasterlio"]["min_pts"].as<int>();
        options_.plane_icp_weight_ = yaml["fasterlio"]["plane_icp_weight"].as<float>();

        options_.proj_kfs_ = yaml["fasterlio"]["proj_kfs"].as<bool>();

    } catch (...) {
        LLOG_ERROR(logging::kLio, "bad conversion");
        return false;
    }

    LLOG_INFO(logging::kLio, "lidar_type {}", lidar_type);
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LLOG_INFO(logging::kLio, "Using AVIA Lidar");
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LLOG_INFO(logging::kLio, "Using Velodyne 32 Lidar");
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LLOG_INFO(logging::kLio, "Using OUST 64 Lidar");
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LLOG_INFO(logging::kLio, "Using RoboSense Lidar");
    } else if (lidar_type == 5) {
        preprocess_->SetLidarType(LidarType::SIMULATED);
        LLOG_INFO(logging::kLio, "Using Simulated Lidar (instantaneous scan, no per-point time)");
    } else {
        LLOG_ERROR(logging::kLio, "unknown lidar_type");
        return false;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    p_imu_->set_extrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->set_gyr_cov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->set_acc_cov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->set_gyr_bias_cov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->set_acc_bias_cov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new fastlio::ImuProcess());

    /// 常开里程计日志（与 fast_lio odom_log.txt 同格式，便于对比）
    std::filesystem::create_directories("./data");
    odom_log_.open("./data/odom_log.txt", std::ios::out | std::ios::trunc);
    if (odom_log_.is_open()) {
        odom_log_ << "# time pos_x pos_y pos_z q_w q_x q_y q_z roll pitch yaw vel_x vel_y vel_z\n";
    }

    /// 常开位姿日志（同 fast_lio dump_lio_state_to_log 格式）
    pos_log_.open("./data/pos_log.txt", std::ios::out | std::ios::trunc);
    if (pos_log_.is_open()) {
        pos_log_ << "# time rot_x rot_y rot_z pos_x pos_y pos_z omega_x omega_y omega_z "
                    "vel_x vel_y vel_z acc_x acc_y acc_z bg_x bg_y bg_z ba_x ba_y ba_z "
                    "grav_x grav_y grav_z\n";
    }
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr &imu) {
    const double timestamp = imu->timestamp;

    /// 入队阶段：只持 buffer 锁，且只做搬运（deque 操作 + 时间戳记录），必须极短。
    /// 不得在持锁期间调用 UI / StateToNavState()，否则会与 Run() 的锁序（state -> buffer）反向而死锁。
    {
        UL lock(mtx_buffer_);
        publish_count_++;

        if (timestamp < last_timestamp_imu_) {
            LLOG_WARN_THROTTLE(logging::kLio, 2000, "imu loop back, clear buffer");
            imu_buffer_.clear();
        }

        last_timestamp_imu_ = timestamp;
        imu_buffer_.emplace_back(imu);
    }

    /// UI 更新需要读 EKF 状态，放在 buffer 锁之外；
    /// 用非阻塞版本：LIO 线程正在计算时直接跳过本帧 UI 刷新，绝不阻塞 IMU 入队。
    if (ui_) {
        NavState ns = GetIMUStateNonBlocking();
        if (ns.pose_is_ok_) {
            ui_->UpdateNavState(ns);
        }
    }
}

bool LaserMapping::Run() {
    /// Run() 全程独占算法状态（EKF / ikd-Tree / 关键帧 / scan_*）。
    /// SyncPackages() 内部只会短暂获取 mtx_buffer_，满足锁序 state -> buffer。
    std::lock_guard<std::mutex> lock_state(mtx_state_);

    if (!SyncPackages()) {
        LLOG_WARN_THROTTLE(logging::kLio, 2000, "sync package failed");
        return false;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LLOG_WARN_THROTTLE(logging::kLio, 2000, "No point, skip this scan!");
        return false;
    }

    /// the first scan
    if (flg_first_scan_) {
        LLOG_INFO(logging::kLio, "first scan pts: {}", scan_undistort_->size());

        state_point_ = kf_.get_x();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ikdtree_->Build(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            /// 更新UI中的内容
            if (ui_) {
                NavState ns = StateToNavState();
                ui_->UpdateNavState(ns);
                // scan 在 LIDAR 系：先乘 lidar->IMU 外参转到 IMU/body 系，pose 保持 IMU 位姿
                // （不把外参混进 pose，避免 backend 车/轨迹坐标系被污染）
                Eigen::Matrix4f T_ext = Eigen::Matrix4f::Identity();
                T_ext.block<3, 3>(0, 0) = offset_R_lidar_fixed_.cast<float>();
                T_ext.block<3, 1>(0, 3) = offset_t_lidar_fixed_.cast<float>();
                CloudPtr ui_cloud(new PointCloudType);
                pcl::transformPointCloud(*scan_undistort_, *ui_cloud, T_ext);
                ui_->UpdateScan(ui_cloud, ns.GetPose());
            }

            return false;
        }
    }

    LLOG_DEBUG(logging::kLio, "LIO get cloud at beg: {:.14f}, end: {:.14f}", measures_.lidar_begin_time_,
               measures_.lidar_end_time_);

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LLOG_WARN_THROTTLE(logging::kLio, 2000, "检测到雷达断流，时长：{:.3f}",
                           (measures_.lidar_begin_time_ - last_lidar_time_));
    }

    last_lidar_time_ = measures_.lidar_begin_time_;

    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    /// 移动/裁剪局部地图（ikd-Tree）
    LasermapFOVSegment();

    /// downsample
    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    // if (options_.proj_kfs_) {
    //     ProjectKFs();
    // }

    int cur_pts = scan_down_body_->size();

    if (cur_pts < (scan_undistort_->size() * 0.1) || cur_pts < options_.min_pts) {
        /// 降采样太狠了,有效点数不够，用0.1分辨率代替
        // LOG(INFO) << "too few points, using 0.1 resol";
        auto v = voxel_scan_;
        v.setLeafSize(0.1, 0.1, 0.1);
        v.setInputCloud(scan_undistort_);
        v.filter(*scan_down_body_);

        // LOG(INFO) << "Now pts: " << scan_down_body_->size() << ", before: " << cur_pts;
        cur_pts = scan_down_body_->size();
    }

    if (cur_pts < 5) {
        LLOG_WARN_THROTTLE(logging::kLio, 2000, "Too few points, skip this scan! {}, {}", scan_undistort_->size(),
                           scan_down_body_->size());
        return false;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    point_search_sq_dis_.resize(cur_pts);

    // 成员变量预分配
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    NavState pred_state = StateToNavState();

    /// esekfom 迭代更新（内部多次调用 ObsModelFastLio 做点面配准）
    double solve_H_time = 0;
    kf_.update_iterated_dyn_share_modified(0.001, solve_H_time);

    state_point_ = kf_.get_x();
    state_timestamp_ = measures_.lidar_end_time_;
    NavState cur_state = StateToNavState();

    const double delta_translation = (pred_state.pos_ - cur_state.pos_).norm();
    const double delta_rotation_deg = (pred_state.rot_.inverse() * cur_state.rot_).log().norm() * 180.0 / M_PI;
    const double delta_velocity = (pred_state.vel_ - cur_state.vel_).norm();

    const double current_speed = cur_state.vel_.norm();

    LLOG_DEBUG(logging::kLio, "In num: {}, down: {}, map grid num: {}, effect num: {}", scan_undistort_->points.size(),
               cur_pts, ikdtree_->validnum(), effect_feat_surf_);
    LLOG_DEBUG(logging::kLio, "delta trans: {}, ang: {}", (pred_state.pos_ - cur_state.pos_).transpose(),
               delta_rotation_deg);

    /// keyframes
    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = cur_state.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        } else if ((last_pose.so3().inverse() * cur_pose.so3()).log().norm() > 1.0 * M_PI / 180.0) {
            // MapIncremental();
        }
    }

    if (ui_) {
        // scan_down_body_ 在 LIDAR 系：先乘 lidar->IMU 外参转到 IMU/body 系，pose 保持 IMU 位姿
        // （不把外参混进 pose，避免 backend 车/轨迹坐标系被污染）
        Eigen::Matrix4f T_ext = Eigen::Matrix4f::Identity();
        T_ext.block<3, 3>(0, 0) = offset_R_lidar_fixed_.cast<float>();
        T_ext.block<3, 1>(0, 3) = offset_t_lidar_fixed_.cast<float>();
        CloudPtr ui_cloud(new PointCloudType);
        pcl::transformPointCloud(*scan_down_body_, *ui_cloud, T_ext);
        ui_->UpdateScan(ui_cloud, cur_state.GetPose());
        ui_->UpdateNavState(cur_state);
    }

    LLOG_DEBUG(logging::kLio, "LIO state: {}, yaw {:.3f}, vel: {}, grav: {}, grav norm: {:.4f}",
               cur_state.pos_.transpose(), cur_state.rot_.angleZ<double>() * 180 / M_PI, cur_state.vel_.transpose(),
               cur_state.grav_.transpose(), cur_state.grav_.norm());

    /// 常开里程计日志（与 fast_lio odom_log.txt 同格式，便于对比；rpy 单位为度）
    if (odom_log_.is_open()) {
        Eigen::Quaterniond q = cur_state.rot_.unit_quaternion();
        Eigen::Vector3d e = cur_state.rot_.matrix().eulerAngles(2, 1, 0) * 180.0 / M_PI;  // (yaw, pitch, roll) deg
        odom_log_ << std::setprecision(6) << (state_timestamp_ - first_lidar_time_) << " " << cur_state.pos_(0) << " "
                  << cur_state.pos_(1) << " " << cur_state.pos_(2) << " " << q.w() << " " << q.x() << " " << q.y()
                  << " " << q.z() << " " << e(2) << " " << e(1) << " " << e(0) << " " << cur_state.vel_(0) << " "
                  << cur_state.vel_(1) << " " << cur_state.vel_(2) << "\n";
    }

    /// 常开位姿日志 pos_log.txt（同 fast_lio dump_lio_state_to_log 格式；rot 单位为弧度）
    if (pos_log_.is_open()) {
        Eigen::Quaterniond qr(state_point_.rot.toRotationMatrix());
        Eigen::AngleAxisd aa(qr);
        Eigen::Vector3d rot_ang = aa.angle() * aa.axis();
        pos_log_ << std::setprecision(6) << (state_timestamp_ - first_lidar_time_) << " " << rot_ang.transpose() << " "
                 << state_point_.pos.transpose() << " "
                 << "0 0 0 " << state_point_.vel.transpose() << " "
                 << "0 0 0 " << state_point_.bg.transpose() << " " << state_point_.ba.transpose() << " "
                 << state_point_.grav[0] << " " << state_point_.grav[1] << " " << state_point_.grav[2] << "\n";
    }

    return true;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    NavState state = StateToNavState();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    for (auto kf : proj_kfs_) {
        // LOG(INFO) << "projecting kf: " << kf->GetID();
        // if (last_kf_) {
        // auto kf = last_kf_;
        SE3 pose = pose_cur * kf->GetLIOPose();

        int cnt = 0;
        for (auto &pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;

            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;

            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
        // }
    }
}

void LaserMapping::MakeKF() {
    NavState cur_state = StateToNavState();

    // 深拷贝点云：scan_undistort_ 是每帧复用的成员（Process 内 *pcl_out = *meas.scan_ 原地覆盖）。
    // 若直接共享指针，所有关键帧的 GetCloud() 最终都指向最后一帧的点云，导致
    // GetGlobalMap / UI UpdateKF / 回环等基于关键帧云的功能全部错乱（地图结构错误）。
    //
    // kf_cloud_res_ > 0 时在这里就把体素降采样做掉再存（见 kf_cloud_res_ 的说明）。
    // 为什么值得：scan_undistort_ 是未降采样的原始扫描（point_filter_num 只抽点，约 14400 点/帧），
    //   存原图 = ① 每帧 ~450KB，跑 1687 帧就是 **~760MB**；
    //            ② 每次 GetGlobalMap 都要把这 14400 点重新滤一遍 —— 实测占存图总耗时 66%。
    //   而降采样花的是同一份计算量（0.36ms/帧 → 摊到全程只有 ~0.2% CPU），
    //   只是把它从“存图那一下”摊到了“每建一个关键帧”。
    CloudPtr kf_cloud;
    if (kf_cloud_res_ > 0) {
        /// 注意用局部 VoxelGrid：成员 voxel_scan_ 的 leaf size 是 filter_size_scan，
        /// 归 Run() 里的 scan 降采样用 —— 动它会把 LIO 前端的降采样分辨率一起带偏。
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(kf_cloud_res_, kf_cloud_res_, kf_cloud_res_);
        voxel.setInputCloud(scan_undistort_);
        kf_cloud = std::make_shared<PointCloudType>();
        voxel.filter(*kf_cloud);
    } else {
        kf_cloud = std::make_shared<PointCloudType>(*scan_undistort_);
    }
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, kf_cloud, cur_state, kf_cloud_res_);

    if (last_kf_) {
        /// opt pose 用之前的递推
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(cur_state);

    LLOG_INFO(logging::kLio, "LIO: create kf {}, state: {}, kf opt pose: {}, lio pose: {}, time: {:.14f}", kf->GetID(),
              cur_state.pos_.transpose(), kf->GetOptPose().translation().transpose(),
              kf->GetLIOPose().translation().transpose(), cur_state.timestamp_);

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    // 有keyframes时更新local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    /// 更新project kfs
    if (proj_kfs_.size() >= options_.max_proj_kfs_) {
        auto last = proj_kfs_.back();

        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() < 3 || delta.so3().log().norm() < 20 / 180 * M_PI) {
            // proj_kfs_.pop_back();
        } else {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
    /// 注意：preprocess_ 是共享成员，本函数只允许在 ROS executor 线程上串行调用，不得并发。
    /// 重计算（预处理）放在 buffer 锁之外，避免长时间占锁把 IMU 入队一并堵住。
    Timer::Evaluate(
        [&, this]() {
            const double timestamp = ToSec(msg->header.stamp);

            double last_imu_ts = 0.0;
            {
                /// 入队前检查：只读时间戳，持锁时间极短
                UL lock(mtx_buffer_);
                if (timestamp < last_timestamp_lidar_) {
                    LLOG_ERROR_THROTTLE(logging::kLio, 1000, "lidar loop back, dt: {:.14f}",
                                        timestamp - last_timestamp_lidar_);
                    return;
                }
                last_imu_ts = last_timestamp_imu_;
            }

            LLOG_DEBUG(logging::kLio, "get cloud at {:.14f}, latest imu: {:.14f}", timestamp, last_imu_ts);

            /// 重计算：锁外
            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            /// 搬运：只做入队
            UL lock(mtx_buffer_);
            scan_count_++;
            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            arrival_time_buffer_.push_back(NowSteadySec());
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg) {
    /// 同 PointCloud2 版本：重计算在 buffer 锁外，锁内只做搬运
    Timer::Evaluate(
        [&, this]() {
            const double timestamp = ToSec(msg->header.stamp);

            double last_imu_ts = 0.0;
            {
                UL lock(mtx_buffer_);
                if (timestamp < last_timestamp_lidar_) {
                    LLOG_ERROR_THROTTLE(logging::kLio, 1000, "lidar loop back, clear buffer");
                    lidar_buffer_.clear();
                    time_buffer_.clear();  // 并行队列必须一起清，否则时间/到达时刻错位
                    arrival_time_buffer_.clear();
                }
                last_imu_ts = last_timestamp_imu_;
            }

            LLOG_DEBUG(logging::kLio, "get cloud at {:.14f}, latest imu: {:.14f}", timestamp, last_imu_ts);

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            UL lock(mtx_buffer_);
            scan_count_++;
            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            arrival_time_buffer_.push_back(NowSteadySec());
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    /// 已预处理过的点云：无重计算，锁内只做搬运
    Timer::Evaluate(
        [&, this]() {
            const double timestamp = math::ToSec(cloud->header.stamp);

            UL lock(mtx_buffer_);
            scan_count_++;

            if (timestamp < last_timestamp_lidar_) {
                LLOG_ERROR_THROTTLE(logging::kLio, 1000, "lidar loop back, clear buffer");
                lidar_buffer_.clear();
                time_buffer_.clear();  // 并行队列必须一起清，否则时间/到达时刻错位
                arrival_time_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            arrival_time_buffer_.push_back(NowSteadySec());
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    /// 输入队列搬运。调用方 Run() 已持 mtx_state_，这里再取 mtx_buffer_，符合锁序 state -> buffer。
    UL lock(mtx_buffer_);

    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        LLOG_DEBUG(logging::kLio, "lidar or imu is empty");
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LLOG_WARN_THROTTLE(logging::kLio, 2000, "Too few input point cloud!");
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);

            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                /// timestamp 有异常
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_ - 0.005) {
        LLOG_DEBUG(logging::kLio, "sync failed: {:.14f}, {:.14f}", last_timestamp_imu_, lidar_end_time_);
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());

        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();

    /// 记录“刚被消费的这帧”的到达时刻，供上游算该帧自己的 e2e。
    /// 注意不能在 pop 前清空队列为空的情况：lidar_pushed_ 为真时队列可能已经先被 pop 过。
    if (!arrival_time_buffer_.empty()) {
        last_processed_arrival_time_.store(arrival_time_buffer_.front());
        arrival_time_buffer_.pop_front();
    }

    lidar_pushed_ = false;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);  // FIXME 这并发可能有点问题
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ikdtree_->Add_Points(points_to_add, true);
            ikdtree_->Add_Points(point_no_need_downsample, false);
        },
        "    ikd-Tree Add Points");
}
void LaserMapping::LasermapFOVSegment() {
    /// 当前雷达在世界系下的位置
    Vec3d pos_lid = state_point_.pos + state_point_.rot.toRotationMatrix() * offset_t_lidar_fixed_;

    constexpr double MOV_THRESHOLD = 1.5;
    constexpr double DET_RANGE = 300.0;

    if (!local_map_initialized_) {
        for (int i = 0; i < 3; i++) {
            local_map_points_.vertex_min[i] = pos_lid(i) - cube_len_ / 2.0;
            local_map_points_.vertex_max[i] = pos_lid(i) + cube_len_ / 2.0;
        }
        local_map_initialized_ = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_lid(i) - local_map_points_.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lid(i) - local_map_points_.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
    }

    if (!need_move) {
        return;
    }

    BoxPointType new_local_map_points, tmp_boxpoints;
    new_local_map_points = local_map_points_;

    double mov_dist =
        std::max((cube_len_ - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, DET_RANGE * (MOV_THRESHOLD - 1));

    std::vector<BoxPointType> cub_needrm;

    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = local_map_points_;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            new_local_map_points.vertex_max[i] -= mov_dist;
            new_local_map_points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            new_local_map_points.vertex_max[i] += mov_dist;
            new_local_map_points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }

    local_map_points_ = new_local_map_points;

    if (!cub_needrm.empty()) {
        Timer::Evaluate([&, this]() { ikdtree_->Delete_Point_Boxes(cub_needrm); }, "    ikd-Tree FOV Delete");
    }
}
/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModelFastLio(fastlio::state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    LaserMapping *self = obs_model_this_;
    if (self == nullptr || self->scan_down_body_->empty()) {
        ekfom_data.valid = false;
        return;
    }

    int cnt_pts = self->scan_down_body_->size();
    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < cnt_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
        PointType &point_body = self->scan_down_body_->points[i];
        PointType &point_world = self->scan_down_world_->points[i];

        /* transform to world frame */
        Vec3d p_global = s.rot * (s.offset_R_L_I * point_body.getVector3fMap().cast<double>() + s.offset_T_L_I) + s.pos;
        point_world.getVector3fMap() = p_global.cast<float>();
        point_world.intensity = point_body.intensity;

        auto &points_near = self->nearest_points_[i];
        points_near.clear();

        /** Find the closest surfaces in the map **/
        self->ikdtree_->Nearest_Search(point_world, fasterlio::NUM_MATCH_POINTS, points_near,
                                       self->point_search_sq_dis_[i]);
        self->point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;

        /// 能找到3个点以上，则估计平面
        if (self->point_selected_surf_[i]) {
            self->point_selected_surf_[i] =
                math::esti_plane(self->plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
        }

        /// 计算平面阈值
        if (self->point_selected_surf_[i]) {
            auto temp = point_world.getVector4fMap();
            temp[3] = 1.0;
            float pd2 = self->plane_coef_[i].dot(temp);
            Vec3f p_body = point_body.getVector3fMap();

            if (p_body.norm() > 81 * pd2 * pd2) {
                self->point_selected_surf_[i] = true;
                self->residuals_[i] = pd2;
            } else {
                self->point_selected_surf_[i] = false;
            }
        }
    });

    self->effect_feat_surf_ = 0;
    self->corr_pts_.resize(cnt_pts);
    self->corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (self->point_selected_surf_[i]) {
            self->corr_norm_[self->effect_feat_surf_] = self->plane_coef_[i];
            self->corr_pts_[self->effect_feat_surf_] = self->scan_down_body_->points[i].getVector4fMap();
            self->corr_pts_[self->effect_feat_surf_][3] = self->residuals_[i];
            self->effect_feat_surf_++;
        }
    }
    self->corr_pts_.resize(self->effect_feat_surf_);
    self->corr_norm_.resize(self->effect_feat_surf_);

    if (self->effect_feat_surf_ < 20) {
        ekfom_data.valid = false;
        LLOG_WARN_THROTTLE(logging::kLio, 2000, "No enough effective surface points: {}", self->effect_feat_surf_);
        return;
    }

    /*** 计算测量雅可比 H 和残差 h（点面；pose 6 维，外参 6 维置零保持固定） ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(self->effect_feat_surf_, 12);
    ekfom_data.h.resize(self->effect_feat_surf_);

    for (int i = 0; i < self->effect_feat_surf_; i++) {
        Vec3d point_this_be = self->corr_pts_[i].head<3>().cast<double>();
        Vec3d point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        Mat3d point_crossmat = SO3::hat(point_this);

        /*** 最近平面的法向量 ***/
        Vec3d norm_vec = self->corr_norm_[i].head<3>().cast<double>();

        /*** 计算测量雅可比 H ***/
        Vec3d C(s.rot.conjugate() * norm_vec);
        Vec3d A(point_crossmat * C);

        ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0, 0, 0, 0, 0, 0;
        ekfom_data.h(i) = -self->corr_pts_[i][3];
    }
    ekfom_data.valid = true;
}

NavState LaserMapping::StateToNavState() const {
    NavState s;
    s.pose_is_ok_ = true;
    s.timestamp_ = state_timestamp_;
    s.pos_ = state_point_.pos;
    s.rot_ = SO3(state_point_.rot.toRotationMatrix());
    s.vel_ = state_point_.vel;
    s.bg_ = state_point_.bg;
    s.grav_ = Vec3d(state_point_.grav[0], state_point_.grav[1], state_point_.grav[2]);
    return s;
}

/// ==================== 对外读接口 ====================
/// 均在锁保护下返回快照，允许 ROS 侧（slam.cc 的发布/保存路径、service 回调）
/// 从非 LIO 线程并发调用。注意：StateToNavState() 等内部函数本身不加锁，
/// 只能由已持 mtx_state_ 的调用方（Run / 本区接口）使用。

Keyframe::Ptr LaserMapping::GetKeyframe() const {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return last_kf_;
}

NavState LaserMapping::GetState() const {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return StateToNavState();
}

NavState LaserMapping::GetIMUState() const {
    std::lock_guard<std::mutex> lock(mtx_state_);
    if (p_imu_->IsIMUInited()) {
        return StateToNavState();
    }

    NavState s;
    s.pose_is_ok_ = false;
    return s;
}

NavState LaserMapping::GetIMUStateNonBlocking() const {
    std::unique_lock<std::mutex> lock(mtx_state_, std::try_to_lock);
    if (!lock.owns_lock()) {
        /// LIO 正在计算，本帧 UI 刷新直接跳过（不阻塞调用者）
        NavState s;
        s.pose_is_ok_ = false;
        return s;
    }

    if (p_imu_->IsIMUInited()) {
        return StateToNavState();
    }

    NavState s;
    s.pose_is_ok_ = false;
    return s;
}

CloudPtr LaserMapping::GetScanUndist() const {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return scan_undistort_;
}

CloudPtr LaserMapping::GetScanDownWorld() const {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return scan_down_world_;
}

std::vector<Keyframe::Ptr> LaserMapping::GetAllKeyframes() {
    std::lock_guard<std::mutex> lock(mtx_state_);
    return all_keyframes_;
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    /// 先在锁内取关键帧列表快照，真正的拼接（重计算）在锁外做，
    /// 避免 SaveMap 这类调用把 LIO 线程长时间堵住。
    /// 说明：offset_R_lidar_fixed_/offset_t_lidar_fixed_ 是 LoadParamsFromYAML 后只读的配置参数，无需加锁。
    std::vector<Keyframe::Ptr> keyframe_snapshot;
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        keyframe_snapshot = all_keyframes_;
    }

    /// 本函数是 O(关键帧数) 的（整图重算）。分阶段计时的实测结论（1663 kf、57 次调用）：
    ///   Map:kf-voxel    0.377ms/kf -> 62%   ← 已改为缓存（Keyframe::GetCloudDownsampled）
    ///   Map:trans+cat   0.082ms/kf -> 14%
    ///   Map:final-voxel 120ms      -> 24%
    /// 即 T ≈ K × 0.459ms + 120ms（K 为本次快照的关键帧数，末次 K=1663 -> ~892ms）。
    ///
    /// 注意「省掉逐帧滤波、只做最后那一次全量滤波」是反优化：
    ///   关键帧存的是 scan_undistort_（未降采样，约 14400 点），逐帧滤波压缩到几千点。
    ///   去掉逐帧滤波后拼接量涨到 ~11.9M 点，最后那次全量滤波会从 120ms 涨到 ~700ms，
    ///   整体反而更慢（按约 6:1 的压缩比估算约 2 倍）。
    ///   压缩比的确切值看下面日志里的 raw -> filtered。
    ///
    /// 【重要】整图重算只应该发生在「真的需要完整点云」的场合（存图 srv/save_map）。
    /// rviz 看地图走增量路径（GetNewKeyframesCloud + SlamSystem::PublishGlobalMap），
    /// 由 rviz 自己累积，不要把整图重算搬到每帧的发布路径上。
    size_t raw_points = 0;
    size_t filtered_points = 0;
    CloudPtr global_map = BuildCloudFromKeyframes(keyframe_snapshot, 0, keyframe_snapshot.size(), use_lio_pose,
                                                  use_voxel, res, raw_points, filtered_points);

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(res, res, res);
        Timer::Evaluate(
            [&]() {
                voxel.setInputCloud(global_map);
                voxel.filter(*global_map_filtered);
            },
            "Map:final-voxel");
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    /// 点数是判断“能否省掉逐帧滤波”的关键依据：
    ///   raw/filtered 比值大 -> 逐帧滤波确实在压缩数据量，最后的全量滤波才不至于吃掉 2400 万点
    ///   raw/filtered 接近 1 -> 逐帧滤波几乎无用，应当直接交给最后那次全量滤波
    LLOG_INFO(logging::kLio, "global map: {} | kf {} | raw {} -> filtered {} | merged {} | res {:.2f}",
              global_map_filtered->size(), keyframe_snapshot.size(), raw_points, filtered_points, global_map->size(),
              res);

    return global_map_filtered;
}

CloudPtr LaserMapping::GetNewKeyframesCloud(size_t &io_published_count, bool use_lio_pose, float res, bool use_voxel,
                                            size_t max_kfs) {
    std::vector<Keyframe::Ptr> keyframe_snapshot;
    {
        std::lock_guard<std::mutex> lock(mtx_state_);
        keyframe_snapshot = all_keyframes_;
    }

    const size_t begin = io_published_count;
    size_t end = keyframe_snapshot.size();

    /// 限流：rviz 中途连上时要补发全部历史关键帧（可能上千个）。
    /// 一次做完意味着单次发布要拼接上千个关键帧 + toROSMsg + DDS 发几十 MB，
    /// 会把 worker 堵几百毫秒 —— 对 rviz 来说完全没必要，分批发就行了。
    /// 正常运行时每秒只新增几个关键帧，这个上限根本不生效。
    if (max_kfs > 0 && end - begin > max_kfs) {
        end = begin + max_kfs;
    }

    /// 无论本次有没有新增，都把游标推到本次处理到的位置（≤ 快照末尾）
    io_published_count = end;

    if (begin >= end) {
        return nullptr;
    }

    size_t raw_points = 0;
    size_t filtered_points = 0;
    return BuildCloudFromKeyframes(keyframe_snapshot, begin, end, use_lio_pose, use_voxel, res, raw_points,
                                   filtered_points);
}

CloudPtr LaserMapping::BuildCloudFromKeyframes(const std::vector<Keyframe::Ptr> &kfs, size_t begin, size_t end,
                                               bool use_lio_pose, bool use_voxel, float res, size_t &raw_points,
                                               size_t &filtered_points) {
    /// 关键帧点云(scan_undistort_)在 LIDAR 系，需先乘 lidar->IMU 外参，再由 IMU 位姿转到世界
    /// 修复：之前漏乘外参，非单位外参(如 r41 R=[[0,-1,0],[-1,0,0],[0,0,-1]])导致全局地图每帧点云方向错误
    Eigen::Matrix4d T_ext = Eigen::Matrix4d::Identity();
    T_ext.block<3, 3>(0, 0) = offset_R_lidar_fixed_;
    T_ext.block<3, 1>(0, 3) = offset_t_lidar_fixed_;

    CloudPtr out(new PointCloudType);

    /// 预分配：`*out += *cloud_trans` 会反复 realloc，每次都要把已有点全部拷一遍。
    /// 按“每个关键帧滤波后约几千点”估一个下界。
    out->reserve((end - begin) * 3000);

    /// 复用的拼接 buffer：避免上千次 new/delete（每次都是几十 KB 的分配）
    CloudPtr cloud_trans(new PointCloudType);

    for (size_t i = begin; i < end; i++) {
        auto &kf = kfs[i];

        /// 逐帧体素降采样的结果缓存在 Keyframe 里（见 Keyframe::GetCloudDownsampled）。
        /// 这是本路径最大的一笔开销：实测 1663 kf、平均 823 kf/次时占单次全量调用
        /// 的 62%（0.377ms × 823 = 310ms），而每次算出来的结果完全一样。
        /// 计时保留，用来确认缓存命中：avg 应从 0.377ms 掉到 ~0.001ms。
        CloudPtr cloud_filter;
        if (use_voxel) {
            Timer::Evaluate([&]() { cloud_filter = kf->GetCloudDownsampled(res); }, "Map:kf-voxel");
        } else {
            cloud_filter = kf->GetCloud();
        }

        raw_points += kf->GetCloud()->size();
        filtered_points += cloud_filter->size();

        Eigen::Matrix4d Twl = use_lio_pose ? (kf->GetLIOPose().matrix() * T_ext) : (kf->GetOptPose().matrix() * T_ext);

        Timer::Evaluate(
            [&]() {
                pcl::transformPointCloud(*cloud_filter, *cloud_trans, Twl);
                *out += *cloud_trans;
            },
            "Map:trans+cat");
    }

    return out;
}

void LaserMapping::SaveMap() {
    /// 保存地图
    auto global_map = GetGlobalMap(true);

    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);

    LLOG_INFO(logging::kLio, "lio map is saved to ./data/lio.pcd");
}

CloudPtr LaserMapping::GetRecentCloud() {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

bool LaserMapping::HasPendingScan() const {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    return !lidar_buffer_.empty() && !imu_buffer_.empty();
}

size_t LaserMapping::DropStaleScans(size_t keep) {
    std::lock_guard<std::mutex> lock(mtx_buffer_);

    size_t dropped = 0;
    while (lidar_buffer_.size() > keep) {
        lidar_buffer_.pop_front();
        if (!time_buffer_.empty()) {
            time_buffer_.pop_front();
        }
        if (!arrival_time_buffer_.empty()) {
            arrival_time_buffer_.pop_front();
        }
        dropped++;
    }

    if (dropped > 0) {
        LLOG_WARN_THROTTLE(logging::kLio, 2000,
                           "lidar buffer overflow: dropped {} stale scan(s), {} pending, {} imu pending", dropped,
                           lidar_buffer_.size(), imu_buffer_.size());
    }

    return dropped;
}

size_t LaserMapping::GetPendingScanCount() const {
    std::lock_guard<std::mutex> lock(mtx_buffer_);
    return lidar_buffer_.size();
}

CloudPtr LaserMapping::GetProjCloud() {
    std::lock_guard<std::mutex> lock(mtx_state_);
    auto cloud = scan_undistort_;
    ProjectKFs(cloud);
    return cloud;
}

void LaserMapping::SetInitPose(const SE3 &pose) {
    std::lock_guard<std::mutex> lock(mtx_state_);

    LLOG_INFO(logging::kLio, "initial pose lio: {}", pose.so3().unit_quaternion().coeffs().transpose());

    auto x = kf_.get_x();
    x.rot = fastlio::SO3(pose.rotationMatrix());
    x.pos = pose.translation();
    kf_.change_x(x);

    LLOG_INFO(logging::kLio, "set initial translation in laser mapping: {}", pose.translation().transpose());
}

SE3 LaserMapping::GetOptPose() const {
    std::lock_guard<std::mutex> lock(mtx_state_);

    if (last_kf_ == nullptr) {
        return StateToNavState().GetPose();
    }

    // 基于当前帧相对lastKF的LIO位姿，得到回环后的位姿
    SE3 delta = last_kf_->GetLIOPose().inverse() * StateToNavState().GetPose();
    return last_kf_->GetOptPose() * delta;
}

}  // namespace lightning
