//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "common/log.h"
#include "common/path_utils.h"
#include "core/localization/lidar_loc/lidar_loc.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"

#include <pcl/common/transforms.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace lightning {

namespace {

/// SE3 → TransformStamped
geometry_msgs::msg::TransformStamped MakeTf(const builtin_interfaces::msg::Time &stamp, const std::string &parent,
                                            const std::string &child, const SE3 &T) {
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = parent;
    out.child_frame_id = child;
    const Vec3d t = T.translation();
    const auto q = T.unit_quaternion();
    out.transform.translation.x = t.x();
    out.transform.translation.y = t.y();
    out.transform.translation.z = t.z();
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
    return out;
}

/// Transform → SE3
SE3 TfToSE3(const geometry_msgs::msg::Transform &tf) {
    Eigen::Quaterniond q(tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z);
    q.normalize();
    const Vec3d t(tf.translation.x, tf.translation.y, tf.translation.z);
    return SE3(SO3(q.toRotationMatrix()), t);
}

/// 旋转矩阵 → SO(3)：配置里常写有限小数（如 6 位），直接构造会因 R·Rᵀ≠I 触发 Sophus
/// isOrthogonal/det 断言，这里先做正交化投影（四元数归一化），从根上免疫精度误差。
SO3 NormalizeRotation(const Mat3d &R) {
    Eigen::Quaterniond q(R);
    q.normalize();
    return SO3(q.toRotationMatrix());
}

/// 平移(3) / 旋转(9，行主序) 数组 → SE3，缺省单位阵
SE3 ArraysToSE3(const std::vector<double> &t, const std::vector<double> &r) {
    Vec3d tt = Vec3d::Zero();
    Mat3d rr = Mat3d::Identity();
    if (t.size() >= 3) {
        tt << t[0], t[1], t[2];
    }
    if (r.size() >= 9) {
        rr << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
    }
    return SE3(NormalizeRotation(rr), tt);
}

}  // namespace

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    /// 先等换图线程收尾：它内部会调 loc_->Init / Finish，必须早于 loc_ 释放
    if (switch_thread_.joinable()) {
        switch_thread_.join();
    }

    // Init 可能在创建 loc_ 之前就失败返回（如地图路径非法），此处必须判空
    if (loc_) {
        loc_->Finish();
    }
}

bool LocSystem::Init(const std::string &yaml_path, const std::string &map_path_override) {
    YAML_IO yaml(yaml_path);

    /// 地图目录：ROS 参数 map_path 优先，否则用 yaml 里的 system.map_path。
    /// 注意 map_path 不是 ROS 参数（yaml.GetValue 直读文件），所以只能用本函数参数覆盖，
    /// 不能通过 --ros-args -p 覆盖。
    /// 支持 "~/" 前缀：配置在多台机器（用户名不同）间共用时不能写死 /home/<user>/...
    const bool from_param = !map_path_override.empty();
    std::string map_path =
        path_utils::ResolveDir(from_param ? map_path_override : yaml.GetValue<std::string>("system", "map_path"));

    /// 在创建任何 ROS 实体之前先把地图路径校验掉，避免半初始化状态
    if (map_path.empty()) {
        LLOG_ERROR(logging::kLocSys, "map path is empty: set system.map_path in {} or pass map_path:=<dir>", yaml_path);
        return false;
    }
    const std::string map_index = map_path + "/index.txt";
    if (!std::filesystem::exists(map_index)) {
        LLOG_ERROR(logging::kLocSys, "map index not found: {} (map_path = {})", map_index, map_path);
        return false;
    }
    LLOG_INFO(logging::kLocSys, "map path = {} (source: {})", map_path, from_param ? "ros parameter" : "yaml");

    yaml_path_ = yaml_path;
    map_path_ = map_path;

    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    /// rviz/调试选项（缺省关闭，兼容旧配置）
    options_.pub_tf_ = yaml.GetValue<bool>("system", "pub_tf", true);
    options_.pub_odom_ = yaml.GetValue<bool>("system", "pub_odom", true);
    options_.log_pose_opt_ = yaml.GetValue<bool>("system", "log_pose_opt", false);
    /// 轨迹发布间隔（秒），0 = 不发布（详见 Options::path_pub_interval_）
    options_.path_pub_interval_ = yaml.GetValue<double>("system", "path_pub_interval", 1.0);

    LLOG_INFO(logging::kLocSys, "online mode, creating ros2 node ... ");

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    rclcpp::QoS qos(10);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    // TF 回调：无论 pub_tf_ 与否都缓存最近一次 loc 全局位姿（供 nav_state/odom 语义统一），
    // 仅在 pub_tf_ 时按 map→odom→base_link 链广播 TF（路线B：odom=LO 帧，map→odom 为修正）。
    loc_->SetTFCallback([this](const geometry_msgs::msg::TransformStamped &pose) { HandleLocTf(pose); });

    /// 静态外参（缺省单位阵）解析：始终解析（供初值归算/odom 归算使用），仅 pub_tf_ 时广播
    ///  - base_link→imu_link：system.extrinsic_base_imu_T/R
    ///  - imu_link→lidar_link：复用 fasterlio.extrinsic_T/R（与 LIO 内部一致）
    auto t_base_imu =
        yaml.GetValue<std::vector<double>>("system", "extrinsic_base_imu_T", std::vector<double>{0, 0, 0});
    auto R_base_imu = yaml.GetValue<std::vector<double>>("system", "extrinsic_base_imu_R",
                                                         std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    auto t_imu_lidar = yaml.GetValue<std::vector<double>>("fasterlio", "extrinsic_T", std::vector<double>{0, 0, 0});
    auto R_imu_lidar =
        yaml.GetValue<std::vector<double>>("fasterlio", "extrinsic_R", std::vector<double>{1, 0, 0, 0, 1, 0, 0, 0, 1});
    T_base_imu_ = ArraysToSE3(t_base_imu, R_base_imu);
    T_imu_lidar_ = ArraysToSE3(t_imu_lidar, R_imu_lidar);

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

        /// 广播 base_link→imu_link→lidar_link（静态，发布一次）
        std::vector<geometry_msgs::msg::TransformStamped> statics;
        statics.push_back(MakeTf(node_->now(), "base_link", "imu_link", T_base_imu_));
        statics.push_back(MakeTf(node_->now(), "imu_link", "lidar_link", T_imu_lidar_));
        tf_static_broadcaster_->sendTransform(statics);

        LLOG_INFO(logging::kLocSys, "static tf: base_link->imu_link t={}, imu_link->lidar_link t={}",
                  T_base_imu_.translation().transpose(), T_imu_lidar_.translation().transpose());
    }

    if (options_.pub_odom_) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
        nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);
    }

    if (yaml.GetValue<bool>("system", "enable_lidar_loc_rviz", false)) {
        std::string scan_topic =
            yaml.GetValue<std::string>("system", "rviz_current_scan_topic", "lightning/current_scan_cloud");
        cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 10);
        global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/global_map", 1);
    }

    if (yaml.GetValue<bool>("system", "enable_path_rviz", false)) {
        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
    }

    /// 下游感知输入（plan_env/GridMap）：map 系点云 + map 系雷达位姿。
    /// 用 SensorDataQoS（best_effort）：大点云不必重传，与下游 GridMap 的订阅 QoS 一致。
    if (yaml.GetValue<bool>("system", "enable_perception_pub", false)) {
        perception_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("lightning/perception/cloud",
                                                                                       rclcpp::SensorDataQoS());
        perception_pose_pub_ =
            node_->create_publisher<nav_msgs::msg::Odometry>("lightning/perception/pose", rclcpp::SensorDataQoS());
        LLOG_INFO(logging::kLocSys,
                  "perception topics enabled: lightning/perception/cloud (map-frame cloud) + "
                  "lightning/perception/pose (map-frame lidar pose)");
    }

    savepath_service_ = node_->create_service<srv::SavePath>(
        "lightning/save_path", [this](const srv::SavePath::Request::SharedPtr &req,
                                      srv::SavePath::Response::SharedPtr res) { SavePath(req, res); });

    /// 定位状态反馈话题（Web/rviz 订阅，0=IDLE 1=INITIALIZING 2=GOOD 3=FOLLOWING_DR 4=FAIL）
    loc_state_pub_ = node_->create_publisher<std_msgs::msg::Int32>("lightning/loc_state", 10);
    loc_->SetLocStateCallback([this](const std_msgs::msg::Int32 &state) {
        if (loc_state_pub_) {
            loc_state_pub_->publish(state);
        }
    });

    /// 运行中接收初始化位姿：rviz2 2D Pose Estimate 默认发布到 initialpose（可参数/remap 配置）
    init_pose_topic_ = yaml.GetValue<std::string>("system", "init_pose_topic", "initialpose");
    init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        init_pose_topic_, 1,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { OnInitialPose(msg); });

    /// Web 接口：lightning/set_initpose
    set_initpose_service_ = node_->create_service<srv::SetInitPose>(
        "lightning/set_initpose", [this](const srv::SetInitPose::Request::SharedPtr &req,
                                         srv::SetInitPose::Response::SharedPtr res) { HandleSetInitPose(req, res); });

    /// 系统阶段话题（latched）：后启动的订阅者（如感知节点）也能立即拿到当前状态。
    /// 注意与 lightning/loc_state 区分：loc_state 是“定位算法”的状态，
    /// map_state 是“地图/系统生命周期”的状态。
    map_state_pub_ = node_->create_publisher<std_msgs::msg::Int32>("lightning/map_state",
                                                                   rclcpp::QoS(1).reliable().transient_local());

    /// Web 接口：lightning/load_map（运行时换图）
    loadmap_service_ = node_->create_service<srv::LoadMap>(
        "lightning/load_map", [this](const srv::LoadMap::Request::SharedPtr &req,
                                     srv::LoadMap::Response::SharedPtr res) { HandleLoadMap(req, res); });

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LLOG_INFO(logging::kLocSys, "online loc node has been created.");
        PublishMapState(LocPhase::kRunning);
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    /// 换图过程中拒绝外部初值：此时 lidar_loc_ 正在被重建，初值落到旧实例还是新实例
    /// 是不确定的，必须让调用方换图完成后再给。
    if (phase_.load() == LocPhase::kSwitching) {
        LLOG_WARN(logging::kLocSys, "map is switching, ignore init pose (retry after map_state becomes kRunning)");
        return;
    }

    /// pose 语义：base_link(本体) 在地图中的位姿（rviz2/Web 点击即此语义）
    /// lightning 内部 loc/LO 位姿为 IMU 系 → 左乘 base_link→imu_link 外参转换
    const SE3 imu_pose = pose * T_base_imu_;
    LLOG_INFO(logging::kLocSys, "set init pose(base in map): t={}, q={} -> imu t={}", pose.translation().transpose(),
              pose.unit_quaternion().coeffs().transpose(), imu_pose.translation().transpose());

    /// 先置 kRunning：SetExternalPose 之后定位状态机会在下一帧点云上开始 SearchAndInit，
    /// 若此时 phase 还不是 kRunning，那一帧会被 ProcessLidar 的闸门丢掦。
    phase_.store(LocPhase::kRunning);
    loc_->SetExternalPose(imu_pose.unit_quaternion(), imu_pose.translation());

    /// 过渡用 map→odom：定位 GOOD 前 TF 树也需完整（否则 rviz 中 frame=odom 的消息因查不到
    /// map→odom 而被 TF 过滤器丢弃）。用“用户给定的 base in map 位姿”与当前 LO(odom) 位姿推出:
    ///   map→odom = P_base × (odom→base)⁻¹
    /// 效果：机器人立即显示在点选位姿处，且与 GOOD 后的真实 map→odom 基本一致（无跳变）。
    if (loc_) {
        const auto lo_state = loc_->GetState();
        if (lo_state.pose_is_ok_) {
            const SE3 T_imu_base = T_base_imu_.inverse();
            const SE3 T_odom_base = lo_state.GetPose() * T_imu_base;
            pending_map_odom_ = pose * T_odom_base.inverse();
            LLOG_INFO(logging::kLocSys, "pending map->odom: t={}", pending_map_odom_.translation().transpose());
        }
    }
}

void LocSystem::OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto &p = msg->pose.pose;
    const Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Vec3d t(p.position.x, p.position.y, p.position.z);
    t.z() = GetReferenceZ();  // rviz2 2D Pose Estimate 不含 z，自动取参考高度

    LLOG_INFO(logging::kLocSys, "rviz2 initial pose from [{}]: t={}, q={}", msg->header.frame_id, t.transpose(),
              q.coeffs().transpose());
    SetInitPose(SE3(SO3(Eigen::Quaterniond(q).normalized().toRotationMatrix()), t));
}

void LocSystem::HandleSetInitPose(const srv::SetInitPose::Request::SharedPtr req,
                                  srv::SetInitPose::Response::SharedPtr res) {
    Vec3d t(req->x, req->y, req->z);
    if (!req->z_valid) {
        t.z() = GetReferenceZ();
    }
    const Eigen::AngleAxisd Rz(req->yaw, Eigen::Vector3d::UnitZ());

    LLOG_INFO(logging::kLocSys, "web set init pose: t={}, yaw={}", t.transpose(), req->yaw);
    SetInitPose(SE3(SO3(Rz.matrix()), t));

    res->success = true;
    res->message = "init pose accepted (base_link in map, z=" + std::to_string(t.z()) +
                   "), awaiting GICP init, loc_state will report INITIALIZING->GOOD";
}

// ==================== 运行时换图 ====================

void LocSystem::PublishMapState(LocPhase p) {
    phase_.store(p);
    if (map_state_pub_ != nullptr) {
        std_msgs::msg::Int32 msg;
        msg.data = static_cast<int32_t>(p);
        map_state_pub_->publish(msg);
    }
}

LocSystem::MapPrecheck LocSystem::PrecheckMapDir(const std::string &dir) const {
    /// 本函数刻意只读文件系统，不触碰任何成员状态：
    /// 校验失败时调用方直接返回，旧地图继续可用。
    const std::string index = dir + "/index.txt";
    std::error_code ec;
    if (!std::filesystem::exists(index, ec)) {
        return {false, "index.txt not found: " + index};
    }

    std::ifstream fin(index);
    if (!fin) {
        return {false, "cannot read index.txt: " + index};
    }

    /// 解析格式与 TiledMap::LoadMapIndex 保持一致：首行是地图原点，
    /// 之后每行 "<id> <gx> <gy> <path>"，遇到 "# functional points" 进入功能点段。
    /// 注意 path 字段在实际加载时会被忽略（按 <dir>/<id>.pcd 重算），这里也不看它。
    std::vector<int> chunk_ids;
    std::string line;
    bool first_line = true;
    bool reading_fp = false;
    while (std::getline(fin, line)) {
        if (line.empty()) {
            continue;
        }
        if (first_line) {
            first_line = false;
            continue;
        }
        if (line == "# functional points") {
            reading_fp = true;
            continue;
        }
        if (reading_fp) {
            continue;  // 本项目不使用 FP，跳过
        }

        std::istringstream ss(line);
        int id = 0, gx = 0, gy = 0;
        std::string path;
        if (!(ss >> id >> gx >> gy >> path)) {
            continue;
        }
        chunk_ids.push_back(id);
    }
    fin.close();

    if (chunk_ids.empty()) {
        return {false, "index.txt parsed 0 map chunk: " + index};
    }

    /// 每个区块的点云必须存在且非空。
    /// TiledMap::LoadMapIndex 里 loadPCDFile 的失败只打日志，所以必须在这里拦住。
    for (const int id : chunk_ids) {
        const std::string pcd = dir + "/" + std::to_string(id) + ".pcd";
        if (!std::filesystem::exists(pcd, ec)) {
            return {false, "missing chunk pcd: " + pcd};
        }
        if (std::filesystem::file_size(pcd, ec) == 0) {
            return {false, "empty chunk pcd: " + pcd};
        }
    }

    return {true, std::to_string(chunk_ids.size()) + " chunks verified"};
}

void LocSystem::HandleLoadMap(const srv::LoadMap::Request::SharedPtr req, srv::LoadMap::Response::SharedPtr res) {
    /// 本回调在 executor 线程上执行，必须快速返回：
    /// 只做预检 + 启线程，重活交给 LoadMapThread()。
    /// 若在这里同步换图，executor 会被占住几秒，雷达/IMU 回调全停，
    /// DDS reader 队列堆积会反过来把雷达驱动压降频（10Hz->5Hz 那个坑）。
    res->accepted = false;

    if (phase_.load() == LocPhase::kSwitching || switch_busy_.load()) {
        res->message = "map switching already in progress";
        LLOG_WARN(logging::kLocSys, "load_map rejected: {}", res->message);
        return;
    }

    const std::string dir = path_utils::ResolveDir(req->map_path, map_path_);
    if (dir.empty()) {
        res->message = "empty map path (and no default in yaml)";
        LLOG_ERROR(logging::kLocSys, "load_map rejected: {}", res->message);
        return;
    }

    if (phase_.load() == LocPhase::kRunning && dir == map_path_) {
        res->accepted = true;
        res->message = "already using " + dir;
        return;
    }

    /// 预检必须在关闸之前：不通过就原样返回，旧地图继续正常定位
    const MapPrecheck chk = PrecheckMapDir(dir);
    if (!chk.ok) {
        res->message = "precheck failed: " + chk.message;
        LLOG_ERROR(logging::kLocSys, "load_map rejected ({})", res->message);
        return;
    }

    /// 关闸：此后 ProcessIMU/ProcessLidar 走早退分支，立即返回、不阻塞
    PublishMapState(LocPhase::kSwitching);
    switch_busy_.store(true);

    /// 回收上一次的换图线程（phase 已不是 kSwitching，说明它已经跑完）
    if (switch_thread_.joinable()) {
        switch_thread_.join();
    }
    switch_thread_ = std::thread(&LocSystem::LoadMapThread, this, dir);

    res->accepted = true;
    res->message = "accepted: switching to " + dir + " (" + chk.message + "); watch lightning/map_state";
    LLOG_INFO(logging::kLocSys, "load_map accepted: {} ({})", dir, chk.message);
}

void LocSystem::LoadMapThread(const std::string &dir) {
    const auto t0 = std::chrono::steady_clock::now();
    LLOG_INFO(logging::kLocSys, "load_map: switching to {}", dir);

    /// 排空在途回调：闸门已在 HandleLoadMap 里关上，所以只需等那一个正在执行的
    /// ProcessLidar/ProcessIMU 退出，之后临界区必然为空。
    /// 这里只短暂持锁，不能跨越下面的 loc_->Init（否则又会把 executor 堵住）。
    DrainInFlight();

    /// 真正重建定位模块（内部会 Finish 旧图 → 建新 LidarLoc → 加载地图索引）
    const bool ok = loc_->Init(yaml_path_, dir);

    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    if (ok) {
        map_path_ = dir;
        ResetRuntimeStateAfterMapSwitch();
        /// 换图后不沿用旧位姿自动跑：lidar_loc_ 是全新对象，loc_inited_=false、
        /// initial_pose_set_=false，且 init_with_fp=false，会自然停在“等外部位姿”。
        /// 这里保持 kRunning 只为维持数据流（LIO 继续跑）；
        /// loc_state 会从 GOOD 变成 INITIALIZING —— 那就是“请给初值”的信号。
        PublishMapState(LocPhase::kRunning);
        LLOG_INFO(logging::kLocSys,
                  "load_map: done in {:.0f} ms -> {}; waiting for external init pose "
                  "(rviz2 2D Pose Estimate or lightning/set_initpose)",
                  ms, dir);
    } else {
        PublishMapState(LocPhase::kError);
        LLOG_ERROR(logging::kLocSys,
                   "load_map: FAILED in {:.0f} ms -> {}; no usable map now (old map already released)", ms, dir);
    }

    switch_busy_.store(false);
}

void LocSystem::ResetRuntimeStateAfterMapSwitch() {
    /// 轨迹、缓存的 map 位姿、过渡 TF 全部绑在旧地图的坐标系上，必须清掉
    path_ = nav_msgs::msg::Path();
    {
        std::lock_guard<std::mutex> lk(map_pose_mutex_);
        latest_map_pose_valid_ = false;
        latest_map_pose_ = SE3();
    }
    pending_map_odom_ = SE3();
    last_map_pub_time_ = 0;
    last_path_pub_time_ = 0;

    LLOG_INFO(logging::kLocSys, "runtime state reset after map switch (path / cached map pose / pending tf)");
}

double LocSystem::GetReferenceZ() {
    /// 1) 已有定位结果时沿用其 map 系高度
    {
        std::lock_guard<std::mutex> lk(map_pose_mutex_);
        if (latest_map_pose_valid_) {
            return latest_map_pose_.translation().z();
        }
    }
    /// 2) 取地图起始功能点(start)高度
    if (loc_ && loc_->GetLidarLoc() && loc_->GetLidarLoc()->GetMap()) {
        auto fps = loc_->GetLidarLoc()->GetMap()->GetAllFP();
        if (!fps.empty()) {
            return fps.front().pose_.translation().z();
        }
    }
    /// 3) 缺省
    return 0.24;
}

void LocSystem::StartLoc() {
    /// 注意：本函数**不**做任何初始化，也不触发 FP——只是把数据流闸门打开，
    /// 让 LIO 先从第一帧就跑起来，这样用户点 rviz2 2D Pose Estimate 时温启动、无延迟。
    /// 初始化仍然靠 SetInitPose（rviz initialpose / web set_initpose）。
    LLOG_INFO(logging::kLocSys, "start loc: data flow enabled, waiting for external init pose");
    phase_.store(LocPhase::kRunning);
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    ProcessingScope scope(*this);
    if (!scope.active()) {
        return;
    }
    loc_->ProcessIMUMsg(imu);
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    /// 整个函数体（含 PublishDebugAndRviz）都在临界区内：
    /// PublishDebugAndRviz 会访问 loc_->GetLIO()/GetLidarLoc()，而换图会重建那些成员。
    ProcessingScope scope(*this);
    if (!scope.active()) {
        return;
    }
    loc_->ProcessLidarMsg(cloud);
    PublishDebugAndRviz(cloud->header.stamp);
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    ProcessingScope scope(*this);
    if (!scope.active()) {
        return;
    }
    loc_->ProcessLivoxLidarMsg(cloud);
    PublishDebugAndRviz(cloud->header.stamp);
}

void LocSystem::PublishDebugAndRviz(const builtin_interfaces::msg::Time &stamp) {
    /// LO(ESKF) 状态：IMU 系位姿（odom 帧）与速度；lightning 内部 pose 语义为 IMU(本体) 系
    auto state = loc_->GetState();

    /// base_link 归算：base_link = IMU 位姿 左乘 imu→base（T_base_imu^-1），使 TF 树 base_link 落到机器人本体
    const SE3 T_imu_base = T_base_imu_.inverse();
    const SE3 T_odom_base = state.GetPose() * T_imu_base;  // odom→base_link（LO 归算）

    /// 全局 map→IMU 位姿：仅在最近一次 loc 结果就绪后可用。
    /// 经典 odom 模型：LO 为 odom 系（不再被 SetInitPose 搬到地图），未就绪时不得用 LO 顶替 map，
    /// 否则 nav_state/path 会带上 map→odom 的偏差；未定位阶段直接不发地图侧话题。
    SE3 map_imu = SE3();
    bool map_valid = false;
    {
        std::lock_guard<std::mutex> lk(map_pose_mutex_);
        if (latest_map_pose_valid_) {
            map_imu = latest_map_pose_;
            map_valid = true;
        }
    }
    const SE3 T_map_base = map_valid ? (map_imu * T_imu_base) : SE3();  // map→base_link（loc 归算）

    if (options_.log_pose_opt_) {
        const SE3 &T_dbg = map_valid ? T_map_base : T_odom_base;
        auto q = T_dbg.unit_quaternion();
        LLOG_INFO(logging::kLocSys,
                  "base pose in {}: [{:.4f}, {:.4f}, {:.4f}]\tq: [{:.4f}, {:.4f}, {:.4f}, {:.4f}]"
                  "\tvelocity: [{:.4f}, {:.4f}, {:.4f}]",
                  map_valid ? "map" : "odom(未定位)", T_dbg.translation().x(), T_dbg.translation().y(),
                  T_dbg.translation().z(), q.x(), q.y(), q.z(), q.w(), state.vel_.x(), state.vel_.y(), state.vel_.z());
    }

    /// odom 侧（只依赖 LO）：/lightning/odom + TF odom→base_link，定位未就绪时也始终可用
    if (odom_pub_ != nullptr) {
        /// odom：odom 系下 base_link 位姿（LO 归算），与 TF odom→base_link 一致
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = T_odom_base.translation().x();
        odom.pose.pose.position.y = T_odom_base.translation().y();
        odom.pose.pose.position.z = T_odom_base.translation().z();
        auto q_odom = T_odom_base.unit_quaternion();
        odom.pose.pose.orientation.x = q_odom.x();
        odom.pose.pose.orientation.y = q_odom.y();
        odom.pose.pose.orientation.z = q_odom.z();
        odom.pose.pose.orientation.w = q_odom.w();
        odom.twist.twist.linear.x = state.vel_.x();
        odom.twist.twist.linear.y = state.vel_.y();
        odom.twist.twist.linear.z = state.vel_.z();
        odom_pub_->publish(odom);
    }
    if (tf_broadcaster_ != nullptr) {
        tf_broadcaster_->sendTransform(MakeTf(stamp, "odom", "base_link", T_odom_base));

        /// map→odom：定位就绪后由 HandleLocTf 以 loc 结果频率广播真实值（此处不重复发）；
        /// 未就绪时用 pending 过渡值（有初值=P_base 推出的 W，无初值=单位阵）补全 TF 树，
        /// 保证 rviz 在 Fixed Frame=map 时不会因缺 map→odom 而丢弃 frame=odom 的消息。
        if (!map_valid) {
            tf_broadcaster_->sendTransform(MakeTf(stamp, "map", "odom", pending_map_odom_));
        }
    }

    /// 地图侧（依赖 loc 结果，map→base_link）：未定位成功前不发，避免用 odom 顶替 map
    if (nav_state_pub_ != nullptr && map_valid) {
        /// nav_state：全局 map→base_link（loc 结果归算到本体）
        msg::NavState ns;
        ns.header.stamp = stamp;
        ns.header.frame_id = "map";
        ns.pose.position.x = T_map_base.translation().x();
        ns.pose.position.y = T_map_base.translation().y();
        ns.pose.position.z = T_map_base.translation().z();
        auto q = T_map_base.unit_quaternion();
        ns.pose.orientation.x = q.x();
        ns.pose.orientation.y = q.y();
        ns.pose.orientation.z = q.z();
        ns.pose.orientation.w = q.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        ns.confidence = state.confidence_;
        ns.pose_is_ok = true;
        nav_state_pub_->publish(ns);

        /// path：map 下 base_link 轨迹（仅定位就绪时记录，与退出保存语义一致）
        if (path_pub_ != nullptr) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = ns.header;
            ps.pose = ns.pose;
            path_.header = ns.header;
            path_.poses.push_back(ps);  // 仍然每帧累积：save_path 导出轨迹要用它

            /// 发布另行判定：path_ 无界增长，publish 要把整个 vector 序列化一遍
            /// （O(n)/帧、O(n²) 累计）。定位会连续跑几小时，只能按时间节流。
            const bool path_wanted = path_pub_->get_subscription_count() > 0;
            const double path_now = ToSec(ns.header.stamp);
            const bool path_interval_ok =
                options_.path_pub_interval_ > 0 &&
                (last_path_pub_time_ <= 0 || path_now - last_path_pub_time_ >= options_.path_pub_interval_);
            if (path_wanted && path_interval_ok) {
                last_path_pub_time_ = path_now;
                path_pub_->publish(path_);
            }
        }
    }

    if (options_.pub_tf_ && cloud_pub_ != nullptr) {
        // lio_ 在初始化失败时可能为空，此处及后续同类访问一律判空，
        // 避免“地图坏了”变成段错误。
        auto lio = loc_->GetLIO();
        auto scan_world = lio ? lio->GetScanDownWorld() : nullptr;
        if (scan_world && !scan_world->empty()) {
            sensor_msgs::msg::PointCloud2 scan_msg;
            pcl::toROSMsg(*scan_world, scan_msg);
            /// 经典 odom 模型：GetScanDownWorld 由 ESKF 世界系(odom)生成，帧标必须是 odom；
            /// rviz 会经 TF(map→odom) 自动映射到地图下，与 /lightning/global_map 正确叠合。
            scan_msg.header.frame_id = "odom";
            scan_msg.header.stamp = stamp;
            cloud_pub_->publish(scan_msg);
        }
    }

    /// 下游感知（plan_env/GridMap）输入：map 系点云 + map 系雷达位姿。
    ///  - GridMap 的 cloud_is_world=true，点云坐标被直接当作世界系；
    ///  - GridMap 的 ray_pos_ 直接取 Odometry.pose 作为“传感器在世界系的位置”，
    ///    配合其 need_extrinsic=false，此处给的正是 map→lidar_link；
    ///  - 且 GridMap 在未收到 sensor_pose 前会丢弃所有点云，所以两者必须都发。
    ///  - 仅当定位就绪（map_valid）才发布：否则 map 系位姿无意义，会污染下游栅格。
    if (map_valid && (perception_cloud_pub_ != nullptr || perception_pose_pub_ != nullptr)) {
        /// map→odom：map 侧与 odom 侧的连接，用于把 ESKF(odom) 世界系点云搬到 map
        const SE3 T_map_odom = T_map_base * T_odom_base.inverse();

        if (perception_pose_pub_ != nullptr) {
            /// 雷达在 map 下的位姿：map→base→imu→lidar
            const SE3 T_map_lidar = T_map_base * T_base_imu_ * T_imu_lidar_;
            const Vec3d t_lidar = T_map_lidar.translation();
            const auto q_lidar = T_map_lidar.unit_quaternion();

            nav_msgs::msg::Odometry pose_msg;
            pose_msg.header.stamp = stamp;
            pose_msg.header.frame_id = "map";
            pose_msg.child_frame_id = "lidar_link";
            pose_msg.pose.pose.position.x = t_lidar.x();
            pose_msg.pose.pose.position.y = t_lidar.y();
            pose_msg.pose.pose.position.z = t_lidar.z();
            pose_msg.pose.pose.orientation.x = q_lidar.x();
            pose_msg.pose.pose.orientation.y = q_lidar.y();
            pose_msg.pose.pose.orientation.z = q_lidar.z();
            pose_msg.pose.pose.orientation.w = q_lidar.w();
            pose_msg.twist.twist.linear.x = state.vel_.x();
            pose_msg.twist.twist.linear.y = state.vel_.y();
            pose_msg.twist.twist.linear.z = state.vel_.z();
            perception_pose_pub_->publish(pose_msg);
        }

        if (perception_cloud_pub_ != nullptr) {
            auto lio = loc_->GetLIO();
            auto scan_world = lio ? lio->GetScanDownWorld() : nullptr;
            if (scan_world && !scan_world->empty()) {
                CloudPtr scan_map(new PointCloudType());
                pcl::transformPointCloud(*scan_world, *scan_map, T_map_odom.matrix().cast<float>());

                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*scan_map, cloud_msg);
                cloud_msg.header.frame_id = "map";
                cloud_msg.header.stamp = stamp;
                perception_cloud_pub_->publish(cloud_msg);
            }
        }
    }

    /// 定位模式下的全局地图不会变化，每10秒发布一次即可
    double current_time = ToSec(stamp);
    auto lidar_loc = loc_->GetLidarLoc();
    if (global_map_pub_ != nullptr && lidar_loc && lidar_loc->GetMap() && (current_time - last_map_pub_time_ > 10.0)) {
        CloudPtr global_map = lidar_loc->GetMap()->GetStaticCloud2();
        if (global_map && !global_map->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map, map_msg);
            map_msg.header.frame_id = "map";
            map_msg.header.stamp = stamp;
            global_map_pub_->publish(map_msg);
            last_map_pub_time_ = current_time;
        }
    }
}

/// 定位高频结果回调：缓存 map→IMU(loc 全局) 并按路线B广播 map→odom→base_link
void LocSystem::HandleLocTf(const geometry_msgs::msg::TransformStamped &map_base) {
    SE3 map_imu;
    {
        std::lock_guard<std::mutex> lk(map_pose_mutex_);
        latest_map_pose_ = TfToSE3(map_base.transform);
        latest_map_pose_valid_ = true;
        map_imu = latest_map_pose_;
    }

    if (tf_broadcaster_ == nullptr) {
        return;  // 未启用 pub_tf_
    }

    /// 本体归算：base_link = IMU 位姿 左乘 imu→base（T_base_imu^-1）
    const SE3 T_imu_base = T_base_imu_.inverse();

    /// odom→base_link：LO(ESKF, IMU 系) 归算
    auto lo_state = loc_->GetState();
    if (!lo_state.pose_is_ok_) {
        return;
    }
    const SE3 T_odom_base = lo_state.GetPose() * T_imu_base;

    /// map→base_link：loc 全局(IMU 系) 归算
    const SE3 T_map_base = map_imu * T_imu_base;

    /// map→odom = map→base_link * (odom→base_link)^-1 （与 T_base_imu 无关，天然不变式）
    const SE3 T_map_odom = T_map_base * T_odom_base.inverse();

    const auto &stamp = map_base.header.stamp;
    tf_broadcaster_->sendTransform(MakeTf(stamp, "map", "odom", T_map_odom));
    tf_broadcaster_->sendTransform(MakeTf(stamp, "odom", "base_link", T_odom_base));
}

void LocSystem::SavePath(const srv::SavePath::Request::SharedPtr request, srv::SavePath::Response::SharedPtr response) {
    std::string save_path = request->file_path;
    if (save_path.empty()) {
        char time_str[64];
        time_t now = time(nullptr);
        strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", localtime(&now));
        save_path = "./data/path_" + std::string(time_str) + ".txt";
    }

    std::ofstream file(save_path);
    if (!file.is_open()) {
        response->success = false;
        response->message = "Failed to open file: " + save_path;
        return;
    }
    file << "timestamp px py pz qx qy qz qw\n";
    for (const auto &pose : path_.poses) {
        file << std::fixed << std::setprecision(5) << pose.header.stamp.sec << "." << std::setfill('0') << std::setw(9)
             << pose.header.stamp.nanosec << " " << pose.pose.position.x << " " << pose.pose.position.y << " "
             << pose.pose.position.z << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
             << pose.pose.orientation.z << " " << pose.pose.orientation.w << "\n";
    }

    file.close();
    response->success = true;
    response->message = "Path saved to " + save_path + ". Total poses: " + std::to_string(path_.poses.size());
    LLOG_INFO(logging::kLocSys, "{}", response->message);
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
