#include <algorithm>
#include <cmath>
#include <execution>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>

#include "core/localization/lidar_loc/lidar_loc.h"

#include <opencv2/highgui.hpp>

#include "common/log.h"
#include "io/file_io.h"
#include "io/yaml_io.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"

namespace lightning::loc {

LidarLoc::LidarLoc(LidarLoc::Options options) : options_(options) {
    pcl_icp_.reset(new ICPType());
    pcl_icp_->setMaximumIterations(4);
    pcl_icp_->setTransformationEpsilon(0.01);

    LLOG_INFO(logging::kLidarLoc,
              "match name is small_gicp, source res: {}, fine target res: {}, fine max corr dist: {}, "
              "rough target res: {}, rough max corr dist: {}",
              options_.gicp_source_res_, options_.gicp_fine_target_res_, options_.gicp_fine_max_corr_dist_,
              options_.gicp_rough_target_res_, options_.gicp_rough_max_corr_dist_);
}

LidarLoc::~LidarLoc() {
    if (update_map_thread_.joinable()) {
        update_map_quit_ = true;
        update_map_thread_.join();
    }

    recover_pose_out_.close();
}

bool LidarLoc::Init(const std::string& config_path) {
    YAML_IO yaml(config_path);
    options_.map_option_.enable_dynamic_polygon_ = yaml.GetValue<bool>("maps", "with_dyn_area");
    options_.map_option_.max_pts_in_dyn_chunk_ = yaml.GetValue<int>("maps", "max_pts_dyn_chunk");
    options_.map_option_.load_map_size_ = yaml.GetValue<int>("maps", "load_map_size");
    options_.map_option_.unload_map_size_ = yaml.GetValue<int>("maps", "unload_map_size");

    options_.update_kf_dis_ = yaml.GetValue<double>("lidar_loc", "update_kf_dis");
    options_.update_lidar_loc_score_ = yaml.GetValue<double>("lidar_loc", "update_lidar_loc_score");
    options_.min_init_confidence_ = yaml.GetValue<float>("lidar_loc", "min_init_confidence");

    // options_.filter_z_min_ = yaml.GetValue<double>("lidar_loc", "filter_z_min");
    // options_.filter_z_max_ = yaml.GetValue<double>("lidar_loc", "filter_z_max");
    // options_.filter_intensity_min_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_min");
    // options_.filter_intensity_max_ = yaml.GetValue<double>("lidar_loc", "filter_intensity_max");
    options_.lidar_loc_odom_th_ = yaml.GetValue<double>("lidar_loc", "lidar_loc_odom_th");

    options_.init_with_fp_ = yaml.GetValue<bool>("lidar_loc", "init_with_fp");
    options_.enable_parking_static_ = yaml.GetValue<bool>("lidar_loc", "enable_parking_static");
    options_.enable_icp_adjust_ = yaml.GetValue<bool>("lidar_loc", "enable_icp_adjust");
    options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
    options_.try_self_extrap_ = yaml.GetValue<bool>("lidar_loc", "try_self_extrap");

    lidar_loc::grid_search_angle_step = yaml.GetValue<double>("lidar_loc", "grid_search_angle_step");
    lidar_loc::grid_search_angle_range = yaml.GetValue<double>("lidar_loc", "grid_search_angle_range");

    /// 初始化容错搜索参数
    options_.init_search_enable_ = yaml.GetValue<bool>("lidar_loc", "init_search_enable", true);
    options_.init_search_xy_grid_ = yaml.GetValue<int>("lidar_loc", "init_search_xy_grid", 3);
    options_.init_search_xy_step_ = yaml.GetValue<double>("lidar_loc", "init_search_xy_step", 1.0);
    options_.init_search_yaw_range_ = yaml.GetValue<double>("lidar_loc", "init_search_yaw_range", 360.0);
    options_.init_search_yaw_step_ = yaml.GetValue<double>("lidar_loc", "init_search_yaw_step", 10.0);
    options_.init_search_stop_conf_ = yaml.GetValue<double>("lidar_loc", "init_search_stop_conf", 0.8);

    /// small_gicp 配准参数
    options_.gicp_fine_target_res_ = yaml.GetValue<double>("lidar_loc", "gicp_fine_target_res", 0.3);
    options_.gicp_fine_max_corr_dist_ = yaml.GetValue<double>("lidar_loc", "gicp_fine_max_corr_dist", 1.0);
    options_.gicp_fine_max_iterations_ = yaml.GetValue<int>("lidar_loc", "gicp_fine_max_iterations", 50);
    options_.gicp_rough_target_res_ = yaml.GetValue<double>("lidar_loc", "gicp_rough_target_res", 2.0);
    options_.gicp_rough_max_corr_dist_ = yaml.GetValue<double>("lidar_loc", "gicp_rough_max_corr_dist", 4.0);
    options_.gicp_rough_max_iterations_ = yaml.GetValue<int>("lidar_loc", "gicp_rough_max_iterations", 10);
    options_.gicp_source_res_ = yaml.GetValue<double>("lidar_loc", "gicp_source_res", 0.3);
    options_.gicp_num_threads_ = yaml.GetValue<int>("lidar_loc", "gicp_num_threads", 4);

    /// 局部 yaw 精搜索参数
    options_.yaw_refine_enable_ = yaml.GetValue<bool>("lidar_loc", "yaw_refine_enable", true);
    options_.yaw_refine_range_ = yaml.GetValue<double>("lidar_loc", "yaw_refine_range", 5.0);
    options_.yaw_refine_step_ = yaml.GetValue<double>("lidar_loc", "yaw_refine_step", 0.5);
    options_.yaw_refine_trigger_conf_ = yaml.GetValue<double>("lidar_loc", "yaw_refine_trigger_conf", 0.85);
    options_.yaw_refine_interval_ = yaml.GetValue<double>("lidar_loc", "yaw_refine_interval", 1.0);
    LLOG_INFO(logging::kLidarLoc, "yaw refine: enable={}, range=±{} deg, step={} deg, trigger_conf={}, interval={} s",
              options_.yaw_refine_enable_, options_.yaw_refine_range_, options_.yaw_refine_step_,
              options_.yaw_refine_trigger_conf_, options_.yaw_refine_interval_);

    LLOG_INFO(logging::kLidarLoc, "min init confidence: {}", options_.min_init_confidence_);
    LLOG_INFO(logging::kLidarLoc, "init search: enable={}, xy grid={}x{} step={}, yaw range={}, step={}, stop_conf={}",
              options_.init_search_enable_, options_.init_search_xy_grid_, options_.init_search_xy_grid_,
              options_.init_search_xy_step_, options_.init_search_yaw_range_, options_.init_search_yaw_step_,
              options_.init_search_stop_conf_);

    std::string map_policy = yaml.GetValue<std::string>("maps", "dyn_cloud_policy");
    if (map_policy == "short") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::SHORT;
    } else if (map_policy == "long") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::LONG;
    } else if (map_policy == "persistent") {
        options_.map_option_.policy_ = TiledMap::DynamicCloudPolicy::PERSISTENT;
    }

    options_.map_option_.delete_when_unload_ = yaml.GetValue<bool>("maps", "delete_when_unload");
    options_.map_option_.load_dyn_cloud_ = yaml.GetValue<bool>("maps", "load_dyn_cloud");
    options_.map_option_.save_dyn_when_quit_ = yaml.GetValue<bool>("maps", "save_dyn_when_quit");
    options_.map_option_.save_dyn_when_unload_ = yaml.GetValue<bool>("maps", "save_dyn_when_unload");

    map_ = std::make_shared<TiledMap>(options_.map_option_);
    /// 地图索引/点云加载失败必须中止初始化：否则会得到一个“看着加载成功、
    /// 实际没有任何点”的空地图，后续 GICP 匹配莫名失败，根因极难定位。
    if (!map_->LoadMapIndex()) {
        LLOG_ERROR(logging::kLidarLoc, "failed to load map from {}", options_.map_option_.map_path_);
        map_.reset();
        return false;
    }

    auto fps = map_->GetAllFP();
    if (!fps.empty()) {
        map_->LoadOnPose(fps.front().pose_);
        /// 更新一次地图，保证有初始数据
        UpdateGlobalMap();
    }

    /// load recover pose if exist
    if (PathExists(options_.recover_pose_path_)) {
        std::ifstream fin(options_.recover_pose_path_);
        double data[7] = {0, 0, 0, 0, 0, 0, 1};
        for (int i = 0; i < 7; ++i) {
            fin >> data[i];
        }

        SE3 pose(Quatd(data[6], data[3], data[4], data[5]), Vec3d(data[0], data[1], data[2]));
        FunctionalPoint fp_recover;
        fp_recover.name_ = "recover";
        fp_recover.pose_ = pose;
        map_->AddFP(fp_recover);
    }

    update_map_thread_ = std::thread([this]() { LidarLoc::UpdateMapThread(); });

    return true;
}

bool LidarLoc::ProcessCloud(CloudPtr cloud_input) {
    assert(cloud_input != nullptr);

    if (cloud_input->empty() || cloud_input->size() < 50) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "loc input is empty or invalid, sz: {}", cloud_input->size());
        return false;
    }

    // CloudPtr cloud(new PointCloudType);
    // pcl::VoxelGrid<PointType> voxel;

    // float sz = 0.1;
    // voxel.setLeafSize(sz, sz, sz);
    // voxel.setInputCloud(cloud_input);
    // voxel.filter(*cloud);

    current_scan_ = cloud_input;

    Align(cloud_input);
    return true;
}

NavState LidarLoc::GetState() {
    UL lock_res(result_mutex_);
    NavState ns;
    ns.SetPose(current_abs_pose_);
    ns.timestamp_ = current_timestamp_;
    ns.confidence_ = current_score_;

    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        auto s = lo_pose_queue_.back();
        ns.SetVel(s.GetVel());
    }

    return ns;
}

bool LidarLoc::ProcessDR(const NavState& state) {
    // 未初始化成功的数据不接收
    if (!state.pose_is_ok_) {
        return false;
    }

    // DR数据check
    UL lock(dr_pose_mutex_);
    if (!dr_pose_queue_.empty()) {
        const double last_stamp = dr_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            return false;
        }
    }

    dr_pose_queue_.emplace_back(state);
    while (dr_pose_queue_.size() >= 1000) {
        dr_pose_queue_.pop_front();
    }

    return true;
}

bool LidarLoc::ProcessLO(const NavState& state) {
    /// 理论上相对定位是按时间顺序到达的
    UL lock(lo_pose_mutex_);
    if (!lo_pose_queue_.empty()) {
        const double last_stamp = lo_pose_queue_.back().timestamp_;
        if (state.timestamp_ < last_stamp) {
            LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000,
                               "当前相对定位的结果的时间戳应当比上一个时间戳数值大，实际相减得{}",
                               state.timestamp_ - last_stamp);
            return false;
        }
    }

    lo_pose_queue_.emplace_back(state);

    while (lo_pose_queue_.size() >= 50) {
        lo_pose_queue_.pop_front();
    }

    if (state.lidar_odom_reliable_ == false) {
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
    } else {
        if (state.lidar_odom_reliable_ && lo_reliable_cnt_ > 0) {
            lo_reliable_cnt_--;
        }

        if (lo_reliable_cnt_ == 0) {
            lo_reliable_ = true;
        }
    }

    return true;
}

bool LidarLoc::YawSearch(SE3& pose, double& confidence, CloudPtr input, CloudPtr output) {
    SE3 init_pose = pose;
    auto RPYXYZ = math::SE3ToRollPitchYaw(init_pose);
    double init_yaw = RPYXYZ.yaw;

    confidence = 0;
    bool yaw_search_success = false;

    int step = lidar_loc::grid_search_angle_step;
    double radius = lidar_loc::grid_search_angle_range * constant::kDEG2RAD;
    double angle_search_step = 2 * radius / step;

    std::vector<double> searched_yaw;
    std::vector<double> scores(step);
    std::vector<int> index;
    std::vector<SE3> pose_opti(step);

    for (int i = 0; i < step; ++i) {
        double search_yaw = init_yaw + i * angle_search_step - radius;
        searched_yaw.emplace_back(search_yaw);
        index.emplace_back(i);
    }

    LLOG_DEBUG(logging::kLidarLoc, "init yaw: {}, p: {}, ro: {}, search from {} to {}", init_yaw, RPYXYZ.pitch,
               RPYXYZ.roll, searched_yaw.front(), searched_yaw.back());

    /// 粗分辨率
    std::for_each(index.begin(), index.end(), [&](int i) {
        double fitness_score = 0;
        RPYXYZ.yaw = searched_yaw[i];
        SE3 pose_esti = math::XYZRPYToSE3(RPYXYZ);

        Localize(pose_esti, fitness_score, input, output, true);

        scores[i] = fitness_score;
        pose_opti[i] = pose_esti;
    });

    // find best match
    auto best_score_idx = std::max_element(scores.begin(), scores.end()) - scores.begin();
    confidence = scores.at(best_score_idx);
    pose = pose_opti.at(best_score_idx);

    /// 高分辨率
    if (confidence > options_.min_init_confidence_) {
        Localize(pose, confidence, input, output, false);
    }

    if (confidence > options_.min_init_confidence_) {
        LLOG_INFO(logging::kLidarLoc, "init success, score: {}, th={}", confidence, options_.min_init_confidence_);
        Eigen::Vector3d suc_translation = pose.translation();
        Eigen::Matrix3d suc_rotation_matrix = pose.rotationMatrix();
        double suc_x = suc_translation.x();
        double suc_y = suc_translation.y();
        double suc_yaw = atan2(suc_rotation_matrix(1, 0), suc_rotation_matrix(0, 0));
        LLOG_INFO(logging::kLidarLoc, "localization init success, pose: {}, {}, {}, conf: {}", suc_x, suc_y, suc_yaw,
                  confidence);
        yaw_search_success = true;
    }

    return yaw_search_success;
}

bool LidarLoc::InitWithFP(CloudPtr input, const SE3& fp_pose) {
    assert(input != nullptr && !input->empty());

    // 使用功能点的位置进行定位初始化
    double fitness_score;
    SE3 pose_esti = fp_pose;
    CloudPtr output_cloud(new PointCloudType);
    // loc_inited_ = YawSearch(pose_esti, fitness_score, input, output_cloud);
    loc_inited_ = Localize(pose_esti, fitness_score, input, output_cloud);

    if (loc_inited_) {
        FinishInit(pose_esti, fitness_score, input);
        LLOG_INFO(logging::kLidarLoc, "fitness_score is: {}, global_pose is: {}", fitness_score,
                  fp_pose.translation().transpose());
        //  定位成功，则清空失败记录
        fp_init_fail_pose_vec_.clear();
    } else {
        // 添加失败历史记录
        // 参考系统一：Align 的“原地重试节流”用 current_dr_pose_(odom) 与之比较，
        // 经典 odom 模型下 W≠I，若这里再存 map 系的 FP 位姿会造成跨系相减、节流失效。
        LLOG_ERROR(logging::kLidarLoc, "init failed, score: {}", fitness_score);
        if (current_dr_pose_set_) {
            fp_init_fail_pose_vec_.emplace_back(current_dr_pose_);
        }
        fp_last_tried_time_ = 1e-6 * static_cast<double>(input->header.stamp);
    }
    return loc_inited_;
}

void LidarLoc::FinishInit(const SE3& pose, double confidence, const CloudPtr& input) {
    current_timestamp_ = math::ToSec(input->header.stamp);
    localization_result_.confidence_ = confidence;
    current_abs_pose_ = pose;
    localization_result_.pose_ = pose;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = true;
    localization_result_.status_ = LocalizationStatus::GOOD;

    last_abs_pose_set_ = true;
    last_abs_pose_ = pose;

    current_score_ = confidence;
    LLOG_INFO(logging::kLidarLoc, "[Loc init pose]: {}", pose.translation().transpose());
    map_height_ = pose.translation()[2];

    if (current_lo_pose_set_) {
        // 设置上一次的相对定位结果
        last_lo_pose_ = current_lo_pose_;
        last_lo_pose_set_ = true;

        last_dr_pose_ = current_dr_pose_;
        last_dr_pose_set_ = true;
    }
}

bool LidarLoc::RunGicp(const CloudPtr& input, const SE3& init_pose, SE3& out_pose, double& confidence,
                       bool use_rough_res) {
    UL lock(match_mutex_);
    MatcherPtr matcher = use_rough_res ? gicp_rough_ : gicp_fine_;
    if (matcher == nullptr || !matcher->HasTarget()) {
        return false;
    }

    return matcher->Align(input, init_pose, out_pose, confidence);
}

bool LidarLoc::RunGicpPre(const std::shared_ptr<small_gicp::PointCloud>& source_pp, const SE3& init_pose, SE3& out_pose,
                          double& confidence, bool use_rough_res) {
    UL lock(match_mutex_);
    MatcherPtr matcher = use_rough_res ? gicp_rough_ : gicp_fine_;
    if (matcher == nullptr || !matcher->HasTarget()) {
        return false;
    }

    return matcher->AlignPre(source_pp, init_pose, out_pose, confidence);
}

bool LidarLoc::YawRefine(const CloudPtr& input, SE3& pose, double& confidence) {
    const double range_rad = options_.yaw_refine_range_ * constant::kDEG2RAD;
    const double step_rad = options_.yaw_refine_step_ * constant::kDEG2RAD;
    if (range_rad <= 0.0 || step_rad <= 0.0) {
        return false;
    }
    const int n = std::max(1, static_cast<int>(std::round(range_rad / step_rad)));

    /// 源点云只预处理一次，供所有 yaw 候选复用
    std::shared_ptr<small_gicp::PointCloud> source_pp;
    {
        UL lock(match_mutex_);
        if (gicp_fine_ == nullptr || !gicp_fine_->HasTarget()) {
            return false;
        }
        source_pp = gicp_fine_->PreprocessSource(input);
    }
    if (source_pp == nullptr || source_pp->empty()) {
        return false;
    }

    double best_conf = confidence;
    SE3 best_pose = pose;
    double best_dyaw = 0.0;
    bool improved = false;

    for (int k = -n; k <= n; ++k) {
        if (k == 0) {
            continue;
        }
        const double dyaw = k * step_rad;
        /// 绕 map 系 z 轴做小角度偏移后逐候选细 GICP
        const SE3 cand = SE3(SO3::exp(Vec3d(0.0, 0.0, dyaw)), Vec3d::Zero()) * pose;
        SE3 res = cand;
        double conf = 0.0;
        if (!RunGicpPre(source_pp, cand, res, conf, false)) {
            continue;
        }
        if (conf > best_conf) {
            best_conf = conf;
            best_pose = res;
            best_dyaw = dyaw;
            improved = true;
        }
    }

    if (improved) {
        LLOG_DEBUG(logging::kLidarLoc, "yaw refine: offset {} deg, conf {} -> {}", best_dyaw * constant::kRAD2DEG,
                   confidence, best_conf);
        pose = best_pose;
        confidence = best_conf;
    }
    return improved;
}

bool LidarLoc::SearchAndInit(CloudPtr input, const SE3& seed) {
    assert(input != nullptr && !input->empty());
    if (map_ == nullptr) {
        LLOG_ERROR(logging::kLidarLoc, "SearchAndInit: map not loaded");
        return false;
    }

    const double t_now = math::ToSec(input->header.stamp);
    /// 失败重试节流：避免每帧都做全量搜索
    if (init_last_search_time_ > 0 && (t_now - init_last_search_time_) < 0.5) {
        return false;
    }

    /// 确保 seed 附近地图已载入且 GICP target 就绪（重定位/初值可能在任意位置）
    map_->LoadOnPose(seed);
    UpdateGlobalMap();

    // —— 1) 未开启搜索：只对 seed 做一次细配准，直接接受（保持旧行为）——
    if (!options_.init_search_enable_) {
        double conf = 0.0;
        SE3 pose_result = seed;
        RunGicp(input, seed, pose_result, conf, false);
        init_last_search_time_ = t_now;
        loc_inited_ = true;
        has_set_pose_ = true;
        FinishInit(pose_result, conf, input);
        LLOG_INFO(logging::kLidarLoc, "manual init (search disabled), conf={}", conf);
        return true;
    }

    // —— 2) 在 seed 周围构建 xy×yaw 搜索窗口，粗 GICP 快速打分 ——
    // yaw 偏移(deg)，覆盖 [-half,+half]，按 |offset| 升序（首个候选即 seed 本身，配合提前退出）
    std::vector<double> yaw_offsets;
    const double yaw_range = options_.init_search_yaw_range_;
    const double yaw_step = options_.init_search_yaw_step_;
    if (yaw_step > 1e-6) {
        const double half = yaw_range * 0.5;
        const int n = std::max(1, static_cast<int>(std::round(yaw_range / yaw_step)));
        for (int k = 0; k < n; ++k) {
            yaw_offsets.push_back(-half + k * yaw_step);
        }
        std::sort(yaw_offsets.begin(), yaw_offsets.end(),
                  [](double a, double b) { return std::fabs(a) < std::fabs(b); });
    } else {
        yaw_offsets.push_back(0.0);
    }

    // xy 偏移(本地系)，同样按 |offset| 升序
    const int g = std::max(1, options_.init_search_xy_grid_);
    const double xy_step = options_.init_search_xy_step_;
    std::vector<double> xy_offsets;
    for (int i = 0; i < g; ++i) {
        xy_offsets.push_back((i - (g - 1) / 2.0) * xy_step);
    }
    std::sort(xy_offsets.begin(), xy_offsets.end(), [](double a, double b) { return std::fabs(a) < std::fabs(b); });

    /// 源点云按粗匹配参数只预处理一次，窗口内全部候选复用
    std::shared_ptr<small_gicp::PointCloud> pp_rough;
    {
        UL lock(match_mutex_);
        if (gicp_rough_ == nullptr || !gicp_rough_->HasTarget()) {
            LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "SearchAndInit: gicp target not ready");
            return false;
        }
        pp_rough = gicp_rough_->PreprocessSource(input);
    }
    if (pp_rough == nullptr || pp_rough->empty()) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "SearchAndInit: preprocess source failed");
        return false;
    }

    double best_conf = 0.0;
    SE3 best_pose = seed;
    int tried = 0;
    bool early_stop = false;
    for (const double d_yaw : yaw_offsets) {
        const SO3 rel_r = SO3::exp(Vec3d(0.0, 0.0, d_yaw * constant::kDEG2RAD));
        for (const double dx : xy_offsets) {
            for (const double dy : xy_offsets) {
                const SE3 cand = seed * SE3(rel_r, Vec3d(dx, dy, 0.0));
                double conf_c = 0.0;
                SE3 res_c = cand;
                if (!RunGicpPre(pp_rough, cand, res_c, conf_c, true)) {  // 粗 GICP 快速打分
                    continue;
                }
                ++tried;
                if (conf_c > best_conf) {
                    best_conf = conf_c;
                    best_pose = res_c;
                }
                if (conf_c >= options_.init_search_stop_conf_) {
                    early_stop = true;
                    break;
                }
            }
            if (early_stop) {
                break;
            }
        }
        if (early_stop) {
            break;
        }
    }
    LLOG_INFO(logging::kLidarLoc, "window search tried {} candidates, best conf={}", tried, best_conf);

    // —— 3) 只对最优候选做一次细 GICP 精配准 ——
    double fine_conf = best_conf;
    SE3 fine_pose = best_pose;
    if (RunGicp(input, best_pose, fine_pose, fine_conf, false) && fine_conf > best_conf) {
        best_conf = fine_conf;
        best_pose = fine_pose;
    }

    // —— 4) 局部 yaw 精搜索：±range 细步长扫 yaw，补偿 GICP 收敛域内的残差 ——
    YawRefine(input, best_pose, best_conf);

    init_last_search_time_ = t_now;

    // —— 5) 分数校验：达标才接受，否则拒绝并保持未初始化 ——
    if (best_conf >= options_.min_init_confidence_) {
        loc_inited_ = true;
        has_set_pose_ = true;
        FinishInit(best_pose, best_conf, input);
        LLOG_INFO(logging::kLidarLoc, "manual init accepted by search, conf={}", best_conf);
        return true;
    }

    LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000,
                       "manual init REJECTED, best conf={} < {}"
                       " (若误拒可调低 lidar_loc.min_init_confidence，若误收则调高/调低 init_search_stop_conf)",
                       best_conf, options_.min_init_confidence_);
    return false;
}

void LidarLoc::ResetLastPose(const SE3& last_pose) {
    last_abs_pose_ = last_pose;

    // TODO：清空动态图层

    return;
}

bool LidarLoc::TryOtherSolution(CloudPtr input, SE3& pose) {
    double fitness_score;
    SE3 pose_esti = pose;
    CloudPtr output_cloud(new PointCloudType);

    bool loc_success = Localize(pose_esti, fitness_score, input, output_cloud);

    if (loc_success) {
        // 激光重置逻辑
        float score_th = std::min(1.5 * current_score_, current_score_ + 0.3);
        if (fitness_score > score_th && fitness_score > 0.6) {  // 内点比例量纲，[0,1]
            // 显著好于现在的估计
            LLOG_WARN(logging::kLidarLoc, "rtk solution is significantly better: {} {}", fitness_score, current_score_);
            pose = pose_esti;
            localization_result_.lidar_loc_smooth_flag_ = false;
            return true;
        } else {
            LLOG_DEBUG(logging::kLidarLoc, "not using rtk solution: {} {}", fitness_score, current_score_);
            return false;
        }
    }
    return false;
}

bool LidarLoc::UpdateGlobalMap() {
    /// 细匹配：0.3m 目标分辨率 + 1.0m 对应距离
    MatcherPtr fine(new SmallGicpMatcher());
    fine->options_.target_res_ = options_.gicp_fine_target_res_;
    fine->options_.max_corr_dist_ = options_.gicp_fine_max_corr_dist_;
    fine->options_.max_iterations_ = options_.gicp_fine_max_iterations_;
    fine->options_.source_res_ = options_.gicp_source_res_;
    fine->options_.num_threads_ = options_.gicp_num_threads_;
    map_->SetNewTargetForGICP(fine);

    /// 粗匹配：初始化 xy+yaw 搜索窗口的快速打分（低分辨率+大对应距离，单次耗时远低于细匹配）
    MatcherPtr rough(new SmallGicpMatcher());
    rough->options_.target_res_ = options_.gicp_rough_target_res_;
    rough->options_.max_corr_dist_ = options_.gicp_rough_max_corr_dist_;
    rough->options_.max_iterations_ = options_.gicp_rough_max_iterations_;
    rough->options_.source_res_ = options_.gicp_source_res_;
    rough->options_.num_threads_ = options_.gicp_num_threads_;
    map_->SetNewTargetForGICP(rough);

    {
        UL lock(match_mutex_);
        gicp_fine_ = fine;
        gicp_rough_ = rough;
    }

    if (options_.enable_icp_adjust_) {
        ICPType::Ptr icp(new ICPType());
        CloudPtr map_cloud(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel;
        auto sz = 0.5;
        voxel.setLeafSize(sz, sz, sz);
        voxel.setInputCloud(map_->GetAllMap());
        voxel.filter(*map_cloud);
        icp->setInputTarget(map_cloud);
        icp->setMaximumIterations(4);
        icp->setTransformationEpsilon(0.01);
        pcl_icp_ = icp;
    }

    return true;
}

void LidarLoc::UpdateMapThread() {
    LLOG_INFO(logging::kLidarLoc, "UpdateMapThread thread is running");
    while (!update_map_quit_) {
        if (map_->MapUpdated() || map_->DynamicMapUpdated()) {
            UpdateGlobalMap();

            if (ui_) {
                ui_->UpdatePointCloudGlobal(map_->GetStaticCloud());
                ui_->UpdatePointCloudDynamic(map_->GetDynamicCloud());
            }

            map_->CleanMapUpdate();
        }
        usleep(10000);
    }
}

void LidarLoc::SetInitialPose(SE3 init_pose) {
    UL lock(initial_pose_mutex_);
    loc_inited_ = false;
    // map_->ClearMap();

    initial_pose_set_ = true;
    initial_pose_ = init_pose;
    LLOG_INFO(logging::kLidarLoc, "Set initial pose is: {}", initial_pose_.translation().transpose());
}

void LidarLoc::Align(const CloudPtr& input) {
    // 输入必须非空
    assert(input != nullptr);

    // 点云去畸变定到了结束时间，所以该点云的定位也是到结束时间的
    double current_time = math::ToSec(input->header.stamp) + lo::lidar_time_interval;
    current_timestamp_ = current_time;

    LLOG_DEBUG(logging::kLidarLoc, "current time: {:.12f}, size: {}", current_timestamp_, input->size());

    /// 设置当前帧对应的rel_pose
    if (!AssignLOPose(current_time)) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "assign LO pose failed");
    }

    if (!AssignDRPose(current_time)) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "assign DR pose failed");
    }

    /// 1. 车辆静止处理
    if (parking_ && loc_inited_) {
        LLOG_DEBUG(logging::kLidarLoc, "车辆静止，不做匹配");

        UpdateState(input);
        current_abs_pose_ = last_abs_pose_;
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);

        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_time;
        localization_result_.pose_ = current_abs_pose_;

        if (options_.enable_parking_static_) {
            localization_result_.is_parking_ = true;
            localization_result_.valid_ = true;
        }
        return;
    }

    /// 2. 初始化处理
    if (!loc_inited_) {
        UL lock_init(initial_pose_mutex_);
        LLOG_DEBUG(logging::kLidarLoc, "initing lidarloc");
        SetInitRltState();

        if (initial_pose_set_) {
            /// 手动/外部初值：带 xy+yaw 搜索窗口与分数校验的初始化
            if (SearchAndInit(input, initial_pose_)) {
                LLOG_INFO(logging::kLidarLoc, "init with external pose: {}", initial_pose_.translation().transpose());
                initial_pose_set_ = false;
                return;
            }
            /// 手动初值未通过：不自动落到 FP（避免错锁），保持 INITIALIZING 等待重试/新初值
            /// （SearchAndInit 内部已按 0.5s 节流并在真实尝试后打印 REJECTED 原因）
            return;
        }
        // 不使用功能点
        if (options_.init_with_fp_) {
            /// 从功能点初始化
            /// 如果之前尝试过，那么需要间隔一段时间再进行搜索
            if (!fp_init_fail_pose_vec_.empty() && current_dr_pose_set_) {
                SE3 last_tried_pose = fp_init_fail_pose_vec_.back();
                bool should_try =
                    (current_time - fp_last_tried_time_) > 2.0 ||
                    (current_dr_pose_.translation() - last_tried_pose.translation()).norm() > 0.3 ||
                    (current_dr_pose_.so3().inverse() * last_tried_pose.so3()).log().norm() > 10 * M_PI / 180.0;
                if (!should_try) {
                    LLOG_DEBUG(logging::kLidarLoc, "skip trying init, please move to another place.");
                    return;
                }
            } else {
                LLOG_DEBUG(logging::kLidarLoc, "fp tried pose: {}, dr pose set: {}", fp_init_fail_pose_vec_.size(),
                           current_dr_pose_set_);
            }

            auto all_fps = map_->GetAllFP();
            bool fp_init_success = false;
            for (const auto& fp : all_fps) {
                map_->LoadOnPose(fp.pose_);
                if (InitWithFP(input, fp.pose_)) {
                    LLOG_INFO(logging::kLidarLoc, "init with fp: {}", fp.name_);
                    fp_init_success = true;
                    break;
                }
            }

            if (!fp_init_success) {
                LLOG_WARN(logging::kLidarLoc, "FP init failed.");
                if (current_dr_pose_set_) {
                    LLOG_DEBUG(logging::kLidarLoc, "record fp failed time: {:.12f}, pose: {}", current_time,
                               current_dr_pose_.translation().transpose());
                    fp_last_tried_time_ = current_time;
                    fp_init_fail_pose_vec_.emplace_back(current_dr_pose_);
                }
            } else {
                fp_last_tried_time_ = 0;
                fp_init_fail_pose_vec_.clear();
            }
        }

        /// 初始化未成功时，不往下走流程
        return;
    }

    /// 4. 设置当前帧对应的 pose guess
    /// NOTE: LO设置预测的位置和LidarLoc自身递推设置预测的方法并不完全一致，自身外推容易受噪声影响

    SE3 guess_from_lo = last_abs_pose_;
    if (last_lo_pose_set_ && current_lo_pose_set_) {
        // 如果有里程计，则用两个时刻的相对定位来递推，估计一个当前pose的初值
        const SE3 delta = last_lo_pose_.inverse() * current_lo_pose_;
        guess_from_lo = last_abs_pose_ * delta;

        LLOG_DEBUG(logging::kLidarLoc, "lo guess: last lo {}, cur lo {}, lo motion {}, last abs {}, guess {}",
                   last_lo_pose_.translation().transpose(), current_lo_pose_.translation().transpose(),
                   delta.translation().transpose(), last_abs_pose_.translation().transpose(),
                   guess_from_lo.translation().transpose());
    }

    SE3 guess_from_self = guess_from_lo;
    if (lidar_loc_pose_queue_.size() >= 2) {
        SE3 pred;
        TimedPose match;
        if (math::PoseInterp<TimedPose>(
                current_time, lidar_loc_pose_queue_, [](const TimedPose& p) { return p.timestamp_; },
                [](const TimedPose& p) { return p.pose_; }, pred, match, 2.0)) {
            guess_from_self = pred;
        }
    }

    // SE3 guess_from_dr = guess_from_lo;
    // if (last_dr_pose_set_ && current_dr_pose_set_) {
    //     const SE3 delta = last_dr_pose_.inverse() * current_dr_pose_;
    //     guess_from_dr = last_abs_pose_ * delta;
    //     // guess_from_dr.translation()[2] = 0;
    // }

    // bool try_dr = false;
    // if (((guess_from_dr.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //      (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //     LOG(INFO) << "trying dr pose: " << guess_from_dr.translation().transpose() << ", "
    //               << (guess_from_dr.so3().inverse() * guess_from_lo.so3()).log().norm()
    //               << ", vel_norm: " << current_vel_b_.norm();
    //     try_dr = true;
    // }

    bool try_self = false;
    // if (options_.try_self_extrap_) {
    //     if (((guess_from_self.translation() - guess_from_lo.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm() >= try_other_guess_rot_th_) &&
    //         ((guess_from_dr.translation() - guess_from_self.translation()).norm() >= try_other_guess_trans_th_ ||
    //          (guess_from_dr.so3().inverse() * guess_from_self.so3()).log().norm() >= try_other_guess_rot_th_)) {
    //         LOG(INFO) << "trying self extrap pose: " << guess_from_self.translation().transpose() << ", "
    //                   << (guess_from_self.so3().inverse() * guess_from_lo.so3()).log().norm();
    //         try_self = true;
    //     }
    // }

    /// 5. 载入地图, 与地图匹配定位
    /// 尝试各种初始估计
    CloudPtr output_cloud(new PointCloudType);
    double fitness_score = 0;
    SE3 current_pose_esti = guess_from_lo;
    bool loc_success_lo, loc_success_self, loc_success_dr;
    loc_success_lo = loc_success_self = loc_success_dr = false;
    bool loc_success = false;

    /// 注意load on pose存在滞后，优先load on DR
    map_->LoadOnPose(guess_from_lo);

    loc_success_lo = Localize(current_pose_esti, fitness_score, input, output_cloud);  // LO 那个肯定会算
    double score_lo = fitness_score;

    SE3 res_of_lo = current_pose_esti;
    SE3 res_of_dr = current_pose_esti;
    SE3 res_of_self = current_pose_esti;
    double score_dr = 0;

    // 先尝试外部预测，最后用自身
    // if (try_dr) {
    //     /// 尝试DR外推的pose
    //     res_of_dr = guess_from_dr;
    //     loc_success_dr = Localize(res_of_dr, score_dr, input, output_cloud);
    //     if (score_dr > (fitness_score - 0.1)) {
    //         current_pose_esti = res_of_dr;
    //         fitness_score = score_dr;
    //         LOG(INFO) << "take dr guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_dr << ", v_norm: " << current_vel_b_.norm();
    //     }
    // }

    // 用纯激光定位有点太抖了，加一些权重
    // NOTE: 修正量系数过小（如 0.1）会让 yaw 残差多帧都收敛不完，表现为长期对不齐
    Vec6d delta = (guess_from_lo.inverse() * current_pose_esti).log();
    SE3 esti_balanced = guess_from_lo * SE3::exp(delta * 0.5);
    current_pose_esti = esti_balanced;

    /// 局部 yaw 精搜索：分值不足时（存在 GICP 收敛域外的 yaw 残差），在当前估计附近
    /// 小范围扫 yaw 取最优；按时间节流，避免拖慢定位帧率
    if (loc_success_lo && options_.yaw_refine_enable_ && fitness_score < options_.yaw_refine_trigger_conf_ &&
        (yaw_refine_last_time_ < 0 || current_time - yaw_refine_last_time_ > options_.yaw_refine_interval_)) {
        yaw_refine_last_time_ = current_time;
        SE3 refined_pose = current_pose_esti;
        double refined_conf = fitness_score;
        if (YawRefine(input, refined_pose, refined_conf)) {
            current_pose_esti = refined_pose;
            fitness_score = refined_conf;
        }
    }

    // double score_self = 0;
    // if (try_self) {
    //     /// 尝试自身外推的pose
    //     LOG(INFO) << "localize with extrap";

    //     res_of_self = guess_from_self;
    //     loc_success_self = Localize(res_of_self, score_self, input, output_cloud);

    //     // 避免分值接近但长时间采信自身预测，此处更相信外部预测源
    //     if (score_self > (fitness_score + 0.1)) {
    //         current_pose_esti = res_of_self;
    //         fitness_score = score_self;
    //         LOG(INFO) << "take self guess: " << current_pose_esti.translation().transpose()
    //                   << " , confidence: " << score_self;
    //     }
    // }

    /// NOTE 如果LO, DR出发点和收敛点不同，但分值相近，说明场景可能处在退化状态，此时使用DR预测的Pose
    // if (try_dr && (res_of_lo.translation() - res_of_dr.translation()).head<2>().norm() > 0.2 &&
    //     fabs(score_lo - score_dr) < 0.2 && score_lo < 1.2) {
    //     LOG(WARNING) << "判定激光定位进入退化状态，现在会使用DR递推pose而不是激光定位位置";
    //     current_pose_esti = guess_from_dr;
    // }

    if (options_.force_2d_) {
        PoseRPYD RPYXYZ = math::SE3ToRollPitchYaw(current_pose_esti);
        RPYXYZ.roll = 0;
        RPYXYZ.pitch = 0;
        RPYXYZ.z = 0;
        current_pose_esti = math::XYZRPYToSE3(RPYXYZ);
    }

    // if (options_.with_height_) {
    //     current_pose_esti.translation()[2] = map_height_;
    //     LOG(INFO) << "adjust current pose to : " << current_pose_esti.translation().transpose();
    // }

    current_abs_pose_ = current_pose_esti;
    current_score_ = fitness_score;
    double delta_rel_abs_pose = 0;
    bool lidar_loc_odom_valid = true;

    if (loc_success_lo || loc_success_self || loc_success_dr) {
        loc_success = true;
    } else {
        LLOG_DEBUG(logging::kLidarLoc, "loc success is false.");
    }

    if (loc_success) {
        lidar_loc_odom_valid = CheckLidarOdomValid(current_pose_esti, delta_rel_abs_pose);
        last_timestamp_ = current_timestamp_;  // 成功时，更新上一时刻激光定位时间
        match_fail_count_ = 0;
    } else {
        current_score_ = fitness_score;
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "localization failed! score: {:.3f}", current_score_);

        ///  若连续3帧匹配失败就设一个大分值
        ++match_fail_count_;
    }

    /// 确定激光定位是否满足平滑性要求
    Vec3d dpred = current_abs_pose_.translation() - guess_from_self.translation();
    if (fabs(dpred[0]) < 0.5 && fabs(dpred[1]) < 0.5 &&
        (current_abs_pose_.so3().inverse() * guess_from_self.so3()).log().norm() < 2.0 * M_PI / 180.0) {
        localization_result_.lidar_loc_smooth_flag_ = true;
    } else {
        localization_result_.lidar_loc_smooth_flag_ = false;
    }

    localization_result_.lidar_loc_odom_reliable_ = lo_reliable_;
    localization_result_.is_parking_ = false;

    // if (ui_) {
    //     ui_->UpdatePredictPose(guess_from_lo);
    // }

    /// 7. 输出结果
    {
        UL lock(result_mutex_);
        localization_result_.timestamp_ = current_timestamp_;
        localization_result_.confidence_ = fitness_score;
        if (match_fail_count_ < 100) {
            localization_result_.lidar_loc_valid_ = true;
            localization_result_.status_ = LocalizationStatus::GOOD;
        } else if (match_fail_count_ >= 100 && match_fail_count_ < 300) {
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FOLLOWING_DR;
        } else {
            match_fail_count_ = 300;
            localization_result_.lidar_loc_valid_ = false;
            localization_result_.status_ = LocalizationStatus::FAIL;
        }

        localization_result_.lidar_loc_odom_delta_ = delta_rel_abs_pose;
        localization_result_.lidar_loc_odom_error_normal_ = lidar_loc_odom_valid;
        localization_result_.pose_ = current_pose_esti;
    }

    UpdateState(input);

    /// 8. 更新动态图层
    /// 条件：1. 定位成功 2. 与上次更新间隔一定距离 3. 匹配分值（内点比例）大于阈值
    bool score_cond = current_score_ > options_.update_lidar_loc_score_;

    if (options_.update_dynamic_cloud_ && loc_success &&
        (((current_pose_esti.translation() - last_dyn_upd_pose_.pose_.translation()).norm() >
          options_.update_kf_dis_) ||
         fabs(current_time - last_dyn_upd_pose_.timestamp_) > options_.update_kf_time_)) {
        if (score_cond /*  || update_cache_dis_ < options_.max_update_cache_dis_ */) {
            // LOG(INFO) << "passing through z filter, input:" << input->size();
            pcl::PassThrough<PointType> pass;
            pass.setInputCloud(input);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.5, options_.filter_z_max_);

            CloudPtr input_z_filter(new PointCloudType());
            pass.filter(*input_z_filter);

            if (!input_z_filter->empty()) {
                CloudPtr cloud_t(new PointCloudType());
                pcl::transformPointCloud(*input_z_filter, *cloud_t, current_pose_esti.matrix());

                // 以现在的scan来更新地图
                map_->UpdateDynamicCloud(cloud_t, true);

                last_dyn_upd_pose_.timestamp_ = current_time;
                last_dyn_upd_pose_.pose_ = current_pose_esti;
            }
        }
    }

    if (lidar_loc_pose_queue_.empty()) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    } else if (current_time > lidar_loc_pose_queue_.back().timestamp_) {
        lidar_loc_pose_queue_.emplace_back(current_time, current_abs_pose_);
    }

    while (lidar_loc_pose_queue_.size() > 1000) {
        lidar_loc_pose_queue_.pop_front();
    }

    ave_scores_.emplace_back(current_score_);
    while (ave_scores_.size() > 20) {
        ave_scores_.pop_front();
    }

    /// 9. save for recover pose
    recover_pose_out_.open(options_.recover_pose_path_);
    if (recover_pose_out_) {
        Vec3d t = current_pose_esti.translation();
        Quatd q = current_pose_esti.unit_quaternion();
        recover_pose_out_ << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " "
                          << q.w();
        recover_pose_out_.close();
    }
}

bool LidarLoc::CheckLidarOdomValid(const SE3& current_pose_esti, double& delta_posi) {
    delta_posi = ((last_lo_pose_.inverse() * current_lo_pose_).translation() -
                  (last_abs_pose_.inverse() * current_pose_esti).translation())
                     .head(2)
                     .norm();

    bool valid = true;

    if (delta_posi > options_.lidar_loc_odom_th_ && last_lo_pose_set_) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000,
                           "delta_rel_abs_pose is: {:.3f}, LO相对pose: {}, Lidar Loc计算相对pose: {}", delta_posi,
                           (last_lo_pose_.inverse() * current_lo_pose_).translation().transpose(),
                           (last_abs_pose_.inverse() * current_pose_esti).translation().transpose());
        lo_reliable_ = false;
        lo_reliable_cnt_ = 10;
        valid = false;
    }

    last_abs_pose_ = current_pose_esti;
    last_lo_pose_ = current_lo_pose_;
    last_lo_pose_set_ = true;
    last_dr_pose_ = current_dr_pose_;
    last_dr_pose_set_ = true;

    return valid;
}

bool LidarLoc::Localize(SE3& pose, double& confidence, CloudPtr input, CloudPtr output, bool use_rough_res) {
    bool loc_success = false;

    LLOG_DEBUG(logging::kLidarLoc, "loc from: {}", pose.translation().transpose());

    UL lock(match_mutex_);
    MatcherPtr matcher = use_rough_res ? gicp_rough_ : gicp_fine_;
    if (matcher == nullptr || !matcher->HasTarget()) {
        LLOG_WARN_THROTTLE(logging::kLidarLoc, 2000, "lidar loc target is null, skip");
        return false;
    }

    const SE3 guess_pose = pose;
    loc_success = matcher->Align(input, guess_pose, pose, confidence);
    Eigen::Matrix4f trans = pose.matrix().cast<float>();

    if (loc_success && output != nullptr) {
        pcl::transformPointCloud(*input, *output, pose.matrix().cast<float>());
    }

    if (options_.enable_icp_adjust_ && loc_inited_) {
        Eigen::Matrix4f adjust_trans;
        CloudPtr input_voxel(new PointCloudType);
        pcl::VoxelGrid<PointType> voxel_icp;

        double ls = 0.2;
        voxel_icp.setLeafSize(ls, ls, ls);
        voxel_icp.setInputCloud(input);
        voxel_icp.filter(*input_voxel);
        pcl_icp_->setInputSource(input_voxel);
        Timer::Evaluate([&]() { pcl_icp_->align(*output, trans); }, "pcl_icp adjust", true);
        adjust_trans = pcl_icp_->getFinalTransformation();

        Eigen::Matrix3f rotation_diff = trans.block<3, 3>(0, 0).transpose() * adjust_trans.block<3, 3>(0, 0);
        Eigen::AngleAxisf angle_axis(rotation_diff);
        float a = angle_axis.angle();
        float d = (trans.block<3, 1>(0, 3) - adjust_trans.block<3, 1>(0, 3)).norm();
        LLOG_DEBUG(logging::kLidarLoc, "icp adjust d: {}, a: {}", d, a);

        if (pcl_icp_->hasConverged() && std::fabs(d) <= 0.05 && std::fabs(a) <= 0.05) {
            LLOG_DEBUG(logging::kLidarLoc, "icp adjust trans set success");
            trans = adjust_trans;
            Eigen::Matrix3d rot = trans.block<3, 3>(0, 0).cast<double>();
            Quatd q_3d = Quatd(rot);
            q_3d.normalize();
            pose = SE3(q_3d, trans.block<3, 1>(0, 3).cast<double>());
        }
    }

    // 每帧明细在 DEBUG 级别；此处保留 1Hz 状态摘要便于运行时观察
    LLOG_INFO_THROTTLE(logging::kLidarLoc, 1000, "confidence: {:.3f}, t: {}, succ: {}", confidence,
                       pose.translation().transpose(), loc_success);

    return loc_success;
}

bool LidarLoc::CheckStatic(double timestamp) {
    if (parking_) {
        // if (current_vel_b_.norm() < common::options::lo::parking_speed) {
        // LOG(INFO) << "car is in static mode";
        static_count_++;
        if (static_count_ >= lo::parking_count) {
            static_count_ = 0;
            return false;
        }
        return true;
    } else {
        static_count_ = 0;
        return false;
    }
}

void LidarLoc::UpdateState(const CloudPtr& input) { last_timestamp_ = current_timestamp_; }

void LidarLoc::SetInitRltState() {
    UL lock(result_mutex_);
    localization_result_.confidence_ = 0.0;
    localization_result_.timestamp_ = current_timestamp_;
    localization_result_.lidar_loc_valid_ = false;
    localization_result_.status_ = LocalizationStatus::INITIALIZING;
}

bool LidarLoc::LocInited() {
    // UL lock( data_mutex_);
    return loc_inited_;
}

bool LidarLoc::AssignLOPose(double timestamp) {
    UL lock(lo_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    // 无法拿到最新的，插值结果都是外推出来的，和真实有时偏差会很大（╯﹏╰）
    // if (!lo_pose_queue_.empty()) {
    //     LOG(INFO) << "lo interp: " << timestamp << " " << lo_pose_queue_.back().timestamp_ << " "
    //               << lo_pose_queue_.back().pose_.translation().transpose();
    // }

    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, lo_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        current_lo_pose_ = interp_pose;
        current_lo_pose_set_ = true;

        current_vel_b_ = best_match.GetRot().inverse() * best_match.GetVel();
        current_vel_ = best_match.GetVel();

        // if (options_.with_height_) {
        //     current_lo_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        current_lo_pose_set_ = false;
        return false;
    }
}

bool LidarLoc::AssignDRPose(double timestamp) {
    UL lock(dr_pose_mutex_);
    SE3 interp_pose;
    NavState best_match;
    bool pose_interp_success = math::PoseInterp<NavState>(
        timestamp, dr_pose_queue_, [](const NavState& dr) { return dr.timestamp_; },
        [](const NavState& dr) { return dr.GetPose(); }, interp_pose, best_match, 5.0);

    if (pose_interp_success) {
        parking_ = best_match.is_parking_;
        current_dr_pose_ = interp_pose;
        current_dr_pose_set_ = true;

        // if (options_.with_height_) {
        //     current_dr_pose_.translation()[2] = map_height_;
        // }

        return true;
    } else {
        parking_ = false;
        current_dr_pose_set_ = false;
        return false;
    }
}

void LidarLoc::Finish(bool save_dyn) {
    if (map_) {
        update_map_quit_ = true;
        update_map_thread_.join();

        /// 永久保存时，再存储地图。
        /// save_dyn=false 用于运行时换图：旧地图的动态图层不该在“切换”时写盘，
        /// 那既耗时又会按当前 map_path_ 覆盖磁盘上的内容。
        if (save_dyn && options_.map_option_.policy_ == TiledMap::DynamicCloudPolicy::PERSISTENT &&
            options_.map_option_.save_dyn_when_quit_ && !has_set_pose_) {
            LLOG_INFO(logging::kLidarLoc, "saving maps");
            map_->SaveToBin(true);
            LLOG_INFO(logging::kLidarLoc, "dynamic maps saved");
        } else if (!save_dyn) {
            LLOG_INFO(logging::kLidarLoc, "skip saving dynamic maps (map switching)");
        }
    }
}

}  // namespace lightning::loc
