//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "common/log.h"
#include "common/path_utils.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "ui/pangolin_window.h"
#include "utils/pointcloud_utils.h"
#include "wrapper/ros_utils.h"

#include <pthread.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <opencv2/opencv.hpp>
#include <vector>

/// FLANN 1.9.1 的序列化缺少 std::unordered_map 支持：pcl/filters/radius_outlier_removal.h 的包含链会实例化
/// LshIndex 的 saveIndex/loadIndex 模板，在本机 GCC/C++17 下编译报
/// "std::unordered_map has no member named serialize"。此处补一个序列化重载（须位于 PCL 滤波头之前）。
/// 仅编译期需要，运行时不会走到 LSH 索引存取路径。
// #include <flann/util/serialization.h>
// #include <unordered_map>
// namespace flann {
// namespace serialization {
// template <class Archive, typename Key, typename Value>
// void serialize(Archive& ar, std::unordered_map<Key, Value>& map) {
//     std::size_t size = map.size();
//     ar & size;
//     for (auto& kv : map) {
//         Key key = kv.first;  // 拷贝一份，兼容 LoadArchive 的非常量引用
//         ar & key;
//         ar & kv.second;
//     }
// }
// }  // namespace serialization
// }  // namespace flann

namespace lightning {

namespace {

/// SE3 → TransformStamped（与 loc_system 一致）
geometry_msgs::msg::TransformStamped MakeTf(const builtin_interfaces::msg::Time& stamp, const std::string& parent,
                                            const std::string& child, const SE3& T) {
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

/// 解析建图保存根目录。
/// 优先级：ROS 参数(override) > yaml 的 system.map_root > $HOME/rcs/maps
/// 支持 "~/" 前缀展开为 $HOME；若 $HOME 不可用则退化到 ./data。
/// 地图最终保存到 <root>/<map_id>/。
std::string ResolveMapRoot(const std::string& yaml_root, const std::string& override_root) {
    const std::string raw = !override_root.empty() ? override_root : yaml_root;
    if (!raw.empty()) {
        return path_utils::ResolveDir(raw);
    }

    // 留空 → $HOME/rcs/maps；$HOME 也没有 → ./data
    const char* home_env = std::getenv("HOME");
    if (home_env == nullptr || home_env[0] == '\0') {
        return "./data";
    }
    return std::string(home_env) + "/rcs/maps";
}

/// 旋转矩阵 → SO(3)：配置常写有限小数，直接构造会因 R·Rᵀ≠I 触发 Sophus 断言，先做四元数正交化
SO3 NormalizeRotation(const Mat3d& R) {
    Eigen::Quaterniond q(R);
    q.normalize();
    return SO3(q.toRotationMatrix());
}

/// 平移(3)/旋转(9，行主序) 数组 → SE3（缺省单位阵）
SE3 ArraysToSE3(const std::vector<double>& t, const std::vector<double>& r) {
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

/// 秒（double）→ ROS 时间戳
builtin_interfaces::msg::Time ToRosTime(double t) {
    builtin_interfaces::msg::Time out;
    out.sec = static_cast<int32_t>(t);
    out.nanosec = static_cast<uint32_t>((t - static_cast<double>(out.sec)) * 1e9);
    return out;
}

/// 单调时钟当前时刻（秒），用于端到端延迟统计。
/// 时间戳（雷达/GPS 时间）可能跳变，不能用来算耗时差。
double NowSteadySec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path, const std::string& map_root_override) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LLOG_ERROR(logging::kSlam, "failed to init lio module");
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);

    /// 建图保存根目录：地图存到 <map_root_>/<map_id>/。
    /// yaml 里的旧注释 "map_path" 是定位的加载目录，与这里无关（保存/加载是两个概念）。
    const std::string yaml_map_root =
        yaml["system"]["map_root"] ? yaml["system"]["map_root"].as<std::string>() : std::string("");
    map_root_ = ResolveMapRoot(yaml_map_root, map_root_override);
    LLOG_INFO(logging::kSlam, "map save root = {} (source: {})", map_root_,
              !map_root_override.empty() ? "ros parameter" : (yaml_map_root.empty() ? "default" : "yaml"));
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    /// rviz/调试选项（缺省关闭，兼容旧配置）
    options_.threaded_lio_ = yaml["system"]["threaded_lio"] ? yaml["system"]["threaded_lio"].as<bool>() : true;
    options_.log_lidar_time_ = yaml["system"]["log_lidar_time"] ? yaml["system"]["log_lidar_time"].as<bool>() : false;
    /// LIO 工作线程优先级：x86 建议 0，RK3588 建议 10（详见 Options::worker_nice_ 注释）
    options_.worker_nice_ = yaml["system"]["lio_worker_nice"] ? yaml["system"]["lio_worker_nice"].as<int>() : 10;
    /// 全局地图发布间隔（秒），0 = 不发布（详见 Options::global_map_pub_interval_）
    options_.global_map_pub_interval_ =
        yaml["system"]["global_map_pub_interval"] ? yaml["system"]["global_map_pub_interval"].as<double>() : 1.0;

    /// 全局地图发布模式与分辨率（详见 Options::global_map_pub_mode_ 的说明）
    options_.global_map_pub_mode_ =
        yaml["system"]["global_map_pub_mode"] ? yaml["system"]["global_map_pub_mode"].as<std::string>() : "incremental";
    options_.global_map_pub_res_ =
        yaml["system"]["global_map_pub_res"] ? yaml["system"]["global_map_pub_res"].as<float>() : 0.1f;
    options_.global_map_pub_max_kf_ =
        yaml["system"]["global_map_pub_max_kf"] ? yaml["system"]["global_map_pub_max_kf"].as<int>() : 0;
    LLOG_INFO(logging::kSlam, "global map pub: mode {} | interval {:.1f}s | res {:.2f} | max kf/publish {}",
              options_.global_map_pub_mode_, options_.global_map_pub_interval_, options_.global_map_pub_res_,
              options_.global_map_pub_max_kf_);
    /// 轨迹发布间隔（秒），0 = 不发布（详见 Options::path_pub_interval_）
    options_.path_pub_interval_ =
        yaml["system"]["path_pub_interval"] ? yaml["system"]["path_pub_interval"].as<double>() : 1.0;
    options_.pub_tf_ = yaml["system"]["pub_tf"] ? yaml["system"]["pub_tf"].as<bool>() : false;
    options_.pub_odom_ = yaml["system"]["pub_odom"] ? yaml["system"]["pub_odom"].as<bool>() : false;
    options_.log_pose_opt_ = yaml["system"]["log_pose_opt"] ? yaml["system"]["log_pose_opt"].as<bool>() : false;
    options_.enable_lidar_rviz_ =
        yaml["system"]["enable_lidar_loc_rviz"] ? yaml["system"]["enable_lidar_loc_rviz"].as<bool>() : false;
    options_.enable_path_rviz_ =
        yaml["system"]["enable_path_rviz"] ? yaml["system"]["enable_path_rviz"].as<bool>() : false;
    if (options_.enable_lidar_rviz_ && !options_.enable_path_rviz_) {
        options_.enable_path_rviz_ = true;  // 发布路径时会包含位姿信息，方便调试
    }

    /// pcd2pgm：2D栅格生成参数（缺省用默认值，与独立pcd2pgm包的config一致）
    options_.with_pcd2pgm_ = yaml["system"]["with_pcd2pgm"] ? yaml["system"]["with_pcd2pgm"].as<bool>() : true;
    if (yaml["pcd2pgm"]) {
        auto p2p = yaml["pcd2pgm"];
        if (p2p["thre_z_min"]) options_.pcd2pgm_.thre_z_min_ = p2p["thre_z_min"].as<double>();
        if (p2p["thre_z_max"]) options_.pcd2pgm_.thre_z_max_ = p2p["thre_z_max"].as<double>();
        if (p2p["thre_radius"]) options_.pcd2pgm_.thre_radius_ = p2p["thre_radius"].as<double>();
        if (p2p["thres_point_count"]) options_.pcd2pgm_.thres_point_count_ = p2p["thres_point_count"].as<int>();
        if (p2p["map_resolution"]) options_.pcd2pgm_.map_resolution_ = p2p["map_resolution"].as<double>();
    }
    LLOG_INFO(logging::kSlam, "pcd2pgm: enabled {}, z band [{}, {}], radius {}, count {}, res {}",
              options_.with_pcd2pgm_, options_.pcd2pgm_.thre_z_min_, options_.pcd2pgm_.thre_z_max_,
              options_.pcd2pgm_.thre_radius_, options_.pcd2pgm_.thres_point_count_, options_.pcd2pgm_.map_resolution_);

    if (options_.with_loop_closing_) {
        LLOG_INFO(logging::kSlam, "slam with loop closing");
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);

        /// 回环优化会改写**历史**关键帧的位姿 -> 已经发到 rviz 的点位置全部失效。
        /// 这里只置一个标记，由工作线程在下一次发布时决定怎么处理（见 PublishGlobalMap）；
        /// 回调跑在回环线程上，不能在这里做重活。
        /// 顺带（若开了 2.5D 栅格）触发一次重绘。
        lc_->SetLoopClosedCB([this]() {
            map_pose_corrected_ = true;
            if (g2p5_) {
                g2p5_->RedrawGlobalMap();
            }
        });
    }

    if (options_.with_visualization_) {
        LLOG_INFO(logging::kSlam, "slam with 3D UI");
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        /// 注：回环回调统一在上面的 with_loop_closing_ 分支里注册 ——
        /// 它除了重绘栅格图，还要置 map_pose_corrected_（增量地图发布要用），
        /// 所以不能只在开了栅格图时才注册。

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LLOG_INFO(logging::kSlam, "online mode, creating ros2 node ... ");

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        /// QoS 说明（重要）：
        ///  - 点云驱动 (rslidar_sdk) 的 publisher 是 RELIABLE + KEEP_LAST(100)，单帧约 1.5MB；
        ///    livox_ros_driver2 同样是 RELIABLE。
        ///  - 【为什么用可靠订阅】大点云走 UDP 分片时链路层丢包可观：实测 best_effort 订阅
        ///    在 40s 回放中只收到 89/400 帧（丢约 78%），reliable 靠重传可全部补齐。
        ///  - 【为什么可靠订阅现在不会拖慢驱动】从 M1 起点云回调只做“预处理 + 入队”
        ///    （实测 ~1.6ms），消息会被立刻取走，reader 队列不再积压，
        ///    writer 也就不必长时间保留样本等 ack —— 反压链路已被解耦。
        ///    真正的计算在 RunWorker() 工作线程里，它的积压发生在应用层的 lidar_buffer_，
        ///    不会影响 DDS 的 reader 队列。
        ///  - 因此这里必须与驱动保持一致使用 RELIABLE；若回调再次变慢，需重新评估。
        auto lidar_qos = rclcpp::QoS(10);  // reliable + keep_last(10)，与驱动 publisher 一致
        /// IMU 数据量极小，保持 RELIABLE 以保证 ESKF 积分精度；depth 放大到 1s 量级避免被短暂卡顿丢弃
        auto imu_qos = rclcpp::QoS(200);

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, lidar_qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                const double cb_t0 = NowSteadySec();
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
                last_lidar_cb_ms_ = (NowSteadySec() - cb_t0) * 1e3;
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, lidar_qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                const double cb_t0 = NowSteadySec();
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
                last_lidar_cb_ms_ = (NowSteadySec() - cb_t0) * 1e3;
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        if (options_.enable_lidar_rviz_) {
            std::string scan_topic = yaml["system"]["rviz_current_scan_topic"]
                                         ? yaml["system"]["rviz_current_scan_topic"].as<std::string>()
                                         : "lightning/current_scan";
            std::string map_topic = yaml["system"]["rviz_global_map_topic"]
                                        ? yaml["system"]["rviz_global_map_topic"].as<std::string>()
                                        : "lightning/global_map";
            cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic, 10);
            /// 地图话题用更深的队列：rviz 渲染几百万点时会明显变慢，
            /// depth 1 的可靠发布容易与它的 ack 互相钳制，给点缓冲。
            map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, rclcpp::QoS(10));
        }

        if (options_.enable_path_rviz_) {
            path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("lightning/path", 10);
        }

        if (options_.pub_odom_) {
            odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("lightning/odom", 10);
            nav_state_pub_ = node_->create_publisher<msg::NavState>("lightning/nav_state", 10);
        }

        /// 静态外参（base_link→imu_link / imu_link→lidar_link）：始终解析（位姿归算用），仅 pub_tf_ 时广播
        std::vector<double> t_base_imu{0, 0, 0}, R_base_imu{1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::vector<double> t_imu_lidar{0, 0, 0}, R_imu_lidar{1, 0, 0, 0, 1, 0, 0, 0, 1};
        if (yaml["system"]["extrinsic_base_imu_T"]) {
            t_base_imu = yaml["system"]["extrinsic_base_imu_T"].as<std::vector<double>>();
        }
        if (yaml["system"]["extrinsic_base_imu_R"]) {
            R_base_imu = yaml["system"]["extrinsic_base_imu_R"].as<std::vector<double>>();
        }
        if (yaml["fasterlio"]["extrinsic_T"]) {
            t_imu_lidar = yaml["fasterlio"]["extrinsic_T"].as<std::vector<double>>();
        }
        if (yaml["fasterlio"]["extrinsic_R"]) {
            R_imu_lidar = yaml["fasterlio"]["extrinsic_R"].as<std::vector<double>>();
        }
        T_base_imu_ = ArraysToSE3(t_base_imu, R_base_imu);
        T_imu_lidar_ = ArraysToSE3(t_imu_lidar, R_imu_lidar);

        if (options_.pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

            /// 静态 TF：base_link→imu_link→lidar_link（发布一次，与定位一致）
            std::vector<geometry_msgs::msg::TransformStamped> statics;
            statics.push_back(MakeTf(node_->now(), "base_link", "imu_link", T_base_imu_));
            statics.push_back(MakeTf(node_->now(), "imu_link", "lidar_link", T_imu_lidar_));
            tf_static_broadcaster_->sendTransform(statics);
            LLOG_INFO(logging::kSlam, "static tf: base_link->imu_link t={}, imu_link->lidar_link t={}",
                      T_base_imu_.translation().transpose(), T_imu_lidar_.translation().transpose());
        }

        savepath_service_ = node_->create_service<srv::SavePath>(
            "lightning/save_path", [this](const srv::SavePath::Request::SharedPtr& req,
                                          srv::SavePath::Response::SharedPtr res) { SavePath(req, res); });

        LLOG_INFO(logging::kSlam, "online slam node has been created.");

        /// 启动 LIO 工作线程：自此点云回调只负责入队，Run() 与发布都在工作线程上。
        /// 线程内部以 running_ 为开关，StartSLAM() 之后才真正开工。
        if (options_.threaded_lio_) {
            worker_ = std::thread(&SlamSystem::RunWorker, this);
            LLOG_INFO(logging::kSlam, "lio worker thread started");
        } else {
            LLOG_WARN(logging::kSlam,
                      "lio worker thread DISABLED: LIO runs inside the ROS callback (legacy mode, may block "
                      "IMU callbacks and back-pressure the lidar driver)");
        }
    }

    return true;
}

SlamSystem::~SlamSystem() {
    /// 必须先停工作线程：否则 rclcpp::shutdown() 之后它仍可能调用 publish（会抛异常）
    StopWorker();

    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    if (!map_name.empty()) {
        map_name_ = map_name;
    }
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    if (request->map_id.empty()) {
        LLOG_ERROR(logging::kSlam, "save_map: map_id is empty, refuse to save into the map root itself");
        response->response = 3;  // 3 = 无法保存
        return;
    }

    map_name_ = request->map_id;

    /// 保存到 <map_root_>/<map_id>/（map_root_ 缺省为 $HOME/rcs/maps）
    const std::string save_path = map_root_ + "/" + map_name_;
    LLOG_INFO(logging::kSlam, "save_map: id={} -> {}", map_name_, save_path);

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = map_root_ + "/" + map_name_;
    }

    LLOG_INFO(logging::kSlam, "slam map saving to {}", save_path);

    /// 【诊断用】本函数**整个跑在 ROS executor 线程上**（srv/save_map 的回调）。
    /// 单线程 executor 下，这意味着期间 lidar 回调会被完全阻塞：
    /// 消息堆在 DDS → 回调恢复后 arrival_dt 突刺 → 超 kRecvBacklogWarnMs 就报
    /// 「recv falling behind」。所以这条日志的时间戳可以直接和那条警告对齐。
    /// 同时也会把 worker 的 lidar_buffer_ 攒起来（>DropStaleScans 的 keep 会丢帧）。
    const auto save_t0 = std::chrono::steady_clock::now();
    LLOG_INFO(logging::kSlam, "save_map >>> 开始（executor 线程，期间 lidar 回调阻塞）");

    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    /// 全量建图的**唯一**调用点（发布路径走增量，见 PublishGlobalMap）。
    /// 这是存图流程的大头：实测 1687 关键帧约 0.9s，
    /// 内部拆为 Map:kf-voxel / Map:trans+cat / Map:final-voxel，这里再记一个总数。
    CloudPtr global_map;
    Timer::Evaluate([&]() { global_map = lio_->GetGlobalMap(!options_.with_loop_closing_); }, "SaveMap:GetMap");

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    Timer::Evaluate([&]() { tm.ConvertFromFullPCD(global_map, start_pose, save_path); }, "SaveMap:Tiled");

    Timer::Evaluate([&]() { pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map); },
                    "SaveMap:PCD");
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    /// 由全局地图（世界系）直接生成2D栅格地图 map.pgm + map.yaml
    if (options_.with_pcd2pgm_) {
        Timer::Evaluate([&]() { ConvertGlobalMapToGridMap(global_map, save_path); }, "SaveMap:GridMap");
    }

    /// 保存关键帧位姿（回环优化后的位姿，与全局地图一致）与关键帧点云（雷达系去畸变点云）
    {
        auto keyframes = lio_->GetAllKeyframes();

        /// 1) 关键帧位姿文件: timestamp px py pz qx qy qz qw
        std::ofstream pose_file(save_path + "/keyframe_poses.txt");
        if (pose_file.is_open()) {
            pose_file << "timestamp px py pz qx qy qz qw\n";
            for (auto& kf : keyframes) {
                const SE3 pose = kf->GetOptPose();
                const Quatd q = pose.unit_quaternion();
                const Vec3d t = pose.translation();
                pose_file << std::fixed << std::setprecision(9) << kf->GetState().timestamp_ << " " << t.x() << " "
                          << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                          << "\n";
            }
            pose_file.close();
            LLOG_INFO(logging::kSlam, "keyframe poses saved: {}", keyframes.size());
        } else {
            LLOG_ERROR(logging::kSlam, "failed to write keyframe_poses.txt");
        }

        /// 2) 关键帧点云: keyframes/kf_<id>.pcd
        std::string kf_dir = save_path + "/keyframes";
        std::filesystem::create_directories(kf_dir);
        /// 上千个小文件逐个压缩写，I/O 量不小；单独计时
        Timer::Evaluate(
            [&]() {
                char kf_name[64];
                for (auto& kf : keyframes) {
                    snprintf(kf_name, sizeof(kf_name), "kf_%06lu.pcd", kf->GetID());
                    pcl::io::savePCDFileBinaryCompressed(kf_dir + "/" + kf_name, *kf->GetCloud());
                }
            },
            "SaveMap:KfPCD");
        LLOG_INFO(logging::kSlam, "keyframe clouds saved to {}, num: {}", kf_dir, keyframes.size());
    }

    /// 2D栅格的pgm/yaml由pcd2pgm路径产出时，跳过g2p5的写盘（避免同目录双写冲突）
    if (options_.with_gridmap_ && !options_.with_pcd2pgm_) {
        /// 存为ROS兼容的模式
        auto map = g2p5_->GetNewestMap()->ToROS();
        const int width = map.info.width;
        const int height = map.info.height;

        cv::Mat nav_image(height, width, CV_8UC1);
        for (int y = 0; y < height; ++y) {
            const int rowStartIndex = y * width;
            for (int x = 0; x < width; ++x) {
                const int index = rowStartIndex + x;
                int8_t data = map.data[index];
                if (data == 0) {                                   // Free
                    nav_image.at<uchar>(height - 1 - y, x) = 255;  // White
                } else if (data == 100) {                          // Occupied
                    nav_image.at<uchar>(height - 1 - y, x) = 0;    // Black
                } else {                                           // Unknown
                    nav_image.at<uchar>(height - 1 - y, x) = 128;  // Gray
                }
            }
        }

        cv::imwrite(save_path + "/map.pgm", nav_image);

        /// yaml
        std::ofstream yamlFile(save_path + "/map.yaml");
        if (!yamlFile.is_open()) {
            LLOG_ERROR(logging::kSlam, "failed to write map.yaml");
            return;  // 文件打开失败
        }

        try {
            YAML::Emitter emitter;
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
            emitter << YAML::Key << "mode" << YAML::Value << "trinary";
            emitter << YAML::Key << "width" << YAML::Value << map.info.width;
            emitter << YAML::Key << "height" << YAML::Value << map.info.height;
            emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);
            std::vector<double> orig{map.info.origin.position.x, map.info.origin.position.y, 0};
            emitter << YAML::Key << "origin" << YAML::Value << orig;
            emitter << YAML::Key << "negate" << YAML::Value << 0;
            emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
            emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;

            emitter << YAML::EndMap;

            yamlFile << emitter.c_str();
            yamlFile.close();
        } catch (...) {
            yamlFile.close();
            return;
        }
    }

    const double save_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - save_t0).count();
    LLOG_INFO(logging::kSlam, "save_map <<< 完成，耗时 {:.1f} s（这段时间 lidar 回调是阻塞的）", save_sec);
    LLOG_INFO(logging::kSlam, "map saved");
}

/// 由全局地图（世界系）生成2D栅格地图 map.pgm + map.yaml
/// 复用独立pcd2pgm包的算法：z高度带过滤（世界系z，与雷达安装倾斜无关）+ 半径去噪 + XY投影，不做膨胀
void SlamSystem::ConvertGlobalMapToGridMap(const CloudPtr& global_map, const std::string& save_path) {
    const auto& params = options_.pcd2pgm_;

    /// 1) 世界系z高度带过滤
    CloudPtr cloud_z(new PointCloudType);
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(global_map);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(params.thre_z_min_, params.thre_z_max_);
    pass.filter(*cloud_z);

    /// 2) 半径离群去噪
    CloudPtr cloud_f(new PointCloudType);
    pcl::RadiusOutlierRemoval<PointType> ror;
    ror.setInputCloud(cloud_z);
    ror.setRadiusSearch(params.thre_radius_);
    ror.setMinNeighborsInRadius(params.thres_point_count_);
    ror.filter(*cloud_f);

    LLOG_INFO(logging::kSlam, "pcd2pgm: z-filtered {} / {}, denoised {}", cloud_z->size(), global_map->size(),
              cloud_f->size());

    if (cloud_f->empty()) {
        LLOG_ERROR(logging::kSlam, "pcd2pgm: filtered cloud is empty, skip 2d grid map");
        return;
    }

    /// 3) XY投影为占据栅格
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    for (const auto& pt : cloud_f->points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        y_max = std::max(y_max, static_cast<double>(pt.y));
    }

    const double res = params.map_resolution_;
    const int width = static_cast<int>(std::ceil((x_max - x_min) / res));
    const int height = static_cast<int>(std::ceil((y_max - y_min) / res));

    /// pgm: 0=占据(黑) 205=未知(灰)；不做膨胀、不涂空闲
    /// 行序上下翻转（第0行对应y_max），符合ROS map_server的origin左下角约定
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(205));
    for (const auto& pt : cloud_f->points) {
        const int i = static_cast<int>(std::floor((pt.x - x_min) / res));
        const int j = static_cast<int>(std::floor((pt.y - y_min) / res));
        if (i >= 0 && i < width && j >= 0 && j < height) {
            img.at<uint8_t>(height - 1 - j, i) = 0;
        }
    }
    cv::imwrite(save_path + "/map.pgm", img);

    /// yaml（nav2 map_server 格式）
    std::ofstream yamlFile(save_path + "/map.yaml");
    if (!yamlFile.is_open()) {
        LLOG_ERROR(logging::kSlam, "failed to write map.yaml");
        return;
    }
    try {
        YAML::Emitter emitter;
        emitter << YAML::BeginMap;
        emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
        emitter << YAML::Key << "mode" << YAML::Value << "trinary";
        emitter << YAML::Key << "resolution" << YAML::Value << res;
        emitter << YAML::Key << "origin" << YAML::Value << YAML::Flow << std::vector<double>{x_min, y_min, 0.0};
        emitter << YAML::Key << "negate" << YAML::Value << 0;
        emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
        emitter << YAML::Key << "free_thresh" << YAML::Value << 0.196;
        emitter << YAML::EndMap;
        yamlFile << emitter.c_str() << "\n";
        yamlFile.close();
    } catch (...) {
        yamlFile.close();
        LLOG_ERROR(logging::kSlam, "failed to emit map.yaml");
        return;
    }

    LLOG_INFO(logging::kSlam, "pcd2pgm: grid map saved to {} ({}x{}, res {})", save_path, width, height, res);
}

// ros服务回调，保存轨迹
// 如果请求里没有指定路径，则默认保存在./data/下，文件名为path_年月日时分秒.txt
// ros2 service call /lightning/save_path lightning/srv/SavePath
void SlamSystem::SavePath(const srv::SavePath::Request::SharedPtr request,
                          srv::SavePath::Response::SharedPtr response) {
    std::string save_path = request->file_path;
    bool success = SavePath(save_path);
    response->success = success;
    if (success) {
        response->message = "Path saved successfully. Total poses: " + std::to_string(path_.poses.size());
    } else {
        response->message = "Failed to save path.";
    }
}

bool SlamSystem::SavePath(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        char time_str[64];
        time_t now = time(nullptr);
        strftime(time_str, sizeof(time_str), "%Y%m%d_%H%M%S", localtime(&now));
        save_path = "./data/path_" + std::string(time_str) + ".txt";
    }

    std::ofstream file(save_path);
    if (!file.is_open()) {
        LLOG_ERROR(logging::kSlam, "Failed to open file: {}", save_path);
        return false;
    }
    file << "timestamp px py pz qx qy qz qw\n";
    for (const auto& pose : path_.poses) {
        file << std::fixed << std::setprecision(5) << pose.header.stamp.sec << "." << std::setfill('0') << std::setw(9)
             << pose.header.stamp.nanosec << " " << pose.pose.position.x << " " << pose.pose.position.y << " "
             << pose.pose.position.z << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " "
             << pose.pose.orientation.z << " " << pose.pose.orientation.w << "\n";
    }

    file.close();
    LLOG_INFO(logging::kSlam, "Path saved to {}. Total poses: {}", save_path, path_.poses.size());
    return true;
}

/// 统一 TF/话题发布（建图）：map→odom→base_link→imu_link→lidar_link
///  - 与定位 loc_system 同构（路线B）：odom = LO(ESKF) 系，map = 回环后全局；建图时 map≈odom，map→odom 仅含回环修正
///  - lightning 状态/位姿为 IMU(=lidar) 系；base_link 用 system.extrinsic_base_imu 归算
void SlamSystem::PublishMappingTf(const builtin_interfaces::msg::Time& stamp) {
    const auto state = lio_->GetState();  // ESKF(LO) 状态（odom 系，IMU 帧）
    const SE3 T_imu_base = T_base_imu_.inverse();
    const SE3 T_odom_imu = state.GetPose();    // odom→imu（LO 实时）
    const SE3 T_map_imu = lio_->GetOptPose();  // map→imu（回环后全局）

    const SE3 T_odom_base = T_odom_imu * T_imu_base;            // odom→base_link
    const SE3 T_map_base = T_map_imu * T_imu_base;              // map→base_link
    const SE3 T_map_odom = T_map_base * T_odom_base.inverse();  // map→odom（建图≈I）

    /// 动态 TF：map→odom + odom→base_link（每帧点云）
    if (tf_broadcaster_ != nullptr) {
        std::vector<geometry_msgs::msg::TransformStamped> tfs;
        tfs.push_back(MakeTf(stamp, "map", "odom", T_map_odom));
        tfs.push_back(MakeTf(stamp, "odom", "base_link", T_odom_base));
        tf_broadcaster_->sendTransform(tfs);
    }

    if (nav_state_pub_ != nullptr) {
        /// nav_state：全局 map→base_link（回环后位姿归算）
        msg::NavState ns;
        ns.header.stamp = stamp;
        ns.header.frame_id = "map";
        const Vec3d t_map_base = T_map_base.translation();
        const auto q_map_base = T_map_base.unit_quaternion();
        ns.pose.position.x = t_map_base.x();
        ns.pose.position.y = t_map_base.y();
        ns.pose.position.z = t_map_base.z();
        ns.pose.orientation.x = q_map_base.x();
        ns.pose.orientation.y = q_map_base.y();
        ns.pose.orientation.z = q_map_base.z();
        ns.pose.orientation.w = q_map_base.w();
        ns.velocity.x = state.vel_.x();
        ns.velocity.y = state.vel_.y();
        ns.velocity.z = state.vel_.z();
        ns.confidence = 1.0;
        ns.pose_is_ok = true;
        nav_state_pub_->publish(ns);

        if (odom_pub_ != nullptr) {
            /// odom：odom→base_link（LO 归算），与 TF odom→base_link 一致
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            const Vec3d t_odom_base = T_odom_base.translation();
            const auto q_odom_base = T_odom_base.unit_quaternion();
            odom.pose.pose.position.x = t_odom_base.x();
            odom.pose.pose.position.y = t_odom_base.y();
            odom.pose.pose.position.z = t_odom_base.z();
            odom.pose.pose.orientation.x = q_odom_base.x();
            odom.pose.pose.orientation.y = q_odom_base.y();
            odom.pose.pose.orientation.z = q_odom_base.z();
            odom.pose.pose.orientation.w = q_odom_base.w();
            odom.twist.twist.linear.x = state.vel_.x();
            odom.twist.twist.linear.y = state.vel_.y();
            odom.twist.twist.linear.z = state.vel_.z();
            odom_pub_->publish(odom);
        }
    }

    /// path：map 下 base_link 轨迹（发布与退出保存统一）
    if (options_.enable_path_rviz_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = stamp;
        ps.header.frame_id = "map";
        const Vec3d t_map_base = T_map_base.translation();
        const auto q_map_base = T_map_base.unit_quaternion();
        ps.pose.position.x = t_map_base.x();
        ps.pose.position.y = t_map_base.y();
        ps.pose.position.z = t_map_base.z();
        ps.pose.orientation.x = q_map_base.x();
        ps.pose.orientation.y = q_map_base.y();
        ps.pose.orientation.z = q_map_base.z();
        ps.pose.orientation.w = q_map_base.w();
        path_.header = ps.header;
        path_.poses.push_back(ps);  // 仍然每帧累积：save_path 导出轨迹要用它

        /// 发布另行判定，与 global_map 同一类问题：
        /// path_ 随运行时间无界增长，而 publish 要把整个 vector 序列化一遍，
        /// 是 O(n)/帧、O(n²) 累计。实测 3190 帧时 PublishMappingTf 已从
        /// 0.311ms 涨到 0.989ms（代码没改，纯粹是 path_ 变长）。
        /// 因此：① 没人订阅就不发；② 按时间节流（轨迹不需要 10Hz 刷新）。
        const bool path_wanted = path_pub_ != nullptr && path_pub_->get_subscription_count() > 0;
        const double path_now = ToSec(stamp);
        const bool path_interval_ok =
            options_.path_pub_interval_ > 0 &&
            (last_path_pub_time_ <= 0 || path_now - last_path_pub_time_ >= options_.path_pub_interval_);
        if (path_wanted && path_interval_ok) {
            last_path_pub_time_ = path_now;
            path_pub_->publish(path_);
        }
    }
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    /// 回调线程只做预处理 + 入队（毫秒级以内）。
    /// 绝不能在回调里跑 LIO 与发布：那会把 executor 堵住，连带拖慢 IMU 回调
    /// 以及 save_map / save_path 服务。真正的计算在 RunWorker() 里。
    const size_t pts = static_cast<size_t>(cloud->width) * cloud->height;
    lio_->ProcessPointCloud2(cloud);

    /// 记录本帧点数，供工作线程输出逐帧耗时
    if (options_.log_lidar_time_) {
        last_scan_points_.store(pts);
        MonitorRecvHealth(ToSec(cloud->header.stamp));
    }

    /// 兼容旧行为（system.threaded_lio=false）：在回调里同步跑完
    if (!options_.threaded_lio_) {
        RunOnceAndPublish();
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    /// 同 PointCloud2 版本：回调只入队，计算与发布在 RunWorker() 里
    const size_t pts = cloud->point_num;
    lio_->ProcessPointCloud2(cloud);

    if (options_.log_lidar_time_) {
        last_scan_points_.store(pts);
        MonitorRecvHealth(ToSec(cloud->header.stamp));
    }

    /// 兼容旧行为（system.threaded_lio=false）：在回调里同步跑完
    if (!options_.threaded_lio_) {
        RunOnceAndPublish();
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);

        /// spin 返回意味着已收到 Ctrl-C / rclcpp shutdown，
        /// 立刻停掉工作线程，避免 ROS 上下文失效后继续 publish
        StopWorker();
    }
}

/// ==================== M1: LIO 工作线程 ====================

namespace {
/// 工作线程积压阈值：待处理点云帧超过该值就丢最旧的帧。
/// 10Hz 下相当于允许 1 秒的瞬时抖动（例如回环优化、voxel 重建卡顿）。
constexpr size_t kMaxPendingScans = 10;
/// 接收侧积压告警阈值（毫秒）：到达间隔比时间戳间隔超出这么多就告警。
///
/// 为什么从 30 放宽到 80：实测 rosbag2 播放器自身就有 **±30% 的调度抖动**
/// （`--rate 0.5` 时标称 200ms，实测 min 142ms / max 253ms），折算到 10Hz 就是 ±30ms。
/// 30ms 的阈值会被这种固有抖动反复误触发，把真正的问题（数百毫秒的突刺）淹掉。
/// 真正的突刺是 300~800ms 量级，80ms 仍然能干净地抓住；
/// 而且离丢帧阈值（kMaxPendingScans = 10 帧）还留了足够余量。
constexpr double kRecvBacklogWarnMs = 80.0;
/// 驱动出帧间隔异常阈值（毫秒）：10Hz 下正常为 100ms，超过 150ms 视为丢帧/严重卡顿。
/// 仅用于统计提示，不影响主流程。
constexpr double kDriverGapWarnMs = 150.0;
/// 空闲时的轮询间隔（毫秒）
constexpr int kIdlePollMs = 1;
}  // namespace

void SlamSystem::MonitorRecvHealth(double stamp_sec) {
    const double arrival = NowSteadySec();

    if (last_lidar_stamp_ > 0 && stamp_sec > last_lidar_stamp_) {
        const double stamp_dt_ms = (stamp_sec - last_lidar_stamp_) * 1e3;
        const double arrival_dt_ms = (arrival - last_lidar_arrival_) * 1e3;
        const double backlog_ms = arrival_dt_ms - stamp_dt_ms;

        last_scan_dt_ms_.store(stamp_dt_ms);

        /// 累计统计，退出时汇总（stamp_dt 明显偏大 = 驱动/雷达侧丢帧）
        frame_count_++;
        if (stamp_dt_ms > kDriverGapWarnMs) {
            driver_gap_warn_count_++;
        }

        /// stamp_dt 反映驱动实际出帧节奏（会在 [lidar] 行的 dt 列输出）：
        ///   dt 偶发 ~200ms -> 驱动/雷达自己丢帧（CPU 饥饿、驱动接收线程被抢、网卡丢包等）
        ///   arrival_dt 持续大于 stamp_dt -> 接收侧（DDS reader 队列 / executor 调度）在积压，
        ///     这正是会反过来拖慢驱动 publish 的那条链路
        /// 注意：驱动丢整帧时 stamp_dt 与 arrival_dt 会同步变大，backlog 仍约为 0，
        ///       所以后者无告警并不代表驱动正常，必须结合 dt 列一起看。
        if (backlog_ms > kRecvBacklogWarnMs) {
            /// 顺带报出「上次回调自身耗时」和「worker 当前积压帧数」，让这条警告自证原因：
            ///   上次回调长      -> executor 被我们自己的回调占住了
            ///   worker 积压 > 0 -> worker 落后（rviz 补发整图、回环优化…）
            ///   两者都正常      -> 阻塞在回调之外：DDS 投递 / 调度 / 别的进程抢 CPU
            /// 注：GetPendingScanCount() 只拿 mtx_buffer_，而此刻本回调尚未进入
            /// ProcessPointCloud2（那里才拿该锁），所以不会自锁。
            const size_t pending = lio_ ? lio_->GetPendingScanCount() : 0;
            LLOG_WARN(logging::kSlam,
                      "[lidar] recv falling behind: arrival_dt {:.1f} ms vs stamp_dt {:.1f} ms (backlog +{:.1f} ms) "
                      "| 上次回调 {:.1f} ms | worker 积压 {} 帧",
                      arrival_dt_ms, stamp_dt_ms, backlog_ms, last_lidar_cb_ms_, pending);
        }
    }

    last_lidar_stamp_ = stamp_sec;
    last_lidar_arrival_ = arrival;
}

void SlamSystem::RunWorker() {
    /// 给本线程命名：否则 top -H / ps -L 里所有线程都显示为 run_slam_online，
    /// 无法区分“哪个是 LIO 工作线程、哪个是 executor 主线程”。
    (void)pthread_setname_np(pthread_self(), "lio_worker");

    /// 主动降低本线程的调度优先级（nice 在 Linux 上是 per-thread 的）。
    ///
    /// 当初加它的动机：算力紧张的平台（如 RK3588 上单帧 LIO 需 ~70ms，占满一个核），
    /// 若不降优先级，executor 线程上的 IMU / 点云回调会被饿住，
    /// reader 队列随之积压，最终反过来阻塞雷达驱动的 publish。
    ///
    /// 但 nice 只在“有竞争”时起作用：
    ///  - RK3588：LIO 占单核 30~70%，降优先级是必要的 -> lio_worker_nice: 10
    ///  - x86   ：LIO 仅占 ~4%（~4ms / 100ms），降优先级反而让本线程在
    ///            bag/rviz/回环优化抢 CPU 时第一个被牺牲 -> lio_worker_nice: 0
    if (options_.worker_nice_ != 0) {
        (void)nice(options_.worker_nice_);
    }

    LLOG_INFO(logging::kSlam, "lio worker: thread started");

    while (!stop_worker_) {
        if (!running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdlePollMs));
            continue;
        }

        /// 积压保护：工作线程跟不上时丢掉最旧的帧，避免内存无限增长。
        /// 正常情况（10Hz 点云、单帧处理 << 100ms）永远不会触发。
        lio_->DropStaleScans(kMaxPendingScans);

        /// 无待处理数据时直接休眠，避免反复进出 Run() 刷出无意义的
        /// “sync package failed” 告警（该告警只在真正有帧但同步不上时才有价值）
        if (!lio_->HasPendingScan()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdlePollMs));
            continue;
        }

        /// Run() 在“同步未满足 / 本帧跳过”时返回 false，此时让出 CPU 后重试。
        /// 用 while 循环而非“一帧回调一次”，还能把积压的帧尽快追平。
        if (!RunOnceAndPublish()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kIdlePollMs));
        }
    }

    LLOG_INFO(logging::kSlam, "lio worker: thread stopped");
}

bool SlamSystem::RunOnceAndPublish() {
    const auto t0 = std::chrono::steady_clock::now();

    /// Timer::Evaluate 返回 void，故用捕获变量取 Run() 的返回值；
    /// 它同时把单帧 LIO 耗时记入 “LIO Run”，退出时由 Timer::PrintAll 汇总。
    bool ran = false;
    Timer::Evaluate([&, this]() { ran = lio_->Run(); }, "LIO Run");

    if (!ran) {
        return false;
    }

    const double lio_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    ProcessOnce(lio_ms);
    return true;
}

void SlamSystem::ProcessOnce(double lio_ms) {
    /// 用 LIO 状态的时间戳作为发布时戳：它对应本次实际处理的那一帧，
    /// 而不是“最新收到的帧”（两者在积压时并不相同）
    const auto state = lio_->GetState();
    const builtin_interfaces::msg::Time stamp = ToRosTime(state.timestamp_);

    /// 逐帧耗时（system.log_lidar_time=true 时输出）：
    ///   pts   = 本帧点云点数
    ///   dt    = 驱动出帧间隔（header.stamp 差值），正常应稳定在 100ms 附近
    ///   lio   = EKF + 点面配准 + 增量建图
    ///   e2e   = 从“本帧入队”到“本帧处理完”（含排队等待），queue = e2e - lio
    /// 健康的标志：dt 稳定 ~100ms、lio 稳定在几毫秒、queue 接近 0
    ///
    /// 注意：e2e 必须用“本帧自己的”到达时刻（由 LaserMapping 随帧保存）。
    /// 早期版本用的是全局“最新到达时刻”，worker 追赶积压时那个值不再刷新，
    /// 算出的 e2e/queue 会随处理进度线性虚增，看着像延迟在涨，实际是测量伪影。
    if (options_.log_lidar_time_) {
        const double enqueued = lio_->GetLastProcessedArrivalTime();
        const double e2e_ms = enqueued > 0 ? (NowSteadySec() - enqueued) * 1e3 : 0.0;
        const double queue_ms = e2e_ms > lio_ms ? e2e_ms - lio_ms : 0.0;
        LLOG_INFO(
            logging::kSlam,
            "[lidar] pts {:6d} | dt {:6.1f} ms | lio {:6.2f} ms | e2e {:6.2f} ms | queue {:5.2f} ms | pending {:2d}",
            static_cast<int>(last_scan_points_.load()), last_scan_dt_ms_.load(), lio_ms, e2e_ms, queue_ms,
            static_cast<int>(lio_->GetPendingScanCount()));
    }

    if (options_.log_pose_opt_) {
        auto q = state.rot_.unit_quaternion();
        LLOG_INFO(logging::kSlam,
                  "lio pose: [{:.4f}, {:.4f}, {:.4f}]\tq: [{:.4f}, {:.4f}, {:.4f}, {:.4f}]"
                  "\tvelocity: [{:.3f}, {:.3f}, {:.3f}]m/s",
                  state.pos_.x(), state.pos_.y(), state.pos_.z(), q.x(), q.y(), q.z(), q.w(), state.vel_.x(),
                  state.vel_.y(), state.vel_.z());
    }

    /// 统一 TF/话题发布：map→odom→base_link→imu_link→lidar_link（建图时 map≈odom），与定位一致
    ///
    /// 注意：以下各段都在工作线程上串行执行，且 **不包含在 lio_ms 里**。
    /// 出问题时它们会直接掏空 worker 的吞吐量（出现 lidar_buffer_ 积压 / 丢帧），
    /// 所以必须单独计时：退出时 Timer::PrintAll() 能看到各自贡献。
    Timer::Evaluate([&]() { PublishMappingTf(stamp); }, "PublishMappingTf");

    if (options_.enable_lidar_rviz_ && cloud_pub_ != nullptr) {
        auto scan_world = lio_->GetScanDownWorld();
        if (scan_world && !scan_world->empty()) {
            sensor_msgs::msg::PointCloud2 scan_msg;
            pcl::toROSMsg(*scan_world, scan_msg);
            scan_msg.header.frame_id = "map";
            scan_msg.header.stamp = stamp;
            cloud_pub_->publish(scan_msg);
        }
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    /// 回环关键帧：只入队（LoopClosing 有自己的线程），但入队本身也要花时间
    if (options_.with_loop_closing_) {
        Timer::Evaluate([&]() { lc_->AddKF(cur_kf_); }, "LoopClosing AddKF");
    }

    if (options_.with_gridmap_) {
        Timer::Evaluate([&]() { g2p5_->PushKeyframe(cur_kf_); }, "G2P5 PushKeyframe");
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }

    /// 全局地图发布（lightning/global_map，仅供 rviz 看）。
    ///
    /// 【为什么不再定期重算整图】
    /// 原来是「每隔几秒把**全部关键帧**重新拼接 + 体素滤波一次」，实测 1663 kf 时
    /// 单次 ~500ms（P95 950ms），周期性地把 worker 堵死（表现为 arrival_dt 突刺）。
    /// 但 rviz 的点云显示项本来就会把历史点留着（Decay Time），
    /// 定期重发整图纯属浪费 —— 只要把**新出现的关键帧**补上去就够了。
    /// 具体发什么由 PublishGlobalMap() 按 options_.global_map_pub_mode_ 决定。
    ///
    /// 这里只负责：① 没人订阅就不算不发；② 按时间节流
    /// （不是按关键帧数 —— 后者会随地图变大而变密）。
    const double now_sec = ToSec(stamp);
    const size_t global_map_sub_count =
        (map_pub_ != nullptr && options_.global_map_pub_interval_ > 0) ? map_pub_->get_subscription_count() : 0;

    if (global_map_sub_count == 0) {
        /// 没人看：什么都不做。顺便把增量发布攒下的状态释放掉（内存只在使用 rviz 期间占用）；
        /// 下次有人订阅时会从头重发一遍完整地图（见 PublishGlobalMap）。
        if (pub_sub_count_ != 0) {
            pub_sub_count_ = 0;
            pub_kf_count_ = 0;
            std::unordered_set<uint64_t>().swap(pub_voxels_);
            map_pose_corrected_ = false;
        }
    } else if (last_map_pub_time_ <= 0 || now_sec - last_map_pub_time_ >= options_.global_map_pub_interval_) {
        last_map_pub_time_ = now_sec;
        Timer::Evaluate([&]() { PublishGlobalMap(global_map_sub_count); }, "PublishGlobalMap");
    }
}

void SlamSystem::PublishGlobalMap(size_t sub_count) {
    if (map_pub_ == nullptr) {
        return;
    }

    /// 0 -> 非0：rviz 刚打开或重连（用 rviz 调试时很常见：关掉看一眼、再打开），
    /// 它手里什么都没有，必须把整张地图从头补发一遍。
    /// 直接走增量路径（把游标清 0），效果就等于重发全图。
    ///
    /// 注意：这是一次性停顿，不是原来的“每 5 秒都要付一次”。
    /// 成本 ≈ 0.4ms × 历史关键帧数（逐帧降采样走缓存，剩下的主要是 toROSMsg+DDS），
    /// 500 个关键帧就是 ~200ms —— 会触发一次 recv falling behind 警告，但无害
    /// （积压一两帧，随即追平）。嫌它难看的可用 global_map_pub_max_kf 分批补。
    if (pub_sub_count_ == 0) {
        pub_kf_count_ = 0;
        std::unordered_set<uint64_t>().swap(pub_voxels_);
        map_pose_corrected_ = false;
        LLOG_INFO(logging::kSlam,
                  "global map: 检测到新的订阅者（rviz 刚打开/重连），从头补发整张地图。"
                  "预计一次性停顿 ~0.4ms/历史关键帧；若嫌停顿大可调小 system.global_map_pub_max_kf 分批补");
    }
    pub_sub_count_ = sub_count;

    const float res = options_.global_map_pub_res_;
    const bool use_lio_pose = !options_.with_loop_closing_;
    const bool incremental = options_.global_map_pub_mode_ != "full";

    /// 本次到底发了哪一段关键帧 —— 日志用。
    /// 补发时是 [0, N)，平时是 [上次游标, 本次游标)。
    const size_t kf_begin = pub_kf_count_;

    CloudPtr cloud;
    if (!incremental) {
        /// 老行为：每次都重算整图。1663 kf 时单次 ~500ms，只在需要完整点云时才用。
        Timer::Evaluate([&]() { cloud = lio_->GetGlobalMap(use_lio_pose, true, res); }, "GetMapFull");
    } else {
        if (map_pose_corrected_.exchange(false)) {
            /// 回环优化改写了历史关键帧的位姿 -> 已经发出去的点位置全错了。
            /// rviz 无法「撤回」已画出的点，只能把修正后的地图重新发一遍；
            /// 修正量大的话会看到重影，需要重开该显示项才能清干净。
            LLOG_WARN(logging::kSlam,
                      "检测到回环修正了历史位姿，rviz 中已累积的点位置已失效，正在重发；如有重影请重开 rviz 显示");
            pub_kf_count_ = 0;
            std::unordered_set<uint64_t>().swap(pub_voxels_);
        }

        Timer::Evaluate(
            [&]() {
                cloud = lio_->GetNewKeyframesCloud(pub_kf_count_, use_lio_pose, res, true,
                                                   static_cast<size_t>(options_.global_map_pub_max_kf_));
            },
            "GetMapDelta");
    }

    if (!cloud || cloud->empty()) {
        return;
    }

    /// 增量模式下跨调用做体素去重：同一个体素只发一次。
    /// 否则同一面墙会被经过它的每个关键帧各发一遍，rviz 累积的点数随运行时间无界增长。
    CloudPtr to_pub = cloud;
    if (incremental) {
        to_pub = std::make_shared<PointCloudType>();
        to_pub->reserve(cloud->size());

        /// 预扩桶：不预扩的话，首次连 rviz 时要一次性插几百万个体素，
        /// unordered_set 会在扩容点反复 rehash（单次可达几十毫秒），
        /// 每一次都表现为一个 arrival_dt 突刺。
        pub_voxels_.reserve(pub_voxels_.size() + cloud->size());

        Timer::Evaluate(
            [&]() {
                for (const auto& p : cloud->points) {
                    if (pub_voxels_.insert(VoxelKey(p, res)).second) {
                        to_pub->points.push_back(p);
                    }
                }
            },
            "MapVoxelDedup");
        to_pub->height = 1;
        to_pub->width = to_pub->points.size();
        to_pub->is_dense = false;

        LLOG_INFO(logging::kSlam, "map delta: kf [{}, {}){} | delta pts {} -> new {} | 累计体素 {}", kf_begin,
                  pub_kf_count_, (kf_begin == 0) ? "  <- 补发整图" : "", cloud->size(), to_pub->points.size(),
                  pub_voxels_.size());
    }

    /// toROSMsg 对大点云是逐字段拷贝，几百万点时会明显花时间；
    /// 单独计时，才能分清「拼接/去重」和「序列化+发布」各占多少。
    Timer::Evaluate(
        [&]() {
            sensor_msgs::msg::PointCloud2 ros_map;
            pcl::toROSMsg(*to_pub, ros_map);
            ros_map.header.frame_id = "map";
            ros_map.header.stamp = node_->now();
            map_pub_->publish(ros_map);
        },
        "MapToRos");
}

void SlamSystem::StopWorker() {
    stop_worker_ = true;
    if (worker_.joinable()) {
        worker_.join();

        /// 接收侧健康度汇总：出帧间隔异常的帧数。
        /// 它持续增长说明驱动/雷达在丢帧（而非我们消费不过来）。
        if (options_.log_lidar_time_ && frame_count_.load() > 0) {
            LLOG_INFO(logging::kSlam, "[lidar] frames {} | driver gap > {:.0f} ms: {} ({:.2f}%)", frame_count_.load(),
                      kDriverGapWarnMs, driver_gap_warn_count_.load(),
                      100.0 * driver_gap_warn_count_.load() / frame_count_.load());
        }

        /// 注：各阶段耗时的汇总由 run_slam_online 在 Spin() 返回后统一打印
        /// （Timer::PrintAll），这里不再重复调用。
    }
}

}  // namespace lightning
