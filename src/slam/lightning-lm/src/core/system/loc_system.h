//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lightning/msg/nav_state.hpp"
#include "lightning/srv/load_map.hpp"
#include "lightning/srv/save_path.hpp"
#include "lightning/srv/set_init_pose.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    /// 定位系统的运行阶段（注意：与 lightning/loc_state 是不同的东西）
    ///  - loc_state 是“定位算法”的状态（IDLE/INITIALIZING/GOOD/FAIL），由 LidarLoc/PGO 产出
    ///  - LocPhase 是“系统生命周期”的状态，控制要不要接收雷达/IMU、能不能换图
    enum class LocPhase : int32_t {
        kIdle = 0,       // 未启动（StartLoc 之前，很短）
        kRunning = 1,    // 数据流已开；定位是否就绪看 loc_state（INITIALIZING/GOOD）
        kSwitching = 2,  // 换图中：丢弃输入、拒绝 initialpose
        kError = 3,      // 地图不可用，需人工介入
    };

    struct Options {
        bool pub_tf_ = true;         // 是否发布tf
        bool pub_odom_ = true;       // 是否发布nav_state和odometry
        bool log_pose_opt_ = false;  // 是否打印位姿

        /// 轨迹（lightning/path）的最小发布间隔（秒），0 = 不发布。
        /// path_ 随运行时间无界增长，每帧 publish 要把整个 vector 序列化一遍
        /// （O(n)/帧，O(n²) 累计）；定位会连续跑几小时，所以必须节流，
        /// 且只在确实有订阅者时才发。path_ 本身仍然每帧累积（save_path 依赖它）。
        double path_pub_interval_ = 1.0;
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化
    /// @param yaml_path          配置文件路径
    /// @param map_path_override  地图目录；非空时覆盖 yaml 里的 system.map_path
    ///                           （供上层在启动时决定加载哪张地图）
    bool Init(const std::string& yaml_path, const std::string& map_path_override = "");

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 启动定位（不指定初始位姿，由地图功能点 FP 自动初始化）
    /// 定位初值错误时（如工厂多层环境 identity 会锁错层），应优先用本接口，
    /// 让 LidarLoc 遍历地图 start/recover 等真实功能点完成初始化。
    void StartLoc();

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

    /// 当前阶段（供外部查询/诊断）
    LocPhase phase() const { return phase_.load(); }

   private:
    /// 处理临界区（gate & drain 的“读者侧”）。
    ///
    /// 设计要点：
    ///  - 第一道判断不取锁：换图/未启动时立刻返回（微秒级），不会堵住 executor 回调，
    ///    也就不会反压雷达驱动（参见 doc/mapping_threading_refactor.md 的 10Hz→5Hz 案例）。
    ///  - 第二道判断在取锁后做：关闭闸门（phase 置 kSwitching）与取锁之间可能存在窗口，
    ///    必须二次确认后才能碰 loc_ 的成员。
    ///  - 与 DrainInFlight() 配对：换图线程先关闸、再排空，之后才能安全重建 loc_。
    ///
    /// 锁序：mtx_processing_ -> Localization::global_mutex_（反向获取会死锁）
    class ProcessingScope {
       public:
        explicit ProcessingScope(LocSystem& owner) : owner_(owner) {
            if (owner_.phase_.load() != LocPhase::kRunning) {
                return;
            }
            lock_ = std::unique_lock<std::mutex>(owner_.mtx_processing_);
            active_ = owner_.phase_.load() == LocPhase::kRunning;
        }

        /// true 表示已进入临界区，可以安全使用 loc_
        bool active() const { return active_; }

       private:
        LocSystem& owner_;
        std::unique_lock<std::mutex> lock_;
        bool active_ = false;
    };

    /// 排空在途读者（gate & drain 的“写者侧”）。
    /// 调用前必须已经把关闸（phase_ = kSwitching），否则新读者会持续涌入。
    /// 只等那一个在途回调退出（≤ 一次回调耗时），因此不应长时间持锁。
    void DrainInFlight() { std::lock_guard<std::mutex> lk(mtx_processing_); }

    /// ros端保存轨迹的实现
    void SavePath(const srv::SavePath::Request::SharedPtr request, srv::SavePath::Response::SharedPtr response);

    /// 发布调试信息与rviz可视化话题（nav_state/odom/path/当前扫描/全局地图）
    void PublishDebugAndRviz(const builtin_interfaces::msg::Time& stamp);

    /// 定位高频结果回调：缓存 map→base_link，并按 map→odom→base_link 链广播 TF
    void HandleLocTf(const geometry_msgs::msg::TransformStamped& map_base);

    /// 运行中接收初始化位姿（rviz2 2D Pose Estimate → initialpose 话题）
    void OnInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /// Web 接口 lightning/set_initpose
    void HandleSetInitPose(const srv::SetInitPose::Request::SharedPtr req, srv::SetInitPose::Response::SharedPtr res);

    /// 自动参考高度：当前定位高度 → 地图起始功能点(start)高度 → 0
    double GetReferenceZ();

    /// ===== 运行时换图 =====

    /// 换图前置校验结果
    struct MapPrecheck {
        bool ok = false;
        std::string message;
    };

    /// 换图前置校验：目录/索引/区块点云全部合格才允许关闸。
    /// 整个校验过程 **不触碰任何现有状态**，因此失败时旧地图完全不受影响。
    /// 检查项：① index.txt 存在可读 ② 能解析出 ≥ 1 个区块 ③ 每个 <id>.pcd 存在且非空。
    /// 不检查功能点（FP）——本项目不使用 FP 初始化。
    MapPrecheck PrecheckMapDir(const std::string& dir) const;

    /// service 入口（在 executor 线程上跑，只做预检 + 启动线程，不阻塞）
    void HandleLoadMap(const srv::LoadMap::Request::SharedPtr req, srv::LoadMap::Response::SharedPtr res);

    /// 实际换图（在独立线程上跑，不能占用 executor：否则雷达/IMU 回调全停，
    /// DDS reader 队列堆积会反压雷达驱动降频）
    void LoadMapThread(const std::string& dir);

    /// 发布并记录系统阶段。map_state 是 latched（transient_local），
    /// 后启动的订阅者也能立即拿到当前状态。
    void PublishMapState(LocPhase p);

    /// 换图后清理与旧地图绑定的运行态（轨迹、缓存的 map 位姿、过渡 TF）
    void ResetRuntimeStateAfterMapSwitch();

    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;  // 定位接口

    /// 系统生命周期状态（与 loc_state 话题含义不同，见 LocPhase 注释）
    std::atomic<LocPhase> phase_{LocPhase::kIdle};

    /// 保护“使用 loc_ 的整段处理”，与换图线程的 DrainInFlight() 配对。
    /// 覆盖 ProcessIMU / ProcessLidar 的完整函数体（含 PublishDebugAndRviz），
    /// 因为 PublishDebugAndRviz 也会访问 loc_->GetLIO() / GetLidarLoc()，
    /// 那些成员在换图时会被重建。
    std::mutex mtx_processing_;

    /// 实际生效的两个路径（地图目录已归一化去掉尾部 '/'）。
    /// 记录在这里是为了后续运行时换图（LoadMap）能复用同一份 yaml 与默认地图目录。
    std::string yaml_path_;
    std::string map_path_;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ = nullptr;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;

    /// rviz/调试发布
    rclcpp::Publisher<msg::NavState>::SharedPtr nav_state_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;

    /// 下游感知（plan_env / GridMap）专用：map 系点云 + map 系雷达位姿。
    /// GridMap 要求 cloud_is_world=true（点云直接取坐标当世界系），且它以 Odometry.pose
    /// 作为 raycast 起点（传感器在世界系的位置）；二者必须成对且同一坐标系，
    /// 因此这里统一发 map 系，下游无需再换算。
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr perception_cloud_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr perception_pose_pub_ = nullptr;
    rclcpp::Service<srv::SavePath>::SharedPtr savepath_service_ = nullptr;

    /// ===== 运行时换图 =====
    rclcpp::Service<srv::LoadMap>::SharedPtr loadmap_service_ = nullptr;
    /// 系统阶段话题（latched）：0=kIdle 1=kRunning 2=kSwitching 3=kError
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr map_state_pub_ = nullptr;
    std::thread switch_thread_;            // 换图线程（一次一张图）
    std::atomic_bool switch_busy_{false};  // 防重入（phase_ 已是 kSwitching 时的补充保护）

    /// 运行中初始化位姿接口（rviz2 / Web）
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_ = nullptr;
    rclcpp::Service<srv::SetInitPose>::SharedPtr set_initpose_service_ = nullptr;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr loc_state_pub_ = nullptr;
    std::string init_pose_topic_ = "initialpose";

    /// 静态外参（base_link→imu_link / imu_link→lidar_link），缺省单位阵
    SE3 T_base_imu_ = SE3();
    SE3 T_imu_lidar_ = SE3();

    /// 最近一次 loc 全局位姿（实际语义为 map→IMU；loc_system 内再乘 T_imu_base 归算 base）
    std::mutex map_pose_mutex_;
    SE3 latest_map_pose_;
    bool latest_map_pose_valid_ = false;

    /// 定位 GOOD 前的过渡 map→odom（由 SetInitPose 的初始位姿与当时 LO 推出，缺省=单位阵），
    /// 用于补全 TF 树（否则 frame=odom 的消息在 rviz 中因缺 map→odom 被丢弃）
    SE3 pending_map_odom_ = SE3();

    double last_map_pub_time_ = 0;

    /// 上一次发布 lightning/path 的时刻（lidar 时间戳，秒）
    double last_path_pub_time_ = 0;

    nav_msgs::msg::Path path_;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
