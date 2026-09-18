//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <atomic>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>
#include <unordered_set>

#include "lightning/msg/nav_state.hpp"
#include "lightning/srv/save_map.hpp"
#include "lightning/srv/save_path.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

class LaserMapping;  //  lio 前端
class LoopClosing;   // 回环检测

namespace ui {
class PangolinWindow;
}

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;  // 在线模式，在线模式下会起一些子线程来做异步处理

        bool with_cc_ = true;               // 是否需要带交叉验证
        bool with_gridmap_ = true;          // 是否需要2D栅格
        bool with_loop_closing_ = true;     // 是否需要回环检测
        bool with_visualization_ = true;    // 是否需要可视化UI
        bool with_2dvisualization_ = true;  // 是否需要2D可视化UI

        bool step_on_kf_ = true;  // 是否在关键帧处暂停p

        bool pub_tf_ = false;             // 是否发布TF（map->odom->base_link->imu_link->lidar_link，与定位一致）
        bool pub_odom_ = false;           // 是否发布Odometry与NavState
        bool enable_lidar_rviz_ = false;  // 是否需要RViz可视化点云
        bool enable_path_rviz_ = false;   // 是否发布Path
        bool log_pose_opt_ = false;       // 是否打印位姿和速度

        /// 是否启用 LIO 工作线程（推荐 true）。
        /// true : 点云回调只做预处理+入队（实测 ~1.4ms），LIO 与发布都在工作线程上，
        ///        不会把 executor 与驱动侧的 publish 拖住；
        /// false: 兼容旧行为，在回调里同步跑完 LIO 与发布（回调变慢，
        ///        可能重新触发驱动侧反压，仅供回滚排查）。
        bool threaded_lio_ = true;

        /// LIO 工作线程的 nice 值（-20 ~ 19，默认 10）。
        /// 动机（当初为 RK3588 加的）：算力紧张平台上单帧 LIO 会占满一个核，
        /// 若不降优先级，executor 线程上的 IMU / 点云回调会被饿住，
        /// reader 队列积压后会反过来把雷达驱动压降频。
        /// 但 nice 只在“有竞争”时起作用：x86 上 LIO 仅用 ~4% 单核，
        /// 降优先级反而让工作线程在 bag/rviz 抢 CPU 时第一个被牺牲。
        /// 建议：x86 设 0，RK3588 保持 10（可用 launch 的 lio_worker_nice:= 覆盖）。
        int worker_nice_ = 10;

        /// 是否逐帧输出雷达处理耗时（点数 / LIO 耗时 / 端到端延迟）。
        /// 10Hz 下相当于每秒一行，调试用；正式跑建议关闭。
        bool log_lidar_time_ = false;

        /// 全局地图（lightning/global_map）的发布间隔（秒），0 = 不发布。
        /// 只在确实有订阅者时才计算/发布。含义随 global_map_pub_mode_ 变化。
        double global_map_pub_interval_ = 1.0;

        /// 全局地图发布模式（详见 SlamSystem::PublishGlobalMap）：
        ///
        ///   "incremental"（默认）只发「新出现的关键帧」的点云，由 rviz 自己累积成整图。
        ///                 rviz 侧要把该显示项的 Decay Time 设为 0（永不衰减）。
        ///                 跨调用做体素去重：同一个体素只发一次，否则同一面墙会被 N 个
        ///                 关键帧各发一遍，rviz 累积的点数会随运行时间无界增长。
        ///                 单次成本 = O(新增关键帧) ≈ 亚毫秒，与地图大小无关。
        ///
        ///   "full"       每次重算整张图（老行为）。实测 1663 kf 时单次 ~500ms（P95 950ms），
        ///                 会把 worker 周期性地堵住。只在「需要拿到完整点云」时用，
        ///                 例如 init_selftest / pcd2pgm 这类离线流程。
        ///
        ///   "off"        不发布（等同 global_map_pub_interval = 0）。
        std::string global_map_pub_mode_ = "incremental";

        /// 增量发布时单个关键帧的体素分辨率，也就是 rviz 最终看到的点云密度。
        /// 0.1 与 GetGlobalMap 的默认值一致（视觉上等密度）；调大可显著降低 rviz 负担。
        float global_map_pub_res_ = 0.1f;

        /// 单次增量发布最多处理多少个关键帧（限流），0 = 不限（默认）。
        ///
        /// 正常运行时每秒只新增几个关键帧，这个上限平时不起作用。
        /// 它只影响一种情况：rviz 中途连上，要把全部历史关键帧补发一遍。
        /// 补发的主要成本是「每个关键帧首次降采样」0.377ms（缓存冷），
        /// 所以 800 个关键帧左右就是 ~300ms 级的一次性停顿。
        ///
        /// 权衡（实测预警阈值 kRecvBacklogWarnMs = 30ms）：
        ///   0   （默认）一次停顿把地图补齐 —— 只埋一次警告，但停顿随地图大小增长
        ///        （1 小时的图 ~4000 关键帧，停顿约 3s）。
        ///   32  每次发布只多花 ~20ms（低于阅值，不再报警），代价是地图要
        ///        几十秒才能“长”完（补发速度 = 上限 - 关键帧产生速度）。
        int global_map_pub_max_kf_ = 0;

        /// 轨迹（lightning/path）的最小发布间隔（秒），0 = 不发布。
        /// 同 global_map 的毛病：path_ 无界增长，每帧 publish 要把整个 vector
        /// 序列化一遍（O(n)/帧，O(n²) 累计）。现在只在有订阅者时按时间节流发布；
        /// path_ 本身仍然每帧累积（save_path 导出轨迹依赖它）。
        double path_pub_interval_ = 1.0;

        /// pcd2pgm：保存时由全局地图（世界系）直接生成2D栅格地图的参数
        struct Pcd2PgmParams {
            double thre_z_min_ = 0.0;       // 高度带下沿（世界系z，起点传感器高度处为0）
            double thre_z_max_ = 2.0;       // 高度带上沿，带内点视为障碍
            double thre_radius_ = 0.2;      // 半径去噪的搜索半径（m）
            int thres_point_count_ = 10;    // 半径内邻居数低于该值的点被剔除
            double map_resolution_ = 0.05;  // 栅格分辨率（m/格）
        };

        bool with_pcd2pgm_ = true;  // 保存地图时生成2D栅格（开启后替代g2p5的pgm输出）
        Pcd2PgmParams pcd2pgm_;
    };

    using SaveMapService = srv::SaveMap;

    SlamSystem(Options options);
    ~SlamSystem();

    /// 初始化
    /// @param yaml_path         配置文件路径
    /// @param map_root_override 建图保存根目录；非空时覆盖 yaml 里的 system.map_root
    bool Init(const std::string& yaml_path, const std::string& map_root_override = "");

    /// 对外部交互接口
    /// 开始建图，输入地图名称
    void StartSLAM(std::string map_name);

    /// 保存地图到 <map_root_>/<map_name>/（map_root_ 见 system.map_root，缺省 $HOME/rcs/maps）
    void SaveMap(const std::string& path = "");

    /// 保存轨迹，默认保存至./data/path_时间戳.txt
    bool SavePath(const std::string& path = "");

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    /// ros端保存地图的实现
    void SaveMap(const SaveMapService::Request::SharedPtr request, SaveMapService::Response::SharedPtr response);

    /// ros端保存轨迹的实现
    void SavePath(const srv::SavePath::Request::SharedPtr request, srv::SavePath::Response::SharedPtr response);

    /// 由全局地图（世界系）生成2D栅格地图 map.pgm+map.yaml（pcd2pgm逻辑：z高度带+半径去噪+XY投影，无膨胀）
    void ConvertGlobalMapToGridMap(const CloudPtr& global_map, const std::string& save_path);

    /// 统一发布 TF/odom/nav_state/path：map→odom→base_link（建图时 map≈odom），与定位 loc_system 同构
    void PublishMappingTf(const builtin_interfaces::msg::Time& stamp);

    /// ================= 工作线程（M1） =================
    /// 线程模型：ROS executor 单线程，回调只做「入队」这类微秒级操作；
    /// LIO 计算与全部发布（TF/点云/关键帧链路）都在 worker_ 上执行。
    /// 并发边界：LaserMapping 内部由 mtx_state_ / mtx_buffer_ 保护（见 laser_mapping.h）。

    /// 工作线程主循环：不断尝试消费 LIO 输入队列并发布结果
    void RunWorker();

    /// 执行一次 Run()，成功则接着走发布链路。
    /// 串行执行路径（system.threaded_lio=false）也会复用它。
    /// @return Run() 是否成功
    bool RunOnceAndPublish();

    /// Run() 成功后的发布链路，在工作线程上执行
    /// @param lio_ms 本帧 LIO 耗时（毫秒），仅用于逐帧日志
    void ProcessOnce(double lio_ms);

    /// 接收侧健康度统计（仅由回调线程调用，内部状态无需加锁）。
    /// 对比“消息到达间隔”与“消息内时间戳间隔”：
    ///   stamp_dt   变大 -> 驱动/雷达自己丢帧
    ///   arrival_dt 持续大于 stamp_dt -> 接收侧（DDS/调度）在积压
    /// @param stamp_sec 消息 header.stamp（秒）
    void MonitorRecvHealth(double stamp_sec);

    /// 停止并 join 工作线程（幂等，可重复调用）
    void StopWorker();

    /// 发布 lightning/global_map（调用点见 ProcessOnce）。
    /// 按 options_.global_map_pub_mode_ 决定发增量还是整图；整图路径见 Options 里的说明。
    /// 只由工作线程调用。
    /// @param sub_count 本次看到的订阅者数量
    void PublishGlobalMap(size_t sub_count);

    Options options_;
    std::atomic_bool running_ = false;

    /// LIO 工作线程
    std::thread worker_;
    std::atomic_bool stop_worker_ = false;

    /// 逐帧日志用：最近入队点云的点数。
    /// 回调线程写、工作线程读，故用 atomic；仅在 log_lidar_time_ 开启时有意义。
    /// 注意「入队时刻」不在这里：它必须跟着每一帧走（见 LaserMapping::arrival_time_buffer_），
    /// 否则 worker 追赶积压时这个值不再更新，算出的 e2e 会随处理进度线性虚增。
    std::atomic<size_t> last_scan_points_{0};

    /// 接收侧健康度：上一帧的 header.stamp 与到达时刻（仅回调线程访问，无需 atomic）
    double last_lidar_stamp_ = 0.0;
    double last_lidar_arrival_ = 0.0;

    /// 上一次 lidar 回调**自身**的耗时（毫秒，仅回调线程访问）。
    ///
    /// 用处是把「recv falling behind」的锅分清（警告里会直接打出来）：
    ///   上次回调很长      -> executor 线程被我们自己的回调占住了
    ///   worker 有积压     -> worker 落后（rviz 补发整图、回环优化…）
    ///   两者都正常        -> 阻塞在回调之外：DDS 投递 / 调度 / 别的进程抢 CPU
    double last_lidar_cb_ms_ = 0.0;

    /// 上一次发布 lightning/global_map 的时刻（lidar 时间戳，秒）。仅工作线程访问。
    double last_map_pub_time_ = 0.0;

    /// ===== 增量地图发布状态（仅工作线程访问，无需加锁）=====
    /// 已经发出去的关键帧数（下标），下次发布从这里往后取。
    size_t pub_kf_count_ = 0;
    /// 上一次发布时看到的订阅者数量。0 -> 非0 视为“rviz 刚打开/重连”：
    /// 它手里什么都没有，必须从头重发一遍（否则只能看到后半段地图）。
    size_t pub_sub_count_ = 0;
    /// 已经发出去的体素 key。跨调用去重，保证同一面墙只发一次。
    /// 订阅者归零时释放，内存只在使用 rviz 期间占用。
    std::unordered_set<uint64_t> pub_voxels_;
    /// 回环优化改写了**历史**关键帧的位姿 -> 已经发到 rviz 的点位置全部失效。
    /// 由回环回调（回环线程）置位，工作线程在下次发布时读取并清零。
    std::atomic_bool map_pose_corrected_ = false;

    /// 上一次发布 lightning/path 的时刻（lidar 时间戳，秒）。仅工作线程访问。
    double last_path_pub_time_ = 0.0;
    /// 最近一帧的驱动出帧间隔（= header.stamp 差值，毫秒）。
    /// 回调线程写、工作线程读，用于逐帧日志，故用 atomic。
    std::atomic<double> last_scan_dt_ms_{0.0};

    /// 接收侧健康度汇总（退出时打印）：总帧数 与 出帧间隔异常的帧数。
    /// 后者持续增长即说明驱动/雷达侧在丢帧，而不是我们消费不过来。
    std::atomic<int> frame_count_{0};
    std::atomic<int> driver_gap_warn_count_{0};

    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_ = nullptr;
    rclcpp::Service<srv::SavePath>::SharedPtr savepath_service_ = nullptr;

    std::string map_name_;  // 地图名

    /// 建图保存根目录（已展开 ~、已去尾部 '/'）。地图保存到 <map_root_>/<map_name_>/。
    /// 取值：ROS 参数 map_root > yaml 的 system.map_root > $HOME/rcs/maps > ./data
    std::string map_root_ = "./data";

    std::shared_ptr<LaserMapping> lio_ = nullptr;       // lio 前端
    std::shared_ptr<LoopClosing> lc_ = nullptr;         // 回环检测
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;  // ui
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;        // 栅格地图

    Keyframe::Ptr cur_kf_ = nullptr;

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;

    /// rviz/调试发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_ = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ = nullptr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_ = nullptr;
    rclcpp::Publisher<msg::NavState>::SharedPtr nav_state_pub_ = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ = nullptr;

    /// 静态外参：base_link→imu_link / imu_link→lidar_link（与定位 loc_system 同键，缺省单位阵）
    SE3 T_base_imu_ = SE3();
    SE3 T_imu_lidar_ = SE3();

    nav_msgs::msg::Path path_;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
