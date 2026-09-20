#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <list>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <vector>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/log.h"
#include "common/options.h"
#include "fastlio/imu_processing.hpp"
#include "ikd-Tree/ikd_Tree.h"
#include "pointcloud_preprocess.h"
#include "utils/timer.h"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        bool enable_icp_part_ = true;    // 是否添加ICP部分
        double plane_icp_weight_ = 1.0;  // 点面ICP部分的权重
        double icp_weight_ = 100;        // ICP部分的权重

        int min_pts = 300;  // 配准所需的点数

        /// 关键帧阈值
        double kf_dis_th_ = 2.0;
        double kf_angle_th_ = 15 * M_PI / 180.0;

        bool proj_kfs_ = false;
        int max_proj_kfs_ = 5;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using KDTreeType = KD_TREE<PointType>;
    using ESKFType = ::esekfom::esekf<fastlio::state_ikfom, 12, fastlio::input_ikfom>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        if (odom_log_.is_open()) {
            odom_log_.close();
        }
        if (pos_log_.is_open()) {
            pos_log_.close();
        }
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LLOG_DEBUG(logging::kLio, "laser mapping deconstruct");
    }

    /// init without ros
    bool Init(const std::string &config_yaml);

    bool Run();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    /// 设置初始位姿（定位模式下将 LO 对齐到地图系）
    void SetInitPose(const SE3 &pose);

    /// 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    /// 以下读接口均在锁保护下返回快照，允许 ROS 侧（service 回调/其他线程）
    /// 与 LIO 工作线程并发调用。注意：返回的是内部 shared_ptr 的副本，
    /// 其指向的点云仍会被下一帧 Run() 原地覆盖，调用方应立即使用、不要长期持有。

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const;

    /// 获取激光的状态
    NavState GetState() const;

    /// 获取经过回环优化后的位姿（基于当前帧相对lastKF的LIO位姿递推）
    SE3 GetOptPose() const;

    /// 获取IMU状态
    NavState GetIMUState() const;

    /// 获取IMU状态（非阻塞）：拿不到 state 锁时立刻返回 pose_is_ok_=false。
    /// 专供 IMU 回调使用 —— 回调绝不能被 LIO 线程的长计算阻塞。
    NavState GetIMUStateNonBlocking() const;

    CloudPtr GetScanUndist() const;
    CloudPtr GetScanDownWorld() const;
    CloudPtr GetProjCloud();

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    /// 输入队列是否还有待处理的数据（buffer 锁保护）。
    /// 供工作线程做空转保护：无数据时不必调用 Run()，免得刷出无意义的告警。
    bool HasPendingScan() const;

    /// 积压保护：队列中未处理的点云帧超过 keep 帧时，丢弃最旧的帧。
    /// 工作线程跟不上生产速度时（如 CPU 被抢、回环优化卡顿），
    /// 丢旧帧比让 lidar_buffer_ 无限增长导致内存爆更安全；
    /// 被丢帧覆盖的那段 IMU 会在下一帧同步时被自然跳过。
    /// @return 本次丢弃的帧数
    size_t DropStaleScans(size_t keep);

    /// 当前待处理的点云帧数（仅用于监控/告警）
    size_t GetPendingScanCount() const;

    /// 最近一次被 Run() 消费的那帧的到达时刻（steady clock，秒）。
    /// 上游用它算 e2e/queue：必须取“该帧自己的”到达时刻，不能用全局“最新到达时刻”。
    double GetLastProcessedArrivalTime() const { return last_processed_arrival_time_.load(); }

    std::vector<Keyframe::Ptr> GetAllKeyframes();

    /**
     * 计算全局地图（整图重算，O(关键帧数)）
     *
     * 只应该用于「真的需要完整点云」的场合（存图 srv/save_map）。
     * rviz 看地图请用 GetNewKeyframesCloud() 增量发，靠 rviz 自己累积，
     * 不要把这个整图重算搬到每帧的发布路径上。
     *
     * @param use_lio_pose true=用前端位姿，false=用后端优化位姿
     * @param use_voxel    是否做体素降采样
     * @param res          体素分辨率
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

    /**
     * 取「第 io_published_count 个及之后」的关键帧拼成的世界系点云（增量发布用）。
     *
     * 返回后 io_published_count 被更新为本次快照的关键帧总数，
     * 所以下一次调用自然只处理新出现的那些 —— 单次成本 = O(新增关键帧)，
     * 与地图大小无关（对比 GetGlobalMap 的 O(全部关键帧)）。
     *
     * 注意：没有做最后一次全量体素滤波，同一个体素会在不同关键帧里重复出现，
     * 调用方若在乎点数需要自己跨调用去重（见 SlamSystem::PublishGlobalMap）。
     *
     * @param io_published_count 入/出参：起始关键帧下标；返回后更新为本次快照长度
     * @param use_lio_pose       true=用前端位姿，false=用后端优化位姿
     * @param res                单帧体素降采样分辨率（走 Keyframe 内的缓存）
     * @param max_kfs            本次最多取多少个关键帧，0 = 不限。
     *                           用于限流：rviz 中途连上时要补发全部历史关键帧（可能上千个），
     *                           一次做完会堵住 worker 几百毫秒；分批发就是“地图慢慢长出来”。
     *                           正常每秒只新增几个关键帧，此上限不起作用。
     * @return 无新增关键帧（或已达上限且无可取）时返回 nullptr
     */
    CloudPtr GetNewKeyframesCloud(size_t &io_published_count, bool use_lio_pose, float res, bool use_voxel = true,
                                  size_t max_kfs = 0);

   private:
    // sync lidar with imu
    bool SyncPackages();

    /**
     * 把关键帧区间 [begin, end) 逐个降采样 + 转世界系 + 拼成一块点云。
     *
     * GetGlobalMap() / GetNewKeyframesCloud() 共用这一条路径 ——
     * lidar->IMU 外参的处理只在这一处，避免以后再出现「某个调用点漏乘外参」的 bug。
     * 子阶段计时名：Map:kf-voxel / Map:trans+cat。
     *
     * @param raw_points / filtered_points 累计点数，供日志用
     */
    CloudPtr BuildCloudFromKeyframes(const std::vector<Keyframe::Ptr> &kfs, size_t begin, size_t end, bool use_lio_pose,
                                     bool use_voxel, float res, size_t &raw_points, size_t &filtered_points);

    /// esekfom 观测模型（点面），由 esekf 迭代更新时调用
    static void ObsModelFastLio(fastlio::state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

    /// state_ikfom → NavState 转换
    NavState StateToNavState() const;

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot.toRotationMatrix() *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    /// 移动/裁剪局部地图（ikd-Tree box删除）
    void LasermapFOVSegment();

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

    /// 将附近的关键帧投影至cloud中
    void ProjectKFs(CloudPtr cloud, int size_limit = 1000);

   private:
    Options options_;

    /// ===================== 并发保护 =====================
    /// 锁序固定为 mtx_state_ -> mtx_buffer_，反向获取会死锁。
    ///
    /// mtx_buffer_: 只保护输入队列 lidar_buffer_/time_buffer_/imu_buffer_ 及其时间戳
    ///              (last_timestamp_lidar_/last_timestamp_imu_/lidar_pushed_)。
    ///              只允许做“入队/出队搬运”，禁止在持锁期间做预处理、配准等重计算。
    /// mtx_state_ : 保护其余全部算法状态与结果 —— EKF 状态 (kf_/state_point_/state_timestamp_)、
    ///              ikd-Tree、关键帧 (last_kf_/all_keyframes_/kf_id_/proj_kfs_)、
    ///              scan_undistort_/scan_down_body_/scan_down_world_、measures_ 等。
    ///              Run() 全程持有此锁；ROS 侧读接口在锁内取快照后立即释放。
    mutable std::mutex mtx_state_;

    /// modules
    std::shared_ptr<KDTreeType> ikdtree_ = nullptr;               // localmap in ikd-Tree
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<fastlio::ImuProcess> p_imu_ = nullptr;        // fastlio imu process (esekfom)

    /// local map related
    double filter_size_map_min_ = 0;

    /// 关键帧点云的存储分辨率（米）。0 = 存原始去畸变扫描（见 MakeKF）。
    /// 取值与 GetGlobalMap 的 res 相同时，存图时那一段逐帧滤波会被完全跳过。
    float kf_cloud_res_ = 0;

    double cube_len_ = 1000.0;            // 局部地图cube边长
    bool local_map_initialized_ = false;  // 局部地图box是否初始化
    BoxPointType local_map_points_;       // 当前局部地图box

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    std::string map_file_path_;
    std::ofstream odom_log_;  // 常开里程计日志 (data/odom_log.txt)
    std::ofstream pos_log_;   // 常开位姿日志 (data/pos_log.txt，同 fast_lio 格式)

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    /// 点面相关
    std::vector<PointVector> nearest_points_;              // nearest points of current scan
    std::vector<std::vector<float>> point_search_sq_dis_;  // ikd-Tree 最近邻平方距离
    std::vector<Vec4f> corr_pts_;                          // inlier pts
    std::vector<Vec4f> corr_norm_;                         // inlier plane norms
    std::vector<float> residuals_;                         // point-to-plane residuals
    std::vector<char> point_selected_surf_;                // selected points
    std::vector<Vec4f> plane_coef_;                        // plane coeffs

    /// 点到点相关
    std::vector<char> point_selected_icp_;  // 点到点的selected points

    /// 输入队列专锁：见文件顶部 mtx_state_ 处的锁序说明
    mutable std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;
    /// 与 lidar_buffer_ 并行的“到达时刻”（steady clock，秒）。
    /// 必须是每帧一个：之前 SlamSystem 只用单个 atomic 记录“最新到达时刻”，
    /// worker 追赶积压时那个值不再更新，算出的 e2e/queue 会随处理进度线性虚增，
    /// 看起来像“延迟在涨”，实际只是测量伪影。
    std::deque<double> arrival_time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    /// 最近一次被 Run() 消费的那帧的到达时刻（steady 秒），供上游算 e2e 用
    std::atomic<double> last_processed_arrival_time_{0.0};

    /// options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;  // 雷达是否需要跳帧
    int skip_lidar_num_ = 5;         // 每隔多少帧跳一个雷达
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKFType kf_;                       // esekfom: 点云时刻的IMU状态
    fastlio::state_ikfom state_point_;  // ekf current state (esekfom)
    double state_timestamp_ = 0;        // 状态时间戳

    bool use_aa_ = false;  // use anderson acceleration? (esekfom 不使用，仅兼容配置读取)

    std::list<Keyframe::Ptr> proj_kfs_;  // 投影到当前帧的关键帧

    static LaserMapping *obs_model_this_;  // 供静态 ObsModelFastLio 访问实例

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
