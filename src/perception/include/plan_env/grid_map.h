#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;
// voxel hashing
template <typename T> struct matrix_hash {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  double double_cylinder_radius_, double_cylinder_offset_;
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting

  /* visualization and computation time display */
  double vis_height_, ground_height_;
  bool show_occ_time_;

  /* mapping sensor input */
  string sensor_type_;
  bool cloud_is_world_;
  bool need_extrinsic_;
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  double unknown_flag_;

  /* ---------- 2D 局部感知产物（高度带投影切片 + 可选 ESDF）---------- */
  bool enable_2d_;                // 总开关：false = 完全不做 2D，零额外开销
  bool pub_2d_occupancy_;         // 发原始占据 OccupancyGrid
  bool pub_2d_occupancy_inflate_; // 发膨胀后占据 OccupancyGrid（3D 双圆柱膨胀的
                                  // 2D 投影）
  bool pub_2d_esdf_;       // 发 ESDF（点云形式，intensity = 到最近障碍的距离）
  double pub_2d_interval_; // 发布节流间隔（秒），0 = 不限
  bool proj_relative_to_sensor_; // 高度带是否相对当前传感器高度（跨楼梯时有用）
  double proj_z_min_,
      proj_z_max_; // 投影高度带（map 系绝对高度 / 相对传感器高度）
  bool esdf_unknown_as_occupied_; // 未知区域是否当障碍（默认 false：乐观，与 3D
                                  // 层一致）
  double esdf_max_dist_;          // ESDF 距离截断（米）
  int esdf_pub_step_;             // ESDF 点云抽点步长（1 = 不抽）
  /**
   * 是否让 ESDF 保持“随时可用”：
   *   false（默认，省 CPU）= 只在「有订阅者」或下面这个开关为 true 时、且 2D
   * 占据确实变了 （脏标记）才算 EDT； true                 =
   * 每次发布都重算（旧行为）。 注意：进程内直接调 getDistance2D()、且没人订阅
   * ESDF 时，必须把它置 true， 否则会拿到陈旧的 ESDF → getDistance2D() 返回
   * -1。
   */
  bool esdf2d_query_en_;
  bool verify_2d_; // 自检：每帧用“暴力全扫”重算并与增量结果比对（默认
                   // false，很费 CPU）
  /**
   * 强制每帧全窗口重扫（退化为改动前的行为）。
   * 用途：① 与增量实现做 A/B；② 万一增量路径在现场出问题，可一键退回旧行为。
   */
  bool force_full_2d_;
  /**
   * 增量路径每隔多少次构建做一次“全量重扫”兜底（0 = 不做）。
   * 作用是**保险**而非修正已知缺陷：2D 层是 3D 缓冲的纯函数，增量路径只在
   * “某个 3D 写入点忘了挂钩标脏”时才会漏；定期全量把那类失效的寿命从“永久”
   * 压到 N 次构建，代价约为全量重扫的 1/N。默认 100（约 10 s @10 Hz）。
   */
  int refresh_full_2d_interval_;
  string topic_2d_occupancy_, topic_2d_occupancy_inflate_, topic_2d_esdf_;

  /* ---------- 输入话题名（相对名，会被解析成 <命名空间>/名字）----------
   * 默认值就是原来写死的名字，所以已有配置行为不变。
   * remap（launch 里 -r cloud:=...）优先级高于参数，两种方式都可用。
   */
  string topic_cloud_, topic_pose_, topic_depth_, topic_body_pose_;

  /* ---------- 2D 层 footprint 清除 ----------
   * 目的：雷达/相机架在机体上时，总有一部分自车几何会进视场。那些点若照常积分，
   * 2D 图上就会在机器人脚下画出一圈障碍（2D 是 3D 沿高度带的投影），规划器随后
   * 会把自身（及其膨胀）当成障碍。
   * 做法：每帧把机器人自身占的那块矩形从 2D 图（占据 + 膨胀）上抹掉 —— 在**输出
   * 侧**给一个几何保证：不管自车点云从哪来（雷达/相机/別家累积图），机器人脚下都
   * 不会是障碍。Nav2 的 local costmap 也是这么做的（每周期清 footprint 区域）。
   *
   * 矩形定义在**机体系**：x = 前后（长），y =
   * 左右（宽）；中心默认在传感器位置， 可用 offset
   * 挪到机体几何中心；朝向取传感器位姿的 yaw + yaw_offset 修正。 每帧在
   * build2DLayer() 末尾执行（增量路径被重扫的列也会重新抹一遍）。
   */
  bool footprint_clear_enable_;
  double footprint_length_;   // 机体系 x 方向（m）；<= 0 表示关闭
  double footprint_width_;    // 机体系 y 方向（m）；<= 0 表示关闭
  double footprint_offset_x_; // 矩形中心相对“传感器位置”的机体系偏移（m）
  double footprint_offset_y_;
  double footprint_yaw_offset_deg_; // 机体朝向 - 传感器朝向（度）
  /// true = 抹成未知(-1)；false = 抹成空闲(0)。默认空闲（规划器可走过自己脚下）
  bool footprint_clear_as_unknown_;

  /// 是否发布 footprint 可视化（MarkerArray：轮廓 + 半透明填充）
  bool pub_footprint_viz_;
  string topic_footprint_viz_;

  /* ---------- 可视化开销控制 ---------- */
  /// 可视化定时器周期（s）。它只管“给人看”的东西：TF、bbox、footprint、点云、2D。
  double vis_interval_;
  /**
   * 3D 占据/膨胀点云的发布间隔（s）。
   * 这两个点云是“全图体素 → 点”的输出，每发布一次都是 O(体素数)（见 off_idx_
   * 索引）。 TF/bbox 需要 10~20Hz，但点云 2~5Hz 就够看 —— 两者用不同频率。
   */
  double pub_map_interval_;
  /**
   * 占据体素索引自检：打开后定期用“暴力全扫”数一遍占据体素数，
   * 与索引列表比对，不一致打 ERROR。用于验证增量索引没有漏记。
   */
  bool verify_occ_idx_;
};

// intermediate mapping data for fusion

/// 某一时刻 footprint 在 map 系的位姿（位置 + yaw 的 cos/sin）
struct FootprintPose {
  double cx{0.0}, cy{0.0}, c{1.0}, s{0.0};
  bool valid{false};
};

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data

  cv::Mat depth_image_;
  int image_cnt_;
  // flags of map state

  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  /* ---------- 2D 局部层 ----------
   * x-y 与 3D 栅格同分辨率、同窗口（覆盖 [map_bound_min_idx_,
   * map_bound_max_idx_]）， 但**按全局索引用连续下标排列**（i = ix + iy *
   * nx），不做环形 wrap —— 这样它就是一个世界系下真正的矩形，EDT
   * 不会跨边界泄漏。 3D 缓冲是环形下标的，所以构建时是“读 3D（自动 wrap）→ 写
   * 2D（连续）”。
   */
  std::vector<int8_t> occ2d_; // 2D 占据：100 = 占据，0 = 空闲，-1 = 未知
  std::vector<int8_t> occ2d_inflate_; // 同上，来自 occupancy_buffer_inflate_
  std::vector<float> esdf2d_; // 到最近障碍的距离（米），截断于 esdf_max_dist_
  bool has_2d_;               // 是否已生成过可发布的 2D 数据（换图后复位）
  double proj_z_lo_,
      proj_z_hi_; // 本帧实际使用的投影高度带（发布时写进 origin.z）
  /**
   * 构建时刻的窗口最小索引。
   * ⚠ 必须记住它：滑动地图会在 updateSlidingMap() 里改 mp_.map_bound_min_idx_，
   * 而 build2DLayer() 与 publish2D()/查询发生在不同时刻（构建在 raycast 之后、
   * 发布在 visCallback），若发布时重新取 map_bound_min_idx_，滑动一次就会让整张
   * 2D 图与 origin 错位（实车一直在滑动 → 地图持续漂移）。
   */
  Eigen::Vector3i occ2d_min_idx_;

  /* ---------- 增量重建状态 ----------
   * 3D 层每真正改变一个体素的 occ/free/unknown 类别，就通过 mark2DColumnDirty()
   * 把该体素所属的 2D 列登记下来；build2DLayer() 只重扫这些列（每列
   * O(高度带层数)）， 不再每帧全窗口 200x200 全扫。 滑动窗口移动时把 2D
   * 数组原地平移以保持世界系对齐，新露出的条带全部标脏。
   */
  std::vector<uint8_t> occ2d_dirty_;  // 脏标记（去重用，1 字节/格）
  std::vector<int> occ2d_dirty_list_; // 脏列下标（i2d = ix + iy*nx）
  std::vector<int> occ2d_last_build_; // 每列最后一次重扫的构建序号（自检用）
  int occ2d_build_cnt_{0};            // 构建计数器
  int occ2d_since_full_{0};           // 距上次全量重扫过了多少次构建

  int occ2d_kz0_{0},
      occ2d_kz1_{-1};            // 当前 2D 数据对应的高度带（全局 z 体素下标）
  bool occ2d_band_valid_{false}; // 高度带是否有效（未构建 / 换图后为 false）
  bool esdf2d_stale_{true};      // ESDF 是否需要重算

  // footprint 清除：本帧 / 上一帧的机器人位姿（map 系）。
  // 上一帧那份用来把"上一帧被抹掉、本帧已离开 footprint"的格子标脏重扫 ——
  // 否则机器人走过后会残留"被抹成空闲"的假值（静止时看不出来，自检能抓到）。
  FootprintPose fp_cur_, fp_prev_;

  // 上一次点云回调耗时（ms）。帧耗时日志用：把"解析点云+筛选"和"3D
  // 融合"分开看。
  double t_cloud_ms_{0.0};

  /* ---------- 占据/膨胀体素索引（可视化发布用）----------
   * 为什么需要：occupancy_buffer_ 是稠密数组，publishMap() 原来只能遍历全部体素
   * （res0.10/10x10x5m = 50 万个，res0.05 时 400 万个）才找出占据体素，
   * 一次 ~100ms，却只在 20Hz 的可视化定时器里干“给人看”的活。
   * 这里增量维护一份“占据体素地址列表”，发布时只遍历列表（通常几千个）。
   *
   * 约定：
   *   · list 里可能含**坟墓**（旧条目已经不再占据）：发布时顺便原地压缩。
   *   · flag[addr] 保证同一个 addr 在 list 里最多出现一次（避免重复点）。
   *   · 任何“直接写缓冲”的地方（归零为未知、滑动清图、换图）都必须同步清 flag。
   */
  std::vector<int> occ_idx_list_, infl_idx_list_;
  std::vector<char> occ_idx_flag_, infl_idx_flag_;

  // 构建 2D 层用的临时缓冲（initMap 时一次性分配，避免每帧 new）
  std::vector<float> edt_f_, edt_sq_, edt_line_in_, edt_line_out_, edt_z_;
  std::vector<int> edt_v_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int &x, int &y, int &z);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);

  inline void boundIndex(Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3d &pos);
  inline bool isKnownFree(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3i &id);

  void initMap(rclcpp::Node *node);

  /**
   * 清空整张栅格并复位“已收到传感器位姿”标志。
   *
   * 用于运行时换图：map 系定义变了，旧地图下累积的所有体素全部失效。
   * 必须同时复位 has_ray_pose_——旧的 ray_pos_ 在新地图坐标系里没有意义，
   * 若不复位，换图后的第一批点云会用错误的射线原点做 raycast，把栅格打歪。
   * 复位后 cloudCallback 会重新等一个 sensor_pose 才继续更新（这是期望行为）。
   */
  void resetMap();

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  /* ---------- 2D 局部感知产物 ---------- */

  /**
   * 把 3D 占据栅格按高度带 [proj_z_min_, proj_z_max_] 投影成 2D 占据。
   *
   * 由 updateOccupancyCallback() 在 3D 融合结束后调用；
   * mp_.enable_2d_ 为 false 时直接返回（零开销）。
   *
   * **增量**：只重扫本帧发生类别变化（脏）的列，其余列沿用上一帧结果；
   * 滑动窗口移动时对 2D 数组做原地平移以保持世界系对齐，新露出的条带全部重扫。
   * 最坏情况（全图都脏）退化为全窗口重扫，即旧行为。
   * ESDF 不在这里算（见 build2DESDF），避免无人订阅时白算。
   */
  void build2DLayer();

  /**
   * 2D ESDF（到最近障碍的距离场，Felzenszwalb 1D EDT）。
   * 仅在 pub_2d_esdf_ 为真、且（esdf2d_query_en_ 或有人在订阅
   * ESDF）、且数据变脏时才重算。
   */
  void build2DESDF();

  /**
   * footprint 清除：把机器人自身那一块从 2D 图（占据 + 膨胀）上抹掉。
   * 由 build2DLayer() 末尾调用（每帧都执行），只遍历矩形外接圆覆盖的格子，
   * 代价与 footprint 面积成正比（典型 <100 格）。
   */
  void applyFootprintClear();

  /// 本帧 footprint 的位姿（清除/可视化/自检共用同一套算法）
  FootprintPose computeFootprintPose() const;

  /// 刷位姿缓存：fp_prev_ ← fp_cur_，fp_cur_ ← 当前位姿（build2DLayer 开头调）
  void updateFootprintPoseCache();

  /// 该位姿对应矩形的外扩矩形外接圆半径（用于遍历包围盒）
  double footprintCircumradius() const;

  /**
   * 发布 footprint 可视化（MarkerArray）：
   *   marker[0] LINE_STRIP 轮廓（绿，不透明）
   *   marker[1] CUBE 半透明填充（绿，alpha 0.25）
   * 只在本节点启用 footprint 时发；无人订阅时不构造不收。
   */
  void publishFootprintViz();

  /// 该格心（map 系 xy，单位 m）是否落在本帧 footprint 矩形内（含 yaw）
  inline bool inFootprint(double x, double y) const {
    if (!mp_.footprint_clear_enable_ || mp_.footprint_length_ <= 0.0 ||
        mp_.footprint_width_ <= 0.0 || !md_.fp_cur_.valid)
      return false;
    return inFootprintRect(md_.fp_cur_, x, y);
  }

  /// 某一位姿的（外扩半格）矩形判定
  inline bool inFootprintRect(const FootprintPose &fp, double x,
                              double y) const {
    const double dx = x - fp.cx;
    const double dy = y - fp.cy;
    const double lx = fp.c * dx + fp.s * dy;
    const double ly = -fp.s * dx + fp.c * dy;
    /* 外扩半格：判的是“**格与矩形相交**”而不是“格心在矩形内”。
       因为栅格里的占据格是“包含该点的格子”，格心最多能比真实几何外扩半格；
       不扩的话，按真实尺寸配的 footprint 会沿边缘漏一圈（实测过：0.30m 宽
       的机体 + 0.35m 宽的 mask，边缘格心正好落在 0.175 边界上，一格都没清）。
     */
    const double m = 0.5 * mp_.resolution_;
    return std::abs(lx) <= 0.5 * mp_.footprint_length_ + m &&
           std::abs(ly) <= 0.5 * mp_.footprint_width_ + m;
  }
  /// footprint 内格子应被抹成的值
  inline int8_t footprintValue() const {
    return mp_.footprint_clear_as_unknown_ ? static_cast<int8_t>(-1)
                                           : static_cast<int8_t>(0);
  }

  /* ---------- 占据体素索引维护（内联，热路径上只多一次分支）---------- */
  /// 体素“转入占据”时入列（flag 保证不重复）
  inline void idxNote(std::vector<int> &lst, std::vector<char> &flag,
                      int addr) const {
    if (addr < 0 || addr >= static_cast<int>(flag.size()) || flag[addr])
      return;
    flag[addr] = 1;
    lst.push_back(addr);
  }
  /// 体素“不再占据”时清位（条目留在 list 里当坟墓，发布时压缩）
  inline void idxDrop(std::vector<char> &flag, int addr) const {
    if (addr >= 0 && addr < static_cast<int>(flag.size()))
      flag[addr] = 0;
  }

  /// 占据索引自检（verify_occ_idx_ 打开时由 visCallback 定期调用）
  void verifyOccIdx();

  /**
   * 自检（verify_2d_ 打开时由 build2DLayer 调用）：用“暴力全扫”重算一遍 2D
   * 占据，
   * 与增量结果逐格比对，不一致就打日志。目的是让增量实现在真数据上也能自证正确。
   */
  void verify2DLayer(int dx = 0, int dy = 0, bool full = false,
                     int rebuilt = -1);

  /**
   * 发布 2D 产物：
   *   topic_2d_occupancy_         nav_msgs/OccupancyGrid   原始占据
   *   topic_2d_occupancy_inflate_ nav_msgs/OccupancyGrid   膨胀后占据
   *   topic_2d_esdf_              sensor_msgs/PointCloud2  xyz +
   * intensity(距离) 带节流（pub_2d_interval_）与“无订阅者不发布”。
   */
  void publish2D();

  /// 2D 层是否已生成（换图后会复位为 false）
  inline bool has2DLayer() const { return md_.has_2d_; }
  /// 2D 占据：100/0/-1；越界或未生成返回 -1
  inline int getOccupancy2D(int x, int y) const;
  inline int getOccupancy2D(const Eigen::Vector3d &pos);
  /**
   * 2D 距离（米）；越界、未生成、ESDF 未开启或当前是陈旧值（stale）时返回 -1。
   * ⚠ 本函数不会触发重算（const，且查询可能在任意线程/高频调用）。
   * 需要它随时可用就把 grid_map.esdf2d_query_en 置 true。
   */
  inline float getDistance2D(int x, int y) const;
  inline float getDistance2D(const Eigen::Vector3d &pos);
  /// 2D 层尺寸与 (0,0) 格角点在 map 系下的坐标
  inline void get2DInfo(int &nx, int &ny, double &origin_x,
                        double &origin_y) const;

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  /* ---------- 2D 层增量辅助（内联，3D 热路径上只多一次分支判断）---------- */

  /// 体素 log-odds 的三分类：0 = 未知，1 = 空闲，2 = 占据
  inline int occ2dClass(double log_odds) const {
    if (log_odds > mp_.min_occupancy_log_)
      return 2;
    return log_odds >= mp_.clamp_min_log_ ? 1 : 0;
  }

  /**
   * 把全局体素 (gx,gy,gz) 所属的 2D 列标记为脏。
   * 由 3D 侧的三个写入点调用：applyOccupancyUpdate / resetCellByAddress /
   * updateInflationLayer。enable_2d_ 关闭或高度带无效时是空操作。
   *
   * 下标基准用 **2D 数据自己的锚点** md_.occ2d_min_idx_（而不是当前的
   * mp_.map_bound_min_idx_）：因为脏列是给“当前这份 2D
   * 数组”用的，滑动发生在本帧内时 build2DLayer()
   * 会把脏列一起平移过去。新露出条带里的体素可能落到该矩形之外而被丢弃，
   * 但那些列本来就会被“全部标脏”规则覆盖，不影响正确性。
   */
  inline void mark2DColumnDirty(int gx, int gy, int gz) {
    if (!md_.occ2d_band_valid_ || md_.occ2d_dirty_.empty())
      return;
    if (gz < md_.occ2d_kz0_ || gz > md_.occ2d_kz1_)
      return;
    const int ix = gx - md_.occ2d_min_idx_(0);
    const int iy = gy - md_.occ2d_min_idx_(1);
    if (ix < 0 || iy < 0 || ix >= mp_.map_voxel_num_(0) ||
        iy >= mp_.map_voxel_num_(1))
      return;
    const int i2d = ix + iy * mp_.map_voxel_num_(0);
    if (!md_.occ2d_dirty_[i2d]) {
      md_.occ2d_dirty_[i2d] = 1;
      md_.occ2d_dirty_list_.push_back(i2d);
    }
  }

  // get depth image and sensor pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                         const nav_msgs::msg::Odometry::ConstSharedPtr &pose);
  void sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose);
  void
  slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img);

  // update occupancy by raycasting
  void updateOccupancyCallback();
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i &pt, int inf_step_xy,
                           int inf_step_z_up, int inf_step_z_down,
                           vector<Eigen::Vector3i> &pts);
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos,
                                           const std::vector<char> &buffer);
  inline int getLocalIndex(int id, int dim) const;
  inline int toAddressLocal(const Eigen::Vector3i &id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt,
                                   const Eigen::Vector3d &ray_pos);
  void updateSlidingMap(const Eigen::Vector3d &center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int addr);
  void resetCellByAddressForSliding(int addr,
                                    const std::vector<char> &clear_mask);
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i &id_g) const;
  void applyOccupancyUpdate(const Eigen::Vector3i &id, double new_log_odds);
  void rebuildInflationOffsets();
  void updateInflation(const Eigen::Vector3i &id, int delta,
                       const std::vector<char> *ignore_mask = nullptr);
  void updateInflationLayer(const Eigen::Vector3i &id, int delta,
                            const vector<Eigen::Vector3i> &offsets,
                            std::vector<int> &cnt_buffer,
                            std::vector<char> &flag_buffer,
                            const std::vector<char> *ignore_mask);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>>
      SynchronizerImagePose;

  rclcpp::Node *node_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      depth_pose_sub_;
  SynchronizerImagePose sync_image_pose_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      sliding_map_frame_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      sliding_map_bbox_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      footprint_viz_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr extrinsic_pose_pub_;

  /* 2D 局部产物发布器（enable_2d_ 为 false 时不创建）*/
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ2d_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ2d_inf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf2d_pub_;
  std::chrono::steady_clock::time_point last_2d_pub_time_;
  bool last_2d_pub_valid_{false};

  rclcpp::TimerBase::SharedPtr occ_timer_, vis_timer_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) *
             mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int &x, int &y, int &z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0)
    local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i &id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id) {
  Eigen::Vector3i id1;
  id1(0) =
      max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) =
      max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) =
      max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i &id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d &pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i &id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
         md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  int front_occ =
      getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
  if (front_occ != 0)
    return front_occ;

  return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);
}

inline int
GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos,
                                       const std::vector<char> &buffer) {
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(buffer[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 ||
      pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 ||
      pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) ||
      idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) ||
      idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos,
                                Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id,
                                Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int inf_step_xy,
                                  int inf_step_z_up, int inf_step_z_down,
                                  vector<Eigen::Vector3i> &pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y) {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline int GridMap::getOccupancy2D(int x, int y) const {
  if (!md_.has_2d_)
    return -1;
  const int ix = x - md_.occ2d_min_idx_(0);
  const int iy = y - md_.occ2d_min_idx_(1);
  if (ix < 0 || iy < 0 || ix >= mp_.map_voxel_num_(0) ||
      iy >= mp_.map_voxel_num_(1))
    return -1;
  return static_cast<int>(md_.occ2d_[ix + iy * mp_.map_voxel_num_(0)]);
}

inline int GridMap::getOccupancy2D(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy2D(id(0), id(1));
}

inline float GridMap::getDistance2D(int x, int y) const {
  // esdf2d_stale_：ESDF 按需重算（见 esdf2d_query_en_ /
  // build2DESDF），陈旧时返回 -1
  if (!md_.has_2d_ || md_.esdf2d_.empty() || md_.esdf2d_stale_)
    return -1.f;
  const int ix = x - md_.occ2d_min_idx_(0);
  const int iy = y - md_.occ2d_min_idx_(1);
  if (ix < 0 || iy < 0 || ix >= mp_.map_voxel_num_(0) ||
      iy >= mp_.map_voxel_num_(1))
    return -1.f;
  return md_.esdf2d_[ix + iy * mp_.map_voxel_num_(0)];
}

inline float GridMap::getDistance2D(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance2D(id(0), id(1));
}

inline void GridMap::get2DInfo(int &nx, int &ny, double &origin_x,
                               double &origin_y) const {
  nx = mp_.map_voxel_num_(0);
  ny = mp_.map_voxel_num_(1);
  origin_x = md_.occ2d_min_idx_(0) * mp_.resolution_;
  origin_y = md_.occ2d_min_idx_(1) * mp_.resolution_;
}

#endif
