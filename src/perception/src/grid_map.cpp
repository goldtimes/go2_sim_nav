#include "plan_env/grid_map.h"

#include <cmath>
#include <cstring> // memmove（2D 数组平移）
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <string>

namespace {
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value,
                    const T &default_value) {
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}

/// EDT 的“无穷大”哨兵值（用有限大数，避免 inf - inf 出 NaN）
constexpr float kEdtInf = 1e10f;

/**
 * 1D 欧氏平方距离变换（Felzenszwalb & Huttenlocher, 2012）
 * f[q] = 0 表示 q 处是障碍（种子），否则为 kEdtInf；d[q]
 * 输出到最近种子的平方距离（单位：索引²）。 v、z 为调用者提供的 scratch，长度需
 * >= n+1 / n+2。
 */
void edt1D(const std::vector<float> &f, std::vector<float> &d, int n,
           std::vector<int> &v, std::vector<float> &z) {
  int k = 0;
  v[0] = 0;
  z[0] = -kEdtInf;
  z[1] = kEdtInf;
  for (int q = 1; q < n; ++q) {
    float s = 0.f;
    while (true) {
      const float qf = static_cast<float>(q);
      const float vf = static_cast<float>(v[k]);
      s = ((f[q] + qf * qf) - (f[v[k]] + vf * vf)) / (2.f * (qf - vf));
      if (s > z[k])
        break;
      --k;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kEdtInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q))
      ++k;
    const float dq = static_cast<float>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

/**
 * 把一个长 nx*ny 的 2D 数组在**窗口坐标系**里按 (dx,dy) 平移，保持世界系对齐：
 *   new[ix, iy] = old[ix + dx, iy + dy]    （逐轴平移；越界的源视为“新露出”）
 * 新露出的条带用 new_fill 填充。调用方需保证 |dx| < nx 且 |dy| < ny。
 *
 * 这就是“滑动时不用全图重建”的关键：滑动只会让世界系里同一个格子
 * 在窗口内换个下标，搬一下内存（几十 KB）远比重新扫 3D 栅格便宜。
 */
template <typename T>
void shift2DArray(std::vector<T> &v, int nx, int ny, int dx, int dy,
                  T new_fill) {
  if (dx == 0 && dy == 0)
    return;
  const size_t row = static_cast<size_t>(nx) * sizeof(T);

  // 1) y 方向整行搬运
  if (dy > 0) {
    std::memmove(&v[0], &v[static_cast<size_t>(dy) * nx],
                 static_cast<size_t>(ny - dy) * row);
    for (int iy = ny - dy; iy < ny; ++iy)
      std::fill(v.begin() + static_cast<size_t>(iy) * nx,
                v.begin() + static_cast<size_t>(iy + 1) * nx, new_fill);
  } else if (dy < 0) {
    std::memmove(&v[static_cast<size_t>(-dy) * nx], &v[0],
                 static_cast<size_t>(ny + dy) * row);
    for (int iy = 0; iy < -dy; ++iy)
      std::fill(v.begin() + static_cast<size_t>(iy) * nx,
                v.begin() + static_cast<size_t>(iy + 1) * nx, new_fill);
  }

  // 2) x 方向逐行搬运
  if (dx != 0) {
    const int keep = nx - std::abs(dx);
    for (int iy = 0; iy < ny; ++iy) {
      T *p = &v[static_cast<size_t>(iy) * nx];
      if (dx > 0) {
        std::memmove(p, p + dx, static_cast<size_t>(keep) * sizeof(T));
        std::fill(p + keep, p + nx, new_fill);
      } else {
        std::memmove(p - dx, p, static_cast<size_t>(keep) * sizeof(T));
        std::fill(p, p - dx, new_fill);
      }
    }
  }
}
} // namespace

void GridMap::initMap(rclcpp::Node *node) {
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  load_parameter(node_, "grid_map.local_update_range_x",
                 mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y",
                 mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z",
                 mp_.local_update_range_(2), z_size / 2.0);

  load_parameter(node_, "grid_map.obstacles_inflation_z_up",
                 mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down",
                 mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius",
                 mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset",
                 mp_.double_cylinder_offset_, 0.0);
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_,
                 mp_.resolution_);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist",
                 mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist",
                 mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin",
                 mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor",
                 mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);
  /* max_ray_length 的真实语义（见 raycastProcess）：
       length > max_ray_length 的点会被截断到该半径，并把截断点标记为 free。
     所以它就是"每帧能把多远清成空闲"的半径，也是 2D 图里未知面积的直接阀门。
     打开 ray_length_from_local_range 后它与 local_update_range 强制一致，
     改窗口尺寸/局部范围时不会再漏改这一项。 */
  load_parameter(node_, "grid_map.ray_length_from_local_range",
                 mp_.ray_length_from_local_range_, false);
  if (mp_.ray_length_from_local_range_)
    mp_.max_ray_length_ =
        std::max(mp_.local_update_range_(0), mp_.local_update_range_(1));

  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);

  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id",
                 mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);

  /* ---------- 2D 局部感知产物 ----------
   * 注意：代码默认全关（enable_2d_ = false）—— 任何未显式打开 2D 的配置
   * （例如 3D 规划器用的 planner.yaml）行为与改动前完全一致、零额外开销。
   */
  load_parameter(node_, "grid_map.enable_2d", mp_.enable_2d_, false);
  load_parameter(node_, "grid_map.pub_2d_occupancy", mp_.pub_2d_occupancy_,
                 true);
  load_parameter(node_, "grid_map.pub_2d_occupancy_inflate",
                 mp_.pub_2d_occupancy_inflate_, true);
  load_parameter(node_, "grid_map.pub_2d_esdf", mp_.pub_2d_esdf_, true);
  load_parameter(node_, "grid_map.pub_2d_interval", mp_.pub_2d_interval_, 0.1);
  load_parameter(node_, "grid_map.pub_2d_stats_interval",
                 mp_.pub_2d_stats_interval_, 0.0);
  load_parameter(node_, "grid_map.proj_relative_to_sensor",
                 mp_.proj_relative_to_sensor_, false);
  load_parameter(node_, "grid_map.proj_z_min", mp_.proj_z_min_, -0.5);
  load_parameter(node_, "grid_map.proj_z_max", mp_.proj_z_max_, 2.0);
  load_parameter(node_, "grid_map.esdf_unknown_as_occupied",
                 mp_.esdf_unknown_as_occupied_, false);
  load_parameter(node_, "grid_map.esdf_max_dist", mp_.esdf_max_dist_, 3.0);
  load_parameter(node_, "grid_map.esdf_pub_step", mp_.esdf_pub_step_, 2);
  load_parameter(node_, "grid_map.esdf2d_query_en", mp_.esdf2d_query_en_,
                 false);
  load_parameter(node_, "grid_map.verify_2d", mp_.verify_2d_, false);
  load_parameter(node_, "grid_map.force_full_2d", mp_.force_full_2d_, false);
  load_parameter(node_, "grid_map.refresh_full_2d_interval",
                 mp_.refresh_full_2d_interval_, 20);
  load_parameter(node_, "grid_map.topic_2d_occupancy", mp_.topic_2d_occupancy_,
                 string("grid_map/occupancy_2d"));
  load_parameter(node_, "grid_map.topic_2d_occupancy_inflate",
                 mp_.topic_2d_occupancy_inflate_,
                 string("grid_map/occupancy_inflate_2d"));
  load_parameter(node_, "grid_map.topic_2d_esdf", mp_.topic_2d_esdf_,
                 string("grid_map/esdf_2d"));
  if (mp_.esdf_pub_step_ < 1)
    mp_.esdf_pub_step_ = 1;
  if (mp_.esdf_max_dist_ < 0.0)
    mp_.esdf_max_dist_ = 0.0;

  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_,
                 string("lidar"));

  /* 输入话题名：默认值保持改动前写死的相对名，现有配置/launch 行为不变 */
  load_parameter(node_, "grid_map.topic_cloud", mp_.topic_cloud_,
                 string("cloud"));
  load_parameter(node_, "grid_map.topic_pose", mp_.topic_pose_,
                 string("sensor_pose"));
  load_parameter(node_, "grid_map.topic_depth", mp_.topic_depth_,
                 string("depth"));
  load_parameter(node_, "grid_map.topic_body_pose", mp_.topic_body_pose_,
                 string("body_pose"));

  /* ---------- 2D 层 footprint 清除（默认关）---------- */
  load_parameter(node_, "grid_map.footprint_clear_enable",
                 mp_.footprint_clear_enable_, false);
  load_parameter(node_, "grid_map.footprint_length", mp_.footprint_length_,
                 0.0);
  load_parameter(node_, "grid_map.footprint_width", mp_.footprint_width_, 0.0);
  load_parameter(node_, "grid_map.footprint_offset_x", mp_.footprint_offset_x_,
                 0.0);
  load_parameter(node_, "grid_map.footprint_offset_y", mp_.footprint_offset_y_,
                 0.0);
  load_parameter(node_, "grid_map.footprint_yaw_offset_deg",
                 mp_.footprint_yaw_offset_deg_, 0.0);
  load_parameter(node_, "grid_map.footprint_clear_as_unknown",
                 mp_.footprint_clear_as_unknown_, false);
  load_parameter(node_, "grid_map.pub_footprint_viz", mp_.pub_footprint_viz_,
                 true);
  load_parameter(node_, "grid_map.topic_footprint_viz",
                 mp_.topic_footprint_viz_, string("grid_map/footprint"));

  /* ---------- 可视化开销控制 ---------- */
  load_parameter(node_, "grid_map.vis_interval", mp_.vis_interval_, 0.1);
  load_parameter(node_, "grid_map.pub_map_interval", mp_.pub_map_interval_,
                 0.2);
  load_parameter(node_, "grid_map.verify_occ_idx", mp_.verify_occ_idx_, false);
  if (mp_.footprint_clear_enable_) {
    if (mp_.footprint_length_ <= 0.0 || mp_.footprint_width_ <= 0.0) {
      RCLCPP_WARN(node_->get_logger(),
                  "[GridMap] footprint_clear_enable=true，但 length=%.2f "
                  "width=%.2f（需>0）→ 实际不抹任何格，请检查配置",
                  mp_.footprint_length_, mp_.footprint_width_);
    } else {
      RCLCPP_INFO(
          node_->get_logger(),
          "[GridMap] 2D footprint 清除已开：矩形 %.2fx%.2f m（机体系）| "
          "offset (%.2f, %.2f) yaw_offset %.1f° | 抹成 %s",
          mp_.footprint_length_, mp_.footprint_width_, mp_.footprint_offset_x_,
          mp_.footprint_offset_y_, mp_.footprint_yaw_offset_deg_,
          mp_.footprint_clear_as_unknown_ ? "未知(-1)" : "空闲(0)");
    }
  }

  /* ---------- 几何自洽性：窗口 / 写占据范围 / 清空半径 ----------
     这三个数决定 2D 图里"空闲"最多能占多少，是最容易配歪的一组：
       · local_update_range = 能写占据的范围（看到多远就记多远）
       · max_ray_length     = 能清成空闲的半径（raycastProcess 的截断）
     清空半径小于写占据范围时，中间会留一圈"能写障碍、却永远不清空"的带子，
     2D 图上就是一大片未知（规划器看到的是"未知里夹着远处障碍"）。 */
  {
    const double clear_r = mp_.max_ray_length_;
    const double lr_min =
        std::min(mp_.local_update_range_(0), mp_.local_update_range_(1));
    const double win_area = std::max(x_size * y_size, 1e-9);
    /* 单帧最多清 π r²："从当前位置靠射线能清出的面积上限"。
       机器人开过去还能继续扩大，所以这只是个量级参考。 */
    const double clear_frac =
        std::min(std::acos(-1.0) * clear_r * clear_r / win_area, 1.0);
    RCLCPP_INFO(
        node_->get_logger(),
        "[GridMap] 几何: 窗口 %.1fx%.1f m (%.1f m^2) | 写占据 ±%.1f/±%.1f m | "
        "清空半径 %.2f m%s | 单帧最多清 %.0f%% 窗口面积",
        x_size, y_size, win_area, mp_.local_update_range_(0),
        mp_.local_update_range_(1), clear_r,
        mp_.ray_length_from_local_range_ ? " (=local_update_range)" : "",
        clear_frac * 100.0);
    if (clear_r < lr_min - 1e-9)
      RCLCPP_WARN(
          node_->get_logger(),
          "[GridMap] 清空半径 %.2f m < 写占据范围 %.2f m：%.2f~%.2f m 那一圈"
          "能写障碍却不清空 → 2D 图会有一大片未知。只为省 CPU 可忽略；"
          "否则把 grid_map.ray_length_from_local_range 置 true（或把 "
          "max_ray_length 提到 %.2f）",
          clear_r, lr_min, clear_r, lr_min, lr_min);
  }

  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  mp_.lidar_extrinsic_ << 1.0, 0.0, 0.0, -0.01100, 0.0, 1.0, 0.0, -0.02329, 0.0,
      0.0, 1.0, 0.04412, 0.0, 0.0, 0.0, 1.00000;

  mp_.depth_extrinsic_ << 0.0, 0.707107, 0.707107, -0.15170, -1.0, 0.000000,
      0.000000, 0.00000, 0.0, -0.707107, 0.707107, 0.07510, 0.0, 0.000000,
      0.000000, 1.00000;

  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth") {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
        mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ =
      Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ =
      std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ *
                                             mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ =
      mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size =
      mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ =
      vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  /* 占据/膨胀体素索引（可视化发布用，见头文件注释）*/
  md_.occ_idx_flag_.assign(buffer_size, 0);
  md_.infl_idx_flag_.assign(buffer_size, 0);
  md_.occ_idx_list_.clear();
  md_.infl_idx_list_.clear();
  md_.occ_idx_list_.reserve(4096);
  md_.infl_idx_list_.reserve(8192);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* ---------- 2D 局部层缓冲 ----------
   * x-y 覆盖整个滑动窗口（与 3D 同分辨率），但**按全局索引连续排列**
   * （i2d = ix + iy * nx，与 nav_msgs/OccupancyGrid 的 data 布局一致），
   * 这样它是一个世界系下真正的矩形，EDT 不会跨边界泄漏。
   */
  {
    const int nx = mp_.map_voxel_num_(0);
    const int ny = mp_.map_voxel_num_(1);
    const int n2d = nx * ny;
    md_.occ2d_.assign(n2d, static_cast<int8_t>(-1));
    md_.occ2d_inflate_.assign(n2d, static_cast<int8_t>(-1));
    md_.esdf2d_.assign(n2d, -1.f);
    md_.occ2d_dirty_.assign(n2d, 0);
    md_.occ2d_dirty_list_.clear();
    md_.occ2d_dirty_list_.reserve(n2d);
    md_.occ2d_last_build_.assign(n2d, -1);
    md_.occ2d_build_cnt_ = 0;
    md_.has_2d_ = false;
    md_.occ2d_band_valid_ = false;
    md_.occ2d_kz0_ = 0;
    md_.occ2d_kz1_ = -1;
    md_.esdf2d_stale_ = true;
    md_.proj_z_lo_ = mp_.proj_z_min_;
    md_.proj_z_hi_ = mp_.proj_z_max_;
    md_.occ2d_min_idx_ = mp_.map_bound_min_idx_;

    const int nline = std::max(nx, ny);
    md_.edt_f_.assign(n2d, kEdtInf);
    md_.edt_sq_.assign(n2d, kEdtInf);
    md_.edt_line_in_.assign(nline, kEdtInf);
    md_.edt_line_out_.assign(nline, kEdtInf);
    md_.edt_z_.assign(nline + 2, 0.f);
    md_.edt_v_.assign(nline + 1, 0);
  }

  /* init callback */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);

  if (mp_.sensor_type_ == "depth") {
    depth_sub_ = std::make_shared<
        message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<
        message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, mp_.topic_depth_, rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, mp_.topic_pose_,
                               rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(
        new message_filters::Synchronizer<SyncPolicyImagePose>(
            SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(std::bind(&GridMap::depthPoseCallback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2));
  } else if (mp_.sensor_type_ == "lidar") {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        mp_.topic_pose_, rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        mp_.topic_cloud_, rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      mp_.topic_body_pose_, rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this,
                std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GridMap::updateOccupancyCallback, this));
  /* 可视化频率参数化：TF/bbox/footprint 需要 10~20Hz，点云不需要（见
   * pub_map_interval_）*/
  const int vis_ms = static_cast<int>(mp_.vis_interval_ * 1000.0);
  vis_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(vis_ms > 0 ? vis_ms : 50),
      std::bind(&GridMap::visCallback, this));

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "grid_map/occupancy", rclcpp::SensorDataQoS());
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "grid_map/occupancy_inflate", rclcpp::SensorDataQoS());
  sliding_map_bbox_pub_ =
      node_->create_publisher<visualization_msgs::msg::Marker>(
          "grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "grid_map/sensor_pose_extrinsic", 10);

  /* ---------- 2D 局部产物发布器（enable_2d_ 为 false 时一个都不建）----------
   */
  if (mp_.enable_2d_) {
    if (mp_.pub_footprint_viz_) {
      footprint_viz_pub_ =
          node_->create_publisher<visualization_msgs::msg::MarkerArray>(
              mp_.topic_footprint_viz_, rclcpp::QoS(1));
    }
    if (mp_.pub_2d_occupancy_) {
      occ2d_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
          mp_.topic_2d_occupancy_, rclcpp::QoS(1));
    }
    if (mp_.pub_2d_occupancy_inflate_) {
      occ2d_inf_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
          mp_.topic_2d_occupancy_inflate_, rclcpp::QoS(1));
    }
    if (mp_.pub_2d_esdf_) {
      esdf2d_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          mp_.topic_2d_esdf_, rclcpp::QoS(1));
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "[GridMap] 2D 局部层已开启 | 高度带 [%.2f, %.2f] m（%s）"
        " | occupancy=%d inflate=%d esdf=%d | 节流 %.2fs | 窗口 %dx%d 格",
        mp_.proj_z_min_, mp_.proj_z_max_,
        mp_.proj_relative_to_sensor_ ? "相对传感器" : "map 系绝对",
        static_cast<int>(mp_.pub_2d_occupancy_),
        static_cast<int>(mp_.pub_2d_occupancy_inflate_),
        static_cast<int>(mp_.pub_2d_esdf_), mp_.pub_2d_interval_,
        mp_.map_voxel_num_(0), mp_.map_voxel_num_(1));
    if (!mp_.pub_2d_occupancy_ && !mp_.pub_2d_occupancy_inflate_ &&
        !mp_.pub_2d_esdf_) {
      RCLCPP_WARN(node_->get_logger(),
                  "[GridMap] enable_2d 为 true，但三个产物全关，等于白算"
                  "（仍保留同进程查询接口）");
    }
    RCLCPP_INFO(
        node_->get_logger(), "[GridMap] 2D 增量重建已启用 | ESDF %s | 自检 %s",
        mp_.esdf2d_query_en_ ? "每帧重算（esdf2d_query_en=true）"
                             : "仅在有订阅者/查询开关打开时算（省 CPU）",
        mp_.verify_2d_ ? "开（每帧暴力全扫对比）" : "关");
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[GridMap] 2D 局部层未开启（grid_map.enable_2d=false）");
  }

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex() {
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ =
      mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ =
      mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ =
      (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) *
      mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets() {
  const double double_radius = std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  const int inf_step_z_up =
      ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down =
      ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  md_.inflate_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y) {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() >= double_radius)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

void GridMap::resetMap() {
  resetAllMapData();

  /// 复位与“上一张地图”绑定的状态
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.proj_points_cnt = 0;
  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.ray_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();
  md_.sliding_map_frame_pos_.setZero();

  /// 让滑动地图原点回到初始位置，下一帧 sensor_pose 会重新居中
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  if (node_ != nullptr) {
    RCLCPP_WARN(node_->get_logger(),
                "[GridMap] occupancy grid reset (map switched); waiting for a "
                "fresh sensor_pose");
  }
}

void GridMap::resetAllMapData() {
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(),
            mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(),
            md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(),
            md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);

  /* 整张地图作废 → 占据/膨胀索引也一起清空 */
  md_.occ_idx_list_.clear();
  md_.infl_idx_list_.clear();
  std::fill(md_.occ_idx_flag_.begin(), md_.occ_idx_flag_.end(), 0);
  std::fill(md_.infl_idx_flag_.begin(), md_.infl_idx_flag_.end(), 0);

  /* 2D 层属于“上一张地图/上一段观测”的数据，一起作废；
     否则换图后 getDistance2D() 会返回旧地图的距离场。 */
  if (!md_.occ2d_.empty()) {
    std::fill(md_.occ2d_.begin(), md_.occ2d_.end(), static_cast<int8_t>(-1));
    std::fill(md_.occ2d_inflate_.begin(), md_.occ2d_inflate_.end(),
              static_cast<int8_t>(-1));
    std::fill(md_.esdf2d_.begin(), md_.esdf2d_.end(), -1.f);
    std::fill(md_.occ2d_dirty_.begin(), md_.occ2d_dirty_.end(), 0);
    md_.occ2d_dirty_list_.clear();
    md_.has_2d_ = false;
    md_.occ2d_band_valid_ = false; // 下一次 build2DLayer() 走全量重扫
    md_.esdf2d_stale_ = true;
  }
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i &id_g) const {
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) /
            mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i) {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

void GridMap::updateInflationLayer(const Eigen::Vector3i &id, int delta,
                                   const vector<Eigen::Vector3i> &offsets,
                                   std::vector<int> &cnt_buffer,
                                   std::vector<char> &flag_buffer,
                                   const std::vector<char> *ignore_mask) {
  for (const auto &offset : offsets) {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    const char old_flag = flag_buffer[addr];
    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;

    /* 膨胀体素索引：只在 0↔1 翻转时维护 */
    if (flag_buffer[addr] != old_flag) {
      if (flag_buffer[addr])
        idxNote(md_.infl_idx_list_, md_.infl_idx_flag_, addr);
      else
        idxDrop(md_.infl_idx_flag_, addr);
    }

    // 2D 膨胀产物只关心“该格是否为膨胀格”，所以只在 0↔1 翻转时标脏
    if (mp_.enable_2d_ && flag_buffer[addr] != old_flag)
      mark2DColumnDirty(inf_id(0), inf_id(1), inf_id(2));
  }
}

void GridMap::updateInflation(const Eigen::Vector3i &id, int delta,
                              const std::vector<char> *ignore_mask) {
  updateInflationLayer(id, delta, md_.inflate_offsets_,
                       md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i &id,
                                   double new_log_odds) {
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const double old_log_odds = md_.occupancy_buffer_[addr];
  const bool was_occ = old_log_odds > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);

  /* 占据体素索引：仅“转入占据/退出占据”时维护一次 */
  if (was_occ != now_occ) {
    if (now_occ)
      idxNote(md_.occ_idx_list_, md_.occ_idx_flag_, addr);
    else
      idxDrop(md_.occ_idx_flag_, addr);
  }

  /* 2D 层：只有“未知/空闲/占据”三分类真的变了才需要重扫该列。
     注：已知存在极少数列会漏标（自检可见），因此 build2DLayer() 还会每隔
     refresh_full_2d_interval_ 次构建做一次全量刷新兜底。 */
  if (mp_.enable_2d_ && occ2dClass(old_log_odds) != occ2dClass(new_log_odds))
    mark2DColumnDirty(id(0), id(1), id(2));
}

void GridMap::resetCellByAddress(int addr) {
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  const double old_log_odds = md_.occupancy_buffer_[addr];
  if (old_log_odds > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
  idxDrop(md_.occ_idx_flag_, addr); // 不再占据 → 退出占据索引

  // 归零成“未知”：原来不是未知的话，该 2D 列要重扫
  if (mp_.enable_2d_ && occ2dClass(old_log_odds) != 0)
    mark2DColumnDirty(id_g(0), id_g(1), id_g(2));
}

void GridMap::resetCellByAddressForSliding(
    int addr, const std::vector<char> &clear_mask) {
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1, &clear_mask);
}

void GridMap::updateSlidingMap(const Eigen::Vector3d &center) {
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any()) {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size =
      mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i &id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr]) {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim) {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0) {
      for (int k = 0; k < shift; ++k) {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b) {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    } else {
      for (int k = 0; k < -shift; ++k) {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b) {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs) {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
    /* 滑动清图是直接写缓冲（没走 applyOccupancyUpdate/resetCellByAddress），
       所以这两个索引必须在这里同步清位，否则会留下指向空体的坟墓。 */
    idxDrop(md_.occ_idx_flag_, addr);
    idxDrop(md_.infl_idx_flag_, addr);
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::resetBuffer() {
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y) {
      for (int z = min_id(2); z <= max_id(2); ++z) {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1) {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage() {
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_) {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_;
       v += mp_.skip_pixel_) {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
         u += mp_.skip_pixel_) {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0) {
        depth = mp_.max_ray_length_ + 0.1;
      } else if (depth < mp_.depth_filter_mindist_) {
        continue;
      } else if (depth > mp_.depth_filter_maxdist_) {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

void GridMap::raycastProcess() {
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w)) {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_) {
        pt_w =
            (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    } else {
      length = (pt_w - md_.ray_pos_).norm();

      if (length > mp_.max_ray_length_) {
        pt_w =
            (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      } else {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX) {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
        continue;
      } else {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt)) {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX) {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
          break;
        } else {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty()) {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >=
                md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns]
            ? mp_.prob_hit_log_
            : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 &&
        md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
      continue;
    } else if (log_odds_update <= 0 &&
               md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) &&
                    idx(1) >= min_id(1) && idx(1) <= max_id(1) &&
                    idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local) {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update,
                          mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt,
                                          const Eigen::Vector3d &ray_pos) {
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback() {

  /* 3D 占据/膨胀点云是"全图 → 点"的输出，单独限频；
     TF/bbox/footprint 这些便宜的每 tick 都发。 */
  bool pub_map_now = true;
  const auto now = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point last_map;
  static bool last_map_valid = false;
  if (last_map_valid && mp_.pub_map_interval_ > 0.0 &&
      std::chrono::duration<double>(now - last_map).count() <
          mp_.pub_map_interval_)
    pub_map_now = false;
  if (pub_map_now) {
    last_map = now;
    last_map_valid = true;
  }

  if (pub_map_now) {
    publishMap();
    publishMapInflate(true);
  }
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
  publish2D();
  publishFootprintViz();

  /* 收紧墓碑：publishMap/publishMapInflate 只在"有人订阅"时才压缩，
     长期不开 RViz 的话列表会被坟墓擑大（只影响内存与遍历长度）。 */
  auto compact_idx = [](std::vector<int> &lst, std::vector<char> &flag,
                        const auto &live) {
    size_t w = 0;
    for (size_t i = 0; i < lst.size(); ++i) {
      const int a = lst[i];
      if (flag[a] && live(a)) {
        lst[w++] = a;
      } else {
        flag[a] = 0;
      }
    }
    lst.resize(w);
  };
  if (md_.occ_idx_list_.size() > 20000)
    compact_idx(md_.occ_idx_list_, md_.occ_idx_flag_, [&](int a) {
      return md_.occupancy_buffer_[a] >= mp_.min_occupancy_log_;
    });
  if (md_.infl_idx_list_.size() > 40000)
    compact_idx(md_.infl_idx_list_, md_.infl_idx_flag_,
                [&](int a) { return md_.occupancy_buffer_inflate_[a] != 0; });

  if (mp_.verify_occ_idx_)
    verifyOccIdx();
}

void GridMap::updateOccupancyCallback() {
  if (!md_.occ_need_update_)
    return;

  /* 每帧耗时统计：3D 融合通常是绝对大头，2D 层很小 —— 打印出来才好定位
     （show_occ_time 打开时，每处理一帧打一行）。 */
  const auto t_frame = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point last_frame;
  static bool last_frame_valid = false;
  double frame_gap_ms = 0.0;
  if (last_frame_valid)
    frame_gap_ms =
        std::chrono::duration<double, std::milli>(t_frame - last_frame).count();
  last_frame = t_frame;
  last_frame_valid = true;

  if (!md_.use_cloud_update_)
    projectDepthImage();

  const auto t_3d = std::chrono::steady_clock::now();
  raycastProcess();
  const double ms_3d = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t_3d)
                           .count();

  /* 3D 融合结束 → 更新 2D 局部层（高度带投影 + 可选 ESDF）*/
  double ms_2d = 0.0;
  if (mp_.enable_2d_) {
    const auto t0 = std::chrono::steady_clock::now();
    build2DLayer();
    ms_2d = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0)
                .count();
  }

  if (mp_.show_occ_time_) {
    const double ms_cb = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t_frame)
                             .count();
    const double hz = frame_gap_ms > 0.5 ? 1000.0 / frame_gap_ms : 0.0;
    RCLCPP_INFO(
        node_->get_logger(),
        "[GridMap] 帧耗时: 总 %.1f ms = （点云回调 %.1f + 3D融合 %.1f + 2D "
        "%.1f）ms | 点数 %d | 帧间隔 %.1f ms（%.2f Hz）",
        ms_cb, md_.t_cloud_ms_, ms_3d, ms_2d, md_.proj_points_cnt, frame_gap_ms,
        hz);
  }

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
}

void GridMap::depthPoseCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &img,
    const nav_msgs::msg::Odometry::ConstSharedPtr &pose) {
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    (cv_ptr->image)
        .convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " <<
  // md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y,
                           sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y,
                          sensor_pose.position.z);
  if (mp_.need_extrinsic_) {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id = pose->child_frame_id.empty()
                                      ? "sensor_extrinsic"
                                      : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_)) {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  } else {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg) {
  if (mp_.sensor_type_ != "lidar")
    return;

  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y,
                           sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y,
                          sensor_pose.position.z);
  if (mp_.need_extrinsic_) {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) ||
      !std::isfinite(ray_pos.z()))
    return;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  updateSlidingMap(md_.ray_pos_);
}

void GridMap::slidingMapFrameCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr &pose) {
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
}

void GridMap::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img) {
  if (mp_.sensor_type_ != "lidar")
    return;

  const auto t_cb = std::chrono::steady_clock::now();
  // 退出前把本次点云回调耗时记下来（帧耗时日志要用）
  auto note_ms = [&]() {
    md_.t_cloud_ms_ = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t_cb)
                          .count();
  };

  if (!md_.has_ray_pose_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[GridMap] no sensor_pose received for lidar cloud update");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) ||
      !std::isfinite(ray_pos.z()))
    return;

  updateSlidingMap(ray_pos);

  md_.proj_points_cnt = 0;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_) {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    } else {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range = fabs(devi(0)) <= mp_.local_update_range_(0) &&
                                fabs(devi(1)) <= mp_.local_update_range_(1) &&
                                fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    md_.proj_points_cnt++;
  }

  if (md_.proj_points_cnt == 0) {
    note_ms();
    return;
  }

  note_ms();
  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

/* ============================== 2D 局部层 ==============================
 * 把 3D 占据栅格按高度带投影成 2D，并按需算 2D ESDF。
 *
 * 几个刻意的设计选择：
 *  1) 所有 2D 产物按**全局索引连续排列**（i2d = ix + iy * nx），不做环形 wrap。
 *     3D 缓冲为了滑动查表是环形下标的，所以这里是 "读 3D（自动 wrap）→ 写
 * 2D（连续）"， 于是 2D 层在世界系下是一个真正的矩形，EDT
 * 不会跨越边界算出假的近距离。 这个布局与 nav_msgs/OccupancyGrid 的 data[x +
 * y*width] 完全一致，发布时可以整段拷贝。 2) 每格三种状态：占据 100 / 空闲 0 /
 * 未知 -1。 高度带内**只要有一个体素是占据**就算占据（与 3D 的 inflate
 * 语义一致）。 3) ESDF 默认对未知区域取乐观解释（当自由），与 3D 层一致；
 *     需要保守规划时把 esdf_unknown_as_occupied 打开。
 */

void GridMap::build2DLayer() {
  if (!mp_.enable_2d_)
    return;

  /* footprint 位姿缓存：fp_prev_ 保留上一帧的（用于下面的“已离开区域”重扫），
     fp_cur_ 换成当前位姿（后面 mask / 自检 / 可视化都用它）。 */
  updateFootprintPoseCache();

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);
  const int n2d = nx * ny;

  /* ---------- 1) 高度带 → 全局 z 体素索引区间 ---------- */
  double z_lo = mp_.proj_z_min_;
  double z_hi = mp_.proj_z_max_;
  if (mp_.proj_relative_to_sensor_ && md_.has_ray_pose_) {
    z_lo += md_.ray_pos_(2);
    z_hi += md_.ray_pos_(2);
  }
  if (z_lo > z_hi)
    std::swap(z_lo, z_hi);
  int kz0 = static_cast<int>(std::floor(z_lo * mp_.resolution_inv_));
  int kz1 = static_cast<int>(std::floor(z_hi * mp_.resolution_inv_));
  kz0 = std::max(kz0, mp_.map_bound_min_idx_(2));
  kz1 = std::min(kz1, mp_.map_bound_max_idx_(2));
  md_.proj_z_lo_ = z_lo;
  md_.proj_z_hi_ = z_hi;
  /* ⚠ 这里**不能**先把锚点改成当前窗口原点：下面要用 md_.occ2d_min_idx_（上一帧
     构建时的锚点）算 dx/dy 来决定是否需要平移 2D 数组。锚点只在函数末尾统一更新
     （见 md_.occ2d_min_idx_ = anchor）。早期版本在这里多写了一次，导致 dx 恒为
     0、 平移永不执行 → 每次滑窗后整张 2D 图与锚点错位。 */
  const bool need_infl = mp_.pub_2d_occupancy_inflate_;
  const Eigen::Vector3i anchor = mp_.map_bound_min_idx_;

  /* ---------- 2) 决定重建范围 ----------
   * full：整窗重扫（首次构建 / 换图 / 高度带变了 / 窗口大跳 / 尺寸不匹配）
   * 平移 + 增量：窗口只平移时，先把 2D
   * 数据在窗口坐标系里原地平移以保持世界系对齐， 然后只重扫「本帧 3D
   * 真正改变分类的列」+「新露出的条带」。
   */
  bool full = mp_.force_full_2d_ || !md_.has_2d_ || !md_.occ2d_band_valid_ ||
              md_.occ2d_.size() != static_cast<size_t>(n2d) ||
              md_.occ2d_kz0_ != kz0 || md_.occ2d_kz1_ != kz1;

  /* 周期性全量刷新：增量路径存在极少数“漏标脏”的情形（自检可见），
     定期整窗重扫一次把它彻底扫掉，把误差寿命限制在 refresh_full_2d_interval_
     次构建以内。 */
  if (!full && mp_.refresh_full_2d_interval_ > 0 &&
      ++md_.occ2d_since_full_ >= mp_.refresh_full_2d_interval_)
    full = true;
  if (full)
    md_.occ2d_since_full_ = 0;

  int dx = 0, dy = 0;
  if (!full) {
    dx = anchor(0) - md_.occ2d_min_idx_(0);
    dy = anchor(1) - md_.occ2d_min_idx_(1);
    if (std::abs(dx) >= nx || std::abs(dy) >= ny)
      full = true; // 窗口整体换过（位姿大跳 / 换图）
    /* z 方向滑动（anchor(2) 变化）既不需要平移、也不需要整窗重扫：
       2D 数组只按 (x,y) 索引，z 仅通过高度带 kz0/kz1 影响结果；而 z 滑动
       清掉的 slab 只可能是 new_min 以下 / new_max 以上的平面，于是：
         · 带被窗口边界裁剪时（kz0==new_min 或 kz1==new_max），
           kz 相对上一帧必然变化 → 上面的 full 条件已经命中；
         · 带严格落在窗口内时，被清的平面完全在带外 → 不影响任何列的带内内容。
       因此这里不再有 dz!=0 ⇒ full 的保守分支（已用自检验证）。 */
  }

  /* 脏列登记（增量路径用）。放在 if/else 之前：footprint 的区域修正在
     “窗口有无平移”两种情况下都要用。 */
  auto add_dirty = [&](int i2d) {
    if (!md_.occ2d_dirty_[i2d]) {
      md_.occ2d_dirty_[i2d] = 1;
      md_.occ2d_dirty_list_.push_back(i2d);
    }
  };

  if (full) {
    std::fill(md_.occ2d_dirty_.begin(), md_.occ2d_dirty_.end(), 0);
    md_.occ2d_dirty_list_.clear();
  } else if (dx != 0 || dy != 0) {
    /* 脏列下标要跟着数据一起换坐标系：旧下标先作废，再按新下标重建，
       落到窗口外的（滑出去的列）直接丢弃。逐轴换算，与 shift2DArray 同一语义。
     */
    std::fill(md_.occ2d_dirty_.begin(), md_.occ2d_dirty_.end(), 0);
    size_t w = 0;
    for (int i2d : md_.occ2d_dirty_list_) {
      const int ix = i2d % nx, iy = i2d / nx;
      const int sx = ix - dx, sy = iy - dy;
      if (sx < 0 || sx >= nx || sy < 0 || sy >= ny)
        continue; // 该列已滑出窗口
      const int k = sx + sy * nx;
      md_.occ2d_dirty_list_[w++] = k;
      md_.occ2d_dirty_[k] = 1;
    }
    md_.occ2d_dirty_list_.resize(w);

    // 数据数组平移（新露出的条带先填“未知”，随后会被标脏重扫）
    shift2DArray(md_.occ2d_, nx, ny, dx, dy, static_cast<int8_t>(-1));
    shift2DArray(md_.occ2d_inflate_, nx, ny, dx, dy, static_cast<int8_t>(-1));

    // 新露出的条带没有任何历史数据 → 全部标脏
    if (dy > 0) {
      for (int iy = ny - dy; iy < ny; ++iy)
        for (int ix = 0; ix < nx; ++ix)
          add_dirty(ix + iy * nx);
    } else if (dy < 0) {
      for (int iy = 0; iy < -dy; ++iy)
        for (int ix = 0; ix < nx; ++ix)
          add_dirty(ix + iy * nx);
    }
    if (dx > 0) {
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = nx - dx; ix < nx; ++ix)
          add_dirty(ix + iy * nx);
    } else if (dx < 0) {
      for (int iy = 0; iy < ny; ++iy)
        for (int ix = 0; ix < -dx; ++ix)
          add_dirty(ix + iy * nx);
    }
  }

  if (!full) {
    /* ---------- footprint 离开区域的重扫修正 ----------
       机器人移动后，**上一帧**被 footprint mask
       抹成“空闲”的格子如果本帧不再被抹， 它们的真实投影值就没人重扫 →
       会永久留下假空闲（自检能抓到：期望 100 实际 0，
       且提示“本帧未重扫”）。这里把上一帧 footprint
       覆盖的列全部标脏，让它们重扫回 真实投影；本帧仍在 footprint
       内的格子随后会被 applyFootprintClear() 再抹一遍，
       所以多标一点没有副作用。代价只有矩形内几十格。
       ⚠ 下标必须用**本帧** anchor 换算：数组此时已经平移到新窗口坐标系了。 */
    const FootprintPose &fprev = md_.fp_prev_;
    if (fprev.valid && mp_.footprint_length_ > 0.0 &&
        mp_.footprint_width_ > 0.0) {
      const double half = footprintCircumradius();
      const auto to_ix = [&](double w, int a) {
        return static_cast<int>(std::floor(w * mp_.resolution_inv_)) - a;
      };
      const int fx0 = std::max(0, to_ix(fprev.cx - half, anchor(0)));
      const int fx1 = std::min(nx - 1, to_ix(fprev.cx + half, anchor(0)));
      const int fy0 = std::max(0, to_ix(fprev.cy - half, anchor(1)));
      const int fy1 = std::min(ny - 1, to_ix(fprev.cy + half, anchor(1)));
      for (int iy = fy0; iy <= fy1; ++iy) {
        const double wy = (anchor(1) + iy + 0.5) * mp_.resolution_;
        for (int ix = fx0; ix <= fx1; ++ix) {
          const double wx = (anchor(0) + ix + 0.5) * mp_.resolution_;
          if (inFootprintRect(fprev, wx, wy))
            add_dirty(ix + iy * nx);
        }
      }
    }
  }

  /* ---------- 3) 重扫：全量所有列，或只扫脏列 ---------- */ /// toAddress
                                                              /// 的布局是
                                                              /// lx*(ny*nz) +
                                                              /// ly*nz +
  /// lz，这里把不随格子变化的量提到循环外
  std::vector<int> kz_local;
  kz_local.reserve(static_cast<size_t>(kz1 - kz0 + 1));
  for (int kz = kz0; kz <= kz1; ++kz)
    kz_local.push_back(getLocalIndex(kz, 2));
  const int stride_x = ny * nz;

  auto rebuild_column = [&](int i2d) {
    const int ix = i2d % nx;
    const int iy = i2d / nx;
    const int base = getLocalIndex(anchor(0) + ix, 0) * stride_x +
                     getLocalIndex(anchor(1) + iy, 1) * nz;
    bool has_occ = false, has_free = false, has_infl = false;
    for (size_t k = 0; k < kz_local.size(); ++k) {
      const int adr = base + kz_local[k];
      const double log_odds = md_.occupancy_buffer_[adr];
      if (log_odds > mp_.min_occupancy_log_) {
        has_occ = true;
      } else if (log_odds >= mp_.clamp_min_log_) {
        has_free = true;
      }
      if (need_infl && md_.occupancy_buffer_inflate_[adr] != 0)
        has_infl = true;
      // 早退：结果的两种情况（有无占据 /
      // 有无非占据）都确定了，后面的层不影响结果
      if (has_occ && (has_free || (need_infl && has_infl)))
        break;
    }
    md_.occ2d_[i2d] =
        has_occ ? static_cast<int8_t>(100)
                : (has_free ? static_cast<int8_t>(0) : static_cast<int8_t>(-1));
    if (need_infl)
      md_.occ2d_inflate_[i2d] = has_infl ? static_cast<int8_t>(100)
                                         : (has_free ? static_cast<int8_t>(0)
                                                     : static_cast<int8_t>(-1));
    md_.occ2d_last_build_[i2d] = md_.occ2d_build_cnt_;
  };

  int rebuilt = 0;
  ++md_.occ2d_build_cnt_;
  if (full) {
    for (int i2d = 0; i2d < n2d; ++i2d)
      rebuild_column(i2d);
    rebuilt = n2d;
    md_.esdf2d_stale_ = true;
  } else {
    rebuilt = static_cast<int>(md_.occ2d_dirty_list_.size());
    for (int i2d : md_.occ2d_dirty_list_) {
      md_.occ2d_dirty_[i2d] = 0;
      rebuild_column(i2d);
    }
    md_.occ2d_dirty_list_.clear();
    if (rebuilt > 0)
      md_.esdf2d_stale_ = true;
  }

  /// 固定本帧 2D 数据对应的窗口原点（发布/查询时都用它，避免与滑动地图错位）
  md_.occ2d_min_idx_ = anchor;
  md_.occ2d_kz0_ = kz0;
  md_.occ2d_kz1_ = kz1;
  md_.occ2d_band_valid_ = true;
  md_.has_2d_ = true;

  /* ---------- 4) footprint 清除 ----------
     放在每帧构建的**最后**：增量路径下被重扫的列也会在这里重新抹一遍，于是
     “机器人脚下永远不是障碍”是个几何保证，与自车点云从哪来无关。
     必须在 verify2DLayer 之前，否则自检会看到未经抹除的期望值。 */
  applyFootprintClear();

  if (mp_.show_occ_time_) {
    static int stat_cnt = 0;
    if (++stat_cnt % 100 == 0)
      RCLCPP_INFO(node_->get_logger(),
                  "[GridMap] 2D 增量重建：本帧 %d / %d 列（%.1f%%）", rebuilt,
                  n2d, 100.0 * rebuilt / static_cast<double>(std::max(1, n2d)));
  }

  /// 几何参数体检（内部按 pub_2d_stats_interval_ 自行节流，<= 0 直接返回）
  maybeLog2DStats();

  if (mp_.verify_2d_)
    verify2DLayer(dx, dy, full, rebuilt);
}

void GridMap::maybeLog2DStats() {
  if (mp_.pub_2d_stats_interval_ <= 0.0 || !md_.has_2d_ || md_.occ2d_.empty())
    return;

  const double now_s = std::chrono::duration<double>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
  if (md_.t_2d_stats_s_ > 0.0 &&
      now_s - md_.t_2d_stats_s_ < mp_.pub_2d_stats_interval_)
    return;
  md_.t_2d_stats_s_ = now_s;

  size_t n_occ = 0, n_free = 0, n_unk = 0;
  for (const int8_t v : md_.occ2d_) {
    if (v == 100)
      ++n_occ;
    else if (v == 0)
      ++n_free;
    else
      ++n_unk;
  }
  const double n = static_cast<double>(md_.occ2d_.size());
  if (n <= 0.0)
    return;
  const double pct = 100.0 / n;
  const double cell_area = mp_.resolution_ * mp_.resolution_;
  RCLCPP_INFO(node_->get_logger(),
              "[GridMap] 2D 图: 空闲 %zu (%.1f%%, %.1f m^2) | 未知 %zu "
              "(%.1f%%, %.1f m^2) | 占据 %zu (%.1f%%, %.1f m^2) | 清空半径 "
              "%.2f m | 窗口 %.1fx%.1f m",
              n_free, n_free * pct, n_free * cell_area, n_unk, n_unk * pct,
              n_unk * cell_area, n_occ, n_occ * pct, n_occ * cell_area,
              mp_.max_ray_length_, mp_.map_voxel_num_(0) * mp_.resolution_,
              mp_.map_voxel_num_(1) * mp_.resolution_);
}

FootprintPose GridMap::computeFootprintPose() const {
  FootprintPose fp;
  if (!mp_.footprint_clear_enable_ || !md_.has_ray_pose_)
    return fp; // valid = false

  /* 机器人朝向：取传感器位姿的 yaw（纯俯仰安装时 yaw 仍等于机体朝向；
     机体与雷达朝向不一致时用 footprint_yaw_offset_deg 修正）。 */
  const Eigen::Quaterniond &q = md_.ray_q_;
  double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  yaw += mp_.footprint_yaw_offset_deg_ * M_PI / 180.0;
  fp.c = std::cos(yaw);
  fp.s = std::sin(yaw);
  // 矩形中心 = 传感器位置 + 机体系偏移（offset_x 沿机体前向，offset_y
  // 沿机体左向）
  fp.cx = md_.ray_pos_.x() + fp.c * mp_.footprint_offset_x_ -
          fp.s * mp_.footprint_offset_y_;
  fp.cy = md_.ray_pos_.y() + fp.s * mp_.footprint_offset_x_ +
          fp.c * mp_.footprint_offset_y_;
  fp.valid = true;
  return fp;
}

void GridMap::updateFootprintPoseCache() {
  md_.fp_prev_ = md_.fp_cur_;
  md_.fp_cur_ = computeFootprintPose();
}

double GridMap::footprintCircumradius() const {
  const double m = 0.5 * mp_.resolution_;
  return 0.5 * std::hypot(mp_.footprint_length_ + 2.0 * m,
                          mp_.footprint_width_ + 2.0 * m);
}

void GridMap::applyFootprintClear() {
  if (!mp_.enable_2d_ || !md_.has_2d_ || mp_.footprint_length_ <= 0.0 ||
      mp_.footprint_width_ <= 0.0 || !md_.fp_cur_.valid)
    return;

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const Eigen::Vector3i &anchor = md_.occ2d_min_idx_;

  /* 只需遍历 footprint 覆盖的格子。
     ⚠ 包围盒必须用**外扩后矩形**的外接圆半径：判定用的是"格与矩形相交"
     （矩形外扩半格），斜着转（yaw≠0）时外扩矩形的 x/y 投影会超出
     0.5*hypot(L,W)，用后者会漏掉边缘几格没抹（自检能抓到，实测过）。 */
  const double half_diag = footprintCircumradius();
  const FootprintPose &fp = md_.fp_cur_;
  const int ix0 = std::max(0, static_cast<int>(std::floor(
                                  (fp.cx - half_diag) * mp_.resolution_inv_)) -
                                  anchor(0));
  const int ix1 = std::min(
      nx - 1,
      static_cast<int>(std::ceil((fp.cx + half_diag) * mp_.resolution_inv_)) -
          anchor(0));
  const int iy0 = std::max(0, static_cast<int>(std::floor(
                                  (fp.cy - half_diag) * mp_.resolution_inv_)) -
                                  anchor(1));
  const int iy1 = std::min(
      ny - 1,
      static_cast<int>(std::ceil((fp.cy + half_diag) * mp_.resolution_inv_)) -
          anchor(1));

  const int8_t v = footprintValue();
  const bool need_infl = mp_.pub_2d_occupancy_inflate_;
  int changed = 0;
  for (int iy = iy0; iy <= iy1; ++iy) {
    const double wy = (anchor(1) + iy + 0.5) * mp_.resolution_;
    for (int ix = ix0; ix <= ix1; ++ix) {
      const double wx = (anchor(0) + ix + 0.5) * mp_.resolution_;
      if (!inFootprint(wx, wy))
        continue;
      const int i2d = ix + iy * nx;
      if (md_.occ2d_[i2d] != v) {
        md_.occ2d_[i2d] = v;
        ++changed;
      }
      if (need_infl && md_.occ2d_inflate_[i2d] != v) {
        md_.occ2d_inflate_[i2d] = v;
        ++changed;
      }
    }
  }
  if (changed > 0)
    md_.esdf2d_stale_ = true; // footprint 抹掉的格也要反映到 ESDF
}

void GridMap::publishFootprintViz() {
  if (!mp_.enable_2d_ || !mp_.pub_footprint_viz_ || !footprint_viz_pub_ ||
      mp_.footprint_length_ <= 0.0 || mp_.footprint_width_ <= 0.0 ||
      footprint_viz_pub_->get_subscription_count() == 0)
    return;
  const FootprintPose fp = computeFootprintPose();
  if (!fp.valid)
    return;

  // 与 2D 栅格同一节流节奏（pub_2d_interval_），避免无谓刷屏
  const auto now = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point last_viz;
  static bool last_viz_valid = false;
  if (last_viz_valid && mp_.pub_2d_interval_ > 0.0 &&
      std::chrono::duration<double>(now - last_viz).count() <
          mp_.pub_2d_interval_)
    return;
  last_viz = now;
  last_viz_valid = true;

  /* 画在 2D 栅格同一高度（带中点）上，这样在 RViz 里能和 2D 图叠在一起看；
     2D 层还没建起来时退化成雷达高度。 */
  const float z =
      md_.has_2d_ ? static_cast<float>(0.5 * (md_.proj_z_lo_ + md_.proj_z_hi_))
                  : static_cast<float>(md_.ray_pos_.z());
  const float hl = 0.5f * static_cast<float>(mp_.footprint_length_);
  const float hw = 0.5f * static_cast<float>(mp_.footprint_width_);
  const double yaw = std::atan2(fp.s, fp.c); // 由 cos/sin 还原 yaw
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  visualization_msgs::msg::MarkerArray arr;
  const rclcpp::Time stamp = node_->now();

  // 四个角（机体系）→ map 系
  const double lx[4] = {-hl, hl, hl, -hl};
  const double ly[4] = {-hw, -hw, hw, hw};
  auto to_map = [&](double x, double y, geometry_msgs::msg::Point &out) {
    out.x = fp.cx + cy * x - sy * y;
    out.y = fp.cy + sy * x + cy * y;
    out.z = z + 0.02; // 稍抬高一点，避免与 2D 栅格平面 z-fighting
  };

  visualization_msgs::msg::Marker outline;
  outline.header.stamp = stamp;
  outline.header.frame_id = mp_.frame_id_;
  outline.ns = "footprint";
  outline.id = 0;
  outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
  outline.action = visualization_msgs::msg::Marker::ADD;
  outline.pose.orientation.w = 1.0;
  outline.scale.x = 0.02; // 线宽
  outline.color.r = 0.0f;
  outline.color.g = 1.0f;
  outline.color.b = 0.3f;
  outline.color.a = 1.0f;
  outline.points.resize(5);
  for (int i = 0; i < 4; ++i)
    to_map(lx[i], ly[i], outline.points[i]);
  outline.points[4] = outline.points[0]; // 闭合
  arr.markers.push_back(outline);

  visualization_msgs::msg::Marker fill = outline;
  fill.id = 1;
  fill.type = visualization_msgs::msg::Marker::CUBE;
  fill.points.clear();
  fill.pose.position.x = fp.cx;
  fill.pose.position.y = fp.cy;
  fill.pose.position.z = z + 0.01;
  fill.pose.orientation.z = std::sin(yaw * 0.5);
  fill.pose.orientation.w = std::cos(yaw * 0.5);
  fill.scale.x = mp_.footprint_length_;
  fill.scale.y = mp_.footprint_width_;
  fill.scale.z = 0.01;
  fill.color.a = 0.25f;
  arr.markers.push_back(fill);

  footprint_viz_pub_->publish(arr);
}

void GridMap::build2DESDF() {
  if (!mp_.enable_2d_ || !mp_.pub_2d_esdf_ || !md_.has_2d_ ||
      md_.esdf2d_.empty() || !md_.esdf2d_stale_)
    return;

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int n2d = nx * ny;

  const float inv_res = static_cast<float>(mp_.resolution_inv_);
  const float max_cells = static_cast<float>(mp_.esdf_max_dist_) * inv_res;
  const float max_sq = max_cells * max_cells;

  for (int i = 0; i < n2d; ++i) {
    const int8_t c = md_.occ2d_[i];
    const bool blocked =
        (c == 100) || (mp_.esdf_unknown_as_occupied_ && c == -1);
    md_.edt_f_[i] = blocked ? 0.f : kEdtInf;
  }
  for (int iy = 0; iy < ny; ++iy) { // 行方向
    const int row = iy * nx;
    for (int ix = 0; ix < nx; ++ix)
      md_.edt_line_in_[ix] = md_.edt_f_[row + ix];
    edt1D(md_.edt_line_in_, md_.edt_line_out_, nx, md_.edt_v_, md_.edt_z_);
    for (int ix = 0; ix < nx; ++ix)
      md_.edt_sq_[row + ix] = md_.edt_line_out_[ix];
  }
  for (int ix = 0; ix < nx; ++ix) { // 列方向
    for (int iy = 0; iy < ny; ++iy)
      md_.edt_line_in_[iy] = md_.edt_sq_[ix + iy * nx];
    edt1D(md_.edt_line_in_, md_.edt_line_out_, ny, md_.edt_v_, md_.edt_z_);
    for (int iy = 0; iy < ny; ++iy)
      md_.esdf2d_[ix + iy * nx] =
          std::sqrt(std::min(md_.edt_line_out_[iy], max_sq)) / inv_res;
  }

  md_.esdf2d_stale_ = false;
}

void GridMap::verify2DLayer(int dx, int dy, bool full, int rebuilt) {
  if (!mp_.enable_2d_ || !md_.has_2d_ || !md_.occ2d_band_valid_)
    return;

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);
  const int stride_x = ny * nz;
  const Eigen::Vector3i anchor = md_.occ2d_min_idx_; // 2D 数据自己的锚点
  const bool need_infl = mp_.pub_2d_occupancy_inflate_;

  std::vector<int> kz_local;
  for (int kz = md_.occ2d_kz0_; kz <= md_.occ2d_kz1_; ++kz)
    kz_local.push_back(getLocalIndex(kz, 2));

  int bad_occ = 0, bad_infl = 0, first_i2d = -1, first_expected = 0,
      first_got = 0;
  // 自检里 occ / inflate 两层的首例失配分开记（否则报错信息会打出无意义的格号）
  int first_inf_i2d = -1, first_inf_expected = 0, first_inf_got = 0;
  int min_ix = 1 << 30, max_ix = -1, min_iy = 1 << 30, max_iy = -1;
  int first_stamp = -1;
  int bad_now = 0, bad_stale = 0; // “本帧刚重扫过但仍不对” / “本帧没被重扫”
  for (int iy = 0; iy < ny; ++iy) {
    const int ly = getLocalIndex(anchor(1) + iy, 1) * nz;
    for (int ix = 0; ix < nx; ++ix) {
      const int base = getLocalIndex(anchor(0) + ix, 0) * stride_x + ly;
      bool has_occ = false, has_free = false, has_infl = false;
      for (size_t k = 0; k < kz_local.size(); ++k) {
        const int adr = base + kz_local[k];
        const double log_odds = md_.occupancy_buffer_[adr];
        if (log_odds > mp_.min_occupancy_log_)
          has_occ = true;
        else if (log_odds >= mp_.clamp_min_log_)
          has_free = true;
        if (need_infl && md_.occupancy_buffer_inflate_[adr] != 0)
          has_infl = true;
      }
      const int i2d = ix + iy * nx;
      /* footprint 内的格子期望值就是“抹除后的值”（applyFootprintClear 已在
         本帧构建末尾执行）—— 否则自检会把正确处理报成失败。 */
      const bool in_fp = inFootprint((anchor(0) + ix + 0.5) * mp_.resolution_,
                                     (anchor(1) + iy + 0.5) * mp_.resolution_);
      const int8_t e_occ =
          in_fp ? footprintValue() : (has_occ ? 100 : (has_free ? 0 : -1));
      if (md_.occ2d_[i2d] != e_occ) {
        if (bad_occ == 0) {
          first_i2d = i2d;
          first_expected = e_occ;
          first_got = md_.occ2d_[i2d];
          first_stamp = md_.occ2d_last_build_[i2d];
        }
        min_ix = std::min(min_ix, ix);
        max_ix = std::max(max_ix, ix);
        min_iy = std::min(min_iy, iy);
        max_iy = std::max(max_iy, iy);
        if (md_.occ2d_last_build_[i2d] == md_.occ2d_build_cnt_)
          ++bad_now;
        else
          ++bad_stale;
        ++bad_occ;
      }
      if (need_infl) {
        const int8_t e_inf =
            in_fp ? footprintValue() : (has_infl ? 100 : (has_free ? 0 : -1));
        if (md_.occ2d_inflate_[i2d] != e_inf) {
          if (first_inf_i2d < 0) {
            first_inf_i2d = i2d;
            first_inf_expected = e_inf;
            first_inf_got = md_.occ2d_inflate_[i2d];
          }
          ++bad_infl;
        }
      }
    }
  }

  if (bad_occ || bad_infl) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "[GridMap] 2D 自检失败：occ=%d 格 inflate=%d 格 | 首例 (ix=%d,iy=%d) "
        "期望 %d 实际 %d（最后一次重扫@第 %d 次构建，当前第 %d 次）| "
        "差异范围 ix[%d,%d] iy[%d,%d] | 本帧 full=%d dx=%d dy=%d 已重扫=%d | "
        "本帧扫过仍错=%d 本帧未重扫=%d | 带[%d,%d] 锚点(%d,%d) | inflate 首例 "
        "(ix=%d,iy=%d) 期望 %d 实际 %d",
        bad_occ, bad_infl, first_i2d % nx, first_i2d / nx, first_expected,
        first_got, first_stamp, md_.occ2d_build_cnt_, min_ix, max_ix, min_iy,
        max_iy, static_cast<int>(full), dx, dy, rebuilt, bad_now, bad_stale,
        md_.occ2d_kz0_, md_.occ2d_kz1_, anchor(0), anchor(1),
        first_inf_i2d % nx, first_inf_i2d / nx, first_inf_expected,
        first_inf_got);
  } else {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "[GridMap] 2D 自检通过：%d 格与暴力全扫完全一致",
                         nx * ny);
  }
}

void GridMap::publish2D() {
  if (!mp_.enable_2d_ || !md_.has_2d_)
    return;

  /* 节流用 steady_clock：本节点 use_sim_time=false，仿真时间在这里不可靠 */
  const auto now = std::chrono::steady_clock::now();
  if (last_2d_pub_valid_ && mp_.pub_2d_interval_ > 0.0) {
    const double dt =
        std::chrono::duration<double>(now - last_2d_pub_time_).count();
    if (dt < mp_.pub_2d_interval_)
      return;
  }
  last_2d_pub_time_ = now;
  last_2d_pub_valid_ = true;

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const float res = static_cast<float>(mp_.resolution_);
  const float z_mid =
      static_cast<float>(0.5 * (md_.proj_z_lo_ + md_.proj_z_hi_));
  const rclcpp::Time stamp = node_->now();

  auto make_grid_msg = [&](const std::vector<int8_t> &src) {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = mp_.frame_id_;
    msg.info.map_load_time = stamp;
    msg.info.resolution = res;
    msg.info.width = static_cast<uint32_t>(nx);
    msg.info.height = static_cast<uint32_t>(ny);
    // OccupancyGrid 的 origin 是 (0,0) 格的**角点**，格心 = origin +
    // (i+0.5)*res
    msg.info.origin.position.x = md_.occ2d_min_idx_(0) * mp_.resolution_;
    msg.info.origin.position.y = md_.occ2d_min_idx_(1) * mp_.resolution_;
    msg.info.origin.position.z =
        z_mid; // 只作为切片高度的记录（RViz Map 显示会用它）
    msg.info.origin.orientation.w = 1.0;
    msg.data = src; // 布局一致，直接拷贝
    return msg;
  };

  if (occ2d_pub_ && occ2d_pub_->get_subscription_count() > 0)
    occ2d_pub_->publish(make_grid_msg(md_.occ2d_));

  if (occ2d_inf_pub_ && occ2d_inf_pub_->get_subscription_count() > 0)
    occ2d_inf_pub_->publish(make_grid_msg(md_.occ2d_inflate_));

  /* ESDF 只在真正需要时才算（esdf2d_query_en_ 或有人在订阅）；
     本函数受 pub_2d_interval_ 节流，所以进程内查询场景下 ESDF 的刷新率 =
     发布率。 */
  if (mp_.pub_2d_esdf_ &&
      (mp_.esdf2d_query_en_ ||
       (esdf2d_pub_ && esdf2d_pub_->get_subscription_count() > 0))) {
    const auto t_esdf = std::chrono::steady_clock::now();
    build2DESDF();
    if (mp_.show_occ_time_) {
      static int esdf_cnt = 0;
      if (++esdf_cnt % 50 == 0)
        RCLCPP_INFO(node_->get_logger(), "[GridMap] 2D ESDF 构建 %.2f ms",
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t_esdf)
                        .count());
    }
  }

  if (esdf2d_pub_ && esdf2d_pub_->get_subscription_count() > 0) {
    const int step = std::max(1, mp_.esdf_pub_step_);
    const float max_d = static_cast<float>(mp_.esdf_max_dist_);
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(static_cast<size_t>(nx / step + 1) * (ny / step + 1));
    for (int iy = 0; iy < ny; iy += step) {
      const int row = iy * nx;
      for (int ix = 0; ix < nx; ix += step) {
        const float d = md_.esdf2d_[row + ix];
        if (d <= 0.f || d > max_d)
          continue; // 障碍内部（0）与超出截断的不发
        pcl::PointXYZI pt;
        pt.x = static_cast<float>((md_.occ2d_min_idx_(0) + ix + 0.5) *
                                  mp_.resolution_);
        pt.y = static_cast<float>((md_.occ2d_min_idx_(1) + iy + 0.5) *
                                  mp_.resolution_);
        pt.z = z_mid;
        pt.intensity = d;
        cloud.push_back(pt);
      }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = mp_.frame_id_;
    esdf2d_pub_->publish(msg);
  }
}

void GridMap::verifyOccIdx() {
  /* 自检：索引是增量维护的，一旦某个写入路径忘了同步 flag，发布出去的点云就会
     少几块（很难看出来）。这里用暴力全扫重新数一遍，逐体素核对 flag。 */
  const auto now = std::chrono::steady_clock::now();
  static std::chrono::steady_clock::time_point last;
  static bool last_valid = false;
  if (last_valid &&
      std::chrono::duration<double>(now - last).count() < 1.0) // 最多 1Hz
    return;
  last = now;
  last_valid = true;

  size_t occ_bf = 0, infl_bf = 0, occ_miss = 0, infl_miss = 0;
  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        const int addr = toAddress(x, y, z);
        if (md_.occupancy_buffer_[addr] >= mp_.min_occupancy_log_) {
          ++occ_bf;
          if (!md_.occ_idx_flag_[addr])
            ++occ_miss; // 漏记
        }
        if (md_.occupancy_buffer_inflate_[addr] != 0) {
          ++infl_bf;
          if (!md_.infl_idx_flag_[addr])
            ++infl_miss;
        }
      }

  if (occ_miss || infl_miss)
    RCLCPP_ERROR(node_->get_logger(),
                 "[GridMap] 占据索引自检失败：占据漏记 %zu/%zu 格，膨胀漏记 "
                 "%zu/%zu 格（索引列表 %zu/%zu 条）",
                 occ_miss, occ_bf, infl_miss, infl_bf, md_.occ_idx_list_.size(),
                 md_.infl_idx_list_.size());
  else
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                         "[GridMap] 占据索引自检通过：占据 %zu 格、膨胀 %zu 格"
                         "（索引列表 %zu/%zu 条，含坟墓）",
                         occ_bf, infl_bf, md_.occ_idx_list_.size(),
                         md_.infl_idx_list_.size());
}

void GridMap::publishMap() {

  if (map_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  /* 不再扫全图体素（那是 O(体素数)，50 万次里绝大多数是 continue），
     只遍历"占据体素索引"（通常几千个），并顺手把坟墓（不再占据的旧条目）
     原地压掉。索引的维护见 applyOccupancyUpdate / resetCellByAddress /
     滑动清图 / resetAllMapData。 */
  std::vector<int> &lst = md_.occ_idx_list_;
  std::vector<char> &flag = md_.occ_idx_flag_;
  const std::vector<double> &occ = md_.occupancy_buffer_;
  const int z_vis_max =
      md_.has_ray_pose_
          ? static_cast<int>(std::floor((md_.ray_pos_(2) + mp_.vis_height_) *
                                        mp_.resolution_inv_))
          : std::numeric_limits<int>::max();

  size_t w = 0;
  for (size_t i = 0; i < lst.size(); ++i) {
    const int addr = lst[i];
    if (!flag[addr] || occ[addr] < mp_.min_occupancy_log_) {
      flag[addr] = 0; // 坟墓：丢弃（下次再占据时会重新入列）
      continue;
    }
    lst[w++] = addr;

    Eigen::Vector3i id_g;
    hashIdToGlobalIndex(addr, id_g);
    if (id_g(2) >
        z_vis_max) // 与旧实现一致：跳过传感器上方 vis_height 以上的体素
      continue;
    Eigen::Vector3d pos;
    indexToPos(id_g, pos);
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    cloud.push_back(pt);
  }
  lst.resize(w);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info) {

  if (map_inf_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  /* 同 publishMap：遍历"膨胀体素索引"而不是全图（膨胀集比占据集换得更勤，
     但索引只在 0↔1 翻转时维护，见 updateInflationLayer）。 */
  std::vector<int> &lst = md_.infl_idx_list_;
  std::vector<char> &flag = md_.infl_idx_flag_;
  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  const int z_vis_max =
      md_.has_ray_pose_
          ? static_cast<int>(std::floor((md_.ray_pos_(2) + mp_.vis_height_) *
                                        mp_.resolution_inv_))
          : std::numeric_limits<int>::max();

  size_t w = 0;
  for (size_t i = 0; i < lst.size(); ++i) {
    const int addr = lst[i];
    if (!flag[addr] || inflate_buffer[addr] == 0) {
      flag[addr] = 0; // 坟墓
      continue;
    }
    lst[w++] = addr;

    Eigen::Vector3i id_g;
    hashIdToGlobalIndex(addr, id_g);
    if (id_g(2) > z_vis_max)
      continue;
    Eigen::Vector3d pos;
    indexToPos(id_g, pos);
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    cloud.push_back(pt);
  }
  lst.resize(w);

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publishSlidingMapFrame() {
  if (mp_.sliding_map_frame_id_.empty())
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox() {
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d &min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d &max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d &p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
      {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto &edge : edges) {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown() {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] <
            mp_.clamp_min_log_ - 1e-3) {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  unknown_pub_->publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] *
//   mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size) {
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud() {
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i) {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
