#include "plan_env/grid_map.h"
#include <cmath>
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
 * f[q] = 0 表示 q 处是障碍（种子），否则为 kEdtInf；d[q] 输出到最近种子的平方距离（单位：索引²）。
 * v、z 为调用者提供的 scratch，长度需 >= n+1 / n+2。
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
  load_parameter(node_, "grid_map.proj_relative_to_sensor",
                 mp_.proj_relative_to_sensor_, false);
  load_parameter(node_, "grid_map.proj_z_min", mp_.proj_z_min_, -0.5);
  load_parameter(node_, "grid_map.proj_z_max", mp_.proj_z_max_, 2.0);
  load_parameter(node_, "grid_map.esdf_unknown_as_occupied",
                 mp_.esdf_unknown_as_occupied_, false);
  load_parameter(node_, "grid_map.esdf_max_dist", mp_.esdf_max_dist_, 3.0);
  load_parameter(node_, "grid_map.esdf_pub_step", mp_.esdf_pub_step_, 2);
  load_parameter(node_, "grid_map.topic_2d_occupancy",
                 mp_.topic_2d_occupancy_, string("grid_map/occupancy_2d"));
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
    md_.occ2d_occ_cnt_.assign(n2d, 0);
    md_.occ2d_free_cnt_.assign(n2d, 0);
    md_.occ2d_infl_cnt_.assign(n2d, 0);
    md_.esdf2d_.assign(n2d, -1.f);
    md_.has_2d_ = false;
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
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose",
                               rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(
        new message_filters::Synchronizer<SyncPolicyImagePose>(
            SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(std::bind(&GridMap::depthPoseCallback,
                                                 this, std::placeholders::_1,
                                                 std::placeholders::_2));
  } else if (mp_.sensor_type_ == "lidar") {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this,
                std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GridMap::updateOccupancyCallback, this));
  vis_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
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

  /* ---------- 2D 局部产物发布器（enable_2d_ 为 false 时一个都不建）---------- */
  if (mp_.enable_2d_) {
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
    RCLCPP_INFO(node_->get_logger(),
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

  /* 2D 层属于“上一张地图/上一段观测”的数据，一起作废；
     否则换图后 getDistance2D() 会返回旧地图的距离场。 */
  if (!md_.occ2d_.empty()) {
    std::fill(md_.occ2d_.begin(), md_.occ2d_.end(), static_cast<int8_t>(-1));
    std::fill(md_.occ2d_inflate_.begin(), md_.occ2d_inflate_.end(),
              static_cast<int8_t>(-1));
    std::fill(md_.occ2d_occ_cnt_.begin(), md_.occ2d_occ_cnt_.end(), 0);
    std::fill(md_.occ2d_free_cnt_.begin(), md_.occ2d_free_cnt_.end(), 0);
    std::fill(md_.occ2d_infl_cnt_.begin(), md_.occ2d_infl_cnt_.end(), 0);
    std::fill(md_.esdf2d_.begin(), md_.esdf2d_.end(), -1.f);
    md_.has_2d_ = false;
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

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
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
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

void GridMap::resetCellByAddress(int addr) {
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
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

  publishMap();
  publishMapInflate(true);
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
  publish2D();
}

void GridMap::updateOccupancyCallback() {
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" <<
  // (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 -
  //   t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  /* 3D 融合结束 → 更新 2D 局部层（高度带投影 + 可选 ESDF）*/
  if (mp_.enable_2d_) {
    const auto t0 = std::chrono::steady_clock::now();
    build2DLayer();
    if (mp_.show_occ_time_) {
      const double ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
      RCLCPP_INFO(node_->get_logger(), "[GridMap] 2D layer build %.2f ms", ms);
    }
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

  if (md_.proj_points_cnt == 0)
    return;

  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
}

/* ============================== 2D 局部层 ==============================
 * 把 3D 占据栅格按高度带投影成 2D，并按需算 2D ESDF。
 *
 * 几个刻意的设计选择：
 *  1) 所有 2D 产物按**全局索引连续排列**（i2d = ix + iy * nx），不做环形 wrap。
 *     3D 缓冲为了滑动查表是环形下标的，所以这里是 "读 3D（自动 wrap）→ 写 2D（连续）"，
 *     于是 2D 层在世界系下是一个真正的矩形，EDT 不会跨越边界算出假的近距离。
 *     这个布局与 nav_msgs/OccupancyGrid 的 data[x + y*width] 完全一致，发布时可以整段拷贝。
 *  2) 每格三种状态：占据 100 / 空闲 0 / 未知 -1。
 *     高度带内**只要有一个体素是占据**就算占据（与 3D 的 inflate 语义一致）。
 *  3) ESDF 默认对未知区域取乐观解释（当自由），与 3D 层一致；
 *     需要保守规划时把 esdf_unknown_as_occupied 打开。
 */

void GridMap::build2DLayer() {
  if (!mp_.enable_2d_)
    return;

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
  /// 固定本帧 2D 数据对应的窗口原点（发布/查询时都用它，避免与滑动地图错位）
  md_.occ2d_min_idx_ = mp_.map_bound_min_idx_;

  const bool need_infl = mp_.pub_2d_occupancy_inflate_;
  std::fill(md_.occ2d_.begin(), md_.occ2d_.end(), static_cast<int8_t>(-1));
  std::fill(md_.occ2d_occ_cnt_.begin(), md_.occ2d_occ_cnt_.end(), 0);
  std::fill(md_.occ2d_free_cnt_.begin(), md_.occ2d_free_cnt_.end(), 0);
  if (need_infl)
    std::fill(md_.occ2d_infl_cnt_.begin(), md_.occ2d_infl_cnt_.end(), 0);

  if (kz0 > kz1) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "[GridMap] 2D 投影高度带 [%.2f, %.2f] 与滑动地图 z 区间不相交，2D 层全为未知"
        "（检查 proj_z_min/proj_z_max 与 sliding_map_size_z）",
        mp_.proj_z_min_, mp_.proj_z_max_);
    md_.has_2d_ = false;
    return;
  }

  /* ---------- 2) 逐 (x,y) 扫高度带 ---------- */
  /// toAddress 的布局是 lx*(ny*nz) + ly*nz + lz，这里把不随格子变化的量提到循环外
  std::vector<int> kz_local;
  kz_local.reserve(static_cast<size_t>(kz1 - kz0 + 1));
  for (int kz = kz0; kz <= kz1; ++kz)
    kz_local.push_back(getLocalIndex(kz, 2));
  const int stride_x = ny * nz;
  const int bx0 = mp_.map_bound_min_idx_(0);
  const int by0 = mp_.map_bound_min_idx_(1);

  for (int iy = 0; iy < ny; ++iy) {
    const int ly = getLocalIndex(by0 + iy, 1) * nz;
    for (int ix = 0; ix < nx; ++ix) {
      const int base = getLocalIndex(bx0 + ix, 0) * stride_x + ly;
      int occ_cnt = 0, free_cnt = 0, infl_cnt = 0;
      for (size_t k = 0; k < kz_local.size(); ++k) {
        const int adr = base + kz_local[k];
        const double log_odds = md_.occupancy_buffer_[adr];
        if (log_odds > mp_.min_occupancy_log_) {
          ++occ_cnt;
        } else if (log_odds >= mp_.clamp_min_log_) {
          ++free_cnt;
        }
        if (need_infl && md_.occupancy_buffer_inflate_[adr] != 0)
          ++infl_cnt;
      }

      const int i2d = ix + iy * nx;
      md_.occ2d_occ_cnt_[i2d] = occ_cnt;
      md_.occ2d_free_cnt_[i2d] = free_cnt;
      if (need_infl)
        md_.occ2d_infl_cnt_[i2d] = infl_cnt;

      md_.occ2d_[i2d] = occ_cnt > 0 ? static_cast<int8_t>(100)
                                    : (free_cnt > 0 ? static_cast<int8_t>(0)
                                                    : static_cast<int8_t>(-1));
      if (need_infl)
        md_.occ2d_inflate_[i2d] =
            infl_cnt > 0 ? static_cast<int8_t>(100)
                         : (free_cnt > 0 ? static_cast<int8_t>(0)
                                         : static_cast<int8_t>(-1));
    }
  }

  /* ---------- 3) 2D ESDF（可选）---------- */
  if (mp_.pub_2d_esdf_) {
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
  }

  md_.has_2d_ = true;
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
    // OccupancyGrid 的 origin 是 (0,0) 格的**角点**，格心 = origin + (i+0.5)*res
    msg.info.origin.position.x = md_.occ2d_min_idx_(0) * mp_.resolution_;
    msg.info.origin.position.y = md_.occ2d_min_idx_(1) * mp_.resolution_;
    msg.info.origin.position.z = z_mid; // 只作为切片高度的记录（RViz Map 显示会用它）
    msg.info.origin.orientation.w = 1.0;
    msg.data = src; // 布局一致，直接拷贝
    return msg;
  };

  if (occ2d_pub_ && occ2d_pub_->get_subscription_count() > 0)
    occ2d_pub_->publish(make_grid_msg(md_.occ2d_));

  if (occ2d_inf_pub_ && occ2d_inf_pub_->get_subscription_count() > 0)
    occ2d_inf_pub_->publish(make_grid_msg(md_.occ2d_inflate_));

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

void GridMap::publishMap() {

  if (map_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

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

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z) {
        if (inflate_buffer[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

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
