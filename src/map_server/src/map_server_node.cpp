// 全局地图服务器（自研）：加载静态地图并按约定发布。
//
// 本期（M1+M2）：
//   M1 加载 nav2 标准 2D 栅格（map.yaml + map.pgm），以 latched QoS 发布
//   M2 运行时通过 LoadMap 服务切换站点（传目录或 yaml 路径）
//
// 3D 点云（同目录的 global.pcd）只留接口，本期不实现。
//
// 与 perception 的分工：perception 发**动态局部**图（grid_map/occupancy_2d），
// 本节点发**静态全局**图（global_map/occupancy）；两者各自独立发布，
// 由规划器自行合成（全局规划用全局图、局部避障用局部图）。

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <nav2_msgs/srv/load_map.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "map_server/map_io.hpp"

namespace fs = std::filesystem;

namespace map_server {

class MapServerNode : public rclcpp::Node {
public:
  MapServerNode() : rclcpp::Node("map_server") {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    topic_occ_ = declare_parameter<std::string>("topic_occupancy",
                                                "global_map/occupancy");
    topic_meta_ =
        declare_parameter<std::string>("topic_metadata", "global_map/metadata");
    srv_name_ =
        declare_parameter<std::string>("srv_load_map", "global_map/load_map");
    publish_metadata_ = declare_parameter<bool>("publish_metadata", true);
    republish_interval_ = declare_parameter<double>("republish_interval", 1.0);
    /* 强烈建议保留 false：置 true 会把“未知”一起当“空闲”发布。
       为什么要这个开关：实测现有的 map.pgm 只含黑(0)+灰(205)，**没有任何白色
       (254) 空闲像素** → 若下游“未知当障碍”，规划器会找不到路。打开本开关等价于
       声明“没看到障碍的地方都能走”（与 perception 的 esdf_unknown_as_occupied=
       false 同一取向）。默认关 = 忠实发布地图原意。 */
    unknown_as_free_ = declare_parameter<bool>("unknown_as_free", false);

    /* ---------- 3D 地图（M3 接口，本期不实现内容）----------
       设计：3D 地图与 2D 在**同一个地图目录**下（global.pcd），跟着同一个
       load_map
       服务一起加载（避免“换图三件套不同源”）；所以不需要单独的服务类型。 */
    publish_3d_ = declare_parameter<bool>("publish_3d", false);
    pcd_file_ = declare_parameter<std::string>("pcd_file", "");
    topic_cloud_3d_ =
        declare_parameter<std::string>("topic_cloud_3d", "global_map/cloud");
    cloud_frame_id_ =
        declare_parameter<std::string>("cloud_frame_id", frame_id_);
    cloud_voxel_leaf_ = declare_parameter<double>("cloud_voxel_leaf", 0.0);
    require_3d_ = declare_parameter<bool>("require_3d", false);

    const std::string map_dir = declare_parameter<std::string>("map_dir", "");
    const std::string map_yaml = declare_parameter<std::string>("map_yaml", "");

    /* 静态地图用 latched（transient_local）：后启动的订阅者也能立刻拿到。
       ⚠ 但 RViz 的 Map 显示项默认 Durability=Volatile，那样晚启动的订阅者
       收不到历史样本 —— 所以下面还会按 republish_interval 周期性重发一次，
       让任何 QoS 的订阅者都能拿到（代价：一张图每秒重发，可忽略）。 */
    auto latched = rclcpp::QoS(1).transient_local();
    occ_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(topic_occ_, latched);
    if (publish_metadata_)
      meta_pub_ =
          create_publisher<nav_msgs::msg::MapMetaData>(topic_meta_, latched);
    if (publish_3d_) {
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          topic_cloud_3d_, latched);
      RCLCPP_INFO(get_logger(),
                  "[map_server] 3D 接口：已创建发布器 %s（latched）| pcd=%s | "
                  "frame=%s | voxel_leaf=%.2f | require_3d=%d",
                  topic_cloud_3d_.c_str(),
                  pcd_file_.empty() ? "<map_dir>/global.pcd"
                                    : pcd_file_.c_str(),
                  cloud_frame_id_.c_str(), cloud_voxel_leaf_,
                  static_cast<int>(require_3d_));
      RCLCPP_WARN(
          get_logger(),
          "[map_server] ⚠ M3 载入/发布逻辑尚未实现（PCD 解析待做）→ 当前不会"
          "发出任何点云；话题存在只是为了先把接口/接线固定下来");
    }
    load_srv_ = create_service<nav2_msgs::srv::LoadMap>(
        srv_name_, std::bind(&MapServerNode::handleLoadMap, this,
                             std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "[map_server] 发布 %s（transient_local, depth 1）| 服务 %s | "
                "frame_id=%s",
                topic_occ_.c_str(), srv_name_.c_str(), frame_id_.c_str());

    const std::string initial = !map_yaml.empty() ? map_yaml : map_dir;
    if (unknown_as_free_)
      RCLCPP_WARN(
          get_logger(),
          "[map_server] unknown_as_free = true：会把“未知”一起当“空闲”发布 "
          "（下游不会再看到 -1）");
    if (initial.empty()) {
      RCLCPP_WARN(get_logger(),
                  "[map_server] 未指定 map_dir / map_yaml → 暂未加载地图；"
                  "可用服务 %s 加载（传站点目录或 yaml 路径）",
                  srv_name_.c_str());
    } else {
      std::string err;
      if (!loadAndPublish(initial, err)) {
        RCLCPP_ERROR(
            get_logger(),
            "[map_server] 初始地图加载失败：%s\n  节点继续运行，可用服务 "
            "%s 重试",
            err.c_str(), srv_name_.c_str());
      }
    }

    if (republish_interval_ > 0.0) {
      repub_timer_ =
          create_wall_timer(std::chrono::duration<double>(republish_interval_),
                            std::bind(&MapServerNode::republish, this));
    }
  }

private:
  /// 加载并发布（成功返回 true；resp 非空时把地图填进服务响应）
  bool loadAndPublish(const std::string &dir_or_yaml, std::string &err,
                      nav_msgs::msg::OccupancyGrid *resp = nullptr) {
    const auto t0 = std::chrono::steady_clock::now();
    OccupancyMap m;
    if (!loadOccupancyMap(dir_or_yaml, m, err))
      return false;

    /* 3D（M3 接口）：先加载再发布 2D，这样 require_3d=true 时能干净地中止 */
    PointCloud3D cloud;
    bool cloud_ok = false;
    std::string cloud_err;
    if (publish_3d_) {
      cloud_ok = loadCloud3D(dir_or_yaml, cloud, cloud_err);
      if (!cloud_ok) {
        if (require_3d_) {
          err = cloud_err;
          return false;
        }
        RCLCPP_WARN(get_logger(), "[map_server] 3D 地图未加载（不影响 2D）：%s",
                    cloud_err.c_str());
      }
    }

    map_msg_ = toMsg(m);
    has_map_ = true;
    occ_pub_->publish(map_msg_);
    if (meta_pub_)
      meta_pub_->publish(map_msg_.info);
    if (resp)
      *resp = map_msg_;

    if (publish_3d_ && cloud_ok)
      publishCloud3D(); // TODO(M3)：内容填好后这里会真的发出去

    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    const size_t total = m.n_occupied + m.n_free + m.n_unknown;
    /* unknown_as_free 时发布的图与文件语义不同：日志按**发布后**的统计打，
       否则会出现“日志说 93% 未知、图里却没有未知”的迷惑。 */
    const size_t pub_free = m.n_free + (unknown_as_free_ ? m.n_unknown : 0);
    const size_t pub_unknown = unknown_as_free_ ? 0 : m.n_unknown;
    RCLCPP_INFO(
        get_logger(),
        "[map_server] 已加载 %s\n"
        "             尺寸 %dx%d @ %.3f m = %.2fx%.2f m | origin "
        "(%.3f, %.3f, yaw %.4f) | frame %s\n"
        "             发布语义：占据 %zu / 空闲 %zu / 未知 %zu（共 %zu）%s"
        "| 耗时 %.1f ms | 已发布到 %s",
        resolveMapYaml(dir_or_yaml).c_str(), m.width, m.height, m.resolution,
        m.width * m.resolution, m.height * m.resolution, m.origin_x, m.origin_y,
        m.origin_yaw, frame_id_.c_str(), m.n_occupied, pub_free, pub_unknown,
        total, unknown_as_free_ ? "【未知已当空闲】 " : "", ms,
        topic_occ_.c_str());
    return true;
  }

  nav_msgs::msg::OccupancyGrid toMsg(const OccupancyMap &m) const {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = static_cast<float>(m.resolution);
    msg.info.width = static_cast<uint32_t>(m.width);
    msg.info.height = static_cast<uint32_t>(m.height);
    msg.info.origin.position.x = m.origin_x;
    msg.info.origin.position.y = m.origin_y;
    msg.info.origin.position.z = 0.0;
    const double half = 0.5 * m.origin_yaw;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = std::sin(half);
    msg.info.origin.orientation.w = std::cos(half);
    msg.data = m.data;
    if (unknown_as_free_) {
      size_t n = 0;
      for (auto &v : msg.data) {
        if (v < 0) {
          v = 0;
          ++n;
        }
      }
      if (n > 0)
        RCLCPP_DEBUG(get_logger(),
                     "[map_server] unknown_as_free: %zu 格 -1 → 0", n);
    }
    return msg;
  }

  /// 周期性重发（让 Volatile 订阅者也能拿到静态图）
  void republish() {
    if (!has_map_)
      return;
    map_msg_.header.stamp = now();
    occ_pub_->publish(map_msg_);
    if (meta_pub_)
      meta_pub_->publish(map_msg_.info);
    if (publish_3d_ && has_cloud_3d_)
      publishCloud3D();
  }

  /* ---------------- 3D 地图（M3 接口） ---------------- */

  /// 解析 PCD 路径（pcd_file 优先，否则 <地图目录>/global.pcd）并读入
  bool loadCloud3D(const std::string &dir_or_yaml, PointCloud3D &out,
                   std::string &err) {
    std::string path = pcd_file_;
    if (path.empty()) {
      // 没显式给 pcd_file：默认与 yaml 同目录（目录）
      std::error_code ec;
      const std::string d =
          fs::is_directory(dir_or_yaml, ec)
              ? dir_or_yaml
              : fs::path(resolveMapYaml(dir_or_yaml)).parent_path().string();
      path = resolvePcd(d);
    } else {
      path = resolvePcd(path);
    }

    const auto t0 = std::chrono::steady_clock::now();
    if (!loadPcd(path, out, err))
      return false;

    /* TODO(M3)：把 out.xyz 填进 cloud_msg_（sensor_msgs/PointCloud2，xyz
       float32）， 按 cloud_voxel_leaf_ 可选降采样，并设 has_cloud_3d_ = true。
       发布器/lathed/重发/服务接入都已经就绪，这里填完就生效。 */
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    RCLCPP_INFO(
        get_logger(),
        "[map_server] 3D 地图已加载 %s（%zu 点）| frame %s | 耗时 %.1f ms",
        path.c_str(), out.n_points, cloud_frame_id_.c_str(), ms);
    return true;
  }

  /// 发布 3D 点云（latched + 周期重发）
  void publishCloud3D() {
    if (!cloud_pub_ || !has_cloud_3d_)
      return;
    cloud_msg_.header.stamp = now();
    cloud_pub_->publish(cloud_msg_);
  }

  void
  handleLoadMap(const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> req,
                std::shared_ptr<nav2_msgs::srv::LoadMap::Response> res) {
    using Res = nav2_msgs::srv::LoadMap::Response;
    std::string path = req->map_url;
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "[map_server] LoadMap 收到空的 map_url");
      res->result = Res::RESULT_INVALID_MAP_METADATA;
      return;
    }
    if (path.rfind("package://", 0) == 0) {
      RCLCPP_ERROR(get_logger(),
                   "[map_server] 暂不支持 package:// URL（请直接给绝对路径或 "
                   "file:// 路径）：%s",
                   path.c_str());
      res->result = Res::RESULT_MAP_DOES_NOT_EXIST;
      return;
    }
    if (path.rfind("file://", 0) == 0)
      path = path.substr(7); // 去掉 file:// 前缀

    const std::string yaml_path = resolveMapYaml(path);
    if (!fs::exists(yaml_path)) {
      RCLCPP_ERROR(get_logger(), "[map_server] LoadMap 失败：%s 不存在",
                   yaml_path.c_str());
      res->result = Res::RESULT_MAP_DOES_NOT_EXIST;
      return;
    }
    std::string err;
    if (!loadAndPublish(path, err, &res->map)) {
      RCLCPP_ERROR(get_logger(), "[map_server] LoadMap 失败：%s", err.c_str());
      res->result = Res::RESULT_INVALID_MAP_DATA;
      return;
    }
    res->result = Res::RESULT_SUCCESS;
  }

  // 参数
  std::string frame_id_;
  std::string topic_occ_;
  std::string topic_meta_;
  std::string srv_name_;
  bool publish_metadata_{true};
  double republish_interval_{1.0};
  bool unknown_as_free_{false};
  // 3D（M3 接口）
  bool publish_3d_{false};
  std::string pcd_file_;
  std::string topic_cloud_3d_;
  std::string cloud_frame_id_;
  double cloud_voxel_leaf_{0.0};
  bool require_3d_{false};

  // 发布器/服务
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr meta_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_srv_;
  rclcpp::TimerBase::SharedPtr repub_timer_;

  // 状态
  nav_msgs::msg::OccupancyGrid map_msg_;
  bool has_map_{false};
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  bool has_cloud_3d_{false};
};

} // namespace map_server

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_server::MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
