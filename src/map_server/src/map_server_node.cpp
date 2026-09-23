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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nav2_msgs/srv/load_map.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>

#include "map_server/map_io.hpp"
// 路网数据层（ROS-free）复用 pnc_2d：本节点负责把路网当**地图资产**管起来
// （加载、校验、按站点切换、可视化），规划器只负责在上面找通路。
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/map_zones.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "pnc_2d/msg/zone_array.hpp"

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
    /* 重发策略：
         · republish_interval > 0 ：周期重发（老行为，仅调试/兼容用；实测本项目
           186349 格 ≈ 182 KB 的图每秒发一遍会白烧 ~1.5 Mbit/s，还会每秒唤醒
           所有下游节点）；
         · republish_on_new_subscriber（默认开）：轻量看门狗，只在“新订阅者出现”
           时补发一次 —— 载图/换图那一次 latched 发布之外，稳态下不再发。 */
    republish_interval_ = declare_parameter<double>("republish_interval", 0.0);
    republish_on_new_subscriber_ =
        declare_parameter<bool>("republish_on_new_subscriber", true);
    /* 强烈建议保留 false：置 true 会把“未知”一起当“空闲”发布。
       为什么要这个开关：实测现有的 map.pgm 只含黑(0)+灰(205)，**没有任何白色
       (254) 空闲像素** → 若下游“未知当障碍”，规划器会找不到路。打开本开关等价于
       声明“没看到障碍的地方都能走”（与 perception 的 esdf_unknown_as_occupied=
       false 同一取向）。默认关 = 忠实发布地图原意。 */
    unknown_as_free_ = declare_parameter<bool>("unknown_as_free", false);

    /* 地图本体的**膨胀层** [m]（对障碍做圆形膨胀，默认 0 = 关）。
       为什么要它：全局图是共享资产，消费它的模块不都会做朝向感知的车体扫掠
       检查 —— RViz、以后接别的导航栈、以及任何"只看中心格"的快速判定，都会
       把车使到离墙 0 的地方。给图加几 cm 余量后，"点在图上"就自动带上余量。
       ⚠ 它会与 `footprint.safe_margin`（规划器自己的安全边）**叠加**：两个都开
       时实际余量 = 两者之和（日志会把总值打出来，避免历史上踩过的"同一件事
       算了两次"那种盲区）。不想要双份就把其中一个置 0。
       ⚠ 只在**载图/重载时**生效（与禁行区烧入同一时机）。 */
    inflate_ = declare_parameter<double>("inflate", 0.0);

    /* ---------------- 路网（**可选**资产） ----------------
       约定：与 map.yaml 同目录的 routes.yaml（换图四件套同源）。
       ⚠ 路网可有可无：站点没这个文件时只打一句
       INFO，**绝不报错**（很多站点还没画）。
       本节点不做规划，但会用地图+车体轮廓校验"这条通道车能不能过"，并在 RViz
       标红。 */
    routes_file_ = declare_parameter<std::string>("routes_file", "");
    publish_routes_ = declare_parameter<bool>("publish_routes", true);
    topic_routes_ =
        declare_parameter<std::string>("topic_routes", "global_map/routes");
    /* 禁行区（zones 里 type:
       forbidden）：**全栈约束**，统一在这里烧进发布的全局图， 这样全局规划 /
       局部规划 / RViz / 将来 nav2 都自动遵守（分头解析一定会漏）。
       zones.burn_into_map=false 时只可视化不烧入（调试用）。 */
    burn_zones_ = declare_parameter<bool>("zones.burn_into_map", true);
    zone_inflate_ =
        declare_parameter<double>("zones.inflate", -1.0); // <0 = 自动
    /* 区域层几何的发布（latched）：**与"烧进全局图"互补**。
       全局图只解决订阅它的人（全局规划器/RViz）；而局部用的是感知的滑动窗 +
       ESDF，里面没有区域 ⇒ 必须把几何单独传过去，由局部自己融合。
       限速区从来就不进栅格，只能靠这个话题传。 */
    publish_zones_ = declare_parameter<bool>("publish_zones", true);
    topic_zones_ =
        declare_parameter<std::string>("topic_zones", "global_map/zones");
    // 参数改动立即写回成员。
    // ★ 为什么必须有：`declare_parameter` 只在启动时读一次，成员变量不会跟着
    //   `ros2 param set` 变。踩过：运行时设 zones.inflate=0 再调 LoadMap 重载，
    //   **烧入还是按启动时的 0.472 算的**，而 `ros2 param get` 显示 0.0 ——
    //   "改了没反应"，极难排查。
    //   ⚠ 重载（LoadMap）会从磁盘重读未烧入的原图并重新 applyZones，所以
    //     "设参数 + 调 LoadMap" 是一个可用的小闭环；不重载则只影响下次加载。
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &ps) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          for (const auto &p : ps) {
            if (p.get_name() == "zones.inflate")
              zone_inflate_ = p.as_double();
            else if (p.get_name() == "zones.burn_into_map")
              burn_zones_ = p.as_bool();
            else if (p.get_name() == "inflate")
              inflate_ = p.as_double();
          }
          return r;
        });
    // 车体轮廓与代价语义（与 pnc_2d 同名，便于对齐；仅用于路网可行性校验）
    fp_.enable = declare_parameter<bool>("footprint.enable", true);
    fp_.length = declare_parameter<double>("footprint.length", 0.70);
    fp_.width = declare_parameter<double>("footprint.width", 0.40);
    fp_.offset_x = declare_parameter<double>("footprint.offset_x", 0.0);
    fp_.offset_y = declare_parameter<double>("footprint.offset_y", 0.0);
    fp_.safe_margin = declare_parameter<double>("footprint.safe_margin", 0.05);
    fp_.check_edges = declare_parameter<bool>("footprint.check_edges", true);
    fp_.fast_path = declare_parameter<bool>("footprint.fast_path", true);
    hard_threshold_ = declare_parameter<int>("common.hard_threshold", 80);
    unknown_as_occupied_ =
        declare_parameter<bool>("common.unknown_as_occupied", true);

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
       ⚠ 但 RViz 的 Map 显示项默认 Durability=Volatile：DDS 规则下 volatile
       订阅者**不接收历史样本**，所以“只发一次”对晚启动的 RViz 等于没发。
       旧做法是每秒重发一遍图（零配置但白烧带宽、每秒唤醒所有下游）；
       现在改成**按需补发**：由 watchSubscribers() 看门狗发现“有新订阅者”时再发
       一次（详见文件末尾 watchSubscribers）。 */
    auto latched = rclcpp::QoS(1).transient_local();
    occ_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(topic_occ_, latched);
    if (publish_metadata_)
      meta_pub_ =
          create_publisher<nav_msgs::msg::MapMetaData>(topic_meta_, latched);
    if (publish_routes_) {
      routes_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          topic_routes_, rclcpp::QoS(1).transient_local());
      RCLCPP_INFO(get_logger(),
                  "[map_server] 路网：发布 %s（latched）| 文件 %s",
                  topic_routes_.c_str(),
                  routes_file_.empty() ? "<地图目录>/routes.yaml"
                                       : routes_file_.c_str());
    }
    if (publish_zones_) {
      zones_pub_ = create_publisher<pnc_2d::msg::ZoneArray>(
          topic_zones_, rclcpp::QoS(1).transient_local());
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 区域层：发布 %s（latched）| 局部据它适配禁行区/限速区",
          topic_zones_.c_str());
    }
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

    /* 重发定时器：稳态下什么都不干。
       Humble 的 rclcpp::PublisherEventCallbacks **没有 matched 回调**（Iron
       之后 才加），所以这里用轻量轮询代替“订阅者匹配事件”：每 0.5 s
       只做一次整数比较，
       只有订阅者数量变多时才去查图（get_subscriptions_info_by_topic）。 */
    if (republish_on_new_subscriber_) {
      constexpr double kWatchPeriodS = 0.5;
      watch_timer_ =
          create_wall_timer(std::chrono::duration<double>(kWatchPeriodS),
                            std::bind(&MapServerNode::watchSubscribers, this));
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 按需补发已开：新订阅者出现时补发一次（轮询 %.1f s，"
          "稳态零流量）",
          kWatchPeriodS);
    }
    if (republish_interval_ > 0.0) {
      repub_timer_ =
          create_wall_timer(std::chrono::duration<double>(republish_interval_),
                            std::bind(&MapServerNode::republish, this));
      RCLCPP_WARN(
          get_logger(),
          "[map_server] 周期性重发已开：每 %.1f s 重发一次（地图 %zu 格 "
          "≈ %.0f KB）—— 仅调试/兼容用，正常应设 republish_interval=0",
          republish_interval_, map_msg_.data.size(),
          static_cast<double>(map_msg_.data.size()) / 1024.0);
    } else if (!republish_on_new_subscriber_) {
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 只在载图/换图时发一次（latched）。若 RViz 看不到图："
          "把 Map 显示项的 Durability 改成 Transient Local，或打开 "
          "republish_on_new_subscriber");
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

    // ---- 地图本体的膨胀层（障碍向外扩 inflate_ 米）：**发布之前**做 ----
    //   顺序：先膨胀真障碍，再烧禁行区 —— 两者独立，日志也能分开归因。
    dilated_cells_ = 0;
    if (inflate_ > 0.0) {
      const size_t occ_before = m.n_occupied;
      dilated_cells_ = dilateOccupied(map_msg_.data, m.width, m.height,
                                      m.resolution, inflate_, hard_threshold_);
      map_msg_.header.stamp = now();
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 地图膨胀 %.3f m（%d 格，圆形）→ 新增占据 %zu 格"
          "（%zu → %zu）%s",
          inflate_,
          std::max(1,
                   static_cast<int>(std::ceil(inflate_ / m.resolution - 1e-9))),
          dilated_cells_, occ_before, occ_before + dilated_cells_,
          fp_.safe_margin > 0.0
              ? "｜⚠ 与 footprint.safe_margin 叠加：规划器实际余量 ≈ 两者之和"
              : "");
    }

    // 区域层（禁行 / 限速）：必须**发布之前**烧进图，否则下游拿到的是旧图
    applyZones(dir_or_yaml);

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
       否则会出现“日志说 93% 未知、图里却没有未知”的迷惑。
       膨胀层/禁行区新增的格子都算在“占据”里，并从空闲/未知里扣掉。
       （不算进去的话日志会自相矛盾：图上明明变多了，统计却说没变。） */
    const size_t free_base = m.n_free + (unknown_as_free_ ? m.n_unknown : 0);
    const size_t extra = dilated_cells_ + burned_cells_;
    const size_t pub_free = free_base > extra ? free_base - extra : 0;
    const size_t pub_unknown = unknown_as_free_ ? 0 : m.n_unknown;
    const size_t pub_occupied = m.n_occupied + extra;
    RCLCPP_INFO(
        get_logger(),
        "[map_server] 已加载 %s\n"
        "             尺寸 %dx%d @ %.3f m = %.2fx%.2f m | origin "
        "(%.3f, %.3f, yaw %.4f) | frame %s\n"
        "             发布语义：占据 %zu / 空闲 %zu / 未知 %zu（共 %zu）%s"
        "| 耗时 %.1f ms | 已发布到 %s",
        resolveMapYaml(dir_or_yaml).c_str(), m.width, m.height, m.resolution,
        m.width * m.resolution, m.height * m.resolution, m.origin_x, m.origin_y,
        m.origin_yaw, frame_id_.c_str(), pub_occupied, pub_free, pub_unknown,
        total, unknown_as_free_ ? "【未知已当空闲】 " : "", ms,
        topic_occ_.c_str());
    if (dilated_cells_ > 0) {
      RCLCPP_INFO(get_logger(),
                  "[map_server] 其中 %zu 格来自**地图膨胀层**（%.3f m）",
                  dilated_cells_, inflate_);
    }
    if (burned_cells_ > 0) {
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 其中 %zu 格来自禁行区（%zu 个区域，已按 %.3f m 膨胀）",
          burned_cells_, zones_.forbiddenCount(), zone_inflate_used_);
    }
    if (zones_.speedCount() > 0) {
      RCLCPP_INFO(get_logger(),
                  "[map_server] 限速区 %zu "
                  "个（不烧入栅格，供速度规划查询；RViz 里已标出）",
                  zones_.speedCount());
    }

    // 路网（可选）：放在地图发布之后，日志顺序好看，且能直接用刚载入的图做可行性校验
    loadRoutes(dir_or_yaml, m);
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

  /* ---------------- 路网（可选资产） ---------------- */

  /// 区域层：从与路网同一个文件读 `zones:`，禁行区烧进
  /// map_msg_（**必须在发布之前调**）
  void applyZones(const std::string &dir_or_yaml) {
    zones_ = pnc_2d::ZoneSet();
    burned_cells_ = 0;
    zone_inflate_used_ = 0.0;
    /* 用 do-while(false) 当"带统一成功出口的失败返回"：
       ★ 无论哪一条失败路上，最后都要**发布一次区域**（哪怕是空数组）——
         latched 的空数组表示“这张图确实没有区域”，与“还没加载”是两回事，
         消费者（局部）必须能区分：前者可以正常跑，后者应该保持今天的自由行为。
     */
    do {
      const std::string path = resolveRoutesFile(dir_or_yaml);
      if (!fs::exists(path))
        break; // 没有文件 = 没有区域，完全正常

      std::string err;
      if (!zones_.loadFromFile(path, err)) {
        RCLCPP_WARN(get_logger(),
                    "[map_server] 区域层（zones）解析失败（不影响地图）：%s",
                    err.c_str());
        zones_ = pnc_2d::ZoneSet();
        break;
      }
      for (const std::string &w : zones_.warnings()) {
        RCLCPP_WARN(get_logger(), "[map_server] 区域层：%s", w.c_str());
      }
      if (zones_.empty())
        break;
      RCLCPP_INFO(get_logger(), "[map_server] 区域层：%s",
                  zones_.summary().c_str());

      // 膨胀量：默认车体**外接圆半径**（保证任何朝向都不侵入禁行区）
      double inflate = zone_inflate_;
      if (inflate < 0.0 && fp_.enable) {
        inflate = std::hypot(fp_.length * 0.5 + fp_.safe_margin,
                             fp_.width * 0.5 + fp_.safe_margin);
      }
      inflate = std::max(0.0, inflate);
      zone_inflate_used_ = inflate;
      if (zones_.forbiddenCount() == 0)
        break; // 只有限速区：不进栅格，但要发布几何
      if (!burn_zones_) {
        RCLCPP_WARN(get_logger(), "[map_server] zones.burn_into_map=false → "
                                  "禁行区**不生效**（仅可视化）");
        break;
      }

      pnc_2d::CostMap2D cm;
      if (!cm.set(static_cast<int>(map_msg_.info.width),
                  static_cast<int>(map_msg_.info.height),
                  map_msg_.info.resolution, map_msg_.info.origin.position.x,
                  map_msg_.info.origin.position.y, 0.0, map_msg_.data,
                  frame_id_)) {
        RCLCPP_WARN(get_logger(), "[map_server] 禁行区烧入失败：地图无效");
        break;
      }
      std::vector<int8_t> burned;
      burned_cells_ = pnc_2d::burnForbidden(zones_, cm, inflate, burned);
      map_msg_.data = std::move(burned);
      map_msg_.header.stamp = now();
    } while (false);

    publishZones();
  }

  /// 发布区域层几何（latched）。与"烧进全局图"互补：全局图只解决订阅它的人
  /// （全局规划器/RViz），而局部用的是感知滑动窗 + ESDF（里面没有区域）⇒
  /// 必须单独传几何，由局部自己融合；限速区本来就只能靠这里传。
  void publishZones() {
    if (!zones_pub_)
      return;
    pnc_2d::msg::ZoneArray msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.inflate = zone_inflate_used_;
    msg.zones.reserve(zones_.zones().size());
    for (const pnc_2d::MapZone &z : zones_.zones()) {
      pnc_2d::msg::Zone out;
      out.name = z.name;
      out.type = pnc_2d::toString(z.type);
      out.value = z.value;
      for (const pnc_2d::Pose2D &p : z.polygon) {
        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(p.x);
        pt.y = static_cast<float>(p.y);
        pt.z = 0.0f;
        out.polygon.points.push_back(pt);
      }
      msg.zones.push_back(out);
    }
    zones_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "[map_server] 区域层发布：%zu 个（禁行 %zu / 限速 %zu），"
                "inflate %.3f m",
                msg.zones.size(), zones_.forbiddenCount(), zones_.speedCount(),
                zone_inflate_used_);
  }

  /// 区域层可视化：禁行区红框 + 名字；限速区橙框 + "0.3 m/s"
  void appendZoneMarkers(visualization_msgs::msg::MarkerArray &arr,
                         const rclcpp::Time &stamp,
                         std::map<std::string, int> &counts) {
    if (zones_.empty())
      return;
    using Marker = visualization_msgs::msg::Marker;
    auto base = [&](const std::string &ns, int id) {
      Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id_;
      m.ns = ns;
      m.id = id;
      m.type = Marker::LINE_STRIP;
      m.action = Marker::ADD;
      m.lifetime = rclcpp::Duration::from_seconds(0.0);
      return m;
    };

    int fid = 0;
    int sid = 0;
    for (const pnc_2d::MapZone &z : zones_.zones()) {
      const bool forb = (z.type == pnc_2d::ZoneType::kForbidden);
      const std::string ns = forb ? "zones_forbidden" : "zones_speed";
      const int id = forb ? fid : sid;
      Marker m = base(ns, id);
      m.scale.x = 0.12;
      m.color.a = 0.9F;
      if (forb) {
        m.color.r = 0.90F;
        m.color.g = 0.10F;
        m.color.b = 0.10F;
      } else {
        m.color.r = 1.00F;
        m.color.g = 0.60F;
        m.color.b = 0.00F;
      }
      double cx = 0.0;
      double cy = 0.0;
      for (const pnc_2d::Pose2D &p : z.polygon) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.03;
        m.points.push_back(pt);
        cx += p.x;
        cy += p.y;
      }
      m.points.push_back(m.points.front()); // 闭合
      arr.markers.push_back(m);

      Marker t = base(ns + "_labels", id);
      t.type = Marker::TEXT_VIEW_FACING;
      const double n = static_cast<double>(z.polygon.size());
      t.pose.position.x = cx / n;
      t.pose.position.y = cy / n;
      t.pose.position.z = 0.60;
      t.scale.z = 0.35;
      t.color.r = t.color.g = t.color.b = 1.0F;
      t.color.a = 0.95F;
      std::ostringstream oss;
      oss << z.name;
      if (!forb)
        oss << "  " << z.value << " m/s";
      t.text = oss.str();
      arr.markers.push_back(t);

      (forb ? fid : sid)++;
    }
    counts["zones_forbidden"] = fid;
    counts["zones_forbidden_labels"] = fid;
    counts["zones_speed"] = sid;
    counts["zones_speed_labels"] = sid;
  }

  /// 路网文件路径：显式 routes_file 优先，否则 <地图目录>/routes.yaml
  std::string resolveRoutesFile(const std::string &dir_or_yaml) const {
    if (!routes_file_.empty())
      return routes_file_;
    std::error_code ec;
    const std::string dir =
        fs::is_directory(dir_or_yaml, ec)
            ? dir_or_yaml
            : fs::path(resolveMapYaml(dir_or_yaml)).parent_path().string();
    return (fs::path(dir) / "routes.yaml").string();
  }

  /// 载入路网。⚠ 没有文件 /
  /// 文件坏了都只提示，**不影响地图发布**（路网是可选的）
  void loadRoutes(const std::string &dir_or_yaml, const OccupancyMap &m) {
    (void)m;
    has_routes_ = false;
    infeasible_edges_.clear();
    if (!publish_routes_)
      return;
    const std::string path = resolveRoutesFile(dir_or_yaml);

    if (!fs::exists(path)) {
      RCLCPP_INFO(get_logger(),
                  "[map_server] 该站点没有路网文件（路网是可选的，跳过）：%s",
                  path.c_str());
      publishRoutes(nullptr); // 换站点时清掉上一个站点的可视化
      return;
    }
    std::string err;
    if (!routes_.loadFromFile(path, err)) {
      RCLCPP_WARN(get_logger(), "[map_server] 路网载入失败（不影响地图）：%s",
                  err.c_str());
      publishRoutes(nullptr);
      return;
    }
    has_routes_ = true;
    for (const std::string &w : routes_.warnings()) {
      RCLCPP_WARN(get_logger(), "[map_server] 路网：%s", w.c_str());
    }
    checkRoutesFeasibility();
    RCLCPP_INFO(get_logger(), "[map_server] 路网已加载 %s → %s", path.c_str(),
                routes_.summary().c_str());
    if (!infeasible_edges_.empty()) {
      std::string names;
      for (const int i : infeasible_edges_) {
        const pnc_2d::RouteEdge &e = routes_.edges()[static_cast<size_t>(i)];
        names += (names.empty() ? "" : ", ") +
                 routes_.nodes()[static_cast<size_t>(e.from)].name + "→" +
                 routes_.nodes()[static_cast<size_t>(e.to)].name;
      }
      RCLCPP_WARN(get_logger(),
                  "[map_server] ⚠ %zu 条通道车体过不去（RViz 里已标红）：%s",
                  infeasible_edges_.size(), names.c_str());
      RCLCPP_WARN(
          get_logger(),
          "             车体 %.2fx%.2f + margin %.2f；请挪通道或确认地图",
          fp_.length, fp_.width, fp_.safe_margin);
    } else {
      RCLCPP_INFO(
          get_logger(),
          "[map_server] 路网全部通道车体可通过（%.2fx%.2f + margin %.2f）",
          fp_.length, fp_.width, fp_.safe_margin);
    }
    publishRoutes(&routes_);
  }

  /// 用**发布后的地图** + 车体轮廓逐段校验通道（与 pnc_2d 规划器同一套判据）
  void checkRoutesFeasibility() {
    infeasible_edges_.clear();
    pnc_2d::CostMap2D cm;
    if (!cm.set(static_cast<int>(map_msg_.info.width),
                static_cast<int>(map_msg_.info.height),
                map_msg_.info.resolution, map_msg_.info.origin.position.x,
                map_msg_.info.origin.position.y, 0.0, map_msg_.data,
                frame_id_)) {
      RCLCPP_WARN(get_logger(),
                  "[map_server] 无法用当前地图校验路网（CostMap2D 构造失败）");
      return;
    }
    pnc_2d::FootprintCollisionChecker chk;
    chk.configure(fp_, hard_threshold_, unknown_as_occupied_);
    chk.setMap(&cm);
    std::vector<pnc_2d::RouteEdge> &edges = routes_.mutableEdges();
    for (size_t i = 0; i < edges.size(); ++i) {
      bool ok = true;
      const auto &pl = edges[i].polyline;
      for (size_t k = 0; k + 1 < pl.size() && ok; ++k) {
        // 朝向取该段方向；车体过不去 → 这条通道标记不可行
        if (chk.edgeInCollision(pl[k].x, pl[k].y, pl[k + 1].x, pl[k + 1].y))
          ok = false;
      }
      edges[i].feasible = ok;
      if (!ok)
        infeasible_edges_.push_back(static_cast<int>(i));
    }
  }

  /// 路网可视化：节点（按语义着色 + 名字）、通道（不可行标红）、单向箭头。
  /// 传 nullptr 只做"清理上一次发布的标记"（换站点/载入失败时用）。
  void publishRoutes(const pnc_2d::RouteGraph *g) {
    if (!routes_pub_)
      return;
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();
    std::map<std::string, int> counts;
    using Marker = visualization_msgs::msg::Marker;

    auto base = [&](const std::string &ns, int id, int type) {
      Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id_;
      m.ns = ns;
      m.id = id;
      m.type = type;
      m.action = Marker::ADD;
      m.lifetime = rclcpp::Duration::from_seconds(0.0);
      return m;
    };

    if (g != nullptr && g->valid()) {
      int id = 0;
      for (const pnc_2d::RouteNode &n : g->nodes()) {
        Marker m = base("routes_nodes", id, Marker::SPHERE);
        m.pose.position.x = n.x;
        m.pose.position.y = n.y;
        m.pose.position.z = 0.08;
        m.scale.x = m.scale.y = m.scale.z = 0.20;
        switch (n.type) {
        case pnc_2d::RouteNodeType::kStation:
          m.color.r = 0.16F;
          m.color.g = 0.63F;
          m.color.b = 0.16F;
          break;
        case pnc_2d::RouteNodeType::kCharge:
          m.color.r = 1.00F;
          m.color.g = 0.50F;
          m.color.b = 0.05F;
          break;
        case pnc_2d::RouteNodeType::kPark:
          m.color.r = 0.58F;
          m.color.g = 0.40F;
          m.color.b = 0.74F;
          break;
        default:
          m.color.r = 0.12F;
          m.color.g = 0.47F;
          m.color.b = 0.71F;
        }
        m.color.a = 1.0F;
        arr.markers.push_back(m);

        Marker t = base("routes_labels", id, Marker::TEXT_VIEW_FACING);
        t.pose.position.x = n.x;
        t.pose.position.y = n.y;
        t.pose.position.z = 0.50;
        t.scale.z = 0.30;
        t.color.r = t.color.g = t.color.b = 1.0F;
        t.color.a = 0.95F;
        t.text = n.name;
        arr.markers.push_back(t);
        ++id;
      }
      counts["routes_nodes"] = id;
      counts["routes_labels"] = id;

      id = 0;
      for (const pnc_2d::RouteEdge &e : g->edges()) {
        Marker m = base("routes_edges", id, Marker::LINE_STRIP);
        m.scale.x = 0.06;
        if (e.feasible) {
          m.color.r = 0.0F;
          m.color.g = 0.70F;
          m.color.b = 0.90F;
        } else {
          m.color.r = 0.90F;
          m.color.g = 0.10F;
          m.color.b = 0.10F;
        }
        m.color.a = 0.9F;
        for (const pnc_2d::Pose2D &p : e.polyline) {
          geometry_msgs::msg::Point pt;
          pt.x = p.x;
          pt.y = p.y;
          pt.z = 0.02;
          m.points.push_back(pt);
        }
        arr.markers.push_back(m);

        if (e.one_way) {
          const pnc_2d::Pose2D mid = e.pointAt(e.length * 0.5);
          Marker a = base("routes_oneway", id, Marker::ARROW);
          a.pose.position.x = mid.x;
          a.pose.position.y = mid.y;
          a.pose.position.z = 0.10;
          const double yaw = e.yawAt(e.length * 0.5);
          a.pose.orientation.z = std::sin(0.5 * yaw);
          a.pose.orientation.w = std::cos(0.5 * yaw);
          a.scale.x = 0.35;
          a.scale.y = 0.08;
          a.scale.z = 0.08;
          a.color.r = 1.0F;
          a.color.g = 0.65F;
          a.color.b = 0.0F;
          a.color.a = 1.0F;
          arr.markers.push_back(a);
        }
        ++id;
      }
      counts["routes_edges"] = id;
      counts["routes_oneway"] = id;
    }

    // 区域层（禁行 / 限速）与路网无关，单独拉：即使没有路网文件也要能看到区域
    appendZoneMarkers(arr, stamp, counts);

    // 清理：上一次发过、这一次没有的 id 要显式 DELETE（MarkerArray 不会自动删）
    for (const auto &kv : published_counts_) {
      const int from = counts.count(kv.first) ? counts[kv.first] : 0;
      for (int id = from; id < kv.second; ++id) {
        Marker d = base(kv.first, id, Marker::LINE_STRIP);
        d.action = Marker::DELETE;
        arr.markers.push_back(d);
      }
    }
    published_counts_ = counts;

    if (!arr.markers.empty())
      routes_pub_->publish(arr);
  }

  /// 周期性重发（仅 republish_interval>0 时启用；正常不建议开）
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

  /// 按需补发看门狗：订阅者数量变多 → 若有 Volatile 订阅者（收不到历史样本）
  /// 就补发一次。稳态下每次只做一次整数比较，不做图查询、不发布。
  void watchSubscribers() {
    const auto n = occ_pub_->get_subscription_count();
    if (n <= last_sub_count_) {
      last_sub_count_ = n;
      return;
    }
    last_sub_count_ = n;
    if (!has_map_)
      return;
    bool has_volatile = false;
    for (const auto &info : get_subscriptions_info_by_topic(topic_occ_)) {
      if (info.qos_profile().durability() ==
          rclcpp::DurabilityPolicy::Volatile) {
        has_volatile = true;
        break;
      }
    }
    if (!has_volatile)
      return; // 订阅者都是 transient_local：已自动收到历史样本，不用补发
    RCLCPP_DEBUG(get_logger(),
                 "[map_server] 新订阅者（共 %zu）中有 Volatile → 补发一次", n);
    republish();
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
  double republish_interval_{0.0};
  bool republish_on_new_subscriber_{true};
  bool unknown_as_free_{false};
  // 3D（M3 接口）
  bool publish_3d_{false};
  std::string pcd_file_;
  std::string topic_cloud_3d_;
  std::string cloud_frame_id_;
  double cloud_voxel_leaf_{0.0};
  bool require_3d_{false};

  // 路网（可选资产）
  std::string routes_file_; ///< 空 = <地图目录>/routes.yaml
  bool publish_routes_{true};
  std::string topic_routes_;
  pnc_2d::RouteGraph routes_;
  bool has_routes_{false};
  std::vector<int> infeasible_edges_;
  // 地图本体的膨胀层（障碍向外扩 inflate_ 米；见 loadAndPublish 里的注释）
  double inflate_{0.0};
  std::size_t dilated_cells_{0};
  // 区域层（禁行 / 限速）
  pnc_2d::ZoneSet zones_;
  bool burn_zones_{true};
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  double zone_inflate_{-1.0};
  double zone_inflate_used_{0.0};
  std::size_t burned_cells_{0};
  // 区域层几何的发布（latched，给局部等消费者；与"烧进全局图"互补）
  bool publish_zones_{true};
  std::string topic_zones_;
  rclcpp::Publisher<pnc_2d::msg::ZoneArray>::SharedPtr zones_pub_;
  pnc_2d::FootprintParams fp_;
  int hard_threshold_{80};
  bool unknown_as_occupied_{true};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      routes_pub_;
  std::map<std::string, int>
      published_counts_; ///< 各 ns 上次发过的标记数，用于 DELETE

  // 发布器/服务
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr meta_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_srv_;
  rclcpp::TimerBase::SharedPtr repub_timer_;
  rclcpp::TimerBase::SharedPtr watch_timer_;
  std::size_t last_sub_count_{0};

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
