// 全局规划节点（ROS 2）：
//   输入：全局地图（OccupancyGrid，latched）+ 当前位姿（Odometry）+
//   目标（PoseStamped） 输出：几何路径（nav_msgs/Path）+ 起终点箭头 +
//   起点处车体轮廓（MarkerArray）
//
// 触发方式：**事件驱动** —— 收到目标才规划一次（见 doc/pnc2d_dev_plan.md §2）。
// 说明：
//   · 本节点不做 TF 变换（与 map_server / perception 一致），但会校验
//   frame_id，
//     不一致时告警并拒用（静默混坐标系是最难查的一类 bug）；
//   · 允许未声明参数（allow_undeclared_parameters）+ **自动声明来自 yaml/命令行
//     的覆盖项**（automatically_declare_parameters_from_overrides）：于是
//     astar.* / footprint.* / common.* 这些"节点本身不认识、由规划库读取"的参数
//     也能真正生效。
//     ⚠ 只写 allow_undeclared_parameters(true) 是不够的：get_parameter_or() 对
//     **未声明**参数不会去查覆盖项，会静默返回代码默认值 —— 实测踩过（配了
//     footprint.length=0.99 却仍然用 0.70）。

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/global_planner.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "ros_param_reader.hpp"

namespace pnc_2d {
namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

} // namespace

class GlobalPlannerNode : public rclcpp::Node {
public:
  GlobalPlannerNode()
      : rclcpp::Node(
            "global_planner",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)) {
    planner_type_ = paramString("planner.type", "astar");
    frame_id_ = paramString("planner.frame_id", "map");
    topic_map_ = paramString("topics.map", "/global_map/occupancy");
    topic_odom_ = paramString("topics.odom", "/lightning/perception/pose");
    topic_goal_ = paramString("topics.goal", "/goal_pose");
    topic_path_ = paramString("topics.path", "/pnc_2d/global_path");
    topic_markers_ = paramString("topics.markers", "/pnc_2d/plan_markers");

    planner_ = createPlanner(planner_type_);
    if (!planner_) {
      std::string names;
      for (const auto &n : availablePlanners())
        names += (names.empty() ? "" : ", ") + n;
      RCLCPP_ERROR(get_logger(),
                   "[planner] 未知 planner.type='%s'（可用：%s）→ 回退 astar",
                   planner_type_.c_str(), names.c_str());
      planner_type_ = "astar";
      planner_ = createPlanner(planner_type_);
    }

    RosParamReader reader(*this);
    if (!planner_->configure(reader)) {
      RCLCPP_ERROR(get_logger(), "[planner] 参数装载失败");
    }

    // 把"这一次到底吃了哪些
    // yaml/命令行参数"打到日志：配置静默失效是最难查的问题
    {
      const auto &overrides =
          get_node_parameters_interface()->get_parameter_overrides();
      std::string keys;
      std::size_t shown = 0;
      for (const auto &kv : overrides) {
        if (kv.first == "use_sim_time" ||
            kv.first.rfind("qos_overrides.", 0) == 0)
          continue;
        if (shown++ >= 20) {
          keys += " ...";
          break;
        }
        keys += (keys.empty() ? "" : " ") + kv.first;
      }
      RCLCPP_INFO(get_logger(), "[planner] yaml/命令行参数 %zu 项：%s",
                  overrides.size(), keys.c_str());
    }

    // 地图用 transient_local：map_server 是 latched 发布，晚启动也能拿到
    const auto map_qos = rclcpp::QoS(1).transient_local();
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_map_, map_qos,
        std::bind(&GlobalPlannerNode::onMap, this, std::placeholders::_1));
    // 位姿：定位端（lightning run_loc_online）以 **BEST_EFFORT** 发布，而默认的
    // RELIABLE 订阅不会与之匹配（DDS 规则：offered 必须 ≥ requested）
    // → 现象是“一条位姿也收不到且无任何报错”，极难查（实测踩过）。
    // SensorDataQoS = best_effort + keep_last(5) + volatile：能匹配
    // BEST_EFFORT， 也兼容 RELIABLE 的发布端。
    odom_qos_ = rclcpp::SensorDataQoS();
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, odom_qos_,
        std::bind(&GlobalPlannerNode::onOdom, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_goal_, 1,
        std::bind(&GlobalPlannerNode::onGoal, this, std::placeholders::_1));

    // 路径/标记是"事件驱动的一次性产物"：为了让后连接的订阅者（RViz
    // 后开、控制器 后启）也能拿到最近一次结果，用 latched(transient_local)
    // 发布，保留最后一份。
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
        topic_path_, rclcpp::QoS(1).transient_local());
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        topic_markers_, rclcpp::QoS(1).transient_local());

    const FootprintParams &fp = planner_->footprintParams();
    RCLCPP_INFO(
        get_logger(),
        "[planner] type=%s frame=%s | 地图 %s | 位姿 %s | 目标 %s | 路径 %s",
        planner_type_.c_str(), frame_id_.c_str(), topic_map_.c_str(),
        topic_odom_.c_str(), topic_goal_.c_str(), topic_path_.c_str());
    RCLCPP_INFO(
        get_logger(),
        "[planner] QoS：地图 transient_local（匹配 map_server latched）| "
        "位姿 best_effort+keep_last(5)（匹配 lightning 定位）| 目标 reliable");
    RCLCPP_INFO(get_logger(),
                "[planner] footprint %s: %.2fx%.2f m（机体系）+ margin %.2f | "
                "内切半径 %.3f "
                "外接半径 %.3f | 边扫掠 %s | 快路径 %s",
                fp.enable ? "开" : "关", fp.length, fp.width, fp.safe_margin,
                fp.inscribedRadius(), fp.circumscribedRadius(),
                fp.check_edges ? "on" : "off", fp.fast_path ? "on" : "off");
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    auto map = std::make_shared<CostMap2D>();
    const auto &info = msg->info;
    const double yaw = yawFromQuaternion(info.origin.orientation);
    if (!map->set(static_cast<int>(info.width), static_cast<int>(info.height),
                  info.resolution, info.origin.position.x,
                  info.origin.position.y, yaw, msg->data,
                  msg->header.frame_id)) {
      RCLCPP_ERROR(
          get_logger(),
          "[planner] 地图无效：%ux%u @ %.3f m，data 长度 %zu（应为 %u）",
          info.width, info.height, info.resolution, msg->data.size(),
          info.width * info.height);
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(),
                  "[planner] 地图 frame='%s' 与 planner.frame_id='%s' "
                  "不一致，仍按后者解释",
                  msg->header.frame_id.c_str(), frame_id_.c_str());
    }
    // map_server 会按 republish_interval 周期重发**同一张**图（让 RViz 这种
    // Volatile 订阅者也收得到）。同一张图没必要反复重建距离场与方向偏移，
    // 以 map_load_time 为主键去重；真正换图时 map_load_time 会变。
    if (has_map_meta_ && msg->info.map_load_time == map_load_time_ &&
        msg->info.width == map_width_ && msg->info.height == map_height_ &&
        msg->info.resolution == map_resolution_) {
      return;
    }
    planner_->setCostMap(map);
    has_map_ = true;
    has_map_meta_ = true;
    map_load_time_ = msg->info.map_load_time;
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    map_resolution_ = msg->info.resolution;
    RCLCPP_INFO(get_logger(),
                "[planner] 收到地图 %dx%d @ %.3f m = %.1fx%.1f m | origin "
                "(%.3f, %.3f, yaw %.4f)",
                map->width(), map->height(), map->resolution(),
                map->width() * map->resolution(),
                map->height() * map->resolution(), map->originX(),
                map->originY(), map->originYaw());

    // 路网类算法：把路网信息报出来，并把全网画到 RViz
    if (const RouteGraph *g = planner_->routeGraph();
        g != nullptr && g->valid()) {
      RCLCPP_INFO(get_logger(), "[planner] 路网 %s", g->summary().c_str());
      for (const std::string &w : g->warnings()) {
        RCLCPP_WARN(get_logger(), "[planner] 路网：%s", w.c_str());
      }
      publishMarkers(nullptr, nullptr);
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    start_.x = msg->pose.pose.position.x;
    start_.y = msg->pose.pose.position.y;
    start_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    start_.has_yaw = true;
    odom_frame_ = msg->header.frame_id;
    has_odom_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!has_map_ || planner_->costMap() == nullptr) {
      RCLCPP_WARN(get_logger(), "[planner] 还没收到地图，忽略本次目标");
      return;
    }
    if (!has_odom_) {
      // 区分"定位没起"与"QoS 不匹配"：后者最坑 —— 话题上有发布者、数据也在刷，
      // 但 DDS 因为 reliability 不兼容根本不投递，且没有任何报错。
      const std::size_t pubs = count_publishers(topic_odom_);
      RCLCPP_WARN(get_logger(),
                  "[planner] 还没收到位姿（%s），无法确定起点，忽略本次目标 | "
                  "该话题发布者 %zu 个%s",
                  topic_odom_.c_str(), pubs,
                  pubs == 0
                      ? "（定位节点似乎没在跑）"
                      : "（有发布者 → 多为 QoS 不匹配：本节点订阅 best_effort，"
                        "若发布端不是 best_effort/reliable 兼容关系，请用 "
                        "ros2 topic info -v 核对）");
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(),
                  "[planner] 目标 frame='%s' != '%s'，拒用本次目标（请把 RViz "
                  "的 Fixed Frame 设为 %s）",
                  msg->header.frame_id.c_str(), frame_id_.c_str(),
                  frame_id_.c_str());
      return;
    }
    if (!odom_frame_.empty() && odom_frame_ != frame_id_) {
      RCLCPP_WARN(get_logger(),
                  "[planner] 位姿 frame='%s' != '%s'，拒用本次目标",
                  odom_frame_.c_str(), frame_id_.c_str());
      return;
    }

    PlanRequest req;
    req.start = start_;
    req.goal.x = msg->pose.position.x;
    req.goal.y = msg->pose.position.y;
    req.goal.yaw = yawFromQuaternion(msg->pose.orientation);
    req.goal.has_yaw = true;

    const double dist =
        std::hypot(req.goal.x - req.start.x, req.goal.y - req.start.y);
    RCLCPP_INFO(get_logger(),
                "[planner] 规划请求：(% .2f, % .2f, % .1f°) → (% .2f, % .2f, "
                "%.1f°)，直线距离 %.2f m",
                req.start.x, req.start.y, req.start.yaw * 180.0 / M_PI,
                req.goal.x, req.goal.y, req.goal.yaw * 180.0 / M_PI, dist);

    const PlanResult result = planner_->plan(req);
    publishResult(result, req);
  }

  void publishResult(const PlanResult &result, const PlanRequest &req) {
    const auto &st = result.stats;
    if (!result.ok()) {
      RCLCPP_ERROR(get_logger(),
                   "[planner] 规划失败：%s（%s）| 窗口尝试 %d 次 | 扩展节点 "
                   "%ld | 耗时 %.1f ms",
                   toString(result.status), result.message.c_str(),
                   st.windows_tried, st.expanded_nodes, st.plan_time_ms);
      publishMarkers(&result, &req); // 只画起终点，不画路径
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(result.path.size());
    for (const auto &p : result.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = quaternionFromYaw(p.has_yaw ? p.yaw : 0.0);
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);

    RCLCPP_INFO(get_logger(),
                "[planner] 规划成功：%zu 点 / %.2f m | 耗时 %.1f ms | 扩展节点 "
                "%ld（发现 %ld，峰值开放集 %ld）"
                "| 窗口尝试 %d 次 | 完整 footprint 检查 %ld 次",
                result.path.size(), st.path_length, st.plan_time_ms,
                st.expanded_nodes, st.discovered_nodes, st.max_open_set,
                st.windows_tried, st.footprint_full_checks);
    publishMarkers(&result, &req);
  }

  void publishMarkers(const PlanResult *result, const PlanRequest *req) {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();
    appendRouteNetwork(arr, stamp);

    auto arrow = [&](int id, const Pose2D &p, float r, float g, float b,
                     const char *ns) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id_;
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = 0.05;
      m.pose.orientation = quaternionFromYaw(p.has_yaw ? p.yaw : 0.0);
      m.scale.x = 0.45;
      m.scale.y = 0.08;
      m.scale.z = 0.08;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = 1.0;
      m.lifetime = rclcpp::Duration::from_seconds(0.0);
      arr.markers.push_back(m);
    };
    if (req) {
      arrow(0, req->start, 0.1F, 1.0F, 0.1F, "start");
      arrow(1, req->goal, 1.0F, 0.1F, 0.1F, "goal");
    }

    // 起点处的车体轮廓：把"规划器认为的车体"画出来，便于诊断"为什么这里过不去"
    const FootprintParams &fp = planner_->footprintParams();
    if (req && fp.enable) {
      double corners[8] = {0};
      planner_->collisionChecker().footprintCorners(
          req->start.x, req->start.y, req->start.has_yaw ? req->start.yaw : 0.0,
          corners);
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id_;
      m.ns = "footprint";
      m.id = 2;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.03;
      m.color.r = 0.2F;
      m.color.g = 0.6F;
      m.color.b = 1.0F;
      m.color.a = 0.9F;
      for (int i = 0; i < 4; ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = corners[2 * i];
        pt.y = corners[2 * i + 1];
        pt.z = 0.05;
        m.points.push_back(pt);
      }
      m.points.push_back(m.points.front()); // 闭合
      arr.markers.push_back(m);
    }
    (void)result;

    pub_markers_->publish(arr);
  }

  /// 把路网画到 RViz：节点（按语义着色 + 名字标签）、通道（车体过不去的标红）、
  /// 当前通路高亮。路网是静态资产，随地图一起发一次即可（latched，晚开的 RViz
  /// 也看得到）。
  void appendRouteNetwork(visualization_msgs::msg::MarkerArray &arr,
                          const rclcpp::Time &stamp) {
    const RouteGraph *g = planner_->routeGraph();
    if (g == nullptr || !g->valid())
      return;
    using Marker = visualization_msgs::msg::Marker;

    auto base = [&](const char *ns, int id, int type) {
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

    int id = 0;
    for (const RouteNode &n : g->nodes()) {
      Marker m = base("route_nodes", id, Marker::SPHERE);
      m.pose.position.x = n.x;
      m.pose.position.y = n.y;
      m.pose.position.z = 0.08;
      m.scale.x = 0.18;
      m.scale.y = 0.18;
      m.scale.z = 0.18;
      switch (n.type) {
      case RouteNodeType::kStation:
        m.color.r = 0.16F;
        m.color.g = 0.63F;
        m.color.b = 0.16F;
        break;
      case RouteNodeType::kCharge:
        m.color.r = 1.00F;
        m.color.g = 0.50F;
        m.color.b = 0.05F;
        break;
      case RouteNodeType::kPark:
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

      Marker t = base("route_node_labels", id, Marker::TEXT_VIEW_FACING);
      t.pose.position.x = n.x;
      t.pose.position.y = n.y;
      t.pose.position.z = 0.45;
      t.scale.z = 0.28;
      t.color.r = 1.0F;
      t.color.g = 1.0F;
      t.color.b = 1.0F;
      t.color.a = 0.95F;
      t.text = n.name;
      arr.markers.push_back(t);
      ++id;
    }

    id = 0;
    for (const RouteEdge &e : g->edges()) {
      Marker m = base("route_edges", id, Marker::LINE_STRIP);
      m.scale.x = 0.05;
      if (e.feasible) {
        m.color.r = 0.0F;
        m.color.g = 0.70F;
        m.color.b = 0.90F;
      } else {
        m.color.r = 0.90F;
        m.color.g = 0.10F;
        m.color.b = 0.10F; // 车体过不去
      }
      m.color.a = 0.9F;
      for (const Pose2D &p : e.polyline) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.02;
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);

      if (e.one_way) { // 单向：在中点放个箭头
        const Pose2D mid = e.pointAt(e.length * 0.5);
        Marker a = base("route_oneway", id, Marker::ARROW);
        a.pose.position.x = mid.x;
        a.pose.position.y = mid.y;
        a.pose.position.z = 0.10;
        a.pose.orientation = quaternionFromYaw(e.yawAt(e.length * 0.5));
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

    // 当前通路高亮（上一次规划实际走过的通道）
    // ⚠ MarkerArray 不会自动删掉"这次没出现的旧标记"：规划失败后高亮会残留在
    // RViz，
    //   所以比上次少出来的 id 要显式发 DELETE。
    id = 0;
    for (const int ei : planner_->activeRouteEdges()) {
      if (ei < 0 || ei >= static_cast<int>(g->edges().size()))
        continue;
      const RouteEdge &e = g->edges()[static_cast<std::size_t>(ei)];
      Marker m = base("route_active", id++, Marker::LINE_STRIP);
      m.scale.x = 0.12;
      m.color.r = 1.0F;
      m.color.g = 0.95F;
      m.color.b = 0.2F;
      m.color.a = 1.0F;
      for (const Pose2D &p : e.polyline) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.06;
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    }
    for (int stale = id; stale < active_marker_count_; ++stale) {
      Marker d = base("route_active", stale, Marker::LINE_STRIP);
      d.action = Marker::DELETE;
      arr.markers.push_back(d);
    }
    active_marker_count_ = id;
  }

  /// 读字符串参数：来自 yaml/命令行的键已经被
  /// automatically_declare_parameters_from_overrides 声明过了，此时**不能**再
  /// declare_parameter（会抛
  /// ParameterAlreadyDeclaredException）；没有配置过的键 才补一个默认声明，让
  /// ros2 param get/list 也能看到。
  std::string paramString(const std::string &key, const std::string &def) {
    if (!has_parameter(key))
      return declare_parameter<std::string>(key, def);
    const rclcpp::Parameter p = get_parameter(key);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      return p.as_string();
    RCLCPP_WARN(get_logger(),
                "[planner] 参数 %s 类型应为 string，实际 %s → 用默认值 %s",
                key.c_str(), p.get_type_name().c_str(), def.c_str());
    return def;
  }

  std::string planner_type_;
  std::string frame_id_;
  std::string topic_map_;
  std::string topic_odom_;
  std::string topic_goal_;
  std::string topic_path_;
  std::string topic_markers_;

  std::unique_ptr<GlobalPlanner> planner_;
  rclcpp::QoS odom_qos_{rclcpp::SensorDataQoS()};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_markers_;

  Pose2D start_;
  std::string odom_frame_;
  bool has_map_{false};
  bool has_odom_{false};
  // 已接受地图的元信息（用于识别 map_server 的周期性重发）
  bool has_map_meta_{false};
  int active_marker_count_{0}; ///< 上次发布的通路高亮个数（用于发 DELETE）
  builtin_interfaces::msg::Time map_load_time_;
  uint32_t map_width_{0};
  uint32_t map_height_{0};
  double map_resolution_{0.0};
};

} // namespace pnc_2d

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_2d::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
