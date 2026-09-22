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
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/global_planner.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "pnc_2d/msg/planner_status.hpp"
#include "pnc_2d/srv/plan_path.hpp"
#include "pnc_2d/srv/switch_planner.hpp"
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
    topic_status_ = paramString("topics.status", "/pnc_2d/global_status");
    // 到达目标自动清空：**默认关**。"任务是否结束"是状态机的判断（P4），
    // 规划器不该替它做决定；需要这个便利行为时把它打开即可。
    clear_on_reach_ = paramBool("clear.auto_on_goal_reached", false);
    reach_tol_ = paramDouble("clear.goal_tolerance", 0.30);

    // 创建 + 配置（启动、热切换、热重载共用同一段逻辑）
    {
      std::string err;
      if (!buildPlanner(planner_type_, err)) {
        std::string names;
        for (const auto &n : availablePlanners())
          names += (names.empty() ? "" : ", ") + n;
        RCLCPP_ERROR(get_logger(),
                     "[planner] %s（可用：%s）→ 回退 astar", err.c_str(),
                     names.c_str());
        planner_type_ = "astar";
        buildPlanner(planner_type_, err);
      }
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
    // 状态同样是 latched：晚启动的状态机/监控也能拿到"最近一次规划到底成不成"
    pub_status_ = create_publisher<pnc_2d::msg::PlannerStatus>(
        topic_status_, rclcpp::QoS(1).transient_local());

    // 显式清除入口：路径/标记是 latched 的，"当前没有有效计划"必须有人能主动说
    // 一声（状态机切模式、RViz 手工清、任务取消都用它）。
    srv_clear_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_path",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          clearPlan("服务调用");
          res->success = true;
          res->message = "路径与规划标记已清空";
        });

    // 状态机（manager）主导的规划入口：同步返回"成/败 + 路径 + 原因"。
    // 与 /goal_pose 话题触发共用同一段规划逻辑（见 onPlanPathRequest）。
    srv_plan_path_ = create_service<pnc_2d::srv::PlanPath>(
        "~/plan_path",
        std::bind(&GlobalPlannerNode::onPlanPathRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 运行时热切换算法（不重启）。P4 的状态机切模式时就用它。
    srv_switch_ = create_service<pnc_2d::srv::SwitchPlanner>(
        "~/switch_planner",
        [this](const std::shared_ptr<pnc_2d::srv::SwitchPlanner::Request> req,
               std::shared_ptr<pnc_2d::srv::SwitchPlanner::Response> res) {
          std::string err;
          if (switchPlanner(req->type, err)) {
            res->success = true;
            res->message = "已切换到 " + planner_type_;
          } else {
            res->success = false;
            res->message = err;
            RCLCPP_ERROR(get_logger(), "[planner] 热切换 → '%s' 失败：%s",
                         req->type.c_str(), err.c_str());
          }
        });

    // 热重载参数：用当前参数值重新 configure（换 footprint / 剪枝策略不用重启）
    srv_reload_ = create_service<std_srvs::srv::Trigger>(
        "~/reload_params",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::string err;
          if (reloadParams(err)) {
            res->success = true;
            res->message = "已按当前参数重新装载 " + planner_type_;
          } else {
            res->success = false;
            res->message = err;
          }
        });

    const FootprintParams &fp = planner_->footprintParams();
    RCLCPP_INFO(
        get_logger(),
        "[planner] type=%s frame=%s | 地图 %s | 位姿 %s | 目标 %s | 路径 %s | 状态 %s",
        planner_type_.c_str(), frame_id_.c_str(), topic_map_.c_str(),
        topic_odom_.c_str(), topic_goal_.c_str(), topic_path_.c_str(),
        topic_status_.c_str());
    RCLCPP_INFO(get_logger(),
                "[planner] 清空策略：失败时自动清空 | 到达目标(≤ %.2f m)自动清空：%s%s "
                "| 也可调服务 ~/clear_path",
                reach_tol_, clear_on_reach_ ? "开" : "关",
                clear_on_reach_
                    ? ""
                    : "（clear.auto_on_goal_reached:=true 可打开；默认由状态机决定"
                      "何时清）");
    RCLCPP_INFO(
        get_logger(),
        "[planner] QoS：地图 transient_local（匹配 map_server latched）| "
        "位姿 best_effort+keep_last(5)（匹配 lightning 定位）| 目标 reliable");
    logFootprintInfo();

    // 启动清场：把上一个进程（或上一次运行）留在 latched 话题 / RViz 里的
    // 路径与通路高亮清掉，避免"换了算法重启，旧路网路径还挂在那里"。
    //
    // ⚠ 必须**重复发几次**：RViz 的 MarkerArray 订阅是 VOLATILE 的，只收
    // "writer 与 reader 匹配完成之后"发布的消息；启动瞬间发一次会早于 DDS
    // discovery，被直接丢掉（实测就是这个问题：旧高亮留在 RViz 里）。
    // 一旦有过规划结果就不再重发，避免反过来把刚发的新路径清掉。
    timer_startup_cleanup_ = create_wall_timer(
        std::chrono::milliseconds(500), [this]() {
          if (first_result_seen_)
            return;
          publishStartupCleanup();
          if (++startup_cleanup_ticks_ >= 6) // 3 s 足够覆盖发现时间
            timer_startup_cleanup_->cancel();
        });
    publishStartupCleanup();

    // planner.type 支持**热切换**：按参数改会真的重建规划器（见 switchPlanner）。
    // 失败时拒绝这次参数修改（successful=false）—— 保证"参数值 = 实际生效值"，
    // 否则会出现"以为切成 A* 了，结果还在走路网"这种最难查的误解。
    cb_set_params_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &ps) {
          rcl_interfaces::msg::SetParametersResult r;
          r.successful = true;
          for (const auto &p : ps) {
            if (p.get_name() != "planner.type")
              continue;
            const std::string want =
                p.get_type() == rclcpp::ParameterType::PARAMETER_STRING
                    ? p.as_string()
                    : std::string();
            if (want == planner_type_)
              continue;
            std::string err;
            if (switchPlanner(want, err)) {
              RCLCPP_INFO(get_logger(), "[planner] 已按参数热切换到 '%s'",
                          planner_type_.c_str());
            } else {
              r.successful = false;
              r.reason = "热切换失败：" + err + "（仍在使用 " + planner_type_ +
                         "）";
              RCLCPP_ERROR(get_logger(),
                           "[planner] planner.type → '%s' 失败：%s",
                           want.c_str(), err.c_str());
            }
          }
          return r;
        });
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
    map_ = map; // 热切换/热重载时要把同一张图交给新规划器
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
    logRoutesInfo();
    publishMarkers(nullptr, nullptr);
  }

  /// 车体信息：热重载 footprint.* 后要能看到改没改成功
  void logFootprintInfo() {
    const FootprintParams &fp = planner_->footprintParams();
    RCLCPP_INFO(get_logger(),
                "[planner] footprint %s: %.2fx%.2f m（机体系）+ margin %.2f | "
                "内切半径 %.3f 外接半径 %.3f | 边扫掠 %s | 快路径 %s",
                fp.enable ? "开" : "关", fp.length, fp.width, fp.safe_margin,
                fp.inscribedRadius(), fp.circumscribedRadius(),
                fp.check_edges ? "on" : "off", fp.fast_path ? "on" : "off");
  }

  /// 路网信息（有路网才报）：节点数/通道数/总长 + 每条过不去的通道点名。
  /// 地图到达、热切换、热重载后都会调一次。
  void logRoutesInfo() {
    const RouteGraph *g = planner_->routeGraph();
    if (g == nullptr || !g->valid())
      return;
    RCLCPP_INFO(get_logger(), "[planner] 路网 %s", g->summary().c_str());
    for (const std::string &w : g->warnings())
      RCLCPP_WARN(get_logger(), "[planner] 路网：%s", w.c_str());
    std::string bad;
    int n_bad = 0;
    for (const RouteEdge &e : g->edges()) {
      if (e.feasible)
        continue;
      ++n_bad;
      bad += (bad.empty() ? "" : ", ") +
             g->nodes()[static_cast<std::size_t>(e.from)].name + "->" +
             g->nodes()[static_cast<std::size_t>(e.to)].name;
    }
    if (n_bad > 0)
      RCLCPP_WARN(get_logger(),
                  "[planner] 车体过不去的通道 %d 条（不参与路由）：%s", n_bad,
                  bad.c_str());
  }

  /// 创建 + 配置一个算法实例（不动现有 planner_）。启动/热切换/热重载共用。
  bool buildPlanner(const std::string &type, std::string &err,
                    std::unique_ptr<GlobalPlanner> &out) {
    std::unique_ptr<GlobalPlanner> next = createPlanner(type);
    if (!next) {
      std::string names;
      for (const auto &n : availablePlanners())
        names += (names.empty() ? "" : ", ") + n;
      err = "未知 planner.type='" + type + "'（可用：" + names + "）";
      return false;
    }
    RosParamReader reader(*this);
    if (!next->configure(reader)) {
      err = "参数装载失败（" + type + "）";
      return false;
    }
    out = std::move(next);
    return true;
  }

  bool buildPlanner(const std::string &type, std::string &err) {
    std::unique_ptr<GlobalPlanner> next;
    if (!buildPlanner(type, err, next))
      return false;
    planner_ = std::move(next);
    planner_type_ = type;
    return true;
  }

  /// 运行时热切换算法：构建 → 交地图（路网模式会立刻重跑可行性校验）→
  /// 换指针 → 清空旧路径/标记 → 重报路网信息。失败时保留旧算法。
  ///
  /// 线程前提：节点用默认的**单线程** executor，goal/参数/服务回调互相串行，
  /// 不存在"规划到一半把 planner_ 换掉"的重入。若将来改用
  /// MultiThreadedExecutor，这里（以及 planner_ 的访问）必须加锁。
  bool switchPlanner(const std::string &type, std::string &err) {
    if (type == planner_type_) {
      err = "已在使用 " + type;
      return true;
    }
    std::unique_ptr<GlobalPlanner> next;
    if (!buildPlanner(type, err, next))
      return false;
    if (map_)
      next->setCostMap(map_); // 路网模式在这里做可行性校验
    planner_ = std::move(next);
    planner_type_ = type;
    clearPlan("切换算法");
    logFootprintInfo();
    logRoutesInfo();
    RCLCPP_INFO(get_logger(), "[planner] 已热切换到 %s", planner_type_.c_str());
    return true;
  }

  /// 热重载参数：用当前参数值重新构建**同类**算法（footprint / 剪枝策略等
  /// 改了不用重启）。失败时保留旧实例。
  bool reloadParams(std::string &err) {
    std::unique_ptr<GlobalPlanner> next;
    if (!buildPlanner(planner_type_, err, next))
      return false;
    if (map_)
      next->setCostMap(map_);
    planner_ = std::move(next);
    clearPlan("热重载参数");
    logFootprintInfo();
    logRoutesInfo();
    RCLCPP_INFO(get_logger(), "[planner] 已按当前参数重新装载 %s",
                planner_type_.c_str());
    return true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    start_.x = msg->pose.pose.position.x;
    start_.y = msg->pose.pose.position.y;
    start_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    start_.has_yaw = true;
    odom_frame_ = msg->header.frame_id;
    has_odom_ = true;

    // 到达目标就收掉路径：路径是事件驱动产物，任务完成了就不该再挂在
    // latched 话题上（RViz 会一直显示）。
    // 只在"规划时本来就离目标更远"时生效，避免目标就在车边时刚发就清。
    if (clear_on_reach_ && has_active_plan_ && plan_start_dist_ > reach_tol_) {
      const double d = std::hypot(start_.x - active_goal_.x,
                                  start_.y - active_goal_.y);
      if (d <= reach_tol_)
        clearPlan("已到达目标");
    }
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
                "[planner] 规划请求（话题）：(% .2f, % .2f, %.1f°) → (% .2f, "
                "% .2f, %.1f°)，直线距离 %.2f m",
                req.start.x, req.start.y, req.start.yaw * 180.0 / M_PI,
                req.goal.x, req.goal.y, req.goal.yaw * 180.0 / M_PI, dist);

    publishResult(planner_->plan(req), req);
  }

  /// 输入检查：通过返回 true；不通过时往 `err` 写原因（同时打日志）。
  /// 话题触发与服务触发**共用**，避免两条路径的前置条件慢慢分叉。
  bool inputReady(std::string &err, const char *who) {
    if (!has_map_ || planner_->costMap() == nullptr) {
      err = "还没收到地图（" + topic_map_ + "）";
      RCLCPP_WARN(get_logger(), "[planner] %s：%s，忽略本次目标", who,
                  err.c_str());
      return false;
    }
    if (!has_odom_) {
      // 区分"定位没起"与"QoS 不匹配"：后者最坑 —— 话题上有发布者、数据也在刷，
      // 但 DDS 因为 reliability 不兼容根本不投递，且没有任何报错。
      const std::size_t pubs = count_publishers(topic_odom_);
      err = "还没收到位姿（" + topic_odom_ + "），无法确定起点";
      RCLCPP_WARN(get_logger(),
                  "[planner] %s：%s | 该话题发布者 %zu 个%s", who, err.c_str(),
                  pubs,
                  pubs == 0
                      ? "（定位节点似乎没在跑）"
                      : "（有发布者 → 多为 QoS 不匹配：本节点订阅 best_effort，"
                        "请用 ros2 topic info -v 核对两端 reliability）");
      return false;
    }
    if (!odom_frame_.empty() && odom_frame_ != frame_id_) {
      err = "位姿 frame='" + odom_frame_ + "' != '" + frame_id_ +
            "'，拒用（本节点不做 TF 变换）";
      RCLCPP_WARN(get_logger(), "[planner] %s：%s", who, err.c_str());
      return false;
    }
    return true;
  }

  /// 服务入口：manager（状态机）主导的规划。
  ///
  /// 与话题触发的差别只有两点，规划逻辑完全共用：
  ///   · start 可以用请求里给的（默认用最近一次定位位姿，它更新）；
  ///   · `publish_result=false` 时只看结果、不动 latched 话题（"先试算"场景）。
  void onPlanPathRequest(
      const std::shared_ptr<pnc_2d::srv::PlanPath::Request> req,
      std::shared_ptr<pnc_2d::srv::PlanPath::Response> res) {
    res->success = false;

    if (!req->goal.header.frame_id.empty() &&
        req->goal.header.frame_id != frame_id_) {
      res->status = static_cast<uint8_t>(PlannerStatus::kInvalidInput);
      res->status_name = toString(PlannerStatus::kInvalidInput);
      res->message = "目标 frame='" + req->goal.header.frame_id +
                     "' != planner.frame_id='" + frame_id_ + "'";
      RCLCPP_WARN(get_logger(), "[planner] 服务请求被拒：%s",
                  res->message.c_str());
      return;
    }

    std::string err;
    if (!inputReady(err, "服务请求")) {
      res->status = static_cast<uint8_t>(PlannerStatus::kNotInitialized);
      res->status_name = toString(PlannerStatus::kNotInitialized);
      res->message = err;
      return;
    }

    PlanRequest pr;
    // 默认用最近一次定位位姿（它总比调用方缓存的更新）
    pr.start = start_;
    if (!req->use_current_pose &&
        (req->start.pose.position.x != 0.0 ||
         req->start.pose.position.y != 0.0)) {
      // 调用方显式指定了起点：只在"给得像样"（不是默认的 0,0）时才采信，
      // 否则会把 (0,0) 当成一个真实起点（那是地图外或墙里）→ 报莫名其妙的无解。
      pr.start.x = req->start.pose.position.x;
      pr.start.y = req->start.pose.position.y;
      pr.start.yaw = yawFromQuaternion(req->start.pose.orientation);
      pr.start.has_yaw = true;
    }
    pr.goal.x = req->goal.pose.position.x;
    pr.goal.y = req->goal.pose.position.y;
    pr.goal.yaw = yawFromQuaternion(req->goal.pose.orientation);
    pr.goal.has_yaw = true;

    RCLCPP_INFO(get_logger(),
                "[planner] 规划请求（服务）：(% .2f, % .2f) → (% .2f, % .2f)，"
                "直线距离 %.2f m | 发布结果到话题：%s",
                pr.start.x, pr.start.y, pr.goal.x, pr.goal.y,
                std::hypot(pr.goal.x - pr.start.x, pr.goal.y - pr.start.y),
                req->publish_result ? "是" : "否");

    const PlanResult result = planner_->plan(pr);
    if (req->publish_result)
      publishResult(result, pr);

    const auto &st = result.stats;
    res->status = static_cast<uint8_t>(result.status);
    res->status_name = toString(result.status);
    res->message = result.message;
    res->success = result.ok();
    res->plan_time_ms = st.plan_time_ms;
    res->expanded_nodes = static_cast<int32_t>(st.expanded_nodes);
    res->windows_tried = st.windows_tried;
    // 走廊（P5.3）：只有路网规划器会填 has_corridor；A* 始终是自由空间。
    // 这里只做透传 + 一个一致性修正（strict 必须与 half_width 一致，避免调用方
    // 自己再判一次而判歪）。
    res->has_corridor = result.has_corridor;
    res->corridor_half_width = result.corridor_half_width;
    res->corridor_speed_limit = result.corridor_speed_limit;
    res->strict_corridor = result.strictCorridor();
    res->route_edges.assign(result.route_edges.begin(), result.route_edges.end());
    res->path.header.stamp = now();
    res->path.header.frame_id = frame_id_;
    for (const auto &p : result.path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = res->path.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = quaternionFromYaw(p.has_yaw ? p.yaw : 0.0);
      res->path.poses.push_back(ps);
    }
  }

  void publishResult(const PlanResult &result, const PlanRequest &req) {
    const auto &st = result.stats;
    first_result_seen_ = true; // 有结果了，启动清场停止重发
    if (timer_startup_cleanup_)
      timer_startup_cleanup_->cancel();
    publishStatus(result, req);
    if (!result.ok()) {
      RCLCPP_ERROR(get_logger(),
                   "[planner] 规划失败：%s（%s）| 窗口尝试 %d 次 | 扩展节点 "
                   "%ld | 耗时 %.1f ms",
                   toString(result.status), result.message.c_str(),
                   st.windows_tried, st.expanded_nodes, st.plan_time_ms);
      // 显式清空 latched 路径：否则上层/RViz 会继续显示上一条**已过期**的成功
      // 路径，把"规划失败"误当成"路径还有效"（真的踩过）。
      publishEmptyPath();
      has_active_plan_ = false;
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
                "| 窗口尝试 %d 次 | 完整 footprint 检查 %ld 次%s%s",
                result.path.size(), st.path_length, st.plan_time_ms,
                st.expanded_nodes, st.discovered_nodes, st.max_open_set,
                st.windows_tried, st.footprint_full_checks,
                result.message.empty() ? "" : " | ",
                result.message.c_str());
    has_active_plan_ = true;
    active_goal_ = req.goal;
    plan_start_dist_ = std::hypot(req.goal.x - req.start.x,
                                  req.goal.y - req.start.y);
    publishMarkers(&result, &req);
  }

  /// 发一条空 Path：latched 话题上"当前没有有效路径"就靠这个表达
  void publishEmptyPath() {
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = frame_id_;
    pub_path_->publish(empty);
  }

  /// 清空"当前计划"：空路径 + 删掉起终点箭头/车体轮廓/通路高亮。
  /// 路网本体（节点/通道）是静态资产，保留。
  void clearPlan(const char *why) {
    const bool had = has_active_plan_;
    publishEmptyPath();
    planner_->reset(); // 把上次的通路高亮也一起收掉
    publishClearMarkers();
    has_active_plan_ = false;
    if (had)
      RCLCPP_INFO(get_logger(), "[planner] 已清空当前路径（%s）", why);
  }

  void publishClearMarkers() {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();
    appendRouteNetwork(arr, stamp); // 路网照旧显示
    auto del = [&](const char *ns, int id, int32_t type) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = frame_id_;
      m.ns = ns;
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::DELETE;
      arr.markers.push_back(m);
    };
    del("start", 0, visualization_msgs::msg::Marker::ARROW);
    del("goal", 1, visualization_msgs::msg::Marker::ARROW);
    del("footprint", 2, visualization_msgs::msg::Marker::LINE_STRIP);
    // 当前算法如果没有路网（例如热切换到 A*），本节点之前画的路网也要收掉
    const RouteGraph *g = planner_->routeGraph();
    if (g == nullptr || !g->valid()) {
      for (int i = 0; i < kStaleMarkerIds; ++i) {
        del("route_nodes", i, visualization_msgs::msg::Marker::SPHERE);
        del("route_node_labels", i,
            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        del("route_edges", i, visualization_msgs::msg::Marker::LINE_STRIP);
        del("route_oneway", i, visualization_msgs::msg::Marker::ARROW);
      }
    }
    // route_active 的 id 从 0 连续编号，但**上一个进程**发过几条本进程并不知道
    // （active_marker_count_ 重启后从 0 开始），所以这里按一个足够大的范围全部删掉。
    for (int i = 0; i < kStaleMarkerIds; ++i)
      del("route_active", i, visualization_msgs::msg::Marker::LINE_STRIP);
    active_marker_count_ = 0;
    pub_markers_->publish(arr);
  }

  /// 启动时清一次场。路径/标记都是 latched 且 RViz 不会在发布者消失后自己清，
  /// 所以"杀掉旧节点、用另一种算法重启"时，RViz 里会继续显示上一个进程留下的
  /// 路径与通路高亮。启动时主动发一条空路径 + DELETE 就能清干净。
  void publishStartupCleanup() {
    publishEmptyPath();
    planner_->reset();
    publishClearMarkers();
  }
  /// 状态上报（成功/失败都发，latched）：供状态机与上层错误上报使用
  void publishStatus(const PlanResult &result, const PlanRequest &req) {
    pnc_2d::msg::PlannerStatus s;
    s.header.stamp = now();
    s.header.frame_id = frame_id_;
    s.status = static_cast<uint8_t>(result.status);
    s.status_name = toString(result.status);
    s.message = result.message;
    s.success = result.ok();
    s.start_x = req.start.x;
    s.start_y = req.start.y;
    s.goal_x = req.goal.x;
    s.goal_y = req.goal.y;
    s.path_points = static_cast<uint32_t>(result.path.size());
    s.path_length_m = result.stats.path_length;
    s.plan_time_ms = result.stats.plan_time_ms;
    s.expanded_nodes = static_cast<int32_t>(result.stats.expanded_nodes);
    s.windows_tried = static_cast<int32_t>(result.stats.windows_tried);
    pub_status_->publish(s);
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

  bool paramBool(const std::string &key, bool def) {
    if (!has_parameter(key))
      return declare_parameter<bool>(key, def);
    const rclcpp::Parameter p = get_parameter(key);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      return p.as_bool();
    RCLCPP_WARN(get_logger(),
                "[planner] 参数 %s 类型应为 bool，实际 %s → 用默认值 %s",
                key.c_str(), p.get_type_name().c_str(), def ? "true" : "false");
    return def;
  }

  double paramDouble(const std::string &key, double def) {
    if (!has_parameter(key))
      return declare_parameter<double>(key, def);
    const rclcpp::Parameter p = get_parameter(key);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      return p.as_double();
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      return static_cast<double>(p.as_int());
    RCLCPP_WARN(get_logger(),
                "[planner] 参数 %s 类型应为 double，实际 %s → 用默认值 %.3f",
                key.c_str(), p.get_type_name().c_str(), def);
    return def;
  }

  std::string planner_type_;
  std::string frame_id_;
  std::string topic_map_;
  std::string topic_odom_;
  std::string topic_goal_;
  std::string topic_path_;
  std::string topic_markers_;
  std::string topic_status_;

  std::unique_ptr<GlobalPlanner> planner_;
  rclcpp::QoS odom_qos_{rclcpp::SensorDataQoS()};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_markers_;
  rclcpp::Publisher<pnc_2d::msg::PlannerStatus>::SharedPtr pub_status_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_;
  rclcpp::Service<pnc_2d::srv::SwitchPlanner>::SharedPtr srv_switch_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reload_;
  rclcpp::Service<pnc_2d::srv::PlanPath>::SharedPtr srv_plan_path_;
  /// 当前地图（latched 收到后留着：热切换/热重载要交给新规划器）
  std::shared_ptr<CostMap2D> map_;

  // "当前是否有一个有效的路径"。路径是事件驱动产物 + latched 话题，
  // 所以必须显式跟踪它的有效期，不然 RViz 会一直显示。
  bool has_active_plan_{false};
  Pose2D active_goal_;
  double plan_start_dist_{0.0}; // 规划时车离目标有多远（用于判断"真的是到达"）
  bool clear_on_reach_{true};
  double reach_tol_{0.30};
  /// 清场时删除的 route_active id 上界（够大即可，多发几个 DELETE 无代价）
  static constexpr int kStaleMarkerIds = 64;
  rclcpp::TimerBase::SharedPtr timer_startup_cleanup_;
  int startup_cleanup_ticks_{0};
  bool first_result_seen_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      cb_set_params_;

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
