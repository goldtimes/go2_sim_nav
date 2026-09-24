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
#include "pnc_2d/core/map_zones.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "pnc_2d/msg/planner_status.hpp"
#include "pnc_2d/msg/zone_array.hpp"
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

/// 允许的"起点机头 vs 路径方向"最大差角 [rad]（超过就不做时间参数化，见
/// applyTrajectory 里的说明）。45° = 明显不是"顺着路开"的场景，交给局部原地对正。
constexpr double kMaxHeadingDiff = 45.0 * M_PI / 180.0;

/// 折线长度 [m]（M4：轨迹到路径的截断点/接点算弧长用，口径与局部侧一致）
double polylineLength(const std::vector<Pose2D> &pts) {
  double len = 0.0;
  for (std::size_t i = 1; i < pts.size(); ++i)
    len += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  return len;
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

    // 轨迹优化后端（M4.2）：`traj.type` = none | minco。
    // ★ 默认 **none**：这条链路刚接通，先用 A/B 开关拿数据（同一张图、同一个
    //   目标各跑一遍），确认无回归再改默认值 —— 与 P5/P6 两期的做法一致。
    traj_type_ = paramString("traj.type", "none");
    traj_spacing_ = paramDouble("traj.publish_spacing", 0.05);
    buildTrajectoryOptimizer();

    // 创建 + 配置（启动、热切换、热重载共用同一段逻辑）
    {
      std::string err;
      if (!buildPlanner(planner_type_, err)) {
        std::string names;
        for (const auto &n : availablePlanners())
          names += (names.empty() ? "" : ", ") + n;
        RCLCPP_ERROR(get_logger(), "[planner] %s（可用：%s）→ 回退 astar",
                     err.c_str(), names.c_str());
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
    // 区域层：map_server 的 latched 话题 ⇒ 必须 transient_local 订阅，
    // 否则一条都收不到（volatile 收不到 latched 的历史样本）。
    topic_zones_ = paramString("topics.zones", "/global_map/zones");
    sub_zones_ = create_subscription<pnc_2d::msg::ZoneArray>(
        topic_zones_, rclcpp::QoS(1).transient_local(),
        std::bind(&GlobalPlannerNode::onZones, this, std::placeholders::_1));

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
        "~/plan_path", std::bind(&GlobalPlannerNode::onPlanPathRequest, this,
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

    RCLCPP_INFO(get_logger(),
                "[planner] type=%s frame=%s | 地图 %s | 位姿 %s | 目标 %s | "
                "路径 %s | 状态 %s",
                planner_type_.c_str(), frame_id_.c_str(), topic_map_.c_str(),
                topic_odom_.c_str(), topic_goal_.c_str(), topic_path_.c_str(),
                topic_status_.c_str());
    RCLCPP_INFO(
        get_logger(),
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
    timer_startup_cleanup_ =
        create_wall_timer(std::chrono::milliseconds(500), [this]() {
          if (first_result_seen_)
            return;
          publishStartupCleanup();
          if (++startup_cleanup_ticks_ >= 6) // 3 s 足够覆盖发现时间
            timer_startup_cleanup_->cancel();
        });
    publishStartupCleanup();

    // planner.type 支持**热切换**：按参数改会真的重建规划器（见
    // switchPlanner）。 失败时拒绝这次参数修改（successful=false）——
    // 保证"参数值 = 实际生效值"， 否则会出现"以为切成 A*
    // 了，结果还在走路网"这种最难查的误解。
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
              r.reason =
                  "热切换失败：" + err + "（仍在使用 " + planner_type_ + "）";
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
    // ★ 必须在这里重新接线：`setCostMap()` → `rebuildCollisionServices()` 会
    //   重建 `ClearanceField` ⇒ 优化器手里那个指针已经悬空（见
    //   wireTrajectoryOptimizer 的说明）。
    wireTrajectoryOptimizer();
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
    wireTrajectoryOptimizer();
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
    wireTrajectoryOptimizer();
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

    // 上行速度（M4.2）：MINCO 要知道"车现在多快"才能把剖面接上。
    // ★ 两个来源，优先级不同，**不能只信一个**：
    //   ① odom 的 twist：最准，但很多定位源只填 pose（字段恒为 0）；
    //   ② 位姿差分：0.25 s 窗口 + 一阶低通，抖动小时直接归零。
    //   若只取 ①，在"定位不填 twist"的现场会永远当静止起步（剖面首速 0）；
    //   若只取 ②，在 10 Hz 定位下用单帧差分噪声极大（÷0.1 s 会放大 10 倍）。
    if (std::isfinite(msg->twist.twist.linear.x)) {
      odom_v_ = msg->twist.twist.linear.x;
      odom_w_ = msg->twist.twist.angular.z;
      odom_twist_ok_ = true;
    }
    const rclcpp::Time t = now();
    if (has_prev_pose_) {
      const double dt = (t - prev_stamp_).seconds();
      const double d = std::hypot(start_.x - prev_pose_.x,
                                  start_.y - prev_pose_.y);
      if (dt > 0.0 && dt < 1.0) {
        pose_acc_d_ += d;
        pose_acc_t_ += dt;
        if (pose_acc_t_ >= 0.25) {
          const double v = pose_acc_d_ / pose_acc_t_;
          // 抖动（< 2 cm/s 平均）直接归零，不要把噪声低通成"假装在动"
          pose_speed_ = (v > 0.02) ? 0.4 * pose_speed_ + 0.6 * v : 0.0;
          pose_acc_d_ = 0.0;
          pose_acc_t_ = 0.0;
        }
      }
    }
    prev_pose_ = start_;
    prev_stamp_ = t;
    has_prev_pose_ = true;

    // 到达目标就收掉路径：路径是事件驱动产物，任务完成了就不该再挂在
    // latched 话题上（RViz 会一直显示）。
    // 只在"规划时本来就离目标更远"时生效，避免目标就在车边时刚发就清。
    if (clear_on_reach_ && has_active_plan_ && plan_start_dist_ > reach_tol_) {
      const double d =
          std::hypot(start_.x - active_goal_.x, start_.y - active_goal_.y);
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

    // 区域层信息（限速 + 起终点在禁行区内的点名）要在发布前用上
    PlanResult result = planner_->plan(req);
    applyZoneInfo(result, req);
    applyTrajectory(result, req);
    publishResult(result, req);
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
      RCLCPP_WARN(get_logger(), "[planner] %s：%s | 该话题发布者 %zu 个%s", who,
                  err.c_str(), pubs,
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
  void
  onPlanPathRequest(const std::shared_ptr<pnc_2d::srv::PlanPath::Request> req,
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
    if (!req->use_current_pose && (req->start.pose.position.x != 0.0 ||
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

    PlanResult result = planner_->plan(pr);
    applyZoneInfo(result, pr);
    // 轨迹优化（M4.2）：**必须在 applyZoneInfo 之后**（它可能把结果改成失败，
    // 那就没必要优化了），且在 publishResult / 填响应**之前**（它可能换掉路径）。
    applyTrajectory(result, pr);
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
    res->zone_speed_limit = result.zone_speed_limit;
    res->strict_corridor = result.strictCorridor();
    res->route_edges.assign(result.route_edges.begin(),
                            result.route_edges.end());
    // ★ 逐点走廊：长度必须与路径一致才透传（否则调用方按下标取会错位）。
    //   路网规划器在 hybrid 模式下会只给"落在通道上的点"填半宽，入口/出口自由段
    //   填 <=0 —— 局部据此只在车真在车道里时才启用硬约束（见
    //   local_planner_node）。
    if (result.corridor_width_per_point.size() == result.path.size())
      res->corridor_width = result.corridor_width_per_point;
    else if (result.has_corridor) {
      RCLCPP_WARN(get_logger(),
                  "[planner] 逐点走廊长度 (%zu) 与路径 (%zu) 不一致 → 退回标量 "
                  "半宽 %.3f（整条路径）",
                  result.corridor_width_per_point.size(), result.path.size(),
                  result.corridor_half_width);
      res->corridor_width.assign(result.path.size(),
                                 result.corridor_half_width);
      if (result.strictCorridor())
        std::fill(res->corridor_width.begin(), res->corridor_width.end(), 0.0);
    }
    // 参考速度剖面（M4.2）：与走廊同一口径 —— 只透传，不在这里取 min/截断。
    // ★ 契约：`traj_s/traj_v/traj_w` **等长**且 `s` 严格递增；不满足时宁可
    //   置 `traj_valid=false` 让调用方回退旧行为，也不要发半张表出去
    //   （`pnc_manager_node` 也会再校一道，两边口径一致）。
    res->traj_valid = traj_valid_ && traj_s_.size() >= 2 &&
                      traj_s_.size() == traj_v_.size() &&
                      traj_s_.size() == traj_w_.size();
    res->traj_note = traj_note_;
    if (res->traj_valid) {
      res->traj_s = traj_s_;
      res->traj_v = traj_v_;
      res->traj_w = traj_w_;
    } else if (!traj_note_.empty()) {
      RCLCPP_INFO(get_logger(), "[traj] 本次无剖面：%s", traj_note_.c_str());
    }
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

  /// 按 `traj.type` 建轨迹优化后端，并把**同一份**地图/距离场/轮廓判定器接上。
  void buildTrajectoryOptimizer() {
    traj_opt_ = createTrajectoryOptimizer(traj_type_);
    if (!traj_opt_) {
      if (!traj_type_.empty() && traj_type_ != "none" && traj_type_ != "null") {
        std::string names;
        for (const auto &n : availableTrajectoryOptimizers())
          names += (names.empty() ? "" : ", ") + n;
        RCLCPP_ERROR(
            get_logger(),
            "[traj] 未知 traj.type='%s'（可用：%s）→ 本次不启用轨迹优化",
            traj_type_.c_str(), names.c_str());
      }
      return;
    }
    RosParamReader reader(*this);
    if (!traj_opt_->configure(reader)) {
      RCLCPP_ERROR(get_logger(), "[traj] traj.type='%s' 参数装载失败 → 不启用",
                   traj_type_.c_str());
      traj_opt_.reset();
      return;
    }
    wireTrajectoryOptimizer();
    RCLCPP_INFO(get_logger(),
                "[traj] 已启用轨迹优化后端 %s（free 模式生效；路径发布间距 "
                "%.3f m）",
                traj_opt_->type().c_str(), std::max(0.05, traj_spacing_));
  }

  /// 把当前的地图/距离场/轮廓判定器交给优化器。
  /// ★ **换图、换算法、热重载之后都必须再调一次**：基类的
  ///   `rebuildCollisionServices()` 会重建 `ClearanceField`（指针会变），
  ///   而 `switchPlanner()` / `reloadParams()` 更是把整个 `planner_` 换掉 ⇒
  ///   旧指针立刻悬空（用悬空指针跑优化 = 段错误或静默算错净距）。
  ///   这就是"借出去的那一份"的代价：借用方必须跟着重取。
  void wireTrajectoryOptimizer() {
    if (!traj_opt_)
      return;
    traj_opt_->setMap(map_);
    traj_opt_->setClearanceField(planner_ ? planner_->clearanceField()
                                          : nullptr);
    traj_opt_->setCollisionChecker(planner_ ? &planner_->collisionChecker()
                                            : nullptr);
    traj_opt_->reset();
  }

  /// 上行线速度 [m/s]（MINCO 的 `start_v`）
  double startSpeed() {
    if (odom_twist_ok_ && std::fabs(odom_v_) > 1e-3)
      return std::fabs(odom_v_);
    return pose_speed_;
  }

  /// 上行角速度 [rad/s]（MINCO 的 `start_omega`）。
  /// 位姿差分估角速度噪声太大 ⇒ 拿不到 twist 时**宁可给 0**（按"不转"起步，
  /// 偏保守，但不给优化器喂假数据）。
  double startOmega() {
    if (odom_twist_ok_ && std::fabs(odom_w_) > 1e-3)
      return odom_w_;
    return 0.0;
  }

  /// 参考速度剖面（M4.2）：free 模式下对全局折线跑一次 MINCO 时间参数化。
  ///
  /// 三条语义（都必须守住）：
  ///   ① **只做 free 模式**（`!result.has_corridor`）：走廊是拍过板的硬约束
  ///      （贴线走、遇障只能停），在这里把中心线挪开执行，等于把那个决定
  ///      悄悄改掉；
  ///   ② **失败 = 降级，不是失败**：任何一步不通过都保持 A* 原路径 +
  ///      `traj_valid=false` + 点名原因，**绝不让任务因为优化失败而失败**；
  ///   ③ **通过也要过两道门**（`check_ok` 净距终检 + `kin_ok` 运动学终验），
  ///      两道门互相独立（终检只管碰不碰，kin 只管跟不跟得上）。
  void applyTrajectory(PlanResult &result, const PlanRequest &req) {
    traj_valid_ = false;
    traj_note_.clear();
    traj_s_.clear();
    traj_v_.clear();
    traj_w_.clear();
    if (!traj_opt_)
      return; // traj.type=none / 装载失败 → 完全旧行为
    if (!result.ok()) {
      traj_note_ = "全局规划未成功，不做轨迹优化";
      return;
    }
    if (result.has_corridor) {
      traj_note_ = "走廊模式（贴线硬约束），不做时间参数化";
      return;
    }
    if (result.path.size() < 2) {
      traj_note_ = "路径点数不足";
      return;
    }

    const std::size_t in_pts = result.path.size();
    const double in_len = polylineLength(result.path);

    // ★★ 机头差门槛（M4.4 实测补）：时间参数化的前提是"车已经在路径方向上"。
    //    起点机头与路径初始方向差得大时，MINCO 会把那个大转向**摊进整条轨迹**：
    //    实测折返场景（差 ~185°）产出 `轨迹 4.97 m / **285.42 s** / max|v| 0.07 m/s
    //    / 缩放 ×11.7` —— 一条完全无意义的怪物轨迹，而且花了 440 ms 才算出来。
    //    而这段转向**本来就该由局部层"原地对正"做**（日志里就有
    //    `原地对正机头（偏差 70° > 15°）`）：那是底盘能力，全局层无从表达。
    //    ⇒ 直接跳过（不是失败），并说明原因；等局部对正完、下次重规划再优化。
    if (result.path.size() >= 2) {
      const double path_dir = std::atan2(
          result.path[1].y - result.path[0].y,
          result.path[1].x - result.path[0].x);
      const double head_diff =
          std::fabs(std::remainder(req.start.yaw - path_dir, 2.0 * M_PI));
      if (head_diff > kMaxHeadingDiff) {
        traj_note_ = "起点机头与路径方向差 " +
                     std::to_string(static_cast<int>(head_diff * 180.0 / M_PI)) +
                     "° > " +
                     std::to_string(static_cast<int>(kMaxHeadingDiff * 180.0 /
                                                      M_PI)) +
                     "°（应由局部层原地对正，全局层摊不进一条轨迹）";
        RCLCPP_INFO(get_logger(), "[traj] 本次不优化：%s", traj_note_.c_str());
        return;
      }
    }

    TrajOptRequest treq;
    treq.path = result.path;
    treq.start = req.start;
    treq.start_v = startSpeed();
    treq.start_omega = startOmega();
    treq.goal = req.goal;
    treq.use_goal_yaw = req.goal.has_yaw;
    treq.goal_v = 0.0;

    const TrajOptResult tr = traj_opt_->optimize(treq);
    const TrajOptStats &st = tr.stats;
    RCLCPP_INFO(
        get_logger(),
        "[traj] %s：折线 %zu 点 / %.2f m → 轨迹 %.2f m / %.2f s | 求解 %.1f ms"
        "（平滑 %.1f + MINCO %.1f + 统计 %.1f）| max|v| %.2f |ω| %.2f |a| %.2f "
        "|κ| %.2f | 缩放 ×%.3f（收敛 %s）| 终点误差 %.3f m / %.3f rad | 净距 "
        "%.3f m @s=%.2f | 终检 %s（%d/%d 点不过）| 运动学 %s",
        toString(tr.status), in_pts, in_len, st.path_length, st.duration,
        st.solve_ms, st.smooth_ms, st.minco_ms, st.stats_ms, st.max_v,
        st.max_omega, st.max_a, st.max_curvature, st.time_scale,
        st.kin_ok ? "是" : "否", st.terminal_error, st.terminal_error_yaw,
        st.min_clearance, st.min_clearance_s, st.check_ok ? "过" : "不过",
        st.check_violations, st.check_points, st.kin_ok ? "过" : "不过");

    if (tr.status != TrajStatus::kSuccess || tr.samples.size() < 2) {
      traj_note_ = std::string("MINCO ") + toString(tr.status) + "：" +
                   (tr.message.empty() ? std::string("（无详情）") : tr.message);
      if (!st.check_note.empty())
        traj_note_ += "；" + st.check_note;
      RCLCPP_WARN(get_logger(),
                  "[traj] **不采用** ← %s（保持 A* 原路径 %zu 点，任务照走）",
                  traj_note_.c_str(), result.path.size());
      return;
    }

    // ★★ 契约检查（M4.2，2026-09-24 补）：**剖面是按弧长索引的** ⇒ 轨迹样本的
    //    `s` 必须单调不减。实测踩到：MINCO 在"起点机头与目标差 ~180°"这类输入上
    //    会产出一条**倒退/折返**的轨迹（样本 s 从 4.92 **回到** 3.94 m）。这种轨迹
    //    在弧长参数化下**结构上无法表达**，而当时的 applyTrajectory 照样把它封成
    //    剖面发下去 ⇒ 局部只能以"剖面弧长非严格递增"整份丢弃；现象是全局说
    //    "**采用** MINCO" 而局部说"无参考剖面"，A/B 表 剖面点数=0。
    //    ⇒ 必须在源头拦下并一句话点名。
    //
    //    ★ 容差用**毫米**而不是 1e-9（2026-09-24 第二个坑）：轨迹起点若是"原地
    //      大转向"，前几个样本的弧长几乎不前进（实测 0.003 m），数值积分噪声就能
    //      让相邻两个 s 差 ~1e-7 ⇒ 用 1e-9 会把**正常轨迹**判死（日志里打出来是
    //      "s 回退 0.003092 → 0.003092"，同一个数）。真正的倒退是 **1 m** 量级。
    constexpr double kSRewindTol = 1.0e-3;   // [m] 1 mm 以下的回退算噪声
    for (std::size_t i = 1; i < tr.samples.size(); ++i) {
      if (tr.samples[i].s < tr.samples[i - 1].s - kSRewindTol) {
        traj_note_ =
            "MINCO 轨迹弧长非单调（s 回退 " +
            std::to_string(tr.samples[i - 1].s) + " → " +
            std::to_string(tr.samples[i].s) +
            " m ⋯ 轨迹自身在倒退/折返）⇒ 剖面按弧长索引，无法表达这条轨迹";
        RCLCPP_WARN(get_logger(),
                    "[traj] **不采用** ← %s（峰值 %.2f m/s / 弧长 %.2f m vs 输入 "
                    "%.2f m）；保持 A* 原路径 %zu 点",
                    traj_note_.c_str(), st.max_v, st.path_length, in_len,
                    result.path.size());
        return;
      }
    }

    // ---- 稠密轨迹 → 路径 + 剖面 ----
    // ★ 间距**不能**小于 5 cm：MPC 用相邻路径点算切向，点太密时切向被积分/
    //   栅格噪声主导（与全局路径的下采样口径一致）。
    const double spacing = std::max(0.05, traj_spacing_);
    const double s0 = tr.samples.front().s;
    std::vector<Pose2D> dense;
    double last_s = -1.0e9;
    for (std::size_t i = 0; i < tr.samples.size(); ++i) {
      const TrajSample &sm = tr.samples[i];
      const bool is_last = (i + 1 == tr.samples.size());
      if (!is_last && !dense.empty() && sm.s - last_s < spacing)
        continue;
      Pose2D p;
      p.x = sm.x;
      p.y = sm.y;
      p.yaw = sm.yaw;
      p.has_yaw = true;
      dense.push_back(p);
      traj_s_.push_back(sm.s - s0);
      traj_v_.push_back(sm.v);
      traj_w_.push_back(sm.omega);
      last_s = sm.s;
    }
    if (dense.size() < 2 || traj_s_.size() < 2) {
      traj_s_.clear();
      traj_v_.clear();
      traj_w_.clear();
      traj_note_ = "轨迹采样点不足";
      RCLCPP_WARN(get_logger(), "[traj] **不采用** ← %s", traj_note_.c_str());
      return;
    }

    // 截断（`traj.max_length_m`，默认 40 m）：轨迹只覆盖前 N 米 ⇒ 把 A* 的
    // **剩余段接在后面**。
    // 为什么不是"干脆不换路径"：剖面是**按弧长**索引的，而平滑会把弧长改掉
    // 几个百分点 ⇒ 局部拿原 A* 路径的弧长去查表会系统性错位。轨迹段的几何
    // 与路径段必须同源。
    const double covered = tr.samples.back().s - s0;
    if (covered < in_len - 0.05) {
      double acc = 0.0;
      std::size_t i = 0;
      for (; i + 1 < result.path.size(); ++i) {
        const double seg = std::hypot(result.path[i + 1].x - result.path[i].x,
                                      result.path[i + 1].y - result.path[i].y);
        if (acc + seg >= covered)
          break;
        acc += seg;
      }
      const Pose2D junction = dense.back();
      std::size_t appended = 0;
      double gap = 0.0;
      for (std::size_t k = i + 1; k < result.path.size(); ++k) {
        const Pose2D &p = result.path[k];
        if (appended == 0) {
          // 接点**无条件**接上：哪怕离轨迹末端很近，也不能让路径断开
          gap = std::hypot(p.x - junction.x, p.y - junction.y);
          dense.push_back(p);
        } else {
          if (std::hypot(p.x - dense.back().x, p.y - dense.back().y) < spacing)
            continue;
          dense.push_back(p);
        }
        ++appended;
      }
      RCLCPP_INFO(get_logger(),
                  "[traj] 轨迹只覆盖前 %.2f m（路径共 %.2f m）⇒ 接上 A* 剩余 "
                  "%zu 点（接点间隙 %.3f m）",
                  covered, in_len, appended, gap);
    }
    result.path = std::move(dense);
    traj_valid_ = true;
    RCLCPP_INFO(get_logger(),
                "[traj] **采用** MINCO 轨迹：路径 %zu 点 → 下发 %zu 点"
                "（间距 %.3f m）/ 剖面 %zu 点",
                in_pts, result.path.size(), spacing, traj_s_.size());
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
                result.message.empty() ? "" : " | ", result.message.c_str());
    has_active_plan_ = true;
    active_goal_ = req.goal;
    plan_start_dist_ =
        std::hypot(req.goal.x - req.start.x, req.goal.y - req.start.y);
    publishMarkers(&result, &req);
  }

  // ------------------------------------------------------------ 区域层
  /// 区域层（禁行区/限速区）：map_server latched 发布。
  ///
  /// 禁行区**已经**被 map_server 烧进全局图 ⇒
  /// 路由/可行性校验自动避开它。这里额外做 两件事：
  ///   ① **限速区**：沿规划出的路径取最严限速交给管理器合成"任务限速"。不做的话
  ///      就会出现"局部按区限速、任务限速还是旧值"两边打架；
  ///   ②
  ///   **起终点落在禁行区内时点名**：否则只能看到含糊的"目标不可达/起点在障碍里"，
  ///      而真实原因往往就是区域（现场排查最耗时的一类）。
  void onZones(const pnc_2d::msg::ZoneArray::SharedPtr msg) {
    std::vector<pnc_2d::MapZone> zs;
    zs.reserve(msg->zones.size());
    for (const auto &zin : msg->zones) {
      pnc_2d::MapZone z;
      z.name = zin.name;
      bool ok = false;
      z.type = pnc_2d::zoneTypeFromString(zin.type, &ok);
      if (!ok)
        continue;
      z.value = zin.value;
      for (const auto &p : zin.polygon.points)
        z.polygon.push_back(pnc_2d::Pose2D{p.x, p.y, 0.0});
      zs.push_back(std::move(z));
    }
    zones_.adopt(std::move(zs));
    zone_inflate_ = msg->inflate;
    RCLCPP_INFO(get_logger(),
                "[planner] 区域层：禁行 %zu / 限速 %zu（inflate %.3f m）",
                zones_.forbiddenCount(), zones_.speedCount(), zone_inflate_);
  }

  /// 把区域信息用到规划结果上（见 onZones 的说明）。
  /// 采样按**弧长 0.25 m**走，不是只查路径点：路径点密度由后处理决定，稀疏时
  /// 会整个跳过一个小限速区。
  void applyZoneInfo(PlanResult &r, const PlanRequest &req) const {
    if (zones_.empty())
      return;
    if (r.ok() && r.path.size() >= 2 && zones_.speedCount() > 0) {
      // lookahead<=0 会“只看起点那一点”，所以这里用很大的值表达“整条路径”
      const double anywhere =
          pnc_2d::zoneSpeedLimitAhead(zones_, r.path, 0, 1e9);
      // ★ 只在「整条任务都在限速区里」时把它当成**任务级**限速（起点、终点都在
      //   限速区内 ⇒ 全程都该慢）。只是**路过**一小段时**不设**任务限速。
      //
      //   为什么必须这么分：任务限速会被 manager 折进 `goal.speed_limit`，成为
      //   局部**整条路径**的硬上界。若"路过"也设，一条 100 m 的路线里路过 2 m
      //   的 限速区，整趟就只能爬 0.15 m/s。实测（2026-09-23 test_zones 段
      //   3）： 限速区只是路径中的一小段，机器人出区后速度仍是 0.15 ⇒
      //   "出区后恢复" 这个断言永远测不出来，看着像区域没生效。
      //   路过段的减速由**局部节点的几何前瞻**负责（每周期算"到下一个限速区还有
      //   多远"，前瞻距离 = 刹车距离 v²/(2a) + 0.3 m ⇒ 进区时速度已经合规）。
      double lim_start = 0.0;
      double lim_goal = 0.0;
      const bool start_in_zone =
          zones_.inSpeedZone(r.path.front().x, r.path.front().y, lim_start);
      const bool goal_in_zone =
          zones_.inSpeedZone(r.path.back().x, r.path.back().y, lim_goal);
      if (start_in_zone && goal_in_zone) {
        r.zone_speed_limit = anywhere;
        RCLCPP_INFO(get_logger(),
                    "[planner] 起点与终点都在限速区内 → 任务限速 %.2f m/s",
                    anywhere);
      } else if (anywhere > 0.0) {
        RCLCPP_INFO(
            get_logger(),
            "[planner] 路径经过限速区（%.2f m/s，起终点不在区内）→ 不设任务"
            "限速，改由局部按几何前瞻在该段减速",
            anywhere);
      }
    }
    const std::string gz = zones_.forbiddenNameAt(req.goal.x, req.goal.y);
    const std::string sz = zones_.forbiddenNameAt(req.start.x, req.start.y);
    if (gz.empty() && sz.empty())
      return;
    std::string note;
    if (!sz.empty())
      note += "起点在禁行区 " + sz + " 内";
    if (!sz.empty() && !gz.empty())
      note += "，";
    if (!gz.empty())
      note += "目标在禁行区 " + gz + " 内";
    r.message += "；区域：" + note;
    RCLCPP_WARN(get_logger(), "[planner] 区域：%s", note.c_str());
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
    // （active_marker_count_ 重启后从 0
    // 开始），所以这里按一个足够大的范围全部删掉。
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
  /// 区域层（禁行/限速）：map_server latched 发过来，见 onZones
  pnc_2d::ZoneSet zones_;
  double zone_inflate_{0.0};
  std::string topic_zones_;
  rclcpp::Subscription<pnc_2d::msg::ZoneArray>::SharedPtr sub_zones_;
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

  // ---- 轨迹优化后端（M4.2）----
  /// `traj.type`（none | minco）；默认 none（A/B 开关）
  std::string traj_type_;
  /// 换掉路径时下发的点间距 [m]（**下限 5 cm**：MPC 用相邻点算切向）
  double traj_spacing_{0.05};
  std::unique_ptr<TrajectoryOptimizer> traj_opt_;
  /// 本次规划产出的剖面（`traj_valid_=false` 时三个数组为空）
  bool traj_valid_{false};
  std::string traj_note_;
  std::vector<double> traj_s_, traj_v_, traj_w_;

  // ---- 上行速度（M4.2：MINCO 的 start_v/start_omega）----
  bool odom_twist_ok_{false};
  double odom_v_{0.0};
  double odom_w_{0.0};
  Pose2D prev_pose_;
  bool has_prev_pose_{false};
  double pose_acc_d_{0.0};   // 位姿差分窗口内累计位移 [m]
  double pose_acc_t_{0.0};   // 位姿差分窗口内累计时间 [s]
  double pose_speed_{0.0};   // 低通后的位姿差分速度 [m/s]
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
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
