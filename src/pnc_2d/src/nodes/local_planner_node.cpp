// 局部规划节点（ROS 2）—— P4 只跑 NullLocalPlanner，**不产生任何速度指令**。
//
// 职责（薄壳，算法都在 core/local/ 里）：
//   · 收 action `~/follow_path`：goal 带路径（+ 走廊/限速），周期跑控制循环；
//   · 发 feedback：进度/横向偏差/是否被挡（状态机据此决定是否重规划）；
//   · 发 result：goal_reached / blocked / failed / canceled 四选一（见 action
//   定义）； · 发布 `~/local_status`（latched）供监控；`~//cmd_vel` 只在
//     `producesCmdVel() == true` 时才发。
//
// ★ P4 的关键语义：NullLocalPlanner 的 producesCmdVel() == false，
//   所以 "cmd = 0" 表示"我不管"，**不是**"我命令你停下"。
//   节点因此完全不发 cmd_vel，也绝不去抢 /pnc_2d/cmd_vel 的所有权 —— 否则
//   teleop / 其它控制器会被一个恒零的话题压住。
//
// 为什么用 action 而不是 service：跟路径是长时任务，需要周期反馈与取消能力。
// 阻塞语义见 action/FollowPath.action 的注释（BLOCKED 要连续超时才结束，
// 避免一个瞬时遮挡就把任务判死）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pnc_2d/action/follow_path.hpp"
#include "pnc_2d/core/corridor_slice.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/local_planner.hpp"
#include "pnc_2d/core/map_zones.hpp" // 区域层（禁行区/限速区）
#include "pnc_2d/core/route_graph.hpp" // distanceToPolyline（点到折线中心线的距离）
#include "pnc_2d/msg/local_status.hpp"
#include "pnc_2d/msg/zone_array.hpp" // map_server 发布的区域层（latched）
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

/// nav_msgs::Path → 内部路径表示（世界系）
/// nav_msgs/Path → 内部位姿（**带朝向**）
///
/// ★ `has_yaw = true` 是**必须**的：到点判定要拿"路径末点的目标朝向"与车头比
///   （见 finishReached）。留默认的 false 会让那段判定被静默跳过 —— 功能看起来
///   接好了、实际永远不判朝向（典型的"静默失效"）。
///   朝向可信的依据：本仓库的全局规划器逐点填好了四元数（中间点 = 段方向、
///   末点 = 目标朝向）。
std::vector<Pose2D> pathToPoses(const nav_msgs::msg::Path &p) {
  std::vector<Pose2D> out;
  out.reserve(p.poses.size());
  for (const auto &ps : p.poses)
    out.push_back(Pose2D{ps.pose.position.x, ps.pose.position.y,
                         yawFromQuaternion(ps.pose.orientation),
                         /*has_yaw=*/true});
  return out;
}

double polylineLength(const std::vector<Pose2D> &p) {
  double len = 0.0;
  for (std::size_t i = 1; i < p.size(); ++i)
    len += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  return len;
}

} // namespace

using FollowPath = pnc_2d::action::FollowPath;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

class LocalPlannerNode : public rclcpp::Node {
public:
  LocalPlannerNode()
      : rclcpp::Node(
            "local_planner",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)) {
    local_type_ = paramString("local.type", "none");
    frame_id_ = paramString("local.frame_id", "map");
    topic_odom_ = paramString("topics.odom", "/lightning/perception/pose");
    topic_cmd_vel_ = paramString("local.cmd_vel_topic", "/pnc_2d/cmd_vel");
    topic_status_ = paramString("topics.local_status", "/pnc_2d/local_status");
    // 局部图/距离场：P5 的 MPC 才需要，P4 只是把话题名配好（订阅见下面注释）
    topic_local_map_ =
        paramString("topics.local_map", "/grid_map/occupancy_inflate_2d");
    topic_esdf_ = paramString("topics.esdf", "/grid_map/esdf_2d");
    // 区域层（禁行区/限速区）：由 map_server latched 发布。没这个话题/没收到时
    // 行为与今天完全一致（向后兼容），收到后局部才真正遵守区域。
    topic_zones_ = paramString("topics.zones", "/global_map/zones");

    control_rate_ = paramDouble("local.control_rate", 20.0);
    goal_tolerance_ = paramDouble("local.goal_tolerance", 0.30);
    // BLOCKED 要连续持续这么久才结束 action（把决定权交回状态机）：
    // 瞬时遮挡（有人走过、点云抖一帧）不该让任务失败。
    blocked_abort_s_ = paramDouble("local.blocked_abort_s", 1.0);
    // 距上一个路径点小于该值就算"经过"，用来算进度
    pass_distance_ = paramDouble("local.pass_distance", 0.50);
    odom_timeout_ = paramDouble("local.odom_timeout", 1.0);
    // 距离场超时：超过它就按"拿不到距离场"处理（降级 + 限速），
    // 而不是继续用一张可能已经过期的场去避障。
    esdf_timeout_ = paramDouble("local.esdf_timeout", 0.3);
    // 距离场的截断上限：必须 **≥ 感知发布端的 esdf_max_dist**（默认 3.0），
    // 否则"未覆盖格子"会被我们填成更小的值，车会绕开一片其实很开阔的区域。
    esdf_max_distance_ = paramDouble("local.esdf_max_distance", 3.0);
    // 邻域填充半径（单位：本地图栅格）。感知的点云采样步长 esdf_pub_step 默认 2
    // 格 （即 0.2 m），比本地图分辨率粗，所以至少填 1 圈才能得到连续场。
    esdf_fill_radius_ = paramInt("local.esdf_fill_radius", 2);
    // 限速区前瞻用的减速度兜底值：**优先用算法自报的**（`LocalPlanner::brakeAcc()`
    // = `local_mpc.brake_acc`，与终点制动剖面同一个量 —— 单一来源）；只有算法
    // 不报（非 MPC 实现）时才用这个参数。
    // 默认 0.0 表示"没配就用算法自报值"；两处都配了且明显不一致会 WARN（见
    // effectiveBrakeAcc）。写大了会只在区前一点点才减速（进区还超速），写小了
    // 会过早减速。
    brake_acc_param_ = paramDouble("local.brake_acc", 0.0);
    // ---- 速度反馈（MPC 需要真实的 v；twist 不可靠时改用位姿差分）----
    vel_from_pose_ = paramBool("local.vel_from_pose", false);
    vel_pose_window_ = paramDouble("local.vel_pose_window", 0.25);
    vel_filter_tau_ = paramDouble("local.vel_filter_tau", 0.15);
    // 区域在**局部侧**的额外膨胀：默认 0（见成员声明处的长注释）。
    zones_local_inflate_ = paramDouble("zones.local_inflate", 0.0);
    if (zones_local_inflate_ < 0.0) zones_local_inflate_ = 0.0;

    if (control_rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "[local] local.control_rate=%.2f 非法 → 用 20 Hz",
                  control_rate_);
      control_rate_ = 20.0;
    }

    // 创建 + 配置局部算法（与全局节点同款：启动/热切换/热重载共用一段逻辑）
    {
      std::string err;
      if (!buildPlanner(local_type_, err)) {
        std::string names;
        for (const auto &n : availableLocalPlanners())
          names += (names.empty() ? "" : ", ") + n;
        RCLCPP_ERROR(get_logger(), "[local] %s（可用：%s）→ 回退 none",
                     err.c_str(), names.c_str());
        local_type_ = "none";
        buildPlanner(local_type_, err);
      }
    }

    auto odom_qos = rclcpp::SensorDataQoS(); // 定位是 best_effort，必须匹配
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, odom_qos,
        std::bind(&LocalPlannerNode::onOdom, this, std::placeholders::_1));

    // 局部膨胀图（硬碰撞/障碍判定）与局部 ESDF（避障软代价）：
    // 两者都由 perception 以 **reliable + depth 1 + volatile**
    // 发布（实测确认）， 所以订阅用同样配置（volatile
    // 是有意的：要的是**最新**一帧，不要补发旧数据）。
    const auto live_qos = rclcpp::QoS(1);
    sub_local_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_local_map_, live_qos,
        std::bind(&LocalPlannerNode::onLocalMap, this, std::placeholders::_1));
    sub_esdf_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_esdf_, live_qos,
        std::bind(&LocalPlannerNode::onEsdf, this, std::placeholders::_1));

    // 区域层：map_server 的 latched 话题（只在加载/变更时发一次）⇒ 必须用
    // transient_local 订阅，否则永远只能等到“下一次变更”才收到（踩过：
    // volatile 订阅 latched 话题会一条都收不到）。
    sub_zones_ = create_subscription<pnc_2d::msg::ZoneArray>(
        topic_zones_, rclcpp::QoS(1).transient_local(),
        std::bind(&LocalPlannerNode::onZones, this, std::placeholders::_1));

    pub_status_ = create_publisher<pnc_2d::msg::LocalStatus>(
        topic_status_, rclcpp::QoS(1).transient_local());
    // ⚠ 只在会真发速度时才创建发布者：NullLocalPlanner 不发 cmd_vel，
    //   连发布者都不该存在（否则别人 ros2 topic info 会看到一个恒零的发布端）。
    if (planner_->producesCmdVel()) {
      pub_cmd_vel_ =
          create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 1);
    }

    action_server_ = rclcpp_action::create_server<FollowPath>(
        this, "~/follow_path",
        std::bind(&LocalPlannerNode::onGoalRequest, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&LocalPlannerNode::onCancel, this, std::placeholders::_1),
        std::bind(&LocalPlannerNode::onAccepted, this, std::placeholders::_1));

    srv_switch_ = create_service<pnc_2d::srv::SwitchPlanner>(
        "~/switch_planner",
        [this](const std::shared_ptr<pnc_2d::srv::SwitchPlanner::Request> req,
               std::shared_ptr<pnc_2d::srv::SwitchPlanner::Response> res) {
          std::string err;
          if (switchPlanner(req->type, err)) {
            res->success = true;
            res->message = "已切换到 " + local_type_;
          } else {
            res->success = false;
            res->message = err;
          }
        });

    srv_reload_ = create_service<std_srvs::srv::Trigger>(
        "~/reload_params",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::string err;
          if (reloadParams(err)) {
            res->success = true;
            res->message = "已按当前参数重新装载 " + local_type_;
          } else {
            res->success = false;
            res->message = err;
          }
        });

    // 取消入口：状态机切模式/新目标顶掉旧任务时用它（等价于 action cancel）
    srv_stop_ = create_service<std_srvs::srv::Trigger>(
        "~/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          // 服务端不能主动"取消" action（只有客户端能），所以走 abort，
          // 但把 result.canceled 置真："这不是失败，是被叫停"。
          if (goal_handle_)
            endAsCanceled(LocalPlanResult{}, "收到 ~/stop");
          else
            publishStatus(LocalStatus::kIdle, "收到 ~/stop（本来就没在跟随）",
                          LocalPlanResult{});
          res->success = true;
          res->message = "已停止跟随";
        });

    RCLCPP_INFO(get_logger(),
                "[local] type=%s frame=%s | 位姿 %s | 状态 %s | cmd_vel %s",
                local_type_.c_str(), frame_id_.c_str(), topic_odom_.c_str(),
                topic_status_.c_str(),
                planner_->producesCmdVel()
                    ? topic_cmd_vel_.c_str()
                    : "(本算法不产生速度指令，未创建发布者)");
    RCLCPP_INFO(get_logger(),
                "[local] 到达判定 ≤ %.2f m | 控制周期 %.1f Hz | BLOCKED 连续 "
                "%.2f s 才结束 action",
                goal_tolerance_, control_rate_, blocked_abort_s_);
    if (!planner_->producesCmdVel()) {
      RCLCPP_WARN(get_logger(),
                  "[local] ★ 当前是 '%s'（空实现）：链路可以跑通，但**不会输出 "
                  "/pnc_2d/cmd_vel**。"
                  "P5 接入 MPC 后才会有速度。",
                  local_type_.c_str());
    }
    // 局部图 / 距离场（P5.3 已接线）：
    //   · 膨胀图 → 硬判定/障碍（LocalPlanner::setCostMap）
    //   · esdf_2d 点云 → 栅格化成 LocalDistanceField（setDistanceField）
    //   两者都只对“需要避障的算法”有意义；NullLocalPlanner 收到也不会用。
    RCLCPP_INFO(get_logger(),
                "[local] 局部图 %s | 距离场 %s（超时 %.2f s，截断 %.2f m）",
                topic_local_map_.c_str(), topic_esdf_.c_str(), esdf_timeout_,
                esdf_max_distance_);

    publishStatus(LocalStatus::kIdle, "启动", LocalPlanResult{});

    // ★ 进程退出时必须发零速。底盘的步态控制器**会保持最后收到的指令**：
    //   规划器一退出（Ctrl-C / 被 kill / 崩了），车会以最后那个速度一直走下去
    //   （实际踩过：测试被中断后车继续往前走，只能靠 teleop 追回来）。
    //   `rclcpp::on_shutdown` 在 context 关停**之前**执行，是唯一可靠的时机。
    //   同样的问题也存在于"任务结束"——见 finish()/endAsCanceled()。
    rclcpp::on_shutdown([this]() { publishStop(); });

    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / control_rate_),
                          std::bind(&LocalPlannerNode::onControl, this));
  }

  ~LocalPlannerNode() override {
    // context 已关停时（on_shutdown 那条路径已发过）再发只会报错，跳过
    if (rclcpp::ok())
      publishStop();
  }

private:
  // ------------------------------------------------------------ 算法装载
  bool buildPlanner(const std::string &type, std::string &err) {
    auto p = createLocalPlanner(type);
    if (!p) {
      err = "未知 local.type='" + type + "'";
      return false;
    }
    if (!p->configure(RosParamReader(*this))) {
      err = "local.type='" + type + "' 配置失败";
      return false;
    }
    planner_ = std::move(p);
    local_type_ = type;
    return true;
  }

  void rebuildCmdVelPublisher() {
    // 换算法可能改变"会不会发速度"：发布者要跟着建/拆
    const bool need = planner_->producesCmdVel();
    if (need && !pub_cmd_vel_)
      pub_cmd_vel_ =
          create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 1);
    else if (!need && pub_cmd_vel_)
      pub_cmd_vel_.reset();
  }

  bool switchPlanner(const std::string &type, std::string &err) {
    if (type == local_type_)
      return true;
    auto p = createLocalPlanner(type);
    if (!p) {
      std::string names;
      for (const auto &n : availableLocalPlanners())
        names += (names.empty() ? "" : ", ") + n;
      err = "未知 local.type='" + type + "'（可用：" + names + "）";
      return false;
    }
    if (!p->configure(RosParamReader(*this))) {
      err = "local.type='" + type + "' 配置失败";
      return false;
    }
    planner_ = std::move(p);
    local_type_ = type;
    rebuildCmdVelPublisher();
    resetFollow();
    RCLCPP_INFO(get_logger(), "[local] 已热切换到 '%s'", local_type_.c_str());
    return true;
  }

  bool reloadParams(std::string &err) { return buildPlanner(local_type_, err); }

  // ------------------------------------------------------------ 输入
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!frame_id_.empty() && !msg->header.frame_id.empty() &&
        msg->header.frame_id != frame_id_) {
      // 只告警一次，避免刷屏（与全局节点同款处理）
      if (!warned_odom_frame_) {
        warned_odom_frame_ = true;
        RCLCPP_WARN(get_logger(),
                    "[local] 位姿 frame='%s' 与 local.frame_id='%s' "
                    "不一致，仍按后者解释",
                    msg->header.frame_id.c_str(), frame_id_.c_str());
      }
    }
    pose_ = Pose2D{msg->pose.pose.position.x, msg->pose.pose.position.y,
                   yawFromQuaternion(msg->pose.pose.orientation)};
    last_odom_time_ = now();
    has_odom_ = true;
    // ★ 底盘速度反馈：MPC 的状态量含 v，而 v_cmd 在模型里受 "v_0 + a_max·dt"
    // 限制
    //   —— 也就是说**命令只能从当前速度一点点爬上来**，反馈不准就会永远起不来。
    //   两种来源：
    //   ·
    //   `odom.twist`：延迟低，但对低速噪声/偏置无能为力。本仓库实测（2026-09-22）
    //     lightning 的 twist 在车实际走 0.045 m/s 时读到
    //     −0.03（**均值都偏低**）， 结果是 cmd_v 永远被顶在 0.06 ⇒
    //     车永远起不来（走 60 s 只挪了 3 m）。
    //   · 位姿差分（默认开）：那条位姿本身是可靠的（跟踪指标都由它算）。用
    //     `vel_pose_window` 长度的窗口做差分，噪声从 σ/dt 降到 σ/window。
    //   默认参数关掉，按底盘选（go2_run.yaml 里开）。
    double v_fb = msg->twist.twist.linear.x;
    double w_fb = msg->twist.twist.angular.z;
    if (vel_from_pose_)
      measureVelocity(v_fb, w_fb);
    // 一阶低通（两种来源都过一遍；噪声大的信号本来就该滤）
    const double tsec = now().seconds();
    if (has_vel_prev_) {
      const double dt = tsec - vel_prev_t_;
      if (dt > 1e-4 && dt < 1.0) {
        const double a = 1.0 - std::exp(-dt / std::max(1e-3, vel_filter_tau_));
        v_filt_ += a * (v_fb - v_filt_);
        w_filt_ += a * (w_fb - w_filt_);
      }
    } else {
      v_filt_ = v_fb;
      w_filt_ = w_fb;
    }
    vel_prev_t_ = tsec;
    has_vel_prev_ = true;
    planner_->setCurrentVelocity(v_filt_, w_filt_);
  }

  /// 速度反馈：用"vel_pose_window 秒前的位姿 → 当前位姿"差分（车体系前向 +
  /// 航向率）。 为什么要窗口而不是相邻两帧：位置噪声 σ 除以
  /// dt，帧间差分会把噪声放大 （σ=1 cm、dt=0.05 s ⇒ 0.2 m/s 的速度噪声）；用
  /// 0.25 s 的窗口降到 0.04 m/s。
  void measureVelocity(double &v, double &w) {
    pose_hist_.push_back({now().seconds(), pose_.x, pose_.y, pose_.yaw});
    while (pose_hist_.size() > 2 &&
           pose_hist_.back().t - pose_hist_.front().t > vel_pose_window_ * 3.0)
      pose_hist_.pop_front();
    if (pose_hist_.empty())
      return;
    const auto &old = pose_hist_.front();
    const double dt = pose_hist_.back().t - old.t;
    if (dt < std::max(0.05, 0.5 * vel_pose_window_))
      return; // 还没攒够窗口，先用 twist 的值
    const double dx = pose_.x - old.x;
    const double dy = pose_.y - old.y;
    const double yaw_mid = old.yaw;
    v = (dx * std::cos(yaw_mid) + dy * std::sin(yaw_mid)) / dt;
    const double dyaw = std::atan2(std::sin(pose_.yaw - old.yaw),
                                   std::cos(pose_.yaw - old.yaw));
    w = dyaw / dt;
  }

  /// 区域层（map_server latched 发布）：收到后局部才真正遵守禁行区/限速区。
  ///
  /// 与"烧进全局图"是**互补**关系：全局图只解决订阅它的人（全局规划器/RViz），
  /// 而局部用的是感知滑动窗 + ESDF（里面没有区域）⇒ 必须自己融进去。
  /// 没收到 = 行为与今天一致（向后兼容）；收到空数组 = 这张图确认没有区域。
  void onZones(const pnc_2d::msg::ZoneArray::SharedPtr msg) {
    std::vector<pnc_2d::MapZone> zs;
    zs.reserve(msg->zones.size());
    for (const auto &zin : msg->zones) {
      pnc_2d::MapZone z;
      z.name = zin.name;
      bool ok = false;
      z.type = pnc_2d::zoneTypeFromString(zin.type, &ok);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "[local] 区域 '%s' 类型 '%s' 不认识 → 丢弃",
                    zin.name.c_str(), zin.type.c_str());
        continue;
      }
      z.value = zin.value;
      z.polygon.reserve(zin.polygon.points.size());
      for (const auto &p : zin.polygon.points)
        z.polygon.push_back(pnc_2d::Pose2D{p.x, p.y, 0.0});
      zs.push_back(std::move(z));
    }
    const bool any = zones_.adopt(std::move(zs));
    zone_inflate_msg_ = msg->inflate;
    for (const std::string &w : zones_.warnings())
      RCLCPP_WARN(get_logger(), "[local] 区域层：%s", w.c_str());
    if (!any) {
      RCLCPP_INFO(get_logger(), "[local] 区域层：无区域（行为 = 自由空间）");
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "[local] 区域层：禁行 %zu / 限速 %zu；消息里的 inflate %.3f m（那是"
        "**全局图**的膨胀量），局部按原几何判（额外膨胀 %.3f m）—— "
        "详见 config/pnc_2d.yaml 里 zones.local_inflate 的注释",
        zones_.forbiddenCount(), zones_.speedCount(), zone_inflate_msg_,
        zones_local_inflate_);
    if (zones_local_inflate_ > 1e-9 &&
        std::fabs(zones_local_inflate_ - zone_inflate_msg_) < 1e-9) {
      RCLCPP_WARN(get_logger(),
                  "[local] zones.local_inflate == 消息里的 inflate(%.3f m) ⇒ "
                  "全局与局部的区域判据**完全相等（零余量）**，任何 1~2 cm 的"
                  "栅格/图差都会变成\"全局给路、局部说在障碍里\"（2026-09-23 "
                  "实测任务死锁）；建议留 0",
                  zone_inflate_msg_);
    }
  }

  /// 局部膨胀图 → CostMap2D → 交给算法做硬判定
  void onLocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    auto map = std::make_shared<CostMap2D>();
    const auto &info = msg->info;
    const double yaw = yawFromQuaternion(info.origin.orientation);
    if (!map->set(static_cast<int>(info.width), static_cast<int>(info.height),
                  info.resolution, info.origin.position.x,
                  info.origin.position.y, yaw, msg->data,
                  msg->header.frame_id)) {
      RCLCPP_ERROR(get_logger(), "[local] 局部图无效：%ux%u @ %.3f m",
                   info.width, info.height, info.resolution);
      return;
    }
    if (!frame_id_.empty() && !msg->header.frame_id.empty() &&
        msg->header.frame_id != frame_id_ && !warned_map_frame_) {
      warned_map_frame_ = true;
      RCLCPP_WARN(get_logger(),
                  "[local] 局部图 frame='%s' 与 local.frame_id='%s' "
                  "不一致（不做 TF 变换）",
                  msg->header.frame_id.c_str(), frame_id_.c_str());
    }
    // 滑窗原点每帧都在变（这是正常的），所以**不要**因此告警，否则日志会被刷满。
    // 真正有意义的异常是分辨率/尺寸变了（第一帧不算变化）。
    const bool first_map = !has_local_map_;
    const bool shape_changed =
        !first_map &&
        (map->width() != local_map_->width() ||
         map->height() != local_map_->height() ||
         std::fabs(map->resolution() - local_map_->resolution()) > 1e-9);
    // ★ 禁行区融合（每帧都要做，A1）：局部图来自 perception
    // 的滑动窗，**里面没有
    //   区域**（感知不认识“禁行区”这个语义）。不融的话，车可以贴着/开进禁行区
    //   —— 今天只有“订阅全局图”的模块（全局规划器/RViz）遵守它，局部是盲的。
    //   融进栅格之后，“车体轮廓与禁行区重叠”就会被现有的 footprint 检查判死，
    //   语义与全局一模一样（全局也是把禁行区烧进图）。
    //   成本：只写“区域包围盒 ∩ 局部窗”的格子（80×80 的窗，几百格）≈ 几十 µs。
    if (!zones_.empty() && zones_.forbiddenCount() > 0) {
      std::vector<int8_t> burned;
      const std::size_t n =
          pnc_2d::burnForbidden(zones_, *map, zones_local_inflate_, burned);
      if (n > 0) {
        auto fused = std::make_shared<CostMap2D>();
        if (fused->set(map->width(), map->height(), map->resolution(),
                       map->originX(), map->originY(), map->originYaw(), burned,
                       msg->header.frame_id)) {
          map = fused;
        }
      }
    }
    local_map_ = map;
    has_local_map_ = true;
    planner_->setCostMap(local_map_);
    if (first_map) {
      RCLCPP_INFO(get_logger(),
                  "[local] 收到局部图 %dx%d @ %.3f m，origin (%.2f, %.2f)",
                  map->width(), map->height(), map->resolution(),
                  map->originX(), map->originY());
    } else if (shape_changed) {
      RCLCPP_WARN(
          get_logger(),
          "[local] 局部图尺寸/分辨率变化：%dx%d @ %.3f → 距离场按新几何重建",
          map->width(), map->height(), map->resolution());
    }
  }

  /// 局部 ESDF 点云（x,y,z + intensity=距离 m）→ 栅格化距离场 → 交给算法
  void onEsdf(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!has_local_map_ || !local_map_) {
      // 没有几何就不知道往哪栅格化。等局部图（正常顺序：图先到）。
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "[local] 先收到距离场但还没有局部图（%s），等图到再建场",
          topic_local_map_.c_str());
      return;
    }
    if (!frame_id_.empty() && !msg->header.frame_id.empty() &&
        msg->header.frame_id != frame_id_ && !warned_map_frame_) {
      warned_map_frame_ = true;
      RCLCPP_WARN(get_logger(),
                  "[local] 距离场 frame='%s' 与 local.frame_id='%s' "
                  "不一致（不做 TF 变换）",
                  msg->header.frame_id.c_str(), frame_id_.c_str());
    }

    // 解析点云（PointXYZI）：x,y 是世界坐标，intensity 是到最近障碍的距离 [m]
    std::vector<DistanceSample> samples;
    samples.reserve(msg->width * msg->height / 4 + 1);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_d(*msg, "intensity");
      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_d)
        samples.push_back(DistanceSample{static_cast<double>(*it_x),
                                         static_cast<double>(*it_y),
                                         static_cast<double>(*it_d)});
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "[local] 距离场点云解析失败（需要 x/y/intensity "
                            "三个 float32 字段）：%s",
                            e.what());
      return;
    }

    const double ox = local_map_->originX();
    const double oy = local_map_->originY();
    const double res = local_map_->resolution();
    const int w = local_map_->width();
    const int h = local_map_->height();

    // ★ 空点云不是“拿不到距离场”，而是“这一片很开阔”：
    //   感知只发 0 < d ≤ max_dist 的格子，周围没障碍时点云就是空的。
    //   当成降级会让车在空旷处爬行（且原因看起来像感知挂了）。
    bool ok = false;
    if (samples.empty()) {
      ok = dist_field_.buildFree(ox, oy, res, w, h, esdf_max_distance_);
    } else {
      // 采样间距是感知的 esdf_pub_step（默认 2 格 = 0.2 m），比本地图分辨率粗，
      // 所以必须做邻域填充才能得到连续场（见 LocalDistanceField 文件头）。
      ok = dist_field_.buildFromSamples(ox, oy, res, w, h, samples,
                                        esdf_max_distance_, esdf_fill_radius_);
      if (!ok) // 采样全落在本地图外：同样按“开阔”处理
        ok = dist_field_.buildFree(ox, oy, res, w, h, esdf_max_distance_);
    }
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "[local] 距离场构建失败（几何非法？）→ 本周期不更新");
      return;
    }

    has_esdf_ = true;
    esdf_active_ = true;
    last_esdf_time_ = now();
    // ★
    // 禁行区融合（每帧都要做，A2）：距离场是每帧整张重建的，融合是它的**后处理**
    //   ⇒ 天然不会残留上一帧的旧区域，也不需要额外的“失效判定”。
    //   （反过来放在 50 Hz
    //   的规划里做，就会变成“场是新的、区域是旧的”这种最难查的
    //   不一致。）融进去之后，MPC 的障碍软代价/硬下界、以及解后真实几何复核全部
    //   自动把禁行区当障碍 —— 不贴边开、不选穿区的解。
    if (!zones_.empty() && zones_.forbiddenCount() > 0) {
      const std::size_t n =
          dist_field_.fuseForbidden(zones_, zones_local_inflate_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "[local] 禁行区融合进距离场：%zu 格（禁行区 %zu 个，"
                           "膨胀 %.3f m）",
                           n, zones_.forbiddenCount(), zones_local_inflate_);
    }
    planner_->setDistanceField(&dist_field_);
    if (!logged_esdf_) {
      logged_esdf_ = true;
      RCLCPP_INFO(get_logger(),
                  "[local] 收到距离场：%zu 采样 → %dx%d 栅格（命中 %zu "
                  "格，填充 %zu 格）",
                  samples.size(), w, h, dist_field_.sampleCells(),
                  dist_field_.filledCells());
    } else {
      // 节流（不是只打一次）：运维时需要能从日志看出"场还在不在重建"。
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "[local] 距离场重建：%zu 采样 → 命中 %zu 格 / 填充 %zu 格",
          samples.size(), dist_field_.sampleCells(), dist_field_.filledCells());
    }
  }

  // ------------------------------------------------------------ action 回调
  rclcpp_action::GoalResponse
  onGoalRequest(const rclcpp_action::GoalUUID &,
                std::shared_ptr<const FollowPath::Goal> goal) {
    const auto path = pathToPoses(goal->path);
    if (path.size() < 2) {
      RCLCPP_WARN(get_logger(), "[local] 拒绝目标：路径只有 %zu 点（需要 ≥2）",
                  path.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!has_odom_) {
      RCLCPP_WARN(get_logger(), "[local] 拒绝目标：还没有收到位姿（%s）",
                  topic_odom_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // 新目标顶掉旧目标：先把旧的收掉，避免两个循环同时跑
    if (goal_handle_) {
      RCLCPP_WARN(get_logger(), "[local] 收到新目标，终止上一个未完成的跟随");
      endAsCanceled(LocalPlanResult{}, "被新目标顶掉");
    }
    RCLCPP_INFO(
        get_logger(),
        "[local] 接受目标：%zu 点 / %.2f m | 走廊 %s | 限速 %.2f | 严格贴线 %s",
        path.size(), polylineLength(path),
        goal->corridor_width.empty() ? "无（自由空间模式）" : "有",
        goal->speed_limit, goal->strict_corridor ? "是" : "否");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle>) {
    RCLCPP_INFO(get_logger(), "[local] 收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /// 限速区前瞻该用的减速度 [m/s²]。
  ///
  /// **单一来源 = 算法自报的**（`LocalPlanner::brakeAcc()` =
  /// `local_mpc.brake_acc`， 也是终点制动剖面用的那个量）。`local.brake_acc`
  /// 只在算法不报（非 MPC 实现） 时当兜底。
  ///
  /// 为什么必须这么定：两个数各配一份一定会不一致（实测：节点包里写着 0.15 的
  /// 四足标定值，而 `config/local_mpc.yaml` 里算法是 1.0 ⇒ 前瞻距离差 6.7
  /// 倍）， 而表现在现场只是"限速区有时提前减速、有时进区还超速"，极难归因。
  /// 因此：两处都配了且相差 >5% 就 WARN 一次，把两个值和用的哪个写清楚。
  double effectiveBrakeAcc() {
    if (brake_acc_resolved_ > 0.0)
      return brake_acc_resolved_;
    const double from_algo = planner_ ? planner_->brakeAcc() : 0.0;
    const double from_param = brake_acc_param_;
    if (from_algo > 0.0) {
      if (from_param > 0.0 && std::fabs(from_param - from_algo) >
                                  0.05 * std::max(from_param, from_algo)) {
        RCLCPP_WARN(
            get_logger(),
            "[local] 减速度两处不一致：local.brake_acc=%.3f、算法（local_mpc."
            "brake_acc）=%.3f → 限速区前瞻按**算法值**算（它跟终点制动剖面同"
            "一个量）；建议删掉 local.brake_acc 或改成同值",
            from_param, from_algo);
      }
      brake_acc_resolved_ = from_algo;
    } else if (from_param > 0.0) {
      brake_acc_resolved_ = from_param;
    } else {
      // 谁都没报：用一个明确的通用值，并鸣一下（不静默猜）
      brake_acc_resolved_ = 0.5;
      RCLCPP_WARN(
          get_logger(),
          "[local] 算法未自报减速度、local.brake_acc 也没配 → 限速区前瞻暂用 "
          "%.1f m/s²；换底盘请显式配一个实测值",
          brake_acc_resolved_);
    }
    RCLCPP_INFO(get_logger(), "[local] 限速区前瞻减速度 = %.3f m/s²%s",
                brake_acc_resolved_,
                from_algo > 0.0 ? "（算法自报）" : "（local.brake_acc 兜底）");
    return brake_acc_resolved_;
  }

  /// 限速区：每周期沿参考**前瞻**取区内最严限速，作为动态速度帽交给算法。
  ///
  /// 为什么要前瞻而不是"进区再减速"：进区才减速就已经超速了。前瞻距离取
  /// "从当前速度按最大减速度刹停的距离 + 一点余量"，这样**进区时速度已经 ≤
  /// 限速**。
  ///
  /// 为什么是每周期（50 Hz）而不是跟局部图一样 10~20 Hz：它只是一个点级查询
  /// （沿路径几十个点 × 几个区域），成本 ~µs；而区域几何本身是 latched 的，
  /// 几乎不变。**不做逐格扫描**，所以不会把成本带进控制回路。
  void updateZoneSpeedLimit() {
    if (zones_.empty() || zones_.speedCount() == 0) {
      if (planner_->zoneSpeedLimit() > 0.0)
        planner_->setZoneSpeedLimit(0.0);
      return;
    }
    // 前瞻距离：用**可能达到的**速度（min 算法 v_max、任务限速）算，不能用当前
    // 速度 —— 用当前速度会形成自锁："因为开得慢，所以从不减速"（见
    // speedLookahead 的注释；2026-09-23 段 3 实测：0.26 m/s 爬向 0.7 m
    // 外的限速区， 前瞻 0.53 m 永远够不到，最后 0.30 m/s
    // 穿区）。取反馈值与它的大者， 保证"实际速度比预期还快"时也不会算短。
    const double v_now = std::fabs(planner_->currentV());
    const double a = std::max(0.1, effectiveBrakeAcc());
    double v_look = planner_->maxSpeed();
    if (speed_limit_ > 0.0 && v_look > 0.0)
      v_look = std::min(v_look, speed_limit_);
    else if (v_look <= 0.0)
      v_look = v_now; // 算法没报"最大速度"（非 MPC 实现）→ 退回旧行为
    const double look = pnc_2d::speedLookahead(std::max(v_look, v_now), a);
    // ★ 前瞻的起点必须是"车在路径上的**投影点**"，不能用 pass_index_ 那种
    //   "路点下标"：自由空间路径常常只有 2 个点，下标在车走到终点附近之前一直是
    //   0 ⇒ 扫描窗口固定在"路径起点往后 look 米"，与车在哪毫无关系。
    //   实测（2026-09-23 test_zones 段 3）：车开出限速区 1 m 了帽还在，
    //   全程 0.15 m/s 爬 31 s（"出区后恢复"这项验收永远不成立）。
    const double limit = pnc_2d::zoneSpeedLimitAheadFromProjection(
        zones_, plan_, pose_.x, pose_.y, look);
    planner_->setZoneSpeedLimit(limit);
    if (limit > 0.0)
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "[local] 前方 %.2f m 内有 %.2f m/s 限速区 → 本周期速度帽 "
          "%.2f m/s",
          look, limit, limit);
  }

  void onAccepted(const std::shared_ptr<GoalHandle> handle) {
    goal_handle_ = handle;
    const auto goal = handle->get_goal();
    plan_ = pathToPoses(goal->path);
    plan_length_ = polylineLength(plan_);
    speed_limit_ = goal->speed_limit;
    pass_index_ = 0;
    traveled_ = 0.0;
    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    start_time_ = now();
    last_pose_ = pose_;
    has_last_pose_ = true;
    finished_ = false;

    planner_->reset();
    planner_->setGlobalPlan(plan_);
    planner_->setSpeedLimit(speed_limit_);
    // 走廊：逐点半宽 → 切出**真正受约束的那一段**（通常是路网段；hybrid
    // 的自由入口/ 出口段填 -1）。是否**启用**由 updateCorridorMode()
    // 每周期决定，因为走廊只在
    // 车已经（回到）走廊里时才允许启用（见那个函数的注释）。
    corridor_ = RouteCorridor();
    route_slice_ = pnc_2d::corridorSlice(std::vector<double>(
        goal->corridor_width.begin(), goal->corridor_width.end()));
    if (route_slice_.valid && route_slice_.end < plan_.size()) {
      corridor_.centerline.assign(
          plan_.begin() + static_cast<std::ptrdiff_t>(route_slice_.begin),
          plan_.begin() + static_cast<std::ptrdiff_t>(route_slice_.end) + 1);
      corridor_.half_width = route_slice_.half_width;
      corridor_.speed_limit = goal->speed_limit;
      corridor_.edge_index = -1;
      corridor_attached_ = false; // 等 updateCorridorMode() 判定
      planner_->setCorridor(nullptr);
      RCLCPP_INFO(get_logger(),
                  "[local] 走廊段：路径点 [%zu, %zu] / 共 %zu 点，半宽 %.2f m"
                  "（strict=%s）| 前面 %zu 点是自由入口段",
                  route_slice_.begin, route_slice_.end, plan_.size(),
                  corridor_.half_width, goal->strict_corridor ? "是" : "否",
                  route_slice_.begin);
    } else {
      route_slice_ = pnc_2d::CorridorSlice{};
      corridor_attached_ = false;
      planner_->setCorridor(nullptr);
      if (!goal->corridor_width.empty())
        RCLCPP_WARN(
            get_logger(),
            "[local] 收到走廊数组（%zu 个）但没有有效走廊段 → 当自由空间跟踪",
            goal->corridor_width.size());
    }
    publishStatus(LocalStatus::kFollowing, "开始跟随", LocalPlanResult{});
  }

  /// 走廊**只在车已经在走廊里时才启用**（每周期调用）。
  ///
  /// 为什么必须这样（两次实测的根因）：走廊是硬约束，而在硬约束下"从车道外收敛"
  /// 根本不可行 —— 严格走廊（半宽 0.05 m）下起点横向偏差 ≥0.055 m 时求解器不
  /// 收敛（maximum iterations reached）、120/120 周期被挡、车一步不动。
  /// 所以语义拆成两步：
  ///   ① 还没走完自由入口段、或横向已偏出走廊 → **自由模式**沿同一条参考走
  ///      （自由跟踪本身会把车收到中心线上：实测稳态 0.02 m）；
  ///   ② 走完入口段 且 横向已在走廊内 → 启用走廊硬约束（严格贴线从这里开始）。
  /// 两个条件缺一不可：只看"走完入口段"会在路口切内弯时立刻违反硬约束。
  void updateCorridorMode() {
    if (!route_slice_.valid) {
      if (corridor_attached_) {
        corridor_attached_ = false;
        planner_->setCorridor(nullptr);
      }
      return;
    }
    const double lat =
        pnc_2d::distanceToPolyline(pose_.x, pose_.y, corridor_.centerline);
    const double tol =
        std::max(corridor_.half_width, planner_->corridorTolerance());
    const bool in = pass_index_ >= route_slice_.begin && lat <= tol + 1e-6;
    if (in == corridor_attached_)
      return;
    corridor_attached_ = in;
    planner_->setCorridor(in ? &corridor_ : nullptr);
    if (in) {
      RCLCPP_INFO(get_logger(),
                  "[local] 进入走廊 → 严格贴线（横向 %.3f m ≤ %.3f "
                  "m，已过入口段 %zu 点）",
                  lat, tol, route_slice_.begin);
    } else {
      RCLCPP_WARN(get_logger(),
                  "[local] 退出走廊 → 自由跟踪收敛（横向 %.3f m > %.3f m；"
                  "走廊是硬约束，从车道外收敛实测不可行）",
                  lat, tol);
    }
  }

  // ------------------------------------------------------------ 控制循环
  void onControl() {
    if (!goal_handle_ || finished_)
      return;

    // 客户端请求取消（onCancel 已 ACCEPT）→ 在这里收尾
    if (goal_handle_->is_canceling()) {
      endAsCanceled(LocalPlanResult{}, "客户端取消");
      return;
    }

    LocalPlanResult r;
    const bool odom_fresh =
        has_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_;
    if (!odom_fresh) {
      // 位姿过期：绝不"用旧位姿继续算速度"（这是最容易出事故的一类降级）
      r.status = LocalStatus::kDegraded;
      r.message = "位姿过期（超过 " + std::to_string(odom_timeout_) + " s）";
      publishStatus(r.status, r.message, r);
      return;
    }

    // ★ 车已经在禁行区里 ⇒ 立刻停车报 BLOCKED（用户 2026-09-23
    // 的决定：停，不绕）。
    //   为什么要单独查一次：融合进栅格/距离场之后，边缘情况（区域刚被画上、
    //   定位跳变、路径过期）会表现为"求解失败/侵入障碍"这类含糊的报错，
    //   而这里能直接点名区域 —— 现场一眼就知道是区域的问题，不是控制器坏了。
    //   边界附近的正常情况由融合后的 footprint/障碍硬约束负责（不需要这里管）。
    if (!zones_.empty() && zones_.forbiddenCount() > 0 &&
        zones_.inForbidden(pose_.x, pose_.y)) {
      r.status = LocalStatus::kBlocked;
      r.message = "车已在禁行区内（" +
                  zones_.forbiddenNameAt(pose_.x, pose_.y) +
                  "）→ 停车；禁行区不做「绕行」，请由全局重规划或人工挪车";
      publishStatus(r.status, r.message, r);
      publishFeedback(r, r.message);
      finish(LocalStatus::kBlocked, false, true, false, r, r.message);
      return;
    }

    // 距离场新鲜度：过期就按"拿不到距离场"处理（降级 + 限速），
    // 而不是继续拿一张可能已经过期的场去避障（车动/人走后障碍位置早就变了）。
    // 注意：只有**曾经收到过**才谈得上过期；从未收到时算法自己会按"无场"降级。
    if (has_esdf_) {
      const bool fresh = (now() - last_esdf_time_).seconds() <= esdf_timeout_;
      if (fresh != esdf_active_) {
        esdf_active_ = fresh;
        planner_->setDistanceField(fresh ? &dist_field_ : nullptr);
        RCLCPP_WARN(get_logger(), "[local] 距离场%s（超过 %.2f s）",
                    fresh ? "恢复" : "过期", esdf_timeout_);
      }
    }

    const auto t0 = std::chrono::steady_clock::now();
    // ★ 走廊启用判定要在算命令**之前**（本周期就得用对模式）。它依赖
    // pass_index_
    //   （“入口段走完没有”），所以把进度更新提到前面来 —— 语义与原来一样，
    //   只是从"算完再更新"变成"先更新再算"（原来会差一个控制周期）。
    updateProgress();
    updateCorridorMode();
    updateZoneSpeedLimit();
    r = planner_->computeCommand(pose_, 1.0 / control_rate_);
    r.stats.solve_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t0)
                           .count();

    // ★ 1 Hz 调参诊断：算法自己把内部关键量格式化好（见
    // LocalPlanner::diagString）。
    //   没有这一行，"为什么只发 0.13 m/s"、"为什么一直在扭"只能靠猜 —— 限速、
    //   曲率前瞻、速度反馈不对…… 表现出来都是"走得慢"。
    {
      const std::string diag = planner_->diagString();
      if (!diag.empty())
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "[local] cmd v=%.3f w=%.3f | %s", r.cmd.v, r.cmd.w,
                             diag.c_str());
    }

    // 路程统计（用于 result.traveled_m）
    if (has_last_pose_) {
      traveled_ += std::hypot(pose_.x - last_pose_.x, pose_.y - last_pose_.y);
      last_pose_ = pose_;
    }

    // 进度：找到最近点，把"已越过的点"推进过去
    // （已提到算命令之前，见那里的注释：走廊启用判定需要最新进度）

    // 只有算法声明会发速度时才发：NullLocalPlanner 的 cmd=0 语义是"我不管"
    if (pub_cmd_vel_) {
      geometry_msgs::msg::Twist tw;
      tw.linear.x = r.cmd.v;
      tw.angular.z = r.cmd.w;
      pub_cmd_vel_->publish(tw);
    }

    switch (r.status) {
    case LocalStatus::kGoalReached: {
      const double remain = remaining();
      if (finishReached(remain)) {
        finish(LocalStatus::kGoalReached, true, false, false, r, "到达目标");
      } else {
        // 局部说"到终点了"但离用户目标还远（路径终点 ≠ 用户目标）→ 不擅自算成功
        finish(LocalStatus::kFailed, false, false, true, r,
               "局部到达路径终点，但离用户目标还有 " + std::to_string(remain) +
                   " m");
      }
      return;
    }
    case LocalStatus::kBlocked: {
      if (blocked_since_.nanoseconds() == 0)
        blocked_since_ = now();
      const double blocked_for = (now() - blocked_since_).seconds();
      publishStatus(r.status, r.message, r);
      publishFeedback(r, r.message);
      if (blocked_for >= blocked_abort_s_) {
        finish(LocalStatus::kBlocked, false, true, false, r,
               "连续被挡 " + std::to_string(blocked_for) + " s 仍不可绕");
      }
      return;
    }
    case LocalStatus::kFailed: {
      finish(LocalStatus::kFailed, false, false, true, r, r.message);
      return;
    }
    case LocalStatus::kIdle:
    case LocalStatus::kFollowing:
    case LocalStatus::kDegraded:
    default:
      blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME); // 障碍消失 → 计时清零
      publishStatus(r.status, r.message, r);
      publishFeedback(r, r.message);
      // ★ 节点层的"到达"兜底：**不依赖算法自己报 kGoalReached**。
      //   理由：① 到位判定是任务语义（"到没到用户给的点"），不是控制语义；
      //   ② 任何局部算法忘了报（P4 的 NullLocalPlanner 就是：它只回报状态、
      //      不做控制，永远返回 kFollowing），任务就会**永远不结束** ——
      //      状态机会一直停在 FOLLOWING。这类"链条末端没人收尾"的问题最隐蔽。
      //   代价：算法报"到了"但车其实没到（会被 kGoalReached 分支当成失败）时，
      //   这里仍以位置为准；两者冲突在下面的分支里显式报出来。
      if (finishReached(remaining()))
        finish(LocalStatus::kGoalReached, true, false, false, r,
               "距目标 ≤ " + std::to_string(goal_tolerance_) +
                   " m（已扣除停车惯性 " +
                   std::to_string(planner_->stopCoast()) + " m；节点层判定）");
      return;
    }
  }

  void updateProgress() {
    // 最简单的"最近点 + 单向推进"：越过的路径点不再回头（与 P5 的 MPC 无关，
    // 只用于汇报进度/剩余距离）
    const std::size_t n = plan_.size();
    std::size_t best = pass_index_;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = pass_index_; i < n; ++i) {
      const double d = std::hypot(plan_[i].x - pose_.x, plan_[i].y - pose_.y);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    if (best_d <= pass_distance_)
      pass_index_ = best;
  }

  double remaining() const {
    double rem = std::hypot(plan_.back().x - pose_.x, plan_.back().y - pose_.y);
    for (std::size_t i = pass_index_ + 1; i < plan_.size(); ++i)
      rem +=
          std::hypot(plan_[i].x - plan_[i - 1].x, plan_[i].y - plan_[i - 1].y);
    return rem;
  }

  /// 到点判定：比的是"**预判停点**到目标的距离"，不是当前距离。
  ///
  /// 为什么必须扣掉停车惯性（`stopCoast()`）：底盘的步态在指令归零后还会自己
  /// 走 ~4 cm（实测）。若直接拿"当前距离 ≤ 容差"判定，车一定冲过目标那 4 cm
  /// （实测停点误差 0.024~0.054 m，用户要求 ≤ 3 cm）；扣掉之后指令提前归零，
  /// 惯性正好把最后几厘米带完。
  ///
  /// ★ 2026-09-23 加上**目标朝向**条件（用户指出：原来只管
  /// xy，机头朝哪都算到）：
  ///   位置进容差、朝向也进容差才算到达。容差取自算法（`goalYawTolerance()`）：
  ///   算法自己就靠它做"到点后原地对正"，两边各配一份就会出现
  ///   "算法在 5° 对正、节点按 2° 判" ⇒ 永远不满足 ⇒ 任务卡到超时。
  ///   朝向还没对好时**不判到达**，只打一行日志（算法正在原地转），
  ///   任务自然多走几个周期。
  bool finishReached(double remain) {
    if (remain - planner_->stopCoast() > goal_tolerance_)
      return false;
    const double tol = planner_->goalYawTolerance(); // [rad]，0 = 不判朝向
    if (tol <= 0.0)
      return true;
    // 路径末点没带朝向（纯路径点）⇒ 这个任务本来就没有朝向要求，不判
    if (plan_.empty() || !plan_.back().has_yaw)
      return true;
    const double e = std::fabs(pnc_2d::wrapAngle(plan_.back().yaw - pose_.yaw));
    if (e <= tol)
      return true;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[local] 已到点位置，正在原地对正目标朝向（偏差 %.1f° > "
        "%.1f°）",
        e * 180.0 / M_PI, tol * 180.0 / M_PI);
    return false;
  }

  // ------------------------------------------------------------ 停车
  /// 发几次零速。
  ///
  /// ★ 为什么必须有：底盘的步态控制器**不会因为没人发指令就自己停**，
  ///   它会保持最后一条指令。
  ///   · 任务结束时（到点/失败/被挡/取消）节点就不再发速度了
  ///     ⇒ 车按最后那个速度一直走（验收里"到点后末速 0.264 m/s"就是这个）；
  ///   · 进程退出/被中断时同理。
  ///   发几次是因为最后一两条可能正赶上控制器消费/被覆盖（成本极低）。
  ///   注意：这是**规划器侧的义务**，不代替底盘侧应有的 cmd_vel 超时看门狗
  ///   （控制器在 X ms 没收到指令就该自己停——那是更根本的傅底）。
  void publishStop() {
    if (!pub_cmd_vel_)
      return;
    geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      pub_cmd_vel_->publish(zero);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  /// 结束本次跟随。`end_status` 是**对外的结束状态**，不一定等于算法内部状态：
  /// 例如算法报 kGoalReached 但离用户目标还远时，对外应是 FAILED。
  void finish(LocalStatus end_status, bool reached, bool blocked, bool failed,
              const LocalPlanResult &r, const std::string &why) {
    if (!goal_handle_ || finished_)
      return;
    finished_ = true;
    publishStop(); // 先停车再收尾（顺序反了就没人发了）

    auto res = std::make_shared<FollowPath::Result>();
    res->goal_reached = reached;
    res->blocked = blocked;
    res->failed = failed;
    res->canceled = false; // 取消不走这里（见 onCancel 路径）
    res->status = static_cast<uint8_t>(end_status);
    res->status_name = toString(end_status);
    res->message = why;
    res->elapsed_s = (now() - start_time_).seconds();
    res->traveled_m = traveled_;
    res->final_cross_track_m = r.stats.cross_track;
    res->progress = r.stats.progress;

    if (reached) {
      RCLCPP_INFO(get_logger(),
                  "[local] 到达目标：%.2f s / 走了 %.2f m / 路径 %.2f m",
                  res->elapsed_s, res->traveled_m, plan_length_);
      goal_handle_->succeed(res);
    } else {
      RCLCPP_WARN(get_logger(), "[local] 跟随结束（%s）：%s",
                  blocked ? "被挡" : "失败", why.c_str());
      goal_handle_->abort(res);
    }
    publishStatus(end_status, why, r);
    resetFollow();
  }

  /// 取消 / 服务端强制停止的收尾。
  ///
  /// ⚠ `ServerGoalHandle::canceled()` **只在 `is_canceling()` 为真时才合法**
  /// （否则抛
  /// UnawareGoalHandleError）。服务端自己发起的停止（`~/stop`、被新目标
  /// 顶掉）拿不到 canceling 状态，只能走 `abort()` —— 两种情况都用
  /// result.canceled=true 标清楚"这不是失败"，调用方就不会把它当成出错。
  void endAsCanceled(const LocalPlanResult &r, const std::string &why) {
    if (!goal_handle_)
      return;
    auto res = std::make_shared<FollowPath::Result>();
    res->canceled = true;
    res->status = static_cast<uint8_t>(LocalStatus::kIdle);
    res->status_name = toString(LocalStatus::kIdle);
    res->message = why;
    res->elapsed_s = (now() - start_time_).seconds();
    res->traveled_m = traveled_;
    res->progress = r.stats.progress;

    const bool via_cancel = goal_handle_->is_canceling();
    RCLCPP_INFO(get_logger(),
                "[local] 跟随结束：%s（已走 %.2f m，收尾方式 %s）", why.c_str(),
                res->traveled_m, via_cancel ? "canceled" : "abort");
    if (via_cancel)
      goal_handle_->canceled(res);
    else
      goal_handle_->abort(res);

    finished_ = true;
    publishStop(); // 取消/被叫停也要停（否则车会抱着旧指令继续走）
    publishStatus(LocalStatus::kIdle, why, r);
    resetFollow();
  }

  void resetFollow() {
    goal_handle_.reset();
    plan_.clear();
    planner_->reset();
    pass_index_ = 0;
    traveled_ = 0.0;
    finished_ = false;
    blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  // ------------------------------------------------------------ 输出
  void publishStatus(LocalStatus st, const std::string &msg,
                     const LocalPlanResult &r) {
    if (!pub_status_)
      return;
    pnc_2d::msg::LocalStatus m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.status = static_cast<uint8_t>(st);
    m.status_name = toString(st);
    m.message = msg;
    m.produces_cmd_vel = planner_ ? planner_->producesCmdVel() : false;
    m.cmd_v = r.cmd.v;
    m.cmd_w = r.cmd.w;
    m.solve_time_ms = r.stats.solve_ms;
    m.solver_iter = r.stats.solver_iter;
    m.cross_track_m = r.stats.cross_track;
    m.progress = r.stats.progress;
    m.time_to_goal_s = r.stats.time_to_goal;
    m.path_points = static_cast<uint32_t>(plan_.size());
    m.route_mode = planner_ && planner_->mode() == LocalPlanner::Mode::kRoute;
    pub_status_->publish(m);
  }

  void publishFeedback(const LocalPlanResult &r, const std::string &msg) {
    if (!goal_handle_ || finished_)
      return;
    auto fb = std::make_shared<FollowPath::Feedback>();
    fb->header.stamp = now();
    fb->header.frame_id = frame_id_;
    fb->status = static_cast<uint8_t>(r.status);
    fb->status_name = toString(r.status);
    fb->message = msg;
    fb->progress = plan_.size() < 2 ? 0.0
                                    : static_cast<double>(pass_index_) /
                                          static_cast<double>(plan_.size() - 1);
    fb->cross_track_m = r.stats.cross_track;
    fb->remaining_m = remaining();
    fb->cmd_v = r.cmd.v;
    fb->cmd_w = r.cmd.w;
    fb->blocked = (r.status == LocalStatus::kBlocked);
    fb->blocked_for_s = blocked_since_.nanoseconds() == 0
                            ? 0.0
                            : (now() - blocked_since_).seconds();
    fb->solve_time_ms = r.stats.solve_ms;
    goal_handle_->publish_feedback(fb);
  }

  // ------------------------------------------------------------ 参数读取
  std::string paramString(const std::string &key, const std::string &def) {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_string();
  }
  double paramDouble(const std::string &key, double def) {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_double();
  }
  bool paramBool(const std::string &key, bool def) {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_bool();
  }
  /// ★ 整数参数必须用这个读：ROS 2 参数是**带类型**的，yaml 里写 `2` 是
  /// integer，
  ///   拿 as_double() 读会抛 ParameterTypeException 把节点直接搞崩
  ///   （踩过：local.esdf_fill_radius）。
  int paramInt(const std::string &key, int def) {
    if (!has_parameter(key))
      declare_parameter(key, static_cast<int64_t>(def));
    return static_cast<int>(get_parameter(key).as_int());
  }

  // ------------------------------------------------------------ 成员
  std::string local_type_;
  std::string frame_id_;
  std::string topic_odom_;
  std::string topic_cmd_vel_;
  std::string topic_status_;
  std::string topic_local_map_;
  std::string topic_esdf_;
  std::string topic_zones_;

  double control_rate_{20.0};
  double goal_tolerance_{0.30};
  double blocked_abort_s_{1.0};
  double pass_distance_{0.50};
  double odom_timeout_{1.0};
  double esdf_timeout_{0.3};
  double esdf_max_distance_{3.0};
  int esdf_fill_radius_{2};

  // 速度反馈
  bool vel_from_pose_{false};
  double vel_pose_window_{0.25};
  double vel_filter_tau_{0.15};
  double v_filt_{0.0}, w_filt_{0.0};
  double vel_prev_t_{0.0};
  bool has_vel_prev_{false};
  struct PoseSample {
    double t, x, y, yaw;
  };
  std::deque<PoseSample> pose_hist_;

  std::unique_ptr<LocalPlanner> planner_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_local_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_esdf_;
  rclcpp::Subscription<pnc_2d::msg::ZoneArray>::SharedPtr sub_zones_;
  rclcpp::Publisher<pnc_2d::msg::LocalStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::Service<pnc_2d::srv::SwitchPlanner>::SharedPtr srv_switch_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reload_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
  rclcpp::TimerBase::SharedPtr timer_;

  Pose2D pose_;
  bool has_odom_{false};
  bool warned_odom_frame_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  // 局部图 / 距离场（P5.3）
  std::shared_ptr<CostMap2D> local_map_;
  LocalDistanceField dist_field_;
  bool has_local_map_{false};
  bool has_esdf_{false};
  bool esdf_active_{false}; ///< 距离场当前是否“新鲜”（超时则置 false）
  rclcpp::Time last_esdf_time_{0, 0, RCL_ROS_TIME};
  bool warned_map_frame_{false};
  bool logged_local_map_{false};
  bool logged_esdf_{false};

  std::shared_ptr<GoalHandle> goal_handle_;
  std::vector<Pose2D> plan_;
  RouteCorridor corridor_;
  /// 区域层（禁行区/限速区）：由 map_server latched 发过来；空 = 没区域/没收到
  pnc_2d::ZoneSet zones_;
  /// 区域的膨胀量 [m]：**与 map_server 烧全局图用的是同一个值**（消息带过来），
  /// 两边必须一致，否则同一个区域全局说能过、局部说不能。
  double zone_inflate_msg_{0.0};   // /global_map/zones 消息里报的 inflate（仅记录/告警）
  /// 局部侧额外膨胀 [m]：**默认 0** —— 全局图已经把区域按 zones.inflate 膨胀过了，
  /// 局部再叠加同一个值 ⇒ 两边判据完全相等（零余量），任何栅格离散化/图差都会
  /// 变成"全局给路、局部在障碍里" ⇒ BLOCKED ⇒ 恢复重规划又因起点余量重叠失败
  /// ⇒ 任务死锁（2026-09-23 实测）。与 "topics.local_map 用未膨胀图" 完全同构。
  double zones_local_inflate_{0.0};
  /// 限速区前瞻用的减速度 [m/s²] 的**兜底值**（算法不报时用它）
  double brake_acc_param_{0.0};
  /// `effectiveBrakeAcc()` 的缓存（-1 = 还没算过），同时保证只 WARN 一次
  double brake_acc_resolved_{-1.0};
  /// 逐点走廊切出来的一段（下标指到 `plan_`；自由入口/出口段不算在里面）
  pnc_2d::CorridorSlice route_slice_;
  /// 本周期是否已经把走廊交给算法（走廊只在车已在走廊里时才启用，见
  /// updateCorridorMode；用成员记录是为了只在切换时打日志/重设指针）
  bool corridor_attached_{false};
  double plan_length_{0.0};
  double speed_limit_{0.0};
  std::size_t pass_index_{0};
  double traveled_{0.0};
  Pose2D last_pose_;
  bool has_last_pose_{false};
  bool finished_{false};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
};

} // namespace pnc_2d

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_2d::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
