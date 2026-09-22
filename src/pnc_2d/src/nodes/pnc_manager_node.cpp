// 任务状态机节点（ROS 2）—— 三节点的主控（P4）。
//
// 它是 doc/pnc2d_restructure_plan.md §5 数据流里的 MGR：
//   /goal_pose ──┐
//                ├─→ manager ──service PlanPath──→ global_planner_node
//   odom ────────┘        └──action FollowPath──→ local_planner_node
//                                        └─→ /pnc_2d/cmd_vel ──→ 底盘
//
// ★ 本节点**不产生速度指令**（P4：local 只跑 Null 实现，链路空跑）。
//   cmd_vel 的唯一owner 是 local 节点：谁控制车谁发速度，两个节点都发就会打架。
//
// 实现约定：
//   · 状态机逻辑全在 sm/（纯库、可单测）；本文件只做两件翻译：
//        ROS 世界 → 事件（喂给 ManagerSm::handle）
//        SideEffect → ROS 调用（服务/动作/日志）
//   · 回调**不阻塞**：PlanPath 与 FollowPath 都用异步调用 + 回调，
//     并且用**序号**丢掉过期响应（"上一个目标的规划结果属于上一个任务"）。
//   · 单线程 executor + wall timer 轮询：状态机是"事件 + 心跳"驱动的，
//     不需要多线程（也就不需要为 sm_ 加锁）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pnc_2d/action/follow_path.hpp"
#include "pnc_2d/core/types.hpp"
#include "pnc_2d/msg/manager_state.hpp"
#include "pnc_2d/sm/manager_sm.hpp"
#include "pnc_2d/srv/plan_path.hpp"
#include "ros_param_reader.hpp"

namespace pnc_2d {
namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

using FollowPath = pnc_2d::action::FollowPath;

class PncManagerNode : public rclcpp::Node {
public:
  PncManagerNode()
      : rclcpp::Node(
            "pnc_manager",
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)) {
    frame_id_ = paramString("sm.frame_id", "map");
    topic_goal_ = paramString("topics.goal", "/goal_pose");
    topic_odom_ = paramString("topics.odom", "/lightning/perception/pose");
    topic_state_ = paramString("topics.state", "/pnc_2d/state");
    srv_global_plan_ = paramString("sm.global_plan_service", "/global_planner/plan_path");
    act_local_follow_ = paramString("sm.local_follow_action", "/local_planner/follow_path");
    srv_global_clear_ = paramString("sm.global_clear_service", "/global_planner/clear_path");
    srv_local_stop_ = paramString("sm.local_stop_service", "/local_planner/stop");

    SmParams sp;
    sp.max_recoveries = paramInt("sm.max_recoveries", 2);
    sp.max_plan_failures = paramInt("sm.max_plan_failures", 0);
    sm_ = ManagerSm(sp);

    // 卡住判据：位置参照点长时间没推动 → Stuck。
    // 用**自车位移**而不是局部反馈的进度：不依赖局部实现，也能抓到"局部说在走、
    // 其实在原地"的情况。
    stuck_timeout_ = paramDouble("sm.stuck_timeout", 10.0);
    stuck_min_progress_ = paramDouble("sm.stuck_min_progress", 0.20);
    odom_jump_threshold_ = paramDouble("sm.odom_jump_threshold", 1.0);
    // 跳变冷却："跳变"是异常事件，但它可能**连续**发生（例如同时有两个定位源
    // 在发、或重定位后恢复期内）。没有冷却的话，每次跳变都会触发一次重规划，
    // 形成"重规划风暴"—— 真踩过：20 Hz 位姿 × 两条源交替 → 每秒几十次事件。
    odom_jump_cooldown_ = paramDouble("sm.odom_jump_cooldown", 2.0);
    heartbeat_hz_ = paramDouble("sm.heartbeat_hz", 5.0);

    pub_state_ = create_publisher<pnc_2d::msg::ManagerState>(
        topic_state_, rclcpp::QoS(1).transient_local());

    auto odom_qos = rclcpp::SensorDataQoS();  // 定位是 best_effort，必须匹配
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, odom_qos,
        std::bind(&PncManagerNode::onOdom, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_goal_, 1, std::bind(&PncManagerNode::onGoal, this, std::placeholders::_1));

    cli_plan_ = create_client<pnc_2d::srv::PlanPath>(srv_global_plan_);
    cli_clear_ = create_client<std_srvs::srv::Trigger>(srv_global_clear_);
    cli_stop_ = create_client<std_srvs::srv::Trigger>(srv_local_stop_);
    cli_follow_ = rclcpp_action::create_client<FollowPath>(this, act_local_follow_);

    srv_cancel_ = create_service<std_srvs::srv::Trigger>(
        "~/cancel",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = true;
          res->message = std::string("状态 ") + toString(sm_.state()) +
                         (postEvent(Event::kCancel, "服务 ~/cancel")
                              ? "：已取消"
                              : "：取消被状态机拒绝");
        });

    RCLCPP_INFO(get_logger(), "[sm] frame=%s | 目标 %s | 位姿 %s | 状态 %s",
                frame_id_.c_str(), topic_goal_.c_str(), topic_odom_.c_str(),
                topic_state_.c_str());
    RCLCPP_INFO(get_logger(), "[sm] 全局 service %s | 局部 action %s",
                srv_global_plan_.c_str(), act_local_follow_.c_str());
    RCLCPP_INFO(get_logger(),
                "[sm] 上限：恢复 %d 次/任务 | 规划重试 %d 次 | 卡住判据 %.1f s 内位移 < %.2f m",
                sp.max_recoveries, sp.max_plan_failures, stuck_timeout_,
                stuck_min_progress_);

    publishState();

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / heartbeat_hz_),
                               std::bind(&PncManagerNode::onHeartbeat, this));
  }

private:
  // ============================================================ 事件入口
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(get_logger(),
                  "[sm] 目标 frame='%s' != '%s'，拒用（请把 RViz Fixed Frame 设为 %s）",
                  msg->header.frame_id.c_str(), frame_id_.c_str(), frame_id_.c_str());
      return;
    }
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = yawFromQuaternion(msg->pose.orientation);
    goal_.has_yaw = true;
    RCLCPP_INFO(get_logger(), "[sm] 收到目标 (%.2f, %.2f, %.1f°)",
                goal_.x, goal_.y, goal_.yaw * 180.0 / M_PI);
    postEvent(Event::kGoalReceived, "收到 /goal_pose");
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Pose2D prev = pose_;
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.yaw = yawFromQuaternion(msg->pose.pose.orientation);

    if (!has_odom_) {
      has_odom_ = true;
      last_pose_ = pose_;
      return;
    }

    // 定位跳变：两帧之间位移大于阈值。正常步态不会一帧跳 1 m，
    // 一旦发生说明定位重定位了/坐标跳了 → 基于旧位姿的路径不可信。
    const double jump = std::hypot(pose_.x - prev.x, pose_.y - prev.y);
    if (odom_jump_threshold_ > 0.0 && jump > odom_jump_threshold_) {
      const double since = (now() - last_jump_time_).seconds();
      if (odom_jump_cooldown_ <= 0.0 || since >= odom_jump_cooldown_) {
        last_jump_time_ = now();
        RCLCPP_WARN(get_logger(), "[sm] 定位跳变 %.2f m（> %.2f）", jump,
                    odom_jump_threshold_);
        postEvent(Event::kOdomJump, "定位跳变");
      } else {
        // 冷却期内：只在 DEBUG 里记，避免刷屏（也不会重复触发重规划）
        RCLCPP_DEBUG(get_logger(), "[sm] 定位跳变 %.2f m，冷却中（%.1f s）",
                     jump, since);
      }
    }
    last_pose_ = pose_;
  }

  /// 心跳：推进"卡住"判据 + 周期刷新状态话题（latched 也要让后连接的能看到）
  void onHeartbeat()
  {
    if (sm_.state() == State::kFollowing) {
      const double moved = std::hypot(pose_.x - stuck_anchor_.x,
                                      pose_.y - stuck_anchor_.y);
      if (moved > stuck_min_progress_) {
        // 有推进：把参照点挪到当前位置，计时重新开始
        stuck_anchor_ = pose_;
        stuck_since_ = now();
      } else if (stuck_timeout_ > 0.0 &&
                 (now() - stuck_since_).seconds() > stuck_timeout_) {
        RCLCPP_WARN(get_logger(),
                    "[sm] 卡住：%.1f s 内位移 %.2f m（< %.2f）",
                    (now() - stuck_since_).seconds(), moved, stuck_min_progress_);
        postEvent(Event::kStuck, "长时间没有推进");
      }
    }
    publishState();
  }

  // ============================================================ 状态机桥接
  /// 喂事件并执行该事件要求的动作。返回事件是否被状态机接受。
  bool postEvent(Event e, const char *why)
  {
    const State before = sm_.state();
    const Transition tr = sm_.handle(e);
    if (!tr.accepted) {
      RCLCPP_DEBUG(get_logger(), "[sm] 忽略事件 %s：%s", toString(e),
                   tr.reason.c_str());
      return false;
    }
    if (tr.changed())
      RCLCPP_INFO(get_logger(), "[sm] %s --%s--> %s  [%s]", toString(before),
                  toString(e), toString(tr.to), why);
    else
      RCLCPP_INFO(get_logger(), "[sm] %s --%s--> %s  [%s]", toString(before),
                  toString(e), tr.reason.c_str(), why);

    publishState();
    runEffect(tr.effect);
    return true;
  }

  /// SideEffect → 真的去调 ROS。这里是"状态机说什么就做什么"的唯一出口。
  void runEffect(SideEffect eff)
  {
    switch (eff) {
      case SideEffect::kPlanPath:
        doPlanPath();
        break;
      case SideEffect::kStartFollow:
        doStartFollow();
        break;
      case SideEffect::kStopRobot:
        doStopRobot();
        break;
      case SideEffect::kRunRecovery:
        doRecovery();
        break;
      case SideEffect::kNone:
        break;
    }
  }

  // ------------------------------------------------------------ 副作用实现
  void doPlanPath()
  {
    ++plan_seq_;                 // 本次请求的序号：回调用它丢掉过期响应
    const int seq = plan_seq_;

    if (!has_odom_) {
      RCLCPP_ERROR(get_logger(),
                   "[sm] 规划失败：还没收到位姿（%s）—— 定位没起或 QoS 不匹配",
                   topic_odom_.c_str());
      postEvent(Event::kPlanFail, "没有位姿");
      return;
    }
    if (!cli_plan_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "[sm] 规划失败：全局规划服务 %s 不可用",
                   srv_global_plan_.c_str());
      postEvent(Event::kPlanFail, "全局规划服务不可用");
      return;
    }

    auto req = std::make_shared<pnc_2d::srv::PlanPath::Request>();
    req->header.stamp = now();
    req->header.frame_id = frame_id_;
    req->goal.header.stamp = now();
    req->goal.header.frame_id = frame_id_;
    req->goal.pose.position.x = goal_.x;
    req->goal.pose.position.y = goal_.y;
    req->goal.pose.orientation.z = std::sin(0.5 * goal_.yaw);
    req->goal.pose.orientation.w = std::cos(0.5 * goal_.yaw);
    req->use_current_pose = true;      // 起点由全局节点用最新定位定
    req->publish_result = publish_plan_;  // RViz 可视化（默认开）

    RCLCPP_INFO(get_logger(), "[sm] → 请求全局规划（目标 %.2f, %.2f）", goal_.x,
                goal_.y);
    cli_plan_->async_send_request(
        req, [this, seq](rclcpp::Client<pnc_2d::srv::PlanPath>::SharedFuture f) {
          onPlanResponse(seq, f);
        });
  }

  void onPlanResponse(int seq,
                      rclcpp::Client<pnc_2d::srv::PlanPath>::SharedFuture fut)
  {
    if (seq != plan_seq_) {
      // 过期响应：用户已经给了新目标 / 任务已被取消。**必须丢**，
      // 否则"上一个目标的路径"会覆盖当前任务。
      RCLCPP_WARN(get_logger(), "[sm] 丢弃过期的规划响应（#%d，当前 #%d）", seq,
                  plan_seq_);
      return;
    }
    const auto res = fut.get();
    if (!res->success) {
      RCLCPP_ERROR(get_logger(), "[sm] 全局规划失败：%s（%s）",
                   res->status_name.c_str(), res->message.c_str());
      postEvent(Event::kPlanFail, res->message.c_str());
      return;
    }
    path_ = res->path;
    // 走廊（P5.3）：从全局规划结果透传给局部。局部把**整条路径**当中心线、
    // 宽度取数组最小值，所以这里按路径点数展开成等宽数组即可（长度对齐是
    // 为了将来支持"分段不同宽度"时字段语义不用改）。
    has_corridor_ = res->has_corridor;
    corridor_half_width_ = res->corridor_half_width;
    corridor_speed_limit_ = res->corridor_speed_limit;
    strict_corridor_ = res->strict_corridor;
    route_edges_.assign(res->route_edges.begin(), res->route_edges.end());
    RCLCPP_INFO(get_logger(),
                "[sm] 全局规划成功：%zu 点 / %.1f ms | 走廊 %s"
                "（半宽 %.2f m，限速 %.2f m/s，%s）| 通道 %zu 条",
                path_.poses.size(), res->plan_time_ms,
                has_corridor_ ? "有" : "无", corridor_half_width_,
                corridor_speed_limit_, strict_corridor_ ? "严格贴线" : "允许绕障",
                route_edges_.size());
    postEvent(Event::kPlanOk, "路径就绪");
  }

  void doStartFollow()
  {
    if (path_.poses.size() < 2) {
      RCLCPP_ERROR(get_logger(), "[sm] 无法开始跟随：路径只有 %zu 点",
                   path_.poses.size());
      postEvent(Event::kFollowFail, "路径点数不足");
      return;
    }
    if (!cli_follow_->action_server_is_ready()) {
      RCLCPP_ERROR(get_logger(), "[sm] 跟随失败：action %s 不可用",
                   act_local_follow_.c_str());
      postEvent(Event::kFollowFail, "局部 action 不可用");
      return;
    }

    ++follow_seq_;
    const int seq = follow_seq_;

    auto goal = FollowPath::Goal();
    goal.header = path_.header;
    goal.path = path_;
    // 走廊（P5.3）：来自全局规划结果。
    //   · has_corridor=false（A* 自由空间）→ 数组留空，局部按自由空间跟踪；
    //   · has_corridor=true（路网通道）→ 每个路径点都带同一个半宽，局部把它当
    //     "允许横向偏离 ±W" 的硬约束 → 贴线走。
    // ★ 没接通之前这里一直 clear()，导致 route 模式的走廊约束**从未生效**过：
    //   路径确实是沿通道算的，但局部只会"尽量跟"，可以自由绕障跑到通道外面。
    goal.corridor_width.clear();
    if (has_corridor_)
      goal.corridor_width.assign(path_.poses.size(), corridor_half_width_);
    goal.speed_limit = speed_limit_;
    if (has_corridor_ && corridor_speed_limit_ > 0.0) {
      // 通道限速与全局限速（限速区）取更保守的那个：谁小听谁的
      goal.speed_limit = (speed_limit_ > 0.0)
                             ? std::min(speed_limit_, corridor_speed_limit_)
                             : corridor_speed_limit_;
    }
    goal.strict_corridor = strict_corridor_;

    // 卡住判据的参照点：从"开始跟随"这一刻算起
    stuck_anchor_ = pose_;
    stuck_since_ = now();

    rclcpp_action::Client<FollowPath>::SendGoalOptions opt;
    opt.goal_response_callback = [this, seq](const auto &handle) {
      onFollowGoalResponse(seq, handle);
    };
    opt.feedback_callback = [this, seq](auto, const auto &fb) {
      onFollowFeedback(seq, fb);
    };
    opt.result_callback = [this, seq](const auto &wrapped) {
      onFollowResult(seq, wrapped);
    };

    RCLCPP_INFO(get_logger(), "[sm] → 开始跟随（%zu 点）", goal.path.poses.size());
    cli_follow_->async_send_goal(goal, opt);
  }

  void onFollowGoalResponse(int seq,
                            const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr &h)
  {
    if (seq != follow_seq_)
      return;
    if (!h) {
      RCLCPP_ERROR(get_logger(), "[sm] 局部节点拒绝了跟随目标");
      postEvent(Event::kFollowFail, "局部拒绝目标");
      return;
    }
    follow_handle_ = h;
  }

  void onFollowFeedback(int seq, const std::shared_ptr<const FollowPath::Feedback> fb)
  {
    if (seq != follow_seq_ || !fb)
      return;
    last_feedback_ = fb;
    // 只在"变了"的时候打日志，20 Hz 的反馈不能每个周期都刷
    if (!fb->message.empty() && fb->message != last_feedback_msg_) {
      last_feedback_msg_ = fb->message;
      RCLCPP_INFO(get_logger(), "[sm] ← 局部反馈：%s（%s，进度 %.0f%%，剩余 %.2f m）",
                  fb->message.c_str(), fb->status_name.c_str(),
                  fb->progress * 100.0, fb->remaining_m);
    }
  }

  void onFollowResult(int seq,
                      const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult &w)
  {
    if (seq != follow_seq_) {
      RCLCPP_WARN(get_logger(), "[sm] 丢弃过期的跟随结果（#%d，当前 #%d）", seq,
                  follow_seq_);
      return;
    }
    follow_handle_.reset();

    // 结果判读顺序很重要：**先看语义标志，再看 action 的 code**。
    // 因为服务端强制停止走的是 abort（canceled() 只在 canceling 时合法），
    // 只看 code 会把"被叫停"误判成"失败"。
    const auto res = w.result;
    if (res && res->canceled) {
      RCLCPP_INFO(get_logger(), "[sm] 跟随被取消：%s", res->message.c_str());
      return;   // 取消通常由状态机自己发起（Cancel 事件已处理过状态）
    }
    if (res && res->goal_reached) {
      RCLCPP_INFO(get_logger(), "[sm] 到达目标：%.2f s / 走了 %.2f m",
                  res->elapsed_s, res->traveled_m);
      postEvent(Event::kReached, "局部报到达");
      return;
    }
    if (res && res->blocked) {
      RCLCPP_WARN(get_logger(), "[sm] 跟随被挡：%s", res->message.c_str());
      postEvent(Event::kBlocked, res->message.c_str());
      return;
    }
    RCLCPP_ERROR(get_logger(), "[sm] 跟随失败：%s",
                 res ? res->message.c_str() : "(没有 result)");
    postEvent(Event::kFollowFail, res ? res->message.c_str() : "无结果");
  }

  /// 停车：取消跟随 action（cmd_vel 的唯一 owner 是 local 节点，
  /// 这里**不**自己发零速 —— 两个节点都发速度会打架）。
  void doStopRobot()
  {
    if (follow_handle_) {
      RCLCPP_INFO(get_logger(), "[sm] 停止跟随（取消 action）");
      cli_follow_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();
    }
    ++follow_seq_;   // 让旧的回调失效
  }

  /// 恢复行为。**P4 没有实现**：工件里 availableRecoveries() 是空的（P6 才做）。
  /// 这里如实报告"没有可用行为"，状态机会按恢复次数上限走到 FAILED。
  void doRecovery()
  {
    // 先兜底清掉局部状态与全局旧路径：不管有没有恢复行为，"上次的路径已经
    // 不可信"这件事必须让下游知道。
    if (cli_stop_->service_is_ready())
      cli_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (cli_clear_->service_is_ready())
      cli_clear_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_WARN(get_logger(),
                "[sm] 恢复行为：本阶段（P4）没有可用实现（P6 才做）→ 直接算失败。"
                "已顺手清空局部跟随与全局旧路径。");
    postEvent(Event::kRecoveryFail, "无可用恢复行为（P4）");
  }

  // ------------------------------------------------------------ 输出
  void publishState()
  {
    pnc_2d::msg::ManagerState m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.state = static_cast<uint8_t>(sm_.state());
    m.state_name = toString(sm_.state());
    m.message = sm_.lastTransition().reason;
    m.goal_x = goal_.x;
    m.goal_y = goal_.y;
    m.path_points = static_cast<uint32_t>(path_.poses.size());
    double len = 0.0;
    for (std::size_t i = 1; i < path_.poses.size(); ++i)
      len += std::hypot(path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x,
                        path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y);
    m.path_length_m = len;
    m.plan_requests = sm_.stats().plan_requests;
    m.plan_failures = sm_.stats().plan_failures;
    m.recoveries = sm_.stats().recoveries;
    pub_state_->publish(m);
  }

  // ------------------------------------------------------------ 参数读取
  std::string paramString(const std::string &key, const std::string &def)
  {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_string();
  }
  double paramDouble(const std::string &key, double def)
  {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_double();
  }
  int paramInt(const std::string &key, int def)
  {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return static_cast<int>(get_parameter(key).as_int());
  }
  bool paramBool(const std::string &key, bool def)
  {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_bool();
  }

  // ------------------------------------------------------------ 成员
  std::string frame_id_;
  std::string topic_goal_, topic_odom_, topic_state_;
  std::string srv_global_plan_, act_local_follow_, srv_global_clear_, srv_local_stop_;
  bool publish_plan_{true};

  double stuck_timeout_{10.0};
  double stuck_min_progress_{0.20};
  double odom_jump_threshold_{1.0};
  double odom_jump_cooldown_{2.0};
  double heartbeat_hz_{5.0};
  double speed_limit_{0.0};   // P5：从路网/限速区来

  // 走廊（P5.3）：从全局规划响应带到跟随目标，见 onPlanResponse / doStartFollow
  bool has_corridor_{false};
  double corridor_half_width_{0.0};
  double corridor_speed_limit_{0.0};
  bool strict_corridor_{false};
  std::vector<int> route_edges_;

  ManagerSm sm_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Publisher<pnc_2d::msg::ManagerState>::SharedPtr pub_state_;
  rclcpp::Client<pnc_2d::srv::PlanPath>::SharedPtr cli_plan_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_clear_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cli_stop_;
  rclcpp_action::Client<FollowPath>::SharedPtr cli_follow_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cancel_;
  rclcpp::TimerBase::SharedPtr timer_;

  Pose2D pose_, last_pose_, goal_;
  bool has_odom_{false};
  int plan_seq_{0};
  int follow_seq_{0};
  rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr follow_handle_;
  nav_msgs::msg::Path path_;
  FollowPath::Feedback::ConstSharedPtr last_feedback_;
  std::string last_feedback_msg_;

  Pose2D stuck_anchor_;
  rclcpp::Time stuck_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_jump_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace pnc_2d

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_2d::PncManagerNode>());
  rclcpp::shutdown();
  return 0;
}
