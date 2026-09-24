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
#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/recovery_behavior.hpp"
#include "pnc_2d/core/types.hpp"
#include "pnc_2d/msg/manager_state.hpp"
#include "pnc_2d/sm/manager_sm.hpp"
#include "pnc_2d/srv/plan_path.hpp"
#include "ros_param_reader.hpp"

namespace pnc_2d {
namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

} // namespace

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
    srv_global_plan_ =
        paramString("sm.global_plan_service", "/global_planner/plan_path");
    act_local_follow_ =
        paramString("sm.local_follow_action", "/local_planner/follow_path");
    srv_global_clear_ =
        paramString("sm.global_clear_service", "/global_planner/clear_path");
    srv_local_stop_ =
        paramString("sm.local_stop_service", "/local_planner/stop");

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

    // 恢复行为（P5.4）：`recovery.type` 从 P3 起就留着这个槽位，现在真正接上。
    // 默认 "replan"：本仓库里"被挡"绝大多数是**路径与当前局面不同源**
    // （地图/区域刚改、定位跳变、动态障碍占道）⇒ 正确处置是"按当前地图重规划"。
    recovery_type_ = paramString("sm.recovery.type", "replan");
    recovery_ = pnc_2d::createRecoveryBehavior(recovery_type_);
    if (recovery_) {
      // 行为自己加前缀（约定：前缀 = name() + "."，见 recovery_behavior.hpp）
      if (!recovery_->configure(pnc_2d::RosParamReader(*this))) {
        RCLCPP_WARN(get_logger(), "[sm] 恢复行为 %s 读参数失败 → 用默认值",
                    recovery_type_.c_str());
      }
      RCLCPP_INFO(get_logger(), "[sm] 恢复行为：%s", recovery_->name().c_str());
    } else {
      RCLCPP_WARN(
          get_logger(),
          "[sm] 恢复行为 '%s' 无实现（可用：%s）→ 被挡/卡住会直接判失败",
          recovery_type_.c_str(),
          pnc_2d::availableRecoveries().empty()
              ? "（无）"
              : pnc_2d::availableRecoveries()[0].c_str());
    }

    pub_state_ = create_publisher<pnc_2d::msg::ManagerState>(
        topic_state_, rclcpp::QoS(1).transient_local());

    auto odom_qos = rclcpp::SensorDataQoS(); // 定位是 best_effort，必须匹配
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, odom_qos,
        std::bind(&PncManagerNode::onOdom, this, std::placeholders::_1));
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_goal_, 1,
        std::bind(&PncManagerNode::onGoal, this, std::placeholders::_1));

    cli_plan_ = create_client<pnc_2d::srv::PlanPath>(srv_global_plan_);
    cli_clear_ = create_client<std_srvs::srv::Trigger>(srv_global_clear_);
    cli_stop_ = create_client<std_srvs::srv::Trigger>(srv_local_stop_);
    cli_follow_ =
        rclcpp_action::create_client<FollowPath>(this, act_local_follow_);

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
                "[sm] 上限：恢复 %d 次/任务 | 规划重试 %d 次 | 卡住判据 %.1f s "
                "内位移 < %.2f m",
                sp.max_recoveries, sp.max_plan_failures, stuck_timeout_,
                stuck_min_progress_);

    publishState();

    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / heartbeat_hz_),
                          std::bind(&PncManagerNode::onHeartbeat, this));
  }

private:
  // ============================================================ 事件入口
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != frame_id_) {
      RCLCPP_WARN(
          get_logger(),
          "[sm] 目标 frame='%s' != '%s'，拒用（请把 RViz Fixed Frame 设为 %s）",
          msg->header.frame_id.c_str(), frame_id_.c_str(), frame_id_.c_str());
      return;
    }
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_.yaw = yawFromQuaternion(msg->pose.orientation);
    goal_.has_yaw = true;
    RCLCPP_INFO(get_logger(), "[sm] 收到目标 (%.2f, %.2f, %.1f°)", goal_.x,
                goal_.y, goal_.yaw * 180.0 / M_PI);
    postEvent(Event::kGoalReceived, "收到 /goal_pose");
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
  void onHeartbeat() {
    if (sm_.state() == State::kFollowing) {
      const double moved =
          std::hypot(pose_.x - stuck_anchor_.x, pose_.y - stuck_anchor_.y);
      if (moved > stuck_min_progress_) {
        // 有推进：把参照点挪到当前位置，计时重新开始
        stuck_anchor_ = pose_;
        stuck_since_ = now();
      } else if (stuck_timeout_ > 0.0 &&
                 (now() - stuck_since_).seconds() > stuck_timeout_) {
        RCLCPP_WARN(get_logger(), "[sm] 卡住：%.1f s 内位移 %.2f m（< %.2f）",
                    (now() - stuck_since_).seconds(), moved,
                    stuck_min_progress_);
        postEvent(Event::kStuck, "长时间没有推进");
      }
    }
    publishState();
  }

  // ============================================================ 状态机桥接
  /// 喂事件并执行该事件要求的动作。返回事件是否被状态机接受。
  /// why 用 std::string（不是 const char*）：恢复行为返回的原因本身是
  /// std::string， 传 c_str() 会给临时对象取地址，容易埋生命周期坑。
  bool postEvent(Event e, const std::string &why) {
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
  void runEffect(SideEffect eff) {
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
  void doPlanPath() {
    ++plan_seq_; // 本次请求的序号：回调用它丢掉过期响应
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
    req->use_current_pose = true;        // 起点由全局节点用最新定位定
    req->publish_result = publish_plan_; // RViz 可视化（默认开）

    RCLCPP_INFO(get_logger(), "[sm] → 请求全局规划（目标 %.2f, %.2f）", goal_.x,
                goal_.y);
    cli_plan_->async_send_request(
        req,
        [this, seq](rclcpp::Client<pnc_2d::srv::PlanPath>::SharedFuture f) {
          onPlanResponse(seq, f);
        });
  }

  void onPlanResponse(int seq,
                      rclcpp::Client<pnc_2d::srv::PlanPath>::SharedFuture fut) {
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
    // 走廊（P5.3）：从全局规划结果透传给局部。
    //
    // ★★ 这里**必须逐点透传**，不能"按标量展开成等宽数组"（踩过）：
    //   hybrid 模式下路径 = [自由入口段, 路网段, 自由出口段]，把入口段也标成
    //   "走廊半宽"会让局部把入口段也当成"严格贴线的中心线"，于是车在车道外
    //   几厘米处就被硬约束判死（实测：横向偏差 ≥0.055 m ⇒ 求解器不收敛、
    //   120/120 周期被挡、车一步不动；现场现象就是"有角度的路网时机器基本
    //   不会动"）。
    has_corridor_ = res->has_corridor;
    corridor_half_width_ = res->corridor_half_width;
    corridor_speed_limit_ = res->corridor_speed_limit;
    zone_speed_limit_ = res->zone_speed_limit;
    strict_corridor_ = res->strict_corridor;
    route_edges_.assign(res->route_edges.begin(), res->route_edges.end());
    corridor_width_per_point_.assign(res->corridor_width.begin(),
                                     res->corridor_width.end());
    if (corridor_width_per_point_.size() != path_.poses.size()) {
      // 全局没给（或长度不一致，全局已告警）⇒ 退回"整条路径同一宽度"的旧行为
      corridor_width_per_point_.clear();
      if (has_corridor_)
        corridor_width_per_point_.assign(
            path_.poses.size(), strict_corridor_ ? 0.0 : corridor_half_width_);
    }
    // 参考速度剖面（M4）：全局规划器可能已经算好了剖面（MINCO 时间参数化）。
    // ★ 只透传，不在这里取 min / 截断：局部规划器还要叠加底盘钳位、末段
    //   approach/crawl 减速、降级限速等运行时约束，在这里动它只会让两边打架。
    traj_valid_ = res->traj_valid;
    traj_note_ = res->traj_note;
    traj_s_.assign(res->traj_s.begin(), res->traj_s.end());
    traj_v_.assign(res->traj_v.begin(), res->traj_v.end());
    traj_w_.assign(res->traj_w.begin(), res->traj_w.end());
    if (traj_valid_ &&
        !(traj_s_.size() == traj_v_.size() && traj_s_.size() == traj_w_.size() &&
          traj_s_.size() >= 2)) {
      // 上游字段自相矛盾 ⇒ 宁可当"没有剖面"（回退旧行为），也不要拿半张表去限速
      RCLCPP_WARN(get_logger(),
                  "[sm] 参考剖面长度不自洽（s=%zu v=%zu w=%zu）⇒ 丢弃",
                  traj_s_.size(), traj_v_.size(), traj_w_.size());
      traj_valid_ = false;
      traj_s_.clear();
      traj_v_.clear();
      traj_w_.clear();
    }
    if (traj_valid_)
      RCLCPP_INFO(get_logger(), "[sm] 参考剖面：%zu 点 / 弧长 %.2f m / 峰值 %.2f m/s",
                  traj_s_.size(), traj_s_.back(),
                  traj_v_.empty() ? 0.0
                                  : *std::max_element(traj_v_.begin(), traj_v_.end()));
    else if (!traj_note_.empty())
      RCLCPP_INFO(get_logger(), "[sm] 无参考剖面：%s", traj_note_.c_str());
    const std::size_t corr_pts = static_cast<std::size_t>(std::count_if(
        corridor_width_per_point_.begin(), corridor_width_per_point_.end(),
        [](double w) { return w > 0.0; }));
    RCLCPP_INFO(
        get_logger(),
        "[sm] 全局规划成功：%zu 点 / %.1f ms | 走廊 %s"
        "（半宽 %.2f m，限速 %.2f m/s，%s）| 通道 %zu 条 | 受走廊约束的点 "
        "%zu/%zu%s",
        path_.poses.size(), res->plan_time_ms, has_corridor_ ? "有" : "无",
        corridor_half_width_, corridor_speed_limit_,
        strict_corridor_ ? "严格贴线" : "允许绕障", route_edges_.size(),
        corr_pts, path_.poses.size(),
        (corr_pts > 0 && corr_pts < path_.poses.size())
            ? "（入口/出口段自由，只有车道段严格贴线）"
            : "");
    postEvent(Event::kPlanOk, "路径就绪");
    last_plan_time_ = now(); // 恢复行为的防抖用（见 doRecovery）
  }

  void doStartFollow() {
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
    // 走廊（P5.3）：来自全局规划结果，**逐点**透传（见 doPlan 里的说明）。
    //   · 数组为空（A* 自由空间）→ 局部按自由空间跟踪；
    //   · 逐点值 >0 → 该点受"允许横向偏离 ±W"的硬约束；<=0 → 该点无走廊约束
    //     （hybrid 的自由入口/出口段）。
    // ★ 没接通之前这里一直 clear()，导致 route 模式的走廊约束**从未生效**过：
    //   路径确实是沿通道算的，但局部只会"尽量跟"，可以自由绕障跑到通道外面。
    goal.corridor_width = corridor_width_per_point_;
    // 参考速度剖面（M4）：与走廊同一条"全局 → 局部"通道，同样逐点透传。
    // traj_valid=false 时局部会完全回退旧行为（自己按曲率/制动限速）。
    goal.traj_valid = traj_valid_;
    goal.traj_note = traj_note_;
    goal.traj_s = traj_s_;
    goal.traj_v = traj_v_;
    goal.traj_w = traj_w_;
    goal.speed_limit = speed_limit_;
    if (has_corridor_ && corridor_speed_limit_ > 0.0) {
      // 通道限速与全局限速（限速区）取更保守的那个：谁小听谁的
      goal.speed_limit = (speed_limit_ > 0.0)
                             ? std::min(speed_limit_, corridor_speed_limit_)
                             : corridor_speed_limit_;
    }
    // ★
    // 限速区（区域层）也要合成进来：只让**局部**按区限速的话，任务限速还是旧值，
    //   出/入区时两者会打架（局部降到 0.2、任务级还写着
    //   0.35，日志上像"限速没生效"）。
    //   三者取最小：任务/全局限速、通道限速、区域限速。
    for (const double l : {corridor_speed_limit_, zone_speed_limit_}) {
      if (l > 0.0)
        goal.speed_limit =
            (goal.speed_limit > 0.0) ? std::min(goal.speed_limit, l) : l;
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

    RCLCPP_INFO(get_logger(), "[sm] → 开始跟随（%zu 点）",
                goal.path.poses.size());
    cli_follow_->async_send_goal(goal, opt);
  }

  void onFollowGoalResponse(
      int seq,
      const rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr &h) {
    if (seq != follow_seq_)
      return;
    if (!h) {
      RCLCPP_ERROR(get_logger(), "[sm] 局部节点拒绝了跟随目标");
      postEvent(Event::kFollowFail, "局部拒绝目标");
      return;
    }
    follow_handle_ = h;
  }

  void onFollowFeedback(int seq,
                        const std::shared_ptr<const FollowPath::Feedback> fb) {
    if (seq != follow_seq_ || !fb)
      return;
    last_feedback_ = fb;
    // 只在"变了"的时候打日志，20 Hz 的反馈不能每个周期都刷
    if (!fb->message.empty() && fb->message != last_feedback_msg_) {
      last_feedback_msg_ = fb->message;
      RCLCPP_INFO(get_logger(),
                  "[sm] ← 局部反馈：%s（%s，进度 %.0f%%，剩余 %.2f m）",
                  fb->message.c_str(), fb->status_name.c_str(),
                  fb->progress * 100.0, fb->remaining_m);
    }
  }

  void onFollowResult(
      int seq,
      const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult &w) {
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
      return; // 取消通常由状态机自己发起（Cancel 事件已处理过状态）
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
  void doStopRobot() {
    if (follow_handle_) {
      RCLCPP_INFO(get_logger(), "[sm] 停止跟随（取消 action）");
      cli_follow_->async_cancel_goal(follow_handle_);
      follow_handle_.reset();
    }
    ++follow_seq_; // 让旧的回调失效
  }

  /// 恢复行为（被挡/卡住时执行）。
  ///
  /// ★ 2026-09-23 之前这里是空的（"P4 没有实现"），后果比想象严重：**进入
  /// RECOVERING
  ///   之后永远出不来** ——
  ///   没人触发任何动作、也没有事件把它推出去，任务既不会完成 也不会失败，而且
  ///   RECOVERING **不接受新目标**（实测：仿真里被挡一次之后，
  ///   后续目标全被静默拒绕，只能重启节点）。
  ///   现在：真正执行恢复行为；成功后状态机走 `Recovering → Planning →
  ///   Following` （先按当前地图重规划，而不是盲目重跟同一条旧路径 ——
  ///   后者会立刻再被挡， 把恢复额度白白烧完）。
  void doRecovery() {
    // 先兜底清掉局部状态与全局旧路径：不管恢复行为是什么，"上次的路径已经
    // 不可信"这件事必须让下游知道。
    if (cli_stop_->service_is_ready())
      cli_stop_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
    if (cli_clear_->service_is_ready())
      cli_clear_->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());

    if (!recovery_) {
      RCLCPP_WARN(get_logger(),
                  "[sm] 恢复：没有可用的恢复行为（recovery.type=%s）→ 算失败",
                  recovery_type_.c_str());
      postEvent(Event::kRecoveryFail, "无可用恢复行为");
      return;
    }

    pnc_2d::RecoveryContext ctx;
    // 本次恢复的意图 = 重规划（真正的重规划由状态机在 kRecoveryDone → Planning
    // 这一步发起，见 manager_sm 的转移表）。这个回调保留下来是因为：
    //   ① 将来的多步恢复行为（先后退再重规划）需要同一套"请求重规划"入口；
    //   ② 行为本身要能如实报"我确实请求了"，否则单测无法断言它做了什么。
    ctx.requestReplan = [this]() { replan_requested_ = true; };
    ctx.timeSinceLastPlan = [this]() {
      return (now() - last_plan_time_).seconds();
    };
    const pnc_2d::RecoveryResult r = recovery_->run(ctx);
    RCLCPP_WARN(get_logger(), "[sm] 恢复行为 %s：%s", recovery_->name().c_str(),
                r.message.c_str());
    if (!r.success) {
      postEvent(Event::kRecoveryFail, r.message);
      return;
    }
    postEvent(Event::kRecoveryDone, r.message);
  }

  // ------------------------------------------------------------ 输出
  void publishState() {
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
      len += std::hypot(
          path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x,
          path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y);
    m.path_length_m = len;
    m.plan_requests = sm_.stats().plan_requests;
    m.plan_failures = sm_.stats().plan_failures;
    m.recoveries = sm_.stats().recoveries;
    pub_state_->publish(m);
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
  int paramInt(const std::string &key, int def) {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return static_cast<int>(get_parameter(key).as_int());
  }
  bool paramBool(const std::string &key, bool def) {
    if (!has_parameter(key))
      declare_parameter(key, def);
    return get_parameter(key).as_bool();
  }

  // ------------------------------------------------------------ 成员
  std::string frame_id_;
  std::string topic_goal_, topic_odom_, topic_state_;
  std::string srv_global_plan_, act_local_follow_, srv_global_clear_,
      srv_local_stop_;
  bool publish_plan_{true};

  double stuck_timeout_{10.0};
  double stuck_min_progress_{0.20};
  double odom_jump_threshold_{1.0};
  double odom_jump_cooldown_{2.0};
  double heartbeat_hz_{5.0};
  double speed_limit_{0.0}; // P5：从路网/限速区来

  // 恢复行为（P5.4）：被挡/卡住时执行；见 doRecovery
  std::string recovery_type_{"replan"};
  std::unique_ptr<pnc_2d::RecoveryBehavior> recovery_;
  bool replan_requested_{false};
  /// 上一次**成功规划**的时刻（恢复行为的防抖用：刚规划完就再规划没有意义）
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  // 走廊（P5.3）：从全局规划响应带到跟随目标，见 onPlanResponse / doStartFollow
  bool has_corridor_{false};
  double corridor_half_width_{0.0};
  double corridor_speed_limit_{0.0};
  /// 限速区沿路径的最严限速（来自全局规划响应的区域层计算结果）
  double zone_speed_limit_{0.0};
  bool strict_corridor_{false};
  std::vector<int> route_edges_;
  /// ★ 逐点走廊半宽（长度 = 路径点数；<=0 = 该点无走廊约束）。
  /// 为什么逐点：hybrid 的路径含自由入口/出口段，那些段没有走廊；标量会把它们
  /// 也当成"严格贴线的中心线"，车在车道外几厘米就被判死（见 doPlan 的说明）。
  std::vector<double> corridor_width_per_point_;

  /// ★ 参考速度剖面（M4）：全局规划器给的"弧长→速度/转向速率"表。
  /// 这里只做**存储与透传**，不做任何解释/限幅（解释权在局部规划器，
  /// 因为只有它知道底盘钳位、末段减速、降级限速等运行时约束）。
  bool traj_valid_{false};
  std::string traj_note_;
  std::vector<double> traj_s_, traj_v_, traj_w_;

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

} // namespace pnc_2d

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pnc_2d::PncManagerNode>());
  rclcpp::shutdown();
  return 0;
}
