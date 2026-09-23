// 任务状态机（纯库，ROS-free）。
//
// 用法（节点的视角）：
//   ManagerSm sm(params);
//   auto tr = sm.handle(Event::kGoalReceived);   // 问："该干什么？"
//   if (tr.accepted) { 按 tr.effect 去调 ROS; }   //
//   做完把结果变成下一个事件喂回来
//
// 关键设计：**handle() 不阻塞、不做副作用**。它只回答两件事：
//   1. 这个事件在当前状态下收不收（不收就保持原状态，并给出原因）；
//   2. 接受的话，接下来该执行哪个动作（SideEffect）与进入哪个状态。
//   于是"转移表"能被单测穷举，而 ROS 调用只留在薄壳节点里。
//
// 状态转移表（完整版，单测逐条断言）：
//
//   from        event          to            effect
//   ───────────────────────────────────────────────────────────────
//   Idle        GoalReceived   Planning      PlanPath
//   Idle        Cancel         Idle          None（本来就空闲）
//   Planning    PlanOk         Following     StartFollow
//   Planning    PlanFail       Failed        StopRobot
//   Planning    Cancel         Idle          StopRobot
//   Planning    GoalReceived   Planning      PlanPath（顶掉旧目标）
//   Following   Reached        GoalReached   StopRobot
//   Following   Blocked        Recovering    RunRecovery    ← 计数+1
//   Following   Stuck          Recovering    RunRecovery    ← 计数+1
//   Following   FollowFail     Failed        StopRobot
//   Following   OdomJump       Planning      PlanPath（旧路径基于旧位姿）
//   Following   GoalReceived   Planning      PlanPath（新目标打断旧任务）
//   Following   Cancel         Idle          StopRobot
//   Recovering  RecoveryDone   Planning      PlanPath       ← ★
//   恢复后先重规划再跟随 Recovering  RecoveryFail   超限?Failed:Recovering
//   StopRobot / RunRecovery Recovering  Blocked/Stuck  Recovering    StopRobot
//   ← 恢复期间又被挡：不叠加计数 Recovering  Cancel         Idle StopRobot
//   GoalReached GoalReceived   Planning      PlanPath
//   GoalReached Cancel         Idle          StopRobot
//   Failed      GoalReceived   Planning      PlanPath
//   Failed      Cancel         Idle          StopRobot
//
// 其余组合一律**拒绝**（保持原状态），并把原因写进 Transition::reason ——
// "静默忽略一个事件"是状态机最难查的问题，所以宁可显式拒绝。

#pragma once

#include <cstdint>
#include <string>

#include "pnc_2d/sm/events.hpp"
#include "pnc_2d/sm/states.hpp"

namespace pnc_2d {

/// 进入新状态时该执行的动作（节点的薄壳负责翻译成 ROS 调用）
enum class SideEffect : uint8_t {
  kNone = 0,    ///< 什么都不用做（只是记录状态变化）
  kPlanPath,    ///< 调 global 的 PlanPath 服务；完成 → kPlanOk / kPlanFail
  kStartFollow, ///< 把路径交给 local 的 FollowPath action；结果 →
                ///< kReached/kBlocked/…
  kStopRobot,   ///< 停车：取消跟随 action + 发零速（P4 的 Null
                ///< 不发速度，但语义要保留）
  kRunRecovery, ///< 执行一次恢复行为；完成 → kRecoveryDone / kRecoveryFail
};

const char *toString(SideEffect e);

/// 可调参数（从 `sm.*` 读）
struct SmParams {
  /// 一次任务内允许的恢复尝试总次数。**恢复成功也不清零** ——
  /// 这正是"防止无限循环"的地方：挡住了→恢复→再挡→再恢复… 必须有个头。
  int max_recoveries{2};
  /// 允许的全局规划**重试**次数（0 = 不重试，一次失败即失败）。
  /// 默认 0：全局规划失败通常是"目标不可达"这类确定性结论，重试没意义。
  int max_plan_failures{0};
};

/// 一次转移的答案
struct Transition {
  bool accepted{false}; ///< 事件是否被接受（false = 保持原状态）
  State from{State::kIdle};
  State to{State::kIdle};
  SideEffect effect{SideEffect::kNone};
  std::string reason; ///< 接受=转移说明；拒绝=为什么拒绝

  bool changed() const { return accepted && from != to; }
};

/// 运行统计（进 /pnc_2d/state，用来回答"到底重规划了几次"）
struct SmStats {
  int plan_requests{0}; ///< 已发出的全局规划请求次数（含失败重试）
  int plan_failures{0}; ///< 其中失败次数
  int recoveries{0};    ///< 已触发的恢复行为次数
};

class ManagerSm {
public:
  explicit ManagerSm(const SmParams &params = SmParams{});

  /// 喂一个事件，拿到"该干什么"。**无副作用**，可重复调用（幂等查询：同样的
  /// 状态 + 同样的事件 → 同样的答案）。
  Transition handle(Event e);

  State state() const { return state_; }
  const SmParams &params() const { return params_; }
  const SmStats &stats() const { return stats_; }
  /// 本次任务已用掉的恢复次数（新目标会清零）
  int recoveriesUsed() const { return recoveries_used_; }
  /// 连续规划失败次数
  int consecutivePlanFailures() const { return plan_failures_run_; }
  /// 最近一次转移（日志用）
  const Transition &lastTransition() const { return last_; }

  /// 回到初始空闲状态并清零统计（节点重启用；**不**清 params）
  void reset();

private:
  /// 查表：返回该 (from, event) 的规则；找不到返回 false
  static bool lookup(State from, Event e, State &to, SideEffect &effect,
                     const char **note);

  SmParams params_;
  State state_{State::kIdle};
  SmStats stats_;
  int recoveries_used_{0};
  int plan_failures_run_{0};
  Transition last_;
};

} // namespace pnc_2d
