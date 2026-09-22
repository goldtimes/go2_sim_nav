#include "pnc_2d/sm/manager_sm.hpp"

namespace pnc_2d {
namespace {

/// 转移表的一行
struct Rule {
  State from;
  Event event;
  State to;
  SideEffect effect;
  const char *note;
};

/// ★ 全部转移都在这一张表里（doc/pnc2d_restructure_plan.md §6 P4）
const Rule kRules[] = {
    // ---------------------------------------------------------- Idle
    {State::kIdle, Event::kGoalReceived, State::kPlanning, SideEffect::kPlanPath,
     "收到目标 → 请求全局规划"},
    {State::kIdle, Event::kCancel, State::kIdle, SideEffect::kNone,
     "空闲状态下取消：本来就是空闲"},

    // ---------------------------------------------------------- Planning
    {State::kPlanning, Event::kGoalReceived, State::kPlanning,
     SideEffect::kPlanPath, "规划期间来了新目标 → 直接按新目标重规划"},
    {State::kPlanning, Event::kPlanOk, State::kFollowing, SideEffect::kStartFollow,
     "规划成功 → 开始跟随"},
    {State::kPlanning, Event::kPlanFail, State::kFailed, SideEffect::kStopRobot,
     "规划失败 → 任务失败（目标不可达之类的确定性结论，重试没意义）"},
    {State::kPlanning, Event::kCancel, State::kIdle, SideEffect::kStopRobot,
     "规划期间被取消 → 回空闲"},

    // ---------------------------------------------------------- Following
    {State::kFollowing, Event::kReached, State::kGoalReached,
     SideEffect::kStopRobot, "到达目标"},
    {State::kFollowing, Event::kBlocked, State::kRecovering, SideEffect::kStopRobot,
     "前方被挡且不可绕 → 停车 + 恢复"},
    {State::kFollowing, Event::kStuck, State::kRecovering, SideEffect::kStopRobot,
     "长时间没有前进 → 停车 + 恢复"},
    {State::kFollowing, Event::kFollowFail, State::kFailed, SideEffect::kStopRobot,
     "局部规划失败 → 任务失败"},
    {State::kFollowing, Event::kOdomJump, State::kPlanning, SideEffect::kPlanPath,
     "定位跳变：路径是旧位姿下算的 → 重规划"},
    {State::kFollowing, Event::kGoalReceived, State::kPlanning,
     SideEffect::kPlanPath, "跟随期间来了新目标 → 放弃旧任务，按新目标重规划"},
    {State::kFollowing, Event::kCancel, State::kIdle, SideEffect::kStopRobot,
     "跟随期间被取消 → 停车回空闲"},

    // ---------------------------------------------------------- Recovering
    {State::kRecovering, Event::kRecoveryDone, State::kFollowing,
     SideEffect::kStartFollow, "恢复成功 → 继续跟随（路径不变）"},
    {State::kRecovering, Event::kRecoveryFail, State::kFailed,
     SideEffect::kStopRobot, "恢复超限 → 任务失败（是否 Retry 由上层决定）"},
    {State::kRecovering, Event::kBlocked, State::kRecovering,
     SideEffect::kStopRobot, "恢复期间又被挡：不叠加计数，等本轮恢复结果"},
    {State::kRecovering, Event::kStuck, State::kRecovering, SideEffect::kStopRobot,
     "恢复期间再次卡住：同上"},
    {State::kRecovering, Event::kReached, State::kGoalReached,
     SideEffect::kStopRobot, "恢复期间（重新）到达目标"},
    {State::kRecovering, Event::kCancel, State::kIdle, SideEffect::kStopRobot,
     "恢复期间被取消 → 停车回空闲"},

    // ---------------------------------------------------------- GoalReached
    {State::kGoalReached, Event::kGoalReceived, State::kPlanning,
     SideEffect::kPlanPath, "到达后收到新目标 → 新任务"},
    {State::kGoalReached, Event::kCancel, State::kIdle, SideEffect::kNone,
     "到达后取消 → 回空闲（车本来就停着）"},

    // ---------------------------------------------------------- Failed
    {State::kFailed, Event::kGoalReceived, State::kPlanning, SideEffect::kPlanPath,
     "失败后收到新目标 → 新任务（复位计数）"},
    {State::kFailed, Event::kCancel, State::kIdle, SideEffect::kNone,
     "失败后取消 → 回空闲"},
};

}  // namespace

const char * toString(State s)
{
  switch (s) {
    case State::kIdle: return "IDLE";
    case State::kPlanning: return "PLANNING";
    case State::kFollowing: return "FOLLOWING";
    case State::kGoalReached: return "GOAL_REACHED";
    case State::kRecovering: return "RECOVERING";
    case State::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

const char * toString(Event e)
{
  switch (e) {
    case Event::kGoalReceived: return "GoalReceived";
    case Event::kPlanOk: return "PlanOk";
    case Event::kPlanFail: return "PlanFail";
    case Event::kReached: return "Reached";
    case Event::kBlocked: return "Blocked";
    case Event::kStuck: return "Stuck";
    case Event::kFollowFail: return "FollowFail";
    case Event::kOdomJump: return "OdomJump";
    case Event::kRecoveryDone: return "RecoveryDone";
    case Event::kRecoveryFail: return "RecoveryFail";
    case Event::kCancel: return "Cancel";
  }
  return "UNKNOWN";
}

const char * toString(SideEffect e)
{
  switch (e) {
    case SideEffect::kNone: return "None";
    case SideEffect::kPlanPath: return "PlanPath";
    case SideEffect::kStartFollow: return "StartFollow";
    case SideEffect::kStopRobot: return "StopRobot";
    case SideEffect::kRunRecovery: return "RunRecovery";
  }
  return "UNKNOWN";
}

ManagerSm::ManagerSm(const SmParams &params) : params_(params) {}

bool ManagerSm::lookup(State from, Event e, State &to, SideEffect &effect,
                       const char **note)
{
  for (const Rule &r : kRules) {
    if (r.from == from && r.event == e) {
      to = r.to;
      effect = r.effect;
      if (note)
        *note = r.note;
      return true;
    }
  }
  return false;
}

Transition ManagerSm::handle(Event e)
{
  Transition tr;
  tr.from = state_;
  tr.to = state_;

  State to = state_;
  SideEffect effect = SideEffect::kNone;
  const char *note = nullptr;
  if (!lookup(state_, e, to, effect, &note)) {
    tr.accepted = false;
    tr.effect = SideEffect::kNone;
    tr.reason = std::string("状态 ") + toString(state_) + " 不接受事件 " +
                toString(e);
    last_ = tr;
    return tr;
  }
  std::string reason = note ? note : "";      // 表里的默认文案

  // ------- 计数器与"超限"分支（表里放不下的那部分规则）-------
  //
  // 计数口径（只在这里改，避免重复计数）：
  //   recoveries_used_   = 本次任务**已经尝试过**的恢复次数
  //   stats_.recoveries  = 同上（对外统计，随 recoveries_used_ 一起加）
  // 恢复次数**成功后不清零**：这正是"防无限循环"的地方（挡住→恢复→再挡…）。
  switch (state_) {
    case State::kIdle:
    case State::kGoalReached:
    case State::kFailed:
      // 新任务开始：计数从零起（恢复上限是**每任务**的）
      if (e == Event::kGoalReceived) {
        recoveries_used_ = 0;
        plan_failures_run_ = 0;
      }
      break;

    case State::kPlanning:
      if (e == Event::kPlanOk) {
        plan_failures_run_ = 0;
      } else if (e == Event::kPlanFail) {
        ++plan_failures_run_;
        ++stats_.plan_failures;
        if (plan_failures_run_ <= params_.max_plan_failures) {
          to = State::kPlanning;      // 留在 Planning，重发请求
          effect = SideEffect::kPlanPath;
          reason = "规划失败但未超重试上限（第 " +
                   std::to_string(plan_failures_run_) + "/" +
                   std::to_string(params_.max_plan_failures) + " 次）→ 重试";
        }
      }
      break;

    case State::kFollowing:
      if (e == Event::kBlocked || e == Event::kStuck) {
        if (recoveries_used_ >= params_.max_recoveries) {
          // 额度用完：不再尝试恢复，直接判失败（max_recoveries=0 时必然走这里）
          to = State::kFailed;
          effect = SideEffect::kStopRobot;
          reason = "恢复次数已用尽（" + std::to_string(params_.max_recoveries) +
                   " 次）→ 失败";
        } else {
          ++recoveries_used_;         // 即将执行第 recoveries_used_ 次恢复
          ++stats_.recoveries;
          reason = std::string(e == Event::kBlocked ? "被挡" : "卡住") +
                   " → 进恢复（第 " + std::to_string(recoveries_used_) + "/" +
                   std::to_string(params_.max_recoveries) + " 次）";
        }
      }
      break;

    case State::kRecovering:
      if (e == Event::kRecoveryDone) {
        reason = "恢复成功 → 继续跟随";
      } else if (e == Event::kRecoveryFail) {
        if (recoveries_used_ < params_.max_recoveries) {
          ++recoveries_used_;         // 再试一次
          ++stats_.recoveries;
          to = State::kRecovering;
          effect = SideEffect::kRunRecovery;
          reason = "恢复失败但未超上限 → 再试一次（第 " +
                   std::to_string(recoveries_used_) + "/" +
                   std::to_string(params_.max_recoveries) + " 次）";
        } else {
          reason = "恢复失败且次数已用尽（" +
                   std::to_string(params_.max_recoveries) + " 次）→ 失败";
        }
      }
      break;
  }

  if (to == State::kPlanning && effect == SideEffect::kPlanPath)
    ++stats_.plan_requests;

  tr.accepted = true;
  tr.to = to;
  tr.effect = effect;
  tr.reason = reason;
  state_ = to;
  last_ = tr;
  return tr;
}

void ManagerSm::reset()
{
  state_ = State::kIdle;
  stats_ = SmStats{};
  recoveries_used_ = 0;
  plan_failures_run_ = 0;
  last_ = Transition{};
}

}  // namespace pnc_2d
