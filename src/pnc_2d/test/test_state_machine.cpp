// 状态机单测（纯库，不需要 ROS 环境）。
//
// 为什么值得单独测：状态机是"任务为什么会卡住/乱跑"的第一现场，而它的行为
// 全在一张转移表里 —— 表是数据，数据就该被穷举验证：
//   1. 表里每条规则的行为（含 reason 与 effect）；
//   2. **全部 (状态 × 事件) 组合**都被显式处理：要么有条规则，要么明确拒绝
//      （静默忽略事件是状态机最难查的问题）；
//   3. 计数与上限（重试、恢复额度、每任务清零）；
//   4. 典型任务序列端到端走一遍（成功 / 失败 / 取消 / 打断）。

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "pnc_2d/sm/manager_sm.hpp"

namespace pnc_2d {
namespace {

const State kAllStates[] = {State::kIdle,       State::kPlanning,
                            State::kFollowing,  State::kGoalReached,
                            State::kRecovering, State::kFailed};

const Event kAllEvents[] = {
    Event::kGoalReceived, Event::kPlanOk,   Event::kPlanFail,
    Event::kReached,      Event::kBlocked,  Event::kStuck,
    Event::kFollowFail,   Event::kOdomJump, Event::kRecoveryDone,
    Event::kRecoveryFail, Event::kCancel};

/// 把状态机推到指定状态（用一条最短的合法事件序列）
void driveTo(ManagerSm &sm, State want) {
  // 只有这几条路径会被用到；assert 在下面兜底
  switch (want) {
  case State::kIdle:
    break;
  case State::kPlanning:
    sm.handle(Event::kGoalReceived);
    break;
  case State::kFollowing:
    sm.handle(Event::kGoalReceived);
    sm.handle(Event::kPlanOk);
    break;
  case State::kGoalReached:
    sm.handle(Event::kGoalReceived);
    sm.handle(Event::kPlanOk);
    sm.handle(Event::kReached);
    break;
  case State::kRecovering:
    sm.handle(Event::kGoalReceived);
    sm.handle(Event::kPlanOk);
    sm.handle(Event::kBlocked);
    break;
  case State::kFailed:
    sm.handle(Event::kGoalReceived);
    sm.handle(Event::kPlanFail);
    break;
  }
}

// ------------------------------------------------------------ 转移表逐条

TEST(ManagerSm, TableIdleAndPlanning) {
  ManagerSm sm;
  EXPECT_EQ(sm.state(), State::kIdle);

  auto a = sm.handle(Event::kPlanOk);
  EXPECT_FALSE(a.accepted) << "空闲状态不该接受 PlanOk";
  EXPECT_EQ(sm.state(), State::kIdle) << "拒绝事件必须保持原状态";
  EXPECT_NE(a.reason.find("不接受"), std::string::npos) << "拒绝要说清原因";

  a = sm.handle(Event::kGoalReceived);
  EXPECT_TRUE(a.accepted);
  EXPECT_EQ(a.from, State::kIdle);
  EXPECT_EQ(a.to, State::kPlanning);
  EXPECT_EQ(a.effect, SideEffect::kPlanPath);
  EXPECT_TRUE(a.changed());
  EXPECT_EQ(sm.state(), State::kPlanning);

  a = sm.handle(Event::kPlanOk);
  EXPECT_EQ(a.to, State::kFollowing);
  EXPECT_EQ(a.effect, SideEffect::kStartFollow);

  // 一个任务只能走一条主线：到达后再喂 PlanOk 属于乱序，必须被拒
  sm.handle(Event::kReached);
  EXPECT_EQ(sm.state(), State::kGoalReached);
  EXPECT_FALSE(sm.handle(Event::kPlanOk).accepted);
}

TEST(ManagerSm, PlanFailGoesToFailedByDefault) {
  ManagerSm sm; // 默认 max_plan_failures = 0
  sm.handle(Event::kGoalReceived);
  auto a = sm.handle(Event::kPlanFail);
  EXPECT_EQ(a.to, State::kFailed);
  EXPECT_EQ(a.effect, SideEffect::kStopRobot);
  EXPECT_EQ(sm.state(), State::kFailed);
}

TEST(ManagerSm, PlanFailRetriesWhenAllowed) {
  SmParams p;
  p.max_plan_failures = 2; // 允许重试 2 次
  ManagerSm sm(p);
  sm.handle(Event::kGoalReceived); // PlanPath #1

  auto a = sm.handle(Event::kPlanFail); // 第 1 次失败：重试
  EXPECT_EQ(a.to, State::kPlanning);
  EXPECT_EQ(a.effect, SideEffect::kPlanPath);
  a = sm.handle(Event::kPlanFail); // 第 2 次失败：重试
  EXPECT_EQ(a.to, State::kPlanning);
  a = sm.handle(Event::kPlanFail); // 第 3 次失败：超限
  EXPECT_EQ(a.to, State::kFailed);
  EXPECT_EQ(sm.stats().plan_failures, 3);
  EXPECT_EQ(sm.stats().plan_requests, 3) << "初次 + 两次重试都算请求";
}

TEST(ManagerSm, PlanOkResetsFailureCounter) {
  SmParams p;
  p.max_plan_failures = 1;
  ManagerSm sm(p);
  sm.handle(Event::kGoalReceived);
  sm.handle(Event::kPlanFail); // run=1
  sm.handle(Event::kPlanFail); // run=2 > 1 → 已 Failed
  EXPECT_EQ(sm.state(), State::kFailed);

  // 新任务复位
  sm.handle(Event::kGoalReceived);
  EXPECT_EQ(sm.consecutivePlanFailures(), 0);
  sm.handle(Event::kPlanFail); // run=1 ≤ 1 → 重试
  EXPECT_EQ(sm.state(), State::kPlanning);
}

TEST(ManagerSm, FollowingTransitions) {
  { // 到达
    ManagerSm sm;
    driveTo(sm, State::kFollowing);
    auto a = sm.handle(Event::kReached);
    EXPECT_EQ(a.to, State::kGoalReached);
    EXPECT_EQ(a.effect, SideEffect::kStopRobot);
  }
  { // 局部失败
    ManagerSm sm;
    driveTo(sm, State::kFollowing);
    EXPECT_EQ(sm.handle(Event::kFollowFail).to, State::kFailed);
  }
  { // 定位跳变 → 重规划（路径是旧位姿下算的）
    ManagerSm sm;
    driveTo(sm, State::kFollowing);
    auto a = sm.handle(Event::kOdomJump);
    EXPECT_EQ(a.to, State::kPlanning);
    EXPECT_EQ(a.effect, SideEffect::kPlanPath);
  }
  { // 新目标打断旧任务
    ManagerSm sm;
    driveTo(sm, State::kFollowing);
    auto a = sm.handle(Event::kGoalReceived);
    EXPECT_EQ(a.to, State::kPlanning);
    EXPECT_EQ(a.effect, SideEffect::kPlanPath);
    EXPECT_EQ(sm.recoveriesUsed(), 0) << "新目标要复位恢复计数";
  }
  { // 取消
    ManagerSm sm;
    driveTo(sm, State::kFollowing);
    auto a = sm.handle(Event::kCancel);
    EXPECT_EQ(a.to, State::kIdle);
    EXPECT_EQ(a.effect, SideEffect::kStopRobot);
  }
}

TEST(ManagerSm, BlockedEntersRecoveryAndCounts) {
  SmParams p;
  p.max_recoveries = 2;
  ManagerSm sm(p);
  driveTo(sm, State::kFollowing);

  auto a = sm.handle(Event::kBlocked);
  EXPECT_EQ(a.to, State::kRecovering);
  // ★ 2026-09-23：这里必须是 kRunRecovery（之前是 kStopRobot）。
  // 用 kStopRobot 的后果是**恢复行为永远不会被调用** → 既没有 RecoveryDone
  // 也没有 RecoveryFail → 状态机永久停在 RECOVERING，且该状态不接受新目标。
  EXPECT_EQ(a.effect, SideEffect::kRunRecovery)
      << "被挡后必须真的去跑恢复行为，否则状态机永远停在 RECOVERING";
  EXPECT_EQ(sm.recoveriesUsed(), 1);
  EXPECT_NE(a.reason.find("第 1/2 次"), std::string::npos) << a.reason;

  // 恢复失败 → 还有额度 → 再试一次
  a = sm.handle(Event::kRecoveryFail);
  EXPECT_EQ(a.to, State::kRecovering);
  EXPECT_EQ(a.effect, SideEffect::kRunRecovery);
  EXPECT_EQ(sm.recoveriesUsed(), 2);

  // 再失败 → 额度用尽
  a = sm.handle(Event::kRecoveryFail);
  EXPECT_EQ(a.to, State::kFailed);
  EXPECT_NE(a.reason.find("用尽"), std::string::npos) << a.reason;
  EXPECT_EQ(sm.stats().recoveries, 2) << "统计与已用次数一致，不能重复计数";
}

TEST(ManagerSm, RecoveryDoneReplansBeforeFollowing) {
  ManagerSm sm;
  driveTo(sm, State::kFollowing);
  sm.handle(Event::kBlocked);
  auto a = sm.handle(Event::kRecoveryDone);
  // ★ 2026-09-23：恢复成功**不能**直接回 Following。
  // 旧路径（RecoveryDone → Following/kStartFollow）会去跟同一条已经证明走不通的
  // 路径：局部会立刻再报 Blocked，把恢复额度白白烧完，而且掩盖了根因
  // （实测就是这个行为让仿真里的区域用例永远走不到终点）。
  EXPECT_EQ(a.to, State::kPlanning) << "恢复成功后先重规划";
  EXPECT_EQ(a.effect, SideEffect::kPlanPath);
  // 重规划成功后才回到跟随
  const auto b = sm.handle(Event::kPlanOk);
  EXPECT_EQ(b.to, State::kFollowing);
  EXPECT_EQ(b.effect, SideEffect::kStartFollow);
}

TEST(ManagerSm, ZeroRecoveryLimitFailsImmediately) {
  SmParams p;
  p.max_recoveries = 0; // 不许恢复
  ManagerSm sm(p);
  driveTo(sm, State::kFollowing);
  auto a = sm.handle(Event::kBlocked);
  EXPECT_EQ(a.to, State::kFailed);
  EXPECT_EQ(sm.stats().recoveries, 0) << "没尝试过就不能计一次";
}

TEST(ManagerSm, RecoveryUsedDoesNotResetOnSuccess) {
  SmParams p;
  p.max_recoveries = 1;
  ManagerSm sm(p);
  driveTo(sm, State::kFollowing);

  sm.handle(Event::kBlocked);      // 用掉唯一额度
  sm.handle(Event::kRecoveryDone); // 恢复成功 → 先重规划
  sm.handle(Event::kPlanOk);       // 重规划成功 → 回到跟随
  EXPECT_EQ(sm.state(), State::kFollowing);
  EXPECT_EQ(sm.recoveriesUsed(), 1);

  // 再被挡：额度已用完 → 直接失败。
  // 这就是"挡住→恢复→再挡→再恢复…"无限循环的防线，**故意**不在恢复成功时清零。
  EXPECT_EQ(sm.handle(Event::kBlocked).to, State::kFailed);
}

TEST(ManagerSm, StuckEntersRecoveryAndRunsBehavior) {
  SmParams p;
  p.max_recoveries = 1;
  ManagerSm sm(p);
  driveTo(sm, State::kFollowing);
  auto a = sm.handle(Event::kStuck);
  EXPECT_EQ(a.to, State::kRecovering);
  EXPECT_EQ(a.effect, SideEffect::kRunRecovery) << "卡住与被挡同路";
  EXPECT_NE(a.reason.find("卡住"), std::string::npos) << a.reason;
  EXPECT_EQ(sm.recoveriesUsed(), 1);
}

TEST(ManagerSm, RecoveryStateIsStableAgainstRepeatedBlocked) {
  SmParams p;
  p.max_recoveries = 3;
  ManagerSm sm(p);
  driveTo(sm, State::kFollowing);
  sm.handle(Event::kBlocked); // used = 1
  const int used = sm.recoveriesUsed();

  // 恢复期间局部还在持续报被挡：不叠加计数，否则一个持续障碍会瞬间吃掉额度
  for (int i = 0; i < 5; ++i) {
    auto a = sm.handle(Event::kBlocked);
    EXPECT_EQ(a.to, State::kRecovering);
    EXPECT_EQ(a.effect, SideEffect::kStopRobot);
  }
  EXPECT_EQ(sm.recoveriesUsed(), used) << "恢复期间的 Blocked 不该叠加计数";
  EXPECT_EQ(sm.stats().recoveries, 1);
}

TEST(ManagerSm, TerminalStatesAcceptNewGoal) {
  for (State s : {State::kGoalReached, State::kFailed}) {
    ManagerSm sm;
    driveTo(sm, s);
    ASSERT_EQ(sm.state(), s);
    auto a = sm.handle(Event::kGoalReceived);
    EXPECT_TRUE(a.accepted) << toString(s);
    EXPECT_EQ(a.to, State::kPlanning) << toString(s);
    EXPECT_EQ(sm.state(), State::kPlanning);
  }
}

TEST(ManagerSm, CancelFromEveryStateGoesIdle) {
  for (State s : kAllStates) {
    ManagerSm sm;
    driveTo(sm, s);
    ASSERT_EQ(sm.state(), s) << "driveTo 没走到 " << toString(s);
    auto a = sm.handle(Event::kCancel);
    EXPECT_TRUE(a.accepted) << toString(s);
    EXPECT_EQ(a.to, State::kIdle) << toString(s);
    EXPECT_EQ(sm.state(), State::kIdle) << toString(s);
  }
}

TEST(ManagerSm, EveryStateEventPairIsExplicitlyHandled) {
  // 穷举 6×11 = 66 组：每条要么有条规则（accepted），要么明确拒绝且状态不变。
  for (State s : kAllStates) {
    for (Event e : kAllEvents) {
      ManagerSm sm;
      driveTo(sm, s);
      ASSERT_EQ(sm.state(), s);
      const State before = sm.state();
      auto a = sm.handle(e);
      const std::string tag = std::string(toString(s)) + " + " + toString(e);

      EXPECT_FALSE(a.reason.empty()) << tag << "：不管收不收都要给原因";
      if (a.accepted) {
        EXPECT_EQ(a.from, before) << tag;
        EXPECT_EQ(sm.state(), a.to) << tag;
      } else {
        EXPECT_EQ(sm.state(), before) << tag << "：拒绝事件必须保持原状态";
        EXPECT_EQ(a.effect, SideEffect::kNone) << tag << "：拒绝了就别下指令";
      }
    }
  }
}

TEST(ManagerSm, RejectedEventsDoNotMutateAnything) {
  ManagerSm sm;
  driveTo(sm, State::kFollowing);
  const auto stats = sm.stats();
  const int used = sm.recoveriesUsed();

  for (Event e : {Event::kPlanOk, Event::kPlanFail, Event::kRecoveryDone,
                  Event::kRecoveryFail}) {
    EXPECT_FALSE(sm.handle(e).accepted)
        << toString(e) << " 不该被 Following 接受";
  }
  EXPECT_EQ(sm.state(), State::kFollowing);
  EXPECT_EQ(sm.recoveriesUsed(), used);
  EXPECT_EQ(sm.stats().plan_failures, stats.plan_failures);
  EXPECT_EQ(sm.stats().recoveries, stats.recoveries);
}

// ------------------------------------------------------------ 端到端序列

TEST(ManagerSm, HappyPathSequence) {
  ManagerSm sm;
  std::vector<State> seen{sm.state()};
  auto step = [&](Event e) {
    auto a = sm.handle(e);
    EXPECT_TRUE(a.accepted) << toString(e) << "：" << a.reason;
    seen.push_back(a.to);
    return a.effect;
  };

  EXPECT_EQ(step(Event::kGoalReceived), SideEffect::kPlanPath);
  EXPECT_EQ(step(Event::kPlanOk), SideEffect::kStartFollow);
  step(Event::kReached);

  const std::vector<State> want{State::kIdle, State::kPlanning,
                                State::kFollowing, State::kGoalReached};
  EXPECT_EQ(seen, want);
  EXPECT_EQ(sm.stats().plan_requests, 1);
  EXPECT_EQ(sm.stats().plan_failures, 0);
  EXPECT_EQ(sm.stats().recoveries, 0);
}

TEST(ManagerSm, BlockedThenRecoveredThenReach) {
  SmParams p;
  p.max_recoveries = 2;
  ManagerSm sm(p);
  std::vector<std::string> seq;
  auto step = [&](Event e) {
    auto a = sm.handle(e);
    seq.push_back(std::string(toString(a.from)) + "->" + toString(a.to));
    return a;
  };

  step(Event::kGoalReceived);
  step(Event::kPlanOk);
  step(Event::kBlocked);
  step(Event::kRecoveryFail); // 未超限 → 再试
  step(Event::kRecoveryDone); // 成功 → 先重规划
  step(Event::kPlanOk);       // 重规划成功 → 回 Following
  const auto last = step(Event::kReached);

  EXPECT_EQ(last.to, State::kGoalReached);
  EXPECT_EQ(seq.back(), "FOLLOWING->GOAL_REACHED");
  EXPECT_EQ(sm.recoveriesUsed(), 2);
}

TEST(ManagerSm, ResetClearsStateAndStats) {
  ManagerSm sm;
  driveTo(sm, State::kFollowing);
  sm.handle(Event::kBlocked);
  ASSERT_NE(sm.state(), State::kIdle);
  ASSERT_GT(sm.stats().plan_requests, 0);

  sm.reset();
  EXPECT_EQ(sm.state(), State::kIdle);
  EXPECT_EQ(sm.stats().plan_requests, 0);
  EXPECT_EQ(sm.stats().recoveries, 0);
  EXPECT_EQ(sm.recoveriesUsed(), 0);
}

TEST(ManagerSm, NamesAreAvailableForEveryValue) {
  for (State s : kAllStates)
    EXPECT_STRNE(toString(s), "UNKNOWN") << "枚举扩了但 toString 漏了分支";
  for (Event e : kAllEvents)
    EXPECT_STRNE(toString(e), "UNKNOWN") << "枚举扩了但 toString 漏了分支";
  for (SideEffect f :
       {SideEffect::kNone, SideEffect::kPlanPath, SideEffect::kStartFollow,
        SideEffect::kStopRobot, SideEffect::kRunRecovery})
    EXPECT_STRNE(toString(f), "UNKNOWN");
  EXPECT_STREQ(toString(State::kFollowing), "FOLLOWING");
  EXPECT_STREQ(toString(Event::kBlocked), "Blocked");
  EXPECT_STREQ(toString(SideEffect::kPlanPath), "PlanPath");
}

} // namespace
} // namespace pnc_2d
