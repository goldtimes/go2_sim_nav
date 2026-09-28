// M5.1：到点判定的单测（`core/goal_checker`）。
//
// 测什么（以及为什么不测别的）：
//   · **三条判据彼此独立**：沿向过、横向不过 ⇒ **不许**报到达（反之亦然）。
//     这两条混在一起过一次（原来只有"到目标点的欧氏距离"一个数），
//     实测让算法与节点在同一瞬间给出相反结论（见节点 `EndResidual` 的注释）。
//   · **`stop_coast` 必须被扣掉**：底盘的步态在指令归零后还会自己走 ~9 cm；
//     不扣就会冲过目标。判据用"当前残差 − 停车惯性"，所以同一组残差
//     在 `stop_coast` 不同时结论必须不同。
//   · **`stateful` 锁存**：满足后**漂走也不翻案**。它治的是真故障：
//     算法已报"到终点"，判定之后车还滑行 ~9 cm，残差被推出容差 ⇒
//     现在会把一次成功的到点报成 `FAILED`。
//   · **参数重载不许抖掉锁存**（热改参数是本项目的常规操作）。
//   · **诊断要走 `describe()`**：只报一句"没到"在上报里没有任何信息量。

#include <cmath>
#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "pnc_2d/core/goal_checker.hpp"

namespace pnc_2d {
namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

/// 一份"典型"参数：沿向 1 cm、横向 10 cm、朝向 10°、停车惯性 4 cm
GoalChecker::Params typical() {
  GoalChecker::Params p;
  p.along_tolerance = 0.01;
  p.lateral_tolerance = 0.10;
  p.yaw_tolerance = 10.0 * kDeg;
  p.stop_coast = 0.04;
  p.stateful = true;
  return p;
}

/// 已到位的残差（`along` 正好把停车惯性占满）
GoalResidual atGoal() {
  GoalResidual r;
  r.along = 0.045; // 0.045 − stop_coast 0.04 = 0.005 ≤ 0.01 ✓
  r.lateral = 0.005;
  r.yaw_err = 2.0 * kDeg;
  r.has_yaw = true;
  return r;
}

TEST(GoalChecker, LateralOverToleranceBlocksArrivalEvenWhenAlongIsDone) {
  GoalChecker c;
  c.configure(typical());
  GoalResidual r = atGoal();
  EXPECT_TRUE(c.satisfied(r)) << "先确认基准残差是过的";
  r.lateral = 0.30; // 沿向仍然过
  EXPECT_FALSE(c.satisfied(r)) << "沿向过、横向不过 ⇒ 不许报到达";
  EXPECT_STREQ(c.firstFailure(r), "lateral");
  EXPECT_NE(c.describe(r).find("横向超"), std::string::npos);
  std::printf("[M5.1] 横向超但沿向过：%s\n", c.describe(r).c_str());
}

TEST(GoalChecker, AlongOverToleranceBlocksArrivalEvenWhenLateralIsPerfect) {
  GoalChecker c;
  c.configure(typical());
  GoalResidual r = atGoal();
  r.along = 0.30; // 还没走到（横向完美贴线）
  EXPECT_FALSE(c.satisfied(r));
  EXPECT_STREQ(c.firstFailure(r), "along");
  EXPECT_NE(c.describe(r).find("沿向超"), std::string::npos);
}

TEST(GoalChecker, YawOverToleranceBlocksArrivalEvenWhenXyIsDone) {
  GoalChecker c;
  c.configure(typical());
  GoalResidual r = atGoal();
  r.yaw_err = 30.0 * kDeg; // 位置到了但机头没对正
  EXPECT_FALSE(c.satisfied(r));
  EXPECT_STREQ(c.firstFailure(r), "yaw");
  EXPECT_NE(c.describe(r).find("朝向超"), std::string::npos);
}

TEST(GoalChecker, YawIsSkippedWhenToleranceIsZeroOrThePathHasNoYaw) {
  GoalChecker c;
  // ① 本任务不要求朝向（容差 0）
  GoalChecker::Params p = typical();
  p.yaw_tolerance = 0.0;
  c.configure(p);
  GoalResidual r = atGoal();
  r.yaw_err = 90.0 * kDeg;
  EXPECT_TRUE(c.satisfied(r));
  // ② 路径末点没带朝向（纯几何点）
  c.configure(typical());
  r.has_yaw = false;
  EXPECT_TRUE(c.satisfied(r)) << "没有朝向要求的目标不许被朝向卡住";
}

TEST(GoalChecker, StopCoastIsSubtractedFromTheAlongResidual) {
  GoalChecker c;
  GoalChecker::Params p = typical(); // stop_coast = 0.04
  c.configure(p);
  GoalResidual r = atGoal();
  // ⚠ 别用"正好等于容差"的数：`0.04 + 0.01` 在 double 里是
  // 0.05000000000000001，
  //   减 0.04 之后 > 0.01 ⇒ 会变成"没到"。判据里用的是严格
  //   `>`，测试就别压在边界上。
  r.along = 0.04 + 0.011; // 比"容差 + 停车惯性"多 1 mm ⇒ 还没到
  EXPECT_FALSE(c.satisfied(r));
  r.along = 0.04 + 0.009; // 差 1 mm 用完 ⇒ 到
  EXPECT_TRUE(c.satisfied(r));
  // 把停车惯性去掉（底盘不再滑行）⇒ 同一组残差必须**不过**
  p.stop_coast = 0.0;
  c.configure(p);
  EXPECT_FALSE(c.satisfied(r))
      << "stop_coast=0 时同一组残差就是没走到（说明这一项真的参与了判定）";
}

TEST(GoalChecker, StatefulKeepsArrivalAfterThePoseDriftsAway) {
  GoalChecker c;
  c.configure(typical());
  const GoalResidual ok = atGoal();
  EXPECT_TRUE(c.update(ok));
  EXPECT_TRUE(c.latched());
  EXPECT_TRUE(c.justLatched()) << "第一次满足应报上升沿";
  // 判定之后车又滑行 ~9 cm（实测数字）⇒ 残差被推出容差
  GoalResidual drifted = ok;
  drifted.along += 0.09;
  EXPECT_TRUE(c.update(drifted))
      << "锁存以后漂走不许翻案（否则会把一次成功的到点报成 FAILED）";
  EXPECT_FALSE(c.justLatched()) << "上升沿只许报一次";
}

TEST(GoalChecker, NonStatefulFollowsTheInstantaneousVerdict) {
  GoalChecker c;
  GoalChecker::Params p = typical();
  p.stateful = false;
  c.configure(p);
  const GoalResidual ok = atGoal();
  EXPECT_TRUE(c.update(ok));
  EXPECT_FALSE(c.latched()) << "非锁存模式不许置锁存位";
  GoalResidual drifted = ok;
  drifted.lateral = 0.20;
  EXPECT_FALSE(c.update(drifted)) << "非锁存 = 每周期按当下残差判";
}

TEST(GoalChecker, ResetClearsTheLatchForTheNextTask) {
  GoalChecker c;
  c.configure(typical());
  EXPECT_TRUE(c.update(atGoal()));
  c.reset();
  EXPECT_FALSE(c.latched());
  GoalResidual r = atGoal();
  r.along = 0.50; // 新任务的起点还远
  EXPECT_FALSE(c.update(r));
  EXPECT_TRUE(c.update(atGoal())) << "reset 之后能重新判到点";
}

TEST(GoalChecker, ConfigureDoesNotClearTheLatch) {
  GoalChecker c;
  c.configure(typical());
  EXPECT_TRUE(c.update(atGoal()));
  GoalChecker::Params p = typical();
  p.lateral_tolerance = 0.03; // 热重载参数（本项目常规操作）
  c.configure(p);
  EXPECT_TRUE(c.latched()) << "参数重载不许把'已到'抖掉";
}

TEST(GoalChecker, DescribeAlwaysNamesTheFailingTermOrSaysReached) {
  GoalChecker c;
  c.configure(typical());
  EXPECT_NE(c.describe(atGoal()).find("到点"), std::string::npos);
  GoalResidual r = atGoal();
  r.lateral = 0.5;
  const std::string d = c.describe(r);
  EXPECT_NE(d.find("横向残差"), std::string::npos)
      << "必须报数值，不能只说'没到'";
  EXPECT_NE(d.find("容差"), std::string::npos);
  std::printf("[M5.1] 未到点的诊断串：%s\n", d.c_str());
}

} // namespace
} // namespace pnc_2d
