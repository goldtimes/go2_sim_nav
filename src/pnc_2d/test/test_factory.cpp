// 工厂与局部规划接口的单测（P3）。
//
// 覆盖：
//   1. 三套工厂都能按名字造出实例，名字表与实际能造出来的东西**一致**；
//   2. 未注册类型返回 nullptr（不抛异常、不静默回退到别的算法）；
//   3. NullLocalPlanner 的契约：不产生 cmd_vel、状态反映"有没有路径"、模式由走廊决定；
//   4. 基类 set*/mode()/hasPlan() 这些公共行为（换任何算法都不该变的那部分）。

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_planner.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/types.hpp"
#include "pnc_2d/local/null_local_planner.hpp"

namespace pnc_2d {
namespace {

std::vector<Pose2D> makePath(int n)
{
  std::vector<Pose2D> p;
  for (int i = 0; i < n; ++i)
    p.push_back(Pose2D{static_cast<double>(i), 0.0, 0.0});
  return p;
}

RouteCorridor makeCorridor()
{
  RouteCorridor c;
  c.centerline = makePath(3);
  c.half_width = 0.6;
  c.speed_limit = 0.5;
  c.edge_index = 7;
  return c;
}

// ---------------- 全局工厂 ----------------

TEST(Factory, GlobalKnownTypes)
{
  for (const auto &name : availablePlanners()) {
    auto p = createPlanner(name);
    ASSERT_NE(p, nullptr) << "名字表里有 " << name << " 却造不出来";
    EXPECT_EQ(p->type(), name) << "注册名与 type() 不一致（导致日志/参数前缀错位）";
  }
}

TEST(Factory, GlobalUnknownTypeIsNull)
{
  EXPECT_EQ(createPlanner("no_such_planner"), nullptr);
  EXPECT_EQ(createPlanner(""), nullptr);
  // 大小写敏感：写错大小写应该暴露出来，而不是悄悄用上另一个算法
  EXPECT_EQ(createPlanner("AStar"), nullptr);
}

TEST(Factory, GlobalNameTableNotEmpty)
{
  const auto names = availablePlanners();
  EXPECT_FALSE(names.empty());
  // 顺序稳定：报错信息里列出可用算法时用户看到的是同一个顺序
  EXPECT_EQ(names, (std::vector<std::string>{"astar", "route_network"}));
}

// ---------------- 局部工厂 ----------------

TEST(Factory, LocalKnownTypes)
{
  auto p = createLocalPlanner("null");
  ASSERT_NE(p, nullptr);
  EXPECT_EQ(p->type(), "null");

  // yaml 里写 none 更自然，与 null 等价（两个别名指向同一个实现）
  auto q = createLocalPlanner("none");
  ASSERT_NE(q, nullptr);
  EXPECT_EQ(q->type(), "null");
}

TEST(Factory, LocalUnknownTypeIsNull)
{
  // 注意：P3 时这里断言的是 "mpc" 必须为 nullptr（那时还没实现，用一条会失败的
  // 测试提醒"恢复行为/MPC 还没接"）。P5.1 实现后它变成合法类型，于是换成真正
  // 不存在的名字 —— 否则这条测试就变成"永远失败"的噪音。
  EXPECT_NE(createLocalPlanner("mpc"), nullptr) << "P5.1 起 mpc 必须可用";
  EXPECT_EQ(createLocalPlanner("dwa"), nullptr);
  EXPECT_EQ(createLocalPlanner("teb"), nullptr);
  EXPECT_EQ(createLocalPlanner(""), nullptr);
}

TEST(Factory, LocalNameTableMatchesCreation)
{
  for (const auto &name : availableLocalPlanners()) {
    EXPECT_NE(createLocalPlanner(name), nullptr) << "名字表里有 " << name << " 却造不出来";
  }
}

// ---------------- 恢复行为（P3 明确为空）----------------

TEST(Factory, RecoveryTableIsEmptyForNow)
{
  // 本期的"正确行为"就是空表；P6 加实现时这条测试会失败 —— 那是**故意的**，
  // 提醒改动者同时更新文档与状态机，而不是忘了恢复行为还没接。
  EXPECT_TRUE(availableRecoveries().empty());
  EXPECT_EQ(createRecoveryBehavior("clear_costmap"), nullptr);
  EXPECT_EQ(createRecoveryBehavior(""), nullptr);
}

// ---------------- NullLocalPlanner 契约 ----------------

TEST(NullLocalPlanner, DoesNotProduceCmdVel)
{
  NullLocalPlanner lp;
  // 状态机靠这个布尔量决定"要不要把 cmd_vel 发出去"：
  // false 表示零速度不是指令，而是"我不管" —— 否则会被误读成紧急停车。
  EXPECT_FALSE(lp.producesCmdVel());
  // Null 缺距离场也不算降级（它压根不用）
  EXPECT_FALSE(lp.degraded());
}

TEST(NullLocalPlanner, StatusFollowsPlanPresence)
{
  NullLocalPlanner lp;

  auto r0 = lp.computeCommand(Pose2D{0.0, 0.0, 0.0}, 0.1);
  EXPECT_EQ(r0.status, LocalStatus::kIdle);
  EXPECT_FALSE(r0.ok());
  EXPECT_FALSE(r0.message.empty()) << "说明原因比只回状态码好排查";
  EXPECT_EQ(r0.cmd.v, 0.0);
  EXPECT_EQ(r0.cmd.w, 0.0);

  lp.setGlobalPlan(makePath(4));
  auto r1 = lp.computeCommand(Pose2D{1.0, 0.0, 0.0}, 0.1);
  EXPECT_EQ(r1.status, LocalStatus::kFollowing);
  EXPECT_TRUE(r1.ok());
  EXPECT_EQ(r1.cmd.v, 0.0) << "即使有路径也不发速度";
  EXPECT_EQ(r1.cmd.w, 0.0);
  EXPECT_EQ(lp.commandCalls(), 2);
}

TEST(NullLocalPlanner, PlanNeedsTwoPoints)
{
  NullLocalPlanner lp;
  lp.setGlobalPlan(makePath(1));  // 单点不是路径
  EXPECT_FALSE(lp.hasPlan());
  EXPECT_EQ(lp.computeCommand(Pose2D{}, 0.1).status, LocalStatus::kIdle);
  lp.setGlobalPlan({});
  EXPECT_FALSE(lp.hasPlan());
}

TEST(NullLocalPlanner, ModeComesFromCorridorNotConfig)
{
  NullLocalPlanner lp;
  std::vector<Pose2D> path = makePath(3);
  lp.setGlobalPlan(path);
  EXPECT_EQ(lp.mode(), LocalPlanner::Mode::kFree);

  RouteCorridor c = makeCorridor();
  lp.setCorridor(&c);
  EXPECT_EQ(lp.mode(), LocalPlanner::Mode::kRoute);
  EXPECT_EQ(lp.corridor(), &c);
  EXPECT_DOUBLE_EQ(lp.speedLimit(), 0.0) << "走廊限速要通过 setSpeedLimit 显式传，不隐式生效";

  lp.setSpeedLimit(c.speed_limit);
  EXPECT_DOUBLE_EQ(lp.speedLimit(), 0.5);

  // nullptr 是"回到 free 模式"的唯一入口（换 planner 类型时靠它复位）
  lp.setCorridor(nullptr);
  EXPECT_EQ(lp.mode(), LocalPlanner::Mode::kFree);

  // 中心线不足 2 点的走廊不算数，仍视作 free（防止空走廊把 MPC 约束成"原地不动"）
  RouteCorridor bad;
  bad.centerline = makePath(1);
  lp.setCorridor(&bad);
  EXPECT_FALSE(bad.valid());
  EXPECT_EQ(lp.mode(), LocalPlanner::Mode::kFree);
}

TEST(NullLocalPlanner, ResetClearsEverything)
{
  NullLocalPlanner lp;
  std::vector<Pose2D> path = makePath(3);
  lp.setGlobalPlan(path);
  RouteCorridor c = makeCorridor();
  lp.setCorridor(&c);
  lp.setSpeedLimit(1.0);
  lp.setDynamicObstacles({DynamicObstacle{}});
  lp.computeCommand(Pose2D{}, 0.1);

  lp.reset();
  EXPECT_FALSE(lp.hasPlan());
  EXPECT_EQ(lp.mode(), LocalPlanner::Mode::kFree);
  EXPECT_DOUBLE_EQ(lp.speedLimit(), 0.0);
  EXPECT_TRUE(lp.dynamicObstacles().empty());
  EXPECT_EQ(lp.commandCalls(), 0);
}

TEST(NullLocalPlanner, ConfigureAcceptsAnyParams)
{
  NullLocalPlanner lp;
  MemoryParamReader empty;  // 全默认：不该因为"没配参数"而失败
  EXPECT_TRUE(lp.configure(empty));
}

// ---------------- 通过基类指针使用（状态机就是这样用的）----------------

TEST(LocalPlannerBase, PolymorphicUseThroughFactory)
{
  std::unique_ptr<LocalPlanner> lp = createLocalPlanner("null");
  ASSERT_NE(lp, nullptr);

  lp->setGlobalPlan(makePath(5));
  ASSERT_TRUE(lp->hasPlan());
  EXPECT_EQ(lp->globalPlan().size(), 5u);

  auto r = lp->computeCommand(Pose2D{0.0, 0.0, 0.0}, 0.1);
  EXPECT_EQ(r.status, LocalStatus::kFollowing);
  EXPECT_FALSE(lp->producesCmdVel());
}

TEST(LocalStatusString, AllValuesHaveNames)
{
  const LocalStatus all[] = {
      LocalStatus::kIdle, LocalStatus::kFollowing, LocalStatus::kGoalReached,
      LocalStatus::kBlocked, LocalStatus::kDegraded, LocalStatus::kFailed};
  for (auto s : all) {
    const std::string name = toString(s);
    EXPECT_STRNE(name.c_str(), "UNKNOWN") << "枚举扩了但 toString 漏了分支";
    EXPECT_FALSE(name.empty());
  }
  EXPECT_STREQ(toString(LocalStatus::kFollowing), "FOLLOWING");
}

}  // namespace
}  // namespace pnc_2d
