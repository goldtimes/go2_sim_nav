// 清图恢复行为（`clear_map`）的单测。
//
// 为什么值得单独测：它是**恢复链路的最后一环**（被挡 → 清图 → 重规划），而且它的
// 效果是"作用在**别人**（感知节点）身上"的副作用 ⇒ 只有用假回调断言"到底调没调、
// 调了几次"，才能在没有 ROS/没有仿真的情况下把它锁住。

#include <gtest/gtest.h>

#include "pnc_2d/core/clear_map_recovery.hpp"
#include "pnc_2d/core/param_reader.hpp"

using pnc_2d::ClearMapRecoveryBehavior;
using pnc_2d::MemoryParamReader;
using pnc_2d::RecoveryContext;

namespace {

/// 造一个"配了参数"的 reader（`min_interval_s` < 0 = 不配，用代码默认值）
MemoryParamReader params_with(double min_interval_s) {
  MemoryParamReader p;
  if (min_interval_s >= 0.0)
    p.setDouble("clear_map.min_interval_s", min_interval_s);
  return p;
}

} // namespace

// 1) 没有注入钩子 ⇒ **如实报失败**（不许静默成功：那会让状态机以为清过了）
TEST(ClearMapRecovery, NoHookReportsFailure) {
  ClearMapRecoveryBehavior b;
  auto p = params_with(-1.0);
  ASSERT_TRUE(b.configure(p));
  RecoveryContext ctx; // clearLocalCostMap 为空
  const auto r = b.run(ctx);
  EXPECT_FALSE(r.success);
  EXPECT_NE(r.message.find("clearLocalCostMap"), std::string::npos);
}

// 2) 有钩子 ⇒ 调一次、报成功，并且消息里说清"谁会被清、之后会发生什么"
TEST(ClearMapRecovery, CallsHookOnce) {
  ClearMapRecoveryBehavior b;
  auto p = params_with(-1.0);
  ASSERT_TRUE(b.configure(p));
  int calls = 0;
  RecoveryContext ctx;
  ctx.clearLocalCostMap = [&calls]() { ++calls; };
  const auto r = b.run(ctx);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(calls, 1);
  EXPECT_NE(r.message.find("清空局部代价地图"), std::string::npos);
}

// 3) 防抖：min_interval_s 内第二次**不调钩子**且报失败（防"挡住→清图→再挡"刷屏）
TEST(ClearMapRecovery, DebounceBlocksSecondCall) {
  ClearMapRecoveryBehavior b;
  auto p = params_with(60.0); // 60 s 防抖
  ASSERT_TRUE(b.configure(p));
  int calls = 0;
  RecoveryContext ctx;
  ctx.clearLocalCostMap = [&calls]() { ++calls; };
  EXPECT_TRUE(b.run(ctx).success);
  const auto r2 = b.run(ctx);
  EXPECT_FALSE(r2.success);
  EXPECT_EQ(calls, 1) << "防抖期间不许再动感知的地图";
  EXPECT_NE(r2.message.find("防抖"), std::string::npos);
}

// 4) 防抖默认关（0.0）：连续两次都调 —— 与 replan 同口径（额度由 sm.max_recoveries
//    收住，不该由行为自己造第二道闸，否则会把"清完立刻又被真障碍挡住"判成失败）
TEST(ClearMapRecovery, DefaultNoDebounce) {
  ClearMapRecoveryBehavior b;
  auto p = params_with(0.0);
  ASSERT_TRUE(b.configure(p));
  int calls = 0;
  RecoveryContext ctx;
  ctx.clearLocalCostMap = [&calls]() { ++calls; };
  EXPECT_TRUE(b.run(ctx).success);
  EXPECT_TRUE(b.run(ctx).success);
  EXPECT_EQ(calls, 2);
}

// 5) 名字与工厂一致（`recovery.type: clear_map` 能建出它）
TEST(ClearMapRecovery, NameIsClearMap) {
  ClearMapRecoveryBehavior b;
  EXPECT_EQ(b.name(), "clear_map");
}
