// RollingReplanShimPlanner（滚动式局部重规划）的单测。
//
// 用**合成栅格**测，不起 ROS：一堵横墙挡在参考路径前方，墙的左侧留一条可绕的缝。
// 断言的是"这类绕行到底有没有发生、有没有守住红线不变量"：
//   · 终点**必须不变**（绕行不能改任务）—— 最重要的一条；
//   · 绕行幅度有界、而且修补段自己得是**无碰撞**的（用同一套轮廓检查器复验）；
//   · 走廊模式**不绕行**（配置语义）；
//   · 节流生效（别每拍都做 A*）；
//   · 未知格：`unknown_as_occupied=false` 能绕过去（用户口径），置 true 就绕不过。

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/local/rolling_replan_shim.hpp"

using namespace pnc_2d;

namespace {

constexpr double kRes = 0.10;

/// 造一张局部图：0.10 m 栅格，世界范围 [-5,5]×[-5,5]（100×100，原点 (-5,-5)）。
/// `wall_x` 处放一堵沿 y 的墙，**挡住 y ∈ [block_y0, block_y1]**（其余地方留缝）。
/// `unknown_*` 指定要标成未知(-1)的 y 带（用来验"未知可通行"）。
std::shared_ptr<CostMap2D> make_map(double wall_x, double block_y0,
                                    double block_y1,
                                    double unknown_y0 = 1e9,
                                    double unknown_y1 = -1e9) {
  const int W = 100, H = 100;
  std::vector<int8_t> d(static_cast<std::size_t>(W) * H, 0);
  auto set = [&](double wx, double wy, int8_t v) {
    const int ix = static_cast<int>((wx + 5.0) / kRes);
    const int iy = static_cast<int>((wy + 5.0) / kRes);
    if (ix >= 0 && iy >= 0 && ix < W && iy < H)
      d[static_cast<std::size_t>(iy) * W + ix] = v;
  };
  // 墙（厚 0.2 m）：只挡 [block_y0, block_y1] 这一段，其余是缝
  for (double wy = -5.0; wy < 5.0; wy += kRes / 2)
    for (double dx = -0.1; dx <= 0.1; dx += kRes / 2)
      if (wy >= block_y0 && wy <= block_y1)
        set(wall_x + dx, wy, CostMap2D::kOccupied);
  // 未知带（放在缝的必经之路上）
  for (double wy = unknown_y0; wy <= unknown_y1; wy += kRes / 2)
    for (double wx = wall_x - 1.0; wx <= wall_x + 1.0; wx += kRes / 2)
      set(wx, wy, CostMap2D::kUnknown);
  auto m = std::make_shared<CostMap2D>();
  const bool ok = m->set(W, H, kRes, -5.0, -5.0, 0.0, d, "map");
  EXPECT_TRUE(ok);
  return m;
}

/// 参考路径：从 (0,0) 直着往 +x 走 4 m
std::vector<Pose2D> straight_path() {
  std::vector<Pose2D> p;
  for (double x = 0.0; x <= 4.0 + 1e-9; x += 0.05) {
    Pose2D q;
    q.x = x;
    q.y = 0.0;
    q.yaw = 0.0;
    q.has_yaw = true;
    p.push_back(q);
  }
  return p;
}

MemoryParamReader shim_params(double lookahead, bool unknown_as_occupied) {
  MemoryParamReader p;
  p.setString("rolling.primary", std::string("mpc")); // 主控制器：mpc 就够（不参与断言）
  p.setDouble("rolling.lookahead_m", lookahead);
  p.setDouble("rolling.back_m", 0.30);
  p.setDouble("rolling.roll_m", 1.50);
  p.setDouble("rolling.max_roll_m", 3.00);
  p.setDouble("rolling.max_offset_m", 1.20);
  p.setDouble("rolling.min_interval_s", 0.50);
  p.setDouble("rolling.min_progress_m", 0.10);
  p.setBool("rolling.unknown_as_occupied", unknown_as_occupied);
  p.setDouble("rolling.trigger_clearance_m", 0.45);
  p.setDouble("rolling.blocked_clearance_m", 0.25);
  p.setDouble("rolling.path_margin_m", 0.06);
  // 与全局层同源的轮廓
  p.setDouble("footprint.length", 0.78);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.offset_x", -0.038);
  p.setDouble("footprint.safe_margin", 0.05);
  return p;
}

/// 复验：一条路径上任何点都不能"轮廓碰障"（用与算法同一套检查器）
bool path_is_clear(const CostMap2D &map, const FootprintParams &fp,
                   const std::vector<Pose2D> &path) {
  FootprintCollisionChecker c;
  c.configure(fp, 50, false);
  c.setMap(&map);
  for (const auto &p : path)
    if (c.poseInCollision(p.x, p.y, p.yaw))
      return false;
  return true;
}

} // namespace

// ① 前方被挡 ⇒ 绕过去，且**终点不变**
TEST(RollingReplan, DetoursAroundBlockageAndKeepsGoal) {
  auto map = make_map(/*wall_x=*/1.2, /*block y∈*/ -0.6, 0.6); // 挡路径 1.2 m 宽
  LocalDistanceField field;
  ASSERT_TRUE(field.buildFromClearance(*map, 50, false, 3.0));
  auto params = shim_params(/*lookahead=*/1.5, /*unknown=*/false);

  RollingReplanShimPlanner shim;
  ASSERT_TRUE(shim.configure(params)) << "装饰器 + 主控制器都得建出来";
  shim.setCostMap(map);
  shim.setDistanceField(&field);
  shim.setGlobalPlan(straight_path());
  const Pose2D goal = shim.originalPlan().back();

  const Pose2D pose{0.0, 0.0, 0.0, true};
  (void)shim.computeCommand(pose, 0.05);

  EXPECT_GE(shim.repairStats().repairs, 1) << "前方 1.2 m 有墙，应该触发绕行";
  // ★ 终点必须还是任务目标（绕行不改任务）
  ASSERT_FALSE(shim.activePlan().empty());
  EXPECT_NEAR(shim.activePlan().back().x, goal.x, 1e-6);
  EXPECT_NEAR(shim.activePlan().back().y, goal.y, 1e-6);
  // 绕行幅度有界 + 真的绕了（墙占 y>-0.6，要绕就得往 -y 偏）
  EXPECT_GT(shim.repairStats().last_offset_m, 0.30);
  EXPECT_LE(shim.repairStats().last_offset_m, 1.20 + 1e-6);
  // 修补段自己必须无碰撞（不过就宁可不交）
  FootprintParams fp;
  fp.length = 0.78;
  fp.width = 0.40;
  fp.offset_x = -0.038;
  fp.safe_margin = 0.05;
  EXPECT_TRUE(path_is_clear(*map, fp, shim.activePlan()));
}

// ② 节流：紧接着的第二个周期不许再做一次 A*
TEST(RollingReplan, RateLimited) {
  auto map = make_map(1.2, -0.6, 0.6);
  LocalDistanceField field;
  ASSERT_TRUE(field.buildFromClearance(*map, 50, false, 3.0));
  auto params = shim_params(1.5, false);
  RollingReplanShimPlanner shim;
  ASSERT_TRUE(shim.configure(params));
  shim.setCostMap(map);
  shim.setDistanceField(&field);
  shim.setGlobalPlan(straight_path());
  const Pose2D pose{0.0, 0.0, 0.0, true};
  (void)shim.computeCommand(pose, 0.05);
  const int after_first = shim.repairStats().repairs;
  (void)shim.computeCommand(pose, 0.05); // 同一位姿、紧接一拍
  EXPECT_EQ(shim.repairStats().repairs, after_first)
      << "min_interval_s 内不许再修（否则每拍一次 A*，还会来回抖）";
}

// ③ 走廊模式：不绕行（配置语义 —— 走廊是"只能贴线"）
TEST(RollingReplan, NoDetourInCorridorMode) {
  auto map = make_map(1.2, -0.6, 0.6);
  LocalDistanceField field;
  ASSERT_TRUE(field.buildFromClearance(*map, 50, false, 3.0));
  auto params = shim_params(1.5, false);
  RollingReplanShimPlanner shim;
  ASSERT_TRUE(shim.configure(params));
  shim.setCostMap(map);
  shim.setDistanceField(&field);
  shim.setGlobalPlan(straight_path());
  RouteCorridor corridor;
  corridor.centerline = {{0.0, 0.0, 0.0, true}, {4.0, 0.0, 0.0, true}};
  corridor.half_width = 0.35;
  shim.setCorridor(&corridor);
  const Pose2D pose{0.0, 0.0, 0.0, true};
  (void)shim.computeCommand(pose, 0.05);
  EXPECT_EQ(shim.repairStats().repairs, 0) << "走廊模式下不该横挪（用户口径）";
}

// ④ 未知格可通行（用户口径）：同一堵墙，把缝标成未知
//    · unknown_as_occupied=false ⇒ 能绕过去
//    · true ⇒ 绕不过去（当作障碍）
TEST(RollingReplan, UnknownTraversableControl) {
  const double wall_x = 1.2;
  // 只留 y > 0.6 可以过，而那条缝正是**未知带** ⇒ 绕得过去 = “未知可通行”
  auto map_false = make_map(wall_x, -3.0, 0.6, /*u0=*/0.8, /*u1=*/2.0);
  auto map_true = make_map(wall_x, -3.0, 0.6, 0.8, 2.0);
  LocalDistanceField f_false, f_true;
  ASSERT_TRUE(f_false.buildFromClearance(*map_false, 50, false, 3.0));
  ASSERT_TRUE(f_true.buildFromClearance(*map_true, 50, true, 3.0));

  RollingReplanShimPlanner a, b;
  ASSERT_TRUE(a.configure(shim_params(1.5, false)));
  ASSERT_TRUE(b.configure(shim_params(1.5, true)));
  a.setCostMap(map_false);
  a.setDistanceField(&f_false);
  a.setGlobalPlan(straight_path());
  b.setCostMap(map_true);
  b.setDistanceField(&f_true);
  b.setGlobalPlan(straight_path());
  const Pose2D pose{0.0, 0.0, 0.0, true};
  (void)a.computeCommand(pose, 0.05);
  (void)b.computeCommand(pose, 0.05);
  EXPECT_GE(a.repairStats().repairs, 1) << "未知可通行时应该能绕";
  EXPECT_EQ(b.repairStats().repairs, 0) << "未知算障碍时不该绕（它会成为一堵墙）";
}
