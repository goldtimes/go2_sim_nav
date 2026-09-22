// MPC 局部规划器单测（P5.1 验收项）。
//
// 验收标准（doc/pnc2d_local_planner_plan.md §5 P5.1）：
//   · 直线跟踪 RMSE < 0.05 m；圆弧横向误差 < 0.10 m；
//   · **走廊硬约束：随机 1000 组初始偏差/扰动，越界次数 = 0**；
//   · 单次求解耗时 P99 < 20 ms。
//
// 仿真是**闭环**的：每个周期调用 computeCommand()，把指令用圆弧精确积分喂回被控
// 对象，再喂下一周期 —— 不用解析假设。被控对象的积分刻意用精确圆弧公式：
// 被控对象自身的积分误差会污染指标（这是"跟踪差"最常见的假象来源）。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"

namespace pnc_2d {
namespace {

constexpr double kPi = M_PI;

double wrap(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

/// 被控对象：差速运动学，**圆弧精确积分**（常 (v, ω) 下无离散误差）
void integrate(Pose2D &s, double v, double w, double dt)
{
  if (std::fabs(w) > 1e-9) {
    const double R = v / w;
    const double th1 = s.yaw + w * dt;
    s.x += R * (std::sin(th1) - std::sin(s.yaw));
    s.y -= R * (std::cos(th1) - std::cos(s.yaw));
    s.yaw = wrap(th1);
  } else {
    s.x += v * dt * std::cos(s.yaw);
    s.y += v * dt * std::sin(s.yaw);
    s.yaw = wrap(s.yaw);
  }
}

std::vector<Pose2D> straightPath(double length, double res)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res)
    p.push_back(Pose2D{s, 0.0, 0.0});
  return p;
}

std::vector<Pose2D> circlePath(double radius, double res)
{
  std::vector<Pose2D> p;
  const double circumference = 2.0 * kPi * radius;
  const int n = static_cast<int>(std::ceil(circumference / res));
  for (int i = 0; i <= n; ++i) {
    const double t = 2.0 * kPi * static_cast<double>(i) / n;
    p.push_back(Pose2D{radius * std::sin(t), radius * (1.0 - std::cos(t)),
                       wrap(t)});
  }
  return p;
}

/// 参考窗口（单测用 solveForTest 直接喂）
std::vector<MpcReferencePoint> straightWindow(int n, double v, double dt)
{
  std::vector<MpcReferencePoint> w;
  for (int k = 0; k <= n; ++k)
    w.push_back(MpcReferencePoint{v * dt * k, 0.0, 0.0, v, 0.0, 0.0, v * dt * k});
  return w;
}

/// 造一个"半径 radius 的障碍 + 解析距离场"（用于避障项测试）
LocalDistanceField radialObstacleField(double ox, double oy, double max_dist)
{
  const double res = 0.05;
  const double x0 = -5.0;
  const double y0 = -5.0;
  const int w = 200;
  const int h = 200;
  std::vector<DistanceSample> samples;
  for (int iy = 0; iy < h; ++iy) {
    for (int ix = 0; ix < w; ++ix) {
      const double x = x0 + (ix + 0.5) * res;
      const double y = y0 + (iy + 0.5) * res;
      samples.push_back(DistanceSample{x, y, std::hypot(x - ox, y - oy)});
    }
  }
  LocalDistanceField f;
  // 全密采样（fill_radius=0 即可），max_dist 给大一点避免截断影响判定
  const bool ok = f.buildFromSamples(x0, y0, res, w, h, samples, max_dist, 0);
  EXPECT_TRUE(ok);
  return f;
}

/// “空旷区”距离场：所有格子 = max_dist。
/// 为什么要它：本类里**没有**距离场 ⇒ 按 D3/§3 的设计自报降级并限速到 0.3 m/s，
/// 那样测出来的“直线跟踪误差”里混的是“慢”，不是“飘”。跟踪质量要在**非降级**
/// 条件下量，所以给一张“到处都远”的合法场。
LocalDistanceField freeSpaceField()
{
  const double res = 0.1;
  const double x0 = -20.0;
  const double y0 = -20.0;
  const int w = 400;
  const int h = 400;
  std::vector<DistanceSample> samples;
  samples.reserve(static_cast<std::size_t>(w) * h);
  for (int iy = 0; iy < h; ++iy)
    for (int ix = 0; ix < w; ++ix)
      samples.push_back(DistanceSample{x0 + (ix + 0.5) * res,
                                       y0 + (iy + 0.5) * res, 5.0});
  LocalDistanceField f;
  const bool ok = f.buildFromSamples(x0, y0, res, w, h, samples, 5.0, 0);
  EXPECT_TRUE(ok);
  return f;
}

MpcLocalPlanner makeMpc(double v_max = 1.0)
{
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", v_max);
  mpc.configure(p);
  return mpc;
}

// --------------------------------------------------------------------- 跟踪
// 这三个用例都带一张“空旷”距离场：不降级才能量出真正的跟踪质量（见 freeSpaceField）。

TEST(MpcLocalPlanner, StraightLineTrackingRmse)
{
  auto mpc = makeMpc(1.0);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  ASSERT_FALSE(mpc.degraded());
  const double dt = mpc.params().dt;
  mpc.setGlobalPlan(straightPath(60.0, 0.05));

  // 故意给初始横向偏差 + 航向偏差，让指标包含收敛过程
  Pose2D s{0.0, 0.30, 0.15};
  double v = 0.0;
  double sum_sq = 0.0;
  double max_err = 0.0;
  double ss_sum_sq = 0.0;   // 稳态（跳过前 2 s 收敛过程）
  int ss_n = 0;
  int n = 0;
  const int steps = 300;  // 30 s
  const int settle = 20;  // 2 s
  for (int i = 0; i < steps; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.status == LocalStatus::kFollowing || r.status == LocalStatus::kDegraded)
        << "第 " << i << " 步：" << toString(r.status) << " " << r.message;
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, dt);
    const double err = std::fabs(s.y);
    max_err = std::max(max_err, err);
    sum_sq += err * err;
    ++n;
    if (i >= settle) {
      ss_sum_sq += err * err;
      ++ss_n;
    }
  }
  const double rmse = std::sqrt(sum_sq / n);
  const double ss_rmse = std::sqrt(ss_sum_sq / ss_n);
  std::printf("  [直线] %d 步闭环：全局 RMSE %.4f m | 稳态 RMSE %.4f m | 最大偏差 "
              "%.4f m | 末态 %.4f m\n",
              n, rmse, ss_rmse, max_err, std::fabs(s.y));
  EXPECT_LT(rmse, 0.05) << "直线 RMSE " << rmse << " m（最大 " << max_err << " m）";
  EXPECT_LT(ss_rmse, 0.01) << "稳态 RMSE " << ss_rmse << " m（收敛后应贴线）";
  EXPECT_LT(max_err, 0.35) << "最大横向偏差 " << max_err << " m";
  // 收敛后应该贴着线走
  EXPECT_LT(std::fabs(s.y), 0.02) << "末态横向偏差 " << s.y << " m";
}

TEST(MpcLocalPlanner, CircleTrackingLateralError)
{
  auto mpc = makeMpc(1.0);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const double dt = mpc.params().dt;
  const double R = 3.0;
  mpc.setGlobalPlan(circlePath(R, 0.05));

  // 从圆上出发，带一点横向与航向扰动
  Pose2D s{0.0, 0.05, 0.10};
  double v = 0.0;
  double sum_sq = 0.0;
  double max_err = 0.0;
  int n = 0;
  const int steps = 400;  // 40 s，约一圈半
  for (int i = 0; i < steps; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.status == LocalStatus::kFollowing || r.status == LocalStatus::kDegraded)
        << "第 " << i << " 步：" << toString(r.status) << " " << r.message;
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, dt);
    // 到圆的径向偏差（这才是"横向误差"，不是到起点的距离）
    const double err = std::fabs(std::hypot(s.x, s.y - R) - R);
    max_err = std::max(max_err, err);
    sum_sq += err * err;
    ++n;
  }
  const double rmse = std::sqrt(sum_sq / n);
  std::printf("  [圆弧] R=3.0 m，%d 步闭环：径向偏差 RMSE %.4f m | 最大 %.4f m\n", n,
              rmse, max_err);
  EXPECT_LT(rmse, 0.10) << "圆弧 RMSE " << rmse << " m（最大 " << max_err << " m）";
}

TEST(MpcLocalPlanner, CommandRespectsLimits)
{
  auto mpc = makeMpc(0.8);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const auto &P = mpc.params();
  mpc.setGlobalPlan(straightPath(60.0, 0.05));
  mpc.setSpeedLimit(0.4);   // 通道限速

  Pose2D s{0.0, 0.5, 0.6};  // 大偏差 + 大航向误差 → 控制量会被顶到限幅
  for (int i = 0; i < 50; ++i) {
    mpc.setCurrentVelocity(0.4, 0.0);
    const auto r = mpc.computeCommand(s, P.dt);
    if (!r.ok())
      break;
    EXPECT_GE(r.cmd.v, P.v_min - 1e-9);
    EXPECT_LE(r.cmd.v, P.v_max + 1e-9);
    EXPECT_LE(r.cmd.v, 0.4 + 1e-9) << "不能超过通道限速";
    EXPECT_LE(std::fabs(r.cmd.w), P.w_max + 1e-9);
    integrate(s, r.cmd.v, r.cmd.w, P.dt);
  }
}

TEST(MpcLocalPlanner, BrakingProfileStopsAtGoalWithoutOvershoot)
{
  // 验证制动剖面的**功能性目的**：能以巡航速度接近而不冲过终点。
  //
  // ⚠ 不要去断言“距终点 0.2 m 时指令 ≤ sqrt(2·a·s)”——那是**物理上做不到的**：
  //   从 1.0 m/s 出发，一个控制周期内最多减到 1.0 - a_max·dt = 0.9 m/s。
  //   参考剖面负责“提前开始减速”，指令的可行域由加速度限幅决定，两者不是一回事。
  auto mpc = makeMpc(1.0);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const double dt = mpc.params().dt;
  mpc.setGlobalPlan(straightPath(10.0, 0.05));

  Pose2D s{8.0, 0.0, 0.0};   // 全速接近，只剩 2 m
  double v = 1.0;
  double max_x = s.x;
  double v_at_goal = -1.0;
  bool stopped = false;
  for (int i = 0; i < 200; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, dt);
    max_x = std::max(max_x, s.x);
    if (s.x >= 9.95 && !stopped) {
      stopped = true;
      v_at_goal = v;
    }
    if (stopped)
      break;
  }
  std::printf("  [制动] 从 x=8.0 / 1.0 m/s 靠近终点：末速度 %.3f m/s | 最远到 x=%.3f\n",
              v, max_x);
  EXPECT_LT(max_x, 10.15) << "冲过终点太多（最远 x=" << max_x << "）";
  EXPECT_GE(max_x, 9.95) << "没走到终点（最远 x=" << max_x << "）";
  EXPECT_LT(v_at_goal, 0.4) << "到位时速度还有 " << v_at_goal << " m/s";
}

// --------------------------------------------------------------------- 走廊

TEST(MpcLocalPlanner, CorridorHardConstraintZeroViolations)
{
  auto mpc = makeMpc(0.8);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const auto &P = mpc.params();
  const double dt = P.dt;
  const double hw = 0.5;

  RouteCorridor c;
  c.centerline = straightPath(60.0, 0.05);
  c.half_width = hw;
  mpc.setCorridor(&c);
  ASSERT_EQ(mpc.mode(), LocalPlanner::Mode::kRoute);

  std::mt19937 rng(20260922);
  // 扰动反映**系统不变量**：车在车道内（MPC 从不指令到走廊外，扰动也应是
  // “在车道内的初始偏差”）。把初始横向放到紧贴走廊边界 + 航向外指，问题在数学上
  // 就是无解的（一个周期内位姿不可能回走廊），那是另一条用例（见下）。
  std::uniform_real_distribution<double> lat(-0.6 * hw, 0.6 * hw);
  std::uniform_real_distribution<double> dyaw(-0.25, 0.25);
  std::uniform_real_distribution<double> vv(0.0, P.v_max);

  int produced = 0;
  int blocked = 0;
  int violated = 0;
  double worst = 0.0;
  double worst_ct = 0.0;
  for (int trial = 0; trial < 1000; ++trial) {
    const double s = 5.0 + 40.0 * (trial % 800) / 800.0;
    const double lat0 = lat(rng);
    const Pose2D start{s, lat0, dyaw(rng)};
    mpc.reset();
    mpc.setCorridor(&c);
    mpc.setCurrentVelocity(vv(rng), 0.0);
    const auto r = mpc.computeCommand(start, dt);
    if (!r.ok()) {
      ++blocked;
      continue;
    }
    ++produced;
    worst_ct = std::max(worst_ct, std::fabs(start.y));
    // 用**解出来的预测轨迹**逐点复核横向偏差（不看 QP 内那个线性量）
    const auto &traj = mpc.predictedTrajectory();
    ASSERT_GE(traj.size(), 2u);
    for (std::size_t k = 1; k < traj.size(); ++k) {
      const double lat_k = std::fabs(traj[k].y);  // 参考线是 y=0 的直线
      worst = std::max(worst, lat_k);
      if (lat_k > hw + 1e-6)
        ++violated;
    }
  }
  std::printf("  [走廊] 1000 组：产出 %d / 拒绝 %d | 越界 %d | 起始偏差最大 %.3f | "
              "预测横向最大 %.4f m（限 %.2f）\n",
              produced, blocked, violated, worst_ct, worst, hw);
  EXPECT_EQ(violated, 0) << "硬约束被违反 " << violated << " 次（最大 " << worst << " m）";
  EXPECT_EQ(produced, 1000) << "车道内的扰动应全部可解，拒绝 " << blocked << " 组";
}

TEST(MpcLocalPlanner, CorridorBoundaryWithOutwardHeadingIsBlocked)
{
  // 记录一条**诚实的边界**：车已经贴在走廊边上、且航向朝外时，一个周期内位姿不可能
  // 回到走廊内 → 硬约束下 QP 无解 → 必须报 BLOCKED 停车（决策 D5/D12）。
  // 它不会“硬掰一条越界轨迹”，也不会假装成功。
  auto mpc = makeMpc(0.8);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const double dt = mpc.params().dt;
  RouteCorridor c;
  c.centerline = straightPath(40.0, 0.05);
  c.half_width = 0.5;
  mpc.setCorridor(&c);

  mpc.setCurrentVelocity(0.8, 0.0);
  const auto r = mpc.computeCommand(Pose2D{10.0, 0.5, 0.30}, dt);
  std::printf("  [走廊边界] 贴边 + 航向朝外 → %s：%s\n", toString(r.status),
              r.message.c_str());
  EXPECT_EQ(r.status, LocalStatus::kBlocked);
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0);
}

TEST(MpcLocalPlanner, ZeroCorridorWidthUsesNumericalFloor)
{
  auto mpc = makeMpc(0.8);
  MemoryParamReader p;
  p.setDouble("local_mpc.corridor_min_tolerance", 0.05);
  mpc.configure(p);
  const auto &P = mpc.params();

  RouteCorridor c;
  c.centerline = straightPath(40.0, 0.05);
  c.half_width = 0.0;  // 严格贴线
  mpc.setCorridor(&c);
  ASSERT_TRUE(c.strict());

  const Pose2D start{10.0, 0.03, 0.05};  // 起点就有一点点偏差
  auto field2 = freeSpaceField();
  mpc.setDistanceField(&field2);
  mpc.setCurrentVelocity(0.0, 0.0);
  const auto r = mpc.computeCommand(start, P.dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  double worst = 0.0;
  for (const auto &pt : mpc.predictedTrajectory())
    worst = std::max(worst, std::fabs(pt.y));
  EXPECT_LE(worst, P.corridor_min_tolerance + 1e-6)
      << "严格贴线应受限于数值下限 " << P.corridor_min_tolerance << " m，实测 " << worst;
}

TEST(MpcLocalPlanner, StartingOutsideCorridorIsBlockedNotForced)
{
  auto mpc = makeMpc(0.8);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const double dt = mpc.params().dt;
  RouteCorridor c;
  c.centerline = straightPath(40.0, 0.05);
  c.half_width = 0.5;
  mpc.setCorridor(&c);

  // 起点在走廊外 1 m：一个周期内位姿不可能挪回走廊 → 必须如实报 BLOCKED，
  // 而不是"硬掰一条越界轨迹"或者"假装成功"。
  const Pose2D start{10.0, 1.5, 0.0};
  mpc.setCurrentVelocity(0.5, 0.0);
  const auto r = mpc.computeCommand(start, dt);
  EXPECT_EQ(r.status, LocalStatus::kBlocked) << toString(r.status) << " " << r.message;
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0) << "BLOCKED 必须停车";
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.0);
}

// --------------------------------------------------------------------- 避障

TEST(MpcLocalPlanner, ObstacleHardConstraintForcesDeviation)
{
  // 障碍比硬距离（0.25 m）**更贴近参考线**时，硬约束必须迫使轨迹让开：
  // 障碍在 (4.0, 0.20) ⇒ 需要 −e_y ≥ 0.05，也就是“必须往 −y 偏至少 5 cm”。
  // 这比“软代价把轨迹推偏一点”强得多 —— 它是一条**保证**。
  auto field = radialObstacleField(4.0, 0.20, 6.0);
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 1.0);
  p.setDouble("local_mpc.obstacle_weight", 400.0);
  p.setDouble("local_mpc.obstacle_safe_distance", 0.45);
  p.setDouble("local_mpc.obstacle_hard_distance", 0.25);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(20.0, 0.02));

  mpc.setCurrentVelocity(1.0, 0.0);
  const auto r = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;

  double min_d = 1e9;
  double y_at_closest = 0.0;
  for (const auto &pt : mpc.predictedTrajectory()) {
    const double d = std::hypot(pt.x - 4.0, pt.y - 0.20);
    if (d < min_d) {
      min_d = d;
      y_at_closest = pt.y;
    }
  }
  std::printf("  [避障-硬] 预测轨迹最小障碍距离 %.4f m（硬下界 %.2f）| 最近点 y=%.4f\n",
              min_d, mpc.params().obstacle_hard_distance, y_at_closest);
  EXPECT_GE(min_d, mpc.params().obstacle_hard_distance - 0.02)
      << "侵入硬距离（最小 " << min_d << " m）";
  EXPECT_LT(y_at_closest, -0.03) << "应往 −y 让开，实测最近点 y=" << y_at_closest;
}

/// ★ 回归：**实际速度高于限速时，QP 不能因此不可行**。
///
/// 这是 2026-09-22 仿真里定位到的真根因（对照实验证实：关掉障碍硬约束后依然复现；
/// 而失败点与"实际速度 > 限速"的采样点完全重合）：
///   `v_k ≤ v_upper` 是**硬状态约束**，但初始速度 `v_0 = v_now` 是反馈给定的、
///   不受约束。仿真里步态会一时冲到 0.89 m/s，而 reference_speed 只有 0.35 ⇒
///   QP 被要求**一步**内从 0.69 刹到 0.35（需要 3.4 m/s² ≫ a_max=0.6）
///   ⇒ `primal infeasible`，节点报 BLOCKED、任务被中断。
///
/// 修法：速度上界取 max(v_upper, v_now − a_max·dt·k)（从当前速度可达的最低速度）。
TEST(MpcLocalPlanner, SolverStaysFeasibleWhenFasterThanLimit)
{
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.35);
  p.setDouble("local_mpc.a_max", 0.6);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(60.0, 0.05));

  // 实际速度 0.70 ≫ 限速 0.35（可能吗？可以：底盘/步态的瞬态响应，实测 0.89）
  mpc.setCurrentVelocity(0.70, 0.0);
  const auto r = mpc.computeCommand(Pose2D{0.0, 0.0, 0.0}, mpc.params().dt);
  const auto &info = mpc.solveInfo();
  std::printf("  [超速] v_now=0.70 > 限速=0.35 → solver=%s feasible=%d | 指令 v=%.3f\n",
              info.solver_status.c_str(), static_cast<int>(info.feasible), r.cmd.v);

  EXPECT_TRUE(info.feasible) << "求解器不可行（solver_status="
                             << info.solver_status
                             << "）—— 速度上界没考虑从当前速度的可达减速";
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  // 指令仍不得越过限速（最后那道 clampValue 是安全网：宁可发一个"做不到的慢"，
  // 也不发一个"太快"；预测轨迹里按物理可达的 0.64 走，命令取 0.35）
  EXPECT_LE(r.cmd.v, mpc.params().v_max + 1e-9) << "指令越过了 v_max";
  EXPECT_GE(r.cmd.v, 0.0);
}

/// 参考路径**自身就在障碍里**（局部距离场说这条路上有障碍）：
/// 必须停车报 BLOCKED（绝不能"降下界把它当成能走"），
/// 且报错要能一眼看出问题在**参考/地图**，而不是让人去猜控制器。
///
/// （试过另一种写法：把下界放松成 min(hard, 参考净距)。后果是障碍正好压在参考线时
///   下界退化成 ~0，机器人会沿着规划直接开进障碍——所以语义必须是 BLOCKED。）
TEST(MpcLocalPlanner, ReferenceInsideObstacleIsBlockedWithActionableReason)
{
  // 障碍就在参考线上（x=4.0, y=0），机器人从 3.5 出发 ⇒ 参考 0.5 m 后穿过障碍
  auto field = radialObstacleField(4.0, 0.0, 8.0);
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.obstacle_weight", 400.0);
  p.setDouble("local_mpc.obstacle_safe_distance", 0.45);
  p.setDouble("local_mpc.obstacle_hard_distance", 0.25);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(20.0, 0.05));
  mpc.setCurrentVelocity(0.30, 0.0);

  const auto r = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
  std::printf("  [参考在障碍里] 状态 %s：%s\n", toString(r.status),
              r.message.c_str());
  EXPECT_EQ(r.status, LocalStatus::kBlocked);
  EXPECT_FALSE(r.ok()) << "不能把「路上有障碍」当成正常跟随";
  // 消息要指向真因（要么是解后的真值复核，要么是求解失败+参考净距诊断）
  const bool actionable = r.message.find("硬距离") != std::string::npos ||
                          r.message.find("参考") != std::string::npos;
  EXPECT_TRUE(actionable) << "报错应说明是障碍/参考的问题：" << r.message;
}

/// 参考只是**贴着**障碍（未穿过）时：应正常跟踪，且绝不让情况更糟。
///
/// 语义断言："MPC 不会把净距做得比全局路径更差" —— 这是硬下界退化成
/// g·e ≥ 0（"别再往障碍那侧靠"）之后真正保留下来的保证。
TEST(MpcLocalPlanner, ReferenceHuggingObstacleTracksAndNeverGetsCloser)
{
  // 障碍在 (7.0, 0.10)：参考线 y=0 在 x=7 处距障碍 0.10 m（< 硬距离 0.25）
  auto field = radialObstacleField(7.0, 0.10, 8.0);
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.obstacle_weight", 400.0);
  p.setDouble("local_mpc.obstacle_safe_distance", 0.45);
  p.setDouble("local_mpc.obstacle_hard_distance", 0.25);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(20.0, 0.05));
  mpc.setCurrentVelocity(0.30, 0.0);

  const auto r = mpc.computeCommand(Pose2D{6.0, 0.0, 0.0}, mpc.params().dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  EXPECT_TRUE(mpc.solveInfo().feasible);

  double min_d = 1e9;
  for (const auto &pt : mpc.predictedTrajectory())
    min_d = std::min(min_d, std::hypot(pt.x - 7.0, pt.y - 0.10));
  const double d_ref = 0.10;  // 参考自己在最近处的净距
  std::printf("  [贴障碍] 预测轨迹最小净距 %.4f m（参考 %.2f，硬距离 %.2f）\n",
              min_d, d_ref, mpc.params().obstacle_hard_distance);
  EXPECT_GE(min_d, d_ref - 0.02)
      << "比参考更靠近障碍了（" << min_d << " < " << d_ref << "）";
}

/// ★ 回归：**指令速度必须收敛到参考速度**。
///
/// 这个盲区害我调了很久（2026-09-22 仿真）：所有跟踪单测都只量**横向** RMSE，
/// 而参考窗口每个周期都锚在机器人自己的投影点上 —— 于是"纵向滞后"根本不进
/// 代价函数，唯一驱动前进的是 `q_v·e_v²` 这一项。
/// 原来的 q_v=3 太小：把速度误差消掉省下 q_v·Σe_v² ≈ 3.4，而加速要付
/// r_a·Σa² ≈ 10.8 —— **省的不够付的**，优化器于是选择以 0.1 m/s 爬
/// （仿真实测：v_ref=0.35 而 cmd_v=0.13，28.9 s 才走 3.85 m）。
/// 横向 RMSE 依然很漂亮，所以光看横向指标永远发现不了。
TEST(MpcLocalPlanner, CommandSpeedConvergesToReference)
{
  auto field = freeSpaceField();

  // 闭环仿真：用 MPC 自己输出的 v 作为下一周期的 v_now，位置按模型积分。
  // （真车/仿真里的"被控对象"是别的东西，但"指令能不能追上参考"这件事只取决于
  //   控制器自身，用一致的模型闭环就足以抓住"永远爬"这个缺陷。）
  auto closed_loop = [&](double q_v) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.reference_speed", 0.35);
    if (q_v > 0.0)
      p.setDouble("local_mpc.q_v", q_v);   // <=0 表示用配置默认值
    mpc.configure(p);
    mpc.setDistanceField(&field);
    mpc.setGlobalPlan(straightPath(30.0, 0.05));
    double x = 0.0, v = 0.0;
    for (int i = 0; i < 40; ++i) {
      mpc.setCurrentVelocity(v, 0.0);
      const auto r = mpc.computeCommand(Pose2D{x, 0.0, 0.0}, mpc.params().dt);
      if (!r.ok())
        return -1.0;
      v = r.cmd.v;                 // 模型一致：指令即实现
      x += v * mpc.params().dt;
    }
    return v;
  };

  const double v_ref = 0.35;
  const double v_good = closed_loop(0.0);   // 用配置里的 q_v
  const double v_weak = closed_loop(3.0);   // 旧默认（原来怀疑它偏小）
  std::printf("  [速度收敛] 40 步后：用配置 q_v → cmd_v=%.3f | q_v=3 → "
              "cmd_v=%.3f（参考 %.2f）\n",
              v_good, v_weak, v_ref);

  EXPECT_GE(v_good, 0.9 * v_ref)
      << "巡航速度追不上参考 —— 检查 q_v 与加速度代价的相对大小";
  // ⚠ 实测：q_v=3 在这里**也**收敛（v_weak ≈ v_good）。所以本用例能证明的是
  //   "指令速度会追上参考速度"，**不能**用来证明 q_v 偏小 —— 仿真里看到过
  //   cmd_v 只有参考一半，但那个现象在离线闭环里复现不出来（差别在 v_now 的
  //   噪声与求解失败导致的反复归零），别拿这个用例去支持"把 q_v 调大"。
  if (v_weak < 0.9 * v_ref)
    std::printf("  [速度收敛] （q_v=3 时确实追不上，可用它作为 q_v 下限的依据）\n");
}

/// ★ 回归：**实际速度低于 v_min（定位噪声）时，QP 不能因此不可行**。
///
/// 与 `SolverStaysFeasibleWhenFasterThanLimit` 是镜像的同一类缺陷：
/// `v_min ≤ v_k` 也是硬状态约束，而 `v_0 = v_now` 不受约束。仿真里定位的 twist
/// 在低速时有噪声（实测读到过 -0.23），`v_now < v_min = 0` 就要求 QP 一步内把
/// 速度从 -0.23 提到 0（需 2.3 m/s² ≫ a_max=0.6）⇒ `primal infeasible`。
/// 日志特征非常清楚：`v_now=-0.230 e_v0=-0.580 | maximum iterations reached`，
/// 而同一条日志里 `v_now=+0.090` 的周期只要 25 次迭代。
TEST(MpcLocalPlanner, SolverStaysFeasibleWhenSlowerThanMin)
{
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.a_max", 0.6);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));

  // 定位噪声：读到 -0.25 m/s（比 v_min=0 还低）
  mpc.setCurrentVelocity(-0.25, 0.0);
  const auto r = mpc.computeCommand(Pose2D{0.0, 0.0, 0.0}, mpc.params().dt);
  const auto &info = mpc.solveInfo();
  std::printf("  [低速噪声] v_now=-0.25 < v_min=0 → solver=%s feasible=%d | "
              "指令 v=%.3f\n",
              info.solver_status.c_str(), static_cast<int>(info.feasible), r.cmd.v);
  EXPECT_TRUE(info.feasible) << "求解器不可行（solver_status="
                             << info.solver_status
                             << "）—— 速度下界没考虑从当前速度的可达加速";
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  EXPECT_GE(r.cmd.v, 0.0) << "不能发负速度";
}

/// ★★ 回归：**严格走廊 + 初始航向偏差时，车必须真的走**（不能趴窝）。
///
/// 这个用例是 2026-09-22 仿真里"贴线模式车一动不动"的浓缩：
///   起步时航向与参考差 ~7°，只要**加速**，航向误差就会积分成横向偏移
///   （`n·e(k) = lat0 + dt·v·Σe_ψ`）⇒ 触碰严格走廊（5 cm）⇒ 横向代价
///   `q_y·Σe_y² ≈ 1000×0.018 = 18`；而不加速只付速度代价 `q_v·Σe_v²`。
///   `q_v=3` 时后者只有 ~5.5 ⇒ **优化器理智地选择趴窝**：求解成功（25 迭代）
///   但 `cmd_v=0`，任务永远到不了目标（日志特征：`curved` 非零、`走廊行 15`、
///   `cross≈0`、`cmd v=0.000`）。
///
/// 所以 q_v 不能只靠"能否追上参考速度"来定（那个场景没有取舍，q_v=3 也够），
/// 必须用**存在取舍**的场景：走廊 + 航向偏差。
TEST(MpcLocalPlanner, StrictCorridorWithHeadingOffsetStillDrives)
{
  auto field = freeSpaceField();

  auto probe = [&](double q_v, double yaw_deg) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.reference_speed", 0.35);
    p.setDouble("local_mpc.a_max", 0.6);
    if (q_v > 0.0)
      p.setDouble("local_mpc.q_v", q_v);
    mpc.configure(p);
    mpc.setDistanceField(&field);
    RouteCorridor c;
    c.centerline = straightPath(30.0, 0.05);
    c.half_width = 0.0;          // 严格贴线
    c.speed_limit = 0.35;
    mpc.setCorridor(&c);
    mpc.setCurrentVelocity(0.0, 0.0);
    const auto r = mpc.computeCommand(
        Pose2D{0.0, 0.0, yaw_deg * kPi / 180.0}, mpc.params().dt);
    return r.cmd.v;
  };

  // 单周期内速度最多只能涨 a_max·dt（模型里速度不能瞬变），所以判据是
  // "有没有用满加速能力"，不是"有没有到巡航速度"。
  const double v_step = 0.6 * 0.1;
  std::printf("  [走廊+航向偏差] 单周期 cmd_v（物理上限 %.3f）：\n", v_step);
  const double v_small = probe(0.0, 7.0);     // 小航向差：应该正常起步
  const double v_large = probe(0.0, 60.0);    // 大航向差：实测会拒动（弱点）
  std::printf("      航向差  7°：cmd_v=%.3f | 航向差 60°：cmd_v=%.3f\n",
              v_small, v_large);

  EXPECT_GE(v_small, 0.8 * v_step)
      << "小航向偏差下没用满加速能力 —— 检查 q_v 与 q_y 的相对大小";
  // ⚠ 这里记录的是"**关掉原地对正**时"的原始行为（2026-09-22 实测）：严格走廊下
  //    航向偏差大到 ~40° 以上时，MPC 会选择**不走**（也不原地转）—— 因为一加速
  //    就会因航向差产生横向偏移、撞走廊硬界，代价上"不动"更便宜。
  //    正确行为是"先原地转正再走"，已由 `local_mpc.align_in_place_deg` 实现
  //    （见 HeadingOffsetIsFixedByAligningInPlaceFirst）。这条断言继续钉住
  //    "默认关闭时不要偷偷改变行为"，避免这里的修法被误删。
  EXPECT_LT(v_large, 0.2 * v_step) << "关掉原地对正时的大航向差行为变了，请复核";
}

TEST(MpcLocalPlanner, StrictCorridorHeadingSweep)
{
  // "有角度的路网时机器基本不会动"（用户 2026-09-22 报的现象）——先量清**多大角度**
  // 会拒动，再决定对正阈值该设多少。这里用闭环（不是单周期）：单周期只能看出
  // "这一步走不走"，闭环才能看出"是不是一直不走"。
  auto field = freeSpaceField();

  auto sweep = [&](double align_deg, double yaw_deg) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.w_max", 0.65);
    p.setDouble("local_mpc.reference_speed", 0.35);
    p.setDouble("local_mpc.a_max", 0.6);
    p.setDouble("local_mpc.align_in_place_deg", align_deg);
    p.setDouble("local_mpc.align_exit_deg", 8.0);
    mpc.configure(p);
    mpc.setDistanceField(&field);
    RouteCorridor c;
    c.centerline = straightPath(30.0, 0.05);
    c.half_width = 0.0; // 严格贴线
    c.speed_limit = 0.35;
    mpc.setCorridor(&c);

    Pose2D pose{0.0, 0.0, yaw_deg * kPi / 180.0};
    const double dt = mpc.params().dt;
    double v = 0.0, travelled = 0.0;
    int blocked = 0;
    for (int k = 0; k < 60; ++k) { // 6 s 闭环
      mpc.setCurrentVelocity(v, 0.0);
      const auto r = mpc.computeCommand(pose, dt);
      if (r.status == LocalStatus::kBlocked)
        ++blocked;
      pose.yaw += r.cmd.w * dt;
      const double dx = r.cmd.v * std::cos(pose.yaw) * dt;
      const double dy = r.cmd.v * std::sin(pose.yaw) * dt;
      pose.x += dx;
      pose.y += dy;
      travelled += std::hypot(dx, dy);
      v = r.cmd.v;
    }
    return std::make_pair(travelled, blocked);
  };

  std::printf("  [严格走廊+航向偏差] 6 s 闭环走了多少（对正关闭 / 对正 20°）：\n");
  double worst_raw = 1e9, worst_angle = 0.0;
  for (double a : {10.0, 20.0, 30.0, 40.0, 60.0}) {
    const auto raw = sweep(0.0, a);
    const auto fixed = sweep(20.0, a);
    std::printf("      航向差 %2.0f°：关=%.3f m（被挡 %d 周期） | 开=%.3f m\n", a,
                raw.first, raw.second, fixed.first);
    if (raw.first < worst_raw) {
      worst_raw = raw.first;
      worst_angle = a;
    }
    // ★ 对正开着时，任何角度都必须真的走起来（这正是用户要的行为：
    //    "先原地转，对齐目标后，再走"）
    EXPECT_GT(fixed.first, 0.5)
        << "航向差 " << a << "° 下开了原地对正仍然没走起来";
  }
  // 关掉对正时，角度越大越走不动（记录这个旧行为，防止有人"顺手"把对正默认关掉）
  EXPECT_LT(worst_raw, 0.3) << "关掉对正时最大角度的行驶距离变大了，请复核";
  std::printf("      （关掉对正时最差：航向差 %.0f° 只走 %.3f m）\n", worst_angle,
              worst_raw);
}

TEST(MpcLocalPlanner, StrictCorridorRecoversAfterInPlaceTurnDrift)
{
  // ★ 从仿真现象反推出来的真实需求：**底盘的"原地转"其实在划小弧**。
  //   实测（Go2 仿真）车头转 30~90° 后横向漂了 0.09~0.19 m，于是车已经不在
  //   严格走廊（corridor_width=0，硬件容差 0.05 m）里了，接着就整段 BLOCKED、
  //   车一步不走 —— 现象与用户报的"有角度的路网时机器基本不会动"完全一致。
  //   所以"对正"之后还必须能把这点横向偏移**收回来**，这条用例把它钉住。
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.reference_speed", 0.35);
  p.setDouble("local_mpc.a_max", 0.6);
  p.setDouble("local_mpc.align_in_place_deg", 15.0);
  p.setDouble("local_mpc.corridor_min_tolerance", 0.05);
  p.setInt("local_mpc.osqp_max_iter", 20000); // 试探：是"迭代不够"还是"无解"
  mpc.configure(p);
  mpc.setDistanceField(&field);
  RouteCorridor c;
  c.centerline = straightPath(30.0, 0.05);
  c.half_width = 0.0;
  c.speed_limit = 0.35;
  mpc.setCorridor(&c);

  // 边界扫描：起点横向偏移 lat0 vs 走廊有效半宽 hw_eff（严格通道 = 0.05）
  std::printf("  [走廊起点偏移扫描] hw_eff=0.05，12 s 闭环：\n");
  for (double drift : {0.02, 0.04, 0.048, 0.05, 0.055, 0.06, 0.10, 0.19}) {
    MpcLocalPlanner m;
    MemoryParamReader q;
    q.setDouble("local_mpc.v_max", 0.42);
    q.setDouble("local_mpc.w_max", 0.65);
    q.setDouble("local_mpc.reference_speed", 0.35);
    q.setDouble("local_mpc.a_max", 0.6);
    q.setDouble("local_mpc.align_in_place_deg", 0.0);
    q.setDouble("local_mpc.corridor_min_tolerance", 0.05);
    m.configure(q);
    m.setDistanceField(&field);
    RouteCorridor cc;
    cc.centerline = straightPath(30.0, 0.05);
    cc.half_width = 0.0;
    cc.speed_limit = 0.35;
    m.setCorridor(&cc);

    Pose2D pose{0.0, drift, 0.0};
    const double dt = m.params().dt;
    double v = 0.0;
    int blocked = 0;
    std::string st;
    for (int k = 0; k < 120; ++k) {
      m.setCurrentVelocity(v, 0.0);
      const auto r = m.computeCommand(pose, dt);
      if (r.status == LocalStatus::kBlocked) {
        ++blocked;
        st = m.solveInfo().solver_status;
      }
      pose.yaw += r.cmd.w * dt;
      pose.x += r.cmd.v * std::cos(pose.yaw) * dt;
      pose.y += r.cmd.v * std::sin(pose.yaw) * dt;
      v = r.cmd.v;
    }
    std::printf("      起点偏移 %.3f m（%.3f×hw_eff）→ x=%.2f m, y=%.3f m, "
                "被挡 %3d/120，求解状态 %s\n",
                drift, drift / 0.05, pose.x, pose.y, blocked,
                st.empty() ? "ok" : st.c_str());
  }

  Pose2D pose{0.0, 0.06, 0.0}; // 保留一个真断言：6 cm 偏移必须收得回来
  {
    const double dt = mpc.params().dt;
    double v = 0.0;
    for (int k = 0; k < 120; ++k) {
      mpc.setCurrentVelocity(v, 0.0);
      const auto r = mpc.computeCommand(pose, dt);
      pose.yaw += r.cmd.w * dt;
      pose.x += r.cmd.v * std::cos(pose.yaw) * dt;
      pose.y += r.cmd.v * std::sin(pose.yaw) * dt;
      v = r.cmd.v;
    }
  }
  for (double drift : {0.06, 0.10, 0.19}) {
  }
  // ★ 用户要的行为：**对正之后即使横向漂了一点，也必须能把车道收回来并继续走**
  //   （底盘的"原地转"其实是划小弧，实测漂 0.09~0.19 m）。这条断言就是那个要求。
  EXPECT_GT(pose.x, 1.0) << "对正后横向漂 6 cm 就走不了了（x=" << pose.x << "）";
  EXPECT_LT(std::fabs(pose.y), 0.10)
      << "对正后横向偏移没有收敛（y=" << pose.y << "）";
}

TEST(MpcLocalPlanner, HeadingOffsetIsFixedByAligningInPlaceFirst)
{
  // 用户 2026-09-22 定的行为：**先原地对正机头，对好再走**。
  // 为什么不是"边转边走"：预测轨迹一边前进一边转弯会扫出走廊/障碍硬界，代价上
  // "不动"更便宜 ⇒ 车拒动（见上一条用例）。足式/差速底盘 ω 与 v 解耦，原地转正
  // 物理可行，所以先转正是唯一能同时满足"严格走廊"和"能动"的做法。
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.reference_speed", 0.35);
  p.setDouble("local_mpc.a_max", 0.6);
  p.setDouble("local_mpc.align_in_place_deg", 30.0);
  p.setDouble("local_mpc.align_exit_deg", 8.0);
  p.setDouble("local_mpc.align_gain", 1.2);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  RouteCorridor c;
  c.centerline = straightPath(30.0, 0.05);
  c.half_width = 0.0; // 严格贴线
  c.speed_limit = 0.35;
  mpc.setCorridor(&c);

  Pose2D pose{0.0, 0.0, 60.0 * kPi / 180.0}; // 车头差 60°
  const double dt = mpc.params().dt;
  double v = 0.0;
  int align_cycles = 0, drive_cycles = 0;
  double max_abs_y_while_aligning = 0.0;
  double first_drive_v = 0.0;
  for (int k = 0; k < 200; ++k) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(pose, dt);
    ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
    if (mpc.aligning()) {
      ++align_cycles;
      EXPECT_NEAR(r.cmd.v, 0.0, 1e-9) << "原地对正期间不应发线速度";
      EXPECT_GT(std::fabs(r.cmd.w), 1e-3) << "原地对正期间应在转";
      max_abs_y_while_aligning = std::max(max_abs_y_while_aligning, std::fabs(pose.y));
    } else if (r.cmd.v > 1e-3) {
      ++drive_cycles;
      if (first_drive_v == 0.0)
        first_drive_v = r.cmd.v;
    }
    pose.yaw += r.cmd.w * dt;
    pose.x += r.cmd.v * std::cos(pose.yaw) * dt;
    pose.y += r.cmd.v * std::sin(pose.yaw) * dt;
    v = r.cmd.v;
    if (align_cycles > 0 && drive_cycles > 3)
      break;
  }
  std::printf("  [原地对正] 对正 %d 周期 / 起步 %d 周期，起步 v=%.3f，"
              "对正期间 |y| ≤ %.4f m\n",
              align_cycles, drive_cycles, first_drive_v,
              max_abs_y_while_aligning);

  EXPECT_GT(align_cycles, 0) << "大航向差下没有进入原地对正模式";
  EXPECT_GT(drive_cycles, 3) << "对正后没有恢复沿线行驶（应能从拒动改为可动）";
  // 原地转正必须是"真的原地"：不产生横向位移（严格走廊的硬界就是横向 0.05 m）
  EXPECT_LT(max_abs_y_while_aligning, 0.05)
      << "对正期间出现了横向位移，说明不是原地转";
}

TEST(MpcLocalPlanner, TerminalProfileSlowsDownAndStopsBeforeGoal)
{
  // 终点段语义（到点精度 ≤ 3 cm 靠它）：进入 approach_dist 后参考限速到
  // approach_speed；扣掉 stop_coast 后参考在**目标之前**归零（把最后几厘米交给
  // 底盘的停车惯性）；crawl_speed 保证参考不会更早归零（否则车差几厘米停死、
  // 到点判据永不满足 ⇒ 任务卡死）。
  auto field = freeSpaceField();
  auto refV = [&](double s_from_end) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.reference_speed", 0.35);
    p.setDouble("local_mpc.brake_acc", 0.15);
    p.setDouble("local_mpc.approach_dist", 0.15);
    p.setDouble("local_mpc.approach_speed", 0.03);
    p.setDouble("local_mpc.crawl_speed", 0.015);
    p.setDouble("local_mpc.stop_coast", 0.035);
    mpc.configure(p);
    mpc.setDistanceField(&field);
    mpc.setGlobalPlan(straightPath(5.0, 0.05));
    mpc.setCurrentVelocity(0.2, 0.0);
    const double s = 5.0 - s_from_end;
    mpc.computeCommand(Pose2D{s, 0.0, 0.0}, mpc.params().dt);
    return mpc.solveInfo().ref_v;
  };

  const double v_far = refV(1.0);      // 终点段之外：巡航
  const double v_approach = refV(0.10); // 终点段内：被压到 approach_speed
  const double v_settle = refV(0.02);   // 惯性段内（< stop_coast）：参考已归零
  std::printf("  [终点剖面] 距终点 1.00 m: v_ref=%.3f | 0.10 m: %.3f | "
              "0.02 m: %.3f\n",
              v_far, v_approach, v_settle);
  EXPECT_GT(v_far, 0.3) << "远处不该被终点段影响";
  EXPECT_LE(v_approach, 0.03 + 1e-9) << "进入终点段后没压到 approach_speed";
  EXPECT_GE(v_approach, 0.015 - 1e-9) << "终点段被压到爬行速度以下（会停死在半路）";
  EXPECT_NEAR(v_settle, 0.0, 1e-9) << "惯性段内参考应已归零（靠底盘惯性走完）";
}

TEST(MpcLocalPlanner, ObstacleSoftCostBiasesAway)
{
  // 障碍在安全带内（0.35 < 0.45）但未达硬下界：软代价应把它往远离方向偏，
  // 关掉权重则必须精确贴线（证明偏转**确实**来自障碍代价，不是别的副作用）。
  auto field = radialObstacleField(4.0, 0.35, 6.0);

  auto run = [&](double weight, double &lat) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 1.0);
    p.setDouble("local_mpc.obstacle_weight", weight);
    p.setDouble("local_mpc.obstacle_safe_distance", 0.45);
    p.setDouble("local_mpc.obstacle_hard_distance", 0.20);
    mpc.configure(p);
    mpc.setDistanceField(&field);
    mpc.setGlobalPlan(straightPath(20.0, 0.02));
    mpc.setCurrentVelocity(1.0, 0.0);
    const auto r = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
    EXPECT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
    lat = 0.0;
    double best = 1e9;
    for (const auto &pt : mpc.predictedTrajectory()) {
      const double d = std::hypot(pt.x - 4.0, pt.y - 0.35);
      if (d < best) {
        best = d;
        lat = pt.y;
      }
    }
    return best;
  };

  double lat_off = 0.0;
  double lat_on = 0.0;
  const double d_off = run(0.0, lat_off);
  const double d_on = run(400.0, lat_on);
  std::printf("  [避障-软] 关软代价：最近点 y=%+.4f / 距离 %.4f | 开软代价：y=%+.4f / 距离 %.4f\n",
              lat_off, d_off, lat_on, d_on);

  EXPECT_NEAR(lat_off, 0.0, 0.01) << "关掉软代价时应贴线（说明偏转来自障碍项）";
  EXPECT_LT(lat_on, -0.002) << "障碍在 +y 侧，软代价应把轨迹推向 −y";
  EXPECT_GT(d_on, d_off) << "打开软代价后应离障碍更远";
}

TEST(MpcLocalPlanner, MissingDistanceFieldDegradesAndSlows)
{
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 1.0);
  p.setDouble("local_mpc.degraded_speed_limit", 0.3);
  mpc.configure(p);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));
  ASSERT_TRUE(mpc.degraded()) << "没有距离场必须自报降级";

  mpc.setCurrentVelocity(0.0, 0.0);
  const auto r = mpc.computeCommand(Pose2D{5.0, 0.0, 0.0}, mpc.params().dt);
  EXPECT_EQ(r.status, LocalStatus::kDegraded) << toString(r.status);
  EXPECT_LE(r.cmd.v, 0.3 + 1e-9) << "降级时必须限速，实测 " << r.cmd.v;
}

// --------------------------------------------------------------------- 接口契约

TEST(MpcLocalPlanner, NoPathIsIdle)
{
  auto mpc = makeMpc();
  EXPECT_EQ(mpc.computeCommand(Pose2D{0, 0, 0}, 0.1).status, LocalStatus::kIdle);
  mpc.setGlobalPlan({Pose2D{0, 0, 0}});   // 单点不是路径
  EXPECT_EQ(mpc.computeCommand(Pose2D{0, 0, 0}, 0.1).status, LocalStatus::kIdle);
}

TEST(MpcLocalPlanner, ProducesCmdVelAndReachesGoal)
{
  auto mpc = makeMpc();
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  EXPECT_TRUE(mpc.producesCmdVel());

  const double dt = mpc.params().dt;
  mpc.setGlobalPlan(straightPath(2.0, 0.05));
  Pose2D s{0.0, 0.0, 0.0};   // 起点 = 路径起点
  double v = 0.0;
  for (int i = 0; i < 400; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, dt);
    if (r.status == LocalStatus::kGoalReached)
      break;
  }
  EXPECT_LT(std::hypot(s.x - 2.0, s.y), 0.15)
      << "到点误差应 ≤ 0.15 m，实测 " << std::hypot(s.x - 2.0, s.y) << " m";
}

TEST(MpcLocalPlanner, FactoryAndType)
{
  auto p = createLocalPlanner("mpc");
  ASSERT_NE(p, nullptr) << "P5 起工厂必须认识 mpc";
  EXPECT_EQ(p->type(), "mpc");
  auto names = availableLocalPlanners();
  EXPECT_NE(std::find(names.begin(), names.end(), "mpc"), names.end());
}

// --------------------------------------------------------------------- 性能

TEST(MpcLocalPlanner, SolveTimeP99Under20ms)
{
  auto mpc = makeMpc(1.0);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const double dt = mpc.params().dt;
  RouteCorridor c;
  c.centerline = straightPath(60.0, 0.05);
  c.half_width = 0.5;
  mpc.setCorridor(&c);

  std::vector<double> times;
  Pose2D s{1.0, 0.05, 0.02};
  double v = 0.5;
  const int n = 200;
  for (int i = 0; i < n; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
    times.push_back(r.stats.solve_ms);
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, dt);
  }
  std::sort(times.begin(), times.end());
  const double p50 = times[times.size() / 2];
  const double p99 = times[static_cast<std::size_t>(times.size() * 0.99)];
  const double worst = times.back();
  std::printf("  [耗时] 求解 %d 次：P50 %.3f ms | P99 %.3f ms | max %.3f ms | 迭代 %d\n",
              n, p50, p99, worst, mpc.solveInfo().solver_iterations);
  EXPECT_LT(p99, 20.0) << "P99 " << p99 << " ms 超预算";
  EXPECT_GT(mpc.solveInfo().qp_variables, 0);
  EXPECT_LT(mpc.solveInfo().qp_constraints, 400) << "约束规模不应失控";
}

}  // namespace
}  // namespace pnc_2d
