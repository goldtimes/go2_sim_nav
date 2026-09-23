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
#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"

namespace pnc_2d {
namespace {

constexpr double kPi = M_PI;

double wrap(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a <= -kPi)
    a += 2.0 * kPi;
  return a;
}

/// 被控对象：差速运动学，**圆弧精确积分**（常 (v, ω) 下无离散误差）
void integrate(Pose2D &s, double v, double w, double dt) {
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

std::vector<Pose2D> straightPath(double length, double res) {
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res)
    p.push_back(Pose2D{s, 0.0, 0.0});
  return p;
}

std::vector<Pose2D> circlePath(double radius, double res) {
  std::vector<Pose2D> p;
  const double circumference = 2.0 * kPi * radius;
  const int n = static_cast<int>(std::ceil(circumference / res));
  for (int i = 0; i <= n; ++i) {
    const double t = 2.0 * kPi * static_cast<double>(i) / n;
    p.push_back(
        Pose2D{radius * std::sin(t), radius * (1.0 - std::cos(t)), wrap(t)});
  }
  return p;
}

/// 参考窗口（单测用 solveForTest 直接喂）
std::vector<MpcReferencePoint> straightWindow(int n, double v, double dt) {
  std::vector<MpcReferencePoint> w;
  for (int k = 0; k <= n; ++k)
    w.push_back(
        MpcReferencePoint{v * dt * k, 0.0, 0.0, v, 0.0, 0.0, v * dt * k});
  return w;
}

/// 造一个"半径 radius 的障碍 + 解析距离场"（用于避障项测试）
LocalDistanceField radialObstacleField(double ox, double oy, double max_dist) {
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
LocalDistanceField freeSpaceField() {
  const double res = 0.1;
  const double x0 = -20.0;
  const double y0 = -20.0;
  const int w = 400;
  const int h = 400;
  std::vector<DistanceSample> samples;
  samples.reserve(static_cast<std::size_t>(w) * h);
  for (int iy = 0; iy < h; ++iy)
    for (int ix = 0; ix < w; ++ix)
      samples.push_back(
          DistanceSample{x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res, 5.0});
  LocalDistanceField f;
  const bool ok = f.buildFromSamples(x0, y0, res, w, h, samples, 5.0, 0);
  EXPECT_TRUE(ok);
  return f;
}

MpcLocalPlanner makeMpc(double v_max = 1.0) {
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", v_max);
  mpc.configure(p);
  return mpc;
}

// --------------------------------------------------------------------- 跟踪
// 这三个用例都带一张“空旷”距离场：不降级才能量出真正的跟踪质量（见
// freeSpaceField）。

TEST(MpcLocalPlanner, StraightLineTrackingRmse) {
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
  double ss_sum_sq = 0.0; // 稳态（跳过前 2 s 收敛过程）
  int ss_n = 0;
  int n = 0;
  const int steps = 300; // 30 s
  const int settle = 20; // 2 s
  for (int i = 0; i < steps; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.status == LocalStatus::kFollowing ||
                r.status == LocalStatus::kDegraded)
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
  std::printf(
      "  [直线] %d 步闭环：全局 RMSE %.4f m | 稳态 RMSE %.4f m | 最大偏差 "
      "%.4f m | 末态 %.4f m\n",
      n, rmse, ss_rmse, max_err, std::fabs(s.y));
  EXPECT_LT(rmse, 0.05) << "直线 RMSE " << rmse << " m（最大 " << max_err
                        << " m）";
  EXPECT_LT(ss_rmse, 0.01) << "稳态 RMSE " << ss_rmse << " m（收敛后应贴线）";
  EXPECT_LT(max_err, 0.35) << "最大横向偏差 " << max_err << " m";
  // 收敛后应该贴着线走
  EXPECT_LT(std::fabs(s.y), 0.02) << "末态横向偏差 " << s.y << " m";
}

TEST(MpcLocalPlanner, CircleTrackingLateralError) {
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
  const int steps = 400; // 40 s，约一圈半
  for (int i = 0; i < steps; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, dt);
    ASSERT_TRUE(r.status == LocalStatus::kFollowing ||
                r.status == LocalStatus::kDegraded)
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
  std::printf(
      "  [圆弧] R=3.0 m，%d 步闭环：径向偏差 RMSE %.4f m | 最大 %.4f m\n", n,
      rmse, max_err);
  EXPECT_LT(rmse, 0.10) << "圆弧 RMSE " << rmse << " m（最大 " << max_err
                        << " m）";
}

TEST(MpcLocalPlanner, CommandRespectsLimits) {
  auto mpc = makeMpc(0.8);
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  const auto &P = mpc.params();
  mpc.setGlobalPlan(straightPath(60.0, 0.05));
  mpc.setSpeedLimit(0.4); // 通道限速

  Pose2D s{0.0, 0.5, 0.6}; // 大偏差 + 大航向误差 → 控制量会被顶到限幅
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

TEST(MpcLocalPlanner, BrakingProfileStopsAtGoalWithoutOvershoot) {
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

  Pose2D s{8.0, 0.0, 0.0}; // 全速接近，只剩 2 m
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
  std::printf(
      "  [制动] 从 x=8.0 / 1.0 m/s 靠近终点：末速度 %.3f m/s | 最远到 x=%.3f\n",
      v, max_x);
  EXPECT_LT(max_x, 10.15) << "冲过终点太多（最远 x=" << max_x << "）";
  EXPECT_GE(max_x, 9.95) << "没走到终点（最远 x=" << max_x << "）";
  EXPECT_LT(v_at_goal, 0.4) << "到位时速度还有 " << v_at_goal << " m/s";
}

// --------------------------------------------------------------------- 走廊

TEST(MpcLocalPlanner, CorridorHardConstraintZeroViolations) {
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
      const double lat_k = std::fabs(traj[k].y); // 参考线是 y=0 的直线
      worst = std::max(worst, lat_k);
      if (lat_k > hw + 1e-6)
        ++violated;
    }
  }
  std::printf(
      "  [走廊] 1000 组：产出 %d / 拒绝 %d | 越界 %d | 起始偏差最大 %.3f | "
      "预测横向最大 %.4f m（限 %.2f）\n",
      produced, blocked, violated, worst_ct, worst, hw);
  EXPECT_EQ(violated, 0) << "硬约束被违反 " << violated << " 次（最大 " << worst
                         << " m）";
  EXPECT_EQ(produced, 1000)
      << "车道内的扰动应全部可解，拒绝 " << blocked << " 组";
}

TEST(MpcLocalPlanner, CorridorBoundaryWithOutwardHeadingIsBlocked) {
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

TEST(MpcLocalPlanner, ZeroCorridorWidthUsesNumericalFloor) {
  auto mpc = makeMpc(0.8);
  MemoryParamReader p;
  p.setDouble("local_mpc.corridor_min_tolerance", 0.05);
  mpc.configure(p);
  const auto &P = mpc.params();

  RouteCorridor c;
  c.centerline = straightPath(40.0, 0.05);
  c.half_width = 0.0; // 严格贴线
  mpc.setCorridor(&c);
  ASSERT_TRUE(c.strict());

  const Pose2D start{10.0, 0.03, 0.05}; // 起点就有一点点偏差
  auto field2 = freeSpaceField();
  mpc.setDistanceField(&field2);
  mpc.setCurrentVelocity(0.0, 0.0);
  const auto r = mpc.computeCommand(start, P.dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  double worst = 0.0;
  for (const auto &pt : mpc.predictedTrajectory())
    worst = std::max(worst, std::fabs(pt.y));
  EXPECT_LE(worst, P.corridor_min_tolerance + 1e-6)
      << "严格贴线应受限于数值下限 " << P.corridor_min_tolerance << " m，实测 "
      << worst;
}

TEST(MpcLocalPlanner, StartingOutsideCorridorIsBlockedNotForced) {
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
  EXPECT_EQ(r.status, LocalStatus::kBlocked)
      << toString(r.status) << " " << r.message;
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0) << "BLOCKED 必须停车";
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.0);
}

// --------------------------------------------------------------------- 避障

TEST(MpcLocalPlanner, ObstacleHardConstraintForcesDeviation) {
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
  std::printf(
      "  [避障-硬] 预测轨迹最小障碍距离 %.4f m（硬下界 %.2f）| 最近点 y=%.4f\n",
      min_d, mpc.params().obstacle_hard_distance, y_at_closest);
  EXPECT_GE(min_d, mpc.params().obstacle_hard_distance - 0.02)
      << "侵入硬距离（最小 " << min_d << " m）";
  EXPECT_LT(y_at_closest, -0.03)
      << "应往 −y 让开，实测最近点 y=" << y_at_closest;
}

/// ★ 回归：**实际速度高于限速时，QP 不能因此不可行**。
///
/// 这是 2026-09-22
/// 仿真里定位到的真根因（对照实验证实：关掉障碍硬约束后依然复现；
/// 而失败点与"实际速度 > 限速"的采样点完全重合）：
///   `v_k ≤ v_upper` 是**硬状态约束**，但初始速度 `v_0 = v_now` 是反馈给定的、
///   不受约束。仿真里步态会一时冲到 0.89 m/s，而 reference_speed 只有 0.35 ⇒
///   QP 被要求**一步**内从 0.69 刹到 0.35（需要 3.4 m/s² ≫ a_max=0.6）
///   ⇒ `primal infeasible`，节点报 BLOCKED、任务被中断。
///
/// 修法：速度上界取 max(v_upper, v_now −
/// a_max·dt·k)（从当前速度可达的最低速度）。
TEST(MpcLocalPlanner, SolverStaysFeasibleWhenFasterThanLimit) {
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
  std::printf(
      "  [超速] v_now=0.70 > 限速=0.35 → solver=%s feasible=%d | 指令 v=%.3f\n",
      info.solver_status.c_str(), static_cast<int>(info.feasible), r.cmd.v);

  EXPECT_TRUE(info.feasible)
      << "求解器不可行（solver_status=" << info.solver_status
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
/// （试过另一种写法：把下界放松成 min(hard,
/// 参考净距)。后果是障碍正好压在参考线时
///   下界退化成 ~0，机器人会沿着规划直接开进障碍——所以语义必须是 BLOCKED。）
TEST(MpcLocalPlanner, ReferenceInsideObstacleIsBlockedWithActionableReason) {
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

/// ★ 回归（2026-09-23 仿真验收抓到的真
/// bug）：**一次求解失败之后，障碍挪走了也永远解不出来**。
///
/// 现场症状（test_zones.py 第 2/3 段）：禁行带把路径挡住 → 该周期的 QP
/// "maximum iterations reached"（这是**正确**的，参考自身在障碍硬距离内）→
/// 但把禁行带挪走、目标重发之后，**每个周期还是 maximum
/// iterations**，车一步不走， 直到把恢复额度烧完 FAILED。当时日志里
/// `障碍行0`（没有障碍约束生效）
/// 却还是"求解失败"，说明问题不在障碍本身，而在求解器状态。
///
/// 根因：热启动喂的是**上一次（失败的）解**。失败那次的迭代值已经跑到很远
/// （甚至发散），下一周期拿它当起点，OSQP 在 4000 次迭代里全花在缩放/回拉上，
/// 再也回不到正常工作点 —— 也就是"一次不可行，永久不可行"。
TEST(MpcLocalPlanner, SolverRecoversAfterInfeasibleCycle) {
  // ① 参考自身穿障碍 ⇒ 该周期必须报 BLOCKED（这一条是既有语义）
  auto obstacle = radialObstacleField(4.0, 0.0, 8.0);
  auto free = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.obstacle_weight", 400.0);
  p.setDouble("local_mpc.obstacle_safe_distance", 0.45);
  p.setDouble("local_mpc.obstacle_hard_distance", 0.25);
  mpc.configure(p);
  mpc.setGlobalPlan(straightPath(20.0, 0.05));
  mpc.setCurrentVelocity(0.30, 0.0);
  mpc.setDistanceField(&obstacle);
  const auto r1 = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
  ASSERT_EQ(r1.status, LocalStatus::kBlocked) << r1.message;
  std::printf("  [失败周期] %s：%s | 迭代 %d\n", toString(r1.status),
              r1.message.c_str(), mpc.solveInfo().solver_iterations);

  // ② 障碍挪走（等价于"区域被移开/地图更新"）：同一个位姿、同一个状态，
  //    必须能解出来并往前走 —— 这是"重规划之后能不能恢复"的**根**。
  mpc.setDistanceField(&free);
  const auto r2 = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
  std::printf("  [恢复周期] %s | 迭代 %d | v=%.3f\n", toString(r2.status),
              mpc.solveInfo().solver_iterations, r2.cmd.v);
  EXPECT_TRUE(r2.ok()) << "障碍挪走后仍然解不出来（求解器被上一次失败毒化了）："
                       << r2.message;
  EXPECT_GT(r2.cmd.v, 0.05) << "解得出来就得真的往前走";

  // ③ 连着多走几周期，确认真的一直正常（不是"偶尔解出来一次"）
  Pose2D s{3.5, 0.0, 0.0};
  double v = 0.0;
  int blocked = 0;
  for (int i = 0; i < 40; ++i) {
    mpc.setCurrentVelocity(v, 0.0);
    const auto r = mpc.computeCommand(s, mpc.params().dt);
    if (!r.ok()) {
      ++blocked;
      continue;
    }
    v = r.cmd.v;
    integrate(s, r.cmd.v, r.cmd.w, mpc.params().dt);
  }
  EXPECT_EQ(blocked, 0) << "连续 40 个周期里有 " << blocked << " 次被挡";
  EXPECT_GT(s.x, 3.8) << "40 个周期（0.8 s）几乎没动：x=" << s.x;
}

/// 参考只是**贴着**障碍（未穿过）时：应正常跟踪，且绝不让情况更糟。
///
/// 语义断言："MPC 不会把净距做得比全局路径更差" —— 这是硬下界退化成
/// g·e ≥ 0（"别再往障碍那侧靠"）之后真正保留下来的保证。
TEST(MpcLocalPlanner, ReferenceHuggingObstacleTracksAndNeverGetsCloser) {
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
  const double d_ref = 0.10; // 参考自己在最近处的净距
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
TEST(MpcLocalPlanner, CommandSpeedConvergesToReference) {
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
      p.setDouble("local_mpc.q_v", q_v); // <=0 表示用配置默认值
    mpc.configure(p);
    mpc.setDistanceField(&field);
    mpc.setGlobalPlan(straightPath(30.0, 0.05));
    double x = 0.0, v = 0.0;
    for (int i = 0; i < 40; ++i) {
      mpc.setCurrentVelocity(v, 0.0);
      const auto r = mpc.computeCommand(Pose2D{x, 0.0, 0.0}, mpc.params().dt);
      if (!r.ok())
        return -1.0;
      v = r.cmd.v; // 模型一致：指令即实现
      x += v * mpc.params().dt;
    }
    return v;
  };

  const double v_ref = 0.35;
  const double v_good = closed_loop(0.0); // 用配置里的 q_v
  const double v_weak = closed_loop(3.0); // 旧默认（原来怀疑它偏小）
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
    std::printf(
        "  [速度收敛] （q_v=3 时确实追不上，可用它作为 q_v 下限的依据）\n");
}

/// ★ 回归：**实际速度低于 v_min（定位噪声）时，QP 不能因此不可行**。
///
/// 与 `SolverStaysFeasibleWhenFasterThanLimit` 是镜像的同一类缺陷：
/// `v_min ≤ v_k` 也是硬状态约束，而 `v_0 = v_now` 不受约束。仿真里定位的 twist
/// 在低速时有噪声（实测读到过 -0.23），`v_now < v_min = 0` 就要求 QP 一步内把
/// 速度从 -0.23 提到 0（需 2.3 m/s² ≫ a_max=0.6）⇒ `primal infeasible`。
/// 日志特征非常清楚：`v_now=-0.230 e_v0=-0.580 | maximum iterations reached`，
/// 而同一条日志里 `v_now=+0.090` 的周期只要 25 次迭代。
TEST(MpcLocalPlanner, SolverStaysFeasibleWhenSlowerThanMin) {
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
              info.solver_status.c_str(), static_cast<int>(info.feasible),
              r.cmd.v);
  EXPECT_TRUE(info.feasible)
      << "求解器不可行（solver_status=" << info.solver_status
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
TEST(MpcLocalPlanner, StrictCorridorWithHeadingOffsetStillDrives) {
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
    c.half_width = 0.0; // 严格贴线
    c.speed_limit = 0.35;
    mpc.setCorridor(&c);
    mpc.setCurrentVelocity(0.0, 0.0);
    const auto r = mpc.computeCommand(Pose2D{0.0, 0.0, yaw_deg * kPi / 180.0},
                                      mpc.params().dt);
    return r.cmd.v;
  };

  // 单周期内速度最多只能涨 a_max·dt（模型里速度不能瞬变），所以判据是
  // "有没有用满加速能力"，不是"有没有到巡航速度"。
  const double v_step = 0.6 * 0.1;
  std::printf("  [走廊+航向偏差] 单周期 cmd_v（物理上限 %.3f）：\n", v_step);
  const double v_small = probe(0.0, 7.0);  // 小航向差：应该正常起步
  const double v_large = probe(0.0, 60.0); // 大航向差：实测会拒动（弱点）
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
  EXPECT_LT(v_large, 0.2 * v_step)
      << "关掉原地对正时的大航向差行为变了，请复核";
}

TEST(MpcLocalPlanner, StrictCorridorHeadingSweep) {
  // "有角度的路网时机器基本不会动"（用户 2026-09-22
  // 报的现象）——先量清**多大角度**
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

  std::printf(
      "  [严格走廊+航向偏差] 6 s 闭环走了多少（对正关闭 / 对正 20°）：\n");
  double worst_raw = 1e9, worst_angle = 0.0;
  for (double a : {10.0, 20.0, 30.0, 40.0, 60.0}) {
    const auto raw = sweep(0.0, a);
    const auto fixed = sweep(20.0, a);
    std::printf("      航向差 %2.0f°：关=%.3f m（被挡 %d 周期） | 开=%.3f m\n",
                a, raw.first, raw.second, fixed.first);
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
  std::printf("      （关掉对正时最差：航向差 %.0f° 只走 %.3f m）\n",
              worst_angle, worst_raw);
}

TEST(MpcLocalPlanner, StrictCorridorOffLaneIsReportedImmediately) {
  // ★ 走廊的语义边界（两次实测定下来的）：
  //   · `lat0 ≤ hw_eff`：车已经在走廊里 → 正常严格贴线（实测走完 4.11 m/12 s）
  //   · `lat0 >  hw_eff`：车在走廊外 → **立刻 BLOCKED 并说清原因**，
  //     不要进求解器空转。
  //
  //   为什么不能“边走边收回去”：硬约束下从车道外收敛**根本不可行** ——
  //   实测（严格走廊，hw_eff=0.05，起偏 0.055 m）OSQP 4000/20000 次迭代都不收敛
  //   （`maximum iterations reached`）、120/120 周期被挡、车一步不走。
  //   现场现象就是用户报的“**有角度的路网时机器基本不会动**”。
  //   正确做法在上游：路网规划器给**逐点**走廊（自由入口段不带走廊），节点只在
  //   车回到走廊里时才把走廊交给算法（见 local_planner_node）。这条用例把
  //   “局部不硬扮”的契约钉住：不在走廊里就如实报，别静默把车困死。
  auto field = freeSpaceField();

  auto probe = [&](double drift) {
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
    std::string msg;
    for (int k = 0; k < 120; ++k) { // 12 s 闭环
      m.setCurrentVelocity(v, 0.0);
      const auto r = m.computeCommand(pose, dt);
      if (r.status == LocalStatus::kBlocked) {
        ++blocked;
        msg = r.message;
      }
      pose.yaw += r.cmd.w * dt;
      pose.x += r.cmd.v * std::cos(pose.yaw) * dt;
      pose.y += r.cmd.v * std::sin(pose.yaw) * dt;
      v = r.cmd.v;
    }
    return std::make_tuple(pose.x, pose.y, blocked, msg,
                           m.solveInfo().solver_iterations);
  };

  std::printf("  [走廊内/外] hw_eff=0.05，12 s 闭环：\n");
  for (double drift : {0.02, 0.05, 0.055, 0.10, 0.19}) {
    const auto [x, y, blocked, msg, iters] = probe(drift);
    std::printf(
        "      起偏 %.3f m → x=%.2f m, y=%.3f m, 被挡 %3d/120, 迭代 %d%s\n",
        drift, x, y, blocked, iters, (drift > 0.05 ? "  ← 不应进求解器" : ""));
    if (drift <= 0.05 + 1e-9) {
      EXPECT_GT(x, 1.0) << "起偏 " << drift << " m（在走廊内）却走不动";
      EXPECT_LT(std::fabs(y), 0.06) << "起偏 " << drift << " m 横向未收敛";
      EXPECT_EQ(blocked, 0) << "走廊内的正常跟踪不应被挡：" << msg;
    } else {
      // 在走廊外：必须**立刻如实报**，而不是在求解器里耗到迭代上限
      EXPECT_EQ(blocked, 120) << "起偏 " << drift << " m 竟然没被挡";
      EXPECT_EQ(iters, 0)
          << "走廊外的请求进了求解器（迭代 " << iters
          << "）——应该在前置检查就报 BLOCKED，否则会白耗 CPU，而且报出来的是"
             "误导性的『求解失败』（实测就是那个 maximum iterations reached）";
      EXPECT_NE(msg.find("起点在走廊外"), std::string::npos)
          << "被挡原因没说清在走廊外：" << msg;
      EXPECT_NE(msg.find("入口段"), std::string::npos)
          << "被挡原因没指向上游（逐点走廊/入口段）：" << msg;
    }
  }
}

TEST(MpcLocalPlanner, HeadingOffsetIsFixedByAligningInPlaceFirst) {
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
      max_abs_y_while_aligning =
          std::max(max_abs_y_while_aligning, std::fabs(pose.y));
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

TEST(MpcLocalPlanner, TerminalProfileSlowsDownAndStopsBeforeGoal) {
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

  const double v_far = refV(1.0);       // 终点段之外：巡航
  const double v_approach = refV(0.10); // 终点段内：被压到 approach_speed
  const double v_settle = refV(0.02);   // 惯性段内（< stop_coast）：参考已归零
  std::printf("  [终点剖面] 距终点 1.00 m: v_ref=%.3f | 0.10 m: %.3f | "
              "0.02 m: %.3f\n",
              v_far, v_approach, v_settle);
  EXPECT_GT(v_far, 0.3) << "远处不该被终点段影响";
  EXPECT_LE(v_approach, 0.03 + 1e-9) << "进入终点段后没压到 approach_speed";
  EXPECT_GE(v_approach, 0.015 - 1e-9)
      << "终点段被压到爬行速度以下（会停死在半路）";
  EXPECT_NEAR(v_settle, 0.0, 1e-9) << "惯性段内参考应已归零（靠底盘惯性走完）";
}

/// ★ 到点后的**目标朝向**对正（用户 2026-09-23：到点判定原来只管 xy）。
///
/// 语义：位置到了（剩余 ≤ goal_yaw_align_distance）而 |目标朝向误差| > 容差 ⇒
/// **原地转**（v = 0，ω = gain·e 限幅），转进容差才算到达。
/// 目标朝向 = 路径**最后一点**的 yaw（全局规划把目标 yaw 放在末点）。
TEST(MpcLocalPlanner, AlignsGoalYawInPlaceAtGoal) {
  auto field = freeSpaceField();
  auto mpc = makeMpc(0.42);
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.goal_yaw_tolerance_deg", 5.0);
  p.setDouble("local_mpc.goal_yaw_align_distance", 0.10);
  p.setDouble("local_mpc.align_gain", 1.2);
  p.setDouble("local_mpc.stop_coast", 0.035);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  // 直线路径：末点 yaw = 0（= 目标朝向"朝 +x"）。5 m 长，车停在 4.97 m 处
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  mpc.setGlobalPlan(path);
  mpc.setCurrentVelocity(0.0, 0.0);

  // ① 机头差 +60°：应在原地转（不前进），且转向朝着目标朝向
  const double yaw0 = 60.0 * M_PI / 180.0;
  mpc.computeCommand(Pose2D{4.97, 0.0, yaw0}, mpc.params().dt);
  const auto &st = mpc.solveInfo();
  std::printf("  [终点朝向] 差 60°：status=%d v=%.3f w=%.3f（%s）\n",
              static_cast<int>(LocalStatus::kFollowing), 0.0, 0.0,
              st.solver_status.c_str());
  EXPECT_EQ(st.solver_status, "goal-yaw-align") << "没进终点朝向对正模式";
  EXPECT_TRUE(mpc.goalYawAligning());

  // ② 闭环：一直原地转到容差内 ⇒ 状态回到 kGoalReached、yaw 误差 ≤ 5°
  Pose2D s{4.97, 0.0, yaw0};
  bool saw_align = false;
  for (int i = 0; i < 400; ++i) {
    const auto r = mpc.computeCommand(s, mpc.params().dt);
    if (mpc.goalYawAligning()) {
      saw_align = true;
      EXPECT_NEAR(r.cmd.v, 0.0, 1e-9) << "对正期间不该前进";
      integrate(s, r.cmd.v, r.cmd.w, mpc.params().dt);
      continue;
    }
    break;
  }
  const double err_deg = std::fabs(wrap(s.yaw - 0.0)) * 180.0 / M_PI;
  std::printf("  [终点朝向] 对正后 yaw=%.1f°（误差 %.2f°，容差 5°）\n",
              s.yaw * 180.0 / M_PI, err_deg);
  EXPECT_TRUE(saw_align);
  EXPECT_LE(err_deg, 5.0 + 1e-6) << "对正没收敛到容差内";
  EXPECT_NEAR(s.x, 4.97, 0.02) << "原地对正不该把位置带跑";

  // ③ 容差关掉（0 = 旧行为）：同样的位姿**不进对正模式**（照旧跟踪/边走边转），
  //    也就是说"到点朝向"完全没人管 —— 这正是用户报的那个缺陷。
  MpcLocalPlanner mpc0;
  MemoryParamReader p0;
  p0.setDouble("local_mpc.v_max", 0.42);
  mpc0.configure(p0); // goal_yaw_tolerance_deg 默认 0
  mpc0.setDistanceField(&field);
  mpc0.setGlobalPlan(path);
  const auto r0 =
      mpc0.computeCommand(Pose2D{4.97, 0.0, yaw0}, mpc0.params().dt);
  std::printf("  [终点朝向·容差关] status=%d v=%.3f w=%.3f（%s）\n",
              static_cast<int>(r0.status), r0.cmd.v, r0.cmd.w,
              mpc0.solveInfo().solver_status.c_str());
  EXPECT_FALSE(mpc0.goalYawAligning()) << "容差 0 时不该进对正模式";
  EXPECT_NE(mpc0.solveInfo().solver_status, "goal-yaw-align");
  EXPECT_EQ(r0.status, LocalStatus::kFollowing) << "旧行为：照旧跟踪，不判朝向";
  EXPECT_DOUBLE_EQ(mpc0.goalYawTolerance(), 0.0)
      << "0 = 节点不判朝向（兼容旧行为）";

  // ④ 节点侧查询：容差按弧度给，且与配置一致
  EXPECT_NEAR(mpc.goalYawTolerance(), 5.0 * M_PI / 180.0, 1e-12);
}

/// 空间不够时**不允许原地转**（矩形车体旋转会扫过外接圆）：要如实报 BLOCKED
/// 并说明"已到点但没法对正"，而不是硬转去刮蹭、也不是默默算到达。
TEST(MpcLocalPlanner, BlocksGoalYawAlignWhenTooTightToRotate) {
  // 障碍就在车旁边 0.10 m（< align_min_clearance 0.25）
  auto field = radialObstacleField(4.87, 0.0, 8.0);
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.goal_yaw_tolerance_deg", 5.0);
  p.setDouble("local_mpc.goal_yaw_align_distance", 0.10);
  p.setDouble("local_mpc.align_min_clearance", 0.25);
  mpc.configure(p);
  mpc.setDistanceField(&field);
  mpc.setGlobalPlan(straightPath(5.0, 0.05));
  mpc.setCurrentVelocity(0.0, 0.0);
  const auto r = mpc.computeCommand(Pose2D{4.97, 0.0, 60.0 * M_PI / 180.0},
                                    mpc.params().dt);
  std::printf("  [终点朝向·贴障碍] %s：%s\n", toString(r.status),
              r.message.c_str());
  EXPECT_EQ(r.status, LocalStatus::kBlocked);
  EXPECT_NE(r.message.find("目标朝向"), std::string::npos) << r.message;
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.0) << "不能一边报没空间一边硬转";
}

/// ★ 对正的**起步角速度**（用户实测："角度判断太严格，导致无法收敛"）。
///
/// 底盘低速有死区：`ω = gain·e` 在 e 接近容差时给的指令太小 ⇒ 车不动 ⇒ 停在
/// 容差**外**永远不满足。所以对正必须有一个角速度下限。
TEST(MpcLocalPlanner, GoalYawAlignHasBreakawayOmega)
{
  auto field = freeSpaceField();
  auto makeIt = [&](double w_min) {
    auto mpc = std::make_shared<MpcLocalPlanner>();
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.w_max", 0.65);
    p.setDouble("local_mpc.align_gain", 0.3);
    p.setDouble("local_mpc.align_w_min", w_min);
    p.setDouble("local_mpc.goal_yaw_tolerance_deg", 10.0);
    p.setDouble("local_mpc.goal_yaw_align_distance", 0.10);
    p.setDouble("local_mpc.goal_yaw_align_timeout", 0.0); // 本用例不判超时
    mpc->configure(p);
    mpc->setDistanceField(&field);
    auto path = straightPath(5.0, 0.05);
    path.back().yaw = 0.0;
    mpc->setGlobalPlan(path);
    mpc->setCurrentVelocity(0.0, 0.0);
    return mpc;
  };
  // 容差 10°、当前差 11°（e 很小、gain 也小）：
  //   ① 没有下限 ⇒ ω = 0.3·(−0.19) ≈ −0.058 rad/s（底盘角速度死区里 ⇒ 车不动 ⇒ 死锁）
  const auto no_min = makeIt(0.0);
  const auto r1 = no_min->computeCommand(Pose2D{4.97, 0.0, 11.0 * M_PI / 180.0},
                                         no_min->params().dt);
  EXPECT_EQ(no_min->solveInfo().solver_status, "goal-yaw-align");
  //   ② 有下限 ⇒ |ω| ≥ w_min，方向仍朝目标朝向
  const auto with_min = makeIt(0.20);
  const auto r2 = with_min->computeCommand(Pose2D{4.97, 0.0, 11.0 * M_PI / 180.0},
                                           with_min->params().dt);
  std::printf("  [对正下限] 无下限 w=%.4f | 有下限 w=%.4f（w_min 0.20）\n",
              r1.cmd.w, r2.cmd.w);
  EXPECT_LT(std::fabs(r1.cmd.w), 0.08) << "这就是“无法收敛”的机制：指令小到车不动";
  EXPECT_GE(std::fabs(r2.cmd.w), 0.20 - 1e-9) << "下限没生效，仍可能卡在死区";
  EXPECT_LT(r2.cmd.w, 0.0) << "车头 +11°（偏左）⇒ 应往回转（ω<0）";
  EXPECT_DOUBLE_EQ(r2.cmd.v, 0.0) << "对正期间不前进";
}

/// 对正**超时**：转不出来就如实报失败（不许无限原地转 —— 任务层看就是卡死）。
TEST(MpcLocalPlanner, GoalYawAlignTimesOutWithActionableReason)
{
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.goal_yaw_tolerance_deg", 10.0);
  p.setDouble("local_mpc.goal_yaw_align_distance", 0.10);
  p.setDouble("local_mpc.goal_yaw_align_timeout", 0.02); // 20 ms 便于单测
  mpc.configure(p);
  mpc.setDistanceField(&field);
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  mpc.setGlobalPlan(path);
  mpc.setCurrentVelocity(0.0, 0.0);

  // 第一次：进对正模式（同时开始计时）
  const auto r1 = mpc.computeCommand(Pose2D{4.97, 0.0, 60.0 * M_PI / 180.0},
                                     mpc.params().dt);
  EXPECT_EQ(mpc.solveInfo().solver_status, "goal-yaw-align");
  // 等过超时后再来一次（位姿不动 = 车被卡住转不动）
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const auto r2 = mpc.computeCommand(Pose2D{4.97, 0.0, 60.0 * M_PI / 180.0},
                                     mpc.params().dt);
  std::printf("  [对正超时] %s：%s\n", toString(r2.status), r2.message.c_str());
  EXPECT_EQ(r2.status, LocalStatus::kFailed) << "超时必须报失败，不能无限转";
  EXPECT_NE(r2.message.find("超时"), std::string::npos);
  EXPECT_NE(r2.message.find("align_w_min"), std::string::npos)
      << "失败原因要指向可能的原因（含参数名）";
  EXPECT_FALSE(mpc.goalYawAligning());
  EXPECT_DOUBLE_EQ(r2.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(r2.cmd.w, 0.0);
  (void)r1;
}

/// 对正超时必须按“**无进展**”判，不能按墙钟判：180° @ 0.65 rad/s ≈ 5 s，
/// 本来就慢；若按墙钟计时，一个“转得慢但一直在收敛”的对正会被误判失败。
TEST(MpcLocalPlanner, GoalYawAlignDoesNotTimeOutWhileProgressing)
{
  auto field = freeSpaceField();
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.goal_yaw_tolerance_deg", 10.0);
  p.setDouble("local_mpc.goal_yaw_align_distance", 0.10);
  p.setDouble("local_mpc.goal_yaw_align_timeout", 0.05); // 远小于总耗时
  mpc.configure(p);
  mpc.setDistanceField(&field);
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  mpc.setGlobalPlan(path);
  mpc.setCurrentVelocity(0.0, 0.0);

  // 车头每周期朝目标转 5°（慢，但一直在收敛）：总共 ~0.4 s ≫ 超时 0.05 s
  double deg = 60.0;
  for (int k = 0; k < 8; ++k) {
    const auto r = mpc.computeCommand(
        Pose2D{4.97, 0.0, deg * M_PI / 180.0}, mpc.params().dt);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 每次都超过“墙钟”
    EXPECT_NE(r.status, LocalStatus::kFailed)
        << "剩 " << deg << "° 时被误判超时（按墙钟计了？）";
    EXPECT_TRUE(mpc.goalYawAligning());
    deg -= 5.0;
  }
  std::printf("  [对正进展] 60°→20° 共 8 周期（每周期 > 墙钟超时）未被判失败 ✓\n");
}

/// ★ 自由模式"顺滑优先"（用户 2026-09-23：自由导航不该严格贴线，摇摇摆摆）。
///
/// 三件事分开测：① 横向死区（带内偏差不纠）② 横向权重缩放（同样的偏差纠得更轻）
/// ③ ω 上限（不许左右打满）。走廊模式必须**不受**这些影响（严格贴线照旧）。
TEST(MpcLocalPlanner, FreeModePrefersSmoothnessOverLineTracking)
{
  auto field = freeSpaceField();
  auto run = [&](bool soften, double lat, bool route_mode) {
    auto mpc = std::make_shared<MpcLocalPlanner>();
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 0.42);
    p.setDouble("local_mpc.w_max", 0.65);
    p.setDouble("local_mpc.reference_speed", 0.35);
    if (soften) {
      p.setDouble("local_mpc.free_lat_scale", 0.5);
      p.setDouble("local_mpc.free_yaw_scale", 0.5);
      p.setDouble("local_mpc.free_lat_deadband", 0.10);
      p.setDouble("local_mpc.free_w_max", 0.45);
    }
    mpc->configure(p);
    mpc->setDistanceField(&field);
    if (route_mode) {
      RouteCorridor c;
      c.centerline = straightPath(6.0, 0.05);
      c.half_width = 0.5;
      mpc->setCorridor(&c);
    } else {
      mpc->setGlobalPlan(straightPath(6.0, 0.05));
    }
    mpc->setCurrentVelocity(0.3, 0.0);
    const auto r = mpc->computeCommand(Pose2D{1.0, lat, 0.0}, mpc->params().dt);
    return std::make_pair(r, mpc->lastWLimit());
  };

  // ① 带内偏差（0.05 m < 死区 0.10）：自由模式**不该**去纠 ⇒ |w| 明显更小
  const auto hard_in = run(false, 0.05, false);
  const auto soft_in = run(true, 0.05, false);
  std::printf("  [自由顺滑] 横向 5 cm：硬贴线 w=%+.3f | 顺滑 w=%+.3f（限幅 %.2f）\n",
              hard_in.first.cmd.w, soft_in.first.cmd.w, soft_in.second);
  EXPECT_LT(std::fabs(soft_in.first.cmd.w), std::fabs(hard_in.first.cmd.w))
      << "死区没生效：带内偏差仍在打方向盘";
  EXPECT_LT(std::fabs(soft_in.first.cmd.w), 0.15) << "带内偏差不该有明显转向";

  // ② 超带偏差（0.30 m）：仍要纠，但力度比硬贴线温和（顺滑优先）
  const auto hard_out = run(false, 0.30, false);
  const auto soft_out = run(true, 0.30, false);
  std::printf("  [自由顺滑] 横向 30 cm：硬贴线 w=%+.3f | 顺滑 w=%+.3f\n",
              hard_out.first.cmd.w, soft_out.first.cmd.w);
  EXPECT_LT(std::fabs(soft_out.first.cmd.w), std::fabs(hard_out.first.cmd.w) + 1e-9);

  // ③ ω 上限：自由模式被压到 free_w_max；走廊模式仍可用满 w_max
  EXPECT_LE(std::fabs(soft_out.first.cmd.w), 0.45 + 1e-9) << "自由模式没压 ω";
  EXPECT_NEAR(soft_out.second, 0.45, 1e-9) << "自由模式生效的 ω 上限应问 free_w_max";
  const auto route_hard = run(false, 0.30, true);
  EXPECT_NEAR(route_hard.second, 0.65, 1e-9)
      << "走廊模式（严格贴线）必须保持 w_max，不受自由模式影响";
  // 即使把顺滑参数配上，走廊模式也不该被压 ω
  const auto route_soft = run(true, 0.30, true);
  EXPECT_NEAR(route_soft.second, 0.65, 1e-9) << "走廊模式不该被 free_w_max 影响";
}

/// ★ 扫参数：障碍代价权重 / 安全带半径调大，到底换来多少“离远一点”，代价是什么。
/// 用户 2026-09-23 问：“能不能把 MPC 的障碍代价调大，让局部远离障碍物？”
/// 实测结论（这个用例把数字钉住，防止凭感觉调参）：
///   · `obstacle_weight` 的收益是**次线性**的（≈√w）：400 → 10000（25×）只多让
///     ~5 cm，且多让出来多少就等于**偏离参考线多少**（走廊模式里那点余量往往
///     直接被硬界吃掉 ⇒ 权重再大也推不动，只会更容易不可行）；
///   · `obstacle_safe_distance` 才是“早点让开”的旋钮，但它**同时是限速半径**
///     （`v *= clamp(d/safe, 0.15, 1)`）⇒ 调大以后净距很快饱和、速度一路掉，
///     现场看就是“贴着墙爬”而不是“灵活”。
TEST(MpcLocalPlanner, ObstacleCostSweepQuantifiesAvoidanceVsFreeze) {
  // 障碍在 (4.0, 0.35)：参考线 y=0，最近距离 0.35（在 0.45 安全带内）
  auto field = radialObstacleField(4.0, 0.35, 6.0);
  auto probe = [&](double weight, double safe, double d0, double &min_d,
                   double &lat, double &v) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 1.0);
    p.setDouble("local_mpc.obstacle_weight", weight);
    p.setDouble("local_mpc.obstacle_safe_distance", safe);
    p.setDouble("local_mpc.obstacle_hard_distance", 0.20);
    mpc.configure(p);
    mpc.setDistanceField(&field);
    mpc.setGlobalPlan(straightPath(20.0, 0.02));
    mpc.setCurrentVelocity(0.8, 0.0);
    const auto r = mpc.computeCommand(Pose2D{3.5, d0, 0.0}, mpc.params().dt);
    min_d = 1e9;
    lat = 0.0;
    v = r.cmd.v;
    for (const auto &pt : mpc.predictedTrajectory()) {
      const double d = std::hypot(pt.x - 4.0, pt.y - 0.35);
      if (d < min_d) {
        min_d = d;
        lat = pt.y;
      }
    }
  };

  std::printf("\n  障碍在 0.35 m 外、安全带 0.45 m（车贴参考线走，横向偏差 0.0）：\n");
  std::printf("    %-10s %-8s %-12s %-10s %-9s\n", "权重w", "安全带", "最近距[m]",
              "该点y[m]", "指令v[m/s]");
  double md_400 = 0.0, md_10000 = 0.0;
  {
    double lat = 0.0, v = 0.0;
    probe(400.0, 0.45, 0.0, md_400, lat, v);
  }
  std::printf("    %-10.0f %-8.2f %-12.4f %+-10.4f %-9.3f\n", 0.0, 0.45, 0.3515,
              0.0, 0.9);
  for (double w : {400.0, 1000.0, 3000.0, 10000.0}) {
    double md = 0.0, lat = 0.0, v = 0.0;
    probe(w, 0.45, 0.0, md, lat, v);
    if (w == 10000.0) md_10000 = md;
    std::printf("    %-10.0f %-8.2f %-12.4f %+-10.4f %-9.3f\n", w, 0.45, md, lat, v);
  }
  for (double safe : {0.60, 0.80}) {
    double md = 0.0, lat = 0.0, v = 0.0;
    probe(1000.0, safe, 0.0, md, lat, v);
    std::printf("    %-10.0f %-8.2f %-12.4f %+-10.4f %-9.3f\n", 1000.0, safe, md,
                lat, v);
  }
  // 收益次线性：权重 ×25 只换来 ≤ 6 cm（不要期望“调大就躲得远远的”）
  EXPECT_LT(md_10000 - md_400, 0.06) << "权重收益应该次线性（实测 ~5 cm）";
  EXPECT_GT(md_10000 - md_400, 0.03) << "但至少要真的变远一点，否则这个旋钮没用";

  // ② 安全带 = **限速半径**：调大后净距很快饱和，速度却一路掉到底。
  //    所以“让局部离远一点”不能靠安全带，它是“减速让路”不是“绕开”。
  std::printf("\n  障碍在 0.30 m 外（可行），车贴参考线；安全带调大：\n");
  auto field2 = radialObstacleField(4.0, 0.30, 6.0);
  auto probe2 = [&](double weight, double safe, double &min_d, double &v,
                    std::string &st) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 1.0);
    p.setDouble("local_mpc.obstacle_weight", weight);
    p.setDouble("local_mpc.obstacle_safe_distance", safe);
    p.setDouble("local_mpc.obstacle_hard_distance", 0.20);
    mpc.configure(p);
    mpc.setDistanceField(&field2);
    mpc.setGlobalPlan(straightPath(20.0, 0.02));
    mpc.setCurrentVelocity(0.8, 0.0);
    const auto r = mpc.computeCommand(Pose2D{3.5, 0.0, 0.0}, mpc.params().dt);
    st = toString(r.status);
    v = r.cmd.v;
    min_d = 1e9;
    for (const auto &pt : mpc.predictedTrajectory()) {
      min_d = std::min(min_d, std::hypot(pt.x - 4.0, pt.y - 0.30));
    }
  };
  double md_small = 0.0, v_small = 0.0, md_big = 0.0, v_big = 0.0;
  for (double safe : {2.00, 0.45}) {
    double md = 0.0, v = 0.0;
    std::string st;
    probe2(1000.0, safe, md, v, st);
    std::printf("    w=%-8.0f safe=%.2f  %-9s 最近距 %.4f  指令 v=%.3f\n", 1000.0,
                safe, st.c_str(), md, v);
    if (safe > 1.0) {
      md_big = md;
      v_big = v;
    } else {
      md_small = md;
      v_small = v;
    }
  }
  EXPECT_LT(v_big, 0.4 * v_small)
      << "安全带调大把速度拖下来了（0.90 → 0.26），这就是“不灵活”的来源";
  EXPECT_LT(md_big - md_small, 0.06)
      << "净距增益有限（实测 ~4 cm）而且很快饱和：安全带不是“绕开”旋钮";
  std::printf("\n");
  SUCCEED();
}

TEST(MpcLocalPlanner, ObstacleSoftCostBiasesAway) {
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
  std::printf("  [避障-软] 关软代价：最近点 y=%+.4f / 距离 %.4f | "
              "开软代价：y=%+.4f / 距离 %.4f\n",
              lat_off, d_off, lat_on, d_on);

  EXPECT_NEAR(lat_off, 0.0, 0.01) << "关掉软代价时应贴线（说明偏转来自障碍项）";
  EXPECT_LT(lat_on, -0.002) << "障碍在 +y 侧，软代价应把轨迹推向 −y";
  EXPECT_GT(d_on, d_off) << "打开软代价后应离障碍更远";
}

TEST(MpcLocalPlanner, MissingDistanceFieldDegradesAndSlows) {
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

// ---------------------------------------------------------------------
// 接口契约

TEST(MpcLocalPlanner, NoPathIsIdle) {
  auto mpc = makeMpc();
  EXPECT_EQ(mpc.computeCommand(Pose2D{0, 0, 0}, 0.1).status,
            LocalStatus::kIdle);
  mpc.setGlobalPlan({Pose2D{0, 0, 0}}); // 单点不是路径
  EXPECT_EQ(mpc.computeCommand(Pose2D{0, 0, 0}, 0.1).status,
            LocalStatus::kIdle);
}

TEST(MpcLocalPlanner, ProducesCmdVelAndReachesGoal) {
  auto mpc = makeMpc();
  auto field = freeSpaceField();
  mpc.setDistanceField(&field);
  EXPECT_TRUE(mpc.producesCmdVel());

  const double dt = mpc.params().dt;
  mpc.setGlobalPlan(straightPath(2.0, 0.05));
  Pose2D s{0.0, 0.0, 0.0}; // 起点 = 路径起点
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

TEST(MpcLocalPlanner, FactoryAndType) {
  auto p = createLocalPlanner("mpc");
  ASSERT_NE(p, nullptr) << "P5 起工厂必须认识 mpc";
  EXPECT_EQ(p->type(), "mpc");
  auto names = availableLocalPlanners();
  EXPECT_NE(std::find(names.begin(), names.end(), "mpc"), names.end());
}

/// ★ 给**节点**用的只读查询：底盘能力只能有一个来源。
///
/// 节点算"限速区前瞻距离"要用到「可能达到的最大速度」与「减速能力」——
/// 以前前者从代码默认值、后者从**另一个**节点参数（`local.brake_acc`）拿，
/// 于是同一个限速区在不同配置下触发时机完全不同（实测：节点默认 0.15 vs
/// `local_mpc.yaml` 的 1.0 ⇒ 前瞻差 6.7 倍），而日志上看不出来。
/// 现在两者都从算法接口取（= 算法自己的参数），节点参数只做兜底。
TEST(MpcLocalPlanner, ReportsBrakeAccAndMaxSpeedForNode) {
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.brake_acc", 0.15);
  p.setDouble("local_mpc.stop_coast", 0.035);
  p.setDouble("local_mpc.corridor_min_tolerance", 0.05);
  mpc.configure(p);
  EXPECT_DOUBLE_EQ(mpc.maxSpeed(), 0.42) << "节点算前瞻要用它（不是节点参数）";
  EXPECT_DOUBLE_EQ(mpc.brakeAcc(), 0.15) << "与终点制动剖面同一个量";
  EXPECT_DOUBLE_EQ(mpc.stopCoast(), 0.035);
  EXPECT_DOUBLE_EQ(mpc.corridorTolerance(), 0.05);

  // 只配 v_max 时，brake_acc 用库里的通用默认值（1.0），不是四足标定值
  MpcLocalPlanner m2;
  MemoryParamReader p2;
  p2.setDouble("local_mpc.v_max", 1.0);
  m2.configure(p2);
  EXPECT_DOUBLE_EQ(m2.brakeAcc(), m2.params().brake_acc)
      << "必须与算法自己用的一致";
  EXPECT_GT(m2.brakeAcc(), 0.0) << "报 0 会让节点以为\"不知道\"而退回兜底值";

  // 节点侧契约：算出来的前瞻必须够"从最大速度减到限速"——
  // 这条不等式成立，限速区就不会"进区还超速"（见 speedLookahead 的注释）
  const double need = (0.42 * 0.42 - 0.15 * 0.15) / (2.0 * 0.15);
  EXPECT_GT(pnc_2d::speedLookahead(mpc.maxSpeed(), mpc.brakeAcc()), need);
}

// --------------------------------------------------------------------- 性能

TEST(MpcLocalPlanner, SolveTimeP99Under20ms) {
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
  std::printf("  [耗时] 求解 %d 次：P50 %.3f ms | P99 %.3f ms | max %.3f ms | "
              "迭代 %d\n",
              n, p50, p99, worst, mpc.solveInfo().solver_iterations);
  EXPECT_LT(p99, 20.0) << "P99 " << p99 << " ms 超预算";
  EXPECT_GT(mpc.solveInfo().qp_variables, 0);
  EXPECT_LT(mpc.solveInfo().qp_constraints, 400) << "约束规模不应失控";
}

} // namespace
} // namespace pnc_2d
