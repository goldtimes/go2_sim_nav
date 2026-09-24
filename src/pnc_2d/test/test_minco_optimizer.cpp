// M2（P6 轨迹层）单元测试：MINCO 后端（`(θ,s)` + SDF 平滑 + 运动学 + 终点）。
// 不依赖 ROS —— 全部离线、毫秒级。
//
// 测什么（以及为什么不测别的）：
//   · **几何**：平滑度（曲率）下降、净距不退化（柱子场景要真的变远）；
//   · **运动学**：采样轨迹的 max|v|/|ω|/|a|/|α| 必须真的在界内 —— 这是"轨迹可行"
//     唯一诚实的定义（不是"算出个系数看起来合理"）；
//   · **终点**：XY 是 (θ,s) 的**积分派生量** ⇒ 终点误差必须单独量、且必须在容差内；
//   · **剖面自洽**：相邻采样点的 Δs/Δt 与 Δyaw/Δt 要等于剖面给的 v/ω ——
//     这条能把"采样点和剖面各说各话"这类错误立刻抓出来；
//   · **退化输入**：单点/重合/无地图/无距离场都不能崩，且要给出结论。
//
// 口径（与 plan §7 一致）：
//   · 净距统计**排除首末点**（首点=车自己、末点=目标，会掩盖差异）；
//   · 净距用**轮廓**净距（不是点净距）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/trajectory_optimizer.hpp"
#include "pnc_2d/traj/minco_optimizer.hpp"

namespace pnc_2d {
namespace {

constexpr int kHard = 80;
constexpr double kRes = 0.10;

struct Fixture {
  std::shared_ptr<CostMap2D> map;
  ClearanceField cf;
  FootprintCollisionChecker ck;
};

/// 建图 + 距离场 + 轮廓判定器（判定器持有裸指针 ⇒ 必须由 Fixture 保命）
/// ★ `margin` 默认 0.05，但 0.05 恰好等于一格（res = 0.10）的一半 —— 见
///   `FootprintCollisionChecker::poseInCollision` 的"贴格心"注释：判定时位姿会被
///   吸附到格心，误差最大 ±半格 = ±余量 ⇒ **此时"余量带"在格心模型下是空集**。
///   要构造"真实轮廓放得下、含余量不通"的场景，余量必须 > 一格。
std::shared_ptr<Fixture> makeFixture(int w, int h, std::vector<int8_t> data,
                                     double margin = 0.05)
{
  auto f = std::make_shared<Fixture>();
  f->map = std::make_shared<CostMap2D>();
  if (!f->map->set(w, h, kRes, 0.0, 0.0, 0.0, std::move(data), "map")) return nullptr;
  if (!f->cf.build(*f->map, kHard, false)) return nullptr;
  FootprintParams fp;
  fp.enable = true;
  fp.length = 0.70;
  fp.width = 0.40;
  fp.safe_margin = margin;
  f->ck.configure(fp, kHard, false);
  f->ck.setMap(f->map.get());
  f->ck.setClearanceField(&f->cf);
  return f;
}

std::shared_ptr<Fixture> openFixture(int w = 120, int h = 80)
{
  return makeFixture(w, h, std::vector<int8_t>(static_cast<std::size_t>(w) * h, 0));
}

/// 0.4×0.4 m 的柱子（格 (x0..x0+3, y0..y0+3)）
std::shared_ptr<Fixture> pillarFixture(int w, int h, int x0, int y0, double margin = 0.05)
{
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  for (int x = x0; x < x0 + 4; ++x) {
    for (int y = y0; y < y0 + 4; ++y) {
      data[static_cast<std::size_t>(y) * w + x] = 100;
    }
  }
  return makeFixture(w, h, std::move(data), margin);
}

/// 一整行致命格（第 y0 行）：用来造"贴着墙走"这种**端点净距受限**的场景
std::shared_ptr<Fixture> wallRowFixture(int w, int h, int y0, double margin = 0.05)
{
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  for (int x = 0; x < w; ++x) {
    data[static_cast<std::size_t>(y0) * w + x] = 100;
  }
  return makeFixture(w, h, std::move(data), margin);
}

MemoryParamReader baseParams()
{
  MemoryParamReader p;
  p.setDouble("traj.resample_ds", 0.15);
  p.setDouble("traj.smooth_margin", 0.15);
  p.setDouble("traj.v_max", 0.90);
  p.setDouble("traj.a_max", 0.50);
  p.setDouble("traj.w_max", 0.65);
  p.setDouble("traj.alpha_max", 2.50);
  p.setDouble("traj.v_ref", 0.60);
  return p;
}

Pose2D pose(double x, double y, bool has_yaw = false, double yaw = 0.0)
{
  Pose2D p;
  p.x = x;
  p.y = y;
  p.yaw = yaw;
  p.has_yaw = has_yaw;
  return p;
}

/// 直线折线（只有首末两点：最考验重采样）
std::vector<Pose2D> straightPath(double x0, double y0, double x1, double y1)
{
  return {pose(x0, y0), pose(x1, y1)};
}

MincoOptimizer makeOptimizer(const std::shared_ptr<Fixture> & f,
                             const MemoryParamReader & prm)
{
  MincoOptimizer opt;
  EXPECT_TRUE(opt.configure(prm));
  opt.setMap(f->map);
  opt.setClearanceField(&f->cf);
  opt.setCollisionChecker(&f->ck);
  return opt;
}

/// 找**余量带的上沿**：含余量判定不通、但真实轮廓放得下 —— 取带内**最高**的那个 y。
/// 为什么要上沿：下沿的"真实净距"≈ 0（车身稍一低头就真撞），那种用例测的是
/// "终检灵不灵"（另有专门用例），而这里要测的是"**会不会过度拒绝**"，所以要在
/// 带内取净距最富裕的位置。
double topOfBandY(const Fixture & f, double x, double y_from, double y_to, double step)
{
  for (double y = y_to; y > y_from; y -= step) {
    if (!f.ck.poseInCollisionNoMargin(x, y, 0.0) && f.ck.poseInCollision(x, y, 0.0)) {
      return y;
    }
  }
  return 0.0;
}

/// 沿折线量**最小轮廓净距**（排除首端 exclude_s 弧长）；同时给出采样点数
double minClearanceAlong(const FootprintCollisionChecker & ck,
                         const std::vector<Pose2D> & poly, double exclude_s)
{
  double best = 1e9;
  double s = 0.0;
  for (std::size_t i = 1; i < poly.size(); ++i) {
    const double dx = poly[i].x - poly[i - 1].x;
    const double dy = poly[i].y - poly[i - 1].y;
    const double seg = std::hypot(dx, dy);
    if (seg < 1e-9) continue;
    const double yaw = std::atan2(dy, dx);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / 0.10)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      const double px = poly[i - 1].x + dx * t;
      const double py = poly[i - 1].y + dy * t;
      const double sc = s + seg * t;
      if (sc < exclude_s) continue;
      best = std::min(best, ck.signedClearanceAt(px, py, yaw));
    }
    s += seg;
  }
  return best;
}

/// 沿轨迹采样量最小轮廓净距（同样的口径）
double minClearanceAlong(const FootprintCollisionChecker & ck,
                         const std::vector<TrajSample> & samples, double exclude_s)
{
  double best = 1e9;
  for (const TrajSample & t : samples) {
    if (t.s < exclude_s) continue;
    best = std::min(best, ck.signedClearanceAt(t.x, t.y, t.yaw));
  }
  return best;
}

void expectKinematicsWithinLimits(const TrajOptResult & r, const MemoryParamReader & prm,
                                  double tol_ratio = 1.02)
{
  const double vmax = prm.getDouble("traj.v_max", 0.0);
  const double amax = prm.getDouble("traj.a_max", 0.0);
  const double wmax = prm.getDouble("traj.w_max", 0.0);
  const double almax = prm.getDouble("traj.alpha_max", 0.0);
  EXPECT_LE(r.stats.max_v, vmax * tol_ratio) << "max_v=" << r.stats.max_v;
  EXPECT_LE(r.stats.max_a, amax * tol_ratio) << "max_a=" << r.stats.max_a;
  EXPECT_LE(r.stats.max_omega, wmax * tol_ratio) << "max_omega=" << r.stats.max_omega;
  EXPECT_LE(r.stats.max_alpha, almax * tol_ratio) << "max_alpha=" << r.stats.max_alpha;
}

// ------------------------------------------------------------------ 直线基线

TEST(MincoOptimizer, StraightPathIsSmoothKinematicAndReachesGoal)
{
  auto f = openFixture();
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 4.0, 9.0, 4.0);
  req.start = pose(1.0, 4.0, true, 0.0);
  req.goal = pose(9.0, 4.0, true, 0.0);
  req.use_goal_yaw = true;

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  ASSERT_GE(r.samples.size(), 10u);

  expectKinematicsWithinLimits(r, prm);
  EXPECT_LE(r.stats.terminal_error, 0.01 + 1e-6) << "终点残差 " << r.stats.terminal_error;
  EXPECT_LE(r.stats.terminal_error_yaw * 180.0 / M_PI, 1.0);
  EXPECT_LT(r.stats.max_curvature, 0.05);        // 直线：曲率应当 ~0
  EXPECT_GT(r.stats.max_v, 0.3);                 // 别"算出来了但车不动"
  EXPECT_LT(r.stats.path_shift_max, 0.05);       // 直线不该被平滑改动
  EXPECT_TRUE(r.samples.front().t == 0.0);
  EXPECT_NEAR(r.samples.back().t, r.stats.duration, 1e-9);
  EXPECT_LT(r.stats.solve_ms, 50.0);

  std::printf("[M2] 直线 8 m：%zu 采样 / %.2f s / 净距 %s / max_v %.2f / max|κ| %.4f / "
              "终点 %.4f m / %.1f ms\n",
              r.samples.size(), r.stats.duration,
              r.stats.clearance_valid ? std::to_string(r.stats.min_clearance).c_str() : "n/a",
              r.stats.max_v, r.stats.max_curvature, r.stats.terminal_error, r.stats.solve_ms);
}

TEST(MincoOptimizer, ProfileIsSelfConsistentWithSamples)
{
  // 剖面自洽：相邻采样点的 Δs/Δt 与 Δyaw/Δt 必须等于剖面给出的 v 与 ω
  // （错了就说明"采样点"与"剖面"各说各话 —— 下游按剖面走就会和车实际走的对不上）
  auto f = openFixture();
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 2.0, 7.0, 6.0);   // 斜线
  req.start = pose(1.0, 2.0, true, 0.0);         // 车头朝 +x，路径 45° ⇒ 先转正
  req.goal = pose(7.0, 6.0, true, std::atan2(4.0, 6.0));

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  ASSERT_GE(r.samples.size(), 10u);

  double max_v_err = 0.0;
  double max_w_err = 0.0;
  // 用**窗口**而不用相邻两点：相邻两点是"弦长/Δt"对"中点速度"的近似，
  // 离散化误差是 O(Δt²·κ) 量级；改用 0.25 s 的窗口后这项误差平方级下降。
  const int win = 5;                              // 5 × output_dt(0.05) = 0.25 s
  for (std::size_t i = win; i < r.samples.size(); ++i) {
    const TrajSample & a = r.samples[i - win];
    const TrajSample & b = r.samples[i];
    const double dt = b.t - a.t;
    ASSERT_GT(dt, 0.0);
    const double ds = std::hypot(b.x - a.x, b.y - a.y);
    const double dth = wrapAngle(b.yaw - a.yaw);
    double v_mean = 0.0;
    double w_mean = 0.0;
    for (std::size_t j = i - win + 1; j <= i; ++j) {
      v_mean += r.samples[j].v;
      w_mean += r.samples[j].omega;
    }
    v_mean /= static_cast<double>(win);
    w_mean /= static_cast<double>(win);
    // 窗口内的弧长与弦长差别（转弯时）用均值速度无法表达 ⇒ 只看速度量级的地方
    if (std::fabs(v_mean) > 0.10) {
      max_v_err = std::max(max_v_err, std::fabs(ds / dt - v_mean) - 0.10 * std::fabs(v_mean));
    }
    if (std::fabs(w_mean) > 0.10) {
      max_w_err = std::max(max_w_err, std::fabs(dth / dt - w_mean) - 0.10 * std::fabs(w_mean));
    }
  }
  std::printf("[M2] 剖面自洽（0.25 s 窗口）：最大 v 偏差 %.4f m/s、ω 偏差 %.4f rad/s\n",
              max_v_err, max_w_err);
  EXPECT_LT(max_v_err, 0.05);
  EXPECT_LT(max_w_err, 0.05);
}

// ------------------------------------------------------------------ 避障（Stage A）

TEST(MincoOptimizer, SmoothingIncreasesClearanceAroundPillar)
{
  // 柱子：格 x∈[60,63]、y∈[40,43] ⇒ 世界 x∈[6.0,6.4]、y∈[4.0,4.4]
  auto f = pillarFixture(120, 80, 60, 40);
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  // 输入路径沿 y = 3.69 走：中心线横向只有 0.31 m ⇒ 轮廓净距 0.11 m（够过但很贴）
  TrajOptRequest req;
  req.path = straightPath(1.0, 3.69, 11.0, 3.69);
  req.start = pose(1.0, 3.69, true, 0.0);
  req.goal = pose(11.0, 3.69, true, 0.0);

  const double in_clearance = minClearanceAlong(f->ck, req.path, 0.20);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  const double out_clearance = minClearanceAlong(f->ck, r.samples, 0.20);

  std::printf("[M2] 柱子避障：输入轮廓净距 %.3f m → 输出 %.3f m（中心线目标 %.3f m）；"
              "路径偏移 max %.3f m，长度 %.2f m，%.1f ms\n",
              in_clearance, out_clearance, opt.smoothClearanceTarget(),
              r.stats.path_shift_max, r.stats.path_length, r.stats.solve_ms);

  EXPECT_GT(out_clearance, in_clearance + 0.10);
  EXPECT_LT(out_clearance, 0.60);      // 也不会"为了避障跑太远"（目标 0.55 附近）
  EXPECT_LT(r.stats.path_shift_max, 0.60);
  EXPECT_GT(r.stats.path_length, 9.0);  // 不能缩成一条捷径
}

// ------------------------------------------------------------------ 运动学与时间

TEST(MincoOptimizer, KinematicsStayWithinLimitsEvenWithAggressiveReference)
{
  // v_ref 提得很高 ⇒ 剖面被 v_max 卡住（v_cap = min(v_max, v_ref)）⇒ 这条用例
  // 现在考核的是"**不管参考多激进，最终轨迹的运动学都在界内**"。
  // （旧版剖面是几何盲的梯形，那时靠时间缩放 k>2 拉长才能守限；新剖面自己就守，
  //   所以不再断言 k 的大小 —— 断言"限值不被破"才是真正不变量。）
  auto f = openFixture();
  ASSERT_NE(f, nullptr);
  MemoryParamReader prm = baseParams();
  prm.setDouble("traj.v_ref", 3.0);
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 4.0, 9.0, 4.0);
  req.start = pose(1.0, 4.0, true, 0.0);
  req.goal = pose(9.0, 4.0, true, 0.0);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  std::printf("[M2] 激进参考 v_ref=3.0：k = %.3f，时长 %.2f s，max_v %.3f（限 %.2f）\n",
              r.stats.time_scale, r.stats.duration, r.stats.max_v, prm.getDouble("traj.v_max", 0));
  expectKinematicsWithinLimits(r, prm);
  EXPECT_GT(r.stats.solve_ms, 0.0);
}

TEST(MincoOptimizer, CornerIsSmoothedAndCurvatureStaysFinite)
{
  auto f = openFixture(160, 160);
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = {pose(2.0, 2.0), pose(10.0, 2.0), pose(10.0, 10.0)};   // 90° 折角
  req.start = pose(2.0, 2.0, true, 0.0);
  req.goal = pose(10.0, 10.0, true, M_PI / 2.0);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  // 折角处曲率是无穷（折线）⇒ 轨迹必须是有限的、且受 ω_max/v 约束
  EXPECT_GT(r.stats.max_curvature, 0.0);
  EXPECT_LT(r.stats.max_curvature, prm.getDouble("traj.w_max", 1.0) / 0.05);
  expectKinematicsWithinLimits(r, prm);
  EXPECT_LE(r.stats.terminal_error, 0.01 + 1e-6);
  EXPECT_LE(r.stats.terminal_error_yaw * 180.0 / M_PI, 1.0);
  // 切角 ⇒ 长度略短于折线（16.0 m）
  EXPECT_LT(r.stats.path_length, 16.05);
  EXPECT_GT(r.stats.path_length, 15.0);
  std::printf("[M2] 90° 折角：长度 %.2f m（折线 16.00）/ max|κ| %.3f / 终点 %.4f m / 末朝向误差 %.2f°\n",
              r.stats.path_length, r.stats.max_curvature, r.stats.terminal_error,
              r.stats.terminal_error_yaw * 180.0 / M_PI);
}

// ------------------------------------------------------------------ 终点朝向

TEST(MincoOptimizer, GoalYawIsAStateConstraintNotATangent)
{
  // 目标朝向比路径末端切线差 90°：use_goal_yaw=true 时轨迹末段要把机头转过去，
  // 而且**终点 XY 仍然要准**（这正是 (θ,s) 里"朝向是状态、XY 是派生量"的直接后果）
  auto f = openFixture(160, 120);
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  const std::vector<Pose2D> path = straightPath(2.0, 3.0, 12.0, 3.0);
  TrajOptRequest req;
  req.path = path;
  req.start = pose(2.0, 3.0, true, 0.0);
  req.goal = pose(12.0, 3.0, true, M_PI / 2.0);   // 要求机头朝 +y
  req.use_goal_yaw = true;

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  EXPECT_LE(r.stats.terminal_error, 0.01 + 1e-6);
  EXPECT_LE(r.stats.terminal_error_yaw * 180.0 / M_PI, 1.0);
  expectKinematicsWithinLimits(r, prm);

  // 关掉它 ⇒ 末朝向应贴着路径切线（≈0）
  TrajOptRequest req2 = req;
  req2.use_goal_yaw = false;
  const TrajOptResult r2 = opt.optimize(req2);
  ASSERT_EQ(r2.status, TrajStatus::kSuccess) << r2.message;
  EXPECT_LT(std::fabs(wrapAngle(r2.samples.back().yaw - 0.0)), 1.0 * M_PI / 180.0);
  std::printf("[M2] 终点朝向：use_goal_yaw=on → 末朝向误差 %.3f°、XY 残差 %.4f m；"
              "off → 末朝向 %.3f°\n", r.stats.terminal_error_yaw * 180.0 / M_PI,
              r.stats.terminal_error, r2.samples.back().yaw * 180.0 / M_PI);
}

// ------------------------------------------------------------------ 退化与降级

TEST(MincoOptimizer, DegenerateInputsFailCleanlyInsteadOfCrashing)
{
  auto f = openFixture(60, 60);
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  // ① 单点
  {
    TrajOptRequest req;
    req.path = {pose(1.0, 1.0)};
    req.start = pose(1.0, 1.0, true, 0.0);
    req.goal = pose(2.0, 2.0);
    const TrajOptResult r = opt.optimize(req);
    EXPECT_EQ(r.status, TrajStatus::kNoInput);
    EXPECT_TRUE(r.samples.empty());
  }
  // ② 全部点重合（重采样后只剩 1 点）
  {
    TrajOptRequest req;
    req.path = {pose(1.0, 1.0), pose(1.0, 1.0), pose(1.0, 1.0)};
    req.start = pose(1.0, 1.0, true, 0.0);
    req.goal = pose(1.0, 1.0);
    const TrajOptResult r = opt.optimize(req);
    EXPECT_EQ(r.status, TrajStatus::kNoInput) << r.message;
  }
  // ③ 没有地图
  {
    MincoOptimizer o2;
    ASSERT_TRUE(o2.configure(prm));
    TrajOptRequest req;
    req.path = straightPath(1.0, 1.0, 3.0, 1.0);
    req.start = pose(1.0, 1.0, true, 0.0);
    req.goal = pose(3.0, 1.0);
    const TrajOptResult r = o2.optimize(req);
    EXPECT_EQ(r.status, TrajStatus::kNoInput);
  }
  // ④ 无距离场 + 无判定器：仍要能出轨迹，只是净距没测
  {
    MincoOptimizer o3;
    ASSERT_TRUE(o3.configure(prm));
    o3.setMap(f->map);
    TrajOptRequest req;
    req.path = straightPath(1.0, 1.0, 4.0, 1.0);
    req.start = pose(1.0, 1.0, true, 0.0);
    req.goal = pose(4.0, 1.0);
    const TrajOptResult r = o3.optimize(req);
    ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
    EXPECT_FALSE(r.stats.clearance_valid);
    EXPECT_GT(r.samples.size(), 5u);
    expectKinematicsWithinLimits(r, prm);
  }
  // ⑤ 起点**真的压在障碍上**（真实轮廓，safe_margin = 0）⇒ 必须拒绝，不给轨迹。
  //   口径与 A* 的虚拟起点守卫一致（astar_planner.cpp）：真实轮廓都放不下时该
  //   停车报错交人工，**不能**从重叠位置规划一条"开出去"的轨迹 —— 那样的净距
  //   统计/终点修正全建在假前提上，而且会把"定位错了/地图错了"这个真故障藏起来。
  {
    auto fp = pillarFixture(60, 60, 20, 20);   // 世界 x∈[2.0,2.4)、y∈[2.0,2.4)
    ASSERT_NE(fp, nullptr);
    MincoOptimizer o4 = makeOptimizer(fp, prm);
    TrajOptRequest req;
    req.path = straightPath(2.20, 2.20, 5.0, 2.20);   // 起点在柱子正中
    req.start = pose(2.20, 2.20, true, 0.0);
    req.goal = pose(5.0, 2.20);
    ASSERT_TRUE(fp->ck.poseInCollisionNoMargin(req.start.x, req.start.y, 0.0))
        << "前提不成立：夹具里这个点竟然不是致命的";
    const TrajOptResult r = o4.optimize(req);
    EXPECT_EQ(r.status, TrajStatus::kStartBlocked) << r.message;
    EXPECT_TRUE(r.samples.empty());
    EXPECT_FALSE(r.message.empty());
    std::printf("[M2] 起点压在障碍上：status=%s note='%s'\n", toString(r.status),
                r.message.c_str());
  }
  // ⑥ 起点只在**余量带**里（真实轮廓放得下、含余量判定不通）⇒ 必须照常规划。
  //   这条防"矫枉过正"：余量带内起步是常态（全局图与感知图差几 cm、运行中新加
  //   禁行区），拒了就成了不可恢复的死局（A* 那边是换虚拟起点）。
  //   ★ 余量取 0.35（> 一格 0.10）：判定器把位姿**吸附到格心**（误差 ±半格），
  //     余量 0.05 时"余量带"在格心模型下是**空集**（见 Fixture 注释）；
  //   ★ 并在带内取**上沿**（真实净距最富裕处）：带的下沿真实净距≈ 0，车身一低头
  //     就真撞了 —— 那种场景该由**终检**抓（另有专门用例），不是这里要测的。
  {
    const double margin = 0.35;
    auto fp = pillarFixture(60, 60, 20, 20, margin);   // 柱子世界 y∈[2.0,2.4)
    ASSERT_NE(fp, nullptr);
    const double y_band = topOfBandY(*fp, 2.20, 2.45, 3.20, 0.005);
    ASSERT_GT(y_band, 0.0) << "夹具参数变了：找不到'真实轮廓放得下、含余量不通'的位置";
    const double sc_band = fp->ck.signedClearanceAt(2.20, y_band, 0.0);
    // ★ 终点必须落在**图内**（60×60 @0.1 ⇒ 6×6 m）：图外一律视为致命，
    //   会被终点守卫判 GOAL_BLOCKED。原来写 (8.0, …) 是**图外**，旧代码没守卫时
    //   就默默算了一条跑出图外的轨迹 —— 这个坑是加了守卫才暴露出来的。
    ASSERT_TRUE(fp->map->inside(45, 26));
    MincoOptimizer o5 = makeOptimizer(fp, prm);
    TrajOptRequest req;
    req.path = straightPath(2.20, y_band, 4.5, y_band);
    req.start = pose(2.20, y_band, true, 0.0);
    req.goal = pose(4.5, y_band);
    const TrajOptResult r = o5.optimize(req);
    EXPECT_EQ(r.status, TrajStatus::kSuccess) << r.message;
    EXPECT_GT(r.samples.size(), 5u);
    std::printf("[M2] 起点在余量带内（真实净距 %.3f m < 余量 %.2f m，y=%.3f）：照常规划 status=%s\n",
                sc_band, margin, y_band, toString(r.status));
  }
}

TEST(MincoOptimizer, StartHeadingMismatchIsTurnedWithAllocatedTime)
{
  // 现场（2026-09-24 用户 RViz 实测）：A* 从当前格朝**任意方向**迈第一步，不看车的
  // 朝向 ⇒ 起点 yaw 与路径首段可以差很多（实测 2.1 rad）。
  // 旧代码两处都看不见这个差：
  //   ① 剖面的 κ 从**折线几何**另算（且 i=0 处 atan2(0,0)=0 使 κ(0) 伪造为 14.0）；
  //   ② 剖面的速度地板 max(v_min, …) 又把 κ=14 处的上限 0.046 抬回 0.2。
  // ⇒ 不给这段转向分配任何时间 ⇒ 未缩放解 max|ω| = 6.57 rad/s（限 0.65）
  // ⇒ 时间缩放 ×10.1 ⇒ 实测 5.62 m 走出 144.75 s、max|v| 只剩 0.070 m/s。
  // 正确行为：
  //   · 剖面用 θ(s) 的**逐段必要条件** v ≤ ω_max·Δs/|Δθ| 给出真实时间下界；
  //   · 路点朝向做**转向速率限幅**，于是 |Δθ_j| ≤ ρ·ω_max·Δt_j 由构造保证；
  //   · 两者合起来 = 起点附近**原地转**（v 很低），转完再正常走。
  auto f = openFixture(160, 80);
  ASSERT_NE(f, nullptr);
  MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 4.0, 9.0, 4.0);   // 路径朝 +x（0°）
  req.start = pose(1.0, 4.0, true, 2.1);         // 车实际朝 120°
  req.goal = pose(9.0, 4.0);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  const double wmax = prm.getDouble("traj.w_max", 0.0);
  // ① 运动学必须在界内 —— 这条是"轨迹可行"唯一诚实的定义
  expectKinematicsWithinLimits(r, prm);
  // ② 时间缩放的**唯一**用途是兜底：这里必须≈1（说明剖面自己就守住了运动学）
  EXPECT_LT(r.stats.time_scale, 1.15) << r.message;
  // ③ 转向必须真的被"边转边走"吃掉，而不是被静态忽略：
  //    起点处 v 被压到 ≈ ω_max·Δs/Δθ ≈ 0.05 m/s，所以时长要涨
  EXPECT_GT(r.stats.duration, 14.0) << "转向时间没有被分配（时长没涨）";
  EXPECT_LT(r.stats.duration, 40.0) << "转向时间被极度高估（又变成 144 s 那种）";
  // ④ 起点确实是"几乎原地转"：头 1 s 内 XY 位移应该很小
  double xy_1s = 0.0;
  for (const TrajSample & t : r.samples) {
    if (t.t <= 1.0) {
      xy_1s = std::hypot(t.x - req.start.x, t.y - req.start.y);
    }
  }
  EXPECT_LT(xy_1s, 0.30) << "起点段走了 " << xy_1s << " m：这不是原地转，是边冲边转";
  std::printf("[M2] 起点朝向差 120°：时长 %.2f s / max|ω| %.3f（限 %.2f）/ k=%.3f / "
              "头 1 s 位移 %.3f m\n",
              r.stats.duration, r.stats.max_omega, wmax, r.stats.time_scale, xy_1s);
  std::printf("[M2]   note='%s'\n", r.message.c_str());
}

TEST(MincoOptimizer, LongPathWithHugeStartMismatchStaysFeasible)
{
  // 现场（用户 2026-09-24 第二次 RViz 实测，另一张图）：
  //   3894 点 / 时长 194.64 s / 路径 17.20 m / max|v| 0.160 m/s / 时间缩放 ×3.95
  //   折线max|κ| 16.4 @s=0.00 ⇒ |Δθ_起点| = 16.4×0.15 = 2.46 rad
  // 这条用例把"长路径 + 大起点朝向差"（会触发 n_piece 上限 60 ⇒ T_step 变粗）钉住：
  // 只要求运动学在界内 + 时长合理 + 起点确实原地转，不强求 k≈1（k 的含义见下）。
  auto f = openFixture(200, 200);   // 20×20 m
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 1.0, 18.0, 1.0);   // 17 m
  req.start = pose(1.0, 1.0, true, 2.46);         // 车朝 141°，路径朝 0°
  req.goal = pose(18.0, 1.0);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  expectKinematicsWithinLimits(r, prm);
  EXPECT_LT(r.stats.duration, 120.0);
  double xy_1s = 0.0;
  for (const TrajSample & t : r.samples) {
    if (t.t <= 1.0) xy_1s = std::hypot(t.x - req.start.x, t.y - req.start.y);
  }
  EXPECT_LT(xy_1s, 0.30);
  std::printf("[M2] 长路径 17 m + 起点差 141°：时长 %.2f s / max|ω| %.3f / k=%.3f / "
              "max|v| %.3f / 头 1 s 位移 %.3f m / 耗时 平滑 %.1f + MINCO %.1f + "
              "统计 %.1f = %.1f ms（终检 %d 位姿）\n",
              r.stats.duration, r.stats.max_omega, r.stats.time_scale, r.stats.max_v, xy_1s,
              r.stats.smooth_ms, r.stats.minco_ms, r.stats.stats_ms, r.stats.solve_ms,
              r.stats.check_points);
}

TEST(MincoOptimizer, GoalInsideObstacleIsRejectedNotPlannedInto)
{
  // 用户 2026-09-24 第二次实测：终点落在障碍里时，本层**硬算**出一条
  //   3894 点 / 194.64 s / 17.20 m / max|v| 0.160 m/s / 时间缩放 ×3.95
  // 的轨迹。这与"起点压在障碍上"是同一类错误：**输入本身不合法**。
  // 理由：
  //   · 轨迹的**末状态是硬约束**（终点 XY + 终点朝向）⇒ 目标点不可达时，
  //     末端修正只能靠"绕/挤"去够它，净距必然被压穿；
  //   · goal_checker 永远不可能判到点 ⇒ 任务卡死，而不是干净失败；
  //   · 与起点守卫对称：起点压障拒绝、终点压障也必须拒绝（同口径，见
  //     astar_planner.cpp 的虚拟起点守卫：真实轮廓放不下就停车报错交人工）。
  // 判据同样只取**真实轮廓**（safe_margin = 0）：终点落在余量带里是常态，要照常规划。
  auto f = pillarFixture(120, 80, 60, 40);   // 柱：世界 x∈[6.0,6.4)、y∈[4.0,4.4)
  ASSERT_NE(f, nullptr);
  const MemoryParamReader prm = baseParams();
  MincoOptimizer opt = makeOptimizer(f, prm);

  TrajOptRequest req;
  req.path = straightPath(1.0, 4.2, 6.2, 4.2);   // 终点 (6.2, 4.2) 在柱子正中
  req.start = pose(1.0, 4.2, true, 0.0);
  req.goal = pose(6.2, 4.2);
  ASSERT_TRUE(f->ck.poseInCollisionNoMargin(req.goal.x, req.goal.y, 0.0))
      << "前提不成立：夹具里这个终点竟然不是致命的";

  const TrajOptResult r = opt.optimize(req);
  EXPECT_EQ(r.status, TrajStatus::kGoalBlocked) << r.message;
  EXPECT_TRUE(r.samples.empty());
  std::printf("[M2] 终点压在障碍上：status=%s note='%s'\n", toString(r.status),
              r.message.c_str());

  // 对称性：终点只在**余量带**里时必须照常规划（不能矫枉过正）。
  //   余量取 0.35（> 一格）+ 带内取**上沿**，理由同起点那条（见上）。
  {
    const double margin = 0.35;
    auto fp = pillarFixture(120, 80, 60, 40, margin);
    ASSERT_NE(fp, nullptr);
    const double y_band = topOfBandY(*fp, 6.2, 4.45, 5.60, 0.005);
    ASSERT_GT(y_band, 0.0) << "夹具参数变了：找不到'真实轮廓放得下、含余量不通'的终点";
    MincoOptimizer o2 = makeOptimizer(fp, prm);
    TrajOptRequest r2;
    r2.path = straightPath(1.0, y_band, 6.2, y_band);
    r2.start = pose(1.0, y_band, true, 0.0);
    r2.goal = pose(6.2, y_band);
    const TrajOptResult rr = o2.optimize(r2);
    EXPECT_EQ(rr.status, TrajStatus::kSuccess) << rr.message;
    std::printf("[M2] 终点在余量带内（y=%.3f，真实净距 %.3f < 余量 %.2f）：照常规划 status=%s\n",
                y_band, fp->ck.signedClearanceAt(6.2, y_band, 0.0), margin,
                toString(rr.status));
  }
}

// ------------------------------------------------------------------ M3 轨迹终检

TEST(TrajectoryCheck, CatchesInPlaceRotationSweepMissedByArcLengthOnlySampling)
{
  // ★ 这条专门钉住"终检采样必须**同时**按弧长和**朝向**细分"：
  //   车停在柱子旁边**原地转 180°** —— 弧长几乎为 0，朝向却变了 π。
  //   只按 ds（弧长）细分时 steps = ceil(0/0.02) = 1 ⇒ **一个中间位姿都不查**，
  //   而那两个端点位姿（yaw=0 与 yaw=π）对矩形车体**对称、都安全**
  //   ⇒ 会给出"通过"的结论，把真正插进柱子的那个中间朝向整段漏掉。
  //   这正是 M3 要堵的洞（而且现在剖面真的会在起点原地转：起点朝向与路径差很大时）。
  auto f = pillarFixture(120, 80, 60, 40);   // 柱：世界 x∈[6.0,6.4)、y∈[4.0,4.4)
  ASSERT_NE(f, nullptr);

  // 车中心 (6.2, 4.70)：yaw=π/2（车头朝 -y）时长轴伸到 y=4.35 < 4.4 ⇒ 插进柱子；
  //                        yaw=0 或 π 时车身横着（y∈[4.5,4.9]）⇒ 安全。
  const double cx = 6.2;
  const double cy = 4.70;
  std::vector<pnc_2d::TrajSample> two;
  for (double yaw : {0.0, M_PI}) {
    pnc_2d::TrajSample t;
    t.s = 0.0;   // **弧长不变**：纯原地转
    t.x = cx;
    t.y = cy;
    t.yaw = yaw;
    two.push_back(t);
  }
  pnc_2d::TrajCheckParams prm;
  prm.stats_exclude_s = 0.0;
  const pnc_2d::TrajCheckResult r = pnc_2d::checkTrajectory(two, f->ck, prm);
  EXPECT_GT(r.points, 5) << "朝向细分没生效（只有端点被查）";
  EXPECT_FALSE(r.ok) << "中间朝向插进柱子的那段被漏掉了：" << r.note;
  EXPECT_GT(r.violations, 0);
  std::printf("[M3] 原地转 180° 扫掠：查了 %d 个位姿 / %d 个不过 / 最差净距 %.3f m @s=%.2f\n",
              r.points, r.violations, r.worst_clearance, r.worst_clearance_s);

  // 反面对照：同样两个端点，但**朝向细分关掉**（dyaw 设得比整段转角还大）
  pnc_2d::TrajCheckParams coarse = prm;
  coarse.dyaw = 10.0;
  const pnc_2d::TrajCheckResult rc = pnc_2d::checkTrajectory(two, f->ck, coarse);
  EXPECT_EQ(rc.points, 2) << "对照组应当只查端点";
  EXPECT_TRUE(rc.ok) << "对照组本来就该漏（这就是为什么必须有 dyaw 细分）";
}

TEST(TrajectoryCheck, ReportsPointCoordinatesWhenTrajectoryHitsPillar)
{
  // 一条"直穿柱子"的轨迹：终检必须不过，并且**点名坐标 + 实测净距**（可操作，
  // 不是"优化失败"四个字）。
  auto f = pillarFixture(120, 80, 60, 40);
  ASSERT_NE(f, nullptr);
  std::vector<pnc_2d::TrajSample> s;
  for (int i = 0; i <= 100; ++i) {
    pnc_2d::TrajSample t;
    t.s = 4.0 * i / 100.0;
    t.x = 4.0 + t.s;
    t.y = 4.2;   // 正好穿过柱子中心线
    t.yaw = 0.0;
    s.push_back(t);
  }
  pnc_2d::TrajCheckParams prm;
  const pnc_2d::TrajCheckResult r = pnc_2d::checkTrajectory(s, f->ck, prm);
  EXPECT_FALSE(r.ok);
  EXPECT_GT(r.violations, 0);
  EXPECT_LT(r.worst_clearance, 0.0);
  // 第一个失败点应该在柱子附近（x∈[6.0,6.4] 减去半车长 0.35 ⇒ x≈5.65 起）
  EXPECT_NEAR(r.first_bad_x, 5.65, 0.10) << r.note;
  EXPECT_LT(r.first_bad_clearance, 0.0);
  EXPECT_NE(r.note.find("终检不过"), std::string::npos);
  std::printf("[M3] 穿柱轨迹：%s\n", r.note.c_str());
}

TEST(TrajectoryCheck, SamplesFromPathUseSegmentDirectionAsYaw)
{
  // `trajSamplesFromPath`：折线没有朝向时取线段方向（"车头沿路径"），
  // 弧长要累计对。用途是**降级路径（A* 原路径）也必须被终检过一遍**。
  const std::vector<pnc_2d::Pose2D> path = {pose(1.0, 1.0), pose(3.0, 1.0), pose(3.0, 3.0)};
  const std::vector<pnc_2d::TrajSample> s = pnc_2d::trajSamplesFromPath(path, 0.5);
  ASSERT_GE(s.size(), 5u);
  EXPECT_NEAR(s.front().x, 1.0, 1e-9);
  EXPECT_NEAR(s.front().y, 1.0, 1e-9);
  EXPECT_NEAR(s.back().x, 3.0, 1e-9);
  EXPECT_NEAR(s.back().y, 3.0, 1e-9);
  EXPECT_NEAR(s.back().s, 4.0, 1e-6);       // 2 m + 2 m
  EXPECT_NEAR(s.front().yaw, 0.0, 1e-9);    // 第一段朝 +x
  EXPECT_NEAR(s.back().yaw, M_PI / 2, 1e-9); // 末段朝 +y
  for (const pnc_2d::TrajSample & t : s) {
    EXPECT_GT(t.s, -1e-9);
    EXPECT_LE(t.s, 4.0 + 1e-6);
  }
}

TEST(MincoOptimizer, FinalCheckHasFinalSayOverThePenaltyTarget)
{
  // ★ 计划书要求的用例："罚函数收敛但终检不过" ⇒ 终检**说了算**。
  //   构造：让轨迹从柱子旁边掠过（平滑把中心线推到 0.30 m 净距，罚函数**收敛了**），
  //   但把硬门卡在罚目标**之上**（0.80 > 罚目标 0.62）⇒ 轨迹必须被终检拦下。
  //   反过来把硬门放回 0（"不碰"）⇒ 同一条路必须通过。
  auto f = pillarFixture(120, 80, 60, 40);   // 柱：世界 x∈[6.0,6.4)、y∈[4.0,4.4)
  ASSERT_NE(f, nullptr);
  TrajOptRequest req;
  req.path = straightPath(1.0, 3.69, 11.0, 3.69);   // 从柱子旁边贴着急过
  req.start = pose(1.0, 3.69, true, 0.0);
  req.goal = pose(11.0, 3.69);

  // 对照：硬门 = 0（只要"不碰"）⇒ 通过
  const MemoryParamReader prm_ok = baseParams();
  MincoOptimizer opt_ok = makeOptimizer(f, prm_ok);
  const TrajOptResult r_ok = opt_ok.optimize(req);
  ASSERT_EQ(r_ok.status, TrajStatus::kSuccess) << r_ok.message;
  ASSERT_TRUE(r_ok.stats.check_ok);
  ASSERT_GT(r_ok.stats.check_points, 300);   // 10 m / 0.02 m ⇒ 约 500 个位姿
  std::printf("[M3] 硬门 0.0 ⇒ status=%s，终检查了 %d 个位姿，最差净距 %.4f m\n",
              toString(r_ok.status), r_ok.stats.check_points,
              r_ok.stats.check_worst_clearance);

  // 硬门 0.80 > 实际达成的 0.30（也 > 罚目标 0.62）⇒ 终检必须拦下
  MemoryParamReader prm = baseParams();
  prm.setDouble("traj.check_hard_margin", 0.80);
  MincoOptimizer opt = makeOptimizer(f, prm);
  const TrajOptResult r = opt.optimize(req);
  EXPECT_EQ(r.status, TrajStatus::kCheckFailed) << r.message;
  // ★ 样本**保留**（失败点要看得到），但状态说明不得执行
  EXPECT_GT(r.samples.size(), 5u);
  EXPECT_FALSE(r.stats.check_ok);
  EXPECT_GT(r.stats.check_violations, 0);
  EXPECT_NE(r.message.find("终检不过"), std::string::npos);
  std::printf("[M3] 硬门 0.80 > 罚目标 0.62 ⇒ status=%s\n  %s\n", toString(r.status),
              r.message.c_str());
}

// ------------------------------------------------------------------ 机制 ①：端点自适应

TEST(MincoOptimizer, TightEndpointRelaxesSmoothTargetInsteadOfFightingAnchor)
{
  // **机制 ①**（借自 DDR-opt：`safeDis = min(0.85 × 起点实测净距, safeDis_max)`）。
  // 为什么需要：首末点被**锚定**（车当前位姿 / 目标），平滑**动不了它们** ⇒ 端点附近的
  // 净距**几何上不可达**全局目标 ⇒ SDF 罚与锚定项**死拽**，收敛点落在目标之下
  // （用户现场 log 就是 `净距目标 0.62m/达成 0.47m`），而持续对抗还会把路径挤出畸形
  // 甚至撞上别的障碍。
  // 判据：贴着墙走（端点净距 ≈ 0.43 m < 目标 0.62 m）时
  //   · `smooth_d_cap_head/tail` 必须**真的低于** `smooth_d_target`（机制生效）
  //   · `smooth_shortfall ≈ 0`（逐点目标都达标 —— 这才叫"平滑做到了"）
  //   · `smooth_d_achieved < smooth_d_target`（确实低于全局目标，证明旧口径会误报）
  //   · 轨迹仍然通过**终检**（放松的是"舒适目标"，不是"不碰"这条硬线）
  auto f = wallRowFixture(120, 80, 40);   // 墙在 y∈[4.0,4.1)
  ASSERT_NE(f, nullptr);
  // 沿墙走：中心线离墙 0.60 m（端点自适应会触发），但 footprint 还留 0.40 m
  // （贴到 4.55 时终点附近只剩 5 cm，终检会真的报碰撞 —— 那是几何问题不是机制问题）
  MemoryParamReader prm_on = baseParams();
  prm_on.setDouble("traj.smooth_safe_ratio", 0.85);
  MincoOptimizer opt = makeOptimizer(f, prm_on);

  TrajOptRequest req;
  req.path = straightPath(1.0, 4.70, 11.0, 4.70);
  req.start = pose(1.0, 4.70, true, 0.0);
  req.goal = pose(11.0, 4.70);

  const TrajOptResult r = opt.optimize(req);
  ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
  std::printf("[M2] 贴墙（机制① 开）：目标 %.3f / 端点自适应下限 首 %.3f 末 %.3f / "
              "达成 %.3f / 缺口 %.4f / 偏移 %.3f / 终检 %s\n",
              r.stats.smooth_d_target, r.stats.smooth_d_cap_head,
              r.stats.smooth_d_cap_tail, r.stats.smooth_d_achieved,
              r.stats.smooth_shortfall, r.stats.path_shift_max,
              r.stats.check_ok ? "通过" : "不过");
  EXPECT_LT(r.stats.smooth_d_cap_head, r.stats.smooth_d_target - 1e-6)
      << "端点净距受限时机制 ① 没生效";
  EXPECT_LT(r.stats.smooth_d_cap_tail, r.stats.smooth_d_target - 1e-6);
  EXPECT_TRUE(r.stats.check_ok) << r.message;
  expectKinematicsWithinLimits(r, prm_on);

  // ★★ 判据 = 机制 ① 的**实测效应**（不是"它有没有生效"）：
  //   目标压低 ⇒ SDF 罚提前归零 ⇒ **实际达成净距也变低**（结构性单调关系）。
  //   而终检查的是**实际净距** ⇒ 开它只会让拦停**更频繁**。
  //   缺口变小只是"逐点目标更松"这个报告口径的事，不是安全变好。
  //   ⇒ 默认关掉（`smooth_safe_ratio: 0.0`，与 `max_length_m: 0 = 不截断` 同约定）。
  const MemoryParamReader prm_off = baseParams();   // 默认就是关
  MincoOptimizer opt_off = makeOptimizer(f, prm_off);
  const TrajOptResult ro = opt_off.optimize(req);
  ASSERT_EQ(ro.status, TrajStatus::kSuccess) << ro.message;
  std::printf("[M2] 贴墙（机制① 关）：端点自适应下限 首 %.3f 末 %.3f / 达成 %.3f / "
              "缺口 %.4f / 偏移 %.3f\n",
              ro.stats.smooth_d_cap_head, ro.stats.smooth_d_cap_tail,
              ro.stats.smooth_d_achieved, ro.stats.smooth_shortfall,
              ro.stats.path_shift_max);
  EXPECT_NEAR(ro.stats.smooth_d_cap_head, ro.stats.smooth_d_target, 1e-6)
      << "关掉后不该有任何自适应";
  EXPECT_LT(r.stats.smooth_d_achieved, ro.stats.smooth_d_achieved - 1e-3)
      << "机制① 应当让**实际**达成净距变低（开了 " << r.stats.smooth_d_achieved
      << "，关了 " << ro.stats.smooth_d_achieved << "）";
  EXPECT_LT(r.stats.smooth_shortfall, ro.stats.smooth_shortfall - 1e-3)
      << "机制① 应当让缺口变小（开了 " << r.stats.smooth_shortfall << "，关了 "
      << ro.stats.smooth_shortfall << "）";

  // 对照：**开阔处机制不生效**（端点下限 = 全局目标）
  auto fo = openFixture(120, 80);
  ASSERT_NE(fo, nullptr);
  MincoOptimizer oo = makeOptimizer(fo, baseParams());
  TrajOptRequest req2;
  req2.path = straightPath(1.0, 4.0, 9.0, 4.0);
  req2.start = pose(1.0, 4.0, true, 0.0);
  req2.goal = pose(9.0, 4.0);
  const TrajOptResult r2 = oo.optimize(req2);
  ASSERT_EQ(r2.status, TrajStatus::kSuccess) << r2.message;
  EXPECT_NEAR(r2.stats.smooth_d_cap_head, r2.stats.smooth_d_target, 1e-6)
      << "开阔处不该触发端点自适应";
  EXPECT_NEAR(r2.stats.smooth_shortfall, 0.0, 1e-6);
}

TEST(MincoOptimizer, AvoidanceWeightVsAchievedClearanceIsMonotoneAndMeasured)
{
  // ★ 既然机制 ①（降低目标）是**反效果**，真正的杠杆就只剩"推得更使劲"。
  //   这条用例把那个杠杆量出来（项目纪律：参数只作为**量到收益**的结论落地）：
  //   贴墙走，扫 `smooth_w_sdf`，看 ① 实际中心线净距 ② 平滑偏移 ③ 终检。
  //   并钉住一条**不变式**：加大避障权重**不得**让实际净距变小（否则就是 bug）。
  auto f = wallRowFixture(120, 80, 40);
  ASSERT_NE(f, nullptr);
  TrajOptRequest req;
  req.path = straightPath(1.0, 4.70, 11.0, 4.70);
  req.start = pose(1.0, 4.70, true, 0.0);
  req.goal = pose(11.0, 4.70);

  std::printf("[M2] 避障权重 vs 实际达成净距（贴墙 10 m）：\n");
  double prev_d = -1.0;
  for (double w : {2.0, 4.0, 8.0, 16.0}) {
    MemoryParamReader prm = baseParams();
    prm.setDouble("traj.smooth_w_sdf", w);
    MincoOptimizer opt = makeOptimizer(f, prm);
    const TrajOptResult r = opt.optimize(req);
    ASSERT_EQ(r.status, TrajStatus::kSuccess) << r.message;
    std::printf("[M2]   w_sdf %5.1f → 达成 %.3f m / 缺口 %.3f / 偏移 %.3f m / 终检 %s\n", w,
                r.stats.smooth_d_achieved, r.stats.smooth_shortfall,
                r.stats.path_shift_max, r.stats.check_ok ? "通过" : "不过");
    EXPECT_GT(r.stats.smooth_d_achieved, prev_d - 1e-6)
        << "加大避障权重反而让实际净距变小了（w_sdf=" << w << "）";
    prev_d = r.stats.smooth_d_achieved;
  }
}

TEST(MincoOptimizer, RejectsInvalidConfig)
{
  MincoOptimizer opt;
  MemoryParamReader bad = baseParams();
  bad.setDouble("traj.v_max", 0.0);
  EXPECT_FALSE(opt.configure(bad));          // 限值必须 > 0，否则带状矩阵会除零

  MemoryParamReader bad2 = baseParams();
  bad2.setDouble("traj.min_piece_time", 0.0);
  MincoOptimizer opt2;
  EXPECT_FALSE(opt2.configure(bad2));

  MincoOptimizer opt3;
  EXPECT_TRUE(opt3.configure(baseParams()));
}

}  // namespace
}  // namespace pnc_2d
