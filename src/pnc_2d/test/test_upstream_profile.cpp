// M4.3：**上游剖面**（`ReferenceProfile`）在 MPC 里的作用与边界。
//
// 为什么单独一个测试文件：这一步改的是"速度上限的**唯一来源**"——
//   有剖面时**换掉**本地曲率限速（不是取 min）。这种"二选一"最容易写成
//   "两个都留着、谁小听谁"，而那种写法在一条**直线**上测不出来（两者算出来一样），
//   只有在**弯道 + 一份更宽松的上游剖面**下才会暴露 ⇒ 第 ③ 组就是干这个的。
//
// 三组断言：
//   ① 有剖面 ⇒ 参考速度被压到剖面上限；
//   ② A/B 开关关掉 / 没有剖面 ⇒ 与改前一字不差（回归保证）；
//   ③ 弯道 + 宽松剖面 ⇒ 本地曲率限速**必须消失**（证明是"换掉"而非"取 min"）。
//
// ⚠ 夹具把 `degraded_speed_limit` 设成 100：没有距离场时 MPC 会判定"降级"
//   并把速度压到那个值（默认 0.30，刚好和本文件的用例数值撞车 ⇒ 会把
//   "剖面生效"误判成"降级生效"）。这里测的是剖面逻辑，不是降级路径。

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/reference_profile.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"

namespace pnc_2d {
namespace {

std::vector<Pose2D> straightPath(double length, double res) {
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res)
    p.push_back(Pose2D{s, 0.0, 0.0});
  return p;
}

/// 半径 R 的圆弧（朝向 = 切线）：让本地曲率限速**确定地**起作用
/// （κ ≈ 1/R，与 `ref_curv_` 用 min_span=5 cm 基线算出来的一致）。
std::vector<Pose2D> arcPath(double radius, double length, double res) {
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res) {
    const double th = s / radius;
    p.push_back(Pose2D{radius * std::sin(th), radius * (1.0 - std::cos(th)), th});
  }
  return p;
}

/// 常数剖面：s 从 0 到 `s_end`，全程 v = `v`、ω = `w`
ReferenceProfile makeProfile(double s_end, double v, double w = 0.0) {
  ReferenceProfile pr;
  std::string err;
  const std::vector<double> s{0.0, 0.5 * s_end, s_end};
  const std::vector<double> vv(s.size(), v);
  const std::vector<double> ww(s.size(), w);
  EXPECT_TRUE(pr.set(s, vv, ww, err)) << err;
  return pr;
}

MpcLocalPlanner makeMpc(double v_max, bool use_upstream, int horizon = 15) {
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setInt("local_mpc.horizon", horizon);
  p.setDouble("local_mpc.v_max", v_max);
  p.setBool("local_mpc.use_upstream_profile", use_upstream);
  p.setDouble("local_mpc.degraded_speed_limit", 100.0); // 见文件头说明
  EXPECT_TRUE(mpc.configure(p));
  return mpc;
}

/// 跑一次求解，返回本周期参考速度（= `ref[0].v`，也就是"这一拍允许跑多快"）
double refSpeedOf(MpcLocalPlanner &mpc, const std::vector<Pose2D> &path) {
  mpc.setGlobalPlan(path);
  mpc.setCurrentVelocity(0.0, 0.0);
  Pose2D pose = path.front();
  pose.has_yaw = true;
  const auto r = mpc.computeCommand(pose, mpc.params().dt);
  EXPECT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  return mpc.solveInfo().ref_v;
}

// ---------------------------------------------------------------------------
// ① 剖面上限生效
// ---------------------------------------------------------------------------
TEST(UpstreamProfile, CapsReferenceSpeed) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/true);
  auto prof = makeProfile(10.0, 0.30);
  mpc.setReferenceProfile(&prof);
  const double v = refSpeedOf(mpc, straightPath(10.0, 0.05));
  EXPECT_NEAR(v, 0.30, 1e-9)
      << "有剖面时参考速度必须被压到剖面上限（v_max=1.0 不该起作用）";
}

// ---------------------------------------------------------------------------
// ② 回归保证：没有剖面 / 开关关掉 ⇒ 与改前完全一致
// ---------------------------------------------------------------------------
TEST(UpstreamProfile, NoProfileIsUnchanged) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/true); // 开关开着但没给剖面
  const double v = refSpeedOf(mpc, straightPath(10.0, 0.05));
  EXPECT_NEAR(v, 1.0, 1e-9);
}

TEST(UpstreamProfile, SwitchOffIgnoresProfile) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/false);
  auto prof = makeProfile(10.0, 0.30);
  mpc.setReferenceProfile(&prof);
  const double v = refSpeedOf(mpc, straightPath(10.0, 0.05));
  EXPECT_NEAR(v, 1.0, 1e-9)
      << "A/B 开关关掉时必须完全忽略剖面（= M4.2 之前的现网行为）";
}

TEST(UpstreamProfile, InvalidProfileIsIgnored) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/true);
  ReferenceProfile bad; // 没 set 过 ⇒ valid() == false
  mpc.setReferenceProfile(&bad);
  const double v = refSpeedOf(mpc, straightPath(10.0, 0.05));
  EXPECT_NEAR(v, 1.0, 1e-9) << "无效剖面必须退化成'没有剖面'";
}

// ---------------------------------------------------------------------------
// ③ 弯道：本地曲率限速是**被换掉**，不是被取 min
// ---------------------------------------------------------------------------
TEST(UpstreamProfile, CurvatureLimitIsReplacedNotMinned) {
  const double R = 1.0;                 // 半径 1 m ⇒ κ ≈ 1 /m
  const double V = 2.0;                 // v_max 高于本地曲率限速，才看得出差别
  const double kappa = 1.0 / R;
  const double local_cap = std::sqrt(1.5 / kappa); // lat_acc_max 默认 1.5
  auto path = arcPath(R, 6.0, 0.05);

  // (a) 无剖面：本地曲率限速生效（先证明这条圆弧确实会触发它）
  {
    auto mpc = makeMpc(V, true);
    const double v = refSpeedOf(mpc, path);
    EXPECT_LT(v, V) << "这条圆弧应当触发本地曲率限速";
    EXPECT_NEAR(v, local_cap, 0.15) << "无剖面时参考速度应≈sqrt(lat_acc_max·R)";
  }

  // (b) 宽松剖面（3.0 > v_max）：本地曲率限速**不得**再出现
  {
    auto mpc = makeMpc(V, true);
    auto prof = makeProfile(6.0, 3.0);
    mpc.setReferenceProfile(&prof);
    const double v = refSpeedOf(mpc, path);
    EXPECT_NEAR(v, V, 1e-9)
        << "有剖面时应当是【换掉】本地曲率限速。若这里≈" << local_cap
        << "，说明实现写成了取 min（上游剖面在弯道里会被本地那份盖住）";
  }

  // (c) 同一份宽松剖面 + 关掉开关 ⇒ 又回到本地限速（证明差别来自开关本身）
  {
    auto mpc = makeMpc(V, false);
    auto prof = makeProfile(6.0, 3.0);
    mpc.setReferenceProfile(&prof);
    const double v = refSpeedOf(mpc, path);
    EXPECT_NEAR(v, local_cap, 0.15);
  }
}

// ---------------------------------------------------------------------------
// ④ 剖面的 ω 顶替本地的 κ·v（直线 + 非零 ω 是唯一能分开两者的构造）
// ---------------------------------------------------------------------------
TEST(UpstreamProfile, OmegaComesFromProfileWhenPresent) {
  auto mpc = makeMpc(1.0, true);
  // 直线：本地 κ ≈ 0 ⇒ 若还按 κ·v 算，ω_ref 必然是 0
  auto prof = makeProfile(10.0, 0.5, /*w=*/0.25);
  mpc.setReferenceProfile(&prof);
  mpc.setGlobalPlan(straightPath(10.0, 0.05));
  mpc.setCurrentVelocity(0.0, 0.0);
  Pose2D pose = straightPath(10.0, 0.05).front();
  pose.has_yaw = true;
  const auto r = mpc.computeCommand(pose, mpc.params().dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  // 参考窗口第一步的 ω 直接来自剖面（上界 w_max 默认 0.8，0.25 不会被夹）
  EXPECT_NEAR(mpc.solveInfo().ref_w, 0.25, 1e-9);
}

// ---------------------------------------------------------------------------
// ⑤ M4.4（R9）剖面一致性度量 —— **两个数**：
//    ① `profile_dev`       = |v_ref − v_剖面(s)| / max(v_剖面, 0.1)   参考口径
//    ② `profile_track_dev` = |v_实测 − v_ref| / max(v_ref, 0.1)      跟踪口径
//    ★ 口径边界必须钉住：两个数**回答的是不同问题**，合成一个就会在
//      "起步/对正之后"造出 100% 的假警报（现场就是这么踩到的）。
// ---------------------------------------------------------------------------
double profileDev(MpcLocalPlanner &mpc, const std::vector<Pose2D> &path,
                  double v_now) {
  mpc.setGlobalPlan(path);
  mpc.setCurrentVelocity(v_now, 0.0);
  Pose2D pose = path.front();
  pose.has_yaw = true;
  const auto r = mpc.computeCommand(pose, mpc.params().dt);
  EXPECT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  return r.stats.profile_dev; // 节点看到的就是这个数
}

TEST(UpstreamProfile, RefDeviationIsZeroWhenProfileIsBinding) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/true);
  auto prof = makeProfile(10.0, 0.50);
  mpc.setReferenceProfile(&prof);
  // 剖面就是本周期生效的上限 ⇒ 参考偏差 = 0（**与实测速度无关**：
  // 车还没加速上来不算"剖面没生效"）
  mpc.setGlobalPlan(straightPath(10.0, 0.05));
  mpc.setCurrentVelocity(0.0, 0.0);   // 车还是静止的
  Pose2D pose = straightPath(10.0, 0.05).front();
  pose.has_yaw = true;
  const auto r = mpc.computeCommand(pose, mpc.params().dt);
  ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  EXPECT_NEAR(r.stats.profile_dev, 0.0, 1e-9)
      << "剖面生效时参考偏差必须是 0，即使车还没动";
  EXPECT_NEAR(r.stats.profile_v, 0.50, 1e-9);
  EXPECT_NEAR(r.stats.profile_v_ref, 0.50, 1e-9);
  // 而**跟踪**偏差此时必然很大（车从 0 加速）—— 这就是为什么要拆成两个数
  EXPECT_GT(r.stats.profile_track_dev, 0.9);
}

TEST(UpstreamProfile, RefDeviationReportsWhoCapsTheSpeed) {
  auto path = straightPath(10.0, 0.05);
  auto mpc = makeMpc(1.0, true);
  auto prof = makeProfile(10.0, 0.50);
  mpc.setReferenceProfile(&prof);
  // 任务限速 0.25 < 剖面 0.50 ⇒ 生效的是限速，参考偏差 = (0.5−0.25)/0.5 = 50%
  mpc.setSpeedLimit(0.25);
  EXPECT_NEAR(profileDev(mpc, path, 0.25), 0.50, 1e-9);
}

TEST(UpstreamProfile, RefDeviationDenominatorHasFloor) {
  auto path = straightPath(10.0, 0.05);
  auto mpc = makeMpc(1.0, true);
  auto prof = makeProfile(10.0, 0.03);   // 终点贴拢量级的低速
  mpc.setReferenceProfile(&prof);
  mpc.setSpeedLimit(0.01);               // 限速比剖面更低 ⇒ 生效 0.01
  // |0.01 − 0.03| / max(0.03, 0.10) = 20%（若没地板就是 67%，纯噪声量级）
  EXPECT_NEAR(profileDev(mpc, path, 0.01), 0.20, 1e-9);
}

TEST(UpstreamProfile, DeviationIsNotApplicableWithoutProfile) {
  auto mpc = makeMpc(1.0, /*use_upstream=*/true); // 开关开着但没有剖面
  EXPECT_LT(profileDev(mpc, straightPath(10.0, 0.05), 0.5), 0.0)
      << "没有剖面时必须报\"不适用\"(<0)：报 0 会被读成\"完全一致\"";
}

} // namespace
} // namespace pnc_2d
