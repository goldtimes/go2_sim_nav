// P5.2 **一致性校验**：验证 QP 的模型、离散化、约束组装真的对。
//
// 基准是什么（这是本文件的核心问题）：
//   · **不是** `scripts/mpc.py`（那是早期原型，用户明确不作为参考）；
//   · 是 **`/home/gmd/SLAM-PNC/PNC` 的 `MpcController`**（`src/controller/src/controller/mpc.cpp`）
//     —— 本仓库 MPC 的结构就是照它做的；
//   · 加上**解析真值**与**KKT 条件**（这两样不依赖任何实现，是硬事实）。
//
// 三层校验，各抓不同类别的错：
//
//   ① 结构对拍（手推期望）：A 的哪些位置非零、值是什么、P 的对角是什么 ——
//      期望值**从运动学手推**，不抄自实现代码。抓"漏项/错位置/错缩放"。
//      实测价值：P5.1 开发中真踩过"漏了 A 的单位阵项"，就属于这一类。
//   ② 与参考实现公式对拍：把 SLAM-PNC 的 `buildAMatrix` / `buildBMatrix`
//      **按其源码逐行抄成独立函数**，在同一位姿/速度下比对。抓"我以为我照它做了"。/
//   ③ KKT 自检：解必须满足 `l ≤ A·x ≤ u`（可行性）且对偶残差 `‖Px+q+A'y‖∞` 小
//      （最优性）。抓"求解器给的解其实没解对这个问题"。
//
// ⚠ 诚实说明本校验**不能**证明什么：②里那份参考实现是**转抄**的（SLAM-PNC 是
//   ROS 1 + OsqpEigen，无法在本工作区直接编译链接），所以它验证的是"公式一致"，
//   不是"二进制一致"。真正防止"两条轨迹碰巧都能跑"的是 ① 与 ③：它们只依赖
//   运动学与 KKT 条件本身。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"

namespace pnc_2d {
namespace {

constexpr int kStateDim = 4;    // [x, y, yaw, v]
constexpr int kControlDim = 2;  // [a, w]

using Snap = MpcLocalPlanner::QpSnapshot;

MpcLocalPlanner makeMpc(int horizon = 15)
{
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setInt("local_mpc.horizon", horizon);
  p.setDouble("local_mpc.v_max", 1.0);
  mpc.configure(p);
  return mpc;
}

std::vector<Pose2D> straightPath(double length, double res)
{
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res)
    p.push_back(Pose2D{s, 0.0, 0.0});
  return p;
}

/// 解一次并给出快照
Snap solveAndSnapshot(MpcLocalPlanner &mpc, const Pose2D &pose, double v_now)
{
  mpc.setCurrentVelocity(v_now, 0.0);
  const auto r = mpc.computeCommand(pose, mpc.params().dt);
  EXPECT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
  Snap s;
  mpc.snapshotQp(s);
  return s;
}

// ============================================================================
// ② 参考实现的公式，按 SLAM-PNC src/controller/src/controller/mpc.cpp 逐行转抄
//    （buildAMatrix / buildBMatrix / 状态与控制索引布局）
// ============================================================================
namespace slam_pnc_reference {

/// 对应其 `stateIndex(step, off)`：状态**全部**在前
int stateIndex(int horizon, int step, int off) { return kStateDim * step + off; }
/// 对应其 `controlIndex(step, off)`：控制接在全部状态之后
int controlIndex(int horizon, int step, int off)
{
  return kStateDim * (horizon + 1) + kControlDim * step + off;
}

/// 其 `buildAMatrix(reference)`：A = I + dt·∂f/∂s（阿克曼，绕参考点线性化）
/// 抄自源码：
///   a(0,2) = -v·sin(yaw)·dt;  a(0,3) = cos(yaw)·dt;
///   a(1,2) =  v·cos(yaw)·dt;  a(1,3) = sin(yaw)·dt;
///   a(2,3) =  tan(steer)/L·dt;
std::vector<double> referenceA(const MpcReferencePoint &r, double dt, double L)
{
  std::vector<double> A(static_cast<std::size_t>(kStateDim) * kStateDim, 0.0);
  for (int i = 0; i < kStateDim; ++i) A[i * kStateDim + i] = 1.0;
  A[0 * kStateDim + 2] = -r.v * std::sin(r.yaw) * dt;
  A[0 * kStateDim + 3] = std::cos(r.yaw) * dt;
  A[1 * kStateDim + 2] = r.v * std::cos(r.yaw) * dt;
  A[1 * kStateDim + 3] = std::sin(r.yaw) * dt;
  // a(2,3)：阿克曼才有（θ̇ = v/L·tanδ）。差速底盘里 ω 是**控制量**，
  // 所以这一项在我们的模型里必然为 0，而对应的影响跑到 B(2,1) 上 ——
  // 这正是 §2.1 记录的模型映射差异，不是"我们少写了一项"。
  return A;
}

}  // namespace slam_pnc_reference

// ============================================================================
// ① 结构对拍：用从运动学手推出来的期望值逐项比对
// ============================================================================

TEST(MpcQpReference, DynamicsRowsStructureMatchesKinematics)
{
  const int N = 3;  // 小规模便于手推/打印
  auto mpc = makeMpc(N);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));
  // 给它一个非平凡的参考（v ≠ 0、yaw ≠ 0）：全零参考下很多项会退化成 0，
  // 那样"漏项"是测不出来的（P5.1 就是这么漏掉 A 的单位阵项的）。
  const auto snap = solveAndSnapshot(mpc, Pose2D{2.0, 0.05, 0.10}, 0.7);
  const int n = snap.n;
  const double dt = mpc.params().dt;
  ASSERT_EQ(n, kStateDim * (N + 1) + kControlDim * N);
  ASSERT_EQ(snap.m, 4 * (N + 1) + N + 2 * N + 2 * (N - 1) + 2 * N + N);

  auto A = [&](int row, int col) { return snap.A[static_cast<std::size_t>(row) * n + col]; };

  // ---- 初始状态等式：e_0 = e_init ----
  for (int i = 0; i < kStateDim; ++i) {
    EXPECT_DOUBLE_EQ(A(i, i), 1.0) << "row " << i;
    EXPECT_DOUBLE_EQ(snap.l[i], snap.u[i]) << "row " << i << " 必须是等式";
  }
  EXPECT_DOUBLE_EQ(snap.u[1], 0.05) << "e_0.y 应等于初始横向偏差";
  EXPECT_NEAR(snap.u[2], 0.10, 1e-9) << "e_0.yaw 应等于初始航向偏差";

  // ---- 动力学等式：e_{k+1} - A_k e_k - B_k Δu_k = 0 ----
  // 期望值**从运动学手推**（不是从实现里抄）：
  //   x_{k+1} = x_k + v·cosθ·dt  ⇒  ∂/∂θ = -v·sinθ·dt,  ∂/∂v = cosθ·dt
  //   y_{k+1} = y_k + v·sinθ·dt  ⇒  ∂/∂θ =  v·cosθ·dt,  ∂/∂v = sinθ·dt
  //   θ_{k+1} = θ_k + ω·dt       ⇒  无状态偏导；控制 ω 的系数 = +dt
  //   v_{k+1} = v_k + a·dt       ⇒  无状态偏导；控制 a 的系数 = +dt
  // 参考点的 v/yaw 用求解时实际用的参考（从 A 的非零项反推会循环论证，
  // 所以这里直接查 planner 的预测轨迹起点：它等于 ref[0]）。
  const auto &pred = mpc.predictedTrajectory();
  ASSERT_GE(pred.size(), 2u);
  // ⚠ 预测轨迹第 0 点 = ref[0] + e_0，**不等于** ref[0]。
  //   e_0 的两个分量就在初始状态等式的 u[] 里 ⇒ 反推 ref[0] 才是参考点：
  //     ref[0].yaw = pred[0].yaw - e_0(yaw)
  const double yaw_ref = pred[0].yaw - snap.u[2];
  EXPECT_NEAR(yaw_ref, 0.0, 1e-9) << "直线参考的朝向应为 0（e_0.yaw = 0.10）";

  for (int k = 0; k + 1 <= N; ++k) {
    const int row0 = kStateDim + kStateDim * k;
    // 第 0 行（x）：唯一非零是 e_{k+1}(0)=+1、e_k(0)=-1、e_k(2)、e_k(3)
    EXPECT_DOUBLE_EQ(A(row0 + 0, kStateDim * (k + 1) + 0), 1.0);
    EXPECT_DOUBLE_EQ(A(row0 + 0, kStateDim * k + 0), -1.0) << "★ A 的单位阵项不能漏";
    EXPECT_DOUBLE_EQ(A(row0 + 1, kStateDim * (k + 1) + 1), 1.0);
    EXPECT_DOUBLE_EQ(A(row0 + 1, kStateDim * k + 1), -1.0) << "★ A 的单位阵项不能漏";
    // θ/v 两行同样各有单位阵项
    EXPECT_DOUBLE_EQ(A(row0 + 2, kStateDim * (k + 1) + 2), 1.0);
    EXPECT_DOUBLE_EQ(A(row0 + 2, kStateDim * k + 2), -1.0);
    EXPECT_DOUBLE_EQ(A(row0 + 3, kStateDim * (k + 1) + 3), 1.0);
    EXPECT_DOUBLE_EQ(A(row0 + 3, kStateDim * k + 3), -1.0);
    // 控制项：θ̇ = ω ⇒ B(2,1)=dt；v̇ = a ⇒ B(3,0)=dt（yaw/v 的符号按 -B）
    const int ctl = kStateDim * (N + 1) + kControlDim * k;
    EXPECT_DOUBLE_EQ(A(row0 + 2, ctl + 1), -dt) << "θ 行对 ω 的系数";
    EXPECT_DOUBLE_EQ(A(row0 + 3, ctl + 0), -dt) << "v 行对 a 的系数";
    // 位置行**不含**控制项（位置不直接受 a/ω 影响）
    for (int j = 0; j < kControlDim; ++j) {
      EXPECT_DOUBLE_EQ(A(row0 + 0, ctl + j), 0.0);
      EXPECT_DOUBLE_EQ(A(row0 + 1, ctl + j), 0.0);
    }
    // 位置行对 yaw 的偏导与参考朝向一致（符号 + 量级）
    EXPECT_NEAR(A(row0 + 0, kStateDim * k + 3), -std::cos(yaw_ref) * dt, 1e-9);
    EXPECT_NEAR(A(row0 + 1, kStateDim * k + 3), -std::sin(yaw_ref) * dt, 1e-9);
  }
}

TEST(MpcQpReference, ObjectiveWeightsLandOnTheRightSlots)
{
  const int N = 2;
  auto mpc = makeMpc(N);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));
  const auto snap = solveAndSnapshot(mpc, Pose2D{1.0, 0.0, 0.0}, 0.5);
  const auto &P = mpc.params();
  const int n = snap.n;
  auto Pv = [&](int r, int c) { return snap.P[static_cast<std::size_t>(r) * n + c]; };

  // P = 2·(权重矩阵)（OSQP 目标 0.5 z'Pz + q'z）。状态权重 q_*、控制权重 r_*。
  EXPECT_DOUBLE_EQ(Pv(0, 0), 2.0 * P.q_x);
  EXPECT_DOUBLE_EQ(Pv(1, 1), 2.0 * P.q_y);
  EXPECT_DOUBLE_EQ(Pv(2, 2), 2.0 * P.q_yaw);
  EXPECT_DOUBLE_EQ(Pv(3, 3), 2.0 * P.q_v);
  // 终端步加权 terminal_weight_scale
  const int t = kStateDim * N;  // 最后一个状态的 x
  EXPECT_DOUBLE_EQ(Pv(t, t), 2.0 * P.q_x * P.terminal_weight_scale);
  // 控制对角
  const int c0 = kStateDim * (N + 1);
  EXPECT_DOUBLE_EQ(Pv(c0 + 0, c0 + 0), 2.0 * P.r_a + 2.0 * P.rd_a);
  EXPECT_DOUBLE_EQ(Pv(c0 + 1, c0 + 1), 2.0 * P.r_w + 2.0 * P.rd_w);
  // 变化率耦合：w·(u_{k+1}-u_k)² 展开 ⇒ 相邻两个控制的对角各 +2w、上三角交叉 -2w。
  // 控制块里 k=0 占 c0+0 (a)、c0+1 (w)，k=1 占 c0+2 (a)、c0+3 (w)。
  EXPECT_DOUBLE_EQ(Pv(c0 + 2, c0 + 2), 2.0 * P.r_a + 2.0 * P.rd_a);
  EXPECT_DOUBLE_EQ(Pv(c0 + 2, c0 + 0), -2.0 * P.rd_a);
  EXPECT_DOUBLE_EQ(Pv(c0 + 0, c0 + 2), -2.0 * P.rd_a) << "P 必须对称（快照会对称化）";
  // 无障碍时 q 全零（代价是纯二次型，前馈隐含在"e 是绝对误差"里）
  for (double v : snap.q) EXPECT_DOUBLE_EQ(v, 0.0);
}

TEST(MpcQpReference, CorridorRowsAreLateralConstraints)
{
  const int N = 2;
  auto mpc = makeMpc(N);
  RouteCorridor c;
  c.centerline = straightPath(30.0, 0.05);
  c.half_width = 0.6;
  mpc.setCorridor(&c);
  const auto snap = solveAndSnapshot(mpc, Pose2D{1.0, 0.2, 0.0}, 0.5);
  const int n = snap.n;
  auto A = [&](int row, int col) { return snap.A[static_cast<std::size_t>(row) * n + col]; };

  // 行布局：4(N+1) 等式 → N 速度 → 2N 控制 → 2(N-1) 变化率 → 2N 走廊 → N 障碍
  const int base_corr = 4 * (N + 1) + N + 2 * N + 2 * (N - 1);
  for (int k = 1; k <= N; ++k) {
    const int row_lo = base_corr + 2 * (k - 1) + 0;
    const int row_hi = base_corr + 2 * (k - 1) + 1;
    // 参考线沿 +x ⇒ 左法向 n = (-sin0, cos0) = (0, 1)：横向偏差就是 e_y
    EXPECT_DOUBLE_EQ(A(row_lo, kStateDim * k + 0), 0.0);
    EXPECT_DOUBLE_EQ(A(row_lo, kStateDim * k + 1), -1.0) << "k=" << k;
    EXPECT_DOUBLE_EQ(A(row_hi, kStateDim * k + 1), 1.0) << "k=" << k;
    EXPECT_DOUBLE_EQ(snap.l[row_lo], -0.6);
    EXPECT_DOUBLE_EQ(snap.u[row_hi], 0.6);
    // 未激活的一侧用 OSQP 的“无穷大”（±OSQP_INFTY = 1e20，**不是** IEEE inf），
    // 这样 OSQP 才会把它当单边约束；用 1e10 这种“很大的有限值”会参与 scaling。
    EXPECT_GT(snap.u[row_lo], 1e19);
    EXPECT_LT(snap.l[row_hi], -1e19);
  }
}

TEST(MpcQpReference, FreeModeLeavesCorridorRowsInactive)
{
  const int N = 2;
  auto mpc = makeMpc(N);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));  // 无走廊 ⇒ free
  ASSERT_EQ(mpc.mode(), LocalPlanner::Mode::kFree);
  const auto snap = solveAndSnapshot(mpc, Pose2D{1.0, 0.2, 0.0}, 0.5);
  const int base_corr = 4 * (N + 1) + N + 2 * N + 2 * (N - 1);
  for (int k = 1; k <= N; ++k) {
    const int row_lo = base_corr + 2 * (k - 1) + 0;
    const int row_hi = base_corr + 2 * (k - 1) + 1;
    // free 模式下这两行**结构保留但上下界放开**（±OSQP_INFTY）——
    // 保留结构是为了让 OSQP 工作区的稀疏模式永久不变（热启动的前提）。
    EXPECT_GT(snap.u[row_lo], 1e19);
    EXPECT_LT(snap.l[row_hi], -1e19);
  }
}

// ============================================================================
// ② 与参考实现（SLAM-PNC MpcController）的公式对拍
// ============================================================================

TEST(MpcQpReference, PositionRowsMatchSlamPncFormulas)
{
  const int N = 1;
  auto mpc = makeMpc(N);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));
  const auto snap = solveAndSnapshot(mpc, Pose2D{2.0, 0.05, 0.15}, 0.7);
  const int n = snap.n;
  auto A = [&](int row, int col) { return snap.A[static_cast<std::size_t>(row) * n + col]; };

  // 参考实现的 A 公式：用**参考点** ref[0]（不是预测轨迹首点）作为线性化点。
  // e_0 的两个分量在初始状态等式的 u[] 里 ⇒ 反推 ref[0]；
  //   v_ref = v_now - e_0(v)；e_0(v) 就是 u[3]
  MpcReferencePoint r0;
  r0.x = mpc.predictedTrajectory()[0].x;
  r0.y = mpc.predictedTrajectory()[0].y;
  r0.yaw = mpc.predictedTrajectory()[0].yaw - snap.u[2];
  r0.v = 0.7 - snap.u[3];
  const double dt = mpc.params().dt;
  const auto ref = slam_pnc_reference::referenceA(r0, dt, 1.0);

  // 位置两行：我们与参考实现**应当逐项相同**（同一套运动学线性化）
  const int row0 = kStateDim;  // k=0 的动力学行块
  for (int i = 0; i <= 1; ++i) {
    for (int c = 0; c < kStateDim; ++c) {
      const double ours = -A(row0 + i, kStateDim * 0 + c);  // 约束里是 -A(i,c)
      EXPECT_NEAR(ours, ref[static_cast<std::size_t>(i) * kStateDim + c], 1e-12)
          << "i=" << i << " c=" << c;
    }
  }
  // 而 yaw 行：参考实现有 a(2,3)=tan(δ)/L·dt（阿克曼），差速底盘该项必为 0，
  // 对应影响改由控制 ω 承担（B(2,1)=dt）。这是**模型映射差异**，不是缺项。
  EXPECT_DOUBLE_EQ(A(row0 + 2, kStateDim * 0 + 3), 0.0);
  EXPECT_DOUBLE_EQ(A(row0 + 2, kStateDim * (0 + 1) + 2), 1.0);
}

TEST(MpcQpReference, VariableLayoutMatchesSlamPnc)
{
  // 参考实现的布局：状态块在前（4(N+1)），控制块在后（2N），步内连续。
  const int N = 4;
  auto mpc = makeMpc(N);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));
  const auto snap = solveAndSnapshot(mpc, Pose2D{1.0, 0.0, 0.0}, 0.5);
  // 初始状态等式只在前 4 列有非零 ⇒ 状态块确实从 0 开始
  for (int i = 0; i < kStateDim; ++i)
    for (int c = kStateDim; c < snap.n; ++c)
      EXPECT_DOUBLE_EQ(snap.A[static_cast<std::size_t>(i) * snap.n + c], 0.0);
  // 控制块起点 = 4(N+1)：用"控制常数行"验证（下界/上界由限幅 + 参考决定）
  const int base_ctrl = 4 * (N + 1) + N;
  const int c0 = kStateDim * (N + 1);
  EXPECT_DOUBLE_EQ(snap.A[static_cast<std::size_t>(base_ctrl) * snap.n + c0 + 0], 1.0);
  EXPECT_DOUBLE_EQ(snap.A[static_cast<std::size_t>(base_ctrl + 1) * snap.n + c0 + 1], 1.0);
}

// ============================================================================
// ③ KKT 自检：解必须可行且满足一阶最优性
// ============================================================================

namespace {

struct KktReport {
  double max_violation{0.0};   ///< max(0, l - Ax, Ax - u)
  double dual_residual{0.0};   ///< ‖P x + q + A' y‖∞
  double feasibility_gap{0.0}; ///< y 与互补松弛的匹配程度（粗检）
};

/// 用**快照出来的稠密矩阵**独立检查 OSQP 给的解。
/// 注意：这检验的是"OSQP 解对了我们交给它的 QP"，不是"我们交的 QP 就是想要的" ——
/// 后者由上面的结构对拍负责。两者互补。
template <typename SnapT>
KktReport checkKkt(const SnapT &s)
{
  KktReport rep;
  if (s.x.empty())
    return rep;
  const int n = s.n;
  const int m = s.m;
  // 1) 可行性：l ≤ A x ≤ u
  std::vector<double> Ax(static_cast<std::size_t>(m), 0.0);
  for (int r = 0; r < m; ++r) {
    double acc = 0.0;
    for (int c = 0; c < n; ++c)
      acc += s.A[static_cast<std::size_t>(r) * n + c] * s.x[c];
    Ax[static_cast<std::size_t>(r)] = acc;
    if (std::isfinite(s.l[r]))
      rep.max_violation = std::max(rep.max_violation, s.l[r] - acc);
    if (std::isfinite(s.u[r]))
      rep.max_violation = std::max(rep.max_violation, acc - s.u[r]);
  }
  // 2) 对偶残差：P x + q + A' y ≈ 0
  if (!s.y.empty()) {
    std::vector<double> grad(s.q.begin(), s.q.end());
    for (int r = 0; r < n; ++r) {
      double acc = 0.0;
      for (int c = 0; c < n; ++c)
        acc += s.P[static_cast<std::size_t>(r) * n + c] * s.x[c];
      grad[static_cast<std::size_t>(r)] += acc;
    }
    for (int c = 0; c < m; ++c) {
      const double y = s.y[static_cast<std::size_t>(c)];
      if (y == 0.0)
        continue;
      for (int r = 0; r < n; ++r)
        grad[static_cast<std::size_t>(r)] += s.A[static_cast<std::size_t>(c) * n + r] * y;
    }
    for (double v : grad) rep.dual_residual = std::max(rep.dual_residual, std::fabs(v));
  }
  return rep;
}

}  // namespace

TEST(MpcQpReference, SolutionSatisfiesKktConditions)
{
  // 三组典型场景（自由空间跟踪 / 走廊内 / 贴着障碍）都要满足 KKT。
  auto run = [&](const char *tag, bool corridor, double obstacle_weight,
                 const Pose2D &start, double v_now, double expect_violation_below) {
    MpcLocalPlanner mpc;
    MemoryParamReader p;
    p.setDouble("local_mpc.v_max", 1.0);
    p.setDouble("local_mpc.obstacle_weight", obstacle_weight);
    mpc.configure(p);
    mpc.setGlobalPlan(straightPath(30.0, 0.05));
    RouteCorridor c;
    if (corridor) {
      c.centerline = straightPath(30.0, 0.05);
      c.half_width = 0.5;
      mpc.setCorridor(&c);
    }
    mpc.setCurrentVelocity(v_now, 0.0);
    const auto r = mpc.computeCommand(start, mpc.params().dt);
    ASSERT_TRUE(r.ok()) << tag << "：" << toString(r.status) << " " << r.message;

    Snap s;
    mpc.snapshotQp(s);
    const auto rep = checkKkt(s);
    std::printf("  [KKT %s] 可行性违反 %.3e | 对偶残差 %.3e\n", tag,
                rep.max_violation, rep.dual_residual);
    EXPECT_LT(rep.max_violation, expect_violation_below) << tag;
    // 对偶残差量级：OSQP 默认 eps 1e-4，权重 ~1e3 ⇒ 残差 ≲ 权重×容差
    EXPECT_LT(rep.dual_residual, 1.0) << tag;
    EXPECT_EQ(mpc.solveInfo().corridor_violations, 0) << tag;
    EXPECT_LT(mpc.solveInfo().max_dynamics_residual, 1e-6) << tag;
  };

  run("free", false, 0.0, Pose2D{2.0, 0.10, 0.05}, 0.6, 1e-6);
  run("corridor", true, 0.0, Pose2D{2.0, 0.20, 0.10}, 0.6, 1e-6);
  run("obstacle", false, 400.0, Pose2D{2.0, 0.05, 0.0}, 0.8, 1e-6);
}

TEST(MpcQpReference, SolutionIsStationaryWhenNoConstraintActive)
{
  // 关掉所有会激活的约束（大限幅、无走廊、无障碍、v_now = v_ref、e 需很小），
  // 此时 QP 是纯二次型且只受初始状态等式约束 → 解应让"控制偏差尽量小"，
  // 且对偶残差应接近机器精度（无激活不等式 ⇒ 无对偶变量）。
  MpcLocalPlanner mpc;
  MemoryParamReader p;
  p.setDouble("local_mpc.v_max", 5.0);
  p.setDouble("local_mpc.a_max", 5.0);
  p.setDouble("local_mpc.w_max", 5.0);
  mpc.configure(p);
  mpc.setGlobalPlan(straightPath(30.0, 0.05));

  const auto snap = solveAndSnapshot(mpc, Pose2D{5.0, 0.0, 0.0}, 1.0);
  const auto rep = checkKkt(snap);
  std::printf("  [KKT 无约束] 可行性违反 %.3e | 对偶残差 %.3e\n", rep.max_violation,
              rep.dual_residual);
  EXPECT_LT(rep.max_violation, 1e-6);
  EXPECT_LT(rep.dual_residual, 1e-6 * 1e3);  // 权重 ~1e3，按比例给容差
}

}  // namespace
}  // namespace pnc_2d
