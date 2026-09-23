// MPC 局部规划器实现。算法结构与取舍见头文件顶部的长注释；
// 这里主要注释"为什么这么写"以及踩过的坑。

#include "pnc_2d/local/mpc_local_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include <osqp.h>

namespace pnc_2d {
namespace {

constexpr int kStateDim = 4;   // [x, y, yaw, v]
constexpr int kControlDim = 2; // [a, w]
/// OSQP 的"无穷大"：用它而不是 1e10 —— OSQP 内部按 OSQP_INFTY 判定"单边约束"，
/// 给一个"很大的有限值"反而会参与 scaling，把矩阵条件数搞坏。
constexpr double kInf = OSQP_INFTY;
constexpr double kEps = 1e-9;

double clampValue(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

/// 稀疏三元组 → CSC。**结构每周期不变**，所以这里只做一次性建表：
/// 排序后生成 `col_ptr/row_idx`，并保留 `槽位 → CSC 下标` 映射，
/// 之后每周期只写 `values` —— 这是能真正热启动的前提
/// （SLAM-PNC 每周期 new 一个 OsqpEigen::Solver，其实等于每周期冷启动）。
void buildSparseLayout(SparseLayout &out,
                       const std::vector<std::pair<int, int>> &triplets,
                       int rows, int cols,
                       bool require_upper_triangle = false) {
  out.rows = rows;
  out.cols = cols;
  if (require_upper_triangle) {
    // OSQP 只读 P 的**上三角**，并且会校验收到的矩阵确实是上三角（
    // 否则报 "P is not upper triangular" 并 setup 失败）。这类错误很难从
    // 求解结果反推，所以在建表时就把它变成一条明确断言。
    for (const auto &t : triplets) {
      if (t.first > t.second) {
        std::fprintf(
            stderr,
            "[mpc] 内部错误：P 出现下三角项 (%d,%d)。\n"
            "      P 必须按上三角（row <= col）登记：对称项写在 (min,max)。\n",
            t.first, t.second);
        std::abort();
      }
    }
  }
  const int n = static_cast<int>(triplets.size());
  std::vector<int> order(n);
  for (int i = 0; i < n; ++i)
    order[i] = i;
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    if (triplets[a].second != triplets[b].second)
      return triplets[a].second < triplets[b].second;
    return triplets[a].first < triplets[b].first;
  });
  out.col_ptr.assign(cols + 1, 0);
  out.row_idx.assign(n, 0);
  out.slot_to_csc.assign(n, 0);
  out.values.assign(n, 0.0);
  for (int k = 0; k < n; ++k) {
    const int slot = order[k];
    out.slot_to_csc[slot] = k;
    out.row_idx[k] = triplets[slot].first;
    out.col_ptr[triplets[slot].second + 1]++;
  }
  for (int c = 0; c < cols; ++c)
    out.col_ptr[c + 1] += out.col_ptr[c];
}

/// 造一个 OSQP 能用的 csc 结构。
/// 0.6.2 的 osqp_setup 会把数据拷进工作区，但 update_* 需要一直有东西可指，
/// 所以这两个结构体留到工作区释放时再一起放。
csc *makeCsc(const std::vector<double> &x, const std::vector<int> &i,
             const std::vector<int> &p, int m, int n) {
  auto *M = static_cast<csc *>(c_malloc(sizeof(csc)));
  if (M == nullptr)
    return nullptr;
  M->m = m;
  M->n = n;
  M->nz = -1; // 0.6.2 用 -1 表示"已分配但未压缩"
  M->nzmax = static_cast<c_int>(x.size());
  M->x = static_cast<c_float *>(c_malloc(sizeof(c_float) * (x.size() + 1)));
  M->i = static_cast<c_int *>(c_malloc(sizeof(c_int) * (x.size() + 1)));
  M->p = static_cast<c_int *>(c_malloc(sizeof(c_int) * (p.size() + 1)));
  if (!M->x || !M->i || !M->p) {
    if (M->x)
      c_free(M->x);
    if (M->i)
      c_free(M->i);
    if (M->p)
      c_free(M->p);
    c_free(M);
    return nullptr;
  }
  for (std::size_t k = 0; k < x.size(); ++k) {
    M->x[k] = static_cast<c_float>(x[k]);
    M->i[k] = i[k];
  }
  for (std::size_t k = 0; k < p.size(); ++k)
    M->p[k] = p[k];
  return M;
}

void freeCsc(csc *M) {
  if (M == nullptr)
    return;
  c_free(M->x);
  c_free(M->i);
  c_free(M->p);
  c_free(M);
}

} // namespace

MpcLocalPlanner::MpcLocalPlanner() = default;

MpcLocalPlanner::~MpcLocalPlanner() {
  if (osqp_work_) {
    osqp_cleanup(static_cast<OSQPWorkspace *>(osqp_work_));
    osqp_work_ = nullptr;
  }
}

// ============================================================================
// 参数
// ============================================================================
bool MpcLocalPlanner::configure(const ParamReader &params) {
  const std::string k = "local_mpc.";
  auto d = [&](const char *name, double def) {
    return params.getDouble(k + name, def);
  };
  auto i = [&](const char *name, int def) {
    return params.getInt(k + name, def);
  };
  auto b = [&](const char *name, bool def) {
    return params.getBool(k + name, def);
  };

  p_.dt = d("dt", p_.dt);
  p_.horizon = i("horizon", p_.horizon);
  p_.q_x = d("q_x", p_.q_x);
  p_.q_y = d("q_y", p_.q_y);
  p_.q_yaw = d("q_yaw", p_.q_yaw);
  p_.q_v = d("q_v", p_.q_v);
  p_.r_a = d("r_a", p_.r_a);
  p_.r_w = d("r_w", p_.r_w);
  p_.rd_a = d("rd_a", p_.rd_a);
  p_.rd_w = d("rd_w", p_.rd_w);
  p_.terminal_weight_scale =
      d("terminal_weight_scale", p_.terminal_weight_scale);
  p_.v_max = d("v_max", p_.v_max);
  p_.v_min = d("v_min", p_.v_min);
  p_.a_max = d("a_max", p_.a_max);
  p_.w_max = d("w_max", p_.w_max);
  p_.alpha_max = d("alpha_max", p_.alpha_max);
  p_.constrain_control_rate =
      b("constrain_control_rate", p_.constrain_control_rate);
  p_.reference_speed = d("reference_speed", p_.reference_speed);
  p_.lat_acc_max = d("lat_acc_max", p_.lat_acc_max);
  p_.brake_acc = d("brake_acc", p_.brake_acc);
  p_.approach_dist = d("approach_dist", p_.approach_dist);
  p_.approach_speed = d("approach_speed", p_.approach_speed);
  p_.crawl_speed = d("crawl_speed", p_.crawl_speed);
  p_.stop_coast = d("stop_coast", p_.stop_coast);
  p_.align_in_place_deg = d("align_in_place_deg", p_.align_in_place_deg);
  p_.align_exit_deg = d("align_exit_deg", p_.align_exit_deg);
  p_.align_gain = d("align_gain", p_.align_gain);
  p_.align_min_clearance = d("align_min_clearance", p_.align_min_clearance);
  p_.goal_yaw_tolerance_deg =
      d("goal_yaw_tolerance_deg", p_.goal_yaw_tolerance_deg);
  p_.goal_yaw_align_distance =
      d("goal_yaw_align_distance", p_.goal_yaw_align_distance);
  p_.align_w_min = d("align_w_min", p_.align_w_min);
  p_.goal_yaw_align_timeout =
      d("goal_yaw_align_timeout", p_.goal_yaw_align_timeout);
  p_.free_lat_scale = d("free_lat_scale", p_.free_lat_scale);
  p_.free_yaw_scale = d("free_yaw_scale", p_.free_yaw_scale);
  p_.free_lat_deadband = d("free_lat_deadband", p_.free_lat_deadband);
  p_.free_w_max = d("free_w_max", p_.free_w_max);
  if (p_.goal_yaw_tolerance_deg < 0.0)
    p_.goal_yaw_tolerance_deg = 0.0;
  if (p_.goal_yaw_align_distance < 0.0)
    p_.goal_yaw_align_distance = 0.0;
  if (p_.free_lat_scale < 0.0)
    p_.free_lat_scale = 0.0;
  if (p_.free_yaw_scale < 0.0)
    p_.free_yaw_scale = 0.0;
  if (p_.free_lat_deadband < 0.0)
    p_.free_lat_deadband = 0.0;
  if (p_.align_w_min < 0.0)
    p_.align_w_min = 0.0;
  p_.curvature_lookahead = d("curvature_lookahead", p_.curvature_lookahead);
  p_.back_window = d("back_window", p_.back_window);
  p_.corridor_min_tolerance =
      d("corridor_min_tolerance", p_.corridor_min_tolerance);
  p_.obstacle_weight = d("obstacle_weight", p_.obstacle_weight);
  p_.obstacle_safe_distance =
      d("obstacle_safe_distance", p_.obstacle_safe_distance);
  p_.obstacle_hard_distance =
      d("obstacle_hard_distance", p_.obstacle_hard_distance);
  p_.obstacle_hard_enable = b("obstacle_hard_enable", p_.obstacle_hard_enable);
  p_.degraded_speed_limit = d("degraded_speed_limit", p_.degraded_speed_limit);
  p_.osqp_max_iter = i("osqp_max_iter", p_.osqp_max_iter);
  p_.osqp_eps_abs = d("osqp_eps_abs", p_.osqp_eps_abs);
  p_.osqp_eps_rel = d("osqp_eps_rel", p_.osqp_eps_rel);

  // ---- 合法性兜底：坏参数必须在"配置"这一步挡住，不能带进每周期求解 ----
  if (p_.dt <= 0.0) {
    p_.dt = 0.1;
  }
  p_.horizon = std::max(1, std::min(p_.horizon, 100));
  if (p_.v_max <= 0.0)
    p_.v_max = 1.0;
  p_.v_min = clampValue(p_.v_min, 0.0, p_.v_max); // 不倒车
  p_.a_max = std::fabs(p_.a_max) > kEps ? std::fabs(p_.a_max) : 1.0;
  p_.w_max = std::fabs(p_.w_max) > kEps ? std::fabs(p_.w_max) : 0.8;
  p_.alpha_max = std::fabs(p_.alpha_max) > kEps ? std::fabs(p_.alpha_max) : 1.5;
  p_.obstacle_weight = std::max(0.0, p_.obstacle_weight);
  p_.obstacle_hard_distance = std::max(0.0, p_.obstacle_hard_distance);
  p_.obstacle_safe_distance =
      std::max(p_.obstacle_safe_distance, p_.obstacle_hard_distance);
  p_.terminal_weight_scale = std::max(0.0, p_.terminal_weight_scale);

  // ---- 稀疏结构定死（此后每周期只改数值）----
  const int N = p_.horizon;
  const int state_count = N + 1;
  const int control_count = N;
  const int n_var = kStateDim * state_count + kControlDim * control_count;
  // 障碍硬下界的每步参考净距（仅诊断）：默认 +inf（= 没开启/没有场/离得远）
  obs_ref_dist_.assign(static_cast<std::size_t>(state_count),
                       std::numeric_limits<double>::infinity());

  const int base_vel = kStateDim * state_count;
  const int base_ctrl = base_vel + N;
  const int base_rate = base_ctrl + kControlDim * N;
  const int base_corr = base_rate + kControlDim * std::max(0, N - 1);
  const int base_obs = base_corr + 2 * N;
  const int n_con = base_obs + N;

SparseBuilder_unused:;
  std::vector<std::pair<int, int>> p_tri, a_tri;
  auto padd = [&](int r, int c) {
    p_tri.emplace_back(r, c);
    return static_cast<int>(p_tri.size()) - 1;
  };
  auto aadd = [&](int r, int c) {
    a_tri.emplace_back(r, c);
    return static_cast<int>(a_tri.size()) - 1;
  };
  // ---- P：状态/控制的对角 + 相邻控制的耦合项（Δu 平滑）----
  // 结构里额外留出“位置 2×2 块”的交叉项 (x,y)：障碍软代价是 rank-1 的
  // w·(∇d·e)²，展开后正好是 2w·[gx², gxgy; gxgy, gy²]。P 只要上三角，
  // 所以登记 (x,x),(x,y),(y,y) 三项。
  std::vector<int> p_slot_state(state_count * kStateDim, -1);
  std::vector<int> p_slot_xy(state_count, -1);
  for (int kk = 0; kk < state_count; ++kk) {
    p_slot_state[kk * kStateDim + 0] =
        padd(stateIndex(kk, 0), stateIndex(kk, 0));
    p_slot_xy[kk] = padd(stateIndex(kk, 0), stateIndex(kk, 1));
    p_slot_state[kk * kStateDim + 1] =
        padd(stateIndex(kk, 1), stateIndex(kk, 1));
    p_slot_state[kk * kStateDim + 2] =
        padd(stateIndex(kk, 2), stateIndex(kk, 2));
    p_slot_state[kk * kStateDim + 3] =
        padd(stateIndex(kk, 3), stateIndex(kk, 3));
  }
  std::vector<int> p_slot_ctrl(control_count * kControlDim, -1);
  for (int kk = 0; kk < control_count; ++kk)
    for (int j = 0; j < kControlDim; ++j)
      p_slot_ctrl[kk * kControlDim + j] =
          padd(controlIndex(kk, j), controlIndex(kk, j));
  // 变化率：w·(u_{k+1} - u_k)² 的上三角项在 (k, k+1)——**必须按上三角登记**。
  std::vector<int> p_slot_rate(std::max(0, N - 1) * kControlDim, -1);
  for (int kk = 0; kk + 1 < control_count; ++kk)
    for (int j = 0; j < kControlDim; ++j)
      p_slot_rate[kk * kControlDim + j] =
          padd(controlIndex(kk, j), controlIndex(kk + 1, j));

  // ---- A：等式（初始状态 + 动力学）+ 速度 + 控制 + 变化率 + 走廊 + 障碍 ----
  std::vector<int> a_slot_eq0(state_count * kStateDim, -1);
  std::vector<int> a_slot_next(state_count * kStateDim, -1); // e_{k+1}(i)
  std::vector<int> a_slot_diag(state_count * kStateDim,
                               -1); // -e_k(i)（A 的单位阵部分）
  std::vector<int> a_slot_a(state_count * kStateDim * 4,
                            -1); // A_k 的 4 个结构项
  std::vector<int> a_slot_b(state_count * kStateDim * 2,
                            -1); // B_k 的 2 个结构项
  for (int ii = 0; ii < kStateDim; ++ii)
    a_slot_eq0[ii] = aadd(ii, stateIndex(0, ii));
  for (int kk = 0; kk + 1 < state_count; ++kk) {
    for (int ii = 0; ii < kStateDim; ++ii) {
      const int row = kStateDim + kStateDim * kk + ii;
      a_slot_next[kk * kStateDim + ii] = aadd(row, stateIndex(kk + 1, ii));
      // ⚠ 别忘了 A 的**单位阵部分**（系数 -1）：A = I + dt·∂f/∂s，约束是
      //   e_{k+1} - A e_k - B Δu_k = 0，漏了这一项等于把动力学约束放开了
      //   （表现为"预测轨迹瞬移"，而求解器还报成功）。
      a_slot_diag[kk * kStateDim + ii] = aadd(row, stateIndex(kk, ii));
      // 运动学线性化里结构上非零的偏导：位置对 (yaw, v)、yaw 对控制 w、v 对控制
      // a
      if (ii == 0) {
        a_slot_a[(kk * kStateDim + ii) * 4 + 0] = aadd(row, stateIndex(kk, 2));
        a_slot_a[(kk * kStateDim + ii) * 4 + 1] = aadd(row, stateIndex(kk, 3));
      } else if (ii == 1) {
        a_slot_a[(kk * kStateDim + ii) * 4 + 0] = aadd(row, stateIndex(kk, 2));
        a_slot_a[(kk * kStateDim + ii) * 4 + 1] = aadd(row, stateIndex(kk, 3));
      } else if (ii == 2) {
        a_slot_b[(kk * kStateDim + ii) * 2 + 1] =
            aadd(row, controlIndex(kk, 1));
      } else {
        a_slot_b[(kk * kStateDim + ii) * 2 + 0] =
            aadd(row, controlIndex(kk, 0));
      }
    }
  }
  std::vector<int> a_slot_vel(N, -1);
  for (int kk = 1; kk <= N; ++kk)
    a_slot_vel[kk - 1] = aadd(base_vel + kk - 1, stateIndex(kk, 3));
  std::vector<int> a_slot_ctrl(control_count * kControlDim, -1);
  for (int kk = 0; kk < control_count; ++kk)
    for (int j = 0; j < kControlDim; ++j)
      a_slot_ctrl[kk * kControlDim + j] =
          aadd(base_ctrl + kControlDim * kk + j, controlIndex(kk, j));
  std::vector<int> a_slot_rate(std::max(0, N - 1) * kControlDim * 2, -1);
  for (int kk = 0; kk + 1 < control_count; ++kk)
    for (int j = 0; j < kControlDim; ++j) {
      const int row = base_rate + kControlDim * kk + j;
      a_slot_rate[(kk * kControlDim + j) * 2 + 0] =
          aadd(row, controlIndex(kk + 1, j));
      a_slot_rate[(kk * kControlDim + j) * 2 + 1] =
          aadd(row, controlIndex(kk, j));
    }
  std::vector<int> a_slot_corr(2 * N * 2, -1);
  for (int kk = 1; kk <= N; ++kk)
    for (int side = 0; side < 2; ++side) {
      const int row = base_corr + 2 * (kk - 1) + side;
      a_slot_corr[((kk - 1) * 2 + side) * 2 + 0] = aadd(row, stateIndex(kk, 0));
      a_slot_corr[((kk - 1) * 2 + side) * 2 + 1] = aadd(row, stateIndex(kk, 1));
    }
  std::vector<int> a_slot_obs(N * 2, -1);
  for (int kk = 1; kk <= N; ++kk) {
    const int row = base_obs + (kk - 1);
    a_slot_obs[(kk - 1) * 2 + 0] = aadd(row, stateIndex(kk, 0));
    a_slot_obs[(kk - 1) * 2 + 1] = aadd(row, stateIndex(kk, 1));
  }

  // 结构布局固定，存进成员（之后每周期只写 values()）
  buildSparseLayout(p_layout_, p_tri, n_var, n_var,
                    /*require_upper_triangle=*/true);
  buildSparseLayout(a_layout_, a_tri, n_con, n_var);
  p_slot_state_ = std::move(p_slot_state);
  p_slot_xy_ = std::move(p_slot_xy);
  p_slot_ctrl_ = std::move(p_slot_ctrl);
  p_slot_rate_ = std::move(p_slot_rate);
  a_slot_eq0_ = std::move(a_slot_eq0);
  a_slot_next_ = std::move(a_slot_next);
  a_slot_diag_ = std::move(a_slot_diag);
  a_slot_a_ = std::move(a_slot_a);
  a_slot_b_ = std::move(a_slot_b);
  a_slot_vel_ = std::move(a_slot_vel);
  a_slot_ctrl_ = std::move(a_slot_ctrl);
  a_slot_rate_ = std::move(a_slot_rate);
  a_slot_corr_ = std::move(a_slot_corr);
  a_slot_obs_ = std::move(a_slot_obs);
  row_base_.base_vel = base_vel;
  row_base_.base_ctrl = base_ctrl;
  row_base_.base_rate = base_rate;
  row_base_.base_corr = base_corr;
  row_base_.base_obs = base_obs;

  q_val_.assign(n_var, 0.0);
  l_val_.assign(n_con, -kInf);
  u_val_.assign(n_con, kInf);
  info_.qp_variables = n_var;
  info_.qp_constraints = n_con;

  // 结构变了（horizon 变）⇒ 旧工作区不能再用
  if (osqp_work_) {
    osqp_cleanup(static_cast<OSQPWorkspace *>(osqp_work_));
    osqp_work_ = nullptr;
  }
  if (osqp_P_) {
    freeCsc(static_cast<csc *>(osqp_P_));
    osqp_P_ = nullptr;
  }
  if (osqp_A_) {
    freeCsc(static_cast<csc *>(osqp_A_));
    osqp_A_ = nullptr;
  }
  return true;
}
// ============================================================================
// 输入
// ============================================================================
void MpcLocalPlanner::setGlobalPlan(const std::vector<Pose2D> &path) {
  plan_ = path;
  rebuildReference();
}

void MpcLocalPlanner::setCorridor(const RouteCorridor *corridor) {
  corridor_ = corridor;
  // 模式判定：**由数据决定**（非空且有效 = route），不是配置项（决策 D2）
  route_mode_ = (corridor != nullptr && corridor->valid());
  rebuildReference();
}

void MpcLocalPlanner::setSpeedLimit(double v_limit) { speed_limit_ = v_limit; }

void MpcLocalPlanner::setCostMap(
    std::shared_ptr<const CostMap2D> local_inflated) {
  local_map_ = std::move(local_inflated);
}

void MpcLocalPlanner::setDistanceField(const LocalDistanceField *field) {
  dist_field_ = field;
}

void MpcLocalPlanner::setDynamicObstacles(
    const std::vector<DynamicObstacle> &obs) {
  // v1 只做反应式（决策 D6）：动态障碍的**预测**留到 §7 的未来优化点。
  // 接口先留着，这样将来加"点云聚类 + CV-KF"时不用改接口。
  dynamic_obs_ = obs;
}

void MpcLocalPlanner::reset() {
  progress_s_ = 0.0;
  has_progress_ = false;
  predicted_.clear();
  smooth_v_ = smooth_w_ = 0.0;
  aligning_ = false;
  goal_aligning_ = false;
  goal_align_best_deg_ = 1e9;   // 新任务重新计“无进展超时”
  info_ = MpcSolveInfo{};
}

// ============================================================================
// 参考轨迹：弧长参数化 + 速度剖面
// ============================================================================
void MpcLocalPlanner::rebuildReference() {
  // route 模式下**走廊中心线就是参考线**（D2：走廊本身就代表"我在车道里"）；
  // 否则用全局路径。
  const std::vector<Pose2D> *src = nullptr;
  if (route_mode_ && corridor_ != nullptr && corridor_->valid())
    src = &corridor_->centerline;
  else if (plan_.size() >= 2)
    src = &plan_;

  ref_x_.clear();
  ref_y_.clear();
  ref_s_.clear();
  ref_curv_.clear();
  ref_yaw_.clear();
  ref_length_ = 0.0;
  has_progress_ = false;
  progress_s_ = 0.0;
  if (src == nullptr)
    return;

  // 去重：连续重复点（全局路径偶尔会有）会让切线/曲率出现 0/0
  std::vector<Pose2D> pts;
  for (const auto &p : *src) {
    if (!pts.empty() &&
        std::hypot(p.x - pts.back().x, p.y - pts.back().y) < 1e-6)
      continue;
    pts.push_back(p);
  }
  if (pts.size() < 2)
    return;

  const std::size_t n = pts.size();
  ref_x_.reserve(n);
  ref_y_.reserve(n);
  ref_s_.resize(n, 0.0);
  ref_curv_.assign(n, 0.0);
  ref_yaw_.assign(n, 0.0);

  for (std::size_t i = 0; i < n; ++i) {
    ref_x_.push_back(pts[i].x);
    ref_y_.push_back(pts[i].y);
    if (i > 0)
      ref_s_[i] = ref_s_[i - 1] +
                  std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  ref_length_ = ref_s_.back();
  if (ref_length_ < 1e-6)
    return;

  // 切线朝向：**必须用足够长的基线**。相邻两点可能只差几毫米（全局路径起点与
  // 通道起点重合时就是这样），此时 atan2(dy,dx) 的方向完全由数值噪声决定。
  // 实测（2026-09-22 仿真）：一条 3 点路径的前两点相距 ~1e-5 m，算出的切线朝向
  // 差 ~90°，直接导致 e_ψ(0) ≈ −97°、加速就撞走廊、MPC **趴窝不动**（贴线模式完
  // 全跑不起来，而日志只显示 cmd v=0，看不出原因）。
  // 做法：点 i 的朝向用"离 i 前后各至少 min_span 的第一个点"来定；
  // 整条路径比 min_span 还短时退化成首尾方向。
  const double min_span = 0.05; // 5 cm：远大于定位/路径分辨率噪声
  auto yawAt = [&](std::size_t i) -> double {
    std::size_t a = i;
    while (a + 1 < n && ref_s_[a] < ref_s_[i] + min_span)
      ++a;
    std::size_t b = i;
    while (b > 0 && ref_s_[b] > ref_s_[i] - min_span)
      --b;
    const double dx = ref_x_[a] - ref_x_[b];
    const double dy = ref_y_[a] - ref_y_[b];
    if (std::hypot(dx, dy) > 1e-9)
      return std::atan2(dy, dx);
    return std::atan2(ref_y_[n - 1] - ref_y_[0], ref_x_[n - 1] - ref_x_[0]);
  };
  for (std::size_t i = 0; i < n; ++i)
    ref_yaw_[i] = wrapAngle(yawAt(i));

  // 带符号曲率：κ = Δθ / Δs（Δθ 是相邻两段方向的转角，Δs 取两段平均弧长）
  // **符号很重要**：ω_ref = κ·v，反了方向车就往弯道外侧走。
  // 同样用 min_span 基线，避免毫米级相邻点把 κ 算成天文数字（实测 0.546 1/m）。
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const double ds = ref_s_[i + 1] - ref_s_[i - 1];
    if (ds > 1e-6)
      ref_curv_[i] = wrapAngle(yawAt(i + 1) - yawAt(i - 1)) / ds;
  }
}

double MpcLocalPlanner::projectOntoReference(double x, double y,
                                             double &cross_track) const {
  cross_track = 0.0;
  const std::size_t n = ref_s_.size();
  if (n < 2)
    return 0.0;

  // 搜索窗口：从上次进度往回 back_window、往前 5 m（≈ N·dt·v_max 的三倍余量）。
  // 不全局搜索的理由：路径可能自交（巡检路线），全局最近点会让进度**跳段**。
  const double s_lo = has_progress_ ? progress_s_ - p_.back_window : 0.0;
  const double s_hi = has_progress_ ? progress_s_ + 5.0 : ref_length_;
  const std::size_t i0 = static_cast<std::size_t>(
      std::max(0.0, std::lower_bound(ref_s_.begin(), ref_s_.end(), s_lo) -
                        ref_s_.begin() - 1.0));
  const std::size_t i1 = static_cast<std::size_t>(std::min<double>(
      static_cast<double>(n - 2),
      std::upper_bound(ref_s_.begin(), ref_s_.end(), s_hi) - ref_s_.begin()));

  double best_s = ref_s_[i0];
  double best_d2 = std::numeric_limits<double>::max();
  double best_lat = 0.0;
  for (std::size_t i = i0; i <= i1 && i + 1 < n; ++i) {
    const double ax = ref_x_[i];
    const double ay = ref_y_[i];
    const double bx = ref_x_[i + 1];
    const double by = ref_y_[i + 1];
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12)
      continue;
    double t = ((x - ax) * dx + (y - ay) * dy) / len2;
    t = clampValue(t, 0.0, 1.0);
    const double px = ax + t * dx;
    const double py = ay + t * dy;
    const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = ref_s_[i] + t * (ref_s_[i + 1] - ref_s_[i]);
      // 左法向：侧向偏差 > 0 表示车在参考线左侧
      const double yaw = std::atan2(dy, dx);
      best_lat = -(x - px) * std::sin(yaw) + (y - py) * std::cos(yaw);
    }
  }
  cross_track = best_lat;
  // 进度**不回退超过 back_window**（防抖）：投影抖动会让 v_ref 忽大忽小
  if (has_progress_)
    best_s = std::max(best_s, progress_s_ - p_.back_window);
  return clampValue(best_s, 0.0, ref_length_);
}

double MpcLocalPlanner::speedLimitAt(double s) const {
  double v = p_.v_max;
  if (speed_limit_ > 0.0)
    v = std::min(v, speed_limit_);
  if (zone_speed_limit_ > 0.0)
    v = std::min(v, zone_speed_limit_); // 限速区（节点每周期前瞻算出来）
  if (p_.reference_speed > 0.0)
    v = std::min(v, p_.reference_speed);

  // 曲率限速：向心加速度 a_lat = v²·κ ≤ a_lat_max ⇒ v ≤ sqrt(a_lat_max / κ)
  // 用**前瞻段内最大曲率**：只在弯道入口才开始减速就来不及了。
  double curv = 0.0;
  const double s_look = s + p_.curvature_lookahead;
  for (std::size_t i = 0; i < ref_s_.size(); ++i) {
    if (ref_s_[i] < s)
      continue;
    if (ref_s_[i] > s_look)
      break;
    curv = std::max(curv, std::fabs(ref_curv_[i]));
  }
  if (curv > 1e-6)
    v = std::min(v, std::sqrt(p_.lat_acc_max / curv));

  // 终点制动剖面：v ≤ sqrt(2·a_brake·剩余弧长)。远距离时它大于
  // v_max，自然不生效； 越接近终点越收紧，到终点正好为 0 ——
  // 这样"到点停车"不靠硬刹，而是参考本身就减速。
  //
  // ★ 剩余弧长要扣掉 stop_coast：那是"指令归零后底盘还会自己走的距离"，让惯性替
  //   我们把最后几厘米走完，指令就能提前归零（否则一定冲过目标）。
  const double s_rem = std::max(0.0, ref_length_ - s);
  const double s_eff = std::max(0.0, s_rem - p_.stop_coast);
  v = std::min(v, std::sqrt(2.0 * p_.brake_acc * s_eff));

  // ★ 终点段：慢速贴拢 + 保底爬行（到点精度就靠这两行）
  //   只靠制动剖面不行 —— 它在最后 `v²/(2a)` 米（≈1 cm）才把参考压到
  //   0，而底盘的 步态要 ~1 s 才停住、其间又走 3~5 cm（实测）。所以：
  //     · approach_speed 把终点段整体压慢 ⇒ 下停止命令时速度已经很低；
  //     · crawl_speed 保证参考**在惯性段之前不会归零**（否则车会差几厘米停住、
  //       完成判据永远不满足 ⇒ 任务卡死等着超时）。
  if (p_.approach_dist > 0.0 && p_.approach_speed > 0.0 &&
      s_rem <= p_.approach_dist) {
    v = std::min(v, p_.approach_speed);
    if (p_.crawl_speed > 0.0 && s_rem > p_.stop_coast)
      v = std::max(v, std::min(p_.crawl_speed, p_.approach_speed));
  }

  // 降级（缺距离场）：只看硬判定走路，必须慢
  if (degraded())
    v = std::min(v, p_.degraded_speed_limit);

  return std::max(0.0, v);
}

MpcReferencePoint MpcLocalPlanner::sampleAt(double s,
                                            double /*lookahead*/) const {
  MpcReferencePoint r;
  const std::size_t n = ref_s_.size();
  if (n == 0)
    return r;
  s = clampValue(s, 0.0, ref_length_);
  // 定位到包含 s 的段
  std::size_t i = static_cast<std::size_t>(
      std::upper_bound(ref_s_.begin(), ref_s_.end(), s) - ref_s_.begin());
  if (i > 0)
    --i;
  if (i + 1 >= n)
    i = n - 2;
  const double ds = ref_s_[i + 1] - ref_s_[i];
  const double t = ds > 1e-9 ? (s - ref_s_[i]) / ds : 0.0;

  r.x = ref_x_[i] + t * (ref_x_[i + 1] - ref_x_[i]);
  r.y = ref_y_[i] + t * (ref_y_[i + 1] - ref_y_[i]);
  r.yaw = wrapAngle(ref_yaw_[i] + t * wrapAngle(ref_yaw_[i + 1] - ref_yaw_[i]));
  r.s = s;
  r.v = speedLimitAt(s);
  // κ 取该点处（线性插值）的曲率 → ω_ref = κ·v
  const double kappa = ref_curv_[i] + t * (ref_curv_[i + 1] - ref_curv_[i]);
  r.w = clampValue(kappa * r.v, -p_.w_max, p_.w_max);
  return r;
}

bool MpcLocalPlanner::buildReferenceWindow(
    double s0, std::vector<MpcReferencePoint> &window) const {
  if (ref_s_.size() < 2 || ref_length_ < 1e-6)
    return false;

  const int N = p_.horizon;
  window.clear();
  window.reserve(static_cast<std::size_t>(N + 1));

  // 按"参考车以 v_ref 前进"的方式推进弧长（梯形积分）。
  // 注意这里用的是**参考速度**而不是当前速度：参考窗口是"目标状态序列"，
  // 它按计划走，车去追它 —— 这样速度规划与位置跟踪解耦（见头文件说明）。
  double s = s0;
  for (int k = 0; k <= N; ++k) {
    MpcReferencePoint rp = sampleAt(s, p_.curvature_lookahead);
    window.push_back(rp);
    if (k == N)
      break;
    const double v_next = speedLimitAt(s + std::max(rp.v, 0.05) * p_.dt);
    s += 0.5 * (rp.v + v_next) * p_.dt;
    if (s > ref_length_)
      s = ref_length_;
  }
  // 参考加速度：a_k = (v_{k+1} -
  // v_k)/dt（参考轨迹自己的纵向加速度，进约束与代价）
  for (int k = 0; k < N; ++k) {
    window[k].a = clampValue((window[k + 1].v - window[k].v) / p_.dt,
                             p_.a_min(), p_.a_max);
  }
  window[N].a = 0.0;
  return window.size() == static_cast<std::size_t>(N + 1);
}

// ============================================================================
// QP：建数值 → 解
// ============================================================================
bool MpcLocalPlanner::buildQp(const std::vector<MpcReferencePoint> &ref,
                              const Pose2D &pose, double current_v,
                              bool route_mode, double half_width) {
  const int N = p_.horizon;
  const int state_count = N + 1;
  const int control_count = N;
  // 角速度上限：自由模式被 free_w_max 压（约束与指令用同一个值，见 wMaxEff）
  const double w_lim = wMaxEff(route_mode);

  // 当前状态 s = [x, y, θ, v]；误差 e = s - s_ref（**世界系**绝对误差）
  const double e_init[kStateDim] = {pose.x - ref[0].x, pose.y - ref[0].y,
                                    wrapAngle(pose.yaw - ref[0].yaw),
                                    current_v - ref[0].v};

  p_layout_.zeroValues();
  a_layout_.zeroValues();
  std::fill(q_val_.begin(), q_val_.end(), 0.0);
  std::fill(l_val_.begin(), l_val_.end(), -kInf);
  std::fill(u_val_.begin(), u_val_.end(), kInf);

  auto &pv = p_layout_.values;
  auto &av = a_layout_.values;

  // ---------------- P 与 q ----------------
  //
  // ★ 跟踪代价拆成"纵向/横向"两个方向（旋转帧），而不是世界系的 diag(q_x,q_y)：
  //   ① 物理意义对：横向偏差才需要转方向盘，纵向偏差只需加减速；
  //   ② 自由模式可以**单独**放松横向（`free_lat_scale`）而不影响纵向进度；
  //   ③ 走廊模式完全不变（scale = 1）—— 严格贴线仍然跟得紧。
  //   P_xy = 2·(q_lon·t tᵀ + q_lat·n nᵀ)，t = 参考切线、n = 左法向。
  const double lat_scale = route_mode ? 1.0 : p_.free_lat_scale;
  const double yaw_scale = route_mode ? 1.0 : p_.free_yaw_scale;
  for (int k = 0; k < state_count; ++k) {
    // 终端状态加权：让"最后一步偏差"更贵，直接改善到点精度
    const double scale = (k == N) ? p_.terminal_weight_scale : 1.0;
    const double th = ref[k].yaw;
    const double tx = std::cos(th);
    const double ty = std::sin(th);
    const double nx = -std::sin(th);
    const double ny = std::cos(th);
    const double q_lon = p_.q_x;
    const double q_lat = p_.q_y * lat_scale;
    pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 0])] =
        2.0 * scale * (q_lon * tx * tx + q_lat * nx * nx);
    pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 1])] =
        2.0 * scale * (q_lon * ty * ty + q_lat * ny * ny);
    pv[p_layout_.cscIndex(p_slot_xy_[k])] =
        2.0 * scale * (q_lon * tx * ty + q_lat * nx * ny);
    pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 2])] =
        2.0 * scale * p_.q_yaw * yaw_scale;
    pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 3])] =
        2.0 * scale * p_.q_v;
  }
  const double rs[kControlDim] = {p_.r_a, p_.r_w};
  for (int k = 0; k < control_count; ++k)
    for (int j = 0; j < kControlDim; ++j)
      pv[p_layout_.cscIndex(p_slot_ctrl_[k * kControlDim + j])] = 2.0 * rs[j];
  const double rds[kControlDim] = {p_.rd_a, p_.rd_w};
  for (int k = 0; k + 1 < control_count; ++k)
    for (int j = 0; j < kControlDim; ++j) {
      const double w = 2.0 * rds[j];
      pv[p_layout_.cscIndex(p_slot_ctrl_[k * kControlDim + j])] += w;
      pv[p_layout_.cscIndex(p_slot_ctrl_[(k + 1) * kControlDim + j])] += w;
      pv[p_layout_.cscIndex(p_slot_rate_[k * kControlDim + j])] = -w;
    }

  // ---------------- 障碍软代价（rank-1 二次 + 线性） ----------------
  //
  // 把 d(p_k) 在参考点线性化：d ≈ d_ref + ∇d·e。
  // 惩罚 w·(r_safe - d)² 在 d_ref < r_safe 时展开：
  //     w·(r_safe - d_ref)² - 2w·(r_safe - d_ref)·(∇d·e) + w·(∇d·e)²
  // 常数项丢掉（不影响最优解），剩下线性项 + 半正定的 rank-1 二次项 w·gg'。
  // 这样"离障碍越近推得越狠"是**二次增长**的（线性项只在被激活时起作用），
  // 且始终保持凸性 —— 直接上非凸障碍约束会把 QP 搞成不可解。
  const bool have_field = (dist_field_ != nullptr && dist_field_->valid());
  double d_ref[kStateDim] = {0}; // 占位，避免与下面的数组混用
  (void)d_ref;
  std::vector<double> obs_d(state_count, kInf), obs_gx(state_count, 0.0),
      obs_gy(state_count, 0.0);
  if (have_field && p_.obstacle_weight > 0.0) {
    for (int k = 1; k <= N; ++k) {
      obs_d[k] = dist_field_->distance(ref[k].x, ref[k].y);
      if (obs_d[k] >= p_.obstacle_safe_distance)
        continue;
      double gx = 0.0;
      double gy = 0.0;
      if (!dist_field_->gradient(ref[k].x, ref[k].y, gx, gy)) {
        // 梯度退化（开阔区/障碍内部）：没有可用方向，跳过软代价（硬下界仍生效）
        obs_d[k] = kInf;
        continue;
      }
      obs_gx[k] = gx;
      obs_gy[k] = gy;
      const double pen = p_.obstacle_safe_distance - obs_d[k];
      const double w = p_.obstacle_weight;
      const int sx = stateIndex(k, 0);
      const int sy = stateIndex(k, 1);
      pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 0])] +=
          2.0 * w * gx * gx;
      pv[p_layout_.cscIndex(p_slot_state_[k * kStateDim + 1])] +=
          2.0 * w * gy * gy;
      pv[p_layout_.cscIndex(p_slot_xy_[k])] += 2.0 * w * gx * gy;
      q_val_[sx] += -2.0 * w * pen * gx;
      q_val_[sy] += -2.0 * w * pen * gy;
    }
  }

  // ---------------- A 与 l/u ----------------
  // (1) 初始状态等式 e_0 = e_init
  for (int i = 0; i < kStateDim; ++i) {
    av[a_layout_.cscIndex(a_slot_eq0_[i])] = 1.0;
    l_val_[i] = e_init[i];
    u_val_[i] = e_init[i];
  }

  // (2) 动力学：e_{k+1} = A_k e_k + B_k Δu_k
  for (int k = 0; k + 1 < state_count; ++k) {
    const MpcReferencePoint &r = ref[k];
    // A_k = I + dt·∂f/∂s（绕参考点线性化）
    const double a02 = -r.v * std::sin(r.yaw) * p_.dt;
    const double a03 = std::cos(r.yaw) * p_.dt;
    const double a12 = r.v * std::cos(r.yaw) * p_.dt;
    const double a13 = std::sin(r.yaw) * p_.dt;
    // B_k = dt·∂f/∂u：θ̇ = ω ⇒ B(2,1) = dt；v̇ = a ⇒ B(3,0) = dt
    const double b21 = p_.dt;
    const double b30 = p_.dt;
    for (int i = 0; i < kStateDim; ++i) {
      const int row = kStateDim + kStateDim * k + i;
      av[a_layout_.cscIndex(a_slot_next_[k * kStateDim + i])] = 1.0;
      av[a_layout_.cscIndex(a_slot_diag_[k * kStateDim + i])] =
          -1.0; // A = I + ...
      if (i == 0) {
        av[a_layout_.cscIndex(a_slot_a_[(k * kStateDim + i) * 4 + 0])] = -a02;
        av[a_layout_.cscIndex(a_slot_a_[(k * kStateDim + i) * 4 + 1])] = -a03;
      } else if (i == 1) {
        av[a_layout_.cscIndex(a_slot_a_[(k * kStateDim + i) * 4 + 0])] = -a12;
        av[a_layout_.cscIndex(a_slot_a_[(k * kStateDim + i) * 4 + 1])] = -a13;
      } else if (i == 2) {
        av[a_layout_.cscIndex(a_slot_b_[(k * kStateDim + i) * 2 + 1])] = -b21;
      } else {
        av[a_layout_.cscIndex(a_slot_b_[(k * kStateDim + i) * 2 + 0])] = -b30;
      }
      l_val_[row] = 0.0;
      u_val_[row] = 0.0;
    }
  }

  // (3) 速度上下界（**状态**约束）：v_min ≤ v_ref + e_v ≤ v_upper
  //
  // ★★ 上界与下界**都要**是“从当前速度可达的”：
  //   `v_min ≤ v_k ≤ v_upper` 是硬状态约束，但 `v_0 = v_now` 是反馈给定的、不受
  //   约束。只要 `v_now` 落在 [v_min, v_upper] 之外，就会要求 QP 在一步内吃下
  //   这个差值；超过 a_max 就是 `primal infeasible`。
  //   实测（2026-09-22 仿真）两种都碰上了，而且症状一模一样：
  //     · 实际速度一时高于限速（步态瞬态 0.89 vs 限速 0.35）⇒ 上界不可行；
  //     · 定位的 twist 低速噪声 ⇒ `v_now = -0.23 < v_min = 0` ⇒ **下界**不可行
  //       （日志：`v_now=-0.230 e_v0=-0.580 | maximum iterations reached`，
  //        而同一条日志里 `v_now=+0.090` 的周期只要 25 次迭代就解完）。
  //   修法：下界取 min(v_min, v_now + a_max·dt·k)（从当前速度最快能升到的值）。
  //   正常情况（v_now ≥ v_min）完全不变；传感器噪声再也不会把 QP 顶死。
  //
  //   ★
  //   上界还必须含**限速区帽**（节点每周期前瞻算出来）：只压参考速度、不压硬上界
  //     的话，时域内仍可能"计划"出超速段（代价会把车拉回参考，但那是软约束）。
  double v_max_eff = p_.v_max;
  if (speed_limit_ > 0.0)
    v_max_eff = std::min(v_max_eff, speed_limit_);
  if (zone_speed_limit_ > 0.0)
    v_max_eff = std::min(v_max_eff, zone_speed_limit_);
  const double v_upper = v_max_eff;
  const double v_now = clampValue(current_v, p_.v_min - 2.0, p_.v_max * 3.0);
  for (int k = 1; k <= N; ++k) {
    const int row = row_base_.base_vel + k - 1;
    const double reach_up = v_now + p_.a_max * p_.dt * k;
    const double v_reach_lo = std::min(p_.v_min, reach_up);
    const double v_reach_hi = std::max(v_upper, v_now - p_.a_max * p_.dt * k);
    av[a_layout_.cscIndex(a_slot_vel_[k - 1])] = 1.0;
    l_val_[row] = v_reach_lo - ref[k].v;
    u_val_[row] = v_reach_hi - ref[k].v;
  }

  // (4) 控制上下界（**偏差**量与限幅之差）
  for (int k = 0; k < control_count; ++k) {
    const int row_a = row_base_.base_ctrl + kControlDim * k + 0;
    av[a_layout_.cscIndex(a_slot_ctrl_[k * kControlDim + 0])] = 1.0;
    l_val_[row_a] = p_.a_min() - ref[k].a;
    u_val_[row_a] = p_.a_max - ref[k].a;

    const int row_w = row_base_.base_ctrl + kControlDim * k + 1;
    av[a_layout_.cscIndex(a_slot_ctrl_[k * kControlDim + 1])] = 1.0;
    l_val_[row_w] = -w_lim - ref[k].w;
    u_val_[row_w] = w_lim - ref[k].w;
  }

  // (5) 控制变化率：默认**只惩罚、不硬约束**（同 SLAM-PNC）。
  //     硬约束在参考突变（路网拐角）时会直接把 QP 顶成不可行，反而更难用。
  for (int k = 0; k + 1 < control_count; ++k)
    for (int j = 0; j < kControlDim; ++j) {
      const int row = row_base_.base_rate + kControlDim * k + j;
      av[a_layout_.cscIndex(a_slot_rate_[(k * kControlDim + j) * 2 + 0])] = 1.0;
      av[a_layout_.cscIndex(a_slot_rate_[(k * kControlDim + j) * 2 + 1])] =
          -1.0;
      if (!p_.constrain_control_rate)
        continue; // 保持 ±inf（不生效）
      const double ref_delta =
          (j == 0) ? (ref[k + 1].a - ref[k].a) : (ref[k + 1].w - ref[k].w);
      const double lim = (j == 0) ? (p_.a_max * p_.dt) : (p_.alpha_max * p_.dt);
      l_val_[row] = -lim - ref_delta;
      u_val_[row] = lim - ref_delta;
    }

  // (6) 走廊硬约束：|n_k · e_k[0:2]| ≤ hw，n = (-sinθ, cosθ) 是参考线的左法向。
  //     这是 P5.1 的核心验收项（"随机 1000
  //     组零越界"）：它不是惩罚而是**硬边界**。
  //
  //     ★★ 为什么这里就是最简形式（不再做"从远处慢慢收拢"的漏斗）：
  //       走廊只应该描述"已经在线里的车怎么贴线"，**不应该用来从车道外收敛**。
  //       收拢漏斗试过两版（二次可达量 / 线性速率 + 转向建立时间），都不可靠：
  //       实测严格走廊（corridor_width=0，hw_eff=0.05）下只要起点横向偏差
  //       ≥0.055 m， 约束就压在可行边界上 ⇒ `maximum iterations
  //       reached`、120/120 周期被挡、
  //       车一步不动（用户报的"有角度的路网时机器基本不会动"）。
  //       真正的做法在**上游**：路网规划器给出逐点走廊（只覆盖通道段，自由入口段
  //       不带走廊），节点只在"车已经回到走廊里"时才启用硬约束 ⇒ 本函数永远
  //       在同一起跑线上工作。
  //       若仍被外部扰动推出走廊（lat0 > hw_eff），由 computeCommand 的前置检查
  //       报 BLOCKED 并说明原因；节点下一周期自动退回自由模式沿同一参考收敛。
  const double hw_eff =
      route_mode ? std::max(half_width, p_.corridor_min_tolerance) : 0.0;
  const double nx0 = -std::sin(ref[0].yaw);
  const double ny0 = std::cos(ref[0].yaw);
  const double lat0 = std::fabs(nx0 * e_init[0] + ny0 * e_init[1]);
  int active_corr = 0;
  for (int k = 1; k <= N; ++k) {
    const double nx = -std::sin(ref[k].yaw);
    const double ny = std::cos(ref[k].yaw);
    const int row_lo = row_base_.base_corr + 2 * (k - 1) + 0; // n·e ≥ -hw
    const int row_hi = row_base_.base_corr + 2 * (k - 1) + 1; // n·e ≤ +hw
    av[a_layout_.cscIndex(a_slot_corr_[((k - 1) * 2 + 0) * 2 + 0])] = -nx;
    av[a_layout_.cscIndex(a_slot_corr_[((k - 1) * 2 + 0) * 2 + 1])] = -ny;
    av[a_layout_.cscIndex(a_slot_corr_[((k - 1) * 2 + 1) * 2 + 0])] = nx;
    av[a_layout_.cscIndex(a_slot_corr_[((k - 1) * 2 + 1) * 2 + 1])] = ny;
    if (!route_mode)
      continue; // 系数留着（结构固定），上下界保持 ±inf = 不生效
    l_val_[row_lo] = -hw_eff;
    u_val_[row_hi] = hw_eff;
    ++active_corr;
  }

  // (7) 障碍**硬下界**（线性化）：d_ref + ∇d·e ≥ d_min，d_min =
  // obstacle_hard_distance
  //     ⚠ 线性化只在参考点附近有效，所以解后还要用**真实距离**复核
  //     （见 validateSolution）。两件事都要做：前者让优化器主动避让，
  //     后者是"绝不输出会撞的轨迹"的保证。
  //
  //     ★ 下界**不可**为了"避免不可行"而放松成 min(hard, 参考净距)：试过，
  //       后果是障碍正好压在参考线时下界退化成 ~0，\"下界"形同虚设，
  //       机器人会直接沿着规划开进障碍（单测里确实变成了 FOLLOWING）。
  //       语义必须是："路径在障碍上 ⇒ 停车报 BLOCKED"，不能用"允许贴着走"来
  //       掩盖规划/地图不同源的问题。
  //
  //       （真正把仿真里的 QP 顶成不可行的不是这里，而是速度上界没考虑
  //        从当前速度的可达减速 —— 见第 (3) 组，那里已修。）
  //
  //      判据用**自己算的**距离/梯度，不依赖上面的软代价循环
  //      （那段只在 obstacle_weight > 0 时填 obs_d，否则硬约束会被一起关掉）。
  int active_obs = 0;
  // 每周期先清空（上一周期的值不能留到今天；距离场没了/关掉了都要回到 +inf）
  std::fill(obs_ref_dist_.begin(), obs_ref_dist_.end(),
            std::numeric_limits<double>::infinity());
  if (p_.obstacle_hard_enable && have_field &&
      p_.obstacle_hard_distance > 0.0) {
    for (int k = 1; k <= N; ++k) {
      const int row = row_base_.base_obs + (k - 1);
      const double d_now = dist_field_->distance(ref[k].x, ref[k].y);
      obs_ref_dist_[k] = d_now; // 仅诊断：参考自己的净距（用于失败时说明原因）
      if (d_now >= p_.obstacle_hard_distance + p_.obstacle_safe_distance)
        continue; // 足够远：不激活（保持 l = -inf）
      double gx = obs_gx[k];
      double gy = obs_gy[k];
      if (gx == 0.0 && gy == 0.0 &&
          !dist_field_->gradient(ref[k].x, ref[k].y, gx, gy))
        continue; // 梯度退化（开阔区/障碍内部）：这一行没有方向信息
      if (std::fabs(gx) + std::fabs(gy) < 1e-6)
        continue; // 近零系数会让 OSQP 的缩放变差，且这条约束也没有意义
      av[a_layout_.cscIndex(a_slot_obs_[(k - 1) * 2 + 0])] = gx;
      av[a_layout_.cscIndex(a_slot_obs_[(k - 1) * 2 + 1])] = gy;
      l_val_[row] = p_.obstacle_hard_distance - d_now;
      ++active_obs;
    }
  }

  info_.active_corridor_rows = active_corr;
  info_.active_obstacle_rows = active_obs;
  // 调参诊断：本周期参考速度/限速上界/曲率（看"为什么这么慢"就看这几个数）
  info_.ref_v = ref.empty() ? 0.0 : ref[0].v;
  info_.v_now = current_v;
  info_.ev0 = ref.empty() ? 0.0 : (current_v - ref[0].v);
  info_.upper_v = v_upper;
  // 初始航向误差与横向偏差："车不动/趴窝"最常见的一对原因（见 diagString）
  info_.e_yaw0 = wrapAngle(pose.yaw - ref[0].yaw);
  {
    const double nx0 = -std::sin(ref[0].yaw);
    const double ny0 = std::cos(ref[0].yaw);
    info_.lat0 =
        std::fabs(nx0 * (pose.x - ref[0].x) + ny0 * (pose.y - ref[0].y));
  }
  double curv_max = 0.0;
  const double s0 = ref.empty() ? 0.0 : ref[0].s;
  const double s_look = s0 + p_.curvature_lookahead;
  for (std::size_t i = 0; i < ref_s_.size(); ++i) {
    if (ref_s_[i] < s0)
      continue;
    if (ref_s_[i] > s_look)
      break;
    curv_max = std::max(curv_max, std::fabs(ref_curv_[i]));
  }
  info_.curv = curv_max;
  return true;
}

bool MpcLocalPlanner::solveQp() {
  const int n = p_layout_.nnz() > 0 ? p_layout_.rows : 0;
  const int m = a_layout_.rows;
  if (n <= 0 || m <= 0)
    return false;

  auto *P = static_cast<csc *>(osqp_P_);
  auto *A = static_cast<csc *>(osqp_A_);
  auto *work = static_cast<OSQPWorkspace *>(osqp_work_);

  const bool need_setup = (work == nullptr || P == nullptr || A == nullptr);
  if (need_setup) {
    // 结构只在第一次（或 horizon 变化后）建立；之后走 update_*
    // 的**热启动**路径。 SLAM-PNC 每周期 new 一个
    // OsqpEigen::Solver，其实等于每周期冷启动 ——
    // 我们这里结构不变，可以真正复用工作区。
    if (P) {
      freeCsc(P);
      osqp_P_ = nullptr;
    }
    if (A) {
      freeCsc(A);
      osqp_A_ = nullptr;
    }
    P = makeCsc(p_layout_.values, p_layout_.row_idx, p_layout_.col_ptr,
                p_layout_.rows, p_layout_.cols);
    A = makeCsc(a_layout_.values, a_layout_.row_idx, a_layout_.col_ptr,
                a_layout_.rows, a_layout_.cols);
    if (P == nullptr || A == nullptr) {
      if (P)
        freeCsc(P);
      if (A)
        freeCsc(A);
      return false;
    }
    osqp_P_ = P;
    osqp_A_ = A;

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.max_iter = p_.osqp_max_iter;
    settings.eps_abs = p_.osqp_eps_abs;
    settings.eps_rel = p_.osqp_eps_rel;
    settings.polish = 1;
    settings.warm_start = 1;
    // 注意：本仓库装的这份头文件用的是 **0.5 风格接口**
    // （osqp_setup(&work, const OSQPData*, settings)），不是 0.6 的
    // (P, q, A, l, u, m, n) 形式 —— 尽管 OSQP_VERSION 宏写着 0.6.2。
    // 按真实签名（以头文件为准）调用，不要照抄网上 0.6 的示例。
    OSQPData data;
    data.n = n;
    data.m = m;
    data.P = P;
    data.q = q_val_.data();
    data.A = A;
    data.l = l_val_.data();
    data.u = u_val_.data();
    if (osqp_setup(&work, &data, &settings) != 0) {
      info_.solver_status = "setup_failed";
      info_.feasible = false;
      return false;
    }
    osqp_work_ = work;
    // 热启动缓冲：长度分别等于变量数 n、约束数 m（set 时定尺，之后不再变）
    ws_x_.assign(static_cast<std::size_t>(n), 0.0);
    ws_y_.assign(static_cast<std::size_t>(m), 0.0);
  } else {
    // 值更新：结构不变，只换数值 + 用上一解的 (x, y) 热启动。
    // idx 传 OSQP_NULL + 给全量个数 = “按原顺序全量更新”。
    if (osqp_update_P_A(work, p_layout_.values.data(), OSQP_NULL,
                        p_layout_.nnz(), a_layout_.values.data(), OSQP_NULL,
                        a_layout_.nnz()) != 0 ||
        osqp_update_lin_cost(work, q_val_.data()) != 0 ||
        osqp_update_bounds(work, l_val_.data(), u_val_.data()) != 0) {
      // 更新失败（理论上不该发生）：退回重建一次，保证行为正确优先
      osqp_cleanup(work);
      osqp_work_ = nullptr;
      info_.solver_status = "update_failed";
      info_.feasible = false;
      return false;
    }
    osqp_warm_start(work, ws_x_.data(), ws_y_.data());
  }

  osqp_solve(work);
  const int status = work->info->status_val;
  info_.solver_iterations = work->info->iter;
  info_.solver_status = work->info->status;
  info_.feasible = (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE);

  // ★ 热启动只在**这次真解出来**时才留下：见 .hpp 里 ws_x_ 的注释。
  //   之前直接喂 `work->solution->x/y`，结果一次失败之后永久失败
  //   （仿真里表现为"区域挪开了车也不动", 单测
  //   SolverRecoversAfterInfeasibleCycle）。
  if (info_.feasible && ws_x_.size() == static_cast<std::size_t>(n) &&
      ws_y_.size() == static_cast<std::size_t>(m)) {
    for (int i = 0; i < n; ++i)
      ws_x_[static_cast<std::size_t>(i)] = work->solution->x[i];
    for (int i = 0; i < m; ++i)
      ws_y_[static_cast<std::size_t>(i)] = work->solution->y[i];
  } else {
    std::fill(ws_x_.begin(), ws_x_.end(), 0.0);
    std::fill(ws_y_.begin(), ws_y_.end(), 0.0);
  }
  return info_.feasible;
}

bool MpcLocalPlanner::validateSolution(
    const std::vector<MpcReferencePoint> &ref, bool route_mode,
    double half_width, std::string &why) {
  const int N = p_.horizon;
  auto *work = static_cast<OSQPWorkspace *>(osqp_work_);
  if (work == nullptr || work->solution == nullptr)
    return false;
  const double *z = work->solution->x;

  predicted_.clear();
  predicted_.reserve(static_cast<std::size_t>(N + 1));

  // ★ 动力学残差体检：确认 QP **真的**把 e_{k+1} = A e_k + B Δu_k 当约束。
  // 这一步抓过真 bug：矩阵组装错时求解器照样"成功"，但预测轨迹会瞬移。
  double max_res = 0.0;
  for (int k = 0; k + 1 <= N; ++k) {
    if (k == N)
      break;
    const MpcReferencePoint &r = ref[k];
    const double a02 = -r.v * std::sin(r.yaw) * p_.dt;
    const double a03 = std::cos(r.yaw) * p_.dt;
    const double a12 = r.v * std::cos(r.yaw) * p_.dt;
    const double a13 = std::sin(r.yaw) * p_.dt;
    const double b21 = p_.dt;
    const double b30 = p_.dt;
    const double du_a = z[controlIndex(k, 0)];
    const double du_w = z[controlIndex(k, 1)];
    const double e0 = z[stateIndex(k, 0)];
    const double e1 = z[stateIndex(k, 1)];
    const double e2 = z[stateIndex(k, 2)];
    const double e3 = z[stateIndex(k, 3)];
    const double pred0 = e0 + a02 * e2 + a03 * e3;
    const double pred1 = e1 + a12 * e2 + a13 * e3;
    const double pred2 = e2 + b21 * du_w;
    const double pred3 = e3 + b30 * du_a;
    const double n0 = z[stateIndex(k + 1, 0)];
    const double n1 = z[stateIndex(k + 1, 1)];
    const double n2 = z[stateIndex(k + 1, 2)];
    const double n3 = z[stateIndex(k + 1, 3)];
    max_res = std::max(max_res, std::fabs(n0 - pred0));
    max_res = std::max(max_res, std::fabs(n1 - pred1));
    max_res = std::max(max_res, std::fabs(n2 - pred2));
    max_res = std::max(max_res, std::fabs(n3 - pred3));
  }
  info_.max_dynamics_residual = max_res;

  const double hw_eff = route_mode
                            ? std::max(half_width, p_.corridor_min_tolerance)
                            : std::numeric_limits<double>::max();
  double max_lat = 0.0;
  double min_d = std::numeric_limits<double>::max();
  int violations = 0;

  for (int k = 0; k <= N; ++k) {
    const double ex = z[stateIndex(k, 0)];
    const double ey = z[stateIndex(k, 1)];
    const double eth = z[stateIndex(k, 2)];
    const double x = ref[k].x + ex;
    const double y = ref[k].y + ey;
    predicted_.push_back(Pose2D{x, y, wrapAngle(ref[k].yaw + eth)});
    // 走廊：用**真实几何**的横向偏差复核（不是 QP 里那个线性量）
    const double nx = -std::sin(ref[k].yaw);
    const double ny = std::cos(ref[k].yaw);
    const double lat = std::abs(nx * ex + ny * ey);
    max_lat = std::max(max_lat, lat);
    if (route_mode && lat > hw_eff + 1e-6)
      ++violations;

    if (dist_field_ && dist_field_->valid()) {
      const double d = dist_field_->distance(x, y);
      min_d = std::min(min_d, d);
    }
  }
  info_.max_lateral_deviation = max_lat;
  info_.min_predicted_distance =
      (min_d == std::numeric_limits<double>::max()) ? 0.0 : min_d;
  info_.corridor_violations = violations;

  // ★ 自检：解必须满足动力学。不满足说明矩阵组装/求解出了错 ——
  // 这时**绝不能**拿着这个解去发指令（实测踩过：漏了 A 的单位阵项，
  // 求解器报成功、预测轨迹却瞬移，跟着一条假轨迹开车最危险）。
  if (info_.max_dynamics_residual > 1e-3) {
    why = "QP 解不满足动力学（残差 " +
          std::to_string(info_.max_dynamics_residual) +
          "，应 ≲ 1e-4）—— 拒绝使用该解";
    return false;
  }

  if (route_mode && violations > 0) {
    why = "走廊硬约束被违反（" + std::to_string(violations) + " 步，最大 " +
          std::to_string(max_lat) + " m > " + std::to_string(hw_eff) + " m）";
    return false;
  }
  // 障碍硬下界的**真实**复核（线性化误差可能让解越过它）
  if (p_.obstacle_hard_enable && dist_field_ && dist_field_->valid() &&
      p_.obstacle_hard_distance > 0.0 &&
      info_.min_predicted_distance < p_.obstacle_hard_distance - 0.02) {
    why = "预测轨迹会侵入障碍硬距离（最小 " +
          std::to_string(info_.min_predicted_distance) + " m < " +
          std::to_string(p_.obstacle_hard_distance) + " m" +
          refClearanceNote() + "）";
    return false;
  }
  return true;
}

std::string MpcLocalPlanner::diagString() const {
  char buf[360];
  // ★ 「没有预测轨迹」与「最小净距 0.00
  // m」是**两件完全不同的事**，日志里必须分开。
  //   info_.min_predicted_distance 在"没能算出预测轨迹"时是 0.0（哨兵值），
  //   原来的 %.2f 把它打成 `最小距0.00` ——
  //   排查时看起来就是"贴着障碍/钻进去了"， 实际是"求解没出结果"。2026-09-23
  //   为了这一行字花了很久才找到真正的根因
  //   （求解器被上一次失败的迭代毒化）。所以这里显式区分：无预测 ⇒ `n/a`。
  char clearance[24];
  if (predicted_.empty())
    std::snprintf(clearance, sizeof(clearance), "n/a");
  else
    std::snprintf(clearance, sizeof(clearance), "%.2f",
                  info_.min_predicted_distance);
  // 限速区帽单独打出来：现场"为什么这里只跑 0.2 m/s"十有八九就是它
  // （v_up 已经是含帽后的生效上界，但看不出是任务限速还是区域限速）。
  std::snprintf(
      buf, sizeof(buf),
      "mpc v_ref=%.3f v_now=%.3f e_v0=%+.3f v_up=%.3f"
      "（任务限速%.2f 区域限速%.2f）curv=%.3f | "
      "e_yaw0=%+.1f° lat0=%.2f | "
      "cross=%+.3f prog=%.2f | 走廊行%d 障碍行%d 最小距%s | %s %d迭代 "
      "%.1fms",
      info_.ref_v, info_.v_now, info_.ev0, info_.upper_v, speed_limit_,
      zone_speed_limit_, info_.curv, info_.e_yaw0 * 180.0 / M_PI, info_.lat0,
      info_.cross_track, info_.progress, info_.active_corridor_rows,
      info_.active_obstacle_rows, clearance, info_.solver_status.c_str(),
      info_.solver_iterations, info_.solve_ms);
  return std::string(buf);
}

std::string MpcLocalPlanner::refClearanceNote() const {
  if (!dist_field_ || !dist_field_->valid() || obs_ref_dist_.empty())
    return {};
  double dmin = std::numeric_limits<double>::infinity();
  for (std::size_t k = 1; k < obs_ref_dist_.size(); ++k)
    dmin = std::min(dmin, obs_ref_dist_[k]);
  if (!std::isfinite(dmin) || dmin >= p_.obstacle_hard_distance)
    return {};
  return "；★ 参考路径**自身**就在障碍硬距离内（参考最小净距 " +
         std::to_string(dmin) + " m < " +
         std::to_string(p_.obstacle_hard_distance) +
         " m）—— 问题在规划路径/地图与局部距离场不同源，不是控制器";
}

// 对正用的角速度：`gain·e` 再限幅，**并给一个下限打破底盘死区**。
//
// 为什么需要下限（用户 2026-09-23 实测“角度判断太严格导致无法收敛”）：
// 底盘低速有死区，指令太小时步态/电机根本不动，而 `gain·e` 在 e 接近容差时
// 给的 ω 很小 ⇒ 车停在容差**外**不再转 ⇒ 到点判定永远不满足。
// 下限保证“只要决定转，就转到车真的会动”。

double MpcLocalPlanner::alignRate(double e_yaw) const
{
  if (std::fabs(e_yaw) < 1e-9)
    return 0.0;
  const double want = p_.align_gain * e_yaw;
  const double lo = std::min(p_.align_w_min, p_.w_max);
  const double mag = clampValue(std::fabs(want), lo, p_.w_max);
  return (want < 0.0 ? -1.0 : 1.0) * mag;
}

// ============================================================================
// 到点后的目标朝向对正
// ============================================================================
//
// 为什么单独做（用户 2026-09-23 指出）：到点判定原来只管 xy，机头朝哪都算到达；
// 而任务目标天生带 yaw（面向充电桩/面向通道口/面向装货台）。差速与足式底盘的
// ω 与 v 解耦、可以原地转，所以正确行为是 **位置到了 → 原地把机头对到目标朝向 →
// 才算到达**；而对正本身需要净距（矩形车体旋转会扫过外接圆），所以复用
// align_min_clearance 这个门槛。
//
// 目标朝向取**路径最后一点**的 yaw（全局规划把目标 yaw 放在末点），这样
// "目标朝向"与"跟随的参考"是同一份数据，不需要接口再加一个字段。
bool MpcLocalPlanner::goalYawAlign(const Pose2D &pose, double remaining,
                                   LocalPlanResult &out) {
  const auto t_align = std::chrono::steady_clock::now();
  const double tol = goalYawTolerance(); // [rad]
  if (tol <= 0.0 || plan_.size() < 2) {
    goal_aligning_ = false;
    return false;
  }
  const double goal_yaw = plan_.back().yaw;
  const double e = wrapAngle(goal_yaw - pose.yaw);
  const double deg = std::fabs(e) * 180.0 / M_PI;
  const double tol_deg = tol * 180.0 / M_PI;
  // 到"终点位置附近"才管朝向：距离还很远时该专心走（否则会提前停下转圈）
  const double near_dist = std::max(p_.goal_yaw_align_distance, 1e-3);
  if (remaining > near_dist) {
    goal_aligning_ = false;
    return false;
  }
  // 滞环：进入容差就退出；退出后要超过容差才重新进入（避免在阈值上抖）
  const bool want = goal_aligning_ ? deg > 0.5 * tol_deg : deg > tol_deg;
  if (!want) {
    goal_aligning_ = false;
    return false;
  }

  // 超时：**转不动**才算失败 —— 注意是“**无进展**超时”而不是“墙钟超时”：
  // 大角度对正（180° @ 0.65 rad/s ≈ 5 s）本来就慢，用墙钟计会把“转得慢但一直在收敛”
  // 判成失败（用户实测的“无法收敛”指的是**车根本不动**）。所以只要这一段时间内
  // 偏差没有实质改善（< 2°）才计超时，改善即重新计时。
  if (!goal_aligning_) {
    goal_align_since_ = t_align;
    goal_align_best_deg_ = deg;
  } else if (deg < goal_align_best_deg_ - 2.0) {
    goal_align_best_deg_ = deg;
    goal_align_since_ = t_align;
  }
  if (p_.goal_yaw_align_timeout > 0.0) {
    const double elapsed = std::chrono::duration<double>(
                               t_align - goal_align_since_)
                               .count();
    if (elapsed > p_.goal_yaw_align_timeout) {
      out.status = LocalStatus::kFailed;
      out.message =
          "到点朝向对正超时（" + std::to_string(static_cast<int>(elapsed)) +
          " s 内偏差没有改善，当前仍差 " + std::to_string(static_cast<int>(deg)) +
          "° > 容差 " + std::to_string(static_cast<int>(tol_deg)) +
          "°）—— 检查 align_gain/align_w_min（底盘低速死区）、定位朝向噪声，"
          "或把 goal_yaw_tolerance_deg 放宽";
      out.cmd = Twist2D{0.0, 0.0};
      goal_aligning_ = false;
      return true;
    }
  }

  // 车体是矩形，原地旋转会扫过外接圆：贴得太近就不转（宁可如实报"停不下来对正"）
  const double clear = (dist_field_ && dist_field_->valid())
                           ? dist_field_->distance(pose.x, pose.y)
                           : std::numeric_limits<double>::infinity();
  if (clear < p_.align_min_clearance) {
    goal_aligning_ = false;
    out.status = LocalStatus::kBlocked;
    out.message = "已到点位置，但净距 " + std::to_string(clear) +
                  " m < align_min_clearance " +
                  std::to_string(p_.align_min_clearance) +
                  " m，无法原地对正到目标朝向（还差 " +
                  std::to_string(static_cast<int>(deg)) + "°）";
    out.cmd = Twist2D{0.0, 0.0};
    return true;
  }

  goal_aligning_ = true;
  out.cmd = Twist2D{0.0, alignRate(e)};
  out.status = LocalStatus::kFollowing;
  out.message = "已到点，原地对正目标朝向（偏差 " +
                std::to_string(static_cast<int>(deg)) + "° > " +
                std::to_string(static_cast<int>(tol_deg)) + "°）";
  info_.solver_status = "goal-yaw-align";
  info_.ref_v = 0.0;
  info_.v_now = v_now_;
  info_.solve_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t_align)
                       .count();
  out.stats.solve_ms = info_.solve_ms;
  return true;
}

// ============================================================================
// 主入口
// ============================================================================
LocalPlanResult MpcLocalPlanner::computeCommand(const Pose2D &pose, double dt) {
  const auto t0 = std::chrono::steady_clock::now();
  LocalPlanResult out;

  if (dt > 0.0 && p_.dt > 0.0 && std::fabs(dt - p_.dt) > 0.3 * p_.dt) {
    // 只提示一次：调用频率与预测步长差太多会让"模型"和"实际"对不上
    static bool warned = false;
    if (!warned) {
      warned = true;
      out.message = "调用周期与 local_mpc.dt 不一致（模型按 dt 走）";
    }
  }

  if (ref_s_.size() < 2 || ref_length_ < 1e-6) {
    out.status = LocalStatus::kIdle;
    out.message = "没有可跟随的路径（请先 setGlobalPlan/setCorridor）";
    return out;
  }

  // ---- 1) 投影：当前在参考线的哪里 ----
  double cross = 0.0;
  const double s0 = projectOntoReference(pose.x, pose.y, cross);
  progress_s_ = s0;
  has_progress_ = true;
  info_.progress = ref_length_ > 1e-6 ? s0 / ref_length_ : 0.0;
  info_.cross_track = cross;

  // 到终点：把状态交给节点判定（P4 已定"到位判定在节点层兜底"），
  // 这里只报告 + 给零指令。
  const double remaining = std::max(0.0, ref_length_ - s0);

  // ---- 1.5) 到点位置但朝向不对：原地对正到**目标朝向**（见 params 注释）----
  //   ★ 必须放在下面 `remaining <= 1e-3 → kGoalReached + 零速` **之前**：
  //     否则算法会把车停住、节点又不算到达（朝向不满足）⇒ 双方都在等对方 ⇒
  //     任务卡死到超时。
  if (goalYawAlign(pose, remaining, out))
    return out;

  if (remaining <= 1e-3) {
    out.status = LocalStatus::kGoalReached;
    out.message = "已到路径终点";
    out.cmd = Twist2D{0.0, 0.0};
    return out;
  }

  // ---- 2) 参考窗口 ----
  std::vector<MpcReferencePoint> window;
  if (!buildReferenceWindow(s0, window)) {
    out.status = LocalStatus::kFailed;
    out.message = "参考窗口构造失败（路径太短或弧长异常）";
    return out;
  }

  // ---- 2.5) 大航向偏差：先原地对正机头，再走 ----
  //
  //   ★ 为什么单独做一个模式（实测根因）：参考折线是"沿路径前进"的语义，车头与
  //     参考方向差几十度时，MPC 去找的解都是"边转边走"的——而“走”会让预测轨迹
  //     扫出走廊/障碍硬界，于是代价上"原地不动"更便宜：**车拒动**。实测航向差
  //     7/15/25° 时用满加速能力起步，40/60° 时 `cmd v=0`（`StrictCorridor-
  //     WithHeadingOffsetStillDrives` 把这个现象钉住过）。可是差速/足式底盘的
  //     ω 与 v 是解耦的，原地转正物理上完全可行 ⇒ 正确行为是“先对正再走”。
  {
    const double e_yaw0 = wrapAngle(pose.yaw - window[0].yaw);
    info_.e_yaw0 = e_yaw0; // 诊断行要用（即使本周期不走 QP）
    const double deg = std::fabs(e_yaw0) * 180.0 / M_PI;
    const bool want =
        p_.align_in_place_deg > 0.0 &&
        (aligning_ ? deg > p_.align_exit_deg : deg > p_.align_in_place_deg);
    if (!want) {
      aligning_ = false;
    } else {
      // 车体是矩形，原地旋转会扫过外接圆：贴得太近就不转（宁可停也不剧蹭）
      const double clear = (dist_field_ && dist_field_->valid())
                               ? dist_field_->distance(pose.x, pose.y)
                               : std::numeric_limits<double>::infinity();
      if (clear >= p_.align_min_clearance) {
        aligning_ = true;
        out.cmd = Twist2D{0.0, alignRate(-e_yaw0)};
        out.status = LocalStatus::kFollowing;
        out.message = "原地对正机头（偏差 " +
                      std::to_string(static_cast<int>(deg)) + "° > " +
                      std::to_string(static_cast<int>(p_.align_in_place_deg)) +
                      "°）—— 对正后再沿路径走";
        info_.solver_status = "align-in-place";
        info_.ref_v = 0.0;
        info_.v_now = v_now_;
        info_.solve_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t0)
                             .count();
        out.stats.solve_ms = info_.solve_ms;
        return out;
      }
    }
  }

  // ---- 3) 建 QP 并求解 ----
  const bool route = route_mode_ && corridor_ != nullptr;
  const double half_width = route ? corridor_->half_width : 0.0;

  // ---- 2.9) 自由模式：横向死区（"顺滑优先"，用户 2026-09-23）----
  //   自由导航的参考只是"大致往那儿走"的折线，几 cm 的横向偏差不该动方向盘。
  //   做法：把参考线**平移到车所在的那一侧**（限幅到死区带内）—— 带内的偏差于是
  //   不再是"跟踪误差"（MPC 不会去追），带外的部分照旧追回来。
  //   为什么不用"把代价设成 0"：代价是二次型，分区代价会变成非凸；平移参考是
  //   等价且凸的做法。
  if (!route && p_.free_lat_deadband > 0.0 && window.size() > 1) {
    const double n0x = -std::sin(window[0].yaw);
    const double n0y = std::cos(window[0].yaw);
    const double lat = n0x * (pose.x - window[0].x) + n0y * (pose.y - window[0].y);
    const double shift =
        clampValue(lat, -p_.free_lat_deadband, p_.free_lat_deadband);
    if (std::fabs(shift) > 1e-9) {
      for (auto &r : window) {
        const double nx = -std::sin(r.yaw);
        const double ny = std::cos(r.yaw);
        r.x += shift * nx;
        r.y += shift * ny;
      }
      // ⚠ 不去改 info_.cross_track：它是“车到参考线的横向偏差”，验收脚本会拿它
      //   与独立算出的几何量对照，改了会变成另一个含义。
    }
  }
  // 自由模式的角速度上限（治“左右打满”的摆动）；走廊模式保持 w_max（贴线要紧）
  const double w_lim = wMaxEff(route);
  last_w_limit_ = w_lim;

  // ★ 走廊模式的**前置检查**：车已经不在走廊里时直接报 BLOCKED 并说清原因。
  //   · 为什么不让"边走边收敛"：那等于默认允许车贴着走廊外面开，而走廊就是用来
  //     禁止这件事的（窄通道/巡检路线）。实测在硬约束下从车道外收敛**根本不可行**：
  //     严格走廊（半宽 0.05 m）下起点横向偏差 ≥0.055 m 时 solver
  //     不收敛，车一步不动。
  //   ·
  //   所以正常路径是**上游先解决**：路网规划器给逐点走廊（自由入口段不带走廊），
  //     节点只在"车已在走廊里"时才把走廊交给算法（见 local_planner_node）。
  //     这里留着当兜底：真出现（外部扰动把车推出走廊）就如实停车一周期，
  //     节点下一周期会自动退回自由模式沿同一参考收敛回来。
  if (route) {
    const double hw_eff = std::max(half_width, p_.corridor_min_tolerance);
    const double nx0 = -std::sin(window[0].yaw);
    const double ny0 = std::cos(window[0].yaw);
    const double lat0 =
        std::fabs(nx0 * (pose.x - window[0].x) + ny0 * (pose.y - window[0].y));
    if (lat0 > hw_eff + 1e-9) {
      out.status = LocalStatus::kBlocked;
      out.message =
          "起点在走廊外 " + std::to_string(lat0) + " m（走廊半宽 " +
          std::to_string(hw_eff) +
          " m）→ 本周期停车：走廊只描述「已经在线里怎么贴线」，不能用来从车道外"
          "收敛（实测那样会让 QP "
          "压在可行边界上、求解不收敛）。正常应由上游解决："
          "路网规划器给逐点走廊（自由入口段不带走廊），节点只在车回到走廊里时才"
          "启用它；若反复出现，看入口段是否走完、定位/投影是否与全局一致。";
      out.cmd = Twist2D{0.0, 0.0};
      info_.solve_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t0)
                           .count();
      out.stats.solve_ms = info_.solve_ms;
      return out;
    }
  }

  if (!buildQp(window, pose, v_now_, route, half_width)) {
    out.status = LocalStatus::kFailed;
    out.message = "QP 构造失败";
    return out;
  }

  if (!solveQp()) {
    // D8：求解失败 ⇒ **停车 + 报错**，不做 pure_pursuit 兜底
    out.status = LocalStatus::kBlocked;
    out.message = "MPC 求解失败：" + info_.solver_status + refClearanceNote();
    out.cmd = Twist2D{0.0, 0.0};
    info_.solve_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t0)
                         .count();
    out.stats.solve_ms = info_.solve_ms;
    out.stats.solver_iter = info_.solver_iterations;
    return out;
  }

  std::string why;
  if (!validateSolution(window, route, half_width, why)) {
    // 硬约束（走廊/障碍）在真实几何下不满足 ⇒ BLOCKED（决策
    // D5/D12：停车交状态机）
    out.status = LocalStatus::kBlocked;
    out.message = why;
    out.cmd = Twist2D{0.0, 0.0};
    info_.solve_ms = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t0)
                         .count();
    out.stats.solve_ms = info_.solve_ms;
    out.stats.solver_iter = info_.solver_iterations;
    return out;
  }

  // ---- 4) 取指令（与 SLAM-PNC 一致的两点）----
  const auto *work = static_cast<const OSQPWorkspace *>(osqp_work_);
  const double *z = work->solution->x;
  const double du_a = z[controlIndex(0, 0)];
  const double du_w = z[controlIndex(0, 1)];

  // 角速度直接由控制偏差给出（ω 在我们的模型里就是控制量）
  double w_cmd = window[0].w + du_w;
  // 前向速度取**预测第一步的速度**（= ref[1].v + e_1(3)）。
  // 为什么不用 ref[0].v + Δa·dt：指令发出到执行有一个周期左右的延迟，
  // 用"一步之后应该达到的速度"作为指令可以提前补偿；这也是 SLAM-PNC 的做法
  // （它因为控制量是 a/δ、必须转成 v，天然走了这条路）。
  double v_cmd = window[1].v + z[stateIndex(1, 3)];

  v_cmd = clampValue(v_cmd, p_.v_min, p_.v_max);
  w_cmd = clampValue(w_cmd, -w_lim, w_lim);
  if (p_.obstacle_weight > 0.0 && dist_field_ && dist_field_->valid()) {
    // 离障碍很近时**主动限速**：硬约束保证不撞，但用速度换安全余量更稳
    const double d_now = dist_field_->distance(pose.x, pose.y);
    if (d_now < p_.obstacle_safe_distance) {
      const double ratio =
          clampValue(d_now / p_.obstacle_safe_distance, 0.15, 1.0);
      v_cmd *= ratio;
    }
  }
  smooth_v_ = v_cmd;
  smooth_w_ = w_cmd;

  out.cmd.v = v_cmd;
  out.cmd.w = w_cmd;
  out.status = degraded() ? LocalStatus::kDegraded : LocalStatus::kFollowing;
  if (out.message.empty())
    out.message = degraded() ? "缺距离场：降级（限速 + 只保硬约束）"
                             : (route ? "走廊模式跟踪" : "自由空间跟踪");

  // ---- 5) 统计 ----
  info_.solve_ms = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - t0)
                       .count();
  out.stats.solve_ms = info_.solve_ms;
  out.stats.solver_iter = info_.solver_iterations;
  out.stats.cross_track = cross;
  out.stats.progress = info_.progress;
  out.stats.corridor_violations = info_.corridor_violations;
  out.stats.time_to_goal = v_cmd > 0.05 ? remaining / v_cmd : 0.0;
  return out;
}

bool MpcLocalPlanner::solveForTest(const Pose2D &current, double current_v,
                                   const std::vector<MpcReferencePoint> &ref,
                                   Twist2D &cmd, LocalStatus &status,
                                   std::string &message) {
  // 单测入口：绕过路径投影/弧长参数化，直接给参考窗口。
  // 这样"求解器与约束"可以独立于"参考线构造"被验证（两者出错的表现完全不同）。
  if (static_cast<int>(ref.size()) != p_.horizon + 1)
    return false;
  if (!buildQp(ref, current, current_v, false, 0.0))
    return false;
  if (!solveQp())
    return false;
  std::string why;
  if (!validateSolution(ref, false, 0.0, why)) {
    message = why;
    status = LocalStatus::kBlocked;
    cmd = Twist2D{0.0, 0.0};
    return false;
  }
  const auto *work = static_cast<const OSQPWorkspace *>(osqp_work_);
  const double *z = work->solution->x;
  cmd.w = clampValue(ref[0].w + z[controlIndex(0, 1)], -p_.w_max, p_.w_max);
  cmd.v = clampValue(ref[1].v + z[stateIndex(1, 3)], p_.v_min, p_.v_max);
  status = LocalStatus::kFollowing;
  message.clear();
  return true;
}

} // namespace pnc_2d

// ============================================================================
// 调试导出（只读，不影响求解）
// ============================================================================
namespace pnc_2d {
void MpcLocalPlanner::snapshotQp(QpSnapshot &out) const {
  out = QpSnapshot{};
  out.n = p_layout_.cols;
  out.m = a_layout_.rows;
  out.P.assign(static_cast<std::size_t>(out.n) * out.n, 0.0);
  out.A.assign(static_cast<std::size_t>(out.m) * out.n, 0.0);
  out.q = q_val_;
  out.l = l_val_;
  out.u = u_val_;

  // CSC → 稠密。P 存的是上三角，读出来后**对称化**，这样测试里可以按
  // "P 是 H 的两倍"的数学含义直接比对（不必让测试也去关心上三角约定）。
  for (int c = 0; c < p_layout_.cols; ++c) {
    for (int k = p_layout_.col_ptr[c]; k < p_layout_.col_ptr[c + 1]; ++k) {
      const int r = p_layout_.row_idx[k];
      const double v = p_layout_.values[k];
      out.P[static_cast<std::size_t>(r) * out.n + c] = v;
      if (r != c)
        out.P[static_cast<std::size_t>(c) * out.n + r] = v;
    }
  }
  for (int c = 0; c < a_layout_.cols; ++c) {
    for (int k = a_layout_.col_ptr[c]; k < a_layout_.col_ptr[c + 1]; ++k) {
      out.A[static_cast<std::size_t>(a_layout_.row_idx[k]) * out.n + c] =
          a_layout_.values[k];
    }
  }

  auto *work = static_cast<OSQPWorkspace *>(osqp_work_);
  if (work != nullptr && work->solution != nullptr &&
      work->solution->x != nullptr) {
    out.x.assign(work->solution->x, work->solution->x + out.n);
    if (work->solution->y != nullptr)
      out.y.assign(work->solution->y, work->solution->y + out.m);
  }
}

void MpcLocalPlanner::dumpQp(std::string &out) const {
  out.clear();
  char buf[256];
  std::snprintf(buf, sizeof(buf), "n_var=%d n_con=%d P_nnz=%d A_nnz=%d\n",
                p_layout_.rows, a_layout_.rows, p_layout_.nnz(),
                a_layout_.nnz());
  out += buf;
  const int N = p_.horizon;
  auto dumpA = [&](int row, const char *tag) {
    std::snprintf(buf, sizeof(buf), "row %3d %-10s:", row, tag);
    out += buf;
    for (int k = 0; k < a_layout_.nnz(); ++k) {
      if (a_layout_.row_idx[k] != row)
        continue;
      // 找列：从 col_ptr 反查
      for (int c = 0; c < a_layout_.cols; ++c) {
        if (k >= a_layout_.col_ptr[c] && k < a_layout_.col_ptr[c + 1]) {
          std::snprintf(buf, sizeof(buf), " [%d]=%+.3g", c,
                        a_layout_.values[k]);
          out += buf;
          break;
        }
      }
    }
    std::snprintf(buf, sizeof(buf), "  l=%.4g u=%.4g\n", l_val_[row],
                  u_val_[row]);
    out += buf;
  };
  for (int i = 0; i < 4; ++i)
    dumpA(i, "eq_e0");
  for (int i = 0; i < 4; ++i)
    dumpA(4 + i, "dyn_k0");
  dumpA(row_base_.base_vel, "vel_k1");
  dumpA(row_base_.base_ctrl, "ctrl_k0_a");
  for (int i = 0; i < 4; ++i)
    dumpA(4 + 4 * (N - 1) + i, "dyn_kN-1");
  out += "q[0..7]:";
  for (int i = 0; i < 8; ++i) {
    std::snprintf(buf, sizeof(buf), " %.4g", q_val_[i]);
    out += buf;
  }
  out += "\n";
}
} // namespace pnc_2d
