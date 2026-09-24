// MINCO 后端实现。设计说明见头文件；这里只补两条实现要点：
//
// 1. `(θ, s)` 参数化的一个**关键好处**：由 `ẋ = ṡ·cosθ`、`ẏ = ṡ·sinθ` 可知
//    XY 轨迹的速率恰好是 |ṡ|、角速度恰好是 θ̇ ⇒ **运动学约束就是多项式导数的界**，
//    不需要曲率限速那一套（也不受"折线曲率是噪声"之害）。`s` 就是弧长。
// 2. 时间与几何**解耦**：把所有 T_i 同乘 k，几何完全不变（θ(t)、s(t) 只是被拉伸），
//    而 v 除以 k、a 除以 k² ⇒ 运动学可行性可以用**一个解析的 k** 一次算出来，
//    不需要二分、不需要梯度。终点残差则只依赖几何，单独 1 维修正。

#include "pnc_2d/traj/minco_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "gcopter/lbfgs.hpp"
#include "gcopter/minco.hpp"
#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/param_reader.hpp"

namespace pnc_2d {
namespace {

using Vec2 = Eigen::Vector2d;
constexpr double kEps = 1e-9;

double wrapPi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

/// 把点夹进地图框（支持 origin yaw）。返回原坐标是否已经在框内。
bool clampIntoMap(const CostMap2D & map, Vec2 & p)
{
  const double dx = p.x() - map.originX();
  const double dy = p.y() - map.originY();
  const double c = std::cos(map.originYaw());
  const double s = std::sin(map.originYaw());
  const double lx = c * dx + s * dy;
  const double ly = -s * dx + c * dy;
  const double w = map.width() * map.resolution();
  const double h = map.height() * map.resolution();
  const double cx = std::clamp(lx, 0.0, w);
  const double cy = std::clamp(ly, 0.0, h);
  const double ox = cx - lx;
  const double oy = cy - ly;
  p.x() = map.originX() + c * cx - s * cy;
  p.y() = map.originY() + s * cx + c * cy;
  return (std::fabs(ox) < kEps) && (std::fabs(oy) < kEps);
}

// ------------------------------------------------------------------ Stage A 平滑
//
// 弹性带式调整：自由变量就是折线上的**内点**（首末固定）⇒ SDF 的值/梯度直接作用
// 在变量上，梯度精确、实现简单；不需要"MINCO 内点 → 轨迹点"的链式求导
// （那是原版 optimizer.cpp 里最重的一块，且很容易写错）。
//
// 代价 = 平滑（二阶差分）+ 净距罚 + 锚定（别离原计划太远）+ 图外屏障
struct SmoothCtx {
  const ClearanceField * cf{nullptr};
  const CostMap2D * map{nullptr};
  std::vector<Vec2> pts;      // 含首末
  std::vector<Vec2> anchor;   // 原始（锚定）
  double w_smooth{1.0};
  double w_sdf{2.0};
  double w_anchor{0.2};
  double w_out{10.0};
  double d_target{0.55};
  /// **逐点**净距目标（见 smoothPath 里的"端点自适应"说明）。
  /// ★ 为什么不是全局单值：首末点被**锚定**、平滑动不了它们，端点附近的净距
  ///   天然**不可达**全局目标 ⇒ 罚项与锚定项死拽、收敛点落在目标之下，
  ///   `达成 < 目标` 恒定发生（但那是几何不可达，不是优化没做好）；
  ///   更糟的是持续对抗会把路径挤出畸形（bulge）甚至撞上别的障碍。
  ///   按**点索引**给目标（重采样后点近似等距 ⇒ 索引 × 平均间距 ≈ 弧长）；
  ///   用索引而不是实时弧长，是因为实时弧长是位置的函数，忽略它的导数会
  ///   让梯度失真、L-BFGS 卡住。
  std::vector<double> d_tgt;
  // 诊断
  double min_d{0.0};
  double shortfall{0.0};   ///< max(d_tgt − d)：0 = 逐点目标都达标
  int outside{0};
  int evals{0};
};

double evaluateSmooth(void * instance, const Eigen::VectorXd & x, Eigen::VectorXd & g)
{
  auto * ctx = static_cast<SmoothCtx *>(instance);
  const int n = static_cast<int>(ctx->pts.size());
  const int m = n - 2;                       // 内点数（首末固定）
  for (int i = 0; i < m; ++i) {
    ctx->pts[static_cast<std::size_t>(i + 1)] = Vec2(x(2 * i), x(2 * i + 1));
  }
  g.setZero(2 * m);
  ++ctx->evals;

  double cost = 0.0;

  // ---- 平滑项：E = Σ|p_{i+1} − 2p_i + p_{i−1}|²，∂E/∂p_i = 2(r_{i−1} − 2r_i + r_{i+1})
  for (int i = 1; i <= n - 2; ++i) {
    const Vec2 r = ctx->pts[i + 1] - 2.0 * ctx->pts[i] + ctx->pts[i - 1];
    cost += ctx->w_smooth * r.squaredNorm();
  }
  for (int i = 1; i <= n - 2; ++i) {
    Vec2 acc = Vec2::Zero();
    for (int d = -1; d <= 1; ++d) {
      const int j = i + d;
      if (j < 1 || j > n - 2) continue;
      const Vec2 rj = ctx->pts[j + 1] - 2.0 * ctx->pts[j] + ctx->pts[j - 1];
      acc += (d == 0 ? -2.0 : 1.0) * rj;
    }
    g.segment<2>(2 * (i - 1)) += 2.0 * ctx->w_smooth * acc;
  }

  // ---- 净距罚 + 图外屏障 + 锚定
  const double huge = std::numeric_limits<double>::max();
  double min_d = huge;
  double shortfall = 0.0;
  for (int i = 1; i <= n - 2; ++i) {
    const Vec2 p = ctx->pts[i];
    const double d_tgt_i =
        (static_cast<std::size_t>(i) < ctx->d_tgt.size()) ? ctx->d_tgt[static_cast<std::size_t>(i)]
                                                          : ctx->d_target;
    double d = d_tgt_i;                     // 没有距离场 ⇒ 不罚

    if (ctx->map != nullptr) {
      Vec2 pc = p;
      if (!clampIntoMap(*ctx->map, pc)) {
        ++ctx->outside;
        const Vec2 over = p - pc;           // 越界量（指向图外）
        cost += ctx->w_out * over.squaredNorm();
        g.segment<2>(2 * (i - 1)) += 2.0 * ctx->w_out * over;
      }
    }

    if (ctx->cf != nullptr && ctx->cf->valid()) {
      // ★ 用**保守下界**（扣掉 √2/2·res），不要用双线性原值
      d = ctx->cf->lowerBoundAtWorld(p.x(), p.y(), 1.0e3);
      const double viol = d_tgt_i - d;
      if (viol > 0.0) {
        shortfall = std::max(shortfall, viol);
        double grad[2] = {0.0, 0.0};
        if (ctx->cf->gradientAtWorld(p.x(), p.y(), grad)) {
          cost += ctx->w_sdf * viol * viol;
          // E 只依赖 d；把点往 ∇d 方向推才能减小 viol ⇒ ∂E/∂p = −2·w·viol·∇d
          g.segment<2>(2 * (i - 1)) += 2.0 * ctx->w_sdf * viol * Vec2(-grad[0], -grad[1]);
        }
      }
    }
    min_d = std::min(min_d, d);

    const Vec2 dp = p - ctx->anchor[i];
    cost += ctx->w_anchor * dp.squaredNorm();
    g.segment<2>(2 * (i - 1)) += 2.0 * ctx->w_anchor * dp;
  }
  ctx->min_d = (min_d == huge) ? 0.0 : min_d;
  ctx->shortfall = shortfall;
  return cost;
}

// ------------------------------------------------------------------ 积分 / 采样

/// `(θ, s)` 多项式 → XY 采样（Simpson 三点式），同时取各阶导数
struct RawPoint {
  double t{0.0};
  double theta{0.0};
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double v{0.0};       // ṡ
  double omega{0.0};   // θ̇
  double a{0.0};       // s̈
  double alpha{0.0};   // θ̈
};

void integrateTrajectory(const Trajectory<5, 2> & traj, double x0, double y0,
                         double dt, std::vector<RawPoint> & out)
{
  out.clear();
  const double total = traj.getTotalDuration();
  if (!(total > 0.0)) return;
  const int steps = std::max(1, static_cast<int>(std::ceil(total / std::max(1e-4, dt))));
  const double h = total / steps;
  double x = x0;
  double y = y0;

  auto emplace = [&](double t, const Eigen::VectorXd & p, const Eigen::VectorXd & v,
                     const Eigen::VectorXd & a) {
    RawPoint r;
    r.t = t;
    r.theta = p(0);
    r.s = p(1);
    r.x = x;
    r.y = y;
    r.v = v(1);
    r.omega = v(0);
    r.a = a(1);
    r.alpha = a(0);
    out.push_back(r);
  };

  emplace(0.0, traj.getPos(0.0), traj.getVel(0.0), traj.getAcc(0.0));
  for (int k = 0; k < steps; ++k) {
    const double t0 = h * k;
    const double tm = t0 + 0.5 * h;
    const double t1 = t0 + h;
    const Eigen::VectorXd p0 = traj.getPos(t0);
    const Eigen::VectorXd v0 = traj.getVel(t0);
    const Eigen::VectorXd pm = traj.getPos(tm);
    const Eigen::VectorXd vm = traj.getVel(tm);
    const Eigen::VectorXd p1 = traj.getPos(t1);
    const Eigen::VectorXd v1 = traj.getVel(t1);
    x += (h / 6.0) * (v0(1) * std::cos(p0(0)) + 4.0 * vm(1) * std::cos(pm(0)) +
                      v1(1) * std::cos(p1(0)));
    y += (h / 6.0) * (v0(1) * std::sin(p0(0)) + 4.0 * vm(1) * std::sin(pm(0)) +
                      v1(1) * std::sin(p1(0)));
    emplace(t1, p1, v1, traj.getAcc(t1));
  }
}

/// `(θ,s)` → MINCO（`Trajectory<5,2>` = 5 阶多项式 × 2 维，见 gcopter/minco.hpp；
/// 注意 vendor 里的命名空间实为 `minco`，而 `Trajectory` 是**全局**类型）
bool solveMinco(const Eigen::Matrix<double, 2, 3> & head, double theta_end,
                double s_total, double goal_v,
                const Eigen::Matrix<double, 2, Eigen::Dynamic> & inPs,
                const Eigen::VectorXd & ts, Trajectory<5, 2> & traj)
{
  const int n_piece = static_cast<int>(ts.size());
  if (n_piece < 1) return false;
  for (int i = 0; i < n_piece; ++i) {
    if (!(ts(i) > 1e-6)) return false;
  }
  // ★ 必须用 (2, Dynamic) 而不是 MatrixX2d —— Eigen 的 `MatrixX2d` 是
  //   `Matrix<double, Dynamic, 2>`（列固定 2 行可变），`MatrixX2d m(2, 53)` 在
  //   Release（NDEBUG，断言被关）下会**静默变成 2×2**，之后 m(0,52) 就是越界读。
  //   此处校验列数，就是被这个坑咬过后加的。
  if (inPs.cols() != n_piece - 1 || inPs.rows() != 2) return false;

  Eigen::Matrix<double, 2, 3> tail;
  tail.col(0) = Vec2(theta_end, s_total);
  tail.col(1) = Vec2(0.0, goal_v);
  tail.col(2) = Vec2::Zero();

  minco::MINCO_S3NU minco_opt;
  minco_opt.setConditions(head, tail, n_piece, Vec2(1.0, 1.0));
  minco_opt.setParameters(inPs, ts);
  minco_opt.getTrajectory(traj);
  return traj.getPieceNum() == n_piece;
}

/// 点到折线的距离（`path_shift_max` 用）
double distanceToPolyline(double px, double py, const std::vector<Pose2D> & poly)
{
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 1; i < poly.size(); ++i) {
    const double ax = poly[i - 1].x;
    const double ay = poly[i - 1].y;
    const double bx = poly[i].x;
    const double by = poly[i].y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > kEps) {
      t = ((px - ax) * dx + (py - ay) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0);
    }
    const double qx = ax + t * dx;
    const double qy = ay + t * dy;
    best = std::min(best, std::hypot(px - qx, py - qy));
  }
  return best;
}

}  // namespace

// ================================================================== 配置

bool MincoOptimizer::configure(const ParamReader & params)
{
  const std::string px = "traj";
  resample_ds_ = params.getDouble(px + ".resample_ds", resample_ds_);
  if (resample_ds_ < 0.02) resample_ds_ = 0.02;
  // 调试开关：临时改路点密度（定位"路点太密 ⇒ 带状系统病态"这类问题时用）
  if (const char * ds_env = std::getenv("PNC_TRAJ_DS")) {
    const double v = std::atof(ds_env);
    if (v >= 0.02) resample_ds_ = v;
  }
  smooth_margin_ = params.getDouble(px + ".smooth_margin", smooth_margin_);
  smooth_safe_ratio_ = params.getDouble(px + ".smooth_safe_ratio", smooth_safe_ratio_);
  smooth_ramp_len_ = params.getDouble(px + ".smooth_ramp_len", smooth_ramp_len_);
  smooth_w_smooth_ = params.getDouble(px + ".smooth_w_smooth", smooth_w_smooth_);
  smooth_w_sdf_ = params.getDouble(px + ".smooth_w_sdf", smooth_w_sdf_);
  smooth_w_anchor_ = params.getDouble(px + ".smooth_w_anchor", smooth_w_anchor_);
  smooth_iters_ = params.getInt(px + ".smooth_iters", smooth_iters_);
  smooth_min_step_ = params.getDouble(px + ".smooth_min_step", smooth_min_step_);
  v_max_ = params.getDouble(px + ".v_max", v_max_);
  a_max_ = params.getDouble(px + ".a_max", a_max_);
  w_max_ = params.getDouble(px + ".w_max", w_max_);
  alpha_max_ = params.getDouble(px + ".alpha_max", alpha_max_);
  v_ref_ = params.getDouble(px + ".v_ref", v_ref_);
  v_min_ = params.getDouble(px + ".v_min", v_min_);
  terminal_handle_back_ =
      params.getDouble(px + ".terminal_handle_back", terminal_handle_back_);
  time_step_ = params.getDouble(px + ".time_step", time_step_);
  if (time_step_ < 0.10) time_step_ = 0.10;
  min_piece_time_ = params.getDouble(px + ".min_piece_time", min_piece_time_);
  max_pieces_ = params.getInt(px + ".max_pieces", max_pieces_);
  time_margin_ = params.getDouble(px + ".time_margin", time_margin_);
  output_dt_ = params.getDouble(px + ".output_dt", output_dt_);
  integrate_dt_ = params.getDouble(px + ".integrate_dt", integrate_dt_);
  scan_dt_ = params.getDouble(px + ".scan_dt", scan_dt_);
  terminal_tol_ = params.getDouble(px + ".terminal_tol", terminal_tol_);
  terminal_iters_ = params.getInt(px + ".terminal_iters", terminal_iters_);
  max_solve_ms_ = params.getDouble(px + ".max_solve_ms", max_solve_ms_);
  max_length_m_ = params.getDouble(px + ".max_length_m", max_length_m_);
  clearance_exclude_s_ =
      params.getDouble(px + ".clearance_exclude_s", clearance_exclude_s_);
  check_ds_ = params.getDouble(px + ".check_ds", check_ds_);
  check_dyaw_ = params.getDouble(px + ".check_dyaw", check_dyaw_);
  check_stats_ds_ = params.getDouble(px + ".check_stats_ds", check_stats_ds_);
  check_stats_dyaw_ = params.getDouble(px + ".check_stats_dyaw", check_stats_dyaw_);
  check_hard_margin_ = params.getDouble(px + ".check_hard_margin", check_hard_margin_);
  check_warn_margin_ = params.getDouble(px + ".check_warn_margin", check_warn_margin_);

  // 退化值必须挡住（否则 MINCO 的带状矩阵会出现除零）
  if (v_max_ < 1e-3 || a_max_ < 1e-3 || w_max_ < 1e-3 || alpha_max_ < 1e-3) return false;
  if (v_ref_ < 1e-3) return false;
  if (!(min_piece_time_ > 0.0)) return false;
  if (time_margin_ < 1.0) time_margin_ = 1.0;
  if (output_dt_ < 0.01) output_dt_ = 0.01;
  if (integrate_dt_ < 0.005) integrate_dt_ = 0.005;
  if (scan_dt_ < 0.02) scan_dt_ = 0.02;
  if (terminal_iters_ < 0) terminal_iters_ = 0;
  if (smooth_iters_ < 0) smooth_iters_ = 0;
  if (max_pieces_ < 3) max_pieces_ = 3;
  if (check_ds_ < 0.005) check_ds_ = 0.005;
  if (check_dyaw_ < 0.02) check_dyaw_ = 0.02;
  if (check_stats_ds_ < 0.005) check_stats_ds_ = 0.005;
  if (check_stats_dyaw_ < 0.02) check_stats_dyaw_ = 0.02;

  // 终检参数：**在最后**汇总（要用已经读好的 clearance_exclude_s_）
  check_prm_.ds = check_ds_;
  check_prm_.dyaw = check_dyaw_;
  check_prm_.stats_ds = check_stats_ds_;
  check_prm_.stats_dyaw = check_stats_dyaw_;
  check_prm_.hard_margin = check_hard_margin_;
  check_prm_.warn_margin = check_warn_margin_;
  check_prm_.stats_exclude_s = clearance_exclude_s_;
  return true;
}

void MincoOptimizer::setMap(std::shared_ptr<const CostMap2D> map) { map_ = std::move(map); }
void MincoOptimizer::setClearanceField(const ClearanceField * cf) { cf_ = cf; }
void MincoOptimizer::setCollisionChecker(const FootprintCollisionChecker * ck) { ck_ = ck; }
void MincoOptimizer::reset()
{
  smoothed_.clear();
  smooth_d_target_ = 0.0;
}

// ================================================================== 主流程

TrajOptResult MincoOptimizer::optimize(const TrajOptRequest & req)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsedMs = [&t0]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
        .count();
  };

  TrajOptResult res;
  smoothed_.clear();
  smooth_d_target_ = 0.0;

  if (req.path.size() < 2) {
    res.status = TrajStatus::kNoInput;
    res.message = "输入折线点数 < 2";
    res.stats.solve_ms = elapsedMs();
    return res;
  }
  if (!map_) {
    res.status = TrajStatus::kNoInput;
    res.message = "没有全局地图（setMap 未调用）";
    res.stats.solve_ms = elapsedMs();
    return res;
  }

  // ---- 0) 端点守卫：起/终点**真实轮廓**（safe_margin = 0）压在致命格上 ⇒ 不规划。
  //   ★ 与 A* 的虚拟起点守卫**同一个口径**（astar_planner.cpp："起点在余量带里 ⇒
  //     换虚拟起点；真实轮廓都放不下 ⇒ 停车报错交人工"）。差别只是分工：
  //     A* 负责"从合法位姿出发找一条路"，把车从那 10 cm 挪出来交给局部；
  //     而局部（MPC 跟的）正是本层输出的轨迹 —— 所以车真的压上去时本层必须
  //     同样拒绝，否则 A* 拒了、本层却硬算，口径就分叉了。
  //   ★ 为什么"拒绝"比"硬算"好（实测依据）：
  //     · 起点压障：`30 段/80.2 s, 时间×6.85, 终点残差 0.000 m`；
  //     · **终点压障**：`3894 点/194.64 s/17.20 m, max|v| 0.160 m/s, 时间×3.95`。
  //     两者都"能跑"，但：① 净距统计/终点修正全建在假前提上，数字不可信；
  //     ② 掩盖了"定位错了/地图错了/目标点选错了"这个真故障；
  //     ③ 末状态是**硬约束** ⇒ 目标不可达时末端修正只能硬够，净距必然被压穿；
  //     goal_checker 又永远判不到点 ⇒ 任务卡死。
  //   ★ 只拒"真实轮廓压上"：落在**余量带**里是常态（全局图与感知图差几个 cm、
  //     用户把目标点得离墙近一点），那种情况必须照常规划。
  if (ck_ != nullptr) {
    // 端点朝向：请求给了就用；没给就取路径首/末段方向（与轨迹端点朝向同一取法）
    auto segYaw = [&](std::size_t a, std::size_t b) {
      const double dx = req.path[b].x - req.path[a].x;
      const double dy = req.path[b].y - req.path[a].y;
      return (std::hypot(dx, dy) > 1e-6) ? std::atan2(dy, dx) : 0.0;
    };
    double yaw_start = req.start.yaw;
    if (!req.start.has_yaw) {
      for (std::size_t i = 1; i < req.path.size(); ++i) {
        if (std::hypot(req.path[i].x - req.path[i - 1].x,
                       req.path[i].y - req.path[i - 1].y) > 1e-6) {
          yaw_start = segYaw(i - 1, i);
          break;
        }
      }
    }
    double yaw_goal = req.goal.yaw;
    if (!req.goal.has_yaw) {
      for (std::size_t i = req.path.size(); i-- > 1;) {
        if (std::hypot(req.path[i].x - req.path[i - 1].x,
                       req.path[i].y - req.path[i - 1].y) > 1e-6) {
          yaw_goal = segYaw(i - 1, i);
          break;
        }
      }
    }
    if (ck_->poseInCollisionNoMargin(req.start.x, req.start.y, yaw_start)) {
      const double pen =
          -ck_->signedClearanceAt(req.start.x, req.start.y, yaw_start);   // > 0 = 穿透深度
      char buf[512];
      std::snprintf(buf, sizeof(buf),
                    "起点不可规划：车体真实轮廓已压在障碍上（穿透 ≥ %.3f m）。"
                    "按 A* 起点守卫同口径拒绝 —— 应停车报错交人工（常见原因：定位"
                    "漂移/地图与实际不符），不要从重叠位置规划",
                    pen);
      res.status = TrajStatus::kStartBlocked;
      res.samples.clear();
      res.message = buf;
      res.stats.note = buf;
      res.stats.solve_ms = elapsedMs();
      return res;
    }
    if (ck_->poseInCollisionNoMargin(req.goal.x, req.goal.y, yaw_goal)) {
      const double pen = -ck_->signedClearanceAt(req.goal.x, req.goal.y, yaw_goal);
      char buf[512];
      std::snprintf(buf, sizeof(buf),
                    "终点不可规划：目标位姿的车体真实轮廓已压在障碍上（穿透 ≥ %.3f m）。"
                    "轨迹末状态是**硬约束**，目标不可达时末端修正只能硬够 ⇒ 净距必被"
                    "压穿、goal_checker 永远判不到点。按起点守卫同口径拒绝（常见原因："
                    "目标点选在墙里/障碍上，或全局图与实际不符）",
                    pen);
      res.status = TrajStatus::kGoalBlocked;
      res.samples.clear();
      res.message = buf;
      res.stats.note = buf;
      res.stats.solve_ms = elapsedMs();
      return res;
    }
  }

  // ---- 1) 等距重采样，并强制锚定首（车）末（目标）点
  std::vector<Pose2D> pts;
  if (!resample(req.path, pts)) {
    res.status = TrajStatus::kNoInput;
    res.message = "重采样后点数 < 2（路径过短）";
    res.stats.solve_ms = elapsedMs();
    return res;
  }
  pts.front().x = req.start.x;
  pts.front().y = req.start.y;
  pts.back().x = req.goal.x;
  pts.back().y = req.goal.y;

  // ---- 1b) 长度上限：只优化前 `max_length_m_` 米
  //   为什么：MPC 前瞻只有 1~2 m，没必要对整条（可能几十上百米的）路做全量优化；
  //   而代价是随长度线性涨的（平滑的变量数、MINCO 段数、统计点数）。
  //   ★ 截断时终点变成**途经点**：不减速（末速 = 巡航速度）、终点朝向取切线
  //     （不要把它当成"到点停车 + 对正"）。
  TrajOptRequest eff = req;
  {
    double acc = 0.0;
    std::size_t keep = pts.size();
    bool cut = false;
    if (max_length_m_ > 0.0) {
      for (std::size_t i = 1; i < pts.size(); ++i) {
        const double seg = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        acc += seg;
        if (acc >= max_length_m_) {
          const double r = (seg > 1e-9) ? (max_length_m_ - (acc - seg)) / seg : 0.0;
          pts[i].x = pts[i - 1].x + (pts[i].x - pts[i - 1].x) * std::clamp(r, 0.0, 1.0);
          pts[i].y = pts[i - 1].y + (pts[i].y - pts[i - 1].y) * std::clamp(r, 0.0, 1.0);
          keep = i + 1;
          cut = true;
          break;
        }
      }
    }
    if (cut) {
      pts.resize(keep);
      eff.use_goal_yaw = false;                                    // 途经点：不对正
      eff.goal_v = std::max(0.20, std::min(v_max_, v_ref_));        // 途经点：不停车
      eff.goal = pts.back();
      eff.goal.has_yaw = false;
      res.stats.note += "已按 max_length_m=" + std::to_string(max_length_m_).substr(0, 4) +
                        " 截断（终点为途经点，不减速不对正）";
    }
  }

  // ---- 2) Stage A：避障 + 平滑（几何）
  const auto t_smooth = std::chrono::steady_clock::now();
  const double smooth_cost = smoothPath(pts, res.stats);
  res.stats.smooth_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_smooth)
          .count();
  smoothed_ = pts;
  (void)smooth_cost;

  // ---- 3) Stage B：MINCO 时间参数化（平滑 + 运动学 + 终点）
  const auto t_minco = std::chrono::steady_clock::now();
  std::string err;
  if (!buildTrajectory(eff, pts, res.samples, res.stats, err)) {
    res.status = TrajStatus::kSolverFailed;
    res.message = err;
    res.samples.clear();
    res.stats.solve_ms = elapsedMs();
    return res;
  }

  res.stats.minco_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_minco)
          .count();

  // ---- 4) **两道交付门**：运动学终验（跟得上跟不上）+ M3 终检（碰不碰）。
  //   ★ 两者**独立**：终检只管净距、不管运动学；缩放只管运动学、不管净距。
  //     少任何一道都会让"能过另一道但不可执行"的轨迹漏出去。
  //   ★ `samples` **保留**（诊断/RViz 要看失败在哪），但状态已说明不得执行 ——
  //     调用方按 `status != kSuccess` 处理（M4：降级 A* 原路径，任务照走）。
  if (!res.stats.kin_ok) {
    res.status = TrajStatus::kKinematicsFailed;
    res.message = res.stats.note;
    res.stats.solve_ms = elapsedMs();
    return res;
  }
  if (!res.stats.check_ok) {
    res.status = TrajStatus::kCheckFailed;
    res.message = res.stats.check_note + "；" + res.stats.note;
    res.stats.solve_ms = elapsedMs();
    return res;
  }

  res.status = TrajStatus::kSuccess;
  res.stats.solve_ms = elapsedMs();
  // 分阶段耗时写进消息（节点直接打出来，一眼看出慢在哪一段）
  char tb[128];
  std::snprintf(tb, sizeof(tb), " 耗时：平滑 %.1f / MINCO %.1f / 统计 %.1f ms（共 %.1f）",
                res.stats.smooth_ms, res.stats.minco_ms, res.stats.stats_ms,
                res.stats.solve_ms);
  res.stats.note += tb;
  if (res.stats.solve_ms > max_solve_ms_) {
    res.stats.note += "  ⚠ 求解耗时超预算（" + std::to_string(res.stats.solve_ms) + " ms）";
  }
  res.message = res.stats.note;
  return res;
}

bool MincoOptimizer::resample(const std::vector<Pose2D> & in, std::vector<Pose2D> & out) const
{
  out.clear();
  if (in.size() < 2) return false;
  const double ds = std::max(0.02, resample_ds_);
  out.push_back(in.front());
  double need = ds;   // 距上一个输出点还差多少
  for (std::size_t i = 1; i < in.size(); ++i) {
    const Pose2D & a = in[i - 1];
    const Pose2D & b = in[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < kEps) continue;
    double t = 0.0;
    while (need <= seg - t) {
      t += need;
      need = ds;
      Pose2D p;
      p.x = a.x + (b.x - a.x) * (t / seg);
      p.y = a.y + (b.y - a.y) * (t / seg);
      out.push_back(p);
    }
    need -= (seg - t);
  }
  const Pose2D & last = in.back();
  if (std::hypot(last.x - out.back().x, last.y - out.back().y) > 1e-6) {
    out.push_back(last);
  } else {
    out.back() = last;   // 末点以输入为准（含 yaw/has_yaw）
  }
  return out.size() >= 2;
}

double MincoOptimizer::smoothPath(std::vector<Pose2D> & pts, TrajOptStats & st)
{
  const int n = static_cast<int>(pts.size());
  const double body_r = (ck_ != nullptr) ? ck_->footprint().circumscribedRadius() : 0.0;
  smooth_d_target_ = body_r + std::max(0.0, smooth_margin_);
  st.smooth_d_target = smooth_d_target_;

  // ---- 端点自适应目标（**机制 ①**，DDR-opt：`safeDis = min(ratio × 端点实测净距, max)`）
  //   ★★ 默认**关闭**（`smooth_safe_ratio_ <= 0`）。实测结论（见 yaml/头文件）：
  //     降低目标 ⇒ 罚项提前归零 ⇒ **实际达成净距变低**（贴墙案例 0.554 → 0.439），
  //     而终检查的是实际净距 ⇒ 开它只会让拦停更频繁。
  //     原版用它是对付"起点在死角落里"（那个场景下宁可少推也不要把路径挤歪），
  //     与我们的判据方向相反 ⇒ 默认关，需要时可用 ratio > 0 打开。
  auto endCap = [&](const Pose2D & p) {
    if (smooth_safe_ratio_ <= 0.0) return smooth_d_target_;   // 关闭
    if (cf_ == nullptr || !cf_->valid()) return smooth_d_target_;
    const double d = cf_->lowerBoundAtWorld(p.x, p.y, 1.0e3);
    return std::min(smooth_d_target_, smooth_safe_ratio_ * d);
  };
  const double d_cap_head = endCap(pts.front());
  const double d_cap_tail = endCap(pts.back());
  double L_total = 0.0;
  for (int i = 1; i < n; ++i) {
    L_total += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  const double ds_est = L_total / std::max(1, n - 1);
  const double ramp_steps =
      std::max(1.0, smooth_ramp_len_ / std::max(1e-3, ds_est));

  SmoothCtx ctx;
  ctx.cf = cf_;
  ctx.map = map_.get();
  ctx.w_smooth = smooth_w_smooth_;
  ctx.w_sdf = smooth_w_sdf_;
  ctx.w_anchor = smooth_w_anchor_;
  ctx.d_target = smooth_d_target_;
  ctx.pts.resize(static_cast<std::size_t>(n));
  ctx.anchor.resize(static_cast<std::size_t>(n));
  ctx.d_tgt.assign(static_cast<std::size_t>(n), smooth_d_target_);
  for (int i = 0; i < n; ++i) {
    const double w_head = std::min(1.0, static_cast<double>(i) / ramp_steps);
    const double w_tail = std::min(1.0, static_cast<double>(n - 1 - i) / ramp_steps);
    const double dh = d_cap_head + (smooth_d_target_ - d_cap_head) * w_head;
    const double dt = d_cap_tail + (smooth_d_target_ - d_cap_tail) * w_tail;
    ctx.d_tgt[static_cast<std::size_t>(i)] =
        std::min(smooth_d_target_, std::min(dh, dt));
    ctx.pts[static_cast<std::size_t>(i)] = Vec2(pts[i].x, pts[i].y);
  }
  ctx.anchor = ctx.pts;
  st.smooth_d_cap_head = d_cap_head;
  st.smooth_d_cap_tail = d_cap_tail;

  const int m = n - 2;
  double f = 0.0;
  if (m >= 1) {
    Eigen::VectorXd x(2 * m);
    for (int i = 0; i < m; ++i) {
      x(2 * i) = ctx.pts[static_cast<std::size_t>(i + 1)].x();
      x(2 * i + 1) = ctx.pts[static_cast<std::size_t>(i + 1)].y();
    }
    Eigen::VectorXd g(2 * m);
    lbfgs::lbfgs_parameter_t prm;
    prm.mem_size = 16;
    prm.g_epsilon = 1.0e-6;
    prm.delta = 1.0e-4;
    prm.min_step = smooth_min_step_;
    prm.max_iterations = smooth_iters_;
    const int ret = lbfgs::lbfgs_optimize(x, f, evaluateSmooth, nullptr, nullptr, &ctx, prm);
    std::string tag;
    if (ret == lbfgs::LBFGS_CONVERGENCE || ret == lbfgs::LBFGS_STOP) {
      tag = "收敛";
    } else if (ret == lbfgs::LBFGSERR_MAXIMUMITERATION) {
      tag = "到迭代上限";
    } else {
      tag = std::string("结束(") + lbfgs::lbfgs_strerror(ret) + ")";
    }
    // 抄回最终点（L-BFGS 的返回 x 就是最后一次评估的点）
    for (int i = 0; i < m; ++i) {
      ctx.pts[static_cast<std::size_t>(i + 1)] = Vec2(x(2 * i), x(2 * i + 1));
    }
    st.note += "平滑[" + tag + "," + std::to_string(ctx.evals) + "次评估]";
  }

  // 写回（并夹进地图，越界点不能留给 MINCO）
  for (int i = 1; i < n - 1; ++i) {
    Vec2 p = ctx.pts[static_cast<std::size_t>(i)];
    if (map_) clampIntoMap(*map_, p);
    pts[static_cast<std::size_t>(i)].x = p.x();
    pts[static_cast<std::size_t>(i)].y = p.y();
  }
  st.smooth_iterations = static_cast<double>(ctx.evals);
  st.smooth_d_achieved = ctx.min_d;
  st.smooth_shortfall = ctx.shortfall;
  // ★ 报"**缺口**"而不是"达成 vs 目标"：目标现在是**逐点**的（端点附近自动下调），
  //   拿 min_d 去比那个全局上界会恒定报"未达成"，把真正的问题淹掉。
  //   缺口 = max(逐点目标 − 实际) ⇒ 0 就说明"平滑在自己的目标下做到了"。
  st.note += " 净距目标 " + std::to_string(smooth_d_target_).substr(0, 4) + "m/达成 " +
             std::to_string(ctx.min_d).substr(0, 4) + "m/缺口 " +
             std::to_string(ctx.shortfall).substr(0, 4) + "m";
  if (d_cap_head < smooth_d_target_ - 1e-6 || d_cap_tail < smooth_d_target_ - 1e-6) {
    st.note += "（端点自适应下限 首 " + std::to_string(d_cap_head).substr(0, 4) +
               "/末 " + std::to_string(d_cap_tail).substr(0, 4) + "m）";
  }
  if (ctx.outside > 0) {
    st.note += " (越界点 " + std::to_string(ctx.outside) + " 次被拉回)";
  }
  return f;
}

bool MincoOptimizer::buildTrajectory(const TrajOptRequest & req,
                                     const std::vector<Pose2D> & pts,
                                     std::vector<TrajSample> & out, TrajOptStats & st,
                                     std::string & err)
{
  const auto t0 = std::chrono::steady_clock::now();
  auto elapsedMs = [&t0]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
        .count();
  };

  const int n = static_cast<int>(pts.size());
  if (n < 2) {
    err = "路径点数 < 2";
    return false;
  }

  // ---- 折线累计弧长
  std::vector<double> s(static_cast<std::size_t>(n), 0.0);
  for (int i = 1; i < n; ++i) {
    const double dx = pts[static_cast<std::size_t>(i)].x - pts[static_cast<std::size_t>(i - 1)].x;
    const double dy = pts[static_cast<std::size_t>(i)].y - pts[static_cast<std::size_t>(i - 1)].y;
    s[static_cast<std::size_t>(i)] += s[static_cast<std::size_t>(i - 1)] + std::hypot(dx, dy);
  }
  const double L = s.back();
  if (L < 1e-3) {
    err = "路径总弧长 < 1 mm";
    return false;
  }

  // ---- 速度剖面：**曲率感知 + 加速度受限**的前后向扫描
  //   ★ 为什么不能用"纯时间的梯形"（几何盲）：A* 折线的直角曲率**无穷大**，
  //     不按曲率限速就会逼 MINCO 在极短时间内转 90°（实测要求 ω=4.09 rad/s，
  //     限值 0.65）⇒ 时间缩放把**整条轨迹**拉长 6.3 倍（16 m 的路要 176 s）。
  //   做法：v_cap(s) = min(v_ref, ω_max/κ) 给出"几何允许的速度"，再做一次
  //   前向 + 后向扫描保证 |a| ≤ a_max ⇒ 弯道**局部**减速，而不是全路变慢。
  //   起点/末速由请求给出（v0 / goal_v）。
  const double v0p = std::max(0.0, req.start_v);
  const double vep = std::max(0.0, req.goal_v);
  const double vcp = std::max(0.20, std::min(v_max_, v_ref_));
  // ★ 剖面加速度只用 a_max 的 **一半**：MINCO 要同时满足 C² 与路点位置约束，
  //   它的加速度**峰值**必然高于分段剖面（剖面 v(s) 是分段线性、折点处 jerk 是
  //   冲激，多项式只能用有限峰值去逼近）⇒ 顶到 a_max 的话多项式一定越界，
  //   解析缩放再整体乘 k。留一半余量后峰值约 0.25，离 a_max=0.5 还有 2 倍，
  //   缩放退出（k≈1），速度才**真的**按剖面走。
  //   注：曾经怀疑"多项式跟不上剖面形状"，实测证伪 —— 真正的越界来自端点速度
  //   被地板抬高（见下面 ② 的注释），修掉后这一段余量只是安全垫。
  const double acp = std::max(0.05, a_max_ * 0.50);
  // ★★ 转向速率的**有效系数**：C² 多项式在两个路点**之间**的 ω 峰值会过冲到路点
  //   斜率的 ~1.6 倍（与 `acp = 0.5·a_max` 完全同一个道理：路点只约束端点，
  //   多项式要 C² 连通就得过冲）。实测：路点斜率限到 0.8·ω_max 时，多项式 ω 峰值
  //   反而到 1.26·ω_max（⇒ 时间缩放又乘 k=1.26）。
  //   ⇒ 取 1/1.6 ≈ 0.6。**并且剖面的 κ 上限必须用同一个系数**：否则剖面按
  //     "ω_max 允许的速度"给时间、路点却只能跑到 0.6·ω_max ⇒ 路点被限幅卡住、
  //     时间算不齐、缩放又得开。（两处不一致 = 两套剖面，这个错我们已经犯过。）
  const double rate_rho_w = 0.60;
  const double rate_rho_a = 0.50;
  // 切线基线半宽（**剖面与路点共用同一个**，否则两处 θ 不一致）
  const double tan_base = std::min(0.25, 0.25 * L);

  // 折线上按弧长取点（顺次扫描；pts 已等距重采样，线性插值足够）
  auto atS = [&](double sv, double & px, double & py) {
    std::size_t seg = 1;
    while (seg < static_cast<std::size_t>(n) - 1 && s[seg] < sv) ++seg;
    const double s0 = s[seg - 1];
    const double s1 = s[seg];
    const double r = (s1 > s0) ? std::clamp((sv - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
    px = pts[seg - 1].x + (pts[seg].x - pts[seg - 1].x) * r;
    py = pts[seg - 1].y + (pts[seg].y - pts[seg - 1].y) * r;
  };

  // ① 转向速率 → 速度上限（**判据取自轨迹真正要走的那条 θ(s)**）
  //   ★★ 为什么必须从 θ(s) 推，而不是从折线几何另算一份曲率：
  //     剖面的时间下界来自 ω = v·dθ/ds ⇒ dθ/ds 只能取"θ(s)"。旧代码用几何三点式
  //     算 κ，栽在两个坑上（都有实测取证）：
  //     · **端点退化**：i=0 处 atS(s−win) 与 atS(s) 是**同一个点** ⇒ atan2(0,0)
  //       返回 0 被当成"入方向" ⇒ 转角被算成"相对 0 朝向的绝对朝向" ⇒
  //       κ(0) = 14.0（= 2.1 rad / 0.15 m），纯属伪造。
  //     · **漏掉起点朝向差**：θ(0) = req.start.yaw 是**车的实际朝向**，而 A* 从当前格
  //       朝任意方向迈第一步（不看车朝向）⇒ 实测两者差 2.1 rad。曲率只描述折线自己，
  //       完全看不见这个差 ⇒ 不给它分配任何时间 ⇒ 未缩放解 max|ω| = 6.57 rad/s
  //       （限 0.65）⇒ 时间缩放 ×10.1 ⇒ 实测 5.62 m 走出 144.75 s、max|v| 只剩 0.070 m/s。
  //     （这是与"端点速度被地板抬高"同一类错误：**剖面看不见某个状态约束**。）
  //   做法：按"与路点同一套规则"先生成 θ(s)，再取两个 κ 估计的**大者**：
  //     · **逐段必要条件** |Δθ_i|/Δs_i —— 段内转了 Δθ 且 |ω| ≤ ω_max ⇒ Δt ≥ |Δθ|/ω_max
  //       ⇒ v ≤ ω_max·Δs/|Δθ|。对离散 θ 序列**精确成立**，起点朝向差天然落在 i=1 段。
  //     · ±tan_base 基线窗口 |Δθ|/Δs —— 只当**平滑偏好**（逐段估计在有噪声的轨迹上
  //       偏大，会让弯道过分减速）。
  std::vector<double> theta_d(static_cast<std::size_t>(n), 0.0);
  std::vector<double> v_prof(static_cast<std::size_t>(n), vcp);
  double kappa_max = 0.0;
  double kappa_at_s = 0.0;
  double v_kappa_min = vcp;      // 曲率/转向速率允许的最低速（诊断用）
  double v_kappa_min_s = 0.0;
  {
    // (a) θ(s)：切线用 ±tan_base 基线（相邻两点在毫米级基线上算切线 = 噪声）
    auto tangentAt = [&](double sv) {
      double x0 = 0.0;
      double y0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      atS(std::max(0.0, sv - tan_base), x0, y0);
      atS(std::min(L, sv + tan_base), x1, y1);
      if (std::hypot(x1 - x0, y1 - y0) < 1e-6) return theta_d[0];
      return std::atan2(y1 - y0, x1 - x0);
    };
    theta_d[0] = req.start.has_yaw
                     ? req.start.yaw
                     : std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
    for (int i = 1; i < n; ++i) {
      const double tg = tangentAt(s[static_cast<std::size_t>(i)]);
      theta_d[static_cast<std::size_t>(i)] =
          theta_d[static_cast<std::size_t>(i - 1)] +
          wrapPi(tg - theta_d[static_cast<std::size_t>(i - 1)]);
    }

    // (b) 逐段必要条件（段 i = [i−1, i]）
    std::vector<double> k_seg(static_cast<std::size_t>(n), 0.0);
    for (int i = 1; i < n; ++i) {
      const double dsv = std::max(
          1e-6, s[static_cast<std::size_t>(i)] - s[static_cast<std::size_t>(i - 1)]);
      k_seg[static_cast<std::size_t>(i)] =
          std::fabs(wrapPi(theta_d[static_cast<std::size_t>(i)] -
                           theta_d[static_cast<std::size_t>(i - 1)])) / dsv;
    }
    k_seg[0] = k_seg[1];   // 端点：借用邻段（i=0 没有"前一段"）

    for (int i = 0; i < n; ++i) {
      // 点 i 是段 i 的终点、段 i+1 的起点 ⇒ 两个段的界都要满足
      const double k1 =
          std::max(k_seg[static_cast<std::size_t>(i)],
                   (i + 1 < n) ? k_seg[static_cast<std::size_t>(i + 1)] : 0.0);
      // 基线窗口（平滑偏好）
      int jl = i;
      int jh = i;
      while (jl > 0 && s[static_cast<std::size_t>(jl - 1)] >=
                           s[static_cast<std::size_t>(i)] - tan_base) {
        --jl;
      }
      while (jh < n - 1 && s[static_cast<std::size_t>(jh + 1)] <=
                               s[static_cast<std::size_t>(i)] + tan_base) {
        ++jh;
      }
      double k_win = k1;
      if (jh > jl) {
        const double dsw = std::max(
            1e-6, s[static_cast<std::size_t>(jh)] - s[static_cast<std::size_t>(jl)]);
        k_win = std::fabs(wrapPi(theta_d[static_cast<std::size_t>(jh)] -
                                 theta_d[static_cast<std::size_t>(jl)])) /
                dsw;
      }
      const double kap = std::max(k1, k_win);
      if (kap > kappa_max) {
        kappa_max = kap;
        kappa_at_s = s[static_cast<std::size_t>(i)];
      }
      const double vk = (kap > 1e-9) ? (rate_rho_w * w_max_ / kap) : vcp;
      // ★ 这里**没有 v_min 地板**：地板曾经写成 max(v_floor, …)，实测把 κ=14 处的
      //   上限 0.046 抬到 0.2（4 倍）⇒ 剖面谎报速度 ⇒ 多项式用超限 ω 去追。
      //   地板只能当**告警阈值**（见下面的 note），不能覆盖运动学上限。
      const double vlim = std::max(1e-2, std::min(vcp, vk));
      if (vlim < v_kappa_min) {
        v_kappa_min = vlim;
        v_kappa_min_s = s[static_cast<std::size_t>(i)];
      }
      v_prof[static_cast<std::size_t>(i)] = vlim;
    }
  }
  // ② 前向 + 后向扫描：保证 |dv/dt| ≤ acp（dv/ds = a/v）
  //   ★★ 端点速度（一）**必须如实**取请求给的边界值（可能是 0 = 静止起步），
  //     早先被 `v_min` 地板写成 0.2 m/s ⇒ 剖面声称"0.084 m 用 0.4 s 走完"，
  //     而从静止、|a| ≤ 0.25 走完 0.084 m 至少需要 √(2·0.084/0.25) = 0.82 s。
  //     少算的时间只能靠**超限加速度**补（实测 max|a| = 1.567 = 6.3×acp）
  //     ⇒ 缩放 k=1.78 ⇒ 8 m 直线 27 s、max_v 从 0.9 掉到 0.508。
  //   ★★ 端点速度（二）也**不能反过来盖过运动学上限**：车已经在弯里太快时
  //     （v0p > 该处 κ 允许的速度），只能取小者 —— 硬装快会逼出超限 ω/a，
  //     最后还是靠时间缩放拉长。⇒ 两处都用 `min`，**不要**再写 `= v0p`。
  //   ★ 递推式 v_i = √(v_{i-1}² + 2·acp·Δs) 与"恒定 a"是同一件事，所以
  //     后面 t 的累积式 2Δs/(v_i + v_{i-1}) 是**精确**的（不是近似梯形）。
  {
    // ★ 端点速度**必须如实**取请求给的边界值，但也**不能反过来盖过运动学上限**：
    //   车已经在弯里太快时（v0p > 该处允许速度），只能取小者 —— 硬装快会逼出
    //   超限 ω/a，最后还是靠时间缩放拉长。
    v_prof[0] = std::min(v0p, v_prof[0]);
    for (int i = 1; i < n; ++i) {
      const double dsv = std::max(1e-6, s[static_cast<std::size_t>(i)] - s[static_cast<std::size_t>(i - 1)]);
      const double v = std::sqrt(v_prof[static_cast<std::size_t>(i - 1)] * v_prof[static_cast<std::size_t>(i - 1)] +
                                 2.0 * acp * dsv);
      v_prof[static_cast<std::size_t>(i)] = std::min(v_prof[static_cast<std::size_t>(i)], v);
    }
    v_prof[static_cast<std::size_t>(n - 1)] = std::min(vep, v_prof[static_cast<std::size_t>(n - 1)]);
    for (int i = n - 2; i >= 0; --i) {
      const double dsv = std::max(1e-6, s[static_cast<std::size_t>(i + 1)] - s[static_cast<std::size_t>(i)]);
      const double v = std::sqrt(v_prof[static_cast<std::size_t>(i + 1)] * v_prof[static_cast<std::size_t>(i + 1)] +
                                 2.0 * acp * dsv);
      v_prof[static_cast<std::size_t>(i)] = std::min(v_prof[static_cast<std::size_t>(i)], v);
    }
    // 注：这里**不要**再写 `v_prof[0] = v0p; v_prof[n-1] = vep;` 兜底 ——
    // 那会把上面辛苦算出来的运动学上限（可能只有 0.046 m/s）直接盖回 0.6。
    // 端点的 `min` 已在扫描前做过。
  }
  // ③ 剖面的 t(s)：**必须用"区间内恒定加速度"的精确关系**，不能线性插值。
  //   前向/后向扫描给出 v_i² = v_{i-1}² + 2·a_i·Δs_i，所以区间内运动满足
  //   v² = v_{i-1}² + 2a_i·(s − s_{i-1})，对应 Δt = (v − v_{i-1})/a_i。
  //   ★ 用"v 在 s 上线性插值"会**高估** v（弦在 √s 曲线之上，v ∝ √s 是凸的）：
  //     实测起点处剖面说"s = 0.053 m 用时 0.39 s"，而真实（静止起步、a ≤ 0.25）
  //     需要 √(2·0.053/0.25) = 0.65 s ⇒ 路点被放到**物理上到不了**的位置 ⇒
  //     多项式只能靠超限加速度补 ⇒ 缩放仍乘 1.45（这一段就是修掉端点不自洽后
  //     剩下的残差）。
  //   区间时长用 2Δs/(v_i + v_{i-1})：恒定加速度下它与 (v_i − v_{i-1})/a_i **恒等**，
  //   所以 t_prof 是精确的（不是梯形近似）。
  std::vector<double> t_prof(static_cast<std::size_t>(n), 0.0);
  std::vector<double> a_seg(static_cast<std::size_t>(n), 0.0);   // a_seg[i] = 第 i 段的加速度
  for (int i = 1; i < n; ++i) {
    const double dsv = std::max(1e-6, s[static_cast<std::size_t>(i)] - s[static_cast<std::size_t>(i - 1)]);
    const double v0 = v_prof[static_cast<std::size_t>(i - 1)];
    const double v1 = v_prof[static_cast<std::size_t>(i)];
    a_seg[static_cast<std::size_t>(i)] = (v1 * v1 - v0 * v0) / (2.0 * dsv);
    t_prof[static_cast<std::size_t>(i)] =
        t_prof[static_cast<std::size_t>(i - 1)] + dsv / std::max(1e-3, 0.5 * (v0 + v1));
  }
  const double t_total = t_prof.back();

  /// s → t（剖面反查；用恒定加速度的精确关系）
  auto timeAtS = [&](double sv) {
    std::size_t seg = 1;
    while (seg < static_cast<std::size_t>(n) - 1 && s[seg] < sv) ++seg;
    const double v0 = v_prof[seg - 1];
    const double a = a_seg[seg];
    const double ds = std::max(0.0, sv - s[seg - 1]);
    if (std::fabs(a) < 1e-9) {
      return t_prof[seg - 1] + ds / std::max(1e-3, v0);
    }
    const double v = std::sqrt(std::max(0.0, v0 * v0 + 2.0 * a * ds));
    return t_prof[seg - 1] + (v - v0) / a;
  };

  /// t → s（路点取法用；与 timeAtS 互逆，都是恒定加速度的精确式）
  auto sAtTime = [&](double t) {
    std::size_t lo = 0;
    std::size_t hi = t_prof.size() - 1;
    while (hi - lo > 1) {
      const std::size_t mid = (lo + hi) / 2;
      if (t_prof[mid] <= t) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    const double dt = std::clamp(t - t_prof[lo], 0.0, t_prof[hi] - t_prof[lo]);
    const double v0 = v_prof[lo];
    const double a = a_seg[hi];
    const double ds = v0 * dt + 0.5 * a * dt * dt;
    return std::clamp(s[lo] + ds, s[lo], s[hi]);
  };

  // ---- 路点取法：**时间等分 + 弧长按剖面走**
  //   ★ 反过来（弧长等分 + 时间按剖面走）会让每段时间在 3~4 倍范围内变化而 s 的
  //     增量均匀 ⇒ 带状系统解出剧烈摆动（实测：未缩放解中段峰值 2.15 m/s、
  //     s 冲过终点 3 m 再回来，且与路点密度无关）。时间等分 + 弧长非均匀
  //     与剖面自洽，解出来就是剖面本身。
  //   ★★ **段数上限 60 是有实测依据的**（`max_pieces_`）；曾调到 120，实测**更差**
  //     （单测 k 1.000 → 1.128、`max|a|` 0.5 → 0.637）。原因：a 通道的瓶颈不在
  //     分辨率，而在**末段** —— 实测 `max|a|` 总是出现在终点，且
  //     `max|a| = 次末路点速度 / T_step`（0.27 / 0.43 = 0.63 = 2.5·acp）
  //     ⇒ 末段必须在**一个 T_step 内**把速度降到 0。要治的是"终点减速段的
  //     时间分配"，不是路点密度。
  const int n_piece = std::clamp(
      static_cast<int>(std::ceil(t_total / std::max(0.10, time_step_))), 3,
      std::max(3, max_pieces_));
  const double T_step = std::max(min_piece_time_, t_total / n_piece);
  Eigen::VectorXd ts(n_piece);
  ts.setConstant(T_step);

  std::vector<double> knot_s(static_cast<std::size_t>(n_piece) + 1, 0.0);
  std::vector<double> knot_th(static_cast<std::size_t>(n_piece) + 1, 0.0);
  // ★★★ 路点朝向必须**对"路径切线参考"做转向速率限幅**，不能直接把 θ 钉在切线上。
  //   为什么：θ 只能作为**路点约束**进入 MINCO。若把它钉在切线上，而 θ(0) = 车的
  //   实际朝向与首段切线差 Δθ（实测 2.1 rad），MINCO 就**必须**在一个 Δt 内转完
  //   Δθ ⇒ ω 必然超限 ⇒ 时间缩放把整条轨迹拉长（实测 ×10.1 ⇒ 5.62 m 走 144.75 s）。
  //   限幅后 |Δθ_j| ≤ ρ·ω_max·Δt_j **由构造保证**（Δθ_j 是路点之间唯一的转向接口），
  //   多项式只能在界内转；转不完的部分表现为"轨迹暂时滞后于切线"——起点附近就是
  //   **原地转**（v ≈ ω_max·Δs/Δθ ≈ 0.05 m/s），这段的时间由剖面的 κ 上限
  //   （必要条件 v ≤ ω_max·Δs/|Δθ|）已经补齐，两者是同一个陈述。
  //   ρ = 0.6/0.5 给多项式的过冲留余量（同 acp = 0.5·a_max 的道理，见上面 rate_rho_w）。
  //   ★ 限幅律直接借用项目自己的 **Nav2 rotation_shim 三件套**（M5.2 要复用的那一套）——
  //     同一套律，避免两处各自发明：
  //       ω_des = clamp(e/Δt, ±ρ_ω·ω_max)                     // 希望在 Δt 内到位
  //       ω_des = clamp(ω_des, ω_prev ± ρ_α·α_max·Δt)          // **角加速度限制**
  //       ω_des = sign·min(|ω_des|, √(2·ρ_α·α_max·|e|))        // 防过冲（提前减速）
  //     为什么不能只用硬 clamp（试过）：斜率从 ρ·ω_max 直接掉到 0 ⇒ α 冲激 ⇒
  //     实测 α 峰值 4.34 rad/s² = 1.7 × α_max ⇒ 时间缩放又乘 k = 1.32。
  {
    knot_th[0] = req.start.has_yaw
                     ? req.start.yaw
                     : std::atan2(pts[1].y - pts[0].y, pts[1].x - pts[0].x);
    const double w_lim = rate_rho_w * w_max_;
    const double dw_step = rate_rho_a * alpha_max_ * T_step;   // 单段允许的角速度增量
    const double a_cap = rate_rho_a * alpha_max_;
    double w_prev = 0.0;
    for (int j = 1; j <= n_piece; ++j) {
      const double sv = (j == n_piece) ? L : sAtTime(j * T_step);
      knot_s[static_cast<std::size_t>(j)] = sv;
      // ★ 切线用 ±tan_base 的**基线**算（相邻两点在毫米级基线上算切线 = 噪声，
      //   这个坑我们在 MPC 的 rebuildReference 里踩过）；**与剖面的 θ(s) 同一套基线**
      double x0 = 0.0;
      double y0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      atS(std::max(0.0, sv - tan_base), x0, y0);
      atS(std::min(L, sv + tan_base), x1, y1);
      const double prev = knot_th[static_cast<std::size_t>(j - 1)];
      const double tang =
          (std::hypot(x1 - x0, y1 - y0) > 1e-6)
              ? std::atan2(y1 - y0, x1 - x0)
              : prev;
      const double e = wrapPi(tang - prev);          // 解缠后的朝向误差
      double w_des = std::clamp(e / T_step, -w_lim, w_lim);
      w_des = std::clamp(w_des, w_prev - dw_step, w_prev + dw_step);
      const double w_stop = std::sqrt(std::max(0.0, 2.0 * a_cap * std::fabs(e)));
      w_des = std::copysign(std::min(std::fabs(w_des), w_stop), w_des);
      knot_th[static_cast<std::size_t>(j)] = prev + w_des * T_step;
      w_prev = w_des;
    }
  }

  // 终点朝向：默认取目标 yaw（与 (θ,s) 的"终点朝向是状态约束"一致）；
  // 也可取末端切线（末段对正交给 shim / goal checker）
  double theta_end = knot_th[static_cast<std::size_t>(n_piece)];
  if (req.use_goal_yaw && req.goal.has_yaw) {
    const double prev = knot_th[static_cast<std::size_t>(n_piece) - 1];
    theta_end = prev + wrapPi(req.goal.yaw - prev);
  }

  // ---- 内点（MINCO 的自由变量；初值取"剖面路点"）
  // ⚠ 布局是 **(2, N−1)**（`inPs.col(i)` 才是内点 i）；别写成 `MatrixX2d`，见 solveMinco 注释
  Eigen::Matrix<double, 2, Eigen::Dynamic> inPs(2, n_piece - 1);
  for (int i = 0; i < n_piece - 1; ++i) {
    inPs(0, i) = knot_th[static_cast<std::size_t>(i + 1)];
    inPs(1, i) = knot_s[static_cast<std::size_t>(i + 1)];
  }

  // ---- head 状态：(θ, s) 的 p/v/a
  Eigen::Matrix<double, 2, 3> head;
  head.col(0) = Vec2(knot_th[0], 0.0);
  head.col(1) = Vec2(req.start_omega, req.start_v);
  head.col(2) = Vec2::Zero();

  double s_total = L;
  Trajectory<5, 2> traj;
  std::vector<RawPoint> raw;

  // ---- (a) 终点修正：XY 是 (θ,s) 的**积分派生量**，只能靠调自由变量把它拉回来。
  //   自由变量 = 手柄内点的 (θ_k, s_k) + 末端总弧长 s_total（3 个）
  //   ★ 不能拿末端朝向 θ_end 当自由变量 —— 它是"终点朝向"这个**语义约束**。
  //   响应近似线性 ⇒ 数值 Jacobian + 阻尼最小二乘（每次迭代 4 次求解，很便宜）。
  //   ★★ 为什么必须是 3 个自由度 + 最小二乘，而不是 2×2 求逆：
  //     折角用例的末端 0.8 m 是沿末端朝向的**直线**，此时 ∫cosθ·ds 与这段的
  //     s 分布无关 ⇒ ∂p_end/∂s_k ≡ 0，J 第一列**全零**、det = 0（实测）：
  //       J = [-0.000 -0.302; -0.000 -0.007]
  //     旧代码一见 |det| ≤ 1e-6 就掉进"沿末端切线的 1 维退化分支"，而那个分支
  //     只能沿末端朝向挪 s_total（本例末端朝向 = π/2 ⇒ 只能动 y），可残差
  //     (-0.027, 0.000) **全在 x 上** ⇒ 残差冻在 0.0274 m，12 次迭代一动不动。
  //     补齐 s_total 后 J 是 2×3，(JᵀJ + λI) **恒可逆**，λ→0 时退化为最小范数解
  //     ⇒ 秩亏方向自动不动、有权限的方向一步吃掉残差。
  double term_err = std::numeric_limits<double>::max();
  if (n_piece >= 2) {
    // ★ 手柄取"离终点约 terminal_handle_back_ 米的那个内点"。
    //   紧邻终点的内点**没有权限**：减速段的路点挤在一起（实测末段只有 6.6 cm）。
    int k_sel = n_piece - 2;
    for (int j = n_piece - 2; j >= 1; --j) {
      if (knot_s[static_cast<std::size_t>(j)] <= s_total - terminal_handle_back_) {
        k_sel = j - 1;   // inPs 的列 j−1 ↔ 路点 j
        break;
      }
    }
    double & th_k = inPs(0, k_sel);
    double & s_k = inPs(1, k_sel);
    const double s_lo = knot_s[static_cast<std::size_t>(k_sel)];
    const double s_hi = knot_s[static_cast<std::size_t>(std::min(k_sel + 2, n_piece))];
    // s 的可行区间：`clamp` 要求 lo ≤ hi，路点极密时 s_hi−1e-3 会低于 s_lo+1e-3
    const double s_lo_b = s_lo + 1e-3;
    const double s_hi_b = std::max(s_lo_b, s_hi - 1e-3);
    const double eps_s = 0.05;   // 扰动要够大：太小会被非线性/积分误差淹没
    const double eps_th = 0.05;
    for (int k = 0; k < terminal_iters_; ++k) {
      if (!solveMinco(head, theta_end, s_total, req.goal_v, inPs, ts, traj)) {
        err = "MINCO 求解失败（终点修正阶段）";
        return false;
      }
      integrateTrajectory(traj, req.start.x, req.start.y, scan_dt_, raw);
      if (raw.empty()) {
        err = "积分没有产出采样点";
        return false;
      }
      const double rx = req.goal.x - raw.back().x;
      const double ry = req.goal.y - raw.back().y;
      term_err = std::hypot(rx, ry);
      if (term_err <= terminal_tol_) break;
      if (elapsedMs() > max_solve_ms_) {
        err = "求解超预算（终点修正阶段）";
        return false;
      }
      if (std::getenv("PNC_TRAJ_DEBUG") != nullptr) {
        std::fprintf(stderr, "[traj-dbg] 终点牛顿 %d: 残差 %.4f (s_k=%.3f th_k=%.3f)\n", k,
                     term_err, s_k, th_k);
      }

      // 数值 Jacobian：∂p_end/∂(s_k, θ_k, s_total)
      const double th0 = th_k;
      const double s0 = s_k;
      const double L0 = s_total;
      auto probe = [&](double dth, double ds, double dL, Eigen::Vector2d & out) {
        th_k = th0 + dth;
        s_k = std::clamp(s0 + ds, s_lo_b, s_hi_b);
        s_total = L0 + dL;
        Trajectory<5, 2> t2;
        std::vector<RawPoint> r2;
        if (!solveMinco(head, theta_end, s_total, req.goal_v, inPs, ts, t2)) return false;
        integrateTrajectory(t2, req.start.x, req.start.y, scan_dt_, r2);
        if (r2.empty()) return false;
        out = Eigen::Vector2d(r2.back().x, r2.back().y);
        return true;
      };
      Eigen::Vector2d ps = Eigen::Vector2d::Zero();
      Eigen::Vector2d pt = Eigen::Vector2d::Zero();
      Eigen::Vector2d pl = Eigen::Vector2d::Zero();
      const bool ok_s = probe(0.0, eps_s, 0.0, ps);
      const bool ok_t = probe(eps_th, 0.0, 0.0, pt);
      const bool ok_l = probe(0.0, 0.0, eps_s, pl);
      th_k = th0;
      s_k = s0;
      s_total = L0;
      const Eigen::Vector2d p0 = Eigen::Vector2d(raw.back().x, raw.back().y);
      bool solved = false;
      if (ok_s && ok_t && ok_l) {
        Eigen::Matrix<double, 2, 3> J;
        J.col(0) = (ps - p0) / eps_s;
        J.col(1) = (pt - p0) / eps_th;
        J.col(2) = (pl - p0) / eps_s;
        const Eigen::Vector2d res(rx, ry);
        Eigen::Matrix3d H = J.transpose() * J;
        const double lam = 1e-6 * std::max(1.0, H.diagonal().maxCoeff());
        H.diagonal().array() += lam;
        const Eigen::Vector3d d = H.ldlt().solve(J.transpose() * res);
        // 阻尼：一步最多把末内点的 s 挪半段、θ 挪 0.5 rad（防跳出局部结构）
        const double seg = std::max(0.05, s_total - s_lo);
        th_k = th0 + std::clamp(d(1), -0.5, 0.5);
        s_k = std::clamp(s0 + std::clamp(d(0), -0.5 * seg, 0.5 * seg), s_lo_b, s_hi_b);
        s_total = std::max(0.2 * L, L0 + std::clamp(d(2), -0.5 * seg, 0.5 * seg));
        solved = true;
        if (std::getenv("PNC_TRAJ_DEBUG") != nullptr) {
          std::fprintf(stderr,
                       "[traj-dbg]   J=[%.3f %.3f %.3f; %.3f %.3f %.3f] λ=%.2e\n"
                       "[traj-dbg]   步长 d=(ds_k %.4f, dθ_k %.4f, dL %.4f)\n",
                       J(0, 0), J(0, 1), J(0, 2), J(1, 0), J(1, 1), J(1, 2), lam, d(0),
                       d(1), d(2));
        }
      } else {
        // 连探测都失败（极罕见）：退回一维（沿末端朝向挪 s_total）
        const double te = raw.back().theta;
        s_total = std::max(0.2 * L, L0 + rx * std::cos(te) + ry * std::sin(te));
        solved = true;
      }
      (void)solved;
    }
  } else {
    // 只有一段：没有内点可用，退化到 1 维（调末端弧长）
    for (int k = 0; k < terminal_iters_; ++k) {
      if (!solveMinco(head, theta_end, s_total, req.goal_v, inPs, ts, traj)) {
        err = "MINCO 求解失败（终点修正阶段）";
        return false;
      }
      integrateTrajectory(traj, req.start.x, req.start.y, scan_dt_, raw);
      if (raw.empty()) {
        err = "积分没有产出采样点";
        return false;
      }
      const double rx = req.goal.x - raw.back().x;
      const double ry = req.goal.y - raw.back().y;
      term_err = std::hypot(rx, ry);
      if (term_err <= terminal_tol_) break;
      const double te = raw.back().theta;
      s_total += rx * std::cos(te) + ry * std::sin(te);
      s_total = std::max(s_total, 0.2 * L);
    }
  }

  // ---- (b) 时间缩放：把所有 T_i 同乘 k ⇒ v/k、a/k²。几何不变，所以可以和 (a) 解耦。
  st.time_scale = 1.0;
  for (int it = 0; it < 3; ++it) {
    if (!solveMinco(head, theta_end, s_total, req.goal_v, inPs, ts, traj)) {
      err = "MINCO 求解失败（时间缩放阶段）";
      return false;
    }
    integrateTrajectory(traj, req.start.x, req.start.y, scan_dt_, raw);
    double mv = 0.0;
    double ma = 0.0;
    double mw = 0.0;
    double mal = 0.0;
    double a_at = 0.0;
    double a_at_s = 0.0;
    for (const RawPoint & r : raw) {
      mv = std::max(mv, std::fabs(r.v));
      if (std::fabs(r.a) > ma) { ma = std::fabs(r.a); a_at = r.t; a_at_s = r.s; }
      mw = std::max(mw, std::fabs(r.omega));
      mal = std::max(mal, std::fabs(r.alpha));
    }
    double k = 1.0;
    k = std::max(k, mv / v_max_);
    k = std::max(k, std::sqrt(ma / a_max_));
    k = std::max(k, mw / w_max_);
    k = std::max(k, std::sqrt(mal / alpha_max_));
    if (std::getenv("PNC_TRAJ_DEBUG") != nullptr) {
      std::fprintf(stderr, "[traj-dbg] 缩放迭代 %d: max_v=%.3f max_a=%.3f max_w=%.3f max_al=%.3f "
                           "-> k=%.3f (T=%.2f)\n",
                   it, mv, ma, mw, mal, k, traj.getTotalDuration());
      std::fprintf(stderr, "[traj-dbg]   ↑ max|a| 在 t=%.3f s、s=%.3f m（剖面设计上限 %.3f）\n",
                   a_at, a_at_s, acp);
    }
    if (!(k > 1.0 + 1e-6)) break;
    ts *= k;
    st.time_scale *= k;
    if (elapsedMs() > max_solve_ms_) {
      err = "求解超预算（max_solve_ms）";
      return false;
    }
  }

  // ---- 输出：密集积分（精度）+ 按 output_dt 采样（剖面时间分辨率）
  integrateTrajectory(traj, req.start.x, req.start.y, integrate_dt_, raw);
  if (raw.empty()) {
    err = "积分没有产出采样点";
    return false;
  }

  out.clear();
  double next_t = 0.0;
  auto push = [&out](const RawPoint & r) {
    TrajSample s;
    s.t = r.t;
    s.s = r.s;
    s.x = r.x;
    s.y = r.y;
    s.yaw = r.theta;
    s.v = r.v;
    s.omega = r.omega;
    s.a = r.a;
    s.alpha = r.alpha;
    out.push_back(s);
  };
  for (const RawPoint & r : raw) {
    if (r.t + 1e-9 >= next_t) {
      push(r);
      next_t += output_dt_;
    }
  }
  if (out.empty() || out.back().t < raw.back().t - 1e-9) push(raw.back());

  // ---- 统计（全部量出来的）
  const auto t_stats = std::chrono::steady_clock::now();
  const double total_t = traj.getTotalDuration();
  st.duration = total_t;
  st.path_length = raw.back().s;
  double mv = 0.0;
  double ma = 0.0;
  double mw = 0.0;
  double mal = 0.0;
  double mk = 0.0;
  double shift = 0.0;
  for (const RawPoint & r : raw) {
    mv = std::max(mv, std::fabs(r.v));
    ma = std::max(ma, std::fabs(r.a));
    mw = std::max(mw, std::fabs(r.omega));
    mal = std::max(mal, std::fabs(r.alpha));
    if (std::fabs(r.v) > 0.10) mk = std::max(mk, std::fabs(r.omega) / std::fabs(r.v));
    shift = std::max(shift, distanceToPolyline(r.x, r.y, req.path));
  }
  st.max_v = mv;
  st.max_a = ma;
  st.max_omega = mw;
  st.max_alpha = mal;

  // ---- **运动学终验**（与 M3 终检并列的第二道门）
  //   ★ 为什么必须有：时间缩放最多跑 3 轮，**可能没收敛**。而 M3 终检只管"碰不碰"、
  //     完全不管运动学 ⇒ 不加这一项就会**默默发出一条超限的轨迹**（MPC 跟不上、
  //     履带打滑）。这是个真缺口：加之前没有任何地方回头看这件事。
  //   ★ 容差 1.02 与单测 `expectKinematicsWithinLimits` 同源 —— 不是新口径。
  {
    constexpr double kKinTol = 1.02;
    st.kin_ok = (mv <= v_max_ * kKinTol) && (ma <= a_max_ * kKinTol) &&
                (mw <= w_max_ * kKinTol) && (mal <= alpha_max_ * kKinTol);
    if (!st.kin_ok) {
      char kb[288];
      std::snprintf(kb, sizeof(kb),
                    " ⚠ 运动学终验不过：max|v| %.3f/%.2f、max|a| %.3f/%.2f、"
                    "max|ω| %.3f/%.2f、max|α| %.3f/%.2f（时间缩放 ×%.2f 跑满 3 轮仍未"
                    "收敛）",
                    mv, v_max_, ma, a_max_, mw, w_max_, mal, alpha_max_, st.time_scale);
      st.note += kb;
    }
  }
  st.max_curvature = mk;
  st.path_shift_max = shift;
  st.terminal_error = term_err;
  st.terminal_error_yaw = std::fabs(wrapPi(req.goal.has_yaw ? req.goal.yaw - raw.back().theta
                                                            : 0.0));

  // ---- M3 轨迹终检：逐位姿、含朝向的全机轮廓检查（**这一层才是安全门**）
  //   ★ 为什么不沿用"在输出采样上量最小净距"：那只看**采样到的位姿**，
  //     · 采样间距按时间（0.05 s）⇒ 慢速段过密、快速段过疏；
  //     · **原地转**几乎不产生弧长 ⇒ 旋转扫掠的区域整段漏掉（新剖面在起点
  //       朝向与路径差很大时就是原地转，实测头 1 s 位移 0.001 m）；
  //     · 而且"最小净距"本身不是判据 —— 判据是"有没有低于硬要求"。
  //   ⇒ 交给 `checkTrajectory`：按 `ds` **与** `dyaw` 双重细分，逐点查，
  //     同时给出"统计口径的最小净距"（排除首端）与硬门结果。
  //   ★ 顺带把这里原来的逐点 `signedClearanceAt` 循环删了（同一件事，别做两遍）。
  //   ⚠ 没有判定器时**不做检查**（`check_ok` 保持 true 表示"门没拦"，但 note 必须
  //     说清"未做终检"——不要让调用方以为查过了；与 `clearance_valid=false` 同约定）。
  if (ck_ != nullptr) {
    const TrajCheckResult cr = checkTrajectory(out, *ck_, check_prm_);
    st.check_ok = cr.ok;
    st.check_points = cr.points;
    st.check_stats_points = cr.stats_points;
    st.check_violations = cr.violations;
    st.check_worst_clearance = cr.worst_clearance;
    st.check_worst_s = cr.worst_clearance_s;
    st.check_note = cr.note;
    st.clearance_valid = cr.min_valid;
    if (cr.min_valid) {
      st.min_clearance = cr.min_clearance;
      st.min_clearance_s = cr.min_clearance_s;
    }
  } else {
    st.check_ok = true;
    st.check_note = "未做终检（没有轮廓判定器）";
    st.clearance_valid = false;
  }

  st.note += " MINCO[" + std::to_string(n_piece) + "段/" +
             std::to_string(st.duration).substr(0, 4) + "s, 时间×" +
             std::to_string(st.time_scale).substr(0, 4) + ", 折线max|κ|" +
             std::to_string(kappa_max).substr(0, 4) + "@s=" +
             std::to_string(kappa_at_s).substr(0, 4) + ", 终点残差 " +
             std::to_string(st.terminal_error).substr(0, 5) + "m]";
  // ★ 两条**可行动**的诊断（不是内部细节：都能指向一个具体的上游问题）
  {
    // ① 起点朝向与路径首段的差：A* 从当前格朝任意方向迈第一步，不看车的朝向
    //    ⇒ 这个差必须由轨迹里"边转边走"吃掉（时间下界 = |Δθ|/ω_max）。
    //    差得大往往说明 A* 起点朝向过期或路径首段绕路。
    const double dy0 =
        std::fabs(wrapPi(theta_d[1] - theta_d[0])) * 180.0 / M_PI;
    if (dy0 > 20.0) {
      st.note += "  ⚠ 起点朝向与路径首段差 " +
                 std::to_string(static_cast<int>(dy0)) +
                 "°（轨迹须在起点段用 |Δθ|/ω_max 的时间把它转过来）";
    }
    // ② 剖面被迫低于 v_min：说明**路径本身**太尖，不是调参能解决的
    //    （要么 Stage A 平滑不到位，要么 A* 的路径有折返/尖刺）
    if (v_kappa_min < v_min_ - 1e-6) {
      char vb[160];
      std::snprintf(vb, sizeof(vb),
                    "  ⚠ 因转向速率限制被迫最低速 %.3f m/s @s=%.2f m（< v_min %.2f，"
                    "路径过尖：κ=%.2f ⇒ r=%.2f m）",
                    v_kappa_min, v_kappa_min_s, v_min_, kappa_max,
                    (kappa_max > 1e-9) ? 1.0 / kappa_max : 0.0);
      st.note += vb;
    }
  }
  if (!st.clearance_valid) st.note += " (无轮廓判定器: 净距未测)";
  st.stats_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t_stats)
                    .count();

  // 诊断：PNC_TRAJ_DEBUG=1 时打印（默认静默；定位"轨迹跑到图外"这类问题时必开）
  if (std::getenv("PNC_TRAJ_DEBUG") != nullptr) {
    std::fprintf(stderr,
                 "[traj-dbg] n=%d n_piece=%d s_back=%.3f theta0=%.4f th_end=%.4f "
                 "s_total=%.3f term_err=%.4f k=%.2f T=%.2f\n",
                 n, n_piece, L, knot_th[0], theta_end, s_total, st.terminal_error,
                 st.time_scale, st.duration);
    std::fprintf(stderr, "[traj-dbg] ts[0..3]=%.3f %.3f %.3f %.3f | ts[last]=%.3f\n",
                 ts(0), ts(std::min(1, n_piece - 1)), ts(std::min(2, n_piece - 1)),
                 ts(std::min(3, n_piece - 1)), ts(n_piece - 1));
    std::fprintf(stderr, "[traj-dbg] inPs.s 前5: ");
    for (int i = 0; i < std::min(5, static_cast<int>(inPs.cols())); ++i) {
      std::fprintf(stderr, "%.3f ", inPs(1, i));
    }
    if (inPs.cols() > 0) {
      std::fprintf(stderr, "| 末3: %.3f %.3f %.3f\n", inPs(1, inPs.cols() - 3),
                   inPs(1, inPs.cols() - 2), inPs(1, inPs.cols() - 1));
    }
    const std::size_t stride = std::max<std::size_t>(1, out.size() / 12);
    for (std::size_t i = 0; i < out.size(); i += stride) {
      std::fprintf(stderr, "[traj-dbg]  t=%6.2f s=%7.3f x=%8.3f y=%6.3f yaw=%6.3f v=%6.3f\n",
                   out[i].t, out[i].s, out[i].x, out[i].y, out[i].yaw, out[i].v);
    }
    // ★★ 取证核心：逐段列出「剖面意图」与「多项式实得」。
    //   剖面（v_prof/t_prof）是按 s 索引的，路点 t 是按时间等分的 ⇒ 这里用
    //   profileAtS 反查剖面在该 s 处的时间与速度，再与 MINCO 在同时刻的速度比。
    {
      auto profileAtS = [&](double sv) {
        std::size_t seg = 1;
        while (seg < static_cast<std::size_t>(n) - 1 && s[seg] < sv) ++seg;
        const double s0 = s[seg - 1];
        const double s1 = s[seg];
        const double r = (s1 > s0) ? std::clamp((sv - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
        const double vt = v_prof[seg - 1] + (v_prof[seg] - v_prof[seg - 1]) * r;
        return std::pair<double, double>(timeAtS(sv), vt);
      };
      std::fprintf(stderr,
                   "[traj-dbg] 剖面意图：t_total=%.3f s（MINCO 实得 %.3f s，比 %.2f）\n",
                   t_total, st.duration, st.duration / std::max(1e-6, t_total));
      double a_peak = 0.0;
      double a_peak_t = 0.0;
      for (const RawPoint & r : raw) {
        if (std::fabs(r.a) > a_peak) { a_peak = std::fabs(r.a); a_peak_t = r.t; }
      }
      std::fprintf(stderr, "[traj-dbg] max|a| = %.3f m/s² @ t=%.3f（剖面设计上限 %.3f）\n",
                   a_peak, a_peak_t, acp);
      const int st_step = std::max(1, n_piece / 10);
      for (int j = 1; j <= n_piece; j += st_step) {
        const double sv = (j == n_piece) ? L : sAtTime(j * T_step);
        const auto pv = profileAtS(sv);
        double tc = 0.0;
        for (int i = 0; i < j; ++i) tc += ts(i);
        const Eigen::VectorXd gv = traj.getVel(std::min(tc, st.duration));
        std::fprintf(stderr,
                     "[traj-dbg] 段%2d s=%6.3f | 剖面 t=%5.2f v=%5.3f | 实得 t=%5.2f v=%5.3f"
                     " (差 %+6.3f)\n",
                     j, sv, pv.first, pv.second, tc, gv(1), gv(1) - pv.second);
      }
    }
    // ★★ 取证：**终点减速段**逐路点展开（a/α 通道的瓶颈一直在这里）
    //   实测两次现场都是 `max|a|` 出现在 **终点**（s≈L），且数值满足
    //   `max|a| ≈ 次末路点速度 / T_step` ⇒ 怀疑"末段必须在一个 T_step 内把速度降到 0"。
    //   这里把**剖面对该路点的意图**与**多项式实得**并排打出来，一次定位：
    //     · 若"剖面 v"本身就 > a_max·T_step ⇒ 是**剖面/时间轴**的错（末段分的时间不够）
    //     · 若"剖面 v"很小而"实得 v/a"很大 ⇒ 是**多项式**在末段过冲（尾条件 a(T)=0 与
    //       路点冲突）
    {
      const int n_show = std::min(6, n_piece);
      std::fprintf(stderr, "[traj-dbg] 终点段逐路点（剖面意图 vs 实得；T_step=%.3f，"
                           "a_max·T_step=%.3f）：\n",
                   T_step, a_max_ * T_step);
      double tc = 0.0;
      for (int j = 0; j <= n_piece; ++j) {
        if (j > 0) tc += ts(j - 1);
        if (j < n_piece - n_show) continue;
        const double sv = knot_s[static_cast<std::size_t>(j)];
        double v_prof_j = 0.0;
        if (j > 0) {
          v_prof_j = (sv - knot_s[static_cast<std::size_t>(j - 1)]) / std::max(1e-6, T_step);
        }
        const Eigen::VectorXd gv = traj.getVel(std::min(tc, st.duration));
        const Eigen::VectorXd ga = traj.getAcc(std::min(tc, st.duration));
        std::fprintf(stderr,
                     "[traj-dbg]   路点 %3d t=%6.2f s=%7.3f | 剖面 v=%.3f | 实得 v=%.3f "
                     "a=%+.3f\n",
                     j, tc, sv, v_prof_j, gv(1), ga(1));
      }
    }

    // ★ 关键取证：多项式在**真实路点时刻**的取值是否等于路点
    //   ⚠ 用 `knot_*`（长度 n_piece+1）**不要**用 `inPs`（列数只有 n_piece−1）：
    //     旧版写成 `inPs(1, i)` 而 i 取到 n_piece−1 ⇒ **越界读**（Release 下不报错，
    //     读到的 0.000 让最后一行看起来像"期望 0.000"的假象）。这个类别的错我们
    //     已经栽过一次（MatrixX2d 那次），一律不要信"看起来像"的数字。
    {
      const int step = std::max(1, n_piece / 8);
      double tc = 0.0;
      for (int i = 0; i <= n_piece; ++i) {
        if (i > 0) tc += ts(i - 1);      // tc = 第 i 个路点的全局时刻
        if (i % step != 0 && i != n_piece) continue;
        const double want_s = knot_s[static_cast<std::size_t>(i)];
        const double want_th = knot_th[static_cast<std::size_t>(i)];
        const Eigen::VectorXd gotp = traj.getPos(std::min(tc, st.duration));
        std::fprintf(stderr,
                     "[traj-dbg] 路点 i=%-3d t=%6.2f | s 期望 %6.3f 实得 %6.3f （差 %8.4f）| "
                     "θ 期望 %6.3f 实得 %6.3f\n",
                     i, tc, want_s, gotp(1), gotp(1) - want_s, want_th, gotp(0));
      }
    }
  }
  return true;
}

}  // namespace pnc_2d
