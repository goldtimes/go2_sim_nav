// MINCO 后端实现（`(θ, s)` 参数化）—— 与 DDR-opt 的 `back_end` 同构：
// 状态 `p = (θ, s)`、`v = (ω, ṡ)`、`a = (α, s̈)`，XY 是**积分派生量**
//   `x = x₀ + ∫cosθ·ds`、`y = y₀ + ∫sinθ·ds`
// ⇒ 差速（ICR 在车体中心）就是这个参数化的退化情形（`traj_anal.hpp` 的积分式里
//    没有 ICR 项），所以不需要 `icrekf`，也不需要曲率限速那一套。
//
// ★ 两段式（为什么这么分，见 doc/minco_trajectory_plan.md §4 M2）：
//   A「几何避障」`smoothPath()`：在折线上做弹性带式调整（二阶差分平滑 + SDF 净距罚
//     + 锚定 + 图外屏障）。自由变量就是折线上的点 ⇒ **SDF 梯度直接作用在变量上**，
//     梯度是精确的，不需要"MINCO 内点对轨迹点"的链式求导。
//   B「MINCO 时间参数化」：折线 → `(θ, s)` 路点（θ 取切线并解缠、s 取弧长）→
//     MINCO 带状求解得到 C² 轨迹；时间按运动学**解析缩放**（匀缩放 k 使
//     v/k、a/k² 满足上限）；终点 XY 做 1 维修正（因为它是积分派生量）。
//   ⇒ 避障在 A、平滑与运动学可行在 B；安全性由**轮廓终检**（M3）兜底，
//     不指望罚项本身保证安全（距离场在中轴/脊线上梯度本来就是病态的，
//     这一点 M1 已实测取证并写进 `clearance_field.hpp`）。

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/trajectory_optimizer.hpp"

namespace pnc_2d {

class ClearanceField;
class CostMap2D;
class FootprintCollisionChecker;

class MincoOptimizer : public TrajectoryOptimizer {
public:
  std::string type() const override { return "minco"; }
  bool configure(const ParamReader & params) override;
  void setMap(std::shared_ptr<const CostMap2D> map) override;
  void setClearanceField(const ClearanceField * cf) override;
  void setCollisionChecker(const FootprintCollisionChecker * ck) override;
  void reset() override;
  TrajOptResult optimize(const TrajOptRequest & req) override;

  /// 诊断：上一次**平滑后**的折线（单测/离线工作台看"避障这一步到底改了多少"）
  const std::vector<Pose2D> & lastSmoothed() const { return smoothed_; }
  /// 诊断：平滑用的净距目标 [m]（= 车体外接圆半径 + `traj.smooth_margin`）
  double smoothClearanceTarget() const { return smooth_d_target_; }

private:
  /// 等距重采样（含首末点，末点强制等于输入末点）
  bool resample(const std::vector<Pose2D> & in, std::vector<Pose2D> & out) const;
  /// Stage A：返回平滑后的代价（诊断用）；`pts` 原地修改
  double smoothPath(std::vector<Pose2D> & pts, TrajOptStats & st);
  /// Stage B：`(θ,s)` 路点 → MINCO → 采样输出；失败返回 false
  bool buildTrajectory(const TrajOptRequest & req,
                       const std::vector<Pose2D> & pts,
                       std::vector<TrajSample> & out, TrajOptStats & st,
                       std::string & err);

  // ------------------------------------------------------------ traj.* 参数
  // 采样与平滑
  double resample_ds_{0.15};       ///< 折线重采样间距 [m]（也是 MINCO 的路点间距）
  double smooth_margin_{0.15};     ///< 平滑的净距目标 = 外接圆半径 + 这个 [m]
  /// **端点自适应**（机制 ①，DDR-opt：`safeDis = min(ratio × 端点实测净距, d_base)`）：
  /// 沿点索引在 `smooth_ramp_len_` 内从 `d_cap` 线性升到 `d_base`。
  /// ★★ **默认关闭（0 = 关闭**，同 `max_length_m_` 的约定）。为什么：
  ///   实测（`TightEndpointRelaxesSmoothTargetInsteadOfFightingAnchor`）贴着长墙走：
  ///     · 开（0.85）：端点下限 0.365 → **实际达成净距 0.439**，缺口 0.023
  ///     · 关（默认）  ：端点下限 0.622 → **实际达成净距 0.554**，缺口 0.067
  ///   即**降低目标 ⇒ 罚项提前归零 ⇒ 实际达成净距变低**（结构性单调关系）。
  ///   而终检查的是**实际净距** ⇒ 开它只会让拦停**更频繁**。
  ///   缺口变大只是「报告口径」的事（逐点目标更严苛），不是安全退步。
  ///   ⇒ 这个机制在原版里是对付"起点在死角落里"的，搬过来与我们的判据方向相反。
  double smooth_safe_ratio_{0.0};
  double smooth_ramp_len_{1.0};    ///< [m] 端点自适应目标的过渡长度（ratio > 0 时才用）
  double smooth_w_smooth_{1.0};    ///< 平滑项权重
  double smooth_w_sdf_{2.0};       ///< 净距罚权重
  double smooth_w_anchor_{0.20};   ///< 锚定权重（别离原计划太远）
  int    smooth_iters_{80};
  double smooth_min_step_{1.0e-6};
  // 运动学（算法的**硬上限**；与底盘同源的那几个值由调用方保证一致）
  double v_max_{0.90};
  double a_max_{0.50};
  double w_max_{0.65};
  double alpha_max_{2.50};
  double v_ref_{0.60};             ///< 时间初值用的参考速度 [m/s]
  /// 弯道速度下限 [m/s]：曲率上限 v=ω_max/κ 在直角处会 → 0，剖面会发散
  /// ⇒ 必须给个地板（直角就按这个速度过，转向交给它自己）
  double v_min_{0.12};
  /// 终点修正的自由变量选"离终点这么远的内点"[m]（+ 末端弧长）
  double terminal_handle_back_{0.80};
  double time_step_{0.40};         ///< 路点取法：**时间等分**步长 [s]（段数 ≈ 轨迹时长/它，限 3~60）
  double min_piece_time_{0.05};
  /// 段数上限。**默认 60**（曾经以为"调大更好"，实测**相反**）：
  /// 调到 120 后单测 k 从 1.000 变 1.128（`max|a|` 0.5 → 0.637）——
  /// 因为 a 通道的瓶颈根本不在分辨率，而在**末段**：实测 `max|a|` 总是出现在
  /// **终点**（用户现场 s=9.726/9.745；单测 s=17.011/17.0），且
  /// `max|a| = 次末路点速度 / T_step` ⇒ 未段必须在**一个 T_step 内**把速度降到 0，
  /// 那需要 `0.27/0.43 = 0.63 = 2.5·acp`。⇒ 要治的是"终点减速段的时间分配"。
  int max_pieces_{60};
  double time_margin_{1.15};       ///< 时间初值的余量（宁慢勿超）
  // 采样与终点
  double output_dt_{0.05};         ///< 输出采样步长 [s]（= 剖面的时间分辨率）
  double integrate_dt_{0.01};      ///< 输出用的积分步长 [s]
  double scan_dt_{0.05};           ///< 终点修正/时间缩放用的粗积分步长 [s]
  double terminal_tol_{0.01};      ///< 终点 XY 容差 [m]
  int    terminal_iters_{12};
  double max_solve_ms_{50.0};
  /// 优化长度上限 [m]：只优化前这么多米（0 = 不截断）。
  /// 为什么要有：MPC 前瞻只有 1~2 m，没必要对整条（可能 100 m 的）路做全量优化；
  /// ★ 截断时终点是**途经点** ⇒ `goal_v` 改为巡航速度（**不减速**）、终点朝向取切线。
  double max_length_m_{40.0};
  double clearance_exclude_s_{0.20};  ///< 净距统计排除首端这么长的弧长 [m]
  // ---- M3 轨迹终检（口径见 core/trajectory_optimizer.hpp 的 checkTrajectory）
  double check_ds_{0.02};             ///< [m] 终检弧长采样间距
  double check_dyaw_{0.10};           ///< [rad] 终检朝向采样间距（原地转靠它兜底）
  /// 终检**统计**（最小净距）的网格 —— 比硬门粗得多。理由见 TrajCheckParams：
  /// `signedClearanceAt`（14 步二分）比布尔判定贵一个数量级，而统计只要一个"数"。
  double check_stats_ds_{0.10};       ///< [m]
  double check_stats_dyaw_{0.20};     ///< [rad]
  /// [m] 硬要求：真实轮廓净距不得低于它。0 = "不碰"。
  /// ★ 为什么不默认取 `footprint.safe_margin`：轮廓判定把位姿**吸附到格心**
  ///   （误差 ±半格 = ±5 cm，见 footprint_collision.hpp），半格量级的余量当硬门
  ///   会随机失败 —— 那是判据噪声，不是性能问题。余量只作为 **warn** 报出来。
  double check_hard_margin_{0.0};
  double check_warn_margin_{0.05};    ///< [m] 低于它只记 warn
  TrajCheckParams check_prm_;         ///< 实际传给 checkTrajectory 的那一份

  std::shared_ptr<const CostMap2D> map_;
  const ClearanceField * cf_{nullptr};
  const FootprintCollisionChecker * ck_{nullptr};

  // 诊断
  std::vector<Pose2D> smoothed_;
  double smooth_d_target_{0.0};
};

}  // namespace pnc_2d
