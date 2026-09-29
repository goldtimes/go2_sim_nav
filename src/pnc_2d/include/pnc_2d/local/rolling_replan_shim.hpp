// RollingReplanShimPlanner：**滚动式局部重规划**装饰器。
//
// 解决什么问题（2026-09-29 实测基线）：原链路里"全局路径穿过新出现的障碍"只有一条
// 出路 —— MPC 判 `primal infeasible` → BLOCKED → 交回状态机 → 重规划。可**全局图
// 是离线 PCD，不含新障碍** ⇒ 重规划拿回同一条路 ⇒ 恢复额度烧光、任务判死。
// 也就是：**局部层没有"绕过去"的能力**（实测侧偏只有 0.11~0.26 m）。
//
// 所以本类做的事：**在执行中把参考路径上"被挡住的那一小段"就地替换掉**，用局部图
// （感知 + 全局融合）自己搜一条绕行的短路径接回去，路径**终点不变**（仍是任务目标）。
// 这就是 EGO-Planner 那套"滚动局部重规划"的做法，只是落在 2D 栅格 + 差速车上。
//
// 与 EGO 的对应关系（都是选择，写清理由）：
//   · **只在冲突段内重规划**（`[s_a, s_b]`，默认冲突点前 0.4 m / 后 2.0 m）——
//     不做全路径重优化：局部层的输入（4 m 感知窗）只支持局部决策，动全路径等于
//     在没数据的地方乱改。终点因此天然不变。
//   · **只向前搜**：搜索限制在"沿原路径切线方向不后退"的半空间里 ⇒ 天然**不倒车**
//     （用户要求）；也不产生 U 形掉头。
//   · **绕行幅度有界**（`max_offset_m`）：偏出去太多就放弃、如实交回 BLOCKED ——
//     宁可停，也不做"看起来绕过去了其实进了死胡同"的动作。
//   · **未知格可通行**（用户口径）：未知只加**代价惩罚**（偏好已观测的空地），
//     不当障碍。静态障碍的安全由上层全局图融合保证（局部图里那些格本来就是占据）。
//
// 走廊模式（`setCorridor()` 非空）下**不绕行**（用户口径，默认
// `allow_in_corridor=false`）：走廊是"只能贴线走"的语义，在那里横挪是错的。
// 那时本类完全不介入，行为与只有主控制器时一致（BLOCKED 照样如实上报）。
//
// 工作方式（装饰器 ⇒ local 节点零改动；与 `heading_shim` 同构）：
//   ① 每周期先判要不要修：投影出当前弧长 s_now → 在 `[s_now, s_now+lookahead]` 上
//      逐点查"车体轮廓会不会撞" → 找到第一个阻塞点；命中才做局部 A*；
//   ② 修好了：`new_plan = 原路径[0..a] + 修补段 + 原路径[b..]` 交给主控制器；
//   ③ 交给主控制器算指令（本类不产生速度，`producesCmdVel()` 转发）；
//   ④ 修不出来 / 太频繁 / 反复失败：**什么都不做**，让主控制器按原逻辑报 BLOCKED。
//
// 防抖与放弃（都必要，否则会变成"绕行抖动"）：
//   · `min_interval_s`：两次修补的最小间隔（修补是 A* + 平滑，别每拍都做）；
//   · `min_progress_m`：距上次修补点必须又前进了这么多 —— 否则"挡住了→原地修补→
//     路径没变→再修补"会空转；
//   · `max_consecutive_fail` + `give_up_s`：连续搜不出来就静默一段时间，
//     让主控制器如实报 BLOCKED（恢复行为才有机会接管）。

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/local_planner.hpp"

namespace pnc_2d {

class RollingReplanShimPlanner : public LocalPlanner {
public:
  struct Params {
    /// 被装饰的主控制器（`createLocalPlanner()` 认得的名字）
    std::string primary{"heading_shim"};

    /// 总开关（false = 完全透传主控制器，行为回到"只跟踪不绕行"）
    bool enable{true};

    // ---------------- 什么时候算"被挡住" ----------------
    /// 沿参考路径向前看多远 [m]
    double lookahead_m{1.60};
    /// 前瞻点的**车心净距**小于它就当作被挡 [m]。
    /// ★ 默认 0.45 = `local_mpc.obstacle_safe_distance`（软代价开始生效的量级）。
    ///   为什么不能用硬下界（0.25）当触发：实测（2026-09-29 S1）那时车**已经贴上**
    ///   了（车心→盒面 0.15 m、轮廓穿透 10 cm）—— 绕行要提前在"还有地方挪"的时候
    ///   开始，而不是等硬约束报警。启动时会与 `local_mpc.obstacle_safe_distance`
    ///   对一下并在差得远时 WARN。
    double trigger_clearance_m{0.45};
    /// **硬下界** [m]：低于它 MPC 必然 `primal infeasible`（`obstacle_hard_distance`）。
    /// 用途：① 日志里区分"该提前绕"与"已经晚了"；② 只有当上方触发阈被配得
    /// 比它还紧时，它才作为兼底触发。
    double blocked_clearance_m{0.25};
    /// 修补段的**额外安全余量** [m]：A* 与自检用 `footprint.safe_margin + 本值`
    /// 判碰。为什么必须留：修补段是给 MPC **跟踪**的，而跟踪总有横向误差
    /// （实测 2~4 cm）——按"恰好不碰"验出来的路径，跟偏一点就擦上去。
    double path_margin_m{0.06};
    /// 检测步长 [m]（越小越不容易漏细柱子；0.10 = 一个格）
    double check_step_m{0.10};

    // ---------------- 修补窗口 ----------------
    /// 冲突点**之前**多少开始替换 [m]（留出接回原路径的余量）
    double back_m{0.40};
    /// 冲突点**之后**多少当作"该绕过的范围" [m]
    double roll_m{2.00};
    /// 向外扩的最大范围 [m]（`roll_m` 那段还在障碍里就继续往外找可通行点）
    double max_roll_m{4.00};
    /// 绕行幅度上限 [m]：修补段上任何点到**原路径切线轴**的侧向距离超过它就放弃
    double max_offset_m{1.20};

    // ---------------- 节流/放弃 ----------------
    double min_interval_s{0.50};
    double min_progress_m{0.20};
    /// 连续搜不出来几次就静默一段时间（0 = 不静默）
    int max_consecutive_fail{2};
    double give_up_s{3.00};

    // ---------------- 搜索代价 ----------------
    /// 靠近障碍的代价权重（越大越愿意离障碍远，代价是绕得更远）
    double obs_cost_weight{0.60};
    /// 未知格的额外代价系数（未知可通行，但偏好已观测的空地）
    double unknown_cost_scale{1.60};
    /// 侧向偏离惩罚权重（把绕行拉成"贴着原路径拐一下"而不是大弧线）
    double lateral_cost_weight{0.80};
    /// 平滑（拉直）开关：把 A* 的锯齿做 line-of-sight 捷径
    bool smoothing{true};

    // ---------------- 与全局层同源的几何 ----------------
    // 从 `footprint.*` / `*_threshold` 读（与全局规划器、局部节点同一套键），
    // 否则"全局说能过、局部说撞"这类跨层不一致又会回来。
    bool unknown_as_occupied{false}; ///< 未知格是否算障碍（用户口径：否）
    int hard_threshold{50};
  };

  RollingReplanShimPlanner();
  std::string type() const override { return "rolling_replan"; }
  bool configure(const ParamReader &params) override;

  // ---------------- 输入：自己存一份 + 转发主控制器 ----------------
  void setGlobalPlan(const std::vector<Pose2D> &path) override;
  void setCorridor(const RouteCorridor *corridor) override;
  void setSpeedLimit(double v_limit) override;
  void setZoneSpeedLimit(double v_limit) override;
  void setCostMap(std::shared_ptr<const CostMap2D> local_map) override;
  void setDistanceField(const LocalDistanceField *field) override;
  void setReferenceProfile(const ReferenceProfile *profile) override;
  void setDynamicObstacles(const std::vector<DynamicObstacle> &obs) override;
  void setCurrentVelocity(double v, double w) override;

  LocalPlanResult computeCommand(const Pose2D &pose, double dt) override;
  void reset() override;

  // ---------------- 转发给主控制器 ----------------
  bool producesCmdVel() const override;
  double stopCoast() const override;
  double corridorTolerance() const override;
  double maxSpeed() const override;
  double brakeAcc() const override;
  double maxOmega() const override;
  /// ★ 转发**主控制器**的（朝向由装饰器链的最内层负责，本类不管朝向）
  double goalYawTolerance() const override;
  bool degraded() const override;
  std::string diagString() const override;

  // ---------------- 诊断 / 单测 ----------------
  struct RepairStats {
    int repairs{0};            ///< 累计成功修补次数
    int attempts{0};           ///< 累计尝试次数（含失败）
    int consecutive_fail{0};   ///< 当前连续失败次数
    double last_offset_m{0.0}; ///< 上次修补的最大侧向偏离 [m]
    double last_len_m{0.0};    ///< 上次修补段的长度 [m]
    int last_nodes{0};         ///< 上次 A* 展开的节点数
    double last_ms{0.0};       ///< 上次修补耗时 [ms]
    double last_s_now{0.0};    ///< 上次修补时的弧长 [m]
    double last_s_block{0.0};  ///< 上次检测到的冲突点弧长 [m]
    std::string last_msg;      ///< 上次失败原因（空 = 没失败过）
  };
  const RepairStats &repairStats() const { return stats_; }
  /// 本周期是否修补过（单测/日志用）
  bool repairedThisCycle() const { return repaired_this_cycle_; }
  /// 当前生效的参考路径（= 原路径或修补后的）——单测要断言"终点没变"
  const std::vector<Pose2D> &activePlan() const { return plan_; }
  /// 原始全局路径（未修补）
  const std::vector<Pose2D> &originalPlan() const { return original_; }

private:
  /// 投影：机器人当前在参考路径上的弧长 [m]（并给出最近点索引）
  double projectS(const Pose2D &pose, std::size_t &idx, double &lat) const;
  /// 按弧长取参考路径上的点（线性插值；yaw 取该段朝向）
  bool sampleAt(double s, Pose2D &out) const;
  /// 该点是否"被挡"（净距 < thr，或轮廓碰）
  bool blockedAt(const Pose2D &pt, double thr, double &clearance) const;
  /// A*/自检用的有效轮廓余量（= 配置的 safe_margin + path_margin_m）
  double pathMargin() const { return fp_.safe_margin + p_.path_margin_m; }
  /// 修补：返回 true = 已换掉 plan_ 并同步给主控制器
  bool tryRepair(const Pose2D &pose);
  /// 局部 A*：从 from 到 to（**只向前**），成功输出 waypoints（含首尾精确点）
  bool localSearchForward(const Pose2D &from, const Pose2D &to,
                          std::vector<Pose2D> &out, std::string &why);
  /// 折线总长 / 侧向偏离
  static double polylineLen(const std::vector<Pose2D> &p);
  static double maxLateral(const std::vector<Pose2D> &p, const Pose2D &origin,
                           double tangent_yaw);

  std::unique_ptr<LocalPlanner> primary_;
  Params p_;

  /// 原始全局路径（终点即任务目标；修补时只动中间一段）
  std::vector<Pose2D> original_;
  /// 当前生效的参考路径（可能已被修补）
  std::vector<Pose2D> plan_;
  /// 每段的弧长累积（与 plan_ 同长；用于按弧长采样）
  std::vector<double> arclen_;

  FootprintCollisionChecker coll_;
  FootprintParams fp_;
  double hard_distance_hint_{0.25}; ///< 仅用于启动一致性校验（mpc 的硬下界）

  RepairStats stats_;
  bool repaired_this_cycle_{false};
  bool warned_corridor_{false};
  /// 上一次修补（单调时钟）：节流用
  std::chrono::steady_clock::time_point last_repair_{};
  bool have_last_repair_{false};
  double s_at_last_repair_{0.0};
  std::chrono::steady_clock::time_point give_up_until_{};
  bool giving_up_{false};

  void rebuildArclen();
  void applyPlanToPrimary();
};

} // namespace pnc_2d
