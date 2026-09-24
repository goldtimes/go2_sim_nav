// MPC 局部规划器（差速驱动，(v, ω) 接口）。
//
// 结构参考 **SLAM-PNC/PNC 的 MpcController**（阿克曼 + (a, δ) + OSQP 稀疏
// QP），
// 做两处必要改动，其余刻意保持一致（对齐带来的好处是已知能跑通、坑已被踩过）：
//
//  ① **底盘模型**：SLAM-PNC 是 `s = [x, y, θ, v]`、`u = [a, δ]`（前轮转角，经
//     ω = v/L·tan δ 换算）。Go2 是差速 (v, ω) 接口，**没有前轮转角**，所以取
//     `u = [a, ω]`：ω 在这里扮演 δ 的角色（直接进 θ̇），模型更简单：
//
//         ẋ = v·cosθ,   ẏ = v·sinθ,   θ̇ = ω,   v̇ = a
//
//     A = I + dt·∂f/∂s（绕参考点线性化），B = dt·∂f/∂u。v̇ = a 让 v 也成为状态，
//     于是"加速过程"本身进了预测（这是 4 状态模型相对 3 状态几何跟踪的价值）。
//
//  ② **两处新增**（SLAM-PNC 的 MPC 只做轨迹跟踪，假设上游轨迹已避障）：
//     · **走廊硬约束**（route profile）：`|e_y(k)| ≤ half_width`，线性约束；
//     · **障碍**：ESDF **软代价**（二次型 rank-1 项）+ 线性化**硬下界** +
//     解后校验。 两者都写进同一个 QP，见 buildQp() 的注释。
//
// == 为什么用"误差 + 稀疏 QP"而不是"condensing"==
// 决策变量取 **z = [e_0..e_N, Δu_0..Δu_{N-1}]**（e 是**世界系绝对误差**
// `e = s - s_ref`，Δu = u - u_ref），动力学作为**等式约束**：
//
//     e_0 = e_init;   e_{k+1} = A_k·e_k + B_k·Δu_k
//
// 于是代价 Σ e'Qe + Σ Δu'RΔu + Σ (Δu 变化率) 全是二次型，**gradient = 0**。
// 这一点很关键：参考轨迹的前馈作用**隐含在"e 是相对参考的绝对误差"里**，
// 不需要任何显式前馈项 —— 也就不会出现"把绝对 u 当偏差代入、丢掉前馈导致机器人
// 原地不动"那一类经典 bug（那种 bug 正是 condensing 写法容易犯的）。
//
// 代价：状态维度进 QP（变量数 4(N+1) + 2N），但矩阵是**稀疏**的，QSQP
// 很吃这一套；
// 而且稀疏结构**每周期完全不变**（只有数值变），所以可以真正做热启动
// （SLAM-PNC 每周期 new 一个 OsqpEigen::Solver，其实没有热启动 ——
// 这里改进了）。
//
// == 参考轨迹从哪来 ==
// SLAM-PNC 吃上游 trajopt 输出的**时间参数化**轨迹（每点带 v/a/w）。
// 我们拿到的是一条**纯几何折线**（A*/路网），所以必须自己造时间参数化：
//   弧长参数化 → 曲率限速 + 终点制动剖面 → 按弧长步进得到参考窗口。
// 见 buildReferenceWindow()。

#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/local_planner.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

class ReferenceProfile; ///< M4.3：上游速度剖面（此处只用指针）

/// 参考轨迹上的一点：位置 + 朝向 + 参考速度/加速度/角速度
/// （对应 SLAM-PNC 的 MpcReferencePoint，去掉了阿克曼的 steering_angle）
struct MpcReferencePoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0}; ///< 参考前向速度 [m/s]
  double a{0.0}; ///< 参考纵向加速度 [m/s²]（= dv/dt）
  double w{0.0}; ///< 参考角速度 [rad/s]（= κ·v）
  double s{0.0}; ///< 该点对应的弧长 [m]（诊断/进度）
};

/// MPC 参数（前缀 `local_mpc.`；权重比例参考 SLAM-PNC 的调参，量级按 Go2 缩放）
struct MpcParams {
  // ---------------- 时域 ----------------
  double dt{0.1};  ///< 预测步长 [s]
  int horizon{15}; ///< 预测步数 N（窗口 N+1 点）

  // ---------------- 权重 ----------------
  // 状态误差 e = [x, y, yaw, v]，控制偏差 Δu = [a, ω]，Δ 为相邻控制变化
  double q_x{1000.0};
  double q_y{1000.0};
  double q_yaw{50.0};
  double q_v{3.0};
  double r_a{2.0};
  double r_w{4.0};
  double rd_a{5.0};  ///< 加速度变化率惩罚（平滑）
  double rd_w{14.0}; ///< 角速度变化率惩罚（平滑）
  /// 终点前若干步的状态权重放大倍数（1.0 = 不放大）。
  /// SLAM-PNC 全程同一套 Q；我们给个可调旋钮，因为到点精度会直接决定
  /// "到点停车"的误差（P5.4 验收 ≤ 0.15 m）。
  double terminal_weight_scale{3.0};

  // ---------------- 限幅 ----------------
  double v_max{1.0};
  double v_min{0.0};     ///< 不倒车（差速底盘允许，但任务语义上不要）
  double a_max{1.0};     ///< |纵向加速度| 上限 [m/s²]
  double w_max{0.8};     ///< |角速度| 上限 [rad/s]
  double alpha_max{1.5}; ///< |角加速度| 上限 [rad/s²]（作用在 Δω 上）
  double a_min() const { return -a_max; }
  double w_min_cmd() const { return -w_max; }

  // ---------------- 控制变化率是否作为**硬约束** ----------------
  /// false（默认，同 SLAM-PNC）：只放在代价里惩罚 —— 硬约束在参考突变时会让 QP
  /// 不可行； true：额外加 |Δu| ≤ (a_max·dt, alpha_max·dt) 的硬约束。
  bool constrain_control_rate{false};

  // ---------------- 参考轨迹 ----------------
  double reference_speed{0.0}; ///< >0 时覆盖"默认巡航速度"（0 = 用 v_max/限速）
  /// ★★ A/B 开关（M4.3）：是否用**上游剖面**（`setReferenceProfile()` 给的那一份）
  ///   作为速度上限。
  ///   true （默认）：有剖面时**换掉**本地的曲率限速，并用剖面的 ω 顶替 κ·v；
  ///   false：完全忽略剖面（= M4.2 之前的现网行为，做对照用）。
  bool use_upstream_profile{true};
  double lat_acc_max{1.5};     ///< 曲率限速用：允许的最大向心加速度 [m/s²]
  /// 终点制动剖面用的减速度 [m/s²]。
  /// ★ 有上游剖面时它**仍然是兜底**（不是冗余）：上游剖面可能只覆盖前 N 米，
  ///   尾部需要它接手；而且 `brake_acc` 比 `traj.a_max` 大时上游总是更紧，
  ///   两者不会打架（详见 `speedLimitAt()` 的注释）。
  double brake_acc{1.0};
  double curvature_lookahead{2.0}; ///< 曲率前瞻距离 [m]（提前减速）
  double back_window{1.0}; ///< 投影允许向后回退的最大弧长 [m]（防进度抖）

  // ---------------- 终点段（到点精度）----------------
  /// 进入终点段（剩余弧长 ≤ approach_dist）后把参考速度**压到
  /// approach_speed**。
  ///
  /// 为什么不能只靠 brake_acc 剖面：那个剖面在最后 `v²/(2a)` 米内才归零，对四足
  /// 步态来说太快了 —— 实测"指令归零后车还在 2~4 cm/s 蹭 1 s"，结果停在离目标
  /// 3~5 cm 处（要求 ≤ 3 cm）。改成慢速贴拢后，停止命令下达时车的速度已经很低，
  /// 残余位移从 5 cm 降到 1~2 cm。
  double approach_dist{0.0};  ///< 终点段长度 [m]；0 = 关闭（一直用制动剖面）
  double approach_speed{0.0}; ///< 终点段参考速度上限 [m/s]；0 = 关闭
  /// 终点段参考速度**下限**：参考绝不先于目标归零。
  /// 为什么必须有：底盘有低速死区，且“参考停在半路”会让车差几厘米停住、
  /// 完成判据永远不满足（任务卡死）。留一个很小的爬行速度把最后几厘米走完。
  double crawl_speed{0.0}; ///< 终点段参考速度下限 [m/s]；0 = 关闭
  /// **停车惯性距离** [m]：从“指令变 0”到“车真的停住”，底盘还会自己走多远。
  ///
  /// 这是底盘特性（步态要收完当前步），实测 Go2 仿真约 4 cm，且与快到点时的速度
  /// 无关（0.06/0.04/0.03 m/s 三种贴拢速度下都是 0.062~0.068 m 的姿态位移）。
  /// 用法：终点剖面按 `s_rem − stop_coast` 算 ⇒ **指令提前归零**，让底盘自己的
  /// 惯性把最后这几厘米带完，正好停在目标上。不扣这一段就会冲过目标 3~5 cm
  /// （用户要求停点误差 ≤ 3 cm）。
  double stop_coast{0.0};

  // ---------------- 大航向偏差：先原地对正再走 ----------------
  /// 进入"原地对正"模式的航向偏差阈值 [°/rad]；0 = 关闭。
  ///
  /// 为什么单独做这个模式：参考折线是"沿路径前进"的语义，车头差几十度时 MPC 会
  /// 去找一个"边转边走"的解；在严格走廊/障碍靠近时这个解几乎必然撞硬界 ⇒ 代价上
  /// "原地不动"更便宜 ⇒ **车拒动**（实测 航向差 7/15/25° 用满加速能力起步，
  /// 40/60° → `cmd v=0`）。而差速/足式底盘的 ω 与 v
  /// 是解耦的，原地转正物理可行，
  /// 所以正确行为是"先把机头对到参考方向，再起步"。
  double align_in_place_deg{0.0};
  /// 退出阈值 [°]（滞环，避免在阈值上来回切模式）
  double align_exit_deg{8.0};
  /// 对正角速度增益 [1/s]：ω = gain·(−e_ψ)（再受 w_max 限幅）
  double align_gain{1.2};
  /// 允许原地旋转的最小净距 [m]：车体是矩形，原地转会扫过外接圆。
  /// 离障碍比这个值更近时不转（交给正常流程，宁可停也不剧蹭）。
  double align_min_clearance{0.25};

  // ---------------- 到点后的**目标朝向**对正 ----------------
  /// 目标朝向容差 [°]；0 = 不判定朝向（旧行为）。
  ///
  /// ★ 为什么必须有（用户 2026-09-23 指出）：原来的到点判定只管 xy
  ///   （`remaining − stop_coast ≤
  ///   goal_tolerance`），于是"位置到了、机头朝哪都行" 也算到达 ——
  ///   而任务目标本身是带 yaw 的（例：面向充电桩/面向通道口）。 实测验收只量
  ///   xy，yaw 可以差 90° 都没人发现。
  ///
  /// 行为：到达终点**位置**附近（剩余 ≤ `goal_yaw_align_distance`）而 |目标朝向
  /// 误差| > 本阈值时，原地对正到**目标朝向**（复用 align_gain /
  /// align_min_clearance / w_max），对正到容差内才算到达。
  /// 目标朝向取**路径最后一点**的 yaw（全局规划把目标 yaw 放在末点）。
  double goal_yaw_tolerance_deg{0.0};
  /// 进入"终点朝向对正"的剩余弧长 [m]。
  ///
  /// ⚠ 必须 **> 节点的到点位置容差**（`local.goal_tolerance`）：否则会出现“节点
  ///   已判到点、算法还在等朝向”的死锁 —— 算法在 `remaining≈0` 时会先返回
  ///   kGoalReached + 零速。留 10 cm 是经验值（比常见容差 1~5 cm 大一倍）。
  double goal_yaw_align_distance{0.10};

  // ---------------- 自由模式的“顺滑优先”（用户 2026-09-23）----------------
  //
  // 用户观察：自由导航时机器人**摇摇摆摆**，不是顺滑地走；并且指出
  // “自由导航不需要严格遵循全局路线，而应该保证顺滑”。
  //
  // 拐因：原来自由与走廊用**同一套贴线权重**（`q_x/q_y` 各 1000、`q_yaw` 50），
  // 而自由导航的参考只是“大致往那儿走”的折线（后处理只做共线/视线剪枝）⇒
  // 几个 cm 的横向偏差 + 几度的航向偏差就被当成大误差追，
  // 实测日志：`w` 在两个周期之间从 +0.65 打到 −0.65（顶满），
  // 周期约 1~2 s —— 典型的极限环（控制器比底盘响应快）。
  //
  // 做法：**只有自由模式**放松横向/航向权重 + 加横向死区 + 压 ω 上限；
  // 走廊模式（严格贴线）完全不变。
  //
  // ⚠ 这几个量的默认值是**中立**的（= 旧行为，严格贴线）：
  //   · 库层的"跟踪精度"单测量的是**控制器**本身的能力，默认严格才有意义；
  //   · 而"自由导航要顺滑"是本产品的**策略选择**，写在 local_mpc.yaml 里（已开）。
  /// 自由模式横向权重缩放（1 = 与走廊一样严；走廊模式恒为 1）
  double free_lat_scale{1.0};
  /// 自由模式航向权重缩放（1 = 一样严；走廊模式恒为 1）
  double free_yaw_scale{1.0};
  /// 自由模式横向**死区** [m]：偏差在带内就不纠（把参考线平移到车这一侧），
  /// 只纠超出带的部分。0 = 关。目的是“几 cm 的偏差不值得动方向盘”。
  double free_lat_deadband{0.0};
  /// 自由模式角速度上限 [rad/s]（0 = 用 w_max）。直接治“左右打满”的摆动。
  double free_w_max{0.0};

  // ---------------- 原地对正的“起步角速度” ----------------
  /// 对正（含先对正机头、到点对正目标朝向）时的角速度**下限** [rad/s]，0 = 关。
  ///
  /// ★ 为什么必须有：底盘低速有死区（指令太小步态/电机不动），而
  ///   `ω = align_gain·e` 在 e 小时给不出能动的指令 ⇒ 车停在容差**外**永远
  ///   不再转 ⇒ “无法收敛”（用户 2026-09-23 实测）。给一个下限打破死区：
  ///   宁可多转一点点、下个周期再修，也不要停在那儿不动。
  /// 默认 0（中立）：具体值属底盘特性（死区大小不同），在 <底盘>_run.yaml 里标定。
  double align_w_min{0.0};
  /// 终点朝向对正**无进展超时** [s]（0 = 不限时）：一段时间内偏差没有实质改善
  /// （= 车根本转不动，底盘死区）⇒ 报 kFailed 并说明“还差多少度”，而不是无限
  /// 原地转下去（无限转在任务层看就是卡死）。
  /// ★ 是“无进展”而非“墙钟”：180° @ 0.65 rad/s ≈ 5 s 本来就慢，按墙钟计会把
  ///   “转得慢但一直在收敛”判成失败。
  double goal_yaw_align_timeout{6.0};

  // ---------------- 走廊（route profile）----------------
  /// "严格贴线"（corridor_width = 0）时的数值下限 [m]。
  /// 为什么必须有：要求 0 mm 横向偏差的 QP 在任何噪声下都不可行，会退化成
  /// "一有偏差就 BLOCKED"。0.05 m ≈ 半个格，是"物理上追不到、也没必要追"的量。
  ///
  /// ★ 也是节点判断"车已经（回到了）走廊里"的门槛（`corridorTolerance()`）：
  ///   走廊**只在车真在走廊里时才启用**，参见 local_planner_node 里对
  ///   逐点走廊的处理。
  double corridor_min_tolerance{0.05};

  // ---------------- 障碍（软代价 + 硬下界）----------------
  double obstacle_weight{0.0}; ///< 软代价权重；0 = 关闭（纯跟踪）
  double obstacle_safe_distance{
      0.45}; ///< 软代价开始生效的距离 [m]（约半车宽+余量）
  double obstacle_hard_distance{0.25}; ///< 硬下界 [m]：预测点距离不得小于它
  bool obstacle_hard_enable{true};

  // ---------------- 降级 ----------------
  /// 距离场缺失/过期时的限速 [m/s]（只靠硬判定走路，必须慢）
  double degraded_speed_limit{0.30};

  // ---------------- OSQP ----------------
  int osqp_max_iter{4000};
  double osqp_eps_abs{1e-4};
  double osqp_eps_rel{1e-4};
};

/// MPC 求解结果（诊断用：QP 规模/耗时/是否可行）
struct MpcSolveInfo {
  MpcSolveInfo() = default;
  MpcSolveInfo(const MpcSolveInfo &) = default;
  MpcSolveInfo &operator=(const MpcSolveInfo &) = default;

  int qp_variables{0};
  int qp_constraints{0};
  int solver_iterations{0};
  double solve_ms{0.0};
  std::string solver_status; ///< OSQP 状态字符串
  bool feasible{false};
  int active_corridor_rows{0}; ///< 本周期真正生效的走廊约束行数
  int active_obstacle_rows{0};
  double min_predicted_distance{0.0}; ///< 预测轨迹上的最小障碍距离 [m]
  double max_lateral_deviation{
      0.0};                   ///< 预测轨迹的最大横向偏差 [m]（走廊验收用）
  int corridor_violations{0}; ///< 解后复核出的走廊越界步数（应为 0）
  /// 动力学等式的最大残差 |e_{k+1} - A e_k - B Δu_k|。
  /// 这是"QP 是不是真的把动力学当约束"的体检指标：正常应 ≲ 求解容差（1e-4
  /// 量级），
  /// 明显偏大就说明矩阵组装错了（实测踩过：反应出来就是"预测轨迹瞬移"）
  double max_dynamics_residual{0.0};
  double progress{0.0};    ///< 沿路径进度 0~1
  double cross_track{0.0}; ///< 当前横向偏差 [m]

  // ---- 调参诊断（"为什么只发这么慢/为什么在扭"只能靠这些量说清）----
  double ref_v{0.0};   ///< 参考窗口第一步的速度 ref[0].v（= 本周期限速）
  /// 参考窗口第一步的角速度 ref[0].w（M4.3：有上游剖面时它**来自剖面**，
  /// 否则来自本地 κ·v）。A/B 表里"ω 反向次数/RMS|ω|"就看它。
  double ref_w{0.0};
  /// 上游剖面一致性（M4.4 / R9）：`prof_dev < 0` = 本周期没有剖面。
  /// 口径同 `LocalStats::profile_dev`：分"参考偏差"与"跟踪偏差"两个数。
  double prof_v{0.0};
  double prof_dev{-1.0};
  double prof_track_dev{-1.0};
  double v_now{0.0};   ///< 反馈进来的实际速度
  double ev0{0.0};     ///< 初始速度误差 e_v(0) = v_now - ref[0].v
  double curv{0.0};    ///< 前瞻段内最大 |κ|
  double upper_v{0.0}; ///< 本周期速度上界 v_upper
  double e_yaw0{0.0};  ///< 初始航向误差 e_ψ(0) = robot.yaw − ref[0].yaw [rad]
  double lat0{0.0};    ///< 初始横向偏差 |n_0·e_0[0:2]| [m]
};

/// QP 稀疏结构（列压缩 + “逻辑槽位 → CSC 下标”映射）。
///
/// 放在头文件里是为了让成员有完整类型；**排序/建表**的逻辑（buildSparseLayout）
/// 在 .cpp 里。设计上关键的一点：结构每周期**完全不变**（连数值为 0 的项也保留
/// 原位置），所以 OSQP 工作区可以一直复用并真正热启动；
/// 代价是“每周期先把 values 清零再按槽位填值”。
struct SparseLayout {
  int rows{0};
  int cols{0};
  std::vector<int> col_ptr;     ///< CSC 列指针（长度 cols+1）
  std::vector<int> row_idx;     ///< CSC 行下标（长度 nnz）
  std::vector<double> values;   ///< 数值（长度 nnz，顺序与 CSC 一致）
  std::vector<int> slot_to_csc; ///< 逻辑槽位 → CSC 下标

  int nnz() const { return static_cast<int>(values.size()); }
  int cscIndex(int slot) const {
    return slot_to_csc[static_cast<std::size_t>(slot)];
  }
  void zeroValues() { std::fill(values.begin(), values.end(), 0.0); }
};

class MpcLocalPlanner : public LocalPlanner {
public:
  MpcLocalPlanner();
  ~MpcLocalPlanner() override;

  std::string type() const override { return "mpc"; }
  bool configure(const ParamReader &params) override;

  void setGlobalPlan(const std::vector<Pose2D> &path) override;
  /// 传非空 ⇒ route profile（走廊中心线成为参考线）；传 nullptr ⇒ free profile
  void setCorridor(const RouteCorridor *corridor) override;
  void setSpeedLimit(double v_limit) override;
  void setCostMap(std::shared_ptr<const CostMap2D> local_inflated) override;
  void setDistanceField(const LocalDistanceField *field) override;
  void setReferenceProfile(const ReferenceProfile *profile) override;
  void setDynamicObstacles(const std::vector<DynamicObstacle> &obs) override;

  LocalPlanResult computeCommand(const Pose2D &pose, double dt) override;
  void reset() override;
  /// ★ MPC 真的会发速度
  bool producesCmdVel() const override { return true; }
  /// 缺距离场 ⇒ 降级（只用硬判定 + 限速）
  bool degraded() const override {
    return dist_field_ == nullptr || !dist_field_->valid();
  }
  /// 停车惯性距离 = 终点剖面里扣掉的那个量（见 MpcParams::stop_coast）。
  /// 节点用同一个值做“到点判定”的提前量，不重复配一遍参数。
  double stopCoast() const override { return p_.stop_coast; }
  /// 严格走廊的有效容差（节点用它决定"车是否已在走廊里"）
  double corridorTolerance() const override {
    return p_.corridor_min_tolerance;
  }
  /// 本算法允许的最大速度（节点算限速区前瞻距离用，见 LocalPlanner::maxSpeed）
  double maxSpeed() const override { return p_.v_max; }
  /// 本算法的减速能力（节点算限速区前瞻距离用，见 LocalPlanner::brakeAcc）
  double brakeAcc() const override { return p_.brake_acc; }
  /// 是否处于“原地对正”模式（诊断/单测用）
  bool aligning() const { return aligning_; }
  /// 是否处于“**终点朝向**对正”模式（诊断/单测用）
  bool goalYawAligning() const { return goal_aligning_; }
  /// 上一周期实际生效的角速度上限 [rad/s]（自由模式会被 free_w_max 压；单测用）
  double lastWLimit() const { return last_w_limit_; }

  const MpcParams &params() const { return p_; }
  const MpcSolveInfo &solveInfo() const { return info_; }

  /// 诊断字符串（见
  /// LocalPlanner::diagString）：调参时一眼看清"为什么这么慢/在扭"
  std::string diagString() const override;
  /// 到点后的**目标朝向**对正；返回 true = 本周期已处理（out 已填好，直接
  /// return）
  bool goalYawAlign(const Pose2D &pose, double remaining, LocalPlanResult &out);
  /// 对正用的角速度（`align_gain·e` 限幅 + `align_w_min` 下限打破底盘死区）
  double alignRate(double e_yaw) const;
  /// 本模式下的角速度上限：自由模式会被 `free_w_max` 压（治“左右打满”的摆动）
  double wMaxEff(bool route_mode) const
  {
    return (route_mode || p_.free_w_max <= 0.0)
               ? p_.w_max
               : std::min(p_.w_max, p_.free_w_max);
  }
  /// 见 LocalPlanner::goalYawTolerance（节点用它做到点判定）
  double goalYawTolerance() const override {
    return p_.goal_yaw_tolerance_deg * M_PI / 180.0;
  }
  /// 上一周期解出的预测轨迹（世界系，N+1 点）—— 可视化与单测用
  const std::vector<Pose2D> &predictedTrajectory() const { return predicted_; }

  /// 单测用：直接喂一条参考窗口并求解（绕过路径投影/弧长参数化）
  bool solveForTest(const Pose2D &current, double current_v,
                    const std::vector<MpcReferencePoint> &ref, Twist2D &cmd,
                    LocalStatus &status, std::string &message);

  /// 调试：把当前 QP（H/q/A/l/u 的稀疏非零项）导出成文本。
  /// 排查"约束到底有没有生效"时比猜快得多 —— MPC 出问题九成时候 90% 的时
  /// 都是矩阵组装错了，而不是求解器或权重不对。
  void dumpQp(std::string &out) const;

  /// 单测/离线分析用：把当前 QP
  /// 的**稠密**形式整套导出（含上一次的解与对偶解）。
  ///
  /// 为什么要结构化而不是文本：P5.2 的"一致性校验"要逐项比对矩阵与 KKT 条件，
  /// 文本解析既脆弱又容易把"格式对"当成"数值对"。
  struct QpSnapshot {
    int n{0};                 ///< 决策变量数
    int m{0};                 ///< 约束数
    std::vector<double> P;    ///< n×n，行优先，**已对称化**（OSQP 只存上三角）
    std::vector<double> q;    ///< n
    std::vector<double> A;    ///< m×n，行优先
    std::vector<double> l, u; ///< m
    std::vector<double> x, y; ///< 上一次的解 / 对偶解（未求解时为空）
  };
  void snapshotQp(QpSnapshot &out) const;

private:
  // ---------------- 参考轨迹（弧长参数化）----------------
  /// 把折线变成等距弧长采样（含累计弧长与逐点曲率）
  void rebuildReference();
  /// 把机器人投影到参考线上 → 返回弧长 s（受限在 back_window 内不倒退）
  double projectOntoReference(double x, double y, double &cross_track) const;
  /// 弧长 → 参考点（线性插值 + 切线朝向 + 曲率/速度/加速度）
  MpcReferencePoint sampleAt(double s, double lookahead) const;
  /// 参考速度剖面：v_max/限速 ∩ 曲率限速 ∩ 终点制动
  double speedLimitAt(double s) const;

  /// 是否有一份**可用**的上游剖面（A/B 开关 ⊕ 非空 ⊕ 有效）。
  /// 定义在 .cpp：这里只需要指针，但 `valid()` 需要完整类型。
  bool hasUpstreamProfile() const;
  /// 构造 N+1 点参考窗口（从弧长 s0 起，按 v_ref 积分推进）
  bool buildReferenceWindow(double s0,
                            std::vector<MpcReferencePoint> &window) const;

  // ---------------- QP ----------------
  /// 建 H（上三角 CSC）/q/A/l/u 的**数值**；稀疏结构在 configure 时定死
  bool buildQp(const std::vector<MpcReferencePoint> &ref, const Pose2D &pose,
               double current_v, bool route_mode, double half_width);
  bool solveQp();
  /// 解后校验：预测轨迹的走廊越界量、障碍最小距离
  bool validateSolution(const std::vector<MpcReferencePoint> &ref,
                        bool route_mode, double half_width, std::string &why);

  /// 障碍硬下界的**每步参考净距**（长度 horizon+1），由 buildQp
  /// 填，**仅用于诊断**。 语义：`d(参考点
  /// k)`，即"规划路径自己"在该步离障碍多远。 为什么值得存：QP 报 `primal
  /// infeasible` 时，十有八九是**参考本身就在硬距离内**
  /// （全局图与局部距离场不同源、路径被新障碍占了……）。这个数能让报错直接指向
  /// 真正该修的那一侧，而不是让人去猜模型/权重。
  std::vector<double> obs_ref_dist_;
  /// 失败诊断辅助：参考自身在硬距离内时返回一句可操作的说明，否则空串
  std::string refClearanceNote() const;

  // ---------------- 变量布局（与 SLAM-PNC 一致）----------------
  int stateIndex(int k, int i) const { return 4 * k + i; }
  int controlIndex(int k, int j) const {
    return 4 * (p_.horizon + 1) + 2 * k + j;
  }

  MpcParams p_;
  MpcSolveInfo info_;

  // 参考线（弧长参数化）
  std::vector<double> ref_x_, ref_y_;
  std::vector<double> ref_s_;    ///< 累计弧长
  std::vector<double> ref_curv_; ///< |κ|（1/m）
  std::vector<double> ref_yaw_;  ///< 切线朝向（离散点处的朝向更平滑）
  double ref_length_{0.0};

  /// 上游速度剖面（M4.3）。**非拥有**：节点拥有（`local_planner_node::ref_profile_`），
  /// 算法只是每周期借用 —— 与 `dist_field_` 同款约定。nullptr = 没有剖面。
  const ReferenceProfile *profile_{nullptr};

  /// 当前进度（弧长），用于防倒退与"卡住"诊断
  double progress_s_{0.0};
  bool has_progress_{false};

  /// route 模式下用走廊中心线（非拥有，来自 setCorridor 的指针）
  bool route_mode_{false};

  /// 是否处于“原地对正”模式（带滞环的状态，见 align_in_place_deg）
  bool aligning_{false};

  /// 是否处于“**终点朝向**对正”模式（与 aligning_ 分开：两者目标不同 ——
  /// 前者对的是参考切线方向、后者对的是目标 yaw，滞环也不能共用一个）
  bool goal_aligning_{false};
  /// 进入“终点朝向对正”的时刻（用于 goal_yaw_align_timeout 判超时）
  std::chrono::steady_clock::time_point goal_align_since_{};
  /// 进对正后见过的最小偏差 [°]（偏差改善 ≥ 2° 就把超时计时重置）
  double goal_align_best_deg_{1e9};
  /// 自由模式下实际使用的角速度上限（诊断/单测用）
  double last_w_limit_{0.0};

  // 预测轨迹（世界系）
  std::vector<Pose2D> predicted_;
  double smooth_v_{0.0}; ///< 上一次下发的 v（用于异常时的退化策略）
  double smooth_w_{0.0};

  // ---- QP 静态结构（configure 时定死；每周期只改数值）----
  SparseLayout p_layout_, a_layout_;
  std::vector<int> p_slot_state_, p_slot_xy_, p_slot_ctrl_, p_slot_rate_;
  std::vector<int> a_slot_eq0_, a_slot_next_, a_slot_diag_, a_slot_a_,
      a_slot_b_, a_slot_vel_, a_slot_ctrl_, a_slot_rate_, a_slot_corr_,
      a_slot_obs_;
  /// 各约束块在 A 里的起始行（行布局的唯一真相，改布局只改这里）
  struct RowBase {
    int base_vel{0};
    int base_ctrl{0};
    int base_rate{0};
    int base_corr{0};
    int base_obs{0};
  } row_base_;
  std::vector<double> q_val_, l_val_, u_val_;

  // OSQP C 工作区与 CSC 结构（void* 是为了不在头文件里 include osqp.h）
  void *osqp_work_{nullptr};
  void *osqp_P_{nullptr};
  void *osqp_A_{nullptr};
  /// 热启动用的初值（长度 = 变量数 n / 约束数 m，setup 时定尺）。
  ///
  /// ★ 为什么不能直接 `osqp_warm_start(work, work->solution->x, ...)`：
  ///   上一次求解**失败**时，`solution->x/y`
  ///   是那次迭代跑到的地方（可能已发散）。 拿它当下一次的起点，OSQP 会把 4000
  ///   次迭代全花在往回拉/重缩放上， 于是"一次不可行 →
  ///   永久不可行"：障碍/区域已经挪走了、`障碍行0`， 日志里还是 `maximum
  ///   iterations reached`，车一步不走。 实测踩到（2026-09-23，test_zones.py 第
  ///   2/3 段）：禁行带挡路一次之后， 把带子挪到 8 m 外也再也没动过。
  ///   策略：只把**可行解**存下来做热启动；上一次失败就回零（等价于冷启动）。
  std::vector<double> ws_x_, ws_y_;
};

} // namespace pnc_2d
