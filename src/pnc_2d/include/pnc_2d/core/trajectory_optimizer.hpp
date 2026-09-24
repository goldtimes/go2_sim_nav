// 轨迹优化（后端）抽象层：把一条"几何可行但不可跟踪"的折线，变成
// "几何 + 运动学都可行"的**时间参数化**轨迹。**ROS-free**（与 global_planner.hpp
// / local_planner.hpp 同款约定：类型跟着接口走，不依赖节点）。
//
// 分层分工（见 doc/minco_trajectory_plan.md §2）：
//   全局规划器（A*）   → 拓扑可达的折线（离散避障；净距是"格级"的）
//   本层（后端/MINCO） → 连续净距 + 曲率/运动学可行 + 时间参数化（= **剖面**）
//   局部（MPC）        → 跟踪 + 底盘执行 + 末段停靠
//
// ★ 剖面归属：`t` / `ṡ` / `θ̇` 由**本层**产生，且**必须下发给局部**。
//   原版 DDR-opt 的实测教训（`matrix_q` 里 v 权重为 0）说明：剖面的主要作用是
//   "参考点的时序" ⇒ 局部若仍用自己的时间分配，"跟剖面"就无从谈起。
//
// ★ 失败策略（调用方必须遵守）：本层返回非 SUCCESS 时**降级用输入路径原样走**，
//   绝不因为"平滑失败"让任务失败。

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

class ClearanceField;
class CostMap2D;
class ParamReader;
class FootprintCollisionChecker;

/// 轨迹采样点：`(θ, s)` 参数化的物理量都已还原到 XY（见 minco_optimizer 的说明）
struct TrajSample {
  double t{0.0};       // [s] 从轨迹起点算起（= 剖面时间轴）
  double s{0.0};       // [m] 沿轨迹弧长
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};       // [m/s]    线速度（= ṡ）
  double omega{0.0};   // [rad/s]  角速度（= θ̇）
  double a{0.0};       // [m/s²]   线加速度（= s̈）
  double alpha{0.0};   // [rad/s²] 角加速度（= θ̈）
};

enum class TrajStatus : uint8_t {
  kSuccess = 0,
  /// 优化没做成，用输入路径原样（**任务照走**，调用方只需记一笔）
  kFallbackInput = 1,
  /// 输入不足/退化（点数 < 2、首末点重合……）
  kNoInput = 2,
  /// 求解器/数值失败（时间过短、积分发散……）
  kSolverFailed = 3,
  /// **起点不可规划**：车体**真实轮廓**（safe_margin = 0）已经压在致命格上。
  ///
  /// ★ 这与 A* 的口径**必须一致**（见 `astar_planner.cpp` 的虚拟起点守卫）：
  ///   起点落在**余量带**里是常态（全局图与感知图差几个 cm），要照常规划；
  ///   但真实轮廓都放不下，说明车真的压上去了（或者离障碍不到半车宽）——
  ///   这时该停车报错交人工，**不能**从重叠位置规划一条"开出去"的轨迹：
  ///   ① 那条轨迹的净距统计、终点修正全部建在假前提上，数字是骗人的；
  ///   ② 实测会算出一条 80 s / 时间×6.85 的"逃生"轨迹，比直接失败更糟 ——
  ///      它把"定位错了 / 地图错了"这个**真故障**藏起来了。
  kStartBlocked = 4,
  /// **终点不可规划**：目标位姿的车体**真实轮廓**（safe_margin = 0）压在致命格上。
  ///
  /// ★ 与 `kStartBlocked` **对称**（也就与 A* 虚拟起点守卫同口径）：轨迹的**末状态
  ///   是硬约束**（终点 XY + 终点朝向）⇒ 目标点不可达时，末端修正只能靠"绕/挤"去够它，
  ///   净距必然被压穿；而 goal_checker 又永远判不到点 ⇒ 任务卡死，而不是干净失败。
  ///   实测：终点落在柱子里的输入会算出 `3894 点 / 194.64 s / 17.20 m / max|v| 0.16 m/s
  ///   / 时间缩放 ×3.95` —— 又是一个"能跑但全是假前提"的轨迹。
  ///   只拒"真实轮廓压上"：终点落在余量带里是常态（用户点得离墙近一点），要照常规划。
  kGoalBlocked = 5,
  /// **轨迹终检不过**（M3）：优化出来的轨迹逐位姿查轮廓时发现在硬要求之下。
  /// 调用方**不得执行** `samples`（它们是诊断用的）—— 应降级 A* 原路径，
  /// 并在 status 里报原因（`message` 会点名失败点坐标 + 实测净距）。
  kCheckFailed = 6,
  /// **时间缩放没收敛**：缩放跑满上限后，轨迹的 max|v|/|ω|/|a|/|α| 仍在限外。
  /// 与 `kCheckFailed` 是**两条独立的门**：终检只管"碰不碰"，这里管"跟得上跟不上"。
  /// 调用方同样**不得执行** `samples`（应降级 A* 原路径）。
  kKinematicsFailed = 7,
};

inline const char * toString(TrajStatus s)
{
  switch (s) {
    case TrajStatus::kSuccess: return "SUCCESS";
    case TrajStatus::kFallbackInput: return "FALLBACK_INPUT";
    case TrajStatus::kNoInput: return "NO_INPUT";
    case TrajStatus::kSolverFailed: return "SOLVER_FAILED";
    case TrajStatus::kStartBlocked: return "START_BLOCKED";
    case TrajStatus::kGoalBlocked: return "GOAL_BLOCKED";
    case TrajStatus::kCheckFailed: return "CHECK_FAILED";
    case TrajStatus::kKinematicsFailed: return "KINEMATICS_FAILED";
  }
  return "UNKNOWN";
}

/// 诊断量：**每一个都是量出来的**（给 A/B 表与 status 日志用）
struct TrajOptStats {
  double solve_ms{0.0};
  /// 分阶段耗时（诊断“到底慢在哪里”用；加起来≈ solve_ms）
  double smooth_ms{0.0};   // Stage A 弹性带（L-BFGS）
  double minco_ms{0.0};    // Stage B MINCO（建路点/终点修正/时间缩放）
  double stats_ms{0.0};    // 净距/曲率统计（**曾经是最贵的一段**，见实现注释）
  double duration{0.0};        // 轨迹总时长 [s]
  double path_length{0.0};     // 轨迹弧长 [m]
  /// 到障碍的最小**轮廓净距**（排除首末点：首点=车自己、末点=目标，会掩盖差异）
  double min_clearance{0.0};
  double min_clearance_s{0.0};  // 上面那个点的弧长
  bool clearance_valid{false};  // 没有地图/距离场时为 false（**不要用哨兵值**）
  /// 轨迹相对输入折线的最大偏移 [m]（衡量"平滑改了多少"，也给 MPC 侧参考）
  double path_shift_max{0.0};
  double max_v{0.0};
  double max_omega{0.0};
  double max_a{0.0};
  double max_alpha{0.0};
  double max_curvature{0.0};    // |θ̇/ṡ|
  /// 终点 XY 误差 [m]：`(θ,s)` 的 XY 是**积分派生量**，所以这个必须有
  double terminal_error{0.0};
  double terminal_error_yaw{0.0};
  /// 为了满足运动学整体拉长时间的比例（1.0 = 无需拉长）
  double time_scale{1.0};
  /// 时间缩放的**收敛**结果：true = 缩放后 max|v|/|ω|/|a|/|α| 真的都在限内。
  /// ★ 为什么必须有这一项：时间缩放最多跑 3 轮，**可能没收敛** —— 而 M3 终检只管
  ///   净距、不管运动学 ⇒ 不加这一项就会**默默发出一条超限的轨迹**（MPC 跟不上）。
  bool kin_ok{false};
  /// **终检**（M3）结果：逐位姿、含朝向的全机轮廓检查（口径见 `checkTrajectory`）
  bool check_ok{false};
  int check_points{0};            // 硬门实际查了多少个位姿
  int check_stats_points{0};      // 统计净距用了多少个位姿（网格更粗）
  int check_violations{0};        // 硬要求不过的点数
  double check_worst_clearance{0.0};   // **全部**采样点的最小轮廓净距（不排除首末）
  double check_worst_s{0.0};
  /// 平滑段的净距目标与实际达成（衡量"避障"这一步到底做了多少）
  /// ★ 目标现在可能是**逐点**的（端点锚定附近会自动下调，见 `smooth_d_cap_*`）
  ///   ⇒ **不要**拿 `smooth_d_achieved` 直接比 `smooth_d_target` 来判好坏，
  ///   要看 `smooth_shortfall`（= max(逐点目标 − 实际)，0 = 都达标）。
  double smooth_d_target{0.0};
  double smooth_d_achieved{0.0};   // 平滑后**中心线**沿线的实际最小净距 [m]
  /// `max(逐点目标 − 实际净距)` [m]：**0 = 平滑在自己的目标下做到了**。
  /// 为什么不用全局 `smooth_d_target` 比：首末点被锚定、平滑动不了它们，端点
  /// 附近的净距**几何上不可达**全局目标 ⇒ 旧口径会恒定报"未达成"。
  double smooth_shortfall{0.0};
  /// 端点自适应下限（机制 ①，`min(d_base, 0.85 × 端点实测净距)`）；等于
  /// `smooth_d_target` 时就表示"端点开阔、机制未生效"。
  double smooth_d_cap_head{0.0};
  double smooth_d_cap_tail{0.0};
  double smooth_iterations{0.0};
  std::string note;   // 给日志用的一句人话（不写"优化失败"四个字）
  /// 终检（M3）的一句话结论：通过/不过 + **点名的坐标与实测净距**。
  /// 不过时它就是 `message` 的主体 —— 调用方必须因此**不用**这条轨迹。
  std::string check_note;
};

struct TrajOptRequest {
  /// 输入折线（A* 的结果）。首点通常=车、末点=目标；内部会强制锚定成
  /// `start` / `goal` 的位置。
  std::vector<Pose2D> path;
  Pose2D start;                // 车当前位姿（yaw 用 has_yaw 决定是否采用）
  double start_v{0.0};         // [m/s]   当前线速度（上行状态；拿不到给 0）
  double start_omega{0.0};     // [rad/s] 当前角速度
  Pose2D goal;                 // 目标（位置 + 朝向）
  /// true = 终点朝向取 `goal.yaw`（要求轨迹末段把机头转过去）
  /// false = 终点朝向取路径末端**切线**（末段对正交给 shim/goal checker）
  bool use_goal_yaw{true};
  double goal_v{0.0};          // 末速 [m/s]（0 = 停）
};

struct TrajOptResult {
  TrajStatus status{TrajStatus::kNoInput};
  std::vector<TrajSample> samples;   // 走完（按 t 递增，含首末点）
  TrajOptStats stats;
  std::string message;
};

class TrajectoryOptimizer {
public:
  virtual ~TrajectoryOptimizer() = default;
  virtual std::string type() const = 0;

  /// 参数（前缀 `traj.*`）；失败返回 false（与三个工厂的约定一致）
  virtual bool configure(const ParamReader & params) = 0;
  /// 地图与距离场由外部共享传入（同 FootprintCollisionChecker 的做法）
  virtual void setMap(std::shared_ptr<const CostMap2D> map) = 0;
  /// 距离场的**值与梯度**（平滑那段用：这是唯一需要连续梯度的地方）
  virtual void setClearanceField(const ClearanceField * cf) = 0;
  /// 轮廓判定器：净距统计与终检都用它。
  /// ★ 传**已经配好**的那一份（硬阈值/未知格策略/地图/距离场都是单一数据源），
  ///   不要在这里再配一遍 —— "同一个判据配两遍"我们已经栽过多次。
  virtual void setCollisionChecker(const FootprintCollisionChecker * ck) = 0;
  virtual void reset() = 0;

  virtual TrajOptResult optimize(const TrajOptRequest & req) = 0;
};

// ============================================================================
// M3：**轨迹终检** —— 连续、含朝向的全机轮廓检查（M2 的安全门）
//
// 为什么需要单独一层：
//   · 优化器的净距罚瞄的是"**舒适**净距"（中心线到障碍 ≥ 外接圆半径 + margin），
//     它是**偏好**不是保证，而且用的是**中心线点距**（与朝向无关），比真实要求松；
//   · 最终交给 MPC 执行的东西必须是"**真的不碰**"的 —— 这只能**逐位姿含朝向**地查；
//   · 现有系统只有**节点级**检查（每个路径点单独看），没有**连续**检查：
//     `START_FOOTPRINT_COLLISION` 那类问题都出在这里（点**之间**被扫过的区域没人查）。
//
// ★ 采样必须**同时**按弧长和**朝向**：原地转（起点朝向与路径差很大时就是）几乎
//   不产生弧长，只按 `ds` 采会**整段漏掉**旋转扫掠的区域。所以每个子步取
//   `max(⌈Δs/ds⌉, ⌈|Δyaw|/dyaw⌉)` 个子样本。
//
// 两级阈值（**不要**和优化器的罚函数混成一个数）：
//   · `hard_margin`：**硬要求**，`signedClearanceAt ≥ 它` 才算过。默认 0 = "真实
//     轮廓不碰致命格"。不过 ⇒ 这条轨迹不能用。
//   · `warn_margin`：**只记一笔**（< 它就记 warn 并在 status 里点名）。默认取
//     `footprint.safe_margin`。为什么不拿它当硬门：轮廓判定把位姿**吸附到格心**
//     （误差 ±半格 = ±5 cm，见 `footprint_collision.hpp` 的 R13 说明），半格量级的
//     余量当硬门会**随机失败** —— 那是判据噪声，不是性能问题。
struct TrajCheckParams {
  double ds{0.02};              // [m] **硬门**的弧长采样间距
  double dyaw{0.10};            // [rad] **硬门**的朝向采样间距（原地转靠它兜底）
  /// **统计**（最小净距）用的网格 —— 比硬门粗得多。
  /// 为什么可以粗：硬门已经保证不碰，净距只是给日志/调参看的一个**数**；
  /// 而 `signedClearanceAt` 内部是 14 步二分（每步重建轮廓偏移表，~20~40 µs），
  /// 比硬门用的布尔判定（~3 µs）贵一个数量级 —— 实测用户现场一遍 2 cm 净距
  /// 二分要 **122.7 ms**（超 50 ms 预算）。
  double stats_ds{0.10};        // [m]
  double stats_dyaw{0.20};      // [rad]
  double hard_margin{0.0};      // [m] 硬要求（**含首端**：起点原地转的扫掠要一起查）
  double warn_margin{0.05};     // [m] 低于它只记 warn
  double stats_exclude_s{0.20}; // [m] **统计**最小净距时排除首端（首点 = 车自己）
  double max_search{1.0};       // [m] 净距饱和值
};

struct TrajCheckResult {
  bool ok{false};                  // 硬门结果
  int points{0};                   // 硬门实际查了多少个位姿
  int stats_points{0};             // 统计净距用了多少个位姿（比硬门粗）
  int violations{0};               // `hard_margin` 不过的点数
  /// 统计口径（排除首端 `stats_exclude_s`）的最小轮廓净距
  bool min_valid{false};
  double min_clearance{0.0};
  double min_clearance_s{0.0};
  double min_clearance_x{0.0};
  double min_clearance_y{0.0};
  /// **全部**子样本（不排除首端）的最小净距 —— 硬门的依据
  double worst_clearance{0.0};
  double worst_clearance_s{0.0};
  /// 第一个不过硬门的点（`ok == true` 时无意义）
  double first_bad_s{0.0};
  double first_bad_x{0.0};
  double first_bad_y{0.0};
  double first_bad_clearance{0.0};
  std::string note;                // 一句人话（点名坐标 + 实测净距）
};

/// 沿轨迹逐位姿做全机轮廓终检（见上面的口径说明）。
/// `ck` 没有可用地图时**不做检查**（`ok = true`、`min_valid = false`，与
/// `clearance_valid` 同一约定：没有判据时不要假装查过了）。
TrajCheckResult checkTrajectory(const std::vector<TrajSample> & samples,
                                const FootprintCollisionChecker & ck,
                                const TrajCheckParams & prm);

/// 把折线路径转成"朝向 = 线段方向、v/ω = 0"的样本序列（每段按 `ds` 细分）。
/// 用途：M3 同样要能终检 **A\\* 原路径**（降级路径也必须是查过的），
/// 以及给节点做"发布前再查一遍"。
std::vector<TrajSample> trajSamplesFromPath(const std::vector<Pose2D> & path, double ds);

}  // namespace pnc_2d
