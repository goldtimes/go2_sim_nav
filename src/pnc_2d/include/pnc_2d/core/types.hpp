// 规划的公共类型：位姿、状态码、代价模型、请求/结果、搜索窗口。
// 独立成一个头文件，避免"算法 ←→ 碰撞检查"之间的循环依赖。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pnc_2d {

/// 二维位姿（世界系）。has_yaw=false 表示该点没有明确朝向（例如纯路径点）。
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool has_yaw{false};
};

/// 规划结果状态。失败必须能说明**原因** —— 不允许返回"上一次的路径"充数。
enum class PlannerStatus {
  kSuccess = 0,
  kNotInitialized,             // 地图未设置 / 地图无效
  kInvalidInput,               // NaN、起点终点重合、目标在起点搜索范围内不存在等
  kStartOutOfMap,
  kGoalOutOfMap,
  kStartOccupied,              // 起点中心格致命
  kGoalOccupied,
  kStartFootprintCollision,    // 起点处车体轮廓与障碍重叠（比"中心格致命"更准确）
  kGoalFootprintCollision,
  kNoPath,
  kTimeout,
  kMaxIterations
};

const char * toString(PlannerStatus s);

/// 局部规划（跟随）状态。**独立枚举**，不复用 PlannerStatus：
/// "全局规划失败"与"局部被挡住/到达终点"是两类语义，混用会让状态机写不清。
enum class LocalStatus {
  kIdle = 0,      // 没有可跟随的路径，或尚未启动
  kFollowing,     // 正在沿路径走
  kGoalReached,   // 到达路径终点（局部负责判定）
  kBlocked,       // 前方被挡且不可绕（路网模式下走廊被占 ⇒ 停车等状态机）
  kDegraded,      // 降级运行（例如距离场超时，只用硬碰撞兜底 + 减速）
  kFailed         // 求解失败/内部错误
};

const char * toString(LocalStatus s);

/// 走廊约束：`route` profile 的来源（全局路网规划给出的中心线 + 允许偏离半宽）
struct RouteCorridor {
  std::vector<Pose2D> centerline;  ///< 世界系中心线（已填 yaw 更佳）
  double half_width{0.0};          ///< 允许横向偏离半宽 [m]；0 = 严格贴线
  double speed_limit{0.0};         ///< 该走廊限速 [m/s]；0 = 不限（用算法上限）
  int edge_index{-1};              ///< 来源通道下标（诊断/可视化用）

  bool valid() const { return centerline.size() >= 2; }
  bool strict() const { return half_width <= 1e-9; }
};

/// 动态障碍（供 MPC 做"预测位置处的代价"）。
/// v1（反应式）只收不用；速度/置信度由感知侧的跟踪器在未来提供，
/// 见 doc/mpc_local_planner_plan.md §7。
struct DynamicObstacle {
  int id{0};
  Pose2D pose;              ///< 当前位姿（世界系）
  double vx{0.0};           ///< 世界系速度 [m/s]
  double vy{0.0};
  double radius{0.3};       ///< 等效半径 [m]
  double confidence{1.0};   ///< 置信度 0~1
  double stamp{0.0};        ///< 观测时刻 [s]
};

/// 栅格代价语义（所有算法共用；逐算法可用不同参数）
struct CostModel {
  /// ≥ 该值视为硬障碍（nav2 的惯例是 253/80 这一类阈值）
  int hard_threshold{80};
  /// 未知格（-1）是否视为障碍
  bool unknown_as_occupied{true};
  /// 软代价权重：(raw/threshold)² × soft_cost_weight 加进路径代价；0 = 关闭
  double soft_cost_weight{5.0};

  enum class Kind { kFree, kSoft, kHard };
  Kind classify(int8_t raw) const;
  /// 进入该格的额外代价（不含"走的距离"）；硬障碍返回 +inf
  double extraCost(int8_t raw) const;
};

/// 规划统计（日志/调参用）
struct PlannerStats {
  double plan_time_ms{0.0};
  long expanded_nodes{0};
  long discovered_nodes{0};
  long max_open_set{0};
  int windows_tried{0};        // 搜索窗口尝试次数（>1 说明触发了扩大重试）
  long footprint_full_checks{0};  // 走了完整矩形检查的次数（诊断快路径效果）
  double path_length{0.0};     // 输出路径长度 [m]
};

struct PlanRequest {
  Pose2D start;
  Pose2D goal;
};

struct PlanResult {
  PlannerStatus status{PlannerStatus::kNotInitialized};
  std::string message;
  std::vector<Pose2D> path;    // 世界系，已填 yaw
  PlannerStats stats;

  // ---- 走廊（"贴路网通道走"）----
  // 语义：**path 本身就是走廊中心线**，允许横向偏离 ±corridor_half_width。
  // has_corridor=false ⇒ 自由空间跟踪（局部只需跟踪，不必贴线）。
  //
  // 为什么只存"摘要"而不是每个点一份宽度：本仓库的接收端
  // （local_planner_node）就是按这一条语义实现的——用整条路径当中心线，
  // 宽度取各点最小值。一条任务跨多条通道时取**最严**的那条（半宽最小、
  // 限速最小），宁可保守：偏出严格通道比在宽通道里少偏一点严重得多。
  bool has_corridor{false};
  double corridor_half_width{0.0};   ///< 允许横向偏离半宽 [m]；0 = 严格贴线
  double corridor_speed_limit{0.0};  ///< 走廊限速 [m/s]；0 = 不限
  std::vector<int> route_edges;      ///< 用到的通道下标（诊断/可视化）

  bool ok() const { return status == PlannerStatus::kSuccess; }

  /// 严格贴线的走廊：宽度 0，遇障只能停（不能绕）。
  bool strictCorridor() const
  {
    return has_corridor && corridor_half_width <= 1e-9;
  }
};

/// 栅格搜索窗口（闭区间，含边界）
struct SearchWindow {
  int x0{0};
  int y0{0};
  int x1{-1};
  int y1{-1};

  bool valid() const { return x1 >= x0 && y1 >= y0; }
  bool contains(int x, int y) const { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
  long cells() const
  {
    return valid() ? static_cast<long>(x1 - x0 + 1) * static_cast<long>(y1 - y0 + 1) : 0;
  }
};

}  // namespace pnc_2d
