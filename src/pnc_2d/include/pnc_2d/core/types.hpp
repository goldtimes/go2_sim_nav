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

  bool ok() const { return status == PlannerStatus::kSuccess; }
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
