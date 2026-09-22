// 公共类型的实现：状态码字符串、代价模型语义。

#include "pnc_2d/core/types.hpp"

#include <cmath>
#include <limits>

namespace pnc_2d {

const char * toString(PlannerStatus s)
{
  switch (s) {
    case PlannerStatus::kSuccess: return "SUCCESS";
    case PlannerStatus::kNotInitialized: return "NOT_INITIALIZED";
    case PlannerStatus::kInvalidInput: return "INVALID_INPUT";
    case PlannerStatus::kStartOutOfMap: return "START_OUT_OF_MAP";
    case PlannerStatus::kGoalOutOfMap: return "GOAL_OUT_OF_MAP";
    case PlannerStatus::kStartOccupied: return "START_OCCUPIED";
    case PlannerStatus::kGoalOccupied: return "GOAL_OCCUPIED";
    case PlannerStatus::kStartFootprintCollision: return "START_FOOTPRINT_COLLISION";
    case PlannerStatus::kGoalFootprintCollision: return "GOAL_FOOTPRINT_COLLISION";
    case PlannerStatus::kNoPath: return "NO_PATH";
    case PlannerStatus::kTimeout: return "TIMEOUT";
    case PlannerStatus::kMaxIterations: return "MAX_ITERATIONS";
  }
  return "UNKNOWN";
}

const char * toString(LocalStatus s)
{
  switch (s) {
    case LocalStatus::kIdle: return "IDLE";
    case LocalStatus::kFollowing: return "FOLLOWING";
    case LocalStatus::kGoalReached: return "GOAL_REACHED";
    case LocalStatus::kBlocked: return "BLOCKED";
    case LocalStatus::kDegraded: return "DEGRADED";
    case LocalStatus::kFailed: return "FAILED";
  }
  return "UNKNOWN";
}

CostModel::Kind CostModel::classify(int8_t raw) const
{
  if (raw < 0) return unknown_as_occupied ? Kind::kHard : Kind::kFree;
  if (raw >= static_cast<int8_t>(hard_threshold)) return Kind::kHard;
  return (raw > 0) ? Kind::kSoft : Kind::kFree;
}

double CostModel::extraCost(int8_t raw) const
{
  if (classify(raw) == Kind::kHard) return std::numeric_limits<double>::infinity();
  if (soft_cost_weight <= 0.0 || raw <= 0) return 0.0;
  // 归一化后平方：越靠近障碍，惩罚增长越快，路径因此自觉走通道中间
  const double n = static_cast<double>(raw) / static_cast<double>(hard_threshold);
  return soft_cost_weight * n * n;
}

}  // namespace pnc_2d
