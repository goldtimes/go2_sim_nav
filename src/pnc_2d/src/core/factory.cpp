#include "pnc_2d/core/factory.hpp"

#include "pnc_2d/core/replan_recovery.hpp"
#include "pnc_2d/global/astar_planner.hpp"
#include "pnc_2d/global/route_network_planner.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"
#include "pnc_2d/local/null_local_planner.hpp"

namespace pnc_2d {

std::unique_ptr<GlobalPlanner> createPlanner(const std::string &type) {
  if (type == "astar")
    return std::make_unique<AStarPlanner>();
  if (type == "route_network")
    return std::make_unique<RouteNetworkPlanner>();
  // 新增算法就在这里加一行：
  //   if (type == "jps") return std::make_unique<JpsPlanner>();
  //   if (type == "hybrid_astar") return
  //   std::make_unique<HybridAStarPlanner>(); if (type == "rrt_star") return
  //   std::make_unique<RrtStarPlanner>();
  return nullptr;
}

std::vector<std::string> availablePlanners() {
  return {"astar", "route_network"};
}

std::unique_ptr<LocalPlanner> createLocalPlanner(const std::string &type) {
  if (type == "null" || type == "none")
    return std::make_unique<NullLocalPlanner>();
  if (type == "mpc")
    return std::make_unique<MpcLocalPlanner>();
  return nullptr;
}

std::vector<std::string> availableLocalPlanners() {
  // "none": yaml 里更自然的写法，映射到同一个 NullLocalPlanner
  return {"none", "null", "mpc"};
}

std::unique_ptr<RecoveryBehavior>
createRecoveryBehavior(const std::string &type) {
  if (type == "replan")
    return std::make_unique<ReplanRecoveryBehavior>();
  // 其它类型（清图 / 后退 / 旋转…）以后再加：接口已立着（P3 起）。
  return nullptr;
}

std::vector<std::string> availableRecoveries() { return {"replan"}; }

} // namespace pnc_2d
