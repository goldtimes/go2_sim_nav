#include "pnc_2d/core/factory.hpp"

#include "pnc_2d/global/astar_planner.hpp"
#include "pnc_2d/global/route_network_planner.hpp"

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

} // namespace pnc_2d
