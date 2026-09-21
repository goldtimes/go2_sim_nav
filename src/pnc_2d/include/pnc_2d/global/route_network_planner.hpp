// 路网路由规划器：把"图上最短路"包装成 GlobalPlanner。
//
// 用法：planner.type = route_network，配合 route_network.routes_file 指向
// routes.yaml。 与 A* 的分工（见 doc/pnc2d_restructure_plan.md §6 P2）：
//   · A*           = 自由空间最短路（想去哪就去哪）
//   · route_network = 沿人为画好的通道走（园区/仓库的秩序）
//   · goal_mode=hybrid 时两者**组合**：主干道用路网，最后一段用 A* 补到目标。
//
// 关键约定：
//   1. **禁止视线剪枝**：通道是人为画的，line_of_sight
//   剪枝会切角、破坏原设计的绕行
//      → 本类强制 prune_mode = "collinear"（只合并共线点）。
//   2. 单行（one_way）在路由中严格生效：反向必须绕行，不允许"抄近道逆行"。
//   3. 载入地图后逐条通道做 footprint 可行性校验（车体真能过），
//      reject_infeasible=true 时不可行通道不参与路由（避免"画得漂亮过不去"）。

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pnc_2d/core/global_planner.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "pnc_2d/global/astar_planner.hpp"

namespace pnc_2d {

class RouteNetworkPlanner : public GlobalPlanner {
public:
  std::string type() const override { return "route_network"; }
  bool configure(const ParamReader &params) override;
  PlanResult plan(const PlanRequest &req) override;
  void reset() override;

  /// 载入/重载路网（节点启动时用 route_network.routes_file，将来可由服务触发）
  bool reloadGraph(const std::string &path, std::string &err);

  // ---- 给节点做可视化/诊断用 ----
  const RouteGraph *routeGraph() const override {
    return graph_valid_ ? &graph_ : nullptr;
  }
  std::vector<int> activeRouteEdges() const override {
    return last_route_edges_;
  }
  bool graphValid() const { return graph_valid_; }
  /// 上一次规划用到的通道索引（用于 RViz 高亮当前通路）
  const std::vector<int> &lastRouteEdges() const { return last_route_edges_; }
  /// 通道可行性统计（在首次规划时算一次）：返回 <可行数, 不可行数>
  std::pair<std::size_t, std::size_t> feasibilityCounts() const;

private:
  bool ensureFeasibility();
  PlanResult planOnGraph(const PlanRequest &req);

  RouteGraph graph_;
  bool graph_valid_{false};
  std::string routes_file_;
  std::string graph_error_; ///< 载入失败原因（plan() 里报给用户）

  std::string goal_mode_{"hybrid"}; ///< hybrid | strict
  double max_entry_distance_{10.0}; ///< 起终点离路网的最大允许距离 [m]
  bool reject_infeasible_{true};    ///< 不可行通道是否禁止参与路由

  bool feasibility_done_{false};
  std::size_t feasible_count_{0};
  std::size_t infeasible_count_{0};
  std::vector<int> last_route_edges_;

  /// hybrid 模式下用来补"最后一段/第一段"自由空间
  AStarPlanner fallback_astar_;
};

} // namespace pnc_2d
