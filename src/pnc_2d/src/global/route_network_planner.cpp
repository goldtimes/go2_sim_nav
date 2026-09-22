#include "pnc_2d/global/route_network_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace pnc_2d {
namespace {

double dist2d(double ax, double ay, double bx, double by) {
  return std::hypot(bx - ax, by - ay);
}

/// 追加点，跳过与末点重复的点（折线拼接处必然重复）
void pushUnique(std::vector<Pose2D> &out, const Pose2D &p) {
  if (!out.empty()) {
    const Pose2D &b = out.back();
    if (std::hypot(p.x - b.x, p.y - b.y) < 1e-6)
      return;
  }
  out.push_back(p);
}

} // namespace

// ---------------------------------------------------------------------------
// 参数 / 载入
// ---------------------------------------------------------------------------

bool RouteNetworkPlanner::configure(const ParamReader &params) {
  if (!loadCommonParams(params, "route_network"))
    return false;

  // ⚠ 路网禁止视线剪枝：通道是人为画的，切角会破坏原设计的绕行
  prune_mode_ = "collinear";

  routes_file_ = params.getString("route_network.routes_file", routes_file_);
  goal_mode_ = params.getString("route_network.goal_mode", goal_mode_);
  max_entry_distance_ =
      params.getDouble("route_network.max_entry_distance", max_entry_distance_);
  reject_infeasible_ =
      params.getBool("route_network.reject_infeasible", reject_infeasible_);

  if (goal_mode_ != "hybrid" && goal_mode_ != "strict")
    goal_mode_ = "hybrid";
  if (max_entry_distance_ < 0.0)
    max_entry_distance_ = 0.0;

  // 兜底 A*（hybrid 模式下补"第一段/最后一段"自由空间）用同一份 astar.* 参数
  fallback_astar_.configure(params);

  if (!routes_file_.empty()) {
    std::string err;
    reloadGraph(routes_file_,
                err); // 失败不阻止 configure：留到 plan() 里报明确错误
  }
  return true;
}

bool RouteNetworkPlanner::reloadGraph(const std::string &path,
                                      std::string &err) {
  graph_valid_ = false;
  feasibility_done_ = false;
  feasible_count_ = infeasible_count_ = 0;
  last_route_edges_.clear();
  routes_file_ = path;

  if (path.empty()) {
    err = "routes_file 为空";
    graph_error_ = err;
    return false;
  }
  if (!graph_.loadFromFile(path, err)) {
    graph_error_ = err;
    return false;
  }
  graph_valid_ = true;
  graph_error_.clear();
  return true;
}

void RouteNetworkPlanner::reset() {
  feasibility_done_ = false;
  last_route_edges_.clear();
  if (map_)
    fallback_astar_.setCostMap(map_);
}

std::pair<std::size_t, std::size_t>
RouteNetworkPlanner::feasibilityCounts() const {
  return {feasible_count_, infeasible_count_};
}

// ---------------------------------------------------------------------------
// 可行性校验：车体沿每条通道真的通得过吗
// ---------------------------------------------------------------------------

bool RouteNetworkPlanner::ensureFeasibility() {
  if (feasibility_done_)
    return true;
  if (!map_ || !map_->valid() || !graph_valid_)
    return false;

  feasible_count_ = infeasible_count_ = 0;
  std::vector<RouteEdge> &edges = graph_.mutableEdges();
  for (RouteEdge &e : edges) {
    bool ok = true;
    for (std::size_t i = 0; i + 1 < e.polyline.size() && ok; ++i) {
      // 与 A* 边扫掠同一套判据：朝向取该段方向，查矩形车体（含
      // margin）是否碰硬障碍
      if (!lineIsCollisionFree(e.polyline[i], e.polyline[i + 1]))
        ok = false;
    }
    e.feasible = ok;
    if (ok) {
      ++feasible_count_;
    } else {
      ++infeasible_count_;
    }
  }
  feasibility_done_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// 规划
// ---------------------------------------------------------------------------

PlanResult RouteNetworkPlanner::plan(const PlanRequest &req) {
  const auto t0 = now();
  PlanResult res;

  if (!map_ || !map_->valid()) {
    res.status = PlannerStatus::kNotInitialized;
    res.message = "地图未设置或无效";
    res.stats.plan_time_ms = msSince(t0);
    return res;
  }
  if (!graph_valid_) {
    res.status = PlannerStatus::kNotInitialized;
    res.message = graph_error_.empty()
                      ? "路网未载入（route_network.routes_file 未配置？）"
                      : ("路网载入失败：" + graph_error_);
    res.stats.plan_time_ms = msSince(t0);
    return res;
  }
  if (!std::isfinite(req.start.x) || !std::isfinite(req.start.y) ||
      !std::isfinite(req.goal.x) || !std::isfinite(req.goal.y)) {
    res.status = PlannerStatus::kInvalidInput;
    res.message = "起点/终点坐标含 NaN 或 Inf";
    res.stats.plan_time_ms = msSince(t0);
    return res;
  }

  ensureFeasibility();
  collision_.resetCounters();

  PlanResult r = planOnGraph(req);
  r.stats.plan_time_ms = msSince(t0);
  r.stats.path_length = pathLength(r.path);
  r.stats.footprint_full_checks = collision_.fullChecks();
  return r;
}

PlanResult RouteNetworkPlanner::planOnGraph(const PlanRequest &req) {
  PlanResult res;
  last_route_edges_.clear();

  const RouteProjection ps = graph_.project(req.start.x, req.start.y);
  const RouteProjection pg = graph_.project(req.goal.x, req.goal.y);
  if (!ps.valid || !pg.valid) {
    res.status = PlannerStatus::kInvalidInput;
    res.message = "路网里没有可用通道，无法投影起点/终点";
    return res;
  }
  if (max_entry_distance_ > 0.0 &&
      (ps.dist > max_entry_distance_ || pg.dist > max_entry_distance_)) {
    char buf[256];
    std::snprintf(
        buf, sizeof(buf),
        "起点离路网 %.2f m / 终点离路网 %.2f m，超过 max_entry_distance %.2f m"
        "（地图与路网是否同源？）",
        ps.dist, pg.dist, max_entry_distance_);
    res.status = PlannerStatus::kInvalidInput;
    res.message = buf;
    return res;
  }

  const std::vector<RouteEdge> &edges = graph_.edges();
  const int N = static_cast<int>(graph_.nodes().size());
  const int S = N;     // 起点临时节点（投影点）
  const int G = N + 1; // 终点临时节点（投影点）

  struct Arc {
    int to;
    double cost;
    int edge;
    double s_from;
    double s_to;
  };
  std::vector<std::vector<Arc>> adj(static_cast<std::size_t>(N) + 2);

  auto addArc = [&](int u, int v, int ei, double s0, double s1) {
    const double len = std::abs(s1 - s0);
    if (u == v || len < 1e-9)
      return;
    const RouteEdge &e = edges[static_cast<std::size_t>(ei)];
    // 代价单位取"秒"：走得越慢的通道代价越大（等价于偏好快通道/短通道）
    const double cost = len / std::max(0.05, e.speed_limit);
    adj[static_cast<std::size_t>(u)].push_back(Arc{v, cost, ei, s0, s1});
  };

  // 1) 把起点/终点所在通道在投影处"切开"，让搜索可以从通道中间进出
  {
    const RouteEdge &es = edges[static_cast<std::size_t>(ps.edge)];
    addArc(S, es.to, ps.edge, ps.s, es.length); // 沿正向走到该通道终点
    addArc(es.from, S, ps.edge, 0.0, ps.s);     // 从通道起点正向走到我
    if (!es.one_way) {
      addArc(S, es.from, ps.edge, ps.s, 0.0);     // 反向：回到通道起点
      addArc(es.to, S, ps.edge, es.length, ps.s); // 反向：从通道终点走回来
    }
    const RouteEdge &eg = edges[static_cast<std::size_t>(pg.edge)];
    addArc(eg.from, G, pg.edge, 0.0, pg.s);
    addArc(G, eg.to, pg.edge, pg.s, eg.length);
    if (!eg.one_way) {
      addArc(eg.to, G, pg.edge, eg.length, pg.s);
      addArc(G, eg.from, pg.edge, pg.s, 0.0);
    }
  }

  // 2) 原始通道（严格按单行方向）
  for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei) {
    const RouteEdge &e = edges[static_cast<std::size_t>(ei)];
    addArc(e.from, e.to, ei, 0.0, e.length);
    if (!e.one_way)
      addArc(e.to, e.from, ei, e.length, 0.0);
  }

  // 3) 起终点落在同一条通道上：直连（单行且目标在上游时不加，逼它绕一圈）
  const bool same_edge = (ps.edge == pg.edge);
  const bool trivial = same_edge && std::abs(pg.s - ps.s) < 1e-6;
  if (same_edge && !trivial) {
    const RouteEdge &e = edges[static_cast<std::size_t>(ps.edge)];
    if (pg.s > ps.s || !e.one_way)
      addArc(S, G, ps.edge, ps.s, pg.s);
  }

  // 4) Dijkstra（S → G）
  int expanded = 0;
  auto runDijkstra = [&](bool allow_infeasible, std::vector<int> &prev_node,
                         std::vector<int> &prev_arc) -> bool {
    const double INF = std::numeric_limits<double>::infinity();
    const std::size_t n = adj.size();
    std::vector<double> dist(n, INF);
    std::vector<char> done(n, 0);
    prev_node.assign(n, -1);
    prev_arc.assign(n, -1);
    using QI = std::pair<double, int>;
    std::priority_queue<QI, std::vector<QI>, std::greater<QI>> pq;
    dist[static_cast<std::size_t>(S)] = 0.0;
    pq.push({0.0, S});
    expanded = 0;
    while (!pq.empty()) {
      const auto top = pq.top();
      pq.pop();
      const int u = top.second;
      if (done[static_cast<std::size_t>(u)])
        continue;
      done[static_cast<std::size_t>(u)] = 1;
      ++expanded;
      if (u == G)
        break;
      const auto &out = adj[static_cast<std::size_t>(u)];
      for (std::size_t k = 0; k < out.size(); ++k) {
        const Arc &a = out[k];
        if (!allow_infeasible && reject_infeasible_ &&
            !edges[static_cast<std::size_t>(a.edge)].feasible) {
          continue; // 车体过不去的通道不参与路由
        }
        const double nd = dist[static_cast<std::size_t>(u)] + a.cost;
        if (nd < dist[static_cast<std::size_t>(a.to)] - 1e-12) {
          dist[static_cast<std::size_t>(a.to)] = nd;
          prev_node[static_cast<std::size_t>(a.to)] = u;
          prev_arc[static_cast<std::size_t>(a.to)] = static_cast<int>(k);
          pq.push({nd, a.to});
        }
      }
    }
    return std::isfinite(dist[static_cast<std::size_t>(G)]);
  };

  // 诊断用：把"边序号"翻译成"A->B"，不可行通道点名 —— 只报数量对排查没用
  auto edgeName = [&](int ei) -> std::string {
    const RouteEdge &e = edges[static_cast<std::size_t>(ei)];
    return graph_.nodes()[static_cast<std::size_t>(e.from)].name + "->" +
           graph_.nodes()[static_cast<std::size_t>(e.to)].name;
  };
  // 回溯一条 Dijkstra 结果所经过的边序列（只用于诊断）
  auto collectEdges = [&](const std::vector<int> &pn,
                          const std::vector<int> &pa) -> std::vector<int> {
    std::vector<int> seq;
    int cur = G;
    while (cur != S) {
      const int p = pn[static_cast<std::size_t>(cur)];
      const int k = pa[static_cast<std::size_t>(cur)];
      if (p < 0 || k < 0)
        break;
      seq.push_back(adj[static_cast<std::size_t>(p)][static_cast<std::size_t>(k)].edge);
      cur = p;
    }
    std::reverse(seq.begin(), seq.end());
    return seq;
  };

  std::vector<int> prev_node, prev_arc;
  bool found = runDijkstra(false, prev_node, prev_arc);
  int attempts = 1;
  std::string note;
  if (!found && reject_infeasible_ && infeasible_count_ > 0) {
    // 再看一眼"如果允许走不可行通道"是否本来就无解，好给出准确的诊断
    attempts = 2;
    std::vector<int> pn2, pa2;
    if (runDijkstra(true, pn2, pa2)) {
      res.status = PlannerStatus::kNoPath;
      // 点名：优先列出"唯一通路必须经过的那几条不可行通道"，比只报数量有用得多
      std::string names;
      for (const int ei : collectEdges(pn2, pa2)) {
        if (edges[static_cast<std::size_t>(ei)].feasible)
          continue;
        names += (names.empty() ? "" : ", ") + edgeName(ei);
      }
      if (names.empty())
        names = "（无法定位到具体通道）";
      res.message =
          "路网内连不通：唯一通路必须经过车体过不去的通道 " + names +
          "（共 " + std::to_string(infeasible_count_) +
          " 条不可行；可把 route_network.reject_infeasible 设为 false 强行走，"
          "或修通道/地图/禁行区膨胀）";
      res.stats.expanded_nodes = expanded;
      res.stats.windows_tried = attempts;
      return res;
    }
  }
  if (!found) {
    res.status = PlannerStatus::kNoPath;
    res.message = "路网内无通路（起终点不连通，或被单行方向限制住）";
    res.stats.expanded_nodes = expanded;
    res.stats.windows_tried = attempts;
    return res;
  }

  // 5) 回溯出"走过的通道序列 + 每段的弧长区间"
  std::vector<int> seq_edge;
  std::vector<double> seq_s0, seq_s1;
  {
    int cur = G;
    while (cur != S) {
      const int p = prev_node[static_cast<std::size_t>(cur)];
      const int k = prev_arc[static_cast<std::size_t>(cur)];
      if (p < 0 || k < 0)
        break;
      const Arc &a =
          adj[static_cast<std::size_t>(p)][static_cast<std::size_t>(k)];
      seq_edge.push_back(a.edge);
      seq_s0.push_back(a.s_from);
      seq_s1.push_back(a.s_to);
      cur = p;
    }
    std::reverse(seq_edge.begin(), seq_edge.end());
    std::reverse(seq_s0.begin(), seq_s0.end());
    std::reverse(seq_s1.begin(), seq_s1.end());
  }

  // 6) 拼几何：通道切片（按折线取子段，保持原画线形状）
  auto appendSlice = [&](std::vector<Pose2D> &out, int ei, double s0,
                         double s1) {
    const RouteEdge &e = edges[static_cast<std::size_t>(ei)];
    const double lo = std::min(s0, s1);
    const double hi = std::max(s0, s1);
    const bool forward = (s1 >= s0);
    std::vector<Pose2D> tmp;
    tmp.push_back(e.pointAt(lo));
    double acc = 0.0;
    for (std::size_t i = 0; i + 1 < e.polyline.size(); ++i) {
      acc += dist2d(e.polyline[i].x, e.polyline[i].y, e.polyline[i + 1].x,
                    e.polyline[i + 1].y);
      if (acc > lo + 1e-9 && acc < hi - 1e-9)
        tmp.push_back(e.polyline[i + 1]);
    }
    tmp.push_back(e.pointAt(hi));
    if (!forward)
      std::reverse(tmp.begin(), tmp.end());
    for (const Pose2D &p : tmp)
      pushUnique(out, p);
  };

  std::vector<Pose2D> path;
  for (std::size_t i = 0; i < seq_edge.size(); ++i) {
    appendSlice(path, seq_edge[i], seq_s0[i], seq_s1[i]);
    last_route_edges_.push_back(seq_edge[i]);
  }
  // 成功但路网里有不可行通道：把"绕开了哪几条"写进 message，让上层日志能看到
  if (infeasible_count_ > 0) {
    std::string names;
    for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei) {
      if (edges[static_cast<std::size_t>(ei)].feasible)
        continue;
      names += (names.empty() ? "" : ", ") + edgeName(ei);
    }
    note += "（路网里有 " + std::to_string(infeasible_count_) +
            " 条车体过不去的通道已绕开：" + names + "）";
  }
  if (path.empty()) { // 起终点几乎重合
    pushUnique(path, ps.valid ? Pose2D{ps.x, ps.y, 0.0, false} : req.start);
    pushUnique(path, Pose2D{pg.x, pg.y, 0.0, false});
  }

  // 7) hybrid：起点真实位置 → 投影点；投影点 → 真实终点（自由空间补段）
  auto addFreeSegment = [&](const Pose2D &a, const Pose2D &b,
                            std::vector<Pose2D> &out) {
    const double d = dist2d(a.x, a.y, b.x, b.y);
    if (d < 0.02)
      return true;
    if (lineIsCollisionFree(a, b)) { // 直线能走就不用起 A*
      pushUnique(out, b);
      return true;
    }
    if (goal_mode_ == "hybrid") {
      const PlanResult r = fallback_astar_.plan(PlanRequest{a, b});
      if (r.ok() && r.path.size() >= 2) {
        for (std::size_t i = 1; i < r.path.size(); ++i)
          pushUnique(out, r.path[i]);
        return true;
      }
      note += "（离网段 A* 失败：" + r.message + "）";
      return false;
    }
    note += "（离网段直线不通，strict 模式下不补）";
    return false;
  };

  std::vector<Pose2D> full;
  if (goal_mode_ == "hybrid") {
    pushUnique(full, req.start);
    addFreeSegment(req.start, Pose2D{ps.x, ps.y, 0.0, false}, full);
    for (const Pose2D &p : path)
      pushUnique(full, p);
    addFreeSegment(Pose2D{pg.x, pg.y, 0.0, false}, req.goal, full);
  } else {
    full = path;
  }

  // 8) 后处理：共线合并 + yaw 填充（首点保留当前朝向、末点取目标朝向）
  postProcessPath(full, req);

  res.status = PlannerStatus::kSuccess;
  {
    std::string msg = "沿路网 " + std::to_string(last_route_edges_.size()) +
                      " 条通道（goal_mode=" + goal_mode_ + "）";
    if (infeasible_count_ > 0)
      msg += "；⚠ 路网里有车体过不去的通道";
    msg += note;
    res.message = msg;
  }
  res.path = std::move(full);
  res.stats.expanded_nodes = expanded;
  res.stats.windows_tried = attempts;
  return res;
}

} // namespace pnc_2d
