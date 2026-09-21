// AStarPlanner 的实现。

#include "pnc_2d/global/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_2d {
namespace {

constexpr double kSqrt2 = 1.41421356237309504880;

struct QNode {
  float f;
  int32_t id;
};

struct QNodeGreater {
  bool operator()(const QNode & a, const QNode & b) const { return a.f > b.f; }
};

const int kDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
const int kDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

}  // namespace

bool AStarPlanner::configure(const ParamReader & params)
{
  const std::string px = "astar";
  if (!loadCommonParams(params, px)) return false;

  connectivity_ = params.getInt(px + ".connectivity", connectivity_);
  if (connectivity_ != 4 && connectivity_ != 8) connectivity_ = 8;
  heuristic_weight_ = params.getDouble(px + ".heuristic_weight", heuristic_weight_);
  if (heuristic_weight_ < 1.0) heuristic_weight_ = 1.0;   // <1 会破坏最优性且无收益

  use_search_window_ = params.getBool(px + ".use_search_window", use_search_window_);
  search_window_margin_ = params.getDouble(px + ".search_window_margin", search_window_margin_);
  if (search_window_margin_ < 0.0) search_window_margin_ = 0.0;

  const auto margins = params.getDoubleArray(px + ".search_window_retry_margins", retry_margins_);
  if (!margins.empty()) retry_margins_.assign(margins.begin(), margins.end());
  // 末尾追加"整图"兜底，保证"窗口不够大"最终能退化为全图搜索
  if (retry_margins_.empty() || retry_margins_.back() < 1.0e8) retry_margins_.push_back(1.0e9);
  std::sort(retry_margins_.begin(), retry_margins_.end());

  const long iters = params.getInt(px + ".max_iterations",
                                   static_cast<int>(std::min<long>(max_iterations_, 2000000000L)));
  max_iterations_ = std::max<long>(iters, 1);
  max_search_time_ms_ = params.getDouble(px + ".max_search_time_ms", max_search_time_ms_);
  if (max_search_time_ms_ <= 0.0) max_search_time_ms_ = 1.0e9;

  clearBuffers();
  return true;
}

void AStarPlanner::reset()
{
  clearBuffers();
}

void AStarPlanner::ensureBuffers()
{
  if (!map_ || !map_->valid()) return;
  const int w = map_->width();
  const int h = map_->height();
  if (buf_w_ == w && buf_h_ == h && g_.size() == static_cast<std::size_t>(w) * h) return;
  buf_w_ = w;
  buf_h_ = h;
  const std::size_t n = static_cast<std::size_t>(w) * h;
  closed_.assign(n, 0);
  g_.assign(n, std::numeric_limits<float>::infinity());
  parent_.assign(n, -1);
}

void AStarPlanner::clearBuffers()
{
  buf_w_ = 0;
  buf_h_ = 0;
  closed_.clear();
  g_.clear();
  parent_.clear();
}

float AStarPlanner::heuristic(int x, int y, int gx, int gy) const
{
  const double d = (connectivity_ == 4)
                     ? static_cast<double>(std::abs(x - gx) + std::abs(y - gy))
                     : std::hypot(static_cast<double>(x - gx), static_cast<double>(y - gy));
  return static_cast<float>(heuristic_weight_ * d * map_->resolution());
}

std::vector<Pose2D> AStarPlanner::reconstruct(int goal_id, int start_id) const
{
  std::vector<Pose2D> path;
  const int w = map_->width();
  int id = goal_id;
  while (id >= 0) {
    Pose2D p;
    const int x = id % w;
    const int y = id / w;
    map_->gridToWorld(x, y, p.x, p.y);
    p.has_yaw = false;
    path.push_back(p);
    if (id == start_id) break;
    id = parent_[static_cast<std::size_t>(id)];
  }
  std::reverse(path.begin(), path.end());
  return path;
}

PlanResult AStarPlanner::searchInWindow(const PlanRequest & req, const SearchWindow & win,
                                        std::chrono::steady_clock::time_point t0)
{
  PlanResult res;
  const double res_m = map_->resolution();
  const int w = map_->width();
  auto idOf = [w](int x, int y) { return static_cast<int32_t>(y) * w + x; };

  int sx = 0;
  int sy = 0;
  int gx = 0;
  int gy = 0;
  if (!map_->worldToGrid(req.start.x, req.start.y, sx, sy)) {
    res.status = PlannerStatus::kStartOutOfMap;
    res.message = "起点不在地图内";
    return res;
  }
  if (!map_->worldToGrid(req.goal.x, req.goal.y, gx, gy)) {
    res.status = PlannerStatus::kGoalOutOfMap;
    res.message = "终点不在地图内";
    return res;
  }
  if (!win.contains(sx, sy) || !win.contains(gx, gy)) {
    res.status = PlannerStatus::kNoPath;   // 可重试：窗口不够大
    res.message = "起点/终点不在当前搜索窗口内";
    return res;
  }

  // 朝向：起点用请求给的真实朝向（控制器要先原地对正）；没给就用"走向终点"的方向。
  // 终点用目标 yaw；没给就用"从起点过来的方向"。
  const double to_goal = std::atan2(req.goal.y - req.start.y, req.goal.x - req.start.x);
  const double start_yaw = req.start.has_yaw ? req.start.yaw : to_goal;
  const double goal_yaw = req.goal.has_yaw ? req.goal.yaw : to_goal;

  if (collision_.cellLethal(sx, sy)) {
    res.status = PlannerStatus::kStartOccupied;
    res.message = "起点格是硬障碍";
    return res;
  }
  if (collision_.cellLethal(gx, gy)) {
    res.status = PlannerStatus::kGoalOccupied;
    res.message = "终点格是硬障碍";
    return res;
  }
  if (collision_.poseInCollision(req.start.x, req.start.y, start_yaw)) {
    res.status = PlannerStatus::kStartFootprintCollision;
    res.message = "起点处车体轮廓与障碍重叠";
    return res;
  }
  if (collision_.poseInCollision(req.goal.x, req.goal.y, goal_yaw)) {
    res.status = PlannerStatus::kGoalFootprintCollision;
    res.message = "终点处车体轮廓与障碍重叠";
    return res;
  }

  const int32_t sid = idOf(sx, sy);
  const int32_t gid = idOf(gx, gy);

  // 起点=终点：直接返回单点路径（调用方要求"就在这里"）
  if (sid == gid) {
    Pose2D p = req.goal;
    p.has_yaw = true;
    res.status = PlannerStatus::kSuccess;
    res.path.assign(1, p);
    res.message = "起点与终点同格";
    return res;
  }

  ensureBuffers();
  std::fill(closed_.begin(), closed_.end(), 0);
  std::fill(g_.begin(), g_.end(), std::numeric_limits<float>::infinity());
  std::fill(parent_.begin(), parent_.end(), -1);

  std::priority_queue<QNode, std::vector<QNode>, QNodeGreater> open;
  g_[static_cast<std::size_t>(sid)] = 0.0F;
  parent_[static_cast<std::size_t>(sid)] = -1;
  open.push(QNode{heuristic(sx, sy, gx, gy), sid});

  long expanded = 0;
  long discovered = 1;
  long max_open = 1;
  bool found = false;
  const int n_dirs = (connectivity_ == 4) ? 8 : 8;   // 4 邻域时只取偶数下标
  const bool use_4 = (connectivity_ == 4);

  while (!open.empty()) {
    const QNode top = open.top();
    open.pop();
    const int32_t cid = top.id;
    if (closed_[static_cast<std::size_t>(cid)]) continue;   // 惰性删除
    closed_[static_cast<std::size_t>(cid)] = 1;
    ++expanded;

    if (cid == gid) {
      found = true;
      break;
    }
    if (expanded > max_iterations_) {
      res.status = PlannerStatus::kMaxIterations;
      res.message = "扩展节点数超过上限";
      res.stats.expanded_nodes = expanded;
      return res;
    }
    if ((expanded & 1023) == 0 && msSince(t0) > max_search_time_ms_) {
      res.status = PlannerStatus::kTimeout;
      res.message = "搜索时间超过上限";
      res.stats.expanded_nodes = expanded;
      return res;
    }

    const int cx = cid % w;
    const int cy = cid / w;
    const double g_cur = g_[static_cast<std::size_t>(cid)];

    for (int k = (use_4 ? 0 : 0); k < n_dirs; ++k) {
      if (use_4 && (k % 2) != 0) continue;    // 4 邻域：只保留 0°/90°/180°/270°
      const int nx = cx + kDx8[k];
      const int ny = cy + kDy8[k];
      if (!win.contains(nx, ny)) continue;

      const bool diagonal = (kDx8[k] != 0 && kDy8[k] != 0);
      // 斜穿保护：不允许从两个对角障碍之间的缝隙钻过去
      if (diagonal && (collision_.cellLethal(cx + kDx8[k], cy) ||
                       collision_.cellLethal(cx, cy + kDy8[k])))
      {
        continue;
      }

      const int32_t nid = idOf(nx, ny);
      const double yaw = std::atan2(static_cast<double>(kDy8[k]), static_cast<double>(kDx8[k]));
      double wx = 0.0;
      double wy = 0.0;
      map_->gridToWorld(nx, ny, wx, wy);
      if (collision_.poseInCollision(wx, wy, yaw)) continue;   // 矩形 footprint

      const double step = diagonal ? res_m * kSqrt2 : res_m;
      const double extra = cellExtraCost(nx, ny);
      if (!std::isfinite(extra)) continue;
      const float tentative = static_cast<float>(g_cur + step * (1.0 + extra));

      auto & g_n = g_[static_cast<std::size_t>(nid)];
      if (tentative < g_n) {
        if (std::isinf(g_n)) ++discovered;
        g_n = tentative;
        parent_[static_cast<std::size_t>(nid)] = cid;
        open.push(QNode{tentative + heuristic(nx, ny, gx, gy), nid});
        if (static_cast<long>(open.size()) > max_open) max_open = static_cast<long>(open.size());
      }
    }
  }

  res.stats.expanded_nodes = expanded;
  res.stats.discovered_nodes = discovered;
  res.stats.max_open_set = max_open;

  if (!found) {
    res.status = PlannerStatus::kNoPath;
    res.message = "窗口内无可行路径";
    return res;
  }

  res.path = reconstruct(gid, sid);
  if (res.path.size() < 2) {
    res.status = PlannerStatus::kNoPath;
    res.message = "路径重建失败";
    res.path.clear();
    return res;
  }
  postProcessPath(res.path, req);
  res.status = PlannerStatus::kSuccess;
  res.message = "ok";
  return res;
}

PlanResult AStarPlanner::plan(const PlanRequest & req)
{
  const auto t0 = now();
  PlanResult res;

  if (!map_ || !map_->valid()) {
    res.status = PlannerStatus::kNotInitialized;
    res.message = "地图未设置或无效";
    res.stats.plan_time_ms = msSince(t0);
    return res;
  }
  if (!std::isfinite(req.start.x) || !std::isfinite(req.start.y) ||
      !std::isfinite(req.goal.x) || !std::isfinite(req.goal.y))
  {
    res.status = PlannerStatus::kInvalidInput;
    res.message = "起点/终点坐标含 NaN 或 Inf";
    res.stats.plan_time_ms = msSince(t0);
    return res;
  }

  collision_.resetCounters();

  std::vector<double> margins;
  if (use_search_window_) {
    margins.push_back(search_window_margin_);
    for (const double m : retry_margins_) margins.push_back(m);
  } else {
    margins.push_back(1.0e9);   // 整图
  }

  int tried = 0;
  PlanResult last;
  for (std::size_t i = 0; i < margins.size(); ++i) {
    ++tried;
    const SearchWindow win = makeSearchWindow(req, margins[i]);
    PlanResult r = searchInWindow(req, win, t0);
    if (r.ok()) {
      r.stats.windows_tried = tried;
      r.stats.path_length = pathLength(r.path);
      r.stats.plan_time_ms = msSince(t0);
      r.stats.footprint_full_checks = collision_.fullChecks();
      return r;
    }
    last = r;
    // 只有"窗口不够大"才值得扩大重试；起点/终点/超时这类重试也没用
    if (r.status != PlannerStatus::kNoPath) break;
    if (margins[i] >= 1.0e8) break;   // 已经是整图
  }

  last.stats.windows_tried = tried;
  last.stats.plan_time_ms = msSince(t0);
  last.stats.footprint_full_checks = collision_.fullChecks();
  return last;
}

}  // namespace pnc_2d
