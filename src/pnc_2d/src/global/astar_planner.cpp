// AStarPlanner 的实现。

#include "pnc_2d/global/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

/// 日志里报长度用：固定两位小数（免得出现 0.242412 这种噪声数字）
std::string number(double v)
{
  if (!std::isfinite(v)) return "n/a";
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}

/// 起点朝向：请求里给了就用它（控制器要先原地对正），没给就用"走向终点"的方向。
double startYawOf(const PlanRequest & req)
{
  const double to_goal =
      std::atan2(req.goal.y - req.start.y, req.goal.x - req.start.x);
  return req.start.has_yaw ? req.start.yaw : to_goal;
}

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
  start_escape_radius_ =
      params.getDouble(px + ".start_escape_radius", start_escape_radius_);
  if (start_escape_radius_ < 0.0) start_escape_radius_ = 0.0;

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
    const double d = collision_.distanceToLethal(req.start.x, req.start.y);
    res.status = PlannerStatus::kStartFootprintCollision;
    res.message = "起点处车体与障碍重叠（中心净距 " + number(d) + " m；需求 " +
                  number(collision_.footprint().inscribedRadius()) + " m" +
                  (virtual_start_enabled_
                       ? "；周边 " + number(start_escape_radius_) +
                             " m 内没找到可通行的救命位姿"
                       : "；虚拟起点已关") +
                  "）";
    return res;
  }
  if (collision_.poseInCollision(req.goal.x, req.goal.y, goal_yaw)) {
    res.status = PlannerStatus::kGoalFootprintCollision;
    const double d = collision_.distanceToLethal(req.goal.x, req.goal.y);
    res.message = "终点处车体轮廓与障碍重叠（净距 " + number(d) + " m）";
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
      const bool hit = collision_.poseInCollision(wx, wy, yaw);
      if (hit) continue;   // 矩形 footprint

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
  res.message = relax_note_.empty() ? std::string("ok") : ("ok | " + relax_note_);
  return res;
}

PlanResult AStarPlanner::plan(const PlanRequest & req)
{
  relax_note_.clear();
  // ==========================================================================
  // 起点已经在“余量带”里 ⇒ **换一个虚拟起点**，并把车当前位置插成首点。
  //
  // 现场：全局图是 map_server 发的（障碍已按 `map_server.inflate` 膨胀、禁行区已按
  // `zones.inflate` 烧入），局部用感知的未膨胀图 + 自己的距离场。两边**图不同源**
  // 加栅格离散化（±半格），“全局刚判过、起点恰好擦进余量”是常态
  // （实测：车离禁行区多边形 0.28 m，图上量到 0.224 m，含余量需求 0.25 ⇒ 差 1 mm）。
  // 起点一律判死 ⇒ 恢复重规划也失败 ⇒ 任务永久 FAILED。
  //
  // ★ 为什么不能“放宽判据”（试过两版，都被实测否掉，见头文件）：
  //   ① 放宽整条路径 ⇒ 一路贴墙走；② 只放宽起点附近 ⇒ 车侧面对着墙时，
  //   栅格 A*（节点朝向 = 运动方向）根本出不去（实测 NO_PATH）。
  // ★ 正确分工：**全局只规划“从合法位姿出发”的路**；把车从那 10 cm 挪出来的活
  //   交给**局部**（MPC 真的会转方向，能做侧向挪出来这种动作）。
  //   所以：在起点周边找一个完整判据可通的**虚拟起点**，从它开始搜；
  //   成功则把**车当前位置**插成路径首点（这一段由局部走出来）。
  // ==========================================================================
  PlanRequest req_eff = req;
  bool use_virtual = false;
  double vs_dist = 0.0;
  if (map_ && map_->valid() && virtual_start_enabled_ &&
      start_escape_radius_ > 0.0 && std::isfinite(req.start.x) &&
      std::isfinite(req.start.y) && std::isfinite(req.goal.x) &&
      std::isfinite(req.goal.y) &&
      collision_.poseInCollision(req.start.x, req.start.y, startYawOf(req)) &&
      // 守卫：只有**真实轮廓**（safe_margin=0）仍放得下时才救 —— 真实轮廓都放不下
      // 说明车已经真的压上去了（或者离障碍只剩不到半车宽），这时应停下来报错交人工，
      // 而不是规划一条“从重叠位置开出去”的路（那样只会擦得更厉害）。
      !collision_.poseInCollisionAtMargin(req.start.x, req.start.y,
                                          startYawOf(req), 0.0)) {
    int scx = 0;
    int scy = 0;
    if (map_->worldToGrid(req.start.x, req.start.y, scx, scy)) {
      const double d0 = collision_.distanceToLethal(req.start.x, req.start.y);
      const int r_cells = static_cast<int>(
          std::ceil(start_escape_radius_ / map_->resolution()));
      double best_d2 = std::numeric_limits<double>::infinity();
      int bx = -1;
      int by = -1;
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          const int nx = scx + dx;
          const int ny = scy + dy;
          if (!map_->inside(nx, ny)) continue;
          if (collision_.cellLethal(nx, ny)) continue;
          double wx = 0.0;
          double wy = 0.0;
          map_->gridToWorld(nx, ny, wx, wy);
          // 必须是**完整判据**可通的位姿，而且要比车现在**更远离障碍**
          if (collision_.poseInCollision(wx, wy, startYawOf(req))) continue;
          if (!(collision_.distanceToLethal(wx, wy) > d0 + 0.005)) continue;
          const double d2 = static_cast<double>(dx) * dx + dy * dy;
          if (d2 < best_d2) {
            best_d2 = d2;
            bx = nx;
            by = ny;
          }
        }
      }
      if (bx >= 0) {
        map_->gridToWorld(bx, by, req_eff.start.x, req_eff.start.y);
        req_eff.start.has_yaw = req.start.has_yaw;
        req_eff.start.yaw = startYawOf(req);
        use_virtual = true;
        vs_dist = std::sqrt(best_d2) * map_->resolution();
        relax_note_ = "⚠ 起点在余量带内（净距 " +
                      number(collision_.distanceToLethal(req.start.x,
                                                         req.start.y)) +
                      " m < 需求 " +
                      number(collision_.footprint().inscribedRadius()) +
                      " m）⇒ 从最近的可通行位姿规划（距车 " + number(vs_dist) +
                      " m），首段由局部把车挪出来。常见原因：运行中新加了禁行区/"
                      "改了地图膨胀，或全局图与感知图差几个 cm";
      }
    }
  }
  PlanResult res = planImpl(req_eff);
  if (use_virtual && res.ok() && !res.path.empty()) {
    // 把车当前位置插成首点：局部会从“车现在的位置”出发跟着这条路走
    Pose2D head = req.start;
    head.has_yaw = true;
    head.yaw = startYawOf(req);
    res.path.insert(res.path.begin(), head);
  }
  if (!relax_note_.empty()) {
    if (res.ok()) {
      res.message = relax_note_;
    } else {
      res.message += " | " + relax_note_;
    }
  }
  return res;
}

PlanResult AStarPlanner::planImpl(const PlanRequest & req)
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
