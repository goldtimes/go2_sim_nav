#include "pnc_2d/local/rolling_replan_shim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"

namespace pnc_2d {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double dist2(const Pose2D &a, double x, double y) {
  const double dx = a.x - x, dy = a.y - y;
  return dx * dx + dy * dy;
}

/// 两个世界点之间的朝向
double yawBetween(double x0, double y0, double x1, double y1) {
  return std::atan2(y1 - y0, x1 - x0);
}

/// 沿 a→b 按固定间距插入中间点（首尾保留）
void densify(const Pose2D &a, const Pose2D &b, double step,
             std::vector<Pose2D> &out) {
  const double L = std::hypot(b.x - a.x, b.y - a.y);
  if (L < 1e-6)
    return;
  const int n = std::max(1, static_cast<int>(std::ceil(L / std::max(0.02, step))));
  const double yaw = yawBetween(a.x, a.y, b.x, b.y);
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    Pose2D q;
    q.x = a.x + (b.x - a.x) * t;
    q.y = a.y + (b.y - a.y) * t;
    q.yaw = yaw;
    q.has_yaw = true;
    out.push_back(q);
  }
}

/// A* 的一个节点
struct Node {
  int x{0}, y{0};
  double g{kInf};
  double f{kInf};
  int parent{-1};
};

} // namespace

RollingReplanShimPlanner::RollingReplanShimPlanner() = default;

// ============================================================================
// 参数
// ============================================================================
bool RollingReplanShimPlanner::configure(const ParamReader &params) {
  const std::string pre = std::string("rolling.");
  auto s = [&](const char *k, const std::string &d) {
    return params.getString(pre + k, d);
  };
  auto b = [&](const char *k, bool d) { return params.getBool(pre + k, d); };
  auto d = [&](const char *k, double v) { return params.getDouble(pre + k, v); };
  auto i = [&](const char *k, int v) { return params.getInt(pre + k, v); };

  p_.primary = s("primary", p_.primary);
  p_.enable = b("enable", p_.enable);
  p_.lookahead_m = d("lookahead_m", p_.lookahead_m);
  p_.trigger_clearance_m = d("trigger_clearance_m", p_.trigger_clearance_m);
  p_.blocked_clearance_m = d("blocked_clearance_m", p_.blocked_clearance_m);
  p_.path_margin_m = d("path_margin_m", p_.path_margin_m);
  p_.check_step_m = d("check_step_m", p_.check_step_m);
  p_.back_m = d("back_m", p_.back_m);
  p_.roll_m = d("roll_m", p_.roll_m);
  p_.max_roll_m = d("max_roll_m", p_.max_roll_m);
  p_.max_offset_m = d("max_offset_m", p_.max_offset_m);
  p_.min_interval_s = d("min_interval_s", p_.min_interval_s);
  p_.min_progress_m = d("min_progress_m", p_.min_progress_m);
  p_.max_consecutive_fail = i("max_consecutive_fail", p_.max_consecutive_fail);
  p_.give_up_s = d("give_up_s", p_.give_up_s);
  p_.obs_cost_weight = d("obs_cost_weight", p_.obs_cost_weight);
  p_.unknown_cost_scale = d("unknown_cost_scale", p_.unknown_cost_scale);
  p_.lateral_cost_weight = d("lateral_cost_weight", p_.lateral_cost_weight);
  p_.smoothing = b("smoothing", p_.smoothing);
  // 与上层同源的几何/口径（键名与 global_planner / 局部节点一致）
  p_.unknown_as_occupied = b("unknown_as_occupied", p_.unknown_as_occupied);
  p_.hard_threshold = i("hard_threshold", p_.hard_threshold);

  auto clamp_min = [](double &v, double lo) {
    if (v < lo)
      v = lo;
  };
  clamp_min(p_.lookahead_m, 0.30);
  clamp_min(p_.trigger_clearance_m, 0.0);
  clamp_min(p_.blocked_clearance_m, 0.0);
  clamp_min(p_.path_margin_m, 0.0);
  // 触发阈不得比硬下界还紧（否则"被挡"的定义比 MPC 的不可行条件还苛刻）
  if (p_.trigger_clearance_m < p_.blocked_clearance_m)
    p_.trigger_clearance_m = p_.blocked_clearance_m;
  clamp_min(p_.check_step_m, 0.02);
  clamp_min(p_.back_m, 0.0);
  clamp_min(p_.roll_m, 0.10);
  clamp_min(p_.max_roll_m, p_.roll_m);
  clamp_min(p_.max_offset_m, 0.10);
  clamp_min(p_.min_interval_s, 0.0);
  clamp_min(p_.min_progress_m, 0.0);
  clamp_min(p_.give_up_s, 0.0);
  if (p_.max_consecutive_fail < 0)
    p_.max_consecutive_fail = 0;

  // 与全局规划器同一套车体轮廓（`footprint.*`）
  fp_.enable = params.getBool("footprint.enable", fp_.enable);
  fp_.length = params.getDouble("footprint.length", fp_.length);
  fp_.width = params.getDouble("footprint.width", fp_.width);
  fp_.offset_x = params.getDouble("footprint.offset_x", fp_.offset_x);
  fp_.offset_y = params.getDouble("footprint.offset_y", fp_.offset_y);
  fp_.safe_margin = params.getDouble("footprint.safe_margin", fp_.safe_margin);
  fp_.check_edges = params.getBool("footprint.check_edges", fp_.check_edges);
  fp_.fast_path = params.getBool("footprint.fast_path", fp_.fast_path);
  coll_.configure(fp_, p_.hard_threshold, p_.unknown_as_occupied);

  // 主控制器：由工厂建（默认 heading_shim → 里面还有 mpc）
  primary_ = createLocalPlanner(p_.primary);
  if (!primary_) {
    return false; // 启动期就该失败：配了不认识的名字
  }
  if (!primary_->configure(params)) {
    return false;
  }
  // 与 MPC 的软/硬净距对齐提示：触发阈晚于软净距就会"贴上了才绕"（实测 S1 就是
  // 车心→盒面 0.15 m 才开始）
  hard_distance_hint_ = params.getDouble("local_mpc.obstacle_hard_distance",
                                         hard_distance_hint_);
  const double safe_hint =
      params.getDouble("local_mpc.obstacle_safe_distance", -1.0);
  if (safe_hint > 0.0 && p_.trigger_clearance_m + 0.05 < safe_hint) {
    fprintf(stderr,
            "[rolling_replan] ⚠ rolling.trigger_clearance_m=%.2f 比 "
            "local_mpc.obstacle_safe_distance=%.2f 还激进：会等到很贴才绕行"
            "（建议 ≥ 软净距）\n",
            p_.trigger_clearance_m, safe_hint);
  }
  if (hard_distance_hint_ > 0.0 &&
      p_.blocked_clearance_m + 0.02 < hard_distance_hint_) {
    fprintf(stderr,
            "[rolling_replan] ⚠ rolling.blocked_clearance_m=%.3f 比 "
            "local_mpc.obstacle_hard_distance=%.3f 还小（硬下界之下 MPC 必然无解）\n",
            p_.blocked_clearance_m, hard_distance_hint_);
  }
  return true;
}

// ============================================================================
// 输入
// ============================================================================
void RollingReplanShimPlanner::rebuildArclen() {
  arclen_.assign(plan_.size(), 0.0);
  for (std::size_t i = 1; i < plan_.size(); ++i) {
    arclen_[i] = arclen_[i - 1] +
                 std::hypot(plan_[i].x - plan_[i - 1].x,
                            plan_[i].y - plan_[i - 1].y);
  }
}

void RollingReplanShimPlanner::applyPlanToPrimary() {
  rebuildArclen();
  if (primary_)
    primary_->setGlobalPlan(plan_);
}

void RollingReplanShimPlanner::setGlobalPlan(const std::vector<Pose2D> &path) {
  // 新路径 = 新任务/新一次全局规划 ⇒ 修补记录清零（但累计统计保留，便于观察）
  original_ = path;
  plan_ = path;
  stats_.consecutive_fail = 0;
  have_last_repair_ = false;
  giving_up_ = false;
  warned_corridor_ = false;
  applyPlanToPrimary();
}

void RollingReplanShimPlanner::setCorridor(const RouteCorridor *c) {
  corridor_ = c;
  if (primary_)
    primary_->setCorridor(c);
}
void RollingReplanShimPlanner::setSpeedLimit(double v) {
  speed_limit_ = v;
  if (primary_)
    primary_->setSpeedLimit(v);
}
void RollingReplanShimPlanner::setZoneSpeedLimit(double v) {
  LocalPlanner::setZoneSpeedLimit(v);
  if (primary_)
    primary_->setZoneSpeedLimit(v);
}
void RollingReplanShimPlanner::setCostMap(
    std::shared_ptr<const CostMap2D> m) {
  local_map_ = m;
  coll_.setMap(m.get());
  if (primary_)
    primary_->setCostMap(m);
}
void RollingReplanShimPlanner::setDistanceField(const LocalDistanceField *f) {
  dist_field_ = f;
  if (primary_)
    primary_->setDistanceField(f);
}
void RollingReplanShimPlanner::setReferenceProfile(const ReferenceProfile *p) {
  if (primary_)
    primary_->setReferenceProfile(p);
}
void RollingReplanShimPlanner::setDynamicObstacles(
    const std::vector<DynamicObstacle> &o) {
  dynamic_obs_ = o;
  if (primary_)
    primary_->setDynamicObstacles(o);
}
void RollingReplanShimPlanner::setCurrentVelocity(double v, double w) {
  LocalPlanner::setCurrentVelocity(v, w);
  if (primary_)
    primary_->setCurrentVelocity(v, w);
}

// ============================================================================
// 投影 / 采样 / 阻塞判定
// ============================================================================
double RollingReplanShimPlanner::projectS(const Pose2D &pose, std::size_t &idx,
                                         double &lat) const {
  double best = kInf;
  idx = 0;
  lat = 0.0;
  for (std::size_t i = 0; i < plan_.size(); ++i) {
    const double dd = dist2(plan_[i], pose.x, pose.y);
    if (dd < best) {
      best = dd;
      idx = i;
    }
  }
  lat = std::sqrt(std::max(0.0, best));
  return arclen_.empty() ? 0.0 : arclen_[std::min(idx, arclen_.size() - 1)];
}

bool RollingReplanShimPlanner::sampleAt(double s, Pose2D &out) const {
  if (plan_.size() < 2 || arclen_.size() != plan_.size())
    return false;
  if (s <= 0.0) {
    out = plan_.front();
    return true;
  }
  if (s >= arclen_.back()) {
    out = plan_.back();
    return true;
  }
  // 线性查找（路径点通常几百个；每周期的调用次数 = 前瞻点数，足够便宜）
  std::size_t i = 1;
  while (i < arclen_.size() && arclen_[i] < s)
    ++i;
  const double s0 = arclen_[i - 1], s1 = arclen_[i];
  const double t = (s1 - s0) > 1e-9 ? (s - s0) / (s1 - s0) : 0.0;
  out.x = plan_[i - 1].x + (plan_[i].x - plan_[i - 1].x) * t;
  out.y = plan_[i - 1].y + (plan_[i].y - plan_[i - 1].y) * t;
  out.yaw = yawBetween(plan_[i - 1].x, plan_[i - 1].y, plan_[i].x, plan_[i].y);
  out.has_yaw = true;
  return true;
}

bool RollingReplanShimPlanner::blockedAt(const Pose2D &pt, double thr,
                                        double &clearance) const {
  clearance = kInf;
  if (dist_field_ && dist_field_->valid()) {
    clearance = dist_field_->distance(pt.x, pt.y);
    if (clearance < thr)
      return true;
  }
  if (local_map_ && local_map_->valid() && coll_.enabled()) {
    // 轮廓判定：即使是"净距够但轮廓撞"的窄缝也要能识别（比如柱子在侧面）
    if (coll_.poseInCollision(pt.x, pt.y, pt.has_yaw ? pt.yaw : 0.0)) {
      clearance = std::min(clearance, 0.0);
      return true;
    }
  }
  return false;
}

double RollingReplanShimPlanner::polylineLen(const std::vector<Pose2D> &p) {
  double L = 0.0;
  for (std::size_t i = 1; i < p.size(); ++i)
    L += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
  return L;
}

double RollingReplanShimPlanner::maxLateral(const std::vector<Pose2D> &p,
                                            const Pose2D &ori,
                                            double tangent_yaw) {
  // 原路径切线轴 t̂；点到过 ori 的 t̂ 直线的垂直距离
  const double tx = std::cos(tangent_yaw), ty = std::sin(tangent_yaw);
  double m = 0.0;
  for (const auto &q : p) {
    const double dx = q.x - ori.x, dy = q.y - ori.y;
    const double cross = dx * ty - dy * tx; // |d × t̂|
    m = std::max(m, std::fabs(cross));
  }
  return m;
}

// ============================================================================
// 局部 A*（只向前；未知可通行；未知/近障碍/偏离都给代价）
// ============================================================================
bool RollingReplanShimPlanner::localSearchForward(const Pose2D &from,
                                                 const Pose2D &to,
                                                 std::vector<Pose2D> &out,
                                                 std::string &why) {
  out.clear();
  if (!local_map_ || !local_map_->valid()) {
    why = "没有可用的局部图（局部 A* 无法判定）";
    return false;
  }
  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (!local_map_->worldToGrid(from.x, from.y, sx, sy) ||
      !local_map_->worldToGrid(to.x, to.y, gx, gy)) {
    why = "起点/终点不在局部图内";
    return false;
  }
  if (coll_.poseInCollision(from.x, from.y, from.has_yaw ? from.yaw : 0.0)) {
    why = "修补起点处车体轮廓已在障碍里（车已经被顶进去了，绕行无从下手）";
    return false;
  }
  const double res = local_map_->resolution();
  const double t_yaw = yawBetween(from.x, from.y, to.x, to.y);
  const double tx = std::cos(t_yaw), ty = std::sin(t_yaw);

  const int W = local_map_->width(), H = local_map_->height();
  std::vector<Node> nodes(static_cast<std::size_t>(W) * H);
  for (int y = 0; y < H; ++y)
    for (int x = 0; x < W; ++x) {
      Node &n = nodes[static_cast<std::size_t>(y) * W + x];
      n.x = x;
      n.y = y;
    }
  auto at = [&](int x, int y) -> Node & {
    return nodes[static_cast<std::size_t>(y) * W + x];
  };
  auto h = [&](int x, int y) {
    double wx = 0.0, wy = 0.0;
    local_map_->gridToWorld(x, y, wx, wy);
    return std::hypot(wx - to.x, wy - to.y);
  };

  const int s_idx = sy * W + sx;
  at(sx, sy).g = 0.0;
  at(sx, sy).f = h(sx, sy);
  using QItem = std::pair<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  open.push({at(sx, sy).f, s_idx});
  int expanded = 0;
  int goal_idx = -1;
  const double goal_tol = std::max(2.0 * res, 2.0 * res);
  static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const auto [f_cur, cur] = open.top();
    open.pop();
    Node &cn = nodes[static_cast<std::size_t>(cur)];
    if (f_cur > cn.f + 1e-9)
      continue; // 过期的堆项
    const int cx = cn.x, cy = cn.y;
    ++expanded;
    double wx = 0.0, wy = 0.0;
    local_map_->gridToWorld(cx, cy, wx, wy);
    if (std::hypot(wx - to.x, wy - to.y) <= goal_tol) {
      goal_idx = cur;
      break;
    }
    if (expanded > 60000) {
      why = "局部 A* 展开节点过多（>60000）—— 局面太碎，放弃";
      return false;
    }
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dx8[k], ny = cy + dy8[k];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H)
        continue;
      double nwx = 0.0, nwy = 0.0;
      local_map_->gridToWorld(nx, ny, nwx, nwy);
      // ① 只向前：不许沿原路径切线方向后退（倒车/掉头是禁止动作）
      const double lon = (nwx - from.x) * tx + (nwy - from.y) * ty;
      if (lon < -0.05)
        continue;
      // ② 轮廓碰撞（yaw 取"从当前节点来的行进方向"）＋额外跟踪余量
      const double step_yaw = yawBetween(wx, wy, nwx, nwy);
      if (coll_.enabled() &&
          coll_.poseInCollisionAtMargin(nwx, nwy, step_yaw, pathMargin()))
        continue;
      // ③ 绕行幅度上限：超过就整条放弃（宁可停，不进死胡同）
      const double lat = std::fabs((nwx - from.x) * ty - (nwy - from.y) * tx);
      if (lat > p_.max_offset_m)
        continue;
      const int8_t raw = local_map_->rawValue(nx, ny);
      double cost = std::hypot(nwx - wx, nwy - wy);
      if (raw < 0) // 未知：可通行但有惩罚
        cost *= p_.unknown_cost_scale;
      double clr = kInf;
      if (dist_field_ && dist_field_->valid())
        clr = dist_field_->distance(nwx, nwy);
      else if (coll_.hasMap())
        clr = coll_.signedClearanceAt(nwx, nwy, step_yaw, 1.0);
      if (std::isfinite(clr) && clr > 0.0)
        cost += p_.obs_cost_weight * (res / clr); // 靠障碍越近越贵
      cost += p_.lateral_cost_weight * res * (lat / std::max(0.05, p_.max_offset_m));

      Node &nn = at(nx, ny);
      if (nn.g > cn.g + cost) {
        nn.g = cn.g + cost;
        nn.f = nn.g + h(nx, ny);
        nn.parent = cur;
        open.push({nn.f, ny * W + nx});
      }
    }
  }
  stats_.last_nodes = expanded;
  if (goal_idx < 0) {
    why = "局部 A* 找不到「只向前」的绕行路径（展开 " +
          std::to_string(expanded) + " 节点）";
    return false;
  }

  // ---- 回溯成栅格点序列 ----
  std::vector<int> chain;
  for (int cur = goal_idx; cur >= 0; cur = nodes[cur].parent)
    chain.push_back(cur);
  std::reverse(chain.begin(), chain.end());
  std::vector<Pose2D> raw;
  raw.reserve(chain.size() + 2);
  raw.push_back(from); // 起点用真实位姿（不与栅格中心对不齐）
  for (std::size_t i = 1; i + 1 < chain.size(); ++i) {
    const int c = chain[i];
    double wx = 0.0, wy = 0.0;
    local_map_->gridToWorld(nodes[c].x, nodes[c].y, wx, wy);
    Pose2D q;
    q.x = wx;
    q.y = wy;
    q.has_yaw = true;
    raw.push_back(q);
  }
  raw.push_back(to); // 终点也用精确接回点
  for (std::size_t i = 1; i < raw.size(); ++i)
    raw[i - 1].yaw = yawBetween(raw[i - 1].x, raw[i - 1].y, raw[i].x, raw[i].y);
  raw.back().yaw = raw.size() >= 2
                       ? raw[raw.size() - 2].yaw
                       : (to.has_yaw ? to.yaw : 0.0);

  // ---- 拉直（line-of-sight 捷径）：必须用**车体轮廓**验，不能用"中心线不碰致命格"
  //      —— 轮廓有宽度，直线不碰不等于车能过（会切角）。
  auto segment_ok = [&](const Pose2D &a, const Pose2D &b) {
    const double L = std::hypot(b.x - a.x, b.y - a.y);
    const int n = std::max(1, static_cast<int>(std::ceil(L / 0.10)));
    const double yaw = yawBetween(a.x, a.y, b.x, b.y);
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      const double x = a.x + (b.x - a.x) * t;
      const double y = a.y + (b.y - a.y) * t;
      if (coll_.enabled() &&
          coll_.poseInCollisionAtMargin(x, y, yaw, pathMargin()))
        return false;
    }
    return true;
  };
  std::vector<Pose2D> shortened;
  if (p_.smoothing && raw.size() > 2) {
    shortened.push_back(raw.front());
    std::size_t i = 0;
    while (i + 1 < raw.size()) {
      std::size_t j = raw.size() - 1;
      for (; j > i + 1; --j) {
        if (segment_ok(raw[i], raw[j]))
          break;
      }
      shortened.push_back(raw[j]);
      i = j;
    }
  } else {
    shortened = raw;
  }

  // ---- 最终校验：整段按 0.10 m 抽样查轮廓（不过就放弃，绝不交一条没验过的路）----
  for (std::size_t i = 1; i < shortened.size(); ++i) {
    if (!segment_ok(shortened[i - 1], shortened[i])) {
      why = "修补段自检不过（轮廓碰障）→ 放弃";
      return false;
    }
  }
  out = std::move(shortened);
  return true;
}

// ============================================================================
// 主入口
// ============================================================================
LocalPlanResult RollingReplanShimPlanner::computeCommand(const Pose2D &pose,
                                                        double dt) {
  repaired_this_cycle_ = false;
  if (p_.enable) {
    // 走廊模式不绕行（用户口径）：只提示一次
    if (mode() == Mode::kRoute) {
      if (!warned_corridor_) {
        warned_corridor_ = true;
        fprintf(stderr,
                "[rolling_replan] 走廊模式：不做绕行（配置语义如此），"
                "被挡由主控制器如实上报\n");
      }
    } else if (plan_.size() >= 2 && local_map_ && local_map_->valid()) {
      tryRepair(pose);
    }
  }
  LocalPlanResult r = primary_->computeCommand(pose, dt);
  return r;
}

bool RollingReplanShimPlanner::tryRepair(const Pose2D &pose) {
  // ---- 0) 节流 / 放弃窗口 ----
  const auto now = std::chrono::steady_clock::now();
  if (giving_up_) {
    if (now < give_up_until_)
      return false;
    giving_up_ = false;
    stats_.consecutive_fail = 0;
  }
  if (have_last_repair_) {
    const double dt_s =
        std::chrono::duration<double>(now - last_repair_).count();
    if (dt_s < p_.min_interval_s)
      return false;
  }

  std::size_t idx = 0;
  double lat = 0.0;
  const double s_now = projectS(pose, idx, lat);
  if (have_last_repair_ && s_now - s_at_last_repair_ < p_.min_progress_m)
    return false; // 还没往前走，修了也是白修

  // ---- 1) 前瞻：找第一个"被挡"的点 ----
  const double s_end = std::min(arclen_.back(), s_now + p_.lookahead_m);
  double s_block = -1.0;
  for (double s = s_now + p_.check_step_m; s <= s_end + 1e-9;
       s += p_.check_step_m) {
    Pose2D q;
    if (!sampleAt(s, q))
      break;
    double clr = 0.0;
    if (blockedAt(q, p_.trigger_clearance_m, clr)) {
      s_block = s;
      break;
    }
  }
  if (s_block < 0.0) {
    stats_.consecutive_fail = 0;
    return false; // 前方通畅：本周期不修
  }
  stats_.last_s_block = s_block;
  stats_.last_s_now = s_now;

  // ---- 2) 修补窗口 [s_a, s_b] ----
  const double s_a = std::min(s_block, std::max(s_now, s_block - p_.back_m));
  double s_b = std::min(arclen_.back(), s_block + p_.roll_m);
  const double s_b_max = std::min(arclen_.back(), s_block + p_.max_roll_m);
  while (s_b < s_b_max) {
    Pose2D q;
    if (!sampleAt(s_b, q))
      break;
    double clr = 0.0;
    if (!blockedAt(q, p_.trigger_clearance_m, clr))
      break; // 找到"接回点"（回到触发阈之上）
    s_b = std::min(s_b_max, s_b + p_.check_step_m);
  }
  Pose2D from, to;
  if (!sampleAt(s_a, from) || !sampleAt(s_b, to)) {
    stats_.last_msg = "采样修补窗口失败（路径太短？）";
    return false;
  }

  // ---- 3) 局部 A*（只向前）+ 拉直 + 校验 ----
  const auto t0 = std::chrono::steady_clock::now();
  ++stats_.attempts;
  std::vector<Pose2D> seg;
  std::string why;
  if (!localSearchForward(from, to, seg, why)) {
    stats_.last_msg = why;
    if (++stats_.consecutive_fail >= p_.max_consecutive_fail &&
        p_.max_consecutive_fail > 0) {
      giving_up_ = true;
      give_up_until_ =
          now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(p_.give_up_s));
    }
    return false;
  }
  const auto t1 = std::chrono::steady_clock::now();
  stats_.last_ms = std::chrono::duration<double>(t1 - t0).count() * 1000.0;

  // ---- 4) 侧向偏离上限 ----
  const double tan_yaw = yawBetween(from.x, from.y, to.x, to.y);
  const double off = maxLateral(seg, from, tan_yaw);
  if (off > p_.max_offset_m) {
    stats_.last_msg = "绕行幅度 " + std::to_string(off) + " m 超过上限 " +
                      std::to_string(p_.max_offset_m) + " m → 放弃（宁可停）";
    return false;
  }
  stats_.last_offset_m = off;
  stats_.last_len_m = polylineLen(seg);

  // ---- 5) 拼接：原路径[0..a] + 修补段 + 原路径[b..]（终点不变）----
  std::vector<Pose2D> merged;
  merged.reserve(plan_.size() + seg.size());
  std::size_t i_a = 0, i_b = plan_.size() - 1;
  while (i_a + 1 < plan_.size() && arclen_[i_a + 1] <= s_a)
    ++i_a;
  i_b = i_a;
  while (i_b + 1 < plan_.size() && arclen_[i_b] < s_b)
    ++i_b;
  for (std::size_t k = 0; k <= i_a && k < plan_.size(); ++k)
    merged.push_back(plan_[k]);
  for (const auto &q : seg) {
    if (!merged.empty() && dist2(merged.back(), q.x, q.y) < 1e-6)
      continue;
    merged.push_back(q);
  }
  for (std::size_t k = i_b + 1; k < plan_.size(); ++k) {
    if (!merged.empty() && dist2(merged.back(), plan_[k].x, plan_[k].y) < 1e-6)
      continue;
    merged.push_back(plan_[k]);
  }
  if (merged.size() < 2) {
    stats_.last_msg = "拼接后路径退化（<2 点）→ 放弃";
    return false;
  }
  // 终点与朝向必须仍是原任务的（这条保证"绕行不改变任务"）
  merged.back() = original_.back();

  plan_ = std::move(merged);
  applyPlanToPrimary();
  last_repair_ = now;
  have_last_repair_ = true;
  s_at_last_repair_ = s_now;
  stats_.consecutive_fail = 0;
  stats_.last_msg.clear();
  ++stats_.repairs;
  repaired_this_cycle_ = true;
  fprintf(stderr,
          "[rolling_replan] 绕行修补：冲突 @s=%.2f m，替换 [%.2f, %.2f] → "
          "%.2f m / 最大侧偏 %.2f m / A* %d 节点 / %.1f ms\n",
          s_block, s_a, s_b, stats_.last_len_m, stats_.last_offset_m,
          stats_.last_nodes, stats_.last_ms);
  return true;
}

void RollingReplanShimPlanner::reset() {
  original_.clear();
  plan_.clear();
  arclen_.clear();
  stats_ = RepairStats{};
  repaired_this_cycle_ = false;
  have_last_repair_ = false;
  giving_up_ = false;
  warned_corridor_ = false;
  if (primary_)
    primary_->reset();
}

// ============================================================================
// 转发
// ============================================================================
bool RollingReplanShimPlanner::producesCmdVel() const {
  return primary_ ? primary_->producesCmdVel() : false;
}
double RollingReplanShimPlanner::stopCoast() const {
  return primary_ ? primary_->stopCoast() : 0.0;
}
double RollingReplanShimPlanner::corridorTolerance() const {
  return primary_ ? primary_->corridorTolerance() : 0.0;
}
double RollingReplanShimPlanner::maxSpeed() const {
  return primary_ ? primary_->maxSpeed() : 0.0;
}
double RollingReplanShimPlanner::brakeAcc() const {
  return primary_ ? primary_->brakeAcc() : 0.0;
}
double RollingReplanShimPlanner::maxOmega() const {
  return primary_ ? primary_->maxOmega() : 0.0;
}
double RollingReplanShimPlanner::goalYawTolerance() const {
  return primary_ ? primary_->goalYawTolerance() : 0.0;
}
bool RollingReplanShimPlanner::degraded() const {
  return primary_ ? primary_->degraded() : LocalPlanner::degraded();
}
std::string RollingReplanShimPlanner::diagString() const {
  std::string s = "rolling{修补 " + std::to_string(stats_.repairs) + "/尝试 " +
                  std::to_string(stats_.attempts) + " 次";
  if (stats_.repairs > 0)
    s += "，上次 " + std::to_string(static_cast<int>(stats_.last_len_m * 100) /
                                    100.0) +
         " m/侧偏 " + std::to_string(static_cast<int>(stats_.last_offset_m *
                                                    100) / 100.0) + " m";
  s += "}";
  if (primary_) {
    const std::string in = primary_->diagString();
    if (!in.empty())
      s += " | " + in;
  }
  return s;
}

} // namespace pnc_2d
