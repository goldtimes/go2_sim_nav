// GlobalPlanner 基类的实现：参数装载、共享服务、路径后处理。

#include "pnc_2d/core/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pnc_2d {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

/**
 * 枚举线段穿过的所有格子（Amanatides & Woo 体素遍历）。
 * f(x, y, t) 返回 false 时提前结束，整体返回 false。
 * 用逐格推进而不是"按分辨率采样"：采样步长会**穿过单格厚的薄墙**（仿真/图纸地图里很常见）。
 */
template <typename F>
bool forEachCellOnSegment(const CostMap2D & map, double x0, double y0, double x1,
                          double y1, F && f)
{
  double gx0 = 0.0;
  double gy0 = 0.0;
  double gx1 = 0.0;
  double gy1 = 0.0;
  map.worldToGridContinuous(x0, y0, gx0, gy0);
  map.worldToGridContinuous(x1, y1, gx1, gy1);

  int x = static_cast<int>(std::floor(gx0));
  int y = static_cast<int>(std::floor(gy0));
  const double dxg = gx1 - gx0;
  const double dyg = gy1 - gy0;

  if (!f(x, y, 0.0)) return false;
  if (std::fabs(dxg) < 1e-12 && std::fabs(dyg) < 1e-12) return true;

  const int stepx = (dxg > 0) ? 1 : ((dxg < 0) ? -1 : 0);
  const int stepy = (dyg > 0) ? 1 : ((dyg < 0) ? -1 : 0);
  const double inf = std::numeric_limits<double>::infinity();
  const double tdx = (dxg != 0.0) ? std::fabs(1.0 / dxg) : inf;
  const double tdy = (dyg != 0.0) ? std::fabs(1.0 / dyg) : inf;
  double tmx = (dxg != 0.0)
                 ? ((stepx > 0) ? (static_cast<double>(x) + 1.0 - gx0) : (gx0 - x)) / std::fabs(dxg)
                 : inf;
  double tmy = (dyg != 0.0)
                 ? ((stepy > 0) ? (static_cast<double>(y) + 1.0 - gy0) : (gy0 - y)) / std::fabs(dyg)
                 : inf;

  const long max_steps = static_cast<long>(std::fabs(dxg) + std::fabs(dyg)) + 4;
  double t = 0.0;
  for (long i = 0; i < max_steps; ++i) {
    if (tmx < tmy) {
      t = tmx;
      x += stepx;
      tmx += tdx;
    } else {
      t = tmy;
      y += stepy;
      tmy += tdy;
    }
    if (t > 1.0) break;
    if (!f(x, y, t)) return false;
  }
  return true;
}

}  // namespace

double GlobalPlanner::msSince(const std::chrono::steady_clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

// --------------------------------------------------------------------------
// 参数
// --------------------------------------------------------------------------

bool GlobalPlanner::loadCommonParams(const ParamReader & p, const std::string & prefix)
{
  prefix_ = prefix;

  // ---- 代价模型（全算法共用，键名固定为 common.*，与 config/*.yaml 一致）----
  cost_.hard_threshold = p.getInt("common.hard_threshold", cost_.hard_threshold);
  cost_.unknown_as_occupied =
    p.getBool("common.unknown_as_occupied", cost_.unknown_as_occupied);
  cost_.soft_cost_weight = p.getDouble("common.soft_cost_weight", cost_.soft_cost_weight);

  // ---- 车体轮廓 ----
  fp_.enable = p.getBool("footprint.enable", fp_.enable);
  fp_.length = p.getDouble("footprint.length", fp_.length);
  fp_.width = p.getDouble("footprint.width", fp_.width);
  fp_.offset_x = p.getDouble("footprint.offset_x", fp_.offset_x);
  fp_.offset_y = p.getDouble("footprint.offset_y", fp_.offset_y);
  fp_.safe_margin = p.getDouble("footprint.safe_margin", fp_.safe_margin);
  fp_.check_edges = p.getBool("footprint.check_edges", fp_.check_edges);
  fp_.fast_path = p.getBool("footprint.fast_path", fp_.fast_path);

  // ---- 路径后处理 ----
  prune_mode_ = p.getString(prefix + ".path_prune_mode", prune_mode_);
  prune_max_span_m_ = p.getDouble(prefix + ".path_prune_max_span", prune_max_span_m_);
  path_resample_spacing_ =
    p.getDouble(prefix + ".path_resample_spacing", path_resample_spacing_);
  keep_start_yaw_ = p.getBool(prefix + ".keep_start_yaw", keep_start_yaw_);

  // ---- 合法性 ----
  if (cost_.hard_threshold < 1) cost_.hard_threshold = 1;
  if (cost_.hard_threshold > 100) cost_.hard_threshold = 100;
  if (cost_.soft_cost_weight < 0.0) cost_.soft_cost_weight = 0.0;
  if (fp_.length <= 0.0 || fp_.width <= 0.0) fp_.enable = false;   // 尺寸非法 → 关掉 footprint
  if (fp_.safe_margin < 0.0) fp_.safe_margin = 0.0;
  if (prune_max_span_m_ < 0.0) prune_max_span_m_ = 0.0;

  rebuildCollisionServices();
  return true;
}

void GlobalPlanner::setCostMap(std::shared_ptr<const CostMap2D> map)
{
  map_ = std::move(map);
  rebuildCollisionServices();
  reset();
}

void GlobalPlanner::rebuildCollisionServices()
{
  collision_.configure(fp_, cost_.hard_threshold, cost_.unknown_as_occupied);
  map_has_soft_cells_ = false;

  if (!map_ || !map_->valid()) {
    clearance_.reset();
    collision_.setMap(nullptr);
    collision_.setClearanceField(nullptr);
    return;
  }

  if (!clearance_) clearance_ = std::make_unique<ClearanceField>();
  clearance_->build(*map_, cost_.hard_threshold, cost_.unknown_as_occupied);

  collision_.setMap(map_.get());
  collision_.setClearanceField(clearance_.get());
  collision_.resetCounters();

  // 地图里是否有"软代价格"（0 < raw < hard_threshold）：没有就可以跳过剪枝时的软代价比较
  if (cost_.soft_cost_weight > 0.0) {
    for (const int8_t raw : map_->data()) {
      if (raw > 0 && raw < static_cast<int8_t>(cost_.hard_threshold)) {
        map_has_soft_cells_ = true;
        break;
      }
    }
  }
}

// --------------------------------------------------------------------------
// 共享工具
// --------------------------------------------------------------------------

bool GlobalPlanner::isHardOccupiedGrid(int x, int y) const
{
  return collision_.cellLethal(x, y);
}

bool GlobalPlanner::isHardOccupiedWorld(double wx, double wy) const
{
  return collision_.pointLethal(wx, wy);
}

bool GlobalPlanner::lineIsCollisionFree(const Pose2D & a, const Pose2D & b) const
{
  // 只用到两端位置：车身朝向取线段方向（见 FootprintCollisionChecker::edgeInCollision）
  return !collision_.edgeInCollision(a.x, a.y, b.x, b.y);
}

bool GlobalPlanner::lineHitsHardObstacle(double x0, double y0, double x1, double y1) const
{
  if (!map_ || !map_->valid()) return true;
  return !forEachCellOnSegment(*map_, x0, y0, x1, y1,
                               [this](int x, int y, double) {
                                 return !collision_.cellLethal(x, y);
                               });
}

double GlobalPlanner::cellExtraCost(int x, int y) const
{
  if (!map_ || !map_->valid()) return 0.0;
  return cost_.extraCost(map_->rawValue(x, y));
}

double GlobalPlanner::softCostAlong(double x0, double y0, double x1, double y1) const
{
  if (!map_ || !map_->valid() || cost_.soft_cost_weight <= 0.0 || !map_has_soft_cells_) return 0.0;
  double sum = 0.0;
  // DDA 遍历保证每格只访问一次，直接累加即可
  forEachCellOnSegment(*map_, x0, y0, x1, y1,
                       [&](int x, int y, double) {
                         sum += cellExtraCost(x, y);
                         return true;
                       });
  return sum;
}

SearchWindow GlobalPlanner::makeSearchWindow(const PlanRequest & req, double margin_m) const
{
  SearchWindow w;
  if (!map_ || !map_->valid()) return w;

  double sx = 0.0;
  double sy = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  map_->worldToGridContinuous(req.start.x, req.start.y, sx, sy);
  map_->worldToGridContinuous(req.goal.x, req.goal.y, gx, gy);

  // margin 可能是 1e9（表示整图），先夹到合理范围再换算成格，避免 int 溢出
  const double res = map_->resolution();
  const double safe_margin = std::min(margin_m, 1.0e6);
  const int m = static_cast<int>(std::min(safe_margin / res, 1.0e6));

  const double lo_x = std::min(sx, gx) - m;
  const double hi_x = std::max(sx, gx) + m;
  const double lo_y = std::min(sy, gy) - m;
  const double hi_y = std::max(sy, gy) + m;

  w.x0 = std::max(0, static_cast<int>(std::floor(lo_x)));
  w.y0 = std::max(0, static_cast<int>(std::floor(lo_y)));
  w.x1 = std::min(map_->width() - 1, static_cast<int>(std::ceil(hi_x)));
  w.y1 = std::min(map_->height() - 1, static_cast<int>(std::ceil(hi_y)));
  if (w.x1 < w.x0) w.x1 = w.x0;
  if (w.y1 < w.y0) w.y1 = w.y0;
  return w;
}

double GlobalPlanner::pathLength(const std::vector<Pose2D> & path) const
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

// --------------------------------------------------------------------------
// 路径后处理
// --------------------------------------------------------------------------

void GlobalPlanner::postProcessPath(std::vector<Pose2D> & path, const PlanRequest & req) const
{
  if (path.size() < 2) {
    if (!path.empty()) {
      path.front().yaw = req.goal.has_yaw ? req.goal.yaw : 0.0;
      path.front().has_yaw = true;
    }
    return;
  }

  // ---------- 1) 剪枝 ----------
  if (prune_mode_ == "line_of_sight") {
    const std::size_t n = path.size();
    // 原始路径的累计弧长与累计软代价（软代价比较的基准）
    std::vector<double> arc(n, 0.0);
    std::vector<double> cost_prefix(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
      arc[i] = arc[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      cost_prefix[i] = cost_prefix[i - 1] +
                       softCostAlong(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
    }
    const bool check_soft = mapHasSoftCells() && cost_.soft_cost_weight > 0.0;

    std::vector<Pose2D> out;
    out.reserve(n);
    out.push_back(path.front());
    std::size_t i = 0;
    while (i + 1 < n) {
      std::size_t best = i + 1;
      for (std::size_t j = i + 2; j < n; ++j) {
        if (arc[j] - arc[i] > prune_max_span_m_) break;         // 跨度上限（限制单次检查长度）
        if (lineHitsHardObstacle(path[i].x, path[i].y, path[j].x, path[j].y)) break;
        if (lineIsCollisionFree(path[i], path[j])) {
          // 只有"软代价不变差"才接受捷径：否则会把"绕开软代价区"的路径又拉回去穿过它
          if (check_soft &&
              softCostAlong(path[i].x, path[i].y, path[j].x, path[j].y) >
              cost_prefix[j] - cost_prefix[i] + 1e-6)
          {
            continue;
          }
          best = j;
        }
      }
      out.push_back(path[best]);
      i = best;
    }
    path.swap(out);
  } else if (prune_mode_ == "collinear") {
    std::vector<Pose2D> out;
    out.reserve(path.size());
    out.push_back(path.front());
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      const double ax = path[i].x - out.back().x;
      const double ay = path[i].y - out.back().y;
      const double bx = path[i + 1].x - path[i].x;
      const double by = path[i + 1].y - path[i].y;
      if (std::fabs(ax * by - ay * bx) > 1e-9) out.push_back(path[i]);
    }
    out.push_back(path.back());
    path.swap(out);
  }

  // ---------- 2) yaw 填充 ----------
  const std::size_t n = path.size();
  for (std::size_t i = 0; i + 1 < n; ++i) {
    path[i].yaw = wrapAngle(std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x));
    path[i].has_yaw = true;
  }
  if (req.goal.has_yaw) {
    path[n - 1].yaw = wrapAngle(req.goal.yaw);
  } else if (n >= 2) {
    path[n - 1].yaw = path[n - 2].yaw;
  }
  path[n - 1].has_yaw = true;
  if (keep_start_yaw_ && req.start.has_yaw) {
    // 首点保留当前朝向：控制器用它做"起步原地对正"
    path[0].yaw = wrapAngle(req.start.yaw);
  }

  // ---------- 3) 可选重采样 ----------
  if (path_resample_spacing_ > 1e-6) {
    std::vector<Pose2D> out;
    out.push_back(path.front());
    double carry = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double dx = path[i].x - path[i - 1].x;
      const double dy = path[i].y - path[i - 1].y;
      const double seg = std::hypot(dx, dy);
      if (seg < 1e-9) continue;
      double t = path_resample_spacing_ - carry;
      while (t <= seg) {
        Pose2D p;
        p.x = path[i - 1].x + dx * (t / seg);
        p.y = path[i - 1].y + dy * (t / seg);
        p.yaw = wrapAngle(std::atan2(dy, dx));
        p.has_yaw = true;
        out.push_back(p);
        t += path_resample_spacing_;
      }
      carry = seg - (t - path_resample_spacing_);
    }
    if (out.size() > 1) {
      out.back() = path.back();
      path.swap(out);
    }
  }
}

}  // namespace pnc_2d
