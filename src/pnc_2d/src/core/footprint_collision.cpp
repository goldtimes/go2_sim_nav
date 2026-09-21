// FootprintCollisionChecker 的实现。

#include "pnc_2d/core/footprint_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pnc_2d/core/cost_map_2d.hpp"

namespace pnc_2d {
namespace {

/// 方格 [cx0,cx0+res]×[cy0,cy0+res] 与"中心 (rx,ry)、长轴 (ux,uy)、半长 hu、半宽 hv"的
/// 旋转矩形是否相交（2D 分离轴 SAT，四个轴：世界 x/y + 矩形两个轴）。
bool cellIntersectsRect(double cx0, double cy0, double res,
                        double rx, double ry, double ux, double uy,
                        double hu, double hv)
{
  const double sx = cx0 + 0.5 * res;
  const double sy = cy0 + 0.5 * res;
  const double sh = 0.5 * res;
  const double vx = -uy;
  const double vy = ux;

  // 轴 1：世界 x
  if (std::fabs(sx - rx) > sh + hu * std::fabs(ux) + hv * std::fabs(vx)) return false;
  // 轴 2：世界 y
  if (std::fabs(sy - ry) > sh + hu * std::fabs(uy) + hv * std::fabs(vy)) return false;
  // 轴 3：矩形长轴
  if (std::fabs((sx - rx) * ux + (sy - ry) * uy) > hu + sh * (std::fabs(ux) + std::fabs(uy))) {
    return false;
  }
  // 轴 4：矩形短轴
  if (std::fabs((sx - rx) * vx + (sy - ry) * vy) > hv + sh * (std::fabs(vx) + std::fabs(vy))) {
    return false;
  }
  return true;
}

}  // namespace

void FootprintCollisionChecker::configure(const FootprintParams & fp, int hard_threshold,
                                          bool unknown_as_occupied)
{
  fp_ = fp;
  hard_threshold_ = hard_threshold;
  unknown_as_occupied_ = unknown_as_occupied;
  if (map_) buildDirectionOffsets();
}

void FootprintCollisionChecker::setMap(const CostMap2D * map)
{
  map_ = map;
  buildDirectionOffsets();
}

bool FootprintCollisionChecker::cellLethal(int x, int y) const
{
  if (!map_ || !map_->valid()) return true;
  if (!map_->inside(x, y)) return true;   // 图外视为不可通行（保守）
  const int8_t raw = map_->rawValue(x, y);
  if (raw < 0) return unknown_as_occupied_;
  return raw >= static_cast<int8_t>(hard_threshold_);
}

bool FootprintCollisionChecker::pointLethal(double wx, double wy) const
{
  int x = 0;
  int y = 0;
  if (!map_ || !map_->worldToGrid(wx, wy, x, y)) return true;
  return cellLethal(x, y);
}

void FootprintCollisionChecker::rectOffsets(double yaw, std::vector<Cell> & out) const
{
  out.clear();
  if (!map_ || !map_->valid()) return;

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double ux = c;
  const double uy = s;              // 长轴 = 朝向
  const double hu = fp_.halfLength();
  const double hv = fp_.halfWidth();

  // 矩形中心相对"当前格中心"的偏移（offset 定义在机体系）
  const double rx = fp_.offset_x * c - fp_.offset_y * s;
  const double ry = fp_.offset_x * s + fp_.offset_y * c;

  const double res = map_->resolution();
  const double ext_x = hu * std::fabs(ux) + hv * std::fabs(s);
  const double ext_y = hu * std::fabs(uy) + hv * std::fabs(c);
  const int nx = static_cast<int>(std::ceil((ext_x + std::fabs(rx)) / res)) + 1;
  const int ny = static_cast<int>(std::ceil((ext_y + std::fabs(ry)) / res)) + 1;

  for (int dy = -ny; dy <= ny; ++dy) {
    for (int dx = -nx; dx <= nx; ++dx) {
      const double cx0 = (static_cast<double>(dx) - 0.5) * res;
      const double cy0 = (static_cast<double>(dy) - 0.5) * res;
      if (cellIntersectsRect(cx0, cy0, res, rx, ry, ux, uy, hu, hv)) {
        out.push_back(Cell{dx, dy});
      }
    }
  }
}

void FootprintCollisionChecker::buildDirectionOffsets()
{
  for (auto & v : dir_offsets_) v.clear();
  if (!map_ || !map_->valid()) return;
  for (int k = 0; k < 8; ++k) {
    rectOffsets(k * (M_PI / 4.0), dir_offsets_[static_cast<std::size_t>(k)]);
  }
}

int FootprintCollisionChecker::directionIndex(double yaw) const
{
  const double step = M_PI / 4.0;
  const double q = yaw / step;
  const long k = std::lround(q);
  if (std::fabs(q - static_cast<double>(k)) > 1e-6) return -1;
  long idx = k % 8;
  if (idx < 0) idx += 8;
  return static_cast<int>(idx);
}

std::size_t FootprintCollisionChecker::offsetsAtYaw(double yaw) const
{
  const int dir = directionIndex(yaw);
  if (dir >= 0) return dir_offsets_[static_cast<std::size_t>(dir)].size();
  std::vector<Cell> tmp;
  rectOffsets(yaw, tmp);
  return tmp.size();
}

bool FootprintCollisionChecker::fullCheckAtCell(int cx, int cy, double yaw) const
{
  ++full_checks_;
  const int dir = directionIndex(yaw);
  if (dir >= 0) {
    for (const Cell & o : dir_offsets_[static_cast<std::size_t>(dir)]) {
      if (cellLethal(cx + o.dx, cy + o.dy)) return true;
    }
    return false;
  }
  // 非 8 方向之一（终点/剪枝用）：临时算一次（每次几十个格子，可接受）
  std::vector<Cell> offs;
  rectOffsets(yaw, offs);
  for (const Cell & o : offs) {
    if (cellLethal(cx + o.dx, cy + o.dy)) return true;
  }
  return false;
}

bool FootprintCollisionChecker::poseInCollision(double wx, double wy, double yaw) const
{
  if (!map_ || !map_->valid()) return true;
  int cx = 0;
  int cy = 0;
  if (!map_->worldToGrid(wx, wy, cx, cy)) return true;   // 图外 = 碰撞

  if (!fp_.enable) return cellLethal(cx, cy);            // 退化为点判定

  // ---- 快路径：一次查表 + 两个阈值 ----
  if (fp_.fast_path && cf_ != nullptr && cf_->valid() &&
      cf_->width() == map_->width() && cf_->height() == map_->height())
  {
    const double offs = std::hypot(fp_.offset_x, fp_.offset_y);
    if (cf_->lowerBoundM(cx, cy) - offs >= fp_.circumscribedRadius()) {
      return false;                                      // 任何朝向都安全
    }
    if (cf_->upperBoundM(cx, cy) + offs < fp_.inscribedRadius()) {
      return true;                                       // 任何朝向都碰撞
    }
  }
  return fullCheckAtCell(cx, cy, yaw);
}

void FootprintCollisionChecker::cornerWorld(double x, double y, double yaw, int i,
                                            double & ox, double & oy) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hu = fp_.halfLength();
  const double hv = fp_.halfWidth();
  // 机体系四个角 (±hu, ±hv) → 世界（先旋转再加 offset）
  const double lx = (i == 0 || i == 3) ? hu : -hu;
  const double ly = (i == 0 || i == 1) ? hv : -hv;
  ox = x + fp_.offset_x * c - fp_.offset_y * s + lx * c - ly * s;
  oy = y + fp_.offset_x * s + fp_.offset_y * c + lx * s + ly * c;
}

void FootprintCollisionChecker::footprintCorners(double x, double y, double yaw,
                                                double out[8]) const
{
  for (int i = 0; i < 4; ++i) {
    cornerWorld(x, y, yaw, i, out[2 * i], out[2 * i + 1]);
  }
}

bool FootprintCollisionChecker::lineHitsLethal(double x0, double y0, double x1,
                                               double y1) const
{
  if (!map_ || !map_->valid()) return true;

  double gx0 = 0.0;
  double gy0 = 0.0;
  double gx1 = 0.0;
  double gy1 = 0.0;
  map_->worldToGridContinuous(x0, y0, gx0, gy0);
  map_->worldToGridContinuous(x1, y1, gx1, gy1);

  int x = static_cast<int>(std::floor(gx0));
  int y = static_cast<int>(std::floor(gy0));
  const double dxg = gx1 - gx0;
  const double dyg = gy1 - gy0;

  if (cellLethal(x, y)) return true;
  if (std::fabs(dxg) < 1e-12 && std::fabs(dyg) < 1e-12) return false;

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

  // Amanatides & Woo：逐格推进（不靠采样，因此不会穿过单格厚的薄墙）
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
    if (cellLethal(x, y)) return true;
  }
  return false;
}

bool FootprintCollisionChecker::edgeInCollision(double x0, double y0, double x1,
                                                double y1) const
{
  // 整条边用同一个 yaw = 线段方向：机器人是"直线行驶 + 到点再原地转"，
  // 不是"边平移边旋转"。两端各查一次位姿，中间靠中心线与四角轨迹覆盖。
  const double yaw = std::atan2(y1 - y0, x1 - x0);
  if (poseInCollision(x0, y0, yaw)) return true;
  if (poseInCollision(x1, y1, yaw)) return true;
  if (!fp_.enable || !fp_.check_edges) return false;

  // 中心线：车体中心扫过的路径
  if (lineHitsLethal(x0, y0, x1, y1)) return true;

  // 四角轨迹：挡住"两端都不碰、但中间擦角"的情况
  for (int i = 0; i < 4; ++i) {
    double ax = 0.0;
    double ay = 0.0;
    double bx = 0.0;
    double by = 0.0;
    cornerWorld(x0, y0, yaw, i, ax, ay);
    cornerWorld(x1, y1, yaw, i, bx, by);
    if (lineHitsLethal(ax, ay, bx, by)) return true;
  }
  return false;
}

}  // namespace pnc_2d
