// CostMap2D 的实现：世界系 ↔ 栅格下标（支持 origin 带 yaw）。

#include "pnc_2d/core/cost_map_2d.hpp"

#include <cmath>

namespace pnc_2d {

bool CostMap2D::set(int width, int height, double resolution,
                    double origin_x, double origin_y, double origin_yaw,
                    std::vector<int8_t> data, const std::string & frame_id)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  origin_yaw_ = origin_yaw;
  cos_yaw_ = std::cos(origin_yaw);
  sin_yaw_ = std::sin(origin_yaw);
  frame_id_ = frame_id;
  data_ = std::move(data);
  return valid();
}

bool CostMap2D::worldToGrid(double wx, double wy, int & x, int & y) const
{
  if (!valid()) {
    x = y = -1;
    return false;
  }
  // 平移到地图局部原点，再反向旋转（R(-yaw)）到地图自身坐标系
  const double dx = wx - origin_x_;
  const double dy = wy - origin_y_;
  const double lx = cos_yaw_ * dx + sin_yaw_ * dy;
  const double ly = -sin_yaw_ * dx + cos_yaw_ * dy;
  x = static_cast<int>(std::floor(lx / resolution_));
  y = static_cast<int>(std::floor(ly / resolution_));
  return inside(x, y);
}

void CostMap2D::worldToGridContinuous(double wx, double wy, double & gx,
                                      double & gy) const
{
  if (!valid()) {
    gx = gy = 0.0;
    return;
  }
  const double dx = wx - origin_x_;
  const double dy = wy - origin_y_;
  gx = (cos_yaw_ * dx + sin_yaw_ * dy) * (1.0 / resolution_);
  gy = (-sin_yaw_ * dx + cos_yaw_ * dy) * (1.0 / resolution_);
}

void CostMap2D::gridToWorld(int x, int y, double & wx, double & wy) const
{
  // 取格中心
  const double lx = (static_cast<double>(x) + 0.5) * resolution_;
  const double ly = (static_cast<double>(y) + 0.5) * resolution_;
  wx = origin_x_ + cos_yaw_ * lx - sin_yaw_ * ly;
  wy = origin_y_ + sin_yaw_ * lx + cos_yaw_ * ly;
}

}  // namespace pnc_2d
