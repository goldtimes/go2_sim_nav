#include "pnc_2d/core/local_distance_field.hpp"

#include <algorithm>
#include <cmath>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"

namespace pnc_2d {

bool LocalDistanceField::buildFromSamples(double origin_x, double origin_y,
                                          double resolution, int width, int height,
                                          const std::vector<DistanceSample> &samples,
                                          double max_distance, int fill_radius)
{
  if (resolution <= 0.0 || width <= 0 || height <= 0 || max_distance <= 0.0)
    return false;
  if (samples.empty())
    return false;

  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  max_dist_ = max_distance;
  d_.assign(static_cast<std::size_t>(width) * height, static_cast<float>(max_distance));
  sample_cells_ = filled_cells_ = 0;
  max_fill_used_ = 0;

  // ---- 1) 采样落到**最近格**并取 max ----
  //
  // 为什么取 max 而不是 min/覆盖：距离场是"到最近障碍的距离"，同一格被多个采样
  // 命中时（点云抖动、采样落在格边界）取 max 更接近真实值（min 会被单个异常近的
  // 采样拉低，让机器人无谓地绕远）。
  for (const auto &s : samples) {
    const int x = static_cast<int>(std::floor((s.x - origin_x_) / resolution_));
    const int y = static_cast<int>(std::floor((s.y - origin_y_) / resolution_));
    if (x < 0 || y < 0 || x >= width_ || y >= height_)
      continue;  // 图外的采样直接丢（它们对本地图内的决策没有意义）
    const double dist = std::min(std::max(0.0, s.d), max_distance);
    float &cell = d_[static_cast<std::size_t>(y) * width_ + x];
    if (cell >= static_cast<float>(max_distance) - 1e-6f) {
      ++sample_cells_;  // 首次命中（之前是"未覆盖"）
      cell = static_cast<float>(dist);
    } else {
      cell = std::max(cell, static_cast<float>(dist));
    }
  }

  if (sample_cells_ == 0)
    return false;  // 采样全在图外：宁可报失败，也不要用一张"全是 max_dist"的假场

  // ---- 2) 邻域填充：把稀疏采样糊成连续场 ----
  //
  // esdf_pub_step=2（0.2 m）配 0.1 m 栅格时，原始采样会留下一半空格。
  //
  // ⚠ **不能"复制最近邻居的值"**：那样得到的是阶梯场，双线性插值出来的值误差约
  //   半个采样间距（实测 0.15 m on 0.2 m 采样），而安全距离本身才 0.45 m ——
  //   安全余量会被这个误差吃掉。
  //   改成**取可用邻居的平均**并逐轮外扩：在采样缝里得到线性过渡（两个采样半径
  //   0.3/0.5 之间的格子会得到 0.4，正是解析值），误差降回"双线性插值"本身的量级。
  //   已覆盖的格子**永不被覆盖** —— 真实采样优先。
  for (int r = 1; r <= fill_radius; ++r) {
    const std::vector<float> prev = d_;  // 每轮基于上一轮快照，避免一轮内传播多格
    const float max_f = static_cast<float>(max_distance);
    std::size_t filled_this_round = 0;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const std::size_t idx = static_cast<std::size_t>(y) * width_ + x;
        if (prev[idx] < max_f - 1e-6f)
          continue;  // 已经有真实值
        // 只用上下左右四个邻居：8 邻域会因对角权重相等而各向异性（对调和平均而言）
        double sum = 0.0;
        int cnt = 0;
        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int n = 0; n < 4; ++n) {
          if (nx[n] < 0 || ny[n] < 0 || nx[n] >= width_ || ny[n] >= height_)
            continue;
          const float v = prev[static_cast<std::size_t>(ny[n]) * width_ + nx[n]];
          if (v < max_f - 1e-6f) {
            sum += v;
            ++cnt;
          }
        }
        if (cnt > 0) {
          d_[idx] = static_cast<float>(sum / cnt);
          ++filled_this_round;
          max_fill_used_ = static_cast<std::size_t>(r);
        }
      }
    }
    filled_cells_ += filled_this_round;
    if (filled_this_round == 0)
      break;  // 没有新的可填了
  }

  return true;
}

bool LocalDistanceField::buildFree(double origin_x, double origin_y,
                                   double resolution, int width, int height,
                                   double max_distance)
{
  if (resolution <= 0.0 || width <= 0 || height <= 0 || max_distance <= 0.0)
    return false;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  max_dist_ = max_distance;
  d_.assign(static_cast<std::size_t>(width) * height, static_cast<float>(max_distance));
  sample_cells_ = 0;  // 一个真实采样都没有（区别于"有采样"）
  filled_cells_ = 0;
  max_fill_used_ = 0;
  return true;
}

bool LocalDistanceField::buildFromClearance(const CostMap2D &map, int hard_threshold,
                                            bool unknown_as_occupied,
                                            double max_distance)
{
  if (!map.valid() || max_distance <= 0.0)
    return false;

  ClearanceField cf;
  if (!cf.build(map, hard_threshold, unknown_as_occupied))
    return false;

  origin_x_ = map.originX();
  origin_y_ = map.originY();
  resolution_ = map.resolution();
  width_ = cf.width();
  height_ = cf.height();
  max_dist_ = max_distance;
  d_.assign(static_cast<std::size_t>(width_) * height_,
            static_cast<float>(max_distance));
  sample_cells_ = filled_cells_ = 0;
  max_fill_used_ = 0;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      // ClearanceField 给的是"到最近致命格**中心**"的距离；这里只做软引导，
      // 不追求保守，直接用它即可（硬侧的安全边界由膨胀图/footprint 保证）。
      const double dist = std::min(cf.distanceToLethal(x, y), max_distance);
      d_[static_cast<std::size_t>(y) * width_ + x] = static_cast<float>(dist);
    }
  }
  sample_cells_ = d_.size();  // 自算路径下每个格子都有值（没有"未覆盖"）
  return true;
}

bool LocalDistanceField::worldToGrid(double wx, double wy, int &x, int &y) const
{
  x = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  y = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

double LocalDistanceField::at(int x, int y) const
{
  if (x < 0 || y < 0 || x >= width_ || y >= height_)
    return max_dist_;  // 图外 = 很远（乐观，见文件头）
  return d_[static_cast<std::size_t>(y) * width_ + x];
}

double LocalDistanceField::bilinear(double gx, double gy) const
{
  // gx/gy 是"以格中心为 0.5 偏移"的连续格坐标：gx = (wx - ox)/res - 0.5
  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const double fx = gx - x0;
  const double fy = gy - y0;
  const double v00 = at(x0, y0);
  const double v10 = at(x0 + 1, y0);
  const double v01 = at(x0, y0 + 1);
  const double v11 = at(x0 + 1, y0 + 1);
  return (1.0 - fx) * (1.0 - fy) * v00 + fx * (1.0 - fy) * v10 +
         (1.0 - fx) * fy * v01 + fx * fy * v11;
}

double LocalDistanceField::distance(double wx, double wy) const
{
  if (!valid())
    return max_dist_;
  const double gx = (wx - origin_x_) / resolution_ - 0.5;
  const double gy = (wy - origin_y_) / resolution_ - 0.5;
  // 完全在图外：直接给乐观值（不要插值到图外，那边没有信息）
  if (gx < -0.5 || gy < -0.5 || gx > width_ - 0.5 || gy > height_ - 0.5)
    return max_dist_;
  return bilinear(gx, gy);
}

bool LocalDistanceField::gradient(double wx, double wy, double &gx, double &gy) const
{
  gx = gy = 0.0;
  if (!valid())
    return false;
  int ix = 0;
  int iy = 0;
  if (!worldToGrid(wx, wy, ix, iy))
    return false;
  // 中心差分：场是 C0（分片双线性），一阶差分足够；用整格步长可避开插值本身的
  // 分片结构带来的伪梯度。
  const double d_xp = at(ix + 1, iy);
  const double d_xm = at(ix - 1, iy);
  const double d_yp = at(ix, iy + 1);
  const double d_ym = at(ix, iy - 1);
  gx = (d_xp - d_xm) / (2.0 * resolution_);
  gy = (d_yp - d_ym) / (2.0 * resolution_);
  // 梯度模太小 ⇒ 这里没有可用方向（开阔区/障碍内部），让调用方自己决定怎么办
  return std::hypot(gx, gy) > 1e-6;
}

}  // namespace pnc_2d
