// 精确欧氏距离变换（Felzenszwalb & Huttenlocher 2012）：逐维 1D 变换，
// 复杂度 O(N)，结果与暴力计算完全一致（不是棋盘/倒角距离的近似）。
//
// 为什么坚持"精确"：快路径的两个阈值一个要求距离的**下界**、另一个要求**上界**，
// 近似距离（如倒角距离）只能满足其中一边，会引入误判 —— 详见头文件说明。

#include "pnc_2d/core/clearance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pnc_2d/core/cost_map_2d.hpp"

namespace pnc_2d {
namespace {

constexpr float kInf = std::numeric_limits<float>::max() / 4.0F;

/// 1D 变换：d[q] = min_p ( (q-p)^2 + f[p] )
void dt1d(const std::vector<float> & f, std::vector<float> & d,
          std::vector<int> & v, std::vector<float> & z, int n)
{
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    float s = (f[q] + static_cast<float>(q * q)) -
              (f[v[k]] + static_cast<float>(v[k] * v[k]));
    s /= static_cast<float>(2 * q - 2 * v[k]);
    while (s <= z[k]) {
      --k;
      s = (f[q] + static_cast<float>(q * q)) -
          (f[v[k]] + static_cast<float>(v[k] * v[k]));
      s /= static_cast<float>(2 * q - 2 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<float>(q)) ++k;
    const float dq = static_cast<float>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}

}  // namespace

bool ClearanceField::build(const CostMap2D & map, int hard_threshold,
                          bool unknown_as_occupied)
{
  if (!map.valid()) {
    width_ = height_ = 0;
    dist_.clear();
    return false;
  }
  width_ = map.width();
  height_ = map.height();
  resolution_ = map.resolution();
  margin_ = 0.70710678118 * resolution_;
  origin_x_ = map.originX();
  origin_y_ = map.originY();
  origin_yaw_ = map.originYaw();
  cos_yaw_ = std::cos(origin_yaw_);
  sin_yaw_ = std::sin(origin_yaw_);

  const std::size_t n = distSz();
  dist_.assign(n, kInf);
  lethal_count_ = 0;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int8_t raw = map.rawValue(x, y);
      const bool lethal = (raw < 0) ? unknown_as_occupied
                                    : (raw >= static_cast<int8_t>(hard_threshold));
      if (lethal) {
        dist_[static_cast<std::size_t>(y) * width_ + x] = 0.0F;
        ++lethal_count_;
      }
    }
  }

  // 逐行 → 逐列（两趟 1D 变换 = 精确二维平方距离）
  const int n_max = std::max(width_, height_);
  std::vector<float> f(n_max), d(n_max);
  std::vector<int> v(n_max + 1);
  std::vector<float> z(n_max + 2);

  std::vector<float> col_in(height_), col_out(height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) f[x] = dist_[static_cast<std::size_t>(y) * width_ + x];
    dt1d(f, d, v, z, width_);
    for (int x = 0; x < width_; ++x) dist_[static_cast<std::size_t>(y) * width_ + x] = d[x];
  }
  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) col_in[y] = dist_[static_cast<std::size_t>(y) * width_ + x];
    dt1d(col_in, col_out, v, z, height_);
    for (int y = 0; y < height_; ++y) dist_[static_cast<std::size_t>(y) * width_ + x] = col_out[y];
  }

  // 平方距离 → 距离（单位：格）
  for (auto & value : dist_) value = std::sqrt(value);
  return true;
}

bool ClearanceField::insideWorld(double wx, double wy) const
{
  if (!valid()) return false;
  double gx = 0.0;
  double gy = 0.0;
  worldToGridContinuous(wx, wy, gx, gy);
  const int x = static_cast<int>(std::floor(gx));
  const int y = static_cast<int>(std::floor(gy));
  return inside(x, y);
}

namespace {

/// 提取"双线性插值所需的四个角值 + 权重"，避免两种查询各写一遍
struct BilinearSample {
  double d00{0.0};
  double d10{0.0};
  double d01{0.0};
  double d11{0.0};
  double fu{0.0};
  double fv{0.0};
  double center{0.0};   // 最近格中心的值（= 四角加权后仍是有限的）
};

BilinearSample sampleBilinear(const std::vector<float> & dist, int width, int height,
                              double gx, double gy)
{
  // 以格子下标为单位的连续坐标（格中心 = 整数）；夹到 [0, n-1] 避免越界
  const double u = gx - 0.5;
  const double v = gy - 0.5;
  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  auto at = [&](int x, int y) -> double {
    const int xi = std::clamp(x, 0, width - 1);
    const int yi = std::clamp(y, 0, height - 1);
    return static_cast<double>(dist[static_cast<std::size_t>(yi) * width + xi]);
  };
  BilinearSample s;
  s.fu = u - static_cast<double>(x0);
  s.fv = v - static_cast<double>(y0);
  s.d00 = at(x0, y0);
  s.d10 = at(x0 + 1, y0);
  s.d01 = at(x0, y0 + 1);
  s.d11 = at(x0 + 1, y0 + 1);
  s.center = (1.0 - s.fu) * (1.0 - s.fv) * s.d00 + s.fu * (1.0 - s.fv) * s.d10 +
             (1.0 - s.fu) * s.fv * s.d01 + s.fu * s.fv * s.d11;
  return s;
}

}  // namespace

double ClearanceField::distanceAtWorld(double wx, double wy, double max_m) const
{
  if (!valid()) return max_m;
  if (!insideWorld(wx, wy)) return max_m;   // 越界："不算障碍"
  double gx = 0.0;
  double gy = 0.0;
  worldToGridContinuous(wx, wy, gx, gy);
  const BilinearSample s = sampleBilinear(dist_, width_, height_, gx, gy);
  const double meters = s.center * resolution_;
  if (!std::isfinite(meters)) return max_m;   // 全图无致命格（EDT 初值）
  return std::min(max_m, meters);
}

bool ClearanceField::gradientAtWorld(double wx, double wy, double g[2]) const
{
  if (g != nullptr) {
    g[0] = 0.0;
    g[1] = 0.0;
  }
  if (g == nullptr || !valid() || !insideWorld(wx, wy)) return false;
  double gx = 0.0;
  double gy = 0.0;
  worldToGridContinuous(wx, wy, gx, gy);
  const BilinearSample s = sampleBilinear(dist_, width_, height_, gx, gy);

  // 插值式对"格子下标"的偏导（单位：格/格）
  const double du = (1.0 - s.fv) * (s.d10 - s.d00) + s.fv * (s.d11 - s.d01);
  const double dv = (1.0 - s.fu) * (s.d01 - s.d00) + s.fu * (s.d11 - s.d10);

  // 换算到世界系：d(米)/d(世界) = R(yaw)·(∂D/∂u, ∂D/∂v)
  // （索引↔世界的 1/res 与 格↔米的 res 恰好抵消，所以这里不再除 res）
  g[0] = cos_yaw_ * du - sin_yaw_ * dv;
  g[1] = sin_yaw_ * du + cos_yaw_ * dv;
  return true;
}

}  // namespace pnc_2d
