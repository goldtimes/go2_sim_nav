// 精确欧氏距离变换（Felzenszwalb & Huttenlocher 2012）：逐维 1D 变换，
// 复杂度 O(N)，结果与暴力计算完全一致（不是棋盘/倒角距离的近似）。
//
// 为什么坚持"精确"：快路径的两个阈值一个要求距离的**下界**、另一个要求**上界**，
// 近似距离（如倒角距离）只能满足其中一边，会引入误判 —— 详见头文件说明。

#include "pnc_2d/core/clearance_field.hpp"

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

}  // namespace pnc_2d
