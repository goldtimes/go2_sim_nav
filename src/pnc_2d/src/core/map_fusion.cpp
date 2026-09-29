#include "pnc_2d/core/map_fusion.hpp"

#include <algorithm>
#include <cmath>

#include "pnc_2d/core/cost_map_2d.hpp"

namespace pnc_2d {

std::size_t projectOccupancy(const CostMap2D &src, const CostMap2D &dst,
                             int occupied_threshold,
                             std::vector<int8_t> &mask) {
  const std::size_t n =
      dst.valid() ? static_cast<std::size_t>(dst.width()) *
                        static_cast<std::size_t>(dst.height())
                  : 0u;
  mask.assign(n, static_cast<int8_t>(0));
  if (n == 0 || !src.valid() || occupied_threshold <= 0)
    return 0;

  const double res = dst.resolution();
  const double cy = std::cos(dst.originYaw());
  const double sy = std::sin(dst.originYaw());
  std::size_t marked = 0;

  for (int y = 0; y < dst.height(); ++y) {
    for (int x = 0; x < dst.width(); ++x) {
      // 该目标格的 4 个角 → 世界系 → 源图**连续**格坐标 → 整数包围盒。
      // 走角点而不是中心：只要有一点点重叠就算命中（语义 2 的保守性来源），
      // 且对 origin yaw 天然正确（不需要特判）。
      double gx_lo = 0.0, gx_hi = 0.0, gy_lo = 0.0, gy_hi = 0.0;
      for (int cx = 0; cx < 2; ++cx) {
        for (int ccc = 0; ccc < 2; ++ccc) {
          const double lx = (x + cx) * res; // 目标图局部米（原点在格 (0,0) 左下角）
          const double ly = (y + ccc) * res;
          const double wx = dst.originX() + cy * lx - sy * ly;
          const double wy = dst.originY() + sy * lx + cy * ly;
          double gx = 0.0, gy = 0.0;
          src.worldToGridContinuous(wx, wy, gx, gy);
          if (cx == 0 && ccc == 0) {
            gx_lo = gx_hi = gx;
            gy_lo = gy_hi = gy;
          } else {
            gx_lo = std::min(gx_lo, gx);
            gx_hi = std::max(gx_hi, gx);
            gy_lo = std::min(gy_lo, gy);
            gy_hi = std::max(gy_hi, gy);
          }
        }
      }

      int ix0 = static_cast<int>(std::floor(gx_lo));
      int ix1 = static_cast<int>(std::floor(gx_hi));
      int iy0 = static_cast<int>(std::floor(gy_lo));
      int iy1 = static_cast<int>(std::floor(gy_hi));
      // 夹进源图范围：**交叠区之外一律忽略**（语义 3）。注意这里必须先夹再判
      // 空：源图完全在目标窗外时，包围盒被夹成空区间，跳过即可，不能当成"占据"。
      ix0 = std::max(ix0, 0);
      iy0 = std::max(iy0, 0);
      ix1 = std::min(ix1, src.width() - 1);
      iy1 = std::min(iy1, src.height() - 1);
      if (ix0 > ix1 || iy0 > iy1)
        continue;

      bool occ = false;
      for (int iy = iy0; iy <= iy1 && !occ; ++iy) {
        for (int ix = ix0; ix <= ix1; ++ix) {
          if (src.rawValue(ix, iy) >= occupied_threshold) {
            occ = true;
            break;
          }
        }
      }
      if (occ) {
        mask[static_cast<std::size_t>(y) * static_cast<std::size_t>(dst.width()) +
             static_cast<std::size_t>(x)] = static_cast<int8_t>(100);
        ++marked;
      }
    }
  }
  return marked;
}

std::size_t fuseOccupancyMask(const CostMap2D &dst,
                              const std::vector<int8_t> &mask,
                              int occupied_threshold,
                              std::vector<int8_t> &out) {
  out = dst.data();
  if (!dst.valid() || mask.size() != out.size() || occupied_threshold <= 0)
    return 0;

  std::size_t added = 0;
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (mask[i] < occupied_threshold)
      continue;
    if (out[i] >= occupied_threshold)
      continue; // 本来就是障碍：不算新增（返回值要能反映真实增益）
    out[i] = static_cast<int8_t>(100);
    ++added;
  }
  return added;
}

} // namespace pnc_2d
