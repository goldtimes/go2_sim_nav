// 局部距离场（ESDF）：把"到最近障碍的距离"变成可查、可求梯度的连续函数。
//
// == 它解决什么问题 ==
// MPC 的 QP 里要把 "离障碍足够远" 写成约束/代价，前提是距离能对**决策变量**求导。
// 所以需要的不是"一张地图话题"，而是 d(x, y) 这个函数本身（见
// doc/mpc_local_planner_plan.md §1）。本类就是那个函数。
//
// == 两份输入，两条构造路径（对应 D3/D4）==
//   1. buildFromSamples()：消费感知的 `grid_map/esdf_2d`（PointCloud2，
//      intensity = 距离 m）。**这是主路径** —— 不重复造轮子，直接用感知算好的 ESDF。
//   2. buildFromClearance()：用 core/clearance_field（Felzenszwalb EDT）自算，
//      **兜底 + 一致性校验**（感知挂了/没发布时也能跑；单测里用它跟采样路径对拍）。
//
// == 三个必须讲清楚的语义（都踩过坑）==
//   · **空格 = 很远，不是障碍**：`esdf_2d` 只发布 d ≤ esdf_max_dist 的格子，
//     开阔区域一个点都没有。若把空当成 0（近）就会把"看不见障碍的地方"全变成墙，
//     机器人不敢走。所以未覆盖的格子一律取 max_distance（乐观，D3）。
//   · **采样比栅格稀**：`esdf_pub_step=2` + 0.1 m 栅格 ⇒ 采样间距 0.2 m，
//     直接塞进 0.1 m 栅格会有一半格子是空的 → 双线性插值会在 3.0 与 0.5 之间
//     拉出锯齿，梯度完全不可用。所以采样后要做**小半径邻域填充**（fill_radius），
//     把稀疏采样"糊"成连续场。
//   · **图外 = 很远**（同样是乐观策略，与 ClearanceField 的 out-of-map 处理一致；
//     注意这和全局侧 FootprintCollisionChecker 的"图外即致命"不同 ——
//     那边是**硬**安全边界，这边是**软**引导，两者语义本来就该不同）。
//
// == 梯度 ==
// 中心差分（双线性插值后的场是分片双线性、C0 连续，中心差分足够稳）。
// 返回的是 d 增大的方向，也就是"远离障碍"的方向；障碍内部/极近处梯度可能为 0，
// 此时返回 false（调用方应退回"只用软代价"或停车）。

#pragma once

#include <cstddef>
#include <vector>

namespace pnc_2d {

class CostMap2D;

/// 一个距离采样点（世界系坐标 + 到最近障碍的距离 [m]）
struct DistanceSample {
  double x{0.0};
  double y{0.0};
  double d{0.0};
};

class LocalDistanceField {
public:
  /// 主路径：把稀疏点云采样栅格化到指定几何（通常就是局部膨胀图的范围）。
  ///
  /// @param origin_x/origin_y/resolution/width/height  目标栅格几何
  /// @param samples        采样点（世界系）
  /// @param max_distance   距离上限 [m]：未覆盖格子取它，采样值也按它截断
  /// @param fill_radius    邻域填充半径 [格]；0 = 不填充（只适合采样间距 ≈ 分辨率）
  /// @return 是否构建成功（几何非法或一个采样都没有 → false）
  bool buildFromSamples(double origin_x, double origin_y, double resolution,
                        int width, int height,
                        const std::vector<DistanceSample> &samples,
                        double max_distance, int fill_radius = 2);

  /// 兜底路径：用地图自算 EDT（复用 core/clearance_field）。
  /// @param unknown_as_occupied  未知格是否算障碍；软引导场景应传 false（乐观）
  bool buildFromClearance(const CostMap2D &map, int hard_threshold,
                          bool unknown_as_occupied, double max_distance);

  /// “无障碍”的合法场：每个格子都是 max_distance。
  ///
  /// ★ 为什么必须有这个：感知的 `esdf_2d` 只发 `0 < d ≤ max_dist` 的格，
  ///   **周围没有障碍时点云就是空的**（或者全部落在本地图外）。
  ///   "空点云"的正确含义是"这一片很开阔"，**不是**"拿不到距离场"。
  ///   如果把它当成后者（setDistanceField(nullptr)），MPC 会进入降级并限速到 0.3 m/s
  ///   —— 车会在空旷大路上爬行，而原因看起来像"感知挂了"，极难排查。
  bool buildFree(double origin_x, double origin_y, double resolution, int width,
                 int height, double max_distance);

  bool valid() const
  {
    return width_ > 0 && height_ > 0 && resolution_ > 0.0 &&
           d_.size() == static_cast<std::size_t>(width_) * height_;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double maxDistance() const { return max_dist_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  std::size_t cellCount() const { return d_.size(); }

  /// 世界系 → 栅格下标（图外返回 false）
  bool worldToGrid(double wx, double wy, int &x, int &y) const;

  /// 距离查询（**双线性插值**；图外或非法返回 max_dist = 乐观）
  double distance(double wx, double wy) const;

  /// 距离梯度（中心差分，单位无量纲：Δd / Δx）。图外返回 false。
  bool gradient(double wx, double wy, double &gx, double &gy) const;

  /// 直接读格子（越界 → max_dist）
  double at(int x, int y) const;

  // ---------------- 诊断（单测/日志用）----------------
  std::size_t sampleCells() const { return sample_cells_; }  ///< 真正被采样命中的格数
  std::size_t filledCells() const { return filled_cells_; }  ///< 靠邻域填充补出来的格数
  std::size_t maxFillUsed() const { return max_fill_used_; } ///< 实际用到的最大填充半径

private:
  /// 取"以 (gx,gy) 为中心的双线性插值"，输入为**格中心 + 0.5** 约定下的连续坐标
  double bilinear(double gx, double gy) const;

  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.0};
  int width_{0};
  int height_{0};
  double max_dist_{3.0};
  std::vector<float> d_;  ///< 距离 [m]，已截断到 max_dist

  std::size_t sample_cells_{0};
  std::size_t filled_cells_{0};
  std::size_t max_fill_used_{0};
};

}  // namespace pnc_2d
