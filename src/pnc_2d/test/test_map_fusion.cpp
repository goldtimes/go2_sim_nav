// 全局图融合单元测试：不依赖 ROS。
//
// 重点（对着 map_fusion.hpp 里那三条语义与 fuseObstacleCells 的约定）：
//   1. **保守重采样**：目标格只要与任一占据的源格重叠 ⇒ 判占据（跨分辨率、跨 yaw）
//   2. 只做"占据"方向的 OR：未知(-1)/空闲(0) 一律原样保留
//   3. 只处理交叠区：源图落在目标几何之外的部分不产生任何影响
//   4. 返回值 = **新增**格数（不是命中格数）—— 它是"融合有没有真起作用"的唯一指标
//   5. 距离场压低：与格下标对齐、band 之外一字未改、几何不一致直接拒绝
//
// ⚠ 关于"精确到格"：`projectOccupancy` 有意是**保守**的（源格恰好落在目标格边界上
//   时会把两侧都算上），所以断言写成"包含 + 有界外扩"而不是"恰好 N 格"——
//   后者会把这个刻意的保守性当成 bug（方向错了：保守是安全侧）。

#include <cmath>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/map_fusion.hpp"

namespace pnc_2d {
namespace {

/// 造一张图：几何 + 占据格下标（值 100），其余格填 base
std::shared_ptr<CostMap2D> makeMap(
    int w, int h, double res, double ox, double oy, double oyaw,
    const std::vector<std::pair<int, int>> &occ, int8_t base = 0) {
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, base);
  for (const auto &c : occ)
    data[static_cast<std::size_t>(c.second) * w + c.first] = 100;
  auto m = std::make_shared<CostMap2D>();
  EXPECT_TRUE(m->set(w, h, res, ox, oy, oyaw, data, "map"));
  return m;
}

bool marked(const std::vector<int8_t> &mask, int w, int x, int y) {
  return mask[static_cast<std::size_t>(y) * w + x] >= 50;
}

std::size_t markedCount(const std::vector<int8_t> &mask) {
  std::size_t n = 0;
  for (int8_t v : mask)
    if (v >= 50)
      ++n;
  return n;
}

// ---------------------------------------------------------------- 保守重采样

TEST(MapFusion, ProjectionNeverMissesAnOverlappingCell) {
  // 全局 0.05 m（50x50 = 2.5 m 见方），局部 0.10 m（25x25 同区域）
  const auto src = makeMap(50, 50, 0.05, 0.0, 0.0, 0.0, {{20, 20}});
  const auto dst = makeMap(25, 25, 0.10, 0.0, 0.0, 0.0, {});
  std::vector<int8_t> mask;
  const std::size_t n = projectOccupancy(*src, *dst, 50, mask);
  ASSERT_GT(n, 0u);
  // 源占据格覆盖 [1.00,1.05]² ⇒ 局部格 (10,10) 覆盖 [1.00,1.10]²，必须命中
  EXPECT_TRUE(marked(mask, 25, 10, 10));
  // 有界外扩：源障碍 0.05 m、目标格 0.10 m ⇒ 命中格数不该超过 4（2×2）
  EXPECT_LE(n, 4u);
  std::printf("      [跨分辨率] 源 1 格(0.05) → 目标 %zu 格(0.10)\n", n);
}

TEST(MapFusion, ProjectionIsBoundedWhenObstacleStraddlesBoundary) {
  // 占据格恰好压在局部格边界上（最容易被"点采样"漏掉的位置）
  const auto src = makeMap(50, 50, 0.05, 0.0, 0.0, 0.0, {{20, 20}, {20, 21}});
  const auto dst = makeMap(25, 25, 0.10, 0.0, 0.0, 0.0, {});
  std::vector<int8_t> mask;
  const std::size_t n = projectOccupancy(*src, *dst, 50, mask);
  // 障碍覆盖 x∈[1.00,1.05]、y∈[1.00,1.10] ⇒ 局部格 (10,10)（[1.0,1.1]²）必命中。
  // ⚠ **不**断言 (10,11)：它只与障碍的 y 上边界相切，命不命中取决于"边界算不算
  //   重叠"这个实现选择（当前实现算，方向保守）—— 把它写成硬断言会把一个刻意
  //   的保守行为当成契约，以后想收紧反而改不动。
  EXPECT_TRUE(marked(mask, 25, 10, 10));
  EXPECT_LE(n, 8u) << "保守但必须有界（不能满图乱溼）";
  std::printf("      [压边界] 2 源格 → 目标 %zu 格\n", n);
}

TEST(MapFusion, ProjectionIgnoresSourcesOutsideTargetWindow) {
  // 目标窗只覆盖 [0,1]²，障碍放在 2 m 外
  const auto src = makeMap(50, 50, 0.05, 0.0, 0.0, 0.0, {{40, 40}});
  const auto dst = makeMap(10, 10, 0.10, 0.0, 0.0, 0.0, {});
  std::vector<int8_t> mask;
  EXPECT_EQ(projectOccupancy(*src, *dst, 50, mask), 0u);
  EXPECT_EQ(markedCount(mask), 0u);
}

TEST(MapFusion, ProjectionRespectsTargetOriginYaw) {
  // 目标图带 30° yaw：把障碍放在**某个目标格中心**的世界位置，必须命中那一格
  // 注：目标 origin 取 (3.0, 2.0) 是为了让那个格子的世界坐标落在源图（[0,5]²）内
  const double yaw = M_PI / 6.0;
  const auto dst = makeMap(20, 20, 0.10, 3.0, 2.0, yaw, {});
  double wx = 0.0, wy = 0.0;
  dst->gridToWorld(7, 12, wx, wy);

  const double sres = 0.05;
  const int ix = static_cast<int>(std::floor(wx / sres));
  const int iy = static_cast<int>(std::floor(wy / sres));
  ASSERT_GE(ix, 0);
  ASSERT_GE(iy, 0);
  ASSERT_LT(ix, 100);
  ASSERT_LT(iy, 100);
  const auto src = makeMap(100, 100, sres, 0.0, 0.0, 0.0, {{ix, iy}});

  std::vector<int8_t> mask;
  EXPECT_GT(projectOccupancy(*src, *dst, 50, mask), 0u);
  EXPECT_TRUE(marked(mask, 20, 7, 12))
      << "带 yaw 的目标图上应命中障碍所在那一格（世界系对齐）";
  std::printf("      [yaw=30°] 目标格 (7,12) 中心 (%.3f, %.3f) → 命中\n", wx, wy);
}

// ---------------------------------------------------------------- 只做 OR

TEST(MapFusion, FuseAddsOccupiedAndKeepsUnknownAndFree) {
  // 局部：A(2,2)=100 已是障碍，B(5,5)=-1 未知，其余 0 空闲
  auto local = makeMap(10, 10, 0.10, 0.0, 0.0, 0.0, {{2, 2}}, 0);
  std::vector<int8_t> data = local->data();
  data[5 * 10 + 5] = -1;
  ASSERT_TRUE(local->set(10, 10, 0.10, 0.0, 0.0, 0.0, data, "map"));

  std::vector<int8_t> mask(100, 0);
  mask[5 * 10 + 5] = 100;  // 与未知格重叠
  mask[8 * 10 + 8] = 100;  // 与空闲格重叠
  mask[2 * 10 + 2] = 100;  // 与**已占据**格重叠（不该计入新增）

  std::vector<int8_t> out;
  const std::size_t added = fuseOccupancyMask(*local, mask, 50, out);
  EXPECT_EQ(added, 2u) << "只有 B/C 两格是新增；(2,2) 本来就是障碍";
  EXPECT_EQ(out[5 * 10 + 5], 100);
  EXPECT_EQ(out[8 * 10 + 8], 100);
  EXPECT_EQ(out[2 * 10 + 2], 100);
  // 未被 mask 覆盖的格子原样保留（未知还是未知、空闲还是空闲）
  EXPECT_EQ(out[0], 0);
  EXPECT_EQ(out[7 * 10 + 7], 0);
}

TEST(MapFusion, FuseRejectsSizeMismatch) {
  const auto dst = makeMap(10, 10, 0.10, 0.0, 0.0, 0.0, {});
  std::vector<int8_t> mask(50, 100); // 尺寸不对
  std::vector<int8_t> out;
  EXPECT_EQ(fuseOccupancyMask(*dst, mask, 50, out), 0u);
}

TEST(MapFusion, FusionMakesLocalSeeWhatOnlyGlobalKnows) {
  // 现场场景的缩影：**感知什么都没看见（全空闲）**，全局图说这里有个货柜
  const auto local = makeMap(20, 20, 0.10, 0.0, 0.0, 0.0, {}, 0);
  // 全局：0.05 m，货柜覆盖 1×0.5 m（20×10 格）
  std::vector<std::pair<int, int>> cargo;
  for (int gx = 20; gx < 40; ++gx)
    for (int gy = 20; gy < 30; ++gy)
      cargo.emplace_back(gx, gy);
  const auto global = makeMap(100, 100, 0.05, 0.0, 0.0, 0.0, cargo);

  std::vector<int8_t> mask;
  ASSERT_GT(projectOccupancy(*global, *local, 50, mask), 0u);
  std::vector<int8_t> fused;
  const std::size_t added = fuseOccupancyMask(*local, mask, 50, fused);
  EXPECT_GT(added, 0u);
  // 货柜中心 (1.5, 1.25) 落在局部格 (15,12)/(15,13) ⇒ 融合后必为占据
  EXPECT_EQ(fused[12 * 20 + 15], 100);
  // 远处仍然是空闲（融合没有把整张图变黑）
  EXPECT_EQ(fused[2 * 20 + 2], 0);
  std::printf("      [货柜场景] 全局 400 格(0.05) 并进局部：新增 %zu 格(0.10)\n",
              added);
}

// ---------------------------------------------------------------- 距离场压低

TEST(MapFusion, FieldClampLowersNearIncludedObstaclesOnly) {
  const int w = 20, h = 20;
  const double res = 0.10;
  LocalDistanceField f;
  ASSERT_TRUE(f.buildFree(0.0, 0.0, res, w, h, 3.0));
  EXPECT_DOUBLE_EQ(f.at(5, 5), 3.0);

  // 被并入的障碍：只有 (10,10) 一格
  std::vector<int8_t> occ(static_cast<std::size_t>(w) * h, 0);
  occ[10 * w + 10] = 100;
  auto gmap = std::make_shared<CostMap2D>();
  ASSERT_TRUE(gmap->set(w, h, res, 0.0, 0.0, 0.0, occ, "map"));

  const std::size_t n = f.fuseObstacleCells(*gmap, 50, 1.0);
  EXPECT_GT(n, 0u);
  // 障碍格本身 ⇒ 0；相邻格 ⇒ 约一格距离；band（1.0 m = 10 格）之外一字未改
  EXPECT_DOUBLE_EQ(f.at(10, 10), 0.0);
  EXPECT_LE(f.at(11, 10), 0.11);
  EXPECT_GT(f.at(11, 10), 0.0);
  EXPECT_DOUBLE_EQ(f.at(0, 0), 3.0);
  std::printf("      [压低场] 压低 %zu 格；障碍处 d=%.3f、邻格 d=%.3f、远处 d=%.3f\n",
              n, f.at(10, 10), f.at(11, 10), f.at(0, 0));
}

TEST(MapFusion, FieldClampTakesMinimumNeverRaises) {
  const int w = 10, h = 10;
  LocalDistanceField f;
  ASSERT_TRUE(f.buildFree(0.0, 0.0, 0.10, w, h, 3.0));
  // 先把一整片压到 0.2（模拟感知已经看见的障碍），再并一个更远的障碍
  std::vector<int8_t> occ(static_cast<std::size_t>(w) * h, 0);
  for (int x = 0; x < 5; ++x)
    for (int y = 0; y < 5; ++y)
      occ[y * w + x] = 100;
  auto near = std::make_shared<CostMap2D>();
  ASSERT_TRUE(near->set(w, h, 0.10, 0.0, 0.0, 0.0, occ, "map"));
  f.fuseObstacleCells(*near, 50, 3.0);
  const double before = f.at(2, 2);

  // 远处一格障碍 ⇒ 对 (2,2) 的目标值比 before 大 ⇒ 不得抬高
  std::vector<int8_t> occ2(static_cast<std::size_t>(w) * h, 0);
  occ2[9 * w + 9] = 100;
  auto far = std::make_shared<CostMap2D>();
  ASSERT_TRUE(far->set(w, h, 0.10, 0.0, 0.0, 0.0, occ2, "map"));
  f.fuseObstacleCells(*far, 50, 3.0);
  EXPECT_DOUBLE_EQ(f.at(2, 2), before) << "只取 min，绝不抬高";
}

TEST(MapFusion, FieldClampRejectsGeometryMismatch) {
  LocalDistanceField f;
  ASSERT_TRUE(f.buildFree(0.0, 0.0, 0.10, 10, 10, 3.0));
  const auto other = makeMap(10, 10, 0.05, 0.0, 0.0, 0.0, {{5, 5}});
  EXPECT_EQ(f.fuseObstacleCells(*other, 50, 1.5), 0u)
      << "几何（分辨率）不一致必须直接拒绝，不能静默错位";
}

} // namespace
} // namespace pnc_2d
