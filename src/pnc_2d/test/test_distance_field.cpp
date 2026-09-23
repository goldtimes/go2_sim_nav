// 局部距离场单测（P5.1）。
//
// 为什么单独测：距离场是 MPC
// 避障项的唯一输入，它的语义错一点，表现是"机器人莫名 绕远/不敢走"，很难从 MPC
// 的输出反推。所以这里把三条语义逐一钉死：
//   · 未覆盖的格子 = **很远**（乐观），不是 0/障碍；
//   · 稀疏采样要被"糊"成连续场（否则插值出锯齿，梯度不可用）；
//   · 梯度指向**远离障碍**的方向。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/local_distance_field.hpp"

namespace pnc_2d {
namespace {

constexpr double kRes = 0.1;
constexpr double kOx = -2.0;
constexpr double kOy = -2.0;
constexpr int kW = 40;
constexpr int kH = 40;
constexpr double kMaxDist = 2.0;

/// 解析场：到 (0,0) 一个点障碍的距离（截断到 max_dist）
double analyticRadial(double x, double y) {
  return std::min(std::hypot(x, y), kMaxDist);
}

/// 解析场：到 x=0 这条"墙"的距离（x<0 侧视为障碍内部 → 0）
double analyticHalfPlane(double x, double /*y*/) {
  return std::min(std::max(0.0, x), kMaxDist);
}

/// 按给定解析场生成采样（step 格采样一次，模拟 esdf_pub_step=2 的稀疏点云）
template <typename Fn>
std::vector<DistanceSample> makeSamples(double step_m, Fn fn,
                                        double max_d = kMaxDist) {
  std::vector<DistanceSample> s;
  for (double y = kOy; y <= kOy + kH * kRes; y += step_m) {
    for (double x = kOx; x <= kOx + kW * kRes; x += step_m) {
      const double d = fn(x, y);
      if (d > max_d + 1e-9)
        continue; // 真实 ESDF 只发布 max_dist 以内的格子
      s.push_back(DistanceSample{x, y, d});
    }
  }
  return s;
}

TEST(DistanceField, BuildFromSamplesRadial) {
  LocalDistanceField f;
  const auto samples = makeSamples(0.2, analyticRadial, 1.0);
  ASSERT_FALSE(samples.empty());
  ASSERT_TRUE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 2));
  EXPECT_TRUE(f.valid());
  EXPECT_GT(f.sampleCells(), 0u);

  // 采样点附近：双线性插值误差应该只有一个格子量级。
  // ⚠ 查询区必须限制在**采样覆盖范围内**（+ 填充半径）：“未覆盖 = 很远”
  // 是刻意设计的乐观策略，在那个区域里跟解析值比毫无意义。
  double worst = 0.0;
  int checked = 0;
  for (double x = 0.3; x <= 1.0; x += 0.037) {
    for (double y = -1.0; y <= 1.0; y += 0.041) {
      if (std::hypot(x, y) > 0.9)
        continue; // 超出“有采样”的圆（半径 1.0 m）
      const double got = f.distance(x, y);
      const double want = analyticRadial(x, y);
      worst = std::max(worst, std::fabs(got - want));
      ++checked;
    }
  }
  EXPECT_GT(checked, 100);
  std::printf("  [插值] 采样 0.2 m / 栅格 0.1 m，%d 个查询点最大误差 %.4f m\n",
              checked, worst);
  EXPECT_LT(worst, 0.10) << "径向场插值最大误差 " << worst << " m";
}

TEST(DistanceField, UncoveredIsFarNotFatal) {
  LocalDistanceField f;
  // 只有半径 0.8 m 以内有采样（模拟"3 m 之外不发点"的 esdf_2d）
  const auto samples = makeSamples(0.2, analyticRadial, 0.8);
  ASSERT_TRUE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 2));

  // ★ 关键语义：没有采样的地方必须报"很远"，报 0 会让 MPC 把开阔区当墙
  EXPECT_DOUBLE_EQ(f.distance(kOx + 0.05, kOy + 0.05), kMaxDist);
  EXPECT_DOUBLE_EQ(f.distance(1.5, 1.5), kMaxDist);
  // 图外同样乐观
  EXPECT_DOUBLE_EQ(f.distance(-50.0, 0.0), kMaxDist);
}

TEST(DistanceField, SparseSamplesGetFilled) {
  LocalDistanceField raw, filled;
  const auto samples = makeSamples(0.2, analyticRadial, 1.0); // 0.2 m 间隔
  ASSERT_TRUE(
      raw.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 0));
  ASSERT_TRUE(
      filled.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 2));

  EXPECT_EQ(raw.sampleCells(), filled.sampleCells());
  EXPECT_GT(filled.filledCells(), 0u)
      << "0.1 m 栅格 + 0.2 m 采样，必然有空格要补";
  EXPECT_LE(filled.maxFillUsed(), 2u);

  // 不填充时：采样缝里的值会被"很远"污染（这正是要填充的原因）
  const double x = 0.3, y = 0.1; // 落在两个采样之间
  const double raw_v = raw.distance(x, y);
  const double filled_v = filled.distance(x, y);
  const double want = analyticRadial(x, y);
  EXPECT_GT(raw_v - want, 0.2) << "未填充时应该明显偏大（被 max_dist 拉高）";
  EXPECT_LT(std::fabs(filled_v - want), 0.10)
      << "填充后应接近解析值：got " << filled_v << " want " << want;
}

TEST(DistanceField, GradientPointsAwayFromObstacle) {
  LocalDistanceField f;
  const auto samples = makeSamples(0.1, analyticHalfPlane, kMaxDist);
  ASSERT_TRUE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 1));

  // 障碍在 x ≤ 0 侧：d = x，所以"远离障碍"的方向是 +x
  for (double y : {-0.7, 0.0, 0.55}) {
    double gx = 0.0, gy = 0.0;
    ASSERT_TRUE(f.gradient(0.8, y, gx, gy)) << "y=" << y;
    EXPECT_NEAR(gx, 1.0, 1e-6) << "y=" << y;
    EXPECT_NEAR(gy, 0.0, 1e-6) << "y=" << y;
  }
  // 极深开阔处（场已被截断成常数）没有可用方向 → 必须如实返回 false
  double gx = 0.0, gy = 0.0;
  EXPECT_FALSE(f.gradient(kOx + 0.5, kOy + 0.5, gx, gy));
}

TEST(DistanceField, BuildFromClearanceMatchesBruteForce) {
  // 40x40 @0.05，正中间一格障碍 → 与暴力最近距离对拍
  CostMap2D map;
  const int w = 40, h = 40;
  const double res = 0.05;
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  const int ox = 20, oy = 20;
  data[static_cast<std::size_t>(oy) * w + ox] = 100;
  ASSERT_TRUE(map.set(w, h, res, 0.0, 0.0, 0.0, data, "map"));

  LocalDistanceField f;
  ASSERT_TRUE(f.buildFromClearance(map, 80, false, 1.0));

  double wx = 0.0, wy = 0.0;
  map.gridToWorld(ox, oy, wx, wy);
  EXPECT_NEAR(f.distance(wx, wy), 0.0, 1e-9);
  double wx2 = 0.0, wy2 = 0.0;
  map.gridToWorld(ox + 4, oy + 3, wx2, wy2);
  const double want = std::hypot(4.0, 3.0) * res; // 5 格
  EXPECT_NEAR(f.distance(wx2, wy2), want, 1e-6);
  EXPECT_NEAR(f.distance(wx + 0.30, wy), 0.30, 1e-6);
}

TEST(DistanceField, RejectsBadInput) {
  LocalDistanceField f;
  const auto samples = makeSamples(0.2, analyticRadial, 1.0);

  EXPECT_FALSE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, {}, kMaxDist, 2))
      << "没有采样时必须失败（不能返回一张'全是 max_dist'的假场）";
  EXPECT_FALSE(f.buildFromSamples(kOx, kOy, 0.0, kW, kH, samples, kMaxDist, 2));
  EXPECT_FALSE(f.buildFromSamples(kOx, kOy, kRes, 0, kH, samples, kMaxDist, 2));
  EXPECT_FALSE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, 0.0, 2));

  // 采样全在图外 → 也失败（没有任何信息可用）
  std::vector<DistanceSample> outside{{100.0, 100.0, 1.0}};
  EXPECT_FALSE(
      f.buildFromSamples(kOx, kOy, kRes, kW, kH, outside, kMaxDist, 2));
}

TEST(DistanceField, MaxDistanceClipApplied) {
  LocalDistanceField f;
  const auto samples = makeSamples(0.1, analyticHalfPlane, kMaxDist);
  ASSERT_TRUE(f.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, 0.6, 1));
  // 内部用 float 存距离（省内存），所以容差按 float 精度给（~1e-7 相对误差）
  constexpr double kFloatEps = 1e-6;
  EXPECT_NEAR(f.distance(1.5, 0.0), 0.6, kFloatEps);
  EXPECT_LE(f.distance(1.5, 0.0), 0.6 + kFloatEps);
  int ix = 0, iy = 0;
  ASSERT_TRUE(f.worldToGrid(1.5, 0.0, ix, iy));
  EXPECT_LE(f.at(ix, iy), 0.6 + kFloatEps);
  // 超过 max_dist 的采样要被截断（不能报出 1.2 m 这种“比上限远”的值）
  EXPECT_LE(f.at(0, 0), 0.6 + kFloatEps);
}

TEST(DistanceField, FuseForbiddenZone) {
  // 局部用的是感知滑动窗 + ESDF，**里面没有区域** ⇒ 必须把禁行区并进来，
  // 否则车能贴着/开进禁行区（今天只有订阅全局图的模块才遵守它）。
  // 语义：逐格取 min(现有距离, max(0, 点到多边形距离 − inflate))。
  // ① 没有禁行区 ⇒ 场必须**一字不改**（最关键：空集不能变成“到处是 0”）
  {
    LocalDistanceField g;
    ASSERT_TRUE(g.buildFree(kOx, kOy, kRes, kW, kH, kMaxDist));
    ZoneSet empty;
    EXPECT_EQ(g.fuseForbidden(empty, 0.05), 0u);
    int ix = 0, iy = 0;
    ASSERT_TRUE(g.worldToGrid(0.5, 0.5, ix, iy));
    EXPECT_NEAR(g.at(ix, iy), kMaxDist, 1e-6) << "空区域集把距离场改掉了";
  }

  LocalDistanceField f;
  ASSERT_TRUE(f.buildFree(kOx, kOy, kRes, kW, kH, kMaxDist));

  // ② 一个 1×1 m 的禁行区（x∈[0,1], y∈[0,1]），不膨胀
  MapZone z;
  z.name = "Z_test";
  z.type = ZoneType::kForbidden;
  z.polygon = {Pose2D{0.0, 0.0, 0}, Pose2D{1.0, 0.0, 0}, Pose2D{1.0, 1.0, 0},
               Pose2D{0.0, 1.0, 0}};
  ZoneSet zs;
  ASSERT_TRUE(zs.adopt({z}));
  ASSERT_EQ(zs.forbiddenCount(), 1u);

  const std::size_t changed = f.fuseForbidden(zs, 0.0);
  EXPECT_GT(changed, 0u);
  EXPECT_NEAR(f.distance(0.5, 0.5), 0.0, 1e-6) << "区内应该是 0（= 障碍）";
  EXPECT_NEAR(f.distance(0.5, 1.2), 0.2, 0.02) << "边界外 0.2 m 应该 ≈0.2";
  EXPECT_NEAR(f.distance(0.5, 1.5), 0.5, 0.02) << "边界外 0.5 m 应该 ≈0.5";
  // 远于 band（1.0 m）的格子保持原值 —— 反正不影响 MPC 的 0.25/0.45 阈值
  EXPECT_NEAR(f.distance(0.5, 1.0 + 1.6), kMaxDist, 0.05);

  // ③ 膨胀量要真的扣掉
  LocalDistanceField f2;
  ASSERT_TRUE(f2.buildFree(kOx, kOy, kRes, kW, kH, kMaxDist));
  f2.fuseForbidden(zs, 0.05);
  EXPECT_NEAR(f2.distance(0.5, 1.2), 0.15, 0.02);
  EXPECT_NEAR(f2.distance(0.5, 1.02), 0.0, 0.03)
      << "膨胀带内应该已经是 0（不能等到进区才生效）";

  // ④ 只能**压小**不能压大：ESDF 本来就比区域近的地方必须保持原值
  //    （取格心 0.55/1.55 —— `distance()` 是双线性插值，格心处才等于格值）
  LocalDistanceField f3;
  std::vector<DistanceSample> samples = {DistanceSample{0.5, 1.5, 0.05}};
  ASSERT_TRUE(
      f3.buildFromSamples(kOx, kOy, kRes, kW, kH, samples, kMaxDist, 1));
  f3.fuseForbidden(zs, 0.0);
  EXPECT_NEAR(f3.distance(0.55, 1.55), 0.05, 0.02)
      << "融合把原本更近的障碍距离改大了（ESDF 0.05 应该赢过区域的 0.55）";
}

} // namespace
} // namespace pnc_2d
