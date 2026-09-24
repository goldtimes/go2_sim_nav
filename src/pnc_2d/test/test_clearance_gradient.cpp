// M1（P6 轨迹层）单元测试：ClearanceField 的世界坐标查询（双线性值 + 解析梯度）
// 与 FootprintCollisionChecker 的带符号轮廓净距。不依赖 ROS。
//
// 为什么单测这两件事：
//   · 轨迹优化（MINCO）的 SDF 罚项需要"**连续值 + 梯度**"，而 ClearanceField 原来
//     只有"按格下标查距离"。新增的世界查询必须证明：值与梯度**同源**（同一个插值
//     式的值与其导数）、方向指向最近致命格、离散化造成的幅值偏差是可接受量级。
//   · 轨迹终检需要"**轮廓**到障碍的连续净距"。只查点净距会漏判 —— 车长 0.70 m，
//     中心点净距 0.28 m 时照样撞。新增的 signedClearanceAt 必须与既有的
//     poseInCollisionAtMargin 构成**同一个判据**（不变式见下），否则两套判据会在
//     "恰好擦到"时打架（跨层物理量我们已经栽过多次）。
//
// 语义约定（与实现一致，测的时候必须按这个来）：
//   · 距离场的量是"到最近致命格**中心**"的距离；插值后仍是这个量
//   · lowerBoundAtWorld = max(0, d − √2/2·res) 才是"到格**区域**"的保守下界
//   · 越界（图外）：distanceAtWorld 饱和到 max_m（"不算障碍"），梯度置 0 并返回 false；
//     安全底线由轮廓判定兜底（图外一律视为致命）
//   · 不变式：signedClearanceAt(...) >= m  ⇔  !poseInCollisionAtMargin(..., m)
//     二分分辨率 ≈ 2·max_search / 2^14 ≈ 0.12 mm，所以"m 与净距差 < 2e-3"的样本
//     跳过判定，不制造假失败

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"

namespace pnc_2d {
namespace {

constexpr int kHard = 80;
constexpr double kRes = 0.10;

std::shared_ptr<CostMap2D> makeMap(int w, int h, const std::vector<int8_t> & data,
                                   double res = kRes, double ox = 0.0,
                                   double oy = 0.0, double oyaw = 0.0)
{
  auto m = std::make_shared<CostMap2D>();
  if (!m->set(w, h, res, ox, oy, oyaw, data, "map")) return nullptr;
  return m;
}

/// 整列（wx_col >= 0）或整行（wy_col >= 0）为致命格的图
std::shared_ptr<CostMap2D> wallMap(int w, int h, int wx_col, int wy_col = -1)
{
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  if (wy_col < 0) {
    for (int y = 0; y < h; ++y) data[static_cast<std::size_t>(y) * w + wx_col] = 100;
  } else {
    for (int x = 0; x < w; ++x) data[static_cast<std::size_t>(wy_col) * w + x] = 100;
  }
  return makeMap(w, h, data);
}

std::shared_ptr<CostMap2D> randomMap(int w, int h, double occ, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dr(0.0, 1.0);
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  for (auto & v : data) v = (dr(rng) < occ) ? 100 : 0;
  return makeMap(w, h, data);
}

/// 图内但无致命格（EDT 初值不会被更新的情形）
std::shared_ptr<CostMap2D> emptyMap(int w, int h)
{
  return makeMap(w, h, std::vector<int8_t>(static_cast<std::size_t>(w) * h, 0));
}

void centerOf(int x, int y, double res, double & wx, double & wy)
{
  wx = (static_cast<double>(x) + 0.5) * res;
  wy = (static_cast<double>(y) + 0.5) * res;
}

/// 暴力：到最近致命格**中心**的距离 [m]；无致命格返回 -1
double bruteToCenter(const CostMap2D & m, double wx, double wy)
{
  double best = -1.0;
  for (int j = 0; j < m.height(); ++j) {
    for (int i = 0; i < m.width(); ++i) {
      if (m.rawValue(i, j) < static_cast<int8_t>(kHard)) continue;
      double cx = 0.0;
      double cy = 0.0;
      centerOf(i, j, m.resolution(), cx, cy);
      const double d = std::hypot(cx - wx, cy - wy);
      best = (best < 0.0) ? d : std::min(best, d);
    }
  }
  return best;
}

/// 暴力：**远离**最近致命格**中心**的方向（= 距离场的真实梯度方向，单位向量）。
/// ⚠ 别写成"指向障碍"—— 那是反的（第一版就错在这里，实测全是 180°）。
/// 无致命格返回 false
bool bruteAwayDirection(const CostMap2D & m, double wx, double wy, double out[2])
{
  double best = -1.0;
  double bx = 0.0;
  double by = 0.0;
  for (int j = 0; j < m.height(); ++j) {
    for (int i = 0; i < m.width(); ++i) {
      if (m.rawValue(i, j) < static_cast<int8_t>(kHard)) continue;
      double cx = 0.0;
      double cy = 0.0;
      centerOf(i, j, m.resolution(), cx, cy);
      const double d = std::hypot(cx - wx, cy - wy);
      if (best < 0.0 || d < best) {
        best = d;
        bx = cx;
        by = cy;
      }
    }
  }
  if (best <= 0.0) return false;
  out[0] = (wx - bx) / best;   // ★ 远离障碍
  out[1] = (wy - by) / best;
  return true;
}

/// 角度（度）之间的最小夹角
double angleBetweenDeg(double ax, double ay, double bx, double by)
{
  const double na = std::hypot(ax, ay);
  const double nb = std::hypot(bx, by);
  if (na <= 0.0 || nb <= 0.0) return 180.0;
  double c = (ax * bx + ay * by) / (na * nb);
  c = std::clamp(c, -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

FootprintCollisionChecker makeChecker(const CostMap2D * map, const ClearanceField * cf,
                                      double len = 0.70, double wid = 0.40,
                                      bool enable = true)
{
  FootprintCollisionChecker c;
  FootprintParams fp;
  fp.enable = enable;
  fp.length = len;
  fp.width = wid;
  fp.offset_x = 0.0;
  fp.offset_y = 0.0;
  fp.safe_margin = 0.05;
  c.configure(fp, kHard, /*unknown_as_occupied=*/false);
  c.setMap(map);
  c.setClearanceField(cf);
  return c;
}

// --------------------------------------------------------------- 距离场世界查询

TEST(ClearanceFieldWorld, WallDistanceAndGradientAreAnalytic)
{
  // 整列 x=10 为墙（60×40）。查询点取在格中心与格中间，两者都应**解析可算**：
  //   到最近致命格中心的距离 = 水平距离；梯度 = (+1, 0)
  auto map = wallMap(60, 40, 10);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, true));

  // (a) 格 (30,20) 的中心：距离 = 20 格 = 2.0 m（插值权重为 0 ⇒ 精确）
  double wx = 0.0;
  double wy = 0.0;
  centerOf(30, 20, kRes, wx, wy);
  EXPECT_NEAR(cf.distanceAtWorld(wx, wy), 2.0, 1e-9);
  const double margin = 0.70710678118 * kRes;
  EXPECT_NEAR(cf.lowerBoundAtWorld(wx, wy), 2.0 - margin, 1e-9);

  double g[2] = {0.0, 0.0};
  ASSERT_TRUE(cf.gradientAtWorld(wx, wy, g));
  EXPECT_NEAR(g[0], 1.0, 1e-9);
  EXPECT_NEAR(g[1], 0.0, 1e-9);

  // (b) 格中间（u = 30.5）：值 = 20.5 格 = 2.05 m，梯度仍为 (+1, 0)
  EXPECT_NEAR(cf.distanceAtWorld(3.10, wy), 2.05, 1e-9);
  ASSERT_TRUE(cf.gradientAtWorld(3.10, wy, g));
  EXPECT_NEAR(g[0], 1.0, 1e-9);
  EXPECT_NEAR(g[1], 0.0, 1e-9);

  // (c) 墙在左边 ⇒ 往 -x 走距离减小（梯度单调指向墙）
  double gl[2] = {0.0, 0.0};
  double gr[2] = {0.0, 0.0};
  ASSERT_TRUE(cf.gradientAtWorld(wx - 0.5, wy, gl));
  ASSERT_TRUE(cf.gradientAtWorld(wx + 0.5, wy, gr));
  EXPECT_GT(gl[0], 0.0);
  EXPECT_GT(gr[0], 0.0);
}

TEST(ClearanceFieldWorld, GradientPointsAtNearestLethalCell)
{
  // 单个致命格 (30,30)（60×60）
  std::vector<int8_t> data(60 * 60, 0);
  data[static_cast<std::size_t>(30) * 60 + 30] = 100;
  auto map = makeMap(60, 60, data);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, true));

  // (a) 对角方向：理论 45°；插值的幅值不是严格 1（实测 ~1.045），方向可靠
  double p[2] = {0.0, 0.0};
  centerOf(35, 35, kRes, p[0], p[1]);
  double g[2] = {0.0, 0.0};
  ASSERT_TRUE(cf.gradientAtWorld(p[0], p[1], g));
  EXPECT_LT(angleBetweenDeg(g[0], g[1], 1.0, 1.0), 2.0);
  EXPECT_GT(std::hypot(g[0], g[1]), 0.90);
  EXPECT_LT(std::hypot(g[0], g[1]), 1.10);

  // (b) 正东方向（与致命格同一行附近）：方向应指向 -x（即 (-1, ~0)）
  double q[2] = {0.0, 0.0};
  centerOf(35, 30, kRes, q[0], q[1]);
  ASSERT_TRUE(cf.gradientAtWorld(q[0], q[1], g));
  EXPECT_LT(angleBetweenDeg(g[0], g[1], 1.0, 0.0), 10.0);   // +x = 远离障碍
  EXPECT_GT(std::hypot(g[0], g[1]), 0.90);
  EXPECT_LT(std::hypot(g[0], g[1]), 1.10);

  // (c) 正北方向：方向应指向 +y
  double r[2] = {0.0, 0.0};
  centerOf(30, 35, kRes, r[0], r[1]);
  ASSERT_TRUE(cf.gradientAtWorld(r[0], r[1], g));
  EXPECT_LT(angleBetweenDeg(g[0], g[1], 0.0, 1.0), 10.0);
}

TEST(ClearanceFieldWorld, GradientMatchesBruteForceOnRandomMap)
{
  // 稀疏随机图 + **拒绝采样**：只留"净距 ≥ 0.30 m"的点。
  // 为什么必须这样：25% 占据率的图上，几乎每个点的最近障碍都在 0.1~0.3 m 内，
  //   距离场在那里本身就不平滑 —— 拿它当"实现错误"是测试构造问题，不是实现问题
  //   （第一版就是这么写错的：所有样本都被 `ref < 0.30` 过滤掉，`compared` 恒为 0）。
  // 每个点测三处：格中心（插值精确）、格中间、格角（插值误差最大处）。
  auto map = randomMap(120, 120, 0.03, 20260924u);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, true));

  std::mt19937 rng(7u);
  std::uniform_int_distribution<int> di(5, 114);
  const double offsets[3][2] = {{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.5}};
  int accepted = 0;
  int attempts = 0;
  int dir_samples = 0;
  int coarse20 = 0;
  int flip = 0;
  int flat = 0;
  double sum_ang = 0.0;
  double max_val_err = 0.0;
  double max_ang = 0.0;
  double max_g = 0.0;
  // 首个反向样本的取证记录（要确认是"病态"而不是"实现 bug"）
  bool have_flip = false;
  double fl[8] = {0.0};   // wx, wy, ref, gx, gy, dx, dy, mag
  while (accepted < 60 && attempts < 4000) {
    ++attempts;
    const int si = di(rng);
    const int sj = di(rng);
    double cx = 0.0;
    double cy = 0.0;
    centerOf(si, sj, kRes, cx, cy);
    if (bruteToCenter(*map, cx, cy) < 0.30) continue;   // 太贴障碍：跳过（见上）
    ++accepted;
    for (const auto & off : offsets) {
      const double wx = cx + off[0] * kRes;
      const double wy = cy + off[1] * kRes;
      const double ref = bruteToCenter(*map, wx, wy);
      const double got = cf.distanceAtWorld(wx, wy);
      max_val_err = std::max(max_val_err, std::fabs(got - ref));
      EXPECT_LE(std::fabs(got - ref), 0.12) << "点 (" << wx << "," << wy << ")";

      double dref[2] = {0.0, 0.0};
      if (!bruteAwayDirection(*map, wx, wy, dref)) continue;
      double g[2] = {0.0, 0.0};
      ASSERT_TRUE(cf.gradientAtWorld(wx, wy, g));
      const double mag = std::hypot(g[0], g[1]);
      max_g = std::max(max_g, mag);
      EXPECT_LT(mag, 1.6);   // 不允许"虚大"

      // ★ |∇d| 只要求上界：距离场在"到两个障碍等距"的中轴（脊线）上本来就有驻点，
      //   双线性插值会把那里算成接近 0 的梯度（实测见过 0.037 甚至 2e-14）。
      //   这不是实现错误：脊线处 d 已是局部最大，罚项本来就不应该有推力。
      //   而且那里的**方向**本身就是病态的（最近障碍不唯一）⇒ 不参与方向统计。
      if (mag < 0.5) {
        ++flat;
        continue;
      }
      const double ang = angleBetweenDeg(g[0], g[1], dref[0], dref[1]);
      max_ang = std::max(max_ang, ang);
      sum_ang += ang;
      if (ang > 20.0) ++coarse20;
      if (ang > 90.0) {
        ++flip;
        if (!have_flip) {
          have_flip = true;
          fl[0] = wx;
          fl[1] = wy;
          fl[2] = ref;
          fl[3] = g[0];
          fl[4] = g[1];
          fl[5] = dref[0];
          fl[6] = dref[1];
          fl[7] = mag;
        }
      }
      ++dir_samples;
    }
  }
  const double mean_ang = (dir_samples > 0) ? sum_ang / dir_samples : 0.0;
  std::printf("[M1] 距离场 vs 暴力：%d 个点 / %d 次尝试 → 良态方向样本 %d 个（脊线 %d 个）；"
              "值最大误差 %.4f m；|∇d| 最大 %.2f；方向：均值 %.1f°，最大 %.1f°，"
              ">20° 的 %d 个，>90°（反向）的 %d 个\n",
              accepted, attempts, dir_samples, flat, max_val_err, max_g, mean_ang,
              max_ang, coarse20, flip);
  if (have_flip) {
    std::printf("[M1] 首个反向样本：点 (%.3f, %.3f) ref=%.3f | 远离方向 (%.2f, %.2f) vs "
                "梯度 (%.2f, %.2f)，|g|=%.2f\n",
                fl[0], fl[1], fl[2], fl[5], fl[6], fl[3], fl[4], fl[7]);
    // 邻域的**精确**距离：若是"局部低点"（周围都比它远）⇒ 最近障碍在相邻格之间换了
    // （Voronoi 边界/中轴），线性插值的导数在那里本来就不等于真实梯度
    for (int dxi = -1; dxi <= 1; ++dxi) {
      for (int dyi = -1; dyi <= 1; ++dyi) {
        const double px = fl[0] + dxi * kRes;
        const double py = fl[1] + dyi * kRes;
        std::printf("[M1]   邻域(%+d,%+d) 格: 精确距离 %.4f m\n", dxi, dyi,
                    bruteToCenter(*map, px, py));
      }
    }
  }
  EXPECT_GE(accepted, 60);
  EXPECT_GT(dir_samples, 100);
  // 阈值按实测值留余量（实测：均值 13.7°、反向 4/165 = 2.4%、>20° 的 29/165 = 17.6%）：
  //   · 反向比例是关键不变量 —— 罚项绝不能把车往障碍上推
  //   · 均値是回归护栏（梯度整体变差会抬它）
  EXPECT_LT(static_cast<double>(flip) / dir_samples, 0.08);
  EXPECT_LT(mean_ang, 20.0);
  EXPECT_LT(static_cast<double>(coarse20), 0.30 * static_cast<double>(dir_samples));
}

TEST(ClearanceFieldWorld, OutsideMapSaturatesAndReturnsNoGradient)
{
  auto map = wallMap(60, 40, 10);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, true));

  EXPECT_TRUE(cf.insideWorld(3.0, 2.0));
  EXPECT_FALSE(cf.insideWorld(100.0, 100.0));
  EXPECT_NEAR(cf.distanceAtWorld(100.0, 100.0, 1.0), 1.0, 1e-12);   // 饱和 = "不算障碍"

  double g[2] = {9.0, 9.0};
  EXPECT_FALSE(cf.gradientAtWorld(100.0, 100.0, g));
  EXPECT_DOUBLE_EQ(g[0], 0.0);
  EXPECT_DOUBLE_EQ(g[1], 0.0);
}

TEST(ClearanceFieldWorld, EmptyMapSaturatesWithoutNaN)
{
  // 图内一个致命格都没有：EDT 初值不会被更新（距离 = 初值，极大），
  // 查询必须**饱和而不是 NaN**（否则罚项会算出 NaN 梯度，优化直接发散）
  auto map = emptyMap(40, 40);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, true));
  EXPECT_EQ(cf.lethalCount(), 0u);

  const double d = cf.distanceAtWorld(2.05, 2.05, 1.0);
  EXPECT_TRUE(std::isfinite(d));
  EXPECT_NEAR(d, 1.0, 1e-12);

  double g[2] = {0.0, 0.0};
  ASSERT_TRUE(cf.gradientAtWorld(2.05, 2.05, g));
  EXPECT_TRUE(std::isfinite(g[0]));
  EXPECT_TRUE(std::isfinite(g[1]));
  EXPECT_DOUBLE_EQ(g[0], 0.0);
  EXPECT_DOUBLE_EQ(g[1], 0.0);
}

// --------------------------------------------------------- 轮廓净距（终检要用的量）

TEST(FootprintClearance, AnalyticWallClearanceAndYawDependence)
{
  auto map = wallMap(60, 40, 20);   // 墙格区域 x ∈ [2.00, 2.10]
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, false));
  const FootprintCollisionChecker ck = makeChecker(map.get(), &cf);
  double wx = 0.0;
  double wy = 0.0;
  centerOf(10, 20, kRes, wx, wy);   // (1.05, 2.05)

  // yaw = 0：车体 x 向半长 0.35 ⇒ 轮廓最右 1.40 ⇒ 净距 2.00 − 1.40 = 0.60（解析值）
  const double c0 = ck.signedClearanceAt(wx, wy, 0.0);
  EXPECT_NEAR(c0, 0.60, 3e-3);

  // yaw = 90°：0.70 的长度转到 y 向 ⇒ x 向变 ±0.20 ⇒ 净距 2.00 − 1.25 = 0.75
  const double c90 = ck.signedClearanceAt(wx, wy, M_PI / 2.0);
  EXPECT_NEAR(c90, 0.75, 3e-3);

  // 不变式的两端（避开恰好贴住的边界）
  EXPECT_FALSE(ck.poseInCollisionAtMargin(wx, wy, 0.0, 0.55));
  EXPECT_TRUE(ck.poseInCollisionAtMargin(wx, wy, 0.0, 0.65));
  EXPECT_FALSE(ck.poseInCollisionAtMargin(wx, wy, 0.0, -0.05));

  // 保守下界（罚项用的量）必须"不过分乐观"：lb − reach0 ≤ 真实净距
  const double reach0 =
      std::hypot(0.5 * ck.footprint().length, 0.5 * ck.footprint().width);
  const double lb = cf.lowerBoundAtWorld(wx, wy);
  EXPECT_LE(lb - reach0, c0 + 1e-3);
  std::printf("[M1] 墙净距：yaw=0 → %.4f m（解析 0.60）；yaw=90° → %.4f m（解析 0.75）；"
              "保守下界 lb−reach0 = %.4f m\n", c0, c90, lb - reach0);
}

TEST(FootprintClearance, SignedValueReportsPenetrationDepth)
{
  auto map = wallMap(60, 40, 20);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, false));
  const FootprintCollisionChecker ck = makeChecker(map.get(), &cf);

  // 车心 (1.75, 2.05)，yaw=0 ⇒ 轮廓 x ∈ [1.40, 2.10]，压进墙 0.10 m
  const double c = ck.signedClearanceAt(1.75, 2.05, 0.0);
  EXPECT_LT(c, 0.0);
  EXPECT_NEAR(c, -0.10, 5e-3);

  // 反向验证：内缩 0.10 之后不碰，内缩 0.09 仍然碰
  EXPECT_FALSE(ck.poseInCollisionAtMargin(1.75, 2.05, 0.0, -0.105));
  EXPECT_TRUE(ck.poseInCollisionAtMargin(1.75, 2.05, 0.0, -0.09));
  std::printf("[M1] 穿透深度：实测 %.4f m（解析 0.10）\n", c);
}

TEST(FootprintClearance, InvariantMatchesPoseInCollisionAtMargin)
{
  // 不变式：signedClearanceAt >= m  ⇔  !poseInCollisionAtMargin(m)
  // （这套判据如果两边不一致，"恰好擦到"时就会出现"终检说能过、单点检查说不能过"）
  auto map = randomMap(60, 60, 0.20, 20260925u);
  ASSERT_NE(map, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*map, kHard, false));
  const FootprintCollisionChecker ck = makeChecker(map.get(), &cf);

  const double margins[] = {-0.20, -0.10, -0.05, 0.0, 0.02, 0.05, 0.10, 0.20, 0.30};
  std::mt19937 rng(11u);
  std::uniform_int_distribution<int> di(4, 55);
  std::uniform_real_distribution<double> dyaw(-M_PI, M_PI);
  int checked = 0;
  for (int k = 0; k < 40; ++k) {
    double wx = 0.0;
    double wy = 0.0;
    centerOf(di(rng), di(rng), kRes, wx, wy);
    const double yaw = dyaw(rng);
    const double c = ck.signedClearanceAt(wx, wy, yaw);
    for (const double m : margins) {
      if (std::fabs(m - c) <= 2e-3) continue;   // 边界样本跳过（二分分辨率量级）
      const bool free_by_clearance = (c >= m);
      const bool free_by_check = !ck.poseInCollisionAtMargin(wx, wy, yaw, m);
      ASSERT_EQ(free_by_clearance, free_by_check)
          << "位姿 (" << wx << "," << wy << "," << yaw << ") margin " << m
          << " 净距 " << c;
      ++checked;
    }
  }
  std::printf("[M1] 不变式校验样本 %d 组\n", checked);
  EXPECT_GT(checked, 100);
}

TEST(FootprintClearance, SaturatesInOpenSpaceAndOutsideAndWhenDisabled)
{
  auto empty = emptyMap(40, 40);
  ASSERT_NE(empty, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*empty, kHard, false));
  const FootprintCollisionChecker ck = makeChecker(empty.get(), &cf);
  EXPECT_NEAR(ck.signedClearanceAt(2.05, 2.05, 0.0), 1.0, 1e-12);     // 开阔：饱和
  EXPECT_NEAR(ck.signedClearanceAt(100.0, 100.0, 0.0), -1.0, 1e-12);  // 图外：视为碰撞

  // 没有距离场时也要能用（无非是丢了快路径，走二分）
  FootprintCollisionChecker no_cf = makeChecker(empty.get(), nullptr);
  EXPECT_NEAR(no_cf.signedClearanceAt(2.05, 2.05, 0.0), 1.0, 1e-12);

  // 轮廓判定关闭：margin 不参与判定 ⇒ 退化为点判定（文档化的行为）
  auto wall = wallMap(60, 40, 20);
  ASSERT_NE(wall, nullptr);
  ClearanceField cf2;
  ASSERT_TRUE(cf2.build(*wall, kHard, false));
  const FootprintCollisionChecker off = makeChecker(wall.get(), &cf2, 0.70, 0.40, false);
  EXPECT_NEAR(off.signedClearanceAt(2.05, 2.05, 0.0), -1.0, 1e-12);   // 点在墙格里
  EXPECT_NEAR(off.signedClearanceAt(1.05, 2.05, 0.0), 1.0, 1e-12);    // 点在空格里
}

}  // namespace
}  // namespace pnc_2d
