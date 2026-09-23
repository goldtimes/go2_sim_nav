// ClearanceField（精确欧氏距离变换 / EDT）单元测试：不依赖 ROS。
//
// 为什么值得单测：footprint
// 快路径的两个阈值一个要距离**下界**、一个要**上界**，
// 近似距离（棋盘/倒角）只能满足其中一边 → 会引入误判。这里用**暴力法**把"精确"
// 这件事钉死，并验证上下界确实把"到最近致命格**区域**的距离"夹住。
//
// 语义（与实现一致，测的时候必须按这个来）：
//   · distanceToLethal = 到最近致命格**中心**的距离（格单位 × resolution）
//   · 致命 = raw < 0 ? unknown_as_occupied : raw >= hard_threshold
//   · **图外不算致命**（EDT 只在地图内建）；注意这与 FootprintCollisionChecker
//     的 cellLethal（图外=致命，保守）不同，别混
//   · lowerBoundM = max(0, d − margin)，upperBoundM = d + margin，
//     margin = √2/2 × resolution（格中心到角的距离）

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"

namespace pnc_2d {
namespace {

constexpr int kHard = 80;
constexpr double kRes = 0.10;
/// 精度门限：实现内部用 float（平方距离可到 ~10^5），实测误差 ~5e-9 m；
/// 取 1 µm（比 0.05 m 分辨率小 5 个数量级）既不会被浮点噪声误伤，
/// 又足以证明"不是棋盘/倒角近似"（那些误差是**厘米级**）
constexpr double kEpsM = 1e-6;

std::shared_ptr<CostMap2D>
makeMap(int w, int h, const std::vector<int8_t> &data, double res = kRes) {
  auto m = std::make_shared<CostMap2D>();
  if (!m->set(w, h, res, 0.0, 0.0, 0.0, data, "map"))
    return nullptr;
  return m;
}

/// 随机图：occ 比例占据；unk 比例的未知格（-1）
std::shared_ptr<CostMap2D> randomMap(int w, int h, double res, double occ,
                                     double unk, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dr(0.0, 1.0);
  std::vector<int8_t> data(static_cast<std::size_t>(w) * h, 0);
  for (auto &v : data) {
    const double r = dr(rng);
    v = (r < occ) ? 100 : ((r < occ + unk) ? -1 : 0);
  }
  return makeMap(w, h, data, res);
}

bool isLethal(int8_t raw, bool unknown_as_occupied) {
  return (raw < 0) ? unknown_as_occupied : (raw >= static_cast<int8_t>(kHard));
}

/// 暴力：到最近致命格**中心**的距离 [m]；没有致命格返回 -1
double bruteToCenter(const CostMap2D &m, int x, int y, bool unk_as_occ) {
  double best = -1.0;
  for (int j = 0; j < m.height(); ++j) {
    for (int i = 0; i < m.width(); ++i) {
      if (!isLethal(m.rawValue(i, j), unk_as_occ))
        continue;
      const double d =
          std::hypot(static_cast<double>(i - x), static_cast<double>(j - y)) *
          m.resolution();
      best = (best < 0.0) ? d : std::min(best, d);
    }
  }
  return best;
}

/// 暴力：到最近致命格**区域**(AABB) 的距离 [m]；没有致命格返回 -1
double bruteToRegion(const CostMap2D &m, int x, int y, bool unk_as_occ) {
  double best = -1.0;
  const double half = 0.5 * m.resolution();
  for (int j = 0; j < m.height(); ++j) {
    for (int i = 0; i < m.width(); ++i) {
      if (!isLethal(m.rawValue(i, j), unk_as_occ))
        continue;
      const double dx = std::max(0.0, std::abs(i - x) * m.resolution() - half);
      const double dy = std::max(0.0, std::abs(j - y) * m.resolution() - half);
      const double d = std::hypot(dx, dy);
      best = (best < 0.0) ? d : std::min(best, d);
    }
  }
  return best;
}

// --------------------------------------------------------------- 手算小算例
TEST(ClearanceField, HandCheckedTinyMap) {
  // 5x1，两端障碍：中心格到最近障碍 = 2 格 = 0.2 m
  auto m = makeMap(5, 1, {100, 0, 0, 0, 100});
  ASSERT_NE(m, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*m, kHard, false));
  EXPECT_EQ(cf.lethalCount(), 2u);
  EXPECT_NEAR(cf.distanceToLethal(2, 0), 2.0 * kRes, kEpsM);
  EXPECT_NEAR(cf.distanceToLethal(1, 0), 1.0 * kRes, kEpsM);
  EXPECT_NEAR(cf.distanceToLethal(0, 0), 0.0, kEpsM); // 自己就是障碍

  // 单点障碍在 (1,3)：查 (3,1) → 对角 2√2 格
  std::vector<int8_t> d(25, 0);
  d[3 * 5 + 1] = 100;
  auto m2 = makeMap(5, 5, d);
  ASSERT_NE(m2, nullptr);
  ClearanceField cf2;
  ASSERT_TRUE(cf2.build(*m2, kHard, false));
  EXPECT_NEAR(cf2.distanceToLethal(3, 1), 2.0 * std::sqrt(2.0) * kRes, kEpsM);
  EXPECT_NEAR(cf2.distanceToLethal(1, 3), 0.0, kEpsM);
  EXPECT_NEAR(cf2.distanceToLethal(1, 1), 2.0 * kRes, kEpsM);
}

// --------------------------------------------------- 与暴力法完全一致（5
// 组随机）
TEST(ClearanceField, MatchesBruteForceExactly) {
  const int w = 40, h = 30;
  for (unsigned seed = 1; seed <= 5; ++seed) {
    auto m = randomMap(w, h, kRes, 0.15, 0.10, seed);
    ASSERT_NE(m, nullptr);
    ClearanceField cf;
    ASSERT_TRUE(cf.build(*m, kHard, false));
    double worst = 0.0;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        worst = std::max(worst, std::abs(cf.distanceToLethal(x, y) -
                                         bruteToCenter(*m, x, y, false)));
      }
    }
    EXPECT_LT(worst, kEpsM)
        << "seed " << seed << " 与暴力法不符（EDT 不是精确欧氏距离？）";
    std::printf("      [暴力比对] seed %u：最大误差 %.3e m\n", seed, worst);
  }
}

// -------------------------------------------------
// 上下界必须夹住"到区域"的距离
TEST(ClearanceField, BoundsBracketRegionDistance) {
  const int w = 30, h = 24;
  auto m = randomMap(w, h, kRes, 0.12, 0.0, 7);
  ASSERT_NE(m, nullptr);
  ClearanceField cf;
  ASSERT_TRUE(cf.build(*m, kHard, false));

  double worst_low = 0.0, worst_high = 0.0, tight = 1e18;
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const double region = bruteToRegion(*m, x, y, false);
      const double lo = cf.lowerBoundM(x, y);
      const double hi = cf.upperBoundM(x, y);
      worst_low = std::max(worst_low, lo - region);   // 下界越过真值 = 误判安全
      worst_high = std::max(worst_high, region - hi); // 上界没盖住真值
      tight = std::min(tight, hi - lo);
    }
  }
  EXPECT_LE(worst_low, 1e-9) << "下界越过真值（会让快路径误判'安全'）";
  EXPECT_LE(worst_high, 1e-9) << "上界未覆盖真值";
  EXPECT_GT(tight, 0.0);
  std::printf("      [上下界] 违反量 lo %.3e / hi %.3e m，最窄带宽 %.3f m\n",
              worst_low, worst_high, tight);
}

// ------------------------------------------------------------------ 未知格策略
TEST(ClearanceField, UnknownPolicy) {
  // 左半空闲、右半未知，中间没有真障碍
  std::vector<int8_t> d(20 * 4, 0);
  for (int y = 0; y < 4; ++y)
    for (int x = 10; x < 20; ++x)
      d[y * 20 + x] = -1;
  auto m = makeMap(20, 4, d);
  ASSERT_NE(m, nullptr);

  ClearanceField optimistic; // 未知 = 空闲（MPC 要的语义）
  ASSERT_TRUE(optimistic.build(*m, kHard, false));
  EXPECT_EQ(optimistic.lethalCount(), 0u);
  EXPECT_GT(optimistic.distanceToLethal(2, 2), 1e6) << "无致命格时距离应饱和";

  ClearanceField conservative; // 未知 = 障碍（全局图那套策略）
  ASSERT_TRUE(conservative.build(*m, kHard, true));
  EXPECT_EQ(conservative.lethalCount(), 40u);
  EXPECT_NEAR(conservative.distanceToLethal(9, 0), 1.0 * kRes, kEpsM);
}

// ------------------------------------------------------------- 耗时（信息 +
// 粗门限）
TEST(ClearanceField, BuildTimeOnRealisticSizes) {
  struct Case {
    int w, h;
    double res;
    int trials;
    const char *name;
  };
  const Case cases[] = {
      {607, 307, 0.05, 10, "全局 607x307 @0.05（186k 格）"},
      {80, 80, 0.05, 200, "局部 4m x 4m @0.05（6.4k 格）"},
      {40, 40, 0.10, 500, "局部 4m x 4m @0.10（1.6k 格）"},
  };
  for (const auto &c : cases) {
    auto m = randomMap(c.w, c.h, c.res, 0.065, 0.0, 42);
    ASSERT_NE(m, nullptr);
    ClearanceField cf;
    double sum = 0.0;
    for (int i = 0; i < c.trials; ++i) {
      const auto t0 = std::chrono::steady_clock::now();
      cf.build(*m, kHard, false);
      const auto t1 = std::chrono::steady_clock::now();
      sum += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    const double ms = sum / c.trials;
    std::printf("      [耗时] %s：%.3f ms/次\n", c.name, ms);
    // 粗门限：只为抓"退化成 O(N·M)"这类灾难性回归（全局实测 ~5 ms）
    EXPECT_LT(ms, 50.0) << c.name << " 慢得离谱";
  }
}

} // namespace
} // namespace pnc_2d
