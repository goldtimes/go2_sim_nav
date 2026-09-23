// `dilateOccupied()` 的单测（map_server 的第一个单测）。
//
// 为什么要测它：这是"几何 + 索引"的活，写错了**不会报错**，只会让图看起来还是
// 一张图 —— 而下游（规划器/RViz/任何只做点判定的消费者）的行为会静默变化：
//   · 一轮轮往外扩 ⇒ 膨胀量变成 k·半径，通道被吃掉、规划失败；
//   · 用方阵代替圆盘 ⇒ 斜角多出 (√2−1)·r，看起来"四个方向半径不一样"；
//   · 忘了 y=0 在**地图底部** ⇒ 上下颠倒（本仓库最经典的那类 bug）。
//
// 圆盘半径按**格数**取整：cells = ceil(radius / resolution)，覆盖判据是
// `dx² + dy² ≤ cells²`（圆心在格心）。因此：
//   cells=1 ⇒ 5 格（**十字**：对角格中心距 √2·res > res ⇒ 不在盘内）
//   cells=2 ⇒ 13 格（含对角 (±1,±1)，不含 (±2,±1)、(±2,±2)）
// 想要连对角一起盖住，半径必须 > 1 格（例：0.05 m 分辨率下用 0.06 m）。

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "map_server/map_io.hpp"

namespace map_server {
namespace {

/// 造一张 w×h 的空图（0 = 空闲），可指定若干占据格（100）
std::vector<int8_t> makeGrid(int w, int h,
                             const std::vector<std::pair<int, int>> &occ = {}) {
  std::vector<int8_t> d(static_cast<std::size_t>(w) * h, 0);
  for (const auto &[x, y] : occ)
    d[static_cast<std::size_t>(y) * w + x] = 100;
  return d;
}

int8_t at(const std::vector<int8_t> &d, int w, int x, int y) {
  return d[static_cast<std::size_t>(y) * w + x];
}

std::size_t countOcc(const std::vector<int8_t> &d, int thresh = 50) {
  std::size_t n = 0;
  for (int8_t v : d)
    if (v >= thresh)
      ++n;
  return n;
}

// ------------------------------------------------------------------ 基本行为

TEST(DilateOccupied, ZeroOrNegativeRadiusIsNoOp) {
  auto d = makeGrid(9, 9, {{4, 4}});
  const auto before = d;
  EXPECT_EQ(dilateOccupied(d, 9, 9, 0.1, 0.0), 0u);
  EXPECT_EQ(dilateOccupied(d, 9, 9, 0.1, -1.0), 0u);
  EXPECT_EQ(d, before) << "关掉时必须一格都不改";
  // 非法几何也不能改数据（宁可什么都不做，也不要写坏图）
  EXPECT_EQ(dilateOccupied(d, 0, 9, 0.1, 0.5), 0u);
  EXPECT_EQ(dilateOccupied(d, 9, 9, 0.0, 0.5), 0u);
  // 尺寸与数据不匹配（10×9=90 ≠ 81 格）⇒ 不动：否则会越界写
  EXPECT_EQ(dilateOccupied(d, 10, 9, 0.1, 0.5), 0u);
  EXPECT_EQ(d, before);
}

TEST(DilateOccupied, OneCellRadiusIsPlusShape) {
  // 0.1 m/格、半径 0.1 m ⇒ 1 格 ⇒ 十字 5 格（对角中心距 0.141 m > 0.1 m）
  auto d = makeGrid(9, 9, {{4, 4}});
  EXPECT_EQ(dilateOccupied(d, 9, 9, 0.1, 0.1), 4u) << "十字：4 个新格";
  EXPECT_EQ(countOcc(d), 5u);
  EXPECT_GE(at(d, 9, 3, 4), 50);
  EXPECT_GE(at(d, 9, 5, 4), 50);
  EXPECT_GE(at(d, 9, 4, 3), 50);
  EXPECT_GE(at(d, 9, 4, 5), 50);
  EXPECT_LT(at(d, 9, 3, 3), 50) << "对角格**不在** 1 格圆盘内（这是圆盘语义）";
}

TEST(DilateOccupied, TwoCellRadiusIncludesDiagonals) {
  // 0.1 m/格、半径 0.2 m ⇒ 2 格 ⇒ 13 格（含 (±1,±1)，不含 (±2,±1) / (±2,±2)）
  auto d = makeGrid(11, 11, {{5, 5}});
  EXPECT_EQ(dilateOccupied(d, 11, 11, 0.1, 0.2), 12u) << "13 格盘，减去原障碍";
  EXPECT_EQ(countOcc(d), 13u);
  // 四个轴向上到 ±2 格
  EXPECT_GE(at(d, 11, 3, 5), 50);
  EXPECT_GE(at(d, 11, 7, 5), 50);
  EXPECT_GE(at(d, 11, 5, 3), 50);
  EXPECT_GE(at(d, 11, 5, 7), 50);
  // 对角 (±1,±1)（1+1=2 ≤ 4 ⇒ 在盘内）
  EXPECT_GE(at(d, 11, 4, 4), 50);
  EXPECT_GE(at(d, 11, 6, 6), 50);
  // ★ 圆盘 vs 方阵的分界：这些格子方阵会盖、圆盘不盖
  EXPECT_LT(at(d, 11, 7, 6), 50) << "(2,1)：4+1=5 > 4";
  EXPECT_LT(at(d, 11, 7, 7), 50) << "(2,2)：8 > 4";
}

TEST(DilateOccupied, DoesNotChainExpand) {
  // ★ 最容易写错的一条：只能对**原始**占据集合盖章。
  //   边扩边写 ⇒ 半径 r 会滚成 k·r（这里一圈 1 格会滚满整张图）。
  auto d = makeGrid(21, 21, {{10, 10}});
  dilateOccupied(d, 21, 21, 0.1, 0.1); // 1 格半径
  EXPECT_EQ(countOcc(d), 5u) << "1 格半径只应得到十字 5 格";
  EXPECT_LT(at(d, 21, 10, 8), 50) << "两格外不该被覆盖";
  EXPECT_LT(at(d, 21, 8, 8), 50) << "斜向两格更不该";
}

TEST(DilateOccupied, RadiusSmallerThanCellRoundsUpToOne) {
  // 半径小于一格（0.02 m @ 0.05 m）：按 ceil 取整 ⇒ 仍按 1 格处理（十字）。
  // 这是**有意**的：分辨率以下的膨胀没有意义，但"至少贴一圈"才能真正影响
  // 那些只做点判定的消费者。
  auto d = makeGrid(9, 9, {{4, 4}});
  EXPECT_EQ(dilateOccupied(d, 9, 9, 0.05, 0.02), 4u);
  EXPECT_GE(at(d, 9, 3, 4), 50);
  EXPECT_GE(at(d, 9, 5, 4), 50);
}

TEST(DilateOccupied, RespectsMapBorder) {
  // 角上的障碍：不能越界写（越界会静默改到别的格，症状是"图边缘出现随机墙"）
  auto d = makeGrid(6, 6, {{0, 0}, {5, 5}});
  dilateOccupied(d, 6, 6, 0.1, 0.2);
  // (0,0) 的 2 格盘在界内 6 格：(0,0)+(1,0)+(2,0)+(0,1)+(1,1)+(0,2)
  // (5,5) 同理 6 格；两盘不重叠 ⇒ 12
  EXPECT_EQ(countOcc(d), 12u);
  EXPECT_LT(at(d, 6, 3, 0), 50) << "远端不该被碰到（防越界写错行/列）";
  EXPECT_LT(at(d, 6, 0, 3), 50);
}

TEST(DilateOccupied, SkipsAlreadyOccupiedAndCountsOnlyNew) {
  auto d = makeGrid(7, 7, {{1, 1}, {5, 5}});
  // 两个十字共 10 格，其中 2 格已是障碍 ⇒ 新增 8
  EXPECT_EQ(dilateOccupied(d, 7, 7, 0.1, 0.1), 8u);
  EXPECT_EQ(countOcc(d), 10u);
}

TEST(DilateOccupied, UnknownCellsBecomeOccupied) {
  // -1（未知）也会被覆盖成占据：膨胀层的语义是"这些格不可通行"，
  // "未知怎么算"是消费方的事（common.unknown_as_occupied）。
  std::vector<int8_t> d(25, -1);
  d[12] = 100; // 中心
  EXPECT_EQ(dilateOccupied(d, 5, 5, 0.1, 0.1), 4u);
  EXPECT_EQ(at(d, 5, 2, 1), 100);
  EXPECT_EQ(at(d, 5, 0, 0), -1) << "盘外的未知格保持未知";
}

TEST(DilateOccupied, ModifiesInPlaceWithExpectedRowOrder) {
  // y=0 在**地图底部**（与 OccupancyGrid 一致）：给一个只有底部一行障碍的图，
  // 膨胀必须只影响底部两行 —— 如果行序搞反，症状是图上下颠倒。
  auto d = makeGrid(5, 5);
  for (int x = 0; x < 5; ++x)
    d[static_cast<std::size_t>(0) * 5 + x] = 100; // 第 0 行（底）
  dilateOccupied(d, 5, 5, 0.1, 0.1);
  for (int x = 0; x < 5; ++x) {
    EXPECT_GE(at(d, 5, x, 0), 50) << "底行保持占据";
    EXPECT_GE(at(d, 5, x, 1), 50) << "上一行被膨胀覆盖";
    EXPECT_LT(at(d, 5, x, 2), 50) << "再往上一行不该被碰到";
  }
}

} // namespace
} // namespace map_server
