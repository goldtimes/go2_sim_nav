// 地图区域层（禁行区 / 限速区）单元测试：不依赖 ROS。
//
// 重点：
//   1. 解析（有/无 zones 段、[x,y] 与 {x,y} 两种顶点写法、坏数据报错）
//   2. 点是否在多边形内（凸 / 凹 L 形）
//   3. 限速区取"最严"上限
//   4. **膨胀烧入**：禁行区按车体外接半径膨胀后落格，位置与数量都要对

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/map_zones.hpp"

namespace pnc_2d {
namespace {

const std::string kYaml =
    "frame_id: map\n"
    "zones:\n"
    "  - {name: hall, type: forbidden, polygon: [[4,4], [6,4], [6,6], [4,6]]}\n"
    "  - name: door\n"
    "    type: speed_limit\n"
    "    value: 0.3\n"
    "    polygon: [{x: 8, y: 0}, {x: 10, y: 0}, {x: 10, y: 2}, {x: 8, y: 2}]\n"
    "  - {name: narrow, type: speed_limit, value: 0.15,\n"
    "     polygon: [[9, 1], [11, 1], [11, 3], [9, 3]]}\n";

/// 15x15 m 全空闲地图（0.05 m）
std::shared_ptr<CostMap2D> openMap() {
  auto m = std::make_shared<CostMap2D>();
  std::vector<int8_t> data(static_cast<std::size_t>(300) * 300, 0);
  m->set(300, 300, 0.05, 0.0, 0.0, 0.0, data, "map");
  return m;
}

TEST(MapZones, ParsesZones) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  EXPECT_EQ(zs.zones().size(), 3u);
  EXPECT_EQ(zs.forbiddenCount(), 1u);
  EXPECT_EQ(zs.speedCount(), 2u);
  EXPECT_EQ(zs.zones()[0].name, "hall");
  EXPECT_EQ(zs.zones()[0].type, ZoneType::kForbidden);
  EXPECT_DOUBLE_EQ(zs.zones()[1].value, 0.3);
  std::printf("      [区域] %s\n", zs.summary().c_str());
}

TEST(MapZones, NoZonesSectionIsFine) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString("frame_id: map\nnodes: []\nedges: []\n", err))
      << err;
  EXPECT_TRUE(zs.empty());
  EXPECT_TRUE(zs.valid());
}

TEST(MapZones, BadDataRejected) {
  struct Case {
    std::string yaml;
    const char *why;
  };
  const std::vector<Case> bad = {
      {"zones: {a: 1}\n", "zones 不是序列"},
      {"zones:\n  - {name: x, type: forbidden, polygon: [[0,0],[1,1]]}\n",
       "顶点不足 3"},
      {"zones:\n  - {name: x, polygon: [[0,0],[1,1],\"bad\"]}\n",
       "顶点格式不对"},
  };
  for (const Case &c : bad) {
    ZoneSet zs;
    std::string err;
    EXPECT_FALSE(zs.loadFromString(c.yaml, err)) << c.why;
    EXPECT_FALSE(err.empty());
    std::printf("      [拒绝] %-12s → %s\n", c.why, err.c_str());
  }
}

TEST(MapZones, PointInPolygonConvexAndConcave) {
  const std::vector<Pose2D> square = {
      {0, 0, 0, false}, {4, 0, 0, false}, {4, 4, 0, false}, {0, 4, 0, false}};
  EXPECT_TRUE(pointInPolygon(2.0, 2.0, square));
  EXPECT_FALSE(pointInPolygon(4.5, 2.0, square));
  EXPECT_FALSE(pointInPolygon(-0.1, 2.0, square));

  // L 形（凹）：挖掉右上角
  const std::vector<Pose2D> lshape = {{0, 0, 0, false}, {4, 0, 0, false},
                                      {4, 2, 0, false}, {2, 2, 0, false},
                                      {2, 4, 0, false}, {0, 4, 0, false}};
  EXPECT_TRUE(pointInPolygon(1.0, 3.0, lshape));  // 竖条里
  EXPECT_TRUE(pointInPolygon(3.0, 1.0, lshape));  // 横条里
  EXPECT_FALSE(pointInPolygon(3.0, 3.0, lshape)); // 被挖掉的角
  EXPECT_NEAR(distanceToPolygon(3.0, 3.0, lshape), 1.0, 1e-9);
}

TEST(MapZones, SpeedZoneTakesStrictest) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  double limit = 99.0;
  EXPECT_TRUE(zs.inSpeedZone(9.0, 1.5, limit)); // 两区重叠 → 取更严的
  EXPECT_DOUBLE_EQ(limit, 0.15);
  EXPECT_TRUE(zs.inSpeedZone(9.0, 0.5, limit));
  EXPECT_DOUBLE_EQ(limit, 0.3);
  EXPECT_FALSE(zs.inSpeedZone(5.0, 5.0, limit)); // 禁行区不算限速区
  EXPECT_TRUE(zs.inForbidden(5.0, 5.0));
  EXPECT_FALSE(zs.inForbidden(7.0, 7.0));
}

TEST(MapZones, BurnForbiddenWithInflation) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  auto map = openMap();
  std::vector<int8_t> burned;
  const double r_circ = std::hypot(0.35 + 0.05, 0.20 + 0.05); // 0.472 m
  const std::size_t n = burnForbidden(zs, *map, r_circ, burned);
  std::printf("      [烧入] 膨胀 %.3f m → %zu 格（禁行区本体 %.0f 格）\n",
              r_circ, n, 2.0 * 2.0 / (0.05 * 0.05));

  auto at = [&](double x, double y) {
    const int cx = static_cast<int>(x / 0.05);
    const int cy = static_cast<int>(y / 0.05);
    return burned[static_cast<std::size_t>(cy) * 300 + cx];
  };
  EXPECT_EQ(at(5.0, 5.0), CostMap2D::kOccupied); // 区内
  EXPECT_EQ(at(3.6, 5.0),
            CostMap2D::kOccupied); // 沿边外 0.4 m < 0.472 → 膨胀后仍算禁行
  EXPECT_EQ(at(3.0, 5.0), 0);      // 外 1.0 m > 0.472 → 保留可通行
  EXPECT_EQ(at(3.0, 3.0), 0);      // 角外 1.41 m
  // 限速区不烧进栅格
  EXPECT_EQ(at(9.0, 1.0), 0);
  // 数量级检查：本体 1600 格 + 周边膨胀一圈（周长 8 m × 0.472 ≈ 3.8 m² ≈ 1510
  // 格）
  EXPECT_GT(n, 1600u);
  EXPECT_LT(n, 4200u);
}

TEST(MapZones, AdoptFromGeometryMatchesYamlRules) {
  // 消息 → 内部表示（map_server 的 ZoneArray 给局部/全局用）。
  // 校验规则必须与 YAML
  // 路径**同一套**，否则会出现"文件里合法、运行时消息里非法"
  // 这种分叉（同一个区域，全局说能过、局部说不能）。
  MapZone bad; // 顶点 < 3 → 丢弃
  bad.name = "bad";
  bad.type = ZoneType::kForbidden;
  bad.polygon = {Pose2D{0.0, 0.0, 0}, Pose2D{1.0, 0.0, 0}};

  MapZone sp; // 限速值非法 → 按 0.3 处理
  sp.name = "sp";
  sp.type = ZoneType::kSpeedLimit;
  sp.value = 0.0;
  sp.polygon = {Pose2D{2.0, 2.0, 0}, Pose2D{3.0, 2.0, 0}, Pose2D{3.0, 3.0, 0},
                Pose2D{2.0, 3.0, 0}};

  MapZone fb; // 禁行区：bbox 要自动算出来，value 强制 0
  fb.name = "fb";
  fb.type = ZoneType::kForbidden;
  fb.value = 5.0;
  fb.polygon = {Pose2D{4.0, 4.0, 0}, Pose2D{5.0, 4.0, 0}, Pose2D{5.0, 5.0, 0},
                Pose2D{4.0, 5.0, 0}};

  ZoneSet zs;
  EXPECT_TRUE(zs.adopt({bad, sp, fb}));
  EXPECT_EQ(zs.zones().size(), 2u) << "顶点 < 3 的区域应被丢弃";
  EXPECT_FALSE(zs.warnings().empty())
      << "丢弃/修正过就要有警告（否则静默不一致）";
  EXPECT_EQ(zs.forbiddenCount(), 1u);
  EXPECT_EQ(zs.speedCount(), 1u);
  double limit = 0.0;
  ASSERT_TRUE(zs.inSpeedZone(2.5, 2.5, limit));
  EXPECT_DOUBLE_EQ(limit, 0.3) << "非法限速值应按 0.3 处理（与 YAML 路径一致）";
  for (const MapZone &z : zs.zones()) {
    if (z.name != "fb")
      continue;
    EXPECT_DOUBLE_EQ(z.min_x, 4.0);
    EXPECT_DOUBLE_EQ(z.max_y, 5.0) << "bbox 必须自动算";
    EXPECT_DOUBLE_EQ(z.value, 0.0) << "禁行区的 value 应为 0";
  }
  // 全非法 ⇒ 视为无区域（行为回到自由空间），但仍返回 warnings
  ZoneSet zs2;
  EXPECT_FALSE(zs2.adopt({bad}));
  EXPECT_TRUE(zs2.empty());
  EXPECT_EQ(zs2.forbiddenCount(), 0u);

  // 报错点名（现场一眼看出是哪个区域）
  EXPECT_EQ(zs.forbiddenNameAt(4.5, 4.5), "fb");
  EXPECT_TRUE(zs.forbiddenNameAt(0.0, 0.0).empty());
}

TEST(MapZones, SpeedLimitAheadLooksForwardOnly) {
  // 局部用它算"进区前就要降到的速度帽"（前瞻 = 刹车距离）；
  // 全局用它算"这条路径上的任务限速"（前瞻 = 整条路径）。
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  // 一条沿 x 轴的折线：x=0 → 12（0.5 m 一点）
  std::vector<Pose2D> pts;
  for (double x = 0.0; x <= 12.0 + 1e-9; x += 0.5)
    pts.push_back(Pose2D{x, 1.5, 0.0});
  // 限速区在 x∈[8,10]（0.3）与 [9,11]（0.15，与前者重叠）

  // ① 从起点前瞻 1 m：还没到区 ⇒ 不限
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 0, 1.0), 0.0);
  // ② 前瞻到 x=8.5（8.5 m）⇒ 进入 0.3 的区
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 0, 8.6), 0.3);
  // ③ 前瞻盖住重叠段 ⇒ 取最严 0.15
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 0, 9.6), 0.15);
  // ④ 从进度下标 16（x=8.0）往后看 ⇒ 立刻命中
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 16, 0.4), 0.3);
  // ⑤ 已经过完限速区（x=11.5 之后）⇒ 不限（出区要能恢复，不能被永久压住）
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 23, 2.0), 0.0);
  // ⑥ lookahead<=0 = 只看起点那一点；空集/无速度区 = 不限
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 17, 0.0), 0.3);
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(ZoneSet{}, pts, 0, 1e9), 0.0);
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, {}, 0, 1e9), 0.0);
  // ⑦ from 越界要夹住（不能崩、不能漏最后一点）
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, pts, 999, 1e9), 0.0);
}

/// ★ 回归：路径**只有两个点**（全局自由空间规划就是这么发的）时，
/// 也必须看到位于两点**之间**的限速区。
///
/// 之前的实现只查"路径点"，于是 2 点路径等于只查起点与终点 ⇒ 中间的限速区被
/// 整个跳过。实测（2026-09-23 test_zones 段 3）：限速区就在起点前 0.7 m、
/// 前瞻算到 0.89 m，函数依旧返回 0，车以 0.30 m/s 直接穿区（Z4/Z5 失败）。
TEST(MapZones, SpeedLimitAheadResamplesSparsePath) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  // 只有起点与终点的一条直线（限速区 x∈[8,10]、[9,11] 都在中点之后）
  const std::vector<Pose2D> sparse{Pose2D{0.0, 1.5, 0.0},
                                   Pose2D{12.0, 1.5, 0.0}};
  // 照旧：前瞻不够长 ⇒ 不限（说明不是"无脑全程限速"）
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, sparse, 0, 1.0), 0.0);
  // 前瞻到区里 ⇒ 必须命中（旧实现这里是 0）
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, sparse, 0, 8.6), 0.3);
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, sparse, 0, 9.6), 0.15);
  // 步长不影响结果（0.25 默认 vs 更细/更粗都在同一区间内）
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, sparse, 0, 9.6, 0.1), 0.15);
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(zs, sparse, 0, 9.6, 1.0), 0.15);
  // ① 关键不变量：**前瞻必须够"从最大速度减到限速"用**。
  //    用当前速度算前瞻会形成自锁（车越慢前瞻越短 ⇒ 从不减速），
  //    所以节点用的是 v_max；这里直接断言 0.89 m 这个真实值够减到 0.15。
  const double need = (0.42 * 0.42 - 0.15 * 0.15) / (2.0 * 0.15);
  EXPECT_GT(speedLookahead(0.42, 0.15), need)
      << "前瞻必须 ≥ 从 v_max 减到限速所需的距离（否则进区时还超速）";
  // ② 自锁回归：**慢车**的前瞻也要覆盖住 0.7 m 外的限速区
  //    （旧实现用当前速度 0.26 ⇒ 0.53 m ⇒ 漏掉）
  EXPECT_LT(speedLookahead(0.26, 0.15), 0.7)
      << "这条正是踩过的坑：用当前速度算不够";
  EXPECT_GT(speedLookahead(0.42, 0.15), 0.7)
      << "用 v_max 算才够（节点就是这么用的）";
  // ③ 退化输入不炸
  EXPECT_DOUBLE_EQ(speedLookahead(0.0, 0.15), 0.3);
  EXPECT_DOUBLE_EQ(speedLookahead(0.4, 0.0), 0.3);
}

/// ★ 回归：控制周期里的前瞻必须从**车在路径上的投影点**开始。
///
/// 为什么不能传 `pass_index_`（路点下标）：自由空间路径常常只有 2 个点，
/// 而"最近路点"要等车走进 `pass_distance_` 容差才会前进 ⇒ 整段路下标都是 0，
/// 扫描窗口变成"路径起点往后 look 米"（与车在哪无关）。
/// 实测（2026-09-23 test_zones 段 3）：车开出限速区 1 m 了速度帽还在，
/// 全程 0.15 m/s 爬 31 s，"出区后恢复"永远不成立。
TEST(MapZones, SpeedLimitAheadUsesProjectionNotWaypointIndex) {
  ZoneSet zs;
  std::string err;
  ASSERT_TRUE(zs.loadFromString(kYaml, err)) << err;
  // 只有两个点的直线：限速区 x∈[8,10]
  const std::vector<Pose2D> two_pts{Pose2D{0.0, 1.5, 0.0},
                                    Pose2D{12.0, 1.5, 0.0}};

  // 车在 x=6.0（还没进区），前瞻 0.9 m ⇒ 不限（距离区还有 2.0 m）
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(zs, two_pts, 6.0, 1.5, 0.9), 0.0);
  // 车在 x=7.3（区前 0.7 m），前瞻 0.9 m ⇒ 触发（这就是仿真里差的那一步）
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(zs, two_pts, 7.3, 1.5, 0.9), 0.3);
  // 区内取到值（x=9.0 在 [8,10] 内，且重叠段 [9,11] 的 0.15 更严）
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(zs, two_pts, 9.0, 1.5, 0.0), 0.15);
  // 出区（x=11，已过 1 m）⇒ 立刻摘帽（前瞻只看**前方**）
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(zs, two_pts, 11.0, 1.5, 0.9), 0.0);
  // 侧向偏离时按投影算（车在轴上 8.5 处、侧向偏 1 m，投影仍落在区内；
  // 前瞻 0.5 m 会摸到更严的重叠段 0.15）
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(zs, two_pts, 8.5, 2.5, 0.5), 0.15);
  // 退化：单点路径 / 空集合 / 空路径
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAheadFromProjection(
                       zs, {Pose2D{9.0, 1.5, 0.0}}, 9.0, 1.5, 1.0),
                   0.15);
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(ZoneSet{}, two_pts, 9.0, 1.5, 1e9),
      0.0);
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAheadFromProjection(zs, {}, 9.0, 1.5, 1e9),
                   0.0);

  // ---- 用**仿真的真实几何**对照两个接口（限速区在路径起点前 0.7 m）----
  // 车的路径只有 2 点、长 4.25 m；限速区沿路径占 [0.7, 2.7]。
  ZoneSet sim;
  std::vector<MapZone> z;
  MapZone speed;
  speed.name = "slow";
  speed.type = ZoneType::kSpeedLimit;
  speed.value = 0.15;
  speed.polygon = {Pose2D{0.7, -1.5, 0.0}, Pose2D{2.7, -1.5, 0.0},
                   Pose2D{2.7, 1.5, 0.0}, Pose2D{0.7, 1.5, 0.0}};
  z.push_back(speed);
  ASSERT_TRUE(sim.adopt(z));
  const std::vector<Pose2D> sim_path{Pose2D{0.0, 0.0, 0.0},
                                     Pose2D{4.25, 0.0, 0.0}};
  // 区内（x=1.7）：两种算法都该给 0.15
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(sim, sim_path, 0, 0.89), 0.15);
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(sim, sim_path, 1.7, 0.0, 0.89), 0.15);
  // ★ 已经**开出区**（x=3.7，区后 1 m）：
  //   · 下标版（仿真里就是传 pass_index_=0）照样返回 0.15 ⇒ 帽摘不掉
  //     —— 实测车就这样一路 0.15 m/s 爬到终点，Z6"出区后恢复"永远不成立；
  //   · 投影版返回 0 ⇒ 恢复正常速度。
  EXPECT_DOUBLE_EQ(zoneSpeedLimitAhead(sim, sim_path, 0, 0.89), 0.15)
      << "旧接口（下标版）在出区后仍然命中，这正是 bug 的来源";
  EXPECT_DOUBLE_EQ(
      zoneSpeedLimitAheadFromProjection(sim, sim_path, 3.7, 0.0, 0.89), 0.0)
      << "投影版必须在出区后立刻摘帽";
}

} // namespace
} // namespace pnc_2d
