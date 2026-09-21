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

} // namespace
} // namespace pnc_2d
