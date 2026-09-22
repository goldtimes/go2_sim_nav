// 路网（RouteGraph / RouteNetworkPlanner）单元测试：合成地图 + 临时
// yaml，不依赖 ROS。
//
// 重点验证（见 doc/pnc2d_restructure_plan.md §6 P2）：
//   1. yaml 解析与校验（含宽松 bool：one_way: 0 / 1）
//   2. 投影：任意点 → 最近通道（边/弧长/垂足/垂距）
//   3. **不切角**：路线必须贴着画好的折线走，不许走"视线捷径"穿过转角
//   4. **单行严格**：逆向必须绕整圈回来（不允许抄近道逆行）
//   5. strict / hybrid：目标不在通道上时，前者停在投影点、后者补到目标
//   6. 可行性：车体过不去的通道默认不参与路由，并给出明确诊断

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/route_graph.hpp"
#include "pnc_2d/global/route_network_planner.hpp"

namespace pnc_2d {
namespace {

constexpr double kRes = 0.05;
constexpr int kHard = 80;

// ------------------------------------------------------------------ 地图构造
class MapBuilder {
public:
  MapBuilder(int w, int h, double res = kRes)
      : w_(w), h_(h), res_(res), data_(static_cast<std::size_t>(w) * h, 0) {}

  MapBuilder &rect(double x0, double y0, double x1, double y1, int8_t v = 100) {
    for (int cy = 0; cy < h_; ++cy) {
      for (int cx = 0; cx < w_; ++cx) {
        const double wx = (cx + 0.5) * res_;
        const double wy = (cy + 0.5) * res_;
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) {
          data_[static_cast<std::size_t>(cy) * w_ + cx] = v;
        }
      }
    }
    return *this;
  }

  std::shared_ptr<CostMap2D> build() const {
    auto m = std::make_shared<CostMap2D>();
    m->set(w_, h_, res_, 0.0, 0.0, 0.0, data_, "map");
    return m;
  }

private:
  int w_;
  int h_;
  double res_;
  std::vector<int8_t> data_;
};

/// 15x15 m 开阔地图（路网全部可行，便于验证路由本身）
std::shared_ptr<CostMap2D> openMap() { return MapBuilder(300, 300).build(); }

// ------------------------------------------------------------------ 路网 yaml
/// 一圈矩形环通道：A(1,1)-B(9,1)-C(9,9)-D(1,9)-A，每边 8 m
std::string ringYaml(bool e0_one_way) {
  std::string s;
  s += "frame_id: map\n";
  s += "defaults: {one_way: 0, speed_limit: 1.0, corridor_width: 0.5}\n";
  s += "nodes:\n";
  s += "  - {name: A, x: 1.0, y: 1.0, type: waypoint}\n";
  s += "  - {name: B, x: 9.0, y: 1.0, type: station}\n";
  s += "  - {name: C, x: 9.0, y: 9.0, type: charge}\n";
  s += "  - {name: D, x: 1.0, y: 9.0, type: park}\n";
  s += "edges:\n";
  s += "  - from: A\n    to: B\n    one_way: " +
       std::string(e0_one_way ? "1" : "0") + "\n";
  s += "    polyline: [[1.0,1.0], [5.0,1.0], [9.0,1.0]]\n";
  s += "  - from: B\n    to: C\n    polyline: [{x: 9.0, y: 1.0}, {x: 9.0, y: "
       "5.0}, {x: 9.0, y: 9.0}]\n";
  s += "  - from: C\n    to: D\n    polyline: [[9.0,9.0], [5.0,9.0], "
       "[1.0,9.0]]\n";
  s += "  - from: D\n    to: A\n    polyline: [[1.0,9.0], [1.0,5.0], "
       "[1.0,1.0]]\n";
  return s;
}

std::string ringPlusIsolatedYaml() {
  std::string s;
  s += "frame_id: map\n";
  s += "nodes:\n";
  s += "  - {name: A, x: 1.0, y: 1.0}\n";
  s += "  - {name: B, x: 9.0, y: 1.0}\n";
  s += "  - {name: C, x: 9.0, y: 9.0}\n";
  s += "  - {name: D, x: 1.0, y: 9.0}\n";
  s += "  - {name: F, x: 12.0, y: 12.0}\n";
  s += "  - {name: G, x: 13.0, y: 13.0}\n";
  s += "edges:\n";
  s += "  - {from: A, to: B, polyline: [[1.0,1.0], [5.0,1.0], [9.0,1.0]]}\n";
  s += "  - {from: B, to: C, polyline: [[9.0,1.0], [9.0,5.0], [9.0,9.0]]}\n";
  s += "  - {from: C, to: D, polyline: [[9.0,9.0], [5.0,9.0], [1.0,9.0]]}\n";
  s += "  - {from: D, to: A, polyline: [[1.0,9.0], [1.0,5.0], [1.0,1.0]]}\n";
  s +=
      "  - {from: F, to: G, polyline: [[12.0,12.0], [13.0,13.0]]}\n"; // 孤立的一段
  return s;
}

std::string writeTempRoutes(const std::string &yaml, const std::string &tag) {
  static int n = 0;
  const std::string path =
      "/tmp/pnc2d_test_routes_" + tag + "_" + std::to_string(n++) + ".yaml";
  std::ofstream f(path);
  f << yaml;
  return path;
}

Pose2D mkPose(double x, double y, double yaw = 0.0) {
  Pose2D p;
  p.x = x;
  p.y = y;
  p.yaw = yaw;
  p.has_yaw = true;
  return p;
}

std::unique_ptr<RouteNetworkPlanner>
makePlanner(const std::string &routes_yaml,
            const std::string &goal_mode = "hybrid", double speed = 1.0) {
  const std::string path = writeTempRoutes(routes_yaml, "ring");
  MemoryParamReader p;
  p.setString("route_network.routes_file", path);
  p.setString("route_network.goal_mode", goal_mode);
  p.setDouble("astar.max_search_time_ms", 1000.0);
  auto planner = std::make_unique<RouteNetworkPlanner>();
  EXPECT_TRUE(planner->configure(p)) << "configure 失败";
  planner->setCostMap(openMap());
  (void)speed;
  return planner;
}

double pathLen(const std::vector<Pose2D> &path) {
  double L = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    L += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  }
  return L;
}

/// 路径里是否有离给定点足够近的点
bool pathPassesNear(const std::vector<Pose2D> &path, double x, double y,
                    double tol) {
  for (const Pose2D &p : path) {
    if (std::hypot(p.x - x, p.y - y) <= tol)
      return true;
  }
  return false;
}

// ============================================================== RouteGraph
TEST(RouteGraph, ParseBasic) {
  RouteGraph g;
  std::string err;
  ASSERT_TRUE(g.loadFromString(ringYaml(false), err)) << err;
  EXPECT_EQ(g.nodes().size(), 4u);
  EXPECT_EQ(g.edges().size(), 4u);
  EXPECT_EQ(g.frameId(), "map");
  EXPECT_EQ(g.nodeIndex("C"), 2);
  EXPECT_EQ(g.nodeIndex("不存在"), -1);
  EXPECT_EQ(g.nodes()[1].type, RouteNodeType::kStation);
  EXPECT_EQ(g.nodes()[2].type, RouteNodeType::kCharge);
  EXPECT_EQ(g.nodes()[3].type, RouteNodeType::kPark);
  // defaults 里的 one_way: 0 必须被宽松解析成 false（yaml-cpp 的 as<bool>()
  // 会对整数抛异常）
  EXPECT_FALSE(g.edges()[0].one_way);
  EXPECT_DOUBLE_EQ(g.edges()[1].speed_limit, 1.0);
  EXPECT_DOUBLE_EQ(g.edges()[0].length, 8.0); // 8 m 直通道
  EXPECT_DOUBLE_EQ(g.edges()[1].length, 8.0);
  std::printf("      [路网] %s\n", g.summary().c_str());
}

TEST(RouteGraph, ParseErrors) {
  const std::vector<std::pair<std::string, std::string>> bad = {
      {"nodes: []\nedges: []\n", "空 nodes"},
      {"nodes:\n  - {name: A, x: 0, y: 0}\n", "缺 edges"},
      {"nodes:\n  - {name: A, x: 0, y: 0}\n  - {name: A, x: 1, y: 1}\n"
       "edges:\n  - {from: A, to: A, polyline: [[0,0],[1,1]]}\n",
       "重名 + 自环"},
      {"nodes:\n  - {name: A, x: 0, y: 0}\n"
       "edges:\n  - {from: A, to: Z, polyline: [[0,0],[1,1]]}\n",
       "to 不存在"},
      {"nodes:\n  - {name: A, x: 0, y: 0}\n  - {name: B, x: 1, y: 1}\n"
       "edges:\n  - {from: A, to: B, polyline: [[0,0]]}\n",
       "折线点数不足"},
      {"nodes:\n  - {name: A, x: 0, y: 0}\n  - {name: B, x: 0, y: 0}\n"
       "edges:\n  - {from: A, to: B, polyline: [[0,0],[0,0]]}\n",
       "零长度通道"},
      {"这不是 yaml: [", "语法错误"},
  };
  for (const auto &c : bad) {
    RouteGraph g;
    std::string err;
    EXPECT_FALSE(g.loadFromString(c.first, err)) << "应当拒绝：" << c.second;
    EXPECT_FALSE(err.empty()) << c.second;
    std::printf("      [拒绝] %-14s → %s\n", c.second.c_str(), err.c_str());
  }
}

TEST(RouteGraph, EndpointsSnapToNodes) {
  RouteGraph g;
  std::string err;
  // 端点离节点 0.2 m：应当吸附并给出警告，而不是报错
  const std::string y =
      "nodes:\n  - {name: A, x: 1.0, y: 1.0}\n  - {name: B, x: 5.0, y: 1.0}\n"
      "edges:\n  - {from: A, to: B, polyline: [[1.2,1.15], [5.0,1.0]]}\n";
  ASSERT_TRUE(g.loadFromString(y, err)) << err;
  EXPECT_DOUBLE_EQ(g.edges()[0].polyline.front().x, 1.0);
  EXPECT_DOUBLE_EQ(g.edges()[0].polyline.front().y, 1.0);
  ASSERT_EQ(g.warnings().size(), 1u);
  std::printf("      [吸附] %s\n", g.warnings()[0].c_str());
}

TEST(RouteGraph, ProjectAndInterpolate) {
  RouteGraph g;
  std::string err;
  ASSERT_TRUE(g.loadFromString(ringYaml(false), err)) << err;

  // 点在 e0 上方 0.6 m 处 → 应投影到 e0，弧长 2 m、垂距 0.6 m
  const RouteProjection pr = g.project(3.0, 1.6);
  ASSERT_TRUE(pr.valid);
  EXPECT_EQ(pr.edge, 0);
  EXPECT_NEAR(pr.s, 2.0, 1e-9);
  EXPECT_NEAR(pr.dist, 0.6, 1e-9);

  // 折线内插：e1（[9,1] → [9,5] → [9,9]）
  const RouteEdge &e1 = g.edges()[1];
  EXPECT_NEAR(e1.pointAt(0.0).y, 1.0, 1e-9);
  EXPECT_NEAR(e1.pointAt(4.0).y, 5.0, 1e-9);
  EXPECT_NEAR(e1.pointAt(6.5).y, 7.5, 1e-9);
  EXPECT_NEAR(e1.yawAt(3.0) * 180.0 / M_PI, 90.0, 1e-6);
  EXPECT_NEAR(e1.length, 8.0, 1e-9);
}

TEST(RouteGraph, SaveLoadRoundTrip) {
  RouteGraph g;
  std::string err;
  ASSERT_TRUE(g.loadFromString(ringYaml(true), err)) << err;
  const std::string path = "/tmp/pnc2d_test_routes_roundtrip.yaml";
  ASSERT_TRUE(g.saveToFile(path, err)) << err;

  RouteGraph g2;
  ASSERT_TRUE(g2.loadFromFile(path, err)) << err;
  EXPECT_EQ(g2.nodes().size(), g.nodes().size());
  EXPECT_EQ(g2.edges().size(), g.edges().size());
  EXPECT_TRUE(g2.edges()[0].one_way); // 单向要能存下来
  EXPECT_DOUBLE_EQ(g2.edges()[2].length, g.edges()[2].length);
  EXPECT_EQ(g2.nodes()[3].type, g.nodes()[3].type);
}

// ====================================================== RouteNetworkPlanner
TEST(RouteNetworkPlanner, FollowsLaneWithoutShortcut) {
  auto planner = makePlanner(ringYaml(false), "hybrid");
  const PlanResult r =
      planner->plan(PlanRequest{mkPose(3.0, 1.0), mkPose(9.0, 3.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
  const double L = pathLen(r.path);
  std::printf("      [沿通道] %zu 点 / %.2f m（直线捷径只要 %.2f m）| %s\n",
              r.path.size(), L, std::hypot(6.0, 2.0), r.message.c_str());
  // 必须沿 e0 走到 B(9,1) 再上 e1：6 + 2 = 8 m，绝不允许走 6.3 m 的斜线
  EXPECT_NEAR(L, 8.0, 0.05);
  EXPECT_TRUE(pathPassesNear(r.path, 9.0, 1.0, 0.05)) << "转角点 B 必须保留";
  EXPECT_EQ(planner->lastRouteEdges().size(), 2u);
}

TEST(RouteNetworkPlanner, OneWayForcesLoop) {
  auto open = makePlanner(ringYaml(false), "hybrid");
  const PlanResult r_open =
      open->plan(PlanRequest{mkPose(7.0, 1.0), mkPose(3.0, 1.0)});
  ASSERT_TRUE(r_open.ok()) << toString(r_open.status);
  EXPECT_NEAR(pathLen(r_open.path), 4.0, 0.05); // 双向：直接退回来 4 m

  auto one = makePlanner(ringYaml(true), "hybrid"); // e0 变成 A→B 单向
  const PlanResult r_one =
      one->plan(PlanRequest{mkPose(7.0, 1.0), mkPose(3.0, 1.0)});
  ASSERT_TRUE(r_one.ok()) << toString(r_one.status) << " / " << r_one.message;
  const double L = pathLen(r_one.path);
  std::printf("      [单行绕行] %.2f m（双向时只要 4.00 m），通道数 %zu\n", L,
              one->lastRouteEdges().size());
  EXPECT_NEAR(L, 28.0, 0.1); // 2 + 8 + 8 + 8 + 2
  // 5 段：e0（我→B）+ e1 + e2 + e3 + e0（A→我）—— 同一条通道被用了两次
  EXPECT_EQ(one->lastRouteEdges().size(), 5u);
  EXPECT_EQ(one->lastRouteEdges().front(), one->lastRouteEdges().back());
}

TEST(RouteNetworkPlanner, NoPathWhenDisconnected) {
  auto planner = makePlanner(ringPlusIsolatedYaml(), "hybrid");
  const PlanResult r =
      planner->plan(PlanRequest{mkPose(3.0, 1.0), mkPose(12.5, 12.5)});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, PlannerStatus::kNoPath);
  std::printf("      [不连通] %s\n", r.message.c_str());
}

TEST(RouteNetworkPlanner, StrictStopsAtProjectionHybridReachesGoal) {
  // 目标 (5,3) 离最近通道（e0，y=1）2 m
  const PlanRequest req{mkPose(3.0, 1.0), mkPose(5.0, 3.0)};

  auto strict = makePlanner(ringYaml(false), "strict");
  const PlanResult rs = strict->plan(req);
  ASSERT_TRUE(rs.ok()) << toString(rs.status) << " / " << rs.message;
  EXPECT_NEAR(rs.path.back().y, 1.0, 0.02) << "strict：应停在投影点";
  EXPECT_NEAR(pathLen(rs.path), 2.0, 0.05);

  auto hybrid = makePlanner(ringYaml(false), "hybrid");
  const PlanResult rh = hybrid->plan(req);
  ASSERT_TRUE(rh.ok()) << toString(rh.status) << " / " << rh.message;
  EXPECT_NEAR(rh.path.back().x, 5.0, 0.02) << "hybrid：应补到真实目标";
  EXPECT_NEAR(rh.path.back().y, 3.0, 0.02);
  EXPECT_NEAR(pathLen(rh.path), 4.0, 0.05); // 2 m 沿通道 + 2 m 离网
  std::printf("      [strict/hybrid] %.2f m / %.2f m\n", pathLen(rs.path),
              pathLen(rh.path));
}

TEST(RouteNetworkPlanner, BlockedLaneIsSkippedOthersStillRoute) {
  // 用户最关心的语义：路网拆成多段后，**被挡的那段单独剔除，其余仍可路由**。
  // 环 A-B-C-D 上把 B->C 挡死：从 A->B 边上的点去 C->D 边上的点，必须绕 D->A。
  auto map = MapBuilder(300, 300).rect(8.5, 4.5, 9.5, 5.5).build();
  const std::string yaml = ringYaml(false);

  MemoryParamReader p;
  p.setString("route_network.routes_file", writeTempRoutes(yaml, "skip"));
  p.setBool("route_network.reject_infeasible", true);
  p.setString("route_network.goal_mode", "strict"); // 只看路网段，便于算长度
  RouteNetworkPlanner planner;
  ASSERT_TRUE(planner.configure(p));
  planner.setCostMap(map);

  const PlanResult r =
      planner.plan(PlanRequest{mkPose(3.0, 1.0), mkPose(6.0, 9.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
  // 绕行里程：起点(3,1)→A 2 m + A→D 8 m + D→(6,9) 5 m = 15 m
  EXPECT_NEAR(pathLen(r.path), 15.0, 0.15);
  // 走过的通道里绝不能有被挡的 B->C（节点序号不写死，按名字判）
  const auto used = planner.activeRouteEdges();
  const RouteGraph *g = planner.routeGraph();
  ASSERT_NE(g, nullptr);
  for (const int ei : used) {
    const RouteEdge &e = g->edges()[static_cast<std::size_t>(ei)];
    const std::string nm =
        g->nodes()[static_cast<std::size_t>(e.from)].name + "->" +
        g->nodes()[static_cast<std::size_t>(e.to)].name;
    EXPECT_NE(nm, "B->C") << "不应使用被挡的通道";
  }
  EXPECT_EQ(used.size(), 3u) << "应走 A->B / D->A / C->D 三条";
  std::printf("      [绕开被挡段] %.2f m | 用了 %zu 条通道 | 消息：%s\n",
              pathLen(r.path), used.size(), r.message.c_str());
  EXPECT_NE(r.message.find("B->C"), std::string::npos)
      << "成功时也应说明绕开了哪条：" << r.message;
}

TEST(RouteNetworkPlanner, NoPathMessageNamesTheBlockedLane) {
  // 唯一通路必须经过被挡通道时：报错要**点名**，不能只说"有几条"
  auto map = MapBuilder(300, 300).rect(8.5, 4.5, 9.5, 5.5).build();
  MemoryParamReader p;
  p.setString("route_network.routes_file", writeTempRoutes(ringYaml(false), "name"));
  p.setBool("route_network.reject_infeasible", true);
  RouteNetworkPlanner planner;
  ASSERT_TRUE(planner.configure(p));
  planner.setCostMap(map);

  const PlanResult r =
      planner.plan(PlanRequest{mkPose(3.0, 1.0), mkPose(9.0, 8.0)});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, PlannerStatus::kNoPath);
  EXPECT_NE(r.message.find("B->C"), std::string::npos)
      << "报错里应点名不可行通道：" << r.message;
  std::printf("      [点名] %s\n", r.message.c_str());
}

TEST(RouteNetworkPlanner, InfeasibleLaneIsRejected) {  // 在 e1（x=9，y 从 1 到 9）中间横一道墙 → 车体过不去
  auto map = MapBuilder(300, 300).rect(8.5, 4.5, 9.5, 5.5).build();
  const std::string yaml = ringYaml(false);

  {
    MemoryParamReader p;
    p.setString("route_network.routes_file", writeTempRoutes(yaml, "wall"));
    p.setBool("route_network.reject_infeasible", true);
    RouteNetworkPlanner planner;
    ASSERT_TRUE(planner.configure(p));
    planner.setCostMap(map);
    const PlanResult r =
        planner.plan(PlanRequest{mkPose(3.0, 1.0), mkPose(9.0, 8.0)});
    EXPECT_FALSE(r.ok());
    EXPECT_EQ(r.status, PlannerStatus::kNoPath);
    EXPECT_NE(r.message.find("过不去"), std::string::npos) << r.message;
    const auto counts = planner.feasibilityCounts();
    std::printf("      [不可行] 可行 %zu / 不可行 %zu | %s\n", counts.first,
                counts.second, r.message.c_str());
    EXPECT_EQ(counts.second, 1u);
  }
  {
    MemoryParamReader p; // 允许强行走
    p.setString("route_network.routes_file", writeTempRoutes(yaml, "wall"));
    p.setBool("route_network.reject_infeasible", false);
    RouteNetworkPlanner planner;
    ASSERT_TRUE(planner.configure(p));
    planner.setCostMap(map);
    const PlanResult r =
        planner.plan(PlanRequest{mkPose(3.0, 1.0), mkPose(9.0, 8.0)});
    EXPECT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
    EXPECT_NE(r.message.find("过不去"), std::string::npos)
        << "消息里应提醒路网有问题";
  }
}

TEST(RouteNetworkPlanner, ReportsMissingRoutesFile) {
  MemoryParamReader p;
  RouteNetworkPlanner planner;
  ASSERT_TRUE(planner.configure(p)); // 没配 routes_file 不算致命
  planner.setCostMap(openMap());
  const PlanResult r =
      planner.plan(PlanRequest{mkPose(1.0, 1.0), mkPose(3.0, 1.0)});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, PlannerStatus::kNotInitialized);
  std::printf("      [未载入] %s\n", r.message.c_str());
}

} // namespace
} // namespace pnc_2d
