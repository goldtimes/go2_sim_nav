// AStarPlanner 单元测试：合成地图、不依赖 ROS、毫秒级。
//
// 说明：路径合法性用**独立实现**的暴力校验（把车体矩形按 2 cm
// 网格采样后查原始地图）， 不复用被测的 SAT/偏移表代码 ——
// 否则等于"用自己验证自己"。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/global/astar_planner.hpp"

namespace pnc_2d {
namespace {

constexpr double kRes = 0.05;
constexpr int kHard = 80;

// ------------------------------------------------------------------ 地图构造
class MapBuilder {
public:
  MapBuilder(int w, int h, double res = kRes)
      : w_(w), h_(h), res_(res), data_(static_cast<std::size_t>(w) * h, 0) {}

  void cellCenter(int cx, int cy, double &wx, double &wy) const {
    wx = (cx + 0.5) * res_;
    wy = (cy + 0.5) * res_;
  }

  /// 世界坐标矩形填充
  MapBuilder &rect(double x0, double y0, double x1, double y1, int8_t v = 100) {
    for (int cy = 0; cy < h_; ++cy) {
      for (int cx = 0; cx < w_; ++cx) {
        double wx = 0.0;
        double wy = 0.0;
        cellCenter(cx, cy, wx, wy);
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

  int w() const { return w_; }
  int h() const { return h_; }
  double res() const { return res_; }

private:
  int w_;
  int h_;
  double res_;
  std::vector<int8_t> data_;
};

/// 便利：默认参数 + 覆盖项
std::unique_ptr<AStarPlanner> makePlanner(const MemoryParamReader &p) {
  auto planner = std::make_unique<AStarPlanner>();
  EXPECT_TRUE(planner->configure(p));
  return planner;
}

Pose2D mkPose(double x, double y, double yaw_deg) {
  Pose2D p;
  p.x = x;
  p.y = y;
  p.yaw = yaw_deg * M_PI / 180.0;
  p.has_yaw = true;
  return p;
}

// ------------------------------------------------------ 独立暴力碰撞校验
bool footprintFreeBruteForce(const CostMap2D &map, const FootprintParams &fp,
                             int hard_threshold, double x, double y,
                             double yaw) {
  if (!fp.enable) {
    int cx = 0;
    int cy = 0;
    if (!map.worldToGrid(x, y, cx, cy))
      return false;
    return map.rawValue(cx, cy) < hard_threshold;
  }
  const double hu = fp.halfLength();
  const double hv = fp.halfWidth();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double step = 0.02;
  for (double u = -hu; u <= hu + 1e-9; u += step) {
    for (double v = -hv; v <= hv + 1e-9; v += step) {
      const double ox = fp.offset_x + u;
      const double oy = fp.offset_y + v;
      const double wx = x + ox * c - oy * s;
      const double wy = y + ox * s + oy * c;
      int cx = 0;
      int cy = 0;
      if (!map.worldToGrid(wx, wy, cx, cy))
        return false;
      if (map.rawValue(cx, cy) >= hard_threshold)
        return false;
    }
  }
  return true;
}

/// 沿路径密集采样位姿，逐点暴力校验。
/// 语义：段内**车身朝向 = 线段方向**（不是两端 yaw 插值）—— 机器人是直线行驶，
/// 到点后再原地转向下一段；这里只校验"直线行驶"那一段的包络。
bool pathFreeBruteForce(const CostMap2D &map, const FootprintParams &fp,
                        int hard_threshold, const std::vector<Pose2D> &path,
                        std::string *failure = nullptr) {
  if (path.empty())
    return false;
  // 起点处还要按它自己的朝向查一次（控制器要先原地对正）
  if (path.front().has_yaw &&
      !footprintFreeBruteForce(map, fp, hard_threshold, path.front().x,
                               path.front().y, path.front().yaw)) {
    if (failure != nullptr)
      *failure = "起点按自身朝向就与障碍相交";
    return false;
  }
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Pose2D &a = path[i];
    const Pose2D &b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len = std::hypot(dx, dy);
    const double yaw = std::atan2(dy, dx);
    const int n = std::max(1, static_cast<int>(std::ceil(len / (0.5 * kRes))));
    for (int k = 0; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      const double x = a.x + dx * t;
      const double y = a.y + dy * t;
      if (!footprintFreeBruteForce(map, fp, hard_threshold, x, y, yaw)) {
        if (failure != nullptr) {
          *failure = "路径点 (" + std::to_string(x) + ", " + std::to_string(y) +
                     ") 处车体与障碍相交（yaw=" + std::to_string(yaw) + "）";
        }
        return false;
      }
    }
  }
  return true;
}

// ==========================================================================
// 0) 碰撞检查器的分级判据：含余量 / 真实轮廓 / 真实轮廓再让 tol
//
// 现实场景：全局图是**被膨胀过**的（真障碍 + map_server.inflate 0.05 +
// 禁行区按 zones.inflate 0.05 烧入），且障碍边界被量化到 ±半格。
// 于是"离真障碍 0.28 m"在全局图里量到 0.224 m（最近占据格中心），
// 而含余量需求是 0.25 m —— 只差 1 mm。这三档判据就是用来分辨
// "擦一点" / "差一格" / "真撞"的。
// ==========================================================================
TEST(FootprintCollisionCheckerTest, StagedMarginsDistinguishGrazeFromHit) {
  // 真墙在 x = 3.00，但**发布出来的图已被膨胀 0.05**（模拟 map_server.inflate）
  MapBuilder mb(240, 240);
  mb.rect(2.95, 0.0, 3.40, 12.0);
  auto map = mb.build();
  FootprintParams fp;
  fp.length = 0.70;
  fp.width = 0.40;
  fp.safe_margin = 0.05;
  FootprintCollisionChecker c;
  c.configure(fp, kHard, false);
  c.setMap(map.get());
  const double side_on = M_PI / 2;   // 侧面对着墙 ⇒ 需求 = 半宽 (+margin)

  // 离真墙 0.285 m（图上占据格从 2.95 起 ⇒ 图里量到 ~0.225）
  const double x = 3.00 - 0.285;
  std::printf("  [分级判据] x=%.3f 含余量=%d 真实轮廓=%d 让1格=%d 让2格=%d\n", x,
              c.poseInCollision(x, 3.0, side_on),
              c.poseInCollisionAtMargin(x, 3.0, side_on, 0.0),
              c.poseInCollisionAtMargin(x, 3.0, side_on, -0.05),
              c.poseInCollisionAtMargin(x, 3.0, side_on, -0.10));
  EXPECT_TRUE(c.poseInCollision(x, 3.0, side_on)) << "含余量判据应该拦下来";
  EXPECT_FALSE(c.poseInCollisionAtMargin(x, 3.0, side_on, -0.05))
      << "穿透 ≤1 格 ⇒ 允许规划（这就是现场卡死的情形）";

  // 离真墙 0.20 m：真实轮廓（半宽 0.20）刚好压上 ⇒ 让一格还能过、让两格过不去
  const double x2 = 3.00 - 0.20;
  EXPECT_TRUE(c.poseInCollisionAtMargin(x2, 3.0, side_on, 0.0));
  EXPECT_FALSE(c.poseInCollisionAtMargin(x2, 3.0, side_on, -0.05));

  // 离真墙 0.06 m：真压进去了 ⇒ 让一格也过不去（应人工处理）
  const double x3 = 3.00 - 0.06;
  EXPECT_TRUE(c.poseInCollisionAtMargin(x3, 3.0, side_on, -0.05));
}

// ==========================================================================
// 1) 空地直线
// ==========================================================================
TEST(AStarPlanner, FreeSpaceStraightLine) {
  MapBuilder mb(240, 240); // 12x12 m
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r = planner->plan(
      PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(11.0, 11.0, 90.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
  ASSERT_GE(r.path.size(), 2u);

  const double expect = std::hypot(10.0, 10.0);
  EXPECT_NEAR(r.stats.path_length, expect, expect * 0.05);
  EXPECT_NEAR(r.path.back().x, 11.0, kRes);
  EXPECT_NEAR(r.path.back().y, 11.0, kRes);
  EXPECT_NEAR(std::fabs(r.path.back().yaw), M_PI / 2, 1e-6); // 末点用目标 yaw

  std::string why;
  EXPECT_TRUE(pathFreeBruteForce(
      *planner->costMap(), planner->footprintParams(), kHard, r.path, &why))
      << why;
}

// ==========================================================================
// 2) 墙留洞 → 必须绕行
// ==========================================================================
TEST(AStarPlanner, DetourThroughGap) {
  MapBuilder mb(240, 240);
  mb.rect(6.0, 0.0, 6.2, 10.0); // 竖墙，y 从 0 堵到 10 m（留 10~12 m 的洞）
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(11.0, 1.0, 0.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;

  double max_y = 0.0;
  for (const auto &pt : r.path)
    max_y = std::max(max_y, pt.y);
  EXPECT_GT(max_y, 10.0) << "路径应该从墙顶的洞口绕过去";

  std::string why;
  EXPECT_TRUE(pathFreeBruteForce(
      *planner->costMap(), planner->footprintParams(), kHard, r.path, &why))
      << why;
}

// ==========================================================================
// 3) 目标被完全围住 → 无解，且必须触发窗口扩大重试
// ==========================================================================
TEST(AStarPlanner, EnclosedGoalNoPathAndRetries) {
  MapBuilder mb(240, 240);
  const double cx = 8.0;
  const double cy = 8.0;
  const double rr = 1.0;
  mb.rect(cx - rr, cy - rr, cx + rr, cy - rr + 0.1); // 下
  mb.rect(cx - rr, cy + rr - 0.1, cx + rr, cy + rr); // 上
  mb.rect(cx - rr, cy - rr, cx - rr + 0.1, cy + rr); // 左
  mb.rect(cx + rr - 0.1, cy - rr, cx + rr, cy + rr); // 右
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(cx, cy, 0.0)});
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status, PlannerStatus::kNoPath);
  EXPECT_GT(r.stats.windows_tried, 1) << "窗口失败后应当逐级扩大重试";
  EXPECT_TRUE(r.path.empty()) << "失败时不允许返回路径充数";
}

// ==========================================================================
// 4) 软代价：权重开启后路径绕开代价带，关闭后直接穿过去
// ==========================================================================
TEST(AStarPlanner, SoftCostAvoidance) {
  MapBuilder mb(240, 240);
  mb.rect(5.0, 0.0, 6.0, 8.0, 60); // 软代价带（值 60 < 硬阈值 80），高 8 m
  const auto map = mb.build();

  auto crossesBand = [](const std::vector<Pose2D> &path) {
    // 必须**沿线段采样**：剪枝后路径可能只剩两个端点，只看顶点会漏判
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const double dx = path[i + 1].x - path[i].x;
      const double dy = path[i + 1].y - path[i].y;
      const double len = std::hypot(dx, dy);
      const int n = std::max(1, static_cast<int>(std::ceil(len / 0.025)));
      for (int k = 0; k <= n; ++k) {
        const double t = static_cast<double>(k) / static_cast<double>(n);
        const double x = path[i].x + dx * t;
        const double y = path[i].y + dy * t;
        if (x > 5.1 && x < 5.9 && y < 7.9)
          return true;
      }
    }
    return false;
  };

  {
    MemoryParamReader p;
    p.setDouble("common.soft_cost_weight", 0.0);
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const PlanResult r = planner->plan(
        PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(11.0, 1.0, 0.0)});
    ASSERT_TRUE(r.ok()) << toString(r.status);
    EXPECT_TRUE(crossesBand(r.path))
        << "soft_cost_weight=0 时应当直接穿过代价带（最短）";
  }
  {
    MemoryParamReader p;
    p.setDouble("common.soft_cost_weight", 200.0);
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const PlanResult r = planner->plan(
        PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(11.0, 1.0, 0.0)});
    ASSERT_TRUE(r.ok()) << toString(r.status);
    std::printf("      [软代价 200] %zu 个点：", r.path.size());
    for (const auto &pt : r.path)
      std::printf(" (%.2f,%.2f)", pt.x, pt.y);
    std::printf("\n");
    EXPECT_FALSE(crossesBand(r.path)) << "软代价权重很大时应当绕开代价带";
  }
}

// ==========================================================================
// 5) 起点/终点在障碍里
// ==========================================================================
TEST(AStarPlanner, StartAndGoalOccupied) {
  MapBuilder mb(240, 240);
  mb.rect(0.9, 0.9, 1.6, 1.6);     // 盖住起点
  mb.rect(10.4, 10.4, 11.6, 11.6); // 盖住终点
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  auto r1 = planner->plan(
      PlanRequest{mkPose(1.25, 1.25, 0.0), mkPose(5.0, 5.0, 0.0)});
  EXPECT_EQ(r1.status, PlannerStatus::kStartOccupied);
  auto r2 = planner->plan(
      PlanRequest{mkPose(5.0, 5.0, 0.0), mkPose(11.0, 11.0, 0.0)});
  EXPECT_EQ(r2.status, PlannerStatus::kGoalOccupied);
}

// ==========================================================================
// 5b) 起点"撞到安全余量"vs"真的压上障碍"（2026-09-23 实测的任务死锁）
//
// 为什么必须分清（现场链条）：全局图是 map_server 发的（真障碍已膨胀、
// 禁行区已按 zones.inflate 烧入），局部用的是感知图 + 自己的距离场。
// 两边判据名义上一样但**图不同源** + 栅格有 ±半格离散化 ⇒ "全局刚判过、
// 另一侧差 1~2 cm" 是常态。若起点一律判死：局部先 BLOCKED → 恢复行为
// 重规划 → 这里又失败 ⇒ 任务永久 FAILED（车既不动也不结束）。
// 实测数字：车离禁行区 Z1 边界 0.283 m，含余量需求 0.30 m，差 1.7 cm。
// ==========================================================================
TEST(AStarPlanner, StartTouchingMarginStillPlansButRealCollisionFails) {
  // 真实墙在 x = 3.00，但**发布出来的图已被膨胀 0.05**（map_server.inflate）——
  // 这就是现场：车离禁行区多边形 0.28 m，而全局图里最近占据格中心只有 0.224 m。
  // 墙**有尽头**（y∈[6,11]），否则车侧行出不去（需要原地转向，栅格 A* 不建模）。
  const double wall = 3.00;
  MapBuilder mb(240, 240);
  mb.rect(wall - 0.05, 6.0, wall + 0.4, 11.0);
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  // ① 离真墙 0.285 m（侧面对着，落在余量带里：图上需求 0.30）⇒ 虚拟起点 + 首段交给局部
  const double x_margin = wall - 0.285;
  auto r1 = planner->plan(PlanRequest{mkPose(x_margin, 7.5, 90.0),
                                      mkPose(1.0, 10.0, 90.0)});
  std::printf("  [起点擦边 0.285] %s：%s\n", toString(r1.status), r1.message.c_str());
  EXPECT_EQ(r1.status, PlannerStatus::kSuccess)
      << "只差一格（1 mm 量级）不该把任务判死";
  EXPECT_NE(r1.message.find("可通行位姿"), std::string::npos)
      << "让步必须**点名说明**，否则现场看不出来";
  EXPECT_FALSE(r1.path.empty());
  EXPECT_LT(std::hypot(r1.path.front().x - x_margin, r1.path.front().y - 7.5),
            0.05) << "首点必须是车自己（后续由局部擦出来）";
  // ★★ 关键回归：除了首段（车→虚拟起点），**其余路径必须回到完整余量**。
  //    （第一版把整条路都放宽 ⇒ 一路贴墙走，用户实测“没按全局走、也没避开障碍”）
  {
    double worst_far = 1e9;
    for (std::size_t i = 0; i < r1.path.size(); ++i) {
      const double dx = r1.path[i].x - x_margin;
      const double dy = r1.path[i].y - 7.5;
      if (std::hypot(dx, dy) < 1.5) continue;   // 起点让步圈（1.0 m）内不看
      worst_far = std::min(worst_far, r1.path[i].x - (wall - 0.05));
    }
    std::printf("      圈外最小净距 %.3f m（含图膨胀 0.05 ⇒ 需求 0.30）\n",
                worst_far + 0.05);
    EXPECT_GT(worst_far + 0.05, 0.29)
        << "圈外必须保持完整余量（否则整条路都贴墙走）";
  }

  // ② 离真墙 0.225 m（真实轮廓需求 0.25 也放不下 ⇒ 车已经贴在余量里）
  //    ⇒ **不救**，如实报错（规划一条“从贴着的位姿开出去”的路只会擦得更厉害）。
  const double x_graze = wall - 0.225;
  auto r2 = planner->plan(PlanRequest{mkPose(x_graze, 7.5, 90.0),
                                      mkPose(1.0, 10.0, 90.0)});
  std::printf("  [起点贴到 0.225] %s：%s\n", toString(r2.status), r2.message.c_str());
  EXPECT_EQ(r2.status, PlannerStatus::kStartFootprintCollision)
      << "真实轮廓都放不下就该停下报错，不许“贴着墙开出去”";

  // ③ 离真墙 0.06 m（穿透 ≥ 1 格）⇒ 必须报错，交人工
  const double x_hit = wall - 0.06;
  auto r3 = planner->plan(PlanRequest{mkPose(x_hit, 7.5, 90.0),
                                      mkPose(1.0, 10.0, 90.0)});
  std::printf("  [起点穿透>1格] %s：%s\n", toString(r3.status), r3.message.c_str());
  EXPECT_EQ(r3.status, PlannerStatus::kStartFootprintCollision)
      << "穿透超过 1 格时爬着走更危险，必须如实报错";
  EXPECT_NE(r3.message.find("穿透已超过"), std::string::npos);

  // ③ 关掉让步（半径 0）⇒ 回到严格判据（1 mm 也不算）
  MemoryParamReader strict;
  strict.setDouble("astar.start_escape_radius", 0.0);
  auto sp = makePlanner(strict);
  sp->setCostMap(mb.build());
  auto r4 = sp->plan(PlanRequest{mkPose(x_margin, 7.5, 90.0),
                                 mkPose(1.0, 10.0, 90.0)});
  EXPECT_EQ(r4.status, PlannerStatus::kStartFootprintCollision);
}

// ==========================================================================
// 5c) 全局层的“净距偏好”（2026-09-23 用户要求：让局部远离障碍/禁行区）
//
// 为什么用“偏好”而不是把膨胀调大：用户定的规则是“机体 0.40 + 左右各 10 cm =
// 最窄可通 0.60 m”⇒ 硬判据只能停在 0.30 m（半宽 0.20 + margin 0.05 + 图膨胀 0.05）。
// 硬层再加就会把 0.60 m 的通道直接判死；而偏好（软代价）只是“同样可达时走远一点”，
// 窄通道照样过得去 —— 这两条一起钉住。
// ==========================================================================
TEST(AStarPlanner, ClearancePreferencePrefersMiddleAndKeepsNarrowPassage) {
  // ---- ① 绕过柱子：最短的路贴着柱角过（~0.30 m），开偏好后走远一点 ----
  //
  // ⚠ 几何要选对：**窄而浅的走廊测不出偏好** —— 车长 0.70（半长 0.40），
  //   离墙 0.30 时它根本转不了身（任何朝向变化都让 y 向尺度涨到 0.46），
  //   栅格 A* 的节点朝向 = 运动方向 ⇒ 只能直着走，偏好无处可去（实测踩过）。
  //   所以要给"能绕"的空间：中间一根 0.4×0.4 的柱子。
  MapBuilder mb(240, 240);                     // 12×12 m
  const double bx0 = 5.0, bx1 = 5.4, by0 = 2.0, by1 = 2.4;
  mb.rect(bx0, by0, bx1, by1);
  const double y_lane = 2.2;                   // 正对柱子走 ⇒ 必须绕
  auto map = mb.build();
  auto clearanceToBlock = [&](const PlanResult &r) {
    double worst = 1e9;
    for (std::size_t i = 1; i + 1 < r.path.size(); ++i) {
      const double dx = std::max({bx0 - r.path[i].x, 0.0, r.path[i].x - bx1});
      const double dy = std::max({by0 - r.path[i].y, 0.0, r.path[i].y - by1});
      worst = std::min(worst, std::hypot(dx, dy));
    }
    return worst;
  };
  double d_off = 0.0, d_on = 0.0, len_on = 0.0;
  {
    MemoryParamReader p;                       // 默认：偏好关
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const auto r = planner->plan(
        PlanRequest{mkPose(1.0, y_lane, 0.0), mkPose(10.0, y_lane, 0.0)});
    ASSERT_EQ(r.status, PlannerStatus::kSuccess) << r.message;
    d_off = clearanceToBlock(r);
    std::printf("  [净距偏好] 关：绕柱最小净距 %.3f m | %zu 点 %.2f m\n", d_off,
                r.path.size(), r.stats.path_length);
    EXPECT_LT(d_off, 0.45) << "关掉偏好时应该贴着柱角过";
  }
  {
    MemoryParamReader p;
    p.setDouble("planner.clearance_prefer_dist", 0.60);
    p.setDouble("planner.clearance_cost_weight", 2.0);
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const auto r = planner->plan(
        PlanRequest{mkPose(1.0, y_lane, 0.0), mkPose(10.0, y_lane, 0.0)});
    ASSERT_EQ(r.status, PlannerStatus::kSuccess) << r.message;
    d_on = clearanceToBlock(r);
    len_on = r.stats.path_length;
    std::printf("  [净距偏好] 开：绕柱最小净距 %.3f m | %.2f m\n", d_on, len_on);
    EXPECT_GT(d_on, 0.50) << "开了偏好应该离柱子更远";
    EXPECT_GT(d_on, d_off + 0.10) << "而且要比关掉时明显更远";
    EXPECT_LT(len_on, 12.0) << "只是让开一点，不该绕远";
  }

  // ---- ② 0.60 m 窄通道（= 用户规则“机体 0.40 + 左右各 10 cm”）：偏好是软的，
  //      不能把这种通道判死 ----
  MapBuilder mb2(240, 240);
  mb2.rect(3.0, 0.0, 8.0, 3.00);               // 下墙，留出 y∈[3.00,3.60]
  mb2.rect(3.0, 3.60, 8.0, 12.0);              // 上墙
  MemoryParamReader p2;
  p2.setDouble("planner.clearance_prefer_dist", 0.60);
  p2.setDouble("planner.clearance_cost_weight", 2.0);
  auto planner2 = makePlanner(p2);
  planner2->setCostMap(mb2.build());
  const auto r2 = planner2->plan(
      PlanRequest{mkPose(1.0, 3.30, 0.0), mkPose(10.0, 3.30, 0.0)});
  std::printf("  [净距偏好] 0.60 m 通道：%s\n", toString(r2.status));
  ASSERT_EQ(r2.status, PlannerStatus::kSuccess)
      << "偏好是软的，0.60 m（左右各 10 cm）必须仍然可通：" << r2.message;
  for (const auto &p : r2.path) {
    if (p.x > 3.2 && p.x < 7.8) {
      ASSERT_GT(p.y, 3.24) << "通道里不该压墙";
      ASSERT_LT(p.y, 3.36) << "通道里不该压墙";
    }
  }
}

// ==========================================================================
// 6) 越界
// ==========================================================================
TEST(AStarPlanner, OutOfMap) {
  MapBuilder mb(240, 240);
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  auto r1 =
      planner->plan(PlanRequest{mkPose(-5.0, 1.0, 0.0), mkPose(5.0, 5.0, 0.0)});
  EXPECT_EQ(r1.status, PlannerStatus::kStartOutOfMap);
  auto r2 = planner->plan(
      PlanRequest{mkPose(5.0, 5.0, 0.0), mkPose(99.0, 99.0, 0.0)});
  EXPECT_EQ(r2.status, PlannerStatus::kGoalOutOfMap);
}

// ==========================================================================
// 7) 不允许从对角缝隙斜穿过去
// ==========================================================================
TEST(AStarPlanner, NoDiagonalCornerCutting) {
  // 布置：格 (100,100) 与 (101,101) 空闲，(101,100) 与 (100,101) 被堵
  MapBuilder mb(240, 240);
  double wx = 0.0;
  double wy = 0.0;
  auto fillCell = [&](int cx, int cy) {
    mb.cellCenter(cx, cy, wx, wy);
    mb.rect(wx - 0.5 * kRes, wy - 0.5 * kRes, wx + 0.5 * kRes, wy + 0.5 * kRes);
  };
  fillCell(101, 100);
  fillCell(100, 101);
  // 两侧加墙，迫使路径只能考虑这个对角缝
  mb.rect(5.0, 4.0, 5.6, 5.0);
  mb.rect(5.6, 5.6, 6.2, 7.0);

  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());
  const PlanResult r =
      planner->plan(PlanRequest{mkPose(4.0, 5.1, 0.0), mkPose(7.0, 5.4, 0.0)});
  if (r.ok()) {
    std::string why;
    EXPECT_TRUE(pathFreeBruteForce(
        *planner->costMap(), planner->footprintParams(), kHard, r.path, &why))
        << why;
  } else {
    EXPECT_EQ(r.status, PlannerStatus::kNoPath);
  }
}

// ==========================================================================
// 8) 窄通道能过（0.60 m 宽，车宽 0.40 + margin 0.05）
// ==========================================================================
TEST(AStarPlanner, NarrowCorridorFits) {
  MapBuilder mb(240, 240);
  // 同样的墙，但门宽 0.60 m（y∈[5.3,5.9]）→ 能过
  mb.rect(5.0, 0.0, 5.3, 5.3);
  mb.rect(5.0, 5.9, 5.3, 12.0);
  MemoryParamReader p;
  p.setDouble("footprint.length", 0.70);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.safe_margin", 0.05);
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 5.6, 0.0), mkPose(10.0, 5.6, 0.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
  std::string why;
  EXPECT_TRUE(pathFreeBruteForce(
      *planner->costMap(), planner->footprintParams(), kHard, r.path, &why))
      << why;
}

// ==========================================================================
// 9) 窄通道过不去（0.45 m 宽 < 0.40 + 2×0.05）→ 必须报无解
// ==========================================================================
TEST(AStarPlanner, NarrowCorridorTooTight) {
  MapBuilder mb(240, 240);
  // 横贯整张图的墙，只留 0.45 m 的门（y∈[5.3,5.75]）→ 唯一通路
  mb.rect(5.0, 0.0, 5.3, 5.3);
  mb.rect(5.0, 5.75, 5.3, 12.0);
  MemoryParamReader p;
  p.setDouble("footprint.length", 0.70);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.safe_margin", 0.05);
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  // 起始区在左、目标在右，中间是一道只留 0.45 m 门的墙 → 必须报无解
  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 5.5, 0.0), mkPose(10.0, 5.5, 0.0)});
  std::printf("      [窄通道 0.45m] status=%s tried=%d\n", toString(r.status),
              r.stats.windows_tried);
  EXPECT_FALSE(r.ok()) << "0.45 m 通道装不下 0.40 m 车宽 + 两侧 0.05 m 余量";
  EXPECT_EQ(r.status, PlannerStatus::kNoPath)
      << toString(r.status) << " / " << r.message;
  EXPECT_GT(r.stats.windows_tried, 1);
}

// ==========================================================================
// 10) 同一场景关掉 footprint → 能过（证明是 footprint 在拦）
// ==========================================================================
TEST(AStarPlanner, FootprintDisabledPassesTightCorridor) {
  MapBuilder mb(240, 240);
  mb.rect(0.0, 5.0, 12.0, 5.3);
  mb.rect(0.0, 5.75, 12.0, 6.05);
  MemoryParamReader p;
  p.setBool("footprint.enable", false);
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 5.5, 0.0), mkPose(11.0, 5.5, 0.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
}

// ==========================================================================
// 11) 目标朝向参与判定：同一个窄通道，0° 可过、90° 过不去
// ==========================================================================
TEST(AStarPlanner, GoalYawMattersInNarrowCorridor) {
  MapBuilder mb(240, 240);
  mb.rect(0.0, 5.0, 12.0, 5.3);
  mb.rect(0.0, 5.9, 12.0, 6.2); // 0.60 m 宽的通道
  auto map = mb.build();
  MemoryParamReader p;
  p.setDouble("footprint.length", 0.70);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.safe_margin", 0.05);

  {
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const PlanResult r = planner->plan(
        PlanRequest{mkPose(1.0, 5.6, 0.0), mkPose(11.0, 5.6, 0.0)});
    EXPECT_TRUE(r.ok()) << "目标朝向与通道一致时应当可过："
                        << toString(r.status);
  }
  {
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    const PlanResult r = planner->plan(
        PlanRequest{mkPose(1.0, 5.6, 0.0), mkPose(11.0, 5.6, 90.0)});
    EXPECT_FALSE(r.ok())
        << "目标朝向 90° 时车长 0.70 m 横在 0.60 m 通道里 → 必须报碰撞";
    EXPECT_EQ(r.status, PlannerStatus::kGoalFootprintCollision);
  }
}

// ==========================================================================
// 12) 起终点同格
// ==========================================================================
TEST(AStarPlanner, SameCell) {
  MapBuilder mb(240, 240);
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());
  const PlanResult r = planner->plan(
      PlanRequest{mkPose(3.0, 3.0, 0.0), mkPose(3.02, 3.0, 45.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status);
  EXPECT_EQ(r.path.size(), 1u);
}

// ==========================================================================
// 13) 搜索窗口 + 逐级扩大重试：
//     路径必须绕到"起终点包围盒"之外时，首次窗口应当失败、重试应当成功。
//     同时验证 use_search_window=false（整图）给出等价结果。
// ==========================================================================
TEST(AStarPlanner, SearchWindowRetrySucceeds) {
  MapBuilder mb(600, 600);        // 30x30 m
  mb.rect(10.0, 0.0, 10.4, 25.5); // 竖墙，只在上方留 4.5 m 的绕行空间
  auto map = mb.build();

  auto run = [&](bool use_window) {
    MemoryParamReader p;
    p.setBool("astar.use_search_window", use_window);
    p.setDouble("astar.search_window_margin", 2.0);
    p.setDoubleArray("astar.search_window_retry_margins", {30.0});
    auto planner = makePlanner(p);
    planner->setCostMap(map);
    return planner->plan(
        PlanRequest{mkPose(1.0, 20.0, 0.0), mkPose(20.0, 20.0, 0.0)});
  };

  const PlanResult windowed = run(true);
  ASSERT_TRUE(windowed.ok())
      << toString(windowed.status) << " / " << windowed.message;
  double max_y = 0.0;
  for (const auto &pt : windowed.path)
    max_y = std::max(max_y, pt.y);
  EXPECT_GT(max_y, 25.0) << "应当绕过墙顶";
  EXPECT_GE(windowed.stats.windows_tried, 2)
      << "首个窗口装不下绕行路径时，应当扩大窗口重试（实际尝试 "
      << windowed.stats.windows_tried << " 次）";

  const PlanResult full = run(false);
  ASSERT_TRUE(full.ok()) << toString(full.status);
  EXPECT_EQ(full.stats.windows_tried, 1);
  EXPECT_NEAR(windowed.stats.path_length, full.stats.path_length, 1.0)
      << "开/关窗口不应改变路径量级";
  std::printf(
      "      [窗口] 开窗口 %d 次尝试 / %ld 节点；整图 1 次 / %ld 节点\n",
      windowed.stats.windows_tried, windowed.stats.expanded_nodes,
      full.stats.expanded_nodes);
}

// ==========================================================================
// 14) 剪枝后仍然安全（防止"只查中心线"的剪枝把路径拉过障碍）
// ==========================================================================
TEST(AStarPlanner, PrunedPathStaysCollisionFree) {
  MapBuilder mb(240, 240);
  // L 形障碍：迫使路径拐弯，剪枝若只查中心线就会从角上切过去
  mb.rect(2.0, 2.0, 3.0, 9.0);
  mb.rect(2.0, 9.0, 9.0, 10.0);
  MemoryParamReader p;
  p.setDouble("footprint.length", 0.70);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.safe_margin", 0.05);
  p.setDouble("astar.path_prune_max_span", 12.0);
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());

  const PlanResult r =
      planner->plan(PlanRequest{mkPose(1.0, 5.0, 0.0), mkPose(5.0, 11.0, 0.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status) << " / " << r.message;
  std::string why;
  EXPECT_TRUE(pathFreeBruteForce(
      *planner->costMap(), planner->footprintParams(), kHard, r.path, &why))
      << why;
  std::printf("      [剪枝] 输出 %zu 个点，长度 %.2f m\n", r.path.size(),
              r.stats.path_length);
}

// ==========================================================================
// 15) 距离场快路径确实生效（完整检查次数应远小于节点扩展数）
// ==========================================================================
TEST(AStarPlanner, FastPathReducesFullChecks) {
  MapBuilder mb(400, 400);
  MemoryParamReader p;
  auto planner = makePlanner(p);
  planner->setCostMap(mb.build());
  const PlanResult r = planner->plan(
      PlanRequest{mkPose(1.0, 1.0, 0.0), mkPose(18.0, 18.0, 0.0)});
  ASSERT_TRUE(r.ok()) << toString(r.status);
  std::printf("      [快路径] 扩展 %ld 节点，完整 footprint 检查 %ld 次\n",
              r.stats.expanded_nodes, r.stats.footprint_full_checks);
  EXPECT_LT(r.stats.footprint_full_checks, r.stats.expanded_nodes)
      << "开阔图里绝大多数节点应当被距离场快路径直接放行";
}

} // namespace
} // namespace pnc_2d
