// HeadingShimPlanner 单测（M5.2 验收项）。
//
// 这一层要钉住四件事（对应 doc/minco_trajectory_plan.md §M5.2 的"单测"四条）：
//   ① 大航向差 ⇒ **先原地转正再走**（转的过程中 v=0、不产生横向位移）；
//   ② 迟滞：转到 disengage 以下才交棒，且**交棒瞬间仍在转**（ω 不是慢慢挪到
//   0）； ③ 转不过去（扫掠会碰）⇒ **不转**，如实交回主控制器 / 如实报 BLOCKED；
//   ④ R10：交棒角速度 ≤ 主控制器能给的 `maxOmega()`。
//
// 另外还钉住"装饰器不能把主控制器的状态吃掉"这一类 bug（转发与被转发量），
// 那类 bug 不报错、只是行为悄悄变差（zoneSpeedLimit / setCurrentVelocity
// 都属此类）。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/local/heading_shim_planner.hpp"
#include "pnc_2d/local/mpc_local_planner.hpp"

namespace pnc_2d {
namespace {

constexpr double kPi = M_PI;

double deg2rad(double d) { return d * kPi / 180.0; }
double rad2deg(double r) { return r * 180.0 / kPi; }

double wrap(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a <= -kPi)
    a += 2.0 * kPi;
  return a;
}

/// 被控对象：差速运动学，圆弧精确积分（常 (v, ω) 下无离散误差）
void integrate(Pose2D &s, double v, double w, double dt) {
  if (std::fabs(w) > 1e-9) {
    const double R = v / w;
    const double th1 = s.yaw + w * dt;
    s.x += R * (std::sin(th1) - std::sin(s.yaw));
    s.y -= R * (std::cos(th1) - std::cos(s.yaw));
    s.yaw = wrap(th1);
  } else {
    s.x += v * dt * std::cos(s.yaw);
    s.y += v * dt * std::sin(s.yaw);
    s.yaw = wrap(s.yaw);
  }
}

/// 从 (x0, y0) 起、沿 yaw
/// 方向的直线（**车要放在栅格中央**：轮廓判定把图外当致命，
/// 车放在图角上会被误判成"永远在碰撞"——这坑踩过）
std::vector<Pose2D> straightPathFrom(double x0, double y0, double length,
                                     double res, double yaw = 0.0) {
  std::vector<Pose2D> p;
  for (double s = 0.0; s <= length + 1e-9; s += res)
    p.push_back(
        Pose2D{x0 + s * std::cos(yaw), y0 + s * std::sin(yaw), yaw, true});
  return p;
}

std::vector<Pose2D> straightPath(double length, double res, double yaw = 0.0) {
  return straightPathFrom(0.0, 0.0, length, res, yaw);
}

/// “空旷”距离场（所有格 = max_dist）：不降级才能量真正的行为
LocalDistanceField freeSpaceField() {
  const double res = 0.1;
  const double x0 = -20.0, y0 = -20.0;
  const int w = 400, h = 400;
  std::vector<DistanceSample> samples;
  samples.reserve(static_cast<std::size_t>(w) * h);
  for (int iy = 0; iy < h; ++iy)
    for (int ix = 0; ix < w; ++ix)
      samples.push_back(
          DistanceSample{x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res, 5.0});
  LocalDistanceField f;
  EXPECT_TRUE(f.buildFromSamples(x0, y0, res, w, h, samples, 5.0, 0));
  return f;
}

/// 一张只读局部栅格（世界坐标矩形填充），与 test_astar_planner 里的 MapBuilder
/// 同款
class MapBuilder {
public:
  MapBuilder(int w, int h, double res)
      : w_(w), h_(h), res_(res), data_(static_cast<std::size_t>(w) * h, 0) {}

  MapBuilder &rect(double x0, double y0, double x1, double y1, int8_t v = 100) {
    for (int cy = 0; cy < h_; ++cy) {
      for (int cx = 0; cx < w_; ++cx) {
        const double wx = (cx + 0.5) * res_;
        const double wy = (cy + 0.5) * res_;
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1)
          data_[static_cast<std::size_t>(cy) * w_ + cx] = v;
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
  int w_, h_;
  double res_;
  std::vector<int8_t> data_;
};

/// shim 的常用参数（主控制器 = mpc，量测用的小速度）
void setShimParams(MemoryParamReader &p, double engage = 45.0,
                   double disengage = 22.5, double goal_yaw_tol = 0.0) {
  p.setDouble("shim.engage_deg", engage);
  p.setDouble("shim.disengage_deg", disengage);
  p.setDouble("shim.forward_sampling_distance", 0.50);
  p.setDouble("shim.omega_rot", 0.65);
  p.setDouble("shim.alpha_max", 3.20); // nav2 RotationShimController 的默认值
  p.setDouble("shim.goal_yaw_tolerance_deg", goal_yaw_tol);
  p.setDouble("shim.goal_yaw_align_distance", 0.10);
  p.setDouble("shim.rotation_timeout_s",
              0.0); // 默认不判超时（除专测超时的用例）
  p.setDouble("shim.check_rotation", true);
  p.setDouble("shim.simulate_ahead_time", 1.0);
  p.setDouble("shim.min_clearance", 0.25);
  // 主控制器（MPC）参数
  p.setDouble("local_mpc.v_max", 0.42);
  p.setDouble("local_mpc.w_max", 0.65);
  p.setDouble("local_mpc.reference_speed", 0.35);
  p.setDouble("local_mpc.a_max", 0.6);
}

HeadingShimPlanner makeShim(MemoryParamReader &p,
                            const LocalDistanceField &field,
                            const std::vector<Pose2D> &path,
                            const CostMap2D *map = nullptr) {
  HeadingShimPlanner shim;
  EXPECT_TRUE(shim.configure(p));
  shim.setDistanceField(&field);
  if (map != nullptr) {
    auto m = std::make_shared<CostMap2D>();
    *m = *map;
    shim.setCostMap(m);
  }
  shim.setGlobalPlan(path);
  shim.setCurrentVelocity(0.0, 0.0);
  return shim;
}

// ==================================================== ① 先原地转正再走

TEST(HeadingShim, RotatesInPlaceWhenHeadingOffsetIsLarge) {
  // 用户 2026-09-22 定的行为：车头差得多时**先原地转，对好再走**。
  // 30/60/90° 三档都要"转"，而且转的过程必须真是原地（横向位移 ≈ 0）。
  //   本用例把阈值调成 25°/12°，**顺便验证阈值本身是可配置的**
  //   （默认 45°/22.5° 下 30° 不该转，见 PassesThroughWhenHeadingIsAligned）。
  auto field = freeSpaceField();
  for (double off_deg : {30.0, 60.0, 90.0}) {
    MemoryParamReader p;
    setShimParams(p, /*engage=*/25.0, /*disengage=*/12.0);
    auto shim = makeShim(p, field, straightPath(5.0, 0.05));
    Pose2D pose{0.0, 0.0, deg2rad(off_deg), true};
    const double dt = 0.05;

    int rotate_cycles = 0, drive_cycles = 0;
    double max_abs_y_while_rotating = 0.0;
    double first_w = 0.0;
    for (int k = 0; k < 400; ++k) {
      const auto r = shim.computeCommand(pose, dt);
      ASSERT_TRUE(r.ok()) << toString(r.status) << " " << r.message;
      if (shim.rotating()) {
        ++rotate_cycles;
        EXPECT_NEAR(r.cmd.v, 0.0, 1e-9) << "原地转期间不该发线速度";
        EXPECT_GT(std::fabs(r.cmd.w), 1e-3) << "原地转期间应该在转";
        if (first_w == 0.0)
          first_w = r.cmd.w;
        // 转向必须朝目标（车头偏在左侧 +off ⇒ 要顺时针转回来 ⇒ ω < 0）
        EXPECT_LT(r.cmd.w * deg2rad(off_deg), 0.0) << "转向反了";
        max_abs_y_while_rotating =
            std::max(max_abs_y_while_rotating, std::fabs(pose.y));
      } else if (r.cmd.v > 1e-3) {
        ++drive_cycles;
      }
      integrate(pose, r.cmd.v, r.cmd.w, dt);
      if (rotate_cycles > 0 && drive_cycles > 3)
        break;
    }
    std::printf("  [原地转 %2.0f°] 转 %d 周期（首周期 ω=%.3f）→ 走 %d 周期，"
                "转期间 |y| ≤ %.4f m\n",
                off_deg, rotate_cycles, first_w, drive_cycles,
                max_abs_y_while_rotating);
    EXPECT_GT(rotate_cycles, 0) << off_deg << "° 下没有原地转";
    EXPECT_GT(drive_cycles, 3) << off_deg << "° 下转完没有恢复行驶";
    EXPECT_LT(max_abs_y_while_rotating, 0.02)
        << "转的过程不是原地（出现了横向位移）";
  }
}

TEST(HeadingShim, LookaheadIgnoresKeptStartYaw) {
  // ★ 这条钉住"为什么要看**前方**那一点"（M5.2 相对旧实现的修正）。
  //   全局规划器会把路径**首点**的 yaw
  //   保留成"当前车头"（astar.keep_start_yaw），
  //   所以"看车脚下那一点"永远算不出该转多少（e≈0）—— 旧实现就是这样失效的。
  auto field = freeSpaceField();
  auto path = straightPath(5.0, 0.05, 0.0);
  path[0].yaw = deg2rad(60.0); // 首点朝向 = 车头（模拟 keep_start_yaw）
  path[0].has_yaw = true;

  MemoryParamReader p;
  setShimParams(p);
  auto shim = makeShim(p, field, path);

  // 车在起点、车头 60°：首点 yaw 也 60°（看脚下 ⇒ e=0），但**路的走向是 0°**。
  const auto r =
      shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(60.0), true}, 0.05);
  std::printf("  [首点朝向被保留] rotating=%d e=%.1f° msg=%s\n",
              static_cast<int>(shim.rotating()), rad2deg(shim.rotationError()),
              r.message.c_str());
  EXPECT_TRUE(shim.rotating()) << "看脚下那一点 ⇒ 被 keep_start_yaw "
                                  "骗了、永远不转（这正是旧实现的毛病）";
  EXPECT_NEAR(rad2deg(shim.rotationError()), -60.0, 1.0);
  // 前瞻点的朝向 = **折线切线**（路的走向），不是首点那个"当前车头"
  const Pose2D look = shim.samplePlan(0.50);
  EXPECT_NEAR(rad2deg(look.yaw), 0.0, 1e-6)
      << "samplePlan 应该返回折线切线（路的走向）";
}

TEST(HeadingShim, LookaheadTangentIsNotDilutedByASparseFirstSegment) {
  // ★★ 真实链路里踩到的坑（2026-09-28）：折线是**稀疏**的（首点 →
  // 下一个拐点 3.7 m），
  //   而首点 yaw 被保留成车头。如果前瞻取"两端 yaw 插值"，0.5 m 处插出来的是
  //   车头与路方向的中间值：车头 −90°、路 −20° ⇒ 插值 −80.5° ⇒ e 只有 9.5°
  //   （< 45°）⇒ **不接管**。实测后果：起步航向差 70° 时装饰器一次都没触发，
  //   车照旧边转边走、头 0.9 m 横向甩出 23 cm。取切线后同一个场景 e=70°。
  auto field = freeSpaceField();
  std::vector<Pose2D> path; // 稀疏折线：只有两个点
  path.push_back(Pose2D{0.0, 0.0, deg2rad(-90.0), true});    // 首点 yaw = 车头
  path.push_back(Pose2D{3.52, -1.28, deg2rad(-20.0), true}); // 路的走向 = −20°

  MemoryParamReader p;
  setShimParams(p);
  auto shim = makeShim(p, field, path);
  const auto r =
      shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(-90.0), true}, 0.05);
  std::printf("  [稀疏折线] rotating=%d e=%.1f°（路走向 %.1f°）\n",
              static_cast<int>(shim.rotating()), rad2deg(shim.rotationError()),
              rad2deg(shim.samplePlan(0.5).yaw));
  EXPECT_TRUE(shim.rotating()) << "稀疏折线下又把起步偏差稀释掉了";
  EXPECT_NEAR(rad2deg(shim.rotationError()), 70.0, 2.0);
  (void)r;
}

/// 一个假的"被挡"主控制器：永远报 kBlocked + 零指令。
/// 用途：验证"主控制器被挡 + 车头偏着 ⇒ 接管转正"这条触发（真算法很难稳定复现
/// 严格走廊里那个"一步不走"的状态）。
class StuckPlanner : public LocalPlanner {
public:
  std::string type() const override { return "stuck"; }
  bool configure(const ParamReader &) override { return true; }
  void setGlobalPlan(const std::vector<Pose2D> &) override {}
  void setCorridor(const RouteCorridor *) override {}
  void setSpeedLimit(double) override {}
  void setCostMap(std::shared_ptr<const CostMap2D>) override {}
  void setDistanceField(const LocalDistanceField *) override {}
  void setDynamicObstacles(const std::vector<DynamicObstacle> &) override {}
  LocalPlanResult computeCommand(const Pose2D &, double) override {
    LocalPlanResult r;
    r.status = LocalStatus::kBlocked;
    r.message = "前方被挡且不可绕（假实现，测试用）";
    return r;
  }
  void reset() override {}
  bool producesCmdVel() const override { return true; }
};

TEST(HeadingShim, TakesOverWhenPrimaryIsBlockedAndHeadingIsOff) {
  // ★ 严格走廊里纯跟踪器在航向差 20~30° 就**拒动**（实测 120/120 周期被挡、
  //   一步不走）。那个角度**低于** 45° 的进入阈值 ⇒ 只看角度永远触发不了 ⇒
  //   车卡死。 所以"主控制器自己被挡 +
  //   车头偏着"也是一条触发条件（不需要再拍一个阈值）。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p); // engage 45° / disengage 22.5°

  // ① 30° 偏差（< engage 45°）但主控制器被挡 ⇒ 应该接管转正
  HeadingShimPlanner shim;
  ASSERT_TRUE(shim.configure(p));
  shim.setPrimaryForTest(std::make_unique<StuckPlanner>());
  shim.setDistanceField(&field);
  shim.setGlobalPlan(straightPath(5.0, 0.05));
  const auto r =
      shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(30.0), true}, 0.05);
  std::printf("  [被挡+30°] rotating=%d cmd v=%.3f w=%.3f\n",
              static_cast<int>(shim.rotating()), r.cmd.v, r.cmd.w);
  EXPECT_TRUE(shim.rotating()) << "被挡 + 车头偏 30° 却不接管 ⇒ 严格走廊会卡死";
  EXPECT_NEAR(r.cmd.v, 0.0, 1e-9);
  EXPECT_LT(r.cmd.w, 0.0) << "车头偏左 30° ⇒ 应顺时针转回来";

  // ② 10° 偏差：被挡也不值得专门为它转（偏差 < disengage 22.5°）⇒ 如实上报被挡
  HeadingShimPlanner shim2;
  ASSERT_TRUE(shim2.configure(p));
  shim2.setPrimaryForTest(std::make_unique<StuckPlanner>());
  shim2.setDistanceField(&field);
  shim2.setGlobalPlan(straightPath(5.0, 0.05));
  const auto r2 =
      shim2.computeCommand(Pose2D{0.0, 0.0, deg2rad(10.0), true}, 0.05);
  EXPECT_FALSE(shim2.rotating())
      << "10° 也转 ⇒ 门槛太松（会把小偏差搞成停车转身）";
  EXPECT_EQ(r2.status, LocalStatus::kBlocked) << "被挡要如实上报，不能被吞掉";
}

TEST(HeadingShim, PassesThroughWhenHeadingIsAligned) {
  // 基线：朝向已经很好 ⇒ **不许**触发转向（装饰器不能把正常行驶弄坏）
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p); // 默认 engage 45° / disengage 22.5°
  auto shim = makeShim(p, field, straightPath(5.0, 0.05));
  const auto r =
      shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(2.0), true}, 0.05);
  EXPECT_FALSE(shim.rotating()) << "2° 的偏差不该触发转向";
  EXPECT_GT(r.cmd.v, 0.0) << "应该照常往前走";
  EXPECT_EQ(shim.rotationReason(), HeadingShimPlanner::RotationReason::kNone);

  // 30° < engage(45°)：交给主控制器"边转边走"，本层不接管
  auto shim2 = makeShim(p, field, straightPath(5.0, 0.05));
  const auto r2 =
      shim2.computeCommand(Pose2D{0.0, 0.0, deg2rad(30.0), true}, 0.05);
  std::printf("  [30° < engage 45°] rotating=%d cmd v=%.3f\n",
              static_cast<int>(shim2.rotating()), r2.cmd.v);
  EXPECT_FALSE(shim2.rotating()) << "没到进入阈值就接管了（阈值没生效）";
}

// ==================================================== ② 迟滞与交棒

TEST(HeadingShim, HysteresisAndHandoffKeepsTurning) {
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, /*engage=*/45.0, /*disengage=*/22.5);
  auto shim = makeShim(p, field, straightPath(5.0, 0.05));

  // ① 已经在转：偏差掉到 engage 以下（40°）**仍然要转**（否则会在阈值上抖）
  shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(60.0), true}, 0.05);
  ASSERT_TRUE(shim.rotating());
  shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(40.0), true}, 0.05);
  EXPECT_TRUE(shim.rotating()) << "40° < engage(45°) 就停转 ⇒ 迟滞没生效";

  // ② 掉到 disengage(22.5°) 以下 ⇒ 交棒（rotating=false）
  const double deg = 20.0;
  const auto r =
      shim.computeCommand(Pose2D{0.0, 0.0, deg2rad(deg), true}, 0.05);
  EXPECT_FALSE(shim.rotating()) << "20° < disengage(22.5°) 应该交棒";
  EXPECT_GT(r.cmd.v, 0.0) << "交棒后应由主控制器接管（这里是「往前走」）";

  // ③ 交棒瞬间的角速度必须仍然很大（不是"慢慢挪到 0 再交"）
  std::printf(
      "  [交棒] 交棒角速度 %.3f rad/s（ω上限 %.3f，主控制器 maxω %.3f）\n",
      shim.handoffOmega(), shim.params().omega_rot, shim.maxOmega());
  EXPECT_GE(shim.handoffOmega(), 0.30)
      << "交棒时角速度太小 ⇒ 主控制器要从接近 0 重新加速，接缝看得出来";
}

TEST(HeadingShim, ConfigurationHandoffOmegaFitsPrimaryLimit) {
  // R10：**交棒动力学不可行** —— shim 交棒时的角速度大于主控制器能给的 maxω
  // 时， 交棒那一刻指令被钳、动作打折。这条用例把"出厂参数必须满足"钉住。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5);
  auto shim = makeShim(p, field, straightPath(5.0, 0.05));
  const double primary_max = shim.maxOmega();
  ASSERT_GT(primary_max, 0.0) << "主控制器没报 maxω ⇒ 装饰器无法自检";
  std::printf("  [R10] 交棒 ω=%.3f ≤ 主控制器 maxω=%.3f × 1.05=%.3f\n",
              shim.handoffOmega(), primary_max, primary_max * 1.05);
  EXPECT_LE(shim.handoffOmega(), primary_max * 1.05)
      << "出厂参数就违反 R10（交棒会被主控制器钳掉）";
}

// ==================================================== ③ 转不过去就不转

TEST(HeadingShim, HandsBackWhenRotationWouldSweepIntoObstacle) {
  // 车旁边有一面碍（起始姿态是空的，转起来才扫到）⇒
  // 不许盲转，而是交回主控制器。
  //   ★ 为什么要"转起来才扫到"这个构造：它才能验证判据是在**扫掠过程**上判的，
  //     而不是只看当前姿态（只看当前姿态的实现会放行这次转向）。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p);
  p.setBool("footprint.enable", true);
  p.setDouble("footprint.length", 0.70);
  p.setDouble("footprint.width", 0.40);
  p.setDouble("footprint.safe_margin", 0.05);
  p.setInt("common.hard_threshold", 80);

  // 车头 0°，而**路的走向**是 60° ⇒ 要往 +y 侧转（车体在 y 方向的投影会变大）
  const double cx = 5.0,
               cy = 5.0; // 车放栅格中央（图外 = 致命，放角落会被误判）
  const std::vector<Pose2D> path =
      straightPathFrom(cx, cy, 5.0, 0.05, deg2rad(60.0));
  const auto tight =
      MapBuilder(200, 200, 0.05).rect(4.0, cy + 0.30, 6.0, cy + 3.0).build();
  const auto open =
      MapBuilder(200, 200, 0.05).rect(4.0, cy + 0.80, 6.0, cy + 3.0).build();

  // ① 墙在 0.30 m外：当前姿态（yaw 0，y 方向只占 0.25）是空的，但转起来会扫到
  auto shim = makeShim(p, field, path, tight.get());
  const auto r = shim.computeCommand(Pose2D{cx, cy, 0.0, true}, 0.05);
  std::printf("  [扫掠会碰] rotating=%d cmd v=%.3f w=%.3f | %s\n",
              static_cast<int>(shim.rotating()), r.cmd.v, r.cmd.w,
              shim.lastBlockReason().c_str());
  EXPECT_FALSE(shim.rotating()) << "扫描会碰到墙却还在原地转";
  EXPECT_FALSE(shim.lastBlockReason().empty()) << "不转要说清为什么";

  // ② 同样的朝向差、墙退到 0.80 m ⇒ 正常转（对照，证明①不是"永远不转"）
  auto shim2 = makeShim(p, field, path, open.get());
  shim2.computeCommand(Pose2D{cx, cy, 0.0, true}, 0.05);
  EXPECT_TRUE(shim2.rotating()) << "有空间却不转 ⇒ 判据太保守";
}

TEST(HeadingShim, ReportsBlockedWhenGoalYawRotationIsImpossible) {
  // 已到终点位置、机头差 60°，但旁边就是障碍 ⇒ **必须报出来**。
  //   为什么不静默：报 kFollowing 会让节点判超时、报 kGoalReached
  //   会让"没对正"变成 "到达"。报 kBlocked
  //   才能让上游看到"到了但转不了"这个事实。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5, /*goal_yaw_tol=*/5.0);
  p.setBool("footprint.enable", true);
  p.setInt("common.hard_threshold", 80);
  // 车头 0°、目标朝向 90°：车体在 y 方向的投影会从 0.25 涨到 0.40 ⇒
  //   起始姿态离墙 0.30 m 是空的，转起来会扫到。
  const double cx = 5.0, cy = 5.0;
  const auto tight =
      MapBuilder(200, 200, 0.05).rect(4.0, cy + 0.30, 6.0, cy + 3.0).build();
  auto path = straightPathFrom(cx, cy, 0.20, 0.05);
  path.back().yaw = deg2rad(90.0);

  auto shim = makeShim(p, field, path, tight.get());
  // 车已经贴到末点（0.02 m < goal_yaw_align_distance 0.10）
  const auto r = shim.computeCommand(Pose2D{cx + 0.18, cy, 0.0, true}, 0.05);
  std::printf("  [到点转不了] %s：%s\n", toString(r.status), r.message.c_str());
  EXPECT_EQ(r.status, LocalStatus::kBlocked);
  EXPECT_NE(r.message.find("目标朝向"), std::string::npos) << r.message;
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.0) << "不能一边说没空间一边硬转";
}

// ==================================================== 终点朝向对正（从 MPC
// 搬来）

TEST(HeadingShim, AlignsGoalYawInPlaceAtGoal) {
  // 语义（原 MPC 的 `AlignsGoalYawInPlaceAtGoal`，M5.2 搬到本层）：
  //   位置到了（剩余 ≤ goal_yaw_align_distance）而 |目标朝向误差| > 容差 ⇒
  //   原地转， 转进容差才算完成。目标朝向 = 路径**最后一点**的 yaw。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5, /*goal_yaw_tol=*/5.0);
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  auto shim = makeShim(p, field, path);

  Pose2D s{4.97, 0.0, deg2rad(60.0), true};
  const double dt = 0.05;
  int rotate_cycles = 0;
  for (int k = 0; k < 400; ++k) {
    const auto r = shim.computeCommand(s, dt);
    if (!shim.rotating())
      break;
    ++rotate_cycles;
    EXPECT_EQ(shim.rotationReason(),
              HeadingShimPlanner::RotationReason::kGoalYaw);
    EXPECT_NEAR(r.cmd.v, 0.0, 1e-9) << "对正期间不该前进";
    integrate(s, r.cmd.v, r.cmd.w, dt);
  }
  const double err_deg = std::fabs(rad2deg(wrap(s.yaw - 0.0)));
  std::printf("  [终点朝向] 转 %d 周期 → yaw 误差 %.2f°（容差 5°）\n",
              rotate_cycles, err_deg);
  EXPECT_GT(rotate_cycles, 0);
  EXPECT_LE(err_deg, 5.0 + 1e-6) << "对正没收敛到容差内";
  EXPECT_NEAR(s.x, 4.97, 0.02) << "原地对正不该把位置带跑";
  // 节点侧查询：容差按弧度给，且与配置一致
  EXPECT_NEAR(shim.goalYawTolerance(), deg2rad(5.0), 1e-12);
}

TEST(HeadingShim, GoalYawIsIgnoredWhenToleranceIsZero) {
  // 容差 0 = 不判定朝向（旧行为）⇒ 到了就算到了，不许因为"机头歪着"卡住
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5, /*goal_yaw_tol=*/0.0);
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  auto shim = makeShim(p, field, path);
  const auto r =
      shim.computeCommand(Pose2D{4.97, 0.0, deg2rad(60.0), true}, 0.05);
  EXPECT_NE(shim.rotationReason(),
            HeadingShimPlanner::RotationReason::kGoalYaw);
  EXPECT_DOUBLE_EQ(shim.goalYawTolerance(), 0.0);
  (void)r;
}

TEST(HeadingShim, GoalYawRotationTimesOutWithoutProgress) {
  // 转不动就如实报失败（不许无限原地转 —— 在任务层看就是卡死）。
  //   ★ 是"无进展"超时而不是墙钟：180° @ 0.65 rad/s ≈ 5 s 本来就慢。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5, /*goal_yaw_tol=*/10.0);
  p.setDouble("shim.rotation_timeout_s", 0.02); // 20 ms，便于单测
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  auto shim = makeShim(p, field, path);

  const auto r1 =
      shim.computeCommand(Pose2D{4.97, 0.0, deg2rad(60.0), true}, 0.05);
  ASSERT_TRUE(shim.rotating());
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  // 位姿不动 = 车被卡住转不动
  const auto r2 =
      shim.computeCommand(Pose2D{4.97, 0.0, deg2rad(60.0), true}, 0.05);
  std::printf("  [对正超时] %s：%s\n", toString(r2.status), r2.message.c_str());
  EXPECT_EQ(r2.status, LocalStatus::kFailed) << "超时必须报失败，不能无限转";
  EXPECT_NE(r2.message.find("超时"), std::string::npos);
  EXPECT_NE(r2.message.find("shim.omega_rot"), std::string::npos)
      << "失败原因要指向可能的原因（含参数名）";
  EXPECT_FALSE(shim.rotating());
  EXPECT_DOUBLE_EQ(r2.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(r2.cmd.w, 0.0);
  (void)r1;
}

TEST(HeadingShim, DoesNotTimeOutWhileProgressing) {
  // 慢但一直在收敛 ⇒ 不许被判超时（按墙钟就会）
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p, 45.0, 22.5, /*goal_yaw_tol=*/10.0);
  p.setDouble("shim.rotation_timeout_s", 0.05); // 远小于总耗时
  auto path = straightPath(5.0, 0.05);
  path.back().yaw = 0.0;
  auto shim = makeShim(p, field, path);

  double deg = 60.0;
  for (int k = 0; k < 8; ++k) {
    const auto r =
        shim.computeCommand(Pose2D{4.97, 0.0, deg2rad(deg), true}, 0.05);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(50)); // 每次都超过"墙钟"
    EXPECT_NE(r.status, LocalStatus::kFailed)
        << "剩 " << deg << "° 时被误判超时（按墙钟计了？）";
    EXPECT_TRUE(shim.rotating());
    deg -= 5.0;
  }
  std::printf(
      "  [对正进展] 60°→20° 共 8 周期（每周期 > 墙钟超时）未被判失败 ✓\n");
}

// ==================================================== 装饰器：转发与诊断

TEST(HeadingShim, ForwardsChassisAndLimitQueriesToPrimary) {
  // 这一类 bug 不报错，只是行为悄悄变差（节点按错的上限做判定）：
  //   · stopCoast/corridorTolerance 决定**到点判定**的提前量；
  //   · maxSpeed/brakeAcc 决定限速区**前瞻距离**；
  //   · zoneSpeedLimit 决定"前方有限速区"是否被主控制器看见。
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p);
  p.setDouble("local_mpc.stop_coast", 0.035);
  p.setDouble("local_mpc.corridor_min_tolerance", 0.05);
  p.setDouble("local_mpc.brake_acc", 0.15);
  auto shim = makeShim(p, field, straightPath(5.0, 0.05));

  EXPECT_TRUE(shim.producesCmdVel());
  EXPECT_DOUBLE_EQ(shim.stopCoast(), 0.035);
  EXPECT_DOUBLE_EQ(shim.corridorTolerance(), 0.05);
  EXPECT_DOUBLE_EQ(shim.maxSpeed(), 0.42);
  EXPECT_DOUBLE_EQ(shim.brakeAcc(), 0.15);
  EXPECT_GT(shim.maxOmega(), 0.0);

  // 限速区帽：写进装饰器还不够，主控制器必须也拿到（否则它照旧按 v_max 跑）
  shim.setZoneSpeedLimit(0.12);
  EXPECT_DOUBLE_EQ(shim.zoneSpeedLimit(), 0.12);
  const auto *mpc = static_cast<const MpcLocalPlanner *>(shim.primary());
  ASSERT_NE(mpc, nullptr);
  EXPECT_DOUBLE_EQ(mpc->zoneSpeedLimit(), 0.12)
      << "限速区帽没转发给主控制器 ⇒ 限速区在局部等于失效";

  // 当前速度也要转发：MPC 的状态量含 v，缺了它每周期从 0 加速（走走停停）
  shim.setCurrentVelocity(0.3, 0.0);
  EXPECT_DOUBLE_EQ(shim.currentV(), 0.3);
  EXPECT_DOUBLE_EQ(mpc->currentV(), 0.3) << "v 没转发给主控制器";
}

TEST(HeadingShim, KeepsPrimaryDiagnosticsVisible) {
  // 装饰器把主控制器的诊断吞掉 = 把现场最有用的一行日志弄没了
  auto field = freeSpaceField();
  MemoryParamReader p;
  setShimParams(p);
  auto shim = makeShim(p, field, straightPath(5.0, 0.05));
  shim.computeCommand(Pose2D{0.0, 0.0, 0.0, true}, 0.05);
  const std::string diag = shim.diagString();
  const auto *mpc = static_cast<const MpcLocalPlanner *>(shim.primary());
  std::printf("  [诊断] %s\n", diag.c_str());
  ASSERT_FALSE(mpc->diagString().empty());
  EXPECT_NE(diag.find(mpc->diagString()), std::string::npos)
      << "主控制器的诊断行被吞了";
  EXPECT_NE(diag.find("shim"), std::string::npos);
}

TEST(HeadingShim, ReportsFailureWhenConfiguredWithUnknownPrimary) {
  MemoryParamReader p;
  p.setString("shim.primary", "no_such_controller");
  HeadingShimPlanner shim;
  EXPECT_FALSE(shim.configure(p))
      << "主控制器名字写错必须配置失败（不许静默回退）";
}

} // namespace
} // namespace pnc_2d
