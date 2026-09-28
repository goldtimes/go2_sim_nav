// HeadingShimPlanner：**原地转向装饰器**（对应 Nav2 的
// RotationShimController）。
//
// 为什么把"转向"从跟踪控制器里拿出来单独做一层（M5.2）：
//   ① 旋转与跟踪是两件事。MPC 的目标函数是"贴线 + 前进"的二次型，车头差几十度
//      时它只有解空间里的妥协（"边转边走"），而"原地不动"往往更便宜 ⇒
//      **拒动**。 实测（2026-09-22）严格走廊下航向差 ≥30° ⇒ 120/120
//      周期被挡、一步不走。
//   ② 旋转的**执行规律**与跟踪不同：原地转就是"按角速度指令转、按角加速度斜坡
//      加减速"，不需要解优化。一个 √(2·α·|e|) 的减速律就能同时做到"起步有劲"
//      （不会给出落进底盘死区的小指令）和"到点不冲过头"。
//   ③ **谁控车头必须只有一处**。原来 MPC 里有**两套**对正（起步对正 `align_*`
//      与到点对正 `goal_yaw_*`），阈值/增益各配一份、判据各写一遍。收拢到本类后
//      MPC 变成**纯跟踪器**（见 doc/minco_trajectory_plan.md §M5.2）。
//
// 工作方式（装饰器 ⇒ **local 节点零改动**）：
//   配置 `local.type: heading_shim` + `shim.primary: mpc`。每个控制周期：
//     ①
//     **先问主控制器**（必须问：它的投影/进度/参考剖面状态要保持新鲜，交棒才平滑；
//        它同时还提供 `cross_track` 等诊断量）；
//     ② 再判"要不要原地转"；要 ⇒ **覆盖**成 (v=0, ω=本类给的) + kFollowing；
//     ③ 否则原样放行主控制器的结果（含 kGoalReached / kBlocked / kFailed）。
//
// 与 MPC 原来那两套对正的**有意差异**（都是修正，别当成行为回归）：
//   · 起步对正看**前方 `forward_sampling_distance`
//   处的朝向**，而不是车脚下那一点的
//     朝向。原因：全局路径首点的 yaw 被 `astar.keep_start_yaw` 保留成"当前车头"
//     ⇒ 看脚下永远算不出该转多少（e≈0）⇒ 起步对正在真实链路上**等于没生效**。
//     取前方一点的朝向才是"这条路要我朝哪走"。
//   · 旋转前做 **footprint 扫掠检查**（预测转向过程中每个采样 yaw 都查一次），
//     而不是"车心到障碍的距离 ≥ 某个魔数"。前者回答"会不会扫到"，后者要么过保守
//     要么不够。
//   · 角速度**没有下限参数**：√ 减速律在"需要动"的地方给出的值就远大于底盘死区
//     （`align_w_min` 那种补丁的存在本身就是增益式控制律的缺陷）。
//
// ⚠ 已知行为（要知情，不是 bug）：前方朝向差在**急弯前**就会超阈值 ⇒ 车会在弯前
//   停一下、转正、再走（nav2 的 RotationShimController 同样如此）。要减轻就把
//   `shim.forward_sampling_distance` 调小或把 `shim.engage_deg` 调大。

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/local_planner.hpp"

namespace pnc_2d {

class HeadingShimPlanner : public LocalPlanner {
public:
  /// 为什么在转（诊断与单测用；也决定用哪个阈值交棒）
  enum class RotationReason {
    kNone = 0,
    kHeading, ///< 与前方 `forward_sampling_distance` 处的朝向差太大
    kGoalYaw, ///< 已到终点位置，机头没对到**目标朝向**
  };

  struct Params {
    /// 被装饰的主控制器（`createLocalPlanner()` 认得的名字）
    std::string primary{"mpc"};

    // ---------------- 触发（迟滞） ----------------
    /// 与前方朝向差超过它 ⇒ 开始原地转 [°]；0 = 关闭本功能
    double engage_deg{45.0};
    /// 已经在转时，转到它以下就交棒 [°]。**必须 <
    /// engage_deg**（否则会在阈值上抖）
    double disengage_deg{22.5};
    /// 前瞻距离 [m]：拿路径前方这么远的那一点的朝向当目标（见文件头说明）
    double forward_sampling_distance{0.50};

    // ---------------- 旋转控制律 ----------------
    /// 旋转角速度上限 [rad/s]（也是"匀速段"的值）
    double omega_rot{0.60};
    /// 角加速度上限 [rad/s²]：既是斜坡限幅，也是减速律里的 α。
    /// ★ 取 nav2 `RotationShimController` 的默认 3.2，不是随便取的大数：
    ///   · 太小 ⇒ 斜坡起步那几拍的 |ω| 落在**底盘角速度死区**里（实测 ~0.20
    ///     rad/s），车会先"卡"零点几秒才动 —— 那正是旧 `align_w_min`
    ///     要打的补丁；
    ///   · 太大 ⇒ 与底盘实际能力不符（R10
    ///   的初衷就是别在交棒处制造不可能的跳跃）。 注意 R10 只看 `min(omega_rot,
    ///   √(2·α·disengage))`，`omega_rot` 一般才是主导 项，所以调 α
    ///   不会把交棒角速度顶上去。
    double alpha_max{3.20};
    /// true = 用实测角速度（`currentW()`）当斜坡起点；false = 用上一条指令。
    /// 本仓底盘低速时 `odom.twist` 不可信 ⇒ 默认 false；节点侧用
    /// `local.vel_from_pose` 从位姿差分算出来的 ω 也可以，那种情况再打开。
    bool closed_loop{false};

    // ---------------- 终点朝向 ----------------
    /// 目标朝向容差 [°]；0 = 不判定朝向。**节点侧到点判定取的就是这个值**
    /// （`goalYawTolerance()`）⇒ "算法怎么对正"与"节点怎么判"是同一份数据。
    double goal_yaw_tolerance_deg{0.0};
    /// 距终点位置多近才管朝向 [m]（太远时该专心走）。
    /// ⚠ 必须 **> 节点的到点位置容差**（`local.goal_tolerance`），否则会出现
    ///   "节点已判到点、算法还在等朝向"的死锁。
    double goal_yaw_align_distance{0.10};

    /// **无进展**超时 [s]（0 = 不限时）：一段时间内偏差没有实质改善（>2°）⇒ 报
    /// kFailed 并说清还差多少度。不是墙钟：180° @ 0.6 rad/s 本来就慢。
    double rotation_timeout_s{6.0};

    // ---------------- 旋转前的碰撞检查 ----------------
    /// 是否做旋转扫掠检查（关掉 = 盲转，只用于对照实验）
    bool check_rotation{true};
    /// 前向模拟多久的转向扫掠 [s]（每个控制周期都重判一次）
    double simulate_ahead_time{1.0};
    /// **没有局部图**时的退路：用车心到最近障碍的距离 ≥ 本值 [m]。
    /// 有局部图时不用它（那时用 footprint 扫掠，见类注释）。
    double min_clearance{0.25};
    /// 局部图里 **未知格**是否当障碍。
    /// ★ 默认 false，与
    /// `common.unknown_as_occupied`（全局图的口径，那里没有未知格）
    ///   **故意不同**：局部图是感知的滑动窗，没观测到 ≠ 有障碍；当障碍会让车在
    ///   新区域里永远不敢转身。
    bool unknown_as_occupied{false};
  };

  HeadingShimPlanner();
  // ⚠ 不声明析构函数：一旦声明（哪怕 `=
  // default`），隐式**移动**构造/赋值就被抑制，
  //   而这个类持有 unique_ptr（主控制器）⇒
  //   它既不能拷贝也不能移动，单测里按值返回 就编译不过。默认析构已经够用。

  std::string type() const override { return "heading_shim"; }
  bool configure(const ParamReader &params) override;

  // ---------------- 输入：既要给主控制器，也要进基类 ----------------
  // 基类的 plan_/corridor_/speed_limit_/...
  // 是**节点会直接读**的（mode()/hasPlan()），
  // 所以本类必须自己存一份，同时转发给主控制器。少转发一样的后果是"节点看到的世界"
  // 与"算法看到的世界"不一致。
  void setGlobalPlan(const std::vector<Pose2D> &path) override;
  void setCorridor(const RouteCorridor *corridor) override;
  void setSpeedLimit(double v_limit) override;
  void setZoneSpeedLimit(double v_limit) override;
  void setCostMap(std::shared_ptr<const CostMap2D> local_map) override;
  void setDistanceField(const LocalDistanceField *field) override;
  void setReferenceProfile(const ReferenceProfile *profile) override;
  void setDynamicObstacles(const std::vector<DynamicObstacle> &obs) override;
  void setCurrentVelocity(double v, double w) override;

  LocalPlanResult computeCommand(const Pose2D &pose, double dt) override;
  void reset() override;

  // ---------------- 转发给主控制器（节点会问这些） ----------------
  bool producesCmdVel() const override;
  double stopCoast() const override;
  double corridorTolerance() const override;
  double maxSpeed() const override;
  double brakeAcc() const override;
  double maxOmega() const override;
  /// ★ 本类自己的（不是主控制器的）：到点判定要的**目标朝向**容差
  double goalYawTolerance() const override {
    return p_.goal_yaw_tolerance_deg * M_PI / 180.0;
  }

  std::string diagString() const override;

  // ---------------- 诊断/单测 ----------------
  bool rotating() const { return rotating_; }
  RotationReason rotationReason() const { return reason_; }
  /// 本周期"目标朝向"（不在转时无意义）
  double rotationTargetYaw() const { return target_yaw_; }
  /// 本周期剩余的朝向偏差 [rad]（目标 − 当前；正 = 要逆时针转）
  double rotationError() const { return e_yaw_; }
  /// 本单位最近一次"为什么没能转"（空 = 没有；诊断用）
  const std::string &lastBlockReason() const { return block_reason_; }

  /// 交棒瞬间的角速度 [rad/s]：`min(omega_rot, √(2·α·disengage))`。
  ///
  /// 用途：① R10 配置校验（交棒角速度必须 ≤ 主控制器能给的 `maxOmega()`，否则
  ///   交棒那一刻指令被钳、动作打折）；② 单测断言"交棒时仍在转"（不是慢慢挪到
  ///   零再交棒 —— 那样主控制器要从 0 重新加速，接缝会看出来）。
  double handoffOmega() const;

  const Params &params() const { return p_; }
  const LocalPlanner *primary() const { return primary_.get(); }

  /// 单测用：替换主控制器（生产走 `configure()` 里的工厂）。
  /// 用途：造"主控制器报 kBlocked"这种用真算法很难稳定复现的状态。
  void setPrimaryForTest(std::unique_ptr<LocalPlanner> primary) {
    primary_ = std::move(primary);
  }

  /// 单测用：路径投影（返回弧长 s，`cross` 填横向偏差）
  double projectOnPlan(double x, double y, double *cross) const;
  /// 单测用：路径弧长 s 处的位姿。
  /// ★ **yaw 一律取折线切线**（= 路的走向），不取 yaw 通道的插值 —— 原因见 .cpp
  ///   （yaw 通道的首点被 `keep_start_yaw`
  ///   占成"当前车头"，插值会把起步该转的角度 稀释掉：实测 70°
  ///   的起步偏差被稀释到 9.5° ⇒ 永远不触发）。
  Pose2D samplePlan(double s) const;

private:
  /// 本周期是否要原地转，以及往哪转、转到什么时候算完
  struct RotationRequest {
    RotationReason reason{RotationReason::kNone};
    double target_yaw{0.0};
    double e{0.0};   ///< wrapAngle(target − pose.yaw)
    double tol{0.0}; ///< |e| ≤ tol 即交棒 [rad]
  };

  RotationRequest decideRotation(const Pose2D &pose,
                                 const LocalPlanResult &base) const;
  /// 旋转角速度（√ 减速律 + 角加速度斜坡；见文件头）
  double rotationRate(double e, double dt) const;
  /// 扫掠检查：本周期发的转向会不会扫到障碍
  bool rotationIsSafe(const Pose2D &pose, double target_yaw, double dt,
                      std::string &why) const;
  /// 无进展超时（返回非空 = 已超时，内容是要报的失败消息）
  std::string checkRotationTimeout(double e_rad);

  Params p_{};
  std::unique_ptr<LocalPlanner> primary_;

  /// 旋转碰撞检查用的轮廓表（与全局规划器**同一份** `footprint.*` 参数）
  FootprintCollisionChecker collision_;
  std::unique_ptr<ClearanceField> clearance_;
  FootprintParams fp_{};
  int hard_threshold_{80};

  // ---- 旋转状态 ----
  bool rotating_{false};
  RotationReason reason_{RotationReason::kNone};
  double target_yaw_{0.0};
  double e_yaw_{0.0};
  double last_cmd_w_{0.0}; ///< 上一条发出的角速度（开环时的斜坡起点）
  std::string block_reason_;

  // ---- 无进展超时 ----
  std::chrono::steady_clock::time_point since_{};
  double best_deg_{1e9};
};

} // namespace pnc_2d
