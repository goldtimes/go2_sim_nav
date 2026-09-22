// 局部规划算法的抽象基类（对齐 ROS 1 nav_core::BaseLocalPlanner）。
//
// 设计要点（见 doc/mpc_local_planner_plan.md）：
//   1. 库 **ROS-free**：只依赖 ParamReader / CostMap2D / 距离场 / 走廊数据，
//      单测不必起 ROS；
//   2. **一套算法覆盖两种模式**：模式不是参数，而是"有没有走廊"这个数据事实 ——
//      setCorridor() 非空 ⇒ route profile（贴线 + 走廊硬约束），否则 free profile
//      （跟踪全局路径 + 距离场避障）。见 mode()；
//   3. 输入分四类：路径 / 走廊 / 地图与距离场 / 动态障碍；输出只有一个：
//      computeCommand() 给出 (v, ω) 与本周期状态；
//   4. 谁产生 cmd_vel 由 producesCmdVel() 声明：NullLocalPlanner 返回 false
//      （"我不产生速度"），状态机据此决定是否转发，避免"零速度=停车"被误读成有效指令。

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

/// 距离场（ESDF）查询接口：P5 实现（`core/local_distance_field.hpp`）。
/// 这里只前向声明，避免 P3 就引入实现细节。
class LocalDistanceField;

/// 速度指令（底盘接口是差速：(v, ω)）
struct Twist2D {
  double v{0.0};  ///< 前向速度 [m/s]，v ≥ 0（不倒车）
  double w{0.0};  ///< 角速度 [rad/s]
};

/// 局部规划的诊断信息（进 `/pnc_2d/local_status`，也用于日志/调参）
struct LocalStats {
  double solve_ms{0.0};      ///< 本周期的求解耗时 [ms]
  int solver_iter{0};        ///< 求解器迭代数（OSQP 的 iter）
  double cross_track{0.0};   ///< 当前横向偏差 [m]
  double progress{0.0};      ///< 路径进度 0~1
  double time_to_goal{0.0};  ///< 预估剩余时间 [s]
  int corridor_violations{0};///< 本周期被硬约束修正的次数（>0 说明贴线吃紧）
};

/// 一个控制周期的结果
struct LocalPlanResult {
  LocalStatus status{LocalStatus::kIdle};
  std::string message;  ///< 失败/降级时说明原因，不允许空着
  Twist2D cmd;
  LocalStats stats;

  bool ok() const
  {
    // kDegraded 也算"有可用结果"：它意味着**仍在跟踪**，只是缺距离场而保守限速。
    // 把它排除在外会让调用方把"降级但正常"当成失败（P5.2 的测试就踩过这个）。
    return status == LocalStatus::kFollowing ||
           status == LocalStatus::kGoalReached ||
           status == LocalStatus::kDegraded;
  }
};

class LocalPlanner {
public:
  /// 运行模式：**由数据驱动**（有没有走廊），不是配置项
  enum class Mode { kFree, kRoute };

  virtual ~LocalPlanner() = default;

  // ---------------- 子类必须实现 ----------------
  /// 算法名（用于 local.type 匹配与日志）
  virtual std::string type() const = 0;
  /// 读参数（前缀如 "local_mpc."）；取不到就用默认值
  virtual bool configure(const ParamReader &params) = 0;

  /// 全局路径（世界系，已填 yaw）。空或 <2 点 ⇒ 视为"没有路径"
  virtual void setGlobalPlan(const std::vector<Pose2D> &path) = 0;
  /// 走廊约束；**传 nullptr 表示进入 free 模式**（这是模式切换的唯一入口）
  virtual void setCorridor(const RouteCorridor *corridor) = 0;
  /// 限速（来自路网 speed_limit / 限速区）；<=0 表示不限制
  virtual void setSpeedLimit(double v_limit) = 0;

  /// 局部膨胀图（硬碰撞判定用；可能为空 = 还没收到）
  virtual void setCostMap(std::shared_ptr<const CostMap2D> local_inflated) = 0;
  /// 距离场（软代价/引导用；可能为空 = 降级运行）
  virtual void setDistanceField(const LocalDistanceField *field) = 0;
  /// 动态障碍（v1 反应式：只收不用；接口先留好，见 §7）
  virtual void setDynamicObstacles(const std::vector<DynamicObstacle> &obs) = 0;

  /// 核心：给定当前位置与周期 dt，给出本周期指令与状态
  virtual LocalPlanResult computeCommand(const Pose2D &pose, double dt) = 0;
  /// 清空内部状态（换路径/换图/任务取消后调用）
  virtual void reset() = 0;
  /// 本算法是否会真正输出速度指令（Null = false）
  /// 算法特有的**诊断字符串**（可选，默认空）。
  ///
  /// 为什么放在接口上：调参时要看的是算法内部量（参考速度、限速上界、曲率、
  /// 生效的约束行数……），但节点**不该认识具体算法**（不该 include / dynamic_cast
  /// 到 MpcLocalPlanner —— 那样每加一个算法就要改节点，也把算法与节点绑死了）。
  /// 于是让算法自己把想被看到的东西格式化成一行，节点只负责打。
  virtual std::string diagString() const { return {}; }

  /// **停车惯性距离** [m]：从"指令变 0"到"车真的停住"，底盘还会自己走多远。
  ///
  /// 节点用它做"到点判定"的提前量：判定要比的是 `剩余距离 − stopCoast()` 与容差，
  /// 否则 ① 判定点就等于车的停点，到点误差 = 整车停车惯性（实测 Go2 仿真 ≈4 cm，
  /// 而用户要求 ≤ 3 cm）；② 提前量写错会反过来：车已停在惯性段内、剩余距离
  /// 永远大于容差 ⇒ 任务卡死等超时。
  ///
  /// 放在接口上而不是节点参数里：它是**算法的终点剖面**用的量（见
  /// `MpcParams::stop_coast`），节点再配一遍就会两边不一致。
  /// 默认 0 = 底盘无惯性（速度可瞬时归零的模型）。
  virtual double stopCoast() const { return 0.0; }

  virtual bool producesCmdVel() const = 0;

  // ---------------- 可选实现（基类给默认行为）----------------
  /// 当前底盘速度（来自 odom 的 twist）。
  /// 纯几何算法（pure_pursuit 之类）可以忽略；**MPC 必须知道 v** ——
  /// 它的状态量是 s = [x, y, θ, v]，缺了 v 就只能假设"车当前静止"，
  /// 于是每周期都从 0 开始加速（实测会表现为走走停停）。
  /// 不放进 computeCommand() 的参数里，是因为那会污染所有算法的签名。
  virtual void setCurrentVelocity(double v, double w)
  {
    v_now_ = v;
    w_now_ = w;
  }
  double currentV() const { return v_now_; }
  double currentW() const { return w_now_; }

  // ---------------- 基类提供（公共状态与只读查询）----------------
  Mode mode() const
  {
    return (corridor_ != nullptr && corridor_->valid()) ? Mode::kRoute
                                                        : Mode::kFree;
  }
  bool hasPlan() const { return plan_.size() >= 2; }
  const std::vector<Pose2D> &globalPlan() const { return plan_; }
  const RouteCorridor *corridor() const { return corridor_; }
  double speedLimit() const { return speed_limit_; }
  const CostMap2D *localCostMap() const { return local_map_.get(); }
  const LocalDistanceField *distanceField() const { return dist_field_; }
  const std::vector<DynamicObstacle> &dynamicObstacles() const
  {
    return dynamic_obs_;
  }
  /// 是否处于降级（缺距离场 / 距离场超时）—— 子类可覆盖补充自己的判据
  virtual bool degraded() const { return dist_field_ == nullptr; }

protected:
  std::vector<Pose2D> plan_;
  const RouteCorridor *corridor_{nullptr};  ///< 非拥有
  double speed_limit_{0.0};
  std::shared_ptr<const CostMap2D> local_map_;
  const LocalDistanceField *dist_field_{nullptr};  ///< 非拥有
  std::vector<DynamicObstacle> dynamic_obs_;
  double v_now_{0.0};  ///< 见 setCurrentVelocity()
  double w_now_{0.0};
};

}  // namespace pnc_2d
