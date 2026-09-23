// NullLocalPlanner：不做任何控制的"空"实现（P3 交付，`local.type: none`）。
//
// 存在意义：
//   1. 让 P4 的三节点链路**能空跑**：manager → global(service) → local(action)
//   → 状态机，
//      但**不发任何速度指令**（producesCmdVel() = false）；
//   2. 作为"局部规划器"接口的最小契约测试对象（工厂/状态机单测都用它）；
//   3. 明确区分于"真实算法还没接好"：它报告 kFollowing 表示"我给你了路径，
//      但我不负责走"—— 状态机据此知道现在不该期望 cmd_vel。

#pragma once

#include "pnc_2d/core/local_planner.hpp"

namespace pnc_2d {

class NullLocalPlanner : public LocalPlanner {
public:
  std::string type() const override { return "null"; }

  /// 没有参数：accept 一切配置（但读一下 prefix，便于日志里看到配了什么）
  bool configure(const ParamReader &params) override;

  void setGlobalPlan(const std::vector<Pose2D> &path) override;
  void setCorridor(const RouteCorridor *corridor) override;
  void setSpeedLimit(double v_limit) override;
  void setCostMap(std::shared_ptr<const CostMap2D> local_inflated) override;
  void setDistanceField(const LocalDistanceField *field) override;
  void setDynamicObstacles(const std::vector<DynamicObstacle> &obs) override;

  /// 永远返回 cmd = 0；状态只反映"有没有可跟随的路径"
  LocalPlanResult computeCommand(const Pose2D &pose, double dt) override;
  void reset() override;
  /// ★ 关键：声明"我不产生速度指令"，状态机据此决定是否转发 cmd_vel
  bool producesCmdVel() const override { return false; }
  /// Null 不依赖距离场，缺它也不算降级
  bool degraded() const override { return false; }

  /// 诊断：被调用的次数（单测断言链路真的走到了这里）
  long commandCalls() const { return command_calls_; }

private:
  long command_calls_{0};
};

} // namespace pnc_2d
