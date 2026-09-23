// 恢复行为的抽象基类（对齐 ROS 1 nav_core::RecoveryBehavior）。
//
// 接口与 cote（P3）不变；P5.4 接上了**第一个**具体实现
// `ReplanRecoveryBehavior`（见 replan_recovery.hpp）。仍未实现的：
// "清图" 与 "小幅移动" —— `nudge` 这一类物理动作属底盘范畴，
// 本阶段明确不做（用户 2026-09-23 决定）。
//
// 为什么用 std::function 注入而不是直接拿节点指针：
//   · 库保持 ROS-free，单测可以用假函数断言"到底被调了什么"；
//   · 恢复行为需要的"副作用"其实就三类（清图 / 请求重规划 / 小幅移动），
//     显式列出来比塞一个 Node& 更清楚，也防止它偷偷改别的状态。

#pragma once

#include <functional>
#include <string>

#include "pnc_2d/core/param_reader.hpp"

namespace pnc_2d {

/// 恢复行为能用的"手脚"：由调用方（节点/状态机）注入，行为本身不直接碰 ROS
struct RecoveryContext {
  /// 清空/重置局部代价地图（例如把来自旧地图的障碍清掉）
  std::function<void()> clearLocalCostMap;
  /// 请求一次全局重规划（回到规划阶段）
  std::function<void()> requestReplan;
  /// 小幅移动：v [m/s]、ω [rad/s]、持续时长 [s]；v<0 表示后退
  std::function<void(double v, double w, double duration_s)> nudge;
  /// 距离上一次成功规划过去了多久 [s]（用于判断"路径是不是已经过期"）
  std::function<double()> timeSinceLastPlan;
};

struct RecoveryResult {
  bool success{false};
  std::string message; ///< 失败时必须说明原因
};

class RecoveryBehavior {
public:
  virtual ~RecoveryBehavior() = default;

  /// 行为名（用于 params 前缀与日志）
  virtual std::string name() const = 0;
  /// 读参数（前缀 = name() + "."）
  virtual bool configure(const ParamReader &params) = 0;
  /// 执行一次恢复；**同步返回**（长动作由调用方决定超时）
  virtual RecoveryResult run(const RecoveryContext &ctx) = 0;
  /// 清空内部状态
  virtual void reset() {}
};

} // namespace pnc_2d
