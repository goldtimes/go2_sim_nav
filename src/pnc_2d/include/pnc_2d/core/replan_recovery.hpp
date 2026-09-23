// 最小可用的恢复行为：**请求按当前地图重新规划**。
//
// 为什么先做它（而不是"清图 / 后退 / 旋转"这类动作）：
//   本仓库里"被挡"的绝大多数原因是**路径与当前局面不同源** —— 地图/区域刚改过、
//   定位跳变、动态障碍正好占住了路径、或者路径是旧位姿下算的。这些的正确处置都是
//   "用**当前**地图重新规划一次"，而不是让车在原地做动作（后退/旋转并不改变
//   路径本身不可用这件事）。
//
// 放在 `RecoveryBehavior` 接口里（而不是直接在管理器里判一下）：
//   · 工厂/参数 `recovery.type` 从 P3 起就留着这个槽位，接口注释里也明确写了
//     "清图 / 后退 / 请求重规划…" 就是这里的用途 —— 顺着既有设计走，别绕开它；
//   · 行为库保持 ROS-free，单测可以直接注入假回调断言"到底被调了什么"。
//
// 语义（与状态机对齐）：
//   · run() **同步返回**；`success=true` 表示"已请求重规划"，状态机收到
//     `RecoveryDone` 后会走 `Recovering → Planning → Following`（见 manager_sm
//     的 转移表：恢复成功后**先重规划再跟随**，否则旧路径会立刻再被挡）。
//   · 可选防抖（`replan.min_interval_s`，用 ctx.timeSinceLastPlan() 判）：
//     **默认 0.0 = 关闭**。为什么默认关："挡住→恢复→再挡"已经被状态机的
//     `max_recoveries` 额度收住了（超了就直接 FAILED），不需要第二道闸；
//     而防抖会把"规划后 0.3 s 才出现的动态障碍"这类**真可恢复**的情况
//     直接判成失败（而失败消息还会盖掉真正的原因）。要开就显式配。

#pragma once

#include <string>

#include "pnc_2d/core/recovery_behavior.hpp"

namespace pnc_2d {

class ReplanRecoveryBehavior : public RecoveryBehavior {
public:
  std::string name() const override { return "replan"; }
  bool configure(const ParamReader &params) override;
  RecoveryResult run(const RecoveryContext &ctx) override;
  void reset() override {}

private:
  /// 两次重规划之间的最小间隔 [s]（用 ctx.timeSinceLastPlan()
  /// 判）：小于它就说明
  /// "路径刚算过"，再规划一遍不会得到新结果，于是**如实报恢复失败**。
  /// 默认 0.0（关闭），理由见文件头。
  double min_interval_s_{0.0};
};

} // namespace pnc_2d
