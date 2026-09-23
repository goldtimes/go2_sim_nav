// 状态机的**状态集**（ROS-free，可单测）。
//
// 对齐 ROS 1 move_base 的做法：状态是**显式枚举 + 显式转移表**，
// 不是"一堆 if 拼出来的隐式行为"。好处：
//   · 全部转移可以单测穷举（见 test/test_state_machine.cpp）；
//   · 出问题时日志能直接说"在 FOLLOWING 收到 Blocked → RECOVERING"，
//     而不是让人猜代码走到哪了。
//
// 状态集（P4 定案）：
//   Idle → Planning → Following → GoalReached
//   Planning 失败 → Failed；Following 被挡/卡住 → Recovering →（成功回
//   Following / 超限 → Failed）

#pragma once

#include <cstdint>

namespace pnc_2d {

enum class State : uint8_t {
  kIdle = 0,    ///< 空闲：没有任务
  kPlanning,    ///< 正在请求全局规划（等 PlanPath 服务返回）
  kFollowing,   ///< 正在跟随路径（等 action 反馈/结果）
  kGoalReached, ///< 到达目标（等新目标或 Cancel）
  kRecovering, ///< 执行恢复行为（P4 无实现 ⇒ 必然失败，见 manager_sm.cpp
               ///< 的说明）
  kFailed, ///< 本次任务失败（等新目标或 Cancel 复位）
};

/// 枚举名，用于日志与 /pnc_2d/state 的 state_name 字段
const char *toString(State s);

} // namespace pnc_2d
