// 状态机的**事件集**（ROS-free，可单测）。
//
// 事件是"外面发生了什么事"，**不带动作**：动作由转移表决定（见 manager_sm.hpp
// 的 SideEffect）。这样"什么时候做什么"全部集中在表里，节点只负责把 ROS 世界
// 翻译成事件、把 SideEffect 翻译成 ROS 调用。
//
// 事件从哪来（P4 实现）：
//   kGoalReceived  ← /goal_pose
//   kPlanOk/Fail   ← PlanPath 服务返回值
//   kReached       ← FollowPath action 的 result.goal_reached
//   kBlocked       ← action 的 result.blocked（局部已连续被挡超时）
//   kFollowFail    ← action 的 result.failed / action abort
//   kStuck         ← 节点自己检测（跟随中长时间没有前进，见 manager 的 stuck
//   判据） kOdomJump      ←
//   定位跳变（与上一次位姿偏差过大，缓存的位姿已不可信） kRecoveryDone/Fail ←
//   恢复行为执行结果（P6 才有实现） kCancel        ← 显式取消服务 /
//   新目标顶掉旧目标

#pragma once

#include <cstdint>

namespace pnc_2d {

enum class Event : uint8_t {
  kGoalReceived = 0, ///< 收到新目标（RViz 2D Goal Pose / 服务）
  kPlanOk,           ///< 全局规划成功
  kPlanFail,         ///< 全局规划失败（NO_PATH / 超时 / 输入非法…）
  kReached,          ///< 到达目标（局部报 GOAL_REACHED）
  kBlocked,          ///< 前方被挡且不可绕（局部报 BLOCKED 超时）
  kStuck,            ///< 跟随中卡住（长时间没有实际前进）
  kFollowFail,       ///< 局部规划失败/内部错误/缺输入
  kOdomJump,         ///< 定位跳变：缓存的路径基于旧位姿，必须重规划
  kRecoveryDone,     ///< 恢复行为成功
  kRecoveryFail,     ///< 恢复行为失败（或本阶段根本没有可用行为）
  kCancel,           ///< 取消当前任务
};

/// 枚举名，用于日志与单测失败信息
const char *toString(Event e);

} // namespace pnc_2d
