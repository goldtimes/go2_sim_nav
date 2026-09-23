// 算法工厂：类型字符串 → 算法实例（全局/局部/恢复三套）。
// 新增算法时**只需**在这里加一行 + 在本文件对应的 available* 里加个名字
// （见 doc/pnc2d_dev_plan.md §4.3 的第 3 步）。

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/global_planner.hpp"
#include "pnc_2d/core/local_planner.hpp"
#include "pnc_2d/core/recovery_behavior.hpp"

namespace pnc_2d {

/// 创建全局规划实例；type 未注册时返回 nullptr（调用方决定是报错还是回退）
std::unique_ptr<GlobalPlanner> createPlanner(const std::string &type);

/// 已注册的全局类型名（用于报错提示与日志）
std::vector<std::string> availablePlanners();

/// 创建局部规划实例（`local.type`）
std::unique_ptr<LocalPlanner> createLocalPlanner(const std::string &type);

/// 已注册的局部类型名（目前只有 "null"）
std::vector<std::string> availableLocalPlanners();

/// 创建恢复行为实例（`recovery.type`）。本期（P3）**无实现**，总是 nullptr；
/// 接口先立着，P6 再填具体行为。
std::unique_ptr<RecoveryBehavior>
createRecoveryBehavior(const std::string &type);

/// 已注册的恢复行为名（本期为空）
std::vector<std::string> availableRecoveries();

} // namespace pnc_2d
