// 算法工厂：类型字符串 → 算法实例。
// 新增算法时**只需**在这里加一行（见 doc/pnc2d_dev_plan.md §4.3 的第 3 步）。

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/global_planner.hpp"

namespace pnc_2d {

/// 创建算法实例；type 未注册时返回 nullptr（调用方决定是报错还是回退）
std::unique_ptr<GlobalPlanner> createPlanner(const std::string & type);

/// 已注册的类型名（用于报错提示与日志）
std::vector<std::string> availablePlanners();

}  // namespace pnc_2d
