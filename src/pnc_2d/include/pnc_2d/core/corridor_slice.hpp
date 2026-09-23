// 逐点走廊半宽 → "真正受走廊约束的那一段"（纯逻辑，可单测）
//
// 背景（2026-09-23）：hybrid 路网规划出来的路径是
//     [**自由入口段**, 路网段, **自由出口段**]
// 走廊只对**路网段**有意义。以前接口里只有一个标量半宽，于是入口段也被当成
// "严格贴线的中心线"，车在车道外几厘米处就被硬约束判死（实测：严格走廊半宽
// 0.05 m 时，起点横向偏差 ≥0.055 m ⇒ QP 不收敛、120/120 周期被挡、车一步不动，
// 现场现象就是"有角度的路网时机器基本不会动"）。
//
// 现在的约定（与 `FollowPath.action` / `PlanPath.srv` 的 `corridor_width[]`
// 一致）：
//     w[i] >  0  →  该点允许横向偏离 ±w[i]  （宽通道）
//     w[i] == 0  →  该点**严格贴线**（半宽 0；语义就是"遇障只能停"）
//     w[i] <  0  →  该点**没有走廊约束**（自由跟踪：入口/出口段）
//    数组为空     →  整条路径按自由跟踪（A* 自由空间任务）
//
// ⚠ 0 是"严格贴线"而不是"无走廊" —— 这两个语义必须分清，否则严格贴线的任务会
//   被当成自由任务（反过来更糟：自由入口段会被当成严格贴线）。

#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace pnc_2d {

/// 走廊段（闭区间下标，指到路径点）
struct CorridorSlice {
  bool valid{false};
  std::size_t begin{0};
  std::size_t end{0};     ///< 闭区间终点
  double half_width{0.0}; ///< 该段中最严的半宽 [m]（0 = 严格贴线）

  std::size_t size() const { return valid ? (end - begin + 1) : 0; }
};

/// 从逐点半宽数组切出走廊段。
///
/// · 空数组 / 全部 < 0 ⇒ `valid=false`（自由空间模式）
/// · 有多段（中间夹着自由段）时取 **第一段起点 ~ 最后一段终点**：本仓库的路径
///   最多只有一段路网段，中间若真夹着自由段，一并按走廊处理是**保守**的
///   （宁可严格，不可放任偏出）。
/// · `strict_when_absent` 只在数组为空时用于表达"调用方说它是严格走廊"，但那时
///   没有路径点信息，所以这里只把它体现在 `half_width=0` 且 `valid=true`…
///   实际上数组为空我们按自由处理；严格与否由调用方的 `strict_corridor` 决定。
inline CorridorSlice corridorSlice(const std::vector<double> &w) {
  CorridorSlice s;
  if (w.empty())
    return s;
  std::size_t first = w.size();
  std::size_t last = 0;
  double hw = 0.0;
  bool any = false;
  for (std::size_t i = 0; i < w.size(); ++i) {
    if (w[i] < 0.0)
      continue; // 该点无走廊约束
    if (!any) {
      first = i;
      hw = w[i];
      any = true;
    } else {
      hw = std::min(hw, w[i]);
    }
    last = i;
  }
  if (!any)
    return s;
  s.valid = true;
  s.begin = first;
  s.end = last;
  s.half_width = std::max(0.0, hw);
  return s;
}

} // namespace pnc_2d
