// P6/M4：**参考速度剖面**（上游轨迹层 → 局部 MPC 的唯一下行数据）。
//
// 为什么按**弧长**索引（而不是时间）——2026-09-24 用户确认的"决定 A"：
//   · 局部 MPC 的参考本来就是弧长参数化的（`ref_s_` / `projectOntoReference` →
//     `progress_s_`），OCP 里消费的就是 v_ref(s) ⇒ 弧长是**天然**的接口；
//   · 按时间传需要 MPC 维护与上游一致的**时间基准**（停车/延迟/取消都要重同步）
//     —— 那是 M6 的活，而且脆；
//   · 弧长索引**对"停车再走"天然免疫**：被动态障碍挡 10 s 后再走，s 没跳、
//     v(s) 还是对的，不会出现"追进度"那种冲出去的问题。
//   丢掉的只有"必须在 t 时刻到 s"这种时序约束 —— 跟踪任务不需要。
//
// ★ 校验纪律：**长度不匹配 / 非单调 / 非有限 ⇒ 整体作废**，绝不半信半疑地用一半
//   （半份剖面 = 两套剖面，正是 R8/R9 警告的那类问题，而且从日志上看不出来）。
//
// 本文件**不依赖 ROS**，可直接单测；节点只负责把消息里的数组搬进来。

#pragma once

#include <string>
#include <vector>

namespace pnc_2d {

/// 参考剖面：弧长 s 严格递增，v/ω 与之逐点对齐。
///
/// ★★ **不要在这里直接暴露 `std::vector` 成员**（2026-09-24 血的教训）：
///   M4.1 的初版里有 `std::vector<double> s, v, w;` 三个**公共**成员，而真正被
///   `set()` 填的是私有的 `s_/v_/w_` ⇒ 两者同名、前者永远是空的。调用方写
///   `*std::max_element(p.v.begin(), p.v.end())` **编译得过**、单测也测不到
///   （单测只用 `speedAt/omegaAt`），现场却直接 SIGSEGV（对 `end()` 解引用）。
///   ⇒ 数据只留一份，且只通过**带语义的访问器**给出。
struct ReferenceProfile {
  /// 装载并校验。任一条不满足 ⇒ **整体作废**（返回 false，`err` 说明原因，
  /// 内部清空）。校验内容：
  ///   · 三个数组长度一致且 ≥ 2；
  ///   · 所有值有限（无 NaN/Inf）、v ≥ 0；
  ///   · `s` 严格递增。
  bool set(const std::vector<double> & s_in, const std::vector<double> & v_in,
           const std::vector<double> & w_in, std::string & err);

  void clear();
  bool valid() const { return valid_; }
  std::size_t size() const { return s_.size(); }
  /// 弧长总长 [m]（无效时为 0）
  double length() const { return valid_ ? s_.back() : 0.0; }

  /// v(s)：**线性插值**，端点处夹取（越界不报错 —— 调用方按"没有更多信息"处理）。
  double speedAt(double s_query) const;
  /// ω(s)：同上
  double omegaAt(double s_query) const;

  /// 弧长轴 / 速度 / 角速度（只读；长度一致，无效时为空）
  const std::vector<double> & sAxis() const { return s_; }
  const std::vector<double> & speeds() const { return v_; }
  const std::vector<double> & omegas() const { return w_; }
  /// 峰值速度 [m/s]（**无效或为空时返回 0，不会崩**）。
  /// 为什么把它收进结构体：日志/A-B 表每处都要打峰值，让调用方各自写
  /// `max_element` 就是反复往同一个坑里跳（见上面的说明）。
  double peakSpeed() const;

private:
  std::vector<double> s_;
  std::vector<double> v_;
  std::vector<double> w_;
  bool valid_{false};
};

}  // namespace pnc_2d
