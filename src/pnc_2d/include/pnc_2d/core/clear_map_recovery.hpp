// 恢复行为：**清空局部代价地图**（"清图"）。
//
// 为什么需要它（2026-09-29 用户要求 + 实测）：
//   `replan` 只解决"路径与当前局面不同源"，解决不了"**局部图本身是脏的**"：
//     · 动态障碍离开后，感知图里残留"幽灵障碍"（用户报："障碍物消失了，感知也没有
//       清除障碍物"）⇒ 车对着空处一直报 BLOCKED，重规划也还是同一条"被挡"的判定；
//     · 障碍被车顶开/定位跳变后，旧观测留在滑动窗口里，与当前局面不同源；
//     · 上层的逻辑（例如某个区域刚被开放）改过，而局部图还记着旧的占格。
//   这三类的正确处置都是**把局部图整个作废、按新观测重建**，而不是再规划一次。
//
// 代价（必须写清，不能只说好处）：
//   清图后 2D 层变成**未知**，在"未知按可通行"的配置下局部图会短暂地"什么都看不见"
//   （~0.2~0.5 s 内被新点云重建）。静态障碍的安全由 **map_server 全局图 + 局部融合**
//   兜底（`local.fuse_global_map`），所以不会因此撞已知的墙/柱子/货架。
//
// 与状态机的配合（与 `replan` 完全一致）：
//   `run()` 同步返回；`success=true` ⇒ `Recovering → Planning`（manager_sm 转移表）
//   ⇒ **清完图一定会重新规划再跟随**，所以本行为不需要自己请求重规划。
//
// 参数（前缀 = `clear_map.`）：
//   · `clear_map.min_interval_s`：两次清图的最小间隔 [s]，默认 **0.0（关）**。
//     与 `replan` 同理——反复"挡住→清图→再挡"已经被 `sm.max_recoveries` 收住了；
//     开它反而会把"清完图立刻又被真障碍挡住"这种**真**情况判成恢复失败。
//     用**单调时钟**（steady_clock）判，不跟仿真时间挂钩（清图是 I/O 动作，
//     按墙钟防抖才符合直觉）。

#pragma once

#include <chrono>
#include <string>

#include "pnc_2d/core/recovery_behavior.hpp"

namespace pnc_2d {

class ClearMapRecoveryBehavior : public RecoveryBehavior {
public:
  std::string name() const override { return "clear_map"; }
  bool configure(const ParamReader &params) override;
  RecoveryResult run(const RecoveryContext &ctx) override;
  void reset() override { last_clear_ = std::chrono::steady_clock::time_point{}; }

private:
  double min_interval_s_{0.0};
  /// 上一次真正调用清图钩子的时刻（墙钟）
  std::chrono::steady_clock::time_point last_clear_{};
  /// 是否清过（避免首次用 0 判断）
  bool cleared_once_{false};
};

} // namespace pnc_2d
