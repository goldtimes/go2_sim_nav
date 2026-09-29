#include "pnc_2d/core/clear_map_recovery.hpp"

#include <string>

namespace pnc_2d {

bool ClearMapRecoveryBehavior::configure(const ParamReader &params) {
  min_interval_s_ = params.getDouble(name() + ".min_interval_s", min_interval_s_);
  if (min_interval_s_ < 0.0)
    min_interval_s_ = 0.0;
  return true;
}

RecoveryResult ClearMapRecoveryBehavior::run(const RecoveryContext &ctx) {
  RecoveryResult r;
  if (!ctx.clearLocalCostMap) {
    r.success = false;
    r.message = "调用方没有注入 clearLocalCostMap —— 恢复行为无法生效"
                "（检查管理器是否配了感知清图服务名）";
    return r;
  }
  const auto now_tp = std::chrono::steady_clock::now();
  if (cleared_once_ && min_interval_s_ > 0.0) {
    const double since = std::chrono::duration<double>(now_tp - last_clear_).count();
    if (since < min_interval_s_) {
      r.success = false;
      r.message = "距上次清图仅 " + std::to_string(since) + " s（< " +
                  std::to_string(min_interval_s_) +
                  " s）→ 不清图（防抖），如实报恢复失败";
      return r;
    }
  }
  ctx.clearLocalCostMap();
  last_clear_ = now_tp;
  cleared_once_ = true;
  r.success = true;
  r.message = "已清空局部代价地图（3D 占据/2D 层/距离场一起作废，下一帧起按新观测"
              "重建）；状态机随后会按新地图重规划 → 幽灵障碍/旧观测残留属于这一类";
  return r;
}

} // namespace pnc_2d
