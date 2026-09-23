#include "pnc_2d/core/replan_recovery.hpp"

#include <string>

namespace pnc_2d {

bool ReplanRecoveryBehavior::configure(const ParamReader &params) {
  min_interval_s_ =
      params.getDouble(name() + ".min_interval_s", min_interval_s_);
  if (min_interval_s_ < 0.0)
    min_interval_s_ = 0.0;
  return true;
}

RecoveryResult ReplanRecoveryBehavior::run(const RecoveryContext &ctx) {
  RecoveryResult r;
  if (!ctx.requestReplan) {
    r.success = false;
    r.message = "调用方没有注入 requestReplan —— 恢复行为无法生效";
    return r;
  }
  // 防抖：刚要过路径就再要一次，只会拿到同一条路径、同样被挡（白烧恢复额度）。
  if (ctx.timeSinceLastPlan) {
    const double since = ctx.timeSinceLastPlan();
    if (since >= 0.0 && since < min_interval_s_) {
      r.success = false;
      r.message = "距上次规划仅 " + std::to_string(since) + " s（< " +
                  std::to_string(min_interval_s_) +
                  " s）→ 现在重规划只会得到同一条路径，先如实报恢复失败";
      return r;
    }
  }
  ctx.requestReplan();
  r.success = true;
  r.message = "已请求按当前地图重新规划（禁用/新增区域、地图更新、定位跳变、"
              "动态障碍占道都属于这一类）";
  return r;
}

} // namespace pnc_2d
