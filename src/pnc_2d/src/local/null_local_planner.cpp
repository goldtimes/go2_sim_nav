#include "pnc_2d/local/null_local_planner.hpp"

#include <utility>

namespace pnc_2d {

bool NullLocalPlanner::configure(const ParamReader &params) {
  // 没有自己的参数。读一个不存在的键只是为了把 prefix 记进日志，
  // 顺便证明"配置来自 ParamReader"这条链路是通的。
  (void)params.getInt("local_null.placeholder", 0);
  return true;
}

void NullLocalPlanner::setGlobalPlan(const std::vector<Pose2D> &path) {
  plan_ = path;
}

void NullLocalPlanner::setCorridor(const RouteCorridor *corridor) {
  corridor_ = corridor; // 只是记下来（mode() 会因此变成 kRoute），不做控制
}

void NullLocalPlanner::setSpeedLimit(double v_limit) { speed_limit_ = v_limit; }

void NullLocalPlanner::setCostMap(
    std::shared_ptr<const CostMap2D> local_inflated) {
  local_map_ = std::move(local_inflated);
}

void NullLocalPlanner::setDistanceField(const LocalDistanceField *field) {
  dist_field_ = field;
}

void NullLocalPlanner::setDynamicObstacles(
    const std::vector<DynamicObstacle> &obs) {
  dynamic_obs_ = obs;
}

LocalPlanResult NullLocalPlanner::computeCommand(const Pose2D &pose,
                                                 double dt) {
  (void)pose;
  (void)dt;
  ++command_calls_;

  LocalPlanResult r;
  r.cmd = Twist2D{}; // 恒为 0：这不是"停车指令"，而是"没有指令"
  if (!hasPlan()) {
    r.status = LocalStatus::kIdle;
    r.message = "没有可跟随的路径";
  } else {
    r.status = LocalStatus::kFollowing;
    r.message = "NullLocalPlanner：只回报状态，不产生速度指令";
  }
  return r;
}

void NullLocalPlanner::reset() {
  plan_.clear();
  corridor_ = nullptr;
  speed_limit_ = 0.0;
  dynamic_obs_.clear();
  command_calls_ = 0;
}

} // namespace pnc_2d
