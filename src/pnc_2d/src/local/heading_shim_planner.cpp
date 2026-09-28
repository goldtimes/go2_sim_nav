// HeadingShimPlanner 的实现。设计说明见头文件。
//
// 本文件的三个要点，按"容易被改坏"的程度排序：
//   ①
//   **先问主控制器、再决定覆盖**（顺序不能反）：主控制器的投影/进度/参考剖面要
//      每周期刷新，交棒那一刻它才不是"从头开始"。
//   ② **交棒角速度**：√ 减速律 + 只有"远离目标"时才可能降速 ⇒ 交棒时 ω 仍然是
//      满值（不是慢慢挪到 0）。R10 要求它 ≤ 主控制器的 `maxOmega()`。
//   ③ **旋转安全**：footprint 扫掠（有局部图）→ ESDF 车心净距（没有图时的退路）
//      → 都没有就**不转**（不能证明安全就不做）。

#include "pnc_2d/local/heading_shim_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "pnc_2d/core/factory.hpp"
#include "pnc_2d/core/local_distance_field.hpp"

namespace pnc_2d {
namespace {

constexpr double kDeg = M_PI / 180.0;

double clampd(double v, double lo, double hi) {
  return std::min(std::max(v, lo), hi);
}

double toDeg(double rad) { return rad / kDeg; }

} // namespace

// ============================================================================
// 构造 / 配置
// ============================================================================

HeadingShimPlanner::HeadingShimPlanner() = default;

bool HeadingShimPlanner::configure(const ParamReader &params) {
  const std::string pre = "shim.";
  p_.primary = params.getString(pre + "primary", p_.primary);

  p_.engage_deg = params.getDouble(pre + "engage_deg", p_.engage_deg);
  p_.disengage_deg = params.getDouble(pre + "disengage_deg", p_.disengage_deg);
  p_.forward_sampling_distance = params.getDouble(
      pre + "forward_sampling_distance", p_.forward_sampling_distance);
  p_.omega_rot = params.getDouble(pre + "omega_rot", p_.omega_rot);
  p_.alpha_max = params.getDouble(pre + "alpha_max", p_.alpha_max);
  p_.closed_loop = params.getBool(pre + "closed_loop", p_.closed_loop);
  p_.goal_yaw_tolerance_deg = params.getDouble(pre + "goal_yaw_tolerance_deg",
                                               p_.goal_yaw_tolerance_deg);
  p_.goal_yaw_align_distance = params.getDouble(pre + "goal_yaw_align_distance",
                                                p_.goal_yaw_align_distance);
  p_.rotation_timeout_s =
      params.getDouble(pre + "rotation_timeout_s", p_.rotation_timeout_s);
  p_.check_rotation = params.getBool(pre + "check_rotation", p_.check_rotation);
  p_.simulate_ahead_time =
      params.getDouble(pre + "simulate_ahead_time", p_.simulate_ahead_time);
  p_.min_clearance = params.getDouble(pre + "min_clearance", p_.min_clearance);
  p_.unknown_as_occupied =
      params.getBool(pre + "unknown_as_occupied", p_.unknown_as_occupied);

  // ---- 合法性：宁可在启动时就把它改对，也不要在运行中给出奇怪的行为 ----
  if (p_.engage_deg <= 0.0)
    p_.engage_deg = 0.0; // 0 = 关闭"起步对正"
  if (p_.disengage_deg < 0.0)
    p_.disengage_deg = 0.0;
  // 退出阈值必须严格小于进入阈值，否则在阈值上每周期来回切（滞环失效）
  if (p_.engage_deg > 0.0 && p_.disengage_deg >= p_.engage_deg)
    p_.disengage_deg = 0.5 * p_.engage_deg;
  if (p_.forward_sampling_distance < 0.0)
    p_.forward_sampling_distance = 0.0;
  if (p_.omega_rot < 0.0)
    p_.omega_rot = 0.0;
  if (p_.alpha_max <= 0.0)
    p_.alpha_max =
        1e-3; // 斜坡/减速律都要用它，0 会让 ω 永远为 0（而且斜坡也失效）
  if (p_.goal_yaw_tolerance_deg < 0.0)
    p_.goal_yaw_tolerance_deg = 0.0;
  if (p_.goal_yaw_align_distance < 0.0)
    p_.goal_yaw_align_distance = 0.0;
  if (p_.simulate_ahead_time < 0.0)
    p_.simulate_ahead_time = 0.0;
  if (p_.min_clearance < 0.0)
    p_.min_clearance = 0.0;

  // ---- 车体轮廓：与全局规划器读**同一批 key**（跳层物理量只配一遍） ----
  fp_.enable = params.getBool("footprint.enable", fp_.enable);
  fp_.length = params.getDouble("footprint.length", fp_.length);
  fp_.width = params.getDouble("footprint.width", fp_.width);
  fp_.offset_x = params.getDouble("footprint.offset_x", fp_.offset_x);
  fp_.offset_y = params.getDouble("footprint.offset_y", fp_.offset_y);
  fp_.safe_margin = params.getDouble("footprint.safe_margin", fp_.safe_margin);
  if (fp_.length <= 0.0 || fp_.width <= 0.0)
    fp_.enable = false;
  if (fp_.safe_margin < 0.0)
    fp_.safe_margin = 0.0;
  hard_threshold_ = params.getInt("common.hard_threshold", hard_threshold_);
  if (hard_threshold_ < 1)
    hard_threshold_ = 1;
  if (hard_threshold_ > 100)
    hard_threshold_ = 100;
  collision_.configure(fp_, hard_threshold_, p_.unknown_as_occupied);

  // ---- 主控制器 ----
  primary_ = createLocalPlanner(p_.primary);
  if (!primary_) {
    return false;
  }
  if (!primary_->configure(params)) {
    primary_.reset();
    return false;
  }

  rotating_ = false;
  reason_ = RotationReason::kNone;
  block_reason_.clear();
  since_ = {};
  best_deg_ = 1e9;
  last_cmd_w_ = 0.0;
  return true;
}

// ============================================================================
// 输入：自己存一份（节点会直接读基类状态）+ 转发给主控制器
// ============================================================================

void HeadingShimPlanner::setGlobalPlan(const std::vector<Pose2D> &path) {
  plan_ = path;
  // 换路径 ⇒ 旋转状态作废（新路径的目标朝向完全不同）
  rotating_ = false;
  reason_ = RotationReason::kNone;
  since_ = {};
  best_deg_ = 1e9;
  if (primary_)
    primary_->setGlobalPlan(path);
}

void HeadingShimPlanner::setCorridor(const RouteCorridor *corridor) {
  corridor_ = corridor;
  if (primary_)
    primary_->setCorridor(corridor);
}

void HeadingShimPlanner::setSpeedLimit(double v_limit) {
  speed_limit_ = v_limit;
  if (primary_)
    primary_->setSpeedLimit(v_limit);
}

void HeadingShimPlanner::setZoneSpeedLimit(double v_limit) {
  // ★ 必须转发：限速区的动态帽是**位置相关**的，主控制器自己算速度上界要用它。
  //   （基类的默认实现只写自己的成员 ⇒ 不转发的话主控制器永远看不到限速区。）
  LocalPlanner::setZoneSpeedLimit(v_limit);
  if (primary_)
    primary_->setZoneSpeedLimit(v_limit);
}

void HeadingShimPlanner::setCostMap(std::shared_ptr<const CostMap2D> map) {
  local_map_ = map;
  if (map && map->valid()) {
    if (!clearance_)
      clearance_ = std::make_unique<ClearanceField>();
    if (!clearance_->build(*map, hard_threshold_, p_.unknown_as_occupied))
      clearance_.reset();
    collision_.configure(fp_, hard_threshold_, p_.unknown_as_occupied);
    collision_.setMap(map.get());
    collision_.setClearanceField(clearance_.get());
  } else {
    clearance_.reset();
    collision_.setMap(nullptr);
    collision_.setClearanceField(nullptr);
  }
  if (primary_)
    primary_->setCostMap(std::move(map));
}

void HeadingShimPlanner::setDistanceField(const LocalDistanceField *field) {
  dist_field_ = field;
  if (primary_)
    primary_->setDistanceField(field);
}

void HeadingShimPlanner::setReferenceProfile(const ReferenceProfile *profile) {
  if (primary_)
    primary_->setReferenceProfile(profile);
}

void HeadingShimPlanner::setDynamicObstacles(
    const std::vector<DynamicObstacle> &obs) {
  dynamic_obs_ = obs;
  if (primary_)
    primary_->setDynamicObstacles(obs);
}

void HeadingShimPlanner::setCurrentVelocity(double v, double w) {
  // ★ 必须转发：MPC 的状态量里有 v，缺了它每周期都从 0 开始加速（走走停停）。
  //   同时基类自己也要留一份（节点的 `currentV()` 是读基类的）。
  LocalPlanner::setCurrentVelocity(v, w);
  if (primary_)
    primary_->setCurrentVelocity(v, w);
}

void HeadingShimPlanner::reset() {
  rotating_ = false;
  reason_ = RotationReason::kNone;
  block_reason_.clear();
  since_ = {};
  best_deg_ = 1e9;
  last_cmd_w_ = 0.0;
  if (primary_)
    primary_->reset();
}

// ============================================================================
// 转发查询（节点会问这些；少转发一个的后果是节点按错的上限做判定）
// ============================================================================

bool HeadingShimPlanner::producesCmdVel() const {
  return primary_ ? primary_->producesCmdVel() : false;
}

double HeadingShimPlanner::stopCoast() const {
  return primary_ ? primary_->stopCoast() : 0.0;
}

double HeadingShimPlanner::corridorTolerance() const {
  return primary_ ? primary_->corridorTolerance() : 0.0;
}

double HeadingShimPlanner::maxSpeed() const {
  return primary_ ? primary_->maxSpeed() : 0.0;
}

double HeadingShimPlanner::brakeAcc() const {
  return primary_ ? primary_->brakeAcc() : 0.0;
}

double HeadingShimPlanner::maxOmega() const {
  return primary_ ? primary_->maxOmega() : 0.0;
}

double HeadingShimPlanner::handoffOmega() const {
  // 交棒发生在 |e| = disengage
  // 那一刻（再往下就交给主控制器了），此时减速律给的就是 √(2·α·|e|)（再被
  // omega_rot 钳）。这就是"交棒瞬间的角速度"。
  const double e = p_.disengage_deg * kDeg; // deg → rad
  return std::min(p_.omega_rot, std::sqrt(2.0 * p_.alpha_max * std::fabs(e)));
}

// ============================================================================
// 路径投影 / 取样（只用于"决定转多少"，不参与跟踪 ⇒ 不必与 MPC 逐位一致）
// ============================================================================

double HeadingShimPlanner::projectOnPlan(double x, double y,
                                         double *cross) const {
  if (cross)
    *cross = 0.0;
  if (plan_.size() < 2)
    return 0.0;

  double best_d2 = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  double best_cross = 0.0;
  double s = 0.0;
  for (std::size_t i = 1; i < plan_.size(); ++i) {
    const double ax = plan_[i - 1].x, ay = plan_[i - 1].y;
    const double bx = plan_[i].x, by = plan_[i].y;
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    const double seg = std::sqrt(len2);
    double t = 0.0;
    if (len2 > 1e-12)
      t = clampd(((x - ax) * dx + (y - ay) * dy) / len2, 0.0, 1.0);
    const double px = ax + t * dx, py = ay + t * dy;
    const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = s + t * seg;
      // 横向偏差带符号：左法向为正（与 MPC 的 `cross_track` 同号约定）
      const double inv = (seg > 1e-9) ? 1.0 / seg : 0.0;
      best_cross = -(dy * inv) * (x - px) + (dx * inv) * (y - py);
    }
    s += seg;
  }
  if (cross)
    *cross = best_cross;
  return best_s;
}

Pose2D HeadingShimPlanner::samplePlan(double s) const {
  // ★★ 取**折线切线**（方向即"路的走向"），**不取 yaw 通道的插值**。
  //   这条是实测改出来的（2026-09-28，M5.2 联调），不是随手选的：
  //   原实现"两端都有朝向就插值"，而全局规划器把**首点** yaw 保留成当前车头
  //   （`astar.keep_start_yaw`），折线又是**稀疏**的（首点到下一个拐点 3.7 m
  //   常见） ⇒ 前瞻 0.5 m 处插值出来的 yaw = 车头与"路的方向"按 0.5/3.7
  //   混出来的中间值：
  //       车头 −90°、路 −20° ⇒ 插值 −80.5° ⇒ e 只有 9.5°（< 45° 阈值）⇒
  //       **不接管**。
  //   实测后果：起步航向差 70° 时装饰器**一次都没触发**，车照旧"边转边走"，
  //   头 0.9 m 就横向甩出 23 cm（到点横向残差 5 cm+）。
  //   取切线后同一个场景第一个周期就 e=70° ⇒
  //   立刻原地转正，再走（这才是本层的目的）。 ⚠
  //   目标朝向**不用**这里：那一路直接用 `plan_.back().yaw`（末点 yaw
  //   是全局规划
  //     器明确写进去的"目标朝向"，不是插值出来的）。
  Pose2D out{};
  if (plan_.empty())
    return out;
  if (plan_.size() == 1) {
    out = plan_[0];
    return out;
  }

  double acc = 0.0;
  for (std::size_t i = 1; i < plan_.size(); ++i) {
    const double ax = plan_[i - 1].x, ay = plan_[i - 1].y;
    const double bx = plan_[i].x, by = plan_[i].y;
    const double seg = std::hypot(bx - ax, by - ay);
    if (seg < 1e-12) {
      continue;
    }
    if (acc + seg >= s || i + 1 == plan_.size()) {
      const double t = clampd((s - acc) / seg, 0.0, 1.0);
      out.x = ax + t * (bx - ax);
      out.y = ay + t * (by - ay);
      out.yaw = std::atan2(by - ay, bx - ax);
      out.has_yaw = true;
      return out;
    }
    acc += seg;
  }
  out = plan_.back();
  return out;
}

// ============================================================================
// 触发判定
// ============================================================================

HeadingShimPlanner::RotationRequest
HeadingShimPlanner::decideRotation(const Pose2D &pose,
                                   const LocalPlanResult &base) const {
  RotationRequest req;

  // ---- ① 终点朝向（优先级更高：它才是"任务的最终朝向"）----
  if (p_.goal_yaw_tolerance_deg > 0.0 && plan_.size() >= 2 &&
      plan_.back().has_yaw) {
    const double goal_yaw = plan_.back().yaw;
    const double e = wrapAngle(goal_yaw - pose.yaw);
    const double dist =
        std::hypot(pose.x - plan_.back().x, pose.y - plan_.back().y);
    const double near_dist = std::max(p_.goal_yaw_align_distance, 1e-3);
    // "到终点位置附近"的两个来源：主控制器自己说到了，或者几何上已经贴到末点。
    //   （只信前者的话，主控制器在终点附近报 kBlocked 时就没人管朝向了。）
    const bool at_goal =
        (base.status == LocalStatus::kGoalReached) || (dist <= near_dist);
    const double tol = p_.goal_yaw_tolerance_deg * kDeg;
    // 滞环：已经在按目标朝向转 ⇒ 转到 0.5·tol 才退出（避免在阈值上抖）
    const bool was_goal = rotating_ && reason_ == RotationReason::kGoalYaw;
    const double exit_tol = was_goal ? 0.5 * tol : tol;
    if (at_goal && std::fabs(e) > exit_tol) {
      req.reason = RotationReason::kGoalYaw;
      req.target_yaw = goal_yaw;
      req.e = e;
      req.tol = tol;
      return req;
    }
  }

  // ---- ② 前方朝向（起步/急弯前"先转正再走"）----
  if (p_.engage_deg > 0.0 && plan_.size() >= 2) {
    const double s0 = projectOnPlan(pose.x, pose.y, nullptr);
    const Pose2D look = samplePlan(s0 + p_.forward_sampling_distance);
    if (look.has_yaw) {
      const double e = wrapAngle(look.yaw - pose.yaw);
      const double deg = std::fabs(toDeg(e));
      const bool already = rotating_ && reason_ == RotationReason::kHeading;
      const double thr = already ? p_.disengage_deg : p_.engage_deg;
      // ★ 第二个条件：**主控制器说自己被挡了**，而车头又偏着 ⇒ 也接管。
      //   为什么需要它（否则会漏掉一整类场景）：严格走廊里纯跟踪器在航向差
      //   20~30° 就**拒动**（实测 120/120 周期被挡、一步不走，见
      //   test_mpc_local_planner.cpp 的 StrictCorridorHeadingSweep）。那个角度
      //   **低于** 45° 的进入阀值，光靠角度永远触发不了 ⇒ 车卡死。
      //   而"被挡 +
      //   车头偏"恰好是"先把机头转正再试"的充分信号，不需要再拍一个阀值。 （用
      //   disengage 当这里的门槛：比它更小的偏差不值得为"被挡"专门转一次。）
      const bool stuck_with_heading_error =
          base.status == LocalStatus::kBlocked && deg > p_.disengage_deg;
      if (deg > thr || stuck_with_heading_error) {
        req.reason = RotationReason::kHeading;
        req.target_yaw = look.yaw;
        req.e = e;
        req.tol = p_.disengage_deg * kDeg;
        return req;
      }
    }
  }
  return req;
}

// ============================================================================
// 旋转控制律：√ 减速律 + 角加速度斜坡
// ============================================================================

double HeadingShimPlanner::rotationRate(double e, double dt) const {
  if (std::fabs(e) < 1e-9)
    return 0.0;
  const double sgn = (e > 0.0) ? 1.0 : -1.0;

  // ① 减速律：越接近目标越慢（`√(2·α·|e|)` 是"以 α 减速刚好停在 e=0"的解）。
  //    在 |e| 大时它饱和到 omega_rot ⇒ 匀速段；**不会**出现"增益式控制律在 e 小
  //    时给出落进底盘死区的小指令"那个老毛病。
  const double want =
      std::min(p_.omega_rot, std::sqrt(2.0 * p_.alpha_max * std::fabs(e)));

  // ② 斜坡：|ω| 每周期最多变化 α·dt（从这里也能看出"起步就有劲"）
  const double prev_raw = p_.closed_loop ? currentW() : last_cmd_w_;
  const double prev = clampd(prev_raw, -p_.omega_rot, p_.omega_rot);
  const double dw = p_.alpha_max * std::max(dt, 1e-3);
  return clampd(sgn * want, prev - dw, prev + dw);
}

// ============================================================================
// 旋转安全：footprint 扫掠 → ESDF 车心净距 → 都不行就不转
// ============================================================================

bool HeadingShimPlanner::rotationIsSafe(const Pose2D &pose, double target_yaw,
                                        double dt, std::string &why) const {
  why.clear();
  if (!p_.check_rotation)
    return true;

  // ---- ① footprint 扫掠（有局部图时；这是本仓"车体安全"的唯一判据）----
  if (collision_.hasMap() && fp_.enable) {
    const double e = wrapAngle(target_yaw - pose.yaw);
    const double w =
        (e > 0.0 ? 1.0 : -1.0) *
        std::min(p_.omega_rot, std::sqrt(2.0 * p_.alpha_max * std::fabs(e)));
    const double step = std::max(dt, 1e-3);
    const int steps =
        std::max(1, static_cast<int>(std::ceil(
                        std::max(p_.simulate_ahead_time, step) / step)));
    for (int k = 0; k <= steps; ++k) {
      const double yaw = wrapAngle(pose.yaw + w * step * k);
      if (collision_.poseInCollision(pose.x, pose.y, yaw)) {
        char buf[256];
        if (k == 0) {
          // 当前姿态就已经重叠：**说清楚是"现在"**，别说成"转起来会碰"
          //   （两种情况要修的东西完全不同：前者是上游把我放进了障碍里，
          //     后者才是"再转就扫上去"。）
          std::snprintf(buf, sizeof(buf),
                        "当前姿态（机头 %.1f°）下车体轮廓已与障碍重叠 ⇒ "
                        "不原地转（这是上游/定位问题，不是转向问题）",
                        toDeg(yaw));
        } else {
          std::snprintf(buf, sizeof(buf),
                        "按 %.2f rad/s 转向会在 %.2f s 内扫到障碍"
                        "（机头到 %.1f° 时轮廓重叠）⇒ 不原地转",
                        w, step * k, toDeg(yaw));
        }
        why = buf;
        return false;
      }
    }
    return true;
  }

  // ---- ② 没有局部图：用 ESDF 车心净距兜底（粗糙但保守）----
  if (dist_field_ && dist_field_->valid()) {
    const double d = dist_field_->distance(pose.x, pose.y);
    if (d < p_.min_clearance) {
      char buf[256];
      std::snprintf(buf, sizeof(buf),
                    "车心到障碍 %.2f m < shim.min_clearance %.2f m，"
                    "原地转会把车体扫上去 ⇒ 不转",
                    d, p_.min_clearance);
      why = buf;
      return false;
    }
    return true;
  }

  // ---- ③ 一个判据都没有：不能证明安全 ⇒
  // 不转（宁可"没对正"，也不要"刮上去"）----
  why = "既没有局部图也没有距离场，无法判定旋转是否安全 ⇒ 不原地转";
  return false;
}

std::string HeadingShimPlanner::checkRotationTimeout(double e_rad) {
  if (p_.rotation_timeout_s <= 0.0)
    return {};
  const auto now = std::chrono::steady_clock::now();
  const double deg = std::fabs(toDeg(e_rad));
  if (!rotating_) {
    since_ = now;
    best_deg_ = deg;
    return {};
  }
  if (deg < best_deg_ - 2.0) { // 有实质进展（>2°）⇒ 重新计时
    best_deg_ = deg;
    since_ = now;
    return {};
  }
  const double elapsed = std::chrono::duration<double>(now - since_).count();
  if (elapsed <= p_.rotation_timeout_s)
    return {};
  char buf[512];
  std::snprintf(
      buf, sizeof(buf),
      "原地转向无进展超时（%.1f s 内偏差没有改善，"
      "当前还差 %.1f°）—— 查 shim.omega_rot / shim.alpha_max 是否太小、"
      "底盘低速角速度死区、定位朝向噪声；"
      "或把 shim.goal_yaw_tolerance_deg 放宽",
      elapsed, deg);
  return std::string(buf);
}

// ============================================================================
// 主入口
// ============================================================================

LocalPlanResult HeadingShimPlanner::computeCommand(const Pose2D &pose,
                                                   double dt) {
  LocalPlanResult out;
  if (!primary_) {
    out.status = LocalStatus::kFailed;
    out.message = "heading_shim 没有主控制器（检查 shim.primary / local.type）";
    return out;
  }

  // ---- ① 先问主控制器（顺序不能反，见文件头）----
  LocalPlanResult base = primary_->computeCommand(pose, dt);
  // 主控制器直接报失败（自身状态问题，转也救不了）⇒ 原样上报
  if (base.status == LocalStatus::kFailed) {
    last_cmd_w_ = base.cmd.w;
    rotating_ = false;
    reason_ = RotationReason::kNone;
    return base;
  }

  const RotationRequest req = decideRotation(pose, base);

  // ---- ② 不转：原样放行（含 kGoalReached / kBlocked）----
  if (req.reason == RotationReason::kNone) {
    rotating_ = false;
    reason_ = RotationReason::kNone;
    last_cmd_w_ = base.cmd.w;
    block_reason_.clear();
    since_ = {};
    best_deg_ = 1e9;
    return base;
  }

  target_yaw_ = req.target_yaw;
  e_yaw_ = req.e;

  // ---- ③ 安全：不转得动就如实说，不许"盲转"也不许"假装到了" ----
  std::string why;
  if (!rotationIsSafe(pose, req.target_yaw, dt, why)) {
    block_reason_ = why;
    rotating_ = false;
    reason_ = RotationReason::kNone;
    if (req.reason == RotationReason::kGoalYaw) {
      // 到点位置但转不了：**必须报出来**，不能让"没转"变成"到了"
      //   （节点侧还会再判一次朝向，报 kFollowing 的话它就是一条超时/失败）
      out.status = LocalStatus::kBlocked;
      out.message = "已到点位置，但没有空间原地对正目标朝向（还差 " +
                    std::to_string(static_cast<int>(std::fabs(toDeg(req.e)))) +
                    "°）：" + why;
      out.cmd = Twist2D{0.0, 0.0};
      out.stats = base.stats;
      last_cmd_w_ = 0.0;
      return out;
    }
    // 起步对正转不了 ⇒ 交回主控制器（它自己去"边转边走"，与旧行为一致）
    last_cmd_w_ = base.cmd.w;
    return base;
  }

  // ---- ④ 无进展超时 ----
  const std::string timeout_msg = checkRotationTimeout(req.e);
  if (!timeout_msg.empty()) {
    rotating_ = false;
    reason_ = RotationReason::kNone;
    out.status = LocalStatus::kFailed;
    out.message = timeout_msg;
    out.cmd = Twist2D{0.0, 0.0};
    out.stats = base.stats;
    last_cmd_w_ = 0.0;
    return out;
  }

  // ---- ⑤ 发旋转指令 ----
  const double w = rotationRate(req.e, dt);
  rotating_ = true;
  reason_ = req.reason;
  block_reason_.clear();

  out.status = LocalStatus::kFollowing;
  out.cmd = Twist2D{0.0, w};
  // ★ 统计量照抄主控制器的：节点的 `cross_track_m` 反馈取自这里，验收脚本拿它
  //   当横向偏差指标。装饰器自己造一套会让"旋转期间"的指标凭空消失。
  out.stats = base.stats;
  const char *what =
      (req.reason == RotationReason::kGoalYaw) ? "终点朝向" : "前方朝向";
  out.message = std::string("原地转向（") + what + "：还差 " +
                std::to_string(static_cast<int>(std::fabs(toDeg(req.e)))) +
                "°，交棒阈值 " +
                std::to_string(static_cast<int>(toDeg(req.tol))) + "°）";
  last_cmd_w_ = w;
  return out;
}

// ============================================================================
// 诊断
// ============================================================================

std::string HeadingShimPlanner::diagString() const {
  std::string s;
  if (rotating_) {
    const char *what =
        (reason_ == RotationReason::kGoalYaw) ? "终点朝向" : "前方朝向";
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "shim: 旋转中(%s) e=%+.1f° → ω=%+.3f | ω上限 %.3f α %.2f "
                  "交棒ω %.3f",
                  what, toDeg(e_yaw_), last_cmd_w_, p_.omega_rot, p_.alpha_max,
                  handoffOmega());
    s = buf;
  } else if (!block_reason_.empty()) {
    s = "shim: 未转（" + block_reason_ + "）";
  } else {
    s = "shim: 跟踪中（未触发转向）";
  }
  // ★ 主控制器的诊断**必须照常打出来**：现场排查"为什么这么慢/在扭"看的就是它，
  //   装饰器把它吞掉等于把最有用的一行日志弄没了。
  if (primary_) {
    const std::string sub = primary_->diagString();
    if (!sub.empty())
      s += " | " + sub;
  }
  return s;
}

} // namespace pnc_2d
