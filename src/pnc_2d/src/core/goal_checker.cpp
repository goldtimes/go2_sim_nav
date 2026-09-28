#include "pnc_2d/core/goal_checker.hpp"

#include <cmath>
#include <sstream>

namespace pnc_2d {

bool GoalChecker::satisfied(const GoalResidual &r) const {
  // ① **沿向**：有没有走完（扣掉停车惯性：指令提前归零，惯性带完最后几厘米）
  if (r.along - p_.stop_coast > p_.along_tolerance)
    return false;
  // ② **横向**：有没有贴到线上。与沿向**正交**，必须单独判 ——
  //    横向残差不是"没走到"，但"停在离目标 10 cm 的侧面"同样不是到点。
  if (r.lateral > p_.lateral_tolerance)
    return false;
  // ③ **朝向**：容差 <=0 表示本任务不判朝向；路径末点没带朝向（纯几何点）
  //    同样不判 —— 否则会给一个没有朝向要求的目标凭空加一条判据。
  if (p_.yaw_tolerance > 0.0 && r.has_yaw &&
      std::fabs(r.yaw_err) > p_.yaw_tolerance)
    return false;
  return true;
}

bool GoalChecker::update(const GoalResidual &r) {
  const bool ok = satisfied(r);
  just_latched_ =
      ok && !prev_ok_; // 上升沿（与 stateful 无关：都能用来"只报一次"）
  prev_ok_ = ok;
  if (ok && p_.stateful)
    latched_ = true;
  return p_.stateful ? latched_ : ok;
}

const char *GoalChecker::firstFailure(const GoalResidual &r) const {
  if (r.along - p_.stop_coast > p_.along_tolerance)
    return "along";
  if (r.lateral > p_.lateral_tolerance)
    return "lateral";
  if (p_.yaw_tolerance > 0.0 && r.has_yaw &&
      std::fabs(r.yaw_err) > p_.yaw_tolerance)
    return "yaw";
  return "";
}

std::string GoalChecker::describe(const GoalResidual &r) const {
  // 全部单位都写出来（m / °）：这类日志历史上因为"没说清是哪一个量"查了好几轮
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  std::ostringstream s;
  s.setf(std::ios::fixed);
  s.precision(4);
  s << "沿向残差 " << r.along << " m（容差 " << p_.along_tolerance
    << "，含停车惯性 " << p_.stop_coast << "）";
  s << " / 横向残差 " << r.lateral << " m（容差 " << p_.lateral_tolerance
    << "）";
  if (p_.yaw_tolerance > 0.0 && r.has_yaw) {
    s.setf(std::ios::fixed);
    s.precision(1);
    s << " / 朝向误差 " << std::fabs(r.yaw_err) * kRadToDeg << "°（容差 "
      << p_.yaw_tolerance * kRadToDeg << "°）";
  } else {
    s << " / 朝向：不判（"
      << (p_.yaw_tolerance <= 0.0 ? "容差为 0" : "路径末点没带朝向") << "）";
  }
  const char *bad = firstFailure(r);
  if (!bad[0]) {
    s << " ⇒ **到点**";
    if (latched_)
      s << "（锁存中）";
  } else if (std::string(bad) == "along") {
    s << " ⇒ **沿向超**（没走到）";
  } else if (std::string(bad) == "lateral") {
    s << " ⇒ **横向超**（没贴线）";
  } else {
    s << " ⇒ **朝向超**（没对正）";
  }
  return s.str();
}

} // namespace pnc_2d
