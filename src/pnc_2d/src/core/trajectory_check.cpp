// M3：轨迹终检的实现（口径与理由见头文件 trajectory_optimizer.hpp）。
//
// 实现要点（**两遍，判据不同、代价差一个数量级**）：
//   ① **硬门**：按 (ds, dyaw) 细分，默认用**便宜的布尔判定** `poseInCollisionNoMargin`
//      （走 8 方向预计算表，~3 µs）；`hard_margin > 0` 时才退回 `signedClearanceAt`。
//   ② **统计**（最小净距）：按更粗的 (stats_ds, stats_dyaw) 走 `signedClearanceAt`。
//   ★ 为什么要拆：`signedClearanceAt` 内部是 **14 步二分、每步重建一次轮廓偏移表**
//     （几十格 SAT）≈ 20~40 µs，比布尔判定贵一个数量级。用户现场（40 m 路径、
//     1891 个输出点）一遍 2 cm 净距二分要 **122.7 ms**（超 50 ms 预算），
//     实测绠大部分耗在这里；而**硬门只需要"碰没碰"**，根本不需要连续净距。
//   ★ 统计可以粗到 10 cm：硬门已经保证不碰，净距只是给日志/调参看的一个**数**，
//     10 cm 分辨率上的误差远小于轮廓判定自身的贴格心噪声（±5 cm，见 R13）。
//   ★ 子采样必须**同时**按弧长与**朝向**（原地转靠后者兜底，否则整段漏掉旋转扫掠）。

#include "pnc_2d/core/trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>

#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {
namespace {

/// 沿样本序列做"**同时**按弧长与朝向细分"的遍历：相邻两样本之间取
/// `max(⌈Δs/ds⌉, ⌈|Δyaw|/dyaw⌉)` 个子步（原地转时 dist≈0 而 |Δyaw| 很大，
/// 只按弧长细分会把整段旋转扫掠**漏掉** —— 这类漏检正是 M3 要堵的洞）。
void walkDense(const std::vector<TrajSample> & samples, double ds, double dyaw,
               const std::function<void(double, double, double, double)> & fn)
{
  const double s_step = std::max(1e-3, ds);
  const double y_step = std::max(1e-3, dyaw);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const TrajSample & a = samples[i];
    if (i == 0) {
      fn(a.s, a.x, a.y, a.yaw);
      continue;
    }
    const TrajSample & p = samples[i - 1];
    const double dx = a.x - p.x;
    const double dy = a.y - p.y;
    const double dth = wrapAngle(a.yaw - p.yaw);
    const double dist = std::hypot(dx, dy);
    const int n_s = static_cast<int>(std::ceil(dist / s_step));
    const int n_y = static_cast<int>(std::ceil(std::fabs(dth) / y_step));
    const int steps = std::max(1, std::max(n_s, n_y));
    for (int j = 1; j <= steps; ++j) {
      const double t = static_cast<double>(j) / static_cast<double>(steps);
      fn(p.s + (a.s - p.s) * t, p.x + dx * t, p.y + dy * t, p.yaw + dth * t);
    }
  }
}

}  // namespace

TrajCheckResult checkTrajectory(const std::vector<TrajSample> & samples,
                                const FootprintCollisionChecker & ck,
                                const TrajCheckParams & prm)
{
  TrajCheckResult r;
  r.ok = true;

  // 没有判据时不要假装查过了（与 TrajOptStats::clearance_valid 同一约定）
  if (!ck.hasMap() || !ck.enabled() || samples.size() < 2) {
    r.note = "未做终检（无地图/轮廓判定未启用/样本不足 2 点）";
    r.min_valid = false;
    return r;
  }

  // ---------------- ① 硬门 ----------------
  // 默认走便宜的布尔判定；`hard_margin > 0` 才需要连续净距（显式配置的慢路径）
  const bool gate_needs_clearance = prm.hard_margin > 0.0;
  walkDense(samples, prm.ds, prm.dyaw, [&](double s, double x, double y, double yaw) {
    ++r.points;
    bool bad = false;
    if (gate_needs_clearance) {
      bad = ck.signedClearanceAt(x, y, yaw, prm.max_search) < prm.hard_margin;
    } else {
      bad = ck.poseInCollisionNoMargin(x, y, yaw);
    }
    if (!bad) return;
    ++r.violations;
    r.ok = false;
    if (r.violations == 1) {
      r.first_bad_s = s;
      r.first_bad_x = x;
      r.first_bad_y = y;
      // 失败点的实测净距（只算一次，用于"可操作"的报告；布尔判定本身不给数）
      r.first_bad_clearance = ck.signedClearanceAt(x, y, yaw, prm.max_search);
    }
  });

  // ---------------- ② 统计（最小净距） ----------------
  // 更粗的网格：硬门已保证不碰，这里只要一个"数"。
  //   `worst_clearance`：**含首端**全部子样本的最小值（首端是原地转扫掠发生的地方，
  //                       必须一起算 —— 但用不着 2 cm 那么密）；
  //   `min_clearance`  ：排除首端 `stats_exclude_s`（首点 = 车自己，会掩盖差异）
  double worst = std::numeric_limits<double>::infinity();
  double min_c = std::numeric_limits<double>::infinity();
  walkDense(samples, prm.stats_ds, prm.stats_dyaw,
            [&](double s, double x, double y, double yaw) {
              ++r.stats_points;
              const double c = ck.signedClearanceAt(x, y, yaw, prm.max_search);
              if (c < worst) {
                worst = c;
                r.worst_clearance_s = s;
              }
              if (s >= prm.stats_exclude_s && c < min_c) {
                min_c = c;
                r.min_clearance = c;
                r.min_clearance_s = s;
                r.min_clearance_x = x;
                r.min_clearance_y = y;
                r.min_valid = true;
              }
            });
  r.worst_clearance = std::isfinite(worst) ? worst : 0.0;

  char buf[384];
  if (r.ok) {
    std::snprintf(buf, sizeof(buf),
                  "终检通过：硬门 %d 个位姿（%s），全段最小轮廓净距 %.4f m @s=%.2f m%s",
                  r.points, gate_needs_clearance ? "净距判定" : "布尔轮廓判定",
                  r.worst_clearance, r.worst_clearance_s,
                  (r.worst_clearance < prm.warn_margin) ? "（⚠ 低于余量，只是提示）" : "");
  } else {
    std::snprintf(buf, sizeof(buf),
                  "终检不过：第 1 个失败点在 (%.3f, %.3f) / s=%.2f m，轮廓净距 %.4f m "
                  "（硬要求 ≥ %.3f m）；共 %d/%d 个位姿不过。修法优先级：① 平滑没收敛"
                  "（看\"净距目标/达成\"）② 轨迹偏离平滑路径 ③ 地图/定位与实际不符",
                  r.first_bad_x, r.first_bad_y, r.first_bad_s, r.first_bad_clearance,
                  prm.hard_margin, r.violations, r.points);
  }
  r.note = buf;
  return r;
}

std::vector<TrajSample> trajSamplesFromPath(const std::vector<Pose2D> & path, double ds)
{
  std::vector<TrajSample> out;
  if (path.size() < 2) return out;
  const double step = std::max(1e-3, ds);
  double s_acc = 0.0;
  auto push = [&](double s, double x, double y, double yaw) {
    TrajSample t;
    t.t = 0.0;   // 纯几何路径没有时间轴：终检只看位姿
    t.s = s;
    t.x = x;
    t.y = y;
    t.yaw = yaw;
    out.push_back(t);
  };
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Pose2D & a = path[i];
    const Pose2D & b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double seg = std::hypot(dx, dy);
    if (seg < 1e-9) continue;
    // 朝向：折线自己没给就取线段方向（"车头沿路径"），给了就用折线给的
    const double yaw_a = a.has_yaw ? a.yaw : std::atan2(dy, dx);
    const double yaw_b = b.has_yaw ? b.yaw : std::atan2(dy, dx);
    const int steps = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int j = 0; j < steps; ++j) {
      const double t = static_cast<double>(j) / static_cast<double>(steps);
      push(s_acc + seg * t, a.x + dx * t, a.y + dy * t, yaw_a + wrapAngle(yaw_b - yaw_a) * t);
    }
    s_acc += seg;
  }
  const Pose2D & last = path.back();
  push(s_acc, last.x, last.y,
       last.has_yaw ? last.yaw
                    : (out.empty() ? 0.0 : out.back().yaw));   // 末点朝向没给就沿用上一个
  return out;
}

}  // namespace pnc_2d
