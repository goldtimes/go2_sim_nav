// 矩形车体轮廓（footprint）碰撞检查 —— 所有规划算法共用的**唯一实现**。
//
// 语义（关键，与 nav2 的 FootprintCollisionChecker 一致）：
//   · 只查**致命格**（raw ≥ hard_threshold，或未知格当障碍时的未知格）；
//   · **不看膨胀梯度**：安全性由 footprint 保证，舒适性由软代价保证。
// 于是"用不用膨胀层"与"车体是否安全"解耦：本包的全局图没有膨胀层也能保证不压障碍。
//
// 朝向换一句话说：A* 的 (x,y) 栅格里节点本身没有 yaw，所以调用方约定
//   · 扩展节点 → 用**运动方向**（父 → 当前）作为朝向；
//   · 终点节点 → 用目标 yaw（请求里给了就用）。
// 8 邻域只有 8 个离散方向，因此**按方向预计算"覆盖格偏移集合"**，之后每次检查
// 只是查表 + 数组访问，没有三角函数（nav2 的 oriented_footprints_ 是同一思路，
// 我们进一步预计算到格子偏移这一层）。
//
// 快路径（比 nav2 更彻底）：利用精确距离场做两步快速判定
//   d - e - |offset| ≥ R_circ → 任何朝向都安全（开阔区域，1 次查表）
//   d + e + |offset| <  R_in  → 任何朝向都碰撞（贴障碍，1 次查表）
// 只有落在 [R_in, R_circ] 环带（只出现在障碍附近）才做完整矩形检查。

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

class CostMap2D;

/// 矩形车体轮廓参数（机体系：x = 前后/长，y = 左右/宽；与 perception 命名一致）
struct FootprintParams {
  bool enable{true};
  double length{0.70};
  double width{0.40};
  double offset_x{0.0};       // 矩形中心相对参考点的机体系偏移
  double offset_y{0.0};
  double safe_margin{0.05};   // 四边各外扩
  bool check_edges{true};     // 边检查是否做"四角轨迹 + 中心线"扫掠
  bool fast_path{true};       // 是否启用距离场快路径

  double halfLength() const { return 0.5 * length + safe_margin; }
  double halfWidth() const { return 0.5 * width + safe_margin; }
  /// 内切半径：内切圆里出现障碍 ⇒ 任何朝向都碰撞（快路径的"必撞"阈值）
  double inscribedRadius() const { return std::fmin(halfLength(), halfWidth()); }
  /// 外接半径：整机都在该圆内 ⇒ 圆内无障碍 ⇒ 任何朝向都安全（快路径的"必安全"阈值）
  double circumscribedRadius() const { return std::hypot(halfLength(), halfWidth()); }
  /// 参考点到矩形最远点的距离（含 offset），快路径判定要用
  double reach() const { return circumscribedRadius() + std::hypot(offset_x, offset_y); }
};

class FootprintCollisionChecker {
public:
  void configure(const FootprintParams & fp, int hard_threshold, bool unknown_as_occupied);
  /// 绑定地图并重建方向偏移表（距离场由外部共享传入，避免每个算法各建一份）
  void setMap(const CostMap2D * map);
  void setClearanceField(const ClearanceField * cf) { cf_ = cf; }

  const FootprintParams & footprint() const { return fp_; }
  bool enabled() const { return fp_.enable; }

  /// 单格是否致命（含未知策略；图外一律视为致命）
  bool cellLethal(int x, int y) const;
  bool pointLethal(double wx, double wy) const;

  /// 位姿处车体是否与致命格相交（enable=false 时退化为"中心格是否致命"）
  bool poseInCollision(double wx, double wy, double yaw) const;

  /// 边扫掠检查：机器人沿直线从 (x0,y0) 走到 (x1,y1)，**车身朝向 = 线段方向**
  /// （到点后再原地转向下一段；原地旋转属于平台/局部规划器的职责，不在本检查内）。
  /// 内容：两端位姿 + 中心线 + 四角轨迹。
  bool edgeInCollision(double x0, double y0, double x1, double y1) const;

  /// 线段是否穿过致命格（Amanatides & Woo 体素遍历：逐格检查，不靠采样，不会穿墙）
  bool lineHitsLethal(double x0, double y0, double x1, double y1) const;

  /// 车体四角的世界坐标（顺序 ++, -+, --, +- ，可直接当 LINE_STRIP 用）
  void footprintCorners(double x, double y, double yaw, double out[8]) const;

  /// 诊断：第 dir 个离散方向的覆盖格数（0..7 对应 0°,45°,…,315°）
  std::size_t directionOffsetCount(std::size_t dir) const
  {
    return dir < 8 ? dir_offsets_[dir].size() : 0;
  }
  /// 诊断：某个朝向下覆盖格数（0 表示非 8 方向之一时不便宜预计算）
  std::size_t offsetsAtYaw(double yaw) const;

  /// 诊断：走了**完整矩形检查**的次数（对比快路径命中率，验证快路径有效）
  long fullChecks() const { return full_checks_; }
  void resetCounters() const { full_checks_ = 0; }

private:
  struct Cell {
    int dx;
    int dy;
  };

  /// 求某朝向下车体覆盖的格子偏移集合（相对中心格；用 SAT 判"格与矩形相交"）
  void rectOffsets(double yaw, std::vector<Cell> & out) const;
  void buildDirectionOffsets();
  /// 命中 8 个离散方向则返回 0..7，否则 -1
  int directionIndex(double yaw) const;
  /// 完整矩形检查（无快路径）
  bool fullCheckAtCell(int cx, int cy, double yaw) const;
  void cornerWorld(double x, double y, double yaw, int i, double & ox, double & oy) const;

  const CostMap2D * map_{nullptr};
  const ClearanceField * cf_{nullptr};
  FootprintParams fp_;
  int hard_threshold_{80};
  bool unknown_as_occupied_{true};
  std::array<std::vector<Cell>, 8> dir_offsets_;
  mutable long full_checks_{0};   // 诊断计数（const 方法里累加）
};

}  // namespace pnc_2d
