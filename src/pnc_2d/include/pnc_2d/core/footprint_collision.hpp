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

  /// 是否已绑定**有效**地图。用途：终检等地方要判断"到底有没有判据"——
  /// 不要用别的返回值当哨兵（`poseInCollision*` 在无图时返回 true = 碰，
  /// 那是"保守"语义，不是"没查"）。
  bool hasMap() const;

  /// 单格是否致命（含未知策略；图外一律视为致命）
  bool cellLethal(int x, int y) const;
  bool pointLethal(double wx, double wy) const;

  /// 位姿处车体是否与致命格相交（enable=false 时退化为"中心格是否致命"）
  ///
  /// ★★ 已知局限（实测取证，2026-09）：判定把位姿**吸附到格心**再算 ——
  ///   `rectOffsets` 只用机体系 offset 建矩形（相对"当前格中心"），**丢掉了
  ///   (wx, wy) 在格内的亚格偏移**。于是含余量判定的实际误差可达 **±半格**
  ///   （res = 0.10 ⇒ ±5 cm），实测同一格内 y 从 2.61 扫到 2.70，净距恒为
  ///   0.0499（= 格心 2.65 处的值），而不是连续变化的 0.01→0.10。
  ///   后果：**`safe_margin` 必须大于一格才有意义** —— 取 0.05（= 半格）时
  ///   "余量带"在格心模型下是空集，"起点在余量带里"这类场景根本构造不出来。
  ///   （真要修需要给 `rectOffsets` 加亚格偏移，而 8 方向的预计算表用不上
  ///   逐位姿偏移 ⇒ 要么放弃表、要么改判据。属于跨模块改动，尚未做。）
  bool poseInCollision(double wx, double wy, double yaw) const;

  /// 参考点到**最近致命格**的距离 [m]（需要距离场；拿不到时返回 NaN）。
  /// 用途：把"轮廓与障碍重叠"拆成"真的压上去了"和"只擦到安全余量"——
  /// 后者（比如全局图与感知图差 1~2 cm、运行中新加了一块禁行区）不应该把
  /// 任务判死，但需要能**量出来**才能这么判（见 A* 的起点松弛）。
  double distanceToLethal(double wx, double wy) const;

  /// 把 safe_margin 当成 0 再判一次（= "真实轮廓压上障碍了吗"）。
  /// 与 poseInCollision 的关系：后者含余量（保守），这个只回答"是不是真撞"。
  /// 因为方向预计算表是按当前 footprint 建的，这里走完整检查（每次几十格，
  /// 只在"起点/终点疑似碰撞"这种低频路径上用）。
  bool poseInCollisionNoMargin(double wx, double wy, double yaw) const;

  /// 用**指定的** safe_margin 判一次（不改内部状态；margin 可为负 = 把轮廓缩小）。
  /// 用途：起点已经压进障碍时，要知道"压进去多深":
  ///   margin = 0     → 真实轮廓撞了吗
  ///   margin = -0.05 → 真实轮廓再往里让 5 cm 还撞吗（= 穿透是否 ≤ 5 cm）
  bool poseInCollisionAtMargin(double wx, double wy, double yaw,
                               double margin) const;

  /// 轮廓净距（**带符号**，单位 m）：轨迹终检用（要"轮廓到障碍的连续净距"）。
  ///   > 0 = 真实轮廓（safe_margin = 0）还能均匀外扩这么多米才碰到致命格；
  ///   < 0 = 需要把轮廓均匀内缩这么多米才不碰（= 穿透深度）；
  ///   0   = 刚好贴着。
  ///
  /// ★ 不变式（与 poseInCollisionAtMargin **同一判据**，m 可为负）：
  ///      signedClearanceAt(..., m) >= m   ⇔   !poseInCollisionAtMargin(..., m)
  ///   否则两套判据会在"恰好擦到"时打架（我们已经在跳层物理量上栽过多次）。
  ///
  /// 实现：在 [−max_search, +max_search] 上二分（free(m) 对 m 单调）。
  ///   开阔处（最近致命格足够远）走快路径直接饱和 ⇒ 一次查表；
  ///   返回值饱和于 ±max_search（即"≥ max_search" / "穿透 ≥ max_search"）。
  ///   分辨率 ≈ 2·max_search / 2^14 ≈ 0.12 mm（max_search = 1.0）。
  /// ⚠ fp.enable = false 时 margin 不参与判定，二分无意义 ⇒ 退化为点判定。
  double signedClearanceAt(double wx, double wy, double yaw,
                           double max_search = 1.0) const;

  /// 临时改用另一个 safe_margin（**可以是负数** = 把轮廓缩小）。
  /// 用途：起点已经擦进/压进障碍时，用更松的轮廓再试一次规划
  /// （见 AStarPlanner::start_penetration_tol）—— 否则"起点在带里"就是不可恢复
  /// 的死局：起点节点自身过不了含余量的检查，朝任何方向的下一步也过不了。
  /// 复位用 clearMarginOverride()（**不要用负值当哨兵**：负 margin 是合法取值）。
  void setMarginOverride(double m);
  void clearMarginOverride();
  /// 当前生效的轮廓（可能带 override；诊断用）
  const FootprintParams & effectiveFootprint() const { return fp_eff_; }

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
  /// fp 为 nullptr 时用当前配置 fp_（有个按方向预计算的快路径表）。
  void rectOffsets(double yaw, std::vector<Cell> & out,
                   const FootprintParams * fp = nullptr) const;
  void buildDirectionOffsets();
  /// 命中 8 个离散方向则返回 0..7，否则 -1
  int directionIndex(double yaw) const;
  /// 完整矩形检查（无快路径）；fp 非空时用它（不查预计算表）
  bool fullCheckAtCell(int cx, int cy, double yaw,
                       const FootprintParams * fp = nullptr) const;
  void cornerWorld(double x, double y, double yaw, int i, double & ox, double & oy) const;

  const CostMap2D * map_{nullptr};
  const ClearanceField * cf_{nullptr};
  FootprintParams fp_;        // 配置值（诊断/画标记用它）
  /// 当前生效的轮廓：= fp_，或 setMarginOverride 之后的那份。
  /// **所有几何判定都走它**；fp_ 永远保留配置值。
  FootprintParams fp_eff_{};
  /// ★ 注：不能用“margin_override_ < 0”当“没 override”的哨兵 —— 负 margin
  ///   现在是一个**合法取值**（“真实轮廓再往里让”）。两者必须分开存，
  ///   否则 override 生效时还会去查按原 margin 预计算的方向表，判据就错了。
  bool margin_overridden_{false};
  double margin_override_{-1.0};
  int hard_threshold_{80};
  bool unknown_as_occupied_{true};
  std::array<std::vector<Cell>, 8> dir_offsets_;
  mutable long full_checks_{0};   // 诊断计数（const 方法里累加）
};

}  // namespace pnc_2d
