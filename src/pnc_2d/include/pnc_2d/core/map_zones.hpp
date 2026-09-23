// 地图区域层（禁行区 / 限速区）—— 与路网同文件（`routes.yaml` 的 `zones:`
// 段）。
//
// 语义（2026-09-21 与用户确认）：
//   · 禁行区（forbidden）：**全栈约束**。不是"只给全局规划器"——全局规划不能穿、
//     局部规划也不能进，所以统一由 map_server **烧进发布的全局图**：
//     任何订阅这张图的模块（全局规划器 / 局部规划 / RViz / 将来
//     nav2）自动遵守， 不需要每个算法各自解析（那样一定会漏）。
//   · 限速区（speed_limit）：软约束，不进栅格；由速度规划 / 局部规划（P5）按
//     speedLimitAt() 查询消费，本层只负责解析与可视化。
//
// 膨胀：车体是矩形，要保证"任何朝向都不侵入" → 禁行区按**车体外接圆半径**膨胀
// （0.70x0.40 + margin 0.05 → r = hypot(0.40, 0.25) = 0.472 m），保守但正确。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

enum class ZoneType {
  kForbidden,  ///< 禁行
  kSpeedLimit, ///< 限速
};

std::string toString(ZoneType t);
ZoneType zoneTypeFromString(const std::string &s, bool *ok = nullptr);

struct MapZone {
  std::string name;
  ZoneType type{ZoneType::kForbidden};
  double value{0.0};           ///< type=speed_limit 时是上限 [m/s]
  std::vector<Pose2D> polygon; ///< 顶点（首尾自动闭合，不需要重复首点）
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0}; ///< 包围盒，用于快速排除
};

class ZoneSet {
public:
  bool loadFromFile(const std::string &path, std::string &err);
  bool loadFromString(const std::string &yaml, std::string &err);

  /// 直接用已解析的区域填充（**消息 → 内部表示**：map_server 的 ZoneArray →
  /// 局部/其它消费者）。
  ///
  /// 为什么要这个：`loadFrom*` 只能从 yaml 文本构造，而运行时收到的区域是消息；
  /// 若两边各写一套校验，就一定会分叉（顶点数、包围盒、限速值合法性）。
  /// 这里复用同一套校验：顶点 <3 / 类型非法 / 数值非法的区域被丢弃并记进
  /// warnings。
  /// @return 是否至少保留了一个合法区域（全被丢弃时返回 false，且 ZoneSet
  /// 为空）
  bool adopt(std::vector<MapZone> zones);

  bool empty() const { return zones_.empty(); }
  bool valid() const { return valid_; }
  const std::vector<MapZone> &zones() const { return zones_; }
  const std::vector<std::string> &warnings() const { return warnings_; }
  std::size_t forbiddenCount() const;
  std::size_t speedCount() const;

  /// 点是否落在任一禁行区内（不含膨胀；膨胀由 burnForbidden 负责）
  bool inForbidden(double x, double y) const;
  /// 点落在哪个禁行区里（不在任何区内返回空串）—— **报错点名用**：
  /// "车已在禁行区内（Z1）" 比 "求解失败" 可操作得多。
  std::string forbiddenNameAt(double x, double y) const;
  /// 点是否落在某个限速区内；是则返回其中最严的上限 [m/s]
  bool inSpeedZone(double x, double y, double &limit) const;

  std::string summary() const;

private:
  std::vector<MapZone> zones_;
  std::vector<std::string> warnings_;
  bool valid_{false};
};

/// 射线法多边形测试（polygon 为顶点序列，首尾自动闭合）
bool pointInPolygon(double x, double y, const std::vector<Pose2D> &polygon);

/// 点到多边形**边界**的最近距离（**不区分内外**：在多边形内部它返回的是"到边界的
/// 距离"，不是 0）。
///
/// ⚠ 所以"点在不在区内"必须另外用 `pointInPolygon` 判：想表达"区内 = 0、区外 =
///   到边界的距离"，要写 `pointInPolygon(x,y,p) ? 0.0 : distanceToPolygon(...)`
///   （`burnForbidden` 用的是 `pointInPolygon || distance <=
///   inflate`，同一个道理）。
///   踩过（2026-09-23）：这条注释原来写成"在多边形内时为 0"，照着写就把**区域
///   内部**当成了"离障碍还有好几米" ⇒ 车会以为大区域内部很安全。单测当场抓到：
///   1×1 m 禁行区中心算出 0.45 m。
double distanceToPolygon(double x, double y,
                         const std::vector<Pose2D> &polygon);

/// 把禁行区烧进栅格副本（`out` 先拷贝 map.data()，落在膨胀后区域内的格子置
/// 100）。 返回被改写的格子数；inflate_m 建议给车体外接圆半径。
std::size_t burnForbidden(const ZoneSet &zones, const CostMap2D &map,
                          double inflate_m, std::vector<int8_t> &out);

/// 沿折线**前瞻**取限速区最严限速 [m/s]（0 = 不限）。
///
/// @param pts       折线（世界系）
/// @param from      从第几个点开始（通常是当前进度下标）
/// @param lookahead 前瞻弧长 [m]（`<= 0` = 只看起点那一点；用一个很大的值 =
/// 整条路径）
///
/// 语义：只统计“从 pts[from] 起、累计弧长 ≤ lookahead”的点，多区重叠取最严。
/// 为什么要共用一份：局部拿它算“进区前就要降到的速度帽”（前瞻 = 刹车距离），
/// 全局拿它算“这条路径上的任务限速建议”（前瞻 = 整条路径）。两边各写一份，
/// 一定会分叉（比如一边取最小、一边取最大，或者一边含末点一边不含）。
/// 沿路径**前瞻**取最严限速值（0 = 前方没有限速区）。
///
/// @param pts      路径点（不必稠密）
/// @param from     从此下标开始（通常是自车进度下标）
/// @param lookahead 前瞻弧长 [m]；**<= 0 表示只看起点那一点**（不是"不限制"），
///                  想表达"整条路径"请显式传一个很大的值
/// @param step     **重采样步长** [m]（默认 0.25）。
///
/// ★ 为什么必须重采样：局部节点拿到的路径可能是**只有两个点**的直线
///   （全局自由空间规划的输出就是 2 点）。不重采样的话，只看路径点等于只看
///   起点与终点 —— 中间的限速区会被整个跳过。实测（2026-09-23 test_zones 段
///   3）： 限速区就在起点前 0.7 m、路径 2 点，前瞻 0.89 m 依然返回 0，车以 0.3
///   m/s 穿区。 步长取得比典型栅格分辨率（0.1~0.2 m）小，保证不会跳过小区域。
double zoneSpeedLimitAhead(const ZoneSet &zones, const std::vector<Pose2D> &pts,
                           std::size_t from, double lookahead,
                           double step = 0.25);

/// 同上，但前瞻**从车在路径上的投影点**开始（不是从某个路点下标开始）。
///
/// ★ 控制周期里必须用这个，不能用上面那个 + `pass_index_`：
///   自由空间规划给出的路径常常**只有两个点**（起点、终点），而"路点下标"这种
///   进度只能在车靠近某个路点时前进（节点里还有 `pass_distance_` 容差）⇒
///   整段路 prepass_index_ 都是 0，扫描窗口变成"**路径起点往后 look 米**"，
///   与车实际走到哪儿无关。实测（2026-09-23 test_zones 段 3）：车已经开出限速区
///   1 m 了速度帽还在（全程 0.15 m/s 爬 31 s，"出区后恢复"永远不成立）。
///   投影点给出的是**连续弧长**，与路点密度无关。
double zoneSpeedLimitAheadFromProjection(const ZoneSet &zones,
                                         const std::vector<Pose2D> &pts,
                                         double x, double y, double lookahead,
                                         double step = 0.25);

/// 限速区**前瞻距离** [m]：从速度 v 按减速度 a 减速，再加 margin 余量。
///
/// ★ 必须传"**可能达到的**速度"（v_max / 任务限速），**不能**传当前速度：
///   前瞻 = v²/(2a) + margin，用当前速度算的话，车越慢前瞻越短、越晚才减速，
///   形成自锁 —— "因为开得慢，所以从不减速"。
///   实测（2026-09-23 test_zones 段 3）：车以 0.26 m/s 爬向 0.7 m 外的 0.15 m/s
///   限速区，前瞻 = 0.26²/(2·0.15)+0.3 = 0.53 m < 0.7 m ⇒ 区域帽从未生效，
///   最后以 0.30 m/s 穿区（验收项 Z4/Z5 双双失败，而功能"看起来是接好的"）。
///   用 v_max=0.42 算：0.42²/(2·0.15)+0.3 = 0.89 m ⇒ 距离 0.7 m 时就触发。
///
/// 返回值保证 ≥ 从 v 减到 *任何* 更低速度所需距离（(v²−v_lim²)/(2a) <
/// v²/(2a)）， 所以只要在触发时才开始减速，也不会超速进区。
inline double speedLookahead(double v, double a, double margin = 0.3) {
  if (v <= 0.0 || a <= 0.0)
    return margin > 0.0 ? margin : 0.0;
  return v * v / (2.0 * a) + (margin > 0.0 ? margin : 0.0);
}

} // namespace pnc_2d
