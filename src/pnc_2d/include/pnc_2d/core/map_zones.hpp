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

  bool empty() const { return zones_.empty(); }
  bool valid() const { return valid_; }
  const std::vector<MapZone> &zones() const { return zones_; }
  const std::vector<std::string> &warnings() const { return warnings_; }
  std::size_t forbiddenCount() const;
  std::size_t speedCount() const;

  /// 点是否落在任一禁行区内（不含膨胀；膨胀由 burnForbidden 负责）
  bool inForbidden(double x, double y) const;
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

/// 点到多边形边界的最近距离（在多边形内时为 0）
double distanceToPolygon(double x, double y,
                         const std::vector<Pose2D> &polygon);

/// 把禁行区烧进栅格副本（`out` 先拷贝 map.data()，落在膨胀后区域内的格子置
/// 100）。 返回被改写的格子数；inflate_m 建议给车体外接圆半径。
std::size_t burnForbidden(const ZoneSet &zones, const CostMap2D &map,
                          double inflate_m, std::vector<int8_t> &out);

} // namespace pnc_2d
