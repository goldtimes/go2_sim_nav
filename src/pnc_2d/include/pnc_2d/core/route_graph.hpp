// 路网（route network）：让机器沿"预先画好的通道"走。
//
// 数据模型（ROS-free，见 doc/pnc2d_restructure_plan.md §6 P2）：
//   节点 RouteNode：名字 + 坐标 + 语义类型（waypoint / station / charge /
//   park） 边   RouteEdge：起止节点 + 一条带几何的折线 + 属性
//       · one_way         单向（只能 from → to）
//       · speed_limit     速度上限 [m/s]，参与路由代价（代价 = 弧长 /
//       speed_limit） · corridor_width  允许横向偏离的半宽 [m]；**0.0 =
//       严格贴线，遇障即停** · polyline        折线（首点=from 坐标，末点=to
//       坐标；中间点用来画弧线/绕障）
//
// 文件位置约定：~/rcs/maps/<站点>/routes.yaml（与 map.yaml 同目录，换图同源）
//
// ⚠ yaml-cpp 的坑（沿用 map_server 的教训）：`as<bool>()` 遇到 `0`/`1`
// 这类整数会直接
//   抛 "bad conversion" → 所有 bool 字段必须走宽松解析 parseLooseBool()。

#pragma once

#include <string>
#include <vector>

#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

/// 节点语义：waypoint = 普通拓扑点；其余为"命名站点"（RViz 里用不同颜色 +
/// 名字标签）
enum class RouteNodeType {
  kWaypoint,
  kStation,
  kCharge,
  kPark,
};

std::string toString(RouteNodeType t);
RouteNodeType routeNodeTypeFromString(const std::string &s, bool *ok = nullptr);

struct RouteNode {
  std::string name;
  double x{0.0};
  double y{0.0};
  RouteNodeType type{RouteNodeType::kWaypoint};
};

struct RouteEdge {
  int from{-1};                 ///< 节点索引
  int to{-1};                   ///< 节点索引
  bool one_way{false};          ///< true = 只能 from → to
  double speed_limit{1.0};      ///< [m/s]，>0
  double corridor_width{0.6};   ///< 允许偏离半宽 [m]；0.0 = 严格贴线
  std::vector<Pose2D> polyline; ///< 首=from、末=to（yaw 字段此处不用）
  double length{0.0};           ///< 弧长 [m]（载入时算好）
  bool feasible{true}; ///< 车体沿该通道是否真的通得过（载入地图后校验）

  /// 折线上弧长 s 处的点（s 会被夹到 [0, length]）
  Pose2D pointAt(double s) const;
  /// 折线在弧长 s 处的切向 yaw
  double yawAt(double s) const;
};

/// 投影结果：世界点落在哪条边、弧长参数、垂足、垂距
struct RouteProjection {
  bool valid{false};
  int edge{-1};
  double s{0.0}; ///< 从该边起点算起的弧长
  double x{0.0};
  double y{0.0};
  double dist{0.0}; ///< 点到垂足的距离（通道外时 >0）
};

class RouteGraph {
public:
  // ---------------- 读写 ----------------
  bool loadFromFile(const std::string &path, std::string &err);
  /// 直接给 yaml 文本（单测用，避免依赖临时文件）
  bool loadFromString(const std::string &yaml, std::string &err);
  bool saveToFile(const std::string &path, std::string &err) const;

  bool valid() const { return valid_; }
  const std::string &frameId() const { return frame_id_; }
  const std::vector<RouteNode> &nodes() const { return nodes_; }
  const std::vector<RouteEdge> &edges() const { return edges_; }
  std::vector<RouteEdge> &mutableEdges() { return edges_; }
  /// 载入/校验过程中的非致命问题（端点自动吸附、速度非法已修正…）
  const std::vector<std::string> &warnings() const { return warnings_; }

  /// 按名字找节点，找不到返回 -1
  int nodeIndex(const std::string &name) const;

  /// 把世界点投影到最近的边（不考虑单行方向，路由阶段再看方向）
  RouteProjection project(double x, double y) const;

  /// 统计信息（日志用）
  std::string summary() const;

private:
  bool parse(const std::string &yaml_text, std::string &err);
  void computeLengths();
  bool validate(std::string &err) const;

  std::string frame_id_{"map"};
  std::vector<RouteNode> nodes_;
  std::vector<RouteEdge> edges_;
  std::vector<std::string> warnings_;
  bool valid_{false};
};

/// 宽松解析 bool：接受 true/false、yes/no、on/off、0/1（含 "0"/"1" 字符串）
bool parseLooseBool(const std::string &text, bool &out);

} // namespace pnc_2d
