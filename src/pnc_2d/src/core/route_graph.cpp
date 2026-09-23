#include "pnc_2d/core/route_graph.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

namespace pnc_2d {
namespace {

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// 从 yaml 标量取字符串（整数/浮点也接受，转成文本）
bool scalarText(const YAML::Node &n, std::string &out) {
  if (!n || !n.IsScalar())
    return false;
  out = n.Scalar();
  return true;
}

/// 宽松取 bool：走文本解析，避免 as<bool>() 对 0/1 抛异常
bool looseBool(const YAML::Node &n, bool def, bool *got = nullptr) {
  std::string t;
  if (!scalarText(n, t)) {
    if (got)
      *got = false;
    return def;
  }
  bool v = def;
  if (!parseLooseBool(t, v)) {
    if (got)
      *got = false;
    return def;
  }
  if (got)
    *got = true;
  return v;
}

bool looseDouble(const YAML::Node &n, double *out) {
  std::string t;
  if (!scalarText(n, t))
    return false;
  try {
    *out = std::stod(trim(t));
    return std::isfinite(*out);
  } catch (...) {
    return false;
  }
}

/// 折线元素：支持 [x, y] 与 {x: , y: } 两种写法
bool parsePolylinePoint(const YAML::Node &n, Pose2D &out) {
  if (!n)
    return false;
  if (n.IsSequence()) {
    if (n.size() < 2)
      return false;
    double x = 0.0, y = 0.0;
    if (!looseDouble(n[0], &x) || !looseDouble(n[1], &y))
      return false;
    out.x = x;
    out.y = y;
    return true;
  }
  if (n.IsMap()) {
    double x = 0.0, y = 0.0;
    if (!looseDouble(n["x"], &x) || !looseDouble(n["y"], &y))
      return false;
    out.x = x;
    out.y = y;
    return true;
  }
  return false;
}

} // namespace

// ---------------------------------------------------------------------------
// 节点类型
// ---------------------------------------------------------------------------

std::string toString(RouteNodeType t) {
  switch (t) {
  case RouteNodeType::kWaypoint:
    return "waypoint";
  case RouteNodeType::kStation:
    return "station";
  case RouteNodeType::kCharge:
    return "charge";
  case RouteNodeType::kPark:
    return "park";
  }
  return "waypoint";
}

RouteNodeType routeNodeTypeFromString(const std::string &s, bool *ok) {
  const std::string t = lower(trim(s));
  if (ok)
    *ok = true;
  if (t.empty() || t == "waypoint" || t == "node")
    return RouteNodeType::kWaypoint;
  if (t == "station" || t == "stop")
    return RouteNodeType::kStation;
  if (t == "charge" || t == "charging")
    return RouteNodeType::kCharge;
  if (t == "park" || t == "parking")
    return RouteNodeType::kPark;
  if (ok)
    *ok = false;
  return RouteNodeType::kWaypoint;
}

bool parseLooseBool(const std::string &text, bool &out) {
  const std::string t = lower(trim(text));
  if (t == "1" || t == "true" || t == "yes" || t == "on") {
    out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "off") {
    out = false;
    return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// RouteEdge
// ---------------------------------------------------------------------------

Pose2D RouteEdge::pointAt(double s) const {
  Pose2D p;
  if (polyline.empty())
    return p;
  if (polyline.size() == 1 || length <= 1e-9)
    return polyline.front();

  const double sc = std::clamp(s, 0.0, length);
  double acc = 0.0;
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const double seg = std::hypot(polyline[i + 1].x - polyline[i].x,
                                  polyline[i + 1].y - polyline[i].y);
    if (seg <= 1e-12)
      continue;
    if (acc + seg >= sc) {
      const double t = (sc - acc) / seg;
      p.x = polyline[i].x + (polyline[i + 1].x - polyline[i].x) * t;
      p.y = polyline[i].y + (polyline[i + 1].y - polyline[i].y) * t;
      return p;
    }
    acc += seg;
  }
  return polyline.back();
}

double RouteEdge::yawAt(double s) const {
  // 用有限差分取切向，避免再写一遍"定位到具体段"的逻辑
  const double d = std::min(0.05, std::max(length, 1e-3) * 0.5);
  const double s0 = std::clamp(s - d, 0.0, length);
  const double s1 = std::clamp(s + d, 0.0, length);
  const Pose2D a = pointAt(s0);
  const Pose2D b = pointAt(s1);
  if (std::hypot(b.x - a.x, b.y - a.y) < 1e-9) {
    // 退化：用整条边的首尾方向
    const Pose2D &f = polyline.front();
    const Pose2D &l = polyline.back();
    return std::atan2(l.y - f.y, l.x - f.x);
  }
  return std::atan2(b.y - a.y, b.x - a.x);
}

double distanceToPolyline(double x, double y,
                          const std::vector<Pose2D> &polyline) {
  // 量的是到**中心线**的最近距离（开放折线）：点偏出 0.2 m 就返回 0.2。
  // 与 distanceToPolygon（区域，内部返回 0）不同，这里不能有"内部"的概念。
  if (polyline.empty())
    return std::numeric_limits<double>::infinity();
  if (polyline.size() == 1)
    return std::hypot(x - polyline[0].x, y - polyline[0].y);
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    const double ax = polyline[i].x, ay = polyline[i].y;
    const double bx = polyline[i + 1].x, by = polyline[i + 1].y;
    const double dx = bx - ax, dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = 0.0;
    if (l2 > 1e-18)
      t = std::clamp(((x - ax) * dx + (y - ay) * dy) / l2, 0.0, 1.0);
    best = std::min(best, std::hypot(x - (ax + t * dx), y - (ay + t * dy)));
  }
  return best;
}

// ---------------------------------------------------------------------------
// 载入
// ---------------------------------------------------------------------------

bool RouteGraph::loadFromFile(const std::string &path, std::string &err) {
  std::ifstream f(path);
  if (!f) {
    err = "打不开路网文件: " + path;
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  if (!loadFromString(ss.str(), err)) {
    err = path + " : " + err;
    return false;
  }
  return true;
}

bool RouteGraph::loadFromString(const std::string &yaml, std::string &err) {
  valid_ = false;
  if (!parse(yaml, err))
    return false;
  return valid_;
}

bool RouteGraph::parse(const std::string &yaml_text, std::string &err) {
  nodes_.clear();
  edges_.clear();
  warnings_.clear();
  frame_id_ = "map";

  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const std::exception &e) {
    err = std::string("yaml 解析失败: ") + e.what();
    return false;
  }
  if (!root || !root.IsMap()) {
    err = "yaml 顶层必须是 map（含 frame_id / nodes / edges）";
    return false;
  }

  if (root["frame_id"]) {
    std::string s;
    if (scalarText(root["frame_id"], s) && !s.empty())
      frame_id_ = trim(s);
  }

  // ---- 可选 defaults：给所有边设缺省属性，少写重复字段 ----
  bool def_one_way = false;
  double def_speed = 1.0;
  double def_corridor = 0.6;
  if (root["defaults"] && root["defaults"].IsMap()) {
    const YAML::Node d = root["defaults"];
    def_one_way = looseBool(d["one_way"], def_one_way);
    looseDouble(d["speed_limit"], &def_speed);
    looseDouble(d["corridor_width"], &def_corridor);
  }

  // ---- 节点 ----
  const YAML::Node yn = root["nodes"];
  if (!yn || !yn.IsSequence() || yn.size() == 0) {
    err = "nodes 缺失或为空";
    return false;
  }
  std::unordered_map<std::string, int> by_name;
  for (std::size_t i = 0; i < yn.size(); ++i) {
    const YAML::Node &n = yn[i];
    if (!n.IsMap()) {
      err = "nodes[" + std::to_string(i) + "] 不是 map";
      return false;
    }
    RouteNode node;
    if (!scalarText(n["name"], node.name) || trim(node.name).empty()) {
      err = "nodes[" + std::to_string(i) + "] 缺少 name";
      return false;
    }
    node.name = trim(node.name);
    if (!looseDouble(n["x"], &node.x) || !looseDouble(n["y"], &node.y)) {
      err = "节点 " + node.name + " 缺少合法的 x/y";
      return false;
    }
    if (n["type"]) {
      std::string ts;
      scalarText(n["type"], ts);
      bool ok = true;
      node.type = routeNodeTypeFromString(ts, &ok);
      if (!ok) {
        warnings_.push_back("节点 " + node.name + " 的 type='" + trim(ts) +
                            "' 不认识 → 按 waypoint 处理");
      }
    }
    if (by_name.count(node.name)) {
      err = "节点名重复: " + node.name;
      return false;
    }
    by_name[node.name] = static_cast<int>(nodes_.size());
    nodes_.push_back(node);
  }

  // ---- 边 ----
  const YAML::Node ye = root["edges"];
  if (!ye || !ye.IsSequence() || ye.size() == 0) {
    err = "edges 缺失或为空";
    return false;
  }
  for (std::size_t i = 0; i < ye.size(); ++i) {
    const YAML::Node &e = ye[i];
    const std::string tag = "edges[" + std::to_string(i) + "]";
    if (!e.IsMap()) {
      err = tag + " 不是 map";
      return false;
    }
    std::string from, to;
    if (!scalarText(e["from"], from) || !scalarText(e["to"], to)) {
      err = tag + " 缺少 from/to";
      return false;
    }
    from = trim(from);
    to = trim(to);
    const auto itf = by_name.find(from);
    const auto itt = by_name.find(to);
    if (itf == by_name.end()) {
      err = tag + " 的 from='" + from + "' 不是已定义节点";
      return false;
    }
    if (itt == by_name.end()) {
      err = tag + " 的 to='" + to + "' 不是已定义节点";
      return false;
    }
    if (itf->second == itt->second) {
      err = tag + " 是自环（from == to），不支持";
      return false;
    }

    RouteEdge edge;
    edge.from = itf->second;
    edge.to = itt->second;
    edge.one_way = looseBool(e["one_way"], def_one_way);
    if (!looseDouble(e["speed_limit"], &edge.speed_limit))
      edge.speed_limit = def_speed;
    if (!looseDouble(e["corridor_width"], &edge.corridor_width)) {
      edge.corridor_width = def_corridor;
    }
    if (edge.speed_limit <= 0.0) {
      warnings_.push_back(tag + " speed_limit <= 0 → 按 1.0 m/s 处理");
      edge.speed_limit = 1.0;
    }
    if (edge.corridor_width < 0.0) {
      warnings_.push_back(tag + " corridor_width < 0 → 按 0.0（严格贴线）处理");
      edge.corridor_width = 0.0;
    }

    const YAML::Node yp = e["polyline"];
    if (!yp || !yp.IsSequence() || yp.size() < 2) {
      err = tag + " 的 polyline 缺失或点数 < 2";
      return false;
    }
    edge.polyline.reserve(yp.size());
    for (std::size_t k = 0; k < yp.size(); ++k) {
      Pose2D p;
      if (!parsePolylinePoint(yp[k], p)) {
        err = tag + " 的 polyline[" + std::to_string(k) +
              "] 既不是 [x,y] 也不是 {x,y}";
        return false;
      }
      edge.polyline.push_back(p);
    }

    // 端点与节点坐标自动吸附（画图时手抖几个厘米很常见，不该报错）
    const RouteNode &nf = nodes_[static_cast<std::size_t>(edge.from)];
    const RouteNode &nt = nodes_[static_cast<std::size_t>(edge.to)];
    const double df = std::hypot(edge.polyline.front().x - nf.x,
                                 edge.polyline.front().y - nf.y);
    const double dt = std::hypot(edge.polyline.back().x - nt.x,
                                 edge.polyline.back().y - nt.y);
    if (df > 1e-6) {
      if (df > 0.05) {
        warnings_.push_back(tag + " 起点与节点 " + nf.name + " 相差 " +
                            std::to_string(df) + " m → 已吸附到节点坐标");
      }
      edge.polyline.front().x = nf.x;
      edge.polyline.front().y = nf.y;
    }
    if (dt > 1e-6) {
      if (dt > 0.05) {
        warnings_.push_back(tag + " 终点与节点 " + nt.name + " 相差 " +
                            std::to_string(dt) + " m → 已吸附到节点坐标");
      }
      edge.polyline.back().x = nt.x;
      edge.polyline.back().y = nt.y;
    }

    edges_.push_back(std::move(edge));
  }

  computeLengths();

  if (!validate(err))
    return false;
  valid_ = true;
  return true;
}

void RouteGraph::computeLengths() {
  for (RouteEdge &e : edges_) {
    double L = 0.0;
    for (std::size_t i = 0; i + 1 < e.polyline.size(); ++i) {
      L += std::hypot(e.polyline[i + 1].x - e.polyline[i].x,
                      e.polyline[i + 1].y - e.polyline[i].y);
    }
    e.length = L;
  }
}

bool RouteGraph::validate(std::string &err) const {
  for (std::size_t i = 0; i < edges_.size(); ++i) {
    if (edges_[i].length <= 1e-6) {
      err = "edges[" + std::to_string(i) + "] 长度为 0（折线所有点重合？）";
      return false;
    }
  }
  if (!std::isfinite(frame_id_.size() > 0 ? 1.0 : 0.0)) {
    err = "内部错误";
    return false;
  }
  return true;
}

bool RouteGraph::saveToFile(const std::string &path, std::string &err) const {
  if (!valid_) {
    err = "路网无效，拒绝保存";
    return false;
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "frame_id" << YAML::Value << frame_id_;
  out << YAML::Key << "nodes" << YAML::Value << YAML::BeginSeq;
  for (const RouteNode &n : nodes_) {
    out << YAML::Flow << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << n.name;
    out << YAML::Key << "x" << YAML::Value << n.x;
    out << YAML::Key << "y" << YAML::Value << n.y;
    out << YAML::Key << "type" << YAML::Value << toString(n.type);
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::Key << "edges" << YAML::Value << YAML::BeginSeq;
  for (const RouteEdge &e : edges_) {
    out << YAML::BeginMap;
    out << YAML::Key << "from" << YAML::Value
        << nodes_[static_cast<std::size_t>(e.from)].name;
    out << YAML::Key << "to" << YAML::Value
        << nodes_[static_cast<std::size_t>(e.to)].name;
    out << YAML::Key << "one_way" << YAML::Value << e.one_way;
    out << YAML::Key << "speed_limit" << YAML::Value << e.speed_limit;
    out << YAML::Key << "corridor_width" << YAML::Value << e.corridor_width;
    out << YAML::Key << "polyline" << YAML::Value << YAML::BeginSeq;
    for (const Pose2D &p : e.polyline) {
      out << YAML::Flow << YAML::BeginSeq << p.x << p.y << YAML::EndSeq;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  std::ofstream f(path);
  if (!f) {
    err = "写不了文件: " + path;
    return false;
  }
  f << out.c_str() << "\n";
  if (!f) {
    err = "写入失败: " + path;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 查询
// ---------------------------------------------------------------------------

int RouteGraph::nodeIndex(const std::string &name) const {
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].name == name)
      return static_cast<int>(i);
  }
  return -1;
}

RouteProjection RouteGraph::project(double x, double y) const {
  RouteProjection best;
  double best_d2 = std::numeric_limits<double>::max();
  for (std::size_t ei = 0; ei < edges_.size(); ++ei) {
    const RouteEdge &e = edges_[ei];
    double acc = 0.0;
    for (std::size_t i = 0; i + 1 < e.polyline.size(); ++i) {
      const double ax = e.polyline[i].x, ay = e.polyline[i].y;
      const double bx = e.polyline[i + 1].x, by = e.polyline[i + 1].y;
      const double dx = bx - ax, dy = by - ay;
      const double seg2 = dx * dx + dy * dy;
      if (seg2 <= 1e-18)
        continue;
      double t = ((x - ax) * dx + (y - ay) * dy) / seg2;
      t = std::clamp(t, 0.0, 1.0);
      const double px = ax + dx * t, py = ay + dy * t;
      const double d2 = (x - px) * (x - px) + (y - py) * (y - py);
      if (d2 < best_d2) {
        best_d2 = d2;
        best.edge = static_cast<int>(ei);
        best.s = acc + std::sqrt(seg2) * t;
        best.x = px;
        best.y = py;
      }
      acc += std::sqrt(seg2);
    }
  }
  if (best.edge >= 0) {
    best.valid = true;
    best.dist = std::sqrt(best_d2);
  }
  return best;
}

std::string RouteGraph::summary() const {
  std::size_t stations = 0;
  for (const RouteNode &n : nodes_) {
    if (n.type != RouteNodeType::kWaypoint)
      ++stations;
  }
  double total = 0.0;
  std::size_t one_way = 0;
  for (const RouteEdge &e : edges_) {
    total += e.length;
    if (e.one_way)
      ++one_way;
  }
  std::ostringstream ss;
  ss << nodes_.size() << " 节点（含站点 " << stations << "）/ " << edges_.size()
     << " 条通道（单向 " << one_way << "，总长 " << total << " m）";
  return ss.str();
}

} // namespace pnc_2d
