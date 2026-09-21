#include "pnc_2d/core/map_zones.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace pnc_2d {
namespace {

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos)
    return "";
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

bool scalarText(const YAML::Node &n, std::string &out) {
  if (!n || !n.IsScalar())
    return false;
  out = n.Scalar();
  return true;
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

/// 顶点写法：[x, y] 或 {x:, y:}
bool parseVertex(const YAML::Node &n, Pose2D &p) {
  if (!n)
    return false;
  if (n.IsSequence()) {
    if (n.size() < 2)
      return false;
    return looseDouble(n[0], &p.x) && looseDouble(n[1], &p.y);
  }
  if (n.IsMap()) {
    return looseDouble(n["x"], &p.x) && looseDouble(n["y"], &p.y);
  }
  return false;
}

double pointSegDistance(double px, double py, double ax, double ay, double bx,
                        double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double seg2 = dx * dx + dy * dy;
  if (seg2 <= 1e-18)
    return std::hypot(px - ax, py - ay);
  double t = ((px - ax) * dx + (py - ay) * dy) / seg2;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(px - (ax + dx * t), py - (ay + dy * t));
}

} // namespace

// ---------------------------------------------------------------------------
// 类型
// ---------------------------------------------------------------------------

std::string toString(ZoneType t) {
  return t == ZoneType::kForbidden ? "forbidden" : "speed_limit";
}

ZoneType zoneTypeFromString(const std::string &s, bool *ok) {
  const std::string t = lower(trim(s));
  if (ok)
    *ok = true;
  if (t.empty() || t == "forbidden" || t == "keepout" || t == "blocked") {
    return ZoneType::kForbidden;
  }
  if (t == "speed_limit" || t == "speed" || t == "slow")
    return ZoneType::kSpeedLimit;
  if (ok)
    *ok = false;
  return ZoneType::kForbidden;
}

// ---------------------------------------------------------------------------
// 几何
// ---------------------------------------------------------------------------

bool pointInPolygon(double x, double y, const std::vector<Pose2D> &poly) {
  if (poly.size() < 3)
    return false;
  bool inside = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double xi = poly[i].x, yi = poly[i].y;
    const double xj = poly[j].x, yj = poly[j].y;
    // 射线法：向 +x 方向投射，统计跨越次数（用半开区间避免顶点重复计数）
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

double distanceToPolygon(double x, double y, const std::vector<Pose2D> &poly) {
  if (poly.empty())
    return std::numeric_limits<double>::infinity();
  if (poly.size() == 1)
    return std::hypot(x - poly[0].x, y - poly[0].y);
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Pose2D &a = poly[i];
    const Pose2D &b = poly[(i + 1) % poly.size()];
    best = std::min(best, pointSegDistance(x, y, a.x, a.y, b.x, b.y));
  }
  return best;
}

// ---------------------------------------------------------------------------
// ZoneSet
// ---------------------------------------------------------------------------

bool ZoneSet::loadFromFile(const std::string &path, std::string &err) {
  std::ifstream f(path);
  if (!f) {
    err = "打不开文件: " + path;
    return false;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return loadFromString(ss.str(), err);
}

bool ZoneSet::loadFromString(const std::string &yaml, std::string &err) {
  zones_.clear();
  warnings_.clear();
  valid_ = false;

  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const std::exception &e) {
    err = std::string("yaml 解析失败: ") + e.what();
    return false;
  }
  if (!root || !root.IsMap()) {
    err = "yaml 顶层必须是 map";
    return false;
  }

  const YAML::Node yz = root["zones"];
  if (!yz) { // 没有 zones 段是完全正常的
    valid_ = true;
    return true;
  }
  if (!yz.IsSequence()) {
    err = "zones 必须是序列";
    return false;
  }

  for (std::size_t i = 0; i < yz.size(); ++i) {
    const YAML::Node &n = yz[i];
    const std::string tag = "zones[" + std::to_string(i) + "]";
    if (!n.IsMap()) {
      err = tag + " 不是 map";
      return false;
    }
    MapZone z;
    std::string name;
    if (scalarText(n["name"], name))
      z.name = trim(name);
    if (z.name.empty())
      z.name = "zone" + std::to_string(i);

    if (n["type"]) {
      std::string ts;
      scalarText(n["type"], ts);
      bool ok = true;
      z.type = zoneTypeFromString(ts, &ok);
      if (!ok) {
        warnings_.push_back(tag + " type='" + trim(ts) +
                            "' 不认识 → 按 forbidden 处理");
      }
    }
    if (z.type == ZoneType::kSpeedLimit) {
      if (!looseDouble(n["value"], &z.value) || z.value <= 0.0) {
        warnings_.push_back(tag + "（限速区 " + z.name +
                            "）缺少合法 value → 按 0.3 m/s 处理");
        z.value = 0.3;
      }
    }

    const YAML::Node yp = n["polygon"];
    if (!yp || !yp.IsSequence() || yp.size() < 3) {
      err = tag + " 的 polygon 缺失或顶点 < 3";
      return false;
    }
    z.polygon.reserve(yp.size());
    for (std::size_t k = 0; k < yp.size(); ++k) {
      Pose2D p;
      if (!parseVertex(yp[k], p)) {
        err = tag + " 的 polygon[" + std::to_string(k) +
              "] 既不是 [x,y] 也不是 {x,y}";
        return false;
      }
      z.polygon.push_back(p);
    }
    // 自交检查只做提示：自交多边形会让"在不在里面"变得反直觉
    z.min_x = z.polygon[0].x;
    z.max_x = z.polygon[0].x;
    z.min_y = z.polygon[0].y;
    z.max_y = z.polygon[0].y;
    for (const Pose2D &p : z.polygon) {
      z.min_x = std::min(z.min_x, p.x);
      z.max_x = std::max(z.max_x, p.x);
      z.min_y = std::min(z.min_y, p.y);
      z.max_y = std::max(z.max_y, p.y);
    }
    if (z.max_x - z.min_x < 1e-6 || z.max_y - z.min_y < 1e-6) {
      warnings_.push_back(tag + "（" + z.name + "）面积近似为 0，可能退化");
    }
    zones_.push_back(std::move(z));
  }

  valid_ = true;
  return true;
}

std::size_t ZoneSet::forbiddenCount() const {
  std::size_t n = 0;
  for (const MapZone &z : zones_) {
    if (z.type == ZoneType::kForbidden)
      ++n;
  }
  return n;
}

std::size_t ZoneSet::speedCount() const {
  std::size_t n = 0;
  for (const MapZone &z : zones_) {
    if (z.type == ZoneType::kSpeedLimit)
      ++n;
  }
  return n;
}

bool ZoneSet::inForbidden(double x, double y) const {
  for (const MapZone &z : zones_) {
    if (z.type != ZoneType::kForbidden)
      continue;
    if (x < z.min_x || x > z.max_x || y < z.min_y || y > z.max_y)
      continue;
    if (pointInPolygon(x, y, z.polygon))
      return true;
  }
  return false;
}

bool ZoneSet::inSpeedZone(double x, double y, double &limit) const {
  bool hit = false;
  double best = std::numeric_limits<double>::infinity();
  for (const MapZone &z : zones_) {
    if (z.type != ZoneType::kSpeedLimit)
      continue;
    if (x < z.min_x || x > z.max_x || y < z.min_y || y > z.max_y)
      continue;
    if (pointInPolygon(x, y, z.polygon)) {
      hit = true;
      best = std::min(best, z.value);
    }
  }
  if (hit)
    limit = best;
  return hit;
}

std::string ZoneSet::summary() const {
  std::ostringstream ss;
  ss << zones_.size() << " 个区域（禁行 " << forbiddenCount() << " / 限速 "
     << speedCount() << "）";
  for (const MapZone &z : zones_) {
    ss << " | " << z.name << ":" << toString(z.type);
    if (z.type == ZoneType::kSpeedLimit)
      ss << "@" << z.value << "m/s";
  }
  return ss.str();
}

// ---------------------------------------------------------------------------
// 烧进栅格
// ---------------------------------------------------------------------------

std::size_t burnForbidden(const ZoneSet &zones, const CostMap2D &map,
                          double inflate_m, std::vector<int8_t> &out) {
  out = map.data();
  std::size_t burned = 0;
  if (zones.empty() || !map.valid())
    return burned;
  const int w = map.width();
  const int h = map.height();
  const double res = map.resolution();
  const double ox = map.originX();
  const double oy = map.originY();
  const double inf = std::max(0.0, inflate_m);

  for (const MapZone &z : zones.zones()) {
    if (z.type != ZoneType::kForbidden)
      continue;
    // 只扫包围盒（+膨胀）范围内，大图上也很快
    const int x0 =
        std::max(0, static_cast<int>(std::floor((z.min_x - inf - ox) / res)));
    const int x1 = std::min(
        w - 1, static_cast<int>(std::ceil((z.max_x + inf - ox) / res)));
    const int y0 =
        std::max(0, static_cast<int>(std::floor((z.min_y - inf - oy) / res)));
    const int y1 = std::min(
        h - 1, static_cast<int>(std::ceil((z.max_y + inf - oy) / res)));
    for (int cy = y0; cy <= y1; ++cy) {
      for (int cx = x0; cx <= x1; ++cx) {
        const double wx = ox + (cx + 0.5) * res;
        const double wy = oy + (cy + 0.5) * res;
        const bool hit = pointInPolygon(wx, wy, z.polygon) ||
                         distanceToPolygon(wx, wy, z.polygon) <= inf;
        if (!hit)
          continue;
        int8_t &cell = out[static_cast<std::size_t>(cy) * w + cx];
        if (cell < CostMap2D::kOccupied) {
          cell = CostMap2D::kOccupied;
          ++burned;
        }
      }
    }
  }
  return burned;
}

} // namespace pnc_2d
