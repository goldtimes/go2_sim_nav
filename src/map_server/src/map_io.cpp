#include "map_server/map_io.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace map_server {
namespace {

/// 读 PGM 头部的下一个 token（跳过 # 注释、空行）
bool nextToken(std::istream &is, std::string &tok) {
  tok.clear();
  char c;
  while (is.get(c)) {
    if (c == '#') { // 注释到行尾
      std::string dummy;
      std::getline(is, dummy);
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c)))
      continue;
    break;
  }
  if (!is)
    return false;
  tok.push_back(c);
  while (is.get(c)) {
    if (std::isspace(static_cast<unsigned char>(c)))
      break;
    tok.push_back(c);
  }
  return !tok.empty();
}

std::string dirOf(const std::string &path) {
  const fs::path p(path);
  return p.has_parent_path() ? p.parent_path().string() : std::string(".");
}

/**
 * 健壮的 bool 解析。map.yaml 里很常见 `negate: 0` / `negate: 1`（整数）而不是
 * true/false，而 yaml-cpp 的 as<bool>() 对整数会直接报 "bad conversion"。
 */
bool asBool(const YAML::Node &n, bool def) {
  if (!n)
    return def;
  if (n.IsScalar()) {
    const std::string s = n.Scalar();
    if (s == "true" || s == "True" || s == "TRUE" || s == "1" || s == "yes")
      return true;
    if (s == "false" || s == "False" || s == "FALSE" || s == "0" || s == "no")
      return false;
  }
  try {
    return n.as<bool>();
  } catch (const std::exception &) {
    return def;
  }
}

} // namespace

std::string resolveMapYaml(const std::string &dir_or_yaml) {
  if (dir_or_yaml.empty())
    return dir_or_yaml;
  std::error_code ec;
  if (fs::is_directory(dir_or_yaml, ec))
    return (fs::path(dir_or_yaml) / "map.yaml").string();
  return dir_or_yaml;
}

bool loadMapYaml(const std::string &yaml_path, MapYaml &out, std::string &err) {
  if (!fs::exists(yaml_path)) {
    err = "map.yaml 不存在: " + yaml_path;
    return false;
  }
  try {
    const YAML::Node y = YAML::LoadFile(yaml_path);
    if (!y["image"]) {
      err = "map.yaml 缺少 image 字段: " + yaml_path;
      return false;
    }
    out.image = y["image"].as<std::string>();
    if (y["mode"])
      out.mode = y["mode"].as<std::string>();
    if (y["resolution"])
      out.resolution = y["resolution"].as<double>();
    if (y["negate"])
      out.negate = asBool(y["negate"], false);
    if (y["occupied_thresh"])
      out.occupied_thresh = y["occupied_thresh"].as<double>();
    if (y["free_thresh"])
      out.free_thresh = y["free_thresh"].as<double>();
    if (y["origin"]) {
      const auto o = y["origin"].as<std::vector<double>>();
      if (o.size() < 2) {
        err = "map.yaml 的 origin 至少要有 x,y: " + yaml_path;
        return false;
      }
      out.origin_x = o[0];
      out.origin_y = o[1];
      out.origin_yaw = (o.size() >= 3) ? o[2] : 0.0;
    }
    if (out.resolution <= 0.0) {
      err = "map.yaml 的 resolution 必须 > 0";
      return false;
    }
  } catch (const std::exception &e) {
    err = std::string("map.yaml 解析失败: ") + e.what();
    return false;
  }
  return true;
}

bool loadPgm(const std::string &pgm_path, int &w, int &h,
             std::vector<uint8_t> &pixels, std::string &err) {
  std::ifstream ifs(pgm_path, std::ios::binary);
  if (!ifs) {
    err = "PGM 打不开: " + pgm_path;
    return false;
  }
  std::string magic, sw, sh, smax;
  if (!nextToken(ifs, magic)) {
    err = "PGM 头部读取失败: " + pgm_path;
    return false;
  }
  if (magic != "P5" && magic != "P2") {
    err = "不支持的 PGM 格式（只支持 P5 二进制 / P2 ASCII）: " + magic;
    return false;
  }
  if (!nextToken(ifs, sw) || !nextToken(ifs, sh) || !nextToken(ifs, smax)) {
    err = "PGM 头部不完整: " + pgm_path;
    return false;
  }
  try {
    w = std::stoi(sw);
    h = std::stoi(sh);
    const int maxval = std::stoi(smax);
    if (w <= 0 || h <= 0) {
      err = "PGM 尺寸非法";
      return false;
    }
    if (maxval > 255) {
      err = "PGM maxval > 255（16bit）暂不支持";
      return false;
    }
  } catch (const std::exception &e) {
    err = std::string("PGM 头部数字解析失败: ") + e.what();
    return false;
  }

  pixels.assign(static_cast<size_t>(w) * h, 0);
  if (magic == "P5") {
    // 头之后通常有 1 个空白字符（已被 nextToken 吃掉）
    ifs.read(reinterpret_cast<char *>(pixels.data()),
             static_cast<std::streamsize>(pixels.size()));
    if (ifs.gcount() != static_cast<std::streamsize>(pixels.size())) {
      err = "PGM 像素数据不足: 期望 " + std::to_string(pixels.size()) +
            " 字节，实得 " + std::to_string(ifs.gcount());
      return false;
    }
  } else { // P2：ASCII 灰度
    for (size_t i = 0; i < pixels.size(); ++i) {
      int v = 0;
      if (!(ifs >> v)) {
        err = "PGM(P2) 像素数据不足";
        return false;
      }
      pixels[i] = static_cast<uint8_t>(std::max(0, std::min(255, v)));
    }
  }
  return true;
}

bool loadOccupancyMap(const std::string &dir_or_yaml, OccupancyMap &out,
                      std::string &err) {
  const std::string yaml_path = resolveMapYaml(dir_or_yaml);
  MapYaml my;
  if (!loadMapYaml(yaml_path, my, err))
    return false;

  // image 路径：相对 yaml 所在目录，也允许绝对路径
  std::string img = my.image;
  if (!fs::path(img).is_absolute())
    img = (fs::path(dirOf(yaml_path)) / img).string();

  int w = 0, h = 0;
  std::vector<uint8_t> px;
  if (!loadPgm(img, w, h, px, err))
    return false;

  out.width = w;
  out.height = h;
  out.resolution = my.resolution;
  out.origin_x = my.origin_x;
  out.origin_y = my.origin_y;
  out.origin_yaw = my.origin_yaw;
  out.data.assign(static_cast<size_t>(w) * h, -1);
  out.n_occupied = out.n_free = out.n_unknown = 0;

  /* trinary 语义（与 nav2 map_server 一致）：
       p = 灰度；negate 先把黑白反过来
       occ = (255 - p) / 255      → 黑(0) = 1.0 = 占据
       occ > occupied_thresh → 100；occ < free_thresh → 0；其余 → -1
     ⚠ PGM 第 0 行是图像**顶部**，OccupancyGrid 的 y=0 是地图**底部** →
     必须翻转。 */
  for (int y = 0; y < h; ++y) {
    const uint8_t *row = &px[static_cast<size_t>(h - 1 - y) * w];
    int8_t *dst = &out.data[static_cast<size_t>(y) * w];
    for (int x = 0; x < w; ++x) {
      const double p = static_cast<double>(row[x]);
      const double occ = my.negate ? (p / 255.0) : ((255.0 - p) / 255.0);
      int8_t v;
      if (occ > my.occupied_thresh)
        v = 100;
      else if (occ < my.free_thresh)
        v = 0;
      else
        v = -1;
      dst[x] = v;
      if (v == 100)
        ++out.n_occupied;
      else if (v == 0)
        ++out.n_free;
      else
        ++out.n_unknown;
    }
  }
  return true;
}

std::string resolvePcd(const std::string &dir_or_pcd) {
  if (dir_or_pcd.empty())
    return dir_or_pcd;
  const std::string lower = [&] {
    std::string s = dir_or_pcd;
    for (auto &c : s)
      c = static_cast<char>(std::tolower(c));
    return s;
  }();
  if (lower.size() > 4 && lower.compare(lower.size() - 4, 4, ".pcd") == 0)
    return dir_or_pcd;
  std::error_code ec;
  if (fs::is_directory(dir_or_pcd, ec))
    return (fs::path(dir_or_pcd) / "global.pcd").string();
  // 不是目录也不是 .pcd：当成"地图目录"再拼一次（让调用方得到明确的不存在报错）
  return (fs::path(dir_or_pcd) / "global.pcd").string();
}

bool loadPcd(const std::string &pcd_path, PointCloud3D &out, std::string &err) {
  out.xyz.clear();
  out.n_points = 0;
  if (!fs::exists(pcd_path)) {
    err = "PCD 不存在: " + pcd_path;
    return false;
  }

  /* ---- 头部解析（这部分是"接口"该做的：校验 + 自省）----
     PCD 头部是一行一个 "KEY value"，到 DATA 行为止；# 开头是注释。
     实测这些地图是：VERSION 0.7 / FIELDS x y z intensity time /
     SIZE 4 4 4 4 8 / DATA binary_compressed（LZF 压缩）。 */
  std::ifstream ifs(pcd_path);
  if (!ifs) {
    err = "PCD 打不开: " + pcd_path;
    return false;
  }
  size_t points = 0;
  std::string fields, data_fmt;
  bool saw_version = false, saw_data = false;
  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream ls(line);
    std::string key;
    ls >> key;
    if (key == "VERSION")
      saw_version = true;
    else if (key == "FIELDS" || key == "COLUMNS")
      std::getline(ls, fields);
    else if (key == "POINTS")
      ls >> points;
    else if (key == "WIDTH" && points == 0)
      ls >> points; // 没有 POINTS 行时用 WIDTH（HEIGHT=1 的常见情形）
    else if (key == "DATA") {
      ls >> data_fmt;
      saw_data = true;
      break; // 头部结束
    }
  }
  if (!saw_version || !saw_data) {
    err = "不是合法的 PCD（缺少 VERSION 或 DATA 行）: " + pcd_path;
    return false;
  }
  const bool has_xyz = fields.find('x') != std::string::npos &&
                       fields.find('y') != std::string::npos &&
                       fields.find('z') != std::string::npos;
  if (!has_xyz) {
    err = "PCD 缺少 x/y/z 字段（FIELDS =" + fields + "）: " + pcd_path;
    return false;
  }
  out.n_points = points; // 供日志/调用方参考（下面的"内容"还没实现）

  /* ---- M3 待实现：把点解码进 out.xyz ----
     需要处理 DATA=ascii / binary / binary_compressed（后者是
     LZF，本文件就是它）。 建议：先支持 ascii+binary，binary_compressed 用 LZF
     解压（可参考 PCL 的 io::loadPCDFile 或直接依赖 pcl_io）。解完把 xyz 填进
     sensor_msgs/PointCloud2 （发布器/latched/周期重发/服务接入都已就绪，见
     map_server_node.cpp）。 */
  err = "PCD 解析未实现（M3 待做）：" + pcd_path + " | " +
        std::to_string(points) + " 点 | FIELDS =" + fields +
        " | DATA =" + data_fmt;
  return false;
}

} // namespace map_server
