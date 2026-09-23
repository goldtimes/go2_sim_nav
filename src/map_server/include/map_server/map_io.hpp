#ifndef MAP_SERVER__MAP_IO_HPP_
#define MAP_SERVER__MAP_IO_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace map_server {

/**
 * map.yaml 的字段（nav2 / ROS 标准）。
 * 只实现 trinary 语义（我们的地图都是 trinary）；其它 mode 会告警并按 trinary
 * 处理， 避免"静默算错"。
 */
struct MapYaml {
  std::string image;           ///< 相对 yaml 所在目录（也支持绝对路径）
  std::string mode{"trinary"}; ///< trinary / scale / raw
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0}; ///< origin: [x, y, yaw]，yaw 单位弧度
  bool negate{false};
  double occupied_thresh{0.65};
  double free_thresh{0.196};
};

/**
 * 加载结果：语义与 nav_msgs/OccupancyGrid 完全一致
 *   data[y * width + x]，y=0 是**地图底部**（与 OccupancyGrid 一致，不是 PGM
 * 的行序） 取值 100(占据) / 0(空闲) / -1(未知)
 */
struct OccupancyMap {
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<int8_t> data;
  size_t n_occupied{0};
  size_t n_free{0};
  size_t n_unknown{0};
};

/**
 * 把"目录或 yaml 路径"统一成 yaml 路径：
 *   目录      → <dir>/map.yaml
 *   *.yaml    → 原样返回
 * 方便多站点切换时直接传站点目录。
 */
std::string resolveMapYaml(const std::string &dir_or_yaml);

/// 解析 map.yaml。失败时 err 填原因
bool loadMapYaml(const std::string &yaml_path, MapYaml &out, std::string &err);

/// 解析 PGM（支持 P5 二进制 / P2 ASCII，maxval<=255）。pixels 为 8bit 灰度、PGM
/// 原始行序
bool loadPgm(const std::string &pgm_path, int &w, int &h,
             std::vector<uint8_t> &pixels, std::string &err);

/**
 * 完整加载：yaml + pgm → 占据栅格。
 * 关键点：trinary 阈值映射 + **行序翻转**（PGM 第 0 行在图像顶部，OccupancyGrid
 * 的 第 0 行在地图底部；漏了翻转地图会上下颠倒，是这里最经典的 bug）。
 */
bool loadOccupancyMap(const std::string &dir_or_yaml, OccupancyMap &out,
                      std::string &err);

/* ======================= 3D 地图（M3 接口）=======================
 * 只定义数据形态与入口：同一个地图目录下的 global.pcd。
 * 本期不实现解析（loadPcd 会明确返回"未实现"，不静默当空图）。
 */
struct PointCloud3D {
  std::vector<float> xyz; ///< 3*N，x,y,z 交错
  size_t n_points{0};     ///< = xyz.size() / 3
};

/// 地图目录 → <dir>/global.pcd；若给的就是 .pcd 则原样返回
std::string resolvePcd(const std::string &dir_or_pcd);

/**
 * 读取 PCD。**M3 待实现**：目前只做"文件是否存在"+ PCD 头合法性校验，
 * 然后返回 false 并在 err 里明说"PCD 解析未实现"（便于 M3 时在这里填内容）。
 */
bool loadPcd(const std::string &pcd_path, PointCloud3D &out, std::string &err);

/* ======================= 栅格后处理 ======================= */

/**
 * 对占据格做**圆形膨胀**（给共享的全局图加一层几 cm 的安全余量）。
 *
 * 做的事情：把"离任意**原始**占据格 ≤ radius"的格也标成占据。
 *
 * 为什么要它：全局图是共享资产，消费它的模块不都会做朝向感知的车体扫掠检查
 * （RViz、以后接 nav2/别的导航栈、以及任何"只看中心格"的快速判定）。图上没有
 * 余量时，这些消费者会把车使到离墙 0 的地方。几 cm 的膨胀层能让"点在图上"就
 * 自带安全余量。
 *
 * 三个容易写错的点（都有单测钉住）：
 *  1. **只能对原始占据集合盖章**（先收集索引再写），否则一轮轮扩散成 k·radius；
 *  2. **用圆盘不用方阵**：方阵在斜角上会多出 (√2−1)·r 的余量，看起来像
 *     "四个方向半径不一致"；
 *  3. 膨胀量按**格数取整（ceil）** ⇒ 实际值 ≥ 请求值，误差 < 1 格分辨率；
 *     判据是格心到格心的距离（`dx²+dy² ≤ cells²`），所以
 *     **cells=1 是十字（5 格，对角格中心距 √2·res > radius，不在盘内）**，
 *     cells=2 是 13 格（含对角 (±1,±1)）。想要连对角一起盖住，半径必须 > 1 格
 *     （例：0.05 m 分辨率下用 0.06 m）。
 *
 * 未知格（-1）也会被覆盖成占据：膨胀层的语义就是"这些格不可通行"，而"未知怎么算"
 * 是消费方的事（`common.unknown_as_occupied`）。如果图本来就是 unknown_as_free
 * 的语义，调用前 -1 已经被换成 0 了。
 *
 * @param data    栅格（行优先，`data[y*width+x]`，y=0 在地图**底部**；-1 =
 * 未知）
 * @param resolution 分辨率 [m/格]
 * @param radius  膨胀半径 [m]；<= 0 = 不做事（返回 0）
 * @param threshold  ≥ 该值算"占据"（与规划器的 hard_threshold 同语义，默认 50）
 * @param occupied_value 膨胀出来的格子写什么值（默认 100）
 * @return 新变成占据的格数（0 = 一格未改）
 */
std::size_t dilateOccupied(std::vector<int8_t> &data, int width, int height,
                           double resolution, double radius, int threshold = 50,
                           int8_t occupied_value = 100);

} // namespace map_server

#endif // MAP_SERVER__MAP_IO_HPP_
