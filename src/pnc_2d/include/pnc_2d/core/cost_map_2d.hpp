// 二维占据/代价栅格：只负责"数据 + 几何"，不含任何障碍语义。
//
// 障碍语义（硬阈值、软代价、未知格策略）刻意放在 GlobalPlanner 的 CostModel 里：
//   reference 工程 SLAM-PNC 是每个搜索器各有一套阈值，本包沿用这个灵活性，
//   但把"通用部分"做成基类的共享实现，避免每个算法重写一遍。
//
// 坐标系：支持 origin 带 yaw（nav_msgs/OccupancyGrid 允许），
//   world = origin + R(yaw) * (局部坐标)，局部原点在栅格左下角。

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pnc_2d {

class CostMap2D {
public:
  static constexpr int8_t kUnknown = -1;
  static constexpr int8_t kOccupied = 100;

  /// 从 OccupancyGrid 的字段填充。成功返回 true（尺寸/分辨率/数据长度必须自洽）。
  bool set(int width, int height, double resolution,
           double origin_x, double origin_y, double origin_yaw,
           std::vector<int8_t> data, const std::string & frame_id);

  bool valid() const
  {
    return width_ > 0 && height_ > 0 && resolution_ > 0.0 &&
           data_.size() == static_cast<std::size_t>(width_) * height_;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  std::size_t cellCount() const { return data_.size(); }
  const std::string & frameId() const { return frame_id_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double originYaw() const { return origin_yaw_; }
  const std::vector<int8_t> & data() const { return data_; }

  bool inside(int x, int y) const
  {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  /// 原始值；越界按"占据"返回，这样调用方不必单独处理越界。
  int8_t rawValue(int x, int y) const
  {
    return inside(x, y) ? data_[static_cast<std::size_t>(y) * width_ + x] : kOccupied;
  }

  /// 世界系 → 栅格下标。点落在图外返回 false（下标值未定义）。
  bool worldToGrid(double wx, double wy, int & x, int & y) const;

  /// 世界系 → **连续**栅格坐标（单位：格；格中心为整数+0.5）。不判越界，
  /// 供 DDA 体素遍历/插值使用。
  void worldToGridContinuous(double wx, double wy, double & gx, double & gy) const;

  /// 栅格下标 → 该格中心的世界坐标。
  void gridToWorld(int x, int y, double & wx, double & wy) const;

private:
  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double cos_yaw_{1.0};
  double sin_yaw_{0.0};
  std::string frame_id_{"map"};
  std::vector<int8_t> data_;
};

}  // namespace pnc_2d
