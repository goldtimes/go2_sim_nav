// 精确距离场（到最近致命格的距离）：footprint 检查的"快路径"基础。
//
// 为什么要它：矩形 footprint 的完整检查要遍历约 40 个格子。而绝大多数格子都在
// 开阔区域，根本不需要这么细的检查 —— 只要知道"离最近障碍有多远"就够了。
// 有了精确距离场，判断变成**一次查表 + 两个阈值**：
//
//   设 d = 到最近致命格中心的距离，R_in = 内切半径，R_circ = 外接半径，
//   e = (√2/2)·分辨率（格中心到该格最远点的距离，用来把"到格中心"修正成
//       保守的"到格区域"界）：
//     d - e >= R_circ  → **任何朝向都安全**（整机都在以 d-e 为半径的无障碍圆内）
//     d + e <  R_in    → **任何朝向都碰撞**（内切圆里就有障碍）
//   落在两者之间（R_in 与 R_circ 之间的窄环带，只在障碍附近出现）才做完整矩形检查。
//
// 两条快路径都是**保守正确**的：前者可能漏判"其实安全"（多做一次完整检查），
// 后者可能把"其实安全"判成碰撞 —— 但后者不会放过真正的碰撞。所以：
//   · 快路径只用于"加速"，判定结果与完整检查一致；
//   · 绝不因为快路径而放宽安全检查。

#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

namespace pnc_2d {

class CostMap2D;

class ClearanceField {
public:
  /// 由地图 + 致命判定重建。unknown_as_occupied=true 时未知格也算致命格。
  bool build(const CostMap2D & map, int hard_threshold, bool unknown_as_occupied);

  bool valid() const { return width_ > 0 && height_ > 0 && dist_.size() == distSz(); }

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }

  /// 到最近致命格**中心**的距离 [m]；非法下标返回 0（保守 = 认为贴着障碍）。
  double distanceToLethal(int x, int y) const
  {
    if (!inside(x, y)) return 0.0;
    return dist_[static_cast<std::size_t>(y) * width_ + x] * resolution_;
  }

  /// 由"到格中心距离"推出的"到格区域"下界/上界 [m]，见文件头说明。
  double lowerBoundM(int x, int y) const { return std::max(0.0, distanceToLethal(x, y) - margin_); }
  double upperBoundM(int x, int y) const { return distanceToLethal(x, y) + margin_; }

  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

  /// 诊断：致命格数量
  std::size_t lethalCount() const { return lethal_count_; }

private:
  std::size_t distSz() const
  {
    return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  }

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double margin_{0.0};          // (√2/2)·resolution
  std::size_t lethal_count_{0};
  std::vector<float> dist_;     // 到最近致命格的距离，单位：格
};

}  // namespace pnc_2d
