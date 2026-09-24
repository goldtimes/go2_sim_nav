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
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  double originYaw() const { return origin_yaw_; }
  /// √2/2·resolution：格中心到格角的距离（把"到格中心"修正成"到格区域"的裕量）
  double marginM() const { return margin_; }

  /// 到最近致命格**中心**的距离 [m]；非法下标返回 0（保守 = 认为贴着障碍）。
  double distanceToLethal(int x, int y) const
  {
    if (!inside(x, y)) return 0.0;
    return dist_[static_cast<std::size_t>(y) * width_ + x] * resolution_;
  }

  // ------------------------------------------------------------------
  // 世界坐标查询（轨迹优化的 SDF 罚项用：要"连续值 + 梯度"）
  //
  // 与下标版的关系：世界查询是下标版的**双线性插值**，因此：
  //   · 值与梯度**同源**（梯度就是该插值式的解析导数）—— 必须如此，否则
  //     罚项的值与方向不一致，优化会在格边界上来回震荡；
  //   · 代价是梯度在格边界处不连续（ESDF 类方法的已知问题）。
  //
  // ★★ 已实测的局限（**不是 bug**，写清楚免得以后误判）：
  //   1. 幅值不是严格的 1：稀疏随机图实测 max|∇d| = 1.15（插值误差 ~5%），
  //      不允许出现"虚大"（> 1.6）。
  //   2. 在"到两个障碍等距"的**中轴/脊线**上，d 是**局部最大** ⇒ 真实梯度为零、
  //      方向本身无定义；双线性插值的导数会指向某个任意方向
  //      （实测：靠近脊线的良态样本里约 2% 反向、均值 13.7°、最大 152.6°）。
  //      取证就是"中心格的距离比周围八格都大" —— 见 test_clearance_gradient.cpp
  //      （它会打印首个反向样本的九宫格精确距离）。
  //   ⇒ 对策是**分层**而不是修梯度：脊线处 d 通常已 ≥ 安全距离（罚项不激活），
  //     且安全性最终由**轮廓终检**（精确判据）与 MPC 硬下界兜底。
  //     ⇒ 罚项只当"推力/偏好"用，**不当安全保证**；权重也不宜过大。
  //
  // 越界约定：**不算障碍**（distanceAtWorld 饱和到 max_m、梯度返回 false）。
  //   安全底线不依赖它：轮廓判定（图外一律视为致命）负责兜底。
  // ------------------------------------------------------------------

  /// 世界系是否落在图内
  bool insideWorld(double wx, double wy) const;

  /// 双线性插值净距 [m]（到最近致命格**中心**），饱和于 max_m。
  /// 越界同样返回 max_m（"开阔"），由调用方决定是否关心这件事。
  double distanceAtWorld(double wx, double wy, double max_m = 1.0e3) const;

  /// 保守下界（到格**区域**的距离）：distanceAtWorld − margin_。
  /// ★ 罚项请用这个，不要用 distanceAtWorld：后者可能高估最多 margin_（7 cm）。
  double lowerBoundAtWorld(double wx, double wy, double max_m = 1.0e3) const
  {
    return std::max(0.0, distanceAtWorld(wx, wy, max_m) - margin_);
  }

  /// 距离场梯度（世界系，单位：m/m）。
  /// 返回 false（且 g 置 0）当：场无效 / g == nullptr / 查询点在图外。
  bool gradientAtWorld(double wx, double wy, double g[2]) const;

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

  /// 世界系 → **连续**格坐标（格中心 = 整数 + 0.5）；不做越界检查
  void worldToGridContinuous(double wx, double wy, double & gx, double & gy) const
  {
    const double dx = wx - origin_x_;
    const double dy = wy - origin_y_;
    gx = (cos_yaw_ * dx + sin_yaw_ * dy) / resolution_;
    gy = (-sin_yaw_ * dx + cos_yaw_ * dy) / resolution_;
  }

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double margin_{0.0};          // (√2/2)·resolution
  std::size_t lethal_count_{0};
  std::vector<float> dist_;     // 到最近致命格的距离，单位：格
  // 世界系原点（下标查询不需要，世界查询需要）
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double cos_yaw_{1.0};
  double sin_yaw_{0.0};
};

}  // namespace pnc_2d
