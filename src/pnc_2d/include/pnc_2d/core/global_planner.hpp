// 全局规划算法的抽象基类。
//
// 设计要点（见 doc/pnc2d_dev_plan.md §4）：
//   1. 库 **ROS-free**：只依赖 ParamReader / CostMap2D /
//   clearance_field，单测不必起 ROS；
//   2. 算法只实现三件事：type() / configure() / plan()；
//   3. 公共部分全部在基类里做**唯一实现**，避免每加一个算法都重写一遍：
//        · 代价模型（硬阈值 / 软代价 / 未知格策略）
//        · 碰撞判定（矩形 footprint + 距离场快路径，见
//        FootprintCollisionChecker） ·
//        搜索窗口（起终点包围盒外扩，供"逐级扩大重试"使用） ·
//        线段单格遍历（软代价累加 / 硬碰检查，Amanatides & Woo） ·
//        路径后处理（剪枝 + yaw 填充 + 可选重采样）
//
// 新增一种算法（JPS / 混合 A* / RRT*）只要三步：继承 + 在 configure
// 里读自己的参数 + 在 planner_factory.cpp 里加一行。

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/core/param_reader.hpp"
#include "pnc_2d/core/types.hpp"

namespace pnc_2d {

class RouteGraph;

class GlobalPlanner {
public:
  virtual ~GlobalPlanner() = default;

  // ---------------- 子类必须实现 ----------------
  /// 算法名（用于 planner.type 匹配与日志）
  virtual std::string type() const = 0;
  /// 读参数（前缀如 "astar."）；公共项请调 loadCommonParams()
  virtual bool configure(const ParamReader &params) = 0;
  /// 规划。失败必须填 status + message，**不允许**返回上次的路径充数
  virtual PlanResult plan(const PlanRequest &req) = 0;
  /// 清空内部缓存（换图/换参数后由 setCostMap 调用）
  virtual void reset() {}

  // ---------------- 基类提供 ----------------
  /// 绑定地图（非虚：地图是所有算法共享的资源，换图时重建距离场与方向偏移表）
  void setCostMap(std::shared_ptr<const CostMap2D> map);

  const CostMap2D *costMap() const { return map_.get(); }
  const CostModel &costModel() const { return cost_; }
  const FootprintParams &footprintParams() const { return fp_; }
  const FootprintCollisionChecker &collisionChecker() const {
    return collision_;
  }
  /// 距离场（**值与梯度**）：轨迹优化器（MINCO）的平滑段要用。
  /// ★ 与 `collisionChecker()` 一样是"借出去的那一份"：节点不自己再建一份
  ///   （"同一判据配两遍"已经裁过多次），而且这里返回的指针在
  ///   `setCostMap()` 重建时**会变** ⇒ 调用方必须在地图更新后重新取一次。
  ///   地图无效时为 nullptr。
  const ClearanceField *clearanceField() const { return clearance_.get(); }

  // ---- 可选能力：只有"路网类"算法才有内容，供节点做可视化/诊断 ----
  /// 路网（没有则返回 nullptr）
  virtual const RouteGraph *routeGraph() const { return nullptr; }
  /// 上一次规划走过的通道索引（用于高亮）
  virtual std::vector<int> activeRouteEdges() const { return {}; }

  // ---- 共享工具（子类直接用，不要重复实现）----
  /// 栅格是否硬障碍（含未知策略；图外为 true）
  bool isHardOccupiedGrid(int x, int y) const;
  bool isHardOccupiedWorld(double wx, double wy) const;

  /// 机器人能否从位姿 a 直线移动到 b（含 footprint 扫掠检查），供剪枝使用
  bool lineIsCollisionFree(const Pose2D &a, const Pose2D &b) const;
  /// 纯几何：线段是否穿过硬障碍（不含 footprint）
  bool lineHitsHardObstacle(double x0, double y0, double x1, double y1) const;
  /// 线段穿过的格子上的软代价之和（用于剪枝时保护"绕开软代价区"的收益）
  double softCostAlong(double x0, double y0, double x1, double y1) const;

  /// 进入某格除"走的距离"之外的额外代价（硬障碍返回
  /// +inf）——搜索算法算路径代价用
  double cellExtraCost(int x, int y) const;

  /// 搜索窗口：起终点包围盒外扩 margin_m 后裁剪到地图内（margin 可给 1e9
  /// 表示整图）
  SearchWindow makeSearchWindow(const PlanRequest &req, double margin_m) const;

  double pathLength(const std::vector<Pose2D> &path) const;

  // ---- 计时（子类 plan() 里用）----
  static std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
  }
  static double msSince(const std::chrono::steady_clock::time_point &t0);

protected:
  /// 读 common.*（代价模型 + 路径后处理）与 footprint.*（车体）参数
  bool loadCommonParams(const ParamReader &params, const std::string &prefix);

  /// 由当前参数/地图重建碰撞与距离场服务（initialize 与 setCostMap 都调）
  void rebuildCollisionServices();

  /// 路径后处理：剪枝 → yaw 填充 → 可选重采样
  void postProcessPath(std::vector<Pose2D> &path, const PlanRequest &req) const;

  /// 诊断：地图里是否存在软代价格（没有就跳过剪枝时的软代价比较，省一遍遍历）
  bool mapHasSoftCells() const { return map_has_soft_cells_; }

  std::string prefix_{"planner"};
  CostModel cost_;
  FootprintParams fp_;
  std::string prune_mode_{"line_of_sight"};
  double prune_max_span_m_{8.0};
  double path_resample_spacing_{0.0};
  bool keep_start_yaw_{true};
  /// ★ **净距偏好**（2026-09-23 用户要求："让局部远离障碍/禁行区"）
  ///   `planner.clearance_prefer_dist`：希望离障碍至少这么远 [m]（0 = 关）；
  ///   `planner.clearance_cost_weight`：低于它时每米额外代价的权重（0 = 关）。
  ///
  /// 为什么不能用"把 map_server.inflate 调大"来达到同样目的：硬膨胀是**约束**，
  ///   会直接把窄通道判死（用户定的规则：机体 0.40 + 左右各 10 cm = 最窄可通
  ///   0.60 m ⇒ 硬判据必须停在 `半宽 + margin + inflate = 0.30 m`）；
  ///   而这里是**偏好**：窄通道仍可走，只是代价高一点 ⇒ 空处自动走中间，
  ///   窄处照样过得去。
  /// 另外一个必要性：全局图（map_server 膨胀后）往往是全 0/100、**没有软代价格**
  ///   ⇒ `common.soft_cost_weight` 实际上是空转的；这里直接用精确距离场算，与
  ///   地图原始值无关，一定生效。
  double clearance_prefer_dist_{0.0};
  double clearance_cost_weight_{0.0};

  std::shared_ptr<const CostMap2D> map_;
  std::unique_ptr<ClearanceField> clearance_;
  FootprintCollisionChecker collision_;
  bool map_has_soft_cells_{false};
};

} // namespace pnc_2d
