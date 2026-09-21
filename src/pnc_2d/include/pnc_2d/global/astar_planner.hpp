// A* 全局规划：八邻域栅格搜索 + 矩形 footprint 碰撞 + 搜索窗口逐级扩大重试。
//
// 与参考工程（SLAM-PNC 的 A*）的关键差别：
//   · 有矩形车体轮廓检查（它只查中心点，声明了轮廓却未接入）；
//   · 斜移要求两个正交邻格空闲（防"对角缝隙钻过去"）；
//   · 窗口失败后按余量序列逐级扩大重试（它用固定 margin 重试）；
//   · 失败返回明确状态码，不返回空路径充数。

#pragma once

#include <chrono>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

#include "pnc_2d/core/global_planner.hpp"

namespace pnc_2d {

class AStarPlanner : public GlobalPlanner {
public:
  std::string type() const override { return "astar"; }
  bool configure(const ParamReader & params) override;
  PlanResult plan(const PlanRequest & req) override;
  void reset() override;

  // ---- 诊断（单测/日志用）----
  bool useSearchWindow() const { return use_search_window_; }
  const std::vector<double> & retryMargins() const { return retry_margins_; }
  int connectivity() const { return connectivity_; }

private:
  /// 单次（单个窗口）搜索
  PlanResult searchInWindow(const PlanRequest & req, const SearchWindow & win,
                            std::chrono::steady_clock::time_point t0);
  float heuristic(int x, int y, int gx, int gy) const;
  std::vector<Pose2D> reconstruct(int goal_id, int start_id) const;
  void ensureBuffers();
  void clearBuffers();

  // 参数
  int connectivity_{8};
  double heuristic_weight_{1.0};
  bool use_search_window_{true};
  double search_window_margin_{10.0};
  std::vector<double> retry_margins_{30.0, 100.0, 1.0e9};
  long max_iterations_{300000};
  double max_search_time_ms_{2000.0};

  // 搜索缓冲（按地图尺寸分配一次，多次搜索复用）
  int buf_w_{0};
  int buf_h_{0};
  std::vector<uint8_t> closed_;
  std::vector<float> g_;
  std::vector<int32_t> parent_;
};

}  // namespace pnc_2d
