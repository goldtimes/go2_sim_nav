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
  /// 真正干活的主体（窗口重试等）；plan() 只负责"起点擦余量时换轮廓重试"
  PlanResult planImpl(const PlanRequest & req);
  float heuristic(int x, int y, int gx, int gy) const;
  std::vector<Pose2D> reconstruct(int goal_id, int start_id) const;
  void ensureBuffers();
  void clearBuffers();

  // 参数
  int connectivity_{8};
  double heuristic_weight_{1.0};
  bool use_search_window_{true};
  /// 起点轮廓已经与障碍重叠时，允许在**起点周边这么远**内找一个“可通行的虚拟起点” [m]
  /// （0 = 关，直接报 START_FOOTPRINT_COLLISION）。
  ///
  /// ★ 为什么不“放宽判据”（试过两版，都被实测否了）：
  ///   ① 放宽**整条路径** ⇒ 起点只要贴到余量（实测差 1 mm），整条路都贴墙走；
  ///   ② 只放宽**起点附近的小圈** ⇒ 根本不行：车体是 0.70×0.40 的矩形，
  ///      侧面对着墙、离墙 0.285 m 时它**无法远离那面墙** —— 侧移不可能，
  ///      而转任何角度都需要车长 0.35~0.40 的净距。而栅格 A* 的节点朝向 = 运动方向，
  ///      所以“圈内放宽”也出不去（实测 NO_PATH）。
  /// ★ 正确分工：**全局只规划“从合法位姿出发”的路，把车从那 10 cm 挪出来的活交给
  ///   局部**（MPC 真的会转方向，能做“侧向挪出去”这种动作）。所以这里找最近一个
  ///   完整判据可通的位姿当起点，并把**车当前位置**插成路径首点。
  double start_escape_radius_{1.0};
  /// 本趟规划中"起点撞到安全余量但没真撞"的说明（成功时附在 message 里）
  std::string relax_note_;
  /// 关掉“虚拟起点”：直接把请求起点当起点（单测用）
  bool virtual_start_enabled_{true};
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
