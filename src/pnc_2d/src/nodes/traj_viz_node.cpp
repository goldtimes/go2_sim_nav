// P6 / M2 阶段性验证节点：**A* 全局路径 → MINCO 轨迹 → RViz 对比**
//
// 为什么单独一个节点，而不是直接插进 global_planner_node：
//   · 阶段性验证要**零风险** —— 不动现网主链路（A* 行为、latched 语义、话题全不变），
//     确认满意之后再按 M4 接进主链路；
//   · 但参数与判据必须与主链路**同源**（`common.*` / `footprint.*` / `traj.*` 用同一批
//     key，地图/距离场/轮廓判定器都按同一套阈值建），否则会出现"我看的图跟你判断的
//     不是一回事"这种跨层不一致 —— 这类问题我们已经栽过好几次。
//
// 用法（在现有仿真栈已经跑起来的前提下另开一个终端）：
//   ros2 run pnc_2d traj_viz_node --ros-args \
//     --params-file $(ros2 pkg prefix pnc_2d)/share/pnc_2d/config/pnc_2d.yaml
// 然后在 RViz 里加两个 Path 显示：
//   · /pnc_2d/global_path   ← A* 原路径（灰/白）
//   · /pnc_2d/minco_traj    ← MINCO 轨迹（绿）
//   状态与指标在 /pnc_2d/minco_status（latched 文本，含净距/终点残差/耗时）
// 触发：每收到一条新的 global_path（latched）就跑一次 MINCO。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "pnc_2d/core/clearance_field.hpp"
#include "pnc_2d/core/cost_map_2d.hpp"
#include "pnc_2d/core/footprint_collision.hpp"
#include "pnc_2d/traj/minco_optimizer.hpp"
#include "ros_param_reader.hpp"

namespace {

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quatOf(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

}  // namespace

class TrajVizNode : public rclcpp::Node {
public:
  TrajVizNode() : rclcpp::Node("traj_viz_node")
  {
    frame_id_ = declare_parameter<std::string>("planner.frame_id", "map");
    topic_map_ = declare_parameter<std::string>("topics.map", "/global_map/occupancy");
    topic_odom_ = declare_parameter<std::string>("topics.odom", "/lightning/perception/pose");
    topic_in_ = declare_parameter<std::string>("traj.viz.input", "pnc_2d/global_path");
    topic_out_ = declare_parameter<std::string>("traj.viz.output", "pnc_2d/minco_traj");
    topic_status_ = declare_parameter<std::string>("traj.viz.status", "pnc_2d/minco_status");

    // ★ 参数必须在这里**一次性声明**：latched 话题（地图/路径）会**重复到达**，
    //   在回调里 declare_parameter 第二次就抛 ParameterAlreadyDeclaredException
    //   —— 实测直接把这个节点打崩。
    hard_ = declare_parameter<int>("common.hard_threshold", 80);
    unk_occ_ = declare_parameter<bool>("common.unknown_as_occupied", false);
    fp_.enable = declare_parameter<bool>("footprint.enable", fp_.enable);
    fp_.length = declare_parameter<double>("footprint.length", fp_.length);
    fp_.width = declare_parameter<double>("footprint.width", fp_.width);
    fp_.offset_x = declare_parameter<double>("footprint.offset_x", fp_.offset_x);
    fp_.offset_y = declare_parameter<double>("footprint.offset_y", fp_.offset_y);
    fp_.safe_margin = declare_parameter<double>("footprint.safe_margin", fp_.safe_margin);

    // 优化器：参数与主链路同源（读同一批 key）
    pnc_2d::RosParamReader reader(*this);
    opt_ = std::make_unique<pnc_2d::MincoOptimizer>();
    if (!opt_->configure(reader)) {
      RCLCPP_ERROR(get_logger(), "MINCO 配置失败：检查 traj.* 参数（限值必须 > 0）");
    }

    pub_traj_ = create_publisher<nav_msgs::msg::Path>(topic_out_,
                                                      rclcpp::QoS(1).transient_local());
    pub_status_ = create_publisher<std_msgs::msg::String>(topic_status_,
                                                          rclcpp::QoS(1).transient_local());
    pub_in_ = create_publisher<nav_msgs::msg::Path>(topic_in_copy_(),
                                                    rclcpp::QoS(1).transient_local());

    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_map_, rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onMap(msg); });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });
    sub_path_ = create_subscription<nav_msgs::msg::Path>(
        topic_in_, rclcpp::QoS(1).transient_local(),
        [this](nav_msgs::msg::Path::SharedPtr msg) { onPath(msg); });

    RCLCPP_INFO(get_logger(),
                "就绪：%s（A*）→ %s（MINCO），状态见 %s；RViz 里同时显示两条 Path 对比",
                topic_in_.c_str(), topic_out_.c_str(), topic_status_.c_str());
  }

private:
  std::string topic_in_copy_() const { return topic_in_ + "_input"; }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) return;
    const double oyaw = yawOf(msg->info.origin.orientation);
    // 重复到达同样的图（latched 补发）就不要再建一遍：重建要几十 ms
    if (map_ && map_->width() == static_cast<int>(msg->info.width) &&
        map_->height() == static_cast<int>(msg->info.height) &&
        std::fabs(map_->resolution() - msg->info.resolution) < 1e-12 &&
        map_->data() == msg->data) {
      return;
    }
    map_ = std::make_shared<pnc_2d::CostMap2D>();
    if (!map_->set(static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
                   msg->info.resolution, msg->info.origin.position.x,
                   msg->info.origin.position.y, oyaw, msg->data, msg->header.frame_id)) {
      RCLCPP_ERROR(get_logger(), "地图字段不自洽（尺寸/分辨率/数据长度）");
      map_.reset();
      return;
    }
    cf_ = std::make_unique<pnc_2d::ClearanceField>();
    if (!cf_->build(*map_, hard_, unk_occ_)) {
      RCLCPP_ERROR(get_logger(), "距离场构建失败");
      cf_.reset();
      return;
    }
    ck_ = std::make_unique<pnc_2d::FootprintCollisionChecker>();
    ck_->configure(fp_, hard_, unk_occ_);
    ck_->setMap(map_.get());
    ck_->setClearanceField(cf_.get());

    opt_->setMap(map_);
    opt_->setClearanceField(cf_.get());
    opt_->setCollisionChecker(ck_.get());
    opt_->reset();

    RCLCPP_INFO(get_logger(), "地图 %dx%d @%.3f m，轮廓 %.2fx%.2f + %.2f，距离场致命格 %zu",
                map_->width(), map_->height(), map_->resolution(), fp_.length, fp_.width,
                fp_.safe_margin, cf_->lethalCount());
    if (pending_path_) {   // 地图晚于路径到达：补跑一次
      const auto p = pending_path_;
      pending_path_.reset();
      onPath(p);
    }
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.yaw = yawOf(msg->pose.pose.orientation);
    pose_.has_yaw = true;
    start_v_ = std::max(0.0, msg->twist.twist.linear.x);
    start_omega_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    using namespace std::chrono;
    if (!map_ || !cf_ || !ck_) {   // 地图还没到：记下来，等地图到了补跑
      pending_path_ = msg;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "等地图（%s）…", topic_map_.c_str());
      return;
    }
    if (msg->poses.size() < 2) {
      // 上游发**空 Path** = 清场（规划失败 / `~/clear_path` / 启动清场）。
      // ★ 必须把自己的输出也清掉，否则 RViz 里会留下**上一轮的绿线**
      //   （latched 话题的经典坑：发布者没了 RViz 也不会自己清）。
      nav_msgs::msg::Path empty;
      empty.header.frame_id = frame_id_;
      empty.header.stamp = now();
      pub_traj_->publish(empty);
      pub_in_->publish(empty);
      std_msgs::msg::String s;
      s.data = "[MINCO] 上游路径为空（清场）⇒ 已清空 minco_traj";
      pub_status_->publish(s);
      RCLCPP_INFO(get_logger(), "%s", s.data.c_str());
      return;
    }

    pnc_2d::TrajOptRequest req;
    req.path.reserve(msg->poses.size());
    for (const auto & ps : msg->poses) {
      pnc_2d::Pose2D p;
      p.x = ps.pose.position.x;
      p.y = ps.pose.position.y;
      p.yaw = yawOf(ps.pose.orientation);
      p.has_yaw = true;
      req.path.push_back(p);
    }
    // 起点用**车当前位姿**（比路径首点新）；没有 odom 就退回路径首点
    req.start = has_odom_ ? pose_ : req.path.front();
    req.start_v = has_odom_ ? start_v_ : 0.0;
    req.start_omega = has_odom_ ? start_omega_ : 0.0;
    req.goal = req.path.back();
    req.use_goal_yaw = req.goal.has_yaw;

    const auto t0 = steady_clock::now();
    const pnc_2d::TrajOptResult r = opt_->optimize(req);
    const double wall_ms =
        duration<double, std::milli>(steady_clock::now() - t0).count();

    // 原路径转发一份（带 _input 后缀），方便 RViz 里成对显示与对比
    {
      nav_msgs::msg::Path out = *msg;
      pub_in_->publish(out);
    }

    if (r.samples.size() < 2) {
      // 没东西可画（无输入/求解失败）：只发状态，不改语义
      std_msgs::msg::String s;
      s.data = std::string("[MINCO] ") + pnc_2d::toString(r.status) + "：" + r.message;
      pub_status_->publish(s);
      RCLCPP_WARN(get_logger(), "%s", s.data.c_str());
      return;
    }
    if (r.status != pnc_2d::TrajStatus::kSuccess) {
      // ★ 终检不过（kCheckFailed）等情况：**仍然把轨迹发出来** —— 这是可视化节点，
      //   用户必须看到"失败在哪、长什么样"才能改。但状态里点名 + 明说不得执行。
      //   （主链路走的是 srv/action，不消费这个话题；失败语义由 `status` 决定。）
      RCLCPP_WARN(get_logger(),
                  "[MINCO] %s —— 轨迹不可执行，仅用于诊断（下方 minco_traj 仍画出）",
                  pnc_2d::toString(r.status));
    }

    nav_msgs::msg::Path out;
    out.header.frame_id = frame_id_;
    out.header.stamp = now();
    out.poses.reserve(r.samples.size());
    for (const pnc_2d::TrajSample & sm : r.samples) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.pose.position.x = sm.x;
      ps.pose.position.y = sm.y;
      ps.pose.position.z = 0.0;
      ps.pose.orientation = quatOf(sm.yaw);
      out.poses.push_back(ps);
    }
    pub_traj_->publish(out);

    const pnc_2d::TrajOptStats & st = r.stats;
    char buf[1024];
    std::snprintf(buf, sizeof(buf),
                  "[MINCO] %s | %zu 点 / 时长 %.2f s / 路径 %.2f m\n"
                  "轮廓净距(排除首末) %s%.3f m @s=%.2f | 输入路径净距见 /pnc_2d/global_path\n"
                  "max|v| %.3f m/s | max|ω| %.3f rad/s | max|a| %.3f | max|α| %.3f | max|κ| %.3f\n"
                  "终点残差 %.4f m / 末朝向误差 %.2f° | 时间缩放 ×%.2f | 平滑偏移 max %.3f m\n"
                  "终检 %s：硬门 %d 位姿（统计 %d 位姿）/ %d 个不过 / 最差净距 %.4f m @s=%.2f m\n"
                  "运动学终验 %s\n"
                  "求解 %.1f ms（wall %.1f ms）\n%s",
                  pnc_2d::toString(r.status), r.samples.size(), st.duration, st.path_length,
                  st.clearance_valid ? "" : "(未测) ", st.min_clearance, st.min_clearance_s,
                  st.max_v, st.max_omega, st.max_a, st.max_alpha, st.max_curvature,
                  st.terminal_error, st.terminal_error_yaw * 180.0 / M_PI, st.time_scale,
                  st.path_shift_max, st.check_ok ? "通过" : "**不过**", st.check_points,
                  st.check_stats_points, st.check_violations, st.check_worst_clearance,
                  st.check_worst_s, st.kin_ok ? "通过" : "**不过（缩放未收敛）**",
                  st.solve_ms, wall_ms, st.note.c_str());
    std_msgs::msg::String s;
    s.data = buf;
    pub_status_->publish(s);
    if (r.status == pnc_2d::TrajStatus::kSuccess) {
      RCLCPP_INFO(get_logger(), "\n%s", buf);
    } else {
      RCLCPP_WARN(get_logger(), "\n%s\n%s", buf, st.check_note.c_str());
    }
  }

  std::string frame_id_;
  std::string topic_map_;
  std::string topic_odom_;
  std::string topic_in_;
  std::string topic_out_;
  std::string topic_status_;

  std::unique_ptr<pnc_2d::MincoOptimizer> opt_;
  std::shared_ptr<pnc_2d::CostMap2D> map_;
  std::unique_ptr<pnc_2d::ClearanceField> cf_;
  std::unique_ptr<pnc_2d::FootprintCollisionChecker> ck_;
  // 构造函数里声明一次（回调里 declare 会被 latched 重复到达打崩）
  int hard_{80};
  bool unk_occ_{false};
  pnc_2d::FootprintParams fp_{};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_in_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;

  pnc_2d::Pose2D pose_{};
  double start_v_{0.0};
  double start_omega_{0.0};
  bool has_odom_{false};
  nav_msgs::msg::Path::SharedPtr pending_path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajVizNode>());
  rclcpp::shutdown();
  return 0;
}
