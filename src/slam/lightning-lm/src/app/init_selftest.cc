//
// Created by xiang on 25-9-12.
// 初始化定位自测节点（交互模式）：
//   - 加载整张 pcd 地图并发布（transient_local，rviz 晚订阅也能拿到全图）
//   - 订阅 /initialpose（rviz 2D Pose Estimate 点击）触发一次判定
//   - 订阅 /lightning/loc_state 与 /lightning/nav_state 判定初始化是否成功
//   - 结果输出到终端 + data/init_selftest_result.csv
// 用法：
//   ros2 run lightning init_selftest --ros-args -p map_pcd:=/path/global.pcd
//

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include "lightning/msg/nav_state.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "common/log.h"

namespace {
// 与 localization_result.h 中 LocalizationStatus 保持一致
constexpr int kLocIdle = 0;
constexpr int kLocInitializing = 1;
constexpr int kLocGood = 2;
constexpr int kLocFollowingDr = 3;
constexpr int kLocFail = 4;

const char* LocStateStr(int s) {
    switch (s) {
        case kLocIdle:
            return "IDLE";
        case kLocInitializing:
            return "INITIALIZING";
        case kLocGood:
            return "GOOD";
        case kLocFollowingDr:
            return "FOLLOWING_DR";
        case kLocFail:
            return "FAIL";
        default:
            return "UNKNOWN";
    }
}
}  // namespace

class InitSelftest : public rclcpp::Node {
   public:
    InitSelftest() : Node("init_selftest") {
        map_pcd_ = declare_parameter<std::string>("map_pcd", "");
        timeout_s_ = declare_parameter<double>("timeout", 5.0);
        confirm_frames_ = declare_parameter<int>("confirm_frames", 3);
        csv_path_ = declare_parameter<std::string>("csv", "./data/init_selftest_result.csv");

        const std::string state_topic = declare_parameter<std::string>("loc_state_topic", "/lightning/loc_state");
        const std::string nav_topic = declare_parameter<std::string>("nav_state_topic", "/lightning/nav_state");
        const std::string init_topic = declare_parameter<std::string>("initial_pose_topic", "/initialpose");

        /// 整图发布（transient_local：晚订阅的 rviz 也能拿到）
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("init_selftest/map",
                                                                   rclcpp::QoS(1).transient_local().reliable());
        PublishMap();

        /// scan 转发（lightning 定位发的 /lightning/current_scan，frame=map），便于 rviz 叠加核对对齐
        scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("init_selftest/scan",
                                                                    rclcpp::QoS(1).transient_local().reliable());
        const std::string scan_topic = declare_parameter<std::string>("current_scan_topic", "/lightning/current_scan");
        scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            scan_topic, 10, [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { OnScan(msg); });

        loc_state_sub_ = create_subscription<std_msgs::msg::Int32>(
            state_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr msg) { OnLocState(msg); });
        nav_state_sub_ = create_subscription<lightning::msg::NavState>(
            nav_topic, 10, [this](const lightning::msg::NavState::SharedPtr msg) { OnNavState(msg); });
        init_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            init_topic, 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { OnInitPose(msg); });

        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { Tick(); });

        LLOG_INFO(lightning::logging::kSelfTest, "init_selftest ready: map_pcd={}, timeout={:.1f}s, confirm_frames={}", map_pcd_,
                  timeout_s_, confirm_frames_);
        LLOG_INFO(lightning::logging::kSelfTest, "在 rviz 中点 '2D Pose Estimate' 下发初值，将自动判定初始化是否成功。");
    }

   private:
    // —— 地图整图加载与发布 ——
    void PublishMap() {
        if (map_pcd_.empty()) {
            LLOG_INFO(lightning::logging::kSelfTest, "map_pcd 为空，整图请订阅 lightning 的 /lightning/global_map");
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile(map_pcd_, *cloud) != 0 || cloud->empty()) {
            LLOG_WARN(lightning::logging::kSelfTest, "加载地图 pcd 失败: {}", map_pcd_);
            return;
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = now();
        map_pub_->publish(msg);

        // 留存整图并建 kd-tree，供 PASS 时做 scan-map 对齐度检查
        map_cloud_ = cloud;
        map_kdtree_.setInputCloud(map_cloud_);
        map_tree_ready_ = true;
        LLOG_INFO(lightning::logging::kSelfTest, "整图 pcd 已发布: {} ({} 点)，已建最近邻索引", map_pcd_, cloud->size());
    }

    // —— 当前扫描转发 + 缓存 ——
    void OnScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        last_scan_ = *msg;
        have_scan_ = true;
        scan_pub_->publish(*msg);
    }

    // —— rviz 点击：开始一次判定 ——
    void OnInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        const auto& p = msg->pose.pose;
        const double yaw =
            std::atan2(2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                       1.0 - 2.0 * (p.orientation.y * p.orientation.y + p.orientation.z * p.orientation.z));
        monitoring_ = true;
        t0_ = now();
        good_count_ = 0;
        cur_state_ = kLocInitializing;
        last_pose_known_ = false;

        LLOG_INFO(lightning::logging::kSelfTest,
                  "========================================\n"
                  "[SELFTEST] 收到初始位姿点击: x={:.2f} y={:.2f} yaw={:.1f}° (frame={})，开始判定...",
                  p.position.x, p.position.y, yaw * 180.0 / M_PI, msg->header.frame_id);
    }

    // —— 定位状态 ——
    void OnLocState(const std_msgs::msg::Int32::SharedPtr msg) {
        cur_state_ = msg->data;
        if (!monitoring_) {
            return;
        }

        if (cur_state_ == kLocGood) {
            ++good_count_;
            LLOG_INFO(lightning::logging::kSelfTest, "[SELFTEST] loc_state=GOOD ({}/{})", good_count_, confirm_frames_);
            if (good_count_ >= confirm_frames_) {
                Finish(true, "loc_state=GOOD 连续 " + std::to_string(confirm_frames_) + " 帧");
            }
        } else if (cur_state_ == kLocFail) {
            Finish(false, "loc_state=FAIL");
        } else {
            good_count_ = 0;  // 回到 INITIALIZING 等：仍在搜索/未锁上
        }
    }

    // —— 缓存最近一次有效导航状态（用于 PASS 时报位姿/置信度）——
    void OnNavState(const lightning::msg::NavState::SharedPtr msg) {
        if (!msg->pose_is_ok) {
            return;
        }
        last_pose_ = msg->pose;
        last_confidence_ = msg->confidence;
        last_pose_known_ = true;
    }

    void Tick() {
        if (!monitoring_) {
            return;
        }
        const double dt = (now() - t0_).seconds();
        if (dt > timeout_s_) {
            std::ostringstream oss;
            oss << "timeout(> " << timeout_s_ << "s)，loc_state 停在 " << LocStateStr(cur_state_);
            Finish(false, oss.str());
        }
    }

    void Finish(bool ok, const std::string& reason) {
        if (!monitoring_) {
            return;
        }
        monitoring_ = false;
        const double dt = (now() - t0_).seconds();

        double x = 0, y = 0, z = 0, yaw = 0, conf = 0;
        bool have_pose = last_pose_known_;
        if (have_pose) {
            x = last_pose_.position.x;
            y = last_pose_.position.y;
            z = last_pose_.position.z;
            const auto& q = last_pose_.orientation;
            yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            conf = last_confidence_;
        }

        const std::string tag = ok ? "PASS" : "FAIL";
        LLOG_INFO(lightning::logging::kSelfTest,
                  "\n########################################\n"
                  "[SELFTEST] 结果: {}  原因: {}  耗时: {:.2f}s\n"
                  "           定位位姿(map/base_link): x={:.2f} y={:.2f} z={:.2f} yaw={:.1f}°  conf={:.4f}\n"
                  "########################################",
                  tag, reason, dt, x, y, z, have_pose ? yaw * 180.0 / M_PI : 0.0, conf);

        // PASS 时额外做 scan-map 对齐检查（视觉/数值双重确认）
        if (ok) {
            CheckScanMapAlignment();
        }

        if (!csv_path_.empty()) {
            WriteCsv(tag, reason, dt, have_pose, x, y, z, yaw, conf);
        }
    }

    /// 最近邻统计：估计当前 scan(map 系) 与整图的重合度
    void CheckScanMapAlignment() {
        if (!map_tree_ready_) {
            LLOG_INFO(lightning::logging::kSelfTest,
                      "[SELFTEST] 未加载整图(map_pcd 为空)，跳过数值对齐检查（请用 rviz 叠加 /lightning/current_scan "
                      "目视核对）");
            return;
        }
        if (!have_scan_) {
            LLOG_WARN(lightning::logging::kSelfTest, "[SELFTEST] 还没收到当前扫描，跳过对齐检查");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(last_scan_, *scan);
        if (scan->empty()) {
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(0.2f, 0.2f, 0.2f);
        vg.setInputCloud(scan);
        vg.filter(*ds);

        const double th = 0.5;  // 近邻距离阈值(m)
        std::vector<int> idx(1);
        std::vector<float> sq(1);
        int matched = 0;
        for (const auto& p : ds->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                continue;
            }
            if (map_kdtree_.nearestKSearch(p, 1, idx, sq) > 0 && sq[0] <= th * th) {
                ++matched;
            }
        }
        const double ratio = ds->empty() ? 0.0 : double(matched) / double(ds->size());
        LLOG_INFO(lightning::logging::kSelfTest, "[SELFTEST] scan-map 对齐检查: {}/{} 点(0.2m降采样) 落在整图 {:.1f}m 邻域内 = {:.1f}%",
                  matched, ds->size(), th, ratio * 100.0);
    }

    void WriteCsv(const std::string& tag, const std::string& reason, double dt, bool have_pose, double x, double y,
                  double z, double yaw, double conf) {
        try {
            const std::filesystem::path p(csv_path_);
            if (p.has_parent_path()) {
                std::filesystem::create_directories(p.parent_path());
            }
            const bool need_header = !std::filesystem::exists(p) || std::filesystem::file_size(p) == 0;
            std::ofstream fout(csv_path_, std::ios::app);
            if (!fout) {
                return;
            }
            if (need_header) {
                fout << "time,result,reason,dt_s,pose_known,x,y,z,yaw_deg,conf\n";
            }
            const auto t = now();
            fout << std::fixed << std::setprecision(3) << t.seconds() << "," << tag << "," << reason << "," << dt << ","
                 << (have_pose ? 1 : 0) << "," << x << "," << y << "," << z << "," << yaw * 180.0 / M_PI << "," << conf
                 << "\n";
        } catch (const std::exception& e) {
            LLOG_WARN(lightning::logging::kSelfTest, "写 CSV 失败: {}", e.what());
        }
    }

    std::string map_pcd_;
    double timeout_s_ = 5.0;
    int confirm_frames_ = 3;
    std::string csv_path_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr loc_state_sub_;
    rclcpp::Subscription<lightning::msg::NavState>::SharedPtr nav_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool have_scan_ = false;
    sensor_msgs::msg::PointCloud2 last_scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree_;
    bool map_tree_ready_ = false;

    bool monitoring_ = false;
    rclcpp::Time t0_;
    int good_count_ = 0;
    int cur_state_ = 0;

    bool last_pose_known_ = false;
    geometry_msgs::msg::Pose last_pose_;
    double last_confidence_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    lightning::logging::Init();  // 无 config，使用默认（stderr + ./log 落盘）
    auto node = std::make_shared<InitSelftest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    lightning::logging::Shutdown();
    return 0;
}
