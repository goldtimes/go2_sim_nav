//
// Created by xiang on 25-3-18.
//

#include <rclcpp/rclcpp.hpp>

#include "common/log.h"
#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/ros_utils.h"

/// 运行定位的测试
int main(int argc, char** argv) {
    using namespace lightning;

    rclcpp::init(argc, argv);

    // config 文件路径通过 ROS2 参数传入（launch 中 Node parameters 注入）
    // 节点名需与 launch 中 Node 的 name 一致，参数文件才能匹配到本节点
    auto param_node = std::make_shared<rclcpp::Node>("run_loc_online");
    param_node->declare_parameter<std::string>("config", "./config/default.yaml");
    const std::string config_path = param_node->get_parameter("config").as_string();
    logging::InitFromYaml(config_path);
    LLOG_INFO(logging::kApp, "config file (from ros2 parameter): {}", config_path);

    // 可选：由上层指定地图目录。为空则用 config 里的 system.map_path。
    // 节点名与 launch 中 Node 的 name 一致，launch 的 map_path:=<dir> 才能匹配到。
    param_node->declare_parameter<std::string>("map_path", "");
    const std::string map_path = param_node->get_parameter("map_path").as_string();
    if (!map_path.empty()) {
        LLOG_INFO(logging::kApp, "map path (from ros2 parameter): {}", map_path);
    }

    // 可选：手动指定初始位姿（不指定则用地图功能点 FP 自动初始化）
    // 注意：错误初值（如 identity）在多层环境中会锁错层，默认不传初值更稳。
    param_node->declare_parameter<double>("init_x", 0.0);
    param_node->declare_parameter<double>("init_y", 0.0);
    param_node->declare_parameter<double>("init_z", 0.0);
    param_node->declare_parameter<double>("init_yaw", 0.0);
    param_node->declare_parameter<bool>("use_init_pose", false);
    const bool use_init_pose = param_node->get_parameter("use_init_pose").as_bool();
    param_node.reset();  // 读取完成即释放，loc 内部会自建节点

    LocSystem::Options opt;
    LocSystem loc(opt);

    if (!loc.Init(config_path, map_path)) {
        LLOG_ERROR(logging::kApp, "failed to init loc");
        // 不能继续：StartLoc()/Spin() 会在未初始化的 loc 上工作，
        // 且退出时 ~LocSystem() 会对空/半初始化的 loc 调 Finish()。
        logging::Shutdown();
        rclcpp::shutdown();
        return 1;
    }

    if (use_init_pose) {
        // 手动给定初始位姿（yaw 绕世界 z 轴）
        auto param_node2 = std::make_shared<rclcpp::Node>("run_loc_online");
        double ix = param_node2->get_parameter("init_x").as_double();
        double iy = param_node2->get_parameter("init_y").as_double();
        double iz = param_node2->get_parameter("init_z").as_double();
        double iyaw = param_node2->get_parameter("init_yaw").as_double();
        param_node2.reset();

        const Eigen::AngleAxisd Rz(iyaw, Eigen::Vector3d::UnitZ());
        loc.SetInitPose(SE3(SO3(Rz.matrix()), Vec3d(ix, iy, iz)));
    } else {
        // 由地图功能点自动初始化（推荐，避免 identity 初值锁错层）
        loc.StartLoc();
    }

    loc.Spin();

    /// 各阶段耗时汇总（Proc Lidar / Preprocess 等），与 run_slam_online 保持一致的可观测性
    Timer::PrintAll();

    rclcpp::shutdown();
    logging::Shutdown();

    return 0;
}