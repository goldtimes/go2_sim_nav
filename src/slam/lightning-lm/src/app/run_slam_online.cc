//
// Created by xiang on 25-3-18.
//

#include <rclcpp/rclcpp.hpp>

#include "common/log.h"
#include "core/system/slam.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

/// 运行一个LIO前端，带可视化
/// NOTE 退出时不自动保存：地图/关键帧/轨迹统一通过 /lightning/save_map 与 /lightning/save_path 服务保存
int main(int argc, char** argv) {
    using namespace lightning;

    /// 需要rclcpp::init
    rclcpp::init(argc, argv);

    // config 文件路径通过 ROS2 参数传入（launch 中 Node parameters 注入）
    // 节点名需与 launch 中 Node 的 name 一致，参数文件才能匹配到本节点
    auto param_node = std::make_shared<rclcpp::Node>("run_slam_online");
    param_node->declare_parameter<std::string>("config", "./config/default.yaml");
    const std::string config_path = param_node->get_parameter("config").as_string();
    logging::InitFromYaml(config_path);
    LLOG_INFO(logging::kApp, "config file (from ros2 parameter): {}", config_path);

    // 可选：建图保存根目录。为空则用 config 里的 system.map_root（缺省 $HOME/rcs/maps）。
    param_node->declare_parameter<std::string>("map_root", "");
    const std::string map_root = param_node->get_parameter("map_root").as_string();
    if (!map_root.empty()) {
        LLOG_INFO(logging::kApp, "map save root (from ros2 parameter): {}", map_root);
    }
    param_node.reset();  // 读取完成即释放，slam 内部会自建节点

    SlamSystem::Options options;
    options.online_mode_ = true;

    SlamSystem slam(options);
    if (!slam.Init(config_path, map_root)) {
        LLOG_ERROR(logging::kApp, "failed to init slam");
        logging::Shutdown();
        return -1;
    }

    slam.StartSLAM("new_map");
    slam.Spin();

    Timer::PrintAll();

    rclcpp::shutdown();

    LLOG_INFO(logging::kApp, "done");
    logging::Shutdown();

    return 0;
}