// 独立感知节点：把 SCAN-Planner 的 GridMap 局部占据栅格从规划器里拆出来单独跑。
//
// 目的：规划算法还没写，先单独验证感知算法是否正常。
// 输入只依赖定位（lightning）输出，不再依赖任何规划器：
//   cloud       <- /lightning/perception/cloud   (PointCloud2, frame_id = map)
//   sensor_pose <- /lightning/perception/pose    (Odometry, map -> lidar_link)
// 这两个是 grid_map.cpp 内部硬编码的相对话题名，由 launch remap 注入，
// 因此本节点和 grid_map.cpp 都不需要改动。
//
// 输出：
//   grid_map/occupancy            占据栅格（膨胀前）
//   grid_map/occupancy_inflate    占据栅格（膨胀后）
//   grid_map/unknown              未知区域
//   grid_map/sliding_map_bbox     滑动地图外框
//   grid_map/depth_cloud          当前帧投影点（lidar
//   路径下即地图范围内的输入点） grid_map/sensor_pose_extrinsic 实际用于
//   raycast 的传感器位姿（外参自检用）

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <plan_env/grid_map.h>

namespace {

/// 与 lightning::LocSystem::LocPhase 一一对应（见 loc_system.h）
enum LocPhase : int32_t {
  kIdle = 0,
  kRunning = 1,
  kSwitching = 2,
  kError = 3,
};

/// 检查配置是否满足"上游只发 map 系点云 + map 系雷达位姿"这一接口契约。
/// 这两个参数配错不会报错，只会静默地把栅格算偏，所以在这里显式告警。
void CheckInterfaceContract(const rclcpp::Node::SharedPtr &node) {
  const bool cloud_is_world =
      node->get_parameter("grid_map.cloud_is_world").as_bool();
  const bool need_extrinsic =
      node->get_parameter("grid_map.need_extrinsic").as_bool();
  const std::string sensor_type =
      node->get_parameter("grid_map.sensor_type").as_string();
  const std::string frame_id =
      node->get_parameter("grid_map.frame_id").as_string();

  if (sensor_type != "lidar") {
    RCLCPP_ERROR(node->get_logger(),
                 "grid_map.sensor_type = '%s'，本节点按 lidar 路径接入（期望 "
                 "'lidar'）。",
                 sensor_type.c_str());
  }
  if (!cloud_is_world) {
    RCLCPP_ERROR(node->get_logger(),
                 "grid_map.cloud_is_world = false，但 "
                 "lightning/perception/cloud 已经是 map 系点云；"
                 "该参数为 false 时点云会被当成 sensor "
                 "系再变换一次，栅格必然错位。请设为 true。");
  }
  if (need_extrinsic) {
    RCLCPP_ERROR(node->get_logger(),
                 "grid_map.need_extrinsic = true，但 lightning/perception/pose "
                 "已经是 map->lidar_link "
                 "（外参已在定位侧折算）；该参数为 true 会在其上再叠一层 "
                 "lidar_extrinsic_。请设为 false。");
  }
  if (frame_id != "map") {
    RCLCPP_WARN(node->get_logger(),
                "grid_map.frame_id = '%s'，建议设为 'map' 以便与 "
                "lightning/global_map 直接叠合。",
                frame_id.c_str());
  }
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("perception_node");

  auto grid_map = std::make_shared<GridMap>();
  grid_map->initMap(node.get()); // 在外部 node 上创建订阅/发布/定时器

  CheckInterfaceContract(node);

  Eigen::Vector3d origin, size;
  grid_map->getRegion(origin, size);
  const double res = grid_map->getResolution();
  const double voxel_num = size.x() * size.y() * size.z() / (res * res * res);
  RCLCPP_INFO(
      node->get_logger(),
      "perception_node started | resolution %.3f | map %.1fx%.1fx%.1f m | "
      "voxels %.0f | map origin (%.2f, %.2f, %.2f)",
      res, size.x(), size.y(), size.z(), voxel_num, origin.x(), origin.y(),
      origin.z());
  RCLCPP_INFO(node->get_logger(),
              "expecting: cloud <- /lightning/perception/cloud (map), "
              "sensor_pose <- /lightning/perception/pose (map -> lidar_link)");

  /// 订阅定位侧的系统阶段（latched，所以即使本节点比定位节点晩启动也能立即拿到当前值）。
  /// 换图后 map 系定义变了，旧地图下累积的体素全部失效，必须清空；
  /// 否则会出现“幽灵障碍”，新建的上层地图上叠着上一张地图的墙。
  auto last_map_state = std::make_shared<int32_t>(-1);
  auto map_state_sub = node->create_subscription<std_msgs::msg::Int32>(
      "lightning/map_state", rclcpp::QoS(1).reliable().transient_local(),
      [&grid_map, &node,
       last_map_state](const std_msgs::msg::Int32::SharedPtr msg) {
        const int32_t state = msg->data;
        if (state == *last_map_state) {
          return; // 去重：resetMap 是 O(体素数) 的，不能重复触发
        }
        *last_map_state = state;

        switch (state) {
        case LocPhase::kSwitching:
          RCLCPP_WARN(
              node->get_logger(),
              "[perception] loc: map switching -> clearing occupancy grid");
          grid_map->resetMap();
          break;
        case LocPhase::kError:
          RCLCPP_ERROR(node->get_logger(),
                       "[perception] loc: no usable map (kError) -> clearing "
                       "occupancy grid");
          grid_map->resetMap();
          break;
        case LocPhase::kRunning:
          RCLCPP_INFO(node->get_logger(),
                      "[perception] loc: map ready (kRunning); waiting for a "
                      "fresh sensor_pose");
          break;
        default:
          break;
        }
      });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
