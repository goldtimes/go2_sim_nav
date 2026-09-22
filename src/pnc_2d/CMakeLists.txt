cmake_minimum_required(VERSION 3.8)
project(pnc_2d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------- 消息/服务
# 只有一个状态消息（失败上报）和一个切换服务（运行时热切换算法）。
# 库仍然是 ROS-free：这些接口只被 nodes/ 里的薄壳使用。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerStatus.msg"
  "srv/SwitchPlanner.srv"
  DEPENDENCIES std_msgs
)
# 路网 yaml 解析（P2）。库仍然 ROS-free：只多了个系统依赖。
# ⚠ 系统 yaml-cpp 0.7 导出的 target 名不确定（yaml-cpp / yaml-cpp::yaml-cpp），
#   map_server 已踩过：用 ${YAML_CPP_LIBRARIES} 最稳，且不要与
#   ament_target_dependencies 的 plain 签名混用（同 target 混用会报错）。
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------- 规划库
# 目录分层（与 include/pnc_2d/ 一一对应，见 doc/pnc2d_restructure_plan.md §3）：
#   core/   公共层：类型、地图、footprint、参数接口、三类抽象基类
#   global/ 全局规划实现
#   local/  局部规划实现（本期只有 Null，见 P2）
#   sm/     状态机（纯库，见 P3）
# 刻意做成 ROS-free：不调用 ament_target_dependencies，库只依赖标准库。
# 好处：① 单测不需要 ROS（毫秒级、可复现）；② 将来可放进 MPC 节点内进程调用。
add_library(pnc_2d_planner
  src/core/types.cpp
  src/core/cost_map_2d.cpp
  src/core/clearance_field.cpp
  src/core/footprint_collision.cpp
  src/core/route_graph.cpp
  src/core/map_zones.cpp
  src/core/global_planner.cpp
  src/core/factory.cpp
  src/global/astar_planner.cpp
  src/global/route_network_planner.cpp
)
target_include_directories(pnc_2d_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(pnc_2d_planner PUBLIC ${YAML_CPP_LIBRARIES})

# ---------------------------------------------------------------- 规划节点
# nodes/ = ROS 薄壳：只做 IO / 参数 / frame 校验，不写算法
add_executable(global_planner_node src/nodes/global_planner_node.cpp)
target_include_directories(global_planner_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes)
target_link_libraries(global_planner_node pnc_2d_planner)
ament_target_dependencies(global_planner_node
  rclcpp std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
)
# 生成的消息头（pnc_2d/msg/planner_status.hpp）在 CMake 构建目录下，
# 必须把本包自己的 typesupport 目标链上，否则找不到头文件。
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(global_planner_node "${cpp_typesupport_target}")

install(TARGETS pnc_2d_planner
  EXPORT export_pnc_2d_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS global_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# 对外导出：map_server 要复用 RouteGraph（路网资产由 map_server 管、规划器消费）
ament_export_targets(export_pnc_2d_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs yaml-cpp)
ament_export_dependencies(rosidl_default_runtime)

# ---------------------------------------------------------------- 测试
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_planner test/test_astar_planner.cpp)
  target_link_libraries(test_astar_planner pnc_2d_planner)
  ament_add_gtest(test_route_network test/test_route_network.cpp)
  target_link_libraries(test_route_network pnc_2d_planner)
  ament_add_gtest(test_map_zones test/test_map_zones.cpp)
  target_link_libraries(test_map_zones pnc_2d_planner)
endif()

ament_package()
