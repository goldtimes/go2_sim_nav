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
find_package(rclcpp_action REQUIRED)   # P4：manager → local 用 action
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)   # P5.3：订阅 ESDF 点云
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# P5：MPC 求解器。用 vendor 包：它会导出名为 osqp 的 imported target
# （实测 OSQP 0.6.2 C API，头文件按 <osqp.h> 包含，链接 -losqp）。
find_package(osqp_vendor REQUIRED)

# ---------------------------------------------------------------- 消息/服务/动作
# 库仍然是 ROS-free：这些接口只被 nodes/ 里的薄壳使用。
#   PlannerStatus : 全局规划状态（latched，失败上报）
#   LocalStatus   : 局部规划状态（latched，P4）
#   ManagerState  : 状态机状态（latched，P4）
#   SwitchPlanner : 运行时热切换算法（全局/局部公用）
#   PlanPath      : manager → global（同步服务，P4）
#   FollowPath    : manager → local（action：有反馈 + 可取消，P4）
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerStatus.msg"
  "msg/LocalStatus.msg"
  "msg/ManagerState.msg"
  "msg/Zone.msg"
  "msg/ZoneArray.msg"
  "srv/SwitchPlanner.srv"
  "srv/PlanPath.srv"
  "action/FollowPath.action"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
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
  src/core/local_distance_field.cpp
  src/core/global_planner.cpp
  src/core/replan_recovery.cpp
  src/core/factory.cpp
  src/global/astar_planner.cpp
  src/global/route_network_planner.cpp
  src/local/null_local_planner.cpp
  src/local/mpc_local_planner.cpp
  src/sm/manager_sm.cpp
)
target_include_directories(pnc_2d_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(pnc_2d_planner PUBLIC ${YAML_CPP_LIBRARIES})
# MPC 用 OSQP C API。**只信 find_path/find_library 的结果**，不依赖 vendor 包
# 是否导出了 imported target 或某个具体变量名 —— vendor（ros-humble-osqp-vendor）
# 用 ament_export_libraries 导出，target/变量/目录名在不同发行版里都可能不一样。
# 实测坑：只写 `if(TARGET osqp) link osqp` 会漏掉 include 目录（osqp.h 找不到），
# 而只用 ${ROS_DISTRO} 拼路径在 CMake 里可能是空的。
find_path(OSQP_INCLUDE_DIR osqp.h
  HINTS ${osqp_vendor_INCLUDE_DIRS}
  PATHS /opt/ros/humble/include/osqp /opt/ros/rolling/include/osqp /usr/include/osqp)
find_library(OSQP_LIBRARY NAMES osqp
  HINTS ${osqp_vendor_LIBRARY_DIRS}
  PATHS /opt/ros/humble/lib /opt/ros/rolling/lib /usr/lib /usr/local/lib)
if(NOT OSQP_INCLUDE_DIR OR NOT OSQP_LIBRARY)
  message(FATAL_ERROR
    "找不到 OSQP（osqp.h / libosqp）：请先 sudo apt install ros-humble-osqp-vendor")
endif()
message(STATUS "OSQP: ${OSQP_LIBRARY} | include ${OSQP_INCLUDE_DIR}")
target_include_directories(pnc_2d_planner PUBLIC ${OSQP_INCLUDE_DIR})
target_link_libraries(pnc_2d_planner PUBLIC ${OSQP_LIBRARY})

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

# 局部规划节点（P4 只跑 Null 实现，不产生速度指令）
add_executable(local_planner_node src/nodes/local_planner_node.cpp)
target_include_directories(local_planner_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes)
target_link_libraries(local_planner_node pnc_2d_planner "${cpp_typesupport_target}")
ament_target_dependencies(local_planner_node
  rclcpp rclcpp_action std_msgs std_srvs nav_msgs geometry_msgs sensor_msgs
)

# 任务状态机节点（三节点的主控，P4）
add_executable(pnc_manager_node src/nodes/pnc_manager_node.cpp)
target_include_directories(pnc_manager_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes)
target_link_libraries(pnc_manager_node pnc_2d_planner "${cpp_typesupport_target}")
ament_target_dependencies(pnc_manager_node
  rclcpp rclcpp_action std_msgs std_srvs nav_msgs geometry_msgs
)

install(TARGETS pnc_2d_planner
  EXPORT export_pnc_2d_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS global_planner_node local_planner_node pnc_manager_node
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
  ament_add_gtest(test_clearance_field test/test_clearance_field.cpp)
  target_link_libraries(test_clearance_field pnc_2d_planner)
  ament_add_gtest(test_factory test/test_factory.cpp)
  target_link_libraries(test_factory pnc_2d_planner)
  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_link_libraries(test_state_machine pnc_2d_planner)
  ament_add_gtest(test_distance_field test/test_distance_field.cpp)
  target_link_libraries(test_distance_field pnc_2d_planner)
  ament_add_gtest(test_mpc_local_planner test/test_mpc_local_planner.cpp)
  target_link_libraries(test_mpc_local_planner pnc_2d_planner)
  ament_add_gtest(test_mpc_qp_reference test/test_mpc_qp_reference.cpp)
  target_link_libraries(test_mpc_qp_reference pnc_2d_planner)
endif()

ament_package()
