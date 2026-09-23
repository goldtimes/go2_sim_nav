cmake_minimum_required(VERSION 3.8)
project(map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# 路网：复用 pnc_2d 的 RouteGraph（ROS-free 数据结构 + yaml 读写）
# 依赖方向：map_server（资产）→ pnc_2d（库）；只用到数据层，不用规划器
find_package(pnc_2d REQUIRED)

# ---- 地图加载库（yaml + pgm → OccupancyGrid 语义；不依赖 rclcpp，可单独复用）----
add_library(map_io src/map_io.cpp)
target_include_directories(map_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIR})
target_link_libraries(map_io PUBLIC ${YAML_CPP_LIBRARIES})

# ---- 节点 ----
add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node
  map_io
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${pnc_2d_TARGETS})

install(TARGETS map_server_node map_io
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/compare_maps.py
  scripts/route_editor.py
  DESTINATION lib/${PROJECT_NAME})

# ---- 单测（纯算法，不需要 ROS 图）----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # 栅格膨胀：几何+索引的活，写错了不报错、只会静默改行为（见测试文件头）
  ament_add_gtest(test_grid_dilate test/test_grid_dilate.cpp)
  target_link_libraries(test_grid_dilate map_io)
endif()

ament_package()
