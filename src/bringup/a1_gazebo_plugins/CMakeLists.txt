cmake_minimum_required(VERSION 3.8)
project(a1_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(a1_joint_pd_plugin SHARED src/joint_pd_plugin.cpp)
target_include_directories(a1_joint_pd_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
ament_target_dependencies(a1_joint_pd_plugin rclcpp sensor_msgs std_msgs gazebo_ros)
target_link_libraries(a1_joint_pd_plugin ${GAZEBO_LIBRARIES})

# 注意：必须装到 share/<pkg>/lib，与 package.xml 里
# <gazebo_ros plugin_path="${prefix}/lib"/> 的 ${prefix}（= share 目录）对应，
# 否则 gazebo_ros_paths 生成的 GAZEBO_PLUGIN_PATH 指向不存在的目录，
# Gazebo 找不到这个 .so 时会**静默跳过**，既不报错也不加载。
install(TARGETS a1_joint_pd_plugin DESTINATION share/${PROJECT_NAME}/lib)

ament_package()
