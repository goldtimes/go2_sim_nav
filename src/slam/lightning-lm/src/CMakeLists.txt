add_library(${PROJECT_NAME}.libs SHARED
        common/nav_state.cc
        common/constant.h
        common/log.cc
        common/options.cc
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc
        ../thirdparty/ikd-Tree/ikd_Tree.cpp

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/small_gicp_matcher.cc

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization.cpp

        utils/timer.cc
        utils/pointcloud_utils.cc

        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_cloud.cc
        ui/ui_trajectory.cc

        core/system/slam.cc
        core/system/loc_system.cc

        wrapper/bag_io.cc
)

ament_target_dependencies(
        ${PROJECT_NAME}.libs
        "rclcpp"
        "std_msgs"
        "geometry_msgs"
        "sensor_msgs"
        "nav_msgs"
        "std_srvs"
        "message_filters"
        "pcl_conversions"
        "tf2_ros"
        "tf2"
)

add_dependencies(${PROJECT_NAME}.libs
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
include_directories(${PROJECT_SOURCE_DIR}/src/core/lio/fastlio)
add_subdirectory(core/miao)

# small_gicp（vendored，来源 src/slam/small_gicp @ 2c6c783，MIT License）
add_library(small_gicp STATIC
        ../thirdparty/small_gicp/src/registration.cpp
        ../thirdparty/small_gicp/src/registration_helper.cpp)
target_include_directories(small_gicp PUBLIC ${PROJECT_SOURCE_DIR}/thirdparty/small_gicp/include)
target_link_libraries(small_gicp PUBLIC
        Eigen3::Eigen
        $<TARGET_NAME_IF_EXISTS:OpenMP::OpenMP_CXX>)

target_link_libraries(${PROJECT_NAME}.libs
        ${third_party_libs}
        ${PCL_LIBRARIES}
        miao.core
        miao.utils
        small_gicp
        livox_interfaces2__rosidl_typesupport_cpp
        ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS ${PROJECT_NAME}.libs miao.core miao.utils small_gicp
        DESTINATION lib)

add_subdirectory(app)
