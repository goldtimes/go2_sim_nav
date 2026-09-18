"""独立感知节点启动文件。

只依赖定位（lightning）的输出，不依赖任何规划器：

    ros2 launch plan_env perception.launch.py

话题映射（grid_map.cpp 内部用相对名 cloud / sensor_pose / body_pose）：
    cloud       <- /lightning/perception/cloud    (PointCloud2, frame_id=map)
    sensor_pose <- /lightning/perception/pose     (Odometry, map -> lidar_link)
    body_pose    不订阅（滑动地图以 sensor_pose 为中心滚动，不需要它）

上游需要开启 lightning 配置里的 system.enable_perception_pub。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('plan_env')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'perception.yaml'])
    default_rviz = PathJoinSubstitution([pkg_share, 'config', 'perception.rviz'])

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='GridMap 参数文件（默认 config/perception.yaml）',
    )
    cloud_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/lightning/perception/cloud',
        description='map 系点云话题',
    )
    pose_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/lightning/perception/pose',
        description='map -> lidar_link 位姿话题',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='是否同时启动 RViz（默认 false，推荐自己起 showbodypc.rviz 等视图）',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz,
        description='RViz 配置文件路径',
    )

    perception_node = Node(
        package='plan_env',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            ('cloud', LaunchConfiguration('cloud_topic')),
            ('sensor_pose', LaunchConfiguration('pose_topic')),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(
        [config_arg, cloud_arg, pose_arg, rviz_arg, rviz_config_arg, perception_node, rviz_node]
    )
