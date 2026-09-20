"""独立感知节点启动文件。

只依赖定位（lightning）的输出，不依赖任何规划器：

    ros2 launch plan_env perception.launch.py

输入话题（由 launch 参数写入 grid_map.topic_* 参数；优先级：launch 参数 >
perception.yaml 里的同名参数 > 代码默认值）：
    cloud        <- /lightning/perception/cloud    (PointCloud2, frame_id=map)
    sensor_pose  <- /lightning/perception/pose     (Odometry, map -> lidar_link)
    body_pose    滑动地图 TF 原点（默认 body_pose）。上游不发时该 TF 会一直发在
                 原点，只影响 RViz 视图，不影响栅格内容。
    map_state    <- /lightning/map_state          (Int32, 换图/失败时清空栅格)

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
        description='map 系点云话题 -> grid_map.topic_cloud',
    )
    pose_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/lightning/perception/pose',
        description='map -> lidar_link 位姿话题 -> grid_map.topic_pose',
    )
    body_pose_arg = DeclareLaunchArgument(
        'body_pose_topic',
        default_value='body_pose',
        description='滑动地图 TF 原点话题 -> grid_map.topic_body_pose（上游无此话题时可无视）',
    )
    map_state_arg = DeclareLaunchArgument(
        'map_state_topic',
        default_value='/lightning/map_state',
        description='定位阶段话题（换图/失败即清空栅格）-> grid_map.topic_map_state',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
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
        parameters=[
            LaunchConfiguration('config'),
            {
                'grid_map.topic_cloud': LaunchConfiguration('cloud_topic'),
                'grid_map.topic_pose': LaunchConfiguration('pose_topic'),
                'grid_map.topic_body_pose': LaunchConfiguration('body_pose_topic'),
                'grid_map.topic_map_state': LaunchConfiguration('map_state_topic'),
            },
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
        [config_arg, cloud_arg, pose_arg, body_pose_arg, map_state_arg,
         rviz_arg, rviz_config_arg, perception_node, rviz_node]
    )
