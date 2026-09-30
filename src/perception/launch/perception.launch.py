"""独立感知节点启动文件。

只依赖定位（lightning）的输出，不依赖任何规划器：

    ros2 launch perception perception.launch.py

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
    pkg_share = FindPackageShare('perception')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'perception.yaml'])
    default_rviz = PathJoinSubstitution([pkg_share, 'config', 'perception.rviz'])

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='GridMap 参数文件（默认 config/perception.yaml）',
    )
    cloud_arg = DeclareLaunchArgument(
        'cloud_topic',
        # ★ 2026-09-30：默认从降采样版切到**未降采样**版（见 doc/dense_cloud_plan.md）。
        #   原因：`filter_size_scan: 0.2` 降采样后射线密度不足，raycast 会出现大量
        #   “该被射线穿过、却没有射线”的体素 ⇒ 动态物体走后留下清不掉的幽灵格。
        #   实测：cloud_full 9280 点 vs cloud 4160 点（2 m 处射线间隔 5.7°→2.0°，
        #   而一个 0.1 m 体素占 2.9°）。
        #   ⚠⚠ **优先级**：launch 参数 > yaml —— 下面用 LaunchConfiguration 写进
        #     `grid_map.topic_cloud`，所以**只改 perception.yaml 不生效**（踩过）。
        #   回退：把 default_value 改回 '/lightning/perception/cloud'，或启动时传
        #     `cloud_topic:=/lightning/perception/cloud`。
        default_value='/lightning/perception/cloud_full',
        description='map 系点云话题 -> grid_map.topic_cloud（⚠ 优先级高于 yaml）',
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
        package='perception',
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
