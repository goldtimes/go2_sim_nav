"""全局规划节点启动文件。

    ros2 launch pnc_2d global_planner.launch.py
    ros2 launch pnc_2d global_planner.launch.py planner_type:=astar

前置话题（本节点不做 TF 变换，三者必须已在同一坐标系，默认 map）：
    /global_map/occupancy        nav_msgs/OccupancyGrid   ← map_server（latched）
    /lightning/perception/pose   nav_msgs/Odometry        ← 定位
    /goal_pose                   geometry_msgs/PoseStamped ← RViz "2D Goal Pose"

输出：
    /pnc_2d/global_path          nav_msgs/Path
    /pnc_2d/plan_markers         visualization_msgs/MarkerArray（起终点箭头 + 车体轮廓）

参数优先级：launch 参数 > config/global_planner.yaml > 代码默认值。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('pnc_2d')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'global_planner.yaml'])

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='参数文件路径')
    planner_type_arg = DeclareLaunchArgument(
        'planner_type', default_value='astar',
        description='规划算法类型（对应 planner.type）')

    node = Node(
        package='pnc_2d',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {'planner.type': LaunchConfiguration('planner_type')},
        ],
    )

    return LaunchDescription([config_arg, planner_type_arg, node])
