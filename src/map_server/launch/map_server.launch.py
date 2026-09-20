"""全局地图服务器启动文件。

    ros2 launch map_server map_server.launch.py
    ros2 launch map_server map_server.launch.py map_dir:=/home/gmd/rcs/maps/r41iws_0917

换图：运行时用服务即可（不用重启）
    ros2 service call /global_map/load_map nav2_msgs/srv/LoadMap \\
        "{map_url: /home/gmd/rcs/maps/r41iws_0917}"

参数优先级：launch 参数 > 参数文件（config/map_server.yaml）> 代码默认值。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_MAP_DIR = '/home/gmd/rcs/maps/go2_sim_factory'


def generate_launch_description():
    pkg_share = FindPackageShare('map_server')
    default_config = PathJoinSubstitution([pkg_share, 'config', 'map_server.yaml'])

    config_arg = DeclareLaunchArgument(
        'config', default_value=default_config,
        description='map_server 参数文件')
    map_dir_arg = DeclareLaunchArgument(
        'map_dir', default_value=DEFAULT_MAP_DIR,
        description='地图目录（内含 map.yaml/map.pgm）')
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml', default_value='',
        description='直接指定 map.yaml（非空时优先级高于 map_dir）')

    node = Node(
        package='map_server',
        executable='map_server_node',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'map_dir': LaunchConfiguration('map_dir'),
                'map_yaml': LaunchConfiguration('map_yaml'),
            },
        ],
    )

    return LaunchDescription([config_arg, map_dir_arg, map_yaml_arg, node])
