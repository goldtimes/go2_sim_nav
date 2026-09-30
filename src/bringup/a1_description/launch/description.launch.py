"""只启动 robot_state_publisher 的轻量 launch，用于在 RViz 里单独查看 A1 模型。

用法:
    ros2 launch a1_description description.launch.py
    ros2 launch a1_description description.launch.py robot_name:=a1 use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _robot_state_publisher(context, *args, **kwargs):
    """在 launch 运行时解析 robot_name，再展开 xacro。"""
    pkg_path = get_package_share_directory('a1_description')
    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_name = LaunchConfiguration('robot_name').perform(context)

    robot_description = xacro.process_file(
        xacro_file, mappings={'robot_name': robot_name}).toxml()

    # 转成真 bool，避免 "true" 字符串被当作 string 型 ROS 参数
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ]


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='a1',
                              description='机器人命名空间'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='是否使用仿真时间'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否同时启动 RViz'),

        OpaqueFunction(function=_robot_state_publisher),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=robot_name,
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
