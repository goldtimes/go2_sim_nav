#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 初始化定位自测节点启动
# 用法：
#   ros2 launch lightning init_selftest.launch.py map_pcd:=/path/global.pcd
#   （先起定位：ros2 launch lightning r41_online_loc.launch.py；再另开终端 ros2 bag play <包>）
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_pcd", default_value="", description="整图 pcd 路径（空则订阅 /lightning/global_map）"),
            DeclareLaunchArgument("timeout", default_value="5.0", description="判定超时(s)"),
            DeclareLaunchArgument("confirm_frames", default_value="3", description="GOOD 连续帧数"),
            DeclareLaunchArgument("csv", default_value="./data/init_selftest_result.csv", description="结果 CSV（空=仅终端）"),
            Node(
                package="lightning",
                executable="init_selftest",
                name="init_selftest",
                parameters=[
                    {
                        "map_pcd": LaunchConfiguration("map_pcd"),
                        "timeout": PythonExpression(["float('", LaunchConfiguration("timeout"), "')"]),
                        "confirm_frames": PythonExpression(["int('", LaunchConfiguration("confirm_frames"), "')"]),
                        "csv": LaunchConfiguration("csv"),
                    }
                ],
                output="screen",
            ),
        ]
    )
