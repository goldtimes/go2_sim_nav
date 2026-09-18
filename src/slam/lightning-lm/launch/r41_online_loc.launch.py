#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============ lightning run_loc_online 启动封装 ============
# 用法（需先 source install/setup.bash，建议在工作区根运行）：
#   ros2 launch lightning r41_online_loc.launch.py
#   # 自定义配置：ros2 launch lightning r41_online_loc.launch.py config:=<path>
#   # 指定地图目录：ros2 launch lightning r41_online_loc.launch.py map_path:=~/rcs/maps/factory
#   # （也支持 map_path:=/data/maps/factory 等绝对路径；留空则用配置里的 system.map_path）
#   # 查看参数：ros2 launch lightning r41_online_loc.launch.py --show-args
# 说明：
#   - config(yaml) 内容不变；仅把 config 文件路径作为 ROS2 参数 "config" 注入。
#   - run_loc_online 内部从参数节点(run_loc_online)读取 config 路径，再照旧解析 yaml。
#   - 定位与建图共用同一套 config（内含 lidar_loc/localization 段，含 init_with_fp 等）。
#   - 包内资源位于 lib/lightning/ 下，用 get_package_prefix 运行时解析，避免硬编码。
#   - 本 launch 只启动定位节点；点云/IMU 数据需另起终端 ros2 bag play 回放。
import os

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 运行时解析 lightning 包安装前缀（例如 <ws>/install/lightning），非硬编码
_PKG_PREFIX = get_package_prefix("lightning")
# lightning 的 config 被 install 到 lib/<pkg>/config/ 下
CONFIG_DEFAULT = os.path.join(
    _PKG_PREFIX, "lib/lightning/config/r41_rk_slam.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=CONFIG_DEFAULT,
                description="定位配置文件路径（作为 ROS2 参数 config 传入；默认安装目录中 r41_online_validate.yaml）",
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value="",
                description=(
                    "地图目录（含 index.txt 与 <id>.pcd）。非空时覆盖配置里的 "
                    "system.map_path，供上层在启动时决定加载哪张地图；留空则用配置值。"
                ),
            ),
            Node(
                package="lightning",
                executable="run_loc_online",
                name="run_loc_online",  # 与 C++ 参数读取节点同名，保证参数注入生效
                parameters=[
                    {
                        "config": LaunchConfiguration("config"),
                        "map_path": LaunchConfiguration("map_path"),
                    }
                ],
                output="screen",
            ),
        ]
    )
