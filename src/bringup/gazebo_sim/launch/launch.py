"""
gazebo_sim 包的主启动文件 (launch.py)

功能概览：
1. 启动 Gazebo Sim 仿真器，并加载指定的世界文件（如 warehouse.sdf / rmuc_2025_world.sdf）
2. 等待 Gazebo 完全启动（sleep 6 秒）
3. 根据 `sensors` 参数决定启动哪个机器人 launch 文件：
   - sensors:=true  → gazebo_go2_sensors.launch.py（带 VLP-16 雷达、D435i 深度相机等外置传感器）
   - sensors:=false → gazebo_go2_self.launch.py（无外置传感器版）

典型用法：
    ros2 launch gazebo_sim launch.py sensors:=true world:=warehouse.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,   # 包含其它 launch 文件
    DeclareLaunchArgument,      # 声明命令行启动参数
    ExecuteProcess,             # 执行外部进程（如 sleep 命令）
    RegisterEventHandler,       # 注册事件处理器（监听进程退出等）
    OpaqueFunction,             # 延迟到 launch 运行时才执行的函数
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter


def create_gazebo_action(context: LaunchContext, package_name: str):
    """
    创建启动 Gazebo Sim 的 action。

    该函数通过 OpaqueFunction 延迟执行，因为其中的参数
    （如 world）需要等 launch 运行时才能解析出实际值。

    Args:
        context:       launch 上下文，用于解析 LaunchConfiguration 的最终值
        package_name:  包名，用于获取包安装目录（此处为 'gazebo_sim'）

    Returns:
        一个包含 Gazebo 启动 action 的列表
    """
    # 解析 world 参数，得到用户指定的世界文件名（如 'warehouse.sdf'）
    world_name = LaunchConfiguration('world').perform(context)
    # 获取 gazebo_sim 包在 install 目录下的路径
    pkg_path = get_package_share_directory(package_name)
    # 拼接世界文件的绝对路径（install/gazebo_sim/share/gazebo_sim/world/<world_name>）
    world_file = os.path.join(pkg_path, 'world', world_name)

    # 复用 ros_gz_sim 包提供的 gz_sim.launch.py 来启动 Gazebo Sim
    gazebo_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r  : 启动后立即运行仿真（不暂停）
            # -v4 : 日志详细级别为 4（较详细）
            'gz_args': ['-r -v4 ', world_file],
            # Gazebo 退出时，整个 launch 进程也随之关闭
            'on_exit_shutdown': 'true'
        }.items()
    )
    return [gazebo_action]


def choose_launch_file(context, *args, **kwargs):
    """
    根据 `sensors` 参数选择要包含的机器人 launch 文件。

    Args:
        context: launch 上下文，用于读取 sensors 参数的值

    Returns:
        一个包含所选 launch 文件（IncludeLaunchDescription）的列表
    """
    # 解析 sensors 参数（用户命令行传入，如 sensors:=true）
    use_sensors = LaunchConfiguration('sensors').perform(context)
    package_name = 'gazebo_sim'
    pkg_path = get_package_share_directory(package_name)

    # sensors:=true  → 传感器版；否则 → 无外置传感器版
    if use_sensors.lower() == 'true':
        launch_file = 'gazebo_go2_sensors.launch.py'
    else:
        launch_file = 'gazebo_go2_self.launch.py'

    # 返回一个 IncludeLaunchDescription，用于包含所选机器人 launch 文件
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, 'launch', launch_file)
            ),
            # 透传 enable_ekf（默认 false）；不声明就会把命令行传的参数静默丢掉
            launch_arguments={
                'enable_ekf': LaunchConfiguration('enable_ekf'),
            }.items()
        )
    ]


def generate_launch_description():
    """
    ROS 2 launch 系统要求的主入口函数。
    返回值是一个 LaunchDescription，描述整个启动流程。
    """
    ld = LaunchDescription()
    package_name = 'gazebo_sim'

    # ---------- 1. 声明 use_sim_time 参数 ----------
    # 控制节点是否使用仿真时间（与 Gazebo 的 /clock 同步）
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    ))
    # 将 use_sim_time 设为全局参数，使所有节点继承
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))

    # ---------- 2. 声明 sensors 参数 ----------
    # true 启动带外置传感器版本，false 启动无外置传感器版本
    ld.add_action(DeclareLaunchArgument(
        'sensors',
        default_value='false',
        description='是否启动传感器版launch文件（默认启动无外置传感器版）'
    ))

    # ---------- 3. 声明 world 参数 ----------
    # 指定要加载的 Gazebo 世界文件（放在 gazebo_sim/world 目录下）
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value='rmuc_2025_world.sdf',
        description='指定要加载的Gazebo世界文件（需放在gazebo_sim/world目录下）'
    ))

    # ---------- 3b. 声明 enable_ekf 参数 ----------
    # 是否启动 robot_localization 的 EKF；仿真里用 lightning 做里程计/定位时不需要（默认 false）。
    ld.add_action(DeclareLaunchArgument(
        'enable_ekf',
        default_value='false',
        description='是否启动 robot_localization EKF（默认关；仿真用 lightning 做定位）'
    ))

    # ---------- 4. 启动 Gazebo Sim ----------
    # 用 OpaqueFunction 延迟调用 create_gazebo_action，
    # 因为其中需要解析上面声明的 world 参数值
    gazebo_action = OpaqueFunction(
        function=create_gazebo_action,
        args=[package_name]
    )
    ld.add_action(gazebo_action)

    # ---------- 5. 等待 Gazebo 完全启动 ----------
    # 通过 sleep 6 秒给 Gazebo 留出初始化时间，
    # 避免机器人节点在 Gazebo world 服务就绪前启动导致连接失败
    pause = ExecuteProcess(
        cmd=['sleep', '6'],
        output='screen'
    )
    ld.add_action(pause)

    # ---------- 6. sleep 结束后再启动机器人相关节点 ----------
    # 同样用 OpaqueFunction 延迟选择并包含机器人 launch 文件
    go2_sensors_launch = OpaqueFunction(function=choose_launch_file)

    # 注册事件：当 pause（sleep）进程退出后，才执行 go2_sensors_launch
    launch_after_pause = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pause,
            on_exit=[go2_sensors_launch]
        )
    )
    ld.add_action(launch_after_pause)

    return ld
