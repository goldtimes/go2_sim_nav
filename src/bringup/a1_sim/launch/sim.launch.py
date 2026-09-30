"""A1 + 多层建筑（Building）的 Gazebo Classic 仿真启动入口。

用法:
    ros2 launch a1_sim sim.launch.py                    # 带 GUI
    ros2 launch a1_sim sim.launch.py gui:=false         # 无界面（服务器模式）
    ros2 launch a1_sim sim.launch.py robot_name:=robot1 stand:=false
    ros2 launch a1_sim sim.launch.py world:=/abs/path/other.world

流程:
    1) 用 gazebo_ros 的 gazebo.launch.py 启动 gzserver（+ 可选 gzclient），
       世界文件默认取本包 share/world/building.world
    2) 启动 robot_state_publisher（发布 /<robot_name>/robot_description）
    3) 用 gazebo_ros 的 spawn_entity.py 把机器人放进世界
    4) spawn 完成后启动 joint_state_broadcaster 与 joint_group_position_controller
    5) stand:=true 时启动 stand_pose.py 让 A1 保持站立
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _make_robot_actions(context, *args, **kwargs):
    """在 launch 运行时解析参数，再展开 xacro 并生成本次启动的所有 action。"""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    # 必须转成真正的 bool：把 "true" 字符串塞进 ROS 参数会让
    # robot_state_publisher 抛 InvalidParameterTypeException 直接退出。
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    stand = LaunchConfiguration('stand').perform(context)
    pause = LaunchConfiguration('pause').perform(context)
    x = float(LaunchConfiguration('x').perform(context))
    y = float(LaunchConfiguration('y').perform(context))
    z = float(LaunchConfiguration('z').perform(context))
    yaw = float(LaunchConfiguration('yaw').perform(context))

    a1_share = get_package_share_directory('a1_description')
    sim_share = get_package_share_directory('a1_sim')

    # ---- 世界文件：相对路径按 a1_sim/world 解析 ----
    world_path = world if os.path.isabs(world) else os.path.join(sim_share, 'world', world)
    if not os.path.isfile(world_path):
        raise RuntimeError(f'找不到世界文件: {world_path}')

    # ---- 机器人描述 ----
    robot_description = xacro.process_file(
        os.path.join(a1_share, 'xacro', 'robot.xacro'),
        mappings={'robot_name': robot_name},
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ---- Gazebo（gzserver + 可选 gzclient）----
    # 这里直接 include gzserver.launch.py 而不是 gazebo.launch.py，
    # 因为后者不透传 pause 参数；暂停启动对调出生构型很有用。
    gazebo_share = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_path,
            'verbose': verbose,
            'pause': pause,
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    # ---- 把机器人放进世界 ----
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-topic', f'/{robot_name}/robot_description',
            '-entity', f'{robot_name}_robot',
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            '-timeout', '120',
        ],
        output='screen',
    )

    # ---- spawn 完成后再启动目标姿态发布节点 ----
    # 关节的 PD 控制在 Gazebo 侧由 a1_gazebo_plugins 的插件完成（带力矩钳位），
    # 这里只需要把目标关节角发过去。
    actions_after_spawn = []
    if stand.lower() == 'true':
        actions_after_spawn.append(
            Node(
                package='a1_description',
                # 只发布目标姿态；力矩计算与钳位在 a1_joint_pd 插件里
                executable='stand_pose.py',
                namespace=robot_name,
                output='screen',
                parameters=[{
                    'ramp_time': float(
                        LaunchConfiguration('stand_ramp').perform(context)),
                    'delay': float(
                        LaunchConfiguration('stand_delay').perform(context)),
                }],
            )
        )

    actions = [robot_state_publisher, gazebo, gzclient, spawn_entity]

    # 注意：on_exit 传空列表会让 launch 直接抛异常，必须先判空
    if actions_after_spawn:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_entity, on_exit=actions_after_spawn)
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='building.world',
                              description='世界文件（a1_sim/world 下的文件名，或绝对路径）'),
        DeclareLaunchArgument('robot_name', default_value='a1',
                              description='机器人命名空间，同时作为 xacro 的 robot_name'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 图形客户端'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='gzserver 是否输出详细日志'),
        DeclareLaunchArgument('pause', default_value='false',
                              description='以暂停状态启动世界（调试出生构型用，'
                                          '之后可用 gz world -p 0 恢复）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否使用仿真时间'),
        DeclareLaunchArgument('stand', default_value='true',
                              description='是否启动站立姿态节点'),
        DeclareLaunchArgument('x', default_value='-5.0', description='初始 x 坐标'),
        DeclareLaunchArgument('y', default_value='7.0', description='初始 y 坐标'),
        DeclareLaunchArgument(
            'z', default_value='0.5',
            description='初始 z 坐标。实测：0.42（伸直腿刚好触地）会因接触求解产生'
                        '巨大穿透力把机器人弹飞；0.5 相对温和。别给到脚穿地的高度'),
        DeclareLaunchArgument('stand_delay', default_value='2.0',
                              description='落地后等待多久再开始下蹲(秒)'),
        DeclareLaunchArgument('stand_ramp', default_value='3.0',
                              description='从伸直腿下蹲到站立姿态的时长(秒)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='初始偏航角(rad)'),

        OpaqueFunction(function=_make_robot_actions),
    ])
