"""三节点一键启动（P4 起的推荐入口）。

    ros2 launch pnc_2d pnc_2d.launch.py
    ros2 launch pnc_2d pnc_2d.launch.py planner_type:=route_network
    ros2 launch pnc_2d pnc_2d.launch.py local_type:=mpc      # P5 后可用

数据流（doc/pnc2d_restructure_plan.md §5）：

    /goal_pose ─→ [pnc_manager] ──service PlanPath──→ [global_planner] ─→ 全局路径
                       │                                                  │
                       └──action FollowPath──→ [local_planner] ←──────────┘
                                                    └─→ /pnc_2d/cmd_vel（P5 才有速度）

⚠ **话题触发点的重映射（重要）**：
   `/goal_pose` 由 manager 独占消费。全局节点自己也有一个"收到目标就规划一次"的
   话题入口，如果两者都听 `/goal_pose`，点一次目标会**规划两次**（一次直接发路径、
   一次经状态机），出现"路径被状态机清掉/又冒出来"这种难查的现象。
   所以这里把全局节点的话题入口重映射到 `/pnc_2d/manual_goal`：调试时仍可用
   `ros2 topic pub` 直接指挥全局节点，日常不与 manager 抢。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GLOBAL_FRAGMENT_BY_TYPE = {
    'astar': 'global_astar.yaml',
    'route_network': 'route_network.yaml',
}
LOCAL_FRAGMENT_BY_TYPE = {
    'none': 'local_null.yaml',
    'null': 'local_null.yaml',
    'mpc': 'local_mpc.yaml',
}
# 全局节点的"手工目标"入口（**相对名**：根命名空间下是 /pnc_2d/manual_goal；
# 加上 ns:=/e2e 就变 /e2e/pnc_2d/manual_goal，不会与另一套栈碰撞）
MANUAL_GOAL_TOPIC = 'pnc_2d/manual_goal'


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def _make_nodes(context, *args, **kwargs):
    share = get_package_share_directory('pnc_2d')
    cfg_dir = os.path.join(share, 'config')

    planner_type = LaunchConfiguration('planner_type').perform(context).strip()
    local_type = LaunchConfiguration('local_type').perform(context).strip()
    extra = LaunchConfiguration('extra_config').perform(context).strip()
    ns = LaunchConfiguration('ns').perform(context).strip()
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))

    global_frag = GLOBAL_FRAGMENT_BY_TYPE.get(
        planner_type, 'global_%s.yaml' % planner_type)
    local_frag = LOCAL_FRAGMENT_BY_TYPE.get(
        local_type, 'local_%s.yaml' % local_type)

    def cfg_path(name):
        path = os.path.join(cfg_dir, name)
        if not os.path.isfile(path):
            raise RuntimeError(
                '找不到参数片段 %s（planner_type=%s, local_type=%s）；可用片段：%s'
                % (path, planner_type, local_type, sorted(os.listdir(cfg_dir))))
        return path

    # 三份 yaml 都传给三个节点：每个节点只看自己那一段（实测：没有自己段落的
    # 文件不会报错，只是不生效），所以可以用同一组文件。
    shared = [cfg_path('pnc_2d.yaml'), cfg_path(local_frag), cfg_path(global_frag)]
    if extra:
        shared.append(extra)

    # ⚠ 话题名是**参数**（topics.goal）而不是节点内部固定的名字，所以要用
    #   参数覆盖，不能用 remapping。
    # ⚠ use_sim_time 必须由**这里**给（字典参数生成的临时 params file 在所有 yaml
    #   之后加载 → 能覆盖 pnc_2d.yaml 里的默认 false）。仿真里忘了它会很坑：
    #   节点用墙钟、话题用仿真钟 → 超时判定（位姿/距离场）全部乱套。
    global_node = Node(
        package='pnc_2d', executable='global_planner_node', name='global_planner',
        namespace=ns, output='screen',
        parameters=shared + [{'planner.type': planner_type,
                              'topics.goal': MANUAL_GOAL_TOPIC,
                              'use_sim_time': use_sim_time}],
    )

    return [
        global_node,
        Node(
            package='pnc_2d', executable='local_planner_node', name='local_planner',
            namespace=ns, output='screen',
            parameters=shared + [{'local.type': local_type,
                                  'use_sim_time': use_sim_time}],
        ),
        Node(
            package='pnc_2d', executable='pnc_manager_node', name='pnc_manager',
            namespace=ns, output='screen',
            parameters=shared + [{'use_sim_time': use_sim_time}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_type', default_value='astar',
            description='全局算法类型（astar / route_network）'),
        DeclareLaunchArgument(
            'local_type', default_value='none',
            description='局部算法类型（none / mpc）'),
        DeclareLaunchArgument(
            'extra_config', default_value='',
            description='追加的参数文件（最后加载，优先级最高）'),
        DeclareLaunchArgument(
            'ns', default_value='',
            description='节点命名空间（如 /e2e）。自家话题/IPC 是相对名，会跟着搬；'
                        '外部话题（地图/定位/目标）是绝对名，如需隔离用 extra_config 覆盖'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='用仿真时钟（起 gazebo 时必须 true，否则超时判定全是墙钟）'),
        OpaqueFunction(function=_make_nodes),
    ])
