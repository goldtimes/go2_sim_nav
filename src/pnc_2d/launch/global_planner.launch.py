"""全局规划节点启动文件。

    ros2 launch pnc_2d global_planner.launch.py
    ros2 launch pnc_2d global_planner.launch.py planner_type:=astar
    ros2 launch pnc_2d global_planner.launch.py planner_type:=route_network

前置话题（本节点不做 TF 变换，三者必须已在同一坐标系，默认 map）：
    /global_map/occupancy        nav_msgs/OccupancyGrid   ← map_server（latched）
    /lightning/perception/pose   nav_msgs/Odometry        ← 定位
    /goal_pose                   geometry_msgs/PoseStamped ← RViz "2D Goal Pose"

输出：
    /pnc_2d/global_path          nav_msgs/Path
    /pnc_2d/plan_markers         visualization_msgs/MarkerArray（起终点箭头 + 车体轮廓）

参数优先级（**后面的覆盖前面的**）：
    1. config/pnc_2d.yaml        总入口：话题/坐标系/代价语义/车体轮廓/路径有效期
    2. config/local_null.yaml    局部片段（目前只有"空"实现）
    3. config/global_<type>.yaml 算法片段：自述 planner.type + 该算法私有参数
       （planner_type:=route_network 时读 config/route_network.yaml）
    4. extra_config              launch 参数，追加一份自定义片段（最后加载，优先级最高）
    5. planner_type / local_type launch 参数，最终强制覆盖类型
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# planner.type → 算法片段文件名。
# ⚠ 新增算法时**这里和 config/ 目录一起加**：路由到缺失的文件会让 launch 直接报错。
FRAGMENT_BY_TYPE = {
    'astar': 'global_astar.yaml',
    'route_network': 'route_network.yaml',
}
# local.type → 局部片段。**不要按类型名拼文件名**：`local.type: none` 对应的文件叫
# local_null.yaml（yaml 里写 none 更自然，但实现类叫 NullLocalPlanner），拼字符串会找错。
LOCAL_FRAGMENT_BY_TYPE = {
    'none': 'local_null.yaml',
    'null': 'local_null.yaml',
}


def _make_node(context, *args, **kwargs):
    share = get_package_share_directory('pnc_2d')
    cfg_dir = os.path.join(share, 'config')

    planner_type = LaunchConfiguration('planner_type').perform(context).strip()
    local_type = LaunchConfiguration('local_type').perform(context).strip()
    extra = LaunchConfiguration('extra_config').perform(context).strip()

    fragment = FRAGMENT_BY_TYPE.get(planner_type, 'global_%s.yaml' % planner_type)
    local_fragment = LOCAL_FRAGMENT_BY_TYPE.get(local_type,
                                                'local_%s.yaml' % local_type)

    def cfg_path(name):
        """找片段：先看 install 目录，找不到就报清楚点（含可用清单）"""
        path = os.path.join(cfg_dir, name)
        if not os.path.isfile(path):
            raise RuntimeError(
                '找不到参数片段 %s（planner_type=%s, local_type=%s）；可用片段：%s'
                % (path, planner_type, local_type, sorted(os.listdir(cfg_dir))))
        return path

    files = [cfg_path('pnc_2d.yaml'), cfg_path(local_fragment), cfg_path(fragment)]
    if extra:
        files.append(extra)

    # 最后一份显式覆盖：即使片段里写错了类型，命令行/launch 参数仍然说了算
    files.append({'planner.type': planner_type,
                  'local.type': local_type})

    return [Node(
        package='pnc_2d',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=files,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_type', default_value='astar',
            description='全局算法类型，决定加载哪个片段（astar / route_network）'),
        DeclareLaunchArgument(
            'local_type', default_value='none',
            description='局部算法类型（本期只有 none；P5 后可用 mpc）'),
        DeclareLaunchArgument(
            'extra_config', default_value='',
            description='追加的参数文件（最后加载，优先级最高）'),
        OpaqueFunction(function=_make_node),
    ])

