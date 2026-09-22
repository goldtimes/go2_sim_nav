"""局部规划节点启动文件（单跑，调试用）。

    ros2 launch pnc_2d local_planner.launch.py
    ros2 launch pnc_2d local_planner.launch.py local_type:=none

前置话题：
    /lightning/perception/pose   nav_msgs/Odometry（best_effort）← 定位

接口：
    ~/follow_path    pnc_2d/action/FollowPath   跟路径（action：有反馈 + 可取消）
    ~/stop           std_srvs/Trigger           立刻停止跟随
    ~/switch_planner pnc_2d/srv/SwitchPlanner   local.type 热切换
    ~/reload_params  std_srvs/Trigger           改权重后重新装载（不用重启）

输出：
    /pnc_2d/local_status  pnc_2d/LocalStatus（latched）
    /pnc_2d/cmd_vel       geometry_msgs/Twist  ← **只有 producesCmdVel()=true 才发**
                          （'none' 空实现下连发布者都不会创建）

参数优先级（**后面的覆盖前面的**）：
    1. config/pnc_2d.yaml        总入口（话题/坐标系/控制周期）
    2. config/local_<类型>.yaml  局部算法片段（自述 local.type）
    3. extra_config              launch 参数，追加一份自定义片段
    4. local_type                launch 参数，最终强制覆盖 local.type
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# local.type → 片段文件名。
# ⚠ 不要把类型名拼成文件名：`local.type: none` 对应的文件叫 local_null.yaml
#   （yaml 里写 none 更自然，但实现类叫 NullLocalPlanner）。
FRAGMENT_BY_TYPE = {
    'none': 'local_null.yaml',
    'null': 'local_null.yaml',
    'mpc': 'local_mpc.yaml',
}


def _make_node(context, *args, **kwargs):
    share = get_package_share_directory('pnc_2d')
    cfg_dir = os.path.join(share, 'config')

    local_type = LaunchConfiguration('local_type').perform(context).strip()
    extra = LaunchConfiguration('extra_config').perform(context).strip()
    fragment = FRAGMENT_BY_TYPE.get(local_type, 'local_%s.yaml' % local_type)

    def cfg_path(name):
        path = os.path.join(cfg_dir, name)
        if not os.path.isfile(path):
            raise RuntimeError(
                '找不到参数片段 %s（local_type=%s）；可用片段：%s'
                % (path, local_type, sorted(os.listdir(cfg_dir))))
        return path

    files = [cfg_path('pnc_2d.yaml'), cfg_path(fragment)]
    if extra:
        files.append(extra)
    files.append({'local.type': local_type})   # 最终强制覆盖

    return [Node(
        package='pnc_2d',
        executable='local_planner_node',
        name='local_planner',
        output='screen',
        parameters=files,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'local_type', default_value='none',
            description='局部算法类型（none / mpc）'),
        DeclareLaunchArgument(
            'extra_config', default_value='',
            description='追加的参数文件（最后加载，优先级最高）'),
        OpaqueFunction(function=_make_node),
    ])
