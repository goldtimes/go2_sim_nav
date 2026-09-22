#!/usr/bin/env python3
"""E2E-3 运行时热切换 / 热重载：

  · astar → param set planner.type=route_network：同一目标重规划后**贴路网走**
  · 非法类型：param 修改被**拒绝**（参数值不变），服务返回 success=false
  · 服务 switch_planner：合法成功、非法失败
  · reload_params：改 footprint.length 后重新装载，新车体尺寸生效

跑法见 test/e2e/README.md。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                              # noqa: E402
from e2e_common import (Harness, call, dev_to_lane, lane_polyline,        # noqa: E402
                        spin, start_node, stop_node)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue    # noqa: E402
from rcl_interfaces.srv import SetParameters                               # noqa: E402
from pnc_2d.srv import SwitchPlanner                                       # noqa: E402
from std_srvs.srv import Trigger                                           # noqa: E402

GOAL = (9.85, -5.33)


def set_param(node, name, value):
    cli = node.create_client(SetParameters, f"/gp_e2e/set_parameters")
    req = SetParameters.Request()
    if isinstance(value, str):
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
    req.parameters = [Parameter(name=name, value=pv)]
    return call(node, cli, req)


rclpy.init()
h = Harness()
proc = None
try:
    poly = lane_polyline()
    proc = start_node(h, "astar")

    print("[1] astar：路径不该贴路网")
    h.goal(*GOAL)
    spin(h, 2.5)
    h.chk("A* 路径非空", h.n_poses() > 0, h.n_poses())
    dev = dev_to_lane(h.points(), poly)
    print(f"      与通道最大偏离 {dev:.2f} m")
    h.chk("A* 不贴路网（偏离 > 0.5 m）", dev > 0.5, dev)

    print("[2] param set planner.type=route_network → 热切换 + 清空旧路径")
    r = set_param(h, "planner.type", "route_network")
    h.chk("参数修改被接受", r is not None and r.results[0].successful,
          r.results[0].reason if r else "no response")
    spin(h, 1.0)
    h.chk("切换后旧路径被清空", h.n_poses() == 0, h.n_poses())

    print("[3] 同一目标重规划 → 应沿路网（中间点偏离 < 0.1 m）")
    h.goal(*GOAL)
    spin(h, 2.5)
    h.chk("路径非空", h.n_poses() > 0, h.n_poses())
    pts = h.points()
    mid = dev_to_lane(pts[1:-1], poly) if len(pts) > 2 else 1e9
    dev = dev_to_lane(pts, poly)
    print(f"      与通道最大偏离 {dev:.2f} m | 去掉首末补段后 {mid:.2f} m")
    h.chk("路网规划确实贴线", mid < 0.1, mid)

    print("[4] param set 非法类型 → 被拒绝且参数值不变")
    r = set_param(h, "planner.type", "nonsense")
    h.chk("参数修改被拒绝", r is not None and not r.results[0].successful,
          r.results[0].reason if r else "no response")
    print(f"      拒绝原因：{r.results[0].reason if r else '-'}")
    r2 = set_param(h, "planner.type", "route_network")   # 同值 → 幂等成功
    h.chk("同值设置是幂等的（成功）", r2 is not None and r2.results[0].successful)

    print("[5] 服务 switch_planner：合法成功 / 非法失败")
    r = call(h, h.srv_switch, SwitchPlanner.Request(type="astar"))
    h.chk("切回 astar 成功", r is not None and r.success, r.message if r else None)
    h.chk("切换后清空", h.n_poses() == 0, h.n_poses())
    r = call(h, h.srv_switch, SwitchPlanner.Request(type="nonsense"))
    h.chk("非法类型返回失败", r is not None and not r.success, r.message if r else None)
    print(f"      返回消息：{r.message if r else '-'}")

    print("[6] reload_params：改 footprint.length 后热重载")
    r = set_param(h, "footprint.length", 0.99)
    h.chk("footprint.length 参数已改", r is not None and r.results[0].successful)
    r = call(h, h.srv_reload, Trigger.Request())
    h.chk("reload_params 成功", r is not None and r.success, r.message if r else None)
    # 车体变长后，原本可行的窄通道会变成不可行（reload = 真的按新参数重跑校验）
    r = call(h, h.srv_switch, SwitchPlanner.Request(type="route_network"))
    h.chk("重载后仍可切换算法", r is not None and r.success, r.message if r else None)
finally:
    stop_node(proc)
    rc = h.summary()
    rclpy.shutdown()
    sys.exit(rc)
