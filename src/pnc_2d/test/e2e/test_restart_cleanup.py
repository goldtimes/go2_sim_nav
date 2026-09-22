#!/usr/bin/env python3
"""E2E-2 换算法重启后的清场：旧进程留下的路径/通路高亮必须被新进程清掉。

场景（用户实际踩到的）：
  ① 跑路网规划（会画"走过的通道"黄色高亮 + 发布 latched 路径）
  ② 杀掉节点、用 A* 重启
  ③ 旧路径与黄色高亮**必须消失** —— 靠新进程启动时的 publishStartupCleanup()

跑法见 test/e2e/README.md。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                        # noqa: E402
from e2e_common import (Harness, start_node, stop_node, spin)        # noqa: E402

GOAL = (9.85, -5.33)

rclpy.init()
h = Harness()
proc = None
try:
    print("[1] route_network 实例：规划一次 → 有路径 + 有通路高亮")
    proc = start_node(h, "route_network")
    h.goal(*GOAL)
    spin(h, 2.0)
    h.chk("路径非空", h.n_poses() > 0, h.n_poses())
    h.chk("有 route_active 通路高亮(ADD)",
          len(h.markers_in("route_active", 0)) > 0,
          len(h.markers_in("route_active", 0)))

    print("[2] 杀掉它，用 astar 重启（模拟\"换成普通规划\"）")
    stop_node(proc)
    proc = None
    spin(h, 1.0)
    h.paths.clear(), h.markers.clear()
    proc = start_node(h, "astar")
    spin(h, 2.0)
    h.chk("启动即发空路径（清掉 latched 里的旧路径）",
          h.path is not None and h.n_poses() == 0, h.n_poses())
    h.chk("启动即 DELETE 残留的通路高亮", h.saw_delete("route_active"))

    print("[3] astar 正常规划 → 新路径，且不再有路网高亮")
    h.goal(*GOAL)
    spin(h, 2.0)
    h.chk("新的 A* 路径非空", h.n_poses() > 0, h.n_poses())
    h.chk("重启后没有任何 route_active 高亮(ADD)", not h.saw_add("route_active"))
finally:
    stop_node(proc)
    rc = h.summary()
    rclpy.shutdown()
    sys.exit(rc)
