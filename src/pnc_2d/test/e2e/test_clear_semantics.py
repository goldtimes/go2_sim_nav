#!/usr/bin/env python3
"""E2E-1 路径有效期：失败清空 / 成功发布 / 到达目标（按参数）/ 显式清空服务 / 状态上报。

跑法（先设 DDS 环境、source install，见 test/e2e/README.md）：
    python3 test/e2e/test_clear_semantics.py                 # 默认：到达目标不清空
    EXPECT_AUTOCLEAR=1 python3 test/e2e/test_clear_semantics.py   # 打开自动清空
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                              # noqa: E402
from e2e_common import (Harness, call, spin, start_node,   # noqa: E402
                        stop_node)
from std_srvs.srv import Trigger                          # noqa: E402

GOAL = (9.85, -5.33)          # 路网上的点
UNREACHABLE = (0.0, -20.0)    # 地图外
AUTO = os.environ.get("EXPECT_AUTOCLEAR", "0") == "1"

rclpy.init()
h = Harness()
proc = None
try:
    proc = start_node(h, "route_network",
                      params=["clear.auto_on_goal_reached:=true"] if AUTO else [])

    print(f"[1] 可达目标 → 非空路径 + SUCCESS 状态（auto_clear={'on' if AUTO else 'off'}）")
    h.paths.clear(), h.statuses.clear()
    h.goal(*GOAL)
    spin(h, 2.0)
    h.chk("路径非空", h.n_poses() > 0, h.n_poses())
    s = h.status
    h.chk("状态 SUCCESS", s is not None and s.success and s.status_name == "SUCCESS",
          f"{s.status_name if s else None} {s.message if s else ''}")
    if h.path:
        import math
        L = sum(math.dist(a, b) for a, b in zip(h.points(), h.points()[1:]))
        h.chk("状态里带长度/点数",
              s is not None and abs(s.path_length_m - L) < 0.5 and s.path_points == len(h.points()),
              f"{s.path_length_m:.2f}m vs {L:.2f}m")

    print("[2] 不可达目标 → 清空路径 + NO_PATH 状态（失败必须清，避免过期路径）")
    h.goal(*UNREACHABLE)
    spin(h, 2.0)
    h.chk("路径被清空（poses=0）", h.path is not None and h.n_poses() == 0, h.n_poses())
    s = h.status
    h.chk("状态非成功", s is not None and not s.success, s.status_name if s else None)
    h.chk("失败原因非空", s is not None and len(s.message) > 0, s.message if s else "")

    print("[3] 重新规划成功 → 把车开到目标附近")
    h.pose = (0.0, 0.03)
    h.goal(*GOAL)
    spin(h, 2.0)
    h.chk("先有路径", h.n_poses() > 0, h.n_poses())
    h.pose = (9.80, -5.30)
    spin(h, 2.0)
    if AUTO:
        h.chk("到达后路径被清空（自动清空开着）", h.n_poses() == 0, h.n_poses())
    else:
        h.chk("到达后路径仍保留（默认由状态机决定何时清）", h.n_poses() > 0, h.n_poses())

    print("[4] 显式清空服务 ~/clear_path")
    h.pose = (0.0, 0.03)
    h.goal(*GOAL)
    spin(h, 2.0)
    h.chk("先有路径", h.n_poses() > 0, h.n_poses())
    r = call(h, h.srv_clear, Trigger.Request())
    h.chk("服务返回成功", r is not None and r.success, r.message if r else "no response")
    h.chk("服务后路径被清空", h.n_poses() == 0, h.n_poses())
finally:
    stop_node(proc)
    rc = h.summary()
    rclpy.shutdown()
    sys.exit(rc)
