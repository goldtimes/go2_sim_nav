#!/usr/bin/env python3
"""把车开到**最开阔的地方**停下 —— 贴线/角度验收的前置工具。

为什么需要它：`test_route_lane.py` 要求"车前 3~6 m 有一条净距 ≥0.55 m 的直线"，
而自由行驶验收结束时车常停在**墙边 0.5 m** 处 ⇒ 怎么放宽转角都挑不出通道，脚本只能
报"先挪一下车"。这时跑一下本脚本，车会开到全局图里"离最近障碍最远"的地方
（图上是整片空地），随后贴线验收就能生成通道了。

    python3 test/sim/park_open.py                # 默认：附近 30 m 内、净距 ≥1.5 m
    python3 test/sim/park_open.py --radius 20 --min-clear 1.2

实现说明：净距用**切比雪夫距离**（八邻域反复腐蚀自由空间）近似，不引入 scipy ——
系统 scipy(1.8) 与 venv 的 numpy(2.x) ABI 不兼容（ImportError: numpy.core.multiarray），
而这里只需要"离障碍足够远"这个定性结论。
"""
from __future__ import annotations

import argparse
import math
import sys
import time

sys.path.insert(0, "/home/gmd/r41_ws/src/pnc_2d/test/sim")

import numpy as np
import rclpy

import sim_common as sc


def widest_point(grid, cx, cy, radius, min_clear):
    """全局图里"离最近障碍最远"的自由格（限定在以 (cx,cy) 为心、radius 为半径内）

    用**切比雪夫距离**（八邻域反复腐蚀）而不是 EDT：只要"离障碍足够远"这个
    定性结论，不需要精确的欧氏距离；而且系统 scipy 与 venv 的 numpy 不兼容
    （ImportError: numpy.core.multiarray failed to import），不能依赖 scipy。
    """
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = np.array(grid.data, dtype=np.int16).reshape(h, w)
    free = data < 50
    ys, xs = np.mgrid[0:h, 0:w]
    gx = ox + (xs + 0.5) * res
    gy = oy + (ys + 0.5) * res
    free &= (gx - cx) ** 2 + (gy - cy) ** 2 <= radius * radius
    if not free.any():
        return None
    r_max = int(min_clear / res)          # 想达到的净距（格）
    best_mask = free.copy()
    best_r = 0
    for r in range(1, max(r_max + 1, 2)):
        # 八邻域腐蚀一步 = 切比雪夫距离 +1 格
        er = (free[:-2, :-2] & free[:-2, 1:-1] & free[:-2, 2:] &
              free[1:-1, :-2] & free[1:-1, 1:-1] & free[1:-1, 2:] &
              free[2:, :-2] & free[2:, 1:-1] & free[2:, 2:])
        free = er
        if free.sum() < 4:                # 剩下的已经是一个小坑，不值当
            break
        best_mask, best_r = free.copy(), r
        if r >= r_max:
            break
    if best_r == 0:
        return None
    ys_i, xs_i = np.nonzero(best_mask)
    px = float(gx[ys_i, xs_i].mean())
    py = float(gy[ys_i, xs_i].mean())
    return px, py, best_r * res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=30.0)
    ap.add_argument("--min-clear", type=float, default=1.5)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    proc, log = sc.launch("astar")
    rclpy.init()
    p = sc.Probe("park_open")
    try:
        p.wait_for(lambda: p.pose is not None and p.local_msgs > 0, timeout=25.0)
        sx, sy, syaw = p.pose
        grid = p.global_map if p.global_map is not None else sc.load_map_msg(sc.MAP_DIR)
        tgt = widest_point(grid, sx, sy, args.radius, args.min_clear)
        if tgt is None:
            print(f"车附近 {args.radius:.0f} m 内没有净距 ≥{args.min_clear:.1f} m 的开阔点")
            return 2
        gx, gy, clr = tgt
        print(f"车在 ({sx:.2f}, {sy:.2f})；最开阔点 ({gx:.2f}, {gy:.2f}) 净距 {clr:.2f} m"
              f"（直线 {math.hypot(gx - sx, gy - sy):.2f} m）")
        p.begin()
        p.send_goal(gx, gy, syaw)
        t0 = time.time()
        while time.time() - t0 < args.timeout:
            p.spin(0.2)
            if "GOAL_REACHED" in p.state_seen:
                break
            if p.final_state == "FAILED":
                print(f"失败了：{p.last_sm_msg}")
                return 1
        fx, fy, _ = p.pose
        print(f"结束状态 {p.final_state or '-'}（用时 {time.time() - t0:.1f} s）"
              f"，停在 ({fx:.2f}, {fy:.2f})，到点误差 {math.hypot(gx - fx, gy - fy):.3f} m")
    finally:
        p.destroy_node()
        rclpy.shutdown()
        sc.stop(proc)
    print(f"节点日志：{log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
