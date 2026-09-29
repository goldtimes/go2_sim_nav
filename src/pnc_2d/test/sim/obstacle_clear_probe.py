#!/usr/bin/env python3
"""撤障后**感知到底清没清**：把箱子放在车前方 2 m，出现后再删掉，逐秒看两层图。

用户 2026-09-29 报："障碍物消失了，感知也没有清除障碍物"（RViz 里留着一坨假障碍）。
本脚本把这件事量化，并**分清是哪一层的锅**：

  · `occupancy_2d`（感知发布的 2D 占据层，0.10 m）
      - 箱矩形内占据格数：障碍在 → 几十格；删掉后应回落到 0（或只剩背景结构）
  · `esdf_2d`（PointCloud2，intensity = 到最近障碍的距离）
      - 箱心处的距离值：障碍在 → 0~0.5 m；删掉后应回到"到别的东西的距离"

同时报告**背景基线**（箱矩形外、1.5~3 m 环形内的占据格数），这样"回到基线"才可信。

用法（仿真 + 定位 + perception 在跑；本脚本不起栈、不动车、自己起 gz 服务桥）：

    python3 test/sim/obstacle_clear_probe.py                 # 前方 2 m，default
    python3 test/sim/obstacle_clear_probe.py --d 1.5 --settle 20
    python3 test/sim/obstacle_clear_probe.py --solid          # 让箱子参与碰撞（默认 ghost）

⚠ 默认 ghost（collide_bitmask=0）：我们要测的是**感知的清除**，不希望箱子被车顶开。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_avoidance as ta                                       # noqa: E402
from near_field_probe import Probe                                 # noqa: E402

OCC_THRESH = 50


def count_occ_rect(m, cx, cy, sx, sy):
    """矩形内占据格数（轴对齐；与 near_field_probe 同口径）"""
    if m is None:
        return 0
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    hx, hy = sx / 2 + res / 2, sy / 2 + res / 2
    n = 0
    for j in range(int((cy - hy - oy) // res), int((cy + hy - oy) // res) + 1):
        for i in range(int((cx - hx - ox) // res), int((cx + hx - ox) // res) + 1):
            if 0 <= i < m.info.width and 0 <= j < m.info.height:
                if m.data[j * m.info.width + i] >= OCC_THRESH:
                    n += 1
    return n


def count_occ_ring(m, cx, cy, r0, r1, skip_rect):
    """1.5~3 m 环带内的占据格数（背景基线；排掉箱矩形本身）"""
    if m is None:
        return 0
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    n = 0
    i0 = max(0, int((cx - r1 - ox) // res))
    i1 = min(m.info.width - 1, int((cx + r1 - ox) // res))
    j0 = max(0, int((cy - r1 - oy) // res))
    j1 = min(m.info.height - 1, int((cy + r1 - oy) // res))
    sx, sy = skip_rect
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            x = ox + (i + 0.5) * res
            y = oy + (j + 0.5) * res
            d = math.hypot(x - cx, y - cy)
            if not (r0 <= d <= r1):
                continue
            if abs(x - cx) < sx / 2 + res and abs(y - cy) < sy / 2 + res:
                continue
            if m.data[j * m.info.width + i] >= OCC_THRESH:
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world_demo")
    ap.add_argument("--d", type=float, default=2.0, help="箱子放在车前方多远 [m]")
    ap.add_argument("--sizes", default="0.6,0.6,0.8")
    ap.add_argument("--settle", type=float, default=25.0,
                    help="撤障后最多观察多久 [s]")
    ap.add_argument("--solid", action="store_true", help="箱子参与物理碰撞（默认 ghost）")
    args = ap.parse_args()

    sizes = [float(v) for v in args.sizes.split(",")]
    bridge = ta.start_bridge(args.world)
    rclpy.init()
    p = Probe(args.world, "clear_probe_box")
    try:
        if not p.cli_create.wait_for_service(timeout_sec=10.0):
            print("没有 /world/%s/create —— 服务桥没起来？" % args.world)
            return 2
        end = time.time() + 25.0
        while time.time() < end and (p.pose is None or p.map is None):
            p.spin(0.2)
        if p.pose is None or p.map is None:
            print("缺位姿或局部图：仿真/定位/perception 都在跑吗？")
            return 2
        x, y, yaw = p.pose
        bx = x + args.d * math.cos(yaw)
        by = y + args.d * math.sin(yaw)
        print(f"雷达位姿 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°) | "
              f"箱子放 ({bx:.2f}, {by:.2f}) 尺寸 {sizes[0]}×{sizes[1]}×{sizes[2]}"
              f" | 局部图 {p.map.info.width}×{p.map.info.height}")

        base_box = count_occ_rect(p.map, bx, by, sizes[0], sizes[1])
        base_ring = count_occ_ring(p.map, x, y, 1.5, 3.0, (sizes[0], sizes[1]))
        base_cloud = p.cloud_rect_count(bx, by, sizes[0], sizes[1])
        print(f"\n基线（还没放箱子）：箱矩形内 {base_box} 格 | 1.5~3 m 环带 "
              f"{base_ring} 格 | 上游点云在箱矩形内 {base_cloud} 点")

        if not p.spawn(bx, by, sizes, ghost=not args.solid):
            print("⚠ 生成箱子失败（服务桥？）")
            return 3
        print("\n[放箱子] 逐秒观察（箱矩形内占据格数 / 箱心 ESDF / 环带基线）：")
        t0 = time.time()
        seen = 0
        while time.time() - t0 < 15.0:
            p.spin(1.0)
            box_now = count_occ_rect(p.map, bx, by, sizes[0], sizes[1])
            e = p.esdf_at(bx, by)
            if box_now > 5:
                seen = box_now
                print(f"  +{time.time()-t0:5.1f}s  箱内 {box_now:3d} 格 | "
                      f"ESDF {(e if e is not None else float('nan')):.2f} m")
                break
            print(f"  +{time.time()-t0:5.1f}s  箱内 {box_now:3d} 格 | "
                  f"ESDF {(e if e is not None else float('nan')):.2f} m（还没进图）")
        if seen == 0:
            print("  ⚠ 15 s 内箱子没进图 —— 位置/高度带/topic 有问题，先查这个")

        p.remove()
        print("\n[撤障] 逐秒观察（应回落到基线）：")
        t0 = time.time()
        first_clean = None
        while time.time() - t0 < args.settle:
            p.spin(1.0)
            box_now = count_occ_rect(p.map, bx, by, sizes[0], sizes[1])
            ring_now = count_occ_ring(p.map, x, y, 1.5, 3.0,
                                      (sizes[0], sizes[1]))
            cloud_now = p.cloud_rect_count(bx, by, sizes[0], sizes[1])
            e = p.esdf_at(bx, by)
            if box_now <= max(1, base_box) and first_clean is None:
                first_clean = time.time() - t0
            print(f"  +{time.time()-t0:5.1f}s  箱内 {box_now:3d} 格（基线 "
                  f"{base_box}）| 环带 {ring_now:4d} 格 | 点云 {cloud_now:4d} 点"
                  f"（基线 {base_cloud}）| ESDF "
                  f"{(e if e is not None else float('nan')):.2f} m | 图消息 {p.n_map}")
            if first_clean is not None and time.time() - t0 > first_clean + 3.0:
                break
        if first_clean is None:
            print(f"\n★ 阶段一结论：**撤障后 {args.settle:.0f} s 内没清干净** —— "
                  f"箱矩形内残留占据格 ⇒ 感知没把障碍从图上抹掉。")
        else:
            print(f"\n★ 阶段一结论：撤障后约 {first_clean:.1f} s 回到基线（清干净了）"
                  f"—— 若 RViz 里仍看到残影，要查的是**可视化/订阅端**。")

        # ---------------- 阶段二：调清图服务（恢复行为 clear_map 干的就是这件事）
        print(f"\n[阶段二] 调 /grid_map/clear_map（std_srvs/Trigger）—— 看是否立刻干净：")
        from std_srvs.srv import Trigger
        cli = p.create_client(Trigger, "/grid_map/clear_map")
        if not cli.wait_for_service(timeout_sec=5.0):
            print("  ⚠ 没有 /grid_map/clear_map 服务（perception 没重编/服务名不对）"
                  " ⇒ 恢复行为 clear_map 会静默无效（管理器会 WARN）")
        else:
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(p, fut, timeout_sec=5.0)
            ok = fut.result() if fut.done() else None
            print(f"  服务返回：success={getattr(ok, 'success', None)} "
                  f"msg={getattr(ok, 'message', '（无响应）')}")
            t0 = time.time()
            while time.time() - t0 < 6.0:
                p.spin(1.0)
                box_now = count_occ_rect(p.map, bx, by, sizes[0], sizes[1])
                ring_now = count_occ_ring(p.map, x, y, 1.5, 3.0,
                                          (sizes[0], sizes[1]))
                print(f"  +{time.time()-t0:4.1f}s  箱内 {box_now:3d} 格 | "
                      f"环带 {ring_now:4d} 格（基线 {base_ring}）")
                if time.time() - t0 > 3.0:
                    break
            print("  读法：箱内格数应该**一下子**落到 0；环带不会掉到 0（那是周围"
                  "真实结构，会被重新观测回来）—— 这才叫“清了假障碍、没丢真障碍”。")
        return 0
    finally:
        try:
            p.destroy_node()
            rclpy.shutdown()
        except Exception:                                          # noqa: BLE001
            pass
        ta.stop_proc(bridge)


if __name__ == "__main__":
    sys.exit(main())
