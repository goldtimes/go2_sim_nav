#!/usr/bin/env python3
"""复现并**量化**某一次"全局路径贴障碍、MPC 走到一半撞上"的现场。

用法（仿真栈已起、pnc_2d 三节点由本脚本起）：

    python3 test/sim/replay_goal.py --goal 14.27 2.87 --yaw 87.9 --local-type mpc
    python3 test/sim/replay_goal.py --goal 14.27 2.87 --yaw 87.9 \
            --set astar.path_prune_extra_margin=0        # 旧行为对照

它回答三个问题（而不是只报"撞了"）：
  1. **规划出来的路本身**还剩多少车身余量（车体矩形还能外扩多少米才碰）——
     用发布的全局图离线重算，不看规划器自己的自述；
  2. **实际跑出来的位姿轨迹**最窄走到多少余量、在路径的哪个里程（对比"车心净距"
     更能说明纠错空间）；
  3. 真的碰了吗？碰在哪（里程/时间）、那一刻控制器自报的横向误差 `cross_track`
     是多少 —— 这个数是"路径余量 vs 跟踪误差"谁大谁小的直接证据。

⚠ 与 `check_path_clearance.py` 的分工：那个**只规划不动车**（怀疑规划有问题时
   先跑它）；这个会**真的开车**，用来复现"跟踪阶段"的碰撞。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                      # noqa: E402
from nav_msgs.msg import OccupancyGrid                            # noqa: E402

import sim_common as sc                                           # noqa: E402
import check_path_clearance as cpc                                # noqa: E402


class ReplayProbe(sc.Probe):
    """比 Probe 多订阅一张**感知图**（MPC 避障实际用的那张），好在同一坐标上
    对比"两张图各报多少净距"。"""

    def __init__(self, name="p5_replay_probe"):
        super().__init__(name)
        self.local_map = None
        # ⚠ 感知图是 **VOLATILE** 发的（用 LATCHED 订会收不到，只报一条 QoS 不兼容）
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d",
                                 self.on_local_map, sc.LIVE)

    def on_local_map(self, m):
        self.local_map = m


class _P:
    """给 `check_path_clearance.path_footprint_scan` 用的最小 pose 壳（只要
    `.pose.position.x/.y`，和 nav_msgs/Path 里那几个字段同形）。"""

    class _Pos:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class _Pose:
        def __init__(self, x, y):
            self.position = _P._Pos(x, y)

    def __init__(self, x, y):
        self.pose = _P._Pose(x, y)


def arc_to(path, x, y):
    """(x, y) 在路径上的近似里程 + 有符号横向偏差（左+右-）。"""
    if len(path) < 2:
        return 0.0, 0.0
    best = (1.0e9, 0.0, 0.0)
    s = 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg * seg)))
        px, py = ax + dx * t, ay + dy * t
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            lat = ((x - px) * -dy + (y - py) * dx) / seg   # 左法向为正
            best = (d, s + t * seg, lat)
        s += seg
    return round(best[1], 3), round(best[2], 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--yaw", type=float, default=0.0, help="目标朝向 [°]")
    ap.add_argument("--local-type", default="mpc", help="mpc / heading_shim")
    ap.add_argument("--timeout", type=float, default=70.0)
    ap.add_argument("--wait-extra", type=float, default=4.0,
                    help="发目标前额外等待 [s]（等全局节点建完图/距离场）")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="开车前临时改 global_planner 参数（可重复），退出恢复")
    args = ap.parse_args()

    proc, log = sc.launch("astar", local_type=args.local_type)
    print(f"  节点日志：{log}")
    rclpy.init()
    p = ReplayProbe()
    try:
        # ★ 等"全局图"到位再发目标：map_server 是后来才 latch 上来的，提前发
        #   目标时 A* 手里没图 ⇒ 直接判 `目标不可达`（踩过：位姿/局部状态都有了
        #   就急着发，结果 0 点 + FAILED）。
        if not p.wait_for(lambda: p.pose is not None and p.local_msgs > 0 and
                          p.global_map is not None, timeout=25.0):
            print("没位姿/没有局部状态/没全局图：仿真、定位、perception、map_server"
                  "起了吗？")
            return 2
        sx, sy, syaw = p.pose
        gyaw = math.radians(args.yaw)
        print(f"车 ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.1f}°) → 目标 "
              f"({args.goal[0]:.2f}, {args.goal[1]:.2f}, {args.yaw:.1f}°) | "
              f"local.type={args.local_type}")
        p.spin(args.wait_extra)   # 等全局节点把图/距离场建完（否则 A* 手里没图）

        restore = []
        for kv in args.set:
            k, _, v = kv.partition("=")
            old = cpc.pget(p, "global_planner", k)
            if old is None:
                print(f"⚠ 没读到 {k} 的原值，跳过")
                continue
            cpc.pset(p, "global_planner", k, v)
            print(f"  临时设 {k}: {old} → {v}")
            restore.append((k, old))

        p.begin()
        p.send_goal(args.goal[0], args.goal[1], gyaw)
        t0 = time.time()
        while time.time() - t0 < args.timeout:
            p.spin(0.1)
            if p.state_seen & {"GOAL_REACHED", "FAILED"}:
                break
        p.spin(1.5)
        print(f"状态序列：{' → '.join(sorted(p.state_seen))}"
              f"（最终 {p.final_state}；{p.last_sm_msg}）")
        print(f"位姿 {len(p.traj)} 点 / 局部状态 {len(p.rows)} 条 / "
              f"用时 {time.time()-t0:.1f} s")

        if p.global_map is None:
            print("没有全局图，量不了净距")
            return 2
        gidx = sc.ObstacleIndex(p.global_map)
        lidx = sc.ObstacleIndex(p.local_map) if p.local_map else None
        path = p.first_path or []
        print(f"发布的全局路径：{len(path)} 点"
              + (f"，折点 {[f'({x:.2f},{y:.2f})' for x, y in path]}"
                 if 0 < len(path) <= 8 else ""))

        # ---- 1) 规划出来的路：车身余量 ----
        if len(path) >= 2:
            poses = [_P(x, y) for x, y in path]
            scan = cpc.path_footprint_scan(p.global_map, gidx, poses)
            if scan:
                n_hit, worst, mw, mf = scan
                L = sum(math.hypot(path[i + 1][0] - path[i][0],
                                   path[i + 1][1] - path[i][1])
                        for i in range(len(path) - 1))
                print(f"  规划路径：{L:.2f} m | 车心最小净距 {worst[0]:.3f} m "
                      f"@里程 {worst[3]:.2f} m | ★ 车身余量全程最窄 "
                      f"{mw[0]*100:.0f} cm / 1 m 之后 {mf[0]*100:.0f} cm "
                      f"@里程 {mf[2]:.2f} m")
                if lidx is not None:
                    print(f"    同一处：全局图净距 "
                          f"{gidx.clearance(mf[1][0], mf[1][1]):.3f} m / "
                          f"感知图净距 {lidx.clearance(mf[1][0], mf[1][1]):.3f}"
                          f" m（MPC 硬下界 0.25 m）")

        # ---- 2) 跑出来的轨迹：最窄余量 + 真的碰了吗 ----
        if not p.traj:
            print("没有轨迹采样")
            return 3
        t0i = p.traj[0][0]
        worst = (1.0e9, None)
        hit_soft = None      # margin = 0（含全局图的 0.05 m 膨胀）
        hit_hard = None      # margin = -0.05（扣掉膨胀 ⇒ 近似"真撞"）
        for (t, x, y, yaw) in p.traj[1:]:
            m = cpc.margin_at(p.global_map, x, y, yaw, hi=0.4)
            if m < worst[0]:
                worst = (m, (t - t0i, x, y, yaw))
            if hit_soft is None and cpc.footprint_hits(p.global_map, x, y, yaw,
                                                       margin=0.0)[0]:
                hit_soft = (t - t0i, x, y, yaw)
            if hit_hard is None and cpc.footprint_hits(p.global_map, x, y, yaw,
                                                       margin=-0.05)[0]:
                hit_hard = (t - t0i, x, y, yaw)
        print(f"  实跑轨迹：最窄车身余量 {worst[0]*100:.0f} cm "
              f"@ t={worst[1][0]:.1f}s ({worst[1][1]:.2f}, {worst[1][2]:.2f})"
              f" | 里程 {arc_to(path, worst[1][1], worst[1][2])[0]:.2f} m"
              f" / 横向 {arc_to(path, worst[1][1], worst[1][2])[1]*100:+.0f} cm")
        for tag, h in (("按发布的全局图（含 0.05 m 膨胀）", hit_soft),
                       ("扣掉图膨胀（近似真撞）", hit_hard)):
            if h is None:
                print(f"  {tag}：未发生")
            else:
                s, lat = arc_to(path, h[1], h[2])
                print(f"  ★ {tag}：t={h[0]:.1f}s 在 ({h[1]:.2f}, {h[2]:.2f})，"
                      f"里程 {s:.2f} m，横向 {lat*100:+.0f} cm")

        # ---- 3) 控制器自报的横向误差（与余量直接对比）----
        if p.rows:
            xs = [(r[0] - t0i, r[8], r[9]) for r in p.rows]   # t, cross_track, progress
            mx = max(xs, key=lambda v: abs(v[1]))
            last = xs[-1]
            print(f"  控制器自报横向误差：最大 {mx[1]*100:+.0f} cm @t={mx[0]:.1f}s"
                  f"，末值 {last[1]*100:+.0f} cm（进度 {last[2]*100:.0f}%）")
        for k, v in reversed(restore):
            cpc.pset(p, "global_planner", k, v)
        if restore:
            print("（已恢复参数：" + ", ".join(f"{k}={v}" for k, v in restore) + "）")
        return 0
    finally:
        rclpy.shutdown()
        sc.stop(proc)


if __name__ == "__main__":
    sys.exit(main())
