#!/usr/bin/env python3
"""M5.0 判别工具：**直接给局部节点飞一条我指定的直线**，量稳态横向偏置。

为什么需要它（2026-09-28 晚）：
  · 走"发目标 → manager → 全局规划 → 局部"这条链时，**全局图可能拒绝起点**
    （实测车处全局净距 0.071 m ⇒ `START_FOOTPRINT`，15 个候选目标 12 个被拒）；
  · 而且那条链里有太多变量（A*/MINCO/走廊/剖面）。本工具**只留跟踪回路**：
    手搭一条直线路径，经 `~/follow_path` action 交给局部节点（**安全网照旧生效**：
    footprint / ESDF / 解后复核都在），拿 `feedback.cross_track_m` 直接量偏置。

★ 已验证的用法（M5.0 第 4 次量测）：
    --turn 0    → 路径与当前车头同向（不用转身）⇒ 稳态偏置 **+0.4 cm**
    --turn 180  → 要先原地转 180°             ⇒ 稳态偏置 **−10.4 cm**
  即"~10 cm 的场景相关横向偏置由**起步的原地转向**触发，与障碍无关"。
  ⇒ 本工具就是"偏置 < 2 cm 场景"与"偏置 ~10 cm 场景"的**可复现生成器 + 判定器**。

★ 待判别的两个机制候选（用 `--dist 6` 区分）：
   ① 只是收敛慢（MPC 无积分项 + 短时域）⇒ 拉长距离偏置会继续衰减；
   ② 真的稳态误差 ⇒ 偏置停在某个值不动。

用法（仿真 + 定位 + perception + **pnc_2d 三节点**在跑；本脚本**不起栈**）：
    python3 test/sim/track_probe.py --dist 2.5 --turn 0
    python3 test/sim/track_probe.py --dist 2.5 --turn 180
    python3 test/sim/track_probe.py --dist 6 --turn 180 --csv /tmp/t6.csv
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HERE = "/home/gmd/r41_ws/src/pnc_2d/test/sim"
sys.path.insert(0, HERE)
import sim_common as sc                                          # noqa: E402

from pnc_2d.action import FollowPath                              # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)


class N(rclpy.node.Node):
    def __init__(self):
        super().__init__("track_probe")
        self.gmap = self.lmap = self.pose = None
        self.traj = []          # 高频位姿 (t, x, y, yaw)：★ 顺滑度只能用位姿算
        self.create_subscription(OccupancyGrid, "/global_map/occupancy", self.og,
                                 LATCHED)
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d", self.ol,
                                 LIVE)
        self.create_subscription(Odometry, "/lightning/perception/pose", self.op,
                                 LIVE)
        self.ac = ActionClient(self, FollowPath, "/local_planner/follow_path")

    def og(self, m):
        self.gmap = m

    def ol(self, m):
        self.lmap = m

    def op(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.traj.append((time.time(), self.pose[0], self.pose[1], yaw))


def pose_omega(traj, t_lo, t_hi, min_dt=0.05):
    """**位姿差分**算角速率序列 [rad/s]（不用 twist：低速时是噪声）。

    ★ 这是本项目钉过的规矩（见 `sim_common.py` 顶部三条铁律）：顺滑度/速度类指标
      一律用位姿差分；用指令或 twist 会把"MPC 每周期重规划的噪声"当成车在摆。
    """
    seg = [p for p in traj if t_lo <= p[0] <= t_hi]
    out = []
    for a, b in zip(seg, seg[1:]):
        dt = b[0] - a[0]
        if dt < min_dt:
            continue
        out.append(sc.wrap_pi(b[3] - a[3]) / dt)
    return out


def reversals(w, tol=0.02):
    """角速率**反向次数**（忽略 |w| < tol 的零区，避免把噪声当反向）"""
    n, prev = 0, 0
    for v in w:
        s = 1 if v > tol else (-1 if v < -tol else 0)
        if s and prev and s != prev:
            n += 1
        if s:
            prev = s
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=2.5, help="直线长度 [m]")
    ap.add_argument("--turn", type=float, default=0.0,
                    help="路径方向相对**当前车头**转多少度（180 = 要先掉头）")
    ap.add_argument("--speed", type=float, default=0.20, help="速度上限 [m/s]")
    ap.add_argument("--min-clear", type=float, default=0.45,
                    help="路径全程要求的最小净距 [m]（两张图都查）")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    if len(sc.running_pnc_nodes()) < 3:
        print("✗ 需要**已经在跑的** pnc_2d 三节点（本脚本不起栈）")
        return 2
    if not rclpy.ok():
        rclpy.init()
    n = N()
    try:
        end = time.time() + 25
        while time.time() < end and not (n.gmap and n.lmap and n.pose):
            rclpy.spin_once(n, timeout_sec=0.05)
        if not (n.gmap and n.lmap and n.pose):
            print(f"✗ 缺数据：global_map={n.gmap is not None} "
                  f"local_map={n.lmap is not None} pose={n.pose is not None}")
            return 2
        cx, cy, cyaw = n.pose
        gidx, lidx = sc.ObstacleIndex(n.gmap), sc.ObstacleIndex(n.lmap)
        a = cyaw + math.radians(args.turn)
        print(f"车 ({cx:.3f},{cy:.3f},{math.degrees(cyaw):.1f}°)  净距 全局 "
              f"{gidx.clearance(cx,cy):.2f} / 感知 {lidx.clearance(cx,cy):.2f} m")
        print(f"路径方向 {math.degrees(a):.1f}°（= 车头 {'+' if args.turn >= 0 else ''}"
              f"{args.turn:.0f}°），长 {args.dist} m，速度上限 {args.speed} m/s")

        # ---- 先查这条路走不走得通（两张图都查；不通过就明说，别让车撞）----
        s, bad = 0.0, None
        while s < args.dist:
            s += 0.10
            g = gidx.clearance(cx + s * math.cos(a), cy + s * math.sin(a))
            l = lidx.clearance(cx + s * math.cos(a), cy + s * math.sin(a))
            if min(g, l) < args.min_clear:
                bad = (s, g, l)
                break
        if bad:
            print(f"✗ 走到 {bad[0]:.2f} m 处净距不足（全局 {bad[1]:.2f} / "
                  f"感知 {bad[2]:.2f} m < {args.min_clear}）——换个方向/缩短距离")
            return 3

        path = Path()
        path.header.frame_id = "map"
        k = 0
        while k * 0.10 <= args.dist:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = cx + k * 0.10 * math.cos(a)
            ps.pose.position.y = cy + k * 0.10 * math.sin(a)
            ps.pose.orientation.z = math.sin(0.5 * a)
            ps.pose.orientation.w = math.cos(0.5 * a)
            path.poses.append(ps)
            k += 1
        if not n.ac.wait_for_server(timeout_sec=5.0):
            print("✗ /local_planner/follow_path 没起来")
            return 3
        goal = FollowPath.Goal()
        goal.header.frame_id = "map"
        goal.path = path
        goal.corridor_width = []          # 空 = 自由跟踪
        goal.speed_limit = args.speed
        goal.strict_corridor = False
        goal.traj_valid = False           # 没有上游剖面 ⇒ 局部自算限速
        goal.traj_note = f"track_probe turn={args.turn:.0f}"

        rows = []
        t0 = time.time()

        def on_fb(m):
            f = m.feedback
            rows.append((time.time() - t0, f.progress, f.cross_track_m, f.cmd_v,
                         f.cmd_w, f.status_name, f.blocked, f.remaining_m))

        send = n.ac.send_goal_async(goal, feedback_callback=on_fb)
        end = time.time() + 10
        while time.time() < end and not send.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if not send.done() or not send.result().accepted:
            print("✗ 目标被拒")
            return 4
        gh = send.result()
        res_f = gh.get_result_async()
        end = time.time() + args.timeout
        while time.time() < end and not res_f.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if not res_f.done():
            print("⚠ 超时，取消")
            gh.cancel_goal_async()
            return 5
        r = res_f.result().result
        print(f"\n结果 {r.status_name} reached={r.goal_reached} blocked={r.blocked} "
              f"| {r.message}")
        print(f"  {r.elapsed_s:.1f} s / 走了 {r.traveled_m:.2f} m / "
              f"末横向 {r.final_cross_track_m*100:+.1f} cm")

        mv = [x for x in rows if abs(x[3]) > 0.05]
        if not mv:
            print("⚠ 车没动（cmd_v 恒 ~0）；前 10 条反馈：")
            for x in rows[:10]:
                print(f"    {x[0]:6.2f}s v {x[3]:5.2f} w {x[4]:+5.2f} {x[5]}")
            return 6
        t_mv = mv[0][0]
        st = [x for x in mv if x[0] >= t_mv + 0.5 * (mv[-1][0] - mv[0][0])] or mv
        bias = sum(x[2] for x in st) / len(st)
        # ---- "摆不摆"（R12：只看顺滑度必然把车调成贴着墙滑行，必须与偏置一起看）----
        w = [x[4] for x in mv]
        rms_w = math.sqrt(sum(v * v for v in w) / len(w))
        rev, prev = 0, 0
        for v in w:
            s = 1 if v > 0.01 else (-1 if v < -0.01 else 0)
            if s and prev and s != prev:
                rev += 1
            if s:
                prev = s
        print(f"\n横向偏差（{len(mv)} 条移动中反馈）：")
        print(f"  全程 max {max(abs(x[2]) for x in mv)*100:5.1f} cm / "
              f"均值 {sum(x[2] for x in mv)/len(mv)*100:+5.1f} cm")
        print(f"  后半段(=稳态) max {max(abs(x[2]) for x in st)*100:5.1f} cm / "
              f"**均值偏置 {bias*100:+.1f} cm**（{len(st)} 条）")
        print(f"  ★ 稳态偏置 {bias*100:+.1f} cm | 指令：RMS|w| {rms_w:.3f} / "
              f"max|w| {max(abs(v) for v in w):.3f} / 反向 {rev} 次")
        # ---- 顺滑度（**位姿**口径；与 M4.5 的表可比）----
        # ⚠ 时间基准别弄混：rows 里是"相对 t0"，n.traj 是绝对 time.time()
        t_st = st[0][0] + t0
        t_en = mv[-1][0] + t0
        wo = pose_omega(n.traj, t_st, t_en)
        if len(wo) >= 5:
            rms_wo = math.sqrt(sum(v * v for v in wo) / len(wo))
            print(f"  ★ 位姿口径：RMS|ω| {rms_wo:.3f} rad/s / max|ω| "
                  f"{max(abs(v) for v in wo):.3f} / **ω 反向 {reversals(wo)} 次**"
                  f"（{len(wo)} 个采样、稳态 {t_en - t_st:.1f} s）")
        else:
            print(f"  ⚠ 位姿样本太少（{len(wo)}），位姿口径的 ω 不可用")
        # 前后半段各算一次：用来判"是不是还没收敛"
        h = len(st) // 2
        if h >= 5:
            b1 = sum(x[2] for x in st[:h]) / h
            b2 = sum(x[2] for x in st[h:]) / len(st[h:])
            print(f"  稳态前半 {b1*100:+.1f} cm → 后半 {b2*100:+.1f} cm"
                  f"（变化 {abs(b1)-abs(b2):+.1f} cm）"
                  f"{'  ← 还在收敛' if abs(b1)-abs(b2) > 0.01 else '  ← 已停住'}")
        print("  逐 15 条：")
        for x in rows[::15]:
            print(f"    {x[0]:6.2f}s prog {x[1]*100:5.1f}% xtrack {x[2]*100:+6.1f}cm "
                  f"v {x[3]:5.2f} w {x[4]:+5.2f} 剩 {x[7]*100:6.1f}cm {x[5]}")
        if args.csv:
            import csv
            with open(args.csv, "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["t", "progress", "cross_track_m", "cmd_v", "cmd_w",
                            "status", "blocked", "remaining_m"])
                w.writerows(rows)
            print(f"  CSV → {args.csv}（稳态偏置 {bias*100:+.1f} cm）")
        return 0
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
