#!/usr/bin/env python3
"""区域层（禁行区 / 限速区）仿真验收。

在**已经跑起来的仿真栈**上跑（gazebo + lightning 定位 + perception + map_server），
本脚本起 pnc_2d 三节点，自己往 `/global_map/zones` 发区域几何，量"局部到底遵守不遵守"。

    python3 test/sim/test_zones.py            # 三段：禁行带挡路 / 挪开能过 / 限速区降速

## 为什么由测试自己发区域（而不是改地图文件 + LoadMap）

测试要的是"**区域在路径算完之后才出现**"这个场景（现场最常见的危险情形：区域刚画上、
或地图没重载）。用 `map_server.LoadMap` 去改站点文件既侵入用户的地图数据，又会把禁行区
一起烧进全局图 —— 那样全局规划自己就绕开了，反而**测不到局部**。

自己发区域（latched，与 map_server 同一个话题）则同时压到两个消费者：
  · 局部：融合进栅格 + 距离场 ⇒ 不能进区（本测试主要验这个）；
  · 全局：路径上的限速区 ⇒ 任务限速（第 3 段验）。
map_server 只在加载/参数变更时发布，所以测试发布之后不会被它盖掉（稳态下没人再发）。

## 判据

1. **禁行带**：轨迹到区域多边形的**最小净距 > 0**（没进区），且留下可诊断的结束原因。
2. **反证**：同一条路径、同一个目标，把区域挪开 ⇒ **能到**（证明是区域起作用，
   不是别的东西挡的 —— 没有这一步，"没进去"可能是碰巧）。
3. **限速区**：进区**之前**速度就已经 ≤ 限速（前瞻生效），区内速度 ≤ 限速，
   出区后恢复。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Point32, Polygon
from pnc_2d.msg import Zone, ZoneArray
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import sim_common as sc
from sim_common import chk, load_limits, signed_offset, summary


def latched() -> QoSProfile:
    # 与 map_server / 局部节点的订阅一致（latched + reliable + depth 1）
    return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def make_zone(name: str, kind: str, value: float, pts) -> Zone:
    z = Zone()
    z.name = name
    z.type = kind
    z.value = value
    poly = Polygon()
    for x, y in pts:
        poly.points.append(Point32(x=float(x), y=float(y), z=0.0))
    z.polygon = poly
    return z


def band(cx, cy, ux, uy, half_len, half_thick):
    """垂直于行驶方向的一条带（u = 行驶方向单位向量）

    横向（±half_len）跨度要大：否则 MPC 会从旁边绕过去，而"绕过去"也是正确行为，
    就验不出"不能进区"了。
    """
    nx, ny = -uy, ux  # 左法向
    corners = []
    for s_thick, s_len in ((-1, -1), (-1, 1), (1, 1), (1, -1)):
        corners.append((cx + s_thick * half_thick * ux + s_len * half_len * nx,
                        cy + s_thick * half_thick * uy + s_len * half_len * ny))
    return corners


def box(cx, cy, ux, uy, half_len, half_thick):
    """沿行驶方向的矩形（限速区用：盖住一段路面）"""
    nx, ny = -uy, ux
    corners = []
    for s_thick, s_len in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((cx + s_thick * half_len * ux + s_len * half_thick * nx,
                        cy + s_thick * half_len * uy + s_len * half_thick * ny))
    return corners


def dist_to_poly(x, y, pts) -> float:
    """点到多边形边界的最近距离（<0 表示在多边形内，用射线法判）"""
    inside = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    best = 1e9
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = max(1e-12, dx * dx + dy * dy)
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return -best if inside else best


def min_dist_traj(traj, poly) -> float:
    return min((dist_to_poly(q[1], q[2], poly) for q in traj), default=1e9)


def run_phase(p, goal, timeout, label):
    """发一个目标跑到**终止态**（到达 / FAILED），返回 (是否到达, 轨迹)

    ⚠ 不能"一看到 RECOVERING 就走"：状态机会在 RECOVERING 里重试恢复（计数 1/2、2/2），
      期间**不接受新目标**（踩过：下一个阶段的目标被静默拒绕，看着像"区域挪开也没用"）。
    """
    p.begin()
    p.send_goal(goal[0], goal[1], goal[2])
    t0 = time.time()
    reached = None
    last_state = ""
    while time.time() - t0 < timeout:
        p.spin(0.1)
        if p.final_state != last_state and p.final_state:
            last_state = p.final_state
            print(f"    [{label}] → {last_state}：{p.last_sm_msg[:70]}")
        if "GOAL_REACHED" in p.state_seen:
            reached = time.time() - t0
            break
        if p.final_state == "FAILED":
            break
    p.spin(1.0)
    print(f"  [{label}] 到达={bool(reached)} 用时={reached and round(reached, 1)} s "
          f"| 终止态 {p.final_state}")
    return reached, list(p.traj)


def pick_goal(obstacle, pose, dist):
    """从**当前位置**挑一个前方净距 ≥0.5 m 的直线目标。

    阶梯放宽（先放宽转角、再缩短距离）：车常常停在"没有长直线"的地方（贴着墙/
    区域旁边），这时不该直接报"先挪车"。
    返回 (gx, gy, gyaw, gd)；挑不出来就返回 None。
    """
    sx, sy, syaw = pose
    for max_turn, d in ((45.0, dist), (90.0, dist), (135.0, dist),
                        (90.0, 0.7 * dist), (135.0, 0.6 * dist)):
        cands = [c for c in sc.goal_candidates(obstacle, (sx, sy), d,
                                               heading=syaw, max_turn=max_turn)
                 if c[3] >= 0.5]
        if cands:
            gx, gy, gd, _dmin, _ang = cands[0]
            return gx, gy, math.atan2(gy - sy, gx - sx), gd
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=4.5, help="目标距离 [m]")
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--csv", default="", help="把三段轨迹写成 csv（排查用）")
    ap.add_argument("--keep-log", action="store_true")
    args = ap.parse_args()

    v_max, w_max, _ = load_limits()
    print(f"配置：v_max={v_max} w_max={w_max}")

    proc, log = sc.launch("astar")
    rclpy.init()
    p = sc.Probe("p5_zone_probe")
    # 测试自己发区域（见文件头：这样才能测到"区域后画"的危险场景）
    pub = p.create_publisher(ZoneArray, "/global_map/zones", latched())

    def publish(zones, inflate=0.05):
        m = ZoneArray()
        m.header.stamp = p.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.inflate = inflate
        m.zones = zones
        pub.publish(m)
        p.spin(0.4)

    try:
        p.wait_for(lambda: p.pose is not None and p.local_msgs > 0, timeout=20.0)
        if p.pose is None:
            print("没有位姿：仿真/定位没起？")
            return 2
        sx, sy, syaw = p.pose
        obstacle = sc.ObstacleIndex(
            p.global_map if p.global_map is not None else sc.load_map_msg(sc.MAP_DIR))
        print(f"起点 ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.0f}°) | "
              f"起点净距 {obstacle.clearance(sx, sy):.2f} m")
        # 阶梯找目标：车常停在一个"没有长直线"的地方（贴着墙/区域旁边），
        # 这时不该直接报"先挪车" —— 先放宽转角、再缩短距离。
        cands = []
        for max_turn, dist in ((45.0, args.dist), (90.0, args.dist),
                              (135.0, args.dist), (90.0, 0.7 * args.dist),
                              (135.0, 0.6 * args.dist)):
            cands = [c for c in sc.goal_candidates(obstacle, (sx, sy), dist,
                                                  heading=syaw, max_turn=max_turn)
                     if c[3] >= 0.5]
            if cands:
                print(f"  目标搜索：转角 ±{max_turn:.0f}°、距离 {dist:.1f} m "
                      f"⇒ {len(cands)} 个候选")
                break
        if not cands:
            print(f"起点净距 {obstacle.clearance(sx, sy):.2f} m；"
                  f"放宽到 ±135°/2.7 m 仍找不到净距 ≥0.5 m 的直线，先挪一下车")
            return 2
        gx, gy, gd, dmin, ang = cands[0]
        ux, uy = (gx - sx) / gd, (gy - sy) / gd
        gyaw = math.atan2(uy, ux)
        print(f"起点 ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.0f}°) | 目标 "
              f"({gx:.2f}, {gy:.2f})：{gd:.2f} m，直线最小净距 {dmin:.2f} m")
        goal = (gx, gy, gyaw)

        # ================= 段 1：禁行带横在路中间 =================
        s_band = min(2.5, gd * 0.55)
        bx, by = sx + s_band * ux, sy + s_band * uy
        band_pts = band(bx, by, ux, uy, half_len=3.0, half_thick=0.4)
        print(f"\n---- 段 1：禁行带（中心距起点 {s_band:.2f} m，横向 ±3 m）----")
        publish([make_zone("band", "forbidden", 0.0, band_pts)])
        reached1, traj1 = run_phase(p, goal, args.timeout, "带区")
        d1 = min_dist_traj(traj1, band_pts)
        print(f"  轨迹到禁行带的最小净距 {d1:+.3f} m（<0 = 进去了）")
        chk("[Z1] 有区域时**没有**进入禁行带（最小净距 > 0）", d1 > 0.0,
            f"{d1:+.3f} m")
        chk("[Z2] 没进区且有可诊断的结束原因", d1 > 0.0 and
            (p.final_state in ("FAILED", "RECOVERING") or not reached1),
            f"状态 {p.final_state}｜{p.last_sm_msg[:60]}")

        # ================= 段 2：反证——把区域挪开，同一目标要能到 =================
        print("\n---- 段 2：反证（把禁行带挪到 8 m 以外）----")
        far_x, far_y = sx + 8.0 * (-uy), sy + 8.0 * (ux)
        publish([make_zone("band", "forbidden", 0.0,
                           band(far_x, far_y, ux, uy, 3.0, 0.4))])
        reached2, traj2 = run_phase(p, goal, args.timeout, "无区")
        fx, fy, _ = p.pose
        d_goal2 = math.hypot(gx - fx, gy - fy)
        print(f"  到点误差 {d_goal2:.3f} m")
        chk("[Z3] ★ 反证：区域挪开后同一目标能到（证明是区域在起作用）",
            reached2 is not None and d_goal2 <= 0.15,
            f"到达={bool(reached2)}，误差 {d_goal2:.3f} m")

        # ================= 段 3：限速区 =================
        print("\n---- 段 3：限速区 0.15 m/s ----")
        # ★ 本段必须**从机器人当前位置重新选路**（而不是继续用起点 + u）。
        #   段 2 修好之后（恢复行为真能重规划了）车子会真的开到终点 —— 于是段 3
        #   开始时它已经不在起点了：再按"起点 + 1.7 m × u"去放限速区，区就落在
        #   屁股后面 ⇒ 轨迹里根本没有"进区前/区内"的样本，Z4/Z5 报 None
        #   （实测踩过：区域明明生效，指标全 None，看着像没实现）。
        sx3, sy3, syaw3 = p.pose
        g3 = pick_goal(obstacle, (sx3, sy3, syaw3), args.dist)
        if g3 is None:
            print(f"  段 3 选不出净距 ≥0.5 m 的前方直线（当前净距 "
                  f"{obstacle.clearance(sx3, sy3):.2f} m），跳过")
            return 2
        gx3, gy3, gyaw3, gd3 = g3
        ux, uy = (gx3 - sx3) / gd3, (gy3 - sy3) / gd3
        print(f"  重新瞄准：({sx3:.2f}, {sy3:.2f}, {math.degrees(syaw3):.0f}°) → "
              f"({gx3:.2f}, {gy3:.2f})：{gd3:.2f} m")
        lim = 0.15
        s_slow = min(2.0, gd3 * 0.4)
        cx, cy = sx3 + s_slow * ux, sy3 + s_slow * uy
        slow_pts = box(cx, cy, ux, uy, half_len=1.0, half_thick=1.5)
        publish([make_zone("slow", "speed_limit", lim, slow_pts)])
        # 目标放到限速区之后 2.5 m（区后沿在 s_slow+1.0）⇒ 区后还剩 ~1.5 m
        # 巡航段，能真正看出"出区后恢复"；又不在终点的刹车段里（否则量到的是
        # 减速，不是限速）。前面的 pick_goal 用 5.5 m 保证这段直线净距被验过。
        far_goal = (sx3 + (s_slow + 2.5) * ux, sy3 + (s_slow + 2.5) * uy)
        reached3, traj3 = run_phase(p, (far_goal[0], far_goal[1], gyaw3),
                                    args.timeout, "限速")

        # 分区统计速度（位姿差分，独立于控制器自报）
        def avg_speed(seg):
            if len(seg) < 2:
                return None
            d = sum(math.hypot(seg[i + 1][1] - seg[i][1], seg[i + 1][2] - seg[i][2])
                    for i in range(len(seg) - 1))
            return d / max(1e-6, seg[-1][0] - seg[0][0])

        # 三段样本（都按"离区中心沿路径的距离"分；区占 s_slow±1.0）：
        #   before  紧贴前边界**外侧** 0.02~0.15 m ⇒ 量"**进区那一刻**的速度"
        #           ⚠ 不能量"进区前 0.6 m 区间的平均速度"：那里正当减速过程，
        #             取平均必然大于限速（踩过：会把"前瞻已生效"误判成没生效）。
        #   inside  区内主体（两边各留 0.2 m 避开边界过渡）
        #   after   出区后 0.2~0.9 m ⇒ 巡航段，不受终点刹车影响
        def along(q):
            return (q[1] - sx3) * ux + (q[2] - sy3) * uy

        before = [q for q in traj3 if s_slow - 1.15 <= along(q) < s_slow - 1.02]
        inside = [q for q in traj3 if s_slow - 0.8 <= along(q) <= s_slow + 0.8]
        after = [q for q in traj3 if s_slow + 1.2 <= along(q) <= s_slow + 1.9]

        v_in, v_before, v_after = avg_speed(inside), avg_speed(before), avg_speed(after)
        print(f"  进区边界前 0.02~0.15 m 速度 "
              f"{v_before if v_before is None else round(v_before, 3)}"
              f" | 区内 {v_in if v_in is None else round(v_in, 3)}"
              f" | 出区后 {v_after if v_after is None else round(v_after, 3)} m/s "
              f"(样本 {len(before)}/{len(inside)}/{len(after)})")
        chk("[Z4] ★ 进区**之前**速度已 ≤ 限速（前瞻生效，进区才减速就晚了）",
            v_before is not None and v_before <= lim + 0.03,
            f"{v_before if v_before is None else round(v_before, 3)} m/s vs 限速 {lim}")
        chk("[Z5] 区内速度 ≤ 限速", v_in is not None and v_in <= lim + 0.03,
            f"{v_in if v_in is None else round(v_in, 3)} m/s vs 限速 {lim}")
        chk("[Z6] 出区后恢复（不再被永久压住）",
            v_after is None or v_after > lim + 0.03,
            f"{v_after if v_after is None else round(v_after, 3)} m/s")
        # ⚠ 不要写"与限速区无碰撞"：限速区本来就该开进去（它不是障碍，只是限速）。
        #   这里只能验"真的往前走了"，否则上面的速度统计可能是在量一辆停着的车。
        moved = max((along(q) for q in traj3), default=0.0)
        chk("[Z7] 限速段确实开进去了（不是停着量的速度）",
            moved > s_slow + 1.2, f"沿路径最远 {moved:.2f} m（限速区占 "
                                  f"{s_slow - 1.0:.2f}~{s_slow + 1.0:.2f} m）")

        if args.csv:
            with open(args.csv, "w", encoding="utf-8") as f:
                f.write("phase,t,x,y\n")
                for name, tr in (("band", traj1), ("clear", traj2), ("slow", traj3)):
                    for q in tr:
                        f.write(f"{name},{q[0]:.3f},{q[1]:.3f},{q[2]:.3f}\n")
    finally:
        # 清掉测试发的区域（只影响本测试起的节点；map_server 的那个发布者不受影响）
        try:
            publish([])
        except Exception:  # noqa: BLE001
            pass
        p.destroy_node()
        rclpy.shutdown()
        sc.stop(proc)

    rc = summary()
    if rc:
        print(f"--- 节点日志（{log}） ---")
        print("".join(open(log, encoding="utf-8", errors="replace").readlines()[-25:]))
    elif args.keep_log:
        print(f"节点日志：{log}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
