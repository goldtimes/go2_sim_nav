#!/usr/bin/env python3
"""P5.4 仿真验收①：自由空间直线行驶（astar 全局 + mpc 局部）。

在**已经跑起来的仿真栈**上跑（gazebo + lightning 定位 + perception + map_server），
本脚本只负责起 pnc_2d 三节点、选目标、进程内高频记录、算指标、判合格。

    python3 test/sim/test_drive_goal.py                # 4 m 直线
    python3 test/sim/test_drive_goal.py --dist 8 --csv /tmp/t.csv

验收标准（doc/mpc_local_planner_plan.md §5.4）：
  A 到点停车误差 ≤ **0.03 m**（用户要求）；末速 ≈ 0（不留残留速度）
  B 稳态横向误差 ≤ 0.10 m、全程（含起转）≤ 0.30 m
  C 全程无碰撞：轨迹到全局图占据格的最小净距 > 0
  D 速度/角速度不越界（上限从 go2_run.yaml 读）

实测（2026-09-22，6/6 通过）：到点 0.017~0.043 m、末速 0.012~0.013 m/s、
横向 0.016~0.026 m（稳态）/0.051~0.058 m（全程）、保真度 0.93~0.96。

⚠ 到点误差只有在 `local.goal_tolerance` **严格小于**验收阈值时才有意义
（踩过：容差也设 0.15 ⇒ 管理器在 0.15 m 处就停机，量到的就是它自己的触发条件）。

公共部分（起停、采样、指标、踩坑说明）见 `sim_common.py`。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy

import sim_common as sc
from sim_common import (chk, load_limits, pose_speed, signed_offset, stop,
                        summary)

# 到点**朝向**容差 [°]：与 local_mpc.goal_yaw_tolerance_deg 同源（不写死）
GOAL_YAW_TOL_DEG = sc.load_goal_yaw_tol_deg()


def self_diagnose(p, obstacle, sx, sy):
    """一条局部状态都没有 ⇒ 根本没进到跟随阶段，最常见的两种原因直接说明"""
    c = obstacle.clearance(sx, sy)
    print(f"\n局部状态 0 条 | 管理器最终状态 {p.final_state} | "
          f"消息：{p.last_sm_msg}")
    print(f"起点在发布的全局图里的净距 {c:.2f} m")
    if c < 0.6:
        print("→ 车停在障碍/禁行区膨胀边界里，**先把车开到空旷处**再跑"
              "（例如 (0, 0) 附近，距 Z1 边界 ≈1.7 m）")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=4.0, help="目标距起点 [m]")
    ap.add_argument("--csv", default="")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--keep-log", action="store_true")
    args = ap.parse_args()

    v_max, w_max, _ = load_limits()
    print(f"配置（go2_run.yaml）：v_max={v_max} w_max={w_max}")

    proc, log = sc.launch("astar")
    rclpy.init()
    p = sc.Probe("p5_drive_probe")
    try:
        p.wait_for(lambda: p.pose is not None and p.local_msgs > 0, timeout=15.0)
        if p.pose is None:
            print("没有位姿：仿真/定位没起？")
            return 2
        sx, sy, syaw = p.pose
        if p.global_map is not None:
            obstacle = sc.ObstacleIndex(p.global_map)
            src = "发布的全局图（含禁行区）"
        else:
            obstacle = sc.ObstacleIndex(sc.load_map_msg(sc.MAP_DIR))
            src = "裸 PGM（⚠ 不含禁行区）"
        print(f"起点 ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.1f}°) | "
              f"起点净距 {obstacle.clearance(sx, sy):.2f} m | 源：{src}")

        cands = sc.goal_candidates(obstacle, (sx, sy), args.dist, heading=syaw)
        if not cands:
            print("找不到可走的直线目标：车可能被围住了，先手动挪一下")
            return 2

        # ★ 几何够空 ≠ 全局可达（大障碍另一侧／目标处 footprint 摆不下）。这是全局
        #   规划器的职责，不该当成本次“局部跟踪”验收的失败 ⇒ 换下一个候选重试。
        deadline = time.time() + args.timeout
        chosen = None
        for i, (gx, gy, gd, dmin, ang) in enumerate(cands[:6]):
            gyaw = math.atan2(gy - sy, gx - sx)
            print(f"候选[{i}] 目标 ({gx:.2f}, {gy:.2f})：方向 {ang}°，距离 "
                  f"{gd:.2f} m，直线最小净距 {dmin:.2f} m")
            p.begin()
            p.send_goal(gx, gy, gyaw)
            t0 = time.time()
            while time.time() - t0 < 6.0 and time.time() < deadline:
                p.spin(0.1)
                if {"FOLLOWING", "GOAL_REACHED"} & p.state_seen:
                    break
                if "FAILED" in p.state_seen:
                    break
            if "FAILED" not in p.state_seen:
                chosen = (gx, gy, gyaw)
                print(f"→ 候选[{i}] 进入跟随（朝向 "
                      f"{math.degrees(gyaw):.1f}°，与行驶方向一致）")
                break
            print(f"→ 候选[{i}] 全局规划 FAILED（{p.last_sm_msg}），换下一个")
        if chosen is None:
            print("所有候选都规划不出来：车可能被困在封闭区域，先手动挪一下")
            return 2
        gx, gy, gyaw = chosen

        reached_at = None
        while time.time() < deadline:
            p.spin(0.1)
            if "GOAL_REACHED" in p.state_seen:
                reached_at = time.time() - t0
                break
        p.spin(1.5)          # 到点后再采一点，量"真的停了没有"

        print(f"\n状态序列：{' → '.join(sorted(p.state_seen))}"
              f"（最终 {p.final_state}）")
        print(f"到点用时 {reached_at and round(reached_at, 2)} s，"
              f"局部状态 {len(p.rows)} 条 / 位姿 {len(p.traj)} 点")

        if not p.rows:
            self_diagnose(p, obstacle, sx, sy)
            return 3

        # ---- 指标（一律用高频位姿轨迹算）----
        traj = p.traj
        tr = [(q[1], q[2]) for q in traj]
        min_clr = min(obstacle.clearance(x, y) for x, y in tr) if tr else 0.0
        path = p.first_path or []
        offs = [signed_offset(q, path) if len(path) >= 2 else 0.0 for q in traj]
        t_end = traj[-1][0] if traj else 0.0
        settle_t = 0.5 * t_end      # 前半程含起转，单独看
        off_st = [offs[i] for i, q in enumerate(traj) if q[0] >= settle_t] or [0.0]
        off_all = [abs(o) for o in offs] or [0.0]
        v_end = pose_speed(traj, t_end - 0.5, t_end)
        max_cmd_v = max((r[6] for r in p.rows), default=0.0)
        max_cmd_w = max((abs(r[7]) for r in p.rows), default=0.0)
        fx, fy, _ = p.pose
        d_goal = math.hypot(gx - fx, gy - fy)

        # ★ 把"判定到达那一刻"与"停机后滑行"分开：3 cm 量级必须分清是
        #   ① 判定条件太松，还是 ② 判完之后车又滑几厘米（底盘的步态响应有滞后）。
        d_at_reached = float("nan")
        coast = 0.0
        if p.reached_t is not None:
            ax, ay = sc.pose_at(traj, p.reached_t)
            d_at_reached = math.hypot(gx - ax, gy - ay)
            after = [(q[1], q[2]) for q in traj if q[0] >= p.reached_t]
            coast = sc.poly_length(after)

        # ★ 残余偏差的**符号**要分开看：偏短（没走到）还是偏长（冲过去）。
        #   两者对应完全不同的修法（提前量 stop_coast / 贴拢速度），只要一个数
        #   字（距离）是看不出方向的。
        along = 0.0
        if p.first_path and len(p.first_path) >= 2:
            (ax0, ay0), (bx0, by0) = p.first_path[-2], p.first_path[-1]
            L = math.hypot(bx0 - ax0, by0 - ay0)
            if L > 1e-6:
                ux, uy = (bx0 - ax0) / L, (by0 - ay0) / L
                along = (fx - gx) * ux + (fy - gy) * uy   # >0 = 冲过目标

        # 被控对象保真度：稳态段"位姿走过的路程 / 指令积分"
        st_rows = [r for r in p.rows if r[0] >= settle_t]
        cmd_int = sum(0.5 * (a[6] + b[6]) * max(0.0, b[0] - a[0])
                      for a, b in zip(st_rows[:-1], st_rows[1:]))
        tr_st = [q for q in traj if q[0] >= settle_t]
        dist_st = sum(math.hypot(tr_st[i + 1][1] - tr_st[i][1],
                                 tr_st[i + 1][2] - tr_st[i][2])
                      for i in range(len(tr_st) - 1)) if len(tr_st) > 1 else 0.0
        gain = dist_st / cmd_int if cmd_int > 1e-6 else float("nan")

        print("\n---- 指标 ----")
        if len(path) >= 2:
            print(f"路径({len(path)}点) {path[0][0]:.2f},{path[0][1]:.2f} → "
                  f"{path[-1][0]:.2f},{path[-1][1]:.2f} | 轨迹 "
                  f"{tr[0][0]:.2f},{tr[0][1]:.2f} → {tr[-1][0]:.2f},{tr[-1][1]:.2f}")
        else:
            print(f"⚠ 没拿到全局路径（len={len(path)}），横向误差无法独立核算")
        print(f"到点误差        {d_goal:.3f} m  ← 停稳后（验收看这个）")
        print(f"  沿行驶方向      {along:+.3f} m（{'冲过目标' if along > 0 else '没走到'}）")
        if p.reached_t is not None:
            print(f"  判定到达时      {d_at_reached:.3f} m（t={p.reached_t:.2f} s）"
                  f" | 判定后滑行 {coast:.3f} m")
            # ★ 滑行到底是"车真的又走了"还是"停稳时位姿估计跳了一下"？只有看清楚
            #   才知道该改控制器（提前减速）还是该质疑定位（估计跳变）。
            tail = [q for q in traj if q[0] >= p.reached_t - 0.4]
            if len(tail) > 1:
                steps = [(q[0] - p.reached_t,
                          math.hypot(q[1] - tail[i - 1][1], q[2] - tail[i - 1][2]),
                          math.hypot(q[1] - gx, q[2] - gy))
                         for i, q in enumerate(tail) if i]
                print("  到达后位姿步长 (Δ t, 本步位移, 到目标距离)：")
                for dt, st, dg in steps[-12:]:
                    print(f"    {dt:+.2f}s  {st * 100:5.2f} cm  {dg * 100:5.2f} cm")
        print(f"末速(位姿差分)  {v_end:.3f} m/s（最后 0.5 s）")
        # ★ 末朝向：任务目标带 yaw（面向栓/门口），原来只验收 xy ⇒ 机头可以差 90°
        #   而没人发现（用户 2026-09-23 指出）。容差取 local_mpc.goal_yaw_tolerance_deg。
        fyaw = p.pose[2] if p.pose else 0.0
        yaw_err = abs(sc.wrap_pi(fyaw - gyaw))
        print(f"末朝向误差      {math.degrees(yaw_err):.2f}°（目标 {math.degrees(gyaw):.1f}°"
              f" / 实际 {math.degrees(fyaw):.1f}°，容差 {GOAL_YAW_TOL_DEG:.0f}°）")
        print(f"横向误差(全程)  max {max(off_all):.3f} m")
        print(f"横向误差(稳态)  max {max(abs(o) for o in off_st):.3f} m | "
              f"均值偏置 {sum(off_st) / len(off_st):+.3f} m")
        print(f"最小障碍净距    {min_clr:.3f} m（内切半径 0.25 m）")
        print(f"指令峰值        |v| {max_cmd_v:.3f} / |w| {max_cmd_w:.3f}")
        # ★ “摇摇摆摆”要能量化（用户 2026-09-23：“不顺滑，摇摇摆摆”）。
        #   自由模式原来和走廊共用一套贴线权重 ⇒ 几 cm/几度偏差被当大误差追，
        #   ω 顶满来回打。这里量两个数（都取稳态窗口）：
        #     ① ω 反向频率：每秒改变符号多少次（真正的“左右摆”）
        #     ② RMS|ω|：越大说明越是“顶满方向”而非“小幅修正”
        #   ⚠ 这两个数是**观测口径**，不设硬门限 —— 阈值要等真机/仿真跑几轮标定
        #   （先看数、再定线；反过来会把正常行为判成失败）。
        stw = [r[7] for r in p.rows if r[0] >= settle_t]
        flips, eps_w = 0, 0.02
        prev = 0.0
        for w in stw:
            if abs(w) < eps_w:
                continue
            if prev != 0.0 and (w > 0) != (prev > 0):
                flips += 1
            prev = w
        dur_st = (st_rows[-1][0] - st_rows[0][0]) if len(st_rows) > 1 else 1e-6
        dur_st = max(1e-6, dur_st)
        flip_hz = flips / dur_st
        rms_w = math.sqrt(sum(w * w for w in stw) / len(stw)) if stw else 0.0
        print(f"★ 顺滑度(稳态)  ω 反向 {flips} 次 / {dur_st:.1f} s = {flip_hz:.2f} Hz | "
              f"RMS|w| {rms_w:.3f} rad/s"
              + ("  ⚠ 反向频繁 ⇒ 像摇摆，检查 free_lat_deadband/free_w_max"
                 if flip_hz > 1.0 else "  ✓ 未见持续左右打摆"))
        print(f"★ 被控对象保真度  稳态段路程/指令积分 = {gain:.3f}"
              f"（{dist_st:.2f} m / {cmd_int:.2f} m·s⁻¹·s；1.0 = 指令完全被实现）")

        chk("[A1] 到点停车误差 ≤ 0.03 m（用户要求）", d_goal <= 0.03,
            f"停稳后 {d_goal:.3f} m；判定时 {d_at_reached:.3f} m + 滑行 {coast:.3f} m；"
            f"判定容差 local.goal_tolerance=0.02")
        chk("[A2] 末速 ≈ 0（位姿差分，不看噪声 twist）", v_end <= 0.05,
            f"{v_end:.3f} m/s")
        # ★ 末期朝向也算“到达”的一部分：位置对、机头不对不算完成（差速可以原地对正）
        chk(f"[A3] 末朝向偏差 ≤ {GOAL_YAW_TOL_DEG:.0f}°（到点后原地对正目标朝向）",
            math.degrees(yaw_err) <= GOAL_YAW_TOL_DEG + 0.5,
            f"{math.degrees(yaw_err):.2f}°（容差 {GOAL_YAW_TOL_DEG:.0f}°）")
        chk("[B1] 稳态横向误差 ≤ 0.10 m", max(abs(o) for o in off_st) <= 0.10,
            f"max {max(abs(o) for o in off_st):.3f} m，"
            f"均值 {sum(off_st) / len(off_st):+.3f} m")
        chk("[B2] 全程横向误差 ≤ 0.30 m（含起转的合理余量）",
            max(off_all) <= 0.30, f"{max(off_all):.3f} m")
        chk("[C1] 全程无碰撞（净距 > 0）", min_clr > 0.0, f"{min_clr:.3f} m")
        chk("[D1] 指令不越界",
            max_cmd_v <= v_max + 1e-6 and max_cmd_w <= w_max + 1e-6,
            f"|v| {max_cmd_v:.3f} / |w| {max_cmd_w:.3f}")

        if args.csv:
            with open(args.csv, "w", encoding="utf-8") as f:
                f.write("t,x,y,yaw,lateral_offset\n")
                for q, o in zip(traj, offs):
                    f.write("%.3f,%.3f,%.3f,%.4f,%.4f\n" % (q[0], q[1], q[2], q[3], o))
            print(f"\n轨迹已存 {args.csv}（{len(traj)} 点）")
    finally:
        p.destroy_node()
        rclpy.shutdown()
        stop(proc)

    rc = summary()
    if rc:
        print(f"--- 节点日志（{log}） ---")
        print("".join(open(log, encoding="utf-8", errors="replace").readlines()[-25:]))
    elif args.keep_log:
        print(f"节点日志：{log}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
