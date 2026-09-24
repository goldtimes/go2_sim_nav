#!/usr/bin/env python3
"""M4.5：轨迹优化（MINCO 剖面）**A/B 对照** —— 同一段走廊、同一个方向上各跑一趟。

在**已经跑起来的仿真栈**上跑（gazebo + lightning 定位 + perception + map_server）。
本脚本自己起 / 停 pnc_2d 三节点（因为 `traj.type` 是启动参数），但**不碰**仿真与定位。

    python3 test/sim/traj_ab.py                 # 默认 4 m，跑 none 与 minco 两趟
    python3 test/sim/traj_ab.py --dist 6 --csv-prefix /tmp/ab
    python3 test/sim/traj_ab.py --only minco    # 只跑一趟（想手动切开关时用）

出表要回答的问题（doc/minco_trajectory_plan.md §M4.5）：
  横向偏差（max/RMS）、RMS|ω|、ω 反向次数、到点误差、**剖面偏差**、求解耗时。

★ 为什么必须"同一段走廊 + 同一个方向跑两趟"：MINCO 的价值全在"同输入下差了多少"，
  两次跑不同路线/不同长度，比出来的数没有意义。

★ 为什么要"折返"而不是"两趟都去同一个目标"：第一趟开到目标就停在那里，第二趟的
  目标在**车背后** ⇒ A* 会绕一个大 U 形，与第一趟完全不是同一段路。折返（第二趟
  回到第一趟的出发点）让两趟走的是**同一条直线、相反方向**，几何可比。

⚠ 相比 `test_drive_goal.py`，本脚本**不做合格判定**（只有观测口径）。理由：MINCO
  开关的效果要先看数、再定线；反过来会把"预期之内的差别"判成失败。

⚠ 跑之前请先停掉**已经在跑的** pnc_2d 三节点（否则两套 manager 抢 `/goal_pose`、
  两个 global_planner 抢服务名，看到的数会随机来自两套）。用 `--kill-existing`
  可让脚本代劳。
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

import rclpy

import sim_common as sc
from sim_common import stop

PNC_NODES = ("pnc_manager", "global_planner", "local_planner")


def rcl_init():
    """按需 init（rclpy 在 Humble 里重复 init 不总是安全，而本脚本一趟 init/
    shutdown 一次、multiple run 之间要反复进出）"""
    if not rclpy.ok():
        rclpy.init()


def running_pnc_nodes(timeout=6.0) -> list:
    """当前域里有没有在跑的 pnc_2d 三节点。
    ★ 真正实现在 `sim_common`（所有 E2E 脚本共用）；这里只是转发，避免两份分叉。
    """
    return sc.running_pnc_nodes(timeout)


def kill_existing():
    for pat in ("pnc_2d.launch.py", "pnc_2d/global_planner_node",
                "pnc_2d/local_planner_node", "pnc_2d/pnc_manager_node"):
        subprocess.run(["pkill", "-f", pat], check=False)
    time.sleep(2.0)


def diagnose_pose(p, topic="/lightning/perception/pose"):
    """收不到位姿时把现场摊开：话题名对不对 / 有没有发布者 / QoS 兼不兼容

    ★ “没发”与“发了但 QoS 不兼容”在现象上完全一样（否则不会专门写这个函数）：
      DDS 不会因为 reliability 不兼容报错，数据就是不投递。所以必须把
      “该话题发布者 N 个”和“同域里有哪些位姿类话题”都打出来。
    """
    pubs = p.count_publishers(topic)
    print(f"\n没有位姿：订阅 {topic} | 该话题发布者 {pubs} 个")
    if pubs == 0:
        print("  → 仿真/定位没起，或者话题名不是这个。同域里的位姿类话题：")
        found = False
        for name, types in sorted(p.get_topic_names_and_types()):
            if not any(("Odometry" in t or "Pose" in t) for t in types):
                continue
            found = True
            print(f"    {name}  [{', '.join(types)}]  "
                  f"发布者 {p.count_publishers(name)} 个")
        if not found:
            print("    （一个都没有 ⇒ 仿真/定位确实没起）")
        else:
            print("  → 用 sim_common.py 里 Probe 的订阅名对上，或临时改"
                  " Probe 的订阅（它是写死的 /lightning/perception/pose）")
    else:
        print("  → 有发布者但收不到：典型的 **QoS 不兼容**（本探针 best_effort，"
              "发布端若不是 best_effort/reliable 兼容关系就不会投递）。"
              "用 `ros2 topic info -v <话题>` 核对两端 reliability。")


def diagnose_local_status(p, log):
    """没有局部状态：要么三节点没起来，要么 **srv/msg 类型不匹配**

    ★ 本期改过 PlanPath.srv / FollowPath.action / LocalStatus.msg ⇒ 类型 hash 变了。
      旧进程与新进程之间不是“字段读不到”，是**连不上**。所以必须提醒重启。
    """
    print("\n没有局部状态（/pnc_2d/local_status）：")
    for k in PNC_NODES:
        print(f"  {k}: {'在' if k in [n.lstrip('/').split('/')[-1] for n in p.get_node_names()] else '不在'}")
    print("  常见原因：")
    print("   ① pnc_2d 三节点没起来（看下面的启动日志）；")
    print("   ② **三节点是旧二进制**（本会话改过 PlanPath.srv / FollowPath.action /"
          " LocalStatus.msg，类型 hash 变了 ⇒ 新旧不兼容）：请 Ctrl-C 重启"
          " `pnc_2d.launch.py`；")
    print("   ③ 位姿话题收不到（局部节点也依赖同一个话题）。")
    if log:
        print(f"  --- pnc_2d 启动日志尾部（{log}）---")
        try:
            with open(log, encoding="utf-8", errors="replace") as f:
                for line in f.readlines()[-18:]:
                    print("   " + line.rstrip())
        except OSError as e:
            print(f"   （读不到日志：{e}）")


def write_type_cfg(tmpdir: str, traj_type: str) -> str:
    """在 **go2_run.yaml 的基础上**追加 `traj.type`，生成这次要用的 extra_config。

    ★★ 不能只写一个含 `traj.type` 的小文件：launch 的 `extra_config` 是**替换**
      而不是追加（`shared = [..., extra]`，最后一个覆盖前面）—— 把 go2_run.yaml
      顶掉，就等于把底盘标定（`local_mpc.v_max 0.42` / `reference_speed 0.35` /
      `approach_speed` / `crawl_speed` / `stop_coast` …）**全部退回代码默认值**。
      实测现象（2026-09-24）：`mpc v_ref=1.000`（默认 v_max）而车一动不动，
      最后被状态机的"10 s 内位移 < 0.20 m"判成卡住 → 恢复 → 失败；
      日志里其它地方看起来**一切正常**，极难归因。
      所以：读原文件 → 合并覆盖 → 另存。
    """
    import yaml

    with open(sc.GO2_CFG, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    for key, section in doc.items():
        if isinstance(section, dict) and "ros__parameters" in section:
            section["ros__parameters"]["traj.type"] = traj_type
    keys = [k for k, v in doc.items()
            if isinstance(v, dict) and "ros__parameters" in v]
    if not keys:  # 基文件结构变了就别默默用一个空配置
        raise RuntimeError(f"{sc.GO2_CFG} 里没有 */ros__parameters，无法合并")
    merged = any(str(v.get("local_mpc.v_max", "")) not in ("", "None")
                 for v in (doc[k]["ros__parameters"] for k in keys))
    tag = "已带上" if merged else "⚠ 没读到"
    print(f"  extra_config = {sc.GO2_CFG} + traj.type={traj_type}"
          f"（local_mpc.v_max {tag}）")
    path = os.path.join(tmpdir, f"traj_{traj_type}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False)
    return path


# ------------------------------------------------------------------ 一趟
def run_once(traj_type: str, goal, timeout: float, tmpdir: str,
             csv_prefix: str, obstacle_hint, settle: float = 4.0,
             dist: float = 5.0):
    """跑一趟，返回 (metrics dict, probe, log路径)

    `goal` = (x, y, yaw, 标签)；`obstacle_hint` = 已经建好的障碍索引（可空）
    """
    cfg = write_type_cfg(tmpdir, traj_type)
    proc, log = sc.launch("astar", extra_cfg=cfg)
    rcl_init()
    p = sc.Probe(f"traj_ab_{traj_type}")
    met = {"type": traj_type}
    try:
        # ★★ 就绪判据是**新鲜位姿**，不是"收到过几条 local_status"。
        #   踩过两次的两个极端：
        #     ① 只等 `local_msgs > 0` —— 探针订阅的是 latched 话题，会立刻收到
        #        **上一次实验残留**的样本 ⇒ 节点刚起来 20 ms 就发目标 ⇒ FAILED；
        #     ② 等 `local_msgs > before + 5` —— 局部节点**空闲时只在启动那一刻
        #        发一条 IDLE**（控制循环没目标就直接 return）⇒ 永远等不到，
        #        白等到超时。
        #   所以：把 pose 置空、强制重收一条真实 odom（证明位姿链路是活的），
        #   再按 `--settle` 固定沉降（DDS 发现 + latched 全局图到齐）。
        #   万一还是太早，下面发目标那段会重试。
        p.spin(settle)
        p.pose = None
        if not p.wait_for(lambda: p.pose is not None, timeout=20.0):
            diagnose_pose(p)
            met["error"] = "nodes-not-live"
            return met, p, log

        sx, sy, syaw = p.pose
        obstacle = obstacle_hint
        if obstacle is None:
            obstacle = (sc.ObstacleIndex(p.global_map) if p.global_map is not None
                        else sc.ObstacleIndex(sc.load_map_msg(sc.MAP_DIR)))
        met["start"] = (sx, sy, syaw)
        met["start_clearance"] = obstacle.clearance(sx, sy)
        print(f"\n[{traj_type}] 起点 ({sx:.2f}, {sy:.2f}, "
              f"{math.degrees(syaw):.1f}°) 净距 {met['start_clearance']:.2f} m")

        if goal is None:
            # ★ 从**车现在的位置**重新选一条直线目标。
            #   ✗ 不要用"折返（第二趟回到第一趟起点）"：那必然要求 180° 转向，
            #     而 MINCO 会把那个转向摊进整条轨迹（实测 4.8 m 的路径产出
            #     285 s / 0.07 m/s 的怪物），且 M4.4 的机头差门槛会直接跳过它
            #     ⇒ 第二趟根本没有剖面，A/B 恒为空。
            #   代价：两趟的起终点略有不同（脚本会把几何打印出来供判断），
            #   换来的是"两趟都是顺着路开的正常直线"，这才是可比的前提。
            cands = sc.goal_candidates(obstacle, (sx, sy), dist, heading=syaw)
            if not cands:
                print(f"[{traj_type}] 找不到可走的直线目标：车可能被围住了")
                met["error"] = "no-goal"
                return met, p, log
            gx, gy, gd, dmin, ang = cands[0]
            gyaw = math.atan2(gy - sy, gx - sx)
            gtag = f"自动选（{ang}°，直线 {gd:.2f} m，净距 {dmin:.2f} m）"
        else:
            gx, gy, gyaw = goal[0], goal[1], goal[2]
            gtag = goal[3]
        print(f"[{traj_type}] 目标 ({gx:.2f}, {gy:.2f}, "
              f"{math.degrees(gyaw):.1f}°) — {gtag}")
        met["goal"] = (gx, gy, gyaw)

        # ★ 发目标：带**启动竞态重试**。
        #   实测（2026-09-24）：节点刚起来 20 ms 就发目标，manager 还没收到 odom，
        #   直接 `IDLE --GoalReceived--> PLANNING --PlanFail--> FAILED`，报
        #   "还没收到位姿"。这不是"定位没起"，是**发目标太早**。
        #   重试而不是"多睡一会儿"：睡多久都没有判据，重试有判据（FAILED 且
        #   没有进过 FOLLOWING）。
        for attempt in range(1, 4):
            p.begin()   # 清采样 + 清 state_seen（上一轮的 FAILED 必须清掉）
            p.send_goal(gx, gy, gyaw)
            t0 = time.time()
            while time.time() - t0 < 8.0:
                p.spin(0.1)
                if {"FOLLOWING", "GOAL_REACHED"} & p.state_seen:
                    break
                if "FAILED" in p.state_seen:
                    break
            if "FAILED" not in p.state_seen:
                break
            print(f"[{traj_type}] 第 {attempt} 次发目标被判 FAILED："
                  f"{p.last_sm_msg or '(无消息)'}")
            if attempt < 3:
                print("  → 典型是**启动竞态**（节点还没收到 odom/全局图），"
                      "等 2 s 重发")
                p.spin(2.0)
        else:
            met["error"] = "goal-failed"
            return met, p, log

        deadline = t0 + timeout
        while time.time() < deadline:
            p.spin(0.1)
            if "GOAL_REACHED" in p.state_seen or "FAILED" in p.state_seen:
                break
        reached_at = (time.time() - t0) if "GOAL_REACHED" in p.state_seen else None
        p.spin(1.5)   # 到点后再采一点，量"真的停了没有"

        met["state_seen"] = sorted(p.state_seen)
        met["final_state"] = p.final_state
        met["sm_message"] = p.last_sm_msg
        met["reached_at"] = reached_at
        met["route_mode_seen"] = p.route_mode_seen
        met["rows"] = len(p.rows)
        met["traj_points"] = len(p.traj)
        if p.first_path:
            met["path_points"] = len(p.first_path)
            met["path_len"] = sc.poly_length(p.first_path)
        print(f"[{traj_type}] 状态 {' → '.join(sorted(p.state_seen))}"
              f"（最终 {p.final_state}）| 到点 "
              f"{'—' if reached_at is None else f'{reached_at:.2f} s'} | "
              f"局部状态 {len(p.rows)} 条")
        if reached_at is None:
            # ★ 没到点就必须**当场**说清原因："FAILED" 本身没有信息量，
            #   `sm_message` 才是；而且下一趟的起点会变成现在这个位置
            #   ⇒ 两趟的几何就不再可比（表尾的可比性检查只能报警，不能补救）。
            print(f"  ⚠ 本趟没到点：{p.last_sm_msg or '(管理器无消息)'}")
            print("  ⚠ 下一趟将从**这趟停下的位置**出发 ⇒ 两趟几何不可比，"
                  "先把车摆回空旷处再跑")

        if not p.rows:
            met["error"] = "no-local-status"
            return met, p, log

        # ---------------- 指标 ----------------
        traj = p.traj
        path = p.first_path or []
        offs = [sc.signed_offset(q, path) if len(path) >= 2 else 0.0
                for q in traj]
        t_end = traj[-1][0] if traj else 0.0
        settle_t = 0.5 * t_end
        off_st = [offs[i] for i, q in enumerate(traj) if q[0] >= settle_t] or [0.0]
        met["off_max_all"] = max(abs(o) for o in offs) if offs else 0.0
        met["off_max_st"] = max(abs(o) for o in off_st)
        met["off_rms_st"] = math.sqrt(sum(o * o for o in off_st) / len(off_st))
        met["off_bias_st"] = sum(off_st) / len(off_st)
        met["v_end"] = sc.pose_speed(traj, t_end - 0.5, t_end)
        met["min_clearance"] = min(obstacle.clearance(q[1], q[2])
                                   for q in traj) if traj else 0.0
        met["travel"] = sc.poly_length([(q[1], q[2]) for q in traj])
        met["max_cmd_v"] = max((r[6] for r in p.rows), default=0.0)
        met["max_cmd_w"] = max((abs(r[7]) for r in p.rows), default=0.0)

        # 顺滑度（稳态窗口）：ω 反向次数/秒 + RMS|ω|
        stw = [r[7] for r in p.rows if r[0] >= settle_t]
        st_rows = [r for r in p.rows if r[0] >= settle_t]
        dur_st = max(1e-6, (st_rows[-1][0] - st_rows[0][0])
                     if len(st_rows) > 1 else 0.0)
        flips, prev = 0, 0.0
        for w in stw:
            if abs(w) < 0.02:
                continue
            if prev != 0.0 and (w > 0) != (prev > 0):
                flips += 1
            prev = w
        met["omega_flips"] = flips
        met["omega_flip_hz"] = flips / dur_st
        met["omega_rms"] = math.sqrt(sum(w * w for w in stw) / len(stw)) \
            if stw else 0.0

        # 求解耗时
        slv = [r[14] for r in p.rows]
        met["solve_mean_ms"] = sum(slv) / len(slv)
        met["solve_max_ms"] = max(slv)

        # ★ 剖面一致性（M4.4 / R9）：dev < 0 = 本周期没有剖面
        #   ① `profile_dev`      参考口径：剖面 vs 本周期生效的 v_ref
        #   ② `profile_track_dev` 跟踪口径：v_ref vs 实测 v
        #   ★ 两个**不能合成一个**：起步/原地对正之后跟踪偏差必然大，
        #     把那个读成"剖面没生效"会一路查错方向（踩过）。
        devs = [r[11] for r in p.rows if r[11] >= 0.0]
        met["prof_rows"] = len(devs)
        met["prof_dev_max"] = max(devs) if devs else -1.0
        met["prof_dev_mean"] = (sum(devs) / len(devs)) if devs else -1.0
        met["prof_over10"] = sum(1 for d in devs if d > 0.10)
        pv = [r[12] for r in p.rows if r[11] >= 0.0]
        pvr = [r[13] for r in p.rows if r[11] >= 0.0]
        trk = [r[15] for r in p.rows if r[15] >= 0.0]
        met["prof_v_mean"] = (sum(pv) / len(pv)) if pv else -1.0
        met["prof_v_ref_mean"] = (sum(pvr) / len(pvr)) if pvr else -1.0
        met["prof_track_max"] = max(trk) if trk else -1.0
        met["prof_track_mean"] = (sum(trk) / len(trk)) if trk else -1.0

        # 到点（判定那一刻 vs 停稳后）
        fx, fy, fyaw = p.pose
        met["d_goal"] = math.hypot(gx - fx, gy - fy)
        met["yaw_err_deg"] = math.degrees(abs(sc.wrap_pi(fyaw - gyaw)))
        if p.reached_t is not None:
            ax, ay = sc.pose_at(traj, p.reached_t)
            met["d_at_reached"] = math.hypot(gx - ax, gy - ay)
            met["coast"] = sc.poly_length(
                [(q[1], q[2]) for q in traj if q[0] >= p.reached_t])

        if csv_prefix:
            csv = f"{csv_prefix}_{traj_type}.csv"
            # 位姿与局部状态是两条流、频率不同 ⇒ 用**单向游标**对齐
            # （不能用 `next(...)` 逐点重扫：几千点 × 几千行 = 平方复杂度）
            # ★ 用 join 拼而**不用 %-格式化**：占位符个数与参数个数对不上是
            #   这类代码最常见的自伤（本次就在这里踩过：加了 track_dev 却忘了加
            #   一个 %.3f，直接 `TypeError` 把整趟跑废）。join 结构上不可能对不上。
            with open(csv, "w", encoding="utf-8") as f:
                f.write("t,x,y,yaw,offset,cmd_v,cmd_w,profile_dev,"
                        "profile_v,profile_v_ref,track_dev,solve_ms\n")
                k = 0
                for i, q in enumerate(traj):
                    while k + 1 < len(p.rows) and p.rows[k + 1][0] <= q[0]:
                        k += 1
                    r = p.rows[k] if p.rows else None
                    cols = ["%.3f" % q[0], "%.3f" % q[1], "%.3f" % q[2],
                            "%.4f" % q[3], "%.4f" % offs[i]]
                    if r is None:
                        cols += [""] * 7
                    else:
                        cols += ["%.3f" % r[6], "%.3f" % r[7],
                                 "%.3f" % r[11], "%.3f" % r[12],
                                 "%.3f" % r[13], "%.3f" % r[15],
                                 "%.1f" % r[14]]
                    f.write(",".join(cols) + "\n")
            met["csv"] = csv
        return met, p, log
    finally:
        try:
            p.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        stop(proc)


# ------------------------------------------------------------------ 出表
ROWS = [
    ("状态序列", "state_seen", "{:s}", lambda m: " → ".join(m.get("state_seen", []))
     if m.get("state_seen") else "—"),
    # ★ 失败原因必须上表："FAILED" 本身没信息量，`sm_message` 才是
    #   （实测的第一版就被 "还没收到位姿" 这种启动竞态耗掉了一整轮）
    ("最终消息", "sm_message", "{:s}", None),
    ("到点用时 [s]", "reached_at", "{:.2f}", None),
    ("到点误差 [m]", "d_goal", "{:.3f}", None),
    ("  判定到达时 [m]", "d_at_reached", "{:.3f}", None),
    ("  判定后滑行 [m]", "coast", "{:.3f}", None),
    ("末速 [m/s]", "v_end", "{:.3f}", None),
    ("末朝向误差 [°]", "yaw_err_deg", "{:.2f}", None),
    ("横向偏差 max 全程 [m]", "off_max_all", "{:.3f}", None),
    ("横向偏差 max 稳态 [m]", "off_max_st", "{:.3f}", None),
    ("横向偏差 RMS 稳态 [m]", "off_rms_st", "{:.3f}", None),
    ("横向偏差 均值偏置 [m]", "off_bias_st", "{:+.3f}", None),
    ("RMS|ω| 稳态 [rad/s]", "omega_rms", "{:.3f}", None),
    ("ω 反向次数", "omega_flips", "{:d}", None),
    ("ω 反向频率 [Hz]", "omega_flip_hz", "{:.2f}", None),
    ("指令峰值 |v| [m/s]", "max_cmd_v", "{:.3f}", None),
    ("指令峰值 |ω| [rad/s]", "max_cmd_w", "{:.3f}", None),
    ("行程 [m]", "travel", "{:.2f}", None),
    ("最小障碍净距 [m]", "min_clearance", "{:.3f}", None),
    ("求解耗时 均值 [ms]", "solve_mean_ms", "{:.1f}", None),
    ("求解耗时 max [ms]", "solve_max_ms", "{:.1f}", None),
    ("**剖面点数**", "prof_rows", "{:d}", None),
    ("**剖面参考偏差 max**", "prof_dev_max", "{:.1%}",
     lambda m: None if m.get("prof_dev_max", -1.0) < 0.0 else m["prof_dev_max"]),
    ("**剖面参考偏差 均值**", "prof_dev_mean", "{:.1%}",
     lambda m: None if m.get("prof_dev_mean", -1.0) < 0.0 else m["prof_dev_mean"]),
    ("  参考偏差 >10% 拍数", "prof_over10", "{:d}", None),
    ("剖面 v 均值 [m/s]", "prof_v_mean", "{:.3f}",
     lambda m: None if m.get("prof_v_mean", -1.0) < 0.0 else m["prof_v_mean"]),
    ("生效 v_ref 均值 [m/s]", "prof_v_ref_mean", "{:.3f}",
     lambda m: None if m.get("prof_v_ref_mean", -1.0) < 0.0
     else m["prof_v_ref_mean"]),
    ("跟踪偏差 max", "prof_track_max", "{:.1%}",
     lambda m: None if m.get("prof_track_max", -1.0) < 0.0
     else m["prof_track_max"]),
    ("跟踪偏差 均值", "prof_track_mean", "{:.1%}",
     lambda m: None if m.get("prof_track_mean", -1.0) < 0.0
     else m["prof_track_mean"]),
    ("走廊模式（应为 False）", "route_mode_seen", "{!s}", None),
]


def fmt(v, f):
    if v is None:
        return "—"
    if f.endswith("%}"):
        return f.format(v)
    try:
        return f.format(v)
    except (ValueError, TypeError):
        return str(v)


def print_table(mets: dict):
    names = list(mets.keys())
    w = 30
    print("\n" + "=" * (w + 20 * max(1, len(names))))
    print("M4.5 轨迹优化 A/B 对照表")
    print("=" * (w + 20 * max(1, len(names))))
    print(f"{'指标':<{w}}" + "".join(f"{n:>20}" for n in names))
    print("-" * (w + 20 * max(1, len(names))))
    for label, key, f, fn in ROWS:
        line = f"{label:<{w}}"
        for n in names:
            m = mets[n]
            v = fn(m) if fn else m.get(key)
            line += f"{fmt(v, f):>20}"
        print(line)
    for n in names:
        m = mets[n]
        if "start" in m:
            print(f"{n} 起点 ({m['start'][0]:.2f}, {m['start'][1]:.2f}, "
                  f"{math.degrees(m['start'][2]):.1f}°) 净距 "
                  f"{m.get('start_clearance', float('nan')):.2f} m | "
                  f"路径 {m.get('path_points', 0)} 点 / "
                  f"{m.get('path_len', 0.0):.2f} m")
    print("=" * (w + 20 * max(1, len(names))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=4.0, help="第一趟目标距起点 [m]")
    ap.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"),
                    help="固定第一趟的目标（不给就自动选一段空直线）")
    ap.add_argument("--types", default="none,minco", help="要跑的 traj.type，逗号分隔")
    ap.add_argument("--only", default="", help="只跑这一种（等价于 --types 一个）")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--settle", type=float, default=4.0,
                    help="起完三节点后等多久再发目标 [s]（DDS 发现 + latched 图）")
    ap.add_argument("--csv-prefix", default="")
    ap.add_argument("--kill-existing", action="store_true",
                    help="先杀掉已在跑的 pnc_2d 三节点（不碰仿真/定位/map_server）")
    ap.add_argument("--keep-log", action="store_true")
    args = ap.parse_args()

    types = [args.only] if args.only else [t.strip() for t in args.types.split(",")
                                           if t.strip()]
    if not types:
        print("--types 为空")
        return 2

    existing = running_pnc_nodes()
    if existing:
        print(f"⚠ 检测到已在运行的 pnc_2d 节点：{', '.join(existing)}")
        if not args.kill_existing:
            print("  两套 manager/planner 抢同一个 /goal_pose 与服务名，数会随机"
                  "来自两套 ⇒ 请先停掉，或加 --kill-existing 让脚本代劳。")
            print("  停法：在启动 pnc_2d.launch.py 的那个终端里 Ctrl-C")
            return 2
        print("  按 --kill-existing 停掉这三个节点……")
        kill_existing()

    tmpdir = tempfile.mkdtemp(prefix="traj_ab_")
    mets = {}
    logs = []
    rcl_init()
    try:
        # ---- 先探一次：拿起点 + 建障碍索引 + 定目标 ----
        # ★ 这里**只等位姿**：pnc_2d 三节点要等目标定好、进 run_once 才会起。
        #   （踩过：预检要求 local_msgs>0，而那时候三节点还没起 ⇒ 必然超时，
        #    报出来的现象却是“仿真或定位没起”，把方向带偏了。）
        p0 = sc.Probe("traj_ab_pick")
        if not p0.wait_for(lambda: p0.pose is not None, timeout=20.0):
            diagnose_pose(p0)
            return 2
        # ★ 位姿先到、latched 的图后到是常态 ⇒ 再多转一会儿把图收全
        p0.spin(1.5)
        sx, sy, syaw = p0.pose
        obstacle = (sc.ObstacleIndex(p0.global_map) if p0.global_map is not None
                    else sc.ObstacleIndex(sc.load_map_msg(sc.MAP_DIR)))
        p0.destroy_node()
        print(f"起点 ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.1f}°) | "
              f"起点净距 {obstacle.clearance(sx, sy):.2f} m")

        if args.goal:
            gx, gy = args.goal[0], args.goal[1]
            gyaw = math.radians(args.goal[2])
            first_goal = (gx, gy, gyaw, "人工指定")
        else:
            cands = sc.goal_candidates(obstacle, (sx, sy), args.dist,
                                       heading=syaw)
            if not cands:
                print("找不到可走的直线目标：车可能被围住了，先手动挪一下")
                return 2
            gx, gy, gd, dmin, ang = cands[0]
            gyaw = math.atan2(gy - sy, gx - sx)
            first_goal = (gx, gy, gyaw,
                          f"自动选（{ang}°，直线 {gd:.2f} m，净距 {dmin:.2f} m）")
            print(f"自动选目标：{first_goal[3]}")

        # ---- 第二趟从"车当时在的位置"重新选目标（不折返，见 run_once 里的说明）----
        goals = [first_goal, None]

        for i, t in enumerate(types):
            met, _p, log = run_once(t, goals[i % len(goals)], args.timeout,
                                    tmpdir, args.csv_prefix, obstacle,
                                    settle=args.settle, dist=args.dist)
            mets[t] = met
            logs.append((t, log))
            if met.get("route_mode_seen"):
                print(f"⚠ [{t}] 本次跑在**走廊模式**下 ⇒ MINCO 会被跳过，"
                      f"A/B 无意义。请用 planner_type:=astar 且地图无 routes.yaml。")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print_table(mets)

    # 两趟几何是否可比（不同起点/目标会让横向偏差与顺滑度都不可比）
    if len(mets) == 2:
        a, b = mets[types[0]], mets[types[1]]
        if "start" in a and "start" in b:
            d = math.hypot(a["start"][0] - b["goal"][0],
                           a["start"][1] - b["goal"][1])
            print(f"\n几何可比性检查：第二趟的目标与第一趟的起点相距 {d:.2f} m"
                  f"（越小越可比）")
        if a.get("path_len") and b.get("path_len"):
            print(f"  路径长度 {a['path_len']:.2f} m vs {b['path_len']:.2f} m"
                  f"（差 {abs(a['path_len'] - b['path_len']):.2f} m）")

    for t, log in logs:
        if args.keep_log:
            print(f"[{t}] 节点日志：{log}")
        else:
            shutil.rmtree(tmpdir, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
