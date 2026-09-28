#!/usr/bin/env python3
"""M5.0 第 3 步：**软代价假设**的任务级 A/B（一次只动一个变量）。

被检验的假设（doc/minco_trajectory_plan.md §M5.0 第 3 次量测）：
  A* 路径按**全局口径**给（保证 `footprint 0.20 + safe_margin 0.05 +
  map_server.inflate 0.05 = 0.30~0.35 m`），而 MPC 的软代价按**感知**距离场
  判"够不够远"，阈值 `obstacle_safe_distance = 0.45 m`（权重
  `obstacle_weight = 1000`）。⇒ 路径若落在 `0.35~0.45 m` 这条带里，软代价会
  **持续侧推**，车最终停在"跟踪项 vs 避障项"的平衡点上 ⇒ 稳态横向偏置
  = 路径"钻进 0.45 带"的深度 ⇒ **0~10 cm、随场景变化**（与 M4.5 的
  A 场景 0.004 m / 坏场景 ~10 cm 一致）。

**本脚本挂在已经跑起来的 pnc_2d 栈上**（自己**不**起栈：两套栈会互相顶目标，
指标全不可信，2026-09-24 为此排查过一整轮）。

三步：
  ① **试算**（不动车）：对一批候选目标调 `/global_planner/plan_path`
     （`publish_result=false`，不碰话题），量每条路径在**感知图**上的净距，
     挑一条"最贴障碍"的 ⇒ 保证场景真的能把软代价压进生效区；
  ② 跑 A 趟（`obstacle_weight` 原值）；
  ③ 折返一趟（回到 A 的起点，为 ③ 提供同一个起点）；
  ④ 把 `obstacle_weight` 置 0、重载参数，再跑一趟同样的目标 ⇒ 与 A 对照。

指标口径与 `traj_ab.py` **完全一致**（`sc.signed_offset` + 同样的稳态切分
`t ≥ 0.5·t_end`），这样数可以直接和 M4.5 的 A/B 表比。

用法（仿真 + 定位 + perception + map_server + **pnc_2d 三节点**都在跑时）：
    python3 test/sim/soft_cost_ab.py --dry-run        # 只试算，不动车
    python3 test/sim/soft_cost_ab.py --csv /tmp/sc.csv

⚠ 纪律：`obstacle_weight` 只**临时**置 0 做对照，脚本结束前会恢复原值；
  **不许**把"降低软代价"当成修法提交（见 doc §M5 的"不动安全网"清单）。
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_common as sc                                        # noqa: E402

from pnc_2d.srv import PlanPath                                # noqa: E402

SOFT_SAFE = 0.45   # = local_mpc.obstacle_safe_distance
HARD = 0.25        # = local_mpc.obstacle_hard_distance
LOCAL_NODE = "/local_planner"
WEIGHT_PARAM = "local_mpc.obstacle_weight"


class AProbe(sc.Probe):
    """在共用 Probe 上加两件事：订**感知**局部图、提供 `plan_path` 试算。"""

    def __init__(self, name="soft_cost_ab"):
        super().__init__(name)
        self.local_map = None
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d",
                                 self.on_local_map, sc.LIVE)
        self.plan_cli = self.create_client(PlanPath, "/global_planner/plan_path")

    def on_local_map(self, m):
        self.local_map = m

    def plan(self, gx, gy, gyaw=0.0, timeout=20.0):
        """调 `~/plan_path` **试算**（用当前位姿作起点、不发结果到话题）。

        返回响应或 None。`use_current_pose=true`：定位比脚本缓存的新。
        """
        if not self.plan_cli.wait_for_service(timeout_sec=5.0):
            return None
        req = PlanPath.Request()
        req.header.frame_id = "map"
        req.goal.header.frame_id = "map"
        req.goal.pose.position.x = float(gx)
        req.goal.pose.position.y = float(gy)
        req.goal.pose.orientation.z = math.sin(0.5 * gyaw)
        req.goal.pose.orientation.w = math.cos(0.5 * gyaw)
        req.use_current_pose = True
        req.publish_result = False          # ★ 不碰 /pnc_2d/global_path
        fut = self.plan_cli.call_async(req)
        end = time.time() + timeout
        while time.time() < end and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        return fut.result() if fut.done() else None


def path_stats(path, gidx, lidx):
    """一条路径的几何指标：长度 / 全局净距 / **感知净距** / 软代价生效占比。"""
    pts = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]
    if len(pts) < 2:
        return None
    gc = [gidx.clearance(x, y) for x, y in pts]
    lc = [lidx.clearance(x, y) for x, y in pts] if lidx else [1.0] * len(pts)
    inside = [c for c in lc if c < SOFT_SAFE]
    return {
        "n": len(pts),
        "len": sc.poly_length(pts),
        "gc_min": min(gc),
        "lc_min": min(lc),
        "lc_mean": sum(lc) / len(lc),
        "soft_frac": len(inside) / len(pts),      # 落在 0.45 带内的点占比
        "soft_depth": (SOFT_SAFE - min(lc)) if inside else 0.0,   # 钻得多深
    }


def heading_diff(p0, p1, yaw):
    """路径首段方向与车头之差 [°]（traj 的门槛是 45°）"""
    if p0 is None or p1 is None:
        return 0.0
    d = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return math.degrees(abs(sc.wrap_pi(d - yaw)))


def set_weight(w) -> bool:
    """`ros2 param set` + `~/reload_params`（局部节点靠重载才重读参数）。"""
    ok = True
    for cmd in (
        ["ros2", "param", "set", LOCAL_NODE, WEIGHT_PARAM, str(w)],
        ["ros2", "service", "call", f"{LOCAL_NODE}/reload_params",
         "std_srvs/srv/Trigger", "{}"],
    ):
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        out = (r.stdout + r.stderr).strip().replace("\n", " ")
        print(f"    $ {' '.join(cmd[:4])}… → {out[:120]}")
        if r.returncode != 0:
            ok = False
    return ok


def get_weight():
    r = subprocess.run(["ros2", "param", "get", LOCAL_NODE, WEIGHT_PARAM],
                       capture_output=True, text=True, timeout=20)
    return (r.stdout + r.stderr).strip()


def run_leg(p, goal, tag, timeout=90.0):
    """发一次目标并采到结束；返回 met（口径同 traj_ab）。"""
    gx, gy, gyaw = goal
    met = {"tag": tag, "goal": goal}
    for attempt in range(1, 4):
        p.begin()
        p.send_goal(gx, gy, gyaw)
        t0 = time.time()
        while time.time() - t0 < 8.0:
            p.spin(0.1)
            if {"FOLLOWING", "GOAL_REACHED"} & p.state_seen or "FAILED" in p.state_seen:
                break
        if "FAILED" not in p.state_seen:
            break
        print(f"  [{tag}] 第 {attempt} 次发目标被判 FAILED：{p.last_sm_msg or '(无消息)'}")
        p.spin(2.0)
    else:
        met["error"] = "goal-failed"
        return met
    deadline = t0 + timeout
    while time.time() < deadline:
        p.spin(0.1)
        if {"GOAL_REACHED", "FAILED"} & p.state_seen:
            break
    p.spin(1.5)          # 到点后再采一点，量"真的停了没有"
    path = p.first_path or []
    traj = p.traj
    if not traj or len(path) < 2:
        met["error"] = "no-data"
        return met
    offs = [sc.signed_offset(q, path) for q in traj]
    t_end = traj[-1][0]
    st = [o for i, q in enumerate(traj) if q[0] >= 0.5 * t_end] or [0.0]
    met.update(
        state=sorted(p.state_seen), final_state=p.final_state,
        sm_msg=p.last_sm_msg, reached_at=p.reached_t,
        off_max_all=max(abs(o) for o in offs),
        off_max_st=max(abs(o) for o in st),
        off_rms_st=math.sqrt(sum(o * o for o in st) / len(st)),
        off_bias_st=sum(st) / len(st),
        path_len=sc.poly_length(path), path_pts=len(path),
        travel=sc.poly_length([(q[1], q[2]) for q in traj]),
        max_cmd_w=max((abs(r[7]) for r in p.rows), default=0.0),
        v_end=sc.pose_speed(traj, t_end - 0.5, t_end),
        mpc_xtrack_last=(p.rows[-1][8] if p.rows else 0.0),
        arrival_err=(
            math.hypot(*(a - b for a, b in zip(
                sc.pose_at(traj, p.reached_t or t_end), (gx, gy))))),
        idle_err=math.hypot(traj[-1][1] - gx, traj[-1][2] - gy),
        start=(traj[0][1], traj[0][2], traj[0][3]),
    )
    return met


def fmt(met):
    if met.get("error"):
        return f"  {met['tag']:10s} ✗ {met['error']} {met.get('sm_msg', '')}"
    rt = "—" if met["reached_at"] is None else f"{met['reached_at']:.1f} s"
    return (f"  {met['tag']:10s} 偏置 {met['off_bias_st']*100:+6.1f} cm | "
            f"max(稳态) {met['off_max_st']*100:5.1f} | "
            f"max(全程) {met['off_max_all']*100:5.1f} | "
            f"到点误差 {met['arrival_err']*100:5.1f} cm | "
            f"停稳后 {met['idle_err']*100:5.1f} | "
            f"用时 {rt} | {met['final_state']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="只试算路径，不动车")
    ap.add_argument("--goal", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"),
                    help="显式指定目标（不给就自动挑'最贴障碍'的那条路径）")
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--min-soft-frac", type=float, default=0.30,
                    help="场景合格线：路径上落入 0.45 带的点占比")
    ap.add_argument("--csv", default="")
    args = ap.parse_args()

    # ---- 挂栈前的守卫：**不要**再起一套 ----
    running = sc.running_pnc_nodes()
    if len(running) < 3:
        print(f"✗ 只找到 {running}；本脚本需要**已经在跑的** pnc_2d 三节点。")
        print(f"  先起： cd {sc.WS} && ros2 launch pnc_2d pnc_2d.launch.py "
              f"planner_type:=astar local_type:={os.environ.get('PNC2D_LOCAL_TYPE', 'heading_shim')} use_sim_time:=true "
              f"extra_config:={sc.GO2_CFG}")
        print("  （**不要**用 sim_common.launch()，那会起第二套栈）")
        return 2
    print(f"✓ 已有 pnc_2d：{', '.join(running)}（本脚本只挂上去，不起栈）")

    # ⚠ `sc.running_pnc_nodes()` 内部 init 完会 shutdown ⇒ 必须重新 init
    if not rclpy.ok():
        rclpy.init()
    p = AProbe()
    try:
        print("等位姿 + 两张图 …")
        if not p.wait_for(lambda: p.pose and p.global_map and p.local_map,
                          timeout=25.0):
            print(f"✗ 缺数据：pose={p.pose is not None} "
                  f"global_map={p.global_map is not None} "
                  f"local_map={p.local_map is not None}")
            return 2
        sx, sy, syaw = p.pose
        print(f"车 ({sx:.3f}, {sy:.3f}, {math.degrees(syaw):.1f}°)")
        gidx = sc.ObstacleIndex(p.global_map)
        lidx = sc.ObstacleIndex(p.local_map)
        print(f"起点净距：全局图 {gidx.clearance(sx, sy):.2f} m / "
              f"感知图 {lidx.clearance(sx, sy):.2f} m"
              f"（软阈值 {SOFT_SAFE} / 硬 {HARD}）")

        # ---------------- ① 试算：挑"路径贴障碍"的场景 ----------------
        if args.goal:
            cands = [(args.goal[0], args.goal[1], math.radians(args.goal[2]))]
            print(f"\n① 试算（显式目标 {args.goal[0]:.2f},{args.goal[1]:.2f}）")
        else:
            cands = []
            for dd in (2.5, 3.5, 4.5):
                for da in (-40.0, -20.0, 0.0, 20.0, 40.0):
                    a = syaw + math.radians(da)
                    cands.append((sx + dd * math.cos(a), sy + dd * math.sin(a),
                                  syaw))
            print(f"\n① 试算 {len(cands)} 个候选目标（只规划、不动车）")
        print(f"  {'目标':>18s}{'状态':>10s}{'点数':>6s}{'长度':>7s}"
              f"{'全局净距':>9s}{'感知净距':>9s}{'感知均值':>9s}"
              f"{'软带占比':>9s}{'钻深':>7s}{'机头差':>8s}")
        ok = []
        for gx, gy, gyaw in cands:
            res = p.plan(gx, gy, gyaw)
            if res is None:
                print(f"  ({gx:6.2f},{gy:6.2f})   服务无响应")
                continue
            st = path_stats(res.path, gidx, lidx)
            if st is None:
                print(f"  ({gx:6.2f},{gy:6.2f})"
                      f"{res.status_name[:9]:>10s}  （无路径）{res.message[:40]}")
                continue
            pts = [(ps.pose.position.x, ps.pose.position.y)
                   for ps in res.path.poses]
            hd = heading_diff(pts[0], pts[1], syaw)
            print(f"  ({gx:6.2f},{gy:6.2f}){res.status_name[:9]:>10s}"
                  f"{st['n']:>6d}{st['len']:>7.2f}{st['gc_min']:>9.2f}"
                  f"{st['lc_min']:>9.2f}{st['lc_mean']:>9.2f}"
                  f"{st['soft_frac']*100:>8.0f}%{st['soft_depth']*100:>6.1f}"
                  f"cm{hd:>7.1f}°")
            if res.success and hd <= 45.0:
                ok.append((st, (gx, gy, gyaw), res))
        if not ok:
            print("✗ 没有一个候选能规划出可用路径（都失败 or 机头差 > 45°）")
            return 3
        # 选"软带占比最高"，并列时取钻得更深的
        ok.sort(key=lambda t: (t[0]["soft_frac"], t[0]["soft_depth"]), reverse=True)
        best_st, best_goal, _ = ok[0]
        print(f"\n★ 选中目标 ({best_goal[0]:.2f}, {best_goal[1]:.2f}): "
              f"路径 {best_st['len']:.2f} m，感知净距 min "
              f"{best_st['lc_min']:.2f} m，"
              f"**{best_st['soft_frac']*100:.0f}% 的点在 {SOFT_SAFE} 带内**"
              f"（钻深 {best_st['soft_depth']*100:.1f} cm）")
        if best_st["soft_frac"] < args.min_soft_frac:
            print(f"⚠ 软带占比 < {args.min_soft_frac*100:.0f}% —— 这个场景"
                  f"**压不进软代价生效区**，A/B 会得到\"两趟都一样\"的假结论。"
                  f"建议换个起点/目标（把车挪到更贴通道的地方）再试。")
        if args.dry_run:
            print("\n（--dry-run：到此为止，不动车）")
            return 0

        # ---------------- ② A 趟（原权重）----------------
        w0 = get_weight()
        print(f"\n② A 趟（{WEIGHT_PARAM} 原值：{w0}）")
        metA = run_leg(p, best_goal, "A-原权重", args.timeout)
        metA["lc_min"] = best_st["lc_min"]
        metA["soft_frac"] = best_st["soft_frac"]
        print(fmt(metA))

        # ---------------- ③ 折返（回到 A 的起点，给 ④ 同一个起点）----------------
        start_goal = (metA["start"][0], metA["start"][1], metA["start"][2])
        print(f"\n③ 折返到 A 的起点 ({start_goal[0]:.2f}, {start_goal[1]:.2f}, "
              f"{math.degrees(start_goal[2]):.1f}°)（这一趟只为了摆位置，"
              f"不看指标）")
        metB = run_leg(p, start_goal, "B-折返", args.timeout)
        print(fmt(metB))
        if metB.get("error"):
            print("⚠ 折返失败 ⇒ ④ 的起点与 A 不同，横向偏置不再完全可比")

        # ---------------- ④ C 趟（obstacle_weight = 0）----------------
        print(f"\n④ C 趟（{WEIGHT_PARAM} 1000 → 0，重载参数后同目标重跑）")
        set_weight(0.0)
        print(f"    现在参数值：{get_weight()}")
        p.spin(2.0)                      # 让新参数生效并清掉残留状态
        metC = run_leg(p, best_goal, "C-权重0", args.timeout)
        metC["lc_min"] = best_st["lc_min"]
        metC["soft_frac"] = best_st["soft_frac"]
        print(fmt(metC))

        # ---- 恢复原值（纪律：只临时置 0）----
        raw = w0.split(":")[-1].strip()
        try:
            set_weight(float(raw))
            print(f"\n已恢复 {WEIGHT_PARAM} = {raw}")
        except ValueError:
            set_weight(1000.0)
            print(f"\n⚠ 读不回原值（{w0}）⇒ 已恢复为 1000.0，请人工核对")

        # ---------------- 对照 ----------------
        print("\n" + "=" * 74)
        print("M5.0 第 3 步 A/B：软代价假设（路径贴障碍时是否把车侧推）")
        print("=" * 74)
        print(f"目标 ({best_goal[0]:.2f}, {best_goal[1]:.2f})，路径 "
              f"{best_st['len']:.2f} m，感知净距 min {best_st['lc_min']:.2f} m，"
              f"软带占比 {best_st['soft_frac']*100:.0f}%")
        print(f"  {'趟':10s}{'偏置':>10s}{'max(稳态)':>11s}{'到点误差':>10s}"
              f"{'停稳后':>9s}{'用时':>9s}")
        for m in (metA, metC):
            if m.get("error"):
                print(f"  {m['tag']:10s} ✗ {m['error']}")
                continue
            rt = "—" if m["reached_at"] is None else f"{m['reached_at']:.1f}s"
            print(f"  {m['tag']:10s}{m['off_bias_st']*100:>+9.1f}cm"
                  f"{m['off_max_st']*100:>10.1f}cm"
                  f"{m['arrival_err']*100:>9.1f}cm"
                  f"{m['idle_err']*100:>8.1f}cm{rt:>9s}")
        if not metA.get("error") and not metC.get("error"):
            d = abs(metA["off_bias_st"]) - abs(metC["off_bias_st"])
            print(f"\n⇒ |偏置|：原权重 {abs(metA['off_bias_st'])*100:.1f} cm → "
                  f"权重 0 {abs(metC['off_bias_st'])*100:.1f} cm"
                  f"（差 {d*100:+.1f} cm）")
            if d > 0.02:
                print("   **偏置随软代价消失而变小** ⇒ 机制 = 软代价把车从"
                      "贴障碍的路径上侧推。修法在**全局侧**（让路径自己就别进 "
                      f"{SOFT_SAFE} 带：提高 planner.clearance_prefer_dist 的"
                      "权重），不要去动局部安全网。")
            elif d < -0.02:
                print("   偏置反而变大 ⇒ 与假设相反，先别下结论：确认两趟起点/"
                      "路径一致（折返那趟是不是没回到原位）。")
            else:
                print("   偏置几乎不变 ⇒ **软代价不是这个偏置的来源**；"
                      "按 doc §M5.0 的判据，接下来查硬侧（footprint / "
                      "obstacle_hard_distance）与底盘侧向。")
        if args.csv:
            import csv as _csv
            with open(args.csv, "w", newline="", encoding="utf-8") as f:
                w = _csv.writer(f)
                keys = [k for k in metA if k not in ("goal", "start", "state")]
                w.writerow(["leg"] + keys)
                for m in (metA, metB, metC):
                    w.writerow([m.get("tag", "")] + [m.get(k, "") for k in keys])
            print(f"\nCSV → {args.csv}")
        return 0
    finally:
        p.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
