#!/usr/bin/env python3
"""局部避障仿真验收 S1~S4（在**已经跑起来的仿真栈**上跑）。

    tmuxp load -d go2_sim_bringup.json          # gazebo + lightning + perception
    ros2 launch map_server map_server.launch.py
    python3 test/sim/test_avoidance.py --scenarios S1,S2,S3,S4 --backend gazebo

四个场景（判定标准与"为什么"）
------------------------------------------------
**S1 静态堵中段** —— 全局路径 45% 弧长处正中放一个 0.6 m 宽的箱子，两侧留得开。
  期望：**不 BLOCKED、绕过去、到达**。这是"灵活避障"的主用例。
**S2 动态横穿** —— 箱子以 `--speed` 从路径一侧横穿到另一侧。
  期望：减速/让/短停后继续，**不碰**。允许出现 BLOCKED（停下来等是合法行为），
  但必须能恢复。
**S3 宽箱堵死** —— 放一个 `--s3-width`（默认 1.8 m）宽的箱子 ⇒ 绕不过。
  期望：**如实报 BLOCKED**（消息里能看出原因）、**停在障碍前**（不蹭着往前顶）。
  "如实上报"是安全属性：绕不过却硬顶 = 撞。
**S4 撤障恢复** —— 同 S3，`--remove-at` 秒后把障碍撤走。
  期望：**自动恢复并到达**（验证 BLOCKED → 恢复 → 重新规划这条链不会卡死）。

指标口径（都写进汇总表）
------------------------------------------------
· **到点误差**：用位姿算（twist 低速不可信，见 `sim_common.py` 铁律 1）
· **最小净距（对障碍）**：轨迹每个采样点到障碍**当刻真实位姿**的带符号距离
  （障碍位姿由 `obstacle_inject.py` 广播，不自己按脚本重算时间基准 —— 两个进程
  的 t0 不同，重算必然对不齐）。>0 = 没碰，<0 = 已穿进去。
· **BLOCKED**：状态为 BLOCKED 的采样条数与消息集合（原因要可分辨）
· **侧向偏离**：轨迹相对**原全局路径**的最大横向偏移 ⇒ "有没有真的绕"
· **卡死**：progress 长时间不前进的最长时长

⚠ 本脚本**不自带仿真**：它只起 pnc_2d 三节点（`sim_common.launch`，含"拒绝双栈"），
  障碍由 `obstacle_inject.py` 子进程按场景注入。仿真没起时会在第 1 步就明确报错。
⚠ 验收前先确认"现象真的发生了"：RViz 里看 `/grid_map/occupancy_2d`，确认感知把
  箱子标成了占据 —— 否则测的是"感知没看见"，不是避障（这类假象我们栽过）。
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

# ⚠ 解释器护栏（同 obstacle_inject.py）：仓库的 .venv 是 Python 3.14，
#   与 Humble 的 rclpy 不兼容，而终端里经常正好激活着它。
try:
    import rclpy
except Exception as _exc:  # noqa: BLE001
    raise SystemExit(
        f"导入 rclpy 失败：{_exc}\n"
        "  ⚠ 本仓库的 .venv 是 Python 3.14，与 Humble 的 rclpy 不兼容\n"
        f"  ⇒ 请用系统解释器：/usr/bin/python3 {__file__}")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.qos import (DurabilityPolicy, QoSProfile,  # noqa: E402
                       ReliabilityPolicy)

import sim_common as sc  # noqa: E402
from sim_common import chk, stop, summary  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
INJECT = os.path.join(HERE, "obstacle_inject.py")
LAT_TOL_M = sc.load_goal_tolerances()[1]
# ★ 车体轮廓（与 pnc_2d.yaml 同源）："撞没撞"必须用它与障碍矩形比，而不是"车心距离"
FOOT_L, FOOT_W, FOOT_OX, FOOT_OY = sc.load_footprint()

# gz 服务 → ROS 服务的桥。**必须起**：实测（2026-09-29）Fortress 这套环境里
# `ign service --req` 走不通 —— 连 `ign msg -i ign.msgs.Pose` 都报
# "Unable to create message of type" ⇒ CLI 的消息工厂没加载，与 --req 的写法无关；
# 而 ROS 侧默认**没有**把这三个服务桥出来（ros_gz_bridge 只桥了话题）。
# 桥起来之后 `ros_gz_interfaces` 的 srv 就能直接用：实测 set_pose 返回 success=True。
BRIDGE_SRV = ("/world/%s/create@ros_gz_interfaces/srv/SpawnEntity",
              "/world/%s/set_pose@ros_gz_interfaces/srv/SetEntityPose",
              "/world/%s/remove@ros_gz_interfaces/srv/DeleteEntity")


def start_bridge(world, timeout=15.0):
    cmd = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + \
          [s % world for s in BRIDGE_SRV]
    print("  bridge:", " ".join(cmd))
    # ★ `start_new_session=True` + 下面按**进程组**杀：`ros2 run` 会 fork 出真正的
    #   bridge 子进程，只 terminate 父进程会留下孤儿桥 ⇒ 下次跑撞名、行为更怪。
    #   （另：**绝对不要**用 `pkill -f parameter_bridge` 清 —— 仿真自己也用同名进程
    #    起 gz↔ROS 桥，一杀就把 /clock 和机器人 odom/cmd_vel 全断了，2026-09-29
    #    真踩过，只能重启整套仿真。）
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT, start_new_session=True)
    end = time.time() + timeout
    while time.time() < end:
        try:
            out = subprocess.check_output(["ros2", "service", "list"],
                                          text=True, timeout=8)
            if ("/world/%s/set_pose" % world) in out:
                print("  bridge: 服务已就绪")
                return proc
        except Exception:  # noqa: BLE001
            pass
        time.sleep(1.0)
    print("  ⚠ 服务桥没起来：gazebo 后端会失败（可改用 --backend cloud）")
    return proc


def stop_proc(proc):
    """按进程组收掉子进程（同 sim_common.stop 的做法）"""
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            continue
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


# ------------------------------------------------------------------ 几何小工具
def polyline_at(pts, frac):
    """折线按**弧长比例**取点，返回 (x, y, 切线角, 全长)"""
    if len(pts) < 2:
        return pts[0][0], pts[0][1], 0.0, 0.0
    segs = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    total = sum(segs)
    want, acc = frac * total, 0.0
    for i, s in enumerate(segs):
        if s > 1e-9 and acc + s >= want:
            k = (want - acc) / s
            x = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * k
            y = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * k
            return x, y, math.atan2(pts[i + 1][1] - pts[i][1],
                                    pts[i + 1][0] - pts[i][0]), total
        acc += s
    return pts[-1][0], pts[-1][1], 0.0, total


def rect_clearance(px, py, cx, cy, sx, sy):
    """点到轴对齐矩形的**带符号**距离（>0 在外，<0 在里 = 穿透）"""
    dx = abs(px - cx) - sx / 2.0
    dy = abs(py - cy) - sy / 2.0
    if dx <= 0.0 and dy <= 0.0:
        return max(dx, dy)
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def lateral_of(px, py, pts):
    """点到折线的**最短距离**（近似横向偏离；只用作"有没有绕"的粗指标）"""
    best = 1e9
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            best = min(best, math.dist((px, py), (x0, y0)))
            continue
        t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
        best = min(best, math.dist((px, py), (x0 + t * dx, y0 + t * dy)))
    return best


# ------------------------------------------------------------------ 辅助探针
class AuxProbe(rclpy.node.Node):
    """两件 `sim_common.Probe` 不管的事：

      ① 订阅 `obstacle_inject.py` 广播的障碍**真实位姿**（动态场景算净距必须有它，
         不能自己按脚本重算 —— 两个进程的 t0 不同，重算必然对不齐）；
      ② 抓 `/pnc_2d/local_status` 的 `message`（S3 要看"被挡原因是否可分辨"）。
         `sim_common.Probe.rows` 只存 status_name，不存 message。

    ⚠ 调用方必须在自己的循环里 `rclpy.spin_once(aux, timeout_sec=0)` ——
      `sc.Probe.spin()` 只 spin 它自己，不会帮你转这个节点（否则净距恒为 inf）。
    """

    def __init__(self):
        super().__init__("avoidance_aux_probe")
        self.samples = []   # (t, x, y)
        self.msgs = set()   # BLOCKED/FAILED 时的 message
        self.t0 = time.time()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseStamped, "/perception_test/obstacle_pose",
                                 self.on_pose, qos)
        self.create_subscription(sc.LocalStatus, "/pnc_2d/local_status",
                                 self.on_status, sc.LATCHED)

    def on_pose(self, m):
        self.samples.append((time.time() - self.t0, m.pose.position.x,
                             m.pose.position.y))

    def on_status(self, m):
        if m.status_name in ("BLOCKED", "FAILED", "DEGRADED") and m.message:
            self.msgs.add(f"{m.status_name}: {m.message[:90]}")

    def pose_at(self, t):
        """最后一个 ≤ t 的位姿（拿不到就 None = 那一刻障碍不在）"""
        best = None
        for s in self.samples:
            if s[0] <= t:
                best = s
            else:
                break
        return best


# ------------------------------------------------------------------ 一次场景
class Result:
    def __init__(self, name):
        self.name = name
        self.reached = False
        self.err_goal = float("nan")
        self.min_clear = float("inf")
        self.min_clear_moving = float("inf")   # 只看“车在动”的那部分拍
        self.max_pen = 0.0          # 车体轮廓与障碍的最大穿透 [m]（0 = 没相交）
        self.max_pen_moving = 0.0   # 同上，但只看“车在动”的拍
        self.min_clear_ctx = None   # 最小净距那一刻的上下文（排查用）
        self.trace = []             # (t,rx,ry,status,bx,by,clear,车速,穿透)
        self.blocked_n = 0
        self.blocked_msgs = set()
        self.blocked_s = 0.0
        self.max_lat_dev = 0.0
        self.stall_s = 0.0
        self.plan_requests = 0
        self.recoveries = 0
        self.note = ""
        self.box = None   # (sx, sy) 本场景用的障碍尺寸（算净距用）


def start_injector(args, at, to, script, remove_at=None, sizes=None,
                   spawn_delay=0.0, start_delay=0.0, duration=180.0):
    """起一个注入器进程；**启动即退出时明确报警**（否则会静默变成"空测"）。

    ★ 负坐标必须写成 `--at=值`（不能 `--at 值`）：argparse 会把 `-3.187,-2.271`
      当成**选项** ⇒ "expected one argument" ⇒ 注入器秒退 ⇒ 既没箱子也没位姿广播
      ⇒ 指标变成"净距 inf / 0 拍 BLOCKED"的**假通过**（2026-09-29 实测吃了两轮）。
      同理把它的输出留到文件里：以后出问题能直接看它自己说了什么。
    """
    cmd = [sys.executable or "python3", INJECT, "--mode", args.backend,
           "--at=%.3f,%.3f" % at,
           "--sizes", sizes or args.sizes,
           "--script", script, "--speed", str(args.speed),
           "--spawn-delay", str(spawn_delay),
           "--start-delay", str(start_delay),
           "--duration", str(duration)]
    if getattr(args, "ghost", False):
        cmd += ["--ghost"]
    if not getattr(args, "solid", False):
        # ★ 默认 ghost（不参与物理碰撞）：验收要量的是**规划器**。实体箱会被车推着走
        #   （5 kg + 传送式驱动），而且 Go2 的 collision 盒远小于外观
        #   （实测躯干 0.376×0.094×0.114 vs 外观 0.46×0.19×0.19）⇒ 车会“钻进”箱子里
        #   40 cm 而物理不拦 —— 那个“穿透”量的是仿真，不是规划器。
        #   幽灵箱无法被推：规划器真开错了就是真穿过去，穿透指标一定抓得到。
        cmd += ["--ghost"]
    if to is not None:
        cmd += ["--to=%.3f,%.3f" % to]
    if remove_at is not None:
        cmd += ["--remove-at", str(remove_at)]
    if args.backend == "cloud":
        cmd += ["--out-topic", "/perception_test/cloud"]
    print("  inject:", " ".join(cmd))
    errf = tempfile.NamedTemporaryFile(prefix="inject_", suffix=".log",
                                       delete=False)
    proc = subprocess.Popen(cmd, stdout=errf, stderr=subprocess.STDOUT)
    time.sleep(1.2)
    if proc.poll() is not None:
        tail = ""
        try:
            errf.flush()
            with open(errf.name, encoding="utf-8", errors="ignore") as f:
                tail = "".join(f.readlines()[-6:])
        except Exception:                                          # noqa: BLE001
            tail = "(读不到注入器日志)"
        print("  \u26a0 注入器**启动即退出**（rc=%s）⇒ 本场景会白测"
              "（净距 inf / 0 拍 BLOCKED）！它的最后几行：\n%s"
              % (proc.returncode, tail.rstrip()))
    return proc


def run_scenario(name, args, p, obst, aux, res):
    """跑一个场景：选目标 → 订障碍 → 等结果 → 采集指标"""
    print(f"\n=== {name} ===")
    p.begin()
    p.final_state = ""
    p.state_seen.clear()
    p.last_sm_msg = ""

    # ---- 1) 选目标：
    #  ✗ 坑 1：**latched 残留**。`/pnc_2d/local_status` 与 `/pnc_2d/state` 是
    #    transient_local ⇒ 新订阅者会立刻收到**上一套栈的最后一条**，于是
    #    "等到 local_msgs>0" 瞬间成立、目标发得太早，管理器还没收到位姿 ⇒
    #    直接 `规划失败：还没收到位姿`（S1 第一次就是这么挂的，与
    #    sim_common 里那条"global_path 是 latched"的铁律同一类）。
    #  ✗ 坑 2：几何够空 ≠ 全局可达（大障碍另一侧 / 目标处摆不下 footprint）。
    #  ⇒ 按 `test_drive_goal` 的做法：**逐个候选重试**，且整轮选不出来就
    #    等栈就绪再重试。
    sx, sy, syaw = p.pose
    cands = sc.goal_candidates(obst, (sx, sy), args.dist, heading=syaw)
    if not cands:
        res.note = "找不到可行目标（车被围住？）"
        print("  " + res.note)
        return
    chosen = None
    deadline = time.time() + max(args.cand_rounds, 1) * 12.0
    while chosen is None and time.time() < deadline:
        for (cx, cy, cd, cmin, cang) in cands[:args.cand_tries]:
            cyaw = math.atan2(cy - sy, cx - sx)
            p.begin()
            p.final_state = ""
            p.send_goal(cx, cy, cyaw)
            t0 = time.time()
            while time.time() - t0 < 6.0:
                p.spin(0.1)
                if "FAILED" in p.state_seen:
                    break
                if "FOLLOWING" in p.state_seen and p.first_path \
                        and len(p.first_path) >= 2:
                    chosen = (cx, cy, cyaw)
                    break
            if chosen:
                print(f"  目标 ({cx:.2f}, {cy:.2f}) 距离 "
                      f"{math.dist((sx, sy), (cx, cy)):.2f} m，方向 {cang}°，"
                      f"直线最小净距 {cmin:.2f} m → 已进入跟随")
                break
            print(f"  候选 ({cx:.2f}, {cy:.2f}) 未进跟随（{sorted(p.state_seen)}）"
                  f" → 换下一个")
        if chosen is None:
            print("  本轮候选全失败（栈可能还没就绪）→ 3 s 后重试")
            time.sleep(3.0)
    if chosen is None:
        res.note = "选不出可跟随的目标（栈没就绪 / 全局规划一直失败）"
        print("  " + res.note)
        return
    gx, gy, gyaw = chosen

    # ---- 2) 路径已经拿到了（用**规划出来的那条**算障碍该放哪）
    ax, ay, ang_p, total = polyline_at(p.first_path, args.place_frac)
    nx, ny = -math.sin(ang_p), math.cos(ang_p)   # 路径左法向
    print(f"  路径 {len(p.first_path)} 点 / 全长 {total:.2f} m；"
          f"在 {args.place_frac*100:.0f}% 处放置障碍 @ ({ax:.2f}, {ay:.2f})"
          f"（切线 {math.degrees(ang_p):.1f}°）")

    # ---- 3) 按场景确定障碍的位姿与脚本
    SZ = [float(v) for v in args.sizes.split(",")]
    inj = None
    if name.startswith("S1"):
        res.box = (SZ[0], SZ[1])
        inj = start_injector(args, (ax, ay), None, "static")
    elif name.startswith("S2"):
        res.box = (SZ[0], SZ[1])
        w = args.s2_span
        # ★ start-delay：箱子先出现在路径**旁边**，等到机器人快要开到才横穿。
        #   ✗ 旧版写死 6 s，只对 ~6 m 的路径合适；路径一变短，机器人早就开过
        #     去了、箱子才从旁边扫过来 ⇒ 那一轮测出来的是"障碍扫到车"的假冲突。
        #   ⇒ 改成按路径长度自适应：箱子从 ±w 处横穿到"路径点"要 w/v_box 秒，
        #     让它在机器人**大约**到达该点时正好经过 ⇒ delay = t_arrive − w/v_box。
        t_arrive = args.place_frac * total / args.v_robot
        delay = args.s2_start_delay
        if delay is None:
            delay = max(0.0, t_arrive - w / args.speed)
        print(f"  S2 时序：路径 {total:.2f} m → 估到达 {t_arrive:.1f} s，横穿半个 "
              f"span {w/args.speed:.1f} s ⇒ 障碍 {delay:.1f} s 后出发"
              f"{'' if args.s2_start_delay is None else '（手动指定）'}")
        inj = start_injector(args, (ax + nx * w, ay + ny * w),
                             (ax - nx * w, ay - ny * w), "cross",
                             start_delay=delay,
                             duration=args.timeout + 30)
    elif name.startswith("S3"):
        res.box = (args.s3_width, SZ[1])
        inj = start_injector(args, (ax, ay), None, "static",
                             sizes="%s,%s,%s" % (args.s3_width, SZ[1], SZ[2]))
    elif name.startswith("S4"):
        res.box = (args.s3_width, SZ[1])
        inj = start_injector(args, (ax, ay), None, "static",
                             sizes="%s,%s,%s" % (args.s3_width, SZ[1], SZ[2]),
                             remove_at=args.remove_at,
                             duration=args.timeout + 30)

    # ---- 4) 跟到结束
    aux.t0 = time.time()
    aux.samples.clear()
    aux.msgs.clear()
    t_begin = time.time()
    last_progress, last_progress_t = -1.0, t_begin
    seen_blocked_since = None
    while time.time() - t_begin < args.timeout:
        p.spin(0.1)
        rclpy.spin_once(aux, timeout_sec=0.0)   # ★ 必须：否则净距算不出来
        now = time.time()
        # 净距：轨迹最新点到障碍**同一时刻**的位姿（时间基各自转成绝对时间）
        if p.traj and res.box is not None:
            t_abs = p.t0 + p.traj[-1][0]
            op = aux.pose_at(t_abs - aux.t0)
            if op is not None:
                _, ox, oy = op
                rx, ry = p.traj[-1][1], p.traj[-1][2]
                ryaw = p.traj[-1][3]
                cl = rect_clearance(rx, ry, ox, oy, res.box[0], res.box[1])
                # ★★ 真判据：**车体轮廓**（与 pnc_2d.yaml 同源的 footprint）与障碍
                #   矩形相交多少。原来的"车心→盒面 > 0.20 m"是错的：车体前沿在车心
                #   前 0.35 m ⇒ 0.256 m 看着"没碰"，实际车头已经插进去了（假通过）。
                pen = sc.rects_overlap(
                    sc.rect_corners(rx + FOOT_OX * math.cos(ryaw),
                                    ry + FOOT_OX * math.sin(ryaw), ryaw,
                                    FOOT_L, FOOT_W),
                    sc.rect_corners(ox, oy, 0.0, res.box[0], res.box[1]))
                st = p.rows[-1][5] if p.rows else "?"
                t_rel = now - t_begin
                # 车速：用上一条轨迹拍算（"是谁撞谁"只能靠它区分）
                rv = 0.0
                if res.trace and t_rel > res.trace[-1][0]:
                    rv = (math.dist((rx, ry), res.trace[-1][1:3]) /
                          (t_rel - res.trace[-1][0]))
                res.trace.append((t_rel, rx, ry, st, ox, oy, cl, rv, pen))
                if cl < res.min_clear:
                    res.min_clear = cl
                    res.min_clear_ctx = (t_rel, st, rx, ry, ox, oy)
                if pen > res.max_pen:
                    res.max_pen = pen
                if rv >= 0.1 and pen > res.max_pen_moving:
                    res.max_pen_moving = pen
            # 侧向偏离（相对**原全局路径**）—— "有没有真的绕"
            res.max_lat_dev = max(res.max_lat_dev,
                                  lateral_of(p.traj[-1][1], p.traj[-1][2],
                                             p.first_path))
        # 状态
        if p.rows:
            _, _, _, _, _, status, _, _, prog, *_ = p.rows[-1]
            if status == "BLOCKED":
                res.blocked_n += 1
                seen_blocked_since = seen_blocked_since or now
            elif seen_blocked_since is not None:
                res.blocked_s += now - seen_blocked_since
                seen_blocked_since = None
            if prog != last_progress:
                last_progress, last_progress_t = prog, now
            res.stall_s = max(res.stall_s, now - last_progress_t)
        if p.last_sm_msg and "BLOCK" in p.last_sm_msg.upper():
            res.blocked_msgs.add(p.last_sm_msg[:90])
        res.blocked_msgs |= aux.msgs
        if {"GOAL_REACHED", "FAILED"} & p.state_seen:
            # S4 要在撤障后继续等
            if not (name.startswith("S4") and "GOAL_REACHED" not in p.state_seen
                    and now - t_begin < args.remove_at + 5):
                break
    if seen_blocked_since is not None:
        res.blocked_s += time.time() - seen_blocked_since

    if inj is not None:
        try:
            inj.terminate()
            inj.wait(timeout=5)
        except Exception:
            inj.kill()

    # ---- 5) 结果
    res.reached = "GOAL_REACHED" in p.state_seen
    if p.pose:
        res.err_goal = math.dist(p.pose[:2], (gx, gy))
    print(f"  状态：{sorted(p.state_seen)} | 最终 {p.final_state} | "
          f"{p.last_sm_msg[:60]}")
    print(f"  到点误差 {res.err_goal*100:.1f} cm | 最小净距 {res.min_clear:.3f} m | "
          f"最大穿透 {res.max_pen*100:.1f} cm（车在动时 {res.max_pen_moving*100:.1f} cm）| "
          f"BLOCKED {res.blocked_n} 拍/{res.blocked_s:.1f} s | "
          f"最大侧偏 {res.max_lat_dev:.2f} m | 最长停滞 {res.stall_s:.1f} s")
    if res.min_clear_ctx is not None:
        t, st, rx, ry, ox, oy = res.min_clear_ctx
        print(f"  ⤷ 最小净距时刻 t={t:.1f}s 状态={st} 车({rx:.2f},{ry:.2f}) "
              f"障碍({ox:.2f},{oy:.2f}) 净距={res.min_clear:.3f} m")
    # 近冲突：净距 < 0.35 m 或 有穿透
    near = [i for i, r in enumerate(res.trace) if r[6] < 0.35 or r[8] > 0.0]
    if near:
        print(f"  近冲突轨迹（净距 < 0.35 m 或穿透 > 0，共 {len(near)} 拍）：")
        print("      t[s]  车x    车y   状态      障x    障y   净距  车速  障速  穿透")
        for i in near[:40]:
            t, rx, ry, st, bx, by, cl, _, pen = res.trace[i]
            rv = bv = float("nan")
            if i > 0:
                dt = t - res.trace[i - 1][0]
                if dt > 1e-6:
                    rv = math.dist((rx, ry), res.trace[i - 1][1:3]) / dt
                    bv = math.dist((bx, by), res.trace[i - 1][4:6]) / dt
            print(f"    {t:6.1f} {rx:6.2f} {ry:6.2f} {st:<9} {bx:6.2f} {by:6.2f} "
                  f"{cl:6.3f} {rv:5.2f} {bv:5.2f} {pen*100:5.1f}cm")
        if len(near) > 40:
            print(f"    …… 另有 {len(near)-40} 拍")
        if res.max_pen > 0.0 and res.max_pen_moving <= 0.02:
            print(f"  ⤷ 穿透只发生在车几乎不动时 ⇒ 是障碍扫到停着的车（不是车撞上去；"
                  f"本车不允许倒车，只能提醒）")


def wait_idle(p, timeout=15.0):
    """场景之间等状态机回到"可接新目标"（GOAL_REACHED / FAILED / IDLE）"""
    end = time.time() + timeout
    while time.time() < end:
        p.spin(0.1)
        if p.final_state in ("GOAL_REACHED", "FAILED", "IDLE"):
            return
    print("  ⚠ 等状态机复位超时，仍继续下一场景")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", default="S1",
                    help="逗号分隔：S1,S2,S3,S4")
    ap.add_argument("--backend", choices=("gazebo", "cloud"), default="gazebo")
    ap.add_argument("--world", default="world_demo", help="gz world 名（见 world sdf）")
    ap.add_argument("--dist", type=float, default=7.0, help="目标距起点 [m]")
    ap.add_argument("--place-frac", type=float, default=0.45,
                    help="障碍放在全局路径的弧长比例处")
    ap.add_argument("--sizes", default="0.6,0.6,0.8", help="S1/S2 的箱子尺寸")
    ap.add_argument("--s3-width", type=float, default=1.8,
                    help="S3/S4 用的宽箱宽度 [m]（要宽到绕不过）")
    ap.add_argument("--s2-span", type=float, default=1.5,
                    help="S2 横穿的半幅 [m]（从 −span 到 +span）")
    ap.add_argument("--s2-start-delay", type=float, default=None,
                    help="S2：箱子出现后多久开始横穿 [s]。**默认 None = 自适应**"
                         "（按路径长度估机器人到达时刻，旧的固定 6 s 只对 ~6 m "
                         "的路径合适；路径一变短，机器人早就开过去了、箱子"
                         "才从旁边扫过来 ⇒ 测出来的是假冲突）")
    ap.add_argument("--v-robot", type=float, default=0.45,
                    help="S2 自适应时序用的机器人平均速度 [m/s]（只估时序，"
                         "不参与判定）")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="S2 障碍速度 [m/s]。★ 2026-09-29 从 0.8 降到 0.3："
                         "0.8 m/s 的箱子**会把车撞飞**（实测净距 -0.026 m，并把 "
                         "Go2 推到贴着墙 → 之后全局规划一直 "
                         "START_FOOTPRINT_COLLISION），那是「障碍撞车」而不是"
                         "「车避障」，测不出局部避障的好坏；0.3 m/s ≈ 慢走的人/"
                         "慢速搬运车，局部图（~5 Hz）每拍它只动 6 cm ⇒ 追得上")
    ap.add_argument("--ghost", action="store_true",
                    help="障碍不参与物理碰撞（默认**就是** ghost，见 start_injector）")
    ap.add_argument("--solid", action="store_true",
                    help="障碍参与物理碰撞（测“车推着障碍走”时才要）")
    ap.add_argument("--remove-at", type=float, default=20.0,
                    help="S4 撤障时刻 [s]（相对障碍生成）")
    ap.add_argument("--timeout", type=float, default=90.0, help="单场景上限 [s]")
    ap.add_argument("--cand-tries", type=int, default=4,
                    help="目标候选试几个（几何够空 ≠ 全局可达）")
    ap.add_argument("--cand-rounds", type=int, default=3,
                    help="候选全失败时整轮重试几次（等栈就绪）")
    ap.add_argument("--keep-log", action="store_true")
    args = ap.parse_args()

    wish = [s.strip().upper() for s in args.scenarios.split(",") if s.strip()]
    bridge = None
    if args.backend == "gazebo":
        bridge = start_bridge(args.world)
    proc, log = sc.launch("astar")
    rclpy.init()
    p = sc.Probe("avoidance_probe")
    aux = AuxProbe()
    try:
        p.wait_for(lambda: p.pose is not None and p.local_msgs > 0, timeout=20.0)
        if p.pose is None:
            print("没有位姿：仿真/定位没起？（本脚本不自带仿真）")
            return 2
        if p.global_map is None:
            print("没有全局图：map_server 没起？")
            return 2
        obstacle_index = sc.ObstacleIndex(p.global_map)
        print(f"起点 ({p.pose[0]:.2f}, {p.pose[1]:.2f}) | 局部算法 "
              f"{os.environ.get('PNC2D_LOCAL_TYPE', 'heading_shim')} | "
              f"后端 {args.backend}")

        results = []
        for nm in wish:
            r = Result(nm)
            run_scenario(nm, args, p, obstacle_index, aux, r)
            results.append(r)
            wait_idle(p)

        # ---------------- 判定 ----------------
        print("\n--- 判定 ---")
        by = {r.name: r for r in results}

        def no_hit(r, tag):
            """“没撞上去”= **车体轮廓与障碍矩形没有相交**（SAT 穿透 = 0）。

            ★ 2026-09-29 换口径：原来用"车心→盒面 > 0.20 m"，但车体前沿在车心前
              0.35 m ⇒ S1 的 0.256 m 看着"没碰"，其实车头已经插进箱子（假通过）。
              穿透 > 0 就是真碰上，不再看中心距离。
            """
            chk(f"[{tag}] 车体轮廓没插进障碍（穿透 = 0）",
                r.max_pen <= 0.0,
                f"最大穿透 {r.max_pen*100:.1f} cm（车在动时 "
                f"{r.max_pen_moving*100:.1f} cm）；全程最小净距 {r.min_clear:.3f} m")

        if "S1" in by:
            r = by["S1"]
            chk("[S1] 绕过去了并到达", r.reached, r.note)
            no_hit(r, "S1")
            chk("[S1] 确实绕了（侧向偏离 > 0.25 m）",
                r.max_lat_dev > 0.25, f"最大侧偏 {r.max_lat_dev:.2f} m")
        if "S2" in by:
            r = by["S2"]
            no_hit(r, "S2")
            chk("[S2] 最终到达（允许中途停等）", r.reached)
            print(f"     （被挡 {r.blocked_s:.1f} s —— 允许，但应能恢复）")
        if "S3" in by:
            r = by["S3"]
            chk("[S3] 如实报 BLOCKED", r.blocked_n > 0, "整场没有 BLOCKED")
            no_hit(r, "S3")
            chk("[S3] 没有错误地当作已到达", not r.reached)
            if r.blocked_msgs:
                print(f"     被挡原因：{sorted(r.blocked_msgs)}")
        if "S4" in by:
            r = by["S4"]
            chk("[S4] 撤障后自动恢复并到达", r.reached, r.note)
            no_hit(r, "S4")

        print("\n--- 汇总表 ---")
        print("  场景 | 到达 | 到点误差 | 最小净距 | 最大穿透 | BLOCKED | 最大侧偏 | "
              "最长停滞")
        for r in results:
            print(f"  {r.name}  |  {'是' if r.reached else '否'}  | "
                  f"{r.err_goal*100:6.1f}cm | {r.min_clear:7.3f}m | "
                  f"{r.max_pen*100:6.1f}cm | "
                  f"{r.blocked_n:4d}拍/{r.blocked_s:5.1f}s | "
                  f"{r.max_lat_dev:6.2f}m | {r.stall_s:5.1f}s")
        if not args.keep_log:
            print(f"\n  pnc_2d 日志：{log}")
        return summary()
    finally:
        stop(proc)
        stop_proc(bridge)
        for n in (p, aux):
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
