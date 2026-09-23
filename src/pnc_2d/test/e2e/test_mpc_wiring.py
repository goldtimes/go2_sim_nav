#!/usr/bin/env python3
"""E2E-6：MPC 局部规划的**接线**验收（P5.3）。

为什么必须端到端测：P5.1/P5.2 的单测把算法本身证得很干净（跟踪 RMSE、KKT、
走廊零违反……），但它们全都在**直接调库**。真正上路的链路还有四段没被任何一个
单测覆盖，而且这四段恰恰是最容易"静默失效"的地方：

  1. 节点有没有真的订阅 `topics.local_map` / `topics.esdf`；
  2. 感知发过来的点云字段布局能不能被解析（★ 实测：PCL 的 PointXYZI 是
     x@0 y@4 z@8 **intensity@16**，point_step=**32** —— 因为 PCL_ADD_POINT4D 把
     前 4 个 float 对齐了。任何"顺手假设 intensity@12、step=16"的解析都会读到垃圾
     或者直接抛异常，但**不会报错**，只会让车莫名其妙地不敢走）；
  3. 底盘速度有没有喂进去（setCurrentVelocity）；
  4. 拿不到距离场时是不是真的降级限速（fail-safe 的方向必须是对的）。

验收标准：
  [A] 完全没数据 → 局部报 DEGRADED（不是 FOLLOWING），且 |cmd_v| ≤ 降级限速；
  [B] 只给膨胀图、不给距离场 → 仍然 DEGRADED（图不能替代场）；
  [C] 给"空点云"→ **必须是 FOLLOWING**。
      ★ 这是本测试最想守住的语义：空点云 = 这一片很开阔（感知只发 0<d≤max_dist
        的格子），**不是**"拿不到距离场"。含混的代价是车在空旷大路上以 0.3 m/s
        爬行，而日志看起来像"感知挂了"—— 这种故障排查成本极高。
  [D] 给一堵挡在路上的墙（真实布局，200 个采样）→ 行为必须变化：BLOCKED 或明显减速；
      同时日志里应出现"200 采样"，证明解析器认得出真实布局；
  [E] 全程速度/角速度不越界（v_max/w_max 从 yaml 读，不在测试里写死）。

★ 测试里的"车"是**闭环**的：把收到的 /pnc_2d/cmd_vel 做差速运动学积分，位姿再喂回去。
  这不是花活，是必需的：
    · 管理器有**卡住判据**（默认 10 s 内位移 < 0.2 m → 取消跟随）。位姿一直不动，
      任务会被管理器自己取消掉，后面的阶段根本跑不到（第一版测试就是这样红的）；
    · 只有车真的动起来，setCurrentVelocity 那条接线（把底盘速度当 MPC 的状态量）
      才被真正走到——而它恰恰是"走走停停"类故障的根源。

用法：python3 test_mpc_wiring.py
"""

from __future__ import annotations

import math
import os
import re
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField

from pnc_2d.msg import LocalStatus, ManagerState

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
# 与感知发布端一致：reliable + depth 1 + volatile（实测过）
LIVE = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from e2e_common import load_map_msg, MAP_DIR  # noqa: E402

WS = os.environ.get("PNC2D_WS") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
CFG = os.path.join(WS, "src", "pnc_2d", "config", "local_mpc.yaml")

NS = "/e2e_mpc"          # 与真实系统、与其它 E2E 都错开
T = "/t_mpc"


# ------------------------------------------------------------------ 真实点云布局
# 实测自 /grid_map/esdf_2d（perception_node）：
#   fields = [x@0, y@4, z@8, intensity@16]，point_step = 32，height = 1
# 这里**刻意用真实布局**造点云，这样"有人把解析改成硬编码偏移"会被本测试抓住。
ESDF_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=16, datatype=PointField.FLOAT32, count=1),
]
POINT_STEP = 32


def make_esdf(pts, frame_id="map") -> PointCloud2:
    """pts = [(x, y, d)] → PointCloud2（PointXYZI：intensity 就是到障碍的距离 [m]）"""
    m = PointCloud2()
    m.header.frame_id = frame_id
    m.height = 1
    m.width = len(pts)
    m.is_bigendian = False
    m.is_dense = True
    m.point_step = POINT_STEP
    m.row_step = POINT_STEP * len(pts)
    m.fields = list(ESDF_FIELDS)
    m.data = b"".join(struct.pack("<ffff", x, y, 0.0, d) + b"\x00" * 16
                      for x, y, d in pts)
    return m


def wall(x0, x1, yc, half, d, nx=40, ny=5):
    """一堵"墙"：x∈[x0,x1]、y∈yc±half 的采样点，距离都是 d [m]"""
    pts = []
    for i in range(nx):
        x = x0 + (x1 - x0) * (i / max(1, nx - 1))
        for j in range(ny):
            y = yc - half + 2 * half * (j / max(1, ny - 1))
            pts.append((x, y, d))
    return pts


# ------------------------------------------------------------------ 隔离配置
def _isolate_config():
    """只把**外部** IO 指到 /t_mpc/*；自家话题靠 ns 自动搬走。

    ⚠ 段落必须是 `/**`：本文件用 `ns:=/e2e_mpc` 起节点，而有命名空间时
      **按节点名写的段完全不匹配**（FQN 变成 /e2e_mpc/global_planner）。
      这个坑在 E2E-5 里踩过一次，这里直接沿用正确写法。
    """
    doc = {"/**": {"ros__parameters": {
        "topics.map": f"{T}/map",
        "topics.odom": f"{T}/odom",
        "topics.goal": f"{T}/goal",
        # ↓ 只对 local 节点有意义：把局部图的来源换成测试自己发的
        "topics.local_map": f"{T}/local_map",
        "topics.esdf": f"{T}/esdf",
        # ↓ 只对 manager 有意义：阶段 A/B 是**故意**不动的（验证降级），
        #   不放宽的话 10 s 后管理器会按"卡住"把任务取消，后面的阶段全都跑不到。
        "sm.stuck_timeout": 120.0,
    }}}
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(doc, f, allow_unicode=True)
    f.close()
    return f.name


def launch(local_type, planner_type="astar"):
    extra = _isolate_config()
    cmd = ("ros2 launch pnc_2d pnc_2d.launch.py "
           f"planner_type:={planner_type} local_type:={local_type} "
           f"ns:={NS} extra_config:={extra}")
    log = tempfile.NamedTemporaryFile(delete=False, suffix=".log")
    proc = subprocess.Popen(["bash", "-lc", cmd], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log.name


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=8)
            return
        except subprocess.TimeoutExpired:
            continue
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


CHECKS = {"pass": 0, "fail": 0}


def chk(desc, cond, detail=""):
    if cond:
        CHECKS["pass"] += 1
        print(f"  PASS {desc}")
    else:
        CHECKS["fail"] += 1
        print(f"  FAIL {desc}" + (f"   ← {detail}" if detail else ""))


def read_limits():
    """从 local_mpc.yaml 读速度上限（不在测试里写死，避免与配置脱节）"""
    doc = yaml.safe_load(open(CFG, encoding="utf-8"))
    p = list(doc.values())[0]["ros__parameters"]
    return float(p["local_mpc.v_max"]), float(p["local_mpc.w_max"]), \
        float(p["local_mpc.degraded_speed_limit"])


# ------------------------------------------------------------------ 探针
class Probe(Node):
    def __init__(self, start):
        super().__init__("pnc2d_p5_probe")
        self.states: list = []
        self.local: list = []
        self.cmd_vel: list = []
        self.pose = list(start)          # [x, y, yaw]，会被下面按 cmd_vel 积分
        self.drive = False               # 是否让"车"真的动
        self.traveled = 0.0              # 累计位移（用于"真的走了"断言）
        self._last_cmd = Twist()
        self._last_t = None
        # 测试自己扮演"感知"：默认什么都不发（模拟感知没起/没数据）
        self.pub_local_map = self.create_publisher(OccupancyGrid, f"{T}/local_map", LIVE)
        self.pub_esdf = self.create_publisher(PointCloud2, f"{T}/esdf", LIVE)
        self.send_map = False
        self.send_esdf = False       # ★ 与局部图分开：B 阶段要"只给图不给场"
        self.esdf_pts: list = []

        self.create_subscription(ManagerState, f"{NS}/pnc_2d/state",
                                 self.states.append, LATCHED)
        self.create_subscription(LocalStatus, f"{NS}/pnc_2d/local_status",
                                 self.local.append, LATCHED)
        self.create_subscription(Twist, f"{NS}/pnc_2d/cmd_vel",
                                 self._on_cmd, 10)

        self.pub_map = self.create_publisher(OccupancyGrid, f"{T}/map", LATCHED)
        self.pub_odom = self.create_publisher(Odometry, f"{T}/odom", BEST_EFFORT)
        self.pub_goal = self.create_publisher(PoseStamped, f"{T}/goal", 1)
        self.create_timer(0.05, self._tick)
        # 距离场 10 Hz（本地默认 esdf_timeout 0.3 s，必须比它密）
        self.create_timer(0.1, self._tick_esdf)

    def _on_cmd(self, msg: Twist):
        self.cmd_vel.append(msg)
        self._last_cmd = msg

    def _tick(self):
        t = time.time()
        if self._last_t is None:
            self._last_t = t
            return
        dt = min(0.2, t - self._last_t)
        self._last_t = t
        if self.drive:
            # 差速底盘：x += v cos\u03b8 dt, y += v sin\u03b8 dt, \u03b8 += \u03c9 dt
            v, w = self._last_cmd.linear.x, self._last_cmd.angular.z
            self.pose[0] += v * math.cos(self.pose[2]) * dt
            self.pose[1] += v * math.sin(self.pose[2]) * dt
            self.pose[2] += w * dt
            self.traveled += abs(v) * dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "map"
        o.pose.pose.position.x = self.pose[0]
        o.pose.pose.position.y = self.pose[1]
        o.pose.pose.orientation.z = math.sin(0.5 * self.pose[2])
        o.pose.pose.orientation.w = math.cos(0.5 * self.pose[2])
        # ★ 真实的底盘速度会跟着位姿一起回给局部节点（MPC 把 v 当状态量）
        o.twist.twist.linear.x = self._last_cmd.linear.x if self.drive else 0.0
        o.twist.twist.angular.z = self._last_cmd.angular.z if self.drive else 0.0
        self.pub_odom.publish(o)
        if self.send_map:
            self.pub_local_map.publish(self._local_grid())

    def _local_grid(self) -> OccupancyGrid:
        """按**实测的真实几何**造局部膨胀图：80x80 @ 0.1 m，内容是"全空闲"
        （本 E2E 只测接线，障碍信息走 ESDF）。

        原点跟随车体（真实的滑动地图也是这样，实测原点 = 车 -4.1,-4.0），
        这样车动起来后 ESDF 仍然落在图内。
        """
        g = OccupancyGrid()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.info.resolution = 0.1
        g.info.width, g.info.height = 80, 80
        g.info.origin.position.x = -4.1 + self.pose[0]
        g.info.origin.position.y = -4.0 + self.pose[1]
        g.info.origin.orientation.w = 1.0
        g.data = [0] * (80 * 80)
        return g

    def _tick_esdf(self):
        if self.send_esdf:
            self.pub_esdf.publish(make_esdf(self.esdf_pts))

    def send_goal(self, x, y, yaw=0.0):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.z = math.sin(0.5 * yaw)
        g.pose.orientation.w = math.cos(0.5 * yaw)
        self.pub_goal.publish(g)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def win(self, seconds, since=0):
        """取一段时间窗内的 (局部状态, cmd_vel) 样本"""
        self.spin(seconds)
        loc = self.local[since:]
        cmd = self.cmd_vel[since:]
        return loc, cmd


def stats(loc, cmd, v_max, w_max, degraded_limit):
    names = [m.status_name for m in loc]
    vs = [abs(m.cmd_v) for m in loc]
    ws = [abs(m.cmd_w) for m in loc]
    msgs = [m.message for m in loc if m.message]
    return {
        "names": names,
        "seen": set(names),
        "msgs": msgs,
        "max_v": max(vs) if vs else 0.0,
        "max_v_status": max(vs) if vs else 0.0,
        "min_v_status": min(vs) if vs else 0.0,
        "max_w": max(ws) if ws else 0.0,
        "cmd_n": len(cmd),
        "twist_max_v": max((abs(t.linear.x) for t in cmd), default=0.0),
        "twist_max_w": max((abs(t.angular.z) for t in cmd), default=0.0),
        "route_mode": any(m.route_mode for m in loc),
        "solve_ms": max((m.solve_time_ms for m in loc), default=0.0),
        "cross_track": max((abs(m.cross_track_m) for m in loc), default=0.0),
    }


def route_plumbing_check():
    """阶段 G：route 模式的走廊能不能一路传到局部（P5.3b）。

    这条链路横跨四个地方：RouteNetworkPlanner → PlanPath.srv → manager → action →
    local。任何一段断了，**表面上完全看不出来**：路径仍然是沿通道算的，只是局部
    拿不到宽度，于是当自由空间跟——"贴线走"静默失效（实际就静默失效了好几轮）。
    所以这里直接看两处独立证据：manager 日志的走廊摘要 + 局部状态里的 route_mode。

    ★ 2026-09-23 修正（P5.4 “先自由上线、再严格贴线”之后 G1 必领新语义）：
      走廊**不再在收到目标时立即生效**，而是“车回到走廊里”之后才启用
      （从车道外用硬约束收敛实测不可行）。所以本阶段必须**让车真的开**
      （`probe.drive = True` + 给图/给场，否则 MPC 降级爬行），并等它上线；
      以前车是“钉在原地”的（drive=False），在旧语义下 route_mode 也会立刻为
      true，于是能勉强通过 —— 现在那样永远等不到，只能看到一直 `走廊行0`。
    """
    proc, log = launch("mpc", planner_type="route_network")
    rclpy.init()
    probe = Probe(start=(0.0, 0.03, 0.0))
    probe.drive = True          # 必须让车动：走廊只在“车已在走廊里”后生效
    probe.send_map = True
    probe.send_esdf = True
    ok = True
    try:
        probe.spin(6.0)
        probe.pub_map.publish(load_map_msg(MAP_DIR))
        probe.spin(1.5)
        probe.send_goal(9.85, -5.33)     # 路网上的点（与 E2E-1/2/3 同一个目标）
        # 等它“回到走廊里”（上限 30 s；上线后立即可停）
        route = False
        t0 = time.time()
        while time.time() - t0 < 30.0:
            probe.spin(0.5)
            if any(m.route_mode for m in probe.local):
                route = True
                break
        seen = {m.status_name for m in probe.local}
        print(f"       局部状态 {sorted(seen)} / route_mode={route} "
              f"/ 走了 {probe.traveled:.2f} m")
        chk("[G1] ★ route 模式到了局部（route_mode=true）", route,
            f"从来没报过 route_mode，状态 {sorted(seen)}，走了 {probe.traveled:.2f} m")
        txt = open(log, encoding="utf-8", errors="replace").read()
        chk("[G2] manager 把走廊写进了日志（说明响应里真的带上了）",
            "走廊 有" in txt, "日志里没有『走廊 有』")
        chk("[G3] 局部也收到了走廊（跟目标时打印）", "走廊 有" in txt,
            "日志里没有『走廊 有』")
        m = re.search(r"走廊 有（半宽 ([\d.]+) m，限速 ([\d.]+) m/s", txt)
        chk("[G4] 半宽取的是 routes.yaml 里的 0.60 m",
            bool(m) and abs(float(m.group(1)) - 0.60) < 1e-6,
            m.group(0) if m else "没解析出半宽")
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        stop(proc)
    return log if not ok else log


def main():
    v_max, w_max, deg_limit = read_limits()
    print(f"配置速限：v_max={v_max} w_max={w_max} 降级限速={deg_limit}")
    proc, log = launch("mpc")
    rclpy.init()
    probe = Probe(start=(0.0, 0.03, 0.0))
    try:
        probe.spin(6.0)
        names = [n for n in probe.get_node_names() if n.endswith(
            ("global_planner", "local_planner", "pnc_manager"))]
        chk("[0] 三节点起来（local_type:=mpc）", len(names) == 3, f"{sorted(names)}")

        probe.pub_map.publish(load_map_msg(MAP_DIR))
        probe.spin(1.5)
        # 目标放远一点：x 方向到 21.8 m 都是空的（扫过地图确认），
        # 这样车在 C/D 阶段能真跑起来而不会提前到达（提前到达会被管理器
        # 判定 GOAL_REACHED 并收尾，后面的阶段就没路径可跟了）。
        GOAL = (12.0, 0.0)
        probe.send_goal(*GOAL)
        probe.spin(8.0)
        chk("[0b] 管理器进入 FOLLOWING（局部拿到了路径）",
            "FOLLOWING" in {s.state_name for s in probe.states},
            f"{[s.state_name for s in probe.states]}")

        # ---------------- A：什么都不给 ----------------
        print("\n--- A：完全没数据（局部图/距离场都没发）---")
        a_start = len(probe.local)
        loc, cmd = probe.win(2.5, a_start)
        A = stats(loc, cmd, v_max, w_max, deg_limit)
        print(f"       状态 {sorted(A['seen'])} / max|v|={A['max_v']:.3f} "
              f"/ 收到 cmd_vel {A['cmd_n']} 条")
        chk("[A1] 无数据 → DEGRADED（不是 FOLLOWING）",
            "DEGRADED" in A["seen"] and "FOLLOWING" not in A["seen"],
            f"{sorted(A['seen'])}")
        chk("[A2] 降级时限速（≤ %.2f m/s）" % deg_limit,
            A["max_v"] <= deg_limit + 1e-6, f"max|v|={A['max_v']:.3f}")

        # ---------------- B：只给膨胀图 ----------------
        print("\n--- B：只给膨胀图（不给距离场）---")
        probe.send_map = True
        probe.win(1.0)
        b_start = len(probe.local)
        loc, cmd = probe.win(2.5, b_start)
        B = stats(loc, cmd, v_max, w_max, deg_limit)
        print(f"       状态 {sorted(B['seen'])} / max|v|={B['max_v']:.3f}")
        chk("[B1] 只有图、没有场 → 仍然 DEGRADED", "DEGRADED" in B["seen"],
            f"{sorted(B['seen'])}")
        chk("[B2] 仍然限速", B["max_v"] <= deg_limit + 1e-6, f"max|v|={B['max_v']:.3f}")

        # ---------------- C：空点云（= 开阔区，不是没数据）----------------
        print("\n--- C：空点云（周围无障碍），并让车真的跑起来 ---")
        probe.esdf_pts = []                      # 空点云
        probe.send_esdf = True                   # 从这里开始才发距离场
        probe.drive = True                       # ★ 从这里开始闭环驱动
        probe.win(1.5)                           # 等超时逻辑与 MPC 反应
        c_start = len(probe.local)
        x0 = probe.pose[0]
        loc, cmd = probe.win(3.0, c_start)
        C = stats(loc, cmd, v_max, w_max, deg_limit)
        moved = probe.pose[0] - x0
        print(f"       状态 {sorted(C['seen'])} / max|v|={C['max_v']:.3f} "
              f"/ 收到 cmd_vel {C['cmd_n']} 条 / 最长求解 {C['solve_ms']:.2f} ms"
              f"/ 前进 {moved:.2f} m")
        chk("[C1] ★ 空点云 → FOLLOWING（不能当成缺距离场）",
            "FOLLOWING" in C["seen"] and "DEGRADED" not in C["seen"],
            f"{sorted(C['seen'])}")
        chk("[C2] 不再限速（max|v| 明显高于降级限速）",
            C["max_v"] > deg_limit + 0.2, f"max|v|={C['max_v']:.3f}")
        chk("[C3] ★ 真的发出了速度指令（MPC 上线前这里一条都没有）",
            C["cmd_n"] > 10, f"收到 {C['cmd_n']} 条")
        chk("[C4] 速度/角速度不越界", C["twist_max_v"] <= v_max + 1e-6 and
            C["twist_max_w"] <= w_max + 1e-6,
            f"|v|max={C['twist_max_v']:.3f} |w|max={C['twist_max_w']:.3f}")
        # 闭环：命令真的驱动了底盘，而且底盘速度回给 MPC 后没有把它憋死
        chk("[C5] ★ 闭环真的在前进（cmd_vel → 位姿积分，速度反馈不憋车）",
            moved > 1.0, f"只前进了 {moved:.2f} m")

        # ---------------- D：一堵挡在路上的墙 ----------------
        xr, yr = probe.pose[0], probe.pose[1]
        print(f"\n--- D：墙横在路上（x∈[车+1.0, 车+2.0]，d=0.05 m，车在 x={xr:.2f}）---")
        probe.esdf_pts = wall(xr + 1.0, xr + 2.0, yr, 0.4, 0.05)   # 40x5 = 200 点
        print(f"       发布 {len(probe.esdf_pts)} 个采样点")
        # ★ 立即开始取样：局部连续 1.0 s 报 BLOCKED 就会结束跟随（这是对的），
        #   所以取样窗口必须覆盖那 1 s，否则只能看到它已经收工的空窗口。
        d_start = len(probe.local)
        loc, cmd = probe.win(1.4, d_start)
        D = stats(loc, cmd, v_max, w_max, deg_limit)
        print(f"       状态 {sorted(D['seen'])} / max|v|={D['max_v']:.3f}"
              f"/ 消息 {D['msgs'][-2:]}")
        chk("[D1] ★ 墙的代价/约束真的进了规划器（BLOCKED 或明显减速）",
            bool(D["names"]) and ("BLOCKED" in D["seen"] or
                                  D["max_v"] < C["max_v"] - 0.15),
            f"D.max|v|={D['max_v']:.3f} vs C.max|v|={C['max_v']:.3f} "
            f"状态 {sorted(D['seen'])} 样本 {len(D['names'])}")
        chk("[D2] 墙也不会让它越界", D["twist_max_v"] <= v_max + 1e-6 and
            D["twist_max_w"] <= w_max + 1e-6,
            f"|v|max={D['twist_max_v']:.3f} |w|max={D['twist_max_w']:.3f}")
        probe.drive = False
        probe.send_esdf = False

        # ---------------- E：日志证明"真实布局被解析对了" ----------------
        txt = open(log, encoding="utf-8", errors="replace").read()
        chk("[E1] 日志确认收到 esdf 点云（订阅真的接上了）",
            "收到距离场" in txt or "距离场重建" in txt,
            "日志里没有距离场相关行")
        # 贪婪取最后一条 "N 采样"（D 阶段应是 200）
        counts = [int(m) for m in re.findall(r"(\d+) 采样", txt)]
        print(f"       日志里的采样数序列：{counts[-6:]}")
        chk("[E2] ★ 解析器认得出真实布局（intensity@16 / point_step=32）",
            200 in counts,
            f"没在日志里看到 \"200 采样\"（实际 {counts[-6:]}）")

        chk("[E3] A*（自由空间）下 route_mode 应为 false（走廊只由路网给）",
            not C["route_mode"], "free 模式的 route_mode 竟然是 true")

        # ---------------- F：反例 —— local_type:=none 不许发速度 ----------------
        print("\n--- F：反例 local_type:=none ---")
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        stop(proc)

    proc2, log2 = launch("none")
    rclpy.init()
    probe2 = Probe(start=(0.0, 0.03, 0.0))
    try:
        probe2.spin(6.0)
        probe2.pub_map.publish(load_map_msg(MAP_DIR))
        probe2.spin(1.5)
        probe2.send_goal(12.0, 0.0)
        probe2.spin(8.0)
        pubs = probe2.count_publishers(f"{NS}/pnc_2d/cmd_vel")
        chk("[F1] none 下 cmd_vel 无发布者", pubs == 0, f"发现 {pubs} 个")
        chk("[F2] none 下一条速度也不发", not probe2.cmd_vel,
            f"收到 {len(probe2.cmd_vel)} 条")
    finally:
        probe2.destroy_node()
        rclpy.shutdown()
        stop(proc2)

    # ---------------- G：route 模式的走廊贯穿（P5.3b） ----------------
    print("\n--- G：route_network 模式下走廊能否一路传到局部 ---")
    log3 = route_plumbing_check()

    print()
    print(f"汇总：{CHECKS['pass']} 通过 / {CHECKS['fail']} 失败")
    if CHECKS["fail"]:
        for name, path in (("mpc", log), ("none", log2), ("route", log3)):
            print(f"--- {name} 日志尾部（{path}） ---")
            print("".join(open(path, encoding="utf-8",
                               errors="replace").readlines()[-25:]))
    return 1 if CHECKS["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
