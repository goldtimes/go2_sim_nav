"""pnc_2d 仿真测试的公共部分（在**已经跑起来的仿真栈**上跑）。

两个测试脚本共用：
  · `test_drive_goal.py` —— 自由空间直线行驶（astar + mpc）
  · `test_route_lane.py` —— 贴线行驶（route_network + mpc，走廊约束）

这里放：起停被测三节点、位姿/状态高频采样、障碍净距索引、路径几何指标。
**指标一律用位姿算**（twist 低速不可用，见下）。

⚠ 三条踩过的铁律（都会让指标"看起来像真的"却完全错）
  1. 别用 `ros2 topic echo` 轮询做计时：每次调用 0.5~1.5 s 启动开销，"sleep 1 s"
     实际间隔可能 3~4 s（我据此算出过 0.46 m/s 的假速度）。要进程内定时器采样。
  2. `/pnc_2d/global_path` 是 **latched** 的，订阅瞬间会收到**上一轮任务**残留的路径
     （用它算横向误差会得出 12.9 m 这种离谱值）⇒ 给目标前必须清掉。
  3. lightning 的 `twist.linear.x` 低速时是噪声（±0.1，均值都偏低），末速/速度保真度
     一律用**位姿差分**；净距用**发布的全局图**（含禁行区），不用裸 PGM。
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pnc_2d.msg import LocalStatus, ManagerState

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "e2e"))
from e2e_common import MAP_DIR, load_map_msg  # noqa: E402,F401

WS = os.environ.get("PNC2D_WS") or os.path.abspath(
    os.path.join(HERE, "..", "..", "..", ".."))
LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
GO2_CFG = os.environ.get(
    "PNC2D_EXTRA_CFG",
    os.path.join(WS, "src", "pnc_2d", "config", "go2_run.yaml"))


# ------------------------------------------------------------------ 起停
def launch(planner="astar", extra_cfg=None):
    cmd = ("ros2 launch pnc_2d pnc_2d.launch.py "
           f"planner_type:={planner} local_type:=mpc use_sim_time:=true "
           f"extra_config:={extra_cfg or GO2_CFG}")
    log = tempfile.NamedTemporaryFile(delete=False, suffix=".log")
    proc = subprocess.Popen(["bash", "-lc", cmd], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
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


def summary(script_dir=None):
    print(f"\n汇总：{CHECKS['pass']} 通过 / {CHECKS['fail']} 失败")
    return 1 if CHECKS["fail"] else 0


# ------------------------------------------------------------------ 障碍净距
class ObstacleIndex:
    """发布的**全局图**的占据格索引（算"轨迹离障碍最近多少"）。

    这只是**离线度量**：规划器自己用的是 A* 的 footprint 检查 + ESDF，这里用独立
    一张图重算，避免"用被测量的东西去验证自己"。
    ⚠ 必须用发布的全局图（map_server 把禁行区烧进去了），不能用裸 PGM ——
      裸图里禁行区是空的，会选出"在禁行区里"的目标，A* 直接规划失败。
    """

    def __init__(self, grid: OccupancyGrid, thresh=50):
        self.res = grid.info.resolution
        self.ox = grid.info.origin.position.x
        self.oy = grid.info.origin.position.y
        self.w, self.h = grid.info.width, grid.info.height
        self.occ = [(ix, iy) for iy in range(self.h) for ix in range(self.w)
                    if grid.data[iy * self.w + ix] >= thresh]
        self.buckets = {}
        for ix, iy in self.occ:
            self.buckets.setdefault((ix // 20, iy // 20), []).append((ix, iy))

    def clearance(self, x, y):
        """到最近占据格的净距 [m]。

        只搜所在 bucket 的 ±1 邻域（每个 bucket 20×20 格），所以返回值在
        **距障碍 >1.0 m 时被截到 1.0**：再大对我们没意义（阈值才 0.45 m），
        而直接返回哨兵值会印出 `1000000000.00` 这种“看起来像 bug”的数。
        """
        cx, cy = int((x - self.ox) / self.res), int((y - self.oy) / self.res)
        best = 1e9
        for k in [(cx // 20 + a, cy // 20 + b) for a in (-1, 0, 1)
                  for b in (-1, 0, 1)]:
            for ix, iy in self.buckets.get(k, ()):  # noqa: B905
                d = math.hypot((ix + 0.5) * self.res + self.ox - x,
                               (iy + 0.5) * self.res + self.oy - y)
                best = min(best, d)
        return min(best, 1.0)


# ------------------------------------------------------------------ 采样
class Probe(Node):
    """订阅位姿/局部状态/管理器状态/全局路径，记录高频轨迹。"""

    def __init__(self, name="p5_sim_probe"):
        super().__init__(name)
        self.t0 = time.time()
        self.rows = []      # 局部状态 (t, x, y, yaw, v_twist, status, cmd_v,
                            #           cmd_w, cross_track, progress)
        self.traj = []      # 高频位姿 (t, x, y, yaw) —— 指标主源
        self.local_msgs = 0
        self.state_seen = set()
        self.final_state = ""
        self.last_sm_msg = ""
        self.route_mode_seen = False   # 局部是否进过走廊模式（route 验收要看）
        self.global_map = None
        self.first_path = None
        self.first_path3 = None
        self.pose = None
        self.pose_v = 0.0
        self.reached_t = None    # ★ "到达目标"首次出现的时刻：把"判定时误差"
                                 #   与"停机后滑行"分开量（3 cm 验收必须分清）

        self.create_subscription(Odometry, "/lightning/perception/pose",
                                 self.on_pose, LIVE)
        self.create_subscription(LocalStatus, "/pnc_2d/local_status",
                                 self.on_status, LATCHED)
        self.create_subscription(ManagerState, "/pnc_2d/state", self.on_state,
                                 LATCHED)
        self.create_subscription(OccupancyGrid, "/global_map/occupancy",
                                 self.on_global_map, LATCHED)
        self.create_subscription(Path, "/pnc_2d/global_path", self.on_path, LATCHED)
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 1)

    def on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.pose_v = m.twist.twist.linear.x   # 低速时不可信，只作参考
        self.traj.append((time.time() - self.t0, self.pose[0], self.pose[1], yaw))

    def on_status(self, m):
        self.local_msgs += 1
        if m.route_mode:
            self.route_mode_seen = True
        if self.pose:
            # ★ 末位记 route_mode：“严格贴线期间”和“自由上线期间”的横向偏差
            #   是两回事 —— 混在一起算会把“上线过程的外摆”当成“贴线不合格”。
            #   贴线质量必须在 route_mode=true 的样本上量。
            self.rows.append((time.time() - self.t0, self.pose[0], self.pose[1],
                              0.0, self.pose_v, m.status_name, m.cmd_v, m.cmd_w,
                              m.cross_track_m, m.progress, bool(m.route_mode)))

    def on_state(self, m):
        self.state_seen.add(m.state_name)
        self.final_state = m.state_name
        if m.state_name == "GOAL_REACHED" and self.reached_t is None:
            self.reached_t = time.time() - self.t0
        if m.message:
            self.last_sm_msg = m.message

    def on_global_map(self, m):
        self.global_map = m

    def on_path(self, m):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]
        if len(pts) >= 2 and self.first_path is None:
            self.first_path = pts
            # 连朝向一起记："参考里有没有朝向尖刺"只能从这里看出来
            # （朝向误差会积分成横向偏差：n·e(k) = lat0 + dt·v·Σe_ψ）
            self.first_path3 = []
            for ps in m.poses:
                q = ps.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                self.first_path3.append((ps.pose.position.x,
                                         ps.pose.position.y, yaw))

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for(self, cond, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def send_goal(self, x, y, yaw=0.0):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.z = math.sin(0.5 * yaw)
        g.pose.orientation.w = math.cos(0.5 * yaw)
        self.pub_goal.publish(g)

    def begin(self):
        """清空采样并重置时间基准（**给目标前必须调**：顺带清掉 latched 旧路径）"""
        self.rows.clear()
        self.traj.clear()
        self.first_path = None
        self.first_path3 = None
        self.state_seen.clear()
        self.reached_t = None
        self.t0 = time.time()


# ------------------------------------------------------------------ 指标
def pose_at(traj, t):
    """轨迹里时刻 ≥ t 的第一个位姿（拿不到就返回最后一个）

    用于把"**判定到达那一刻**的误差"与"停机后滑行"分开 —— 3 cm 这种量级的
    验收必须分清这两者，否则会去改错的东西（踩过：以为是判定阈值太松，
    实际是判完之后车又滑了 4 cm）。
    """
    for q in traj:
        if q[0] >= t:
            return (q[1], q[2])
    return (traj[-1][1], traj[-1][2]) if traj else (0.0, 0.0)


def poly_length(pts):
    """折线长度 [m]（注意与 `path_distance` 不同：那个是两条线之间的最大偏离）"""
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1)) if len(pts) > 1 else 0.0


def goal_candidates(obstacle, start, want, heading=None, max_turn=75.0,
                    min_clear=0.45, step=0.25):
    """起点周围所有**足够空的直线**方向，按"走得远 + 净距大"排序返回候选目标。

    不许假设"起点+4m 一定是空的"：仿真里的车会停在上一次任务结束的地方（可能贴着
    墙/禁行区），`起点+dist` 很可能落在障碍里 → A* 直接报规划失败，测试看起来像
    "控制器坏了"。

    `heading` 给定时只在**车前 ±max_turn** 内找：目标在正后方时，全局路径首点朝向
    （保留当前朝向）与末点（目标朝向）差 180°，车只能绕个大 U 形（实测走了 5.18 m /
    路径 3.75 m），那种弧线不该拿来当"跟踪误差"的验收场景。

    ★ 几何够空 ≠ 全局可达（可能在大障碍另一侧、或目标的 footprint 摆不下），所以调用
    方要准备**换下一个候选**，别断言"第一个候选一定规划得出来"。
    """
    order = list(range(0, 360, 10))
    if heading is not None:
        d0 = math.degrees(heading)
        order.sort(key=lambda a: abs((a - d0 + 180) % 360 - 180))
    scored = []
    for ang_deg in order:
        if heading is not None:
            diff = abs((ang_deg - math.degrees(heading) + 180) % 360 - 180)
            if diff > max_turn:
                continue
        a = math.radians(ang_deg)
        d, minc = 0.0, 1e9
        while d < want:
            d += step
            c = obstacle.clearance(start[0] + d * math.cos(a),
                                   start[1] + d * math.sin(a))
            minc = min(minc, c)
            if c < min_clear:
                break
        if d - step < 1.0:      # 走不了 1 m 以上的方向不要
            continue
        scored.append(((d, minc), ang_deg, d - step, minc))
    scored.sort(key=lambda s: s[0], reverse=True)
    out = []
    for _score, ang_deg, d, minc in scored:
        a = math.radians(ang_deg)
        out.append((start[0] + d * math.cos(a), start[1] + d * math.sin(a), d,
                    minc, ang_deg))
    return out


def pick_goal(obstacle, start, want, heading=None, max_turn=75.0,
              min_clear=0.45, step=0.25):
    """`goal_candidates` 里最好的那个（只要一个目标时用这个）"""
    c = goal_candidates(obstacle, start, want, heading, max_turn, min_clear, step)
    return c[0] if c else None


def signed_offset(pose, poly):
    """点到路径折线的**带符号**横向偏差 [m]（左侧为正）。

    独立于 MPC 自报的 `cross_track`，这样"控制器说它在线上"不能自证。
    ⚠ `pose` 是轨迹里的整行 `(t, x, y, yaw)` —— 索引别取错（踩过：把 t 当 x，
    横向误差算出 16 m 这种离谱值，而且看起来还挺"像真的"）。
    """
    x, y = pose[1], pose[2]
    best = (1e9, 0.0)
    for (ax, ay), (bx, by) in zip(poly, poly[1:]):
        dx, dy = bx - ax, by - ay
        L2 = max(1e-12, dx * dx + dy * dy)
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        px, py = ax + dx * t, ay + dy * t
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            cross_z = dx * (y - py) - dy * (x - px)
            best = (d, cross_z)
    return best[0] if best[1] >= 0 else -best[0]


def pose_speed(traj, t_lo, t_hi):
    """位姿差分算平均速度 [m/s]（比 twist 可靠得多，尤其低速/停车时）"""
    seg = [p for p in traj if t_lo <= p[0] <= t_hi]
    if len(seg) < 2:
        return 0.0
    d = sum(math.hypot(seg[i + 1][1] - seg[i][1], seg[i + 1][2] - seg[i][2])
            for i in range(len(seg) - 1))
    return d / max(1e-6, seg[-1][0] - seg[0][0])


def path_distance(a, b):
    """两段折线间"一个方向上的最大偏离"（a 上每点到 b 的最近距离的最大值）"""
    best = 0.0
    for x, y in a:
        dmin = 1e9
        for (ax, ay), (bx, by) in zip(b, b[1:]):
            dx, dy = bx - ax, by - ay
            L2 = max(1e-12, dx * dx + dy * dy)
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            dmin = min(dmin, math.hypot(x - (ax + dx * t), y - (ay + dy * t)))
        best = max(best, dmin)
    return best


def load_limits():
    """从 go2_run.yaml 读速度上限（不在测试里写死，避免与配置脱节）"""
    import yaml
    doc = yaml.safe_load(open(GO2_CFG, encoding="utf-8"))
    p = list(doc.values())[0]["ros__parameters"]
    return (float(p["local_mpc.v_max"]), float(p["local_mpc.w_max"]),
            float(p["local_mpc.degraded_speed_limit"]))


def load_goal_yaw_tol_deg():
    """到点**朝向**容差 [°]：从 local_mpc.yaml 读（不在测试里写死；容器用的是
    算法片段里的值，与节点走 `goalYawTolerance()` 取的是同一个参数）"""
    import os
    import yaml
    cfg = os.path.join(os.path.dirname(GO2_CFG), "local_mpc.yaml")
    doc = yaml.safe_load(open(cfg, encoding="utf-8"))
    p = list(doc.values())[0]["ros__parameters"]
    return float(p.get("local_mpc.goal_yaw_tolerance_deg", 0.0))


def wrap_pi(a: float) -> float:
    """角度归一化到 (−π, π]（与库里的 `pnc_2d::wrapAngle` 同语义）"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a
