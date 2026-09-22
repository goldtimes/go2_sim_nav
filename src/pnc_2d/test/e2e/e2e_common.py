"""pnc_2d 端到端测试的公共工具（ROS 图级别，不是纯 C++ 单测）。

三个脚本（clear / restart / hot_switch）共用这一份：

· 起一个**隔离的** global_planner 节点（节点名 gp_e2e，话题重映射到 /t/*），
  跑完自己收掉 → 不干扰正在运行的系统；
· **自带地图**：直接从站点的 map.pgm / map.yaml 造 OccupancyGrid 并以 latched 发出，
  语义与 map_server 一致（未知格按 unknown_as_free 当空闲），所以不需要 map_server 在跑；
· 提供位姿/目标发布、路径/状态/标记订阅、服务客户端与断言计数。

约定（可用环境变量覆盖）：
    PNC2D_MAP_DIR  站点目录，默认 /home/gmd/rcs/maps/go2_sim_factory
    PNC2D_ROUTES   路网文件，默认 <站点>/routes.yaml
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from std_srvs.srv import Trigger

from pnc_2d.msg import PlannerStatus
from pnc_2d.srv import SwitchPlanner

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

MAP_DIR = os.environ.get("PNC2D_MAP_DIR", "/home/gmd/rcs/maps/go2_sim_factory")
ROUTES = os.environ.get("PNC2D_ROUTES", os.path.join(MAP_DIR, "routes.yaml"))
NODE = "gp_e2e"


def _node_executable() -> str | None:
    """直接跑 install 里的可执行文件。

    ⚠ 不要用 `ros2 run`：它是个 wrapper，terminate() 只杀掉 wrapper，真正的节点
    会变成**孤儿进程**继续跑——多个实例抢同一话题会让测试出现各种灵异现象
    （重复消息、别人发的空路径）。直接 exec 才能干净地收掉。
    """
    ws = os.environ.get("PNC2D_WS") or os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    exe = os.path.join(ws, "install", "pnc_2d", "lib", "pnc_2d",
                       "global_planner_node")
    return exe if os.path.isfile(exe) else None

# 隔离话题：与真实系统（/goal_pose、/pnc_2d/global_path…）完全错开
T_MAP, T_ODOM, T_GOAL = "/t/map", "/t/odom", "/t/goal"
T_PATH, T_STATUS, T_MARKERS = "/t/path", "/t/status", "/t/markers"


# --------------------------------------------------------------------- 地图
def load_map_msg(map_dir: str = MAP_DIR) -> OccupancyGrid:
    """map.pgm + map.yaml → OccupancyGrid（未知格按空闲发布，与 map_server 一致）"""
    meta = yaml.safe_load(open(os.path.join(map_dir, "map.yaml"), encoding="utf-8"))
    with open(os.path.join(map_dir, "map.pgm"), "rb") as f:
        assert f.readline().strip() == b"P5", "只支持 P5 二进制 PGM"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        f.readline()  # maxval
        px = f.read(w * h)
    ot = meta["occupied_thresh"]
    data = []
    for row in range(h - 1, -1, -1):  # OccupancyGrid 的 y=0 在地图底部
        for col in range(w):
            v = (255.0 - px[row * w + col]) / 255.0
            data.append(100 if v > ot else 0)
    m = OccupancyGrid()
    m.header.frame_id = "map"
    m.info.resolution = float(meta["resolution"])
    m.info.width, m.info.height = w, h
    m.info.origin.position.x = float(meta["origin"][0])
    m.info.origin.position.y = float(meta["origin"][1])
    m.info.origin.orientation.w = 1.0
    m.info.map_load_time.sec = 1
    m.data = data
    return m


# --------------------------------------------------------------------- 路网
def lane_polyline(routes: str = ROUTES, edge: int = 0):
    """取第 edge 条通道的折线（用于判断路径有没有"贴线走"）"""
    doc = yaml.safe_load(open(routes, encoding="utf-8"))
    edges = doc.get("edges", [])
    if not edges:
        return []
    return [(p[0], p[1]) for p in edges[edge]["polyline"]]


def dev_to_lane(pts, poly) -> float:
    """一组点到折线的最大偏离 [m]；偏离小 = 沿路网走"""
    best = 0.0
    for x, y in pts:
        dmin = 1e9
        for (ax, ay), (bx, by) in zip(poly, poly[1:]):
            dx, dy = bx - ax, by - ay
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) /
                                 max(1e-12, dx * dx + dy * dy)))
            dmin = min(dmin, math.hypot(x - (ax + dx * t), y - (ay + dy * t)))
        best = max(best, dmin)
    return best


# --------------------------------------------------------------------- 被测节点
def start_node(harness: "Harness", planner: str = "route_network", params=(),
               routes: str = ROUTES) -> subprocess.Popen:
    """起隔离实例；跑完用 stop_node() 收掉"""
    exe = _node_executable()
    argv = [exe] if exe else ["ros2", "run", "pnc_2d", "global_planner_node"]
    args = argv + ["--ros-args",
                   "-r", f"__node:={NODE}",
                   "-r", f"/global_map/occupancy:={T_MAP}",
                   "-r", f"/lightning/perception/pose:={T_ODOM}",
                   "-r", f"/goal_pose:={T_GOAL}",
                   "-r", f"/pnc_2d/global_path:={T_PATH}",
                   "-r", f"/pnc_2d/plan_markers:={T_MARKERS}",
                   "-p", f"topics.status:={T_STATUS}",
                   "-p", f"planner.type:={planner}",
                   "-p", f"route_network.routes_file:={routes}"]
    for p in params:
        args += ["-p", p]
    log = open(f"/tmp/pnc2d_e2e_{os.getpid()}.log", "a")
    # start_new_session：整个进程组一起收，避免残留
    proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    time.sleep(3.0)          # 等它订阅上并收到 latched 地图
    harness.publish_map()
    spin(harness, 2.0)
    return proc


def stop_node(proc: subprocess.Popen) -> None:
    """收掉节点：先整组 SIGTERM，超时再 SIGKILL（wrapper/子进程都不会漏）"""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


# --------------------------------------------------------------------- 测试节点
class Harness(Node):
    """订阅路径/状态/标记，发布位姿/目标/地图，自带断言计数"""

    def __init__(self, start=(0.0, 0.03)):
        super().__init__("pnc2d_e2e")
        self.paths: list = []
        self.statuses: list = []
        self.markers: list = []
        self.create_subscription(Path, T_PATH, self.paths.append, LATCHED)
        self.create_subscription(PlannerStatus, T_STATUS, self.statuses.append,
                                 LATCHED)
        from visualization_msgs.msg import MarkerArray
        self.create_subscription(MarkerArray, T_MARKERS, self.markers.append,
                                 LATCHED)
        self.pub_odom = self.create_publisher(Odometry, T_ODOM, BEST_EFFORT)
        self.pub_goal = self.create_publisher(PoseStamped, T_GOAL, 1)
        self.pub_map = self.create_publisher(OccupancyGrid, T_MAP, LATCHED)
        self.srv_clear = self.create_client(Trigger, f"/{NODE}/clear_path")
        self.srv_reload = self.create_client(Trigger, f"/{NODE}/reload_params")
        self.srv_switch = self.create_client(SwitchPlanner, f"/{NODE}/switch_planner")
        self.create_timer(0.05, self._tick)
        self.pose = start
        self._n_pass = 0
        self._n_fail = 0

    # ---- 数据面 ----
    def _tick(self):
        m = Odometry()
        m.header.frame_id = "map"
        m.pose.pose.position.x, m.pose.pose.position.y = self.pose
        m.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(m)

    def publish_map(self, map_dir: str = MAP_DIR):
        self.pub_map.publish(load_map_msg(map_dir))

    def goal(self, x, y, yaw_w: float = 1.0):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.w = yaw_w
        self.pub_goal.publish(g)

    @property
    def path(self) -> Path | None:
        return self.paths[-1] if self.paths else None

    @property
    def status(self) -> PlannerStatus | None:
        return self.statuses[-1] if self.statuses else None

    @property
    def marker_array(self):
        return self.markers[-1] if self.markers else None

    def n_poses(self) -> int:
        return len(self.path.poses) if self.path else 0

    def points(self):
        return [(p.pose.position.x, p.pose.position.y) for p in self.path.poses] \
            if self.path else []

    def markers_in(self, ns: str, action: int | None = None):
        arr = self.marker_array
        if arr is None:
            return []
        return [m for m in arr.markers
                if m.ns == ns and (action is None or m.action == action)]

    def saw_delete(self, ns: str) -> bool:
        """整窗里是否出现过 ns 的 DELETE。

        只能扫"全部收到的消息"，不能只看最后一条：清场消息发出后可能又被
        后续的空 MarkerArray 顶掉，而**空数组并不能删掉 RViz 里的旧标记**。
        """
        for arr in self.markers:
            for m in arr.markers:
                if m.ns == ns and m.action == 2:  # 2 = DELETE
                    return True
        return False

    def saw_add(self, ns: str) -> bool:
        for arr in self.markers:
            for m in arr.markers:
                if m.ns == ns and m.action == 0:  # 0 = ADD
                    return True
        return False

    # ---- 断言 ----
    def chk(self, name: str, cond: bool, extra=""):
        if cond:
            self._n_pass += 1
            print(f"  PASS {name}")
        else:
            self._n_fail += 1
            print(f"  FAIL {name}" + (f"  <{extra}>" if extra else ""))
        return bool(cond)

    def summary(self) -> int:
        total = self._n_pass + self._n_fail
        print(f"RESULTS: {self._n_pass}/{total} pass")
        return 0 if self._n_fail == 0 else 1


def spin(node: Node, sec: float) -> None:
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(node: Harness, client, request, timeout: float = 3.0):
    """同步调服务（内部 spin）"""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(request)
    spin(node, 2.0)
    return fut.result() if fut.done() else None
