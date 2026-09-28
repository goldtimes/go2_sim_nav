#!/usr/bin/env python3
"""诊断"全局路径贴障碍 ⇒ MPC 撞上"这件事（只规划，不动车）。

为什么单独一个脚本（而不是复用 soft_cost_ab）：
  · 用户报的是**一个具体目标**（14.27, 2.87, 87.9°）下的路径质量问题，要能把
    "路径逐点净距"打出来看**在哪里贴**、贴到什么程度；
  · 必须**同时看两张图**：全局图（规划器判据）+ 感知图（MPC 实际避障用的那张）
    —— 两张图不同源，历史上就出过"全局说 0.30、感知说..." 的分歧；
  · 要能**同一目标下 A/B** `traj.type`（A* 折线 vs MINCO 优化后），因为 MINCO 是
    **会挪线**的 —— 挪近还是挪远必须量。

用法（仿真 + 定位 + perception + pnc_2d 三节点在跑；本脚本**不起栈、不动车**）：
    python3 test/sim/check_path_clearance.py --goal 14.27 2.87 --yaw 87.9
    python3 test/sim/check_path_clearance.py --goal 14.27 2.87 --yaw 87.9 --ab
        # --ab：先按当前 traj.type 规划一次，再切成另一个值规划一次做对照
        #       （结束时恢复原值）。切 traj.type 走 ~/reload_params，不用重启。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HERE = "/home/gmd/r41_ws/src/pnc_2d/test/sim"
sys.path.insert(0, HERE)
import sim_common as sc                                          # noqa: E402

from pnc_2d.srv import PlanPath                                   # noqa: E402
from std_srvs.srv import Trigger                                  # noqa: E402

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Node(rclpy.node.Node):
    def __init__(self):
        super().__init__("check_path_clearance")
        self.pose = None
        self.gmap = None
        self.lmap = None
        self.create_subscription(Odometry, "/lightning/perception/pose",
                                 self.on_pose, LIVE)
        self.create_subscription(OccupancyGrid, "/global_map/occupancy",
                                 self.on_gmap, LATCHED)
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d",
                                 self.on_lmap, LIVE)

    def on_pose(self, m):
        self.pose = m.pose.pose

    def on_gmap(self, m):
        self.gmap = m

    def on_lmap(self, m):
        self.lmap = m

    def wait(self, pred, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False


def call(n: Node, srv_type, name, req, timeout=8.0):
    """同步调一个服务（带超时；失败就抛，不要静默）。"""
    cli = n.create_client(srv_type, name)
    try:
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"没有服务 {name}")
        fut = cli.call_async(req)
        t0 = time.time()
        while time.time() - t0 < timeout and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError(f"{name} 超时")
        return fut.result()
    finally:
        n.destroy_client(cli)


def pget(n: Node, node_name: str, key: str):
    """读**另一个节点**的参数（rclpy 的 set_parameters 只管自己，所以要服务）。"""
    r = call(n, GetParameters, f"/{node_name}/get_parameters",
             GetParameters.Request(names=[key]))
    return r.values[0].string_value if r and r.values else None


def pset(n: Node, node_name: str, key: str, value, reload_after=True):
    p = Parameter()
    p.name = key
    p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                             string_value=str(value))
    r = call(n, SetParameters, f"/{node_name}/set_parameters",
             SetParameters.Request(parameters=[p]))
    ok = bool(r and r.results and r.results[0].successful)
    # ★ 只 set 不 reload 是不生效的：全局节点在 reload 时才重建轨迹优化后端
    #   （buildTrajectoryOptimizer），而 traj.type 决定建不建那一个。
    if ok and reload_after:
        call(n, Trigger, f"/{node_name}/reload_params", Trigger.Request())
    return ok


def plan_once(n: Node, gx, gy, gyaw, timeout=12.0, start=None):
    """调 plan_path 服务（publish_result=false ⇒ 不发布、不动车）。

    `start=(x, y, yaw)` 时用**指定起点**（`use_current_pose=False`）—— 车当时在不在
    那儿无所谓，这样才能复现"用户报碰撞时的那个起点"而不必把车开回去。
    """
    req = PlanPath.Request()
    req.header.frame_id = "map"
    req.goal.header.frame_id = "map"
    req.goal.pose.position.x = float(gx)
    req.goal.pose.position.y = float(gy)
    req.goal.pose.orientation.z = math.sin(gyaw / 2.0)
    req.goal.pose.orientation.w = math.cos(gyaw / 2.0)
    if start is None:
        req.use_current_pose = True
    else:
        sx, sy, syaw = start
        req.use_current_pose = False
        req.start.header.frame_id = "map"
        req.start.pose.position.x = float(sx)
        req.start.pose.position.y = float(sy)
        req.start.pose.orientation.z = math.sin(syaw / 2.0)
        req.start.pose.orientation.w = math.cos(syaw / 2.0)
    req.publish_result = False
    return call(n, PlanPath, "/global_planner/plan_path", req, timeout)


def footprint_hits(grid, x, y, yaw, length=0.70, width=0.40, margin=0.05,
                   step=0.02):
    """车体矩形（含 margin）是否压到占据格 —— 与 C++ 的 `poseInCollision` 同口径，
    但这里是**连续采样**（C++ 会把位姿吸附到格心，这里不吸附 ⇒ 更准）。

    返回 (是否碰撞, 压住的点数)。为什么必须用它而不是"车心净距"：
      · `ObstacleIndex.clearance` 量的是**车心**到障碍格心的距离；
      · 而车体是 0.70×0.40 的矩形（外接圆半径 0.472 m）⇒ **车心净距 0.40 m 时，
        角点可能已经压进障碍**（0.472 > 0.40）。"车心够远"不等于"车体不碰"。
    """
    if grid is None:
        return None, 0
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    data = grid.data

    def lethal(px, py):
        ix, iy = int((px - ox) / res), int((py - oy) / res)
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return True  # 图外一律当障碍（与 C++ 同口径）
        return data[iy * w + ix] >= 50

    ca, sa = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0 + margin, width / 2.0 + margin
    nx = max(2, int(2 * hw / step) + 1)
    ny = max(2, int(2 * hl / step) + 1)
    hit = 0
    for j in range(ny):                     # 沿车体 x（前后）
        lx = -hl + (2 * hl) * j / (ny - 1)
        for i in range(nx):                 # 沿车体 y（左右）
            ly = -hw + (2 * hw) * i / (nx - 1)
            px = x + lx * ca - ly * sa
            py = y + lx * sa + ly * ca
            if lethal(px, py):
                hit += 1
    return hit > 0, hit


def footprint_sweep(grid, x, y, yaw_from, yaw_to, step_deg=5.0):
    """在**原地**从 yaw_from 转到 yaw_to，逐角度查车体矩形 —— 返回首次碰撞的角度。"""
    d = math.remainder(yaw_to - yaw_from, 2 * math.pi)
    n = max(1, int(abs(math.degrees(d)) / step_deg))
    for k in range(n + 1):
        yaw = yaw_from + d * k / n
        hit, cnt = footprint_hits(grid, x, y, yaw)
        if hit:
            return math.degrees(yaw), cnt
    return None, 0


def margin_at(grid, x, y, yaw, hi=0.50, tol=0.005):
    """这个位姿**还剩多少余量**：把车体矩形往外涨 m，不碰障碍的最大 m。

    这就是"跟踪误差预算"：MPC 横向偏差超过它，车体就压上去。
    `footprint_hits(..., margin=m)` 涨的就是四边，所以涨 m 相当于"车宽/车长各加
    2m" —— 反过来说 m_max 就是"还能容忍多少横向+纵向偏差"（保守取两者较小）。
    """
    if grid is None or footprint_hits(grid, x, y, yaw, margin=0.0)[0]:
        return 0.0                       # 真实轮廓就已经压着 ⇒ 余量为 0
    lo, hi_ = 0.0, hi
    while hi_ - lo > tol:
        mid = 0.5 * (lo + hi_)
        if footprint_hits(grid, x, y, yaw, margin=mid)[0]:
            hi_ = mid
        else:
            lo = mid
    return lo


def walk_path(poses, step=0.02):
    """沿路径（含点间插值）等距采样，产出 (x, y, yaw, 里程)。"""
    pts = [p.pose.position for p in poses]
    s = 0.0
    for i in range(len(pts) - 1):
        ax, ay = pts[i].x, pts[i].y
        bx, by = pts[i + 1].x, pts[i + 1].y
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(seg / step))
        for k in range(n + 1):
            t = k / n
            yield (ax + (bx - ax) * t, ay + (by - ay) * t,
                   math.atan2(by - ay, bx - ax), s + t * seg)
        s += seg


def path_footprint_scan(grid, idx, poses, step=0.02, m_step=0.20):
    """沿整条路径逐点查车体 —— 回答"这条路本身能不能走 + 还剩多少余量"。

    朝向取**路径切线**（与控制器实际朝向的来源一致；`keep_start_yaw` 的末点用目标
    yaw：全局规划把目标朝向放在末点）。
    净距一律用传进来的 `ObstacleIndex`（= C++ `ObstacleIndex` 的离线镜像），
    **不要**在这里另建一份索引：`id(grid)` 当缓存键会串图（消息对象被回收后
    id 会被复用），量出来的净距会是另一张图的。

    余量（`margin_at`）要跑二分，很贵 ⇒ 先按 `m_step` 粗扫找最小，再在它附近用
    `step` 精扫。返回 (压到的采样点数, 最贴点, 余量最小点)。
    """
    if grid is None or idx is None or len(poses) < 2:
        return None
    walk = list(walk_path(poses, step))
    worst = (1e9, 0.0, None, 0.0)
    n_hit = 0
    for px, py, yaw, s in walk:
        if footprint_hits(grid, px, py, yaw)[0]:
            n_hit += 1
        d = idx.clearance(px, py)
        if d < worst[0]:
            worst = (d, math.degrees(yaw), (px, py), s)

    def margin(px, py, yaw):
        return 0.0 if footprint_hits(grid, px, py, yaw)[0] else \
            margin_at(grid, px, py, yaw)

    # 起步 1 m 内车自己可能就站在窄处（astar.start_escape_radius 允许它），
    # 那会把"路径本身窄不窄"盖住 ⇒ 分开统计（`SKIP` 之外的那份才是路径的问题）。
    SKIP = 1.0
    m_worst = (1e9, None, 0.0)
    m_far = (1e9, None, 0.0)
    for px, py, yaw, s in walk_path(poses, m_step):
        mm = margin(px, py, yaw)
        if mm < m_worst[0]:
            m_worst = (mm, (px, py), s)
        if s > SKIP and mm < m_far[0]:
            m_far = (mm, (px, py), s)
    for c in (m_worst[1], m_far[1]):      # 在最小余量附近精扫
        if c is None:
            continue
        for px, py, yaw, s in walk:
            if math.hypot(px - c[0], py - c[1]) > m_step * 1.5:
                continue
            mm = margin(px, py, yaw)
            if mm < m_worst[0]:
                m_worst = (mm, (px, py), s)
            if s > SKIP and mm < m_far[0]:
                m_far = (mm, (px, py), s)
    return n_hit, worst, m_worst, m_far


def dump_maps(g, p, x, y, span=0.8, cell=0.10):
    """在 (x, y) 附近把两张图并排打印 —— 直接看"感知到底有没有看见"。

    `.`=自由  `#`=占据  `?`=未知(-1)  `x`=图外  `@`=车心所在格
    """
    if g is None or p is None:
        return
    rows = []
    for name, grid in (("全局", g), ("感知", p)):
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        w, h = grid.info.width, grid.info.height
        n = int(2 * span / cell)
        # 车心在哪一格：用**格索引**比，不要用浮点差（0.05 < 0.05 永远为假）
        cx0, cy0 = int((x - ox) / res), int((y - oy) / res)
        lines = []
        for j in range(n - 1, -1, -1):
            yy = y - span + (j + 0.5) * cell
            s = ""
            for i in range(n):
                xx = x - span + (i + 0.5) * cell
                ix, iy = int((xx - ox) / res), int((yy - oy) / res)
                if (ix, iy) == (cx0, cy0):
                    s += "@"
                    continue
                if ix < 0 or iy < 0 or ix >= w or iy >= h:
                    s += "x"
                    continue
                v = grid.data[iy * w + ix]
                s += "?" if v < 0 else ("#" if v >= 50 else ".")
            lines.append(s)
        rows.append((name, lines))
    a, b = rows[0][1], rows[1][1]
    print(f"  ── ({x:.2f}, {y:.2f}) 附近 ±{span:.1f} m，{cell:.2f} m/格"
          f"（左=全局 / 右=感知，@=车心那一格）")
    for la, lb in zip(a, b):
        print(f"      {la}   {lb}")


def fold_angles(poses):
    """折角：相邻线段夹角（度）。**折角越大，控制器越必须把角切掉** ——
    受 ω 上限约束，绕一个 θ 角至少要外摆 r(1-cos(θ/2))，r = v/ω_max。"""
    a = [(p.pose.position.x, p.pose.position.y) for p in poses]
    out = []
    for i in range(1, len(a) - 1):
        v1 = (a[i][0] - a[i - 1][0], a[i][1] - a[i - 1][1])
        v2 = (a[i + 1][0] - a[i][0], a[i + 1][1] - a[i][1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        out.append((abs(math.degrees(math.atan2(cross, dot))), i, a[i]))
    return out


def profile(idx, poses, label, warn_below):
    """逐点净距：打最小值 + 最小值附近的一段（"在哪里贴"）。"""
    if idx is None:
        print(f"  {label}: 没有图")
        return []
    cs = [idx.clearance(p.pose.position.x, p.pose.position.y) for p in poses]
    if not cs:
        print(f"  {label}: 无点")
        return cs
    k = min(range(len(cs)), key=lambda i: cs[i])
    print(f"  {label}: 最小净距 {cs[k]:.3f} m @第 {k}/{len(cs)-1} 点"
          f"（({poses[k].pose.position.x:.2f}, {poses[k].pose.position.y:.2f})）"
          f"| 全程 min/均值 {min(cs):.3f}/{sum(cs)/len(cs):.3f} m")
    lo = max(0, k - 3)
    hi = min(len(cs), k + 4)
    print("      贴障碍段净距：" +
          " ".join(f"{c:.2f}" for c in cs[lo:hi]) +
          f"   ({lo}~{hi-1} 点)")
    n_bad = sum(1 for c in cs if c < warn_below)
    print(f"      净距 < {warn_below:.2f} m 的点数：{n_bad}/{len(cs)}")
    return cs


def one_round(n: Node, gx, gy, gyaw, tag, gi, li, start=None):
    print(f"\n===== {tag} =====")
    res = plan_once(n, gx, gy, gyaw, start=start)
    if not res.success:
        print(f"  规划失败：{res.status_name}（{res.message}）")
        return None
    poses = res.path.poses
    L = sum(math.hypot(poses[i + 1].pose.position.x - poses[i].pose.position.x,
                       poses[i + 1].pose.position.y - poses[i].pose.position.y)
            for i in range(len(poses) - 1))
    print(f"  {res.status_name}：{len(poses)} 点 / {L:.2f} m / "
          f"{res.plan_time_ms:.1f} ms | traj_valid={res.traj_valid}"
          f"（{res.traj_note}）| 剖面 {len(res.traj_s)} 点")
    if len(poses) <= 8:  # A* 剪枝后折点很少：直接印出来，方便和地图对照
        print("  折点：" + " ".join(
            f"({p.pose.position.x:.2f},{p.pose.position.y:.2f})" for p in poses))
    fa = fold_angles(poses)
    if fa:
        mx = max(fa)
        print(f"  最大折角 {mx[0]:.0f}° @第 {mx[1]} 点 ({mx[2][0]:.2f}, "
              f"{mx[2][1]:.2f})｜折角 ≥30° 的点："
              f"{sum(1 for a in fa if a[0] >= 30)}/{len(fa)}")
    print("  【全局图】规划器自己的判据（0.05 m，含禁行区/膨胀）")
    cg = profile(gi, poses, "全局", 0.45)
    print("  【感知图】MPC 实际避障用的那张（0.10 m，未膨胀）")
    cl = profile(li, poses, "感知", 0.45)

    # ---- 车体（矩形）层面的判定：路径本身能不能走 + 到点能不能原地转 ----
    print("  【车体矩形】沿整条路径（含点间插值，朝向=路径切线）")
    scan = path_footprint_scan(n.gmap, gi, poses)
    if scan:
        n_hit, worst, mw, mf = scan
        print(f"      压到障碍的采样点：{n_hit} 个 | 最贴处：车心净距 "
              f"{worst[0]:.3f} m（朝向 {worst[1]:.1f}°，位置 "
              f"({worst[2][0]:.2f}, {worst[2][1]:.2f})，里程 {worst[3]:.2f} m）")
        print(f"      ★ 跟踪误差预算（车体还能容忍多少横向偏差）：全程最窄 "
              f"{mw[0]*100:.0f} cm（({mw[1][0]:.2f}, {mw[1][1]:.2f})，"
              f"里程 {mw[2]:.2f} m）；**起步 1 m 之后**最窄 {mf[0]*100:.0f} cm"
              f"（({mf[1][0]:.2f}, {mf[1][1]:.2f})，里程 {mf[2]:.2f} m）"
              f"｜感知图同一处净距 {li.clearance(mf[1][0], mf[1][1]):.3f} m"
              f"/硬下界 0.25 m")
        # ★ 最贴处两张图并排看一眼：感知是真没障碍，还是根本没看见（未知/图外）？
        wx, wy = worst[2]
        print(f"      同一点：全局净距 {gi.clearance(wx, wy):.3f} m / "
              f"感知净距 {li.clearance(wx, wy):.3f} m")
        x0, y0 = n.pose.position.x, n.pose.position.y
        print(f"      （车当前在 ({x0:.2f}, {y0:.2f})，距该点 "
              f"{math.hypot(wx-x0, wy-y0):.2f} m）")
        dump_maps(n.gmap, n.lmap, wx, wy)
    # 末点：目标朝向（全局规划把目标 yaw 放在末点）
    last = poses[-1]
    gq = last.pose.orientation
    gyaw_last = yaw_of(gq)
    hit, cnt = footprint_hits(n.gmap, last.pose.position.x,
                              last.pose.position.y, gyaw_last)
    print(f"  【末点位姿】({last.pose.position.x:.2f}, {last.pose.position.y:.2f}, "
          f"{math.degrees(gyaw_last):.1f}°) 车体压障碍：{hit}（{cnt} 点）")
    # 末段朝向 → 目标朝向 的原地转（= 到点对正能不能做）
    if len(poses) >= 2:
        a = math.atan2(last.pose.position.y - poses[-2].pose.position.y,
                       last.pose.position.x - poses[-2].pose.position.x)
        bad_yaw, bad_cnt = footprint_sweep(n.gmap, last.pose.position.x,
                                          last.pose.position.y, a, gyaw_last)
        print(f"  【到点原地转】末段朝向 {math.degrees(a):.1f}° → 目标 "
              f"{math.degrees(gyaw_last):.1f}°："
              + (f"★ 转到 {bad_yaw:.0f}° 就会扫到障碍（{bad_cnt} 点）"
                 if bad_yaw is not None else "全程不碰"))
    return {"res": res, "len": L, "cg": cg, "cl": cl,
            "m": None if not scan else scan[2][0],
            "mf": None if not scan else scan[3][0]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, required=True,
                    metavar=("X", "Y"))
    ap.add_argument("--yaw", type=float, default=0.0, help="目标朝向 [°]")
    ap.add_argument("--ab", action="store_true",
                    help="同一目标/同一起点，none 与 minco 各规划一次做对照")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="规划前临时改 global_planner 的参数（可重复），退出时恢复")
    ap.add_argument("--from", dest="start_from", default=None,
                    metavar="X,Y,YAW_DEG",
                    help="用指定起点规划（默认用车的当前位姿）；"
                         "写成单个串是为了容得下负角度（nargs=3 会把 -38.3 "
                         "当成选项报错）")
    args = ap.parse_args()

    rclpy.init()
    n = Node()
    if not n.wait(lambda: n.pose and n.gmap and n.lmap, 15.0):
        print("缺位姿/地图：仿真 + 定位 + perception + map_server 起了吗？")
        return 2
    yaw = yaw_of(n.pose.orientation)
    d = math.hypot(args.goal[0] - n.pose.position.x,
                   args.goal[1] - n.pose.position.y)
    print(f"车 ({n.pose.position.x:.2f}, {n.pose.position.y:.2f}, "
          f"{math.degrees(yaw):.1f}°) → 目标 ({args.goal[0]:.2f}, "
          f"{args.goal[1]:.2f}, {args.yaw:.1f}°)，直线 {d:.2f} m")
    start = None
    if args.start_from is not None:
        sx, sy, syaw_deg = (float(v) for v in args.start_from.split(","))
        start = (sx, sy, math.radians(syaw_deg))
        print(f"※ 本次用**指定起点** ({sx:.2f}, {sy:.2f}, "
              f"{syaw_deg:.1f}°)（与车当前位姿无关）")

    gi = sc.ObstacleIndex(n.gmap)
    li = sc.ObstacleIndex(n.lmap)
    gyaw = math.radians(args.yaw)
    cur = pget(n, "global_planner", "traj.type")
    print(f"当前 traj.type = {cur}")

    # 临时改参 A/B：先记下原值，退出时一律恢复（别把现场留下）
    restore = []
    for kv in args.set:
        k, _, v = kv.partition("=")
        old = pget(n, "global_planner", k)
        if old is None:
            print(f"⚠ 没读到 {k} 的原值，跳过")
            continue
        pset(n, "global_planner", k, v)
        print(f"  临时设 {k}: {old} → {v}")
        restore.append((k, old))
    if restore:
        gi = sc.ObstacleIndex(n.gmap)  # 地图可能已重发
        li = sc.ObstacleIndex(n.lmap)
    if not args.ab:
        one_round(n, args.goal[0], args.goal[1], gyaw, f"traj.type={cur}", gi,
                  li, start=start)
        for k, v in reversed(restore):
            pset(n, "global_planner", k, v)
        if restore:
            print("（已恢复参数：" + ", ".join(f"{k}={v}" for k, v in restore) + "）")
        rclpy.shutdown()
        return 0

    out = {}
    for t in ("none", "minco"):
        if t != cur:
            pset(n, "global_planner", "traj.type", t)
        out[t] = one_round(n, args.goal[0], args.goal[1], gyaw,
                           f"traj.type={t}", gi, li, start=start)
    if cur and cur != "minco":
        pset(n, "global_planner", "traj.type", cur)
        print(f"\n（已恢复 traj.type={cur}）")
    for k, v in reversed(restore):
        pset(n, "global_planner", k, v)
    if out.get("none") and out.get("minco"):
        print("\n===== 对照小结（同一目标/同一起点）=====")
        for t in ("none", "minco"):
            o = out[t]
            if not o:
                continue
            mm = "—" if o["m"] is None else f"{o['m']*100:.0f} cm"
            mf = "—" if o["mf"] is None else f"{o['mf']*100:.0f} cm"
            print(f"  {t:6s}: 路径 {o['len']:.2f} m / {len(o['res'].path.poses)} 点"
                  f" | 全局最小净距 {min(o['cg']):.3f} m"
                  f" | 感知最小净距 {min(o['cl']):.3f} m"
                  f" | 余量 {mm}（1 m 后 {mf}）")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
