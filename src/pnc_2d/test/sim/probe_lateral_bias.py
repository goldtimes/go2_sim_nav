#!/usr/bin/env python3
"""M5.0 第 1 步：量「全局图 vs 局部图」的几何是否同源（**只量测，不改任何算法**）。

为什么要量它（M4.5 的发现，2026-09-24）：
  稳态横向偏置在两个场景是 **+0.004 / -0.000 m**，在另两次是 **+0.104 / -0.096 m**
  ⇒ 它不是恒定标定偏移，而是**场景相关**的。
  而两套图的分工本来就不对称：
    · A* 的路径活在全球图里（`/global_map/occupancy`，0.05 m，**已膨胀 1 格 = 0.05 m**）
    · MPC 的硬约束 / ESDF 活在感知图里（`/grid_map/occupancy_2d`，0.1 m，**未膨胀**）
  ⇒ 两边的"障碍边界"相差：
      全局：+0.05 m 膨胀（**结构性的、恒定的**）
      局部：0.1 m 分辨率的量化 ⇒ 边界位置 ±0.05 m（随**栅格相位**变化）
    两者相加 = **0 ~ 0.10 m** —— 与观测范围吻合。这就是本脚本要钉的东西。

量五件事（**原地不动**，不跑任何规划/控制）：
  ① 两张图的基本参数 + **栅格相位**（origin 对各自分辨率的余数）；
  ② **同一面墙**在两张图里的边界位置 → 实际偏移量（沿 8 个方向扫）；
  ③ 感知 ESDF 点云的**取值直方图** —— 它是否只落在 √n·res 的格点上（量化的证明），
     再用 Lipschitz 不等式夹出车处 d 的**可靠区间**（1-Lipschitz ⇒
     max(d(p)−|车−p|) ≤ d(车) ≤ min(d(p)+|车−p|)）；
  ④ **复刻 MPC 实际查询的那张场**（算法抄自 `core/local_distance_field.cpp`：
     落到最近格 + max 合并 + 邻域平均填充 + 双线性），报出车处的 d、∇d、
     **推车方向**，并与「真实最近障碍方位」（两图各算）对比 —— 这才知道
     软代价到底在往哪个方向推；
  ⑤ **±5 cm 横扫**：只挪查询点，看「推车方向的横向分量」符号会不会翻。
     翻 ⇒ 推车方向由**场的离散化**决定（量化/0.2 m 抽样/填充/插值）；
     不翻 ⇒ 由几何决定，根因在图的对齐（相位/膨胀）。

⚠ 上一版用「平面外推」估计 d(车) 与 ∇d，会给出负的 d（车在墙角时平面拟合
   失效）。本版改成 ④ 的复刻场 —— 它给出的就是 MPC 看到的那张场。

用法（仿真 + 定位 + perception + map_server 都在跑时）：
    python3 test/sim/probe_lateral_bias.py
    python3 test/sim/probe_lateral_bias.py --span 3.0 --step 0.01

⚠ 本脚本**不启动**任何节点（连 pnc_2d 都不用起）—— 它只需要两张图和位姿。
   如果收发不到，它会明确告诉你缺哪个话题、有几个发布者。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

# 全局图：latched（map_server 用 transient_local 发）
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# 局部图 / 位姿：感知与定位是流式发布，用 best_effort 才能兼容任意发布端
LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)

OCC_THRESH = 50  # >= 该值算占据（全局图 100/0，感知图 100/0/-1）
SOFT_SAFE = 0.45   # = local_mpc.obstacle_safe_distance（软代价开始生效）
HARD = 0.25        # = local_mpc.obstacle_hard_distance（硬下界）
# 下面两个必须与 local_mpc / perception 配置一致，否则复刻出来的场不是 MPC 的场：
ESDF_MAX = 3.0     # = local_mpc.esdf_max_distance 与 perception.esdf_max_dist
FILL_RADIUS = 2    # = local_mpc.esdf_fill_radius


def parse_xy_d(msg: PointCloud2, max_pts=60000):
    """手工解析 PointCloud2 的 (x, y, d)，**不依赖 sensor_msgs_py**（未必装了）。

    ESDF 点云的 `intensity` = 该点到最近障碍的距离 [m]（不是到车的距离）——
    所以它不是“障碍物列表”，而是**距离场的采样**；MPC 用的是它插值出来的
    `distance(x,y)` 与 `gradient(x,y)`。
    """
    import struct
    fmap = {f.name: f.offset for f in msg.fields}
    if "x" not in fmap or "y" not in fmap:
        return []
    doff = fmap.get("intensity", fmap.get("z"))
    step = msg.point_step
    if step <= 0:
        return []
    buf = bytes(msg.data)
    n = min(len(buf) // step, max_pts)
    out = []
    for i in range(n):
        base = i * step
        x = struct.unpack_from("<f", buf, base + fmap["x"])[0]
        y = struct.unpack_from("<f", buf, base + fmap["y"])[0]
        if x != x or y != y:  # NaN
            continue
        d = struct.unpack_from("<f", buf, base + doff)[0] if doff is not None else 0.0
        out.append((x, y, d))
    return out


def fit_grad(pts, cx, cy, radius=0.60):
    """在 (cx,cy) 附近最小二乘拟合 d(x,y) = a·x + b·y + c，返回 (a,b,c,用到点数)。

    ★ 这就是 MPC 软代价用的那个梯度的**直接估计** —— 不用猜它的插值实现，
      只用它在同一批采样点上做一次局部平面拟合。
      `|(a,b)|` = 车处的距离场斜率；它的方向 = 车被推的方向的反向。
    """
    n = 0
    sxx = sxy = syy = sx = sy = sd = sxd = syd = 0.0
    for (x, y, d) in pts:
        if math.hypot(x - cx, y - cy) > radius:
            continue
        n += 1
        sxx += x * x
        sxy += x * y
        syy += y * y
        sx += x
        sy += y
        sd += d
        sxd += x * d
        syd += y * d
    if n < 6:
        return None
    # 解 3x3 正规方程（消元，规模小，不求通用性）
    A = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    b = [sxd, syd, sd]
    for i in range(3):
        p = max(range(i, 3), key=lambda r: abs(A[r][i]))
        if abs(A[p][i]) < 1e-12:
            return None
        A[i], A[p] = A[p], A[i]
        b[i], b[p] = b[p], b[i]
        for r in range(i + 1, 3):
            f = A[r][i] / A[i][i]
            for c in range(i, 3):
                A[r][c] -= f * A[i][c]
            b[r] -= f * b[i]
    s = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        v = b[i] - sum(A[i][c] * s[c] for c in range(i + 1, 3))
        s[i] = v / A[i][i]
    return s[0], s[1], s[2], n


class Grid:
    """一张 OccupancyGrid 的只读包装（世界坐标 ↔ 格，越界算"图外"）"""

    def __init__(self, m: OccupancyGrid):
        self.res = m.info.resolution
        self.ox = m.info.origin.position.x
        self.oy = m.info.origin.position.y
        self.w = m.info.width
        self.h = m.info.height
        self.frame = m.header.frame_id
        self.data = m.data
        self.stamp = m.header.stamp

    def cell(self, x, y):
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        if ix < 0 or iy < 0 or ix >= self.w or iy >= self.h:
            return None  # 图外
        return self.data[iy * self.w + ix]

    def is_occ(self, x, y):
        v = self.cell(x, y)
        return None if v is None else (v >= OCC_THRESH)

    def is_unknown(self, x, y):
        v = self.cell(x, y)
        return None if v is None else (v < 0)

    def phase(self, mod: float) -> float:
        """origin 相对某个格距的**相位**（0~mod）。两张图的相位差就是"量化错位"的量。"""
        return math.fmod(math.fmod(self.ox, mod) + mod, mod), \
            math.fmod(math.fmod(self.oy, mod) + mod, mod)

    def nearest_occ(self, x, y, radius=2.0):
        """离 (x,y) 最近的**占据格**（格心到格心）→ (距离, 方位角°, 格心, 格号)。

        ★ 这就是"真实几何"口径的推车方向真值：单障碍时，远离它的方向 =
          从最近障碍指向车 = 方位角 + 180°。注意它量的是**格心**（与感知 ESDF
          的 d 同口径，都是格心到格心），不是到表面的距离。
        找不到（radius 内无占据格）返回 None。
        """
        rc = int(math.ceil(radius / self.res))
        ci = int(math.floor((x - self.ox) / self.res))
        cj = int(math.floor((y - self.oy) / self.res))
        best = None
        for j in range(cj - rc, cj + rc + 1):
            if j < 0 or j >= self.h:
                continue
            row = j * self.w
            for i in range(ci - rc, ci + rc + 1):
                if i < 0 or i >= self.w:
                    continue
                if self.data[row + i] < OCC_THRESH:
                    continue
                cx = self.ox + (i + 0.5) * self.res
                cy = self.oy + (j + 0.5) * self.res
                dd = math.hypot(cx - x, cy - y)
                if best is None or dd < best[0]:
                    best = (dd, math.degrees(math.atan2(cy - y, cx - x)) % 360.0,
                            (cx, cy), (i, j))
        if best is None or best[0] > radius:
            return None
        return best


class FieldReplica:
    """**复刻 MPC 实际查询的那张场**（算法抄自 `core/local_distance_field.cpp`）。

    为什么必须复刻而不是"估一个梯度"：MPC 的障碍软代价/硬下界吃的是
    `LocalDistanceField::distance()/gradient()`，而那张场不是原始点云 ——
    它是点云经过 ① 落到最近格取 max、② 邻域平均填充、③ 双线性查询 之后的
    产物。三个环节都会改变 d 与 ∇d，光看点云是量不出"软代价在往哪推"的。

    约束：采样间距是 perception 的 `esdf_pub_step`（默认 2 格 = 0.2 m），
    所以 `sample_cells` 只占全格的 1/4 左右，其余靠填充 ——
    这也意味着**场里一半以上的值不是真采样**（后面要报出来）。
    """

    def __init__(self, ox, oy, res, w, h, max_dist=ESDF_MAX,
                 fill_radius=FILL_RADIUS):
        self.ox, self.oy, self.res = ox, oy, res
        self.w, self.h = w, h
        self.max_dist = max_dist
        self.fill_radius = fill_radius
        self.d = [max_dist] * (w * h)
        self.sample_cells = 0
        self.filled_cells = 0
        self.max_fill_used = 0

    def _at_rc(self, x, y):
        if x < 0 or y < 0 or x >= self.w or y >= self.h:
            return self.max_dist
        return self.d[y * self.w + x]

    def build(self, samples) -> bool:
        eps = 1e-6
        for (x, y, dd) in samples:
            ix = int(math.floor((x - self.ox) / self.res))
            iy = int(math.floor((y - self.oy) / self.res))
            if ix < 0 or iy < 0 or ix >= self.w or iy >= self.h:
                continue  # 图外的采样直接丢（与 C++ 一致）
            v = min(max(0.0, dd), self.max_dist)
            k = iy * self.w + ix
            if self.d[k] >= self.max_dist - eps:
                self.sample_cells += 1  # 首次命中
                self.d[k] = v
            else:
                self.d[k] = max(self.d[k], v)  # 同格多采样取 max
        if self.sample_cells == 0:
            return False
        for r in range(1, self.fill_radius + 1):
            prev = list(self.d)  # 每轮基于上一轮快照（与 C++ 一致）
            filled = 0
            for y in range(self.h):
                for x in range(self.w):
                    k = y * self.w + x
                    if prev[k] < self.max_dist - eps:
                        continue  # 已经有真值，永不被覆盖
                    s, c = 0.0, 0
                    for (nx, ny) in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                        if nx < 0 or ny < 0 or nx >= self.w or ny >= self.h:
                            continue
                        v = prev[ny * self.w + nx]
                        if v < self.max_dist - eps:
                            s += v
                            c += 1
                    if c:
                        self.d[k] = s / c
                        filled += 1
                        self.max_fill_used = r
            self.filled_cells += filled
            if filled == 0:
                break
        return True

    def _bilinear(self, gx, gy):
        x0 = int(math.floor(gx))
        y0 = int(math.floor(gy))
        fx, fy = gx - x0, gy - y0
        v00 = self._at_rc(x0, y0)
        v10 = self._at_rc(x0 + 1, y0)
        v01 = self._at_rc(x0, y0 + 1)
        v11 = self._at_rc(x0 + 1, y0 + 1)
        return ((1 - fx) * (1 - fy) * v00 + fx * (1 - fy) * v10 +
                (1 - fx) * fy * v01 + fx * fy * v11)

    def distance(self, wx, wy) -> float:
        gx = (wx - self.ox) / self.res - 0.5
        gy = (wy - self.oy) / self.res - 0.5
        if gx < -0.5 or gy < -0.5 or gx > self.w - 0.5 or gy > self.h - 0.5:
            return self.max_dist
        return self._bilinear(gx, gy)

    def gradient(self, wx, wy):
        """中心差分（**整格步长**，与 C++ 一致）→ (gx, gy)；给不出方向返回 None。"""
        ix = int(math.floor((wx - self.ox) / self.res))
        iy = int(math.floor((wy - self.oy) / self.res))
        if ix < 0 or iy < 0 or ix >= self.w or iy >= self.h:
            return None
        gx = (self._at_rc(ix + 1, iy) - self._at_rc(ix - 1, iy)) / (2.0 * self.res)
        gy = (self._at_rc(ix, iy + 1) - self._at_rc(ix, iy - 1)) / (2.0 * self.res)
        if math.hypot(gx, gy) <= 1e-6:
            return None
        return gx, gy


class Probe(Node):
    def __init__(self):
        super().__init__("lateral_bias_probe")
        self.gmap = None
        self.lmap = None
        self.pose = None
        self.esdf = None
        self.esdf_frame = ""
        self.gmap_n = 0
        self.lmap_n = 0
        self.esdf_n = 0
        self.create_subscription(OccupancyGrid, "/global_map/occupancy",
                                 self.on_gmap, LATCHED)
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d",
                                 self.on_lmap, LIVE)
        self.create_subscription(PointCloud2, "/grid_map/esdf_2d",
                                 self.on_esdf, LIVE)
        self.create_subscription(Odometry, "/lightning/perception/pose",
                                 self.on_pose, LIVE)

    def on_esdf(self, m):
        pts = parse_xy_d(m)
        if pts:
            self.esdf = pts
            self.esdf_frame = m.header.frame_id
            self.esdf_n += 1

    def on_gmap(self, m):
        self.gmap = Grid(m)
        self.gmap_n += 1

    def on_lmap(self, m):
        self.lmap = Grid(m)
        self.lmap_n += 1

    def on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, cond, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False


def first_hit(g: Grid, x0, y0, dx, dy, span, step):
    """从 (x0,y0) 沿 (dx,dy) 走，返回第一个被占据的**世界坐标与距离**。

    只把"图外"当作未知并继续（图外不一定是障碍）；但若一直走到图外都没命中，
    返回 None —— 这比编一个哨兵值诚实。
    """
    d = step
    while d <= span:
        x, y = x0 + dx * d, y0 + dy * d
        o = g.is_occ(x, y)
        if o is None:
            return None, None  # 出了图，不再有意义
        if o:
            return (x, y), d
        d += step
    return None, None


def scan_report(name, g: Grid, x, y, span, step):
    """沿 8 个方向扫第一处障碍，返回 {角度: (距离, 世界坐标)}"""
    out = {}
    for ang_deg in range(0, 360, 45):
        a = math.radians(ang_deg)
        p, d = first_hit(g, x, y, math.cos(a), math.sin(a), span, step)
        out[ang_deg] = (d, p)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--span", type=float, default=3.0, help="扫描半径 [m]")
    ap.add_argument("--step", type=float, default=0.01, help="扫描步长 [m]")
    args = ap.parse_args()

    rclpy.init()
    p = Probe()
    try:
        ok = p.wait_for(lambda: p.gmap is not None and p.lmap is not None
                        and p.pose is not None, timeout=20.0)
        if not ok:
            print("缺数据，逐项确认：")
            for topic, have, n in (
                    ("/global_map/occupancy", p.gmap is not None, p.gmap_n),
                    ("/grid_map/occupancy_2d", p.lmap is not None, p.lmap_n),
                    ("/lightning/perception/pose", p.pose is not None, 0)):
                pubs = p.count_publishers(topic)
                print(f"  {topic:34s} 收到 {have}  发布者 {pubs} 个")
            print("\n  · 全局图没有 → map_server 没起（或用了别的 topic）")
            print("  · 局部图没有 → perception 没起（它**只在有订阅者时才计算**，"
                  "本脚本的订阅就是来触发它的）")
            return 2
        p.spin(1.0)  # 让两张图各刷一次

        g, l = p.gmap, p.lmap
        x, y, yaw = p.pose

        print("=" * 78)
        print("M5.0 第 1 步：全局图 vs 局部图 的几何是否同源（只量测）")
        print("=" * 78)
        print(f"车: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}°)  frame={g.frame}")

        # ---------------- ① 基本参数 + 栅格相位 ----------------
        print("\n---- ① 两张图的基本参数 ----")
        print(f"{'':12s}{'分辨率':>10s}{'尺寸':>14s}{'origin':>24s}{'frame':>8s}")
        for nm, gg in (("全局图", g), ("局部图", l)):
            print(f"{nm:12s}{gg.res:>10.4f}{f'{gg.w}x{gg.h}':>14s}"
                  f"{f'({gg.ox:.3f}, {gg.oy:.3f})':>24s}{gg.frame:>8s}")
        # ★ 这两行是"扫不到障碍"时的第一现场：车在不在图里 / 图里到底有没有占据格
        from collections import Counter
        for nm, gg in (("全局图", g), ("局部图", l)):
            v = gg.cell(x, y)
            hist = dict(sorted(Counter(gg.data).items()))
            print(f"{nm} 车处的格值: {v}（None = 车在该图之外）| "
                  f"数据 {len(gg.data)} / 期望 {gg.w * gg.h} | 取值分布 {hist}")
        print("\n栅格相位（origin 对某格距的余数 —— 量化错位就藏在这里）：")
        for mod in (0.05, 0.10):
            gx, gy = g.phase(mod)
            lx, ly = l.phase(mod)
            print(f"  mod {mod:.2f} m:  全局 ({gx:.3f}, {gy:.3f})   "
                  f"局部 ({lx:.3f}, {ly:.3f})   相对相位 "
                  f"({(lx - gx) % mod:.3f}, {(ly - gy) % mod:.3f})")

        # ---------------- ② 同一面墙在两张图里的位置 ----------------
        print(f"\n---- ② 沿 8 个方向扫「最靠前的障碍」（半径 {args.span:.1f} m，"
              f"步长 {args.step:.2f} m）----")
        sg = scan_report("global", g, x, y, args.span, args.step)
        sl = scan_report("local", l, x, y, args.span, args.step)
        print(f"{'方向':>6s}{'全局 距离':>12s}{'局部 距离':>12s}{'差':>10s}"
              f"{'全局 命中点':>22s}{'局部 命中点':>22s}")
        diffs = []
        for ang in sorted(sg):
            dg, pg = sg[ang]
            dl, pl = sl[ang]
            s_d = "—" if dg is None else f"{dg:.2f}"
            s_l = "—" if dl is None else f"{dl:.2f}"
            if dg is None or dl is None:
                s_diff, extra = "—", ""
            else:
                diff = dl - dg
                diffs.append((ang, diff))
                s_diff = f"{diff:+.2f}"
                extra = "  ← 局部比全局更远" if diff > 0.02 else (
                    "  ← 局部比全局更近" if diff < -0.02 else "")
            sp = ("" if pg is None else f"({pg[0]:.2f},{pg[1]:.2f})")
            sl2 = ("" if pl is None else f"({pl[0]:.2f},{pl[1]:.2f})")
            print(f"{ang:>5d}°{s_d:>12s}{s_l:>12s}{s_diff:>10s}"
                  f"{sp:>22s}{sl2:>22s}{extra}")

        # ---------------- ②b 走廊中心偏移：两套图对"走廊中间在哪"的分歧 ----------------
        # ★ 这是 M5.0 **原定的第 1 步**（plan §M5.0「量参考线 vs 车的偏置归属」），
        #   也是"两套图不同源"假设的直接检验。关键推理：
        #     膨胀是**对称**的（两侧各 +1 格）⇒ 它**不会**移动走廊中心；
        #     能移动中心的只有**量化**（0.05 格心 vs 0.1 格心 + 最近格吸附）。
        #   所以：若中心差 ≈ 0 ⇒ 膨胀/两图不同源解释不了 10 cm 偏置。
        print("\n---- ②b 走廊中心偏移（沿车头方向取断面；M5.0 原定第 1 步）----")
        dirx, diry = math.cos(yaw), math.sin(yaw)
        nxl2, nyl2 = -diry, dirx  # 左法向（正 = 车左侧）
        print(f"  断面法向 = 车左法向 ({nxl2:+.2f},{nyl2:+.2f})；"
              f"站位 s 沿车头方向，壁位为单位 m（左+/右−）")
        print(f"  {'站位s':>7s}{'全局 左/右壁':>20s}{'局部 左/右壁':>20s}"
              f"{'全局中心':>10s}{'局部中心':>10s}{'中心差':>11s}")
        offs = []
        for s in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0):
            px0 = x + dirx * s
            py0 = y + diry * s
            wall = {"g": {}, "l": {}}
            for sgn, (dx_, dy_) in ((1, (nxl2, nyl2)), (-1, (-nxl2, -nyl2))):
                for nm, gg in (("g", g), ("l", l)):
                    _, dd_ = first_hit(gg, px0, py0, dx_, dy_, 3.0, 0.01)
                    wall[nm][sgn] = None if dd_ is None else sgn * dd_
            cen = {}
            for nm in ("g", "l"):
                a, b = wall[nm][1], wall[nm][-1]
                cen[nm] = None if (a is None or b is None) else 0.5 * (a + b)

            def fmt_lim(nm):
                a, b = wall[nm][1], wall[nm][-1]
                sa = "—" if a is None else f"{a:+.2f}"
                sb = "—" if b is None else f"{b:+.2f}"
                return f"{sa}/{sb}"

            if cen["g"] is None or cen["l"] is None:
                scg = "—" if cen["g"] is None else f"{cen['g']:+.3f}"
                scl = "—" if cen["l"] is None else f"{cen['l']:+.3f}"
                print(f"  {s:>+7.1f}{fmt_lim('g'):>20s}{fmt_lim('l'):>20s}"
                      f"{scg:>10s}{scl:>10s}{'—':>11s}")
                continue
            d_off = cen["l"] - cen["g"]
            offs.append(d_off)
            print(f"  {s:>+7.1f}{fmt_lim('g'):>20s}{fmt_lim('l'):>20s}"
                  f"{cen['g']:>+10.3f}{cen['l']:>+10.3f}{d_off * 100:>+9.1f}cm")
        if len(offs) >= 3:
            mean = sum(offs) / len(offs)
            print(f"\n  ⇒ 走廊中心差（局部 − 全局）: 均值 {mean * 100:+.1f} cm / "
                  f"范围 {min(offs) * 100:+.1f} ~ {max(offs) * 100:+.1f} cm"
                  f"（{len(offs)} 个断面）")
            if abs(mean) < 0.02 and max(abs(o) for o in offs) < 0.04:
                print("  ⇒ **两套图的走廊中心基本重合（<2 cm）** ⇒ \"两套图不同源\""
                      "**不足以解释 10 cm 的横向偏置**（膨胀是对称的，量化只到 ±半格）。")
                print("     下一步该查场的口径（③④⑤ 已量到 100% 量化 + 72% 填充值），"
                      "或回到 §M5.0 第 2 步（扫起点/朝向）。")
            else:
                print(f"  ⇒ **两套图对\"走廊中间在哪\"分歧最大 "
                      f"{max(abs(o) for o in offs) * 100:.1f} cm** ⇒ 与观测到的偏置"
                      f"（0.4 ~ 10.4 cm）量级一致 ⇒ 保留为横向偏置的第一候选根因。")
        else:
            print("  ⇒ 可算断面的站位不足（一侧探不到壁）—— 在宽走廊/开阔区这是正常的；"
                  "挪到**两侧都有壁**的窄走廊再量。")

        # ---------------- ③ 感知 ESDF 点云：软代价的原始输入 ----------------
        print("\n---- ③ 感知 ESDF 点云（perception build2DESDF 的产物）----")
        if not p.esdf:
            print(f"  没收到 /grid_map/esdf_2d（发布者 "
                  f"{p.count_publishers('/grid_map/esdf_2d')} 个）"
                  f"—— 它**只在有订阅者时才计算**，本脚本的订阅就是来触发它的；"
                  f"再跑一次就有。")
        else:
            pts = p.esdf
            ds = [d for _, _, d in pts]
            print(f"  点数 {len(pts)}  frame={p.esdf_frame}")
            print(f"  d 分布: min {min(ds):.3f} / 均值 {sum(ds)/len(ds):.3f} / "
                  f"max {max(ds):.3f} m（d = 该点到最近障碍的距离 [m]）")
            # ★ 取值直方图：perception 的 d = "格心到最近占据格心的距离"，
            #   单位是**格**（EDT）再乘 resolution ⇒ 只能取 √n·res
            #   （n = 两个格号的平方和）。若点云取值恰好落在这组格点上，
            #   软代价吃的就是一张**台阶场**，台阶高 0.1 m。
            cnt = Counter(round(d, 4) for d in ds)
            print(f"  ★ 不同取值 {len(cnt)} 个；最常见 10 个: "
                  + ", ".join(f"{v:.3f}({c})" for v, c in cnt.most_common(10)))
            allowed = sorted({math.sqrt(n) * l.res
                              for i in range(31) for j in range(31)
                              for n in (i * i + j * j,)})
            hit = sum(c for v, c in cnt.items()
                      if any(abs(v - a) < 1e-3 for a in allowed))
            print(f"    允许值（√n·{l.res:.2f}）前 8 个: "
                  + ", ".join(f"{a:.3f}" for a in allowed[:8])
                  + f" …；实际落在允许值上的点 {hit}/{len(ds)} = "
                    f"{100.0 * hit / max(1, len(ds)):.1f}%")
            # 车处 d 的可靠区间：距离场是 1-Lipschitz 的 ⇒
            #   max(d(p)−|车−p|) ≤ d(车) ≤ min(d(p)+|车−p|)
            lo, hi, m = -1e9, 1e9, 0
            for (px, py, d) in pts:
                r = math.hypot(px - x, py - y)
                if r > 1.5:
                    continue
                m += 1
                lo = max(lo, d - r)
                hi = min(hi, d + r)
            if m:
                print(f"  Lipschitz 区间（{m} 点，半径 1.5 m）: "
                      f"{lo:.3f} ≤ d(车) ≤ {hi:.3f} m")
            # 1 m 内的点按 8 扇区分布：看“单侧”还是“环绕”（单侧=持续被推向一侧）
            near = [(px, py, d) for (px, py, d) in pts
                    if math.hypot(px - x, py - y) <= 1.0]
            sect_n, sect_d = {}, {}
            for (px, py, d) in near:
                k = int(((math.degrees(math.atan2(py - y, px - x)) + 360) % 360)
                        // 45) * 45
                sect_n[k] = sect_n.get(k, 0) + 1
                sect_d[k] = sect_d.get(k, 0.0) + d
            print(f"  1 m 内的采样点 {len(near)} 个，按方向分摊（方向:d均值/个数）：")
            print("    " + "  ".join(
                f"{k}°:{sect_d[k]/sect_n[k]:.2f}/{sect_n[k]}"
                for k in sorted(sect_n)))

        # ---------------- ④ 复刻 MPC 实际查询的场 ----------------
        print(f"\n---- ④ 复刻 MPC 实际查询的场（几何=局部图 / 算法= "
              f"local_distance_field.cpp / max_dist={ESDF_MAX} / "
              f"fill_radius={FILL_RADIUS}）----")
        F = None
        if p.esdf and l is not None:
            F = FieldReplica(l.ox, l.oy, l.res, l.w, l.h, ESDF_MAX, FILL_RADIUS)
            if F.build(p.esdf):
                tot = l.w * l.h
                print(f"  真采样命中 {F.sample_cells}/{tot} 格 = "
                      f"{100.0 * F.sample_cells / tot:.1f}%"
                      f"；靠邻域平均填充 {F.filled_cells} 格（最大填充半径 "
                      f"{F.max_fill_used}）")
                print(f"  ※ 未覆盖格 = {ESDF_MAX} m（乐观）；perception 的 "
                      f"esdf_pub_step=2 ⇒ 采样间距 0.2 m，"
                      f"所以场内一半以上是**填出来的**值")
                dv = F.distance(x, y)
                gvec = F.gradient(x, y)
                print(f"  车处 场 d(车) = {dv:.3f} m   （soft={SOFT_SAFE} / "
                      f"hard={HARD}）")
                if gvec is None:
                    print("  车处 中心差分 ∇d ≈ 0 ⇒ **给不出方向**（相邻格值相同）。"
                          "硬下界的线性化在这一点会退化，这本身就是一个发现。")
                else:
                    gx_, gy_ = gvec
                    mag = math.hypot(gx_, gy_)
                    push = math.degrees(math.atan2(gy_, gx_)) % 360.0
                    print(f"  车处 ∇d = ({gx_:+.3f}, {gy_:+.3f})，"
                          f"|∇d| = {mag:.3f}   （真实距离场应有 |∇d| = 1.000）")
                    print(f"  ⇒ 软代价**推车方向** {push:.1f}°"
                          f"（沿 +∇d = 远离障碍的方向），"
                          f"强度 ∝ weight·|∇d| = {1000.0 * mag:.0f}")
                fit = fit_grad(p.esdf, x, y)
                if fit is not None:
                    print(f"  （对照·不可信：平面拟合给 d(车)={fit[2]:+.3f}、"
                          f"|∇d|={math.hypot(fit[0], fit[1]):.3f} —— "
                          f"近障碍处场是锥面，平面外推会给出负的 d，故本版弃用）")
                for nm, gg in (("全局图(膨胀 1 格)", g), ("局部图(未膨胀)", l)):
                    nb = gg.nearest_occ(x, y, 2.0)
                    if nb is None:
                        print(f"  {nm:16s} 2 m 内没有占据格")
                    else:
                        print(f"  {nm:16s} 最近占据格 {nb[0]:.3f} m @ "
                              f"{nb[1]:.1f}° ⇒ ***真正的*** 推车方向应 ≈ "
                              f"{(nb[1] + 180.0) % 360.0:.1f}°"
                              f"（格心 {nb[2][0]:.2f}, {nb[2][1]:.2f}）")
            else:
                print("  复刻失败：采样全落在局部图外（感知与局部图原点不一致？）")
        else:
            print("  缺 ESDF 点云或局部图，无法复刻")

        # ---------------- ⑤ ±5 cm 横扫：推车方向会不会翻 ----------------
        if F is not None:
            nxl, nyl = -math.sin(yaw), math.cos(yaw)  # 车头左法向 = 横向
            print(f"\n---- ⑤ ±5 cm 横扫（同一张场，只挪查询点；横向=左法向 "
                  f"({nxl:+.2f},{nyl:+.2f})）----")
            print(f"  {'偏移':>8s}{'场 d':>10s}{'|∇d|':>8s}{'∇d 方位':>10s}"
                  f"{'横向分量':>11s}{'真实最近障碍':>20s}")
            rows = []
            nb_true = []
            for off in (-0.05, -0.025, 0.0, 0.025, 0.05):
                px, py = x + nxl * off, y + nyl * off
                dv2 = F.distance(px, py)
                g2 = F.gradient(px, py)
                nb = l.nearest_occ(px, py, 2.0)
                nb_s = "—" if nb is None else f"{nb[1]:.1f}°({nb[0]:.2f}m)"
                if nb is not None:
                    nb_true.append(nb[1])
                if g2 is None:
                    print(f"  {off * 100:+6.1f}cm{dv2:>10.3f}{'—':>8s}"
                          f"{'∇d=0':>10s}{'—':>11s}{nb_s:>20s}")
                    continue
                mag2 = math.hypot(*g2)
                lat = g2[0] * nxl + g2[1] * nyl  # ∇d 在横向上的分量
                rows.append((off, dv2, mag2, lat))
                print(f"  {off * 100:+6.1f}cm{dv2:>10.3f}{mag2:>8.3f}"
                      f"{math.degrees(math.atan2(g2[1], g2[0])) % 360.0:>10.1f}"
                      f"{lat:>+11.3f}{nb_s:>20s}")
            if len(rows) >= 3:
                lat_c = [r[3] for r in rows]
                if min(lat_c) < 0 < max(lat_c):
                    print("  ⇒ ⚠ **横向分量的符号会翻**（同一张场、5 cm 之内）"
                          "⇒ 推车方向由**场的离散化**决定，不是由几何决定。")
                    print("     根因链 = 量化(格心距离 √n·res) → 0.2 m 抽样 → "
                          "邻域平均填充 → 双线性；修法在**场的口径/分辨率**，"
                          "不是在图上挪相位。")
                else:
                    print(f"  ⇒ 横向分量符号**不翻**（{min(lat_c):+.3f} ~ "
                          f"{max(lat_c):+.3f}）⇒ 推车方向由几何决定；"
                          f"5 cm 尺度上不是量化在作祟。")
                if nb_true:
                    print(f"     真实最近障碍方位在这 5 个点上跨 "
                          f"{max(nb_true) - min(nb_true):.1f}°"
                          f"（{min(nb_true):.1f}~{max(nb_true):.1f}°）"
                          f" —— 与上面的横向分量变化对照")
                dmin = min(abs(r[1] - SOFT_SAFE) for r in rows)
                if dmin < 0.05:
                    print(f"     ⚠ 场 d 离 soft={SOFT_SAFE} 最近只差 {dmin:.3f} m "
                          f"⇒ 恰好压在台阶边缘：d 的 0.1 m 量化直接决定"
                          f"软代价开关的抖动")

        # ---------------- ⑥ 场景判定（口径 = 复刻场，不是拟合外推）----------------
        print("\n---- ⑥ 本次场景判定（口径 = ④ 的复刻场）----")
        d_here = F.distance(x, y) if F is not None else None
        if d_here is None:
            print("  场没建起来，判不出场景")
        elif d_here >= SOFT_SAFE:
            print(f"  场景 = **开阔**（场 d(车)={d_here:.2f} ≥ safe {SOFT_SAFE}）"
                  f"⇒ 软/硬代价都**不生效**，本位置不会有\"障碍引起的横向偏置\"。")
            print("  ⇒ 要复现 M5.0 的坏场景，把车挪到**离最近障碍 0.3~0.8 m** 处"
                  "再跑：判据是这里的 d(车) 落进 "
                  f"[{HARD}, {SOFT_SAFE})。")
        elif d_here >= HARD:
            print(f"  ⚠ 场景 = **接近障碍**（场 d(车)={d_here:.2f} 落在 "
                  f"[{HARD}, {SOFT_SAFE})）⇒ 软代价**正在推车**。")
            print("  ⇒ 这时 ④ 的\"推车方向\"就是软代价施加的横向偏置来源；"
                  "把它的横向分量与 M4.5 实测的偏置方向对比即可定性。")
        else:
            print(f"  ⚠⚠ 场景 = **贴住障碍**（场 d(车)={d_here:.2f} < hard "
                  f"{HARD}）⇒ 硬下界也在生效，"
                  f"这一位置本来就不该发任务（先把车挪开再量）。")

        print("\n---- ⑦ 与场景无关的**结构性**错位（每次都存在，恒定）----")
        gx_, gy_ = g.phase(0.05)
        lx_, ly_ = l.phase(0.05)
        print(f"  两张图的格子相对相位 (mod 0.05): "
              f"x {(lx_ - gx_) % 0.05 * 100:.1f} cm / "
              f"y {(ly_ - gy_) % 0.05 * 100:.1f} cm")
        print(f"  分辨率 {g.res:.2f} vs {l.res:.2f} m ⇒ 局部图边界量化 ±"
              f"{l.res / 2 * 100:.0f} cm")
        print(f"  膨胀：全局 +1 格（0.05 m）vs 局部未膨胀 ⇒ 结构性差 5 cm")
        print("  ⇒ 这三项相加 ≈ 0~10 cm。**但它们只在靠近障碍时才被代价用上**"
              "（远离障碍时 d 远大于阈值，三条都不生效）。")

        if diffs:
            pos = [d for _, d in diffs if d > 0.02]
            neg = [d for _, d in diffs if d < -0.02]
            zero = [d for _, d in diffs if abs(d) <= 0.02]
            print(f"\n  沿 8 方向的\"局部 vs 全局\"障碍距离差：偏大 {len(pos)} / "
                  f"偏小 {len(neg)} / 相当 {len(zero)}（共 {len(diffs)}）")
            if pos and neg:
                print("  ⇒ **两个符号都出现** ⇒ 至少一部分来自量化/相位，"
                      "不是单纯膨胀")
            elif pos and not neg:
                print("  ⇒ **一致偏大** ⇒ 更像全局图那 1 格膨胀（0.05 m）")
        print("\n  ⚠ 本脚本只量测、不改任何算法。读完请把 ③④⑤⑥ 的判断"
              "（尤其是 ⑤ 的\"符号会不会翻\"）写进 "
              "doc/minco_trajectory_plan.md §M5.0 的实施记录，再动算法。")
        return 0
    finally:
        p.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
