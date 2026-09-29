#!/usr/bin/env python3
"""动态障碍**扫过**之后，哪些格清不掉？

用户 2026-09-29 报："模拟一个动态障碍物在机器前面运动，发现会有一些栅格无法清除"。

和 `obstacle_clear_probe.py`（原地放/删，只测"静止交替"）不同，这里让障碍**真的动**：
在车前方沿车体横向（或纵向）往复扫 N 个来回，**逐 tick 记录箱子占过的世界格**
（= 它真实占用过的格集合），然后撤障，逐秒统计两层图：

  · `occupancy_2d`         —— 3D 投影出来的 2D 占据层
  · `occupancy_inflate_2d` —— 膨胀层，MPC 做**硬碰撞判定**真正用的那一张

输出：
  · 轨迹带内两层各自还剩几格（每秒一行）
  · **永不消失的格**：逐个打印世界坐标 + 它属于**最早/最晚**哪一趟扫过
    + 上游点云在这一带还有没有点（分辨"感知没清" vs "上游没消"）

★ 两个口径上的讲究（不然会读错数）：
  1. **用世界坐标做键，不用格索引**：局部图是滑动窗口，索引含义会变；
  2. **先减基线**：扫过前就占着的格（墙/货架）当然"清不掉"，那是正常的，
     只统计"扫过期间**新增**、撤障后仍在"的格。

用法（仿真 + 定位 + perception 在跑；**车必须静止**，脚本会先自检）：

    python3 test/sim/obstacle_sweep_probe.py                    # 前方 2 m 横穿，来回 4 次
    python3 test/sim/obstacle_sweep_probe.py --span 1.6 --rounds 6 --speed 0.5
    python3 test/sim/obstacle_sweep_probe.py --mode fwd --d 2.5  # 沿车头方向迎面/远离
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_avoidance as ta                                       # noqa: E402
from near_field_probe import LIVE, OCC_THRESH, Probe              # noqa: E402


class SweepProbe(Probe):
    """Probe + 膨胀层订阅 + 移动障碍的能力"""

    def __init__(self, world, name):
        super().__init__(world, name)
        self.map_infl = None
        self.n_infl = 0
        self.cloud_xyz = []        # 上游点云最近一帧的 (x, y, z) —— 要按 z 分层
        self.sensor_z = None       # 雷达在 map 系的 z（算“射线能穿到多高”要用）
        self.occ3d = None          # /grid_map/occupancy（3D 占据体素中心）
        self.occ3d_t = 0.0
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_inflate_2d",
                                 self.on_map_infl, LIVE)
        self.create_subscription(PointCloud2, "/grid_map/occupancy",
                                 self.on_occ3d, LIVE)

    def on_occ3d(self, m):
        """3D 占据层（体素中心点云）—— 用来判“3D 没清”还是“2D 层漏更新”

        限速 1 s：全图占据体素可能上万点，Python 解一遍不便宜。
        """
        now = time.time()
        if now - self.occ3d_t < 1.0:
            return
        self.occ3d_t = now
        self.occ3d = parse_xyz(m)

    def on_pose(self, m):
        """覆盖父类：多存一个雷达 z（父类只存 x/y/yaw）"""
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        self.sensor_z = m.pose.pose.position.z

    def on_map_infl(self, m):
        self.map_infl = m
        self.n_infl += 1

    def on_cloud(self, m):
        """覆盖父类：**保留 z**。

        父类只存 (x, y) ⇒ 无法区分“箱体表面的点”与“地面点”，而这两者意义完全
        相反：前者说明上游还有东西（感知清不掉是合理的），后者只是背景。
        限速 0.5 s（Python 全解 3 万点会把 spin 拖慢，读数滞后）。
        """
        now = time.time()
        if now - self.cloud_t_last < 0.5:
            return
        self.cloud_t_last = now
        pts = parse_xyz(m)
        if pts:
            self.cloud_xyz = pts
            self.n_cloud += 1

    def remove_checked(self):
        """删模型并**检查返回值**（父类的 remove 不看结果，删失败会被误读成“清不掉”）"""
        from ros_gz_interfaces.srv import DeleteEntity
        req = DeleteEntity.Request()
        req.entity = Entity(name=self.model, type=Entity.MODEL)
        f = self.cli_del.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return bool(f.done() and f.result() and f.result().success)

    def set_pose(self, x, y, z):
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.model, type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.w = 1.0
        self.cli_pose.call_async(req)


def parse_xyz(m):
    """PointCloud2 → [(x, y, z)]（按**字段名**解析，不用偏移）"""
    raw = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
    return [(float(p[0]), float(p[1]), float(p[2])) for p in raw]


# ---------------------------------------------------------------- 世界格工具
def wkey(x, y, res):
    """世界坐标 → 世界格键（不依赖局部图 origin，滑窗也不会错）"""
    return (int(round(x / res)), int(round(y / res)))


def grid_occ_keys(grid, thresh=OCC_THRESH):
    """局部图里所有占据格 → {世界格键: 值}"""
    if grid is None:
        return {}
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h, d = grid.info.width, grid.info.height, grid.data
    out = {}
    for j in range(h):
        base = j * w
        ky = int(round((oy + (j + 0.5) * res) / res))
        for i in range(w):
            if d[base + i] >= thresh:
                out[(int(round((ox + (i + 0.5) * res) / res)), ky)] = d[base + i]
    return out


def rect_keys(cx, cy, sx, sy, res):
    """箱子矩形覆盖的世界格键（外扩半格，与 perception 的 footprint 判据同口径）"""
    hx, hy = sx / 2 + res / 2, sy / 2 + res / 2
    ks = set()
    for j in range(int(math.floor((cy - hy) / res)),
                   int(math.ceil((cy + hy) / res)) + 1):
        for i in range(int(math.floor((cx - hx) / res)),
                       int(math.ceil((cx + hx) / res)) + 1):
            ks.add((i, j))
    return ks


def _intbound(s, ds):
    """复刻 raycast.cpp 的 intbound"""
    if ds < 0:
        return _intbound(-s, -ds)
    s = s % 1.0
    return (1 - s) / ds


def ray_voxels(a, b, res, max_steps=400):
    """复刻 perception/raycast.cpp 的 RayCaster（Amanatides-Woo DDA）。

    返回 a→b 经过的体素集合 {(i, j, k)}，网格口径 = `floor(世界坐标 / res)`
    （与 grid_map 里 `pt_w / resolution` 的写法一致 —— 方向不影响体素集合）。
    """
    s = [a[0] / res, a[1] / res, a[2] / res]
    e = [b[0] / res, b[1] / res, b[2] / res]
    x, y, z = (int(math.floor(v)) for v in s)
    ex, ey, ez = (int(math.floor(v)) for v in e)
    dx, dy, dz = ex - x, ey - y, ez - z
    out = set()
    if dx == 0 and dy == 0 and dz == 0:
        return {(x, y, z)}
    stepx = (dx > 0) - (dx < 0)
    stepy = (dy > 0) - (dy < 0)
    stepz = (dz > 0) - (dz < 0)
    inf = float("inf")
    tmaxx = _intbound(s[0], dx) if dx else inf
    tmaxy = _intbound(s[1], dy) if dy else inf
    tmaxz = _intbound(s[2], dz) if dz else inf
    tdx = (stepx / dx) if dx else inf
    tdy = (stepy / dy) if dy else inf
    tdz = (stepz / dz) if dz else inf
    for _ in range(max_steps):
        out.add((x, y, z))
        if x == ex and y == ey and z == ez:
            break
        if tmaxx < tmaxy:
            if tmaxx < tmaxz:
                x += stepx
                tmaxx += tdx
            else:
                z += stepz
                tmaxz += tdz
        elif tmaxy < tmaxz:
            y += stepy
            tmaxy += tdy
        else:
            z += stepz
            tmaxz += tdz
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world_demo")
    ap.add_argument("--name", default="sweep_box")
    ap.add_argument("--d", type=float, default=2.0, help="轨迹中心在车前方多远 [m]")
    ap.add_argument("--mode", choices=("lat", "fwd"), default="lat",
                    help="lat=沿车体横向（横穿）| fwd=沿车头方向（迎面/远离）")
    ap.add_argument("--span", type=float, default=1.2, help="单程跨度 [m]")
    ap.add_argument("--rounds", type=int, default=4, help="来回次数（1 来回 = 2 趟）")
    ap.add_argument("--speed", type=float, default=0.6, help="运动速度 [m/s]")
    ap.add_argument("--sizes", default="0.6,0.6,0.8", help="箱子 sx,sy,sz [m]")
    ap.add_argument("--z", type=float, default=0.40)
    ap.add_argument("--settle", type=float, default=25.0, help="撤障后观察多久 [s]")
    ap.add_argument("--solid", action="store_true",
                    help="箱子参与物理碰撞（默认 ghost，别让车把它顶开）")
    args = ap.parse_args()

    sizes = [float(v) for v in args.sizes.split(",")]
    bridge = ta.start_bridge(args.world)
    rclpy.init()
    p = SweepProbe(args.world, args.name)
    rc = 0
    try:
        if not p.cli_create.wait_for_service(timeout_sec=10.0):
            print("没有 /world/%s/create —— 服务桥没起来？" % args.world)
            return 2
        end = time.time() + 25.0
        while time.time() < end and (p.pose is None or p.map is None):
            p.spin(0.2)
        if p.pose is None or p.map is None:
            print("缺位姿或局部图：仿真/定位/perception 都在跑吗？")
            return 2

        # ---- 车必须静止（否则 footprint 清除 + 滑窗会污染读数）
        x0, y0, _ = p.pose
        p.spin(2.0)
        x1, y1, yaw = p.pose
        drift = math.hypot(x1 - x0, y1 - y0)
        if drift > 0.05:
            print(f"⚠ 车在动（2 s 动了 {drift * 100:.1f} cm）—— 本探针假定车静止，"
                  f"先停车/取消任务再跑")
            return 2

        res = p.map.info.resolution
        print(f"车心(雷达系) ({x1:.2f}, {y1:.2f}, {math.degrees(yaw):.1f}°) | "
              f"图 {p.map.info.width}×{p.map.info.height} @ {res:.2f} m | "
              f"扩张层 {'有' if p.map_infl is not None else '无'}")

        base_occ = set(grid_occ_keys(p.map))
        base_infl = set(grid_occ_keys(p.map_infl))
        print(f"基线占据格：2D {len(base_occ)} | 膨胀 {len(base_infl)}")

        # ---- 先清掉可能残留的同名模型（上次脚本被杀 / 删除失败会留下），等图清回去
        p.remove_checked()
        p.spin(6.0)
        base_occ = set(grid_occ_keys(p.map))
        base_infl = set(grid_occ_keys(p.map_infl))
        print(f"清残留后基线：2D {len(base_occ)} | 膨胀 {len(base_infl)}")

        # ---- 轨迹：车前方 d 处，沿 lat/fwd 方向往复
        ux, uy = math.cos(yaw), math.sin(yaw)
        lx, ly = -math.sin(yaw), math.cos(yaw)
        cx0, cy0 = x1 + args.d * ux, y1 + args.d * uy
        ax, ay = (lx, ly) if args.mode == "lat" else (ux, uy)

        half = args.span / 2.0
        t_half = max(0.2, args.span / max(0.05, args.speed))
        n_legs = 2 * args.rounds
        t_total = n_legs * t_half
        print(f"轨迹：{args.mode} 向，中心 ({cx0:+.2f}, {cy0:+.2f})，"
              f"span {args.span} m × {args.rounds} 来回 @ {args.speed} m/s "
              f"（总 {t_total:.1f} s）")

        if not p.spawn(cx0 - half * ax, cy0 - half * ay, sizes,
                       ghost=not args.solid):
            # 上一次跑留下的同名模型会让 spawn 失败（脚本被杀就会留盒子）
            print("  ⚠ 生成失败（可能已有同名模型），先删一次再试 …")
            p.remove_checked()
            p.spin(1.0)
            if not p.spawn(cx0 - half * ax, cy0 - half * ay, sizes,
                           ghost=not args.solid):
                print("⚠ 生成箱子失败（服务桥？）")
                return 3

        # ---- 扫掠
        print("\n[扫掠中] 逐趟记录轨迹带：")
        trail, trail_new = set(), set()
        leg_sets = []
        t_start = time.time()
        leg = -1
        while True:
            t = time.time() - t_start
            if t >= t_total:
                break
            leg = min(int(t / t_half), n_legs - 1)
            frac = (t - leg * t_half) / t_half
            s = -half + args.span * frac if leg % 2 == 0 else half - args.span * frac
            bx, by = cx0 + s * ax, cy0 + s * ay
            p.set_pose(bx, by, args.z)
            trail |= rect_keys(bx, by, sizes[0], sizes[1], res)
            p.spin(0.05)
            # 每趟第一次进入时采样一次"新增占据"，确认注入真的进了图
            while len(leg_sets) <= leg:
                leg_sets.append(set())
            leg_sets[leg] |= rect_keys(bx, by, sizes[0], sizes[1], res)
        print(f"  扫过 {len(trail)} 个世界格（{n_legs} 趟）")

        # ---- 撤障
        ok = p.remove_checked()
        print(f"\n[撤障] remove() → "
              f"{'成功' if ok else '★ 失败（箱子还在！那图上有障碍就是对的）'}")
        print("[撤障] 逐秒看轨迹带残留（只算**基线之外新增**的格）：")
        t0 = time.time()
        last = None
        while True:
            el = time.time() - t0
            if el > args.settle:
                break
            p.spin(1.0)
            occ2d = set(grid_occ_keys(p.map))
            occinf = set(grid_occ_keys(p.map_infl))
            new2d = (occ2d - base_occ) & trail
            newinf = (occinf - base_infl) & trail
            trail_new = trail
            last = (el, new2d, newinf, occ2d, occinf)
            print(f"  +{el:4.0f}s  轨迹带残留：2D {len(new2d):3d} 格 | "
                  f"膨胀 {len(newinf):3d} 格   （全场 2D {len(occ2d)} / "
                  f"膨胀 {len(occinf)}）")
            if len(new2d) == 0 and len(newinf) == 0 and el > 6.0:
                print("  （两层都清干净了，提前结束）")
                break

        # ---- 明细
        if last is not None:
            el, new2d, newinf, _, _ = last
            stable2d = sorted(new2d)
            stableinf = sorted(newinf)
            print(f"\n★ 结论：轨迹带 {len(trail_new)} 格，观察 {el:.0f} s 后残留 "
                  f"2D {len(stable2d)} 格 / 膨胀 {len(stableinf)} 格")
            if stable2d or stableinf:
                rc = 1
                # 膨胀层天然是占据层的 5~7 倍（每格按 0.3 m 膨胀 ≈ 7×7），
                # 所以"膨胀层更多"**不是**异常；要看两层是否**同步**下降。
                if stableinf and stable2d:
                    r = len(stableinf) / max(1, len(stable2d))
                    print(f"  （膨胀/占据 = {r:.1f}×：5~7× 属正常；明显偏大才说明"
                          f"膨胀计数没跟着回退）")

                # 上游点云按 z 分层：箱体 z∈[0.0, 0.8]，地面 z≈-0.36
                pts = p.cloud_xyz
                band = [(x, y, z) for (x, y, z) in pts
                        if abs(x - cx0) < 0.6 and abs(y - cy0) < 1.2]
                lo = [t for t in band if t[2] < -0.15]
                mid = [t for t in band if -0.15 <= t[2] <= 0.20]
                hi = [t for t in band if t[2] > 0.20]
                print(f"\n上游点云在轨迹带附近（{len(pts)} 点整帧）：{len(band)} 点 = "
                      f"低(z<-0.15) {len(lo)} + 中(-0.15~0.20) {len(mid)} + "
                      f"高(>0.20) {len(hi)}")
                print("   ★ 判据：中/高档有点 ⇒ **上游点云里真有东西**（箱子的点没消）"
                      "⇒ 感知清不掉是忠实反映输入，要查上游；"
                      "只有低档（地面）⇒ 上游干净，清不掉是感知的锅。")

                print("\n永不消失的格（2D 层，标出最早/最晚被扫到的趟号 + 附近点云）：")
                for k in stable2d[:40]:
                    legs = [i for i, s in enumerate(leg_sets) if k in s]
                    wx, wy = k[0] * res, k[1] * res
                    near = [(x, y, z) for (x, y, z) in pts
                            if abs(x - wx) < 0.25 and abs(y - wy) < 0.25]
                    nh = sum(1 for t in near if t[2] > 0.15)
                    tag = f"趟 {legs[0]}~{legs[-1]}" if legs else "不在箱矩形内?"
                    print(f"   ({wx:+6.2f}, {wy:+6.2f})  {tag:12s} "
                          f"点云 {len(near):3d} 点（高处 {nh}）"
                          f"{'  ← 高处有点 = 上游还有实体' if nh else ''}")

                # ---- 方向验证：残留格所在方向上，点云里还有**更远**的点吗？
                #   raycast 清 free 的唯一途径就是"某条射线从传感器出发穿过了该格"，
                #   而射线来自点云里的回波点。如果那个方向上已经没有**更远的**回波，
                #   就没有任何射线穿过该格 ⇒ 永远收不到 miss ⇒ 永远清不掉。
                print("[3D vs 2D] 残留格处的 3D 体素（按高度看；2D 高度带 = "
                      "proj_z_min -0.20 ~ proj_z_max 2.00）：")
                occ3d = p.occ3d or []
                sz = p.sensor_z if p.sensor_z is not None else 0.0
                inband, lowband, z_all = 0, 0, []
                for k in stable2d[:20]:
                    wx, wy = k[0] * res, k[1] * res
                    zs = sorted(t[2] for t in occ3d
                                if abs(t[0] - wx) < 0.15 and abs(t[1] - wy) < 0.15)
                    z_all += zs
                    nin = [z for z in zs if -0.20 <= z <= 2.00]
                    inband += 1 if nin else 0
                    lowband += 1 if (zs and not nin) else 0
                    # ★ 这个 (x,y) 位置上，点云里哪条射线能穿得**最高**？
                    #   （射线按水平距离线性插值 z；拿它和残留体素的 z 比）
                    d_self = math.hypot(wx - x1, wy - y1)
                    brg = math.atan2(wy - y1, wx - x1)
                    z_ray_max = None
                    for (px, py, pz) in pts:
                        b = math.atan2(py - y1, px - x1)
                        if abs((b - brg + math.pi) % (2 * math.pi) - math.pi) \
                                > 1.5 * math.pi / 180:
                            continue
                        dp = math.hypot(px - x1, py - y1)
                        if dp < d_self + 0.15:
                            continue
                        zr = sz + (pz - sz) * (d_self / dp)
                        if z_ray_max is None or zr > z_ray_max:
                            z_ray_max = zr
                    zs_s = "[" + ", ".join("%+.2f" % z for z in zs) + "]"
                    print(f"   ({wx:+.2f},{wy:+.2f}) 3D 体素 z={zs_s}  "
                          f"| 穿过此处的射线最高可及 z="
                          f"{'—' if z_ray_max is None else '%+.2f' % z_ray_max}")
                print(f"  合计：高度带内(z∈[-0.20,2.00])仍有残留的格 **{inband}** 个；"
                      f"只有带外(地面)残留的格 {lowband} 个（3D 点云 {len(occ3d)} 点，"
                      f"z 范围 "
                      f"{'—' if not z_all else '%+.2f~%+.2f' % (min(z_all), max(z_all))}）")
                print("  ★ 判据：如果残留体素的 z **高于**该处射线最高可及的 z ⇒ "
                      "“撤障后没有任何射线穿过那一层” ⇒ 观测缺失（不是计数错误）；"
                      "若相反，则要查 raycast/缓存去重。")

                print("\n[射线穿行验证] 复刻 perception 的 DDA：那些残留体素到底有没有"
                      "被射线穿过？（这是能不能收到 miss 的唯一途径）")
                n_never = n_all = 0
                for k in stable2d[:20]:
                    wx, wy = k[0] * res, k[1] * res
                    d_self = math.hypot(wx - x1, wy - y1)
                    brg = math.atan2(wy - y1, wx - x1)
                    targets = {
                        (int(math.floor(t[0] / res)), int(math.floor(t[1] / res)),
                         int(math.floor(t[2] / res))) for t in occ3d
                        if abs(t[0] - wx) < 0.15 and abs(t[1] - wy) < 0.15}
                    hit = set()
                    for (px, py, pz) in pts:
                        b = math.atan2(py - y1, px - x1)
                        if abs((b - brg + math.pi) % (2 * math.pi) - math.pi) \
                                > 2 * math.pi / 180:
                            continue
                        if math.hypot(px - x1, py - y1) < d_self + 0.2:
                            continue
                        hit |= ray_voxels((x1, y1, sz), (px, py, pz), res) & targets
                    nvr = sorted(targets - hit)
                    if nvr:
                        n_never += 1
                    else:
                        n_all += 1
                    print(f"   ({wx:+.2f},{wy:+.2f}) 目标体素 {len(targets):2d} → "
                          f"被穿过 {len(hit):2d}，**从未被穿过 {len(nvr):2d}**"
                          + (f"  例: z={['%+.2f' % (v[2] * res) for v in nvr[:4]]}"
                             if nvr else ""))
                print(f"  ★ 合计：{min(len(stable2d), 20)} 个残留格里，存在“从未被"
                      f"任何射线穿过”体素的有 **{n_never}** 个；全部体素都有射线穿过的"
                      f" {n_all} 个")
                print("  ★ 判据：从未被穿过 ⇒ 永远收不到 miss ⇒ **这是观测缺失，不是"
                      "计数错误**；若都有射线穿过却仍占着，则是 raycast/去重逻辑的问题。")
            else:
                print("（轨迹带两层都干净）")

            # ---- 阶段三：清图后残留会不会**立刻重现**（最有分辨力的一刀）
            print("\n[阶段三] 调 /grid_map/clear_map：区分“清不掉” vs “清了又回来”：")
            cli = p.create_client(Trigger, "/grid_map/clear_map")
            if not cli.wait_for_service(timeout_sec=5.0):
                print("  没有 /grid_map/clear_map 服务（perception 没重编/名字不对）")
            else:
                f = cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(p, f, timeout_sec=5.0)
                r = f.result() if f.done() else None
                print(f"  服务返回 success={getattr(r, 'success', None)}")
                for dt in (1.0, 2.0, 4.0):
                    p.spin(dt)
                    occ2d = set(grid_occ_keys(p.map))
                    back = (occ2d - base_occ) & trail
                    print(f"  清图后 +{dt:.0f}s：轨迹带内 {len(back):3d} 格"
                          + ("  ← ★ 立刻重现 ⇒ 上游点云里还有东西"
                             if (back and dt <= 1.0) else ""))
        return rc
    finally:
        try:
            p.destroy_node()
            rclpy.shutdown()
        except Exception:                                          # noqa: BLE001
            pass
        if bridge is not None:
            try:
                os.killpg(os.getpgid(bridge.pid), signal.SIGTERM)
            except Exception:                                      # noqa: BLE001
                pass


if __name__ == "__main__":
    sys.exit(main())
