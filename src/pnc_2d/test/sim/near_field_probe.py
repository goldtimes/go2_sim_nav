#!/usr/bin/env python3
"""近距离盲区诊断：**正前方 d 米处的障碍，感知图 / ESDF 还看得见吗？**

起因（2026-09-29）：S2 动态横穿时箱子会**顶上机器人**（实测车在动时的净距
-0.119 m，车被推得贴墙）。用户问"是近距离没有感知到么"。

代码里确实有一个**结构性**盲区，不是雷达的问题：

    perception.yaml
      grid_map.footprint_clear_enable: true
      grid_map.footprint_length/width: 0.70 / 0.40        （Go2）
      grid_map.footprint_clear_as_unknown: false          （抹成**空闲**）
      判定＝"格与矩形相交"，矩形再外扩半格（res/2 = 0.05 m）

⇒ 感知**每一帧都把机器人自身那块矩形从 2D 图上抹成空闲**（占据层 + 膨胀层，
  Nav2 local costmap 同做法、目的是干掉自车点云）。于是：
  · 车心沿车头 ±(0.35+0.05) = **±0.40 m**、左右 ±(0.20+0.05) = ±0.25 m 的格
    **永远是空闲**；
  · ESDF/MPC 都建立在这张图上 ⇒ 这个范围里的障碍在控制器眼里就是**空地**。

本脚本把这个盲区**量出来**：把箱子依次放到车头前方 d = 1.5 → 0.2 m，每一步读
  ① 局部 2D 图里"盒子矩形覆盖的格"有多少是占据（/grid_map/occupancy_2d）；
  ② 盒子中心的 ESDF 值（/grid_map/esdf_2d 的 intensity = 到最近障碍的距离）；
  ③ 车心前方 d/2 处的 ESDF；
  ④ local_status 的状态。
ESDF 若在 d ≤ 0.40 m 时仍然报 3.0（= 截断上限 = 空旷），就证明"近处看不见"是
**设计使然**，而不是点云质量或时序问题。

用法（仿真 + 定位 + perception + map_server + pnc_2d 都在跑；本脚本**不起栈、
不动车**，只往世界里放/挪一个箱子）：

    python3 test/sim/near_field_probe.py
    python3 test/sim/near_field_probe.py --dists 1.5,1.0,0.6,0.4,0.3,0.2
    python3 test/sim/near_field_probe.py --lateral 0.30   # 往车头左侧偏 0.3 m
    python3 test/sim/near_field_probe.py --solid          # 让箱子参与碰撞（默认 ghost）

⚠ 默认把箱子做成 ghost（collide_bitmask=0）：d 小到 0.2 m 时箱子与车体几何重叠，
  参与碰撞的话会把车**顶开**，测的就不是感知了。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from sensor_msgs.msg import PointCloud2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_common as sc                                          # noqa: E402
import test_avoidance as ta                                      # noqa: E402
from pnc_2d.msg import LocalStatus                               # noqa: E402
from probe_lateral_bias import ESDF_MAX, OCC_THRESH, parse_xy_d   # noqa: E402

LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)

SDF_TMPL = ("<sdf version='1.7'><model name='%s'><static>false</static>"
            "<link name='link'>"
            "<collision name='c'><geometry><box><size>%f %f %f</size></box>"
            "</geometry>%s</collision>"
            "<visual name='v'><geometry><box><size>%f %f %f</size></box></geometry>"
            "<material><ambient>1 0.15 0.15 1</ambient></material></visual>"
            "<inertial><mass>5.0</mass></inertial>"
            "</link></model></sdf>")


class Probe(rclpy.node.Node):
    def __init__(self, world, name):
        super().__init__("near_field_probe")
        self.world = world
        self.model = name
        self.pose = None
        self.map = None
        self.esdf = None
        self.status = ""
        self.status_msg = ""
        self.n_map = 0
        self.n_esdf = 0
        self.cloud = None          # 上游点云（map 系）最近一帧的 (x,y) 列表
        self.n_cloud = 0
        self.cloud_t_last = 0.0
        self.create_subscription(Odometry, "/lightning/perception/pose",
                                 self.on_pose, LIVE)
        self.create_subscription(OccupancyGrid, "/grid_map/occupancy_2d",
                                 self.on_map, LIVE)
        self.create_subscription(PointCloud2, "/grid_map/esdf_2d",
                                 self.on_esdf, LIVE)
        self.create_subscription(PointCloud2, "/lightning/perception/cloud",
                                 self.on_cloud, LIVE)
        self.create_subscription(LocalStatus, "/pnc_2d/local_status",
                                 self.on_status, sc.LATCHED)
        self.cli_create = self.create_client(
            SpawnEntity, "/world/%s/create" % world)
        self.cli_pose = self.create_client(
            SetEntityPose, "/world/%s/set_pose" % world)
        self.cli_del = self.create_client(
            DeleteEntity, "/world/%s/remove" % world)

    # ------------------------------------------------------------ 订阅
    def on_pose(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_map(self, m):
        self.map = m
        self.n_map += 1

    def on_esdf(self, m):
        pts = parse_xy_d(m)
        if pts:
            self.esdf = pts
            self.n_esdf += 1

    def on_cloud(self, m):
        """上游点云（`lightning/perception/cloud`，已是 map 系）→ 只留 (x,y)。

        ★ 为什么要看它：局部图清干净了不等于"幽灵障碍"消失了 —— 如果**点云本身**
        还带着已消失那个物体的点，下一帧就会立刻把障碍重新写回图里，看起来就是
        "清了又回来"。探针里把两路分开数，就能分清是**感知没清**还是**上游没消**。

        ⚠ 解析要**限速**：点云 10 Hz × 6 万点，Python 全解会把 spin 拖到跟不上，
          读数随之滞后（实测：不清图也会"冻"在旧值上）。2 Hz 足够看"点还在不在"。
        """
        now = time.time()
        if now - self.cloud_t_last < 0.5:
            return
        self.cloud_t_last = now
        pts = parse_xy_d(m)
        if pts:
            self.cloud = [(p[0], p[1]) for p in pts]
            self.n_cloud += 1

    def on_status(self, m):
        self.status = m.status_name
        self.status_msg = m.message

    # ------------------------------------------------------------ 查询
    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def map_cell(self, x, y):
        """世界点 → 局部图格值（图外 None）"""
        g = self.map
        if g is None:
            return None
        res = g.info.resolution
        ix = int(math.floor((x - g.info.origin.position.x) / res))
        iy = int(math.floor((y - g.info.origin.position.y) / res))
        if ix < 0 or iy < 0 or ix >= g.info.width or iy >= g.info.height:
            return None
        return g.data[iy * g.info.width + ix]

    def rect_cells(self, cx, cy, sx, sy):
        """盒子矩形覆盖到的局部图格 → (总格数, 占据格数, 未知格数)

        判据与 perception 的 footprint 清除一致：格与矩形相交（矩形外扩半格）。
        盒子摆得与车同向（无旋转），所以直接用轴对齐矩形。
        """
        g = self.map
        if g is None:
            return (0, 0, 0)
        res = g.info.resolution
        hx, hy = sx / 2.0 + res / 2.0, sy / 2.0 + res / 2.0
        x0, x1 = cx - hx, cx + hx
        y0, y1 = cy - hy, cy + hy
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        i0 = int(math.floor((x0 - ox) / res))
        i1 = int(math.ceil((x1 - ox) / res))
        j0 = int(math.floor((y0 - oy) / res))
        j1 = int(math.ceil((y1 - oy) / res))
        tot = occ = unk = 0
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                if i < 0 or j < 0 or i >= g.info.width or j >= g.info.height:
                    continue
                tot += 1
                v = g.data[j * g.info.width + i]
                if v < 0:
                    unk += 1
                elif v >= OCC_THRESH:
                    occ += 1
        return (tot, occ, unk)

    def cloud_rect_count(self, cx, cy, sx, sy):
        """上游点云落在矩形（+半格余量）内的点数 —— 已消失物体的"点残留"看这个"""
        if not self.cloud:
            return 0
        hx, hy = sx / 2, sy / 2
        n = 0
        for (px, py) in self.cloud:
            if abs(px - cx) <= hx and abs(py - cy) <= hy:
                n += 1
        return n

    def esdf_at(self, x, y, max_d=0.15):
        """最近的 ESDF 采样点的距离值（采样间距 0.2 m ⇒ 0.15 m 内必有）"""
        if not self.esdf:
            return None
        best, bd = None, 1e9
        for (px, py, d) in self.esdf:
            dd = (px - x) ** 2 + (py - y) ** 2
            if dd < bd:
                bd, best = dd, d
        if best is None or math.sqrt(bd) > max_d:
            return None
        return best

    # ------------------------------------------------------------ gazebo
    def spawn(self, x, y, sizes, ghost):
        sx, sy, sz = sizes
        cmask = "<collide_bitmask>0x00</collide_bitmask>" if ghost else ""
        req = SpawnEntity.Request()
        req.entity_factory.name = self.model
        req.entity_factory.sdf = SDF_TMPL % (self.model, sx, sy, sz, cmask,
                                             sx, sy, sz)
        req.entity_factory.pose = Pose()
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.orientation.w = 1.0
        f = self.cli_create.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
        return bool(f.done() and f.result() and f.result().success)

    def remove(self):
        req = DeleteEntity.Request()
        req.entity = Entity(name=self.model, type=Entity.MODEL)
        f = self.cli_del.call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="world_demo")
    ap.add_argument("--name", default="nf_probe_box")
    ap.add_argument("--dists", default="2.0,1.5,1.0,0.7,0.5,0.45,0.40,0.35,0.30,0.20",
                    help="箱子中心到**车心**的距离序列 [m]")
    ap.add_argument("--sizes", default="0.6,0.6,0.8")
    ap.add_argument("--lateral", type=float, default=0.0,
                    help="沿车头左侧偏移 [m]（默认 0 = 正前方）")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="每步放好箱子后等多久再读图 [s]（感知 ~5 Hz + 图更新）")
    ap.add_argument("--solid", action="store_true",
                    help="箱子参与物理碰撞（默认 ghost，见文件头注释）")
    args = ap.parse_args()

    sizes = [float(v) for v in args.sizes.split(",")]
    dists = [float(v) for v in args.dists.split(",")]
    bridge = ta.start_bridge(args.world)
    rclpy.init()
    p = Probe(args.world, args.name)
    try:
        if not p.cli_create.wait_for_service(timeout_sec=10.0):
            print("没有 /world/%s/create —— 服务桥没起来？" % args.world)
            return 2
        # 等位姿 + 两张图都到位
        end = time.time() + 25.0
        while time.time() < end and (p.pose is None or p.map is None
                                     or p.esdf is None):
            p.spin(0.2)
        if p.pose is None:
            print("没有位姿：仿真/定位没起？")
            return 2
        x, y, yaw = p.pose
        print(f"车心 ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.1f}° | "
              f"局部图 {p.map.info.width}×{p.map.info.height} "
              f"@{p.map.info.resolution:.2f} m | ESDF 点 {len(p.esdf or [])} | "
              f"local_status={p.status or '（还没收到）'}")
        half_len = 0.78 / 2 + 0.05      # footprint 清除矩形的前向半长（+半格）
        print(f"★ 清除矩形 = 长 0.78（车心为基准偏 −0.038）⇒ 车体系被抹成空闲的"
              f"范围 ≈ x∈[{0.30 - 0.78 / 2 - 0.05:.2f}, "
              f"{0.30 - 0.78 / 2 + 0.78 + 0.05:.2f}]（含半格外扩）")
        print("  注：本栏 d 是从**雷达**量到盒心的距离（定位话题仍是雷达位姿）；"
              "车心比它靠后 0.22 m，车体前沿在雷达前 +0.112 m。")
        print("\n   d[m]  盒矩形格 占据格 未知格 | 盒心ESDF 前方d/2 ESDF | "
              "local_status")
        print("  " + "-" * 78)
        for d in dists:
            bx = x + d * math.cos(yaw) - args.lateral * math.sin(yaw)
            by = y + d * math.sin(yaw) + args.lateral * math.cos(yaw)
            p.remove()
            p.spin(0.4)
            if not p.spawn(bx, by, sizes, ghost=not args.solid):
                print("  ⚠ 生成箱子失败（服务桥？），中止")
                return 3
            p.spin(args.settle)
            tot, occ, unk = p.rect_cells(bx, by, sizes[0], sizes[1])
            e_box = p.esdf_at(bx, by)
            hx = x + 0.5 * d * math.cos(yaw) - args.lateral * math.sin(yaw)
            hy = y + 0.5 * d * math.sin(yaw) + args.lateral * math.cos(yaw)
            e_half = p.esdf_at(hx, hy)
            tag = "  ← 车体矩形内（设计盲区）" if d <= half_len else ""
            print(f"  {d:5.2f}  {tot:7d} {occ:6d} {unk:6d} | "
                  f"{(e_box if e_box is not None else float('nan')):7.2f} "
                  f"{(e_half if e_half is not None else float('nan')):9.2f} | "
                  f"{p.status or '?':<12}{tag}")
        print("\n  读法：")
        print(f"   · 占据格 > 0 = 感知图里看得见；= 0 且 ESDF = {ESDF_MAX:.1f}"
              f"（截断上限）= **看不见**")
        print("   · 车心 → 盒心 d ≤ 0.40 m 那几行若「看不见」，就是这个盲区的证据；")
        print("     它说明：**靠局部图永远追不上贴到身上的障碍** —— 要么让感知"
              "别把 footprint 抹成空闲（要另想办法去掉自车点云），要么加一层"
              "**近身检测**（原始点云落在 footprint+余量里 ⇒ 直接报挡住/停）。")
        p.remove()
        print("\n  （已删除测试箱子）")
    finally:
        try:
            p.destroy_node()
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
        ta.stop_proc(bridge)
    return 0


if __name__ == "__main__":
    sys.exit(main())
