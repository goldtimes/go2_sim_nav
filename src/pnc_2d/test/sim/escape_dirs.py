#!/usr/bin/env python3
"""在当前位姿上看：往哪个方向走得出去（不要求原地转向）。

用发布的全局图（含禁行区 Z1~Z3 烧入）判断：从车当前位姿出发，朝
"当前机头 + Δ" 直行 L 米，车体轮廓是否一直不压障碍。Δ 小（<45°）时
`heading_shim` 不会原地转，MPC 自己边转边走 ⇒ 这类方向才是"能开出去"的。
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy                                                      # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry                  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

import check_path_clearance as cpc                                # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
LIVE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class N(rclpy.node.Node):
    def __init__(self):
        super().__init__("escape_probe")
        self.pose = None
        self.g = None
        self.create_subscription(Odometry, "/lightning/perception/pose",
                                 self.on_pose, LIVE)
        self.create_subscription(OccupancyGrid, "/global_map/occupancy",
                                 self.on_g, LATCHED)

    def on_pose(self, m):
        self.pose = m.pose.pose

    def on_g(self, m):
        self.g = m


def clearance_dir(grid, x, y, yaw, length=2.0, step=0.05):
    """朝 yaw 直行 length 米，车体不压障碍则返回 True，并给出最窄余量。"""
    worst = 1.0e9
    n = max(1, int(length / step))
    for k in range(n + 1):
        px = x + math.cos(yaw) * step * k
        py = y + math.sin(yaw) * step * k
        if cpc.footprint_hits(grid, px, py, yaw, margin=0.0)[0]:
            return False, 0.0
        worst = min(worst, cpc.margin_at(grid, px, py, yaw, hi=0.4))
    return True, worst


def main():
    rclpy.init()
    n = N()
    for _ in range(200):
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.pose and n.g:
            break
    if not (n.pose and n.g):
        print("没位姿/没全局图")
        return 2
    x, y = n.pose.position.x, n.pose.position.y
    q = n.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    print(f"车 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°) | "
          f"全局图最小净距 {cpc.sc.ObstacleIndex(n.g).clearance(x, y):.3f} m")
    print("\n能直行 2.0 m 的方向（Δ=相对机头角度）：")
    ok = []
    for d in range(-180, 181, 15):
        yaw2 = yaw + math.radians(d)
        free, worst = clearance_dir(n.g, x, y, yaw2)
        if free:
            ok.append((d, worst, x + 2.0 * math.cos(yaw2), y + 2.0 * math.sin(yaw2)))
            print(f"  Δ={d:+4d}° → 最窄余量 {worst*100:4.0f} cm，"
                  f"2 m 后到 ({x + 2.0*math.cos(yaw2):.2f}, "
                  f"{y + 2.0*math.sin(yaw2):.2f})")
    if not ok:
        print("  没有任何方向能直行 2 m ⇒ 真的被围住了")
    else:
        best = max(ok, key=lambda v: v[1])
        print(f"\n★ 推荐：Δ={best[0]:+d}°（余量最大 {best[1]*100:.0f} cm）"
              f"目标点可发 ({best[2]:.2f}, {best[3]:.2f})")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
