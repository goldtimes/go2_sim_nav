#!/usr/bin/env python3
"""把自研 map_server 的输出与官方 nav2_map_server 逐格比对。

用法（两个终端）：
  1) 官方参考（lifecycle，需要 configure/activate）：
       ros2 run nav2_map_server map_server --ros-args \\
           -p yaml_filename:=/home/gmd/rcs/maps/go2_sim_factory/map.yaml \\
           -p topic_name:=/nav2_map -p frame_id:=map
       ros2 lifecycle set /map_server configure
       ros2 lifecycle set /map_server activate
  2) 自研：
       ros2 run map_server map_server_node --ros-args \\
           -p map_dir:=/home/gmd/rcs/maps/go2_sim_factory
  3) 比对：
       python3 compare_maps.py

比对内容：width/height/resolution/origin，以及每一格的取值（100/0/-1）是否完全一致。
这是**独立实现互证**：两边的 PGM 解析/阈值/行序翻转若有任何一处不同，格值就会差。
"""

import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

OURS = sys.argv[1] if len(sys.argv) > 1 else "/global_map/occupancy"
REF = sys.argv[2] if len(sys.argv) > 2 else "/nav2_map"
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
# 默认 transient_local：官方 nav2_map_server 只发一次(latched)，Volatile 晚订阅收不到
DURABILITY = (sys.argv[4] if len(sys.argv) > 4 else "tl").lower()


class Cmp(Node):
    def __init__(self):
        super().__init__("compare_maps")
        # durability=tl 用于和官方 latched 发布端比对；volatile 则能验证自研节点的
        # 周期性重发（republish_interval）对任何 QoS 的订阅者都有效
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=(DurabilityPolicy.TRANSIENT_LOCAL
                                  if DURABILITY == "tl" else DurabilityPolicy.VOLATILE))
        self.a = None
        self.b = None
        self.create_subscription(OccupancyGrid, OURS, lambda m: setattr(self, "a", m), q)
        self.create_subscription(OccupancyGrid, REF, lambda m: setattr(self, "b", m), q)
        self.t0 = time.time()
        self.create_timer(0.5, self.tick)

    def tick(self):
        if time.time() - self.t0 < DUR:
            return
        if self.a is None or self.b is None:
            print(f"没收到：自研={self.a is not None} 官方={self.b is not None}")
            print("（官方 nav2_map_server 是 lifecycle 节点，别忘了 configure + activate）")
            raise SystemExit(1)

        ok = True
        for tag, m in (("自研", self.a), ("官方", self.b)):
            print(f"{tag}: {m.info.width}x{m.info.height} @ {m.info.resolution:.3f} "
                  f"origin=({m.info.origin.position.x:.3f},{m.info.origin.position.y:.3f}) "
                  f"frame={m.header.frame_id}")
            n_occ = sum(1 for v in m.data if v == 100)
            n_free = sum(1 for v in m.data if v == 0)
            n_unk = sum(1 for v in m.data if v < 0)
            print(f"      占据 {n_occ} / 空闲 {n_free} / 未知 {n_unk}")

        checks = [
            ("width", self.a.info.width, self.b.info.width),
            ("height", self.a.info.height, self.b.info.height),
            ("resolution", round(self.a.info.resolution, 6),
             round(self.b.info.resolution, 6)),
            ("origin.x", round(self.a.info.origin.position.x, 6),
             round(self.b.info.origin.position.x, 6)),
            ("origin.y", round(self.a.info.origin.position.y, 6),
             round(self.b.info.origin.position.y, 6)),
        ]
        for name, x, y in checks:
            if x != y:
                print(f"  ✗ {name}: 自研 {x} != 官方 {y}")
                ok = False

        if len(self.a.data) != len(self.b.data):
            print(f"  ✗ 格数不同: {len(self.a.data)} vs {len(self.b.data)}")
            ok = False
        else:
            diff = [i for i, (x, y) in enumerate(zip(self.a.data, self.b.data)) if x != y]
            if diff:
                ok = False
                print(f"  ✗ 有 {len(diff)} 格不同，前 5 个：")
                w = self.a.info.width
                for i in diff[:5]:
                    print(f"      (x={i % w}, y={i // w}) 自研 {self.a.data[i]} "
                          f"官方 {self.b.data[i]}")
            else:
                print(f"  ✓ 全部 {len(self.a.data)} 格完全一致")

        print("结论：" + ("✅ 与官方实现完全一致" if ok else "❌ 存在差异（见上）"))
        raise SystemExit(0 if ok else 1)


rclpy.init()
try:
    rclpy.spin(Cmp())
except SystemExit as e:
    sys.exit(e.code if e.code is not None else 0)
