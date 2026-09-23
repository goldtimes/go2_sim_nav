#!/usr/bin/env python3
"""接住一条 latched（transient_local）消息并打印摘要。

为什么需要它：`ros2 topic echo` 默认是 volatile 订阅，拿不到 latched（transient_local）
的历史样本 —— 我们对 `/global_map/zones`、`/global_map/occupancy` 这类"只在加载/变更时
发一次"的话题都必须用 transient_local 订阅才看得到（这个问题在 e2e 测试里也踩过）。

用法：
    python3 test/sim/topic_probe.py /global_map/zones [--timeout 10]
    python3 test/sim/topic_probe.py /global_map/zones --kind zones   # 区域摘要
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from pnc_2d.msg import ZoneArray
except ImportError:  # 还没编译出消息时也能用（只支持 OccupancyGrid）
    ZoneArray = None


def make_qos() -> QoSProfile:
    return QoSProfile(depth=1,
                      history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def describe(msg) -> str:
    """按类型给一行摘要（只认识我们关心的话题；其它就报类型/大小）"""
    t = type(msg).__name__
    if t == "ZoneArray":
        lines = [f"inflate = {msg.inflate:.3f} m | 区域 {len(msg.zones)} 个"]
        for z in msg.zones:
            pts = list(z.polygon.points)
            if pts:
                xs = [p.x for p in pts]
                ys = [p.y for p in pts]
                bbox = (f"bbox x[{min(xs):.2f},{max(xs):.2f}] "
                        f"y[{min(ys):.2f},{max(ys):.2f}]")
            else:
                bbox = "空多边形"
            lines.append(f"  · {z.name:<16} {z.type:<12} value={z.value:.2f} "
                         f"顶点 {len(pts)} | {bbox}")
        return "\n".join(lines)
    if t == "OccupancyGrid":
        import collections
        hist = collections.Counter(msg.data)
        occ = sum(v for k, v in hist.items() if k >= 50)
        return (f"{msg.info.width}x{msg.info.height} @ {msg.info.resolution} m | "
                f"占据格 {occ} | 原点多 ({msg.info.origin.position.x:.2f}, "
                f"{msg.info.origin.position.y:.2f})")
    return f"{t}（{len(str(msg))} 字符）"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("pnc2d_topic_probe")
    got = []

    def on_msg(msg):
        got.append(msg)

    cls = OccupancyGrid
    if args.topic.endswith("zones"):
        if ZoneArray is None:
            print("[probe] 没编译出 pnc_2d/msg/ZoneArray")
            return 1
        cls = ZoneArray
    node.create_subscription(cls, args.topic, on_msg, make_qos())

    end = time.time() + args.timeout
    while time.time() < end and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print(f"[probe] {args.timeout:.0f} s 内没收到 {args.topic}"
              f"（话题存在但没有 latched 样本？发布端没发？）")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"[probe] {args.topic}（{len(got)} 条）")
    print(describe(got[-1]))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
