#!/usr/bin/env python3
"""让仿真里的 A1 保持站立姿态。

上游 legbot_3D_Nav 的站立姿态由 RL 策略（unitree_guide/junior_ctrl）维持，
本仓库暂未移植该策略，因此用这个节点把一个固定的 A1 标准站立姿态
（hip=0.0, thigh=0.9, calf=-1.8 rad）以斜坡方式发布给
joint_group_position_controller，避免机器人一 spawn 就瘫在地上。

用法:
    ros2 run a1_description stand_pose.py --ros-args -r __ns:=/a1 \
        -p delay:=2.0 -p ramp_time:=3.0 -p rate:=50.0

为什么需要 delay：
    A1 spawn 时 12 个关节都是 0（腿伸直）。腿伸直时足端在 base 下方 0.40m，
    正好是一个稳定的四点支撑姿态，先让机器人自由落体到伸直腿上站定，
    再缓慢折叠成站立姿态（下蹲），重心始终落在支撑多边形内，机器人不会翻。
    如果一出生就开始折腿，落地瞬间腿在动，很容易失稳翻倒（实测确实会翻）。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# 与 config/a1_ros2_control.yaml 中 joints 顺序一致：FL, FR, RL, RR × (hip, thigh, calf)
STAND_POSE = [
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
    0.0, 0.9, -1.8,
]


class StandPose(Node):
    def __init__(self):
        super().__init__('a1_stand_pose')

        self.declare_parameter('rate', 50.0)
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('ramp_time', 2.0)
        self.declare_parameter('target', STAND_POSE)
        self.declare_parameter(
            'controller_topic', 'joint_group_controller/commands')

        rate = float(self.get_parameter('rate').value)
        self._delay = float(self.get_parameter('delay').value)
        self._ramp_time = float(self.get_parameter('ramp_time').value)

        target = [float(v) for v in self.get_parameter('target').value]
        self._target = target if target else list(STAND_POSE)

        self._start = [0.0] * len(self._target)
        self._start_time = None

        topic = self.get_parameter('controller_topic').value
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'stand_pose 已启动: topic={topic}, delay={self._delay}s, '
            f'ramp_time={self._ramp_time}s')

    def _tick(self):
        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now

        elapsed = (now - self._start_time).nanoseconds * 1e-9 - self._delay
        if self._ramp_time <= 0:
            ratio = 1.0 if elapsed >= 0 else 0.0
        else:
            ratio = min(1.0, max(0.0, elapsed / self._ramp_time))

        msg = Float64MultiArray()
        msg.data = [
            s + (t - s) * ratio for s, t in zip(self._start, self._target)
        ]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
