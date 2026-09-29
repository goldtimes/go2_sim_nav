#!/usr/bin/env python3
"""在仿真里"造障碍"给局部避障验收用。**两种后端，一个脚本**。

为什么需要它：验收"灵活避障"必须能**可控地**把障碍放到路径上，而且每次位置/时序
一致（否则 A/B 没意义）。两种后端对应两种用途：

  ① `--mode gazebo`：在 gz-sim 里真生成一个盒子（真物理、真雷达）
     ⇒ 走完整链路（雷达 → LIO → 感知 → 规划），**这是最终验收口径**。
  ② `--mode cloud`：在 LIO 输出的点云上叠加合成点
     ⇒ 绕开物理与 LIO，**完全确定性、秒级复现**，适合逻辑回归批量跑。
     因为插在 LIO **之后**，所以不影响定位。

⚠ cloud 模式必须让感知改订合成话题（本脚本不劫持原话题，避免自环）：
     ros2 launch perception perception.launch.py cloud_topic:=/perception_test/cloud
   然后本脚本订 `/lightning/perception/cloud`、发 `/perception_test/cloud`。

用法
----
    # S1 静态堵路（放在路径正中）
    python3 obstacle_inject.py --mode gazebo --at 8.0,1.5
    # S2 动态横穿（0.8 m/s 来回）
    python3 obstacle_inject.py --mode gazebo --at 8.0,1.5 --to 8.0,-1.5 \
        --script cross --speed 0.8 --start-delay 5 --duration 60
    # S3 宽箱堵死（1.8 m 宽 ⇒ 绕不过）
    python3 obstacle_inject.py --mode gazebo --sizes 1.8,0.6,0.8 --at 8.0,1.5
    # S4 出现 → 撤走
    python3 obstacle_inject.py --mode gazebo --at 8.0,1.5 --spawn-delay 3 --remove-at 25
    # cloud 模式（先按上面重启感知）
    python3 obstacle_inject.py --mode cloud --at 8.0,1.5 --script line --speed 0.5

三条按感知链反推的约束（改尺寸/位置前先读）
--------------------------------------------
1. **箱子高度必须穿过雷达扫平面**（Go2 上雷达约离地 0.3~0.4 m）：矮箱可能整个看不见，
   默认高 0.8 m；也不要超过感知高度带上沿 2.00 m。
2. **宽/厚 ≥ 0.6 m**：0.10 m 栅格 + `esdf_pub_step=2` 抽点，太薄的障碍会被吃掉。
3. **有效作用半径约 4 m**：感知 `local_update_range = max_ray_length = 4.0`
   ⇒ 4 m 之外的点云不写占据、还会被标成 free。所以障碍真正"被看见"是在车开到 4 m
   以内时 —— 这正是我们要验的工况，别把箱子放到 8 m 外的路上去等它"提前发现"。

本脚本**只造障碍**，不判合格；判定在 `test_avoidance.py` 里。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

# ⚠ 解释器护栏：本仓库的 `.venv` 是 Python 3.14，与 Humble 的 rclpy 不兼容；
#   而用户终端里经常正好激活着它。不拦就会报一堆难懂的扩展模块错误。
try:
    import rclpy
except Exception as _exc:  # noqa: BLE001
    raise SystemExit(
        f"导入 rclpy 失败：{_exc}\n"
        "  ⚠ 本仓库的 .venv 是 Python 3.14，与 Humble 的 rclpy 不兼容\n"
        f"  ⇒ 请用系统解释器：/usr/bin/python3 {__file__}")

from geometry_msgs.msg import Pose, PoseStamped  # noqa: E402
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity

LIVE = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)


def parse_vec(text, n, what):
    parts = [p for p in str(text).replace(" ", "").split(",") if p]
    if len(parts) != n:
        raise SystemExit(f"{what} 需要 {n} 个数（逗号分隔），得到 {text!r}")
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        raise SystemExit(f"{what} 里有非数字：{text!r}")


def box_surface_points(cx, cy, z_lo, size):
    """盒子表面采样点（世界系，轴对齐）。

    只采**表面**：感知走的是"点云 → 3D 占据体素 → 2D 投影"，表面点足以把整块
    体积标成占据（内部点采不到，射线也进不去）。
    沿 x/y 以 ≈0.06 m 采样、z 上取 4 层 —— 层数太少会让 3D 占据出现空洞，
    2D 投影就会出现"漏格"（实测过的假象）。
    """
    sx, sy, sz = size
    pts = []
    nx = max(2, int(round(sx / 0.06)))
    ny = max(2, int(round(sy / 0.06)))
    nz = max(2, int(round(sz / 0.20)))
    for i in range(nx + 1):
        x = cx - sx / 2 + sx * i / nx
        for k in range(nz + 1):
            z = z_lo + sz * k / nz
            pts.append((x, cy - sy / 2, z))
            pts.append((x, cy + sy / 2, z))
    for j in range(ny + 1):
        y = cy - sy / 2 + sy * j / ny
        for k in range(nz + 1):
            z = z_lo + sz * k / nz
            pts.append((cx - sx / 2, y, z))
            pts.append((cx + sx / 2, y, z))
    # 顶面（雷达斜着扫到时能看见）
    for i in range(nx + 1):
        for j in range(ny + 1):
            pts.append((cx - sx / 2 + sx * i / nx,
                        cy - sy / 2 + sy * j / ny, z_lo + sz))
    return pts


class Injector(Node):
    """按时间脚本驱动一个障碍物；两种后端（gazebo / cloud）共用同一套时间逻辑。"""

    def __init__(self, args):
        super().__init__("obstacle_inject")
        self.args = args
        self.t0 = time.time()
        self.spawned = False
        self.removed = False
        self.pose = None          # (x, y)
        self.cloud_seen = 0

        # ★ 把**障碍的真实位姿**也发出来：验收脚本要算"车轨迹到障碍的净距"，
        #   动态场景下必须知道障碍在该采样时刻在哪。自己按脚本重算一遍时间基准
        #   会对不齐（两个进程的 t0 不同）⇒ 干脆由造障碍的人广播真相。
        self.pub_pose = self.create_publisher(
            PoseStamped, args.pose_topic, 10)

        if args.mode == "gazebo":
            self.create_cli = self.create_client(
                SpawnEntity, f"/world/{args.world}/create")
            self.pose_cli = self.create_client(
                SetEntityPose, f"/world/{args.world}/set_pose")
            self.del_cli = self.create_client(
                DeleteEntity, f"/world/{args.world}/remove")
        else:
            self.sub = self.create_subscription(
                PointCloud2, args.in_topic, self.on_cloud, LIVE)
            self.pub = self.create_publisher(PointCloud2, args.out_topic, 1)

    # ------------------------------------------------------------ 时间脚本
    def script_pose(self, t):
        """t = 相对生成时刻的秒数；返回 (x, y)，或 None = 此刻不该存在"""
        a = self.args
        if t < 0:
            return None
        if a.script == "static" or a.to is None:
            return a.at
        if t < a.start_delay:
            return a.at
        u = (t - a.start_delay) * a.speed
        seg = math.dist(a.at, a.to)
        if seg < 1e-6:
            return a.to
        if a.script == "line":
            f = min(1.0, u / seg)
            k = f
        else:  # cross：来回
            k = (u % (2 * seg)) / seg
            if k > 1.0:
                k = 2.0 - k
        return (a.at[0] + (a.to[0] - a.at[0]) * k,
                a.at[1] + (a.to[1] - a.at[1]) * k)

    def due(self):
        t = time.time() - self.t0
        if self.args.remove_at is not None and t >= self.args.remove_at:
            return None
        if t < self.args.spawn_delay:
            return None
        return self.script_pose(t - self.args.spawn_delay)

    # ------------------------------------------------------------ gazebo 后端
    def _sdf(self):
        sx, sy, sz = self.args.sizes
        # ★ --ghost：把 collide_bitmask 置 0 ⇒ 这个箱子**不参与物理碰撞**，
        #   变成一个纯粹的"感知障碍"。用在：动力学干扰盖过了要测的东西
        #   （实测 0.8 m/s 的箱子会把 Go2 推走 ⇒ 之后全局规划直接死锁）。
        #   代价：规划器真出错时车会**从箱子里穿过去** —— 那正是净距指标
        #   （test_avoidance.py）要抓的，所以指标不会因此变好看。
        cmask = "<collide_bitmask>0x00</collide_bitmask>" if self.args.ghost else ""
        return (
            "<sdf version='1.7'><model name='%s'><static>false</static>"
            "<link name='link'>"
            "<collision name='c'><geometry><box><size>%f %f %f</size></box>"
            "</geometry>%s</collision>"
            "<visual name='v'><geometry><box><size>%f %f %f</size></box></geometry>"
            "<material><ambient>1 0.15 0.15 1</ambient>"
            "<diffuse>1 0.3 0.3 1</diffuse></material></visual>"
            "<inertial><mass>5.0</mass></inertial>"
            "</link></model></sdf>"
            % (self.args.name, sx, sy, sz, cmask, sx, sy, sz))

    def gz_spawn(self, x, y):
        # ★ 先删同名模型再生成：上一次验收被 Ctrl-C 打断会**留下箱子**，
        #   于是下一次 SpawnEntity 报"已存在"直接失败、不广播位姿 ⇒ 探针收到
        #   `净距 inf`（2026-09-29 实测 S1 就这么白测了一轮）。幂等化后再也不用
        #   靠人工清世界。
        # ⚠ 这个服务**默认不存在**：必须先把 gz 服务桥到 ROS 侧（实测 Fortress 这套
        #   环境 `ign service --req` 走不通：连 `ign msg -i ign.msgs.Pose` 都
        #   报消息工厂没加载）。`test_avoidance.py --backend gazebo` 会自动起桥；
        #   手工跑本脚本时请自己先起：
        #     ros2 run ros_gz_bridge parameter_bridge \
        #       '/world/world_demo/create@ros_gz_interfaces/srv/SpawnEntity' \
        #       '/world/world_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose' \
        #       '/world/world_demo/remove@ros_gz_interfaces/srv/DeleteEntity'
        if not self.create_cli.wait_for_service(timeout_sec=3.0):
            print(f"  [inject] ⚠ /world/{self.args.world}/create 服务不存在"
                  f" ⇒ 先起服务桥（见本文件/本函数注释），或改用 --mode cloud")
            return False
        self.gz_remove()          # 幂等：先清掉可能残留的同名模型（见函数注释）
        req = SpawnEntity.Request()
        req.entity_factory.name = self.args.name
        req.entity_factory.sdf = self._sdf()
        req.entity_factory.pose = Pose()
        req.entity_factory.pose.position.x = float(x)
        req.entity_factory.pose.position.y = float(y)
        req.entity_factory.pose.position.z = float(self.args.z)
        req.entity_factory.pose.orientation.w = 1.0
        fut = self.create_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = fut.done() and fut.result() is not None and fut.result().success
        print(f"  [inject] 生成 {'成功' if ok else '失败'} ({(x):.2f}, {y:.2f})"
              f" 尺寸 {self.args.sizes}")
        return ok

    def gz_set_pose(self, x, y):
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.args.name, type=Entity.MODEL)
        req.pose = Pose()
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(self.args.z)
        req.pose.orientation.w = 1.0
        self.pose_cli.call_async(req)

    def gz_remove(self):
        req = DeleteEntity.Request()
        req.entity = Entity(name=self.args.name, type=Entity.MODEL)
        fut = self.del_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if self.args.verbose:
            print("  [inject] 已删除障碍")

    # ------------------------------------------------------------ cloud 后端
    def on_cloud(self, m):
        """转发输入点云 + 追加合成盒子的表面点（障碍"不存在"时就是纯转发）"""
        self.cloud_seen += 1
        pts = list(pc2.read_points(m, field_names=("x", "y", "z"),
                                   skip_nans=True))
        if self.pose is not None:
            sx, sy, sz = self.args.sizes
            pts += box_surface_points(self.pose[0], self.pose[1], self.args.z,
                                      (sx, sy, sz))
        out = pc2.create_cloud_xyz32(m.header, pts)
        self.pub.publish(out)

    # ------------------------------------------------------------ 主循环
    def step(self):
        want = self.due()
        if want is None:
            if self.spawned and not self.removed and self.args.remove_at is not None:
                if self.args.mode == "gazebo":
                    self.gz_remove()
                self.removed = True
            self.pose = None
            return
        if not self.spawned:
            if self.args.mode == "gazebo":
                if not self.gz_spawn(*want):
                    self.spawned = False
                    return
            else:
                print(f"  [inject] 开始注入合成障碍 @ ({want[0]:.2f}, {want[1]:.2f})")
            self.spawned = True
        self.pose = want
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(want[0])
        msg.pose.position.y = float(want[1])
        msg.pose.position.z = float(self.args.z)
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)
        if self.args.mode == "gazebo":
            self.gz_set_pose(*want)


def main():
    ap = argparse.ArgumentParser(description="仿真里造障碍（gazebo / cloud 两模式）")
    ap.add_argument("--mode", choices=("gazebo", "cloud"), default="gazebo")
    ap.add_argument("--world", default="world_demo", help="gz world 名（见 world sdf）")
    ap.add_argument("--name", default="dyn_obs", help="gz 里的模型名")
    ap.add_argument("--verbose", action="store_true",
                    help="多打一点（如每次删除模型的提示）；默认安静")
    ap.add_argument("--ghost", action="store_true",
                    help="箱子**不参与物理碰撞**（collide_bitmask=0）：只当感知障碍。"
                         "用在动力学干扰盖过了要测的东西时（慢速下一般不需要）。"
                         "注意：规划器真出错时车会从箱子里穿过去——净距指标仍能抓到")
    ap.add_argument("--at", default="8.0,1.5", help="障碍初始位置 x,y（世界系）")
    ap.add_argument("--to", default=None, help="运动终点 x,y（不给则静止）")
    ap.add_argument("--script", choices=("static", "line", "cross"), default="static",
                    help="line=走一次到 to；cross=来回")
    ap.add_argument("--speed", type=float, default=0.3,
                    help="[m/s] line/cross 用。★ 2026-09-29 从 0.8 降到 0.3："
                         "真实缺陷是**移动的箱子会把车撞飞**（实测把 Go2 推到贴着墙，"
                         "之后全局规划器从那个位姿报 START_FOOTPRINT_COLLISION，"
                         "后面的目标全部规划失败）—— 那是「物体撞车」，不是「车避障」。"
                         "0.3 m/s ≈ 慢走的人/慢速搬运车，也还能被局部图跟上"
                         "（~5 Hz 每拍只动 6 cm）")
    ap.add_argument("--sizes", default="0.6,0.6,0.8", help="盒子 sx,sy,sz [m]")
    ap.add_argument("--z", type=float, default=0.40,
                    help="盒子**底面**高度 [m]（0.4 —— 见文件头的扫平面说明）")
    ap.add_argument("--spawn-delay", type=float, default=0.0,
                    help="[s] 相对脚本启动，多久之后障碍才出现")
    ap.add_argument("--pose-topic", default="/perception_test/obstacle_pose",
                    help="广播障碍真实位姿（验收脚本算净距用）")
    ap.add_argument("--start-delay", type=float, default=0.0,
                    help="[s] 出现后多久开始按脚本运动")
    ap.add_argument("--remove-at", type=float, default=None,
                    help="[s] 相对脚本启动，多久之后撤走障碍（撤走=删除/停止注入）")
    ap.add_argument("--duration", type=float, default=1e9, help="[s] 脚本总时长")
    ap.add_argument("--rate", type=float, default=20.0, help="[Hz] 位姿更新率")
    ap.add_argument("--in-topic", default="/lightning/perception/cloud")
    ap.add_argument("--out-topic", default="/perception_test/cloud")
    args = ap.parse_args()

    a = parse_vec(args.at, 2, "--at")
    t = parse_vec(args.to, 2, "--to") if args.to else None
    s = parse_vec(args.sizes, 3, "--sizes")
    args.at, args.to, args.sizes = a, t, s

    rclpy.init()
    node = Injector(args)
    print(f"  [inject] mode={args.mode} sizes={s} at={a} to={t} "
          f"script={args.script} speed={args.speed}")
    if args.mode == "cloud":
        print(f"  [inject] 转发 {args.in_topic} → {args.out_topic}"
              f"（感知要 cloud_topic:={args.out_topic}）")
        # 先确认上游点云在流动（否则"没效果"会被误判成规划问题）
        end = time.time() + 5.0
        while time.time() < end and node.cloud_seen == 0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.cloud_seen == 0:
            print("  [inject] ⚠ 5 s 内没收到输入点云：上游没起？QoS 不匹配？")
    period = 1.0 / max(args.rate, 1.0)
    try:
        while rclpy.ok() and time.time() - node.t0 < args.duration:
            node.step()
            rclpy.spin_once(node, timeout_sec=period)
    except KeyboardInterrupt:
        pass
    finally:
        if args.mode == "gazebo" and node.spawned and not node.removed:
            node.gz_remove()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
