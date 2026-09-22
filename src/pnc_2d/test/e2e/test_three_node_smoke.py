#!/usr/bin/env python3
"""E2E-5：三节点空跑（P4 验收）。

验收标准（doc/pnc2d_restructure_plan.md §6 P4）：
  点一个目标 → /pnc_2d/state 上应看到 Idle → Planning → Following → GoalReached，
  **且不发任何速度指令**（/pnc_2d/cmd_vel 上连发布者都不该有）。

为什么必须端到端测：状态机单测只验证"表",验证不了
  · `~/plan_path` 服务与 `~/follow_path` action 的名字/类型是否对得上；
  · 异步回调（规划响应、action 结果）是否真的把事件喂回了状态机；
  · 结果判读顺序（先看语义标志 canceled/goal_reached/blocked，再看 action code）；
  · cmd_vel 到底有没有被谁发出来。

做法：自带地图 + 位姿，**自己充当 local 节点的"假底盘"**？不 —— 这里刻意起真的
三节点（global + local + manager），只是把地图与位姿喂给它们，然后观察：
  1. /pnc_2d/state 的转移序列；
  2. /pnc_2d/global_path 是否出现非空路径；
  3. /pnc_2d/local_status 是否为 FOLLOWING；
  4. **不能**出现 /pnc_2d/cmd_vel 发布者（Null 局部不发速度）；
  5. 车"走到"目标附近后（我们手动把位姿挪过去）→ GOAL_REACHED。
     ⚠ 因为局部是空实现，实车不会动，所以由**测试**把位姿推到目标点来触发到达，
     这一步验证的是"状态机能正确结束任务"，不是"车会走"。

用法：python3 test_three_node_smoke.py
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)

from pnc_2d.msg import LocalStatus, ManagerState

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from e2e_common import load_map_msg, MAP_DIR  # noqa: E402

WS = os.environ.get("PNC2D_WS") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


NS = "/e2e"          # 本测试全部节点跑在 /e2e 下，与真实系统错开
T = "/t"            # 外部 IO（地图/位姿/目标）重定向到这里


def _isolate_config():
    """写一份 extra_config：只把**外部** IO 指到 /t/*。

    为什么需要：本仓库的真实定位（lightning run_loc_online）可能正在后台跑并发布
    `/lightning/perception/pose`。测试如果直接用真实话题名，自己的假位姿会与真实位姿
    **交替**出现 → 管理器看到的每帧都是"跳变"（真的踩过，刷了几百条 WARN）。
    自家话题（路径/状态/cmd_vel）与 IPC 名字在配置里写的是**相对名**，
    配合 ns:=/e2e 会自动搬到 /e2e/*，不需要在这里覆盖。

    ⚠ 段落必须写 `/**`：本文件用于 `ns:=/e2e` 的节点，而有命名空间时
      **按节点名写的段落完全不匹配**（FQN 是 /e2e/global_planner）。
      这个坑我在这儿也踩了一次 —— 第一版写的就是节点名，结果三个节点全都还在用
      真实话题名，goal 收不到、测试全红。
      另外 `/**` 在一个文件里只能出现一次，所以三组参数合并写在一起。
    """
    doc = {"/**": {"ros__parameters": {
        "topics.map": f"{T}/map",
        "topics.odom": f"{T}/odom",      # 三个节点共用（键相同，值相同）
        "topics.goal": f"{T}/goal",      # 只对 manager 有意义
    }}}
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(doc, f, allow_unicode=True)
    f.close()
    return f.name


def launch_three(planner_type="astar"):
    """用真正的 pnc_2d.launch.py 起三节点（顺便验证 launch 本身可用）"""
    extra = _isolate_config()
    cmd = ("ros2 launch pnc_2d pnc_2d.launch.py "
           f"planner_type:={planner_type} ns:={NS} extra_config:={extra}")
    log = tempfile.NamedTemporaryFile(delete=False, suffix=".log")
    proc = subprocess.Popen(["bash", "-lc", cmd], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log.name


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=8)
            return
        except subprocess.TimeoutExpired:
            continue
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


CHECKS = {"pass": 0, "fail": 0}


def chk(desc, cond, detail=""):
    if cond:
        CHECKS["pass"] += 1
        print(f"  PASS {desc}")
    else:
        CHECKS["fail"] += 1
        print(f"  FAIL {desc}" + (f"   ← {detail}" if detail else ""))


class Probe(Node):
    def __init__(self, start):
        super().__init__("pnc2d_p4_probe")
        self.states: list = []
        self.paths: list = []
        self.local: list = []
        self.cmd_vel: list = []
        self.pose = start

        self.create_subscription(ManagerState, f"{NS}/pnc_2d/state",
                                 self.states.append, LATCHED)
        self.create_subscription(Path, f"{NS}/pnc_2d/global_path",
                                 self.paths.append, LATCHED)
        self.create_subscription(LocalStatus, f"{NS}/pnc_2d/local_status",
                                 self.local.append, LATCHED)
        self.create_subscription(Twist, f"{NS}/pnc_2d/cmd_vel", self.cmd_vel.append, 10)

        self.pub_map = self.create_publisher(OccupancyGrid, f"{T}/map", LATCHED)
        self.pub_odom = self.create_publisher(Odometry, f"{T}/odom", BEST_EFFORT)
        self.pub_goal = self.create_publisher(PoseStamped, f"{T}/goal", 1)
        self.create_timer(0.05, self._tick)

    def _tick(self):
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = "map"
        o.pose.pose.position.x = self.pose[0]
        o.pose.pose.position.y = self.pose[1]
        o.pose.pose.orientation.z = math.sin(0.5 * self.pose[2])
        o.pose.pose.orientation.w = math.cos(0.5 * self.pose[2])
        self.pub_odom.publish(o)

    def send_goal(self, x, y, yaw=0.0):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = "map"
        g.pose.position.x = x
        g.pose.position.y = y
        g.pose.orientation.z = math.sin(0.5 * yaw)
        g.pose.orientation.w = math.cos(0.5 * yaw)
        self.pub_goal.publish(g)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def state_names(self):
        return [s.state_name for s in self.states]

    def saw_state(self, name):
        return name in self.state_names()


def main():
    # ⚠ 不 pkill 别人的节点：本仓库的真实栈（global_planner/定位）可能正在跑。
    # 隔离靠 ns + extra_config（见上），不靠杀进程。
    proc, log = launch_three("astar")
    rclpy.init()
    probe = Probe(start=(0.0, 0.03, 0.0))
    try:
        # 等三个节点起来（launch + 各自初始化）
        probe.spin(6.0)

        names = [n for n in probe.get_node_names() if n in
                 ("global_planner", "local_planner", "pnc_manager")]
        chk("三个节点都起来了（在 %s 下）" % NS, len(names) == 3, f"看到 {sorted(names)}")

        probe.pub_map.publish(load_map_msg(MAP_DIR))
        probe.spin(2.0)

        chk("[1] 起始状态是 IDLE", probe.saw_state("IDLE"),
            f"收到的状态 {probe.state_names()}")

        # ---- 点一个目标：应在可通行区域内（地图 30.4x15.4 m，原点 -8.07,-6.65）
        gx, gy = 4.0, 0.0
        probe.send_goal(gx, gy)
        probe.spin(8.0)

        seq = probe.state_names()
        print(f"       状态序列：{' → '.join(seq)}")
        chk("[2] 收到目标进入 PLANNING", "PLANNING" in seq, f"{seq}")
        chk("[3] 规划成功进入 FOLLOWING", "FOLLOWING" in seq, f"{seq}")

        ok_paths = [p for p in probe.paths if p.poses]
        chk("[4] /pnc_2d/global_path 有非空路径", bool(ok_paths),
            f"共收到 {len(probe.paths)} 条（非空 {len(ok_paths)}）")
        if ok_paths:
            print(f"       路径 {len(ok_paths[-1].poses)} 点")

        local_names = [m.status_name for m in probe.local]
        print(f"       局部状态：{local_names[-3:] if local_names else '(无)'}")
        chk("[5] 局部报 FOLLOWING", "FOLLOWING" in local_names, f"{local_names}")

        # ---- ★ 反例：Null 局部绝不能产生速度
        pubs = probe.count_publishers(f"{NS}/pnc_2d/cmd_vel")
        chk("[6] /pnc_2d/cmd_vel 上没有发布者（Null 局部不发速度）", pubs == 0,
            f"发现 {pubs} 个发布者")
        chk("[7] 也确实没收到任何 cmd_vel 消息", not probe.cmd_vel,
            f"收到 {len(probe.cmd_vel)} 条")

        # ---- 把"车"推到目标附近 → 局部报到达 → 状态机 GOAL_REACHED
        probe.pose = (gx - 0.1, gy, 0.0)
        probe.spin(6.0)
        seq = probe.state_names()
        print(f"       状态序列：{' → '.join(seq)}")
        chk("[8] 到达目标进入 GOAL_REACHED", "GOAL_REACHED" in seq, f"{seq}")

        last = probe.states[-1] if probe.states else None
        if last:
            chk("[9] GOAL_REACHED 带上了目标坐标",
                abs(last.goal_x - gx) < 1e-6 and abs(last.goal_y - gy) < 1e-6,
                f"汇报目标 ({last.goal_x}, {last.goal_y})")
            # 允许 >=1：定位跳变/卡住都会合法地追加规划请求
            chk("[10] 统计里记录了规划请求", last.plan_requests >= 1,
                f"plan_requests={last.plan_requests}")

        # ---- 不可达目标 → FAILED + 状态里带原因
        probe.states.clear()
        probe.send_goal(100.0, 100.0)      # 地图外
        probe.spin(8.0)
        seq = probe.state_names()
        print(f"       状态序列：{' → '.join(seq)}")
        chk("[11] 不可达目标进入 FAILED", "FAILED" in seq, f"{seq}")
        last = probe.states[-1] if probe.states else None
        if last:
            chk("[12] FAILED 带上了原因", bool(last.message), f"message='{last.message}'")

        # ---- 取消服务
        probe.states.clear()
        plog = []
        from std_srvs.srv import Trigger
        cli = probe.create_client(Trigger, f"{NS}/pnc_manager/cancel")
        chk("[13] ~/cancel 服务可用", cli.wait_for_service(timeout_sec=5.0))
        if cli.service_is_ready():
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(probe, fut, timeout_sec=5.0)
            res = fut.result()
            chk("[14] 取消返回成功且回到 IDLE",
                res is not None and res.success and probe.saw_state("IDLE"),
                (res.message if res else "无响应") + f" | {probe.state_names()}")
            plog.append(res)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        stop(proc)

    print()
    print(f"汇总：{CHECKS['pass']} 通过 / {CHECKS['fail']} 失败")
    if CHECKS["fail"]:
        print(f"--- launch 日志尾部（{log}） ---")
        print("".join(open(log, encoding="utf-8", errors="replace").readlines()[-40:]))
    return 1 if CHECKS["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
