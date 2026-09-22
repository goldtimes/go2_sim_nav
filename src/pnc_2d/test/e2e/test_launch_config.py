#!/usr/bin/env python3
"""E2E-4：**launch + yaml 合并**是否按设计干活（配置文件拆分的回归）。

为什么单独一个脚本：其它 E2E 都用 `-p key:=value` 在命令行塞参数，**绕过了 yaml 与
launch**。而"参数放哪、谁覆盖谁"正是配置文件拆分成
`pnc_2d.yaml(总入口) + local_*.yaml + global_*.yaml(算法片段) + extra_config` 之后
最容易出错的地方，必须单独验证。

断言：
  [1] planner_type:=astar            → 类型、A* 私有参数、公共参数都到位
  [2] planner_type:=route_network    → 换片段后类型与路网私有参数到位
  [3] extra_config 覆盖片段           → 证明"后加载的胜出"这条加载顺序真的成立
  [4] planner_type:=不存在的类型      → launch 明确报错，而不是静默用别的算法

用法：python3 test_launch_config.py
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

NODE = "/global_planner"          # launch 文件里的固定节点名
WS = os.environ.get("PNC2D_WS") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def launch(planner_type: str, extra_config: str = ""):
    """用真正的 launch 文件起节点（这才是我们要验证的东西）"""
    extra = f" extra_config:={extra_config}" if extra_config else ""
    cmd = ("ros2 launch pnc_2d global_planner.launch.py "
           f"planner_type:={planner_type}{extra}")
    log = tempfile.NamedTemporaryFile(delete=False, suffix=".log")
    proc = subprocess.Popen(["bash", "-lc", cmd], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    return proc, log.name


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=5)


class ParamClient(Node):
    """只问参数：launch 起的节点话题没重映射，我们不需要跟它传数据"""

    def __init__(self):
        super().__init__("pnc2d_launch_probe")

    def wait_and_get(self, names, timeout=40.0):
        """等节点出现 → 调 get_parameters。

        ⚠ 必须重试 + **每轮重建客户端**：上一个用例的同名节点刚被杀时，DDS 里它的
        服务记录还会残留几秒，`wait_for_service` 会立刻成功但异步调用石沉大海——
        日志看起来像"节点没起来"，实际是消息发给了一个已经死掉的端点。
        """
        deadline = time.time() + timeout
        attempt = 0
        while time.time() < deadline:
            attempt += 1
            cli = self.create_client(GetParameters, f"{NODE}/get_parameters")
            if cli.wait_for_service(timeout_sec=2.0):
                req = GetParameters.Request()
                req.names = list(names)
                fut = cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.result() is not None:
                    # ParameterType：0=NOT_SET(未声明) 1=BOOL 2=INT 3=DOUBLE 4=STRING
                    return {name: {
                        1: v.bool_value,
                        2: v.integer_value,
                        3: v.double_value,
                        4: v.string_value,
                    }.get(v.type)
                            for name, v in zip(names, fut.result().values)}
            self.destroy_client(cli)
            time.sleep(1.0)
        self.get_logger().error(f"重试 {attempt} 次仍拿不到参数")
        return None


CHECKS = {"pass": 0, "fail": 0}


def chk(desc, cond, detail=""):
    if cond:
        CHECKS["pass"] += 1
        print(f"  PASS {desc}")
    else:
        CHECKS["fail"] += 1
        print(f"  FAIL {desc}" + (f"   ← {detail}" if detail else ""))


def case(planner_type, expect, extra_config="", expect_launch_fail=False,
         topics_expect=()):
    print(f"[{planner_type}"
          + (f" + extra_config" if extra_config else "") + "]")
    proc, log = launch(planner_type, extra_config)
    try:
        if expect_launch_fail:
            # 起不来才对；等它自己退出
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                pass
            text = open(log, encoding="utf-8", errors="replace").read()
            chk("launch 明确失败（没有静默回退到别的算法）",
                proc.poll() is not None and proc.returncode != 0,
                f"returncode={proc.returncode}")
            chk("错误信息里能看出是哪个类型不认识 / 缺哪个片段",
                ("planner_type" in text or "找不到参数片段" in text
                 or "nonsense" in text),
                text.strip().splitlines()[-1] if text.strip() else "(无输出)")
            return

        rclpy.init()
        probe = ParamClient()
        got = probe.wait_and_get(list(expect.keys()))
        # 顺带验证**真正解析出来的**话题名：参数里写的是相对名（pnc_2d/...），
        # 运行时应在根命名空间下解析成 /pnc_2d/...。这比断言参数串更接近事实。
        if topics_expect:
            deadline = time.time() + 20
            names: set = set()
            while time.time() < deadline:
                names = {n for n, _ in probe.get_topic_names_and_types()}
                if set(topics_expect) <= names:
                    break
                time.sleep(1.0)
            for t in sorted(topics_expect):
                chk(f"话题 {t} 真的存在（相对名解析正确）", t in names,
                    "实际含 pnc 的话题：%s" % sorted(n for n in names if "pnc" in n))
        probe.destroy_node()
        rclpy.shutdown()

        if got is None:
            chk("节点起来了且参数服务可用", False,
                open(log, encoding="utf-8", errors="replace").read()[-500:])
            return
        chk("节点起来了且参数服务可用", True)
        for k, want in expect.items():
            chk(f"{k} == {want!r}", got[k] == want, f"实际 {got[k]!r}")
    finally:
        stop(proc)
        # 给 DDS 一点时间清掉刚死掉节点的服务/话题记录，否则下个用例会被残影干扰
        time.sleep(3.0)


def main():
    # ⚠ 不 pkill 别人的节点：本仓库的真实栈可能正在后台跑（E2E-5 就是被这个坑过）。
    # 这里只看**参数**，起冲突的风险由"跑完就 killpg 收掉自己的 launch"控制。
    # （旧的 pkill -f global_planner_node 会连用户正在跑的节点一起杀掉，已移除。）

    # [1] A* 片段：算法私有参数 + 总入口的公共参数都要在
    case("astar", {
        "planner.type": "astar",
        "astar.path_prune_mode": "line_of_sight",   # 片段私有
        "footprint.length": 0.70,                   # 总入口的公共参数
        "topics.status": "pnc_2d/global_status",    # 自家话题：写的是相对名
    }, topics_expect=("/pnc_2d/global_status", "/pnc_2d/global_path"))

    # [2] 路网片段：换片段后私有参数换成路网那套（证明路由真的按 type 走）
    case("route_network", {
        "planner.type": "route_network",
        "route_network.goal_mode": "hybrid",        # 片段私有
        "route_network.reject_infeasible": True,
        "footprint.length": 0.70,                   # 公共参数依然生效
    })

    # [3] 加载顺序：extra_config 在片段之后，必须赢
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("global_planner:\n  ros__parameters:\n"
                "    astar.path_prune_mode: \"none\"\n")
        extra = f.name
    case("astar", {"planner.type": "astar",
                   "astar.path_prune_mode": "none"},   # ← 被 extra_config 覆盖
         extra_config=extra)
    os.unlink(extra)

    # [4] 类型拼错 / 不存在：launch 要明确报错
    case("nonsense", {}, expect_launch_fail=True)

    print()
    print(f"汇总：{CHECKS['pass']} 通过 / {CHECKS['fail']} 失败")
    return 1 if CHECKS["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
