# pnc_2d 端到端测试（ROS 图级别）

这里跑的是**真节点 + 真 DDS** 的 E2E，验证那些纯 C++ 单测覆盖不到的东西：
latched 话题的"路径有效期"、清场时机、换算法重启、运行时热切换、**launch 与多份 yaml 的合并顺序**。

| 脚本 | 覆盖 |
|---|---|
| `test_clear_semantics.py` | 失败→清空+NO_PATH 状态、成功→路径+SUCCESS 状态、到达目标（按参数开关）、`~/clear_path` 服务 |
| `test_restart_cleanup.py` | 杀掉路网节点→用 A* 重启：旧路径/黄色通路高亮**必须被清掉**（跨进程残留） |
| `test_hot_switch.py` | `ros2 param set planner.type` 真热切换、非法类型被拒、`~/switch_planner`、`~/reload_params` |
| `test_launch_config.py` | **配置拆分回归**：按 `planner_type` 路由到哪个片段、公共参数是否落地、`extra_config` 覆盖顺序、类型写错必须报错、相对名解析 |
| `test_three_node_smoke.py` | **三节点空跑（P4 验收）**：`Idle→Planning→Following→GoalReached` 全链路、不可达 → `FAILED` + 原因、`~/cancel`，以及**绝不能有 `/pnc_2d/cmd_vel` 发布者**（Null 局部不发速度） |
| `test_mpc_wiring.py` | **MPC 接线（P5.3 验收）**：无数据→降级限速、空点云≠缺距离场、真发 cmd_vel、**闭环**前进、墙真的进规划器、`route` 走廊一路传到局部、`none` 反例 |
| `run_all.sh` | 上面六个 + E2E-1 的两种参数模式，一把跑完 |

> ⚠ `test_mpc_wiring.py` 的 G 阶段（走廊接线）**必须让车真的开起来**（`probe.drive=True`
> 且给图/给场），并等它回到走廊里：P5.4 起走廊是"**车已在走廊里**才生效"（从车道外
> 用硬约束收敛实测不可行 ⇒ 先自由上线、再严格贴线）。车钉在原地不动时，
> `route_mode` 永远为 false —— 那条断言在旧语义下能过、在新语义下必然失败（已修）。

> 为什么要有 `test_launch_config.py`：其它脚本都用 `-p key:=value` 在命令行塞参数，
> **绕过了 yaml 与 launch**。而"参数放哪、谁覆盖谁"正是 `pnc_2d.yaml(总入口) +
> local_*.yaml + global_*.yaml(算法片段) + extra_config` 拆分后最容易错的地方。

## 怎么跑

```bash
cd /home/gmd/r41_ws
colcon build --packages-select pnc_2d      # 需要先构建（脚本直接跑 install 里的可执行文件）
bash src/pnc_2d/test/e2e/run_all.sh        # 全部
# 或单个（自己设 DDS 环境）
export CYCLONEDDS_URI=$PWD/src/bringup/cyclonedds.xml ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
python3 src/pnc_2d/test/e2e/test_hot_switch.py
```

要点：

- **不需要 `map_server` 在跑**：脚本自己从站点目录的 `map.pgm`/`map.yaml` 造 `OccupancyGrid`
  并以 latched 发到 `/t/map`，语义与 map_server 一致（未知格按 `unknown_as_free` 当空闲）。
- **不干扰正在跑的系统**：所有话题都重映射到 `/t/*`，节点名固定 `gp_e2e`，跑完自己收掉。
- 需要 **`python3` 里有 `rclpy`**（即 source 过 ROS 环境），所以用系统的 `python3`，不是仓库的 `.venv`。
- 站点/路网可用环境变量覆盖：`PNC2D_MAP_DIR`、`PNC2D_ROUTES`、`PNC2D_WS`。
- 节点日志写在 `/tmp/pnc2d_e2e_<pid>.log`；失败时先看它。

这些**不进 `colcon test`**（会起节点进程、依赖 DDS 环境，不适合放进常规单测）；
纯算法单测仍在 `test/*.cpp`（`colcon test --packages-select pnc_2d`）。

## 写这类测试踩过的五个坑（都已在公共模块/脚本里处理）

1. **别用 `ros2 run` 起被测节点**：它是个 wrapper，`Popen.terminate()` 只杀 wrapper，
   真正的节点会变成**孤儿进程**继续跑。多个实例抢同一话题 → 出现"重复消息"、
   "别人发的空路径把路径清了"这类灵异现象。
   `e2e_common.start_node()` 直接 exec `install/pnc_2d/lib/pnc_2d/global_planner_node`，
   并用 `start_new_session=True` + `killpg` 整组回收。
2. **判据要扫"整窗消息"，不能只看最后一条**：节点一次操作会发多条 `MarkerArray`
   （清场 DELETE、路网、起终点…），只看最后一条会把状态判反。
   `Harness.saw_delete()/saw_add()` 就是干这个的。
3. **同名节点连着起停时，DDS 里有"服务残影"**：上一个用例的 `/global_planner` 刚被
   杀掉，它的服务记录还会残留几秒；此时 `wait_for_service()` **立刻成功**，但异步调用
   石沉大海 —— 日志看起来像"节点没起来"，实际节点早就在正常打日志了。
   `test_launch_config.py` 的 `ParamClient.wait_and_get()` 因此做成
   **重试 + 每轮重建客户端**，并在用例之间等几秒。
4. **★ 别在测试里 `pkill -f <节点名>`**：本仓库的真实栈（`lightning` 定位、`map_server`、
   旧的 `global_planner_node`）**可能正在后台跑**。pkill 会把用户正在用的节点一起杀掉。
   E2E-5 的做法是：`ns:=/e2e` + `extra_config` 把**外部 IO** 指到 `/t/*`，
   自家话题/IPC 因为配置里写的是**相对名**会跟着命名空间搬走，跑完只 `killpg` 自己的 launch。
5. **★ 测试生成的 params 文件段落也要写 `/**:`**：用 `ns:=` 时，按节点名写的段落
   **完全不匹配**（FQN 变成 `/e2e/global_planner`），参数静默失效。
   E2E-5 第一版写的就是 `global_planner:`/`pnc_manager:` 三个段，结果三个节点全都还在
   听真实话题名 → goal 收不到、测试全红；换成单个 `/**` 段就好了。

## 还有两个“不是测试问题”的问题（是它帮我们抓出来的）

- **两个定位源同时喂位姿** → 管理器每帧都判为"跳变"，每次跳变又触发一次重规划
  （实测：1 秒内几百条 WARN）。修法：给跳变加**冷却**（`sm.odom_jump_cooldown`）。
- **`NullLocalPlanner` 永远不报"到达"**（它只回报状态、不做控制），导致 action 永不结束、
  状态机永远停在 `FOLLOWING`。修法：把“到位判定”放在**节点层**兜底（`remaining() <= goal_tolerance`），
  不依赖算法自报 —— 否则任何局部算法忘了报，任务就永远不结束。

## E2E-6（MPC 接线）额外注意的坑

1. **测试里的"车"必须真的动**：管理器有**卡住判据**（默认 10 s 内位移 < 0.2 m → 取消
   跟随）。位姿一直不动的静态测试，任务会被管理器自己取消，后续阶段根本跑不到（第一版
   测试全红就是这个原因）。两招并用：
   · `sm.stuck_timeout: 120` 放宽（降级/空转阶段本来就是故意不动的）；
   · 主要阶段按收到的 `cmd_vel` 做**差速运动学积分**闭环（`x+=v cosθ dt` …），
     顺带把 `setCurrentVelocity`（底盘速度回给 MPC）那条接线也走到。
2. **点云布局不能硬编码**：真实 `grid_map/esdf_2d` 是 PCL `PointXYZI`，实测
   `point_step=32`、`intensity` 在偏移 **16**（不是 12）。所以测试**刻意用真实布局**造
   点云，这样将来有人把解析改成硬编码偏移会被抓住；节点侧则用
   `PointCloud2ConstIterator<float>(msg, "intensity")` 按**字段名**取（与偏移无关）。
3. **空点云是有意义的输入**：感知只发 `0<d≤3 m` 的格，空旷处点云为空。测试专门开一个
   阶段发空点云，断言它是 `FOLLOWING` 而不是 `DEGRADED`。
4. **BLOCKED 会在 1 s 后结束跟随**：想看 `BLOCKED` 状态，取样窗口必须**立即**开始
   （第一版先 `win(1.5)` 再取样，结果只能看到它已经收工后的空窗口）。
5. **需要观测的日志用节流而不是只打一次**：距离场重建日志用
   `INFO_THROTTLE(5000)`，测试才能从日志里验证"到底解析出多少个采样点"。
