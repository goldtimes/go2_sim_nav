# pnc_2d —— 二维规划（全局规划 + 路网 + 区域层）

可插拔的 2D 全局规划包：**ROS-free 规划库 + 薄 ROS 节点**。
当前进度与后续阶段见 `doc/pnc2d_restructure_plan.md`（P1 目录分层 ✅、P2 路网 ✅）。
架构与算法细节见 `doc/pnc2d_dev_plan.md`，MPC/NMPC 原理见 `doc/mpc_nmpc_guide.md`。

```
include/pnc_2d/core/     types / cost_map_2d / clearance_field / footprint_collision
                         route_graph（路网）/ map_zones（禁行·限速区）/ global_planner（抽象基类）/ factory
include/pnc_2d/global/   astar_planner（自由空间 A*）/ route_network_planner（沿路网走）
src/nodes/               global_planner_node（ROS 薄壳：只做 IO/参数/frame 校验）
scripts/                 MPC/NMPC 离线原型（本包的算法验证工具）
test/                    无 ROS 单测（A* / 路网 / 区域层，共 32 个用例）
```

> **画线器在 `map_server`**：`src/map_server/scripts/route_editor.py`
> （`routes.yaml` 是站点资产，加载/校验/发布/编辑都归 map_server；本包只消费）。
> 操作手册见 `src/map_server/README.md`。

---

## 1. 快速开始

```bash
# 0) 每个新终端都要先设 DDS 环境（本仓库约定）
export CYCLONEDDS_URI=/home/gmd/r41_ws/src/bringup/cyclonedds.xml \
       ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/r41_ws && colcon build --packages-select pnc_2d --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 1) 自由空间 A*（默认）
ros2 launch pnc_2d global_planner.launch.py

# 2) 沿路网走
ros2 launch pnc_2d global_planner.launch.py planner_type:=route_network

# 3) 单测（不需要 ROS 环境）
colcon test --packages-select pnc_2d --event-handlers console_direct+
```

前置输入（本节点不做 TF 变换，三者必须已在同一坐标系）：

| 话题 | 类型 | 来源 |
|---|---|---|
| `/global_map/occupancy` | `nav_msgs/OccupancyGrid`（latched） | `map_server` |
| `/lightning/perception/pose` | `nav_msgs/Odometry`（**best_effort**！） | `lightning` 定位 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz "2D Goal Pose" |

输出：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/pnc_2d/global_path` | `nav_msgs/Path`（**latched**） | 当前路径。**没有有效路径时发空 Path**（见下"路径有效期"） |
| `/pnc_2d/plan_markers` | `MarkerArray`（latched） | 起终点箭头 + 起点车体轮廓 + **当前通路高亮**（路网模式） |
| `/pnc_2d/global_status` | `pnc_2d/PlannerStatus`（latched） | 每次规划的状态（`status`/`status_name`/`message` + 点数/长度/耗时/扩展节点）——**给状态机与上层错误上报用** |
| `/global_planner/clear_path` | `std_srvs/Trigger` | 显式清空当前路径与规划标记 |

**路径有效期（latched 话题的三条规则）**

1. **失败 → 自动清空**：发一条空 `Path`（否则 RViz / 上层会继续显示上一条已过期的成功路径，把"规划失败"误当成"路径有效"）；
2. **到达目标 → 自动清空（默认关）**：`clear.auto_on_goal_reached: false`、容差 `clear.goal_tolerance: 0.30` m。"任务是否结束"由状态机判断（P4），规划器不替它决定；打开后会在车进入容差内时清空，且只在"规划时车离目标超过容差"时生效（避免目标就在车边时刚发就清）；
3. **重启节点 → 启动即清场**：新进程启动时先发一次空路径 + 对残留的 `route_active` 高亮发 `DELETE`（id 0..63）。因为**旧进程发过几条高亮新进程并不知道**，不清就会出现"杀掉路网节点、换成 A* 重启，RViz 里路网路径还挂着"。
   ⚠ 这个清场会**重复发几拍**（0.5 s × 6，一旦有规划结果就停）：RViz 的 MarkerArray 订阅是 **VOLATILE**，只收"匹配完成之后"发布的消息，启动瞬间只发一次会早于 DDS discovery 被丢掉。

> **换算法不用重启（支持热切换）**：`planner.type` 可以在运行时改，节点会真的重建规划器；
> 详见下面"运行时热切换与热重载"。

**运行时热切换与热重载**

| 接口 | 用法 | 说明 |
|---|---|---|
| 参数 | `ros2 param set /global_planner planner.type route_network` | 立即重建规划器；**失败会拒绝这次参数修改**（`successful=false` + 原因），保证"参数值 = 实际生效值" |
| 服务 | `ros2 service call /global_planner/switch_planner pnc_2d/srv/SwitchPlanner "{type: 'astar'}"` | 显式命令语义，返回 `success + message`；P4 状态机切模式用它 |
| 服务 | `ros2 service call /global_planner/reload_params std_srvs/srv/Trigger` | 用**当前参数**重新装载**同类**算法：改了 `footprint.*` / `path_prune_*` / `route_network.routes_file` 不用重启 |
| 服务 | `ros2 service call /global_planner/clear_path std_srvs/srv/Trigger` | 清空当前路径与规划标记 |

热切换的语义（实测见下）：

- 新算法按**当前**参数值 configure（yaml 值 + 运行时 `param set` 过的值都算）；
- 若已收到地图，立刻把它交给新算法 → **路网模式会马上重跑可行性校验**并打印"过不去的通道"；
- 切换/重载成功后**清空当前路径与规划标记**（旧算法算出的路径对新算法无意义）；
- 构建或 configure 失败时**保留旧算法**，不会把节点搞成"没规划器"状态；
- 线程前提：节点用默认**单线程** executor，goal / 参数 / 服务回调互相串行，不存在"规划到一半换掉 planner_"。若改用 `MultiThreadedExecutor`，`switchPlanner()` 与 `planner_` 的访问都要加锁（代码里有注释标记）。

实测（隔离实例，`go2_sim_factory`）：

```
[1] astar         → 路径与通道最大偏离 5.00 m（直连，不贴路网）
[2] param set planner.type=route_network → 接受；切换瞬间路径被清空
[3] 同一目标重规划 → 中间点与通道偏离 < 0.1 m（贴线走），起点离网那段仍偏离 5 m（hybrid 补段）
[4] param set planner.type=nonsense → 被拒绝：热切换失败：未知 planner.type='nonsense'
    （可用：astar, route_network）（仍在使用 route_network）；参数值确认未变
[5] 服务 switch_planner：astar → success；nonsense → success=false + 原因
[6] param set footprint.length=0.99 + reload_params → 日志 footprint 开: 0.99x0.40（外接半径 0.600）
```

---

## 2. 路网与区域层

### 2.1 文件位置

```text
~/rcs/maps/<站点>/
├── map.yaml + map.pgm        # 2D 栅格（map_server 发布）
├── routes.yaml               # ★路网 + 区域层（本文件由画线器生成，可选）
└── global.pcd                # 3D 点云（未实现）
```

**可选资产**：站点没有 `routes.yaml` 时，`map_server` 只打一句 INFO，不报错；
路网模式下规划器会返回 `NOT_INITIALIZED` 并说明原因。

### 2.2 格式

```yaml
frame_id: map
nodes:
  - {name: P1, x: 6.550, y: -4.920, type: waypoint}
  - {name: C1, x: 8.200, y: 2.050,  type: charge}
edges:
  - {from: P1, to: P2, speed_limit: 1.00, corridor_width: 0.50,
     polyline: [[6.550,-4.920], [13.850,-4.920], [21.500,-5.670]]}
zones:                                            # 区域层（可选）
  - {name: Z1, type: forbidden,   polygon: [[10,-6], [12,-6], [12,-4], [10,-4]]}
  - {name: Z2, type: speed_limit, value: 0.30, polygon: [[2,1], [4,1], [4,3], [2,3]]}
```

| 字段 | 默认 | 说明 |
|---|---|---|
| `nodes[].type` | `waypoint` | `waypoint` / `station` / `charge` / `park`（只影响 RViz 配色与标签；语义站点，目标仍按坐标给） |
| `edges[].one_way` | **`false`（双向）** | 本仓库路网**默认全部双向**，没有单向通道；需要单向才写 `one_way: true` |
| `edges[].speed_limit` | `1.0` | 参与路由代价（代价 = 弧长 / speed_limit） |
| `edges[].corridor_width` | `0.6` | 允许横向偏离的半宽 [m]；**`0.0` = 严格贴线，遇障即停**（P5 局部规划消费） |
| `edges[].polyline` | 必填 | 折线，≥2 点；中间点用来画弧线/绕障。**首尾会自动吸附到节点坐标** |
| `zones[].type` | `forbidden` | `forbidden`（禁行）/ `speed_limit`（限速） |

> ⚠ 路网模式下**禁止视线剪枝**（通道是人画的，切角会破坏原设计的绕行），代码强制只做共线合并。

**可行性是"按边"判的：一条通道 = 一个不可分割的整体**

| 行为 | 说明 |
|---|---|
| 判据 | 车体（含 margin）沿该边**每一段折线**扫掠，**任一段**过不去 → 整条边 `feasible=false` |
| 后果 | `reject_infeasible: true`（默认）时这条边**不参与路由**，其余边照走 → 所以"拆成多段"很关键：一段被挡不会让整个路网报废 |
| 诊断 | 失败时**点名**：`路网内连不通：唯一通路必须经过车体过不去的通道 B->C`；成功但绕开了也会提示：`⚠ 路网里有车体过不去的通道（已绕开：B->C）` |
| 可视化 | RViz 里不可行的通道画成**红色**（`map_server`），走过的通道黄色高亮（`global_planner`） |
| 怎么拆 | 画线器里按 **`v`**：把最后一条通道在每个点处切开 |

### 2.3 区域层怎么生效

| 类型 | 生效方式 | 谁遵守 |
|---|---|---|
| `forbidden` 禁行 | `map_server` 按车体**外接圆半径**膨胀后**烧进发布的全局图**（占据数会变大，日志会报告烧了多少格） | **全栈**：全局规划 / 路网路由 / 局部规划(P5) / RViz / 将来 nav2 |
| `speed_limit` 限速 | 不烧入栅格（软约束）：解析 + 可视化 + `ZoneSet::inSpeedZone()` 查询 | 速度规划 / 局部规划（P5） |

禁行区**不是"只给全局规划器"**：局部规划若不知道就会出现"全局绕开了、局部冲进去"，
所以统一在地图层注入，任何订阅这张图的模块自动遵守。开关：`zones.burn_into_map`、`zones.inflate`。

**膨胀量怎么定（`zones.inflate`）**

| 取值 | 含义 | 感受 |
|---|---|---|
| `-1`（默认） | 自动 = 车体**外接圆半径** = $\sqrt{(L/2+m)^2+(W/2+m)^2}$；Go2 的 `0.70×0.40 + 0.05` → **0.472 m** | 保守：**点判定**（只看中心格）的消费者也安全，任何朝向都不侵入；代价是每个区域外扩将近半个车身，走廊会被吃掉很多 |
| 显式数值（如 `0.25`） | 手动给一个值；`0.25` = 车体**内切半径** | 宽松：区域的"硬约束"变小。**我们的规划器/画线器本身都做朝向感知的车体扫掠检查**（真障碍仍在图里），所以安全性由 footprint 保证，不会因为膨胀小就撞 |

实测（本仓库 `go2_sim_factory` 的 3 个区域）：`0.472` 让一条贴着柱子的通道判为"过不去"，
改成 `0.25` 后同一条通道通过 —— 膨胀量直接决定路网可用性，**改的时候必须两边一起改**：

```yaml
# map_server 侧
zones.inflate: 0.25
```
```bash
# 画线器侧（同一个值，否则编辑器说"通得过"、map_server 说"过不去"）
.venv/bin/python3 src/map_server/scripts/route_editor.py --zone-inflate 0.25
```

---

## 3. 路网画线器（脚本在 `map_server`）

画线器位于 **`src/map_server/scripts/route_editor.py`** —— `routes.yaml` 与 `zones` 是
站点资产，加载/校验/发布在 `map_server`，编辑工具跟着资产走；本包只负责消费。

```bash
cd ~/r41_ws && .venv/bin/python3 src/map_server/scripts/route_editor.py
# 默认站点 go2_sim_factory；换站点加 --map-dir /home/gmd/rcs/maps/<站点>
# 左键加点 / 右键结束通道；z 进区域模式；s 保存 / q 退出
```

**完整操作手册（按键表、参数、三项自动校验、常见问题）见 `src/map_server/README.md`。**

---

## 4. RViz 显示清单

| 显示项 | 话题 | 说明 |
|---|---|---|
| Map | `/global_map/occupancy` | Durability 建议 `Transient Local`（缺失时会由 map_server 按需补发一次） |
| Path | `/pnc_2d/global_path` | 规划结果 |
| MarkerArray | `global_map/routes` | **全网**（节点/通道/单向箭头）+ **禁行·限速区** ← 由 map_server 发布 |
| MarkerArray | `/pnc_2d/plan_markers` | 起终点箭头、车体轮廓、**当前通路高亮** ← 由规划节点发布 |

---

## 5. 测试

**① 纯算法单测（不需要 ROS 环境，毫秒级）**

```bash
colcon test --packages-select pnc_2d --event-handlers console_direct+
colcon test-result --test-result-base build/pnc_2d     # 期望 0 failures
./build/pnc_2d/test_astar_planner                      # A*：15 用例
./build/pnc_2d/test_route_network                      # 路网：13 用例
./build/pnc_2d/test_map_zones                          # 区域层：6 用例
```

**② 端到端测试（ROS 图级别，`test/e2e/`）**

覆盖单测碰不到的东西：latched 话题的路径有效期、清场时机、换算法重启、运行时热切换。

```bash
colcon build --packages-select pnc_2d
bash src/pnc_2d/test/e2e/run_all.sh
```

| 脚本 | 覆盖 | 实测 |
|---|---|---|
| `test_clear_semantics.py` | 失败清空 / 成功发布 / 到达目标（按参数）/ `~/clear_path` | 11 + 11 项 |
| `test_restart_cleanup.py` | 换算法重启后旧路径与通路高亮被清掉 | 6 项 |
| `test_hot_switch.py` | `param set planner.type` 热切换 / 非法被拒 / `~/switch_planner` / `~/reload_params` | 14 项 |

要点：**不需要 map_server**（脚本自带地图，语义一致）、话题全部重映射到 `/t/*`（不干扰运行中的系统）、
节点日志在 `/tmp/pnc2d_e2e_<pid>.log`。细节与两个坑见 `test/e2e/README.md`。

---

## 6. 边界

- **不碰** `nav2d`（官方 nav2 栈，与本包互不依赖）、`perception`、`lightning`、`map_server`（本包只订阅它的话题）。
- `quadropted_controller`（步态/关节）是下游消费者，将在 P5 通过 `/pnc_2d/cmd_vel` 对接。
- 路网文件与区域层由 `map_server` 加载/校验/发布；本包只消费。
