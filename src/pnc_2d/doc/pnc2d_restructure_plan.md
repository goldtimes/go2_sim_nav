# pnc_2d 整改计划（冻结版）

> 状态：**已评审冻结（2026-09-21）**，按本计划实施。
> 范围：**只动 `src/pnc_2d`**。不碰 `nav2d`（那是官方 nav2 栈的位置）、不碰
> `perception` / `map_server` / `lightning`；`quadropted_controller`（步态/关节）只通过
> `/pnc_2d/cmd_vel` 交互。
> 相关文档：`pnc2d_dev_plan.md`（全局规划的算法细节）、`mpc_nmpc_guide.md`（MPC 原理与调参）。

---

## 0. 已定决策（本次评审）

| # | 问题 | 决策 |
|---|---|---|
| 1 | 节点拓扑 | **三节点**：`pnc_manager_node`（状态机）+ `global_planner_node` + `local_planner_node`，库层仍是一个 lib |
| 2 | 局部规划算法 | **本期不实现具体算法**，只交付抽象基类 + `NullLocalPlanner` + 工厂，保证链路可空跑 |
| 3 | MPC 形态 | **C++ 实现**（不用 rclpy 桥接 `scripts/mpc.py`；Python 那套保留作离线原型/基准） |
| 4 | 状态机风格 | **ROS 1 move_base 风格**：显式状态 + 转移表，纯库可单测 |
| 5 | 算法挂载 | **保持现状**：编译期工厂，通过 yaml（`planner.type` / `local.type`）切换；暂不上 pluginlib |
| 6 | 三节点 IPC（补充确认） | **manager→global 用 service**、**manager→local 用 action**、**local→底盘用话题**；详见 §4 |
| 7 | **新增需求：路网**（2026-09-21） | 路网作为**新 P2**；原 P2/P3/P4/P5 顺延为 P3/P4/P5/P6，详见 §6 P2 |
| 8 | 路网贴线严格度（2026-09-21） | **允许偏离**；`corridor_width: 0.0` 表示**严格贴线，遇障即停**（按通道逐条配置） |
| 9 | 路网画线工具（2026-09-21） | **Python 离线编辑器**（读 map.pgm 当底图 + 鼠标点选 → `routes.yaml`），不依赖 ROS |
| 10 | 路网目标输入（2026-09-21） | **只给坐标**（`/goal_pose`，自动投影到最近通道）；**站点**作为带语义的节点保留（type: station/charge/park，RViz 命名显示），不做“按站名下发目标”的话题；最后一段默认 **`hybrid`** |

**本期（整改）交付范围**：**P1 目录重排（✅ 已完成）+ P2 路网 + P3 三类抽象齐备 + P4 状态机与三节点空跑**。
局部规划的**具体算法**（含 C++ MPC，见 §6 P5）与重规划/恢复（P6）是整改之后的独立议题。

---

## 1. 现状问题清单（整改动因）

| # | 问题 | 证据 |
|---|---|---|
| P1 | 只有全局一条线，全局路径之后没有任何东西产生 `cmd_vel`（现靠 teleop 手控） | `src/global_planner_node.cpp` 只发 `/pnc_2d/global_path` + markers |
| P2 | 没有局部规划抽象：`planner_factory` 只管全局 | `include/pnc_2d/planner_factory.hpp` |
| P3 | 没有状态机：只有"收到 goal → 规划一次"的单动作 | 全包无 sm 相关代码 |
| P4 | 目录职责混放：节点壳、ROS 适配层（`src/ros_param_reader.hpp`）与算法实现同层；`scripts/`（离线原型）与 C++ 规划同层 | `find src/pnc_2d` |
| P5 | 包定位一眼看不出：配置只有 `global_planner.yaml`、launch 只有 `global_planner.launch.py` | `config/` `launch/` |
| P6 | 测试只够放全局，局部/状态机没有预留位置 | `test/test_astar_planner.cpp` |

---

## 2. 目标：对齐 ROS 1 的"三类 + 一个状态机"

| ROS 1 概念 | pnc_2d 抽象基类 | 实现 |
|---|---|---|
| `nav_core::BaseGlobalPlanner` | `core/global_planner.hpp`（**已有**） | `global/astar_planner`（已有）、**`global/route_network_planner`（P2 路网）**、`hybrid_astar`（后续议题） |
| `nav_core::BaseLocalPlanner` | `core/local_planner.hpp`（**新增**） | 本期只有 `local/null_local_planner`；算法见 §6 P5 |
| `nav_core::RecoveryBehavior` | `core/recovery_behavior.hpp`（**新增**） | 本期只留接口，不实现具体行为 |
| `move_base`（状态机） | `sm/manager_sm.hpp`（**新增**，纯库） | `nodes/pnc_manager_node`（薄壳） |

路网（P2）**不新增抽象**：`RouteNetworkPlanner` 就是 `GlobalPlanner` 的另一种实现，
通过现有 `planner.type: route_network` 切换。数据结构 `core/route_graph.hpp` 同时被
路网编辑器节点（P2）与将来的局部规划（贴线跟踪，P5）复用。

库层保持 **ROS-free**；ROS 只出现在 `src/nodes/` 的薄壳里（本包既定原则，见
`pnc2d_dev_plan.md` §4.1）。参数读取、地图转换、时间源等都以接口注入。

---

## 3. 目标目录树

```text
src/pnc_2d/
├── CMakeLists.txt                 # 1 个库(pnc_2d_core) + 3 个可执行 + 测试
├── package.xml
├── include/pnc_2d/
│   ├── core/
│   │   ├── types.hpp  cost_map_2d.hpp  footprint_collision.hpp  clearance_field.hpp
│   │   ├── param_reader.hpp
│   │   ├── route_graph.hpp               # ★P2 路网数据结构（节点/边/属性 + yaml 读写）
│   │   ├── global_planner.hpp            # 抽象类 A（已有）
│   │   ├── local_planner.hpp             # 抽象类 B ★P3 新增
│   │   ├── recovery_behavior.hpp         # 抽象类 C ★P3 新增
│   │   └── factory.hpp                   # 全局/局部/恢复 三套工厂（合并原 planner_factory）
│   ├── global/
│   │   ├── astar_planner.hpp
│   │   └── route_network_planner.hpp     # ★P2 图上路由（Dijkstra + 折线拼接）
│   ├── local/
│   │   └── null_local_planner.hpp        # ★本期唯一"实现"：不做控制，只回报状态
│   ├── recovery/
│   │   └── recovery_interface_only.md    # 占位说明（接口在 core/）
│   └── sm/
│       ├── events.hpp  states.hpp
│       └── manager_sm.hpp                # 状态 + 转移表（无 ROS，可单测）
├── src/
│   ├── core/  global/  local/  sm/       # 与 include 一一对应
│   └── nodes/                            # ★ROS 薄壳：只做 IO / 参数 / frame 校验
│       ├── global_planner_node.cpp       （搬移现有）
│       ├── local_planner_node.cpp        （新增，本期只跑 Null 实现）
│       ├── pnc_manager_node.cpp          （新增）
│       └── ros_param_reader.hpp          （从 src/ 挪入）
├── config/
│   ├── pnc_2d.yaml                       # 总入口：planner.type / local.type / sm.* / common.* / footprint.*
│   ├── global_astar.yaml                 # 全局算法片段（现有 global_planner.yaml 改名）
│   ├── route_network.yaml                # ★P2 路网参数（routes_file / goal_mode / turn_penalty）
│   └── local_null.yaml                   # 局部片段（本期只有 none）
├── launch/
│   ├── pnc_2d.launch.py                  # ★一键三节点（P4 后 tmux 用这个）
│   ├── global_planner.launch.py          # 保留：单跑调试 / 兼容现有 tmux 脚本
│   └── local_planner.launch.py           # 新增
├── test/
│   ├── test_astar_planner.cpp            （现有，路径调整）
│   ├── test_route_network.cpp            # ★P2 路网：yaml 解析 / 单双向 / 断网无解 / 最短路
│   ├── test_factory.cpp                  # 三个工厂 + Null 实现
│   └── test_state_machine.cpp            # 事件序列 → 状态断言（无 ROS）
├── scripts/                              # 离线工具/原型（保留，非运行时代码）
│   └── mpc.py  nmpc.py  traj_utils.py  legacy/   # 算法离线原型
# 注：路网画线器 route_editor.py 随资产放在 map_server/scripts/（见其 README 操作手册）
└── doc/
    ├── pnc2d_restructure_plan.md         # 本文件
    ├── pnc2d_dev_plan.md                 # 全局规划细节（其 §7 布局在 P1 后需同步）
    └── mpc_nmpc_guide.md
```

**地图目录约定扩展**（与 `map_server` 同源，P2 新增第 4 个文件）：

```text
~/rcs/maps/<站点>/
├── map.yaml + map.pgm      # 2D 栅格（map_server）
├── global.pcd / 0.pcd      # 3D 点云（perception/lightning）
└── routes.yaml             # ★P2 路网（节点 + 带几何的通道）
```

---

## 4. 三节点之间的接口（三节点方案必须先定 IPC，这是新增决策点）

| 链路 | 建议机制 | 理由 |
|---|---|---|
| 外部目标 → manager | 订阅 `/goal_pose`（RViz 2D Goal Pose 直接可用） | 不改用户习惯 |
| manager → global | **service** `pnc_2d/plan_path`（`PlanPath.srv`：start/goal → status/path/stats） | 一次性、短时（实测 6~33 ms），同步返回最直观 |
| manager → local | **action** `pnc_2d/follow_path`（goal=path；feedback=progress/cross_track/status；可 cancel） | 长时任务，需要反馈与取消（状态机要用） |
| local → 底盘 | 话题 `/pnc_2d/cmd_vel` | 与 `quadropted_controller` 对接；**不是** `/robot1/cmd_vel`（留一层给安全/仲裁） |
| 各节点 → 监控 | 话题 `/pnc_2d/state`（状态机状态）、`/pnc_2d/local_status` | RViz/终端可看 |

✅ **已确认（2026-09-21）**：按下表实施。“服务 vs 动作”的选择己获用户确认。

---

## 5. 数据流（目标形态）

```mermaid
graph LR
  MS[map_server 全局图] --> MGR[pnc_manager_node 状态机]
  PV[perception 局部图] --> MGR
  LO[lightning 定位] --> MGR
  RT[routes.yaml 路网] -->|P2| GP
  MGR -->|service: start/goal| GP[global_planner_node]
  GP -->|path| MGR
  MGR -->|action: FollowPath| LP[local_planner_node]
  LP -->|/pnc_2d/cmd_vel| QC[quadropted_controller]
  LP -->|feedback 进度/误差| MGR
  MGR -->|/pnc_2d/state| RV[RViz/监控]
```

---

## 6. 阶段划分与验收

> 原则：每阶段**可编译、可验收、不夹带**；话题名全程保持 `/pnc_2d/*` 不变，外部脚本无需改。

### P1 纯目录搬移（零行为变化）✅ **已完成（2026-09-21）**
- 内容：`core/ global/ sm/(空) nodes/` 分层；`ros_param_reader.hpp` 归入 `nodes/`；CMake 路径同步。
  `planner_factory.hpp/.cpp` → `core/factory.hpp/.cpp`。
- 验收：`colcon build` 零错；`colcon test` **16/16**（0 失败）；同一目标重跑结果与搬移前
  **逐项一致**：6 点 / 33.39 m / 扩展 51319 / 发现 52696 / 峰值开放集 5169 /
  完整 footprint 检查 71034 / 窗口尝试 1（耗时 33.0→45.6 ms，机器负载波动，非行为差异）。
- 不做：不改任何算法逻辑、不改参数名、不改话题名。（均遵守）

### P2 路网（route network）：让机器沿预先画好的路线走 ← **新增需求（2026-09-21）** ✅ **核心已实现（2026-09-21）**

**目标**：在地图上引入"路网"（节点 + 带几何的通道），让机器可以沿人为画好的路线走。
本阶段交付 **几何与路由正确 + 可视化 + 可编辑**；"贴着线走"的控制属于 P5 局部规划。

**数据模型**（ROS-free，`core/route_graph.hpp`）：

```yaml
# ~/rcs/maps/<站点>/routes.yaml   ← 与 map.yaml 同目录（换图四件套同源）
frame_id: map
nodes:
  - {name: "A1", x: 1.00, y: 2.00, type: waypoint}   # waypoint|station|charge|park
  - {name: "C1", x: 8.20, y: 2.05, type: charge}
edges:
  - {from: "A1", to: "C1", one_way: false, speed_limit: 0.8, corridor_width: 0.6,
     polyline: [[1.0,2.0], [4.0,2.02], [8.2,2.05]]}    # 中间点用来画弧线/绕障
zones:                              # ★区域层（可选）
  - {name: "charging_hall", type: forbidden,
     polygon: [[10.0,-6.0], [12.0,-6.0], [12.0,-4.0], [10.0,-4.0]]}
  - {name: "door_north", type: speed_limit, value: 0.30, polygon: [...]}
```

**区域层（禁行 / 限速）—— 2026-09-21 已实现**：

| 类型 | 生效方式 | 谁消费 |
|---|---|---|
| `forbidden` | map_server 按**车体外接圆半径**膨胀后**烧进发布的全局图**（`max(地图, 覆盖层)`） | **全栈**：全局规划 / 路由 / 局部规划（P5）/ RViz / 将来 nav2 —— 不需要各自解析，不会漏 |
| `speed_limit` | 不烧入栅格（软约束）：解析 + 可视化 + `ZoneSet::inSpeedZone()` 查询 | 速度规划 / 局部规划（P5） |

- 为什么禁行区不“只给全局规划器”：局部规划如果不知道，就会出现“全局绕开了、局部冲进去”；
  统一注入到地图层是唯一不漏的做法。
- 开关：`zones.burn_into_map`（默认 true）/ `zones.inflate`（<0 = 自动用车体外接圆半径）。
- 实测：2×2 m 禁行区 + 0.472 m 膨胀 → 烧入 **3274 格**，发布的图占据从 12041 变 15315（一致）；
  禁行区压在通道上时，那条通道被判定“车体过不去”并 `WARN`（画错能早发现）。
- 画线器：`z` 进区域模式（左键加顶点 / 右键闭合），`t` 切 forbidden↔speed_limit，`-`/`=` 调限速值；
  编辑器自带的通行性检查**含禁行区膨胀**，与 map_server 同口径。

- 边 = **带几何的折线 + 属性**：`one_way`（单向）、`speed_limit`（m/s）、
  **`corridor_width`（允许横向偏离的半宽 [m]；`0.0` = 严格贴线，遇到障碍即停车）**。
  逐通道可调：宽通道可给 0.5~0.8，窄/危险段给 0.0。
- 节点分两类：拓扑连接点（waypoint）与**语义站点**（station/charge/park，可直接作目标或停靠点）。

**路由算法**（`global/route_network_planner.{hpp,cpp}`，实现现有 `GlobalPlanner` 抽象）：
1. 起点/终点 → 投影到最近的路网边（也支持直接用站点名作目标）；
2. 图上 **Dijkstra**，边代价 = 弧长 / `speed_limit`（可选 `turn_penalty` 惩罚急转弯）；
3. 拼接折线 → 输出 `PlanResult.path`，yaw 取折线切向。
- ⚠ **不做视线剪枝**：通道是人为画的，剪枝会切角、破坏原本设计的绕行；
  路网模式下 `path_prune_mode` 强制为 `collinear`（只合并共线点）。
- **可行性校验**：加载时逐边用现有 footprint 碰撞检查验证"这条通道真的能过"
  （0.70×0.40 + margin），不可行的边报警并标记，避免"画得漂亮却过不去"。

**到目标的最后一段**（`route_network.goal_mode`）：
- `strict`：只走到离目标最近的路网点（默认，先做）；
- `hybrid`：先沿路网到最近点，再用现有 A* 走完最后一段自由空间（可配开关，后做）。

**依赖**：yaml 解析用系统 `yaml-cpp`（与 `map_server` 同款，库仍然 ROS-free）。
⚠ 沿用 `map_server` 的教训：`as<bool>()` 对 `0/1` 会抛异常，`one_way` 这类字段
必须写**宽松解析**（0/1/true/false/yes/no 都收）。

**交付物与验收**：

| 交付 | 位置 | 验收 |
|---|---|---|
| 路网数据结构 + yaml 读写/校验 | `core/route_graph.{hpp,cpp}` | 单测：正常解析、坏文件报错、非法边（悬空节点 / 空折线）报错 |
| 路由规划器 + 工厂注册 | `global/route_network_planner.*`、`core/factory.cpp` | 单测：最短路正确、单向边反向无解、断网无解、站点名目标 |
| 参数 | `config/route_network.yaml` | 改 yaml `planner.type: route_network` 即切换（与现有机制一致） |
| 可视化 | 现有节点发 MarkerArray：全网节点球 + 边线 + 方向箭头 + 当前通路高亮 | RViz 一眼看清 |
| **画线编辑器** | `src/map_server/scripts/route_editor.py`（Python + matplotlib，**不依赖 ROS**） | 用现有地图画 3 条通道 → 保存 → 重开能加载回放；叠加地图位置正确 |
| 站点目标 | 路网里的 `type: station/charge/park` 节点 | 发坐标目标即可 |
| 换图联动 | 站点目录里的 `routes.yaml` 随地图一起切 | ✅ **已实现**（map_server 在 `load_map` 时一并重载路网，并清掉旧站点标记） |
| 资产归属 | 路网由 map_server 加载/校验/可视化 | ✅ **已实现**：`publish_routes` / `routes_file` / `topic_routes`（默认 `global_map/routes`）；**没有 routes.yaml 时只打 INFO，不报错**；
全网可视化在 map_server，当前通路高亮在 pnc_2d |

**不做（P2 边界）**：不做贴线控制（P5）；不做交叉口让行/优先级/多车调度（P6）；
不做 RViz 拖拽式图形编辑（用点选录制代替，拖拽交互成本高）。

**画线方式**（按落地顺序）：
1. **Python 离线编辑器（本期主推，已实现）**：`src/map_server/scripts/route_editor.py`
   —— 用 matplotlib 把
   `map.pgm` 当底图显示，鼠标左键点选画通道、右键结束一条，键盘完成/撑销/删除，
   `s` 保存成 `routes.yaml`；保存前用车体矩形沿每条通道扫一遍，把"过不去"的标红。
   优点：不依赖 ROS、改完即见、与此前 Python 侧碰撞校验的口径一致（已与 C++ 对齐验证过）。
   依赖：`.venv` 已有 matplotlib/numpy；yaml 写出需 `pyyaml`（没有就装，或手写 dump）。
2. 手写 / 脚本生成 yaml —— 批量或程序化生成时用。
3. 从已录制轨迹批处理提取（`data/traj.txt`、rosbag）→ 脚本转 yaml。
4. （以后）RViz 点选录制节点 / 拖拽编辑 / 栅格图骨架化自动提取。

**目标表达 / 最后一段（已定稿 2026-09-21）**：

| 项 | 结论 |
|---|---|
| 目标怎么给 | **只给坐标**（RViz 2D Goal Pose 直接点）→ 自动投影到最近通道；不需要“按站名下发目标”的话题 |
| 站点（station/charge/park） | **保留为语义节点**：存在路网里、RViz 用不同颜色 + 名字标签显示；将来（P6 任务调度）可用作命名目标 |
| 目标不在路网上（最后一段） | 默认 **`hybrid`**：沿路网走完主通道，最后一段用现有自由空间 A* 补到目标；
`strict`（只到最近通道上的投影点）作为可配项 |

> **换图联动：已实现**（2026-09-21）——原本计划推到 P4，但 map_server 本来就有
“按目录/服务换图”的入口，顺带把 `<地图目录>/routes.yaml` 一起载入更自然，
于是直接做了：换站点时路网一起换，且**没有 routes.yaml 时只提示不报错**。

**P2 实测记录（2026-09-21）**：

| 项 | 结果 |
|---|---|
| 单测 | `test_route_network` **11/11 通过**（解析/坏文件/端点吸附/投影插值/存取往返/不切角/单行绕行/不连通/严格vs混合/不可行拒绝/未载入） |
| 全包回归 | `colcon test` **0 失败**（26 个 gtest 用例 + 2 个 ctest 包装） |
| 不切角 | 起点 (3,1) → 终点 (9,3)：路径 **8.00 m**（若允许切角只要 6.32 m），转角点 B(9,1) 保留 |
| 单行绕行 | 同一通道上逆行：(7,1)→(3,1) 双向 **4.00 m**；把该通道改为单向 A→B 后 **28.00 m**（绕一整圈） |
| 严格 vs 混合 | 目标离通道 2 m 时：`strict` **2.00 m**（停在投影点）/ `hybrid` **4.00 m**（补到目标） |
| 不可行通道 | 通道中间横一道墙 → 可行 3 / 不可行 1；`reject_infeasible=true` 时报 `NO_PATH` 并说明“唯一通路经过 1 条车体过不去的通道” |
| 真实地图端到端 | 607×307 工厂地图：路网 3 节点 / 2 通道（单向 1，总长 30.71 m）→ 规划 **6 点 / 30.71 m / 0.0 ms / 仅 5 个图节点**（同图 A* 要 33 ms、5 万格）；逆向因单行返回 `NO_PATH` |
| RViz 可视化 | `route_nodes 3 / route_edges 2 / route_oneway 1 / route_active 2`（当前通路高亮，失败时发 `DELETE` 清掉残留） |
| Python 画线器 | 地图加载、可行性标红、节点按坐标复用（两条通道共享交点不会生成重复节点）、连通性/重复通道告警、属性（单向/限速/走廊）持久化与重载均验证通过 |

实现过程中修掉的两个自己引入的问题：
1. 画线器最初只按“点击时吸附”复用节点 → 程序化/回放场景下同一交点会生成两个同坐标节点、把路网断开；现改为结束通道时**按坐标兜底复用**，并在保存时做**连通性检查**。
2. 规划失败后 RViz 里上一次的通路高亮不消失（MarkerArray 不会自动删缺席标记）→ 对多出来的 id 显式发 `DELETE`。

**尚未做（不阻塞使用）**：站点语义在画线器里已支持（`k` 键切换 waypoint/station/charge/park），但还没做“按站名下发目标”的话题；换图联动（见上）；真实站点的路网需要你在 GUI 里画（我无法操作鼠标）。

### P3 抽象补齐（原 P2）
- 内容：`LocalPlanner` / `RecoveryBehavior` 接口；`NullLocalPlanner`；`factory.hpp`（三套工厂）；
  `config/pnc_2d.yaml`（含 `local.type: none`）；`test/test_factory.cpp`。
- 验收：新单测全绿；`local.type: none` 时全局规划行为与 P1 **完全一致**。
- 不做：不写任何真实局部算法。

### P4 状态机 + 三节点可空跑（原 P3）
- 内容：`sm/manager_sm`（ROS1 风格显式状态表）；
  `pnc_manager_node`（订阅 odom/map/goal → 调全局 service → 调局部 action → 发布状态）；
  `local_planner_node`（只跑 Null 实现，`/pnc_2d/cmd_vel` 输出恒为 0）；
  `launch/pnc_2d.launch.py`；`test/test_state_machine.cpp`；IPC 接口（§4）。
- 状态集（初版）：
  `Idle → Planning → Following → GoalReached`，异常分支 `Planning 失败 → Failed`、
  `Following 卡住/连续失败 → Recovering →（成功回 Following / 超限 → Failed）`。
  事件集：`GoalReceived / PlanOk / PlanFail / Blocked / Stuck / RecoveryDone / Cancel / OdomJump`。
- 验收：状态机单测全绿；仿真里点目标 → 日志与 `/pnc_2d/state` 出现
  `Idle→Planning→Following→GoalReached`；**不发任何速度指令**（Null 局部）。
- 不做：不做恢复行为的具体实现；不做重规划。

### P5（整改之后，另立议题）局部规划算法（**C++**）（原 P4）
- 候选顺序（待单独评审）：先 `pure_pursuit` 打通链路 → 再 **C++ MPC**（`local/mpc_local_planner`，
  求解器 `OSQP`/`CasADi`，与 `scripts/mpc.py` 同构：偏差输入前馈 + 每步线性化 + Δu 平滑）。
- **路网贴线**（P2 对应的控制侧）：横向偏差按 `corridor_width` 约束；
  `corridor_width > 0` 时允许在走廊内侧移绕障并回线；`= 0.0` 时严格贴线，
  遇障碍直接报 `BLOCKED` 停车等状态机处理。
- 验收（届时定）：仿真 A→B 全程无碰撞、到点停车、横向误差 < 10 cm；与 Python 版逐指标对比。
- 备注：`scripts/mpc.py` / `nmpc.py` 继续作**离线基准**，用于验证 C++ 实现数值一致。

### P6（可选）重规划管理器 / 恢复行为 / pluginlib（原 P5）
- 地图版本号 + episode/seq 隔离、卡住检测、失败兜底、恢复行为具体实现；
- 算法多了再考虑 pluginlib 动态挂载。

---

## 7. 风险与回退

| 风险 | 应对 |
|---|---|
| 搬移影响正在跑的 tmux 启动脚本（`go2_sim_bringup.json` 现跑 `ros2 launch pnc_2d global_planner.launch.py`） | P1 保留同名 launch；P4 完成后再切换成 `pnc_2d.launch.py` 并同步改 json |
| 三节点引入 IPC 复杂度（超时/取消/竞态） | 状态机集中管理超时与取消；IPC 用 service（短、同步）+ action（长、可取消） |
| 路网画得与地图不符（通道被障碍堵住 / 车体过不去） | 加载时逐边做 footprint 可行性校验并报警；RViz 叠加显示不可行边 |
| 路网与自由空间不一致（目标不在路网上） | `goal_mode: strict` 先只到路网点；需要时开 `hybrid` 用 A* 收尾 |
| 状态机与库层耦合 ROS | 状态机放 `sm/`（无 ROS），节点只做事件适配；用 `test_state_machine.cpp` 纯库验证 |
| 大改动期间功能不可用 | 每阶段独立提交；P1 为纯 `git mv`，可整体回退 |

---

## 8. 与其它包的边界

- `nav2d`：官方 nav2 栈（配置/启动），与本包**互不依赖、不共享代码**。
- `perception`：发动态局部图；本包只订阅，不改其接口。
- `map_server`：发静态全局图；本包只订阅。
- `quadropted_controller`：步态/关节控制器，订阅 `/pnc_2d/cmd_vel`（P5 起）。
- `pnc_3d`：3D 规划栈；建议其内部目录也采用同样的 `core/ + 算法目录 + nodes/` 约定（非本次范围）。
