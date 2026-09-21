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

输出：`/pnc_2d/global_path`（`nav_msgs/Path`）、`/pnc_2d/plan_markers`（起终点箭头 + 车体轮廓 + **当前通路高亮**）。

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

### 2.3 区域层怎么生效

| 类型 | 生效方式 | 谁遵守 |
|---|---|---|
| `forbidden` 禁行 | `map_server` 按车体**外接圆半径**膨胀后**烧进发布的全局图**（占据数会变大，日志会报告烧了多少格） | **全栈**：全局规划 / 路网路由 / 局部规划(P5) / RViz / 将来 nav2 |
| `speed_limit` 限速 | 不烧入栅格（软约束）：解析 + 可视化 + `ZoneSet::inSpeedZone()` 查询 | 速度规划 / 局部规划（P5） |

禁行区**不是"只给全局规划器"**：局部规划若不知道就会出现"全局绕开了、局部冲进去"，
所以统一在地图层注入，任何订阅这张图的模块自动遵守。开关：`zones.burn_into_map`、`zones.inflate`。

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

```bash
colcon test --packages-select pnc_2d --event-handlers console_direct+
colcon test-result --test-result-base build/pnc_2d     # 期望 0 failures
./build/pnc_2d/test_astar_planner                      # A*：15 用例
./build/pnc_2d/test_route_network                      # 路网：11 用例
./build/pnc_2d/test_map_zones                          # 区域层：6 用例
```

---

## 6. 边界

- **不碰** `nav2d`（官方 nav2 栈，与本包互不依赖）、`perception`、`lightning`、`map_server`（本包只订阅它的话题）。
- `quadropted_controller`（步态/关节）是下游消费者，将在 P5 通过 `/pnc_2d/cmd_vel` 对接。
- 路网文件与区域层由 `map_server` 加载/校验/发布；本包只消费。
