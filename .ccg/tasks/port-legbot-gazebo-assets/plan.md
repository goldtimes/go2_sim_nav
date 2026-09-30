# 移植计划：legbot_3D_Nav 的 Gazebo 世界与机器人 → src/bringup

## 0. 目标

把 `/home/gmd/r41_ws/legbot_3D_Nav`（ROS 1 Noetic + Gazebo Classic 11）中的
**Gazebo 世界模型**（`Building.world` 多层楼梯建筑 + `Building.dae`）与
**机器人模型**（Unitree A1：xacro/meshes/config/scan_mode）复制进
`/home/gmd/r41_ws/src/bringup`（ROS 2 Humble），并在目标侧能加载、spawn、跑通
基本的传感器与关节控制链路。

## 1. 现状核实（已确认的事实）

### 1.1 源工程 `legbot_3D_Nav`

| 项目 | 事实 |
| --- | --- |
| 框架 | ROS 1 Noetic（catkin，`package format="2"`），Gazebo Classic 11 |
| 世界 1 | `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/Building.world`，SDF **1.5**，`<physics type="ode">`，`<include><uri>model://Building</uri>` |
| 世界 1 资源 | `unitree_gazebo/models/Building/{model.sdf,model.config}`（SDF 1.7，mesh=`model://Building.dae`）+ `models/Building.dae`（**20 MB**） |
| 世界 2 | `src/Mid360_imu_sim/worlds/standardrobots_factory.world`，SDF 1.6，534 行；依赖 `house_1/2/3`、`powerplant`、`asphalt_plane`、`person_*` 等**外部模型（本地不存在）** |
| 机器人 | `unitree_guide/unitree_ros/robots/a1_description`（共 35 MB）：`xacro/{robot,const,leg,materials,transmission,stairs,gazebo}.xacro`、`urdf/a1.urdf`、`meshes/*.dae`（含 `livox_mid360.dae`）、`config/robot_control.yaml`、`scan_mode/mid360.csv` |
| 机器人插件 | `libgazebo_ros_control.so`（ROS1 ros_control）、`libgazebo_ros_p3d.so`×5（真值位姿）、`libgazebo_ros_imu_sensor.so`×2、`libgazebo_ros_force.so`、`liblivox_laser_simulation.so`（MID360 → `/scan`）、`libunitreeFootContactPlugin.so`、`libunitreeDrawForcePlugin.so` |
| 控制 | `unitree_legged_control/UnitreeJointController`（自定义 ros_control 控制器，PID p=100/300、d=5/8）×12 + `joint_state_controller`；RL 步态 `unitree_guide/junior_ctrl`；真值里程计 `unitree_guide/state_from_gazebo`；`pointcloud2livox.py`（PointCloud2 → Livox CustomMsg） |
| 许可 | 工程 Apache-2.0；`unitree_ros` 为 BSD-3（Unitree Robotics） |

### 1.2 目标工程 `src/bringup`（ROS 2 Humble）

| 项目 | 事实 |
| --- | --- |
| 仿真栈 | `gazebo_sim` 走的是 **Gazebo Sim 6 / Ignition Fortress**：`ros_gz_sim` + `ros_gz_bridge` + `gz_ros2_control`；world 为 SDF 1.6/1.7/1.8 |
| 入口 | `gazebo_sim/launch/launch.py`（`world:=` 参数）→ `gazebo_go2_sensors.launch.py` / `gazebo_go2_self.launch.py` → `go2_description` |
| 现有机器人 | `go2_description`（ROS2 xacro + `<ros2_control>` 标签，`gz_ros2_control::GazeboSimROS2ControlPlugin`），命名空间 `robot1`，`config/robots.yaml` |
| 现成世界 | `gazebo_sim/world/{warehouse.sdf, rmuc_2025_world.sdf, cafe.world, L02-S0*.sdf}`；`gazebo_sim/models/` 已有 `aws_robomaker_*`、`ground_plane`、`sun`、`cafe*` 等（**没有 Building**） |
| 现有控制器 | `quadropted_controller`（`robot_controller_gazebo.py` 步态、`QuadrupedOdometryNode.py` 里程计、`cmd_vel_pub.py`） |
| 其他 | `quadropted_msgs`、`rslidar_sdk`/`rslidar_msg`（实车用）、`cyclonedds.xml` |

### 1.3 环境（本机已安装，实测）

- ROS 2 Humble（`/opt/ros/humble`），Gazebo Classic **11.10.2**，Gazebo Sim **6.16.0**（`ign gazebo` / `gz sim`）二者共存。
- Gazebo Classic 的 ROS 2 桥已齐全：`gazebo_ros`、`gazebo_ros2_control`、
  `libgazebo_ros_p3d.so`、`libgazebo_ros_imu_sensor.so`、`libgazebo_ros_force.so`、
  `libgazebo_ros_joint_state_publisher.so`、`libgazebo_ros_ray_sensor.so`、`libgazebo_ros_bumper.so`。
- **`ros2_livox_simulation` 已装到 `/opt/ros/humble`**：提供 `libros2_livox.so`（Gazebo Classic 插件的 ROS2 版），
  发布 `livox_ros_driver2/CustomMsg`，自带 `scan_mode/mid360.csv`；`livox_ros_driver2` 亦已安装。
- 结论：源工程用到的每一种插件，在 **Gazebo Classic + ROS 2** 路径上都有 1:1 对应件。

## 2. 关键决策：两条技术路线（需你确认，默认建议 A）

### Track A — Gazebo Classic 11 + `gazebo_ros`（ROS 2）「等价平移」

世界/机器人的 SDF、xacro、mesh、csv 基本可原样复制，只改插件标签与控制配置。
保真度最高（ODE 物理、`model://` 资源、MID360 非重复扫描全部保留），工作量最小。
代价：Gazebo Classic 已 EOL（2025-01），与现有 `gazebo_sim`（Fortress）不是同一后端，需要并列共存。

### Track B — Gazebo Sim 6 Fortress（与现有 `gazebo_sim` / Go2 一致）

与仓库现有仿真栈统一，长期可维护。代价：世界从 SDF 1.5 升到 Fortress、ODE 物理块丢弃、
OGRE1 材质脚本失效（Building 会发黑）、`liblivox_laser_simulation` 没有 gz-sim 版（MID360 扫描模式无法直接复现）。

### 建议

先用 **Track A** 把 Building 场景和 A1 跑起来（快速验证资产正确性），
再把 **Track B** 作为后续独立任务（世界升版 + 插件替换 + 雷达降级方案）。
若你只想要一套栈、能接受画质与雷达模式降级，则直接走 Track B。

### Track A 可行性实测（2026-09-30，本机已验证）

把 `Building/`、`Building.dae`、`Building.world` 复制到 `/tmp/gzcheck/` 后，
用 `GAZEBO_MODEL_PATH=/tmp/gzcheck/models gzserver --verbose /tmp/gzcheck/world/Building.world` 实测：

| 验证项 | 结果 |
| --- | --- |
| Gazebo Classic 11.10.2 加载 SDF **1.5** 世界 | ✅ 通过，无版本告警 |
| `model://Building` 资源解析 | ✅ 通过，无需改写 URI |
| 多层建筑模型注册 | ✅ `gz model -i -m Buliding -w tower` 返回 `id: 9, is_static: true`，pose = 原点 |
| 世界/模型错误日志 | ✅ 无 missing model / missing mesh / 解析错误 |
| 无显示器环境 | ⚠️ `RenderEngine.cc: Can't open display`，物理与话题正常，**渲染被禁用**——跑 GUI 或依赖相机的仿真必须有 X 显示 |

**实测发现（必须记录）**：建筑模型在内层 `model.sdf` / `model.config` 里的名字被上游拼错为
**`Buliding`**（少了一个 `d`），而世界文件引用的是目录名 `model://Building`。
两者拼写不同但仍能工作；后续任何按名字查模型的地方（spawn、桥接、`gz model`、脚本）都得用 `Buliding`。
移植时可选择**保持原样**（零风险）或顺手修正为 `Building`（world 里的 `<name>` 一并改），二选一需明确，别只改一处。

验证残留：`/tmp/gzcheck/`（≈20 MB 暂存副本），可直接复用为 Phase 1 的导入源。

## 3. 插件/接口映射表

| 源（ROS1 + Classic） | Track A（ROS2 + Classic） | Track B（ROS2 + Fortress） |
| --- | --- | --- |
| `libgazebo_ros_control.so` + `<transmission>` | `libgazebo_ros2_control.so` + `<ros2_control>` + controller yaml | `gz_ros2_control::GazeboSimROS2ControlPlugin` + `<ros2_control>` |
| `libgazebo_ros_p3d.so`×5（`<topicName>`） | 同名插件，参数改 `<ros><namespace>` + `<remapping>` | `gz::sim::systems::OdometryPublisher` |
| `libgazebo_ros_imu_sensor.so`×2 | 同名插件（ROS2 参数风格） | `imu` sensor + `ros_gz_bridge` |
| `libgazebo_ros_force.so` | 同名插件 | `gz::sim::systems::ApplyLinkWrench` |
| `liblivox_laser_simulation.so` + `a1_description/scan_mode/mid360.csv` | `libros2_livox.so` + `ros2_livox_simulation/scan_mode/mid360.csv`，输出 `CustomMsg` | 无对应件：用 `gpu_lidar` 近似（丢失非重复扫描）或自研 gz-sim 插件 |
| `libunitreeFootContactPlugin.so` | `libgazebo_ros_bumper.so`，或自编译 ROS2 版 | `contact` sensor + `ros_gz_bridge` |
| `libunitreeDrawForcePlugin.so` | 丢弃（纯可视化） | 丢弃 |
| `joint_state_controller` + `UnitreeJointController`(PID) | `joint_state_broadcaster` + `position_controllers/JointGroupPositionController` / 自研 PID 控制器 | 同左 |
| `state_from_gazebo` 真值里程计 | 复用 Gazebo 真值 → odom/TF | 复用现有 `QuadrupedOdometryNode.py` |
| `pointcloud2livox.py`（ROS1） | 不需要（插件直接出 CustomMsg） | 需要移植成 rclpy 节点（已有 ROS1 参考实现） |

## 4. 阶段计划

### Phase 0 — 基线与决策（S）

- 产出：`task.json`、本 `plan.md`、`context.jsonl`（关键文件清单）。
- 动作：确认 Track A/B；确认包命名（`a1_description` + 新 launch 归属 `gazebo_sim` 还是独立 `legbot_sim` 包）。
- 验收：你签字确认路线，不再变动。

### Phase 1 — 世界模型移植（M，Track A 已实测可行）

- 复制 `Building/{model.config,model.sdf}` + `Building.dae` → 目标 `models/` 目录
  （Track A 可直接复用 `/tmp/gzcheck/models/`，无需二次拷贝）。
  与 `gazebo_sim/models/` 已有的 `ground_plane`、`sun` 不冲突。
- **Track A**：`Building.world` 可**原样**复制为 `world/building.world`（SDF 1.5 在 Classic 11 下正常），
  只需在启动文件里把 `GAZEBO_MODEL_PATH` 指向模型目录。
  **Track B**：需升 SDF、去 ODE 块、材质脚本转 PBR。
- 处理上游 `Buliding` 拼写（保持原样 / 统一改名，见 2.4）。
- 产出文件：`models/Building/{model.config,model.sdf}`、`models/Building.dae`、`world/building.world`。
- 验收：`gzserver --verbose` 加载无 missing model 错误；`gz model -i -m Buliding -w tower` 返回静态模型；
  楼梯碰撞可站立（Phase 2 spawn 后联测）。

### Phase 2 — 机器人模型包（M）

- 新建 `src/bringup/a1_description/`，目录结构与 `go2_description` 对齐：
  `package.xml`、`CMakeLists.txt`、`xacro/`、`urdf/`、`meshes/`、`config/`、`scan_mode/`、`launch/`。
- 复制：A1 xacro 全套、`meshes/*.dae`（35 MB）、`scan_mode/mid360.csv`。
- 重写 `xacro/gazebo.xacro`：按第 3 节映射表替换插件；删除 ROS1 专属参数写法；
  移除无 ROS2 对应件的 `libunitree*` 插件（或先用 `libgazebo_ros_bumper.so` 顶替接触反馈）。
- 保留 `robot.xacro` 的 `$(find a1_description)` 写法（ROS 2 xacro 支持）。
- 产出文件：整个 `a1_description` 包。
- 验收：`xacro xacro/robot.xacro > /tmp/a1.urdf && check_urdf` 通过；spawn 后 `ros2 topic list` 出现 `joint_states`/`imu`/`scan`；TF 树完整。

### Phase 3 — 控制与传感器链路（M）

- `config/a1_ros2_control.yaml`：`joint_state_broadcaster` + 12 关节位置控制器（或按需的 `position_controllers/JointGroupPositionController`）。
- `launch/`：`robot_state_publisher`、`controller_manager` spawner、`use_sim_time`、真值里程计→`odom`+TF。
- Track B 追加：`pointcloud2livox` rclpy 节点（PointCloud2 → `livox_ros_driver2/CustomMsg`），供 FAST-LIO/lightning 消费。
- 命名空间统一（源工程用 `/a1_gazebo`，bringup 用 `robot1`）——按 target 侧惯例统一。
- 验收：`ros2 control list_controllers` 全 active；`ros2 topic hz` 雷达/IMU 频率符合预期；关节位置指令能让机器人站立。

### Phase 4 — 启动集成（M）

- 新增 `src/bringup/gazebo_sim/launch/gazebo_a1_sensors.launch.py`，与 Go2 版并列；
  打通 `ros2 launch gazebo_sim launch.py sensors:=true world:=building.sdf robot:=a1`。
- 不修改现有 Go2 行为（`robot:=` 默认 `go2`，向后兼容）。
- 验收：单条命令起 Building + A1；再跑一次 Go2 流程确认无回归。

### Phase 5 — 验证与文档（S–M）

- 冒烟测试：headless 启动世界、spawn、控制器激活、话题频率检查（可脚本化）。
- 文档：`gazebo_sim/README.md` 增加世界/机器人对照表与启动示例；记录话题表。
- 许可：在 `THIRD_PARTY_NOTICES.md` 记录 `unitree_ros`(BSD-3) 与 legbot 资产来源。
- CCG：写 `review.md`，归档任务到 `.ccg/tasks/archive/2026-09/`。

### Phase 6 —（可选，待你决定）步态/RL 与导航链移植（L+）

- 源工程控制器是 `junior_ctrl`（ROS1 C++ + Unitree SDK + ros_control），移植成本高。
- 备选 ①：沿用 bringup 现有 `quadropted_controller` 步态，重调到 A1 尺寸；
  备选 ②：完整移植 `junior_ctrl` 与 `unitree_legged_control` 到 ROS 2；
  备选 ③：只交付模型+世界，控制交给后续独立任务。
- 验收：A1 能在 Building 内按 `/cmd_vel` 行走并上下楼梯（若选 ①②）。

## 5. 风险与缓解

1. **材质**：`Building/model.sdf` 用 `file://media/materials/scripts/gazebo.material`（OGRE1 材质脚本），
   Track B 下不生效 → 模型发黑。缓解：材质转 PBR/ambient；Track A 无此问题。
2. **MID360 非重复扫描**：gz-sim 的 `gpu_lidar` 无法复现 rosette 扫描模式，
   会改变 FAST-LIO/lightning 的建图与定位质量。缓解：Track A 用 `libros2_livox.so`（保真）；
   Track B 需自研 gz-sim 插件或接受降级。
3. **控制器缺失**：`unitree_legged_control/UnitreeJointController` 无 ROS 2 对应件，
   PID 需要重调，A1 站立稳定性要重新验证。
4. **仓库体积**：`Building.dae` 20 MB、`a1_description` 35 MB，共约 55 MB 进入 git。
   缓解：确认是否接受；或改用 Git LFS（需你同意）。
5. **双 Gazebo 共存**：Classic 与 gz-sim 6 同机，`GAZEBO_MODEL_PATH` 与 `GZ_SIM_RESOURCE_PATH`
   容易混用；启动文件里必须显式设置，避免串栈。
6. **源码树污染**：`gazebo_sim/world/{build,install,log}` 是误落进源码目录的 colcon 产物，
   而 `CMakeLists.txt` 会整目录 `install(DIRECTORY world)`。建议清理（需你确认后我再动）。
7. **外部模型缺失**：`standardrobots_factory.world` 依赖 8 个本地不存在的模型，
   需要联网从 Gazebo 模型库拉取（当前环境网络受限）。
8. **流程缺口**：AGENTS.md 要求 M+ 复杂度做双模型（Gemini + Claude）分析/审查，
   但本机没有 `~/.claude/bin/codeagent-wrapper` 与 CCG prompts 目录，无法调用；
   本计划由本地静态核查得出，后续审查环节同样只能本地进行。

## 6. 待确认问题

1. 目标后端选 **Track A（Gazebo Classic，快、保真）** 还是 **Track B（Gazebo Sim 6，与现有 Go2 栈一致）**？
2. 机器人范围：只要「模型能 spawn、关节可控」，还是要连 **A1 的行走/RL 步态** 一起移植？
3. 世界范围：只要 **Building**，还是也要 **factory 场景**（缺模型，需联网）？

## 7. 验收测试清单（最终）

- [ ] `check_urdf` / `xacro` 无错误
- [ ] 世界加载无 missing model / missing plugin 警告
- [ ] A1 spawn 成功，`ros2 topic list` 含 `joint_states`、`imu`、`scan`
- [ ] `ros2 control list_controllers` 全部 active
- [ ] 雷达/IMU 话题频率与仿真设定一致（`ros2 topic hz`）
- [ ] TF 树无断裂（`ros2 run tf2_tools view_frames`）
- [ ] 现有 Go2 仿真流程无回归
- [ ] `git diff` 仅含计划内变更；任务已归档
