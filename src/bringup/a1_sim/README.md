# a1_sim

Unitree A1 在 Gazebo Classic 11 + ROS 2 Humble 下的仿真启动包。
世界（多层建筑）与机器人来自 `legbot_3D_Nav`（ROS 1 Noetic）的移植。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/r41_ws/install/setup.bash

# 带 Gazebo 界面
ros2 launch a1_sim sim.launch.py

# 无界面（服务器模式）
ros2 launch a1_sim sim.launch.py gui:=false
```

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `world` | `building.world` | 世界文件名（相对 `a1_sim/world`）或绝对路径 |
| `robot_name` | `a1` | 机器人命名空间，同时作为 xacro 的 `robot_name` |
| `gui` | `true` | 是否启动 Gazebo 客户端 |
| `verbose` | `false` | gzserver 详细日志 |
| `stand` | `true` | 是否启动站立姿态节点（`a1_description/scripts/stand_pose.py`） |
| `stand_ramp` | `2.5` | 站立姿态斜坡时间（秒） |
| `x` / `y` / `z` | `-5.0` / `7.0` / `0.5` | 出生位姿 |
| `yaw` | `0.0` | 出生偏航角（弧度） |

## 启动后能看到什么

| 话题 | 类型 |
| --- | --- |
| `/a1/livox/lidar` | `livox_ros_driver2/CustomMsg`（MID360 非重复扫描） |
| `/a1/livox/imu` | `sensor_msgs/Imu` |
| `/a1/imu` | `sensor_msgs/Imu`（躯干） |
| `/a1/joint_states` | `sensor_msgs/JointState` |
| `/a1/joint_group_position_controller/commands` | `std_msgs/Float64MultiArray` |
| `/a1/ground_truth/base_w` | `nav_msgs/Odometry`（世界系真值） |
| `/a1/ground_truth/{FL,FR,RL,RR}_foot` | `nav_msgs/Odometry` |
| `/a1/contact/{FL,FR,RL,RR}_foot` | `gazebo_msgs/ContactState` |

控制器：`/a1/controller_manager` 下的 `joint_state_broadcaster` 与
`joint_group_position_controller`，启动后应均为 `active`：

```bash
ros2 control list_controllers -c /a1/controller_manager
```

## 已知限制（重要）

1. **A1 不会自己站住**。上游 `legbot_3D_Nav` 靠 RL 策略（`unitree_guide/junior_ctrl`）
   维持平衡，本仓库尚未移植。四足本质是倒立摆，只用关节位置控制时，
   出生后一旦落地失稳就会翻倒（实测会滑行数米后侧卧）。
   `stand_pose.py` 只负责把 12 个关节保持在标准站立角（`hip=0, thigh=0.9, calf=-1.8`），
   **不提供平衡能力**。要真正行走/站立，需要移植 RL 策略或另写步态控制器。
2. **出生高度很敏感**。`z=0.42` 是"腿伸直时足端刚好触地"的高度，
   实测这个高度会因接触求解产生巨大穿透力，把机器人弹飞十几米；
   `z=0.5`（默认）相对温和。调这个参数前先读 `a1_description/README.md` 的说明。
3. **建筑模型在 Gazebo 里的名字是 `Buliding`**（上游拼写错误，少一个 `d`）。
   世界文件引用的是目录名 `model://Building`，两者不一致但能正常工作。
   用 `gz model` 按名查询时请用 `Buliding`：

   ```bash
   gz model -i -m Buliding -w tower
   ```

   世界名是 `tower`（来自 `building.world`）。

## 本包与上游的对应关系

| 上游（ROS 1） | 本包 |
| --- | --- |
| `unitree_gazebo/worlds/Building.world` | `world/building.world`（逐字复制） |
| `unitree_gazebo/models/Building*` | `models/Building*`（逐字复制） |
| `unitree_guide/launch/gazeboSim.launch` | `launch/sim.launch.py` |
