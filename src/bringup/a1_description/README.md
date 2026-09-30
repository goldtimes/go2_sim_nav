# a1_description

Unitree A1 四足机器人的 ROS 2 Humble 描述包，面向 **Gazebo Classic 11** 仿真。

## 来源

移植自 `/home/gmd/r41_ws/legbot_3D_Nav/src/unitree_guide/unitree_ros/robots/a1_description`
（原工程为 ROS 1 Noetic + Gazebo Classic 11，Apache-2.0；`unitree_ros` 部分为 Unitree Robotics 的 BSD-3-Clause）。

网格（`meshes/*.dae`）、`scan_mode/mid360.csv`、`urdf/a1.urdf` 与原工程一致；
`xacro/` 与 `config/` 有改动，见下。

## 与上游的差异

| 项目 | 上游（ROS 1） | 本包（ROS 2） |
| --- | --- | --- |
| 控制器插件 | `libgazebo_ros_control.so` + `<transmission>` | `libgazebo_ros2_control.so` + `<ros2_control>` |
| 关节控制器 | `unitree_legged_control/UnitreeJointController`（PID） | `position_controllers/JointGroupPositionController` |
| 控制器配置 | `config/robot_control_ros1.yaml`（保留作参考） | `config/a1_ros2_control.yaml` |
| 真值位姿 | `libgazebo_ros_p3d.so`（`<bodyName>`/`<topicName>`） | 同名插件，参数改为 `<body_name>`/`<ros><remapping>` |
| 躯干 / 雷达 IMU | `libgazebo_ros_imu_sensor.so`（`<topicName>`） | 同名插件，改用 `<ros><remapping>~/out:=...</remapping>` |
| 外力接口 | `libgazebo_ros_force.so`（`<bodyName>`） | 同名插件，改用 `<link_name>` |
| MID360 雷达 | `liblivox_laser_simulation.so`（ROS 1 自研） | `libros2_livox.so`（系统包 `ros2_livox_simulation`） |
| 足端接触 | `libunitreeFootContactPlugin.so`（自研） | `libgazebo_ros_bumper.so`（`gazebo_msgs/ContactState`） |
| 足端力可视化 | `libunitreeDrawForcePlugin.so` | 已移除（纯可视化） |
| `<transmission>` 块 | `xacro/transmission.xacro` 每腿生成 | 不再使用，`leg.xacro` 中已去掉调用（文件保留作参考） |

## 话题（默认 `robot_name:=a1`）

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/a1/livox/lidar` | `livox_ros_driver2/CustomMsg` | MID360 点云（非重复扫描，由 `mid360.csv` 决定） |
| `/a1/livox/imu` | `sensor_msgs/Imu` | 雷达内置 IMU |
| `/a1/imu` | `sensor_msgs/Imu` | 躯干 IMU |
| `/a1/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` 发布 |
| `/a1/joint_group_position_controller/commands` | `std_msgs/Float64MultiArray` | 12 关节位置指令 |
| `/a1/ground_truth/base_w` | `nav_msgs/Odometry` | 世界系真值位姿（6 个 p3d 话题之一） |
| `/a1/contact/<FOOT>` | `gazebo_msgs/ContactState` | 足端接触 |
| `/a1/apply_force/trunk` | `geometry_msgs/Wrench` | 向躯干施加外力 |

## 用法

仅在 RViz 里查看模型：

```bash
ros2 launch a1_description description.launch.py
```

完整仿真请用 `a1_sim` 包（世界 + robot_state_publisher + spawn + 控制器）：

```bash
ros2 launch a1_sim sim.launch.py
```

手动检查 xacro 是否可用：

```bash
xacro $(ros2 pkg prefix --share a1_description)/xacro/robot.xacro robot_name:=a1 > /tmp/a1.urdf
check_urdf /tmp/a1.urdf
```

## 站立姿态

上游靠 RL 策略（`unitree_guide/junior_ctrl`）维持站立，本仓库尚未移植该策略。
在仿真里用下面的节点把标准 A1 站立姿态（`hip=0.0, thigh=0.9, calf=-1.8` rad，
取自上游 `State_RL_test.h`）以斜坡方式发给位置控制器，避免机器人一落地就瘫倒：

```bash
ros2 run a1_description stand_pose.py --ros-args -r __ns:=/a1 -p ramp_time:=2.0
```

`a1_sim/launch/sim.launch.py` 默认会带上它（可用 `stand:=false` 关闭）。

## 已知限制

- **PID 未标定**：上游每个关节的 PID（p=100/300、d=5/8）是配合 `UnitreeJointController`
  的，换成位置控制器后需要重新标定，否则站立刚度和抗扰性能与上游不同。
- **没有平衡控制器**：上游靠 RL 策略 `junior_ctrl` 让 A1 站稳/行走，本包只做模型与
  关节控制。用 `scripts/stand_pose.py` 只能把关节摆到站立角，机器人落地后仍会翻倒。
- **网格路径**：URDF 用 `package://a1_description/meshes/*.dae`，sdformat 在解析 URDF 时
  会自动改写成 `model://a1_description/meshes/*.dae`。因此 Gazebo 的 `GAZEBO_MODEL_PATH`
  必须包含 `<install>/share`，本包通过
  `<export><gazebo_ros gazebo_model_path="${prefix}/.."/></export>` 自动提供，
  由 `gazebo_ros` 的 `gzserver.launch.py` 注入。
- **RL 步态未移植**：`junior_ctrl`（ROS 1 C++ + Unitree SDK）不在本包范围内。

## 移植过程中踩到的坑（改动原因，勿轻易回退）

这几条都是实测踩出来的，回退其中任何一条都会让仿真不可用：

1. **固定关节不能随便保留**。sdformat 默认把 fixed joint 的子 link 合并进父 link，
   `trunk`/`imu_link`/`laser_livox`/`*_foot` 都会被合并掉，按名字引用这些 link 的插件
   （p3d、force、相机/雷达传感器）会报 `link does not exist`。
   本包用 `<preserveFixedJoint>true</preserveFixedJoint>` 保留 **imu_joint、
   laser_livox_joint、livox_imu_joint、*_foot_fixed**。
   ⚠️ 但 **不要保留 `floating_base`**：`base` 是上游用来当根节点的 1mm 哑链接，
   自身没有惯性量，保留它会让整个 URDF→SDF 转换失败（模型连一个 link 都不剩）。
   因此 `trunk` 会被合并进 `base`，插件里凡是指向 `trunk` 的引用都改成了 `base`，
   上游的 `p3d_base_trunk` 话题也一并去掉了（frame=trunk 必然不存在）。
2. **`laser_livox` 需要惯性量**。上游该 link 没有 `<inertial>`，无惯性量的 link
   无法作为独立刚体保留。本包补了 0.1kg / 0.0001 的惯性量。
3. **`laser_livox` 的碰撞体不能用 dae**。上游把 10MB 的 `livox_mid360.dae` 同时当
   碰撞体，而该网格的几何坐标范围是 **-32m ~ +40m**（尺度严重异常），
   实测会让机器人一出生就被弹飞几十米。本包改成 `0.1×0.06×0.06` 的小方块
   （与 `ros2_livox_simulation` 的 mid360 写法一致），可视化仍用原网格。
4. **腿部自碰撞要关掉**。保留 `*_foot_fixed` 后脚与小腿成为两个独立 link，
   上游的 `<self_collide>1</self_collide>` 会让相邻 link 互相排斥，本包统一改为 `0`。
5. **接触刚度**：上游 `kp=1000000.0` 作用在 60g 的脚上超出 1ms 步长的稳定范围，
   本包降到 `10000.0`。
6. **ros2_control 参数文件的顶层键必须带命名空间**。`config/a1_ros2_control.yaml`
   顶层用通配符 `/**`；若写成裸的 `controller_manager:`，带命名空间时匹配不上，
   `libgazebo_ros2_control.so` 会报
   `controller manager doesn't have an update_rate parameter` 并停止工作，
   现象是 controller_manager 的所有 service 调用全部超时。
7. **位置环需要显式 PID**：`config/a1_ros2_control.yaml` 里的 `pid_gains.position.*`
   照搬上游 PID；不配的话控制器虽然显示 `active`，但关节使不上劲。
