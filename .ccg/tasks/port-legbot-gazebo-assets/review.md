# 交付记录：legbot_3D_Nav Gazebo 资产移植

路线：**Track A（Gazebo Classic 11 + ROS 2 Humble）**，用户已确认；世界只取 Building；
55MB 资产进 git，不用 LFS；双模型审查工具本机不可用，审查为本地完成。

## 交付物

| 包 | 路径 | 内容 |
| --- | --- | --- |
| `a1_description` | `src/bringup/a1_description` | A1 xacro/meshes/config/scan_mode + `stand_pose.py` |
| `a1_sim` | `src/bringup/a1_sim` | `world/building.world` + `models/Building*` + `launch/sim.launch.py` |

## 已实测通过（Gazebo 11.10.2 + ROS 2 Humble）

- `colcon build --packages-select a1_description a1_sim` 成功
- `xacro` 展开 + `check_urdf` 通过（25 link / 36 joint 的 URDF）
- `Building.world` 加载无 missing model 报错，`Buliding` 模型静态注册成功
- 转换后模型 20 个 link，`base`/`imu_link`/`laser_livox`/`livox_imu_link`/`*_foot` 均在
- 全部 Gazebo 插件节点启动且 **无 "link does not exist" 报错**
- `joint_state_broadcaster`、`joint_group_position_controller` 均 **active**
- `/a1/joint_states` 有数据；`/a1/imu` 有数据

## 未完成 / 已知问题

1. **A1 不能自主站立甚至翻倒**。四足是倒立摆，上游靠 RL 策略维持平衡，
   本仓库未移植 `junior_ctrl`。位置控制只能保持关节角度，无法抗扰。
   实测：出生后翻倒并侧滑数米（干净环境下复现）。
   下一步选项：① 移植 `junior_ctrl`；② 用 `quadropted_controller` 步态重调；
   ③ 只做模型/世界验证，动力学交给后续任务。
2. **出生高度敏感**：`z=0.42`（伸直腿足端刚好触地）会因接触穿透被弹飞十几米；
   默认已改为 `z=0.5`。
3. **雷达点云帧**：`libros2_livox.so` 不读 `<frame_name>`，`CustomMsg.header.frame_id`
   需要用户侧确认（是否等于 `laser_livox`）。
4. 未做：`factory` 场景（缺 8 个外部模型，需联网）、RL 步态、导航链联调。

## 过程中的环境教训

- 沙箱会阻止 Gazebo 枚举网卡（`Unable to get local interface addresses`），
  跑仿真需要 `require_escalated`。
- `pkill -f gzserver` 会匹配到执行它的 shell 自身；清理必须用 `pkill -x gzserver`
  并校验 `pgrep -x gzserver` 归零，否则残留的旧 gzserver 会让后续测量全部失真
  （曾因此误判"机器人被弹飞几百米"）。
