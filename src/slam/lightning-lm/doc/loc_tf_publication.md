# lightning-lm 定位 TF 发布（map→odom→base_link→imu_link→lidar_link）

> 2026-09-07 实现。在线定位（`run_loc_online`）按 **路线 B（分离式）** 发布完整 TF 树，并统一 `/lightning/odom`、`nav_state` 帧语义。

## 1. 目标与方案

### 目标 TF 树

```mermaid
flowchart LR
    map["map<br/>全局固定"] -->|"map→odom 动态<br/>= loc·LO⁻¹（不变式）"| odom["odom<br/>LO(ESKF) 里程计帧"]
    odom -->|"odom→base_link 动态<br/>= LO位姿·T_base_imu⁻¹"| base["base_link<br/>机器人本体(REP-103)"]
    base -->|"base→imu 静态<br/>T_base_imu"| imu["imu_link"]
    imu -->|"imu→lidar 静态<br/>= fasterlio.extrinsic(单位阵)"| lidar["lidar_link"]
```

### 路线 B 要点
- `odom` = LO(ESKF) 里程计帧：`odom→base_link` 连续、平滑、无跳变，由 LO 位姿归算到本体。
- `map→odom` = loc 对 odom 的慢修正；消费端合成 `map→base = map→odom · odom→base` 与定位平滑输出一致。
- 传感器帧（`imu_link`/`lidar_link`）用静态 TF 挂在 `base_link` 下。

## 2. 帧语义与坐标系（r41）

| 帧 | 含义 | 说明 |
| --- | --- | --- |
| `map` | 全局地图系 | lightning loc 输出所在的固定系 |
| `odom` | LO(ESKF) 里程计起始系 | LO 位姿所在系 |
| `base_link` | 机器人本体 | REP-103：X=前、Y=左、Z=上 |
| `imu_link` | IMU 系 | lightning 内部状态位姿语义 = **IMU 系(Twi)** |
| `lidar_link` | 雷达系 | r41 中与 imu 重合（`fasterlio.extrinsic`=单位阵） |

> 关键：lightning 的 loc/LO 位姿都是 **IMU 系**。要在树里体现真实 `base_link`（本体），必须用 IMU 位姿左乘 `imu→base = T_base_imu⁻¹` 归算；雷达倾斜/滚转安装产生的姿态全部由 `T_base_imu` 描述并在归算中消除。

### r41 真实外参（`config/r41_rk_slam.yaml` `system:` 段）
```yaml
extrinsic_base_imu_T: [0.26, 0, 0]                                  # imu 在 base 前方 0.26m
extrinsic_base_imu_R: [0.0, -0.8660254037844386, -0.5, -1.0, 0.0, 0.0, 0.0, 0.5, -0.8660254037844386]  # 行主序
```
即 $T_{base}^{imu}: \ t=[0.26,0,0],\ R = R_z(-90°)\,R_y(0°)\,R_x(150°)$，等价于矩阵
$$
\begin{bmatrix} 0 & -\tfrac{\sqrt3}{2} & -\tfrac12 \\ -1 & 0 & 0 \\ 0 & \tfrac12 & -\tfrac{\sqrt3}{2} \end{bmatrix}
$$
`imu_link→lidar_link` 复用 `fasterlio.extrinsic_T/R`（r41 为单位阵），保证与 LIO 内部一致。

## 3. 关键推导

约定：$T_{odom}^{imu}$=LO(ESKF) 位姿、$T_{map}^{imu}$=loc 平滑全局位姿（两者都是 lightning 输出的 IMU 系位姿）。

- **本体归算**（imu→base = $T_{base}^{imu}$ 的逆）：
$$T_{odom}^{base}=T_{odom}^{imu}\cdot (T_{base}^{imu})^{-1},\qquad
T_{map}^{base}=T_{map}^{imu}\cdot (T_{base}^{imu})^{-1}$$
- **map→odom（不变式，与 $T_{base}^{imu}$ 无关）**：
$$T_{map}^{odom}=T_{map}^{base}\cdot (T_{odom}^{base})^{-1}=T_{map}^{imu}\cdot (T_{odom}^{imu})^{-1}=loc\cdot LO^{-1}$$
- **一致性判据**（树合成 = lightning 真实点云几何）：
$$T_{map}^{lidar}=T_{map}^{odom}\,T_{odom}^{base}\,T_{base}^{imu}\,T_{imu}^{lidar}
= loc\cdot T_{imu}^{lidar}$$
  r41 中 $T_{imu}^{lidar}=I$，故 `map→lidar_link` 必等于 loc（IMU）位姿，与 lightning 发布在 `map` 下的点云严格重合。

> 推论：无论 $T_{base}^{imu}$ 取什么值，`imu_link`/`lidar_link` 在树中位置恒定且正确；只有 `base_link` 的姿态/位置取决于该外参。因此“机器人不滚转但 base_link 有 roll”即外参 R 未补偿全（曾误用 `Ry(0.52)` 只补偿俯仰，残留 150° 滚转 + −90° 偏航）。

## 4. 代码改动

改动集中在 ROS 侧包装层 `src/core/system/loc_system.h/.cc`，未动核心定位算法。

### `loc_system.h`
- 新增 `tf2_ros::StaticTransformBroadcaster tf_static_broadcaster_`
- 新增 `void HandleLocTf(const geometry_msgs::msg::TransformStamped&)` 声明
- 成员：`SE3 T_base_imu_`、`SE3 T_imu_lidar_`；loc 全局位姿缓存 `latest_map_pose_` + `latest_map_pose_valid_` + `std::mutex map_pose_mutex_`

### `loc_system.cc`
- 匿名空间工具：`MakeTf`（SE3→TransformStamped）、`TfToSE3`、`NormalizeRotation`（正交化）、`ArraysToSE3`
- `Init()`：
  - `SetTFCallback` 改为始终缓存 loc 全局位姿（供 nav_state/odom 语义统一），`pub_tf_` 时构造动态+静态广播器；
  - 解析 `system.extrinsic_base_imu_T/R` 与 `fasterlio.extrinsic_T/R`；广播静态 `base_link→imu_link`、`imu_link→lidar_link`
- `HandleLocTf()`（动态 TF，loc 高频回调触发）：
  ```
  T_odom_base = LO位姿 · T_base_imu⁻¹
  T_map_base  = loc位姿 · T_base_imu⁻¹
  T_map_odom  = T_map_base · (T_odom_base)⁻¹     // 不变式
  broadcast: map→odom, odom→base_link
  ```
- `PublishDebugAndRviz()`（语义统一）：
  - `nav_state`：frame=`map`，位姿=全局 `map→base_link`（loc 归算，未就绪回退 LO）
  - `/lightning/odom`：frame=`odom`、child=`base_link`，位姿=LO 归算（与 TF `odom→base_link` 一致）
  - 另按点云帧率广播 `odom→base_link`，保证定位就绪前 odom 帧可查询

### 平滑位姿说明
loc 高频输出（`pgo.cc::PubResult`）经 `PoseSmoother`（`pose_graph/smoother.h`，DR 预测 + 一阶低通、>2m/5m 自适应收敛）平滑后才进入 `loc_result_` → TF 回调。因此 `map→base` 是平滑全局位姿，`odom→base` 是原始 LO——`map→odom` 自然吸收平滑滞后与修正，符合路线 B 预期。

## 5. Sophus 精度陷阱（重要）

- yaml 旋转矩阵若只写 6 位小数，`R·Rᵀ≠I`、`det≠1`，Sophus `SO3(Matrix3d)` 构造会断言 `"R is not orthogonal"`。
- 修复：`ArraysToSE3` 构造 `SO3` 前先 `NormalizeRotation`（四元数归一化再还原），免疫任意精度；同时 yaml 建议写全精度值。

## 6. 验证

```bash
# 编译（三个同名 lightning 包，务必 --base-paths 限定）
colcon build --base-paths src/slam/lightning-lm --packages-select lightning

# 运行
ros2 launch lightning r41_online_loc.launch.py
ros2 bag play <数据包>

# TF 检查
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link    # 机器人静止放平时 rpy≈0（尤其 roll）
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo imu_link lidar_link
```

**最终判据**：`config/showbodypc.rviz` 叠加当前扫描/全局地图，树合成 `map→lidar_link` 应与 lightning 发布的 `map` 系点云严格重合；静止时 `base_link` 无伪 roll。

## 7. 注意事项
- 仓库存在 `lightning-lm`、`lightning-lm-wmh`、`lightning-lm-gx` 三个同名 `lightning` 包，编译需用 `--base-paths` 限定，避免冲突。
- `T_base_imu` 的方向定义：TF 静态消息 = “`imu_link` 在 `base_link` 系中的位姿”（与用户给出的 base→imu 矩阵一致）。
- 若改外参：yaml 数值随意（代码会正交化），但**物理语义**（轴向/绕轴/符号）变了要同步核对归算方向。
