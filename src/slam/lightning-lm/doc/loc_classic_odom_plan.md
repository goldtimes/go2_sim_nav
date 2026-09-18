# 定位重定位改「经典 odom 模型」开发计划

> 日期：2026-09-09 整理（次日续接用）
> 相关：`loc_tf_publication.md`、`loc_init_selftest.md`
> 涉及包：`lightning-lm`（r41 配置 `config/r41_rk_slam.yaml`）

---

## 0. 状态摘要

- ✅ 已完成（2026-09-09）：**建图侧 TF 树与定位统一**（`map→odom→base_link→imu_link→lidar_link`），用户已验证编译/运行通过。
- ✅ 已完成并验证（2026-09-10）：**重定位改「经典 odom 模型」** —— 已实施、编译通过、用户上车验证 TF 行为正常（实际改动与原计划的差异见文末「实施记录」）。

---

## 1. 已完成：建图侧 TF 统一（2026-09-09）

### 目标
建图、定位发布同一套 TF：`map→odom→base_link→imu_link→lidar_link`；建图时 `map≈odom`。

### 改动
- `src/core/system/slam.h`
  - include `tf2_ros/static_transform_broadcaster.h` / `transform_stamped.hpp`
  - Options 注释更新；新增成员 `tf_static_broadcaster_`、`SE3 T_base_imu_`、`SE3 T_imu_lidar_`
  - 新增私有方法 `void PublishMappingTf(const builtin_interfaces::msg::Time& stamp);`
- `src/core/system/slam.cc`
  - 匿名命名空间 helpers：`MakeTf` / `NormalizeRotation` / `ArraysToSE3`（注：与 `loc_system.cc` 各有一份，改需两处同步）
  - `Init`（~L239-267）：解析外参 `system.extrinsic_base_imu_T/R`、`fasterlio.extrinsic_T/R`（始终解析）；`pub_tf_` 时 `StaticTransformBroadcaster` 广播 `base_link→imu_link→lidar_link`
  - `SlamSystem::PublishMappingTf`（~L556）：每帧点云调用
    - `odom→base = lio state(odom/IMU) · T_base_imu⁻¹`
    - `map→base = GetOptPose()(map/IMU) · T_base_imu⁻¹`
    - `map→odom = map_base · odom_base⁻¹`（建图≈I，仅含回环修正）
    - 发 `map→odom`+`odom→base` TF；`/lightning/odom` frame=odom child=base_link；`nav_state` frame=map pose=map→base；path 记录 map 下 base_link
  - 两个 `ProcessLidar`（PointCloud2 与 livox）发布段收敛为 `PublishMappingTf(cloud->header.stamp);`

### 语义核对（不变式）
`map→lidar = map→odom · odom→base · base→imu · imu→lidar = (map→imu) · T_imu_lidar`
→ 与建图内部地图点云（含外参）一致，rviz 点云显示不回退。

---

## 2. 待实施：重定位「经典 odom 模型」

### 2.1 目标 / 验收
- TF 树不变：`map → odom → base_link → imu_link → lidar_link`
- `odom` 原点固定在**机器人启动/重定位锚定位置**（地图系 ≈P）：
  - 落位瞬间 `odom→base ≈ I`（LO 连续积分、不瞬移）
  - `map→odom` 初始 ≈ `P_base`，此后仅随 loc 修正缓慢漂移（吸收 LO 漂移）
- rviz：odom 坐标系原点显示在机器人身上；点 `initialpose` 机器人**不瞬移/不贴地图零点**
- 话题语义不变：`/lightning/odom`(odom→base, LO)、`/lightning/nav_state`(map→base, loc)、`/lightning/path`

### 2.2 根因（已核实）
`Localization::SetExternalPose`（`src/core/localization/localization.cpp:342`）同时：
1. `lidar_loc_->SetInitialPose(P)` —— loc 种子，正确；
2. `lio_->SetInitPose(P)`（`src/core/lio/laser_mapping.cc:891`）—— **把 ESKF 状态直接覆盖成地图绝对坐标** → LO 的 odom 世界被标定成 map → 落位后 `map→odom≈I`，odom 原点≈map 原点（非经典）。

当初用 `SetInitPose` 的原因：`LidarLoc`/`PGO` 内部假设“DR/LO 与地图同系”（LO 作 GICP 高频初猜 + DR）。

### 2.3 方案：引入 W = map→odom 基准，LIO 永不瞬移
| 层 | 参考系 | 职责 |
|---|---|---|
| `LaserMapping`(LO/ESKF) | **odom** | 从自身原点连续积分；只做 DR/预处理；不再被 SetInitPose 搬走 |
| `LidarLoc`/`PGO`/GICP | **map** | target=地图；结果 map→base；内部保持“同 map 系”假设 |
| **W = map→odom**（新增） | 衔接 | `W(t) = (map→base)_loc · (odom→base)_LO⁻¹`；分发 DR 前 ×W 转 map 系 |

核心思想：**把“改 LO 状态”换成“分发 DR 时乘 W”** → `LidarLoc`/`PGO` 内部几乎零改动（仍只见 map 系 DR）。`loc_system::HandleLocTf` 现有公式 `map→odom=loc×LO⁻¹` 天然就是 W，保持即可。

### 2.4 改动清单
1. `src/core/localization/localization.h` / `.cpp`（核心）
   - 新增 `SE3 map_odom_`(=W) + 锁 + `bool map_odom_valid_` + setter/getter
   - `SetExternalPose`：去掉 `lio_->SetInitPose`；保留 `lidar_loc_->SetInitialPose(P)`；设 W 初值 `P_base·(odom→base_now)⁻¹`（LO 未就绪则 pending）
   - `ProcessIMUMsg` 分发 DR 给 `lidar_loc_->ProcessDR` / `pgo_->ProcessDR` 前，把 dr_state 位姿 ×W 转 map 系
   - `LidarLocProcCloud`：首次 GOOD 后回填 `W = res.pose_·(odom→base)⁻¹`，低通/去抖；之后限幅微调
   - `GetState()` 保持 odom 系（供 loc_system 合成）
2. `src/core/system/loc_system.cc` / `.h`
   - `HandleLocTf`/`PublishDebugAndRviz` 公式不变
   - “未 GOOD”回退：`map_imu` 现回退 LO(odom) 值 → 改为回退 `W·odom`（给 P 后 W 即可用，nav_state/TF 从一开始正确）
3. `src/core/lio/laser_mapping.*`
   - `SetInitPose` 保留接口、定位不再调用（加注释说明已被 W 方案取代）；建图模式不受影响
4. 审计（改动前必查）：
   - `LidarLoc::Align` 内所有用 LO/DR 处（`guess_from_lo`、`current_dr_pose_` 与地图 FP 比较、`recover_pose`、`SetInitRltState`、init 分支）在收到 map 系 DR/seed 后是否自洽
   - `PGO` 输入（loc 结果 + DR）参考系假设
   - 离线入口 `run_loc_offline.cc` 是否走同一 `Localization`（若走则同受益/同改）

### 2.5 配置
- 无需新增键（沿用 `system.extrinsic_base_imu_*`、`fasterlio.extrinsic_*`、`lidar_loc.*`）

### 2.6 验证
- 在线定位（r41 地图）：rviz 看 odom 原点在机器人上；点 initialpose 不瞬移、`map→odom`≈P；长时间静止 map→odom 仅缓漂；`tf tree` 无跳变；话题与 TF 一致
- 回归：建图模式 TF/话题/点云不回退；`run_loc_offline` 轨迹一致

### 2.7 风险与缓解
- W 回填跳变 → 首次只初始化一次 + 限幅/低通 + 日志打印
- 内部“LO≡map”残留假设 → 按 2.4-4 逐点审计
- 停车/低运动 → DR 停顿时 guess 恒定，W 限幅
- 建图模式共享 `LaserMapping` → 改动只在定位分发层，建图零影响

### 2.8 里程碑
- **M0**：审计（2.4-4），确认 `Localization` 内所有 LO 分发点与 PGO/LidarLoc 假设
- **M1**：`Localization` 加 W、去 SetInitPose、DR 分发 ×W、GOOD 回填
- **M2**：`loc_system` 未 GOOD 回退适配 + 离线入口复核
- **M3**：验证（2.6）与建图回归

---

## 3. 实施记录（2026-09-10）

### 3.1 关键审计结论（推翻了原计划的“必须引入 W 层”）
- **`LidarLoc` 的 GICP 初值只用 LO 的相对增量**：`lidar_loc.cc` 的 `guess_from_lo = last_abs_pose_(map) × (last_lo_pose_⁻¹ · current_lo_pose_)`；对形如 `map = W·odom` 的常量变换 **W 精确抵消**，因此 **删除 `SetInitPose` 后主匹配链路天然成立**。
- `PGO` 的相对边（`pgo_impl.cc` `pre⁻¹·cur`）与 `ExtrapolateLocResult` 同样只用相对量/增量 → **`pose_graph` 无需改动**。
- `HandleLocTf` 现有公式 `map→odom = map→base × (odom→base)⁻¹` 本身就是 W，**无需新增 W 成员**。
- 结论：只需修掉“少量把 LO/DR 当绝对值用、与 map 混用”的点即可（原计划 4.1 的 W 成员/`ToMapFrame` 分发改造 **不必要**）。

### 3.2 实际改动
1. `core/localization/localization.cpp` —— `SetExternalPose()`：**删除 `lio_->SetInitPose(SE3(q,t))`**（保留 `lidar_loc_->SetInitialPose`）。这是全仓唯一把 LO 搬到地图绝对坐标的入口；LO 从此保持 odom 系连续积分。
2. `core/localization/lidar_loc/lidar_loc.cc` —— `InitWithFP()` 失败记录统一参考系：`fp_init_fail_pose_vec_.emplace_back(fp_pose)`(map) → **改存 `current_dr_pose_`(odom)**，与 `Align()` 中该向量的比较口径（`current_dr_pose_`）一致，避免经典模型下 W≠I 造成跨系相减、FP 重试节流失效。
3. `core/system/loc_system.cc` —— `PublishDebugAndRviz()`：
   - 删除“未就绪时用 LO(odom) 顶替 map”的回退；**未定位成功（`map_valid==false`）不发 `nav_state`/`path`**；
   - `odom` 发布（`/lightning/odom`、TF `odom→base_link`）拆分出来，**始终可用**（只依赖 LO）；
   - `GetScanDownWorld()` 的点云帧标 **`"map"` → `"odom"`**（该点云由 ESKF 世界系生成，rviz 经 TF 自动映射到地图下——注意此改动要求 `map→odom` 始终存在，见第 4 条）。
4. `core/system/loc_system.cc` / `.h` —— **未定位阶段补全 `map→odom`（rviz 丢帧修复，见 3.4）**：
   - 新增成员 `SE3 pending_map_odom_`（缺省单位阵）；
   - `SetInitPose()` 里由「用户给定初始位姿 P」与当时 LO 推出 `pending_map_odom_ = P_base × (odom→base)⁻¹`；
   - `PublishDebugAndRviz()` 在 `!map_valid` 时按点云帧率广播 `map→odom = pending_map_odom_`（有初值 = 点选位姿推出的过渡值，无初值 = 单位阵），保证定位 GOOD 前 TF 树完整。

### 3.3 编译验证
- `colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release` → **通过**（2026-09-10，仅既有 warning；包名为 `lightning`，不是 `lightning-lm`）。

### 3.4 上车验证（2026-09-10 通过）
- ✅ 在线定位（r41 地图 + rviz2）：TF 行为正常（用户确认）。
- 验证要点：
  - `odom` 坐标系原点在机器人身上（不再贴地图零点）；
  - 点 rviz2 `initialpose`（地图中 P）：机器人**不瞬移**，`map→odom ≈ P_base`，之后仅缓漂；
  - `/lightning/odom`：`frame=odom, child=base_link`；`/lightning/nav_state`：**只在定位 GOOD 后出现**（`frame=map`）；
  - `ros2 run tf2_tools view_frames`：树闭合、无跳变；
  - `lightning/current_scan` 叠加 `/lightning/global_map` 不再错位。
- **踩坑记录（已修）**：把 `current_scan` 帧标改成 `odom` 后，rviz 报
  `Message Filter dropping message: frame 'odom' ... queue is full`。
  原因：定位 GOOD 前 `map→odom` 不存在，rviz（Fixed Frame=map）无法变换 `frame=odom` 的消息 → 队列满丢弃。
  修复见 3.2 第 4 条（未定位阶段用 pending/单位阵补全 `map→odom`）；重新编译后验证正常。
  - 排查命令：`ros2 run tf2_ros tf2_echo map odom`、`ros2 run tf2_tools view_frames`。
- 回归：`run_loc_offline`（`loc::Localization`，走 FP 自动初始化，不受 SetInitialPose 改动影响）轨迹与改动前一致；建图模式（`run_slam_online`）不回退。

### 3.5 已知遗留（评估为安全，本次未改）
- `localization_result.cc::ToGeoMsg()` 的 `frame_id="map" / child_frame_id="base_link"` 实际内容是 map→**IMU**（差一个 `T_base_imu`）；仅回调内部消费、`loc_system` 已按 IMU 处理 → 暂不动（若将来对外暴露该 TF 需修正）。
- `pgo_impl.cc` 的 `result.rel_pose_/vel_b_` 把 odom 系量塞进 map 结果包（当前**无消费者**）。
- `lidar_loc.cc` 中被注释的“把 z 拉平到 `map_height_`”逻辑（911-913/1173/1197）若恢复，需注意其对 LO/DR 绝对高度的假设。
