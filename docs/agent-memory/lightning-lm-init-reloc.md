# lightning-lm 运行中初始化/重定位接口（2026-09-07）

## 现有底层（可复用）
- `LidarLoc::SetInitialPose(SE3)`：`loc_inited_=false; initial_pose_set_=true` → 下一帧云在 Align 初始化分支 `InitWithFP(input, initial_pose_)` 重新 NDT 初始化。**已支持运行时重定位**。
- `Localization::SetExternalPose(q,t)`：同时 `lidar_loc_->SetInitialPose` + `lio_->SetInitPose`（LO/ESKF 对齐到地图）。
- `LidarLoc::Init` 读 `recover_pose.txt` 作为 "recover" FP；建图时 `start` FP（首关键帧位姿）写进 `index.txt` `# functional points` 段。
- 弱点：`min_init_confidence_`/`YawSearch` 基本未参与（`Localize` 恒 true）；初始化=种子处一次 NDT，无分数门槛/多帧确认。

## 新增 ROS 入口（loc_system）
- 初值语义 = **base_link 在地图中的位姿**；内部转 IMU：`imu = pose_base * T_base_imu_` 再 `SetExternalPose`（SetInitPose 已统一该语义）。
- 订阅 rviz2 `initialpose`（PoseWithCovarianceStamped，yaml `system.init_pose_topic` 可改默认 "initialpose"）：2D 无 z → `GetReferenceZ()` 自动取高（当前定位高度 → 地图 start FP 高度 → 0）。
- Service `lightning/set_initpose`（srv/SetInitPose.srv，含 `z_valid`：false=自动取高）。
- 反馈：`/lightning/loc_state` std_msgs/Int32（0=IDLE 1=INITIALIZING 2=GOOD 3=FOLLOWING_DR 4=FAIL），来自 `Localization::SetLocStateCallback`（已在 localization.h 启用）。

## 注意
- 改 .srv 后必须重编译生成消息，否则 clangd 报 "No member z_valid"（旧生成头未更新，非源码错）。
- T_base_imu_ 外参解析已从 `if(pub_tf_)` 内移出到 Init 总是执行（初值归算也需要）。
- run_loc_online 启动参数 init_* 亦走 SetInitPose → 现按 base 语义。

## 初始化容错搜索（2026-09-07 已实现，lidar_loc）
- `SearchAndInit(input, seed)`：先直接细 NDT，conf≥min_init_confidence_ 即接受；不足则按 **3×3(xy,±step) × yaw(360°/10°)** 候选做**粗 NDT(5m)** 打分，conf≥`init_search_stop_conf_` **提前退出**；最优候选再做一次细 NDT 精配准；best_conf≥`min_init_confidence_`(r41=0.8) 才接受，否则 REJECTED 保持 INITIALIZING。
- `RunNdt`(静默、无 tgt.pcd/日志，供批量) + `FinishInit`(提交结果) 为新加辅助；InitWithFP 已重构复用 FinishInit。
- 外部初值失败后**不再自动落 FP**（防错锁），0.5s 节流重试；`map_ 空指针/`LoadOnPose+UpdateGlobalMap` 保证 target 就绪。
- yaml(lidar_loc)：init_search_enable/xy_grid/xy_step/yaw_range/yaw_step/stop_conf（均有默认）。
- ⚠ 调参：`min_init_confidence_` 用 pclomp `getTransformationProbability()`(越大越好)；误拒→调低，误收→调高/调 stop_conf。验证看日志 "manual init accepted/rejected best conf="。
- ✅ 2026-09-08 用户初步验证通过：TF 树、/initialpose+service、SearchAndInit、init_selftest 均可工作。

## 确认后生效 + 超时 FAIL（2026-09-08 已实现）
- 旧行为：点击即 `SetExternalPose` 重置 LO → 机器人/TF 立即跳到点击位姿（错误也会跳）。
- 新：`SetExternalPoseDeferred` 只武装 LidarLoc（loc_inited_=false），**不对齐 LO** → 不跳变；`LidarLocProcCloud` 收到 `GOOD` 后 `lio_->SetInitPose(res.pose_)` 确认才生效落位。
- LocSystem 看门狗：`system.init_confirm_timeout`(默认5s) 未 GOOD → loc_state=FAIL(1Hz)，吞 INITIALIZING；GOOD 清除 pending。文档：doc/loc_init_pose_flow.md。
