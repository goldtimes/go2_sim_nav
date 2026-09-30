# lightning-lm 定位(online loc) scan-地图错位 — 坐标系调查结论（2026-09-02, 只读）

## 关键事实（带行号）
- scan_undistort_ 永远在 **LIDAR 系(扫尾)**：UndistortPcl 公式 imu_processing.hpp ~L299-311；建图模式只在“消费点”乘外参：
  - PointBodyToWorld laser_mapping.h:124-134（rot*(Ril*p+til)+pos）
  - UI 显示 laser_mapping.cc:209-213 / 329-333（T_ext 乘到副本）
  - 存图 GetGlobalMap laser_mapping.cc:840-849（Twl = kfPose·T_ext → 地图点=Twi·Til·p_l）
  - scan_undistort_ 本体从不被修改。
- 定位链路 **完全无外参**：core/localization/ 下除注释 localization.cpp:241 “Twi with Til, here pose means Twl, thus Til=I” 外无 offset_R/T、无 SetTImuLidar、无 extrinsic。
  - 定位 scan=GetProjCloud(localization.cpp:197; laser_mapping.cc:891-895 直接返回 LIDAR 系 scan_undistort_)
  - ProcessCloud lidar_loc.cc:131 → Align:440 → Localize:822，NDT setInputSource(input):843 / align:844，无任何变换。
  - NDT target=TiledMap 磁盘 chunk(world 系, 含 Til) → NDT 输出位姿 = **Twl**(lidar pose)。
- 但所有“位姿/种子”接口是 **Twi(IMU)**：index.txt start FP=首关键帧 GetOptPose（slam.cc:169, tiled_map.cc:18/60-64）; LO NavState/猜测=Twi（lidar_loc.cc:543-547）; run_loc_online.cc:45 SetInitPose(SE3())=identity。
- res.pose_ = NDT 结果（Align 729 存 current_pose_esti，经 Localize 改写 + 10% 向 LO guess 融合 632）=Twl。recover_pose.txt(lidar_loc.cc:782-788)=Twl。
- r41 Til=[0,-1,0;-1,0,0;0,0,-1]≈180°旋转；mid360 Til=I → go2 正常, r41 错位。
- 次要：GetProjCloud 调 ProjectKFs（laser_mapping.cc:339+,891）把 kf LIDAR 点只用 IMU 相对位姿（无 Til）append 进 scan_undistort_ → NDT/UI 混入 ghost 点。

## 结论
定位模式错位根源 = scan 以 LIDAR 系喂给需要 Twl 的 NDT，但种子/FP/LO/recover 全按 Twi 语义 → 初猜与最优差 Til → NDT 初始化收敛到错误局部最优 → 显示位姿偏 Til → 彩色 scan 与含 Til 灰地图错位。修复方向（未实施）：匹配/显示前给 scan 乘一次 Til（与 PointBodyToWorld 一致），使位姿回到 Twi 语义；或把种子统一成 Twl。见 src/slam/lightning-lm/doc/r41_lio_original_eskf_fixes_20260902.md（建图修复同源）。

## TF 树实现（2026-09-07，路线B，loc_system.cc）
- 目标树 map→odom→base_link→imu_link→lidar_link，路线B：odom=LO(ESKF)帧，map→odom=loc修正。
- lightning loc/LO 位姿均为 **IMU 系(Twi)**；r41 fasterlio.extrinsic(imu→lidar)=单位阵 → lidar≈imu。
- r41 **base_link→imu_link 非单位阵**（最终确认 2026-09-07）：base_link=REP-103(X前Y左Z上)；t=[0.26,0,0]；R=Rz(-90°)*Ry(0°)*Rx(150°) = [[0,-√3/2,-1/2],[-1,0,0],[0,1/2,-√3/2]]（yaml system.extrinsic_base_imu_T/R，行主序全精度）。⚠ 之前误用 Ry(0.52) 会残留 roll（150° 滚转+−90° 偏航没补偿）。
- **Sophus 陷阱**：yaml 旋转矩阵写有限小数(6位)会因 R·Rᵀ≠I 触发 SO3 构造断言("R is not orthogonal")。修复：ArraysToSE3 构造前先四元数归一化正交化（NormalizeRotation），免疫任意精度。
- 关键推导：base 位姿 = IMU位姿 × inv(T_base_imu)；map→odom = loc×LO⁻¹ 为不变式（与 T_base_imu 无关）。TF 树合成 map→lidar = loc×Til，Til=I 时与点云几何一致（验证正确性的判据）。
- 静态：base_link→imu_link=T_base_imu(yaml)，imu_link→lidar_link=fasterlio.extrinsic。动态在 HandleLocTf 广播 map→odom+odom→base；PublishDebugAndRviz 另按点云帧率广播 odom→base。/lightning/odom frame=odom child=base_link pose=LO归算；nav_state frame=map pose=loc归算。


## 2026-09-07 补充：loc 模式 TF 树（路线B）实现结论
- loc_system.h/.cc 已实现 map→odom→base_link→imu_link→lidar_link 发布（r41 esekfom 版）：
  - lightning loc/LO 位姿 = **IMU(=lidar) 系**（r41 imu 内置 lidar，fasterlio.extrinsic=I）。
  - base_link 为真实本体：动态位姿 = IMU 系位姿 × (base_link→imu_link 外参)⁻¹。
  - 静态 TF：base_link→imu_link = base_link→lidar_link（用户实测 t=[0.26,0,0] rpy=[0,0.52,0] → Ry(0.52)，配置键 system.extrinsic_base_imu_T/R）；imu_link→lidar_link=I。
  - map→odom 在 HandleLocTf（PGO 高频输出回调）里合成 = map→base×(odom→base)⁻¹；odom→base 用 LO(ESKF)。
  - /lightning/odom 已改 frame=odom、child=base_link（LO 系）；nav_state 保持 map、位姿取 loc 全局。
- 注意：loc 高频输出位姿经 PoseSmoother（pose_graph/smoother.h）平滑后才进 TF；map→odom 因此含平滑滞后差，属预期。
- 旧记录（ikdtree 版 r41 Til=180°）与当前 esekfom 版（extrinsic=I + base 外参分离）不同，勿混用。


## 2026-09-09 补充：建图侧(slam.cc) TF 树已与定位统一（用户验证通过）
- 建图 SlamSystem 现在也发布 map→odom→base_link→imu_link→lidar_link，与 loc_system 同构（路线B）。
- 静态 TF：base_link→imu_link（system.extrinsic_base_imu_T/R）、imu_link→lidar_link（fasterlio.extrinsic_T/R，r41=I），Init 时由 StaticTransformBroadcaster 发布一次。
- 动态（新私有方法 `SlamSystem::PublishMappingTf(stamp)`，slam.cc ~L556）：每帧点云调一次，发 map→odom + odom→base_link；
  odom=LO(ESKF) 系（lio_->GetState().GetPose() 归算 base），map=回环后全局（lio_->GetOptPose() 归算 base）；
  map→odom = map_base×(odom_base)⁻¹ —— 建图时 map≈odom（仅含回环修正），与定位不变式一致。
- 话题语义也已对齐：/lightning/odom frame=odom child=base_link（LO 系归算）；nav_state frame=map pose=map→base_link；path 记录 map 下 base_link。
- 外参解析在 Init 中「始终执行」（pub_tf_ 关闭也要归算 odom/nav_state），仅 pub_tf_ 时广播静态 TF。默认 r41_rk_slam.yaml（pub_tf/pub_odom=true）无需改配置。
- helpers MakeTf/ArraysToSE3/NormalizeRotation 在 loc_system.cc 与 slam.cc 各有一份（各翻译单元匿名 ns），改动需同步两处。

## 2026-09-10 经典 odom 模型已实施（定位侧，编译通过；待上车验证）
- 目标：odom 原点固定在机器人启动/重定位位置（不再被 SetInitPose 拉到地图绝对坐标）。
- **关键结论（推翻原计划）**：`LidarLoc` 的 GICP 初值 `guess_from_lo = last_abs_pose_(map) × δ_LO(相对增量)`，对 `map=W·odom` 常量变换 W 精确抵消 → **删除 SetInitPose 后主匹配链路天然成立**；PGO 相对边/外推也只用相对量 → pose_graph 无需改；`HandleLocTf` 的 `map→odom=map→base×(odom→base)⁻¹` 本身就是 W，**无需新增 W 成员**。
- 实际改动 3 处：
  1. `localization.cpp::SetExternalPose` 删除 `lio_->SetInitPose(...)`（全仓唯一把 LO 搬进 map 的入口）。
  2. `lidar_loc.cc::InitWithFP` 失败记录 `fp_init_fail_pose_vec_` 改存 `current_dr_pose_`(odom)，与 Align 中该向量的比较口径一致（原先混装 map 的 FP 位姿 → 经典模型下节流失效）。
  3. `loc_system.cc::PublishDebugAndRviz`：未定位（`latest_map_pose_valid_==false`）不再用 LO 顶替 map、**不发 nav_state/path**；odom 发布（`/lightning/odom` + TF odom→base_link）拆出**始终可用**；`GetScanDownWorld` 点云帧标 `"map"→"odom"`。
- 编译：`colcon build --packages-select lightning ...`（**包名是 `lightning`**，非 lightning-lm）通过。
- 遗留（安全）：`localization_result.cc::ToGeoMsg()` frame=map/child=base_link 实为 map→IMU（仅内部消费）；`pgo_impl.cc` rel_pose_/vel_b_ 跨系（无消费者）。详见 doc/loc_classic_odom_plan.md。
- 验证：2026-09-10 用户上车验证 TF 行为正常。
- 补充修复（rviz 丢帧）：`current_scan` 帧标改 odom 后，定位 GOOD 前缺 `map→odom` → rviz 报 "frame 'odom' ... queue is full"。修复：LocSystem 新增 `pending_map_odom_`（SetInitPose 时 = P_base×(odom→base)⁻¹，缺省单位阵），`PublishDebugAndRviz` 在 `!map_valid` 时按点云帧率补发 `map→odom`，保证 TF 树在定位前也完整。