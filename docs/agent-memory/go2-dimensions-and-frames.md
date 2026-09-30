# Go2 尺寸 / 参考点（2026-09-29 实测，仿真 + URDF + 实时 TF）

## 结论一句话
URDF 表示的 Go2 尺寸是对的（≈官方 0.70×0.31×0.40 m），但**导航里的"车体"是错位的**：
footprint 以 base_link 为中心、而导航位姿是**雷达**（base_link 前 0.22 m），且车体本身
**不对称**（前 +0.33 / 后 −0.41）⇒ 车头余量只剩 1.8 cm、车尾有 5.7 cm 露在 footprint 外。

## 尺寸
- `const.xacro` 的 `trunk_length/width/height = 0.3762/0.0935/0.114` 是 **collision 盒**
  （简化碰撞体），**不是外观**：`trunk.dae`（乘 DAE 场景矩阵后）= **0.460×0.194×0.186**，
  x∈[−0.128,+0.332] ⇒ 车身本来就"前长后短"。
- 整机（站立姿态、实时 TF、已滤掉 0.001 m 的占位 link：camera_face/velodyne_imu/imu_link/
  d435i_imu/base_link）：
  - **visual 0.739 × 0.324 × 0.421**，x[−0.407,+0.332] y[±0.163] z[−0.285,+0.136]
  - **collision 0.656 × 0.309 × 0.393**（腿是细长盒 0.213/0.267，躯干盒偏小）
  ⇒ 与官方 Go2 0.70×0.31×0.40 基本一致。
- 腿：thigh 0.097×0.073×0.278、calf 0.058×0.040×0.267（骨长沿 **z**，向下）、
  hip 盒 0.118×0.076×0.098、foot 0.045×0.040×0.039；hip 关节 x=±0.1934、y=±0.0465。

## 参考点（这是关键缺陷）
- URDF/link 名：`trunk` / `{lf,lh,rf,rh}_{hip,upper_leg,lower_leg,foot}_link` / `velodyne`
  / `camera_d435` / `lidar`。`base_link` ≡ trunk 原点 = 车体中心（x/y 对称）。
- 雷达/IMU：`velodyne_joint` = base_link + (0.22, 0, 0.09)；lightning 配置
  `go2_gazebo_sim.yaml: extrinsic_base_imu_T: [0.22, 0, 0.09]`。
- `/lightning/perception/pose` = **雷达位姿**（`loc_system.cc:691` `T_map_lidar =
  T_map_base·T_base_imu·T_imu_lidar`，`:698` `child_frame_id=lidar_link`）——
  而 perception/pnc_2d 的 `footprint_offset_x`/`footprint.offset_x` 都是 **0**（没补偿）。
- footprint 0.70×0.40 以原点为中心 ⇒ 车头余量 +0.350−0.332 = **1.8 cm**；
  车尾 −0.350 vs 真实 −0.407 ⇒ **后腿有 5.7 cm 在 footprint 之外**。
- 换到**位姿系**（雷达）：车头看着有 0.350−(0.332−0.22)=**0.238 m** 余量（虚假，真实
  只有 1.8 cm）⇒ 真车头可以顶进障碍 0.238 m；车尾 real −0.407−0.22 = **−0.627 m**，
  而 footprint 只到 −0.35 ⇒ **最后 0.28 m 完全没有碰撞保护**。
- 实测坐实：S2 里车在 FOLLOWING 下顶进箱子，净距 −0.119 m。

## 量法（可复用）
- `scripts/measure_go2_mesh_bbox.py`：DAE 顶点包围盒。★★ 必须乘 DAE 里的 `<matrix>`
  场景变换，且这些 go2 的 dae 要按**行主序**解释（按 Collada 的列主序会把 thigh 量成
  "横着 0.278 m"，与同 link 的 collision 盒矛盾）。
- `scripts/measure_go2_extent.py`：从 `/robot1/robot_state_publisher` 取 robot_description +
  实时 TF 合成整机外廓。两个坑：① TF 在 **/robot1/tf**、tf2 监听器写死绝对话题
  `/tf` ⇒ 必须 `--ros-args -r /tf:=/robot1/tf`（节点命名空间无效）；② 占位 link 的
  0.001 m 盒会把车头极值撑到 +0.33，必须过滤。
