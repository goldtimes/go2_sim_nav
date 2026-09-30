# perception 感知模块独立化（2026-09）

## 结构
- 感知包 = `src/pnc/perception/`，**包名仍为 `plan_env`**，include 路径 `plan_env/grid_map.h` 不变
- **完整说明文档：`src/pnc/perception/doc/perception_gridmap.md`**（原理/接口/参数/坑/验证）
- 原 `src/pnc/SCAN-Planner-Ros2/src/planner/plan_env/` 已加 `COLCON_IGNORE`（同名包不能并存）
- `grid_map.cpp` / `raycast.cpp` 零改动；只新增 `src/perception_node.cpp`、`config/perception.yaml`、`launch/perception.launch.py`
- 节点名 `perception_node`，只做 `gm->initMap(this)` + spin

## 接口契约（上游 lightning）
- `lightning/perception/cloud` — PointCloud2, `frame_id=map`（`GetScanDownWorld()` × `T_map_odom`）
- `lightning/perception/pose` — Odometry, `map → lidar_link`（`T_map_base*T_base_imu_*T_imu_lidar_`）
- 由 `system.enable_perception_pub` 开关，`SensorDataQoS`，每帧雷达发，`map_valid` 门控
- launch 参数写进 `grid_map.topic_*` 参数（见下节），默认生效名仍是相对名 `cloud` / `sensor_pose`
- 必须配 `cloud_is_world: true` + `need_extrinsic: false`；配错**不报错只静默偏掉**（已加启动自检）

## 输入话题配置（2026-09 改：已全部参数化）
- 订阅端原来全是硬编码相对名，现已改为**参数**（默认值 = 原来的相对名，行为不变）：
  `grid_map.topic_cloud`(cloud) / `topic_pose`(sensor_pose) / `topic_depth`(depth) /
  `topic_body_pose`(body_pose) / `topic_map_state`(lightning/map_state)
- 优先级：**launch 参数 > yaml 参数 > 代码默认值**；`-r` remap 作用在“参数里写的那个名字”上
  （所以改了 yaml 里的名字后，原来指向旧名的 remap 会静默失效 —— 别再两套混用）
- `perception.launch.py` 的 4 个 arg（cloud_topic/pose_topic/body_pose_topic/map_state_topic）
  现在写的是**参数**，不再是 remap（改动前只 remap 了 cloud/sensor_pose，且注释错写 body_pose 不订阅）
- 启动时会打 `configured input topics: cloud='...' sensor_pose='...'`，便于核对实际配置
- 发布端：2D 三个话题是参数（`topic_2d_*`）；`grid_map/occupancy` 等仍是硬编码相对名（内部话题）
- 验证方法：不开 remap、只用 `-p grid_map.topic_cloud:=/test/cloud` 也要能通（实测 13 帧 2D 构建）；
  map_state 参数用 `ros2 topic pub --qos-durability transient_local` 打一遍看是否清图

## 帧率改造（2026-09-20 完成，1+2 都落地）
- **① 占据/膨胀体素索引**：`md_.occ_idx_list_/flag_` + `infl_idx_list_/flag_`（`idxNote`/`idxDrop`/
  `compact_idx`，列表 >20000 条时原地压缩坟墓）。维护点：`applyOccupancyUpdate`、
  `resetCellByAddress`、滑窗清图、`resetAllMapData`。`publishMap`/`publishMapInflate` 改为
  只遍历索引（O(占据数)~几千）而不是全图（O(256k)）。`verify_occ_idx:=true` 有暴力全扫自检
  （实测运动+滑窗下 0 漏记：占据 145 / 膨胀 1224 格，列表 6847/21395 条）。
- **② 地图 10x10x5 → 8x8x4 m**（res0.10 下 256k 体素）；`local_update_range` 仍是窗口一半。
- **定时器/节流参数化**：`vis_interval: 0.10`（原硬编码 50ms=20Hz）、
  `pub_map_interval: 0.25`（3D 点云单独节流）、`pub_2d_interval: 0.0`（交给 vis 节奏 → 实测 10 Hz；
  原来 0.1 与 vis 0.1 撞出跳拍变成 6.6 Hz）。
- **每帧耗时日志**（`show_occ_time: true`）：
  `帧耗时: 总 39.7 ms = （点云回调 2.6 + 3D融合 38.4 + 2D 1.3）ms | 点数 6000 | 帧间隔 100.0 ms（10.00 Hz）`
- **实测（合成负载 6000 点/帧 @10Hz）**：纯感知 **10.08 Hz**；带 3D+2D 订阅（模拟 RViz）
  **仍 10.08 Hz**（改造前同样条件下会掉到 4.58 Hz）；2D 栅格输出 10.10 Hz；3D 点云 3.40 Hz。
- **两个 footprint bug（一并修掉）**：
  1) `applyFootprintClear` 的扫描包围盒用了"未含半格外扩"的外接圆 → 斜着转时漏抹边缘格 →
     新增 `footprintCircumradius()`（含 `+2m` 外扩）并在 `applyFootprintClear`/重扫修正里都用它；
  2) **mask 的副作用**：机器人移动后，上一帧被抹成"空闲"的格子不再被抹、却没人重扫 →
     永久假空闲（自检表现："期望 100 实际 0" + "本帧未重扫"）。修法：每帧把**上一帧**
     footprint 覆盖的列全部标脏（`fp_prev_`，几十格）。`fp_cur_/fp_prev_` 由
     `updateFootprintPoseCache()` 在 `build2DLayer` 开头维护。实测运动+滑窗+mask 下
     2D 自检 **0 失败**（修前每帧 1 格）。
- ⚠ **/tmp 会被意外关机清空**：测试脚本（drive_map_test.py 等）丢了会导致命令静默失败
  （输出重定向到 /dev/null 时完全看不出）。跑测试前先 `ls /tmp/*.py`。

## 帧率修复（2026-09-20，已落地 ①+②）
- 配置改为 `resolution: 0.10` + `max_ray_length: 2.0`，并把 `show_occ_time: true`
  （每帧打一行耗时，见下）。
- **实测结果（探针，真实点云 3263 点/帧）**：

  | 场景 | 帧率 | 说明 |
  |---|---|---|
  | 修复前（res0.05 / ray5 / 有订阅者） | **0.50 Hz** | 帧 1854 ms |
  | 修复后 **纯感知**（无订阅者） | **9.43 Hz** | 帧 45~51 ms，其中 3D 融合 45~51、点云回调 ~3、2D <0.1 |
  | 修复后 + RViz 订阅 3D/2D 可视化 | **4.58 Hz** | 帧间隔 219 ms ← **可视化发布成了新瓶颈** |
  | 修复后 + 只订阅 2D 栅格 | ~7 Hz | 受"50ms 定时器 + 100ms 节流"量化限制 |

- **新发现：可视化发布（`publishMap`/`publishMapInflate`）每次要全图扫体素**
  （res0.10 时 100x100x50=50 万体素；res0.05 时是 400 万）→ 单次 visCallback ~200 ms。
  两者都已做"无订阅者直接 return"，所以**不接 RViz 时几乎免费**；一旦 RViz 挂上
  3D 占用点云就立刻吃掉一半以上算力。visCallback 是 50ms 定时器（20 Hz）——即
  **20 Hz 扫全图只为发点云**，非常浪费，建议加发布节流参数（如 5 Hz）。
- 每帧耗时日志格式（`show_occ_time: true` 时）：
  `[GridMap] 帧耗时: 总 45.5 ms = （点云回调 3.1 + 3D融合 45.5 + 2D 0.0）ms | 点数 3269 | 帧间隔 100.0 ms（10.00 Hz）`
- ⚠ 诊断时注意：**"有/无订阅者"会让测量差一倍以上**（可视化成本随订阅出现）。
  做 A/B 时必须固定订阅状态，否则会把可视化成本算到感知头上（我第一轮就踩了）。

## 真实系统帧率低的归因（2026-09-20，实测）
- 症状：`ros2 topic hz /grid_map/occupancy_2d` = **0.5 Hz**（间隔 1.74 s）。
  输入 `lightning/perception/cloud` = **9.83 Hz / 3263 点每帧**（≈32 k点/s，不大）。
  节点 CPU **101%**（单线程 rclcpp::spin 打满）→ 每处理一帧 1.76 s，10 Hz 点云 18 帧只吃 1 帧。
  两个输出（3D 与 2D）完全同拍 → 卡在同一个回调链上。
- 拆解（用 `/**:` 通配键的探针参数 + 独立 `__ns:=/probe`，见下"探针坑"）：

  | 配置 | 帧间隔 | 变化 |
  |---|---|---|
  | 线上同参数（res0.05 / ray5m / inflate r0.25 z±0.1） | 1854 ms | — |
  | **关掉 2D 层** | 1830 ms | ≈0 → **2D 层不是瓶颈**（2D 构建仅 4.4 ms = 0.24%） |
  | inflate 几乎关掉（r0.25→0.05, z→0） | 1827 ms | ≈0 → **膨胀不是瓶颈** |
  | `max_ray_length` 5→1 | 1133 ms | **-39%**（射线步进） |
  | **`resolution` 0.05→0.10** | **313 ms** | **-83%（5.9×）← 最大杠杆** |
  | inflate关 + ray 2 m | 1233 ms | -33% |

- 结论：瓶颈全在 **3D 融合/raycast**。每步（100 体素/5m 射线）≈5.7 µs，本身偏慢，还有代码级空间。
  配置层最有效的是**分辨率**（体素数 ∝1/res³ 且步数 ∝1/res，双降）。
- 影响：0.5 Hz ⇒ 1 m/s 行进时每 ~2 m 才更新一次 2D 图，对 2D 避障不可用。
- ⚠ **探针坑**：参数文件是**按节点全名匹配**的。加 `-r __ns:=/probe` 后节点全名变成
  `/probe/perception_node`，yaml 里的键 `perception_node:` 就不匹配了 → **所有参数静默变代码默认值**
  （日志表现为 resolution=-1、enable_2d=false、need_extrinsic=true）。做法：
  `sed 's/^perception_node:/\/**:/' config/perception.yaml > /tmp/probe.yaml` 用通配键。
- 另注：点云类话题是 BEST_EFFORT，`ros2 topic hz` 默认 QoS 可能收不到；用带
  `ReliabilityPolicy.BEST_EFFORT` 的脚本量（`/tmp/rate_check.py`）。

## 2D 层 footprint 清除（2026-09 新增，**推荐**；默认关）
- 思路（用户提的，也是 Nav2 local costmap 的做法）：每帧把机器人自身占的那块矩形从
  2D 图（占据层 + 膨胀层）上抹掉 → "机器人脚下永远不是障碍"是**几何保证**，与自车点云
  从哪来无关，也不需要配传感器系形状/外参。
- 参数：`footprint_clear_enable` / `footprint_length`(机体系 x 前后) / `footprint_width`(y 左右) /
  `footprint_offset_x|y`(矩形中心相对传感器位置) / `footprint_yaw_offset_deg` /
  `footprint_clear_as_unknown`(false=抹成空闲 0)。
- 朝向：取 `sensor_pose` 的 yaw（= 机体朝向 ⊗ 安装角）。**不需要 body_pose** ——
  lightning 的 `lightning/perception/pose` = `T_map_base*T_base_imu*T_imu_lidar`，机体朝向已含在内。
  （`body_pose` 只有 3D 规划器自己的仿真器在发，见 pnc_3d/go2_kinematic_sim.cpp。）
- 实现：`applyFootprintClear()` 在 `build2DLayer()` 末尾调用（增量路径被重扫的列也会重抹）；
  `verify2DLayer()` 必须用同一个 `inFootprint()` 算期望值，否则自检会把正确处理报成失败。
- ⚠ **半格坑（实测踩到）**：占据格是"包含机体点的格子"，格心最多比真实几何外扩半格。
  若判定用"格心在矩形内"，按真实尺寸配的 mask 会沿边缘漏一圈 —— 实测 0.30m 宽机体 +
  0.35m 宽 mask：边缘格心正好是 0.175，浮点上 `3.5*0.05 > 0.175` → **一格都没清掉**
  （`changed=0`，极难发现）。修法：判定按"格与矩形相交"= 矩形外扩半格。
- 实测（静止 + 注入 0.9x0.3x0.6m 机体点云，mask 1.0x0.40）：
  mask 关 → 原点 0.55m 内占据 **24** 格、全图 105；mask 开 yaw=0/45/90 → **0**、全图 **81**
  （= 无自车点的基线，说明场景其余部分没被误抹）；开 `verify_2d` 自检 0 失败。
  机体(0.9) 比 mask(0.70) 长时 → 剩 4 格（两端各漏一点）：**说明尺寸要按真实机体填**。
- 可视化：`pub_footprint_viz`(true) + `topic_footprint_viz`("grid_map/footprint")，
  发 MarkerArray（[0] LINE_STRIP 绿色轮廓 + [1] CUBE 半透明填充），画在 2D 栅格同一高度
  （带中点，2D 未建好时退化为雷达高度）。RViz 加 MarkerArray 显示项即可。
  位姿统一由 `updateFootprintPose()` 算，清除与可视化共用同一组缓存值（免得两处各算各的）。
  实测：yaml 默认 0.70x0.40 → 轮廓点 ±0.35/±0.20、CUBE scale 0.70x0.40；
  `offset_x=-0.29` → 矩形中心平移到 -0.29；yaw=45° → CUBE yaw +45.0°；mask 关 → **0 帧**。
- ⚠ 踩过的坑：**同一 yaml 里留了重复键**（两次编辑各加了一块 footprint，后面那块带
  `footprint_clear_enable: false`）→ 静默覆盖前面的 `true`，现象是"参数在 yaml 里明明写了
  却不生效、启动日志也不打"。改 yaml 后务必 `python3 -c "import yaml;..."` 读一遍确认
  最终值，并 `grep -c` 查重。

## 自车（footprint）点剔除（2026-09 新增，默认关）
- 背景：雷达/相机装在机体上时自车几何会进视场 → 那些点被积分成障碍；2D 层同样中招
  （2D 是 3D 沿高度带的投影）→ 规划器把机器人自己判成障碍、膨胀后直接把自己围住。
- 实现：`cloudCallback`（lidar 路径）+ `projectDepthImage`（depth 路径）里调
  `isSelfBodyPoint(p_sensor)`，命中就 **`continue` 整点丢弃**（不标占据、也不 raycast）。
  形状定义在**传感器系**；雷达路径下点云在 map 系，先 `sensor_r.transpose()*devi` 转回传感器系。
- 参数：`self_filter_enable`(false) / `self_filter_radius`(圆柱水平半径, <=0 不用) /
  `self_filter_z_min|z_max`(圆柱 z 区间) / `self_filter_box_min|max`(3 数数组，AABB，
  `max-min` 有正分量才算启用)。两形状取并集。
- 只开开关不给形状会 WARN 且不剔任何点（实测）。
- 实测（合成负载，静止，注入 r=0.10~0.30m/z=-0.50~+0.10m 的"自车壳层"）：
  过滤关 原点 0.5m 内占据格 = **105**、全图 186 → 过滤开(r=0.5) = **0**、全图 **81**（= 无自车点时的基线）；
  r 只给 0.15 → 剩 89（形状边界被尊重，不是"全丢"）；盒模式同样 0/81；开 `verify_2d` 自检 0 失败。
- 刻意**不提供** min-range（"丢弃 ray_length<R 的点"）作为主机制：它会连带删掉真实近距
  障碍（贴墙/窄缝里会瞎）。要通用就用圆柱/盒。

## GridMap 已知事实
- `body_pose`（参数 `topic_body_pose`）**确实被订阅且有用**：`slidingMapFrameCallback` 把它写进
  `sliding_map_frame_pos_`，再由 `publishTF`（grid_map.cpp:1705）广播 `sliding_map` 那帧 TF 的位置。
  上游不发时该 TF 停在原点，只影响 RViz 视图，不影响栅格内容。（旧注释“不订阅”是错的，已改）
- `frame_id_` 仅作输出点云的 header.frame_id → 设 `map` 即可与 `lightning/global_map` 叠合
- `cloud_is_world=true` 时 `ray_q` 在 lidar 路径**不参与运算**，只有 `ray_pos` 影响 raycast
- `GridMap::getVoxelNum()` **声明了但未定义**（.cpp 里被注释掉），用了会链接失败
- 无订阅者时 `publishMap()` 直接返回，省 CPU

## 可视化高度裁剪（"感知点云 z 轴很矮"）
- `publishMap` / `publishMapInflate` / `publishUnknown`（grid_map.cpp:961/1002/1115）会跳过 `z > ray_pos.z + vis_height` 的体素
- `ray_pos` 是**雷达在 map 系的位置**，不是地图原点 → `vis_height` 语义是"传感器上方多少米"
- 默认 `0.3`，看起来又扁又贴地；改成 `2.0` 才正常
- **只影响给人看的输出点云**，`occupancy_buffer_` 完整保留 `local_update_range` 内全部高度，不影响后续规划

## 用户惯用的 RViz 视图
- `~/r41_ws/src/slam/lightning-lm/config/showbodypc.rviz`（SLAM/建图）
- `~/r41_ws/src/slam/lightning-lm/config/showglobalmap.rviz`（定位）
- 用 `rviz2 -d <源码路径>` 直接起 → **改源码后不用重新 colcon build**
- 两个文件里所有点云都是 **Reliable**，所以感知话题（BEST_EFFORT）必须单独加显示项并改 QoS
- 已给这两个文件各加了 3 个 Best Effort 显示项（`.bak` 备份在旁）；`perception.rviz` 是独立的感知专用视图
- `showglobalmap.rviz` 里写的 `/lightning/current_scan_cloud` 是**错的话题名**，实际是 `/lightning/current_scan`
- `perception.launch.py` 的 `rviz` 默认已改为 `false`（用户自己起 RViz，不需要 launch 带）

## RViz QoS 坑（排查了半天）
- `SensorDataQoS()` = BEST_EFFORT。**RViz 的 PointCloud2 默认 Reliability = Reliable**，二者 DDS 不兼容 → RViz 一条都收不到（无任何报错，显示项就是空的）
- 实测：同一话题 RELIABLE 订阅收 0 条，BEST_EFFORT 收 50 条；日志 `offering incompatible QoS. Last incompatible policy: RELIABILITY`
- lightning 的 `odom/nav_state/current_scan/global_map/path/loc_state` 都是 `create_publisher(..., 10)` = RELIABLE → 所以 RViz 里能正常看到
- **修法：RViz 显示项里把 Reliability Policy 改成 Best Effort**（脚本化：见 `perception/config/perception.rviz`）
- **不要**为了 RViz 方便把发布端改成 RELIABLE：RViz 是慢消费者，会反压定位节点 —— 和 rslidar_sdk 10Hz→5Hz 是同一个坑
- 症状"发布端 sub count 看起来 >0"可能是 `ros2 topic hz` 自己造的订阅者，别被骗；用 `ros2 topic info` 查

## 构建坑
- 移动包路径后必须 `rm -rf build/plan_env install/plan_env`，否则 CMake 报 "source does not match the cache"
- `colcon build --packages-select plan_env` 时会 WARN 说 plan_env 在 underlay 里 —— 是同一 workspace 的 install/，可忽略
