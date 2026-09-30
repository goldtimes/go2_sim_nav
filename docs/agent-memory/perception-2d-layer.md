# perception（`src/perception`）2D 局部感知层

★ **2026-09-29 包名已改**：原 `plan_env`（抽取自 SCAN-Planner 的名字）→ **`perception`
（= 目录名）**，库目标/导出目标/头文件目录一并改（`include/perception/`）。
原因：包名 ≠ 目录名 ⇒ `colcon build --packages-select perception` **静默什么都不编译**
（错误串是大写 `ERROR`，小写 grep 会漏）。改名后必须 `rm -rf build/<旧> install/<旧>`
再编译，否则 CMake 报 `source ... does not match the source used to generate cache`。

## 数据来源（验证前先确认这个！）
- perception_node 的输入是 **lightning 的输出**：
  - `cloud <- /lightning/perception/cloud`（**map/world 系**的累积点云，不是原始雷达帧）
  - `sensor_pose <- /lightning/perception/pose`（map -> lidar_link）
  - 所以要用 `grid_map.cloud_is_world: true`
- 仓库里的 bag（`go2_bag/*`、`lio_data*`、`lio0902`、`lio_data2/3`）**只有原始 LiDAR+IMU**
  （`/livox/lidar` CustomMsg 或 `/rslidar_points` PointCloud2 + Imu），**没有位姿/没有 map 系点云**
  → **不能直接验证 perception_node**，必须先跑 lightning（LIO）产生位姿与点云。
- ⚠ 教训：不要用自造的合成点云去"验证"感知正确性（只能验证代码跑通）。
  一次合成测试（自己射线投出地面+墙+柱子）里 3D 占据得到 0 个体素，
  差点被当成 bug 追查；这是合成数据的假象，不能作为结论。

## 2D 层接口（enable_2d，默认关）
- 话题：`grid_map/occupancy_2d`、`grid_map/occupancy_inflate_2d`（nav_msgs/OccupancyGrid，100/0/-1）、
  `grid_map/esdf_2d`（PointCloud2，xyz + intensity=距离/m）
- 进程内查询：`getOccupancy2D()`、`getDistance2D()`、`get2DInfo()`、`has2DLayer()`
- 布局：2D 数组按 **全局索引序** `i2d = ix + iy*nx`（与 OccupancyGrid 的 data 一致），
  不是滑动窗口的环形局部序 → 不会跨环绕边界泄漏 EDT，发布可直接拷贝。

## 已修的真 bug：窗口原点竞态（重要）
- `build2DLayer()` 用**构建时刻**的 `map_bound_min_idx_` 生成数据，
  但 `publish2D()`/查询接口原来用**发布时刻**的 `map_bound_min_idx_` 算 origin。
- 滑动地图 `updateSlidingMap()` 一改窗口原点（`map_sliding_thresh` 到达时），
  两者就错位 → 整张 2D 图与 origin 一起漂移。仿真里静止时看不出来，实车一直滑动必然出错。
- 修法：构建时把原点存进 `md_.occ2d_min_idx_`，发布/查询统一用它。

## 高度带参数（易错）
- `proj_z_min/proj_z_max` 是 map 系**绝对高度**（`proj_relative_to_sensor: false`）。
- `proj_z_min` 必须**高于地面**：本机地面在 map 系 z≈-0.36，所以从 -0.20 起（用 -0.40 会把地面投成障碍）。
- 高度带与滑动窗口 z 区间不相交时会打一条 WARN 并让 2D 层全为未知。

## 性能优化（2026-09，已完成并验证）
- 实测（合成负载，`show_occ_time:=true` 打的 "2D layer build"）：
  - 全量重扫：**12.8 ms/帧**
  - 增量（只重扫变化列）：**3.46 ms/帧（约 3.7×）**，自检 0 失败
  - 开自检（每帧暴力全扫比对）会额外 +9.5 ms
- **默认 `force_full_2d: false`（增量），`refresh_full_2d_interval: 100` 当保险**。
- **增量曾出现 1.5%~3.7% 陈旧格的根因（已修）**：`build2DLayer()` 开头残留一行
  早期"原点竞态修复"留下的 `md_.occ2d_min_idx_ = mp_.map_bound_min_idx_;`，
  它在计算 `dx` 之前就把锚点改成新值 ⇒ `dx/dy/dz` **恒为 0** ⇒ `shift2DArray` 永不执行
  ⇒ 数组锁在旧锚点、锚点却报新值 ⇒ 每次滑窗整图系统性错位。
  ⚠ 也正因如此，"滑动即全量""dz 即全量"两个补丁（以 `dx!=0` 为条件）从未触发、看着"无效"。
  修法：删掉那行，锚点只在函数**末尾**（和 `initMap`）更新。
- 教训（通用）：在一个函数顶部新增"状态赋值"，会让其后基于该状态的**增量/delta 计算静默失效**；
  启用增量优化时必须先确认 **delta/guard 真的被触发过**（打点看 `dx`），再谈验证。
- **`dz != 0 ⇒ full` 这个保守分支已删除**（推理：z 滑动要么伴随 kz0/kz1 变化→已判 full，
  要么清掉的 slab 在高度带之外→无害；2D 数组不按 z 索引，不需要 z 平移）。
  实测滑窗 `shift=(dx,dy,dz)`：x 滑 3~5 格时 **dz 同时是 3~6 格**（0.15~0.3 m），
  即该分支在真实平台上会**每次滑窗都触发**，让增量退化成"每次都全量"。
  之前 3.46 ms 的好数字只是因为旧测试驱动把位姿 z 写死 0.36（dz 恒 0，分支形同不存在）。
- 验证方法（外部可观测量优先，避免自欺）：
  - **x 滑动**证据：订阅 `/grid_map/occupancy_2d` 看 `info.origin.position.x`（直接来自窗口原点）。
    ⚠ 但 `origin.position.z` 是**高度带中点**（`z_mid`，只作切片记录），**不能**用来判断 z 窗口是否移动 ——
    差点因此得出"z 没滑"的错误结论。
  - **z 滑动**证据：只能在 `updateSlidingMap()` 里临时打 `shift_num`（show_occ_time 门控），用完撤掉。
  - 驱动 `drive_map_test.py` 第 2 个参数 = VZ（位姿 z 爬升 m/s，模拟爬坡；世界内容仍固定在 map 系）。
  - 最强验证：`verify_2d:=true` + `refresh_full_2d_interval:=0`（**关掉定期兜底**）跑 x/z 同时滑动 →
    失败 0（27 帧比对，13 次滑窗）。
- ⚠ 自检的边界：它比对的是"增量结果 vs 对**同一份 3D 缓冲**的暴力全扫"。
  所以 **3D 侧自身的陈旧（比如滑动清 slab 后邻居 inflate_cnt 未更新）它查不出来**，
  定期全量重扫同样查不出来（两者读的是同一份 3D）。定期兜底只覆盖"2D 侧漏标脏"这一类。
- 新计时（恒定滑动 + z 漂移的负载，位姿每 ~2 帧滑一次）：增量 **6.11 ms** vs 全量 **12.03 ms**；
  旧数据（x 缓行、z 固定）：增量 3.46 ms vs 全量 12.81 ms。两种负载都约 2~3.7×。
- 已排除的假设（都实测过，不是这些）：平移函数索引数学（穷举 812 组全通过）、
  滑动方向、z 带、单线程竞态（节点是 `rclcpp::spin` 单线程）、
  在 raycast cache 弹出点标脏（覆盖全部写入仍漏）。
- 正确的收益（已默认开）：ESDF 从"每次构建都算"改成**按需**（`esdf2d_query_en` 或有人
  订阅），实测无人需要时 0 次构建；列扫描早退。
- 相关开关（`config/perception.yaml`）：`force_full_2d`、`refresh_full_2d_interval`、
  `verify_2d`（自检，失败会打 ERROR 并给出"本帧扫过仍错/本帧未重扫"分类）、
  `esdf2d_query_en`（进程内查 `getDistance2D()` 且无人订阅时必须 true，否则返回 -1）。

## max_ray_length = "清空半径"（2026-09-21 已参数化，重要）
- `raycastProcess()` 语义：`length > max_ray_length` 的点会被**截断到该半径**并把截断点
  标成 **free**（0），**不标占据** → 所以它是"每帧能把多远清成空闲"的半径，
  也直接决定 2D 图未知面积；同时**决定了多远之外的障碍根本不会进图**。
- 它与 `local_update_range`（能写占据的范围）必须自洽，否则 2.0~4.0 m 一圈
  "能写障碍却不清空"。实测 8x8 m 窗口、max_ray_length=2.0（旧值）：
  `occupancy_2d` 未知 63% / 空闲 36%；环带空闲率在 2.0 m 处 88% → 2.5 m 骤降 52%
  → 3.5~4.0 m 仅 17%（断崖就在 max_ray_length 上）；占据格仅 66 个（远处墙看不见）。
- 参数化：
  - `grid_map.ray_length_from_local_range: true` → `max_ray_length = max(local_update_range_x, y)`（推荐）
  - `grid_map.max_ray_length`（显式值，作为开关关闭时的回退；日志会提示它）
  - `grid_map.pub_2d_stats_interval`（>0 时每 N 秒打一行 2D 图 空闲/未知/占据 占比）
  - initMap 里新增"几何自洽性" INFO/WARN：打印 窗口/写占据/清空半径/单帧最多清多少面积，
    并在 `clear_r < min(local_update_range)` 时 WARN 给出修法。
- 改成 4.0 后实测（真实雷达数据，同一地点）：未知 63% → **35.3%**，空闲 36% → **61.9%**，
  未知/空闲 1.75 → 0.57；2.5~3.0 m 空闲率 29% → 78%，3.5~4.0 m 17% → 66%；
  **占据格 66 → 176（2~4 m 环带 29 → 157）** —— 即"2 m 外的障碍原来根本没进图"。
- 代价：射线步数 ∝ 长度/res；本负载下 3D 融合仍 ~1.4 ms（~2172 点/帧）。
- 验证手法（可复用）：开一个**隔离实例**跑真实数据，避免动用户正在跑的节点：
  `ros2 run perception perception_node --ros-args --params-file <带 FQN 键的 yaml>
   -r __ns:=/smoketest -r /tf:=/smoketest/tf -r /tf_static:=/smoketest/tf_static
   -p grid_map.topic_cloud:=/lightning/perception/cloud -p grid_map.topic_pose:=/lightning/perception/pose`
  ⚠ 两个坑：① 带 namespace 时 `--params-file` 里**相对节点名不生效**，必须用 `/smoketest/perception_node`
  作键（用 python 读原 yaml 重新 dump 即可）；② `ros2 run ... | grep` 会因管道缓冲吞日志，
  **必须重定向到文件**再 grep。

## esdf_2d 发布语义（2026-09-22 核实，MPC 要消费它）
- `grid_map/esdf_2d` = `PointCloud2`，`intensity` = 距离 [m]；**`d ≤ 0`（障碍内部）与 `d > esdf_max_dist` 都不发**
  ⇒ 云里只有"离障碍 0~3 m 的带"，**不含障碍本体** → 硬碰撞判定必须另用 `occupancy_inflate_2d`。
- `esdf_pub_step: 2` → 0.20 m 抽点（不是 0.10 m 全分辨）；`esdf_max_dist: 3.0`；`esdf_unknown_as_occupied: false`（乐观）。
- 只在**有人订阅**或 `esdf2d_query_en` 时才构建；发布节流 `pub_2d_interval`。
- EDT 耗时实测（本机、pnc_2d 的 `ClearanceField`，同算法）：全局 607x307 @0.05 = **4.76 ms**；
  局部 4m×4m @0.10 = **0.018 ms**，@0.05 = 0.120 ms ⇒ 20 Hz 预算下自算 EDT 毫无压力。

## 清图服务（2026-09-29 加）
- `perception_node` 提供 **`/grid_map/clear_map`**（`std_srvs/Trigger`；参数
  `grid_map.clear_map_service`，默认 `grid_map/clear_map`）。实现 = `GridMap::resetBuffer()`
  ⇒ 3D 占据/膨胀/计数/raycast 缓存/占据索引 **+ 2D 层与距离场** 全部作废，下一帧起重建。
- ★ 服务名不能写成裸 `clear_map`：ROS 2 相对名只拼**命名空间**、不拼节点名
  ⇒ 那样会变成全局 `/clear_map`（实测撞名风险）。
- 实测：清图前后 `/grid_map/occupancy_2d` 一直 **10 Hz** 不断（不存在"清了不发布"
  的下游拿着旧图的问题）；清完环带占据格 67→11 再爬回。
- 调用方：pnc_2d 的恢复行为 `sm.recovery.type: clear_map`（见 pnc_2d.yaml + 
  `clear_map_recovery.{hpp,cpp}`）⇒ 被挡/卡住时"清图 + 状态机重规划"。

## 撤障后感知会不会自己清？（2026-09-29 实测，结论要记牢）
- 开阔处、静态箱、车不动：**撤障后 ≤1 s 局部图自己就清干净**（箱矩形 9 格→0），
  且上游点云也同步掉（箱矩形内 9 点→7 点，基线 6）⇒ **不是"感知不清"**。
- 所以"幽灵障碍"只在**没有射线穿过那些格**时出现：物体**移动/被传送**留下的尾迹、
  或物体消失时已在**车视野外/身后**（滑窗里没人去标脏它）。这类只能靠
  ①清图恢复 ②"长时间没被再观测到就衰减"的机制。
- 量它的脚本：`test/sim/obstacle_clear_probe.py`（放箱→进图→删箱→逐秒看三层：
  局部图占据格 / 上游点云点数 / 箱心 ESDF；阶段二还会手动调 `/grid_map/clear_map`）。
  ⚠ 该探针解析点云要**限速**（否则 Python 解 10 Hz×6 万点会把 spin 拖慢、读数滞后）。

## ★ 近距离盲区（2026-09-29 实测，设计使然，别当 bug 追）
- `grid_map.footprint_clear_enable: true` + `footprint 0.70×0.40` + 判定"格与矩形相交
  （矩形外扩半格 res/2=0.05）" + `footprint_clear_as_unknown: false`（抹成**空闲**）
  ⇒ 车心沿车头 **±0.40 m**、左右 **±0.25 m** 的格**每帧被写成空闲**（occupied 与 inflate 层都抹）。
- ESDF/MPC 都建立在同一张图上 ⇒ 这个范围内的障碍在**控制器眼里就是空地**。
- 实测（`test/sim/near_field_probe.py`，静态箱 0.6×0.6×0.8 逐步靠前）：
  | 车心→盒心 d | 盒矩形格 | 占据格 | 盒心 ESDF |
  | 2.00/1.50/1.00/0.70/0.50/0.45 m | 81 | 5/3/7/6/1/1 | 0.30/0.41/0.22/0.10/0.14/0.50 |
  | **0.40/0.35/0.30/0.20 m** | 81 | **0/0/0/0** | **0.82/0.82/0.82/1.02** |
  ⇒ 断崖正好在 0.70/2+0.05 = 0.40 m；盲区内 ESDF 报"很空"（不是 3.0 上限，因为旁边还有别的
  结构，但**盒子本身完全不贡献**）。
- 后果（S2 动态横穿实测）：箱子贴上来以后局部图看不见 ⇒ 车在 **FOLLOWING** 下继续往前顶，
  直到净距 -0.119 m、被箱子推着走（Go2 被推向墙 ⇒ 全局规划从该位姿永久
  `START_FOOTPRINT_COLLISION`）。**不是雷达/点云/pcd 的问题。**
- 结论：0.40 m 内部已经"贴着"，**回避**在那里物理上做不到，正确行为是**停**；
  所以修的方向是"别让它继续开"（近身/顶住检测），而不是指望图能看见。

## 经验
- 2D 层只是 `occupancy_buffer_` 的忠实投影：3D 层有几个占据体素，2D 就有几个 100 的格子
  （实测 3D 53 个 / 2D 53 个，一一对应）→ 2D 层本身不会"丢"东西，3D 不对则 2D 跟着不对。
- EDT 用 Felzenszwalb 1D，已用独立小算例验证精确性（0/2.5/1.5/2.1213/2.8284 全对，截断生效）。
- 调试这类节点用 `printf` + `fflush` 写文件，别用 RCLCPP_WARN_THROTTLE（节点 use_sim_time=false）。

## ★ 动态物体"扫过"后必然留下清不掉的格（2026-09-29 定位，有 DDA 证据）
用户报："模拟动态障碍物在机器前面运动，会发现有些栅格无法清除"。
**根因：那些体素从来没有被任何射线穿过**（复刻 perception 的 Amanatides-Woo DDA
逐体素验证：17/17 个残留格都有"从未被穿过"的体素，全部被穿过的 0 个）。

机制（不对称是本质）：
- **写 occupied 很容易**：只要有一条"打到表面"的射线 ⇒ 表面必有回波 ⇒ 一定有射线到位。
- **清 free 很难**：需要一条射线**恰好穿过这个 0.1 m 体素**（角度落在 ~2.9° 内）。
  而点云是 `filter_size_scan: 0.2` 降采样过的 ⇒ 2 m 处相邻射线间隔 **≈5.7°**
  ⇒ 大量体素（尤其高仰角那几层）**不在任何射线的路径上**。
- 箱子在时：箱面体素被命中 → occupied。箱子走后：那些方向**不再有回波**
  （空地/高空没实体），其它方向的射线又恰好不经过 ⇒ **永远收不到 miss** ⇒ 永远 occupied。
- 这也解释了为什么"**静止**箱子撤障后能自己清干净（≤1 s）而**运动**扫过会留痕：
  静止时射线角度不变，撤障后同一条射线还在，会**穿过**原来的体素；运动时每个格
  对应不同的射线角度，其中一部分角度撤障后没有回波。

**排除掉的假设（都实测过，不是这些）**：
- 不是"上游点云还有箱子"：撤障后点云在轨迹带内**中/高档 0 点**，全是地面点（z<-0.15）。
- 不是"清了又回来"：调 `/grid_map/clear_map` 后 0 格，且不再重现。
- 不是 2D 层漏更新：`/grid_map/occupancy`（3D）本来就还占着，2D 只是忠实投影。
- 不是"射线够不着那个高度"：该处射线最高可及 z≈+0.42~0.59 ≥ 残留最高 z=+0.45。
- **不是数据累积太深**：`p_hit 0.85 / p_miss 0.30 / p_occ 0.80 / p_max 0.98`
  ⇒ 从饱和降到阈值只要 **3 帧 miss（0.3 s）**。所以"清不掉"=**根本没被穿过**。

**残留的构成**（`/grid_map/occupancy`，探针实测）：z∈[-0.45,+0.45]；其中 **z<-0.20
（地面及以下）不影响 2D/规划**（`proj_z_min=-0.20` 挡住），真正有害的是高度带内那几层。

**★ 2026-09-30 重要负结果（别重踩）**：把点云从降采样 3255 点换成**未降采样 9280 点**
（×2.85，2 m 处射线间隔 5.7°→2.0°，已远超一个体素占的 2.9°），扫掠残留格数
**一格都没变（20 → 20，同一批位置）**。⇒ 残留的主因是"**物体走后那个方向没有回波**"
（那些格在箱子高度层 z≈0.1~0.4，需要高仰角射线；高处/空地没有回波 ⇒ 射线不存在），
**不是**"射线太稀"。**加点数对它无收益；真正解决的是 `decay`。**
（点数提升的价值要看"小物体可见性"与"未来聚类"，未验证。）
实现与坑：lightning 新增 `lightning/perception/cloud_full`（同帧同姿态未降采样，
`MapIncremental()` 里多一次 `PointBodyToWorld`；`point_filter_num: 2→1` —— 它只影响
`scan_undistort_`，**不影响 LIO 匹配点数**）；perception 侧 **launch 的 `cloud_topic`
优先于 yaml**（只改 yaml 不生效，踩过）；`ros2 topic echo` 要留 ≥10 s 超时（它启动就要 1~3 s）。

## ★ 2026-09-30："要不要先去掉地面再 raycast"→ **不要**（三重隔离 + 近场清空来源）
- 地面点占 **40.1%**（一帧 9280 点）；地面在 map **kz=-5/-4（z∈[-0.50,-0.30)，35.7%）**，
  雷达在 z≈-0.05；带底 `proj_z_min=-0.20`=kz=-2 ⇒ **中间只隔 kz=-3 一格，余量 1 个体素**。
- 三重隔离（地面进不了 2D）：① 高度带投影；② `mark2DColumnDirty()` 的 z 守卫
  `if (gz < occ2d_kz0_ || gz > occ2d_kz1_) return;`（**否则地面每帧把整窗标脏，
  增量优化直接退化成全量**）；③ `rebuildInflationOffsets()` 的 `inf_step_z_up =
  ceil(obstacles_inflation_z_up/res) = 0` ⇒ 地面体素只膨胀到自己那层和下一层
  （kz=-6..-4），离带底差 2 层。⚠ 有**防漏**作用：`obstacles_inflation_z_up` 一旦调成
  0.2，整块地面被膨胀进带 ⇒ **2D 图全黑**。
- 实测 2D 图：空闲 83.3% / 未知 13.7% / 占据 3.0%（8×8 m 窗）→ 没有被地面污染。
- ★ **去掉地面点的代价**（复刻 DDA 逐体素，`scripts/band_clear_sources.py`）：
  带内被穿过的 36140 个体素里 **1095 个只被地面射线穿过（3.0%）**，**全部落在 kz=-2
  一层（z∈[-0.20,-0.10)）= 该层的 23.1%**；kz=-1 及以上 0%。这 1095 个的距离
  **min 0.15 / 中位 1.45 / max 2.10 m（≤1.5 m 占 53.7%）⇒ 纯近场**。
  几何原因：`z(d_h) = z_s - 0.40·d_h/d_g`，要穿过 d_h 处的带底需地面点在
  `d_g ∈ (4.0,12.0] m` ⇒ 只有近场有效；打竖直表面的射线近似水平，只清传感器高度那层。
  ⇒ 丢点会让"地面上的箱子走后带底那层收不到 miss"，**残留变多**。地面点是帮忙清的。
- CPU 也不是理由：3D 融合 **3.4 ms @ 9280 点 @ 10 Hz**（单核 ~3%），且 raycast 对点数
  不敏感（`flag_traverse_/flag_rayend_` 去重 + 撞到已穿过立即 break）。
- 3D 侧：`/grid_map/occupancy` 1646 个占据体素里**地板那层占 41.6%**（+邻居 ~55%），
  但**无下游消费者**（pnc_2d 只订 `occupancy_inflate_2d`/`esdf_2d`）。想清理的正确改法是
  "保留射线、抬高命中点"（末端 z < 带底仍做 traversal，不写 hit），**不是丢点**。
- 工具：`scripts/cloud_z_hist.py`（z 直方图，`--voxel-grid` 按 kz 分档）、
  `scripts/band_clear_sources.py`（DDA 统计清空来源）。必须 `/usr/bin/python3`。
- 文档：`perception/doc/perception_gridmap.md` §3.7。

**已实施（2026-09-29）**：**时效衰减** `grid_map.decay_*`（方案 2）
- 每个体素记"最后一次被**命中**的时刻"（`md_.last_hit_s_`，float 秒），超过
  `decay_timeout_s`（默认 5 s）每帧把 log-odds 往回拉 `decay_rate`（0.30），
  低于 `p_occ` 阈值就不再算障碍。走 `applyOccupancyUpdate()` ⇒ 膨胀/索引/2D 脏列自动同步。
  实现 = `grid_map.h/cpp` 的 `decayStaleOccupancy()`；每帧只遍历占据索引（几千个）<0.1 ms。
- ★ **判据必须用"命中"不能用"穿过"**：残留格**恰恰永不被穿过**（用穿过当判据它永远
  "刚被观测过"）；真障碍在视野内每帧有回波 ⇒ 时间戳每帧刷新 ⇒ 永不超时 ⇒ 不误清。
- **实测**：扫掠后轨迹带残留 **20 → 13 → 11 → 8 → 6 → 5 s 时 0 格**。
- **回归**：静态箱子放 40 s 一直在（箱内 5~6 格稳定）；静止场景 `衰减` 计数恒为 0；
  S1 绕行仍 PASS（净距 0.274 → **0.430 m**、穿透 0.0 cm）。日志 `帧耗时…| 衰减 N`。
- ⚠ **同时修掉一个真 bug**：`md.raycast_num_` 原来是 `char`，**每 256 帧回绕**，回绕时
  与 256 帧前的旧标记"假相等" ⇒ 整条射线被 `continue`/`break` ⇒ **命中/清空静默丢失**。
  已改 `int32_t`（`flag_traverse_/flag_rayend_` 同步改 `vector<int32_t>`，内存 +1.5 MB）。
- ⚠ 残余风险：真障碍**被长时间遮挡**也会超时 ⇒ 保守就把 `decay_timeout_s` 调到 8~10 s，
  或 `decay_max_dist: 2.0` 只治近场。兜底仍是 `clear_map` + `local.fuse_global_map`。
- 改了 perception 必须**重启 tmux `mobilebase:1.3`**（命令 `ros2 launch perception
  perception.launch.py`）。

1. 现状兜底（仍在）：`clear_map` 恢复（清图重建）+ `rolling_replan` 绕行。
3. 通用时效衰减（所有 occupied 超时就衰减）：风险大，被遮挡的真障碍会被误清。
4. 零代码缓解：把上游 `filter_size_scan` 0.2 → 0.1（射线密度翻倍 ⇒ 残留大减，
   但 LIO 前端 CPU 上升）。

**工具**：`src/pnc_2d/test/sim/obstacle_sweep_probe.py`（让箱子在车前往复扫，
撤障后逐秒统计两层图残留 + 按 z 分层 + 复刻 DDA 判定"有没有射线穿过" +
调 clear_map 对照）。用它复现：`--speed 0.3 --rounds 6 --settle 20`。
⚠ 跑之前确认**车静止**（脚本会自检）；`remove()` 必须检查返回值，父类 `Probe.remove()`
不看返回结果，删失败会被误读成"清不掉"（实测踩过：残留基线从 9 格变 23 格就是箱子还在）。
