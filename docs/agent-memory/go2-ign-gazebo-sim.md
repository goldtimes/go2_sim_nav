# Go2 + Ignition (gz-sim6) 仿真 —— 坑与事实（2026-09-18）

> **2026-09-18 用户要求回滚**：mid360 集成 + 三个传感器开关已全部撤销，
> `go2_description/xacro/robot_VLP_D435i.xacro`、`gazebo_VLP_D435i.xacro`、
> `gazebo_sim/launch/gazebo_go2_sensors.launch.py` 已还原为
> `/home/gmd/project/ROS2-Gazebo-GO2/src/...` 的上游版本（逐字节相同）。
> **下面记录的结论仍然是有效的知识**（换机器/以后再上 mid360 时会再遇到），
> 但描述"当前配置"的段落已经不适用于工作区现状。

包：`src/bringup/gazebo_sim`（launch）/ `src/bringup/go2_description`（xacro）/ `src/bringup/livox_mid360_gz_sim`（插件，仍在磁盘上但已无引用）
机器：Optimus 双显卡（Intel 核显 + **NVIDIA RTX 4060**），X 在 `:1`

## 启动方式（必须两步）
1. world（headless）：`ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-s -r -v3 <abs>/gazebo_sim/world/warehouse.sdf"`
   （也可用一体化入口 `ros2 launch gazebo_sim launch.py sensors:=true world:=warehouse.sdf`）
2. robot：`ros2 launch gazebo_sim gazebo_go2_sensors.launch.py enable_rviz:=false`
   注意：该 launch **不启动 Gazebo**，只 spawn 到已运行的 world。

## ★最坑的一个：EGL 选错 vendor → 相机类传感器静默失效
- 症状：`camera` / `rgbd_camera` / `gpu_lidar` 传感器**被正常创建**，`ign topic -l` 里话题也在、有 Publisher，
  但**一帧数据都不发**（`ros2 topic hz` 无输出）。日志只有一句
  `libEGL warning: egl: failed to create dri2 screen`（这是 **Mesa** 打的，不是 NVIDIA），
  `Loading plugin [ignition-rendering-ogre2]` 根本不出现。**不报错、不崩溃，纯静默。**
- 根因：`~/.bashrc` 里只有 PRIME offload 的 **GLX** 两件套
  （`__NV_PRIME_RENDER_OFFLOAD=1`、`__GLX_VENDOR_LIBRARY_NAME=nvidia`），
  而 **Gazebo/Ignition 的离屏（传感器）渲染走 EGL** → glvnd 按 `egl_vendor.d/` 里的
  `50_mesa.json` 选了 Mesa → Mesa 在 NVIDIA 驱动的 X 上建不出 screen。
- **实测有效的做法**：启动 world 时给 gz_args 加 `--headless-rendering`。加上后 ogre2 正常加载、
  GPU 利用率 44%、color 4.7 Hz / rgbd image 17.2 Hz / L1 scan 5.7 Hz 全部出图。
- `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` 有人建议加，
  但**本机单独验证不足以证明必要**：纯 EGL 探针（ctypes 调 eglGetDisplay/eglInitialize）
  在不设该变量时也返回 VENDOR=NVIDIA。该变量已从 `~/.bashrc` 撤回，只在注释里留了提示。
- 判别标准：`grep -c "ignition-rendering-ogre2" <server日志>`。但**渲染是惰性的** ——
  没有渲染类传感器时 ogre2 本来就不加载，所以"纯 world 无机器人"或"相机全关"的对照组
  判别不出真假；必须"带机器人 + 相机开启"才有意义。修好/坏时 `nvidia-smi` 显存 900 MiB vs 11 MiB。
- 本机（2026-09-18）结论：**用户只需要 mid360 + IMU，相机/L1 全部默认关掉，
  也就不需要这套渲染链路了**（GPU 全程 0%、11 MiB）。

## 坑 2：不重启 world 就重复 spawn，会留下第二只狗
- kill 掉 `ros2 launch` **不会**移除 Gazebo 里的 model 实体；再 spawn 时 `-allow_renaming true` 会生成 `robot1_my_bot_0`。
- 症状：`LivoxMid360GzPlugin` 加载两次 → `/robot1/livox/lidar` Publisher count = 2；
  `gz_ros2_control` 插件实例重复 → `Controller 'joint_state_broadcaster' can not be configured from 'active' state` +
  `Could not configure controller ... because no controller with this name exists`，spawner 退出码 1。
- 正确做法：改配置后 **连 world 一起重启**。清理：
  `pkill -9 -f "gazebo_go2_sensors|parameter_bridge|image_bridge|quadropted_controller|robot_localization|OdometryNode|cmd_vel_pub|ign gazebo|gz_sim.launch"`

## 坑 3：`ros2 topic hz` 通过管道 + `timeout` 会丢输出
- `timeout 15 ros2 topic hz X | grep ...` 常常什么都不打印。原因：Python stdout 被管道块缓冲，SIGTERM 到达时缓冲被丢弃。
- 可靠写法：`timeout 15 ros2 topic hz X > /tmp/h.txt 2>&1; grep "average rate" /tmp/h.txt`（`stdbuf -oL` 也救不回来）。

## RTF 事实（warehouse.sdf + warehouse 里的 Go2）
- **瓶颈是物理步进(1ms) + mid360 插件(80 万条 CSV 射线 CPU 投射)**，不是渲染。
  `ruby`(ign gazebo server) 只占 ~75% 单核（28 核机器）→ 不是 CPU 打满，是主循环 per-step 固定开销。
- 同一 world / 同一只狗的 A/B（EGL 修好、真 GPU 渲染）：
  | 配置 | `/robot1/livox/lidar` 墙钟 | RTF | GPU util |
  |---|---|---|---|
  | L1 + 前脸相机 + D435i 全开 | 5.657 Hz | 0.566 | 9~23% |
  | 三个全关 | 5.804 Hz | 0.580 | 0% |
  → 相机只值约 **2.5% RTF**，但 GPU 从 0% 到 9~23%。要提 RTF 得动 `max_step_size` 或 mid360 的采样数，关相机没用。

## 传感器开关（2026-09-18 新增，默认全关）
- xacro args：`enable_l1_lidar` / `enable_face_camera` / `enable_d435i`，默认 `false`；
  `lidar_model` 默认 `mid360`（`lidar_model:=velodyne` 切回 velodyne）。
- launch args 同名透传；`image_bridge`（`ros_gz_image`）与 ros_gz_bridge 桥接列表都随开关条件化
  （桥接列表用 `OpaqueFunction` 拼装，因为 LaunchConfiguration 在 Python 里无法直接 if 判断）。
- 默认只保留：`/robot1/livox/lidar`(CustomMsg)、`/livox/lidar/points`、`/mid360_imu`、
  `/ground_truth/*`、`/tf`、`/joint_states`、`/imu_plugin/out`。
- EGL 修好后各传感器实测频率：color 11.3 Hz、rgbd image/depth 17.1 Hz、L1 scan 5.7 Hz、mid360 5.7 Hz（10Hz 仿真时间）。

## ★IMU 消息被成倍重复（已修，2026-09-18）
- 症状：`/robot1/mid360_imu` 声明 200 Hz，实测 sim 内 596 Hz（≈3×）；躯干 `/robot1/imu_plugin/out`
  声明 100 Hz，实测 299 Hz。`ign topic -i` 显示 GZ 侧一堆发布者。
- 根因：**`<plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu">` 是"全局"的，不是按传感器生效。**
  世界里每多一处这种声明，它就给**每一个** IMU 传感器额外挂一个 gz-transport 发布者。
  而 world 的 `gz-sim-sensors-system`（`warehouse.sdf` 里已声明）本来就按 `<update_rate>` 正常发布。
  于是发布者数 = 1(Sensors系统) + N(Imu插件声明处数)。
- 修法：**删掉 xacro 里所有传感器级的 `libgz-sim-imu-system.so` 声明**（只靠 world 的 sensors-system 发）。
  实测：mid360_imu 201 Hz(声明200) / imu_plugin/out 101 Hz(声明100)，均为 1.01×。
- 排查手法（很通用）：
  1. `ign topic -i -t <topic>` 看 GZ 侧发布者地址；
  2. `ss -tlnp | grep :<port>` 把地址映射到 **pid**，即可区分"server 内多个发布者"和"别的进程"。
     实测 `parameter_bridge` 也会作为发布者出现（因为桥接写成了 `@` 双向）→ IMU 这种单向传感器
     应该写成 `'<topic>[gz.msgs.IMU'`（`[` = 只 GZ→ROS）。
  3. `strings <so> | grep "日志里的关键词"` 反查是哪个库/组件在发。
     IMU 相关日志串在 `/opt/ros/humble/lib/libgz_hardware_plugins.so`（`GazeboSimSystem::registerSensors()`，
     它也会枚举 `sdf::Sensor` 的 `ImuTag`，但那是订阅侧）。
- 注意：`gazebo.xacro` 里还有一份 `imu_sensor` 也带着这个插件，若哪天用到那份 xacro 要一并处理。

### ★2026-09-18 机制修正（上面"1 + N"的模型是错的，已实测澄清）
- 正解：**发布者数 = "带 `libgz-sim-imu-system` 插件的 IMU 传感器个数"**（世界级声明也算 1 个）。
  **`gz-sim-sensors-system` 不发 IMU**，它不是那"额外的 1 份"。
- 实测对照（同一份 xacro，3 处传感器级声明）：
  | 运行 | 模型里的 IMU 传感器 | `ign topic -i` server 侧发布者 | 实测速率 |
  |---|---|---|---|
  | 上一次 | 3（含 d435i_imu） | 3 | 604 / 304 Hz（声明 200/100）= **3×** |
  | 当前 | 2（`ign model -m robot1_my_bot` 只有 imu_sensor、velodyne_imu；d435i_imu 只有 bridge 广告、无数据） | 2 | 396 / 198 Hz = **2×** |
- `ign topic -i` 里多的那 1 条 = **`parameter_bridge` 自己**（`ss -tlnp` 查端口属 parameter_bridge；
  双向 `@` 桥接的广告），**不发数据**，不是元凶。
- **世界级 `gz-sim-imu-system` 各 world 不一致**：`rmuc_2025_world.sdf`、`cafe.world` 有 1 处；
  `warehouse.sdf`（其 `<world name="world_demo">`）、`L02-S03-Fuel-ignition.sdf`、`L02-S05-custom_world.sdf` **没有**。
  ⚠ 所以旧修法"xacro 全删、只靠 world"在 **warehouse.sdf 上会变成 0 Hz**；
  旧记录里删完仍有 1.01× 是因为那次跑的 world 自带世界级 imu-system。
- 正确修法二选一：①xacro 全删 + 每个 world 补 1 处世界级声明（语义正确，推荐）；
  ②xacro 只留 1 处（最小改动，但遇到自带世界级声明的 world 会变 2×）。
- **数值影响：无害**。`esekfom::predict` 两个分支都是 `(dt * f_w) * Q * (dt * f_w)ᵀ`，
  dt = 0 → 状态不变、Q 注入为 0（`ros2 topic hz` 出现 `min: 0.000s` 即"重复包同时间戳"的佐证）。
  代价只是 2~3× 的 IMU 处理/传输/日志开销。
- **判别工具（必用）**：`ign model -m <model名>` 看**世界里有几个 sensor** —— 世界里的模型可能与
  当前 xacro 完全不一致（旧 spawn 残留）。实测踩过：world 里 `robot1_my_bot` 只有 2 个 sensor
  （`velodyne`/`velodyne_imu`）→ IMU 1×；而 `robot_state_publisher` 发布的描述有 7 个 sensor /
  3 个 IMU → 一旦重启 world 就会回到 3×。**倍率 = 模型里带插件的 IMU 传感器个数**，与 xacro 里
  写了几处无关（没被 spawn 的不算）。

## ★仿真 velodyne 点云没有逐点时间（2026-09-18 实测，正在运行的 world 上量的）
- `/robot1/velodyne/points`（gz `gpu_lidar` → `gz.msgs.PointCloudPacked` → ros_gz_bridge）字段只有
  `x,y,z,intensity`(f32, off 0/4/8/16) + `ring`(u16, off 24)，`point_step=32`，640×16=10240 点，
  `frame_id=velodyne`，`is_dense=false`，**没有 `time`**。
- 根因：gz-sim 的 lidar 传感器（lidar/gpu_lidar/ray/gpu_ray）是**一次渲染出整帧**的瞬时扫描，
  SDF 里**没有**逐点时间/扫描线时序的配置项（那是真实驱动才有的）。所以 `<noise>` 之外
  也没有"运动畸变"，仿真点云天然是零畸变。
- 附带实测：x/y/z 里有 `inf`（未命中/超量程）与少量 NaN；`intensity` **恒为 0**；
  stamp 间隔恒 100.0ms 仿真时间；gz 同时发 `<topic>`(LaserScan) 和 `<topic>/points`(PointCloudPacked)。
- PCL 缺字段行为实测（`g++` 单文件复刻字段布局 + `pcl::fromPCLPointCloud2<velodyne_ros::Point>`）：
  打印 `Failed to find match for field 'time'.`，`time` 被**值初始化为 0**（不是随机垃圾，
  因为 `std::vector::resize` 走 `T()` 值初始化）→ lightning `VelodyneHandler` 的
  `given_offset_time_ = (last.time > 0)` 判为 **false** → 走 **yaw/ring 合成时间**分支
  （omega_l=3.61 deg/ms，合成 0~100ms 假时间），再交给 `UndistortPcl` 做运动补偿
  → **给本来零畸变的仿真点云人为注入 100ms 的假畸变**（走路时约 5cm/5° 量级，随运动变化）。
- 修复方向：①改 lightning（检测 msg 无 `time` 字段则所有点 time 置常量，跳过合成分支）；
  或 ②桥接后加转换节点补一个**常量** time 字段（注意必须 >0 才会走 given_offset_time_ 分支，
  但那样 `UndistortPcl` 里 `head->offset_time` 与它比较后 dt≈0，数值上≈恒等，等价于无畸变）。
- velodyne 平台外参（`robot_VLP_D435i.xacro`）：`velodyne` 相对 `base_link` = xyz(0.22,0,0.09)；
  **`velodyne_imu` 与 `velodyne` 完全重合（origin 0 0 0）→ extrinsic_T/R = 单位阵**，
  与 r41 的 mid360 情况一致，lightning 配置可直接填 I。
- ⚠ IMU 仍多倍发布（2026-09-18 实测，上游 xacro 版本）：`/robot1/velodyne_imu` 声明 200Hz 实测 **604Hz**，
  `/robot1/imu_plugin/out` 声明 100Hz 实测 **304Hz**（均 ≈3×）。即"每个 IMU 传感器级都写了
  `libgz-sim-imu-system.so`"这个坑在本工作区**依然存在**（rollback 把修复也回滚了）。

## ★方案A已实施：lightning 新增 lidar_type=5 / SimulatedHandler（2026-09-18，编译+实跑验证通过）
- `src/core/lio/pointcloud_preprocess.h`：`enum class LidarType { ..., SIMULATED = 5 }` + 声明 `SimulatedHandler`
- `src/core/lio/pointcloud_preprocess.cc`：新增独立函数 `SimulatedHandler`（**现有 VelodyneHandler/Oust64/RoboSense 一行未改**）
  - 不走 `pcl::fromROSMsg`，改成按字段名查 offset + memcpy 读原始 buffer → **彻底避开 PCL 缺字段警告/类型限制**，
    且 x/y/z/intensity 任一缺失都能给出明确错误；兼容 FLOAT32/FLOAT64
  - 每点 `time = 0`。**依据**：`ImuProcess::UndistortPcl` 的补偿内层循环是
    `for (; it_pcl->time/1000 > head->offset_time; ...)`，而 `IMUpose[0].offset_time` 硬编码为 0 →
    time=0 的点一次都不被变换，如实保留瞬时几何。`SyncPackages` 那边 `points.back().time/1000 = 0 < 0.5*mean_scantime`
    → 走兜底分支 `lidar_end_time_ = begin + lidar_mean_scantime_(=lo::lidar_time_interval=0.1s)`，与 10Hz 仿真帧长一致 ✔
- `laser_mapping.cc` / `localization.cpp`：各加一个 `lidar_type == 5` 分支（老分支未动）
- 新增配置 `config/go2_gazebo_sim.yaml`（由 go2_sim.yaml 复制，不改原文件）：
  `lidar_topic=/robot1/velodyne/points`、`imu_topic=/robot1/velodyne_imu`、`lidar_type=5`、
  `point_filter_num=2`、`livox_lidar_topic=/sim/unused_livox_custom`（防止同域 livox 驱动串数据）、
  `extrinsic_base_imu_T=[0.22,0,0.09]` R=I（= URDF base→velodyne→velodyne_imu）
- **实跑验证（30s，对着正在跑的 warehouse 仿真）**：
  - 启动日志 `Using Simulated Lidar (instantaneous scan, no per-point time)`、静态 TF `base_link->imu_link t=0.22 0 0.09`
  - `first scan pts: 4939`（10240 原始 → point_filter_num=2 → 扣 inf/NaN/盲区）
  - 每帧 `pts 10240 | dt 100.0ms | lio 4~11ms | e2e ~5ms | queue <1ms | pending 0`，30s 共 ~290 帧
  - **`[warning]` 只有 1 条**（启动首帧 `No point, skip this scan!`，无害），无 error，无 PCL 缺字段警告
- **重要纠正（之前担心错过）**：gz-sim 里 IMU 写了 `<gravity>false</gravity>` **仍然输出重力**！
  实测 `fastlio IMU Initial Done, grav: 0 0 -9.809` —— 不写 false 会更好，但**不影响 lightning 初始化**，
  所以不用改 xacro。（我原先按 SDF 语义推断"加速度≈0 会导致 FAST-LIO 除零"，实测证伪。）

## 评估：/home/gmd/Lidar_nav2_ws/src/livox_laser_simulation_RO2 能否拿来替代（2026-09-18，读码结论）
**不能。** 三条硬理由：
1. **Gazebo Classic 插件**：`class LivoxPointsPlugin : public RayPlugin` + `gazebo_ros/node.hpp`
   + `gazebo::physics::LivoxOdeMultiRayShape`（ODE 多射线）→ gz-sim 6/Ignition **加载不了**。
   要用它就得把 Go2 仿真整体切回 Gazebo Classic（ros2_control 也要从 `gz_ros2_control/GazeboSimSystem`
   换成 `gazebo_ros2_control/GazeboSystem`）。
2. **CMake 硬编码过期库**：`target_link_libraries(... libprotobuf.so.9 libboost_chrono.so.1.71.0)`；
   本机是 protobuf **23** / boost **1.74** → 链接直接失败（`libprotobuf.so.9` 不存在）。
3. **它的"逐点时间"是假的，比没有更糟**：`p.offset_time = (now()-start_time).count()`
   —— 是 **boost 墙钟**"处理第 i 个点花了多少 ns"，不是 CSV 扫描模式里的 `AviaRotateInfo.time`
   （结构体里明明有 time 字段却没用）。量级 ~10ms 且随 CPU 负载抖动 → 会把 lightning 的
   **livox 路径（lidar_type=1）带进"用抖动去畸变"分支**。
   附带：PointCloud2 走的字段是 `setPointCloud2FieldsByString(2,"xyz","rgb")`（没有 intensity/ring/time，
   rgb 还没填=全 0）；未命中点被 `range=0` → **产生一堆 (0,0,0) 原点垃圾点**。
- **本质结论**：仿真里补逐点时间 = 给瞬时几何编一个不存在的采集时刻；LIO 再拿它去"去畸变"
  → 只会把干净几何弄脏。**要么全点同一时刻（推荐），要么仿真侧真的让几何随时间变化（做不到）。**

## ★Gazebo 里狗不动、RViz 却在动（2026-09-18，已修，实测通过）
- 症状：发控制指令后，`/robot1/joint_group_controller/commands` 有 59.5Hz 数据（内容是正确的站立姿态
  `0/0.8615/-1.8826` 重复 4 组），但 `/robot1/joint_states` 恒 ≈0（1e-4 级漂移）、
  `ign model -m robot1_my_bot` 两次采样位姿完全相同 → **物理关节根本没跟随指令**。
  RViz 里看到的"动"来自 ROS 侧里程计/TF，跟物理无关。
- **根因：同一批 12 个腿关节被两套 `<ros2_control>` 重复声明**
  | 位置 | 名称 | command_interface |
  |---|---|---|
  | `gazebo_VLP_D435i.xacro` / `gazebo.xacro`（整机） | `${robot_name}_GazeboSystem` | **position**（与 `ros_control.yaml` 的 `position_controllers/JointGroupPositionController` 匹配） |
  | `leg.xacro`（每腿一份 ×4） | `${name}`(rf/lf/rh/lh) | **effort**（Gazebo Classic 时代遗留；上游用 `gazebo_ros2_control/GazeboSystem`） |
  → resource_manager 刷 20+ 条 `(hardware 'rf'): 'rf_hip_joint/position' state interface already in
  available list ... multiple calls to 'configure'`；同一 gz 关节同时存在 position/effort 命令接口
  → position 控制器的写入到不了物理关节。
- **修法**：删掉 `leg.xacro` 里那份按腿的 ros2_control 块（原地留注释说明），关节定义只保留
  `gazebo_*.xacro` 的整机 system 块。
- **验证**：告警 20+ → **0**；`joint_states` 变成 `-1.8826/0.8615`（跟随站立指令）；
  发 `/robot1/cmd_vel` 0.2m/s 约 2s 后 `ign model -m` 的 x 从 **-0.144 → 1.465 m**（真的走了）。
- **指令入口（记牢）**：`/robot1/cmd_vel`(Twist) → `cmd_vel_pub` 节点 → `/robot1/robot_velocity`
  (quadropted_msgs/RobotVelocity) → `robot_controller_gazebo.py`(节点名 quadruped_controller)
  → `/robot1/joint_group_controller/commands` → gz。
  RViz 的 "2D Nav Goal" 发到 `/robot1/goal_pose`，**该话题 0 个订阅者**，不会驱动狗。

## 仿真 launch 的 EKF 默认关闭（2026-09-18）
- `gazebo_go2_sensors.launch.py` / `gazebo_go2_self.launch.py` 新增 `enable_ekf`（默认 **false**），
  EKF 节点加 `condition=IfCondition(enable_ekf)`；`launch.py` 声明并**透传** enable_ekf
  （不透传的话 `ros2 launch gazebo_sim launch.py ... enable_ekf:=true` 会被静默忽略）。
- 原因：仿真里用 lightning 做里程计/定位，不需要 robot_localization；而且 `ekf.yaml` 的
  `publish_tf: true` 会与 `odom` 节点（QuadrupedOdometryNode，`enable_odom_tf: True`）
  **重复发布 odom→base_link**。
- 副作用：`/robot1/odometry/filtered` 消失；仿真 RViz 的 Odometry 显示项因 launch 重映射
  `("/odom","odometry/filtered")`（rviz_launch.py:77）指向它 → 该项会空。要看里程计改成
  `/robot1/odom`（sim）或 `/lightning/odom`（lightning）。
- 恢复 EKF：`ros2 launch gazebo_sim launch.py sensors:=true world:=warehouse.sdf enable_ekf:=true`

## 未解
- 无（IMU 速率语义已澄清：就是声明值 1×，之前的多倍是重复发布造成的）。
