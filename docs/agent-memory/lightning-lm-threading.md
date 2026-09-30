# lightning-lm 建图线程模型（2026-09 改造，已验证）

## 架构
- ROS executor **单线程**（`rclcpp::spin(node_)`），回调只做「预处理 + 入队」
- `SlamSystem::RunWorker()` 工作线程：`Run()` + 全部发布（TF/点云/关键帧/回环/栅格）
- 开关：`system.threaded_lio`（默认 true；false 则回调内同步跑，仅回滚用）
- 线程生命周期：`Init()` 末尾启动；`Spin()` 返回后与 `~SlamSystem()` 各调一次 `StopWorker()`（幂等）。
  **必须在 `rclcpp::shutdown()` 之前停**，否则 publish 会抛异常。

## 锁（laser_mapping）
- 锁序固定 **`mtx_state_` → `mtx_buffer_`**，反向必死锁
- `mtx_buffer_`：只保护 `lidar_buffer_/time_buffer_/imu_buffer_` + 时间戳；只做搬运，禁止重计算
- `mtx_state_`：EKF/ikd-Tree/关键帧/`scan_*`；`Run()` 全程持有
- `ProcessIMU` 的 UI 更新用 `GetIMUStateNonBlocking()`（try_lock），**绝不阻塞 IMU 回调**
- 对外读接口（`GetState/GetOptPose/GetKeyframe/GetScanDownWorld/GetAllKeyframes/GetGlobalMap`）内部加锁；
  `GetGlobalMap` 为「锁内取 kf 快照、锁外拼接」

## 关键实测数字（RK 平台 / rslidar 32线 / lio_data0909）
| 项 | 值 |
|---|---|
| `Proc Lidar`（回调，含预处理） | **1.3 ms** |
| `LIO Run`（工作线程单帧） | **4.5 ms** |
| 关键帧增量建图 | 0.4 ms |
| 空转轮询间隔 | 1 ms；积压阈值 10 帧（`kMaxPendingScans`） |

## QoS：点云必须 RELIABLE
- 驱动 `create_publisher(topic, ros_queue_length)` → **RELIABLE**，`config.yaml` 里 `ros_queue_length: 100`
- 实测 **best_effort 订阅 40s 回放只收到 89/400 帧（丢 78%）**；reliable 零丢帧
- reliable 之所以不再反压驱动：回调只 1.3ms，reader 队列不积压
- **反压实证**：改造前单线程时 40s 回放只处理 244/400 帧（rosbag2_player 的 publish 被拖慢）

## 验证方法（可复用）
1. `colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release`
2. 起节点：`ros2 launch lightning r41_online_slam.launch.py config:=<src>/config/r41_rk_slam.yaml`
3. `timeout -s INT 40 ros2 bag play lio_data0909 -r 1.0`
4. `pkill -f "[r]un_slam_online"`（**必须让它退出**，`ofstream` 缓冲才会 flush）
5. 看 `wc -l data/odom_log.txt` + 退出时的 `Timer::PrintAll` 汇总

## 坑
- `Timer::Evaluate` 返回 void；要取被包裹函数的返回值需用捕获变量
- 只在工作线程里 `lio_->Run()` 前先查 `HasPendingScan()`，否则空闲时会每 1ms 刷 `sync package failed`
- launch 的 `name=` 会以 `__node:=` 重映射覆盖代码里的节点名（`lightning_slam` → `run_slam_online`）
