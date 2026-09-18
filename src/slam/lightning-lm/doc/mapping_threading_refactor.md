# 建图模块线程模型改造（M0 ~ M3）

> 起因：实时建图时，雷达驱动出帧频率从 10Hz 逐渐掉到 5Hz。
> 结论：根因是**点云回调阻塞 DDS reader 队列 → 反压驱动发布线程**。
> 本文记录诊断过程、改造方案、锁设计、实测数据与回滚方式。

## 1. 问题现象

- 启动建图算法后，`rslidar_sdk` 出帧频率由 10Hz 缓慢下降到 5Hz（渐进，非突降）
- 频率下降**不可逆**，只有重启驱动才恢复
- `ros2 topic hz /rslidar_points` 观测到的是订阅端到达间隔

## 2. 定位过程

关键在于每一步都用于**排除一种假设**，而不是猜。

| 实验 | 结果 | 排除了什么 |
|---|---|---|
| 只注释回调体、保留订阅 | 仍降频 | **无效实验**：reliable reader 队列被填满后不再消费，反压反而更严重 |
| 完全注释掉 `create_subscription` | 不再降频 | 确认问题在「订阅」而非「算法本身」 |
| 订阅 QoS 改 `best_effort` | 不降频，但 40s 回放只收到 89/400 帧（丢 78%） | 反压来自 reliable ack 链，但 best_effort 代价过大（大点云 UDP 分片丢包） |

### 2.1 根因

驱动侧（`rslidar_sdk`）：

```cpp
// source_pointcloud_ros.hpp:476 —— 无 QoS 参数即 RELIABLE + KEEP_LAST(depth)
pub_ = node_ptr_->create_publisher<PointCloud2>(ros_send_topic, ros_queue_length);  // config: 100

// source_driver.hpp:251 —— 唯一的发布线程，同步 publish
void SourceDriver::processPointCloud() {
  while (!to_exit_process_) {
    auto msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL) continue;
    sendPointCloud(msg);          // ← 阻塞在这里
    free_point_cloud_queue_.push(msg);
  }
}
```

SLAM 侧（改造前）：单线程 executor，回调里串行跑完「预处理 + LIO + 回环 + 栅格 + 地图发布」。

链路：**回调慢 → reader 队列积压 → writer 需保留样本等 ack → `publish()` 阻塞 → 驱动出帧节奏被拉长 → 10Hz 渐变到 5Hz**。

最有力的旁证：改造前 40s 回放只处理 244 帧（应为 400），说明 `rosbag2_player` 的 `publish()` 同样被拖慢了。

## 3. 改造方案

### 3.1 线程模型

```
ROS executor（单线程）              LIO 工作线程（RunWorker）
  ├─ IMU  回调：入队（微秒级）   →   lidar_buffer_ / imu_buffer_（mtx_buffer_ 保护）
  ├─ 点云 回调：预处理 + 入队    →   ├─ Run()：EKF + 配准 + 增量建图（mtx_state_ 保护）
  └─ 服务回调：save_map/path          └─ 发布：TF / 点云 / 回环 / 栅格 / 路径
```

- 回调只做「预处理 + 入队」，实测 **1.3 ms**
- 真正的计算与全部发布在工作线程，实测单帧 `Run()` **4.5 ms**
- 积压发生在应用层 `lidar_buffer_`，**不再影响 DDS reader 队列**，反压链路被解耦

### 3.2 关键实现点

- `RunWorker()` 用 `while` 循环而非「一帧回调一次」，可把积压帧尽快追平
- `HasPendingScan()` 做空转保护：无数据时不进 `Run()`，避免刷出无意义的 `sync package failed`
- `DropStaleScans(kMaxPendingScans = 10)`：工作线程跟不上时丢最旧的帧，防止内存无限增长
- 发布时戳取 `lio_->GetState().timestamp_`（本次实际处理的那一帧），而非「最新收到的帧」
- 线程生命周期：`Init()` 启动；`Spin()` 返回后与 `~SlamSystem()` 各调一次 `StopWorker()`（幂等）。
  **必须在 `rclcpp::shutdown()` 之前停**，否则工作线程会在 ROS 上下文失效后 publish

### 3.3 锁设计（`laser_mapping`）

改造前只有一把 `mtx_buffer_`，且 **`Run()` / `SyncPackages()` 完全不加锁** —— 单线程下靠串行侥幸无事，一旦并发即为数据竞争。

现在：

| 锁 | 保护范围 | 约束 |
|---|---|---|
| `mtx_buffer_` | `lidar_buffer_` / `time_buffer_` / `imu_buffer_` + 相关时间戳 | 只做搬运，禁止持锁做预处理 / 配准 |
| `mtx_state_` | EKF 状态、ikd-Tree、关键帧、`scan_*`、`measures_` | `Run()` 全程持有 |

**锁序固定为 `mtx_state_` → `mtx_buffer_`，反向获取必死锁。**

其他要点：

- `ProcessPointCloud2` 把 `preprocess_->Process()`（重计算）移到 buffer 锁之外
- `ProcessIMU` 的 UI 更新移出锁，并改用 `GetIMUStateNonBlocking()`（`try_lock`），**绝不阻塞 IMU 回调**
- 对外读接口（`GetState` / `GetOptPose` / `GetKeyframe` / `GetScanDownWorld` / `GetAllKeyframes` / `GetGlobalMap`）内部加锁
- `GetGlobalMap()` 改为「锁内取关键帧快照、锁外遍历拼接」，避免 `save_map` 把 LIO 线程长时间堵住

### 3.4 QoS 决策

**点云订阅必须与驱动一致使用 RELIABLE。**

- 依据 1：`best_effort` 订阅实测 40s 回放仅收到 89/400 帧（丢 78%），reliable 靠重传可全补齐
- 依据 2：reliable 之所以不再反压驱动，是因为回调只 1.3ms，reader 队列不会积压

> 若将来回调再次变慢，需要重新评估这一结论。

IMU 保持 RELIABLE，depth 放大到 200（1s 量级）。

## 4. 实测数据

环境：RK 平台 / rslidar 32 线 / bag `lio_data0909`（10Hz）/ 40 秒实时回放。

| 指标 | 改造前 | 改造后 |
|---|---|---|
| `Proc Lidar`（回调） | 含 LIO，≥100ms 级 | **1.29 ms** |
| `LIO Run`（工作线程） | 在被阻塞的回调中 | **4.55 ms** |
| 增量建图 / ikd-Tree 加点 | — | 0.44 / 0.34 ms |
| 40 秒处理帧数 | 244 / 400 | **399 / 400（零丢帧）** |
| IMU 回调阻塞 | 是 | 否（`try_lock`） |
| 退出 | — | `thread stopped` → `process has finished cleanly` |

回环优化正常触发（`optimize finished, loops: 2`），关键帧 173 个，轨迹与改造前一致。

## 5. 配置与回滚

```yaml
system:
  threaded_lio: true    # 缺省 true；false 则回到「回调内同步跑 LIO」的旧行为（会有 WARN 提示）
  log_lidar_time: true  # 缺省 false；true 则逐帧输出点数 / LIO 耗时 / 端到端延迟
```

`threaded_lio=false` 时会在启动日志中给出显式警告，仅用于回滚排查，不要长期使用。

### 5.1 逐帧耗时日志

`log_lidar_time=true` 时每帧输出一行（INFO 级）：

```
[lidar] pts  86400 | lio   4.47 ms | e2e   4.97 ms | queue  0.50 ms
```

| 字段 | 含义 |
|---|---|
| `pts` | 本帧点云点数 |
| `lio` | 本帧 EKF + 点面配准 + 增量建图耗时 |
| `e2e` | 从点云入队到本帧处理完（含排队） |
| `queue` | `e2e - lio`，即排队等待时间 |

判断方法：`queue` 恒小于 1ms 说明工作线程跟得上、没有积压；`queue` 持续增大说明消费端
已跟不上生产端（此时可看 `DropStaleScans` 是否开始告警）。

> 注意：`Timer::Evaluate(..., "Proc Lidar", true)` 中的 `print` 分支走的是 `LLOG_DEBUG`，
> 而缺省 `log.level` 是 `info`，所以**改动前后都不会逐帧打印回调耗时**；
> 退出时 `Timer::PrintAll` 的汇总（均值 / 中位数 / 95 分位）不受日志级别影响，一直是可用的。
> 若只是想让回调耗时逐帧可见，也可临时把 `common.log.level` 设为 `debug`，
> 但会连带打开 `laser_mapping` 内部的大量 DEBUG 日志。


## 6. 与定位模式的关系

**定位模式（`LocSystem` / `Localization`）本来就是多线程的，不是本次改造引入的。**

- `localization.h:120`：`sys::AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;`
- `localization.cpp:82-86`：`SetProcFunc(LidarOdomProcCloud)` + `Start()` → 起独立线程
- ROS 回调（executor）只做 `preprocess_->Process` + `AddMessage`；
  `lio_->ProcessPointCloud2()` / `Run()` / 定位计算都在 `AsyncMessageProcess::proc_` 线程上

这与本次建图改造的**思路一致**（回调轻量化 + 独立处理线程）。

### 6.1 本次改造顺带修复了定位模式的数据竞争

改造前 `LaserMapping` 里 `ProcessIMU` 持 `mtx_buffer_`，而 `Run()` / `SyncPackages()` **不加锁**。
定位模式下这两者恰好分处两个线程：

- `ProcessIMUMsg`（executor 线程）→ `lio_->ProcessIMU()`
- `LidarOdomProcCloud`（proc_ 线程）→ `lio_->Run()`

即 `imu_buffer_.emplace_back()` 与 `imu_buffer_.pop_front()` 并发操作同一个 `std::deque` —— 真实的数据竞争。
加上 `SyncPackages()` 的锁之后该隐患已消除。

### 6.2 需要留意的副作用（待办）

`Localization::ProcessIMUMsg` 里用的是**阻塞版** `lio_->GetIMUState()`，而它现在需要等 `mtx_state_`。
`Run()` 持锁约 4.5ms（95% 6ms），IMU 200Hz 周期 5ms，因此该回调可能被推迟约一个 Run 周期。

- 不会崩、也不会丢数据（executor 队列 depth 200），但 `ProcessDR` 会被相应推迟
- 若要消除，可让 `ProcessIMUMsg` 走类似 `GetIMUStateNonBlocking()` 的路径，或把「入队 + 读状态」合并为一次持锁操作
- 属于定位模块的独立优化项，本次未改动 `LocSystem` / `Localization` 的任何代码

## 7. 验证方法（可复用）

```bash
# 1) 编译
colcon build --packages-select lightning --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2) 起节点（config 用 src 下的，避免 install 里的旧副本）
ros2 launch lightning r41_online_slam.launch.py \
    config:=$PWD/src/slam/lightning-lm/config/r41_rk_slam.yaml

# 3) 实时速率回放
timeout -s INT 40 ros2 bag play lio_data0909 -r 1.0

# 4) 停止进程 —— 必须让它正常退出，ofstream 缓冲才会 flush
pkill -f "[r]un_slam_online"
sleep 3

# 5) 看处理量与耗时
wc -l data/odom_log.txt          # 40s @10Hz 应接近 400
```

退出时 `Timer::PrintAll` 会打印 `Proc Lidar` / `LIO Run` / `Preprocess` 的均值、中位数与 95 分位，是判断回调是否变慢的第一手依据。

## 8. 后续建议

1. **真机确认**：bag 回放已证明消费端不再造成反压，但需实机确认驱动恢复稳定 10Hz
2. **RViz**：`showbodypc.rviz` 里 `/rslidar_points` 的 display 默认是 Reliable，它是另一个可靠 reader；
   调试时建议改为 `Best Effort` 或直接关闭该 display
3. **驱动侧**：`rslidar_sdk/config/config.yaml` 的 `ros_queue_length: 100`（reliable + keep_last(100)）
   可降到 10，减少 writer 样本保留压力
4. **架构统一**：建图侧的工作线程是手写的 `std::thread` + 轮询，定位侧用的是现成的
   `sys::AsyncMessageProcess`（条件变量驱动）。若要统一风格，需注意语义差异 ——
   `AsyncMessageProcess` 由「消息到达」驱动，而 LIO 需要「IMU 攒够即可推进」，
   纯事件驱动会漏掉「点云已入队、IMU 后到」的场景，因此建图侧保留轮询是有意的
