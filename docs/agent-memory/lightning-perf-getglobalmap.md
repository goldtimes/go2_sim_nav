# lightning 性能：worker 被 GetGlobalMap 拖垮（2026-09-17）

## 症状
- `[lidar] recv falling behind: arrival_dt 712ms vs stamp_dt 100ms` —— **周期性地反复出现**
- 同时 `[lidar] ... lio 4ms | queue 280ms | pending 3`
- 但如果只看 worker 会觉得它"不忙"（lio 只要 4ms）→ 容易误判方向

## 关键判别方法
`Timer::PrintAll()` 一出来真相就清楚了：
```
[GetGlobalMap]        avg 210.382 ms, med 210.791, 95%: 388.151  called 233   ← 元凶
[LIO Run]             avg   5.083 ms
[Proc Lidar]          avg   1.090 ms, med 0.864, 95%: 1.771                   ← 回调很快
[PublishMappingTf]    avg   0.311 ms
[Preprocess]          avg   1.085 ms
```
**判据**：`arrival_dt` 突刺 200~700ms 但 `Proc Lidar` 只有 1ms
→ 阻塞**不在回调内部**，而在回调之前（DDS 收包 / 执行器调度）

## 根因
`slam.cc::ProcessOnce` 里「每 3 个关键帧发一次 global_map」：
- 关键帧约 0.18s 一个 → 每 0.54s 发一次
- `GetGlobalMap()` 要拼接**全部关键帧**并逐帧体素滤波 → 697 kf 时单次 **210ms（P95 388ms）**
- 233 次 × 210ms = 49s / 126s = **worker 40% 的时间**
- ⚠️ **曾经的错误猜测**：一开始以为「toROSMsg + DDS 发布大点云拖慢 participant」。
  后续用 `PublishGlobalMap` - `GetGlobalMap` 实测差值 = **5.27ms / 490ms = 1.08%**，
  发布本身几乎不花钱 → **纯计算问题**，不要再去开"独立发布线程"。
  （`arrival_dt` 突刺的机制是：worker 被占住 → 帧积压 → 回调批量到达）

## 已修
- `system.global_map_pub_interval`（秒，默认 5.0，0=不发布）替代"每 3 个关键帧"
- 加 `get_subscription_count() > 0` 检查（没人订阅就不算不发；grid_map 早就有，slam 漏了）
- 计时拆两段：`PublishGlobalMap`（外层）/ `GetGlobalMap`（内层）
- 预期：39% → 4%（有 rviz 订阅时），无订阅时 → 0%

## 其他相关修复（同批）
- `system.lio_worker_nice`：原来硬编码 `nice(10)`；x86 设 0，RK3588 保持 10
  （nice 只在有竞争时起作用；x86 LIO 仅占 ~4% 单核）
- **`queue`/`e2e` 伪影**：原来 `SlamSystem` 用单个 atomic 记"最新到达时刻"，
  worker 追赶积压时该值不刷新 → e2e 随处理进度线性虚增（看着像延迟在涨）。
  改为逐帧携带：`LaserMapping::arrival_time_buffer_`（与 `lidar_buffer_` 平行）
  + `GetLastProcessedArrivalTime()`
- 逐帧日志加 `pending` 列；`run_loc_online` 补 `Timer::PrintAll()`
- **顺带修的 pre-existing bug**：`ProcessPointCloud2` 里时间回退时
  `lidar_buffer_.clear()` **没清 `time_buffer_`** → 两个平行队列错位

## 自洽性检验（验证 queue 修复是否生效）
`pending 3` ↔ `queue 280ms` ≈ 3 × 100ms 帧周期 ✓
修复前则是每帧精确 +4.8ms（= lio 值）线性递增 → 明显是伪影

## 同类问题 2：lightning/path 无界增长（已修）
- `PublishMappingTf` 里 `path_.poses.push_back(ps)` 每帧累积永不清理，
  而 `path_pub_->publish(path_)` **每帧把整个 vector 序列化一遍** → O(n)/帧、O(n²) 累计
- 实测证据：代码没改，但 `PublishMappingTf` 从 **0.311ms（1263 帧）→ 0.989ms（3190 帧）**
- 修复：`system.path_pub_interval`（秒，默认 1.0，0=不发布）+ 订阅者检查；
  **`path_` 仍每帧累积**（`save_path` 导出轨迹要用），只是不每帧发
- loc 侧同样处理（`LocSystem::Options::path_pub_interval_`），
  定位会连续跑几小时，这个 O(n²) 更值得防

## 成本模型（2026-09-17 分段计时实测，1663 kf / 57 次调用）
子计时名：`GGM:kf-voxel` / `GGM:trans+cat` / `GGM:final-voxel`（在 `laser_mapping.cc::GetGlobalMap`）

| 阶段 | 单价 | 占比 |
|---|---|---|
| 逐帧体素降采样 | 0.377 ms/kf | **62%** |
| 逐帧变换 + 拼接 | 0.082 ms/kf | 14% |
| 最后一次全量体素滤波 | 120 ms（常数） | **24%** |

$T \approx K \times 0.459\text{ms} + 120\text{ms}$（K = 本次快照的关键帧数）
- 57 次调用的 K 平均 ≈ 823（kf 从 0 线性涨到 1683）→ 823×0.459+120 = **497ms** ✓ 实测 497.472
- 末次 K=1663→1683 → ~892ms ✓ 实测 P95 950ms

**判读 `Timer::PrintAll()` 的坑**：`called times` 是**采样数**（上限 `kMaxSamples=2000`），不是真实调用次数。
`GGM:kf-voxel` 显示 2000 是饱和了，真实次数是 57×K。要靠「GetGlobalMap 总时长 − final-voxel」反推。

### ❌「只对整体滤波一次」是反优化
关键帧存的是 `scan_undistort_`（**未降采样**，~14400 点；`point_filter_num: 6` 抽的点），
不是 `scan_down_body_`（那才是 `filter_size_scan: 0.2` 降采样结果，关键帧**没用它**）。

| 方案 | 拼接量 | 最后滤波输入 |
|---|---|---|
| 现状（逐帧 0.1 + 全量 0.1） | K × 几千 | ~1.6M |
| 只全量滤一次 | K × 14400 = **11.9M** | 11.9M（~190MB）→ final-voxel 涨到 ~700ms，**慢 2.2×** |

### ✅ 已修：逐帧降采样结果缓存（缓存后 T ≈ K×0.082ms + 120ms）
`Keyframe::GetCloudDownsampled(res)`（`common/keyframe.h`）—— 懒计算 + 按 res 缓存。
为什么安全：`cloud_` 在构造函数里赋值后**永不变**（没有 `SetCloud`），
且降采样在 **LIDAR 系**做，与位姿无关 → 同一 res 结果恒定。
- 用**独立的 `cache_mutex_`**，不要并进 `data_mutex_`：
  降采样要 0.3ms，持有 `data_mutex_` 会挡住 worker 线程的 `GetLIOPose()`/回环 `SetOptPose()`。
- 缓存上限 4 个 res（`cloud_downsampled_.size() >= 4` 时 clear），防止被传杂乱 res 时无界增长。

**代价 / 遗留问题**：缓存多占 ~67MB（1683 kf × ~2500 点 × 16B）。
更大的一笔是**关键帧本身就存了 ~387MB 原始点云**（14400 点 × 1683 kf × 16B）。
→ 真正的下一步：关键帧改存降采样后的点云（省 320MB 内存 + 逐帧滤波彻底归零），
  但会改变回环/g2p5/`save_map` 的输入分辨率，需单独评估。

### ✅ 已改：rviz 走「增量发布」，整图不再定期重算（2026-09-17）
用户的判断是对的：**rviz 本来就会累积**（点云显示项的 `Decay Time`，0 = 永不衰减），
所以定期重发整图纯属浪费。现在：

- `slam.cc::PublishGlobalMap(sub_count)` 按 `system.global_map_pub_mode` 分三种：
  | 模式 | 行为 | 用途 |
  |---|---|---|
  | `incremental`（默认） | 只发「新出现的关键帧」，跨调用体素去重 | rviz |
  | `full` | 每次重算整图（老行为，1663kf ≈ 500ms） | init_selftest / pcd2pgm 这类要完整点云的 |
  | `off` | 不发 | — |
- `LaserMapping::GetNewKeyframesCloud(size_t& io_published_count, use_lio_pose, res)`
  取 `[游标, 快照末尾)`，返回后把游标推到末尾 → 单次成本 = **O(新增关键帧)**，与地图大小无关。
- **跨调用体素去重**（`utils/pointcloud_utils.h::VoxelKey` + `pub_voxels_`）：
  没有它的话同一面墙会被经过它的 N 个关键帧各发一遍，rviz 点数随运行时间无界增长。
  有它则 rviz 最终看到的**密度与旧的全量整图完全一致**（都是唯一体素集），
  但运行 1 小时点数也不涨。
- `pub_voxels_` 只在有订阅者时维护；订阅者归零就 `swap` 释放 → 内存只在使用 rviz 期间占用。
- **订阅者 0→非0 = rviz 刚打开/重连** → 游标清 0 重发一遍（否则新开的 rviz 只能看到后半段地图）。
- **回环重发**：`LoopClosing::SetLoopClosedCB`（`loop_closing.cc::PoseOptimization` 是**唯一**
  会改写历史位姿的地方）→ 置 `map_pose_corrected_` → 下次发布清空游标+体素表全量重发，
  并打 `LLOG_WARN` 提示"rviz 里的点位置已失效，有重影请重开显示"。
  rviz 无法撤回已画出的点，这是唯一可行的处理方式。
- **重命名**：子计时 `GGM:kf-voxel/trans+cat/final-voxel` → `Map:kf-voxel/trans+cat/final-voxel`
  （因为 `GetGlobalMap` / `GetNewKeyframesCloud` 共用 `BuildCloudFromKeyframes`）。
  新增 `GetMapFull` / `GetMapDelta` / `MapVoxelDedup`。
- `global_map_pub_interval` 默认 5.0 → **1.0**（增量下几乎不花时间）；新增 `global_map_pub_res`（默认 0.1）。
- **rviz 配置必须把该显示项的 `Decay Time` 设为 0**（`showbodypc.rviz` 原来是 10000 → 已改 0；
  `showglobalmap.rviz` 本来就是 0）。忘了设的话地图只会显示最近 10s，看着像"地图在丢"。

**LaserMapping 重构**：抽出 `BuildCloudFromKeyframes(kfs, begin, end, ...)`，
lidar→IMU 外参的处理只在这一处（避免以后再出现"某个调用点漏乘外参"的 bug）；
`GetGlobalMap` 就是 `[0, size)` + `Map:final-voxel`。

**注意**：`/lightning/global_map` 的语义从"整图"变成了"增量"，
`init_selftest`/`pcd2pgm` 这类直接拿它当整图用的流程要临时切 `full` 模式
（已在 `doc/loc_init_selftest.md` 加警告）。

### ✅ 已改：关键帧按分辨率存储（`fasterlio.kf_cloud_res`，2026-09-18）
**又一个内存大户：关键帧存的是未降采样原始扫描。**
- `scan_undistort_` ≈ 14400 点/帧（`point_filter_num: 6` 抽点后）
  —— 验证方法：`Map:kf-voxel` 0.357ms ÷ PCL VoxelGrid ~25ns/点 ≈ 14000 点 ✓
- `PointXYZIT` = 32B（xyz+intensity 各 4B，double time 8B，对齐到 32）
- 14400 × 32B = **450KB/帧**；1687 帧 = **~760MB**；关键帧 ~5.6 个/s → **~150MB/分钟**

改法：`MakeKF()` 里就按 `kf_cloud_res_` 降采样再存（用**局部** VoxelGrid！成员 `voxel_scan_`
的 leaf size 是 `filter_size_scan`，归 Run() 用，动它会把 LIO 前端带偏）；
`Keyframe` 记 `cloud_res_`，`GetCloudDownsampled(res)` 在 `res == cloud_res_` 时**直接返回**。

关键性质：**输出地图与改前完全一致** —— 逐帧 0.1 体素化（LIDAR 系）这个操作没变，
只是从"每次 GetGlobalMap 时做"提前到"建帧时做一次"。

| 指标 | 改前 | 改后 |
|---|---|---|
| 关键帧点云内存（1687 帧） | ~760MB | **~134MB** |
| `SaveMap` 全量建图 | ~906ms | **~305ms** |
| `Map:kf-voxel` | 602ms | **~0** |
| `MakeKF` | ~0.2ms | ~0.56ms（摊到全程 ~0.2% CPU） |

影响面：回环 submap 本来还会再 `VoxelGrid(r*0.1)`、g2p5 是 2.5D 投影栅格、
`save_map` 的 `keyframes/kf_*.pcd` 变成 0.1 分辨率（文件更小）。
本项目 `with_loop_closing`/`with_g2p5` 都关着，只影响存图。
其他 config（`default.yaml`/`go2_livox.yaml`）没这个 key → 默认 0 → 行为不变。

### ✅ 最终结论：既不是建图计算，也不是发布逻辑 —— 是 **DDS socket 缓冲**（2026-09-18）
排查到最后（用户无 UI、纯 bag 回放、28 核）：
- 自证式警告显示 `上次回调 0.8~1.5ms | worker 积压 1 帧` → **回调没被占住，worker 也没落后**
- 摘掉整个 SLAM、只留一个 `ros2 topic hz` 订阅者：**延迟照样存在，而且更大（max 0.798s）**
- ⇒ 与我们的进程无关

根因：`net.core.rmem/wmem_default` 默认 **208KB**，而一帧点云 **~2MB**；发送侧满了
→ `sendto()` 阻塞 → `ros2 bag play` 线程卡住 → 醒后一次性补发（`min: 0.001s` + `max: 0.798s`）。
修好后 `max 0.798s → 0.127s`，`std dev 0.0653 → 0.0089`，`min 0.001 → 0.025`。
详见用户记忆 `/memories/ros2-dds-transport-tuning.md`。

**这次排查里我猜错过 4 次**（worker 占用、rviz 重连、SaveMap、CPU 竞争），教训：
`arrival_dt` 这类"接收侧"指标出问题时，**先用一个纯订阅者把应用摘出去**，
再谈优化应用侧。我给 `MonitorRecvHealth` 加的自证字段（上次回调耗时 + worker 积压）
是这次能定性的关键 —— 以后所有"卡顿"类告警都该带这种上下文。

**顺带修正**：`kRecvBacklogWarnMs` 30 → **80ms**。rosbag2 播放器固有抖动就是 ±30%
（`--rate 0.5` 时标称 200ms 实测 142~253ms），30ms 阈值会被误触发。

**顺带保留**：`kf_cloud_res: 0.1`（省 626MB 内存 + 存图快 600ms）与此问题无关，但独立成立。

### 📄 完整排查记录已成文
`src/slam/lightning-lm/doc/lidar_latency_debug.md`
（症状 → 指标定义 → 三个真凶 → 方法论/踩坑清单 → 工具 → 改动清单 → 遗留项）
配套工具：`r41_ws/scripts/check_bag_timing.py <bag目录>`（不播包直接读 db3 查时间戳均匀性）

### 📌 `GetGlobalMap` 现在的唯一调用点是 `SaveMap`
实测证据（2026-09-18 一次没有 rviz 订阅者的运行）：
`PublishGlobalMap` / `GetMapDelta` / `MapVoxelDedup` / `MapToRos` **一个都没出现**，
而 `Map:kf-voxel` 恰好 1687 次（= 关键帧总数，一次完整遍历）、`Map:final-voxel` 1 次
→ 全部来自 `SaveMap` 那一次 `GetGlobalMap`。

那一次的开销（1687 kf）：
| 阶段 | 耗时 | 占比 |
|---|---|---|
| `Map:kf-voxel` 0.357 × 1687 | 602ms | 66% |
| `Map:trans+cat` 0.056 × 1687 | 94ms | 10% |
| `Map:final-voxel` | 210ms | 23% |
| 合计 | ~906ms | |

已加计时：`SaveMap:GetMap` / `SaveMap:GridMap`。
**排查思路**：日志里没有 `PublishGlobalMap` 就说明那次全量建图不是发布引起的，别再往发布上查。

### 下一步候选（按性价比）
1. ~~缓存逐帧滤波~~ ✅ 已做：497 → ~187ms
2. ~~发布加 `global_map_pub_res`~~ ✅ 已加（默认 0.1，可调大降低 rviz 负担）
3. ~~增量发布~~ ✅ 已做（见上）。**发布路径的 O(关键帧数) 问题已彻底消除**。
   剩下的 `GetGlobalMap` 只在 `srv/save_map` 里调用，是用户触发的一次性动作，187ms 可接受。
4. **自建 flat voxel hash 替换 PCL VoxelGrid**：PCL 内部是 `map<uint64, Voxel{vector<int>}>`+算质心，
   每次几百万次小分配。自建「开放寻址 + 保留代表点」可快 5~10×。
   现在只在存图路径上还有收益，优先级下降了。
5. **关键帧改存降采样点云**（省 320MB 内存）：会改变回环/g2p5/`save_map` 的输入分辨率，
   需单独评估。目前 ~387MB 原始关键帧云 + ~67MB 降采样缓存是内存大头。

## 测试方法
```bash
ros2 launch lightning r41_online_slam.launch.py
ros2 bag play ~/r41_ws/lio_data0909
# Ctrl-C 后看 Timer::PrintAll()
```

## 实测对比（2026-09-17）
| 指标 | 修复前 | 修复后 |
|---|---|---|
| `GetGlobalMap` | 210ms × 233 次 = 49s | **0 次**（无订阅者时） |
| `queue` | 1~17ms / 积压 170~280ms | **0.35~1.02ms** |
| `pending` | 0~3 | **恒为 0** |
| `arrival_dt` 突刺 | 218/271/712ms 反复 | **完全消失** |
| `Proc Lidar` | 1.090ms | 1.034ms（持平） |
| frames | 1263 | 3190 |

`queue ≈ 0.35~1.02ms` 正好对应 worker 的 `kIdlePollMs = 1` 轮询间隔 —— 这才是真实值。
