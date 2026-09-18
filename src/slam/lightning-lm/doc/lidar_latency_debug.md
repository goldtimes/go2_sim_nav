# 雷达延迟与建图性能排查记录

**日期**：2026-09-18
**环境**：x86 开发机（28 核 / 32 GB）+ ROS 2 Humble + Fast DDS；bag 回放 + 在线 SLAM
**触发问题**：日志反复出现

```
[warning][slam] [lidar] recv falling behind: arrival_dt 712.3 ms vs stamp_dt 100.4 ms (backlog +611.9 ms)
```

---

## 0. 结论速查

| # | 真凶 | 证据 | 修复 |
|---|---|---|---|
| 1 | **周期性重算整张全局地图**（每 3 个关键帧发一次） | `GetGlobalMap` 1663 kf 时单次 **485 ms**，占 worker 40% | 改成按时间节流 + 只在有订阅者时算；最终改成**增量发布** |
| 2 | **DDS socket 缓冲太小**（208 KB vs 2.25 MB 的消息） | `UdpRcvbufErrors` 累计 46200；`ros2 topic hz` max **0.798 s** | `net.core.{r,w}mem_{max,default}` → 32 MB，max 降到 **0.127 s** |
| 3 | **bag 录制时数据本身就不均匀** | 直接读 db3：某包 3191 帧里有 **65 处间隔 > 150 ms**，最长 **797.7 ms** | 无法在运行时修 —— 要查录制侧 |

**最容易被误导的一点**：`ros2 topic hz` 报的 `min/max` **分不清**「运行时卡顿」和「录制时数据就有洞」。
判断运行时必须用 **`arrival_dt − stamp_dt`**。

---

## 1. 先搞清楚指标在量什么

这是整篇文档的基础，不搞清楚会一路查错方向。

`MonitorRecvHealth()` 在 **`ProcessLidar()` 进入的那一刻**打时间戳：

```cpp
// slam.cc
void SlamSystem::ProcessLidar(...) {
    if (options_.log_lidar_time_) {
        last_scan_points_.store(pts);
        MonitorRecvHealth(ToSec(cloud->header.stamp));   // ← 在这里打点
    }
    ...
}
```

所以：

| 指标 | 定义 | 反映什么 |
|---|---|---|
| `stamp_dt` | 本帧与上帧的 **header.stamp 差值** | **数据源**的出帧节奏（驱动 / 录制） |
| `arrival_dt` | 本帧与上帧**回调进入时刻**的差值 | 数据**送到我们手里**的节奏 |
| `backlog` | `arrival_dt − stamp_dt` | **纯粹的运行时延迟**（数据自身的抖动被抵消） |

**`backlog` 才是真正要看的量。** `stamp_dt` 变大说明驱动/录制侧丢帧，此时 `backlog` 仍约为 0。

对 `backlog` 的解读只有两种可能（单线程 executor）：

- **① 上一个回调自己跑了很久** → executor 被我们自己的回调占住
- **② 消息根本没送到** → 阻塞在回调之外：DDS 投递 / 调度 / 别的进程抢 CPU / **publisher 晚发**

---

## 2. 真凶 1：周期性重算整张全局地图

### 症状与定位

`Timer::PrintAll()` 一出来就清楚了：

```
[GetGlobalMap]     avg 210.382 ms, med 210.791, 95%: 388.151  called 233   ← 元凶
[PublishMappingTf] avg   0.311 ms
[Proc Lidar]       avg   1.090 ms, med 0.864, 95%: 1.771                   ← 回调很快
[LIO Run]          avg   5.083 ms
```

**判据**：`arrival_dt` 突刺几百 ms，但 `Proc Lidar` 只有 1 ms
→ 阻塞**不在回调内部**，而在回调之前。

### 根因

原代码在 `slam.cc::ProcessOnce` 里「每 3 个关键帧发一次 global_map」：

- 关键帧约 0.18 s 一个 → 每 0.54 s 发一次
- `GetGlobalMap()` 要拼接**全部关键帧**并逐帧体素滤波
- 233 次 × 210 ms = 49 s / 126 s = **worker 40% 的时间**

### `GetGlobalMap` 成本模型（1687 kf 实测）

| 阶段 | 单价 | 占比 |
|---|---|---|
| `Map:kf-voxel` 逐帧体素降采样 | **0.377 ms/kf** | **62%** |
| `Map:trans+cat` 变换 + 拼接 | 0.082 ms/kf | 14% |
| `Map:final-voxel` 最后一次全量滤波 | 120 ms（近似常数） | 24% |

```
T(GetGlobalMap) ≈ K × 0.459 ms + 120 ms
```

（`K` = 本次快照的关键帧数。1687 kf → 约 0.9 s。实测吻合到 ±1%。）

### ⚠️ 「只对整体地图滤波一次」是反优化

关键帧存的是 `scan_undistort_`（**未降采样**，约 14400 点），**不是** `scan_down_body_`。

| 方案 | 拼接量 | 最后滤波的输入 |
|---|---|---|
| 逐帧滤 + 全量滤 | `K × 几千点` ≈ 1.6 M | 1.6 M |
| **只全量滤一次** | `K × 14400` = **11.9 M 点** | 11.9 M → `final-voxel` 从 120 ms 涨到 ~700 ms，**整体慢 2.2 倍** |

### 修复分两步

**① 避免无谓计算**（`system` 段）

```yaml
global_map_pub_mode: "incremental"   # incremental | full | off
global_map_pub_interval: 1.0         # 秒，0 = 不发布
global_map_pub_res: 0.1              # 只影响“看”，不影响存图
global_map_pub_max_kf: 0             # 单次最多发多少关键帧，0 = 不限
```

- **没人订阅就不算不发**（`get_subscription_count() > 0`）
- **按时间节流**，不按关键帧数（后者会随地图变大而变密）
- **增量模式**：只发「新出现的关键帧」，由 rviz 自己累积成整图
  - 单次成本 `O(新增关键帧)`，**与地图大小无关**
  - 跨调用做**体素去重**（`utils/pointcloud_utils.h::VoxelKey`），
    否则同一面墙会被 N 个关键帧各发一遍，rviz 点数随运行时间无界增长

> rviz 侧必须把该显示项的 **`Decay Time` 设为 0**（永不衰减），否则只显示最近 N 秒。

**② 缓存逐帧降采样结果**（`Keyframe::GetCloudDownsampled`）

逐帧降采样在 **LIDAR 系**做，且 `cloud_` 构造后永不变更，所以**同一 res 的结果恒定**，
可以安全缓存 —— 实测这一项占 `GetGlobalMap` 的 62%，纯属重复劳动。

**③ 关键帧按分辨率存储**（`fasterlio.kf_cloud_res`）

原来关键帧存的是**未降采样**的原始扫描：

| 项 | 值 |
|---|---|
| `scan_undistort_` | ~14400 点/帧（`point_filter_num: 6` 抽点后） |
| `PointXYZIT` | 32 B（xyz+intensity 各 4 B，double time 8 B，对齐到 32） |
| 单帧占用 | **450 KB** |
| 1687 帧 | **~760 MB**，且以 **~150 MB/分钟** 增长 |

改成建帧时就按 `kf_cloud_res: 0.1` 降采样再存：

| 指标 | 改前 | 改后 |
|---|---|---|
| 关键帧点云内存（1687 帧） | ~760 MB | **~134 MB** |
| `SaveMap` 全量建图 | ~906 ms | **~305 ms** |
| `Map:kf-voxel` | 602 ms | **~0** |
| 建图输出分辨率 | 0.1 | **0.1（不变）** |

**输出地图逐点一致** —— 逐帧 0.1 体素化这个操作没变，只是从「每次 `GetGlobalMap` 时做」
提前到「建帧时做一次」。

> ⚠️ 实现要点：必须用**局部** `VoxelGrid`。成员 `voxel_scan_` 的 leaf size 是
> `filter_size_scan`（归 `Run()` 里的 scan 降采样用），动它会把 LIO 前端带偏。

---

## 3. 真凶 2：DDS socket 缓冲

### 关键：先把应用摘出去

被 ①②③ 修完之后仍有零星突刺。此时给告警加上**自证字段**：

```
[lidar] recv falling behind: arrival_dt 392.3 ms vs stamp_dt 100.4 ms (backlog +291.9 ms)
        | 上次回调 0.8 ms | worker 积压 1 帧
```

- `上次回调 0.8 ms` —— 和 `Proc Lidar` 均值 1.4 ms 一致，**回调绝没有跑 390 ms**
- `worker 积压 1 帧` —— 常数，无增长趋势（丢帧阈值是 10 帧）

**两项都正常 ⇒ 阻塞 100% 在回调之外。**

### 决定性实验：只留一个纯订阅者

```bash
# 终端 1 —— 只播 bag，不跑 SLAM
ros2 bag play ~/r41_ws/lio_data0909
# 终端 2
ros2 topic hz /rslidar_points
```

结果：

```
min: 0.001s   max: 0.798s   std dev: 0.0653s   average rate: 10.001
```

**不跑我们的进程，延迟照样存在，而且更大。**
`min: 0.001s` = 几条消息挤在一起瞬间到 —— 典型的「发布端睡过头 → 醒后一次补发」。

### 根因

```bash
$ sysctl net.core.rmem_max net.core.rmem_default net.core.wmem_default
net.core.rmem_max     = 212992      # 208 KB
net.core.rmem_default = 212992
net.core.wmem_default = 212992
```

| 项 | 值 |
|---|---|
| Fast DDS 在 loopback 上的数据报 | 65500 B |
| → 208 KB 接收缓冲只能装 | **3 个数据报** |
| 一帧 `/rslidar_points` | 86400 点 × ~26 B ≈ **2.25 MB** |
| → 一帧需要 | **~34 个数据报** |

**一帧消息就装不进接收缓冲。** 发送侧同理：`sendto()` 写满 208 KB 就阻塞 →
`ros2 bag play` 的播放线程被卡住 → 醒后一次性补发。

旁证：`/proc/net/snmp` 的 UDP 行里 `InErrors == RcvbufErrors == 46200`
（**100% 的 UDP 接收错误都是缓冲溢出**）。

### 修复（四项都要设，**最容易漏 `wmem_default`**）

```bash
sudo tee /etc/sysctl.d/99-ros2-dds.conf >/dev/null <<'EOF'
net.core.rmem_max     = 33554432
net.core.rmem_default = 33554432
net.core.wmem_max     = 33554432
net.core.wmem_default = 33554432
EOF
sudo sysctl --system
```

- `rmem_max` / `wmem_max` 管 `setsockopt` 的上限；`*_default` 管**不调 setsockopt** 的 socket
- **socket 缓冲在建 socket 时确定 → 改完必须重启发布/订阅进程**
- **RK3588 那台也要加**（内存更紧、消息一样大）

### 验证

| | `ros2 topic hz` max | bag 内数据 max | 结论 |
|---|---|---|---|
| 修复前（208 KB） | **0.798 s** | 126.6 ms | 多出的 **671 ms 是真实传输卡顿** |
| 修复后（32 MB） | **0.127 s** | 126.6 ms | 差值 ≈ 0，**传输层干净** |

`std dev` 同时从 0.0653 降到 0.0089；`min` 从 0.001 s 变成 0.025 s（突发消失）。

`0.127 s` 正好等于数据自身的 `126.6 ms` —— 教科书级的对照验证。

---

## 4. 真凶 3：bag 录制时数据本身就不均匀

### `ros2 topic hz` 判断不了运行时

它量的是**消息到达间隔**，下面两种情况在它的 `min`/`max` 上**长得完全一样**：

- 运行时卡了（传输 / 调度 / 磁盘）
- 录制时数据就录成那样

### 直接读 db3，绕开播放

```bash
python3 ~/r41_ws/scripts/check_bag_timing.py ~/lio_data_stair
```

```
=== lio_data_stair
  /rslidar_points
    n=3191  span=319.1s  中位间隔=100.1ms  实际频率=10.0Hz
    间隔 ms:  min 0.9  /  med 100.1  /  max 797.7
    超过 150ms 的处数: 65
    -> !! 数据本身就不均匀，别拿它评估运行时性能
       t+  10.0s   gap   388.6 ms
       t+  15.5s   gap   701.1 ms      ← 与 ros2 topic hz 报的 0.701s 对上
       t+  37.5s   gap   797.7 ms      ← 与 ros2 topic hz 报的 0.797s 对上

=== lio_data0909
    间隔 ms:  min 71.0  /  med 100.5  /  max 126.6
    超过 150ms 的处数: 0          ← 干净的包
```

**`ros2 topic hz` 报的 `0.797 s` / `0.701 s`，在包内数据里逐一对上了。**
它报的 `min: 0.001s` 也正好对应包内的 `min 0.9 ms`。

### 这个包的问题

- 3191 帧 / 319.1 s = **恰好 10 Hz** → **没有丢帧**
- 但间隔从 0.9 ms 到 797.7 ms → **驱动出帧时刻抖动**

`min 0.9 ms` **物理上不可能**（两帧完整的 86400 点扫描不可能隔 0.9 ms 到），
说明**驱动时间戳有问题**（个别帧取了错误的时钟源）。

**后果**：797 ms 的空洞里车还在动，LIO 只能靠 IMU 惯性递推 → **必然产生漂移**。
拿这种包调参或评估建图效果会得出错误结论。

**这个检查我们的代码早就有了**：`[lidar] ... dt ...` 列，以及退出时的汇总：

```
[lidar] frames 3191 | driver gap > 150 ms: 65 (2.04%)
```

---

## 5. 排查方法论

### ✅ 正确的顺序

1. **先定义指标**：搞清楚 `arrival_dt` / `stamp_dt` / `backlog` 分别在量什么
2. **让告警自证**：给警告附上判别所需的最小上下文
   （本次是「上次回调耗时」+「worker 积压帧数」）
3. **把应用摘出去**：用一个纯订阅者复现，判断问题在不在我们的进程里
4. **验证数据源**：直接读 bag，判断问题在不在数据本身
5. **最后才优化应用侧**

### ❌ 本次踩过的坑（5 次错误假设）

| 假设 | 为什么错 |
|---|---|
| worker 被建图计算占住 | `ProcessPointCloud2` 只拿 `mtx_buffer_`，**从不碰 `mtx_state_`**；而 `Run()` 全程持 `mtx_state_` → worker 再忙也影响不到回调进入时刻 |
| rviz 重连补发整图 | 那轮根本没有订阅者（`PublishGlobalMap` 计时器一次都没出现） |
| `SaveMap` 阻塞 executor | 用户确认警告不在存图阶段 |
| CPU 竞争 | 28 核，worker 只占 7% 单核 |
| 磁盘 I/O 抖动 | 把包拷到 `/dev/shm` 内存盘播放，**结果一样** |

### 📌 可复用的判据

| 现象 | 结论 |
|---|---|
| `backlog` 大 + **上次回调大** | executor 被我们自己的回调占住 |
| `backlog` 大 + **worker 有积压** | worker 落后（某段计算太重） |
| `backlog` 大 + **两者都正常** | 阻塞在回调之外：DDS / 调度 / 别的进程 / publisher 晚发 |
| `stamp_dt` 也大 | **数据源**出帧异常（驱动 / 录制），不是我们 |
| `ros2 topic hz` 的 max > 包内数据 max | 运行时**确实**有延迟 |
| `ros2 topic hz` 的 max == 包内数据 max | 纯粹是数据本身，运行时干净 |

---

## 6. 工具

| 工具 | 用途 |
|---|---|
| `scripts/check_bag_timing.py <bag目录>` | 不播包，直接读 db3 检查各话题时间戳是否均匀 |
| `nstat -az UdpRcvbufErrors` | UDP 接收缓冲溢出计数（播放前后取差值） |
| `/proc/net/snmp` 的 `Udp:` 行 | 同上，`InErrors == RcvbufErrors` 即缓冲溢出 |
| `ros2 topic hz <话题>` | 到达间隔统计（**注意它分不清运行时和数据本身**）|
| `Timer::PrintAll()` | 各阶段耗时（退出时打印）|
| `[lidar] pts/dt/lio/e2e/queue/pending` 逐帧日志 | 逐帧健康度 |

---

## 7. 本次改动清单

| 改动 | 与延迟排查的关系 | 保留 |
|---|---|---|
| 全局地图：节流 + 订阅者检查 + **增量发布** + 体素去重 | **真凶 1** | ✅ |
| `Keyframe::GetCloudDownsampled` 缓存 | 关联真凶 1（消除 62% 重复计算） | ✅ |
| `fasterlio.kf_cloud_res: 0.1` | 无关，但省 **626 MB** 内存 + 存图快 600 ms | ✅ |
| `net.core.*mem_*` → 32 MB | **真凶 2**（必须持久化） | ✅ |
| `MonitorRecvHealth` 加自证字段 | 定性的关键 | ✅ |
| `kRecvBacklogWarnMs` 30 → 80 ms | 降噪，非必需 | ✅ |
| `SaveMap` 分段计时 + 起止日志 | 无关，但存图耗时现在可定位 | ✅ |
| `Timer` 加锁（`records_mutex_`） | 新增子计时后并发调用的前提 | ✅ |
| rviz `Decay Time` → 0 | 增量发布的配套要求 | ✅ |

---

## 8. 遗留 / 待办

- [ ] **`lio_data_stair` 包要重录或查明原因**（驱动时间戳问题，`min 0.9 ms`）
- [ ] **RK3588 目标机也要加那四项 sysctl**
- [ ] 录制侧排查：为何 `stamp_dt` 抖动到 797 ms（驱动 CPU / 磁盘写入 / 网口）
- [ ] `Localization::ProcessIMUMsg` 里阻塞版 `GetIMUState()`（见 `mapping_threading_refactor.md` §6.2）
- [ ] `run_slam_online.cc` 里硬编码的 `StartSLAM("new_map")`
- [ ] `SaveMap` 跑在 executor 线程上（地图大时阻塞 lidar 回调数秒）—— 可考虑挪到独立线程

---

## 参考

- `doc/mapping_threading_refactor.md` —— 线程模型与并发边界
- `doc/map_switch.md` —— 地图路径与运行时换图
- `/memories/ros2-dds-transport-tuning.md`（开发机长期记忆）—— DDS 传输调优速查
