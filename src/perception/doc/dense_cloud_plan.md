# 让感知吃上未降采样点云（方案 B）—— 开发计划

> 状态：**已落地并验证**（2026-09-30）。P0/P1/P2 全部完成、代码保留；
> **P3 验收出了个重要负结果——见 §6**（对“幽灵格”无收益，但 CPU 零代价且对未来有利）。
> 目标：消除"**该被射线穿过、却没有射线**"这个根因（详见 `perception_gridmap.md` §3.6）。
> 工具：`src/pnc_2d/test/sim/obstacle_sweep_probe.py`（扫掠残留取证）。

---

## 0. 量化基线（实测 2026-09-30，Go2 仿真）

| 项 | 值 |
|---|---|
| 原始雷达 | `640 × 16 = **10240 点/帧 @ 9.9 Hz**`（`gazebo_VLP_D435i.xacro` 的 `<samples>`） |
| perception 现在收到 | **3255 点**（`filter_size_scan: 0.2` 降采样后） |
| 差距 | **×3.15** |
| 2 m 处射线间隔 | **5.7° → 1.8°**（而一个 0.1 m 体素占 2.9° ⇒ 几乎必有射线穿过） |
| 现状 CPU | 3D 融合 **2.3~3.4 ms**、总 2.5 ms @10 Hz |

---

## 1. 为什么是"多发一路"而不是"自己去订原始点云"

`scan_undistort_`（未降采样、**已去畸变**、body 系）在 lightning 内部**现成**。
在生成 `scan_down_world_` 的**同一处、同一套位姿**下再变换一遍 ⇒ `scan_undist_world_`，
天然满足：**map 系 + 与 pose 成对发布 + 已去畸变**。

自己订 raw 则要重做 **去畸变 + TF + 位姿时间同步**（raw stamp ≠ pose stamp），
而且会丢掉"位姿与点云成对"这个最重要的性质（100 ms 级错位风险）。

---

## 2. 阶段划分

| 阶段 | 内容 | 通过标准 |
|---|---|---|
| **P0** | perception 加 `grid_map.test_dup_cloud`（每帧实时读 ⇒ 可在线扫点数）→ 标定"点数 → 3D 融合耗时"曲线 | 10240 点时 **3D 融合 ≤ 8 ms** |
| **P1** | lightning **增量**发布 `lightning/perception/cloud_full`（开关 `enable_perception_pub_full`） | 10240 点 @10 Hz；**不动**匹配/滤波/去畸变逻辑 |
| **P2** | perception `grid_map.topic_cloud` 切到新话题 | 其余参数不动，仅换话题（一行可回退） |
| **P3** | 验收（见 §3） | 全绿 |
| **P4** | 文档 + 记忆 | — |

P1 的实现要点：
- `laser_mapping.h/.cc`：加 `scan_undist_world_` + `GetScanUndistWorld()`，
  在 `laser_mapping.cc:647-661`（`scan_down_world_` 的生成处）用**同一个位姿**再转一遍；
- `loc_system.cc`：加 publisher `lightning/perception/cloud_full` + 开关；
- config：`go2_gazebo_sim.yaml` / `r41_rk_slam.yaml` 加开关，**默认 false = 完全回到现状**。
- ✗ **不做**"在 loc_system 发布时临时变换"的省事版：那时取的位姿与 scan 不同时刻，
  会静默引入 100 ms 级错位。

---

## 3. 验收

| 项 | 手段 | 通过标准 |
|---|---|---|
| 点云契约 | `ros2 topic echo --field width/height` | ≈10240 @10 Hz |
| CPU | perception `帧耗时` 日志 | 3D 融合 ≤8 ms、总 ≤12 ms |
| **根因是否消除** | `obstacle_sweep_probe.py --speed 0.3 --rounds 6` | 残留 **20 格 → 个位数**（≤5），且**不依赖 decay** |
| 静态不误清 | `obstacle_clear_probe.py --steady 40` | 40 s 一直在 |
| 导航回归 | `test_avoidance.py --scenarios S1` | PASS、穿透 0、净距不退化 |
| decay 兜底 | 静止时日志 | `衰减` 恒为 0 |

---

## 4. 风险与回退

| 风险 | 概率 | 回退 |
|---|---|---|
| CPU 超预算 | 中 | `grid_map.topic_cloud` 改回旧话题（一行）；或收小 `local_update_range` / 分辨率 0.1→0.12 |
| 多发一路拖慢定位循环 | 低（10k 点变换 ≈0.2 ms） | `enable_perception_pub_full: false` |
| 更密点云让自身/近场噪声进图 | 中 | 调 `min_ray_length`、加强 footprint 剔除几何 |
| **实车点数更多**（未降采样 14k+） | 中 | 实车侧 `filter_size_scan: 0.1` 折中；`decay` 保留 |

---

## ★ P3 实测结论（重要负结果，2026-09-30）

### 各阶段实测

| 阶段 | 结果 |
|---|---|
| **P0** CPU 标定 | `test_dup_cloud` = 1/2/4 ⇒ 点数 3279/6514/**13032**，3D 融合始终 **2.2~3.6 ms** ⇒ **耗时对点数几乎不敏感**（同体素的射线被 `flag_rayend_`/`flag_traverse_` 去重） |
| **P1** lightning | 新增 `lightning/perception/cloud_full`（同帧/同姿态/已去畸变；`MapIncremental()` 里多做一次 `PointBodyToWorld`）；`point_filter_num: 2 → 1` ⇒ `scan_undistort_` 4640 → **9280 点** |
| **P2** perception | 切到新话题 ⇒ **3255 → 9280 点（×2.85）**，3D 融合仍是 **2.5~3.1 ms**、稳 10 Hz、静止时 `衰减 0` |
| **P3** 扫掠残留 | ❌ **20 → 20 格（毫无改善）**，两次是**同一批位置**（x=1.70 —— 箱子近侧面那条线） |

### 结论修正：主因是"无回波"，不是"射线太稀"

**密度 ×2.85 对幽灵格毫无收益** ⇒ 那 20 格不属于"射线太稀"那一类，而属于
"**物体走后那个方向根本没有回波**"：

- 它们在**箱子高度层**（z≈0.1~0.4），需要**高仰角**射线才能穿过；
- 高仰角射线**只在有回波时存在**——箱子在时由箱面提供，箱子一走，那个方向是空地/高处，
  **雷达没有回波 ⇒ 那条射线根本不存在**；
- 多出来的点全在"有回波的方向"上（地面/墙面），俯角低，2 m 处 z≈0~0.1，**穿不过那几层**。

⇒ **真正解决它的是 `decay`（时效遗忘），不是点云密度。**

### 为什么仍然保留

| 维度 | 效果 |
|---|---|
| 幽灵格 | ❌ 无改善（靠 `decay`） |
| CPU | ✅ **零代价**（9280 vs 3255 点：3D 融合都是 2.5~3.1 ms、10 Hz） |
| 小物体可见性（细杆/人腿，原本可能被 `filter_size_scan: 0.2` 滤掉） | ❓ **未验证**，是它真正可能的收益 |
| 未来动态障碍聚类 | ❓ 大概率有利（点密度 ×2.85 ⇒ 聚类不易碎），未验证 |

### 回退方法（保留但随时可退）

1. `launch/perception.launch.py` 的 `cloud_topic` 默认值改回 `/lightning/perception/cloud`
   （⚠ **launch 参数优先于 yaml**，只改 yaml 不生效——踩过）；
2. `config/perception.yaml` 的 `grid_map.topic_cloud` 同步改回；
3. `go2_gazebo_sim.yaml` 的 `point_filter_num` 改回 `2`（若想省去畸变算力）。

---

## 5. 预期管理（说清楚，免得验收时误判）

点数 ×3.15 治的是"**该被穿过却没有射线**"；治不了：
- "物体走后那个方向**确实没有回波**"（→ 仍需 `decay`）；
- 几何**遮挡**（物理限制，障碍后面本来就该是 unknown）。

所以目标是"**残留从 20 格降到个位数**"，不是"永久归零"。
