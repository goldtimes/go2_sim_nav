# lidar_loc 配准替换：pclomp NDT → small_gicp（GICP）

日期：2026-09-08

## 变更概述

定位模块（`src/core/localization/lidar_loc/`）的初始化与常规定位配准由魔改版
`pclomp::NormalDistributionsTransform`（NDT-OMP）整体替换为
[vendored small_gicp](https://github.com/koide3/small_gicp)（commit `2c6c783`，MIT）的
**GICP** 配准。NDT 代码路径已删除（`pclomp/` 目录移除），不再保留切换开关。

回环检测（`loop_closing.cc`）仍使用 PCL 自带 NDT，`icp_adjust` 仍使用 PCL ICP，本次未改动。

## 代码结构

| 内容 | 位置 |
|---|---|
| small_gicp 源码（vendored 静态库） | `thirdparty/small_gicp/`（include + registration*.cpp） |
| GICP 匹配器封装 | `src/core/localization/lidar_loc/small_gicp_matcher.{h,cc}` |
| 地图分块 → 目标点累积接口 | `tiled_map.h` 的 `SetNewTargetForGICP(T)` 模板 |
| 配准调用点 | `lidar_loc.cc` 的 `RunGicp` / `Localize` / `UpdateGlobalMap` |

`SmallGicpMatcher` 的行为：

- **目标构建**（`AddTargetCloud` + `BuildTarget`）：将所有已载入 chunk（静态+动态层）的点云
  累积后，经 `small_gicp::preprocess_points` 做体素降采样 + 法向/协方差估计，并构建 kdtree。
  在地图更新后台线程执行，`match_mutex_` 下整体热替换（与原 NDT 相同的模式）。
- **配准**（`Align`）：源点云同样降采样 + 协方差估计后，调用
  `small_gicp::align(target, source, kdtree, init_T, setting)`（GICP 类型，OMP 并行）。
- **细/粗两套 matcher**：细（默认 target 0.3m / max_corr 1.0m / 50 迭代）用于常规定位与最终
  接受；粗（target 2.0m / max_corr 4.0m / 10 迭代）用于初始化 xy+yaw 网格搜索快速打分。
  原 NDT 的粗分辨率 target 从未构建（死路径），本次已修复为实际生效。

## 打分量纲变化（重要）

NDT 的 `getTransformationProbability()`（平均点似然，典型 0~2.6，越大越好）替换为
**内点比例**：

```
confidence = result.num_inliers / 降采样后源点数   ∈ [0, 1]，越大越好
```

`nav_state.confidence`、`localization_result.confidence_` 均随之变为 [0,1] 量纲，
下游若有按 NDT 量纲（如 >1 为好）解释该字段的逻辑需要同步调整。

阈值迁移对照（`config/r41_rk_slam.yaml`）：

| 配置项 | NDT 旧值 | GICP 新值 | 说明 |
|---|---|---|---|
| `min_init_confidence` | 1.0 | 0.6 | 初始化接受阈值 |
| `init_search_stop_conf` | 0.6 | 0.8 | 粗搜提前退出 |
| `update_lidar_loc_score` | 2.5 | 0.9 | 动态图层更新门槛 |
| `TryOtherSolution` 绝对阈值（代码内） | 1.0 | 0.6 | RTK 解替换判据 |

`pgo` 中对 confidence 的阈值（0.6 满权重 / 0.5 下限 / 0.5 告警）在 [0,1] 量纲下语义成立，未改动。
以上新值为首跑推荐值，需按实测匹配分布微调。

## 新增配置项（lidar_loc 段）

```yaml
gicp_fine_target_res: 0.3     # 细匹配目标降采样分辨率 m
gicp_fine_max_corr_dist: 1.0  # 细匹配最大对应距离 m
gicp_fine_max_iterations: 50
gicp_rough_target_res: 2.0    # 粗匹配目标降采样分辨率 m
gicp_rough_max_corr_dist: 4.0 # 粗匹配最大对应距离 m
gicp_rough_max_iterations: 10
gicp_source_res: 0.3          # 源点云降采样分辨率 m
gicp_num_threads: 4
```

注：源点云（LIO 去畸变帧）现在会先降采样到 `gicp_source_res` 再参与配准；
原 NDT 路径为全分辨率输入。这是主要的提速来源之一。

## 已知的其他行为差异

- `Localize()` 不再每帧写 `./data/tgt.pcd` 调试文件。
- GICP 结果不检查 `converged` 标志即输出变换（与原 NDT `align()` 行为一致），
  置信度由内点比例反映质量。

## 升级 small_gicp

`thirdparty/small_gicp` 为源码拷贝（include + `registration.cpp` +
`registration_helper.cpp`，取自 `src/slam/small_gicp` 完整仓库，commit `2c6c783`）。
升级时同步覆盖这三个部分，并在 `src/CMakeLists.txt` 保持静态库目标不变。
