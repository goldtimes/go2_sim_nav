# lightning 地图路径与换图（2026-09）

## 两个不同概念，别混
| 概念 | 配置项 | 语义 |
|---|---|---|
| **建图保存根目录**（SLAM 侧） | `system.map_root` | 地图存到 `<map_root>/<map_id>/`；`map_id` 来自 `lightning/save_map` 服务请求 |
| **定位加载目录**（LOC 侧） | `system.map_path` | 指向**某一张具体地图**（含 `index.txt` + `<id>.pcd`） |

- 两者都不是 ROS 参数，用 `YAML_IO`/`YAML::LoadFile` **直读 yaml** → `--ros-args -p` 无效
- 覆盖方式：launch 参数（已加）
  - `ros2 launch lightning r41_online_slam.launch.py map_root:=<dir>`
  - `ros2 launch lightning r41_online_loc.launch.py map_path:=<dir>`
- 优先级：launch 参数 > yaml > （map_root 缺省）`$HOME/rcs/maps` > `./data`

## 用户名无关（重要）
运行机用户名不是 gmd（RK3588 是 jhc）→ **配置里绝不能写死 `/home/<user>/...`**
- 统一用 `~/` 前缀，由 `src/common/path_utils.h` 的 `path_utils::ResolveDir()` 展开 `$HOME`
- `map_root: ""` 已经天然走 `$HOME/rcs/maps`，无需展开
- **YAML 陷阱**：`map_path: ~/xxx` 不加引号时 `~` 被解析成 **null**，必须写 `map_path: "~/xxx"`
- 实测：`HOME=/tmp/fakeuser` 时 `map_root` → `/tmp/fakeuser/rcs/maps`，`~/other_map` → `/tmp/fakeuser/other_map`

## 地图加载的静默失败（已修）
- `MapChunk::LoadCloud()` 原先**丢弃 `pcl::io::loadPCDFile` 返回值**并无条件 `loaded_=true`
  → `.pcd` 缺失/损坏时"加载成功"但地图里一个点都没有，GICP 莫名失败
- `TiledMap::LoadMapIndex()` 原先 `static_chunks_` 为空也 `return true`
- `LidarLoc::Init` / `Localization::Init` 原先**都忽略返回值** → 失败传不上去
- 已全部修好：逐个 chunk 打 ERROR + 汇总 + 全失败则中止；`Localization::Finish()` 加判空
- **`index.txt` 里的路径字段被代码忽略**（会按 `map_path + "/" + id + ".pcd"` 重算）
  → 地图目录可随意搬动/改名，`index.txt` 不用改 ✓
- **FP 用户不需要** → 换图预检不检查 FP；`fps` 为空有 `if (!fps.empty())` 守卫，不会崩

## 排查用的输入
- 地图目录结构：`index.txt`（首行=原点，然后 `<id> <gx> <gy> <path>`，再 `# functional points` 段）
  + `<id>.pcd` + `<id>_dyn.pcd`(可选) + `global.pcd` + `keyframe_poses.txt` + `keyframes/`
- 复现"假成功"：建个只含 index.txt 的目录，用 `map_path:=` 指过去（修复前会报 `loaded chunks: 8`）

## 运行时换图（已实现，见 doc/map_switch.md）
- **`lightning/load_map`** 服务（`srv/LoadMap.srv`）：`string map_path` → `bool accepted / string message`
- **`lightning/map_state`** 话题（`Int32`, **latched**）：0=kIdle 1=kRunning 2=kSwitching 3=kError
  - 与 `lightning/loc_state`（定位算法状态 IDLE/INITIALIZING/GOOD/FAIL）**是两个东西**，别混
- `LocSystem::LocPhase` 状态机替代了原来的 `loc_started_`（`std::atomic_bool`）
  - `loc_started_` 和死变量 `map_loaded_` 已删除
- **gate & drain**：`ProcessingScope`（读者）/ `DrainInFlight()`（写者）
  - `ProcessLidar`/`ProcessIMU` **整个函数体**都在临界区（含 `PublishDebugAndRviz`，它会访问被重建的 `GetLIO()`/`GetLidarLoc()`）
  - 锁序：`mtx_processing_` → `Localization::global_mutex_`
- **换图必须异步**（独立线程）：`LocSystem::Spin()` 是 `rclcpp::spin` = SingleThreadedExecutor，
  同步换图会把 executor 堵住几秒 → 反压雷达驱动降频
- **预检必须在关闸之前**（`PrecheckMapDir`）：① index.txt 存在 ② 解析出 ≥1 chunk ③ 每个 `<id>.pcd` 存在且非空
  - 失败时旧地图完全不受影响；实测两种拒绝后 `map_state` 仍是 1
  - **不检查 FP**（用户不用 FP，`init_with_fp: false`）
- 换图后**不自动恢复**：新 `LidarLoc` 对象 → `loc_inited_=false`、`initial_pose_set_=false` → 停在等初值
  - 信号：`loc_state` 从 GOOD(2) 变 INITIALIZING(1)
  - `kSwitching` 期间 `SetInitPose` 被拒绝
- 感知侧：`perception_node` 订阅 `map_state`（latched）→ `GridMap::resetMap()`
  - `resetMap()` 必须同时复位 `has_ray_pose_`（旧 ray_pos_ 在新地图坐标系无意义）

## 实测数据
- 换图耗时 **53 ms**，0 错误 0 崩溃
- 感知清空测试：换图前 width=7136 → kSwitching → 0 → kRunning → 7136

## 已修的静默失败
- `MapChunk::LoadCloud()` 丢弃 `loadPCDFile` 返回值 + 无条件 `loaded_=true`
- `TiledMap::LoadMapIndex()` 空索引也 return true
- `LidarLoc::Init` / `Localization::Init` 都忽略失败 → 已逐层传递
- `Localization::Finish()` 未判空 `lidar_loc_`；`PublishDebugAndRviz` 未判空 `GetLIO()`/`GetLidarLoc()`/`GetMap()`

## 用户习惯（重要）
- **不跑 UI**：`with_ui: false`、`with_2dui: false` → 换图不用管 Pangolin 生命周期
- **不用 FP 功能点**：`init_with_fp: false`；初始化一律走 rviz2「2D Pose Estimate」(`/initialpose`) 或 `lightning/set_initpose`
  - `LocSystem::StartLoc()` **不做任何初始化**，只是把数据流闸门打开
- 换图只针对**定位阶段**，SLAM 侧不需要
- 运行机用户名不是 gmd → 配置里禁止写死 `/home/<user>/...`

## 历史坑
- `slam.cc` 原先**硬编码** `"./data/" + map_id + "/"`（相对进程 cwd）→ 已改为 `map_root_`
- `SaveMap` 会 `remove_all(save_path)` **覆盖式保存**，同名地图直接删掉重建，无备份
- `common/options.h` 里的 `extern std::string map_path` 是**死变量**，全仓库无人使用
- `run_slam_online.cc` 里 `StartSLAM("new_map")` 是硬编码默认名（在线模式下会被 save_map 的 map_id 覆盖）
- LOC 启动前会校验 `<map_path>/index.txt` 存在，不存在则**报错退出**（不再半初始化）

## 运行时换图（未实现，计划见对话）
- `Localization::Init` **已支持重入**（`if (lidar_loc_ != nullptr) Finish();`）
- 但 `LocSystem::Init` 会重复建 node/订阅/发布 → **换图不能走 LocSystem::Init**，只能调 `loc_->Init()`
- `LidarLoc::Finish()` 会落盘动态地图（`save_dyn_when_quit: true`）→ 换图需传 `save_dyn=false`

## 编译坑：atomic + lambda auto 返回类型
`cv_msg_.wait(lock, [this]() { return update_flag_; })` 在 `update_flag_` 是 `std::atomic_bool` 时
**编译失败**：lambda 的 `auto` 返回类型推导会尝试按值拷贝 atomic（拷贝构造已删除）。
必须写 `return update_flag_.load();`
