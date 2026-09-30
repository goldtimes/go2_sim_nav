# map_server（全局地图服务器，2026-09-20 新建）

## 定位与分工
- 自研 C++ ament 包 `src/map_server`（**不用**官方 nav2_map_server：它是 lifecycle 节点、只有 2D、
  不带"目录/3D 管理"）。普通节点，节点名 `map_server`。
- 分工：本节点发**静态全局图**；`perception` 发**动态局部图**（`grid_map/occupancy_2d`）。
  两者独立发布，规划器自行合成。

## 地图目录约定（与 lightning 的 map_path 同一目录）
`~/rcs/maps/<站点>/` = `map.yaml`+`map.pgm`(2D) + `global.pcd`/`0.pcd`(3D) + `index.txt`/`keyframes/`。
7 个站点格式一致。**换图三件套必须同源**：lightning map_path / map_server / perception。

## 接口
- 发布 `global_map/occupancy`（OccupancyGrid, **transient_local** depth1）+ `global_map/metadata`
- 服务 `global_map/load_map`（`nav2_msgs/LoadMap`，map_url 收目录或 yaml，也吃 `file://`；
  不支持 `package://`，会明确报错）
- 参数：`map_dir` / `map_yaml` / `frame_id` / `topic_occupancy` / `topic_metadata` /
  `srv_load_map` / `publish_metadata` / `republish_interval`(**默认 0.0**) /
  `republish_on_new_subscriber`(**默认 true**) / `unknown_as_free`(默认 false)

## ⚠ 关键坑
- **latched 不够**：RViz 的 Map 显示项默认 `Durability=Volatile`，Volatile 订阅端**收不到**
  latched 的历史样本 → 先起节点后开 RViz 会空白。
- 【2026-09-21 改】重发策略改为**"载图发一次 + 新订阅者出现时按需补发一次"**：
  默认 `republish_interval=0.0`，新增 `republish_on_new_subscriber=true`（0.5s 看门狗：
  只在订阅者数量变多、且其中**有 Volatile 订阅者**时补发；全是 transient_local 的不补发）。
  Humble 的 `PublisherEventCallbacks` **没有 matched 回调**（Iron 之后才有）→ 只能轻量轮询。
  实测（186349 格 ≈ 182 KB）：新策略 volatile 订阅者 6s 内收 1 条；旧 1 Hz 是 7 条（≈1.27 MB/6s）。
  局限：订阅者"一进一出同时发生"导致计数不变时可能漏补发 → 可开 `republish_interval` 兜底。
- ★【2026-09-30 修 bug】看门狗原先**只登记 `topic_occ_`**，于是"只订点云的 Volatile
  订阅者"（RViz PointCloud2 默认就是 Volatile）**永远等不到补发** → 现象是 2D 图能看到、
  点云空白。现改为 `latch_watchers_`（`std::vector<LatchWatch{pub,topic,last,resend}>`）
  登记**全部 latched 话题**：occupancy/metadata、cloud、routes、zones，各自独立计数。
  启动日志变成"按需补发已开：4 个 latched 话题的新订阅者各补发一次"。
  `republish()`（周期兜底路径）也补上了 routes/zones，并把 `publishZones(log=false)`
  用于周期路径（否则每秒刷一行 INFO）。
- ⚠ **`ros2 topic echo` 的 QoS 陷阱**（踩过，误判成"发布失败"）：
  不给任何 `--qos-*` → 用发现到的 QoS（transient_local）→ 能拿到历史样本；
  **只给一个** `--qos-reliability reliable`（或只给 durability）→ ros2cli 造出的 profile
  与发布端不匹配 → **静默收不到、等到超时**。要显式指定就**两个一起给**。
  （诊断顺序：先 `ros2 topic info -v` 看 durability，再看节点日志有没有"3D 地图已加载…N 点"。）
- **`negate: 0` 是整数**：yaml-cpp 的 `as<bool>()` 对整数直接抛 "bad conversion"。必须自己写
  宽松 bool 解析（0/1/true/false/yes/no）。**任何解析这些 map.yaml 的工具都会踩**。
- CMake：`ament_target_dependencies`(plain 签名) 与 `target_link_libraries(target PUBLIC x)`
  (keyword) **不能混用在同一 target**；系统 yaml-cpp 0.7 导出的 target 是 **`yaml-cpp`**，
  不是 `yaml-cpp::yaml-cpp`（用 `${YAML_CPP_LIBRARIES}` 最稳）。
- `ros2 launch` 输出被 `timeout ... | tail` 吃掉（缓冲未 flush）→ 验收时重定向到文件。

## M3（3D 地图）：★ 2026-09-30 **已完成**
- 设计：3D（`global.pcd`）与 2D **同一目录**、跟着**同一个 `load_map` 服务**一起加载
  （切站点不可能只切一半）→ 不需要额外服务类型。
- 参数：`publish_3d`(本项目 true) / `pcd_file`(""=<map_dir>/global.pcd) /
  `topic_cloud_3d`("global_map/cloud") / `cloud_frame_id`(=frame_id) /
  `cloud_voxel_leaf`(0.0=不降采样) / `require_3d`(本项目 true)。
- 实现：
  · `map_io.cpp::loadPcd()` 用 `pcl::io::loadPCDFile<pcl::PointXYZ>`（自带
    ascii/binary/**binary_compressed(LZF)**）+ 头部点数与实读点数**交叉校验**；
  · `map_server_node.cpp::loadCloud3D()` 用 `PointCloud2Modifier.setPointCloud2FieldsByString(1,"xyz")`
    + `PointCloud2Iterator` 填点，置 `has_cloud_3d_=true`；`cloud_voxel_leaf>0` 时先体素降采样
    （21bit×3 打包成 uint64 键，每体素取质心）。
- ⚠★ **PCL 的 `#include` 必须放在 `map_io.cpp` 文件顶部（`namespace map_server` 之外）**：
  放进命名空间里会把 boost/std 系统头一起包进去 → 编译炸成
  `'::mpl_' has not been declared` / `did you mean 'map_server::std'?`（已踩过一次）。
- CMake：`find_package(PCL REQUIRED COMPONENTS common io)`（系统已装）；
  `map_io` 要 `${PCL_INCLUDE_DIRS}` + `${PCL_LIBRARIES}`。
- 实测：`171590 点 → 171590 点，voxel_leaf 0.000`，载入 12.9~13.9 ms；
  `ros2 topic echo --once --field width /global_map/cloud` → **171590**。

## 地图语义：未知 → 空闲（**已定策略**，2026-09-20）
三个站点的 map.pgm **只有黑(0)+灰(205)，没有白(254)**：
go2_sim_factory 607x307 黑 6.5%/灰 93.5%；r41iws_0917 220x525 黑 10.3%/灰 89.7%；
r41iws_0918 699x545 黑 7.2%/灰 92.8%。即"除障碍外全是未知"。

**用户决定：全局图里灰色当白色（可通行）** → 配置 `unknown_as_free: true`（发布时 -1→0），
理由：全局图是稀疏先验，未观测区不该挡住全局规划（≈ nav2 costmap 的 track_unknown_space=false）。
- 实测：true → 占据 12041/空闲 174308/未知 0；false → 占据 12041/空闲 0/未知 174308（对照）。
- 日志按**发布语义**打印并标【未知已当空闲】（否则日志与图对不上）。
- **与官方 nav2_map_server 逐格比对时必须 `-p unknown_as_free:=false`**（已复跑验证：186349 格一致）。
- 局部图（perception）**保留未知**（-1）+ ESDF 乐观解释；合成建议：任一为 100 即障碍，否则可通行。
- 根治办法（可选）：地图生成端把可通行区标白(254)。

## 验收方法（已跑通）
1. **与官方 nav2_map_server 逐格比对**（`scripts/compare_maps.py`）：官方是 lifecycle 节点，
   需 `ros2 lifecycle set /nav2_ref configure/activate`；比对脚本默认用 transient_local
   订阅（Volatile 收不到官方的 latched 图）。
   实测 go2_sim_factory：**186349 格全部一致**（占据 12041 / 未知 174308）。
2. M2 换图：`ros2 service call /global_map/load_map nav2_msgs/srv/LoadMap "{map_url: <站点目录>}"`
   → 实测切到 r41iws_0917 得到 220x525、占据 11915（与 PGM 黑像素数一致）；不存在的站点→result=1。
3. 坐标系对齐：RViz 里与 `lightning/global_map` 叠合看是否重合。

## 后续
- 地图保存；与 `lightning/map_state` 联动自动换图。
- 【2026-09-30 已做】本节点**只当资产加载器**：不做路网可通行校验（交给
  `route_network_planner` 的 `e.feasible` + `reject_infeasible`）、删除全部
  `footprint.*` 参数与 `common.hard_threshold`、`common.unknown_as_occupied`、
  `inflate` 默认 0.0。车体几何**单一来源** = `pnc_2d/config/pnc_2d.yaml` 的 `footprint.*`
  （perception 的 `grid_map.footprint_*` 必须对齐）。
