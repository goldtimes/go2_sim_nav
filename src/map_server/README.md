# map_server（全局地图服务器）

自研的**全局地图**加载/发布节点。本期实现 M1+M2：

- **M1** 加载 nav2 标准 2D 栅格（`map.yaml` + `map.pgm`），以 **latched** QoS 发布
- **M2** 运行时通过 `LoadMap` 服务切换站点（传目录或 yaml 路径）

3D 点云（同目录的 `global.pcd`）只留接口，未实现。

## 与 perception 的分工

| | 谁发 | 话题 | 性质 |
|---|---|---|---|
| 全局静态图 | 本节点 | `global_map/occupancy` (`OccupancyGrid`) | 先验，不变 |
| 局部动态图 | `perception` (plan_env) | `grid_map/occupancy_2d` | 实时感知 |

两者**各自独立发布**，由规划器自行合成（全局规划用全局图、局部避障用局部图）。

## 地图目录约定

与 lightning 的 `map_path` 用**同一个目录**：

```
~/rcs/maps/<站点>/
  map.yaml + map.pgm      # 2D（本期用）
  global.pcd / 0.pcd      # 3D（后续）
  index.txt, keyframes/   # lightning 定位用
```

## 接口

| 类型 | 名字（可参数改） | 说明 |
|---|---|---|
| 发布 | `global_map/occupancy` | `nav_msgs/OccupancyGrid`，QoS **transient_local** depth 1 |
| 发布 | `global_map/metadata` | `nav_msgs/MapMetaData`（`publish_metadata` 控制） |
| 发布 | `global_map/cloud` | `sensor_msgs/PointCloud2`（**M3 接口，仅 `publish_3d: true` 时创建**；内容待实现） |
| 服务 | `global_map/load_map` | `nav2_msgs/srv/LoadMap`，`map_url` 收**站点目录**或 **yaml 路径**（也接受 `file://` 前缀） |

### 参数（`config/map_server.yaml`）

| 参数 | 默认 | 说明 |
|---|---|---|
| `map_dir` | `~/rcs/maps/go2_sim_factory` | 地图目录（内含 `map.yaml`） |
| `map_yaml` | `""` | 直接指定 yaml，非空时优先于 `map_dir` |
| `frame_id` | `map` | 与 perception/lightning 一致 |
| `topic_occupancy` / `topic_metadata` | `global_map/occupancy` / `global_map/metadata` | 自定义话题名 |
| `srv_load_map` | `global_map/load_map` | 换图服务名 |
| `publish_metadata` | `true` | 是否发 metadata |
| `republish_interval` | `1.0` s | **周期性重发**（见下），0 = 关闭 |
| `unknown_as_free` | **`true`（本项目策略）** | 把"未知"当"空闲"发布（见"地图语义"） |

### 3D 地图参数（M3 接口，本期只接线）

| 参数 | 默认 | 说明 |
|---|---|---|
| `publish_3d` | `false` | 打开则创建 `topic_cloud_3d`（latched，与 2D 同为 transient_local） |
| `pcd_file` | `""` | 空 = `<map_dir>/global.pcd`；也可直接给 `.pcd` 路径 |
| `topic_cloud_3d` | `global_map/cloud` | `sensor_msgs/PointCloud2` |
| `cloud_frame_id` | = `frame_id` | 点云常来自别的源，可单独指定 |
| `cloud_voxel_leaf` | `0.0` | 预留：发布前体素降采样（m） |
| `require_3d` | `false` | true = 3D 加载失败时整个 `load_map` 报错；否则只 WARN（不连累 2D） |

#### 为什么还要周期性重发（不只是 latched）

标准做法是 latched（`transient_local`）：晚启动的订阅者能立刻拿到历史样本。但 **RViz 的
Map 显示项默认 `Durability=Volatile`**，而 Volatile 订阅端**不会**收到历史样本 → 先起节点
后开 RViz 会看到空白。所以本节点在 latched 之外再按 `republish_interval` 重发一次
（一张图每秒重发，开销可忽略），这样任何 QoS 的订阅者都能拿到。

实测（`republish_interval=1.0` vs `0`）：

```
[volatile] 607x307 ... 占据=12041      # 重发开启 → 收得到
[volatile] 没收到                       # 重发关闭 → 收不到（latched 的固有限制）
```

## 3D 地图接口（M3：**已接线，未实现内容**）

### 设计决定

3D 地图（`global.pcd`）与 2D 地图**同一个目录**，跟着**同一个 `load_map` 服务**一起加载
（切站点时不可能只切一半），因此**不需要额外的服务类型**。

### 已经就绪的部分

- `publish_3d: true` 时创建 `global_map/cloud`（latched）→ 话题在图上可见、可被订阅
- 路径解析：`pcd_file` 优先，否则 `<地图目录>/global.pcd`
- PCD **头部校验 + 自省**（VERSION/DATA 行、x/y/z 字段、点数），报错分三种：
  文件不存在 / 不是合法 PCD / 缺 xyz 字段
- 载入失败默认只 WARN（不连累 2D），`require_3d: true` 时才让整次 `load_map` 失败
- 发布路径（latched + 周期重发）与服务接入已写好

### 唯一待填的地方（M3 内容）

`src/map_io.cpp::loadPcd()` 里标了 `---- M3 待实现 ----`：把点**解码**进 `out.xyz`，
然后在 `map_server_node.cpp::loadCloud3D()` 里把 `xyz` 填进 `PointCloud2` 并置
`has_cloud_3d_ = true`（发布器/latched/重发/服务接入都已就绪，填完即生效）。

**实测这些地图的 PCD 格式**（实现时要知道）：

```
VERSION 0.7
FIELDS x y z intensity time      # 多了 intensity(4B float) 与 time(8B double)
SIZE   4 4 4 4 8
POINTS 171590
DATA  binary_compressed          # ← LZF 压缩，需解压（PCL 的 io::loadPCDFile 支持）
```

即：光有 ascii/binary 不够，**这个文件是 `binary_compressed`**。最省事的做法是直接依赖
`pcl_io`（`pcl::io::loadPCDFile`）而不是自己写 LZF。

### 怎么验证接口（当前）

```bash
# 打开接口
ros2 run map_server map_server_node --ros-args \
  --params-file src/map_server/config/map_server.yaml -p publish_3d:=true
# 期望：日志有“3D 接口：已创建发布器 …” + “⚠ M3 载入/发布逻辑尚未实现”
#       + “3D 地图未加载（不影响 2D）：PCD 解析未实现（M3 待做）：… 171590 点 …”
ros2 topic info /global_map/cloud      # Publisher count: 1，但无数据
ros2 topic hz /global_map/cloud        # 无输出（确实不发）
# 关掉开关（默认）则连话题都不存在
```

## 使用

```bash
# 启动（默认站点）
ros2 launch map_server map_server.launch.py
ros2 launch map_server map_server.launch.py map_dir:=/home/gmd/rcs/maps/r41iws_0917

# 运行时换图（不用重启）
ros2 service call /global_map/load_map nav2_msgs/srv/LoadMap \
    "{map_url: /home/gmd/rcs/maps/r41iws_0917}"

# 看结果
ros2 topic echo /global_map/occupancy --once | head -20
```

⚠ **换图三件套必须同源**：lightning 的 `map_path`、本节点（`map_dir` 或 `load_map`）、
perception。三者不一致会出现"定位在新图、规划还在用旧图"。

## 验证方法

### 1. 与官方 `nav2_map_server` 逐格比对（独立实现互证）

```bash
M=/home/gmd/rcs/maps/go2_sim_factory
ros2 run nav2_map_server map_server --ros-args -r __node:=nav2_ref \
    -p yaml_filename:=$M/map.yaml -p topic_name:=/nav2_map -p frame_id:=map
ros2 lifecycle set /nav2_ref configure && ros2 lifecycle set /nav2_ref activate

ros2 run map_server map_server_node --ros-args -p map_dir:=$M

ros2 run map_server compare_maps.py          # 或 python3 scripts/compare_maps.py
```

比对 `width/height/resolution/origin` 和**每一格**的取值。PGM 解析/阈值/行序翻转只要有一处
不同，格值就会差。实测 `go2_sim_factory`：**186349 格全部一致**（占据 12041 / 未知 174308）。

### 2. 坐标系对齐（肉眼看）

RViz 里把 `global_map/occupancy`（Map 显示项）与 `lightning/global_map`（3D 点云）叠合，
两者应完全重合 —— 不重合说明 origin/翻转/单位有问题。

## 地图语义：未知 → 空闲（已定策略）

实测三个站点的 `map.pgm` **只含黑(0) 和灰(205) 两种灰度，没有白色(254)**：

| 站点 | 尺寸 | 黑(占据) | 灰(未知) | 白(空闲) |
|---|---|---|---|---|
| go2_sim_factory | 607×307 | 6.5% | 93.5% | **0** |
| r41iws_0917 | 220×525 | 10.3% | 89.7% | **0** |
| r41iws_0918 | 699×545 | 7.2% | 92.8% | **0** |

即"除了障碍物，其余全是未知"。

### 决定：全局图里把灰色当白色（可通行）

配置里 `unknown_as_free: true`（发布时 -1 → 0）。理由：全局图是**稀疏先验**，没观测到的
区域不应该挡住全局规划（与 nav2 costmap 的 `track_unknown_space=false` 同取向）。

实测（`go2_sim_factory`）：

```
unknown_as_free: true   → 占据 12041 / 空闲 174308 / 未知      0
unknown_as_free: false  → 占据 12041 / 空闲      0 / 未知 174308   （对照）
```

日志会按**发布语义**打印统计，并标出【未知已当空闲】，避免"日志说 93% 未知、图里却没有未知"。

⚠ **与官方 `nav2_map_server` 逐格比对时必须 `-p unknown_as_free:=false`**（忠实发布时
两者的图才可比；开了开关必然在语义上有差异）。

### 与局部图的差异（规划器合成时注意）

| | 未知怎么算 | 理由 |
|---|---|---|
| 全局图（本节点） | **当空闲（可通行）** | 稀疏先验，未观测区不该挡全局规划 |
| 局部图（perception） | **保留未知**（-1），距离场乐观解释（`esdf_unknown_as_occupied: false`） | 近距离实时避障时要能区分"刚看到是空的"和"还没看过" |

合成建议：`占据(任一为 100) → 障碍`，否则可通行。

### 根治办法（可选）

在地图生成端（pcd2pgm 那一步）把可通行区域标成白色(254)，这样全局图就能同时保留
"观测过空闲"与"从未观测"的区别。

## 旧标题（保留以便检索）：已知问题-没有空闲格

## 文件结构

```
src/map_server/
  include/map_server/map_io.hpp    # 加载库接口（yaml + pgm → OccupancyGrid 语义）
  src/map_io.cpp                   # PGM(P5/P2) 解析 + trinary 阈值 + 行序翻转
  src/map_server_node.cpp          # 节点：发布 + 换图服务
  config/map_server.yaml
  launch/map_server.launch.py
  scripts/compare_maps.py          # 与官方 nav2_map_server 逐格比对
```

## TODO（后续）

- [x] M3 **接口**：参数/话题/服务接入/路径解析/头部校验与报错（本期完成，话题已可见但无数据）
- [ ] M3 **内容**：`loadPcd()` 解码（注意 `binary_compressed`）+ 填 `PointCloud2`
- [ ] 地图服务端保存（把 perception 累积的局部 2D 层存回 `pgm/yaml`）
- [ ] 与 `lightning/map_state` 联动自动换图
