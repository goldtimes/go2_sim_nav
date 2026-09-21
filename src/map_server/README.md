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
| `republish_on_new_subscriber` | `true` | **新订阅者出现时补发一次**（推荐，见下） |
| `republish_interval` | `0.0` s | 周期性重发，`0` = 关闭（仅调试/兼容用） |
| `unknown_as_free` | **`true`（本项目策略）** | 把"未知"当"空闲"发布（见"地图语义"） |
| `publish_routes` | `true` | 是否加载路网并发布可视化 |
| `routes_file` | `""` | 路网文件；空 = `<地图目录>/routes.yaml` |
| `topic_routes` | `global_map/routes` | 路网 + 区域可视化（`MarkerArray`，latched） |
| `zones.burn_into_map` | `true` | 把**禁行区**烧进发布的全局图（全栈生效） |
| `zones.inflate` | `-1.0` | 禁行区膨胀量 [m]；<0 = 自动用车体外接圆半径 |
| `footprint.*` / `common.hard_threshold` / `common.unknown_as_occupied` | 同 pnc_2d | **仅用于路网可行性校验**，必须与规划器一致 |

#### 路网（`routes.yaml`）：**可选资产**

本节点把路网当作与地图同级的**站点资产**管起来：加载、按站点切换、逐条校验、可视化。

- 文件位置：与 `map.yaml` 同目录（换图四件套同源）；
- **没有这个文件很正常**：只打一句 `该站点没有路网文件（路网是可选的，跳过）`，
  **不影响地图发布、不报错**；
- 文件坏了 / 格式不对：`WARN` 说明原因，仍然不影响地图发布；
- 加载成功后会：
  1. 用**刚发布的地图** + 车体轮廓（`footprint.*`）逐条通道做碰撞校验，
     过不去的通道在 RViz 里**标红**并 `WARN` 列出 `A→B, B→C` 这样的通道名；
  2. 发布 `global_map/routes`（latched）：节点（按语义着色 + 名字标签）、
     通道（可行青蓝 / 不可行红色）、单向箭头。
- 换站点（`load_map` 服务）时会**一并重载路网**，并把上一个站点多出来的标记发 `DELETE` 清掉。

通道默认**双向**；需要单向时给该条边加 `one_way: true`。
字段含义见 `src/pnc_2d/doc/pnc2d_restructure_plan.md` §6 P2。

#### 区域层（`zones:`）：禁行区 / 限速区

写在同一个 `routes.yaml` 里：

```yaml
zones:
  - {name: charging_hall, type: forbidden, polygon: [[10,-6],[12,-6],[12,-4],[10,-4]]}
  - {name: door_north, type: speed_limit, value: 0.30, polygon: [[2,1],[4,1],[4,3],[2,3]]}
```

- **禁行区（forbidden）= 全栈约束**：本节点按车体**外接圆半径**膨胀后烧进
  `global_map/occupancy`，因此全局规划 / 路网路由 / 局部规划 / RViz **自动都遵守**。
  只给全局规划器是不够的：局部规划不知道就会出现“全局绕开了、局部冲进去”。
  日志会报告烧入了多少格（与图中占据数一致）：
  `其中 3274 格来自禁行区（1 个区域，已按 0.472 m 膨胀）`。
- **限速区（speed_limit）**：不烧入栅格（软约束），只解析 + 可视化 + 供速度规划
  （P5）查询；本节点在 RViz 里用橙色多边形 + `0.3 m/s` 标签标出。
- `zones.burn_into_map=false` 时只可视化不生效（调试用）。
- 禁行区把某条通道切断时，那条通道会被判为“车体过不去”并 `WARN`（画错早发现）。

#### 画线器 `scripts/route_editor.py` 操作手册 ★

用 2D 地图当底图，鼠标点着画通道与区域，存成 `map_server` / `pnc_2d` 直接能吃的
`routes.yaml`。**离线工具，不依赖 ROS**。

> **核心约定：每一次点击都是一个"点"（节点）**。所以既可以直接沿走廊连点画通道，
> 也可以先放点、再进 lane 模式把点依次连起来，两种用法一致。
> 点到已有点的 `--snap` 半径内会**复用那个点**——这就是"接线/分叉"的方式。
>
> 界面文字**全英文**：matplotlib 缺中文字体时中文会变方块（“窗口显示中文有问题”）。

**依赖与启动**

```bash
cd ~/r41_ws
.venv/bin/python3 src/map_server/scripts/route_editor.py                 # 默认站点 go2_sim_factory
.venv/bin/python3 src/map_server/scripts/route_editor.py --map-dir /home/gmd/rcs/maps/office4f
.venv/bin/python3 src/map_server/scripts/route_editor.py --help
```

依赖：`.venv` 里的 `matplotlib` / `numpy` / `pyyaml`（已装）；需要图形界面（本地桌面或 X11 转发）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--map-dir` | `/home/gmd/rcs/maps/go2_sim_factory` | 站点地图目录（含 `map.yaml` + `map.pgm`） |
| `--routes` | `<map-dir>/routes.yaml` | 输出文件；**已存在则先载入**，可继续编辑 |
| `--lane-length` / `--lane-width` / `--margin` | `0.70` / `0.40` / `0.05` | 车体尺寸，用于通行性检查。**必须与 `map_server` 的 `footprint.*`（及规划器）一致**，否则会出现“编辑器说通得过、map_server 说过不去” |
| `--snap` | `0.30` | 点到已有节点的吸附半径 [m]，用于把通道接到一起 |

界面：底图是站点地图（黑=障碍），标题栏显示 `[当前模式] 通道数/节点数/区域数 | ⚠ N 条车体过不去 | 状态`；
窗口左下角状态栏跟着鼠标显示世界坐标，便于对齐。

**通道模式（lane，默认）**

| 操作 | 作用 |
|---|---|
| **左键** | 在当前通道末尾追加**一个点**：命中已有点（≤ `--snap`）就**复用**它（接线/分叉），否则就在该位置**新建一个点**。连续点几下 → 形成一个点链，最后连通成一条通道 |
| **右键** | 结束当前通道（至少 2 个点；起点==终点会丢弃） |
| `n` / `p` / `z` / `x` | 切 lane / point / zone / **delete** 模式（切换会先把未结束的对象收尾） |
| `u` | 撤销：先撤当前通道最后一个点，没有点则撤最后一条通道。**本次新建、且没有别的通道在用的点会一起收回**（不会留下孤立点） |
| `d` | 删除最后一条通道 |
| `f` | 对最后一条通道做样条平滑（点多点少都能出顺滑通道） |
| `a` | 切换最后一条通道 单/双向（**默认双向**） |
| `-` / `=` | 最后一条通道限速 −/+ 0.1 m/s |
| `[` / `]` | 最后一条通道走廊半宽 −/+ 0.1 m（调到 **0 = 严格贴线、遇障即停**） |
| `k` | 切换最后一条通道**末端节点**的类型：`waypoint→station→charge→park`（只影响配色/标签） |

**点模式（point：放独立点 / 站点）**

| 操作 | 作用 |
|---|---|
| **左键** | 放一个独立点（不连任何通道）；命中已有点则复用并提示 |
| `k` | 切换最后放置的点的类型（waypoint/station/charge/park） |
| `u` / `d` | 删掉最后放置的点（若已被某条通道引用则**拒绝**）——要连通道一起删请用下面的 delete 模式 |

**区域模式（zone：禁行 / 限速）**

| 操作 | 作用 |
|---|---|
| `z` | 进入区域模式（左键加顶点、右键闭合多边形，至少 3 个顶点） |
| **左键** | 追加一个顶点 |
| **右键** | 闭合当前多边形（新区域默认 `forbidden`） |
| `t` | 切换最后一个区域的类型：`forbidden ↔ speed_limit` |
| `-` / `=` | 调整最后一个**限速区**的限速值 −/+ 0.1 m/s |
| `d` | 删除最后一个区域 |
| `u` | 撤销：先撤当前多边形最后一个顶点，再撤最后一个区域 |
| `n` | 回到通道模式 |

颜色：禁行区**红色**多边形 + 名字；限速区**橙色**多边形 + `名字 0.30m/s`。

**删除模式（delete：点哪个删哪个）★**

上面的 `u`/`d` 都是“删最后一个”（LIFO），想删中间某个点/某条通道就得把后面画的全部撤回。
按 `x`（或 `Delete`/`Backspace`）进删除模式，**鼠标指哪删哪**：

| 操作 | 作用 |
|---|---|
| `x` | 进入/退出删除模式（进入后会先把未结束的通道/区域收尾） |
| **鼠标悬停** | 光标下的对象**变红高亮**，标题栏显示 `DELETE point P3` / `DELETE lane P1->P3` / `DELETE zone Z1`，不会误删 |
| **左键点一个点** | 删除该点；**若它被通道引用 → 连带删除那些通道**（删路口点会同时删掉相交的几条）。终端会列出被删的通道 |
| **Shift + 左键点一个点** | 同上，并额外清掉**本次删除直接造成孤立的 `waypoint`**（特意放的 `station`/`charge`/`park` 永远不自动删） |
| **左键点通道线上** | 删除整条通道，**它的点保留**（可能变成孤立点，保存时会提示） |
| **左键点区域内部** | 删除该区域 |
| **左键点空白** | 什么都不做，标题栏提示 `nothing under the cursor` |

拾取优先级：**点 > 通道 > 区域**（半径 = `max(--snap, 0.40 m)`）。所以想删一条通道时，别点在它的端点上。

> matplotlib 自带的单键快捷键（`k`=对数坐标轴、`s`=存图、`h`=home、`f`=全屏、`p`=pan、
> `backspace`=后退）已**在启动时屏蔽**，否则会和本画线器抢键（早期按 `k` 会把 x 轴切成对数坐标）。
> 方向键仍可用。

**通用**

| 操作 | 作用 |
|---|---|
| `s` | 保存到输出文件（会先把当前未结束的对象收尾） |
| `h` | 在终端打印按键表 |
| `q` | 退出（**未按 `s` 的修改不会保存**） |

**保存时自动做的三项校验**（每次 `s` 都会打印，并在图上标出问题）

0. **孤立点**：没被任何通道引用的点会单独列出来（不算错误，提醒你别忘了连）；
1. **通行性**：用车体矩形（含 margin）沿每条通道逐位姿扫一遍，**并把禁行区按车体外接圆半径膨胀一并计入**
   （与本节点同口径）→ 过不去的通道**标红**并在终端列出；
2. **连通性**：路网被切成几块会列出每块的节点名 —— 断开的路网会让规划器“只能在同一块里找通路”，
   这是最隐蔽的错误；
3. **重复通道**：同起止点出现多次会提示（多半是重复画了一遍）。

**典型工作流**

```text
1. 起 map_server（把站点地图发出来，供 RViz 叠合看）
2. 跑画线器 → 沿走廊依次左键连点 → 右键结束；路口就点到已有节点附近（自动吸附复用）
   · 画错了按 `x` 进删除模式：鼠标指到点/通道/区域变红后左键即删
3. （可选）z 切区域模式 → 画禁行区/限速区 → t 切类型、- / = 调限速值
4. s 保存 → 看三项校验输出；红标通道就调整或收窄禁行区
5. 重启 map_server（或调 /global_map/load_map 换站点）→ 日志应出现
   "路网已加载 …" / "区域层：N 个区域" / "其中 M 格来自禁行区"
6. ros2 launch pnc_2d global_planner.launch.py planner_type:=route_network
   → RViz 里点目标 → 路径严格沿通道，RViz 高亮当前走过的通道
```

**常见问题**

| 现象 | 原因 / 处理 |
|---|---|
| 想删中间某个点 / 某条通道 | 按 `x` 进**删除模式**，鼠标指到它变红后左键（`u`/`d` 只能删“最后一个”） |
| 删一个路口点，结果两条通道都没了 | 这是设计如此：点被通道引用时**连带删掉引用它的通道**（否则通道会指向一个不存在的点）。只想删通道就点在通道的**线**上，不要点在端点上 |
| 一条通道只有首尾 2 个点 | 旧版把中间点击当成"折线形状点"；**现版每次点击都是点**（节点），重画一次即可 |
| 窗口里的中文是方块 | 已改为**全英文界面**（matplotlib 缺中文字体）；数据文件里的中文注释不影响 |
| 窗口打不开 / 报 display 相关错误 | 需要图形界面（本地桌面或 X11 转发）；纯 SSH 环境请手写 yaml |
| 某条通道是红色的 | 车体过不去：贴着墙、通道太窄，或被禁行区（含 0.472 m 膨胀）挡住 |
| 保存提示“路网被分成 N 块” | 两块之间没接上：把端点画到已有节点 `--snap` 半径内 |
| 规划器报“起点离路网 X m” | 起点/终点离通道太远（默认上限 `max_entry_distance: 10 m`），或地图与路网不同源 |
| 想让某段变成单向 | 编辑器里选中该通道按 `a`，或直接在 yaml 里加 `one_way: true`（本仓库路网默认全双向） |
| 画完规划器仍然走老路线 | `planner.type` 还是 `astar`；或规划节点还是旧进程（重启节点） |

> 可视化分工：**全网**由本节点发布在 `global_map/routes`；**当前通路高亮**由 pnc_2d
> 的规划节点发布在 `/pnc_2d/plan_markers`（只有它知道这次走了哪几条通道）。
> RViz 里各加一个 MarkerArray 显示项即可，不会重复画。

### 3D 地图参数（M3 接口，本期只接线）

| 参数 | 默认 | 说明 |
|---|---|---|
| `publish_3d` | `false` | 打开则创建 `topic_cloud_3d`（latched，与 2D 同为 transient_local） |
| `pcd_file` | `""` | 空 = `<map_dir>/global.pcd`；也可直接给 `.pcd` 路径 |
| `topic_cloud_3d` | `global_map/cloud` | `sensor_msgs/PointCloud2` |
| `cloud_frame_id` | = `frame_id` | 点云常来自别的源，可单独指定 |
| `cloud_voxel_leaf` | `0.0` | 预留：发布前体素降采样（m） |
| `require_3d` | `false` | true = 3D 加载失败时整个 `load_map` 报错；否则只 WARN（不连累 2D） |

#### 重发策略：默认“发一次 + 按需补发”（不再每秒重发）

latched（`transient_local`）是标准做法：晚启动的订阅者能立即拿到历史样本。但 **RViz 的
Map 显示项默认 `Durability=Volatile`**，而 Volatile 订阅端**不会**收到历史样本
（DDS 规则）→ 先起节点后开 RViz 会看到空白。

三种解法：

| 做法 | RViz 零配置 | 稳态流量 |
|---|---|---|
| 只发一次（`republish_on_new_subscriber: false` + `republish_interval: 0`） | ✗ 需把 RViz Map 的 Durability 改成 `Transient Local` | 0 |
| **默认：看门狗按需补发** | ✓ | **0** |
| 周期重发（`republish_interval: 1.0`，旧行为） | ✓ | 一张图/秒（本项目 186349 格 ≈ 182 KB ≈ 1.5 Mbit/s），且**每秒唤醒所有下游节点** |

默认策略是第二种：**载图/换图时发一次**（latched），此后每 0.5 s 只做一次“订阅者数量是否变多”
的整数比较；一旦变多、且其中有 Volatile 订阅者，就补发一次（全是 `transient_local` 的订阅者
已经自动收到历史样本，不补发）。

实现约束：Humble 的 `rclcpp::PublisherEventCallbacks` **没有 `matched` 回调**（Iron 之后才有），
所以只能用轻量轮询，不能用“订阅者匹配事件”。已知局限：若“一个订阅者离开、另一个同时进来”
导致计数不变，可能漏补发 —— 此时可打开 `republish_interval` 兑底。

实测（本项目 `go2_sim_factory`，607×307，186349 格，同一张图）：

```
新默认（republish_interval=0 + 按需补发）：
  Volatile 订阅者接入后 6 s 内收到 1 条（首条 0.46 s 到达，即看门狗补发）
  再接入第二个 Volatile 订阅者：同样 1 条（仍能补发）
  Transient_local 订阅者：0.10 s 内 1 条（走历史样本，不触发补发）
旧行为（republish_interval=1.0）作为对照：
  同一个 Volatile 订阅者 6 s 内收到 7 条（≈ 1.27 MB / 6 s 的无谓流量）
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
