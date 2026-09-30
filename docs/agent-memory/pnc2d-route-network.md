# pnc_2d 路网（P2，2026-09-21 实现）

## 文件
- `include/pnc_2d/core/route_graph.hpp` + `src/core/route_graph.cpp`：节点/边/属性 + yaml 读写 + 投影
- `include/pnc_2d/global/route_network_planner.hpp` + `src/global/route_network_planner.cpp`：图上 Dijkstra 路由
- `scripts/route_editor.py`：matplotlib 点选画通道 → routes.yaml（不依赖 ROS）
  ⚠ 2026-09-21 已搬到 **`src/map_server/scripts/route_editor.py`**（资产编辑工具跟着资产走），
  操作手册在 map_server/README.md；map_server 的 CMake 把它 install 到 lib/map_server/
- `config/route_network.yaml`；`test/test_route_network.cpp`（11 用例）
- 数据文件约定：`~/rcs/maps/<站点>/routes.yaml`

## 关键语义（用户已确认）
- 目标只给坐标（`/goal_pose`）→ 投影到最近通道；站点（station/charge/park）只是带语义的节点
- `goal_mode`：`hybrid`（默认，首尾段用 A* 补自由空间）/ `strict`（只到投影点）
- `corridor_width`：允许横向偏离半宽；**0.0 = 严格贴线、遇障即停**（P5 局部规划用）
- **禁止 line_of_sight 剪枝**（切角会破坏人为设计的绕行）→ 强制 `collinear`
- 载入地图后逐通道做 footprint 可行性校验；`reject_infeasible=true` 时不可行通道不参与路由

## 实测（真实 607×307 地图）
- 路网 3 节点 / 2 通道 → 6 点 / 30.71 m / **0.0 ms / 5 个图节点**（同图 A*：33 ms / 5 万格）
- 单行逆向 → NO_PATH；不切角：8.00 m（切角只要 6.32 m）
- 单测 11/11；全包 0 失败

## 区域层（禁行/限速，2026-09-21 已实现）
- 写在同一个 `routes.yaml` 的 `zones:` 段；`core/map_zones.{hpp,cpp}`
- `forbidden` = **全栈约束**：map_server 按**车体外接圆半径**膨胀后烧进发布的全局图
  （`max(地图,覆盖层)`），因此全局规划/路由/局部规划/RViz/nav2 自动都遵守。
  实测 2×2 m + 0.472 m → 3274 格（占据 12041→15315）
- `speed_limit` = 软约束，不烧入；`ZoneSet::inSpeedZone()` 供 P5 查询
- 开关：`zones.burn_into_map`(true) / `zones.inflate`(-1=自动)
- 画线器：`z` 区域模式、`t` 切类型、`-`/`=` 调数值；编辑器可行性检查含禁行区膨胀

## 画线器交互模型（2026-09-21 重构，用户要求）
- **每次左键点击 = 一个点（节点）**；点到 `--snap` 内已有点则复用（这就是接线/分叉）。
  旧版把中间点击当"折线形状点"，导致一条通道只有 2 个节点 —— 用户提的 bug
- 三/四模式：`n` lane（默认）/ `p` point（独立点、站点）/ `z` zone / `x` delete
- **delete 模式**：鼠标悬停对象变红，左键即删；拾取优先级 点 > 通道 > 区域
  （半径 `max(--snap, 0.40)`）。删被通道引用的点 → **级联删除那些通道**（用户确认的语义）；
  删通道则点保留；`Shift+左键` 额外清掉本次删除造成的孤立 waypoint（station/charge/park 永不自动删）
- lane 模式 `u` 会回收"本次新建且无人引用"的节点，避免残留孤立点
- 界面**全英文**（matplotlib 缺 CJK 字体，中文会成方块）；yaml 里的中文注释保留

## 全局规划节点的"路径有效期"（2026-09-22 实现，用户要求）
- 话题 `/pnc_2d/global_path`、`/pnc_2d/plan_markers`、`/pnc_2d/global_status` 都是 **latched**
- 四条清空规则：① 失败 → 发**空 Path**；② 到达目标（`clear.auto_on_goal_reached` **默认 false**，
  `clear.goal_tolerance` 0.30 m，且只在"规划时离目标 > 容差"时生效）→ 清空（用户 2026-09-22 决定默认关：
  任务是否结束交给状态机）；③ **启动即清场**（`publishStartupCleanup()`）；④ 服务 `~/clear_path`（std_srvs/Trigger）
- `planner.type` **支持运行时热切换**（2026-09-22，用户确认"方案 C"）：
  - 参数：`ros2 param set … planner.type route_network` → 参数回调里真重建规划器；
    失败时返回 `successful=false` + reason（参数值不会变成假的）
  - 服务：`~/switch_planner`（`pnc_2d/srv/SwitchPlanner{string type → bool success, string message}`）
  - 服务：`~/reload_params`（Trigger）= 用当前参数重新 configure **同类**算法
    （改 footprint/path_prune/routes_file 不用重启）
  - 切换语义：按**当前**参数 configure → 交给新算法同一张 map（路网模式立刻重跑可行性）
    → 换指针 → `clearPlan()` → 重报 footprint/路网信息；失败保留旧算法
  - 线程前提：默认单线程 executor（回调串行）；换 MultiThreadedExecutor 必须加锁
  - 实现要点：节点保存 `std::shared_ptr<CostMap2D> map_`；`buildPlanner/switchPlanner/reloadParams`
    三个方法；清空时若当前算法无路网，还要 DELETE `route_nodes/labels/edges/oneway`
  - 坑：`.srv` 注释里**行尾反斜杠**会让 rosidl 模板报 UnicodeDecodeError
- 状态消息 `pnc_2d/msg/PlannerStatus.msg`（首次给 pnc_2d 加 rosidl 生成）：
  CMake 需 `rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")`
  并把该目标链到节点，否则找不到 `pnc_2d/msg/planner_status.hpp`
- `planner.type` 只在**启动时**生效：运行时 `ros2 param set` 不换算法 → 已加 on-set 回调 WARN
- 禁行区膨胀 `zones.inflate` 默认 -1 = 车体**外接圆半径**（Go2 = 0.472 m）；
  改它必须**同时**改画线器的 `--zone-inflate`，否则两边判定打架。
  实测：0.472 会把贴柱子的通道判为不可行，0.25（内切半径）就通过

## P3 抽象层 + 配置拆分（2026-09-22 完成）
- 接口（`include/pnc_2d/core/`）：`local_planner.hpp`（`Twist2D`/`LocalStats`/`LocalPlanResult`/`LocalPlanner`）、
  `recovery_behavior.hpp`（`RecoveryContext` 四个 std::function 注入 + `RecoveryResult` + 抽象类）
- `types.hpp` 新增 `LocalStatus`（**独立于 PlannerStatus**）、`RouteCorridor`、`DynamicObstacle`
- 模式**由数据决定**：`setCorridor(nullptr)`=free，非空=route（`LocalPlanner::mode()`）；
  `producesCmdVel()` 声明"会不会真发速度"（Null = false）
- `factory.hpp` 三套工厂：`createPlanner`/`createLocalPlanner`/`createRecoveryBehavior`，
  **未知类型返回 nullptr**；`availableRecoveries()` 本期**故意为空**（P6 才做恢复行为）
- `NullLocalPlanner`（`local.type: none`/`null`）：不产生任何 cmd_vel

## 配置拆分（launch 合并顺序，后者覆盖前者）
1. `config/pnc_2d.yaml` 总入口（话题/坐标系/common.*/footprint.*/clear.* + 类型默认值）
2. `config/local_<局部>.yaml`（现有 `local_null.yaml`）
3. `config/global_<算法>.yaml` 算法片段，**自述 planner.type**（`global_planner.yaml` 已 git mv → `global_astar.yaml`）
4. `extra_config:=<路径>` 追加片段
5. `planner_type`/`local_type` launch 参数最终强制覆盖

## P4 三节点 + 状态机（2026-09-22 完成）
- 节点：`pnc_manager_node`（状态机主控）/ `global_planner_node`（+`~/plan_path` service）/ `local_planner_node`
- IPC：manager→global = `srv/PlanPath`；manager→local = `action/FollowPath`；
  local→底盘 = `/pnc_2d/cmd_vel`（**唯一 owner 是 local**，manager 不发速度）
- msg：`LocalStatus`/`ManagerState`（都 latched）；状态机转移表全在 `src/sm/manager_sm.cpp`
- 启动：`ros2 launch pnc_2d pnc_2d.launch.py [planner_type:=] [local_type:=] [ns:=] [extra_config:=]`
- `/goal_pose` 由 manager 独占；三节点 launch 把全局节点话题入口 remap 到 `pnc_2d/manual_goal`

### ★ 配置三条硬规则（都实测踩过）
1. **段落必须写 `/**:`，不能写节点名**：`ns:=/e2e` 时 FQN 变 `/e2e/xxx`，节点名段落**静默失效**
   （实测：根 ns 下 4 项生效，加 ns 后只剩 `/**` 的 2 项）。一个文件只能有一个 `/**` 段
   （YAML 顶层重复键互相覆盖）。
2. **外部话题写绝对名，自家话题/IPC 写相对名**（`pnc_2d/state`、`global_planner/plan_path`）：
   根 ns 下等价，加 ns 后整套跟走 → 仿真/实车双栈、E2E 隔离靠这个。
3. `install/(DIRECTORY config)` 只拷不删 → 改名后旧副本会残留。

## P5 局部 MPC（P5.1 库层完成 2026-09-22）
- **参考实现 = `/home/gmd/SLAM-PNC/PNC` 的 `MpcController`**（阿克曼 `s=[x,y,θ,v]`、`u=[a,δ]`、
  稀疏 OSQP：状态+控制都是决策变量、动力学作等式约束、**gradient=0**）。
  用户明确：`scripts/mpc.py` **不作为参考**（只是历史原型）
- 我们的映射：差速 ⇒ `u=[a,ω]`（ω 扮演 δ 的角色），`B(2,1)=dt, B(3,0)=dt`；
  **新增**走廊硬约束（`|n·e|≤hw` 线性）+ ESDF 障碍（软代价 rank-1 + 线性化硬下界 + 解后真实距离复核）
- 文件：`core/local_distance_field.{hpp,cpp}`、`local/mpc_local_planner.{hpp,cpp}`、
  `test/test_distance_field.cpp`(7)、`test/test_mpc_local_planner.cpp`(15)、`config/local_mpc.yaml`
- 实测：直线稳态 RMSE 0.0002 m / 圆弧 0.0099 m / **走廊 1000 组出解 1000、越界 0** /
  P50 0.13 ms、P99 2.1 ms（94 变量 182 约束 25 迭代）
- 全包 **9 程序 / 103 用例 / 0 失败**（colcon 汇总 112）+ E2E 6 组全绿

## P5.3 节点接线（2026-09-22 完成，可用）
- 输入（实测确认）：`grid_map/occupancy_inflate_2d` = OccupancyGrid **80×80 @0.1 m 滑窗**、
  frame `map`、QoS reliable+depth1+volatile；`grid_map/esdf_2d` = PointCloud2，
  `point_step=**32**`、`intensity` 偏移 **16**（PCL PointXYZI 经 `PCL_ADD_POINT4D` 对齐，
  **不是 12**）、`width=1598/height=1`
- ★ 点云必须按**字段名**解析（`PointCloud2ConstIterator<float>(msg,"intensity")`）；
  硬编码偏移会读到 padding=0 → 距离场全 0 → "处处是障碍"且不报错
- ★ **空点云 ≠ 缺距离场**：感知只发 `0<d≤3 m` 的格，空旷处点云为空 →
  新增 `LocalDistanceField::buildFree()`（场有效、全格=max_dist）。
  含混处理的代价是车在空旷处以 0.3 m/s 爬行而日志像"感知挂了"
- ★ 订阅本身就是开关：感知只在 `get_subscription_count()>0` 时算 ESDF
- 超时 `local.esdf_timeout`(0.3 s) → `setDistanceField(nullptr)` 降级限速；
  **不用过期场做硬约束**
- **P5.3b 走廊四跳**：`RouteNetworkPlanner` → `PlanPath.srv`(`has_corridor`/
  `corridor_half_width`/`corridor_speed_limit`/`strict_corridor`/`route_edges`) →
  `pnc_manager` → `FollowPath.action`。语义：**path 即中心线**，允许 ±半宽；
  多通道取最严（半宽 min、限速 min）。没接通时"贴线走"**静默失效**（看不出来）
- 实测：空场 max|v|=1.0 m/s 闭环前进正常；墙横在路上 → `BLOCKED`
  （真值校验 `最小 0.000000 m < 0.250000 m`）；route 模式 `route_mode=true`、半宽 0.60 m
- 新增 `test/e2e/test_mpc_wiring.py`（22 项）

### ★ P5.3 三个真坑
1. **整数参数不能用 `paramDouble` 读**：yaml 写 `local.esdf_fill_radius: 2` → ROS 参数是
   integer → `as_double()` 抛 `ParameterTypeException` → **节点启动即崩** → 加 `paramInt()`
2. 新增订阅 `sensor_msgs` 要同时改 `package.xml` + `find_package` + `ament_target_dependencies`
3. 滑窗原点每帧都变 → "几何变化"告警会永久刷屏 → 只在**尺寸/分辨率**变化时告警

### ★ E2E 里"车不动"会被管理器取消
管理器**卡住判据**（`sm.stuck_timeout` 10 s 内位移 <0.2 m）会 cancel 任务 →
静态位姿测试跑不到后面的阶段。两招：放宽 `sm.stuck_timeout`；
主力阶段按收到的 `cmd_vel` 做**差速运动学积分**闭环（顺带覆盖 `setCurrentVelocity`）。
另外 `BLOCKED` 连续 1 s 就结束跟随 → 想观测 BLOCKED，取样窗口必须**立即**开始。

## P5.4 追加③（2026-09-23：**区域层进局部**，进行中）
用户拍板：膨胀 **5 cm**；限速区**也接全局**；车在禁行区内 ⇒ **停车**；做**最小恢复行为**。

- **① 传输 ✅ 已完成并端到端验证**：新增 `msg/Zone.msg` + `msg/ZoneArray.msg`
  （后者带 `float64 inflate` = 这组几何按多大膨胀用，消费者照用，避免两边判定不一致）；
  `map_server` latched 发布 `/global_map/zones`（`publish_zones`/`topic_zones`），
  **任何加载路径都发一次（含空数组）** —— 空数组 = "这张图确实没有区域"，
  与"还没加载"是两回事，消费者必须能区分（为此把 applyZones 的多个 return 改成统一出口）。
  `zones.inflate: 0.05` + 画线器 `--zone-inflate` 默认值同步。
  实测：`inflate = 0.050 m | 区域 3 个`（Z1/Z2/Z3 禁行，各 4 顶点，带 bbox）。
  验证手法：**latched 话题不能用 `ros2 topic echo`（默认 volatile 收不到）**，
  用 `test/sim/topic_probe.py`（transient_local 订阅 + 打印摘要）。
- **② A1 栅格融合 + A2 距离场融合 + 车内⇒停车 ✅ 编译+单测（126/0）**
  · `ZoneSet::adopt(几何)`：消息→内部表示，校验与 YAML 路径**同一套**（否则"文件里合法、
    运行时非法"会分叉），自动算 bbox；`forbiddenNameAt()` 报错点名用。
  · `LocalDistanceField::fuseForbidden(zones, inflate, band=1.0)`：逐格
    `min(现有, pointInPolygon?0:max(0,到边界距离−inflate))`；只扫"bbox + band"，
    但**不能只写区内**（边界外会留大值 ⇒ 插值出"0 跳到 3 m"的断层）。
  · 节点：订阅区域（transient_local）→ 每帧把禁行区烧进局部栅格副本（footprint 硬判定，
    与全局同款）→ 每帧并进距离场（MPC 避让/硬下界/解后复核）→ 车在区内直接
    BLOCKED 并点名区域（不做绕行）。
- **★ 踩到并修掉的坑（重要）**：`distanceToPolygon` **不区分内外** —— 区内返回的是
  "到边界的距离"（1×1 m 区中心 = 0.45 m），而头文件注释错写成"在多边形内时为 0"。
  照那条注释写融合，就把**区域内部**当成"离障碍还有几米" ⇒ 车会往区里钻。
  必须写 `pointInPolygon ? 0 : distanceToPolygon`（`burnForbidden` 用
  `pointInPolygon || distance <= inflate` ✓ 它是对的）。注释已改对，单测钉住。
- **频率结论**（用户问过）：栅格融合跟着 occupancy **20 Hz**、距离场融合跟着 ESDF
  **10 Hz**（都是"每帧场重建的后处理"，天然不残留旧区域）、速度帽查询在 **50 Hz**
  控制回路里只做点级测试 ⇒ **50 Hz 里没有逐格扫描**，新增 < 0.3% CPU。
- **③ 待做**：A3 限速帽（沿参考前瞻刹车距离）、限速区接全局（walk 路径取最严）、
  最小恢复行为（`requestReplan` 进工厂 + `recovery.type`）、`test/sim/test_zones.py`
  仿真验收（"横一条禁行区带 ⇒ 进不去；把区挪开 ⇒ 同一目标能到"这对反证）。

## P5.4 追加②（2026-09-23：**修好"有角度路网不动"** = 走廊只在车道段生效 + 只在车里时才启用）
用户拍板：**先自由导航上线，再严格贴线**（"按现在的做吧"）。已实现+验证：贴线 60° 角度差
**8/8 通过**、常规通道 8/8、自由空间 6/6、单测 125/0。

★★ **根因（两层，都要修）**
1. **走廊被"整条路径一刀切"**：hybrid 路径 = [自由入口段, 路网段, 自由出口段]，而
   `PlanResult`/srv/action 只传一个标量半宽 ⇒ manager 用
   `assign(path.size(), half_width)` 把入口段也当成"严格贴线的中心线" ⇒ 车在车道外
   几厘米就被硬约束判死。修：**逐点走廊**（接口本来就有 `corridor_width[]`，只是没人填）。
   约定（务必分清，混了必出怪现象）：`w>0` 允许偏离 ±w；`w==0` **严格贴线**；
   `w<0` **无走廊约束**（自由段）；空数组 = 纯自由任务。
2. **走廊硬约束不能用来"从车道外收敛"**（实测：严格走廊 hw_eff=0.05 时，起点横向偏差
   ≤0.05 一切正常（4.11 m/12 s）；**≥0.055 就 120/120 周期 BLOCKED 且 OSQP 4000/20000
   次迭代都不收敛**）。两版"收拢漏斗"都失败（二次可达量 / 线性速率+转向建立时间），
   **已全部回退**成最简硬界 `|n·e| ≤ hw_eff`。正确做法是上游解决（见下）。

**改法（不碰状态机、不加第二个规划器实例）** —— 一条路径，两种语义：
- `route_network_planner`：按几何（点到各通道中心线距离 ≤ 0.05）给每个路径点填半宽，
  入口/出口段填 -1 ⇒ `PlanResult::corridor_width_per_point`。
  ⚠ 判定容差**不能**随走廊宽度放大（`max(0.05, w)` 会把离车道 0.5 m 的入口点也算成
  车道点，于是交给局部的"中心线"前面挂一段偏离车道 0.5 m 的曲线 ⇒ 实测贴线偏差
  0.12~0.17 m）。
- `PlanPath.srv` 加 `float64[] corridor_width`；全局节点按逐点填（长度不一致时告警退回标量）。
- `manager`：逐点透传给 action（**不再** assign 等宽）。
- `local_planner_node`：`corridorSlice()` 切出走廊段（`core/corridor_slice.hpp`，纯逻辑），
  中心线 = **子路径**；**每周期**决定是否启用：`pass_index_ ≥ slice.begin` 且
  `distanceToPolyline(pose, 中心线) ≤ max(half_width, planner_->corridorTolerance())`
  ⇒ 只有"走完入口段且车已在走廊里"才把走廊交给算法，否则自由跟踪沿同一参考收敛。
  进出走廊各打一行日志（"进入走廊 → 严格贴线"）——现场诊断就靠它。
- `LocalPlanner::corridorTolerance()`（默认 0；MPC 返回 `corridor_min_tolerance`），
  单一数据源，避免节点再配一份。
- MPC 前置检查：`lat0 > hw_eff` ⇒ 立刻 BLOCKED + 可操作说明（**不进求解器**）：
  实测用例从 11.6 s 降到 53 ms，`solver_iterations == 0`。

**指标口径也必须分段**（否则会把正常行为判成失败）：贴线误差只在 `route_mode=true`
的样本上考核（严格贴线期间 max 0.049 m ✓）；自由上线/恢复段单独报（0.16~0.19 m，
必然外摆——走廊在硬约束下从车道外收敛不可行）。`Probe.rows` 末位加了 `route_mode`。
代价：角度差大时到点变慢（59° 用 24 s 走 3.75 m；顺行 ~15 s），因为要先对正+回收横向。

## P5.4 追加（2026-09-22 晚：到点 3 cm + 航向差/走廊偏移导致"车不动"）
用户 4 条要求：① 底盘 cmd_vel 看门狗**不做**；② 大航向差要"先原地转正再走" ✅；
③ 局部地图要适配禁行区 + 限速区（**未开始**）；④ 到点误差 ≤ **3 cm** ✅。

- **★★ "有角度的路网时机器基本不会动"（用户报的现象）= 两个独立的坑**
  A. **拒动阈值实测**（严格走廊 `corridor_width=0`，单测 6 s 闭环，`StrictCorridorHeadingSweep`）：
     航向差 10°/20° → 走 2.01 m；**30°/40°/60° → 0.000 m、60/60 周期 BLOCKED**。
     ⇒ 阈值必须取在拒动阈值**以下**：`align_in_place_deg: 15`（原来设 30，要 >30 才触发，
       正漏在边界上 → 就是用户看到的现象）。
  B. **走廊起点横向偏移的硬边界**（`StrictCorridorRecoversAfterInPlaceTurnDrift`）：
     `lat0 ≤ hw_eff`(0.05) 一切正常（4.11 m/12 s）；**`lat0 ≥ 0.055` 就 120/120 周期
     BLOCKED + `maximum iterations reached`**（求解器不收敛，不是显式不可行）。
     原因：收拢漏斗 `0.5·v·ω_max·(k·dt)²` **恰好压在可行边界上**（第 2 步就要求 15 mm
     横移，底盘得先把机头转过去才可能横移）。
     改法（★ **写到一半未验证**）：`corridor_recover_rate`(0.10 ≈ 0.5·v·ω_max) +
     `corridor_align_time`(0.3 s 建立横移时间)，漏斗 = `max(hw_eff, lat0 − rate·
     max(0, k·dt − align_time))`，且**三处必须用同一个函数**（QP 上下界 / 解后复核 /
     前置可达性检查）——`corridorAllow()`。
- **原地对正模式**（用户要的行为）：`local_mpc.align_in_place_deg/align_exit_deg(8)/
  align_gain(1.2)/align_min_clearance(0.25)`；只发 ω、不发 v；单测里横向位移 0.0000 m。
  ⚠ 仿真里"原地转"其实是**划小弧**：车头转 30~90° 会横向漂 0.09~0.19 m → 直接撞上 B。
- **到点 ≤ 3 cm**（用户要求）：`local.goal_tolerance: 0.01`（判定要严格小于要求）
  + 终点段 `approach_dist 0.15 / approach_speed 0.03 / crawl_speed 0.015`
  + `stop_coast: 0.035`（★实测标定："指令归零后底盘还会自己走"的距离；=0 时全部**冲过**
  目标 0.03~0.06 m，0.015 仍偏长 +0.003~+0.037，0.035 → −0.018~+0.010）
  + `brake_acc: 0.15`（实测有效减速 ≈0.13 m/s²，原来写 0.6 = 模型认为能急刹）。
  实测 4/4：停点 **0.015~0.024 m**；改 approach_speed 后 stop_coast 要重标。
  接口：`LocalPlanner::stopCoast()`（节点做到点判定提前量，单一数据源）。
- **验收脚本新增能力**：`--turn <deg>` 强制通道与车头夹角（确定性复现大角度差）、
  `--len`；通道选择**过滤**净距（≥0.55，车体外接圆 0.40）；输出"判定到达时误差 /
  判定后滑行 / 沿行驶方向带符号偏差（偏长还是偏短）"—— 没有符号就没法标 stop_coast。
- **改参数前先做单测扫描**（毫秒级）代替反复跑仿真（40 s/轮）：`--gtest_filter='*Sweep*'`、
  `*TurnDrift*` 这两条就是为此而生的。
- ⚠ 收尾状态：最后一次改动（`corridor_align_time` 漏斗）**只编译过、没跑验证**；
  改前 `StrictCorridorRecoversAfterInPlaceTurnDrift` 仍 FAIL（lat0≥0.055 不动）。
  明天第一步：跑 `./build/pnc_2d/test_mpc_local_planner --gtest_filter='*TurnDrift*'`
  看 0.055/0.06/0.10/0.19 是否都能走起来。

## P5.4 仿真（2026-09-22，自由空间 + 贴线验收全过）
- 仿真栈：gazebo + `run_loc_online`(lightning) + perception + map_server，地图与 E2E 同一张
  （607×307 @0.05，frame `map`，车起点 ≈(0,0.04) 朝 +x）
- 测试：`test/sim/sim_common.py` + `test_drive_goal.py`(6/6) + `test_route_lane.py`(7/7)
  · 自由空间：到点 0.057~0.085 m、末速 0.013~0.024 m/s、横向 0.023~0.026(稳态)/
    0.056~0.058(全程) m、保真度 0.940
  · 贴线（corridor_width=0）：贴线 0.024(全程)/0.019(稳态) m、到点 0.110 m、route_mode ✓
- ★★ **5 个真根因**（都先量后改，详见 doc/mpc_local_planner_plan.md §5.4 表）
  1. **硬状态约束没考虑"从当前状态的可达集"**（速度上界**和**下界都踩了）：
     `v_0=v_now` 不受约束，`v_now` 跑出 [v_min,v_upper] 就要求一步内吃下差值 >
     `a_max·dt` ⇒ `primal infeasible`。修：`u=max(v_up, v_now−a_max·dt·k)`、
     `l=min(v_min, v_now+a_max·dt·k)`
  2. **速度反馈 `odom.twist` 低速不可用**（噪声 ±0.1 且**均值偏低**，车走 0.045 读到
     −0.03）→ 而命令结构是 `v_cmd ≤ v_now+vives a_max·dt` ⇒ 起不来（cmd 卡 0.06，
     60 s 只挪 3 m）。修：`local.vel_from_pose`（0.25 s 窗口位姿差分 + 低通）
  3. **禁行区膨胀算两遍**（map_server 外接圆 0.472 烧图 + 规划器 footprint）⇒
     车位在禁行区外 0.72 m 就 `PlanFail[起点轮廓重叠]`。用户决定 `zones.inflate: 0.0`
  4. **底盘保持最后一条指令**：任务结束/退出不发零速 ⇒ 车一直走（测试被中断后车跑掉）
     → finish/endAsCanceled/on_shutdown/析构发零速。底盘侧仍应加 cmd_vel 看门狗
  5. **★★ 参考折线切线在毫米级基线上算成噪声**：`rebuildReference` 用相邻两点
     `atan2(Δy,Δx)`，而贴线模式路径首两点重合（车位置=通道起点，~1e-5 m）⇒
     `ref_yaw[0]` 差 ~90° ⇒ `e_yaw0=−96.7°`、`curv=0.546`(直线上!) ⇒ 一加速就撞
     走廊硬界、代价上"不动"更便宜 ⇒ **MPC 趴窝**（solved 但 cmd v=0）
     修：切线/曲率都用 `min_span=5cm` 基线（前后各找 ≥5cm 的点）
- **否决过的**：放松障碍硬下界（会沿规划直插障碍，单测抓到）；调 `q_v`（扫过 3 与 50
  行为完全一致 ⇒ 不是它，教训："算出来像"不等于证据）；对 twist 滤波（滤不掉均值偏置）
- **验收口径 5 个陷阱**（比控制器 bug 更容易误判）
  1. **`local.goal_tolerance` 必须严格小于验收阈值**！原来也设 0.15 ⇒ 管理器在
     `remaining≤0.15` 就停机，量到的"到点误差"就是**它自己的触发条件**（同义反复，
     量到 0.141 m 看着合格）。改成 0.08 后残差 0.017~0.043 m —— 这才是真收敛精度
  2. **"几何够空"≠"全局可达"**：候选目标净距 0.82 m 但 A* 报"目标不可达"。验收脚本
     要备**多个候选**、收到 FAILED 就换（可达性是全局规划器的职责，别记成局部失败）
  3. **全程偏差要区分"起步对线的外摆"与稳态**：车头与通道差几度，切线进来必然外摆
     ⇒ 全程 0.048~0.058、稳态 0.025（硬界 0.05 只对稳态）
  4. latched 话题首帧是**上一轮**的残留（见上）
  5. `signed_offset` 索引：轨迹行是 `(t,x,y,yaw)`，`x=pose[1]`（当成 (x,y,..) 会算出 16 m）
- **最终数字（2026-09-22）**：自由空间 6/6 —— 到点 0.017~0.043 m、末速 0.012~0.013 m/s、
  横向 0.016~0.026（稳态）/0.051~0.058（全程）m、保真度 0.93~0.96；贴线 7/7 ——
  0.048（全程）/0.025（稳态）m、到点 0.017 m、`route_mode=true`；单测 109 用例
  /colcon 118 tests 0 failures
- **已知弱点**：严格走廊 + 航向差 >~40° 时 MPC 拒动且不原地转（应"先转正再走"，
  差速车物理上可行）；`StrictCorridorWithHeadingOffsetStillDrives` 把现状钉住
- 底盘特性见 `config/go2_run.yaml`：静态饱和 0.42 m/s、ω 增益 0.70、瞬态可到 0.89、
  `twist.angular.z` 恒为 0。根治要在底盘侧（cmd_vel_pub 按 m/s 直通）

### ★ MPC 三个真坑（都不是调参问题）
1. OSQP 只读 `P` 的**上三角**并会校验；变化率项写成 (k+1,k) 直接被拒
   （`P is not upper triangular`）→ 建表时加自检 abort
2. **动力学等式漏了 `A` 的单位阵项**（`-e_k(i)`）：约束被放开、求解器报成功、
   **预测轨迹瞬移**（最危险：拿假轨迹发指令）→ 加 `max_dynamics_residual` 自检
   （正常 ~1e-14，>1e-3 拒发）+ `dumpQp()` 导出 H/q/A/l/u
3. 距离场填充**别复制最近邻**（阶梯场，插值误差 0.15 m）→ 改"可用邻居取平均 + 逐轮外扩"（0.08 m）

### ★ OSQP 工具链坑
- vendor **没有** `osqp` imported target、`${osqp_vendor_INCLUDE_DIRS}` 可能为空 →
  用 `find_path(OSQP_INCLUDE_DIR osqp.h)` + `find_library(OSQP_LIBRARY osqp)`，拿不到就 FATAL_ERROR
- 头文件是 **0.5 风格**：`osqp_setup(&work, const OSQPData*, &settings)`（不是 0.6 的 P,q,A,l,u,m,n），
  `osqp_update_P_A` 多两个 `*_idx` 参数（传 `OSQP_NULL` = 全量按序更新）——尽管 OSQP_VERSION 写着 0.6.2

## 踩过的坑
- **E2E 起节点不要用 `ros2 run`**：它只是 wrapper，`Popen.terminate()` 杀不到真正的节点 →
  孤儿进程继续跑，多实例抢同一话题会造出"重复消息/别人发的空路径"这类灵异现象。
  直接 exec `install/pnc_2d/lib/pnc_2d/global_planner_node` + `start_new_session=True` + `killpg`
- **启动清场必须重复发几拍**（0.5s × 6，有结果即停）：RViz 的 marker 订阅是 **VOLATILE**，
  只收"匹配之后"的消息；启动瞬间发一次会早于 DDS discovery 被丢弃 → 旧高亮留在 RViz
- **RViz 不会在发布者消失后自己清**；marker 的 `active_marker_count_` 是**每进程**的，
  换算法重启后新进程不知道旧进程发过几条 `route_active` 高亮 → 永远发不出 DELETE，
  黄色高亮残留。修法：清空/启动时按 id 0..63 全范围 DELETE
- 用户地图 `map.pgm` 只有 {0, 205} 两种灰度 → raw 侧 **0 个空闲格**，空闲全靠
  map_server 的 `unknown_as_free: true`；因此"真实障碍"要看 raw，不能看发布图
- **E2E/仿真指标别信这三样**：① `odom.twist`（低速是噪声且均值偏低）；② `local_status`
  的采样时机（任务结束就不发了）；③ latched 话题首帧（是**上一轮**的旧路径 ⇒ 横向误差
  算成 12.9 m 这种离谱值）。一律用**高频位姿**自己算
- **`ros2 topic echo` 轮询不能用来计时**：每次调用 0.5~1.5 s 启动开销，"sleep 1 s"
  实际间隔可能 3~4 s（据此算出过 0.46 m/s 的假速度）。要进程内定时器采样
- **目标朝向要给对**：随手 `orientation.w=1`（yaw=0）而路径方向 140° 时，全局路径
  "末点取目标朝向"会造出**假转弯**（curv 0.49 1/m），曲率前瞻 2 m 外就压速
- **目标别放正后方**：首点保留当前朝向 + 末点目标朝向 ⇒ 差 180° 时车绕大 U 形
  （走 5.18 m / 路径 3.75 m），那种弧线不能当"跟踪误差"场景
- **禁行区不膨胀（2026-09-22 用户决定）**：`zones.inflate: 0.0`（原来 -1=外接圆 0.472，
  与规划器 footprint 检查重复计算 ⇒ 禁入带 0.7~1.0 m，车停在禁行区外 0.72 m 就无法起步）。
  画线器 `--zone-inflate` 默认值同步改。map_server 加了 on-set 参数回调
  （原来 `param set` 只改参数服务、成员不变 ⇒ "改了没反应"）
- ⚠ 手工起 `ros2 launch ... | tee` 后杀终端**不会**杀掉节点 → 残留孤儿节点要手动 kill
- **测 MPI 类 bug 的方法**：先看诊断量（`diagString()` 那行），再做**对照实验**
  （关掉某个约束看现象是否消失），最后才算代价；**"算出来像"不等于证据**
- matplotlib 会自动注册 `key_press_handler`（`FigureManagerBase.__init__`），默认 keymap 与画线器
  抢键：`k`=x 轴对数坐标、`s`=存图、`h`=home、`f`=全屏、`p`=pan、`backspace`=后退 →
  启动时调 `disable_default_keymaps()` 清掉所有单字符键位（方向键保留）
- yaml-cpp `as<bool>()` 对 `0/1` 抛异常 → 统一走 `parseLooseBool`
- 画线器必须按**坐标**复用节点，否则共享交点会生成两个同坐标节点把路网断开；保存时加连通性检查
- MarkerArray 不会自动删除"这次没出现的"标记 → 通路高亮要用 `DELETE` 显式清理
- **局部片段名不能按类型名拼**：`local.type: none` 对应文件 `local_null.yaml`（yaml 写 none 更自然，
  实现类叫 NullLocalPlanner），拼字符串会找 `local_none.yaml` → 用显式映射表
- **`install/(DIRECTORY config ...)` 只拷不删**：改名/删除 `config/` 下的文件后，旧副本会留在
  `install/pnc_2d/share/pnc_2d/config/` 里误导排查 → 改名后手工清一次
- **同名节点连着起停有 DDS 服务残影**：上个节点刚被杀时 `wait_for_service()` 立刻成功但异步调用
  石沉大海，日志看着像"节点没起来"（其实它正常在打日志）→ 重试 + **每轮重建客户端** + 用例间隔几秒
- 单测计数口径：`colcon test-result` 的 tests = gtest 用例数 + 程序级记录（6 程序 + 73 用例 = 79）
- `.venv` 原无 pyyaml → 已装（画线器读 yaml 用）
- **E2E 里别 `pkill -f <节点名>`**：用户真实栈（`run_loc_online`/`map_server`/旧 global_planner）
  可能正在后台跑 → 会被一起杀掉。隔离靠 `ns:=/e2e` + `extra_config` 把外部 IO 指到 `/t/*`，
  自家话题/IPC 是相对名会跟着 ns 搬走，跑完只 `killpg` 自己的 launch
- **两个定位源交替喂位姿** → 管理器每帧判"跳变" → 每秒几百条 WARN + 重规划风暴
  → 加 `sm.odom_jump_cooldown`（默认 2 s）
- **`NullLocalPlanner` 永不报 kGoalReached** → action 永不结束、状态机卡在 FOLLOWING
  → 到位判定放**节点层**兜底（`remaining() <= local.goal_tolerance`），不依赖算法自报
- **`ServerGoalHandle::canceled()` 只在 `is_canceling()` 时合法**（否则抛 UnawareGoalHandleError）
  → 服务端强制停止只能走 `abort()`，用 `result.canceled=true` 区分"被叫停"与"失败"；
  manager 判结果要**先看语义标志再看 action code**

## 区域层消费 + 恢复行为（P5.4，2026-09-23 实测收口）
- 传输：`map_server` 发 **`/global_map/zones`**（`ZoneArray`，latched，**消息里带 `inflate`**）
  → global/local/manager 都订；消费方**必须用消息里的 inflate**（两个值不一致就会出现
  "编辑器/全局说通得过、局部说过不去"）。latched 读不到用 `ros2 topic echo`，
  用 `test/sim/topic_probe.py`
- 局部：① 每帧把区域按 inflate 烧进局部栅格（footprint 判死）② 每帧把区域并进 ESDF
  （`d = max(0, d_poly − inflate)`，区内 0）③ **车已在禁行区内 ⇒ 停车报 BLOCKED 并点名**
  （不绕行：禁行是语义约束，绕行等于降级成"尽量避开"且与全局打架）
- 局部限速帽：每周期前瞻 = `speedLookahead(maxSpeed(), brakeAcc())`；**只压速度，不改路径**
- ★ **跨层的物理量只配一遍**：前瞻要用的最大速度/减速度都从**算法接口**取
  （`LocalPlanner::maxSpeed()` / `brakeAcc()` ⇒ `local_mpc.v_max` / `local_mpc.brake_acc`），
  节点参数 `local.brake_acc` 只在算法不报时兜底（0 = 不兜底，默认）。踩过：节点里写着
  四足标定值 0.15、而 `local_mpc.yaml` 里是 1.0 ⇒ 前瞻差 6.7 倍，日志上看不出来。
  同一个教训：`stopCoast()`、`corridorTolerance()`。两个值差 >5% 会 WARN 并说明用哪个
- 换底盘（轮式）要动什么：算法层**没有**四足专有逻辑（无步态/腿运动学/离地判定），
  运动学是**差速**（状态 [x,y,θ,v]，θ̇=ω、v̇=a，无轴距/前轮转角）；只需换
  `config/<底盘>_run.yaml`（v_max/w_max/a_max/reference_speed/footprint/stop_coast/
  brake_acc/align_in_place_deg）+ 话题适配。**唯一的能力假设 = 可原地转向**
  （align 模式 + ω 独立于 v）：差速轮式 OK，阿克曼要换模型（ω=v/L·tanδ）+ 把原地对正
  改成最小转弯半径内对正。另：底盘侧别照搬 `cmd_vel_pub.py` 的 `0.035(1−e^{−3.5x})`
  （四足步态语义），规划器里没有对它的补偿
- 全局任务限速：**仅当起点与终点都在限速区内**才给（整趟都该慢）；只是路过 ⇒ 交给局部。
  路过也给会让"一条长路线路过 2 m 限速区"整趟爬 0.15（实测段 3 被它压住，
  "出区后恢复"永远测不出来）

### 到点朝向（2026-09-23，用户指出）
- 症状：到点判定只管 xy（`remaining − stop_coast ≤ goal_tolerance`）⇒ **机头朝哪都算到达**；
  而任务目标天生带 yaw（面向桩/门口），验收脚本也只量 xy ⇒ 差 90° 都没人发现
- 修法：`MpcLocalPlanner::goalYawAlign()`：位置到了（剩余 ≤ `goal_yaw_align_distance`）而
  |目标朝向误差| > `goal_yaw_tolerance_deg`（local_mpc.yaml 里 **10.0**，5.0 太严会不收敛，见下）⇒ 原地转
  （v=0，ω=align_gain·e 限幅，需 `align_min_clearance` 净距，否则报 BLOCKED 说明原因）。
  目标朝向 = 路径**末点** yaw。**必须放在 `remaining≤1e-3 → kGoalReached+零速` 之前**，
  否则"算法停住、节点又不算到" ⇒ 死锁
- 节点侧：`finishReached()` 位置 + 朝向都要满足；容差走 `LocalPlanner::goalYawTolerance()`
  （单一来源，别在节点再配一份）。`pathToPoses()` 必须 `has_yaw=true`（否则判定被静默跳过）
- 坑：`wrapAngle` 原来在 `mpc_local_planner.cpp` 是**文件私有**的，节点算不出同样误差 ⇒
  已提到 `core/types.hpp`（公共 inline）；同时"匿名命名空间里再写一份同名函数"会让调用处
  报 ambiguous（`global_planner.cpp` 踩过）
- 验收：单测 `AlignsGoalYawInPlaceAtGoal` / `BlocksGoalYawAlignWhenTooTightToRotate`；
  仿真 `test_drive_goal.py` 新增 `[A3] 末朝向偏差 ≤ 5°`（容差从 local_mpc.yaml 读）

### 三个真 bug（都属"接好了但没生效"）
1. **恢复行为从未被调用 ⇒ 永久卡在 RECOVERING**：转移表里 `{Following, Blocked}`
   的副作用是 `kStopRobot` ⇒ 没人跑恢复、也没事件推出去；而 RECOVERING **不接受新目标**
   （现象：被挡一次后所有目标被静默拒绝，只能重启）。修：`kRunRecovery` +
   `ReplanRecoveryBehavior`（`sm.recovery.type: replan`，防抖 `replan.min_interval_s`
   **默认 0 = 关**，额度上限已经是防循环的闸）。语义定为**任何恢复之后先重规划再跟随**
   （`RecoveryDone → Planning/kPlanPath`），否则会立刻再跟那条走不通的旧路径、白烧额度
2. **一次求解失败 ⇒ 永久失败（OSQP 被毒化）**：热启动喂的是上一次**失败**的解
   （迭代值已发散）⇒ 4000 次迭代全花在回拉/重缩放。现象：区域挡住一次后**把区域挪走也不动**，
   且 `障碍行0`（没有障碍约束生效）说明问题不在障碍。修：只缓存**可行解**做热启动，
   失败即回零；回归 `SolverRecoversAfterInfeasibleCycle`
3. **限速区不生效（三处错误叠加）**：① 前瞻用**当前**速度 ⇒ 越慢看得越近，自锁
   ② 前瞻从**路点下标**开始 ⇒ 2 点路径下标恒 0，帽戴上摘不掉 ③ 旧实现只看路径点 ⇒
   2 点路径等于只看首尾。修：用 `maxSpeed()` 算前瞻 + 从**车在路径上的投影点**前瞻 +
   沿线段按 0.25 m 弧长重采样（`zoneSpeedLimitAheadFromProjection`）

### 验收与口径
- `test/sim/test_zones.py` **7/7**：段 1 禁行带（净距 +0.46 没进区）→ 段 2 **反证**
  （挪开区域后同一目标能到，误差 0.011~0.037）→ 段 3 限速（进区边界前 0.161~0.174 /
  区内 0.166 / 出区后 0.277 m/s，限速 0.15）
- 回归：自由 6/6（0.009~0.043 m）、贴线 8/8 + `--turn 60` 8/8（走廊期间 0.048~0.049 m）、
  单测 **134 tests / 0 failures**
- 口径坑：① **三段必须各自从当前位置重新选路**（前一段修好后车会真的开到终点，
  沿用"起点+距离"会把区域放在车屁股后面 ⇒ 指标全 None）② 速度指标别在减速窗口取平均
  （"进区前"要取紧贴边界外侧 0.02~0.15 m）③ `最小距0.00` 是**哨兵值**（求解失败时没有
  预测轨迹），不是"贴着障碍"—— 打印层已改成 `n/a`，为这行字白花过很久
- 贴线验收依赖"车周围有空地"（要车前 3~6 m 净距 ≥0.55 m）：车停墙边时跑
  **`test/sim/park_open.py`**（按全局图找离障碍最远处开过去；用切比雪夫腐蚀，不用 scipy ——
  系统 scipy1.8 与 venv numpy2 ABI 不兼容）


## 对正收敛 + 自由模式顺滑（2026-09-23 用户实测两条）
用户原话："角度判断太严格，导致无法收敛；机器人摇摇摆摆，自由导航不需要严格遵循全局路线，而是保证顺滑。"

### A. 朝向容差过严 ⇒ 不收敛（根因在**底盘死区**，不在判据）
- `ω = align_gain·e`：靠近容差时 e 已很小（11°、gain 0.3 ⇒ 0.058 rad/s）
  ⇒ Go2 低速角速度死区里**指令发出去车不动** ⇒ 永远停在容差外。
- 三处修（各带单测）：
  1. `align_w_min` **角速度下限**（打破死区）：`|ω| = clamp(|gain·e|, min(align_w_min, w_max), w_max)`，
     方向取 `sign(e)`。**库层默认 0 = 中立**，具体值属底盘特性 ⇒ Go2 标定 0.20 写在 `go2_run.yaml`。
  2. `goal_yaw_tolerance_deg` 5.0 → **10.0**（验收阈值要比机构可达精度松一档）
  3. `goal_yaw_align_timeout`（默认 6 s）：**无进展**超时（连续 6 s 偏差改善 <2°）
     ⇒ `kFailed` 并报"还差几度 + 该查 align_gain/align_w_min/定位噪声，或放宽容差"，
     **不许无限原地转**（任务层看就是卡死）。★ 必须是"无进展"而非墙钟：180° @ 0.65
     rad/s ≈ 5 s 本来就慢，按墙钟计会把"转得慢但在收敛"判失败
- 单测：`GoalYawAlignHasBreakawayOmega`（无下限 0.058 会卡死 / 有下限 ≥0.20 且方向正确）、
  `GoalYawAlignTimesOutWithActionableReason`、`GoalYawAlignDoesNotTimeOutWhileProgressing`

### B. 自由模式"摇摇摆摆"（根因 = **两模式共用一套贴线代价**）
- 自由段参考只是"大致往那儿走"的折线，却和走廊共用 `q_x/q_y=1000`、`q_yaw=50`
  ⇒ 几 cm + 几度偏差被当大误差追 ⇒ 实测 ω 相邻周期 **+0.65 → −0.65**（顶满、1~2 s 周期）= 左右摆。
- 修法（**代价按任务语义分开表达**，不是调参）：
  · `free_lat_scale` / `free_yaw_scale`：自由模式横向/航向权重缩放（走廊 = 1）
  · `free_lat_deadband`：横向死区 —— 实现是**把参考窗口整体横移** `shift = clamp(lat, ±band)`，
    每点沿自身法向抬 `shift`（**不要**去覆写 `info_.cross_track`，单测在比它）；
    带内 shift = lat ⇒ 偏差为 0 ⇒ 不打方向盘
  · `free_w_max`：自由模式 ω 上限（`wMaxEff(route)`，同时用于控制上下界与指令钳位）
  · 实现是**严格推广**：`P_xy = 2·scale·(q_lon·t tᵀ + q_lat·n nᵀ)`，scale=1 时与原
    `diag(q_x,q_y)` 逐位相同 ⇒ **库层默认全部中立（= 严格）**，产品策略写在 `local_mpc.yaml`
- ★ **教训**：一开始把"策略值"写进**库层默认值**，立刻打挂两条精度单测
  （`StraightLineTrackingRmse` 稳态 0.098 m、`ObstacleHardConstraintForcesDeviation`）
  ⇒ 库默认中立、策略在 yaml；走廊模式与精度单测因此完全不受影响（单测已钉住模式隔离）
- 单测：`FreeModePrefersSmoothnessOverLineTracking`（① 带内 5 cm 不纠 vs 硬贴线 −0.42 rad/s
  ② 超带 30 cm 力度更温和 ③ ω 被压到 0.45 且**走廊模式仍 0.65**）
- 单测量（2026-09-23）：`test_mpc_local_planner` 29 → **33 用例**；全包 **142 tests / 0 failures**
  （map_server 另有 10 / 0）。仿真复测**尚未跑**（用户此前说"不用测了"，且当时 gzserver 已停）；
  `test_drive_goal.py` 已加**顺滑度口径**（稳态 ω 反向频率 + RMS|ω|，**只报数不设硬门限**，
  阈值要跑几轮后标定再变断言）与 `[A3] 末朝向 ≤ 10°`


## 全局“净距偏好” + 最小可通宽度不变量（2026-09-23 用户要求）
用户规则：**机体宽 0.40 m + 左右各 10 cm ⇒ 最窄可通 0.60 m**。
这条规则 = 现有硬判据的全部预算（半宽 0.20 + `footprint.safe_margin` 0.05 +
`map_server.inflate` 0.05 = 0.30 = 各 10 cm）⇒ **硬层不能再加**（再加就把 0.60 m 通道判死）。
所以“让路径离障碍远一点”改用**软偏好**：

- 新参数 `planner.clearance_prefer_dist`（希望离障碍至少多远 [m]）+ 
  `planner.clearance_cost_weight`（低于它时每米额外代价权重），实现在
  `GlobalPlanner::cellExtraCost()`（**直接用 ClearanceField**，与地图里有没有“软格”无关：
  map_server 膨胀后全局图往往是全 0/100 ⇒ 原来的 `common.soft_cost_weight` 是空转的）
- `softCostAlong()` 的早退门槛也要跟着看偏好（否则**剪枝**会把路径又贴回墙边）
- 成本表：`planner.clearance_prefer_dist: 0.60`、`planner.clearance_cost_weight: 2.0`

实测（单测 `ClearancePreferencePrefersMiddleAndKeepsNarrowPassage`，0.4×0.4 柱子）：
关偏好贴角过 **0.275 m**；开偏好 **1.74 m**，路径只长 **0.06 m**；0.60 m 窄通道仍可通（软）

两个测量/设计坑（都踩过，写进注释与文档）：
1. **窄而浅的走廊测不出偏好**：车长 0.70（半长 0.40），离墙 0.30 时**转不了身**
   （任何朝向变化把 y 向尺度抬到 0.46 > 0.30），而栅格 A* 的**节点朝向 = 运动方向**
   ⇒ 只能直着走。要测就得给“能绕”的空间（柱子）。
2. **指标别算首末点**：首点=车自己（贴墙）、末点=目标 ⇒ 最小净距永远由它们决定，差异被掩盖。

局部侧：`local_mpc.obstacle_weight` 400 → **1000**（实测 +1.6 cm 且速度不变）；
`obstacle_safe_distance` **不动**（它同时是限速半径 `v *= clamp(d/safe,0.15,1)`：
0.45→2.0 只多 ~4 cm 净距却把 0.90 拖到 0.26 m/s）。

### 起点让步改成分级（同一天，第二次修）
第一版是“① 严格 → ② margin=0 → ③ −tol”三档。问题：**起点只是刚好贴到余量**
（实测差 1 mm）时，整条路都会按 **margin 0** 规划 ⇒ 一路贴墙走。
现在从大到小试 `{0.04,0.03,0.02,0.01,0.0}` 再试负数到 `−start_penetration_tol`，
取**第一个可行**的档位，日志点名“本次路径按余量 X 规划”。
（栅格 A* 没法“只放宽首点”，所以档位会作用于整条路 ⇒ 宁可取大档。）
- 另：`FootprintCollisionChecker::setMarginOverride()` 的复位必须用
  **独立方法 `clearMarginOverride()`**，不能用“负数当哨兵” ——
  负 margin 现在是合法取值（试过用 `m < 0` 判“没 override”，结果 override 永不生效）。
  诊断配套：`poseInCollisionAtMargin(x,y,yaw,m)`（不动内部状态）。


## ★★ 区域膨胀"算两遍" ⇒ 判据零余量 ⇒ 任务死锁（2026-09-23 实测，根因是我改的）
现象：`[sm] ← 局部反馈：MPC 求解失败：primal infeasible；★ 参考路径自身就在障碍硬距离内
（参考最小净距 0.242 < 0.250）` 每 100 ms 一条 → 1 s 后 BLOCKED → 恢复重规划 →
`规划失败：START_FOOTPRINT_COLLISION` → 任务 FAILED，车既不动也不结束。

现场量测（写了 `/tmp/pnc_zone_probe.py`：订 pose + 全局图 + occupancy_2d + ZoneArray，
算车到每张图/每个区域多边形的净距，并把周围 ±1.2 m 画成 ASCII 栅格图）：
- 车 (+0.99, +1.12)；全局图最近占据格中心 **0.238 m**（= 禁行区烧入的）
- 感知 `occupancy_2d` 最近占据 **3.6 m**（说明那"障碍"根本不是真障碍）
- **禁行区 Z1：车中心到多边形边界 0.283 m**；消息 inflate 0.05 ⇒ 局部融合值 0.233 < 0.25 ⇒ 死
- 全局判据 = 0.05(烧图) + 0.20(半宽) + 0.05(margin) = 0.30；局部 = 0.05 + 0.25 = 0.30
  ⇒ **两边完全相等 = 零余量**，差 1.7 cm 就两边都判不合格

根因链：`zones.inflate` 是**全局那层余量**（与真障碍的 `map_server.inflate` 对称：
全局多 0.05，局部按未膨胀几何判 = 5 cm 余量）。我上一轮把用户定的 `zones.inflate: 0.0`
改成 0.05，而**局部也按消息里的 inflate 融一遍** ⇒ 那 5 cm 余量被自己抵消掉。
（区域比真障碍脆：真障碍只在全局图上加 0.05，区域在"全局烧 + 局部融"两处都加。）

修法：
1. **`zones.local_inflate: 0.0`**（`config/pnc_2d.yaml`，新参数）：局部按原几何判，
   余量留给全局图 —— 与 `topics.local_map → occupancy_2d`（未膨胀）完全同构。
   局部节点起动时会 WARN：`zones.local_inflate == 消息 inflate` ⇒ 零余量必死锁
2. **起点擦余量可恢复**：`astar.relax_start_margin`（默认 on）+ `plan()/planImpl()` 拆分：
   起点含余量撞、但**真实轮廓（safe_margin=0）没撞** ⇒ 临时
   `FootprintCollisionChecker::setMarginOverride(0)` 整条路径改用真实轮廓规划一次，
   成功则在 message 里点名警告（节点会把它打出来）；真撞仍 `kStartFootprintCollision`。
   ★ 为什么必须"整条路径换轮廓"而不是只放起点：栅格 A* 的**节点朝向 = 运动方向**，
   起点一旦在余量带里，朝任何方向的下一步都同样在带里（侧行保持同距；转向要车长
   0.35 的净距 > 现有净距）⇒ 严格判据下必然 NO_PATH。
   新增 API：`poseInCollisionNoMargin()`、`distanceToLethal()`、`setMarginOverride()`
   （override 生效时禁用按方向预计算的偏移表）
3. 不变量（写进 doc/README）：**局部判据 + 0 ≤ 全局判据 − 膨胀层，且膨胀层必须非零**
   （5 cm ≈ 2 格，用来吸收"全局图 vs 感知图"的差 + ±半格离散化）。同一个跨层物理量
   两边配成同一个值 ⇒ 一定会在"两边恰好都差一点"时死锁（`brakeAcc`/`stopCoast`/
   `corridorTolerance`/`goalYawTolerance`/区域膨胀都是这一类）。
- 单测：`FootprintCollisionCheckerTest.NoMarginVariantDistinguishesGrazeFromHit`、
  `AStarPlanner.StartTouchingMarginStillPlansButRealCollisionFails`（3 分支：擦余量成功+
  点名 / 真撞报错 / 关松弛回严格）；全包 **144 tests / 0 failures**
- ⚠ 用例坑：`mkPose(x,y,yaw_deg)` 收的是**度**（传 `M_PI/2` 会得到 0.027 rad，
  于是长轴对着墙，"擦余量"场景构造不出来）
