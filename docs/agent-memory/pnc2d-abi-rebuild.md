# ★ 改了 pnc_2d 头文件 ⇒ 必须重建 **map_server**（否则段错误）

## 现象（2026-09-24 真实踩到）
`map_server_node` 启动 **0.1 s 后 exit code -11（SIGSEGV）**，而且日志文件**完全是空的**
（一行输出都没有）⇒ 死得极早、看不到任何错误。`ros2 launch` 只报：
`process has died [pid ..., exit code -11, cmd '.../map_server_node --ros-args -r __node:=map_server --params-file .../map_server.yaml ...']`

## 根因（头/库不一致，我的操作错误）
- `map_server_node.cpp` 里 **在栈上按值构造** `pnc_2d::FootprintCollisionChecker`
  （`grep -n "FootprintCollisionChecker" src/map_server/src/map_server_node.cpp` ⇒ 1 处）
- 而 `pnc_2d` 的头文件里**对象布局变过**（`FootprintCollisionChecker` 加了
  `margin_overridden_`/`margin_override_`；`ClearanceField` 加了 origin/yaw 等 5 个成员）
- 我全程用 `colcon build --packages-select pnc_2d`（只建自己）⇒ `map_server_node`
  还是旧二进制（`stat` 比对：binary 09-23 17:09 vs `libpnc_2d_planner.so` 09-24 10:35）
- ⇒ 栈上按**旧尺寸**分配、库里构造函数按**新布局**写入 ⇒ 越界写 ⇒ SIGSEGV
  （崩溃发生在**加载地图**那条路径上，所以"不带地图启动正常、一加载就死"）

## 修法与纪律
- 修：`colcon build --packages-above pnc_2d`（或整仓 `colcon build`）
  ⇒ **实测**：重建后 map_server 能完整加载 607×307 地图 + 路网 10 节点 + 区域层 3 个
- ★ **纪律**：只要动了 `src/pnc_2d/include/**` 里**类/结构体的成员**（不只是加方法），
  就必须重建所有依赖 pnc_2d 的包。目前依赖方**只有 map_server**
  （`grep -rl "pnc_2d" --include=package.xml src/`）。
  为省时间用 `--packages-select pnc_2d` 时，**改头文件后一定要补一次**
  `--packages-above pnc_2d`。
- 诊断套路（可复用）：节点"秒死且日志为空" ⇒ 先看 `~/.ros/log/.../launch.log` 拿退出码，
  **exit code -11 = 段错误**，然后 `stat` 比对该二进制与 `lib*.so` 的时间戳。


