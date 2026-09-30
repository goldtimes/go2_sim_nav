# pnc_2d 局部规划器：blocked-wait / clear_map / rolling_replan（2026-09 完成）

## 文档（已写完，改代码时同步更新）
- **主文档**：`src/pnc_2d/doc/local_flexible_avoidance_plan.md`（绕/等/清三层、参数表、跨层耦合表、S1 实测、调参手册、文件清单）。
- `src/pnc_2d/README.md`：§1 内"局部避障与恢复"小节 + §5 单测清单（**18 可执行 / 222 用例**）+ §5③ 加 `test_avoidance.py` + §6 边界。
- `src/perception/doc/perception_gridmap.md`：§5.1 清图服务。
- `src/pnc_2d/test/sim/README.md`：`test_avoidance.py` 行（rolling_replan 基线②）。
- `src/pnc_2d/doc/pnc2d_restructure_plan.md`：P5.3/P5.4 ✅、P6 ✅。

## 构建与运行（易错点）
- 构建：`colcon build --packages-select pnc_2d --cmake-args -DCMAKE_BUILD_TYPE=Release`。
  改了 `config/*.yaml` 或 `perception` 的 yaml/src 后**必须重编对应包**，否则 install/ 是旧的。
- 单测：`ctest --test-dir build/pnc_2d`（改 rolling_replan 后 18/18 通过）。
- **launch 必须显式 `local_type:=heading_shim`（或 `rolling_replan`）**：launch 默认 `none`
  → NullLocalPlanner → **完全不发 cmd_vel**，表现为"不发车"而不是报错。
- 运行前：`export CYCLONEDDS_URI=/home/gmd/r41_ws/src/bringup/cyclonedds.xml
  ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && source install/setup.bash`。
  Python 用 `/usr/bin/python3`（仓库 .venv 是 3.14，与 Humble rclpy 不兼容）。
- 残留节点清理：`pkill -f 'pnc_2d/(pnc_manager|global_planner|local_planner)_node'`。
  ⚠ 永远别 `pkill -f parameter_bridge`（会杀掉 sim 自己的网桥）。

## rolling_replan（`local/rolling_replan_shim.{hpp,cpp}`，type=rolling_replan）
装饰器：包住 primary（默认 `heading_shim`），沿参考路径前瞻发现冲突就**滚动地**用 A* 修补一段再交回 primary。
- 参数分组：前瞻 `lookahead_m:1.60` / 触发 `trigger_clearance_m:0.45` / 硬界 `blocked_clearance_m:0.25` /
  `path_margin_m:0.06` / 窗口 `back_m:0.40 roll_m:2.00 max_roll_m:4.00 max_offset_m:1.20` /
  节流 `min_interval_s:0.50 min_progress_m:0.20 max_consecutive_fail:2 give_up_s:3.00`。
- **★ 核心教训：触发阈值必须比硬下界早得多。**
  最初 `trigger=0.25`（= MPC `obstacle_hard_distance`）→ 实测"**贴上才绕**"：车心净距 0.151 m、
  轮廓穿透 **10.3 cm**。改 `trigger=0.45`（= MPC `obstacle_safe_distance`，MPC 开始付软代价的量级）
  + `path_margin_m=0.06` 后：S1 穿透 **0.0 cm** / 最小净距 0.274 m。**绕行要在"还有地方挪"时开始。**
- 判碰用 `CollisionChecker::poseInCollisionAtMargin(x,y,yaw,margin)`：**margin 直接替换
  `footprint.safe_margin`**（0 = 真实轮廓，负数 = 缩小）。修补段额外余量 = `safe_margin + path_margin_m`
  （跟踪有 2~4 cm 横向误差，"恰好不碰"的路径跟偏就擦上）。
- 跨层阈值一致性：启动时 WARN `trigger_clearance_m` vs `local_mpc.obstacle_safe_distance`、
  `blocked_clearance_m` vs `obstacle_hard_distance`。**不一致 = "局部说没挡、MPC 说 primal infeasible"**。
- 合并不变量：`plan_[0..i_a] + seg + plan_[i_b..]`，且强制 `merged.back() = original_.back()`（终点不动）。
- 走廊模式（corridor）不绕行，只打印一次 stderr 提示。

## 坐标：`/lightning/perception/pose` 是 **lidar** 位姿（不是 base_link）
`loc_system.cc:691/698` 发布 `child_frame_id=lidar_link`；lidar = base_link + (0.22, 0, 0.09)。
所有消费端原来都当 base 中心用 ⇒ 有 0.22 m 系统性前偏。修法（**不动 lightning**）：
`pnc_2d.yaml` 里 `pose.base_offset_x: 0.22`（3 个节点）+ 轮廓改非对称（长 0.78 / 宽 0.40 / 偏移 −0.038）。
真实尺寸 0.739×0.324×0.421，x∈[−0.407,+0.332]；URDF `collision` 盒比 visual 小，**仿真判碰别用 collision 盒**。

## 其它已落地
- 感知端 `std_srvs/Trigger` 清图服务（`grid_map/clear_map` → `grid_map->resetBuffer()`）；
  pnc_2d 恢复行为 `clear_map` 调它（`sm.perception_clear_service`）。用于清"幽灵障碍"。
  实测：开阔地感知自己 ~1 s 就清；幽灵只出现在**没有射线穿过**的位置（被挪动/瞬移物体、物体移出 FOV）。
- blocked-wait：`local.blocked_wait_s: 8.0` / `blocked_wait_global_s: 1.0` / `sm.stuck_timeout: 25.0`。

## 仿真测试脚本坑（`src/pnc_2d/test/sim/`）
- 负值 CLI 参数必须写 `--at=-0.5,1.2`（argparse 会把 `-0.5` 当选项）。
- S1/S2 障碍速度用 `--speed 0.3`（0.8 太快，来不及反应）。
- 路径话题是 latched ⇒ 必须过滤上一场景的旧路径，否则判"到达"会假阳。
- 注入器被 SIGTERM 杀掉会**留下盒子**，测试结束要自检+清理。
- 验收指标看 **穿透**（footprint penetration）而不是只看"到达"——10 cm 穿透也能"到达"。
