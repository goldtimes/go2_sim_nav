# pnc_2d 仿真测试（在跑着的仿真栈上跑）

跟 `test/e2e/`（自带地图、不碰真实话题）不同，这里是**真仿真**：gazebo + `run_loc_online`
（lightning 定位）+ perception + map_server 全都已经在跑，本目录的脚本只起 `pnc_2d`
三节点、给目标、量指标。

```bash
cd /home/gmd/r41_ws
export CYCLONEDDS_URI=$PWD/src/bringup/cyclonedds.xml ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash

python3 src/pnc_2d/test/sim/test_drive_goal.py          # 自由空间直线（astar + mpc）
python3 src/pnc_2d/test/sim/test_route_lane.py          # 严格贴线（route_network + mpc）
```

| 脚本 | 覆盖 | 实测（2026-09-22） |
|---|---|---|
| `test_drive_goal.py` | 到点误差、末速、横向误差（全程/稳态）、无碰撞、指令不越界、被控对象保真度 | 6/6：到点 0.017~0.043 m、末速 0.012~0.013 m/s、横向 0.016~0.026（稳态）/ 0.051~0.058（全程）m、保真度 0.93~0.96 |
| `test_route_lane.py` | 严格贴线（`corridor_width=0`）、`route_mode` 是否真的传到局部、到点、无碰撞 | 7/7：贴线 0.048（全程）/ 0.025（稳态）m、到点 0.017 m |

公共部分（起停、采样、指标、坑）在 `sim_common.py`。

⚠ **到点误差要可信，`local.goal_tolerance` 必须严格小于验收阈值**（`go2_run.yaml`
里设 0.08）。踩过：容差也设 0.15 ⇒ 管理器在 0.15 m 处就停机，量到的是它自己的
触发条件（同义反复）。横向误差同理：实测值要与**验收脚本独立算出**的几何量对。

## 设计要点

- **指标一律用高频位姿算**（`sim_common.Probe.traj`），不用 `local_status`、更不用
  `twist`：
  · `twist` 低速时是噪声（实测 −0.23~+0.45 乱跳、均值都偏低）；
  · `local_status` 在任务结束后就不再发，采样时机不可控；
  · latched 话题会给**上一轮任务**的旧数据。
- **净距/选目标用发布的全局图**（`/global_map/occupancy`），不用裸 PGM —— 前者含
  烧进去的禁行区，后者没有。
- **测试自己搜目标**：仿真里的车会停在上一轮结束的地方（可能贴着墙/禁行区），
  `起点+dist` 很可能落在障碍里 ⇒ A* 直接失败。`goal_candidates()` 会沿多个方向找一条
  整条直线净距都够的走法（并对车前 ±75° 内排序），**几何最空的候选未必全局可达**
  （大障碍另一侧/目标处 footprint 摆不下）⇒ 脚本拿到 `FAILED` 就**换下一个候选**，
  不把全局规划器的可达性问题记成局部控制器的失败。
- **`test_route_lane.py` 自带临时路网**：站点通道是 `corridor_width: 0.60`，验不出
  "严格贴线"。所以它读到位姿后**沿车头方向生成一条 6 m 直通道**（半宽 0），写到临时
  文件，再**运行时热切换** `~/switch_planner` 成 `route_network`。好处：不需要用户先
  把车开到站点通道旁，且几何完全已知。
- **测试要自诊断"起点被挡"**：车停在障碍/禁行区边界里时任何目标都规划不出来，
  测试若只报"没有局部状态"极易误判成控制器坏了。

## 诊断（出问题先看这个）

局部节点每 1 Hz 打一行 MPC 内部量（`LocalPlanner::diagString()`，不需要改代码就能看）：

```
[local] cmd v=0.266 w=-0.184 | mpc v_ref=0.350 v_now=0.206 e_v0=-0.144 v_up=0.350
        curv=0.000 | e_yaw0=-0.4° lat0=0.02 | cross=+0.024 prog=0.05 |
        走廊行15 障碍行0 最小距1.34 | solved 325迭代 0.9ms
```

看到 `cmd v≈0` 时按这个顺序查：
1. `e_v0`：速度反馈对不对（`v_now` 与实测速度是否一致）；
2. `e_yaw0` / `lat0`：参考的朝向/横向是否合理（**朝向差 ~90° ⇒ 参考切线算错了**，
   见 `rebuildReference` 的 `min_span` 注释）；
3. `curv`：直线上应该是 0（非 0 ⇒ 同样指向参考构造）；
4. `走廊行/障碍行`：哪些硬约束在生效；
5. `solver_status`：`primal infeasible` ⇒ 有硬约束在初值处就不可行（可达性问题）。
