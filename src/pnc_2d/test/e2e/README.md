# pnc_2d 端到端测试（ROS 图级别）

这里跑的是**真节点 + 真 DDS** 的 E2E，验证那些纯 C++ 单测覆盖不到的东西：
latched 话题的"路径有效期"、清场时机、换算法重启、运行时热切换。

| 脚本 | 覆盖 |
|---|---|
| `test_clear_semantics.py` | 失败→清空+NO_PATH 状态、成功→路径+SUCCESS 状态、到达目标（按参数开关）、`~/clear_path` 服务 |
| `test_restart_cleanup.py` | 杀掉路网节点→用 A* 重启：旧路径/黄色通路高亮**必须被清掉**（跨进程残留） |
| `test_hot_switch.py` | `ros2 param set planner.type` 真热切换、非法类型被拒、`~/switch_planner`、`~/reload_params` |
| `run_all.sh` | 上面三个 + E2E-1 的两种参数模式，一把跑完 |

## 怎么跑

```bash
cd /home/gmd/r41_ws
colcon build --packages-select pnc_2d      # 需要先构建（脚本直接跑 install 里的可执行文件）
bash src/pnc_2d/test/e2e/run_all.sh        # 全部
# 或单个（自己设 DDS 环境）
export CYCLONEDDS_URI=$PWD/src/bringup/cyclonedds.xml ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
python3 src/pnc_2d/test/e2e/test_hot_switch.py
```

要点：

- **不需要 `map_server` 在跑**：脚本自己从站点目录的 `map.pgm`/`map.yaml` 造 `OccupancyGrid`
  并以 latched 发到 `/t/map`，语义与 map_server 一致（未知格按 `unknown_as_free` 当空闲）。
- **不干扰正在跑的系统**：所有话题都重映射到 `/t/*`，节点名固定 `gp_e2e`，跑完自己收掉。
- 需要 **`python3` 里有 `rclpy`**（即 source 过 ROS 环境），所以用系统的 `python3`，不是仓库的 `.venv`。
- 站点/路网可用环境变量覆盖：`PNC2D_MAP_DIR`、`PNC2D_ROUTES`、`PNC2D_WS`。
- 节点日志写在 `/tmp/pnc2d_e2e_<pid>.log`；失败时先看它。

这些**不进 `colcon test`**（会起节点进程、依赖 DDS 环境，不适合放进常规单测）；
纯算法单测仍在 `test/*.cpp`（`colcon test --packages-select pnc_2d`）。

## 写这类测试踩过的两个坑（都已在公共模块里处理）

1. **别用 `ros2 run` 起被测节点**：它是个 wrapper，`Popen.terminate()` 只杀 wrapper，
   真正的节点会变成**孤儿进程**继续跑。多个实例抢同一话题 → 出现"重复消息"、
   "别人发的空路径把路径清了"这类灵异现象。
   `e2e_common.start_node()` 直接 exec `install/pnc_2d/lib/pnc_2d/global_planner_node`，
   并用 `start_new_session=True` + `killpg` 整组回收。
2. **判据要扫"整窗消息"，不能只看最后一条**：节点一次操作会发多条 `MarkerArray`
   （清场 DELETE、路网、起终点…），只看最后一条会把状态判反。
   `Harness.saw_delete()/saw_add()` 就是干这个的。
