# ROS 2 参数机制坑（pnc_2d 实测，2026-09-21）

## 核心结论
- 只开 `NodeOptions().allow_undeclared_parameters(true)` **不够**：
  `get_parameter_or()` 对**未声明**参数不会查覆盖项，静默返回代码默认值
  → yaml/`-p` 里所有"节点不认识、由库读取"的键（如 `footprint.*`/`common.*`/`astar.*`）
  全部失效，且无任何报错。极难发现。
- 正解：`.automatically_declare_parameters_from_overrides(true)`（可与 allow_undeclared 同时开）。
- ⚠ 二者同时开启后，yaml 里已有的键**已被声明**，再调
  `declare_parameter(key, default)` 会抛 `ParameterAlreadyDeclaredException`（节点崩）。
  要写 `if (!has_parameter(k)) declare_parameter(...); else read existing`。
- 读参数做**宽松类型转换**：rclcpp Parameter 强类型，yaml 写 `5`（int）而代码读
  double 会抛 ParameterTypeException。用 `has_parameter` + `get_type()` 分支转换。
- 排查手段：启动时打印 `get_node_parameters_interface()->get_parameter_overrides()`
  的键列表（"yaml/命令行参数 N 项：..."），一眼看出配置是否被吃到。

## launch 参数覆盖（2026-09-29 实测，很坑）
- `pnc_2d.launch.py` **总是**传 `{'local.type': local_type}`，而 `local_type` 的
  `default_value='none'` ⇒ **命令行不给 `local_type:=…` 就等于 none**（空实现，
  节点自己会 WARN「不会输出 /pnc_2d/cmd_vel」）⇒ 车完全不动。
  所以 `go2_run.yaml` 里写 `local.type: heading_shim` **救不了**：launch 的显式值赢。
- 配套坑：纯 `local_type:=mpc` 时 `LocalPlanner::goalYawTolerance()` **恒为 0**
  ⇒ 节点**不判到点朝向**、也不做原地对正（"对正目标朝向"整个活在 heading_shim 里）
  ⇒ 用户 2026-09-29 报"终点时朝向没对准"就是这个配置问题（日志里写得很清楚：
  到点诊断 `朝向：不判（容差为 0）`）。要一次性看清：`ros2 param get /local_planner
  local.type` + `shim.goal_yaw_tolerance_deg`。
- 验收脚本默认用 `heading_shim`（`PNC2D_LOCAL_TYPE` 可切）⇒ 会出现“验收全过、
  仿真不对准”的现象，不要把两者混为一谈。

## QoS 坑（同一个节点上实测）
- 定位 `/lightning/perception/pose`（节点 run_loc_online）以 **BEST_EFFORT** 发布。
  用默认 RELIABLE 订阅 → DDS 不匹配 → **静默收不到任何数据**，无报错。
  正解：`rclcpp::SensorDataQoS()`（best_effort + keep_last5 + volatile）。
  `/global_map/occupancy` 是 transient_local（要和 map_server 对齐）。
- ⚠ `ros2 topic echo` / `ros2 topic hz` 会**自动把 QoS 降到 best_effort**
  （ros2topic/verb/echo.py `choose_qos`，只要发布端不是全 RELIABLE 就降级）
  → "CLI 能看到数据" **不能**证明"自己节点能收到"。核对用 `ros2 topic info -v`。

## 本仓库相关环境
- CYCLONEDDS_URI=/home/gmd/r41_ws/src/bringup/cyclonedds.xml ROS_DOMAIN_ID=30
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp，然后 source install/setup.bash
- `ros2 launch ... | grep` 会吞掉输出（缓冲）→ 先重定向到文件再 grep。
- 数据分析用 /home/gmd/r41_ws/.venv/bin/python3（有 numpy/scipy/matplotlib；无 pyyaml）。
