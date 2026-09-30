# my_bot Gazebo(Classic) 仿真 —— 踩坑与约定（2026-09-10）

## 包与启动
- 路径：`src/bringup/mobile-3d-lidar-sim/my_bot`；**包名 = `my_bot`**。
- 构建：`colcon build --packages-select my_bot`（CMakeLists 已 install `config description launch worlds`）。
- 启动：`ros2 launch my_bot launch_sim.launch.py [world:=<绝对路径>]`，默认 small_house。
- 该仿真走 **Gazebo Classic**（`gazebo_ros` 的 `gazebo.launch.py` + `spawn_entity.py`，`/usr/bin/gazebo`）。
  world 是 Classic 格式（`<sdf version='1.6'>` + `gzclient_camera`）；**勿与 `ros_gz_sim`(Ignition) 混用**。

## 坑 1：launch 未声明 world 参数（已修）
- 原 `launch_sim.launch.py` 没有 `DeclareLaunchArgument('world')` 也未透传给 gazebo → `world:=...` 被忽略、只加载默认空世界。
- 已修：声明 world（默认 `share/my_bot/worlds/small_house/small_house.world`，未安装时回退源码路径）并 `launch_arguments={'world': LaunchConfiguration('world')}` 透传。

## 坑 2：模型 mesh 用相对路径 `file://models/`（已修）
- small_house 的 68 个 `model.sdf`（136 处）原写 `<uri>file://models/<model>/meshes/*.DAE</uri>` —— `file://` 按 **Gazebo 进程 CWD** 解析，从 `~/r41_ws` 启动就找不到 → visual/mesh 全丢，Gazebo 里只剩绿色线框（collision/failed-mesh 显示）。
- 原设计需 `cd .../small_house` 再启动（方案 C 可复现）。
- 已修：`sed -i 's#file://models/#model://#g' models/*/model.sdf` → `model://<model>/meshes/*.DAE`，由 `GAZEBO_MODEL_PATH` 解析，与 CWD 无关。

## 坑 3：GAZEBO_MODEL_PATH 未包含本地模型库（已修）
- launch 里新增 `SetEnvironmentVariable('GAZEBO_MODEL_PATH', <small_house>/models + ':' + 原值)`，且**必须声明在 include gazebo 之前**（当前顺序：world_arg, set_model_path, rsp, gazebo, spawn_entity）。

## 排查命令速查
- 模型加载失败：`grep -iE "unable to find uri|error|fail" ~/.ros/log/gzserver_*.log`
- spawn 结果：`grep -i spawn ~/.ros/log/python3_*.log`（应见 `Successfully spawned entity [my_bot]`）
- 重启前清理：`pkill -9 -f gzserver; pkill -9 -f gzclient`
- 检查残留相对路径：`grep -rn "file://models/" <pkg>/worlds/ | wc -l`
