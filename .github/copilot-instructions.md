# 本仓库的 AI agent 协作约定

## 1. 记忆（memory）在仓库里有主本 —— 换机器先导入

Copilot 的「仓库记忆」**不在 git 里**，而存在本机
`<VS Code User>/workspaceStorage/<工作区 URI 的哈希>/GitHub.copilot-chat/memory-tool/memories/repo/`。
那个哈希由**工作区 URI** 决定，所以**换机器 / 换 clone / 换 SSH 主机名打开本仓库，记忆都是空的**。

本仓库把主本放在 **`docs/agent-memory/`**（跟着 git 走）。

> **★ 如果你（agent）发现 `/memories/repo/` 是空的、或只有很少几条，而本仓库
> `docs/agent-memory/` 里有 20 篇左右的笔记 —— 说明这是新机器/新入口。**
> 请先提示用户执行一次导入，再开始任务；导入后建议**新开会话**（当前会话可能已经把
> 空记忆读进上下文了）：
>
> ```bash
> scripts/agent_memory_sync.sh import     # 主本 → 本机记忆（会先自动备份本机已有记忆）
> scripts/agent_memory_sync.sh status     # 只看差异
> ```
>
> 在**源机器**上新增/修改了记忆后，收工前把它导回仓库并提交：
>
> ```bash
> scripts/agent_memory_sync.sh export     # 本机记忆 → docs/agent-memory/（复核 git diff 后提交）
> ```
>
> 细节见 `docs/agent-memory/README.md`。用户级记忆（`/memories/`，跨工作区）**不参与**
> 本同步，各机器自己长。

**维护要求**：改了代码导致某个"已确认的事实"变化时，请**同时更新对应的记忆条目**
并一起提交；否则下次会读到过期结论，比没有记忆更糟。

## 2. 环境（每次开终端都要）

```bash
export CYCLONEDDS_URI=/home/gmd/r41_ws/src/bringup/cyclonedds.xml \
       ROS_DOMAIN_ID=30 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/gmd/r41_ws/install/setup.bash
```

- Python 一律用 `/usr/bin/python3`（仓库 `.venv` 是 3.14，跑不了 Humble 的 rclpy）。
- 改配置后必须 `colcon build --packages-select <包>`：`install/` **只拷不删**，
  不重编就会跑到旧副本，很容易把调试带偏。
- ⚠ **不要 `pkill -f parameter_bridge`**：会连带杀掉仿真自己的桥（`/clock`、odom、cmd_vel）。

## 3. 仿真与验证

```bash
tmuxp load -d /home/gmd/r41_ws/go2_sim_bringup.json    # 会话名 mobilebase
```

改完 C++/Python 先跑单测（当前基线：`pnc_2d` 18/18、222 个 gtest 用例），
涉及避障行为的改动要在仿真里按 `src/pnc_2d/test/sim/README.md` 的脚本实测，
**不要只靠"代码看起来对"**。

## 4. 排错习惯（都是踩过的）

- `ros2 topic echo --once` 要留 **≥10 s** 超时（它自己启动就要 1–3 s，3 s 会给假空值）。
- `ros2 topic echo` 的 `--qos-*` **只能整组给**：单独给 `--qos-reliability reliable`
  会造出与发布端不匹配的 profile，**静默收不到**（误判成"发布失败"）。
- 判据/参数分散在多层（launch vs yaml、map_server vs perception vs pnc_2d）时，
  先确认**哪一层真正生效**：`ros2 launch` 的实参**优先于** yaml。
- 结论要有**外部可观测量**支撑（话题内容、日志计数、探针脚本），
  不要用"合成数据跑通了"当作正确性证据。
