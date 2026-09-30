# docs/agent-memory —— Copilot 仓库记忆的主本

这里是 **Copilot agent 的「仓库记忆」（repo memory）在版本控制里的唯一主本**。
换机器 / 换 clone / 换 SSH 主机名打开本仓库时，跑一条命令就能把上下文恢复，
不必让 agent 重新踩一遍老坑。

> ⚠ `README.md`（本文件）不是记忆，是给人看的说明。同步脚本会**跳过**它。
> 不要把它改名成别的 `.md` 名字，也不要往里写"记忆"内容。

## 为什么要这么绕

Copilot 的仓库记忆**不在 git 里**，而是存在本机 VS Code 的私有目录：

```
<VS Code User>/workspaceStorage/<工作区 URI 的哈希>/GitHub.copilot-chat/memory-tool/memories/repo/
例：~/.config/Code/User/workspaceStorage/783cf6e0917c61177884ec38d4afee91/.../memories/repo/
```

那个哈希由**工作区 URI** 决定，于是：

| 情况 | 结果 |
|---|---|
| 同一台机器、同一个路径 | 命中同一份记忆 ✅ |
| 换了仓库路径（如把 ws 从 `~/r41_ws` 挪到 `~/work/r41_ws`） | 新哈希 ⇒ **记忆为空** ❌ |
| 换一台机器打开同一个仓库 | 新哈希 ⇒ **记忆为空** ❌ |
| 同一个仓库用**不同 SSH 主机名**打开 | 新哈希 ⇒ **记忆为空** ❌ |

最后一条最容易中招：本机实测有 **5 个** 指向 `…/home/jhc/r41_ws` 的不同
`vscode-remote://ssh-remote+<主机>` 条目，记忆目录全是空的。

好在记忆本身**只是一堆 `.md` 纯文本**（没有索引、没有数据库 —— 已确认
`memory-tool/` 下除 `memories/` 外没有别的文件），所以"移植"就等于拷文件。

## 用法

```bash
# ① 源机器：把本机记忆导出成仓库里的主本，复核后提交
scripts/agent_memory_sync.sh export
git add docs/agent-memory && git diff --cached --stat && git commit -m "记忆同步"

# ② 新机器：clone 之后导进去（会先自动备份本机已有的记忆目录）
scripts/agent_memory_sync.sh import
#    然后【新开一个会话】再用 —— 当前会话可能已经把旧记忆读进上下文了

# ③ 随时看差异，不写任何东西
scripts/agent_memory_sync.sh status
```

参数：

| 参数 | 作用 |
|---|---|
| `--dry-run` | 只打印要做什么，不写 |
| `--storage <哈希\|路径>` | 手工指定工作区 storage 目录（自动发现有多条候选时会提示） |
| `--prune` | 删除对端多余的文件（默认**不删**，怕误删只在某台机器上有的笔记） |
| `--force-import` | 导入时不先备份本机记忆目录 |

自动发现规则：在所有 `workspaceStorage/*/workspace.json` 里找 **`folder` 的路径部分
与本仓库根完全一致** 的条目；多条命中时优先 `file://`（本机直接打开），再比"最近使用"。
找不到会明确报错（通常是：VS Code 还没用这个路径打开过该目录）。

## 已知边界

- **只用 `.md`**：记忆文件都是 Markdown，脚本按 `*.md` 同步，`README.md` 除外。
- **不做三方合并**：同名文件直接覆盖（打印 `UPDATE`）。`export` 方向靠
  `git diff` 复核；`import` 方向靠自动备份 `…/memories/repo.bak-<时间戳>/` 回滚。
  跨机器同时改同一篇时，请先 `status` 看差异再决定。
- **文件里的绝对路径不会自动改**：这些笔记里写了 `/home/gmd/r41_ws/...` 之类的路径，
  换机器后路径可能不同 —— 那是**上下文信息**，不是需要机械替换的东西，按需核对即可。
- **用户级记忆不在这**：`~/.config/Code/User/globalStorage/github.copilot-chat/memory-tool/memories/`
  那份是跨工作区共享的个人偏好，按约定不纳入本同步（各机器自己长）。
- **会话历史（chronicle）是另一套**：`session_store_sql` 那套有独立的索引/同步机制，
  和这里的记忆文件无关。

## 收录范围

这些是**已验证的事实**，主要围绕本仓库的技术栈（ROS 2 Humble + CycloneDDS、
lightning LIO、perception 2D 栅格、pnc_2d 规划、map_server、Gazebo 仿真）：

| 文件 | 内容 |
|---|---|
| `map-server.md` | map_server：加载器定位、latched/按需补发、3D PCD、地图语义 |
| `perception-2d-layer.md` | 2D 层数据来源、增量构建、decay、动态障碍残留根因、地面/高度带关系 |
| `perception-extraction.md` | 从 plan_env 抽包的改动清单 |
| `pnc2d-*.md` | 规划栈：MINCO、路网、rolling_replan、ABI 重建、参数坑、追踪脚本 |
| `lightning-*.md` | LIO：线程、外参、地图路径、初始化重定位、性能 |
| `go2-*.md` / `my-bot-gazebo-sim.md` | 仿真平台、尺寸与坐标系、Gazebo 配置 |
| `clangd-cpp-red-squiggles.md` | 编辑器误报（clangd 找不到 ROS 头）的处理 |

⚠ 这些笔记的**时效性由提交历史保证**：改代码时若结论变了，请同时更新对应
记忆条目并一起提交，否则下次会读到过期的"事实"。
