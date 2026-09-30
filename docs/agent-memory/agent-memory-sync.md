# 记忆本身的移植（2026-09-30 建立）

## 事实：记忆不在 git 里
- 仓库记忆实际位置：`~/.config/Code/User/workspaceStorage/<工作区URI的哈希>/`
  `GitHub.copilot-chat/memory-tool/memories/repo/`
- **哈希由工作区 URI 决定** ⇒ 换机器 / 换 clone 路径 / **换 SSH 主机名**打开同一仓库
  = 新哈希 = **记忆为空**。（实测本机有 5 个指向 `…/home/jhc/r41_ws` 的不同
  `vscode-remote://ssh-remote+<host>` 条目，记忆目录全空。）
- 存储形态：**纯文件目录**（`memory-tool/` 下除 `memories/` 无索引/DB）⇒ 移植 = 拷 .md。
- 用户级记忆在 `~/.config/Code/User/globalStorage/github.copilot-chat/memory-tool/memories/`
  （跨工作区；按用户决定**不纳入**同步）。

## 本仓库的机制
- 主本：**`docs/agent-memory/*.md`**（入库 git）。`README.md` 是说明，不是记忆（脚本跳过）。
- 脚本：**`scripts/agent_memory_sync.sh`** `export` / `import` / `status`
  （`--dry-run` / `--storage` / `--prune` / `--force-import`）：
  - 自动发现 storage 目录：匹配 `workspaceStorage/*/workspace.json` 的 `folder`
    URI **路径部分与本仓库根完全一致**；多条命中优先 `file://`，再比 mtime。
  - `import` 会先备份本机记忆到 `…/repo.bak-<时间戳>/`；默认不删对端多余文件。
  - 已实测：export/import 双向、幂等（全 SAME）、NEW/UPDATE 检测、dry-run 不写。
- 闭环入口：**`.github/copilot-instructions.md`**（agent 自动读取）第 1 节写明
  "发现 /memories/repo/ 为空而 docs/agent-memory/ 有 20 篇 ⇒ 先 import"。

## 踩过的坑
- ★ **`IFS=$'\t' read` 会合并连续分隔符**（tab 是 IFS 空白字符）⇒ `file:///x`
  这种 netloc 为空的 URI 字段错位（host 拿到路径、path 拿到空串）⇒ 永远匹配不上。
  改用非空白分隔符（`\x1f`）。**别再改回 tab。**
- 用 `bash -x` 才定位到的；只有单一分隔符的用例（远程 URI）看不出来。
- README.md 必须排除，否则会被当成一条记忆拷进 agent 目录。

## 工作流提醒
- 改代码导致"已确认的事实"变化时，**同步更新对应记忆条目**并一起提交，否则下次读到过期结论。
