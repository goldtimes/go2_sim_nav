# r41_ws C++ clangd 满屏红排错

- **症状**：VS Code 里 .cpp 满屏红，报 `'cmath' file not found`、`No type named 'string' in namespace 'std'` 等标准库错误。
- **根因**：`build/compile_commands.json` 的编译器是 `/usr/bin/c++` 和 `/usr/bin/cc`（符号链接 → g++-11/gcc-11），但 clangd 的 `--query-driver` 只放了 `/usr/bin/g++` → clangd 无法探测 GCC 系统头路径。
- **修复**：`.vscode/settings.json` 的 clangd.arguments 里 query-driver 改为：
  `--query-driver=/usr/bin/g++,/usr/bin/c++,/usr/bin/gcc,/usr/bin/cc`
- 改完执行 VS Code 命令 `clangd.restart` 生效。
- **环境事实**：vscode-clangd 0.6.0 自动下载 clangd 22.1.6 到 `~/.config/Code/User/globalStorage/llvm-vs-code-extensions.vscode-clangd/install/22.1.6/`（系统 apt 只有 clangd 14，勿用）。C/C++ (cpptools) 的 intelliSense 已禁用。
- **次要**：lightning 包 `src/slam/lightning-lm/package.xml` 需保持 `<test_depend>` 在 `<member_of_group>` 之前，否则 XML 校验红。
