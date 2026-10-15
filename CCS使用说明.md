# 24ht 编译、烧录

在 VS Code 中打开克隆后的仓库根目录。底部状态栏提供：

- **CCS 编译**：用本机 CCS 20.5 构建整个工程，也可以按 `Ctrl+Shift+B`。整个目录已注册到 CCS 工作区。
- **CCS 烧录**：检测 J-Link，编译成功后通过 CCS 的 DSLite 烧录、校验、系统复位并运行。
- **打开 CCS**：在独立 Code Composer Studio 中打开整个目录，使用原生工程和调试界面。

状态栏按钮由已安装的 Tasks 扩展提供；这是调用本机 CCS 的任务入口。当前脚本默认从 `C:\ti\ccs2050\ccs` 查找 CCS，如安装位置不同，请修改 `.vscode/ccs.ps1` 中的 `ccsRoot`。若按钮尚未刷新，运行命令 `Developer: Reload Window`。也可在菜单“终端 → 运行任务”选择上述 CCS 任务。

烧录前连接一个 J-Link，并给 MSPM0G3507 板供电。当前工程的目标配置为 `targetConfigs/MSPM0G3507.ccxml`。若实际使用其他调试器，需要先匹配目标配置，不能直接使用此 J-Link 任务。

工程内部名称保留为 `task4_625_plus`，因此当前构建输出是 `Debug/task4_625_plus.out`。烧录任务从 `.project` 读取名称，不会选用 Debug 中遗留的其他 `.out` 文件。

任务使用现有 CCS、TI Arm Clang 4.0.4.LTS、MSPM0 SDK 2.10.0.04 和 SysConfig 1.27.0。完整构建经过验证；USB 调试器连接及板上运行需在连接硬件后验证。
