# 日常方案与验证方案分离结果

2026-10-09。用户确认选择“日常方案只保留主程序和核心库，测试单独生成”。

已完成：根 CMake 默认 `BUILD_TESTING=OFF`，只定义产品目标 `MeshStudio`、`mesher`；133 个测试、CAD 重放和基准工具目标集中迁移到 `cmake/MeshStudioTests.cmake`，开启测试时完整生成。测试源码没有删除。

`F:/Code/MeshStudio/build/MeshStudio.sln` 已重新生成。现有两个产品目标；CMake 自动生成的 `ALL_BUILD`、`ZERO_CHECK` 收在 `CMake` 分组。`RUN_TESTS` 和验证工程不再进入该日常方案。旧构建缓存文件未批量删除；不在新方案中列出的旧项目文件不会参与默认构建。

测试方案继续使用仓库外 `C:/Users/30861/AppData/Local/MeshStudioValidation/gmsh-reproduction-20261007/MeshStudio.sln`，已显式配置 `BUILD_TESTING=ON`。README 同步了日常构建、独立测试构建及各验证工具路径。

## 检查结果

- 产品默认配置和测试开启配置的 Release 构建都成功，退出 0。产品在仓库外新目录完整构建，避免覆盖用户正在运行的本地 EXE。
- File API 对照全部配置下的原目标、源码、编译定义/选项、包含目录和链接输入保持一致；133 个验证目标没有遗漏。只规范化构建目录、追溯索引和生成规则目录哈希等非语义信息。
- 原 139 项 CTest 的名称、命令、配置和属性保持一致；120 项不同的相关 C++ 回归通过。未将它说成全部 139 项测试通过。
- 新目录默认关闭测试、本地旧目录关闭测试均为零项注册。另在同一独立目录执行 OFF→ON→OFF，测试数量为 0→139→0；关闭时生成空注册清单，旧测试不会残留。
- 460 个源码、测试和资源文件 SHA-256 不变；只调整构建配置、README 与记录文件。算法、GUI 和生产默认未变。
- `git diff --check` 退出 0。
- 本地程序 PID 31948 继续响应，标题“未命名工程 — MeshStudio”；其 EXE SHA-256 仍为 `85e41b2d4117364c3b00f8f94c479ba3c4989204541207f832b7fd95e067e38d`。本轮没有关闭或重链接该程序。

本轮未重跑实际 GUI 操作、完整网格对照矩阵或正式性能测量；此前的数值缺陷和验收限制不因工程整理改变。未创建分支、提交或删除测试资产。

证据与备份：`C:/Users/30861/AppData/Local/MeshStudioValidation/test-project-separation-20261009/`。包含拆分前 CMake/README/本地方案、CTest 与 File API 清单、`configuration-check.json`、`final-check.json`、开关切换清单和构建/回归日志。
