# 日常工程与验证工程分离

用户已选择：日常方案只保留主程序和核心库，测试单独生成。保留测试源码，不改算法和生产默认，不创建分支或提交。

- [x] 备份现有 CMake，记录测试名称、目标及编译配置。
- [x] 根 CMake 保留 MeshStudio/mesher，增加默认关闭的 BUILD_TESTING；验证目标移至 cmake/MeshStudioTests.cmake。
- [x] 更新 README 的日常构建和显式测试构建命令。
- [x] 验证默认方案仅含两个产品目标，BUILD_TESTING=ON 时原测试/目标/编译设置完整保留；执行相关构建与回归。
- [x] 更新 build/MeshStudio.sln，记录结果。当前用户程序保持运行，不覆盖它的 EXE。

验证构建与备份继续放在仓库外。常规方案仍包含 CMake 的 ALL_BUILD/ZERO_CHECK 辅助目标；它们不是测试程序。
