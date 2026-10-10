# 网格参数 Dialog 与显式最大尺寸验收

2026-10-02，按用户批准的方案执行；在原工作区修改，未创建分支/worktree、未提交。

## 实际交付

- 「几何」功能区的「三角面片」「四面体网格」各自打开对应参数页。右侧网格参数 Dock 不再创建，原 Panel 文件保留。
- 同一个 MeshSettingsDialog，两页草稿独立。取消/关闭不提交；生成先校验并只提交当前页。MainWindow 独立持有两套参数，工程保存/恢复不依赖 Dialog 生命周期。
- 三角页：最小/最大单元尺寸、最大几何偏差、最大法向变化角；高级项为质量半径边比、边界尺寸渐变率、节点及工作预算。长度使用模型坐标单位。
- 四面体页：仅目标尺寸和质量优化；明确当前为 T4、自研 Delaunay 与细化，体目标尺寸不是严格输出边长上限。四面体算法未改。
- maxSize>0 约束三维三角边长，贯穿目标钳制、公共边采样、面内评分/分裂和输出验收，容差为 1e-8 相对值。minSize 仍是软下限，特征/几何冲突诊断保留。
- API/旧工程 maxSize=0 保留目标尺寸+5% 原行为。旧工程弹窗显示原允许值及迁移说明，取消保持旧参数；生成采用显式新上限。
- 保持既有浅灰/白色/深茄紫主题、字体和标题栏；高级项内部滚动，底部操作始终可见。后台表面取消保留在主窗口。

## 构建与回归

```powershell
cmake --build build --config Release --parallel 4
ctest --test-dir build -C Release --output-on-failure -j 4
```

最终 Release 全构建成功；37/37 通过（19 单元、18 端到端，CTest 实际 7.82 秒）。新测试覆盖正数输入下限、非法 min>max、表面/体草稿隔离、取消、逐页恢复默认和旧最大尺寸迁移；工程测试实际打包缺失 maxSize 的旧格式并加载，保存/恢复兼容通过。核心测试覆盖最大尺寸、非法范围/非有限参数，CLI 参数测试通过。

最初弹窗测试因部署目录只有 qwindows.dll、没有 qoffscreen.dll 停在 Qt 启动。停止该测试进程后，CTest 插件路径改用 Qt5::QOffscreenIntegrationPlugin 的 SDK 目录，再完整构建/回归通过。没有跳过测试或静默吞错。数值控件先设置小数精度再设置范围，保留 1e-9 正下限。

最终日志：`%TEMP%/MeshStudio-dialog-final-build.txt`、`%TEMP%/MeshStudio-dialog-final-ctest.txt`。git diff --check 通过；仅有工作区已有的换行规范提示。四面体引擎与 3D 核心 diff 为空，两原模型 SHA256 未改变。

## 实际主题窗口验证

```powershell
.\build\Release\CadSurfaceGuiTest.exe test_data/ball.brep 20 40000
.\build\Release\CadSurfaceGuiTest.exe 'D:/DeskBox/30861/other/move.stp' 38.902828952788262 200000
```

加载产品 Fusion/主题，操作实际 MainWindow 和 Dialog：空工程保存打开、导入、两个入口对应页、改草稿后取消、非法范围拒绝、生成与后台取消、最大边长独立遍历、VTK 显示、模型树/统计、保存后新窗口重开。坐标/三角连接/CadFaceId/CadEdgeId/CadCornerUV 逐值一致，两类参数和高级项恢复通过。生成未污染四面体参数。

| 模型 | 最大尺寸 | 最小/偏差 | 工作预算 | 输出点/三角形 | 实测最长边 |
|---|---:|---:|---:|---:|---:|
| ball.brep | 20 | 2 / 2 | 40000 | 22220 / 44432 | 4.39812 |
| move.stp | 38.902828953 | 3.890282895 / 3.890282895 | 200000 | 48636 / 98476 | 38.88833372619608 |

两个模型角度均为 30°，节点预算 200000。move 旧配置 maxSize=0 的最长边 40.8215 超过新要求，因此新配置产生更多面片；没有将原 97568 片/性能记录冒充新版结果。

最终实际窗口图像目录（只写测试自己的临时目录，已检查弹窗与窗口图像）：

- ball：`C:/Users/30861/AppData/Local/Temp/MeshStudio-gui-SvWgAf`
- move：`C:/Users/30861/AppData/Local/Temp/MeshStudio-gui-AOOEjO`

目录包含 surface-dialog.png、surface-advanced-dialog.png、volume-dialog.png、generated-window.png、restored-window.png 和 surface.msp。不是用户手动操作验收；最大尺寸约束不意味着每条边都满足最小尺寸。

## 新配置的独立几何检查

```powershell
.\build\Release\CadSurfaceQualityAuditTest.exe --cad test_data/ball.brep --target 20 --max 20 --min 2 --deviation 2 --angle 30 --work 40000 --timeout 120
.\build\Release\CadSurfaceQualityAuditTest.exe --cad 'D:/DeskBox/30861/other/move.stp' --target 38.902828953 --max 38.902828953 --min 3.890282895 --deviation 3.890282895 --angle 30 --work 200000 --vertices 200000 --timeout 180
```

两项均通过，生成器的拓扑/方向/约束/采样/相交验收仍启用。独立双向采样不复用生成器偏差函数：

| 模型 | 检查面数 | CAD→网格最大距离 | 网格→CAD最大距离 |
|---|---:|---:|---:|
| ball | 17 | 0.29371385173452602 | 0.26676809702124715 |
| move | 2283 | 0.096062127631909536 | 0.061421313951626276 |

日志：`%TEMP%/MeshStudio-dialog-ball-audit.txt`、`%TEMP%/MeshStudio-dialog-move-audit.txt`。有限采样不证明全局误差界。没有在本任务中扩展任意 CAD 自动修复、硬最小尺寸或四面体算法。

一次显式 max 的基准清单位于 `C:/Users/30861/AppData/Local/Temp/MeshStudio-surface-perf/20261002-082456-842-71475adc-e54b-40b1-9862-db775c09f877/run.json`，核心含内部验收 3.4462 秒。该次与窗口任务有重叠，不作为隔离性能基线或多轮速度保证。
