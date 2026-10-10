# 紧凑 Dialog 与 Qt 表单工厂

用户已确认紧凑工程风，并要求复用模板/工厂。本次在原工作区执行，不创建分支/worktree、不提交；保留已有网格算法与数据改动。

- [x] 数值回归：常规值最多三位小数且去尾零，极小/大值科学计数；焦点编辑完整精度，焦点往返/确认不改变未修改的值。
- [x] 新建 ui/DialogFormFactory：精确数值框、整数框、标题、分区/参数行、折叠区及操作栏；纯 Qt，不依赖网格引擎。
- [x] 网格 Dialog 改用工厂：精简标签、单位集中说明、减少套框/留白，保留两页、独立草稿、校验和高级项。
- [x] 模型变换 Dialog 复用数值框/操作栏，保留原三位存储精度、单位、范围与变换信号语义，验证接口。
- [x] CMake/主题/使用示例同步，构建及最小回归；实际主题窗口检查两模型参数精度、取消、保存恢复及图像。
- [x] 更新上下文/结果记录；最小尺寸仍为软下限，四面体算法不改。

范围：新增 src/ui/DialogFormFactory.h/.cpp，调整两个 dialogs、局部主题、测试及 CMake；不引入外部 UI 框架，不批量迁移其他面板。

## 完成证据

- 原显示回归先失败：`MeshSettingsDialogTest` 报 redundant decimals are still shown（日志 `%TEMP%/MeshStudio-form-red-test.txt`）。随后实现控件与模板，精确值、焦点往返、科学计数、locale、键盘增量和确认前提交均通过。
- 使用 Qt FloatingPointShortest 作为编辑/悬停文本：保留完整 double 往返能力，同时 0.1 不显示成 0.10000000000000001。常规显示最多三位小数；程序性格式刷新不会反向触发解析简写。
- `DialogFormFactoryTest` 仅链接 Qt Widgets/dwmapi，证明工厂不依赖网格引擎；验证模型变换原三位存储语义、复位值与信号。使用示例：`docs/dialog-form-factory.md`。
- 新主题限定 formDialog 属性。基本窗口 540×460，减少嵌套边框和单位重复；展开高级项在可用屏幕高度内增高，折叠恢复，屏幕不足时内部滚动。图片检查后补了增高行为，避免高级项展开仍看不到预算。

最终验证：

```powershell
cmake --build build --config Release --parallel 4
ctest --test-dir build -C Release --output-on-failure -j 4
.\build\Release\CadSurfaceGuiTest.exe 'D:/DeskBox/30861/other/move.stp' 38.902828952788262 200000
.\build\Release\CadSurfaceGuiTest.exe test_data/ball.brep 20 40000
```

Release 全构建成功；38/38 CTest 通过（20 单元、18 端到端，8.67 秒）。最终日志：`%TEMP%/MeshStudio-form-build.txt`、`%TEMP%/MeshStudio-form-ctest.txt`、`%TEMP%/MeshStudio-form-move-gui.txt`、`%TEMP%/MeshStudio-form-ball-gui.txt`。

两个实际主题窗口均验证两入口/页、参数草稿取消、非法范围、原始 double 与提交值逐值相等、展开预算可访问、生成与后台取消、三维最大边长、VTK 显示、保存后新窗口重开坐标/连接/CAD 标签及参数逐值一致。move 48636 点/98476 片，最长边约 38.8883；ball 22220 点/44432 片，最长边约 4.39812。工厂阶段没有修改几何/网格算法，也没有把显示三位小数应用到生成参数。

最终图像目录（只写测试自己的临时文件）：

- move：`C:/Users/30861/AppData/Local/Temp/MeshStudio-gui-BPaZew`
- ball：`C:/Users/30861/AppData/Local/Temp/MeshStudio-gui-wJtnSi`

已检查 surface-dialog.png、surface-advanced-dialog.png、surface-edit-dialog.png 及复用的 transform-dialog.png；这是实际 Qt 图像检查，用户手动视觉验收尚未进行。git diff --check 通过，原模型 SHA256 保持；未创建分支/worktree、未提交。
