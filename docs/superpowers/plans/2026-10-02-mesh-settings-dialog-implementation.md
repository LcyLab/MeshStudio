# 网格参数 Dialog 实施清单

用户已确认 `specs/2026-10-02-surface-size-controls-design.md`，在原工作区执行，不创建分支、不提交、不改四面体算法。

- [x] 新增最大尺寸和参数保存回归，先复现旧 API/判据缺失。
- [x] `SurfaceCriteria::maxSize` 可选上限：0 保留旧语义；显式值统一应用采样/队列/分裂/最终验收；CLI/清单/工程 JSON 同步。
- [x] 新建主题一致的 `MeshSettingsDialog`：独立页、草稿、校验、恢复默认、基础/高级设置，仅开放实际支持项。
- [x] MainWindow 独立持有参数；两 Ribbon 入口打开对应页；取消草稿无副作用；移除旧参数 Dock，后台进度/表面取消留在主窗口。
- [x] 工程保存恢复独立参数；兼容 maxSize 缺失的旧工程，重新生成时显式说明采用新最大尺寸。
- [x] 迁移实际 Qt GUI 测试：主题、两入口/页、参数隔离、草稿取消、生成取消/尺寸上限、图像和新窗口保存恢复。
- [x] 新参数 move/ball、Release 全构建/原回归、diff 检查，更新记录和结果。

完成证据：`2026-10-02-mesh-settings-dialog-results.md`。

修改范围：MainWindow、dialogs、SurfaceCriteria/CadSurfaceMesher、ProjectArchive、CLI/测试及 CMake/主题的 Dialog 范围样式。旧 MeshSettingsPanel 文件保留，但不再被主窗口构建或显示。

四面体页核对：当前仅 targetSize 与 optimize 被生成器消费，其他预留参数不作有效输入；不借 UI 任务扩展四面体算法。
