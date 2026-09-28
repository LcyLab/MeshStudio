# MeshStudio

FDTD 前处理独立界面模块（UI 骨架 Phase 0）。

## 定位

- **薄 UI + 厚引擎**：界面只负责收集参数、调用 MeshEngine、展示结果；算法逻辑全部留在 MeshEngine 库中。
- 与 EMStudio 解耦，作为 MeshEngine 的专属前处理 / 调试载体。

## 当前状态（Phase 2 完成）

- 主窗口四区布局：模型树（左）/ 3D 视口（中）/ 网格参数（右）/ 输出日志（下）
- **浅色主题**（现代工程风，主色 #2563eb）、启动画面、浅色 SVG 图标
- **单 VTK 视口架构**（方案 A）：OCC 读入 → 三角化/棱边折线 → vtkPolyData，几何与网格同视口叠加
- **几何导入**：STEP(.step/.stp) / IGES(.iges/.igs) / BRep(.brep) 走 OCC（对齐 EMStudio ShapeLoader），STL 走 VTK 快速路径；支持对话框、拖拽、命令行参数三种方式
- 模型树接真实统计（实体/面/边计数），地面网格随模型尺寸自适应
- VTK 消息重定向到 meshstudio_vtk.log，不再弹窗
- **「生成网格」已接入自研引擎**（取代 Gmsh）：参数面板 → `TetMeshParams` → 后台线程
  `QtConcurrent::run` → `TetMeshEngine::run` → 视口叠加网格 + 质量日志/状态栏
- **输入格式已打通两类**：
  - STL —— 本身即三角形汤，直达引擎
  - **STEP / IGES / BRep** —— 边界表示、**不含三角面片**，先经
    `ShapeTessellator`（OCC `BRepMesh_IncrementalMesh` 逐面离散，弦高随目标尺寸收紧）
    得到三角形汤，再进引擎。逐面离散在共享棱上产生的近似重复顶点由
    `mesher::weldSurfaceMesh` 的容差焊接（3×3×3 邻域探测）缝合
- 网格参数面板结构已定稿；`minSize` / `curvatureAngle` / `elementOrder` / `algo3D` /
  `optimizeThreshold` 及「共形设置」整组**尚未被引擎消费**（待接）

## 构建（VS2022 / x64）

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest -C Release --output-on-failure
```

依赖：Qt 5.14.2 (msvc2017_64)、VTK 9.3.1（products 下）、OpenCASCADE 7.6.0（install 下，头文件在其 build/inc）。
exe 旁已部署 Qt/VTK/OCC 运行库，可直接双击运行；重编译后如缺 DLL，重跑依赖闭包拷贝或用 windeployqt。

## 里程碑

| 阶段 | 内容 |
| --- | --- |
| Phase 0 | UI 骨架 + 主题 + 视口 ✅ |
| Phase 1 | OCC 导入 STEP/IGES/BRep，几何显示、模型树真实数据 ✅ |
| Phase 2 | 参数面板接自研引擎 + 后台线程生成网格；STL 与 STEP/IGES/BRep 两类输入打通 ✅ |
| Phase 3 | 网格显示 / 质量着色 / 剖切 / **导出（.vtu/.msh/.inp）**（待做：目前只有截图导出） |
| Phase 4 | 诊断面板：共形/退化单元列表联动视口 |
