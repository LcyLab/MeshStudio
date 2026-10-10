# MeshStudio

当前代码按 `ui / app / cad / project / geometry / mesh` 分类，VS 工程树同步显示实际模块目录。入口、模块职责和调用链见[架构导航](docs/architecture-overview.md)。

## 算法阅读入口（2026-10-10）

- 快速理解主流程：[Algorithm 6 阅读路线与面试要点](docs/Algorithm6-阅读路线与面试要点.md)。
- 按原理和实现深入阅读：[Algorithm 6 精读手册](docs/Algorithm6-精读手册.md)。
- 代码顺序：`SurfaceGenerator.h` → `SurfaceGenerator.cpp` 中的 `generateSurfaceMesh`、`generateFaceMeshWithRandom` → `FrontGeneration.cpp` → 候选点、定位、空腔和插入模块。
- 核心流程：共享 CAD 边界 → 逐面 UV 图表 → 初始剖分/约束恢复/域分类 → BDS → 前沿插点 → 转移/平滑 → 模型装配。

`mesher` 是纯 C++17 算法库，不链接 Qt、VTK、OCC 或 Gmsh；`MeshStudio` 应用负责 CAD 适配和界面。生产默认仍为 legacy，Algorithm 6 在参数中显式选择。

2026-10-10 已完成目录分类和中文函数注释，详情见[模块整理记录](docs/superpowers/plans/2026-10-10-module-layout-results.md)和[中文注释记录](docs/superpowers/plans/2026-10-10-chinese-function-comments-results.md)。算法质量与性能沿用各报告注明的程序和日期；本次文档更新没有重新执行整模型验收。

## 独立Algorithm 6实验入口（2026-10-09）

Windows MSVC x64构建使用Visual Studio自带MASM编译独立成对三角函数求值，保持固定官方Windows64基线的球面前沿舍入；其他平台使用标准库，不承诺跨平台逐值一致。该计算不依赖Gmsh；原BREP、Newton规则及生产默认保持。

`GmshSurfaceReproductionBenchmark`控制台工具：`generate CAD.brep h extension 新输出目录`只接收BREP与配置，独立生成普通/周期面三角网格，输出阶段/最终VTP、CAD标签、逐角UV和审计报告；可选`--no-stages`、`--point-size value`及`--curvature elements`。`audit CAD.brep mesh.vtp output.json`使用相同独立审计器检查任一侧最终网格。工具依赖OCC/VTK运行库，核心不依赖Gmsh；构建目标为`GmshSurfacePipelineTest`和`GmshSurfaceReproductionBenchmark`。

GUI网格参数可显式选择`Algorithm 6（实验）`，独立设置h、边界传播、原平滑及预算；默认保留legacy。生成完成可显示/保存，但质量与CAD误差须独立审计。阶段A三例六组结果见[阶段A报告](docs/superpowers/plans/2026-10-08-algorithm6-stage-a-results.md)。2026-10-09验收记录为34组相对质量通过、31个绝对审计合格配置的93轮性能通过；3组共有绝对失败保留。完整范围、失败项和历史程序指纹见[最终验收记录](docs/superpowers/plans/2026-10-09-algorithm6-final-acceptance-results.md)，较早[B/C/D检查点](docs/superpowers/plans/2026-10-09-algorithm6-bcd-progress.md)保留作历史记录。

独立对照工具为`tests/GmshSurfacePipelineComparison.py`；参考逐角UV在参考进程中前向核验，不给自研入口提供输入。`tests/GmshAlgorithm6Performance.py`限定当前程序和参考导出脚本已获质量资格的配置，串行三轮复验。`tests/GmshSurfaceMeshDistributions.py 对照目录 --plots`补充最终物理边长和最小角分布，不增加验收门槛。逐项交付状态见[B/C/D核对表](docs/superpowers/plans/2026-10-09-algorithm6-bcd-completion-audit.md)。

`GmshSurfaceInitializationProfile CAD.brep h extension 新输出目录 repetitions`只诊断单个普通面，按独立CAD输入重放各初始化模块并记录CAD回调耗时；每次重放都与真实生成器初始池逐值核验，输出最终VTP用于比对。该工具显式运行，诊断开销与正式三轮性能分开。

同一工具的 `capture-faces CAD.brep h extension 新输出目录 face-id ...` 模式运行完整模型，仅导出指定一基CAD面的初始、平滑前和最终阶段VTP（各次重访分开保存），保留完整模型的共享边与扰动流。完整VTP只在生成成功后导出；此模式不执行独立质量审计，不作为正式性能。

`capture-prefixes` 使用相同参数，只支持指定普通 General 面；另外导出各初始化子模块池、原CAD pcurve/三维曲线采样与分类器交叉诊断，并逐值核对每次重放与实际初始 FaceMesh。`classify-face CAD.brep 一基面号 单面阶段.vtp 新输出.json` 可只读复查已有阶段网格的分类器状态和默认分类器所选边交点，不运行生成或质量验收。诊断结果不改变生成或独立审计规则。

`repro-linear 新输出.json` 构造独立有效的六边形平面，复现小尺度边端点附近的OCC默认分类与点到面距离异常；不需要phone或参考数据。此模式保存原始查询结果，不把异常诊断当作质量通过。

`coverage-grid CAD.brep 完整网格.vtp 新输出.json` 只读比较直线平面81×81采样点的审计分类与两条原OCC分类路径，并保存分歧位置及其到现有网格的距离。它用于定位查询误判，不执行生成或正式性能。

`cad-boundary CAD.brep 一基面号 新输出.json` 只读检查原CAD每条边的三维曲线、pcurve及支撑面是否一致，记录101点采样、OCC默认23点和101点检查的偏差，并补原曲线/pcurve类型、共享端点和完整圆系数一致性；输入形状须保持原身份。支持球面退化极点，不运行生成或修复CAD。`classify-face` 对域外片追加三个原角点的2D/3D及FClass2d查询，供比较阶段来源。

`distance-peaks CAD.brep 完整网格.vtp 新输出.json` 只读重放审计的每片七个物理样本，记录最差20个面的实际三角形/样本位置、原CAD边距离、支撑投影及分类器交叉结果。面、边和三角形编号均为零基；不重生成网格，不改变距离查询策略，也不作为正式计时。

独立审计的CAD查询合同v3在原共享顶点闭合、二维/三维解析边系数一致的平面上直接处理直线和圆弧，保留孔洞、逐角UV和1e-10分类容差；物理距离不按ON容差置零。非解析、非平面或不一致边环保留原OCC查询。采样合同仍为v2；该审计路径不参与网格生成，程序变更后重新取得完整资格与性能证据。

独立审计对完全由一致直线围成的平面采用共享CAD顶点身份构成的闭合线环；检查三维曲线、pcurve和顶点的舍入一致性，其他面保留OCC查询。UV容差仍为1e-10，物理距离不因ON容差归零；报告分别记录采样合同与CAD查询合同版本。`GmshSurfaceCadAuditQueryTest`及`GmshSurfaceCadAuditDistanceTest`覆盖小尺度分类误判、pcurve舍入间隙、孔洞、位置/方向及曲面回退。

## 四面体测试状态（2026-10-02）

按用户要求暂停三角面片侧改动，使用 `test_data/gmsh-tet/` 的 Gmsh 固定 T3 测试自研四面体核心；默认回归不要求安装 Gmsh/Python。已补真实质量/收敛、3D 谓词、闭合体边界检查、有限局部面恢复及凹体/空腔域提取，相关 23 项回归通过（包含明确拒绝和质量告警测试）。10 组输入中 9 组几何通过、6 组质量通过；粗球面恢复及细网格薄片仍未解决。最新产品 GUI/自研表面集成尚未验收。

```powershell
ctest --test-dir build-tests -C Release --output-on-failure -R '^GmshTet'
```

详细结果及完整复现见 [四面体实施与验收记录](docs/superpowers/plans/2026-10-02-tet-correctness.md)；每项生成 `build-tests/GmshTet*.json`，成功项另存 `.vtu/.vtp`。以下章节含此前阶段记录。

用于 CAD 几何导入、表面三角网格和四面体网格生成的独立工程应用。

## 定位

- 界面负责收集参数、启动任务和展示结果；计算几何与网格算法编入 `mesher` 库，CAD 查询通过应用层适配器提供。
- 与 EMStudio 解耦，可独立构建和运行。

## 当前应用流程

- 主窗口布局：模型树（左）/ 3D 视口（中）/ 输出日志（下），属性栏按需打开；网格设置使用独立 Dialog
- **浅色主题**（紫色强调 #583266）、启动画面、浅色 SVG 图标
- **单 VTK 视口架构**（方案 A）：OCC 读入 → 三角化/棱边折线 → vtkPolyData，几何与网格同视口叠加
- **几何导入**：STEP(.step/.stp) / IGES(.iges/.igs) / BRep(.brep) 走 OCC（对齐 EMStudio ShapeLoader），STL 走 VTK 快速路径；支持对话框、拖拽、命令行参数三种方式
- 模型树接真实统计（实体/面/边计数），地面网格随模型尺寸自适应
- VTK 消息重定向到 meshstudio_vtk.log，不再弹窗
- **「四面体网格」已接入自研引擎**（取代 Gmsh）：参数 Dialog → `TetMeshParams` → 后台线程
  `QtConcurrent::run` → `TetMeshEngine::run` → 视口叠加网格 + 质量日志/状态栏
- **输入格式已打通两类**：
  - STL —— 本身即三角形汤，直达引擎
  - **STEP / IGES / BRep** —— 先由 `SurfaceMeshEngine` 生成自研表面，再通过 `TetMeshEngine` 的索引表面入口生成体网格，保留节点及 CAD 标签。
  - 已生成的表面网格可直接复用。OCC 展示离散用于导入后的显示，不能代替这条 CAD 网格生成链。
- 原四面体流程中，`minSize` / `curvatureAngle` / `elementOrder` / `algo3D` /
  `optimizeThreshold` 及「共形设置」整组**尚未被引擎消费**（待接）

## 自研 CAD 表面三角网格

参数 Dialog 使用统一的 Qt 表单工厂：紧凑布局、数值去尾零，非编辑时显示简写，编辑/悬停显示完整精度。网格设置和模型变换已复用；新增 Dialog 可参考 [工厂使用示例](docs/dialog-form-factory.md)。

选中 STEP、IGES 或 BRep 模型，在「几何」功能区点击「三角面片」打开对应参数页，设置最小/最大单元尺寸、最大几何偏差和最大法向变化角，点击「生成三角面片」。默认最大尺寸为包围盒对角线的 2.5%，最小尺寸和偏差为最大尺寸的 10%；长度均采用导入后模型坐标单位。高级设置包含质量目标、边界尺寸渐变率及节点/工作预算。STL 按已有三角网格输入处理。

「四面体网格」按钮打开同一 Dialog 的四面体页，仅提供当前引擎使用的目标尺寸和质量优化。两套参数独立保存，取消/关闭弹窗不修改工程。后台表面生成期间功能区提供「取消表面生成」。

表面 `maxSize>0` 为三维三角边长上限，仅允许 1e-8 相对比较容差；API/旧工程 `maxSize=0` 保留原目标尺寸及 5% 余量。旧工程打开弹窗会说明迁移，取消保持原值。最小尺寸仍是常规细化软下限，小特征/几何要求允许突破并报告原因。

新流程由 OCC 查询几何，自研纯 C++ 核心先离散共享 CAD 棱，再构造各面 UV 参数域的 CDT 并按三维测量结果细化。支持多环及孔、周期缝、退化极点；共享棱需要进一步分裂时重建面网格。生成结果经过约束完整性、边关联及方向、几何采样和相交检查，失败返回具体诊断。

表面网格可随工程保存/重开，`.vtp` 携带 `CadFaceId`、`CadEdgeId` 和逐角 `CadCornerUV`。当前四面体的索引表面入口复用这些节点和 CAD 标签。

装配体按各 CAD 实体独立生成完整表面，保留接触与干涉，新增 `CadBodyId` 及随工程保存的 `CadAssemblyPolicy`/`CadBodyInteractions`。同一实体自交仍拒绝；跨实体网格交叉记录 CAD 关系，查询失败明确标为未核实。纯 C++ API 默认严格策略，实际表面引擎显式选择独立部件策略。接触两侧没有建立一致公共剖分，不能直接作为多区域四面体边界。

表面引擎可在深副本上协调共享端点附近的有界微小裁剪交叉容差，完整 OCC 有效性通过才接受；不改原文件、曲面/曲线、位置和拓扑。接受的 B-Rep 和修复记录存入网格字段。几何细化完成后再做受硬判据约束的三维质量翻边/节点移动；残留质量仍明确报告。phone/凹球的默认参数、质量统计、全部面距离采样及 GUI 验证见 [问题修正记录](docs/superpowers/plans/2026-10-02-phone-concavaball-debug.md)。

几何偏差是采样测得值；相交检查使用尺度容差。质量和最小尺寸在尖角、窄区及极点附近为软目标，并输出冲突诊断。窄间隙发生面片穿越时会收紧相关面的误差并重采样共享边；无法在预算或数值精度内解决时明确失败。`test_data/ball.brep` 和真实 `move.stp` 已通过核心/引擎、独立双向采样及实际 Qt 窗口生成/取消/保存重开回归，并检查窗口图像；这不是用户手动操作验收，不保证所有 CAD/参数成功。架构与限制见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 构建（VS2022 / x64）

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTING=OFF
cmake --build build --config Release
```

日常打开 `build/MeshStudio.sln`，只生成 `MeshStudio` 主程序、`mesher` 核心库及 CMake 的辅助工程。`BUILD_TESTING` 默认关闭；测试、CAD 重放和基准工具的源码保留，按需在独立目录生成：

```bat
cmake -S . -B build-tests -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTING=ON
cmake --build build-tests --config Release
ctest --test-dir build-tests -C Release --output-on-failure
```

验证目标集中定义在 `cmake/MeshStudioTests.cmake`。后文的基准、审计和 GUI 测试工具示例均使用开启测试的 `build-tests` 目录。

Git 保存源码、必要构建配置、界面资源及项目文档；构建产物、验证输出和新增模型数据保留本地。完整测试中的固定 Gmsh 体边界输入位于 `test_data/gmsh-tet/`，该目录不随本次源码提交提供；缺少输入时需先用 `tests/GmshTetFixtures.py` 准备对应案例，不能把源码构建成功当作完整回归通过。

历史阶段报告保留本地证据路径，并标明未入库；相关图片、日志和 JSON 不随 Markdown 文档上传。

依赖：Qt 5.14.2 (msvc2017_64)、VTK 9.3.1（products 下）、OpenCASCADE 7.6.0（install 下，头文件在其 build/inc）。

exe 旁已部署 Qt/VTK/OCC 运行库，可直接双击运行；重编译后如缺 DLL，重跑依赖闭包拷贝或用 windeployqt。

## CAD 表面性能基准与独立审计

`CadSurfaceBenchmark` 为显式运行的控制台工具，不加入默认 CTest 的重型性能运行。默认从 CAD 包围盒解析尺寸；关闭观察用于速度基线，开启观察用于嵌套阶段/逐面时间和工作量分析。

```powershell
.\build-tests\Release\CadSurfaceBenchmark.exe --cad "D:/DeskBox/30861/other/move.stp" --observe off --timeout 300
.\build-tests\Release\CadSurfaceBenchmark.exe --cad "D:/DeskBox/30861/other/move.stp" --observe on --timeout 300
.\build-tests\Release\CadSurfaceQualityAuditTest.exe --cad "D:/DeskBox/30861/other/move.stp" --timeout 600
```

两工具都接受 `--target`、`--min`、`--max`、`--deviation`、`--angle`、`--work`、`--vertices`；省略尺寸采用当前 API 自动值，未指定 `--max` 保留旧目标尺寸及 5% 余量。基准额外接受 `--output`（必须是不存在的新目录）。未指定时保存到 `%TEMP%/MeshStudio-surface-perf/<唯一运行名>`，包含 `run.json`、`stages.jsonl`、`faces.csv`、`diagnostics.txt`；清单包含原模型、源码和二进制指纹、参数、结果指纹和计时。包含子阶段的时间不可直接相加；按自身时间分账。

独立审计保留原单路径 ball 模式（h=20、δ=2、预算 40000、生成时限 30 秒）；具名参数模式可用其他 CAD。`--timeout` 分别限制生成阶段和审计阶段。极窄面若固定格点未覆盖，会补充经过 CAD 裁剪分类的内部采样；仍无样本则失败。该工具验证有限采样距离，不证明全局误差界。

A—F 已实现：几何对象/原参数缓存、局部事务、局部域标签及版本化队列。move 原参数（maxSize=0）串行核心含内部验收三轮中位数 2.83 秒、97568 片，完整双向独立审计另计约 37 秒。Gmsh 实测 36 次，但网格规模和判据不同，不给同质量速度倍数。完整计量、质量、GUI 和限制见 [最终验收报告](docs/superpowers/plans/2026-10-02-surface-performance-stage-f.md)。

实际窗口回归需要桌面/OpenGL 环境，不加入默认 CTest：

```powershell
.\build-tests\Release\CadSurfaceGuiTest.exe 'test_data/ball.brep' 20 40000
.\build-tests\Release\CadSurfaceGuiTest.exe 'D:/DeskBox/30861/other/move.stp' 38.902828952788262 200000
```

测试仅写独立临时工程/设置/图像；保存重开逐值检查网格和 CAD 标签。表面质量统计尚未保存，重开后显示 `—`。`tests/GmshSurfaceBenchmark.py` 和审计器 `--external` 是可选对照工具，不是产品依赖。
## 里程碑

| 阶段 | 内容 |
| --- | --- |
| Phase 0 | UI 骨架 + 主题 + 视口 ✅ |
| Phase 1 | OCC 导入 STEP/IGES/BRep，几何显示、模型树真实数据 ✅ |
| Phase 2 | 参数面板接自研引擎 + 后台线程生成网格；STL 与 STEP/IGES/BRep 两类输入打通 ✅ |
| Phase 3 | 网格显示 / 质量着色 / 剖切 / **导出（.vtu/.msh/.inp）**（待做：目前只有截图导出） |
| Phase 4 | 诊断面板：共形/退化单元列表联动视口 |
