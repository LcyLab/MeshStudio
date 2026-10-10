# Gmsh 表面生成流程复现实施计划

2026-10-08连续平滑检查点：自研普通/周期初始化前缀→前沿→转移→原参数源→平滑共168组零容差通过。较早1D/拓扑/raw CAD图表和前沿尺寸UV仍冻结；平滑参数几何独立计算。63 CTest/24 Python、64参数/120分类平滑回归、8中立性通过，证据smoothing-prefix-release-sha-audit.json。任务9只勾连续平滑子项；generic失败、周期对象清理/装配与完整质量验收未完成。以下为历史检查点。

2026-10-08当前：MeshVertexParameters/独立OCC源实现原默认参数路径，64actual/1240节点/27432邻边计算零容差通过，拓扑和结果不冻结。120真实CAD分类actual平滑/7170访问/971354源请求通过，但入口原未平滑网格冻结，自研前沿→平滑连续链待做；generic失败/robust未完成。45复刻+18既有CTest/21Python、180同面/146转移回归和8中立性通过，231372字节/69导出/11文件补丁实际重放。任务9整项/完整目标active、legacy保持；下一步连续集成。下面为前沿转移历史检查点。

2026-10-08前沿转移检查点：普通/周期自研初始化前缀已连续接FrontGeneration和FrontMeshTransfer，extension0/1共168组逐值通过（172318pop/74807成功/777域拒绝，161新片反馈输入）；输出161870片、64单次等价身份折叠及8对等价诊断一致。146转移叶+168结束池另行重放、61不同CTest/14Python、216259字节/66导出补丁真实重放与8neutrality通过。原1D/拓扑/raw反求冻结、自然失败/非有限退化/全恢复重建/装配及最终质量未完成。

2026-10-08当前：insertFrontPoint节点生命周期已实施，1560同BREP actual叶及extension0/1普通44+周期40共168完整自研前缀首候选逐值通过；失败槽/编号消费与控制失败入口恢复分开。57目标构建/58CTest（40复刻+18既有）/10Python、204验收/79空腔回归通过；192662字节/63导出/10文件补丁真实重放/8neutrality和7报告SHA通过，证据front-point-insertion-release-sha-audit.json。任务8仅单次插点子项新增勾选，完整FrontGeneration/剩余几何/全部失败重建/原后处理/组装和最终质量验收未完成，总目标active。下一阶段合同见front-generation-source-contract.md，以下历史。

2026-10-08当前insertVertexB检查点：独立FrontInsertion实现原早退/尺寸/物理R/h/距离/极钝角/严格UV面积验收，成功局部交错连接/AllTris登记/ActiveTris更新，正常-3/-4/-5恢复标记而-1/-2早退留给外层清理。204冻结叶逐值通过（126成功/504保留新片，-1至-5全部覆盖）；新点XYZ/UV/分类/尺寸、空腔与运行池在此叶明确冻结，未验证insertAPoint生命周期或FrontGeneration。55Release/49CTest（39复现+10既有）/10Python、79空腔叶回归、181227字节/62导出补丁真实重放/8neutrality通过，最终front-insertion-leaf-release-oracle与front-insertion-release-sha-audit.json。下一步实际新节点CAD/BGM/append槽、正常拒绝半径/发布、串联自研前缀和动态循环，再完整几何/失败/后处理/组装与质量验收；总目标active。以下旧检查点为历史。

2026-10-08当前空腔检查点：FrontPointLocation的原UV反映射/搜索/包含片mesh插值与FrontCavity的固定metric DFS/壳边/内部边保护/标记恢复已实施。79冻结叶和普通44/周期40在extension=0/1共168自研前缀阶段、10718候选逐值通过；114850空腔成员/136054壳边，含132次无包含片/122次搜索。53Release/48CTest（38复现+10既有）/10Python、173329字节/61导出补丁真实重放/8neutrality通过，证据为*-front-cavity-extend*-release-oracle及front-cavity-release-sha-audit.json。当前只验收有限初始候选的独立空腔，未执行参数域门控或真实插点反馈；原1D/拓扑/raw反求/指针序仍冻结。Task8仅候选/空腔子项具备证据，整项及完整目标active。下一步以front-insertion-source-contract.md实现CAD/BGM/append槽、验收/提交和动态循环，再剩余几何/失败/后处理/组装与最终质量，不加入优化。以下旧检查点为历史。

2026-10-08当前候选阶段：FrontPointPlacement有限候选/普通OCC圆修正、直接支撑面Value、有符号高度/epsilon及控制状态已经落地；普通/周期自研全前缀在两种extension共168阶段/10718候选/82792求值逐值一致。230叶中228有限一致、2非有限仅诊断；50Release/46CTest/10Python与补丁重放/8neutrality通过，当前证据为*-signed-release-oracle。任务8仅勾候选子项，实际包含片、空腔、插点反馈及原NaN失败处理尚未完成，整体active。以下旧检查点为历史。

2026-10-08当前调度：FrontTriangleOrdering/FrontQueue实现共享身份键、首活动边、原初始break扫描、pop、局部入集和失败0/-1状态。61冻结MTri3 native方法/set叶对照和extension=0/1普通纯/CAD各44、周期40共256前缀排空阶段/16480pop通过；48Release/45CTest（35复现）/8Python、158779字节/59导出补丁实际重放/8neutrality通过。排空没有候选/插点反馈；实际包含片插值、空腔/成功失败连接和动态调度仍未完成，Task7不整项勾选。下一步FrontPointPlacement/FrontCavity，再剩余几何/失败/后处理/组装及最终验收，仍先复刻而不加优化。下文为历史。

2026-10-08当前：NodeSizeInitialization已接周期全前缀并通过40actual，FrontMeshTypes/FrontTriangleInitialization按状态与初始化职责分开。实际buildMeshGenerationDataStructures的MTri3均值/物理R/h、AllTris顺序及connectTriangles邻接与普通/周期自己的完整前缀逐值一致；普通纯/CAD各44、周期40在extension=0/1共256actual/18472三角。46Release构建、44CTest（34复现）/8Python及旧阶段回归通过，151280字节/55导出补丁真实重放和8neutrality通过。前沿反求UV/节点指针序、原1D/拓扑元数据仍冻结；set重复拒绝只源码/单元覆盖。下一步源码活动集/活动边，再候选/空腔/插点；Task7及总目标不整项勾选，不加入优化，全部几何/失败/后处理/组装及最终验收仍未完成。下文为历史。

2026-10-08最新节点阶段：普通面共有转换驱动RED未知模式→GREEN，纯/CAD求导各44actual一致（1552条deleted）；NodeSizeInitialization单元RED12→GREEN并接普通全前缀，纯/CAD求导各44actual/3074节点/16embedded BGM/38内部约束/16不同双尺寸逐值通过。前沿反求UV/原生指针遍历序仍冻结，源尺寸与输出尺寸未冻结。下一步周期节点实际对照和MTri3半径状态/邻接，再队列/候选/空腔/插点；完整几何、失败1D重建、平滑/回退/组装及整模型验收仍未完成。Task7不整项勾选，不加入自有优化；以下为历史。

2026-10-08最新面转换检查点：FaceMeshTypes/PeriodicSeamReconstruction/FaceMeshConversion实现数据、周期缝副本和共有三角转换职责。40actual周期全前缀630副本/3084三角/492deleted记录逐值一致；24实际BDS2GMSH叶覆盖新节点创建/复用、重复身份退化过滤和先创建后过滤。临时/新身份按创建序归一，未比较原生进程全局编号；原1D元数据/拓扑序/副本序/物理链仍冻结。19受影响Release目标构建、42CTest（32复现）/8Python与普通纯/CAD度量各24+44回归通过。下一步普通完整转换驱动、前沿节点尺寸/MTri3/队列，再候选/空腔/插点/平滑/回退/组装；失败1D重建/全几何和整模型验收仍需补齐。Task5/6/7及完整目标不整项勾选，不加入自有优化；下文为历史。

2026-10-08最新周期检查点：同一InitialMeshLegalization已接周期CAD/图表/初始/BDS/恢复/域/清理全前缀，真实阶段RED未实现→GREEN；40actual/3654查询/246交换/63扫描/124资格拒绝逐值通过，圆柱/球面压力补齐原32例无交换缺口。仍冻结1D/拓扑/副本序/物理曲线链，不冻结CAD导数或输出连接。证据periodic-bds-metric-final-oracle。下一步周期缝临时副本与BDS→GFace转换、失败1D重建，再前沿/后处理/回退/组装与整体验收；Task5/6/7不整项勾选，不加入自有优化。

2026-10-08最新检查点：BdsSwapQualification/InitialMeshLegalization已按职责分开，44actual普通全前缀的4131度量查询/776交换/82扫描/4资格拒绝全值一致，独立CAD支撑面D1也一致。270参数面积/725度量圆/42实际Quality(false,true)交换叶函数通过；BRepAdaptor B样条末位差异由实际失败定位，新增独立faceSurfaceDerivatives，没有放宽容差。48Release顺序构建后39CTest（29复现）/8Python和六旧普通/周期/恢复矩阵复核通过。Task5/6仍缺周期恢复驱动、剩余图表/几何/全部失败分支，后续GFace转换/前沿/平滑/回退/组装及最终质量未完成，不勾选整目标。下一步周期恢复驱动与GFace转换，不引入自有优化。

2026-10-08最新检查点：新增独立BdsNodeSizeInitialization，按原MVertex查询BGM赋BDS双尺寸，36actual/2503查询全池一致。尺寸值自研，原查询元数据/前缀CAD输入仍冻结。43Release顺序构建后36CTest（26复现）/8Python与7前阶段矩阵复核通过。任务5/6/7仍缺清理/度量、周期与剩余图表/几何/失败分支，前沿等后续任务未完成；不勾选整目标。下一步池清理/分类传播，再度量合法化。

> 按 superpowers:executing-plans 在本会话逐项执行和验证。遵守用户现有工作区约束：不创建分支/worktree，不提交或推送，不自动分派子代理。

**Goal:** 用本项目几何算法完整复现固定 Gmsh 4.15.2 的 OCC 串行一阶算法 6 表面生成链路，包括周期、退化、原生平滑与失败分支，之后才评估自有优化。

**Architecture:** 新建 `src/core/mesher/gmsh_reference/` 的独立编排入口，复用经对照通过的低层几何/拓扑，不挂接旧产品细化流程。测试参考 Gmsh 放仓库外；核心不链接 Gmsh。数据、模块、函数和验收定义见 [模块设计](../specs/2026-10-07-gmsh-surface-reproduction-modules-design.md)。

**Tech Stack:** C++17、现有 Mesh2D/事务/谓词、GeometrySource/OCC、CMake/CTest；参考端固定 Gmsh 源码/SDK 与 Python -B。构建和证据保存在 `C:/Users/30861/AppData/Local/MeshStudioValidation/`，不修改仓库已有构建产物。

## 执行约定

根目录是 `F:/Code/MeshStudio`。下面新算法文件基址为 `src/core/mesher/gmsh_reference/`，测试基址为 `tests/`，每个模块新增同名 `.h/.cpp`（数据/观察接口除外）。

每项按“冻结源码合同和输入 → 编写外部参照/可失败测试 → 实现最小模块 → 运行目标测试 → 保存阶段证据”执行。代码公式在对应任务开始前从固定源码核对，不能仅据本计划的摘要填充。这里规定实施合同、文件、函数和具体验证场景，不提前把尚未核对的完整算法伪装成可复制成品代码。

新目标加入 CMake 的 mesher 静态库；目标测试注册 CTest 标签 `gmsh_reproduction`。基准为 `GmshSurfaceReproductionBenchmark`，纯模块测试不得依赖 CAD SDK/Gmsh；OCC/参考对照目标按实际依赖链接。项目根目录没有额外 AGENTS.md，本轮已检查。

构建目录定为仓库外 `.../gmsh-reproduction-20261007`，配置沿用已验证的 Qt/VTK/OCC 路径。配置和已有模块目标的构建/测试已执行；后续目标尚待实施：

```powershell
& 'F:/ProgramData/cmake/bin/cmake.exe' -S 'F:/Code/MeshStudio' -B 'C:/Users/30861/AppData/Local/MeshStudioValidation/gmsh-reproduction-20261007' -G 'Visual Studio 17 2022' -A x64
& 'F:/ProgramData/cmake/bin/cmake.exe' --build 'C:/Users/30861/AppData/Local/MeshStudioValidation/gmsh-reproduction-20261007' --config Release --target GmshCurveDiscretizationTest
& 'F:/ProgramData/cmake/bin/ctest.exe' --test-dir 'C:/Users/30861/AppData/Local/MeshStudioValidation/gmsh-reproduction-20261007' -C Release -R '^GmshCurveDiscretizationTest$' --output-on-failure
```

测试启动需要现有 OCC/VTK/Qt 运行目录进入任务级 PATH，不修改系统环境。未创建目标前不运行这些命令；期望目标编译通过、CTest 返回 0。后续任务将 target/-R 替换为该项测试名；阶段结束运行所有 `-L gmsh_reproduction`，模型结束再运行相关旧回归和 GUI。

## 任务 1：先建立可观察的固定参考

文件：新增 `tests/GmshReproductionFixtures.h`、`tests/GmshReproductionTrace.h`、`tests/GmshReproductionComparison.py`；仓库外新增固定参考源码副本和构建，不修改已审查源包。

- [ ] 重新核对源包 SHA256，记录 Gmsh/OCC 编译信息和所有实际选项；保留无观察的参考程序。
- [ ] 参考侧在边密度/平滑、初始化/合法化、前沿决策/提交、原生平滑、方向/回退位置输出只读快照；不重新排序容器或插入额外 CAD 查询影响候选。
- [ ] 建立版本化轨迹：稳定 CAD ID/物理节点 ID、参数分支、mesh/background、gUU/gUV/gVV、活动边、R/h、候选、包含片、空腔及拒绝码；重试/回退另有事件。
- [ ] 检查观察开关前后相同输入的坐标/连接/实际算法及结果指纹一致；不一致先修观察，不据污染参照改自研实现。
- [ ] 冻结直线、平面矩形/带孔、窄带、非线性参数平面、圆柱、球、周期缝、embedded 和失败例子，保存阶段文件。参照使用同一份 CAD 的实际求值和离散，不只复用同名 shape ID。

验收：参考输出有算法和阶段来源，可重放；先前总片数近似的结果不替代此项。

## 任务 2：公共数据、选项和几何接入

阶段状态：生产 OccCadGeometry 的 edgeOnFaceRange/edgeOnFaceCoordinates/edgeSeamOnFace、faceTrimParameterBounds 和 projectOnFaceSurface 已接入；9 面 trueBoundary/9269 条线段、54 周期 BDS 实际几何对照零容差一致。裁剪范围按原生 ShapeAnalysis 计算，保留旧求导范围。支撑曲面直接投影新增 108 次实际原生对照逐值一致，不使用旧裁剪域最近点替代。曲率、generic/robust 反求、完整代理/调用链仍未完成，本项不勾选。

文件：新增 `SurfaceTypes.h`、`SurfaceMeshingTrace.h`、`MeshSizeEvaluator.h/.cpp`；按确需能力局部修改 `src/core/mesher/CadSurfaceMesher.h` 的 GeometrySource 与 `src/core/OccCadGeometry.h/.cpp`；新增 `GmshMeshSizeEvaluatorTest.cpp`。

- [ ] 实现模块设计中的 Options/RunContext/NodeSize/状态；复现 options 与 SurfaceCriteria 分离，未启用的原产品默认不得流入。
- [ ] `evaluateMeshSize` 对应 BackgroundMeshTools:211–280 的组合、回调、限制、倍率；`evaluateCurvatureSize` 按实体维数处理边和相邻面曲率。
- [ ] GeometrySource 仅补尺寸启用时必要的曲率/边最少分段/分支能力，缺失能力明确失败，不悄悄用旧法向差分或硬偏差代替。
- [ ] 测试点/曲率/场/实体尺寸取小，callback 前后、min/max/factor 顺序；含 disabled source、invalid size 和零曲率；和参考表比较。

验收：纯尺寸测试通过；curve 和 face 的尺寸查询可单独重放，两个状态保持区分。

## 任务 3：底层谓词、度量与曲面求交

文件：新增 `SurfaceMetric.h/.cpp`、`CurveSurfaceIntersection.h/.cpp`、`LinearSystem3.h/.cpp`；确有差异时局部修改 `src/core/mesher/GeomPredicates.h/.cpp`；新增 `GmshGeometryPredicatesTest.cpp`、`GmshSurfaceMetricTest.cpp`、`GmshCurveSurfaceIntersectionTest.cpp`、`GmshLinearSystem3Test.cpp`。

- [ ] 核对 orient/incircle 的符号与返回数值用途；sign-only 与带符号面积接口明确分开。原公共语义保持，Gmsh 特定比较放新模块。
- [ ] `evaluateSurfaceMetric` 直接形成第一基本形式；核对 Cholesky 因子与二次型；`computeMetricCircumcircle` 对应固定源码外心公式及实际重载容差。
- [ ] `solveCurveSurfaceIntersection` 对应 Numeric.cpp:685–728 的有限差分、100 次和默认步长收敛；核对实际 fullMatrix::luSolve 编译分支：本参考的 Eigen 路径使用列主元 Householder QR，不按名字替换为 LU。保留 x.norm 上限、全零残差 break 后失败和 +=h/-=h 舍入；`intersectCircleWithSurface` 对应初值与残差快速通过。普通 CAD 求交失败返回原候选状态。
- [ ] 测试近共线/共圆、大/小尺度、UV 剪切、奇异导数、圆不相交、初始残差和 Newton 停滞；比较实际决策，区别正常无交与非法 CAD 查询。

验收：非退化判据和构造与参考相符，退化差异已解释。不得先替换为“更优”物理空腔或解析线搜索。

## 任务 4：共享边完整离散

2026-10-07 阶段状态：BoundaryRegistry 三个公共函数已实现；8 个同一 BREP 常尺寸最终 0D/1D、8 ordinary-map 首/末/内段连续恢复、18 变化 callback 尺寸场生成期边链/分配数通过。10 例删除身份空洞一致；最终 Renumber 独立测试端映射已验证，正式实现归 MeshAssembly。periodic multimap/all、通用反求回退和产品几何接入未验收，以下整体清单保持未完成。

文件：新增 `CurveDiscretization.h/.cpp`、`BoundaryRegistry.h/.cpp`；新增 `GmshCurveDiscretizationTest.cpp`、`GmshBoundaryRegistryTest.cpp`。

- [ ] `integrateCurveDensity` 复现递归梯形误差、深度和累计表；`smoothCurveSizes` 前后扫尺寸，重算累计量。
- [ ] `computeCurvePointCount` 复现最少分段及 int(a+1.99)；`distributeCurveParameters` 按累计等距和表内线性插值；处理端点/闭环/退化边身份。
- [ ] `filterCurveNodes` 对应 filterPoints 的最后保留节点、首次节点查询参数例外、0.3h 弦长删除和全批最少节点门槛；保存过滤前后身份，不将积分点数称为最终边节点数。
- [ ] 保留 MEdgeVertex stored lc 的原生赋值及恢复重试的取值；meshGEdge.cpp:998 内层 d 为曲线速度，不能按外层累计目标量自行“纠正”。零长/degenerate(0/1)、闭环端点重定位和只邻一条曲线的处理分别核对。
- [ ] `buildBoundaryMesh` 对每条 CAD 边只离散一次，记录原参数、分类和使用方向；两面引用同一物理节点；保护部件身份。
- [ ] 测试 a 接近整数/非整数、变化 h、非线性参数、圆/闭环最少分段、曲线尺寸平滑和多面共享；比较整条边参数与中间累计表。

可先落地的独立测试内容（拟定新函数接收累计长度和最少段数）：

```cpp
const int first = computeCurvePointCount(10.0, 1);
const int second = computeCurvePointCount(10.02, 1);
if (first != 11 || second != 12) return 1;
if (computeCurvePointCount(0.2, 7) != 8) return 1;
```

验收：边节点数/参数/尺寸与同一配置参照一致；不继续挂旧参数二分细化。

## 任务 5：普通面初始 CDT、约束、域和初始合法化

文件：新增 `InitialTriangulation.h/.cpp`、`ConstraintRecovery.h/.cpp`、`DomainClassification.h/.cpp`、`InitialMeshLegalization.h/.cpp`；新增 `GmshInitialTriangulationTest.cpp`、`GmshConstraintRecoveryTest.cpp`、`GmshInitialMeshLegalizationTest.cpp`。

周期入口阶段补充：已新增 `DivideAndConquerTriangulation.h/.cpp` 的纯 UV 排序/递归切分/切线合并/有序环邻接，以及 `InitialTriangulation::buildPeriodicInitialTriangulation` 的原生 makeCube/3.5 扩盒/-1..-4 CAD 外围点/连接映射。48 原生分治点集及默认扰动 1e-9 的 32 实际周期入口整阶段逐值通过；输入仍冻结 1D/拓扑序/副本指针序。原版零扰动 actual 137/declared 152 未初始化尾部明确诊断，自研失败；另一零扰动嵌入线 -5 尚待定位。Qtest 原生数值乘积上下溢、约束/域/合法化及全部周期退化分支未完成，本任务不勾选。

普通入口阶段补充：`WalkTriangulation.h/.cpp` 独立保存稳定半边/面/顶点池，实现原生矩形、数值 Walk、六半边分裂、LIFO 和原始朝向 incircle 翻边。`buildInitialTriangulation` 负责早退、原插入序、1.1 扩盒。52 纯点集及 24 实际 GFaceInitialMesh(recover=0) 全池/首边/UV/身份/连接零容差一致；两上限/两尺寸场覆盖常规面和 embedded 点。CAD UV/1D/节点枚举仍冻结，完整普通 CAD 反求、recover=1/域/BDS 转换/后续合法化未验收。临时外围点保留至域模块，第一检查项仍不整项勾选。证据 `ordinary-initial-pools-oracle/walk-triangulation-report.json`。

- [ ] 固定输入顺序测试新默认入口：外围矩形、Walk、逐点插入与局部 UV 翻边；临时外围节点不能残留为输出布局。
- [ ] `recoverConstraintEdges` 分开普通 PolyMesh 的穿越/凸性/标签迁移和后续 BDS 的保护/恢复规则，报告未恢复结果；`classifyFaceDomain` 保留孔/embedded 身份。
- [ ] `legalizeInitialMesh` 对应 meshGFaceBDS.cpp:208–257：边中点度量、数组重载容差、config_modified、swapquad 防重复和 BDS_SwapEdgeTestQuality(false)。阅读 BDS 的翻边实际检查后再实现，不用旧全图 UV 规则代替。
- [ ] 测试带孔/凹域/窄带/共线链、点在边上、必须恢复的穿越链、初始参数严重畸变；保存初始/域/合法化三个阶段。

验收：约束链、标签和初始合法化合同通过；合法共圆多解记录决胜，不能据拓扑不同直接加扰动。

普通恢复阶段补充：新增 HalfEdgeTopology 只做连接改写/交换；ConstraintRecovery 已实现单副本身份映射、原生严格数值乘积相交、顶点星/穿越序/FIFO/凸性/仍穿越排回、返回码和两侧标签。780相交/33点集恢复/28 actual 普通恢复前缀逐值一致，219次点集和6次真实窄孔CAD需要翻边。原24CAD只走已有边，此覆盖缺口已补；窄孔CAD由外部脚本生成/保存完整BREP，嵌入关系显式恢复，两侧边尺寸差异明确。实际CAD输入仍冻结1D/UV/枚举/物理约束链，池端点映射自研。-1失败后继续有原生证据，-2/-3和全部不收敛退化、多副本、域/UV后置/BDS转换与BDS恢复未完整；第二检查项不整项勾选。最终证据 `ordinary-constraint-identity-oracle`，初始剖分抽出共用拓扑后 `walk-shared-topology-oracle` 52+24复核。

域/UV阶段补充（2026-10-08）：DomainClassification保留两Color的LIFO重复访问/种子选择/残余-1；InitialUvLegalization独立于BDS度量轮，保存半边池扫描/raw incircle/严格凸性、空末轮和100次诊断。72 actual阶段调用（36域+36UV）输出全池/域/访问/种子/决策逐值一致，含双孔及4次真实嵌入线UV翻边；后者会改掉前面恢复的线段，后续BDS恢复不能省略。输入1D/UV/枚举/物理链仍冻结，额外additional/多UV/所有失败退化与BDS转换/恢复/度量未完整，以上检查项仍不整项勾选。

## 任务 6：周期、缝和退化图表

阶段状态：普通 FaceChart 单元及 6 面实际 getNodeCopies 冻结 UV 重放通过；assemblePeriodicBoundaryLoop 单元及 108 次实际原生函数对照通过。PeriodicFaceChart 的边界 BDS 六档重试/跨环复用/编号/双尺寸/包围盒/counterpart 已实现，7 BREP 两尺寸场三次环重复共 54 次原生对照通过（18 原环序、36 压力重复）。sampleTrueBoundary 已实现且生产 CAD 精确查询的 9 面/9269 线段原生对照通过；新增 54 次周期 BDS 重放不冻结 UV/面坐标/范围，仍冻结 1D 网格和 CAD 环序。appendPeriodicEmbeddedChart 的新嵌入 BDS/约束/DocRecord 前缀及 buildPeriodicFaceChart 的真边界→边界/早退→嵌入→全部点序串联已实现；各 64 次实际原生对照逐值通过，完整模式独立求所有 CAD 值/边界 BDS，仍冻结 1D/拓扑/副本指针序。其中 24 次正常周期面、24 次平面强制周期入口、16 次端点 seam 压力；初始三角化/全模型未验收。多 UV 最终恢复、普通 embedded seam 分支、初始剖分、通用/robust CAD 适配器未完成，不勾整项。

文件：新增 `FaceChart.h/.cpp`、`PeriodicFaceChart.h/.cpp`；新增 `GmshFaceChartTest.cpp`、`GmshPeriodicBoundaryLoopTest.cpp`、`GmshPeriodicBoundaryLoopReplay.cpp`、`GmshPeriodicBoundaryLoopComparison.py`、`GmshPeriodicBoundaryChartTest.cpp`、`GmshPeriodicBoundaryChartReplay.cpp`、`GmshPeriodicBoundaryChartComparison.py`、`GmshPeriodicEmbeddedChartTest.cpp`、`GmshPeriodicEmbeddedCadReplay.cpp`、`GmshPeriodicEmbeddedComparison.py`；后者 `--check-full-chart` 验证完整点序串联。复用 `tests/OccSplitSurfaceFixtures.h` 中确切适用的 CAD 构造。

- [ ] `buildFaceChart` 显式边 use/branch；`buildPeriodicFaceChart` 对应 meshGeneratorPeriodic 的连续环、真边界及多 UV—单物理身份。
- [ ] 核对 meshGFace.cpp 的周期环构造、初始剖分、singular/degenerate 分类、等价点和最终恢复完整链路；实现前保存逐函数合同。
- [ ] 保留独立 buildPeriodicInitialTriangulation 的 DocRecord divide/conquer、扰动/原 BDS UV 分离和外围盒合同，不能套普通默认 Walk 入口。
- [ ] `restoreEquivalentVertices` 仅依据明确 CAD 身份，不按近距离合并；约束与两个尺寸状态要同步。
- [ ] 测试圆柱缝两侧、球极点不同 UV、跨参数周期的孔、非零参数下界、真实短边与零物理边；对照复制/恢复节点和可提交物理片。

验收：完整周期路径已可对照；不把仅球面出口不报错算此项通过。

## 任务 7：节点尺寸与前沿队列

节点部分已实施（2026-10-08）：`NodeSizeInitialization::initializeNodeSizes`及单元/普通actual全前缀对照见顶部检查点。普通节点尺寸证据已具备，周期实际对照、反求适配和后续MTri3/队列/插点仍待完成，下面检查项不整项勾选。

文件：新增 `NodeSizeInitialization.h/.cpp`、`FrontQueue.h/.cpp`；新增 `GmshNodeSizeInitializationTest.cpp`、`GmshFrontQueueTest.cpp`。

- [ ] `initializeNodeSizes` 最短物理邻边、周期等价排除、半边界段下限、embedded 点例外；不引入 fixed source 图传播和 ceiling。
- [ ] `interpolateNodeMeshSize` 用实际包含片重心坐标（FrontPointLocation已实现并通过当前叶/前缀对照）；background 每个实际新点独立 evaluate 仍待插点阶段。
- [ ] `findActiveEdge` 复现边编号约定；`popActiveTriangle` 按 R3D/h 和参考并列顺序；局部迁移和三角形失效不破坏队列。
- [ ] `markInsertionFailure` 区分找不到包含片 radius=0 与插点拒绝 radius=-1；复现它们对邻片活动资格的影响。
- [ ] 测试扩展开/关、背景与 mesh 尺寸不等、多个活动边、并列队列、被删槽复用和失败后邻域变化；对照逐步队列选择。

验收：尺寸和活动片决策可独立匹配，不添加最短边质量门槛或硬最长边优先级。

## 任务 8：候选点、空腔和前沿循环

文件：新增 `FrontPointPlacement.h/.cpp`、`FrontPointLocation.h/.cpp`、`FrontCavity.h/.cpp`、`FrontInsertion.h/.cpp`、`FrontGeneration.h/.cpp`、`FrontMeshTransfer.h/.cpp`；对应独立Test、Replay和原生Comparison。

- [x] `computeFrontPoint` 用 UV 重心度量、活动边中点、端点尺寸均值和 L=min(h*sqrt(3)*0.5,q)；`correctFrontPointOnSurface` 调任务 3 的原生求交。普通/周期自研全前缀已接，保留负高度/半径/epsilon与普通CAD失败快候选；原非有限候选的NaN/401查询仅明确诊断，实际插点失败处理仍待后续对照，不代表任务8整体完成。
- [x] `buildFrontCavity` 固定本次 metric、保护 embedded 边、定位实际包含片，不改用物理切平面。79冻结原生叶和168自研前缀的有限初始候选阶段通过；正常恢复与调用取消/异常分开。未执行外循环参数域门控、插点提交/动态反馈，不代表任务8整体完成。
- [x] `validateFrontInsertion` 对应 Euler/星形、UV面积、距离和极钝角；`commitFrontInsertion`/`insertFrontTriangles`对应insertVertexB的局部连接/队列与正常-1至-5。204冻结已分配节点/池/空腔叶逐值通过，不代表真实新节点或动态循环验收。
- [x] `insertFrontPoint` 处理实际新点CAD/BGM/分配槽、成功XYZ/身份发布及外层正常拒绝恢复/失败半径。1560同BREP actual insertAPoint叶通过；普通/周期自研前缀已接首候选对照。失败前已经append的参数/尺寸槽与已消费身份保留，可引用节点只在成功时发布；取消/预算/查询异常恢复入口。首提交在测试副本中隔离，不代表FrontGeneration动态反馈或任务8整体完成。
- [x] `generateFrontMesh` 只循环队列—候选—可选参数域门控—空腔—插点—邻域更新；插点成功后不接旧全图合法化/坏片修补。280初始池循环/504域叶通过；普通/周期自研初始化前缀已在两extension共168组完整动态运行逐值一致（161新片反馈输入）。1D/拓扑/反求仍冻结，任务8整项未验收。
- [x] `transferFrontMesh`独立复现原All顺序、朝向、等价重复诊断、单次副本映射和身份退化过滤，不加入去重。146actual叶+168结束池重放及168连续驱动逐值通过；原运行池/临时副本/ghost释放交编排层，不替代后续平滑。
- [ ] 对照固定边界平面、非线性 UV、圆柱/球、窄带汇合、正常拒绝与强制失败；定位首次候选/空腔分歧，分别导出前沿结束网格。

验收：冻结非退化输入逐次决策相符；其他差异有对应节点/阶段/原因，不能仅凭近等边比例通过。

## 任务 9：Gmsh 原生平滑

文件：新增 `SurfaceSmoothing.h/.cpp`、`MeshVertexParameters.h/.cpp`、独立`OccMeshVertexParameters.h/.cpp`、对应unit/CAD Replay/原生Comparison及共享平滑观察JSON。

- [x] `computeSmoothedPosition` 对应 _relocate 的度量加权邻 UV；边界/embedded 分类和边界层外的本范围固定节点不动。核心资格/控制已单元验证，原CAD参数适配仍由下一子项完成。
- [x] `acceptSmoothedPosition` 对应最差 etaShapeMeasure 和 UV 面积；`smoothSurfaceMesh`固定执行五次FACTOR/=1.4试探，即使已成功也继续，trial基于本轮before/after，actual跟随成功位置。默认一轮，验收和成功发布分别查询GPoint。23预期RED→GREEN、180同面MFaceVertex actual零容差通过，不是完整平滑集成。具体合同见surface-smoothing-source-contract.md。
- [x] 测试一次候选接受/缩步/全部拒绝、周期 reparam 邻点、固定边界和轮数 0/1；输出平滑前后节点、连接和质量。64原始参数及120冻结原未平滑网格actual通过，追加0/-1/1/2轮与第二次求值失败。
- [x] 自研前沿/转移结果连续接原始参数源与平滑；两extension共168组逐值一致，周期临时副本保留到平滑结束，原参数不被运行UV覆盖。合同见surface-smoothing-prefix-source-contract.md。
- [ ] 验证周期临时对象在平滑后按原所有权释放；补直接投影失败/generic反求及未覆盖分支，再完成整模型验收。

当前168自研前沿/转移连续平滑逐值通过，另64原始参数及120真实分类平滑当前回归通过；早前180同面叶为历史证据。独立OCC参数源/pcurve/投影/metric/point，较早1D/拓扑/raw CAD图表与前沿尺寸UV仍冻结；generic失败、周期清理/装配和全模型质量待验证，任务9整项不勾选。45复刻+18既有CTest/24Python、8中立性通过。Sphere退化边人为1D stored0在原生报null curve projection，失败记录与非零stored路径分开，不捏造参数修补。

验收：不是项目原 shapeOptimizationSweeps=4；平滑对照失败回到本模块查原因。

## 任务 10：几何重试和 MeshAdapt 自动回退

文件：新增 `FaceMeshingFallback.h/.cpp`、`SurfaceMeshAdaptation.h/.cpp`；任务 4 BoundaryRegistry 增加 `refineUnrecoveredEdges`；新增 `GmshBoundaryRecoveryRetryTest.cpp`、`GmshFaceMeshingFallbackTest.cpp`、`GmshSurfaceMeshAdaptationTest.cpp`。

- [ ] 边恢复失败按固定 maxRetries、未恢复边和 incident face 重试，记录版本和节点更新；不套用旧质量反馈。
- [ ] `checkFaceOrientationConsistency` 对应 TRIANGLE_VALIDITY/isMeshValid 和调用限制；`shouldUseMeshAdaptation` 不把同一面全部反向误判为混合方向。
- [ ] 逐函数核对 meshGFaceBDS.cpp 的 computeNodalSizes、split/collapse/swap/smooth、退化点、finalize 和停止规则；保存合同后实现。已核对主循环使用 0.7/1.4 归一化边阈值、首轮两次 split 和最后 .45 collapse，不能据这些常数直接替代全部 BDS 检查。
- [ ] `adaptSurfaceMesh` 按原调用两次 refineMeshBDS 与实际周期分支串联任务顺序，保护 embedded/边界身份，实际算法输出 MeshAdapt。
- [ ] 测试失败清理后边节点保留、算法临时切换后恢复配置、回退开/关、混合方向触发、退化和短边 collapse；用参考重试/切换日志证明走过目标分支。

验收：自动回退由本项目实现且可对照；未完成本任务仅称纯算法 6 阶段完成。

## 任务 11：编排、组装和独立验收入口

文件：新增 `SurfaceGenerator.h/.cpp`、`MeshAssembly.h/.cpp`；新增 `tests/GmshSurfaceReproductionBenchmark.cpp`、`tests/GmshSurfacePipelineTest.cpp`；修改 `CMakeLists.txt`，必要时接入 `src/core/SurfaceMeshEngine.h/.cpp` 独立实验入口；不修改默认生产路线。

- [ ] `generateFaceMesh` 编排普通/周期初始、尺寸、前沿、平滑、方向判定和回退；`generateSurfaceMesh` 管共享边/模型级重试。
- [ ] `assembleSurfaceMesh` 恢复共享身份、标签/方向/节点压缩；不优化点阵，不合并不同部件接触节点。
- [ ] 基准显式参数 `--cad`、`--out`、`--size-min`、`--size-max`、`--size-factor`、`--size-from-points`、`--size-from-curvature`、`--extend-boundary-size`、`--smoothing`、`--switch-on-failure`，完整记录其他默认；不存在隐式产品判据换算。
- [ ] 分开 GenerationResult 与 CAD/拓扑 AuditResult；预算/取消结果禁止标作完整网格；生成基线未通过产品精度时保存原因，不调用旧细化。
- [ ] 比较全部阶段、shared edge/部件身份、保存重开字段；核心静态库检查没有 Gmsh 链接或 SDK 隐式调用。

验收：入口只编排，上述模块可独立运行，观察开关不改变输出。

## 任务 12：完整范围复核与最终报告

文件：复用 `tests/CadSurfaceQualityReport.h`、`tests/CadSurfaceQualityAuditTest.cpp` 的独立测量；确有新元数据时局部修改 `src/core/ProjectArchive.cpp` 和对应测试；更新 ARCHITECTURE.md/CONTEXT.md/研究证据，保存结果至仓库外。

- [ ] 全 `gmsh_reproduction` 单元/集成通过；同原 CAD、实际参数、串行算法路径比较 phone 和凹球；必要的参考观察构建单独说明。
- [ ] 保存前沿前后/平滑前后/最终结果；比较区域密度、边长/h、全角度、近等边与面积加权指标、边界窄域/内部以及拓扑。近似总片数不能作为唯一标准。
- [ ] 原 CAD 双向有限采样、法向、部件/共享边及相交审计单独报告；不宣称有限采样构成全局精度证明。
- [ ] 实验入口若进入 UI，执行真实窗口生成、取消、保存重开和显示验证；仅构建/CTest 不等于 GUI 验收。
- [ ] 同计时边界串行三轮记录性能，质量/参数不一致时不报速度倍数；差异未解释则继续对应模块研究。
- [ ] 逐行核对设计范围，周期/退化/embedded/重试/回退都有证据且没有未解释布局差异后，才报告完整复现完成；此后另开自有优化阶段。

## 执行时应停下来说明的情况

2026-10-08 BDS转换阶段补充：新增BdsTypes/BdsTopology/BdsMeshConversion及单元/纯重放/actual观察接口。24实际原位拓扑+36实际普通全初始前缀转换+32实际周期CAD完整图表/初始/转换逐值一致；周期另复核64嵌入/64图表及32初始。普通冻结1D/UV/枚举/独立BDS图表与四CAD查询值，周期只冻结1D/拓扑/副本序。全端点复用的嵌入边原版仍登记几何分类，实际对照暴露遗漏后增加独立登记并传到转换；不靠节点分类反推。Task5/6仍不勾选：BDS恢复/域/度量合法化、additional/多UV、全部失败/退化等仍未完成。证据ordinary-bds-conversion-exports-oracle、periodic-bds-classification-oracle；最终复核及构建记录见tests/GmshReproduction.md。

改变用户已确认的范围、需要删除/大范围重构/创建分支、或要改变生产默认时先说明具体变更。普通源码核对、局部实现和必要测试不逐步请求确认。遇到模块证据不符应停在该阶段排查；不能放宽阈值、混入旧生成器或切到创新算法继续过关。

2026-10-08 BDS单边恢复阶段：新增BdsConstraintRecovery（sys2x2/闭区间、资格/原池扫描/首个合格交换、CAD自交双方和fatal诊断）；连接更新归BdsTopology的getBdsEdgeNeighborhood/swapBdsEdgeTopology，旧记录deleted和原追加序/分类继承保留。720数值+136叶调用（50合成+86冻结CAD初始BDS）全池/逐扫描/尝试零容差一致，含2次成功交换后fatal。真实RED11→GREEN；40Release/33CTest（23复现）/8Python与旧转换阶段回归通过。证据bds-recovery-leaf-final-oracle。Task5/6仍缺实际普通/周期恢复驱动、分类pass/域删除/度量、ix>300全退化等；普通pass1登记boundary+embedded，pass2仅boundary，前期UV嵌入线压力不能证明pass2恢复该线段。严格继续原序核对，不先补自有修复策略。

本计划已进入实施。尺寸、曲线、边界、度量/QR/求交及谓词接口已有分阶段实现和对照；普通图表、周期环/边界/嵌入 BDS、真边界及全部周期初始点序已有单元和原生阶段证据。完整几何接入、多 UV 最终恢复、初始剖分、前沿、平滑、回退和组装仍未完成，全部 12 项任务保持未整体验收。具体证据和失败诊断差异记录在 `.planning/2026-10-07-gmsh-reproduction-research/`。

2026-10-08普通域前boundary阶段补充：BdsCurveRecovery完成物理inverse映射、boundary再embedded约束登记和域前boundary逐曲线恢复/分类/空端点localID标记；非fatal结束该曲线后继续，fatal结束boundary，缺映射跳段，refineAll/未恢复交后续重试。单元RED11→GREEN，36actual/2445段全池/登记/分类/尝试逐值通过，初始前缀和连接自研，1D/CAD图表值/曲线拓扑元数据仍冻结。本矩阵全部boundary已有边，不能称全部失败/翻边driver通过。41Release/34CTest/8Python及叶/转换回归通过，证据ordinary-bds-boundary-release-oracle。

完整顺序修正：前条pass2仅boundary只指域前；普通源码在域后另对embedded调用pass2，返回bool被原版忽略，需留诊断且保持顺序。任务5/6下一步按域标记→embedded恢复→节点双尺寸→清理→度量合法化，不提前跳过/合并。临时CLASS_F/CLASS_EXTERIOR指针身份及std::partition非稳定池顺序也属复现合同，非额外创新。全部失败/周期/几何/前沿等与整体验收仍未完成。

2026-10-08域与后置embedded已接入：BdsDomainClassification真实RED8→GREEN、embedded恢复RED4→GREEN；各36actual全前缀一致，域9588访问/4886重复、embedded38段/4真实翻边。42Release/35CTest（25复现）/8Python及前阶段回归通过，证据ordinary-bds-domain-release-oracle/ordinary-bds-embedded-release-oracle。仅普通单副本分支的阶段证据，CAD/1D/图表元数据仍冻结；任务5/6仍缺BGM双尺寸/清理/度量、周期恢复与剩余图表/失败分支，不勾选整项。下一步独立实现BDS节点BGM初始化与清理；它们先于度量合法化，GFace前沿节点尺寸初始化在BDS转换到GFace后。
