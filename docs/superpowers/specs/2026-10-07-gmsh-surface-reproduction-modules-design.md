# Gmsh 表面生成流程复现：模块和命名

2026-10-08当前：普通/周期自研初始化前缀→FrontGeneration→FrontMeshTransfer→SurfaceSmoothing共168组零容差通过；模块表已同步连续平滑几何源与所有权。CAD源、参数选择、移动验收、连接转移和编排分别负责。较早1D/拓扑/raw CAD图表与前沿尺寸UV、generic失败、周期清理/装配和整模型验收仍未完成。合同见surface-smoothing-prefix-source-contract.md，完整目标保持active。下文旧检查点为历史。

2026-10-08当前：FrontInsertion的单点生命周期已经实现，模块表按当前函数职责更新。1560实际insertAPoint叶与普通44/周期40首候选前缀通过；首提交在测试副本中隔离，不能用它代表完整动态循环。FrontGeneration下一步只负责编排pop→候选/修正→可选真参数域门控→单点插入/反馈，参数域严格穿越另立模块；几何查询、局部拓扑和队列职责保持独立。

2026-10-07。用户已确认独立复现入口，并要求模块职责、函数命名明确。目标先完整复现固定 Gmsh 方法，再考虑额外优化。本文规定实现边界，不代表已经实现。

最新节点阶段（2026-10-08）：普通FaceMeshConversion整驱动及NodeSizeInitialization已接初始/恢复/域/尺寸/清理/度量全前缀，纯重放与独立CAD求导版本各44actual逐值通过。前沿节点3074、embedded BGM16、内部约束38、不同双尺寸16。NodeSizeInitialization独立于BdsNodeSizeInitialization：前者按初始物理邻边重置尺寸，后者在清理前按原MVertex实体求BGM。节点遍历序显式输入；每节点先CAD反求、仅embedded原节点用面实体/raw UV求BGM，再在数据入库时覆盖周期UV。MTri3/邻接/队列和插点阶段不在此模块；周期节点实际对照和前沿反求适配尚未验收，当前仍冻结其原生指针序/反求UV。以下为历史。

最新实施（2026-10-08）：FaceMeshTypes、PeriodicSeamReconstruction、FaceMeshConversion已按下表职责实现。40actual周期CAD前缀的临时副本/UV/等价映射/连接逐值一致，24原生BDS2GMSH叶补齐创建与过滤顺序；原1D元数据/拓扑/副本序仍冻结，临时和新节点以创建序归一身份，不比较进程全局编号。普通完整转换驱动、失败重建、完整前沿/后处理/组装及最终形状质量未验收，以下为历史记录。

2026-10-08最新周期初始度量：同一legalizeInitialMesh已接周期独立CAD全前缀，40actual/3654查询/246交换/63扫描/124资格拒绝全池和决策逐值通过；不另造周期翻边器。几何适配只求D1，SurfaceMetric构造度量，InitialMeshLegalization扫描/去重/圆内判断，BdsSwapQualification决定资格，BdsTopology改连接。仍冻结1D/拓扑/副本序/物理链；下一步需独立表示周期缝临时副本及其UV/等价映射，再BDS转前沿网格。失败重建/完整前沿/后处理/回退/组装和最终质量未完成，以下历史状态按本段更新。

2026-10-08最新周期域/清理检查点：周期域与普通域共享LIFO实现，但外域(3,2)、跳过deleted负点种子且不补负对点；清理统一为prepareBdsInterior，普通旧名作为兼容入口。32actual完整自研CAD/图表/初始/BDS/恢复/域/清理逐值通过；只冻结1D/拓扑/副本序/物理曲线链。周期度量、失败1D重建和完整生成仍未完成，以下历史状态按本段更新。

2026-10-08当前实施范围已推进到普通BDS初始度量合法化。资格与扫描分别落在BdsSwapQualification/InitialMeshLegalization，44actual普通面全前缀的4131度量查询/776翻边/82扫描逐值一致；独立CAD支撑面D1重放也一致。对外几何适配新增`OccCadGeometry::faceSurfaceDerivatives`，与通用BRepAdaptor求导分开，保留原版B样条舍入。周期恢复、前沿、后处理、回退、组装和最终质量仍未完成；本文下方较早阶段记录按本行更新。

## 目标范围

最新周期实施：PeriodicBdsRecovery已接完整自研CAD/图表/初始/BDS前缀，32actual恢复阶段逐值通过，80embedded调用/2012环边，0缺边/0翻边/0失败/0重试。相关缺边/失败控制单元RED9→GREEN；周期域/清理和失败1D重建仍待实现，不称周期完整验收。本文下方早期状态按本段及上面的普通度量检查点更新。

固定 Gmsh 4.15.2 的 OCC、一阶三角形、串行算法 6 表面流程：尺寸求值、共享边离散、普通/周期初始网格、约束恢复和域分类、初始度量合法化、前沿布点及插点事务、原生平滑、方向处理、边恢复失败重试和 MeshAdapt 自动切换。曲率、背景尺寸、embedded 点/边属于该流程的配置分支，要覆盖。

不是复现 Gmsh 所有算法、GUI、体网格、高阶或边界层；compound 属于独立跨面生成路线，普通路径复现后单独安排。用户允许跨光滑 CAD 分割边的偏好保留，建立基线时保持与 Gmsh 普通逐面输入相同。

完整范围包含周期和失败回退，不能为了提前完成而关掉它们。阶段中可关闭回退验证纯算法 6，但必须标记为阶段结果。生产默认不随研究阶段自动切换。

## 目录及依赖

算法文件放在 `src/core/mesher/gmsh_reference/`，命名空间 `mesher::gmshReference`。这表示参考方法，不链接 Gmsh；底层计算由本项目实现。几何求值继续通过现有 GeometrySource/OccCadGeometry。Gmsh 只存在于仓库外参考程序和测试中。

只复用经固定源码对照通过的底层计算。当前前沿使用独立稳定池，不依赖旧Mesh2DTransaction；原生拒绝前已append的参数/尺寸槽必须保留，不能套用旧事务的紧缩全回滚。新目录不调用旧 generateCadSurfaceMesh、SurfaceBoundarySizeField、SurfaceQualityPolicy、SurfaceMeshingPartition，也不进入旧的共享边质量重建循环。现有文件保留，避免一次拆改旧流程。

上层分工：

```text
SurfaceMeshEngine / 测试工具
           ↓ 显式选择复现入口
SurfaceGenerator（阶段编排）
 ├─ MeshSizeEvaluator → CurveDiscretization → BoundaryRegistry
 ├─ FaceChart / PeriodicFaceChart → InitialTriangulation
 │   ├─ 普通 PolyMesh：WalkTriangulation → ConstraintRecovery → DomainClassification → InitialUvLegalization → BdsMeshConversion
 │   └─ 周期 DocRecord：DivideAndConquerTriangulation → BdsMeshConversion
 ├─ 普通 BDS：约束登记 → boundary恢复 → BdsDomainClassification → embedded恢复
 │   └─ BdsNodeSizeInitialization → BdsInteriorPreparation → InitialMeshLegalization（BDS度量）
 ├─ 周期 BDS：PeriodicBdsRecovery（embedded→边界环）→classifyPeriodicBdsDomain→prepareBdsInterior→度量合法化，按源码顺序
 ├─ FaceMeshConversion::initializeFaceMeshVertices（原MVertex元数据）
 │   └─ 周期专有：PeriodicSeamReconstruction（临时副本/UV/等价身份）
 ├─ FaceMeshConversion::convertBdsToFaceMesh → NodeSizeInitialization（前沿节点尺寸）
 ├─ FrontTriangleInitialization → FrontGeneration
 │   ├─ FrontQueue
 │   ├─ FrontPointPlacement → SurfaceMetric / CurveSurfaceIntersection
 │   ├─ FrontPointLocation（包含片/mesh尺寸插值）
 │   └─ FrontCavity（固定metric空腔）→ FrontInsertion（新片验收/局部提交）→ FrontQueue局部更新
 ├─ SurfaceSmoothing
 ├─ FaceMeshingFallback → SurfaceMeshAdaptation（仅满足切换条件时）
 └─ MeshAssembly
           ↓ 不改变已生成节点和连接
独立 CAD/拓扑/形状审计
```

BoundaryRegistry 是复现路径内的共享边和节点状态，不复用旧模块的质量分点调度。几何边恢复重试仍可更新它；更新原因和关联面必须显式记录。

## 模块职责和函数名

下表文件均位于新目录，同名 `.h/.cpp`；`SurfaceTypes.h` 为纯数据定义，`SurfaceMeshingTrace.h` 为观察接口。只有模块有完整可测职责时才建文件，不额外引入通用策略工厂或插件框架。

| 模块文件 | 唯一职责 | 对外或模块内明确命名的核心函数 |
|---|---|---|
| SurfaceGenerator | 顺序编排、取消/限制、失败分支；不计算几何公式 | `generateSurfaceMesh`、`generateFaceMesh` |
| MeshSizeEvaluator | 按 Gmsh 顺序组合点/曲率/场/实体尺寸，限制和倍率 | `evaluateMeshSize`、`evaluateCurvatureSize` |
| CurveDiscretization | 速度/h 积分、曲线尺寸平滑、整数分段、累计表反解、节点尺寸元数据和过近点过滤 | `integrateCurveDensity`、`smoothCurveSizes`、`computeMinimumCurveSegments`、`computeCurvePointCount`、`distributeCurveParameters`、`computeStoredCurveSize`、`filterCurveNodes` |
| BoundaryRegistry | 每条 CAD 边只有一份离散；共享节点身份和恢复重试版本 | `buildBoundaryMesh`、`refineUnrecoveredEdges`、`findIncidentFaces` |
| FaceChart | 普通面物理闭合/UV/分类/插点顺序、周期环纯 UV 组装和物理等价身份恢复 | `buildFaceChart`、`assemblePeriodicBoundaryLoop`、`restoreEquivalentVertices` |
| PeriodicFaceChart | 周期面图表；边界初始化、真边界、embedded 的多 UV 对应/分类/双尺寸及初始点序 | `sampleTrueBoundary`、`buildPeriodicBoundaryChart`、`appendPeriodicEmbeddedChart`、`buildPeriodicFaceChart` |
| InitialTriangulation | 普通/周期入口早退、扩盒及连接/身份编排，不实现剖分公式 | `buildInitialTriangulation`、`buildPeriodicInitialTriangulation` |
| WalkTriangulation | 普通初始矩形和稳定半边池、Walk/分裂/局部翻边 | `buildWalkTriangulation`、`locateInitialTriangle`、`splitInitialTriangle`、`swapInitialEdge` |
| HalfEdgeTopology | 有限 UV/池索引验证、三角循环改写和对角线交换；不判断翻边资格或控制迭代 | `validateHalfEdgeTopology`、`rewriteHalfEdgeTriangle`、`swapHalfEdgeDiagonal` |
| DivideAndConquerTriangulation | 周期初始入口的纯 UV 递归切分、切线合并、有序邻接和连接输出 | `buildDivideAndConquerTriangulation` |
| ConstraintRecovery | 普通PolyMesh约束身份转换、追踪穿越边、FIFO恢复和返回码；不承担BDS保护规则 | `prepareInitialConstraints`、`intersectRecoverySegments`、`recoverConstraintEdges`、`collectCrossedEdges` |
| DomainClassification | 根据恢复好的约束标记内域、外域及孔 | `classifyFaceDomain` |
| InitialUvLegalization | 普通 PolyMesh 域分类后的池序/有界 raw UV 翻边，保留标签迁移和上限诊断 | `legalizeInitialUvMesh` |
| BdsTypes / BdsTopology | BDS 分类、稳定节点ID、边/面池、原序双向邻接与连接交换；不判断几何操作资格 | `insertBdsPoint`、`findBdsEdge`、`appendBdsTriangle`、`getBdsTriangleNodes`、`validateBdsTopology`、`getBdsEdgeNeighborhood`、`swapBdsEdgeTopology` |
| BdsMeshConversion | 普通全PolyMesh面和周期DocRecord连接转换，独立坐标/分类登记/身份保持 | `convertOrdinaryInitialMeshToBds`、`convertPeriodicInitialMeshToBds` |
| BdsConstraintRecovery | BDS数值求交、恢复翻边资格、单次恢复扫描及失败诊断；不分类CAD边、不编排外层pass | `intersectBdsRecoverySegments`、`canSwapBdsRecoveryEdge`、`recoverBdsConstraintEdge` |
| BdsCurveRecovery | 普通约束登记、域前boundary和域后embedded恢复、曲线/空端点分类及失败诊断；不着域/清理 | `buildOrdinaryBdsRecoveryRegistry`、`recoverOrdinaryBdsBoundary`、`recoverOrdinaryBdsEmbedded` |
| PeriodicBdsRecovery | 周期embedded先行、边界环恢复、临时CLASS_E/缺边CAD曲线和重试诊断；不着域/重建1D/补端点分类 | `recoverPeriodicBdsConstraints` |
| BdsDomainClassification | 普通/周期BDS临时内/外域分类身份、LIFO访问；保留各自deleted种子与补种子规则，不删片/清理 | `classifyOrdinaryBdsDomain`、`classifyPeriodicBdsDomain` |
| BdsNodeSizeInitialization | BDS阶段原MVertex实体/物理XYZ/曲线参数的BGM查询与双尺寸赋值；不插值前沿尺寸/清理 | `initializeOrdinaryBdsNodeSizes` |
| BdsInteriorPreparation | 普通/周期共有的空域面/孤边删除、两次非稳定cleanup和全池引用映射、分类传播、四外围点删除；不翻边/恢复/求尺寸 | `prepareBdsInterior`（普通旧名兼容委托） |
| BdsSwapQualification | 初始Quality(false,true)的原始参数面积、低连接度/缝/极点/面积守恒资格；不求度量、不改连接 | `computeBdsParameterTriangleArea`、`canSwapInitialBdsEdge` |
| InitialMeshLegalization | 对应delaunayizeBDS的动态池扫描、四节点配置去重和边中点度量圆；交换委托BdsTopology | `legalizeInitialMesh`、`shouldFlipInitialEdge` |
| FaceMeshTypes | 前沿前节点身份、局部映射、临时副本等价、各副本UV及原序三角连接；不表示最终物理编号 | `FaceVertex`、`FaceMesh` |
| PeriodicSeamReconstruction | 重复0D/1D原节点复制及UV/等价映射；不改BDS、折叠副本或生成三角形 | `reconstructPeriodicSeamVertices` |
| FaceMeshConversion | 绑定原MVertex元数据，按BDS活面池序创建/复用节点，再按节点身份过滤三角形；不求CAD/尺寸 | `initializeFaceMeshVertices`、`validateFaceMeshVertices`、`convertBdsToFaceMesh` |
| NodeSizeInitialization | 已实现前沿物理邻边尺寸、embedded双尺寸与内部约束、boundary半段下限、逐节点反求及周期UV覆盖；不构造MTri3/邻接/队列 | `initializeNodeSizes`；实际包含片插值属于FrontPointLocation |
| SurfaceMetric | 第一基本形式、度量长度/外心、3D 外接圆半径；不决定生成尺寸 | `evaluateSurfaceMetric`、`computeMetricCircumcircle`、`computePhysicalCircumradius` |
| CurveSurfaceIntersection | 固定 Gmsh 有限差分 Newton 的圆/曲面修正 | `intersectCircleWithSurface`、`solveCurveSurfaceIntersection` |
| LinearSystem3 | 对应参考构建实际 Eigen 分支的三阶列主元 QR；仅数值求解，不控制求交停止 | `solveColumnPivotedQr` |
| FrontQueue | 活动边资格、R/h 排序、并列决胜、初始扫描/登记、局部活动更新和失败标记 | `findActiveEdge`、`initializeFrontQueue`、`popActiveTriangle`、`registerFrontTriangles`、`updateActiveTriangles`、`markInsertionFailure` |
| FrontPointPlacement | 活动边的度量候选和原生圆修正；不插点 | `computeFrontPoint`、`correctFrontPointOnSurface` |
| FrontPointLocation | 原UV反映射、到重心的首跨边搜索、显式force扫描与包含片mesh尺寸插值；不求CAD/BGM或修改网格 | `mapFrontPointToTriangle`、`intersectFrontSearchSegments`、`findContainingFrontTriangle`、`interpolateNodeMeshSize` |
| FrontCavity | 固定本次度量的DFS、壳边原序、内部边保护与包含片；只修改/恢复deleted，不分配新点或提交连接 | `buildFrontCavity`、`rollbackFrontCavity` |
| FrontInsertion | 前三个函数处理已分配节点的insertVertexB验收/局部连接/队列更新；insertFrontPoint组合空腔与显式查询源，管理CAD/BGM顺序、身份、分配槽和发布/正常失败，不选片或调度全循环 | `validateFrontInsertion`、`commitFrontInsertion`、`insertFrontTriangles`、`insertFrontPoint`；已接共有动态循环 |
| FrontGeneration | 循环串联队列、候选和插点；不重新实现各模块公式 | `generateFrontMesh` |
| FrontMeshTransfer | 原All序输出、统一朝向、等价重复诊断和单次身份映射；不平滑/去重/释放运行池 | `transferFrontMesh` |
| SurfaceSmoothing | 原自由内点度量加权位置、面积/最差eta临时验收恢复、固定五次缩步；成功后继续试探 | `smoothSurfaceMesh`、`computeSmoothedPosition`、`acceptSmoothedPosition`；168自研前沿/转移连续平滑对照通过，原始CAD失败与最终验收待补；质量叶`computeTriangleEta` |
| MeshVertexParameters | 原节点候选序、单点参数、邻边两端分别参照对端首候选；不复制前沿槽UV | `collectMeshVertexParameters`、`reparameterizeMeshVertexOnFace`、`reparameterizeMeshEdgeOnFace`；64 actual逐值通过，generic反求仍待补 |
| OccMeshVertexParameters | 独立CAD适配，重建indexed拓扑/incident序、pcurve、直接曲线/支撑面投影；组合平滑几何源 | `makeOccMeshVertexParameterSource`、`makeOccSurfaceSmoothingSource`；不链接入纯mesher，168连续平滑通过，不承担移动/连接/通用反求回退 |
| FaceMeshingFallback | Gmsh 方向一致性判定、清理失败结果和算法切换决策 | `checkFaceOrientationConsistency`、`shouldUseMeshAdaptation` |
| SurfaceMeshAdaptation | MeshAdapt 回退本身的尺寸、split/collapse/swap/smooth 和退化处理 | `adaptSurfaceMesh`、`splitLongEdges`、`collapseShortEdges`、`swapAdaptationEdges`、`smoothAdaptationVertices` |
| MeshAssembly | 合并面结果、恢复共享身份、方向与 CAD/部件标签；不改变布局质量 | `assembleSurfaceMesh`、`orientSurfaceMesh` |
| SurfaceMeshingTrace | 输出阶段和单步决策；只读算法状态 | `recordStageMesh`、`recordFrontDecision`、`recordAlgorithmSwitch` |

对照源码锚点见 [源码研究](../plans/2026-10-07-gmsh-generation-reproduction-research.md)。本文模块有意拆开两个不同的翻边时机，以及 FrontGeneration 的循环与 SurfaceGenerator 的模型编排。

实施时确认 `fullMatrix::luSolve` 在固定 Eigen 构建中实际调用列主元 Householder QR，因此新增独立数值辅助模块 LinearSystem3；不引入 Eigen 依赖，也不按函数名替换为 LU。

## 周期图表的内部职责边界

`assemblePeriodicBoundaryLoop` 只对应固定 meshGFace.cpp:2091–2303 的 UV 方向/缝选择和闭合部分。输入保持 GEdgeLoop 的边序，各边为 begin / 活内点 / end；几何适配器使用 `parBoundsOnFace` 和 `reparamOnFace` 的方向 +1/-1，不能误用普通 getNodeCopies 的 0/1 分支。函数不调用 CAD、不分配 BDS、不设置尺寸，不合并物理身份。等距最小候选按原生规则失败，续接失败尝试首边反向，闭合失败不额外反向重试。

`PeriodicFaceChart.h/.cpp` 的 `buildPeriodicBoundaryChart` 已负责六档容差及两种首缝选择、跨环 0D 节点复用、BDS 编号/分类/双尺寸、raw UV 包围盒、periodicCounterpart 和边界点数早退。原版只在以前已完成环的 recoverMap 中复用同物理 0D 且 UV 距离小于容差的节点；同一当前环内不能提前合并。计数按环位置增加，复用后可留下 BDS 编号空洞。counterpart 是逐节点覆盖的单对应指针，不能自行替换为全部复制节点互连。

`appendPeriodicEmbeddedChart` 只产生新增嵌入 BDS 点、嵌入约束及 DocRecord 点前缀，不修改传入边界。原生边界 inverse map 在此阶段冻结；独立嵌入 CAD 点与嵌入边端点可以共用物理身份但仍分别分配 BDS 点。负分类 tag 属于创建它的 CAD 点/边，BGM 查询仍使用原物理节点分类。多周期副本按端点距离选参数，再按原生副本指针序找首个 UV 匹配，不按最近 UV 自行修复。原生 seam 选择失败、facile 空覆盖和预估/实际点数差异明确记录。

`buildPeriodicFaceChart` 顺序串联真边界、边界 BDS/早退、嵌入和全部 DocRecord 点序，统一查询预算并原子发布。DocRecord 扰动坐标与 BDS raw UV 分开，点序为嵌入前缀后接逐环所有位置，保留重复引用；General 路线必须显式提供运行上下文的扰动流，即使系数为零也保留随机调用。原生 seam 错误导致预分配数量不足时明确拒绝，避免复刻越界写；不得补点或丢约束掩盖差异。64 组实际原生嵌入阶段及 64 组完整图表串联对照逐值通过，但仍冻结 1D 网格、CAD 拓扑序和副本指针序。其中 24 组为正常圆柱/球/环面周期输入、24 组平面输入强制调用周期私有入口、16 组为端点 seam 压力输入；不代表初始三角化或全模型生成已通过。

`sampleTrueBoundary` 已独立实现原生去重边遍历序、300 参数样本和 0/1 分支合同。它不读取实际 1D 网格，不组装连续环、不分配 BDS；debug 导出由观察层负责。几何适配器的 `edgeOnFaceRange/edgeOnFaceCoordinates/edgeSeamOnFace` 提供精确 pcurve 查询；其中 reversed 相对全局边原朝向。`faceTrimParameterBounds` 按 `ShapeAnalysis::GetFaceUVBounds` 给出图表尺度，与原导数求值器范围分开；不能用扩大容差掩盖两者的舍入差异。通用/robust 反求仍需独立复现，缺失精确能力明确失败。

`GeometrySource::projectOnFaceSurface` 对应 OCCFace::closestPoint 的直接投影路径；支撑曲面和孔/裁剪域的最近点查询职责不同，不能复用旧 closestTrimCoordinates。OCC 生产实现使用原生裁剪范围、仅扩大非周期轴、惰性独立 projector，失败保留输出。108 次同 BREP 实际原生投影逐值通过；无投影时的 generic 反求尚未复现。

`InitialTriangulation` 需要保留两个入口：普通默认入口为 GFaceInitialMesh 的矩形 + Walk/split；周期入口仍为 DocRecord 的 divide/conquer。周期 DocRecord 的随机扰动坐标与原 BDS UV 分开保留，外围盒构造规则也不同。不能因为都采用 Delaunay 就将两条路径统一成旧 CDT，或提前改为更好的初始化。

普通入口已拆出纯 UV 的 `WalkTriangulation`，数据用稳定池索引对应原生 PolyMesh 指针：每个点的入射边、面的可变首边、半边 prev/next/opposite/face 和标签保留。`InitialTriangulation::buildInitialTriangulation` 只处理原生早退、单副本身份校验、getNodeCopies 原顺序和 *=1.1 的舍入；不调用 CAD。Walk 使用 orient2d 数值大小选择跨边，插点后下次定位从已修改的同一 Face 起步；六条半边和两新面追加序、LIFO/已处理边集合及原始朝向 incircle 判据不变。52 原生点集和 24 实际 GFaceInitialMesh(recover=0) 逐池零容差通过，CAD UV/1D/节点枚举输入仍冻结。recover=1 内的恢复/着色/后置 UV 翻边以及到 BDS 的转换尚待接通；这些步骤不能与后续 delaunayizeBDS 度量合法化混为一轮。

后续普通恢复前缀已接通：HalfEdgeTopology 抽出同一 createFace/swap_edge 连接更新，Walk 插点和 ConstraintRecovery 恢复共用，保持池/首边/标签写入序。prepareInitialConstraints 对单副本 FaceChart 按 localId→物理ID→池索引转换，原序保持。collectCrossedEdges 先绕顶点星，再沿原生两候选边严格追踪；恢复用FIFO，对凸四边形翻边，仍穿越则排回。intersectRecoverySegments 使用原生 orient2d 数值乘积 >=0 规则，端点/共线及下溢负零不算相交。此阶段 CAD 半边标签不是翻边锁；边标记会随池对象交换迁移，后续 BDS 恢复/保护规则仍需另验。

原生 -1/-2/-3 记录后继续；静止队列（原版无法进展的循环）、非法拓扑、取消/预算原子失败。780 相交、33 点集恢复、28 actual GFaceInitialMesh 恢复前缀逐值通过；219次纯点集和新增窄孔CAD6次均实际需翻边，CAD覆盖21条初始穿越。最后CAD重放不输入池索引，物理约束链和1D/UV/枚举仍冻结，身份映射自研。域/后置UV翻边/BDS转换、普通多UV、后续BDS恢复和所有失败分支仍未完整，不能把恢复前缀当全流程验收。

2026-10-08已落地DomainClassification/InitialUvLegalization。普通Color保留pop时着色、LIFO和重复访问、三边原序及other_side最后覆盖；从外围点0先着-2，再将返回种子连通区域着faceTag，余-1不主动填孔。UV轮按半边池序，只处理两面faceTag/raw incircle/严格凸性；仍不锁CAD标签，最多100扫描，保留空末轮及上限诊断。池验证迁至HalfEdgeTopology共用，不混入几何资格或域规则。

36域+36UV共72次actual GFaceInitialMesh调用逐值通过，全部有重复Color访问，含两尺寸上限/两场、窄孔/双孔/嵌入线压力；UV4次真正翻边且把先前已恢复的嵌入线段翻成两侧近邻点对角线，原曲线标签保留在池对象。后续BDS恢复因此是复现所需步骤，不能把两轮当作无效重复而合并。输入只冻结1D/UV/枚举/物理链，没有冻结输出域/连通性/种子/访问/决策。additional、多UV及全部退化、BDS转换/恢复/度量合法化和完整几何/生成仍未验收。

已落地 `buildPeriodicInitialTriangulation`：按 CAD 参数跨度判是否 makeCube，盒来自原边界 raw UV，不含嵌入点；makeCube 的边长实际为 raw 盒对角线，保留先 ±1 后 scale 的算术次序，再放大 3.5。外围 ID -1..-4 无扰动、面分类/双尺寸默认值 1e22。其内部调用纯 UV `buildDivideAndConquerTriangulation`；输入位置可对应同一 BDS 身份，排序和三角形索引与物理身份分开。48 原生分治案例、默认扰动的 32 实际周期初始阶段逐值通过，仍冻结 1D/拓扑序/副本指针序。

分治模块的邻接向量只替代原生链表所有权，保存同样的可变环首、atan2/1e-15 决胜、严格左右和共圆决胜、输出序/方向；不按角度重新全排序。完全重复坐标或实际三角形数不等于原生 Euler 预估时明确报错。原版零扰动案例实际 137/声明 152，尾部未初始化；另有嵌入线异常未定位，零扰动全部分支仍待验收。Qtest 的原生行列式数值乘积上下溢路径仍待核对；正常 CAD/有限尺度样例一致不能证明任意指数输入逐步等价。

此节为源码合同，不代表周期图表/初始三角化已全部实现。普通冻结 UV、周期环和边界 BDS 重放只验证各自阶段，自己的几何适配器另行对照。PeriodicBoundaryChart 保留环位置数、唯一 BDS 点数和 maximumLocalId，三者不能互换；每个点保留原物理身份、面求值坐标和两个尺寸状态。

2026-10-08 BDS单次恢复补充：BdsConstraintRecovery与PolyMesh恢复分开。sys2x2奇异阈值/闭区间求交、原池首个合格翻边/config_modified/曲线保护及CAD自交双方记录均按原版；BdsTopology只维护邻域、删除活邻接/旧池记录和追加新连接，不计算凸性。Complete不是边已恢复，须查recoveredEdge/nativeFatal/notRecovered；原生失败保留此前成功翻边，控制失败原子发布。720数值与136叶调用逐值一致（50合成+86冻结CAD初始BDS）；该证据不验收实际CAD恢复驱动/分类pass/域删除。完整普通源码在域前恢复boundary、域后单独恢复embedded，二者都有pass2；上一轮只核对到域前，不应省略域后的原生调用。

普通域前boundary阶段已实现BdsCurveRecovery：boundary再embedded登记约束表，boundary逐曲线/线段恢复和curve分类，曲线正常结束后才补空0D端点分类。缺映射跳段，非fatal早退该曲线后继续，fatal中止boundary，未恢复/refineAll交重试模块。36actual、2445段调用全池/注册/分类/逐尝试一致；自研计算全部初始前缀和inverse映射，不输入原生已变换连接。仍冻结CAD拓扑/物理链/退化/端点、1D/UV/图表值和四CAD查询。本矩阵boundary全部已有边；失败/需要翻边的整面恢复、域后embedded/周期及完整生成未验收。

## 数据和所有权

`SurfaceTypes.h` 只包含本路线数据，不导入产品参数的默认细化策略：

- `SurfaceMeshingOptions`：明确 Gmsh 尺寸、曲率、边扩展、积分、最少曲线分段、SmoothRatio、初始算法、平滑、重试与回退选项。不会把项目硬 maxSize 直接当成 Gmsh MeshSizeMax。
- `GeometrySizeSource`：本次运行不可变的点/实体尺寸、边参数尺寸、背景场及回调；缺少已启用的 CAD 曲率/分支能力时明确失败。
- `RunContext`：持有只读 GeometrySource、options、size sources；引用观察/取消接口，统计资源限制，并持有本次运行的随机流。图表/初始剖分消费同一流，不能各自重新 seed；正式上下文尚待实现，测试端显式提供固定构建的同源扰动回调。
- `BoundaryMesh`：模型级节点池、CAD 边参数列表、节点分类和边界版本。只有 BoundaryRegistry 修改边离散，几何边恢复重试是允许的修改入口。
- `FaceChart`：原面 ID、UV 分支、边环/真边界、UV 节点到物理身份映射和周期等价表；不吞并不同部件或假设所有光滑面同一图表。
- `FaceMesh`：原节点/临时周期副本/新面节点的XYZ、分类、参数及面连接；不承载活动队列或BDS域标签。临时副本与物理等价身份保持分离。
- `NodeSizeState`：稳定节点参数槽、mesh/background双尺寸和精确内部约束；实际插点失败前已append的槽后续仍需保留，不能紧缩后错移index。
- `FrontMesh`：稳定三角创建池、物理R/h、原边号邻接和deleted；初始orderedTriangles只作快照。运行期AllTris/ActiveTris由FrontQueueState持有。
- `NodeSize`：`meshSize` 与 `backgroundSize` 两个长度值，禁止合并；前者在包含片插值，后者独立求值。
- `SurfaceMetric`：直接表达 `gUU/gUV/gVV` 的第一基本形式。若复用旧 Cholesky 因子，通过明确转换函数恢复，禁止混用二次型元素和因子。
- `FrontPoint`：候选 UV/XYZ、当前 metric、目标步长、圆修正状态；不是提交后的新网格节点。
- `FrontCavityResult`：被标记的三角形、原序壳边、固定metric圆判断和实际包含片。当前不分配新节点；后续插点先append局部参数/尺寸，成功才公开物理节点与面连接，正常拒绝不删除此前已append的参数/尺寸槽。
- `FaceMeshingResult`：面网格、实际算法、各阶段状态、正常拒绝统计和错误；不以请求算法名称替代实际算法。
- `GenerationResult`：组装结果和生成状态，独立于产品 CAD/精度验收。

公共头文件只暴露模块输入输出及其必要数据。仅被一个 `.cpp` 调用的函数放匿名命名空间，不为每个辅助公式建公共接口。

## 命名和返回约定

类型 `PascalCase`，函数/成员 `lowerCamelCase`，文件按模块名；沿用项目 C++17，不新增框架。

- `evaluate/compute/find/check` 不修改网格；查找失败有明确返回状态。
- `build` 产生局部结果，成功后交给调用方；不顺带启动下一阶段。
- `insert/split/collapse/swap/smooth/refine/restore/orient` 明确修改对象及事务边界。
- 不用 `process/doWork/fixMesh/optimizeAll`，也不用 `V2/better/new` 命名版本；模块已有上下文，不在函数前重复 Gmsh 名称。
- `validateFrontInsertion` 判定 Gmsh 插点条件；`auditSurfaceGeometry` 是结果审计，两者不能混名。
- `meshSize`、`backgroundSize`、`physicalRadius`、`normalizedRadius`、`minEdgeLength` 分别命名；禁止将 R/h 与 R/minEdge 都叫 quality。
- 正常候选拒绝是 `InsertionResult::Rejected`，不算 fatal error；取消、预算终止、CAD 查询失败和无效拓扑独立状态。不会把有限预算提前终止标作收敛。

每个公共函数注释注明：对应 Gmsh 文件/函数、单位和符号、读取/修改什么、失败后的状态、必要前置条件。源码位置与测试名一起形成对应表；差异必须记录原因和测试证据。

## “完整复现”的验收门槛

1. 以上每个纳入范围的阶段有固定源码合同和模块测试；底层数值、CAD 求值差异单独记录。
2. 阶段结果包含边界、初始域、初始合法化、前沿结束、原生平滑结束、回退结果和最终组装；单步对照能定位首次分歧。
3. 分段数、两个尺寸状态、候选和空腔在冻结非退化样例中相符；共圆/容器顺序/内核差异有明确解释，不能仅以片数接近通过。
4. 平面/孔/窄带/非线性参数面、周期缝/极点、embedded、边恢复重试和自动回退全部覆盖，再检查 phone、凹球的区域密度与形状分布。
5. 原 CAD 支持/部件身份和独立审计完整报告；生成基线不达产品精度时明确报告，不能调用旧细化悄悄改变结果。
6. 自研优化、一般 G1 atlas、并行和体网格均不混入复现阶段。Gmsh 自带的平滑和 MeshAdapt 是目标流程本身，不能删除以简化验收。

资源保护和取消可以不同于 Gmsh 原始实现，但触发时必须报未完成；观察代码不改变生成顺序。完成是逐模块、逐阶段等价证据的结论，不是保证每个三角形正三角形或承诺与不同内核的 Gmsh 二进制逐字节相同。
