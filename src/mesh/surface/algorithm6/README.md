# 固定 Gmsh 表面流程复现

2026-10-10目录导航：本目录是自研 Algorithm 6，已从 `core/mesher/gmsh_reference` 按阶段迁移；模块职责和当前调用链见[架构导航](../../../../docs/architecture-overview.md)。下文保留历史研究检查点，不代表当前待办；当前验收结果以仓库 `docs/superpowers/plans/2026-10-09-algorithm6-final-acceptance-results.md` 为准。

2026-10-08连续平滑：普通/周期初始化前缀→generateFrontMesh→transferFrontMesh→smoothSurfaceMesh共168组零容差通过，节点/连接按自研运行状态连续传递。makeOccSurfaceSmoothingSource独立供参数/pcurve/投影/D1/Value；保留原stored参数，不以周期前沿槽UV替代。63 CTest/24 Python、64参数/120分类平滑回归及8中立性通过。较早1D/拓扑/raw CAD图表与前沿尺寸UV仍冻结；generic失败、周期对象清理/最终装配与整模型验收未完成。职责/证据见`.planning/2026-10-07-gmsh-reproduction-research/surface-smoothing-prefix-source-contract.md`。以下为历史。

2026-10-08当前原始参数：MeshVertexParameters/OccMeshVertexParameters已独立实现候选、单点、邻边参数及OCC默认直接路径。64组原MVertex对照/1240节点计算/27432邻边计算逐值通过；拓扑和结果UV不冻结。120组真实分类的actual平滑也逐值通过（7170访问/971354源请求），但入口网格/分类/XYZ/存储参数仍冻结，自研前沿→平滑连续链尚未接入。Generic反求失败、robust及退化边人为1D零参数路径保持未完成；以下为历史检查点。

2026-10-08当前平滑核：180同面MFaceVertex实际laplaceSmoothing逐值一致（1300访问/300可移动访问/400发布/53816查询）。同BREP独立OCC D1/Value；连接、分类及入口参数/XYZ冻结；0D/1D/seam原始参数适配及前沿到平滑连续集成尚未验证。44复刻+18既有CTest/21Python、146转移叶和8观察中立性通过。完整目标仍active、legacy保持；以下为历史检查点。

2026-10-08当前：普通/周期自研初始化前缀已连续接FrontGeneration和FrontMeshTransfer，extension=0/1共168组逐值通过。172318次pop、76637候选、75860插点尝试、74807成功、777域拒绝及875756几何查询；161输入消费新生成片。最终161870片的连接顺序/顶点次序一致，周期单次映射折叠64片，保留8对等价诊断，不自动去重。146转移叶和168结束池重放另外通过。原1D/拓扑序/raw CAD反求仍冻结，原生平滑/完整几何/全失败重建回退/装配与最终质量未完成；本段是阶段验收，完整目标active，legacy保持。下面旧数值检查点为历史。

2026-10-08动态循环检查点：新增ParametricDomain和FrontGeneration。504严格穿越叶、5类同BREP/280冻结初始池动态运行逐值通过；7289次pop、6231候选、5191插点尝试、2708成功和1040门控拒绝，224输入实际pop了新生成片。原循环直接调用bowyerWatsonFrontal；自研独立查询CAD范围/D1/Value/BGM，不冻结候选、空腔、连接或反馈。初始运行池仍冻结，不代表从CAD入口到最终网格已串联。新节点surfacePoint保留GPoint成功标志，圆求交surface只取坐标；failed GPoint循环传播有单元证据，原生注入-9只在1560单点回归中覆盖。完整前缀衔接、最终转移/平滑/全部失败回退/装配与形状质量仍未完成，legacy保持。

本段运行模块按以下职责调用，几何源只读，运行池和身份由调用方持有：

| 模块 | 主函数 | 职责 |
|---|---|---|
| FrontQueue | initializeFrontQueue / popActiveTriangle | 原集合顺序、活动边与逐次消费 |
| FrontPointPlacement | computeFrontPoint / correctFrontPointOnSurface | 度量候选和CAD圆修正 |
| ParametricDomain | computeParametricFarPoint / testParametricDomain | 原范围远点和严格穿越门控 |
| FrontPointLocation / FrontCavity | findContainingFrontTriangle / interpolateNodeMeshSize / buildFrontCavity | 包含片、局部插值、空腔与壳边 |
| FrontInsertion | validateFrontInsertion / commitFrontInsertion / insertFrontPoint | 新点生命周期、验收、局部连接及反馈 |
| FrontGeneration | generateFrontMesh | 依次调用以上模块，直到活动集耗尽 |
| FrontMeshTransfer | transferFrontMesh | 按运行All顺序输出、原double朝向和单次周期身份映射；不平滑或释放池 |
| SurfaceSmoothing | computeSmoothedPosition / acceptSmoothedPosition / smoothSurfaceMesh | 原度量加权、临时验收恢复、节点身份序和固定五次试探；不改变连接 |
| SurfaceSmoothing质量叶 | computeTriangleEta | 原物理角度/最小角eta；不使用项目gamma判据 |
| MeshVertexParameters | collectMeshVertexParameters / reparameterizeMeshVertexOnFace / reparameterizeMeshEdgeOnFace | 原seam候选序、单点参数和各自参照对端首候选；不复用运行槽UV |
| OCC参数适配 | makeOccMeshVertexParameterSource | 仓库core目录的独立OCC适配器，indexed拓扑/pcurve/curve与surface投影；不链接进纯mesher |

FrontGeneration负责生成编排，FrontMeshTransfer承担连接转移；SurfaceSmoothing显式查询metric/surfacePoint/reparameterizeVertex/reparameterizeEdge。MeshVertexParameters负责参数语义，OCC源只提供实际几何。单点默认failOnSeam=true，返回false仍有UV；原面积忽略此bool，自己的平滑源保留同一行为。eligibleVertices计可移动访问（跨轮重复），acceptedWrites计发布（含零移动）。下一步自研前沿/转移连续接原参数源和平滑，再补全部几何/重建/回退/装配；纯核心没有Gmsh/CAD链接。此前检查点如下。

2026-10-08当前单点阶段：insertFrontPoint已组合独立CAD/BGM与空腔、验收、局部提交；正常拒绝保留分配槽/编号，成功才发布可引用节点，取消/预算/查询异常恢复入口。1560同BREP实际insertAPoint叶全部逐值一致，extension0/1普通44+周期40共168完整自研前缀首候选通过。旧池/节点和查询位置冻结，新点XYZ/BGM/空腔/局部连接独立算；新全局身份按创建序归一，24 failed GPoint为明确注入。57目标构建/58CTest/10Python、204验收/79空腔回归通过，192662字节/63导出补丁真实重放/8neutrality及7报告SHA通过。完整动态循环未完成，不加自有优化。以下历史。

当前插点验收/局部提交（2026-10-08）：FrontInsertion只对应insertVertexB。validateFrontInsertion计算原早退、尺寸均值/物理R/h、距离/极钝角与严格UV面积；commitFrontInsertion按交错局部序连接、登记AllTris并更新ActiveTris；insertFrontTriangles保留原-1/-2早退与-3/-4/-5标记恢复差异。204冻结叶逐值通过，含126成功/504保留新片、全部-1至-5、负双尺寸、边界/面端点高度例外、非平面物理半径、重复外侧片和set重复拒绝。新节点XYZ/UV/分类/尺寸与空腔在此叶明确冻结；未验证实际insertAPoint节点分配、CAD/BGM、失败槽/半径或完整动态循环。55Release/49CTest/10Python、79空腔叶回归和181227字节/62导出补丁真实重放/8neutrality通过。下一步insertFrontPoint生命周期并串联自研前缀，再FrontGeneration和剩余完整流程；整体active。以下旧检查点为历史。

当前空腔阶段（2026-10-08）：FrontPointLocation负责原UV反映射/搜索/包含片mesh插值；FrontCavity负责固定metric的DFS/壳边/内部约束与标记恢复。两者不分配新节点、不提交连接或求新点BGM。MTriangle圆内重载不减容差，已deleted邻片不贡献shell；搜索仍可能进入已deleted起点，正常原生恢复清掉该标记，调用取消/异常则恢复入口状态。79原生叶及普通/周期自研前缀在extension=0/1共168阶段/10718候选逐值一致，含132次无包含片/122次搜索。对照未执行参数域门控、插点提交或动态反馈；原反求/1D/拓扑/指针序仍冻结。完整目标仍active；后续插点合同见`.planning/2026-10-07-gmsh-reproduction-research/front-insertion-source-contract.md`。以下旧检查点为历史。

当前候选阶段（2026-10-08）：FrontPointPlacement::computeFrontPoint对应optimalPointFrontal，correctFrontPointOnSurface对应普通OCC optimalPointFrontalB；两者只生成/修正候选，FrontQueue仍只调度，定位/空腔/插点留给后续模块。普通和周期自研前缀均已接候选阶段；独立CAD度量/支撑面Value，不冻结候选或求值结果。有符号高度/半径/epsilon按源码保留，CAD求交失败仍保留快候选，取消/预算明确传播。原非有限候选的NaN/401查询目前只记录诊断，实际失败半径与插点反馈尚待对齐，不能称完整复刻。以下旧检查点为历史。

当前调度检查点（2026-10-08）：新增FrontTriangleOrdering共享半径/身份键，FrontQueue只负责初始活动集、首活动边、逐个pop、局部登记/活动更新及失败半径0/-1转换，不计算候选或插点。运行期AllTris/ActiveTris用std::set不可变键，FrontMesh的orderedTriangles保持初始快照；每次pop不全图排序，局部更新只暂存本次新增键。61组冻结MTri3状态叶对照通过，含严格阈值、邻片deleted忽略、陈旧删除/邻接变化、失败重新入集键冲突和局部重复。普通纯/CAD各44、周期40在extension=0/1共256阶段/16480pop及16298次允许插点判定逐值一致；这是无候选/插点反馈的初始化和排空对照。45CTest（35复现）/8Python、48相关Release目标通过，补丁158779字节/59导出实际重放及8neutrality通过。原反求UV/指针序与1D/拓扑元数据仍冻结，实际插点失败/空腔/动态队列反馈未验收。下一步FrontPointPlacement及实际包含片插值/FrontCavity，再剩余几何/失败/后处理/组装与最终质量。完整目标active，以下为历史。

最新初始化检查点（2026-10-08）：NodeSizeInitialization已接周期全前缀，40actual的2956节点、128embedded BGM、80内部约束、630临时副本及690次UV覆盖逐值一致。FrontMeshTypes只保存三角运行状态，FrontTriangleInitialization::initializeFrontTriangles只复刻MTri3均值/物理R/h、AllTris身份排序/set拒绝及原边号邻接，不承担活动队列或插点。普通纯/CAD求导各44actual、周期40actual在boundary extension=0/1各通过，共256actual/18472三角；原生半径/排序/邻接未冻结。44CTest（34复现）/8Python与46相关Release构建通过。实际矩阵没有重复set键拒绝，相关行为仅源码/单元覆盖；前沿反求UV/指针序及1D/拓扑元数据仍冻结。下一步活动集/活动边，再候选/空腔/插点及剩余几何/失败/后处理/组装；完整目标与最终形状验收未完成。下文较早检查点均为历史。

目标是固定 4.15.2、OCC、一阶、串行算法 6 的完整流程。此目录为独立实验实现，尚无完整生成入口，不接入旧产品细化规则，核心不链接 Gmsh。先对照流程，再做自有优化。

最新节点尺寸检查点（2026-10-08）：普通面转换驱动已接共有FaceMeshConversion，44actual及独立CAD求导变体逐值一致，1552条deleted记录顺序一致；未出现新面节点或身份过滤，不替代已有24转换叶覆盖。新增NodeSizeInitialization只实现前沿节点阶段：最短物理邻边、直接fake-original边排除、embedded规定尺寸、boundary半段下限、embedded面实体BGM和最后的周期UV覆盖。单元RED12→GREEN；普通完整前缀44actual的3074节点/反求记录、16embedded BGM、38内部约束和16不同双尺寸逐值一致，纯和独立CAD求导版本均通过。前沿原生指针遍历序和反求UV仍冻结，不能称前沿几何适配已完成。MTri3/邻接/队列/插点、周期节点实际对照及最终形状验收仍未完成；以下为历史。

最新检查点（2026-10-08）：周期接缝与前沿前的面转换已落地。`FaceMeshTypes`保存节点身份和映射，`PeriodicSeamReconstruction`只重建临时0D/1D副本及各副本UV，`FaceMeshConversion`只绑定原节点元数据、按BDS活面池顺序转三角形。40组实际周期完整前缀对照逐值一致：630个临时副本、3084个三角形、492条deleted记录；该矩阵未产生新面节点。另24组实际BDS2GMSH叶对照验证新节点创建/复用、身份退化过滤及先创建再过滤。临时/新节点按创建序归一身份，未比较原生进程全局节点编号。证据：periodic-face-mesh-final-oracle、face-mesh-conversion-leaf-final-oracle。原1D元数据/拓扑序/副本序/物理边链仍冻结；普通完整转换驱动、失败1D重建、完整前沿及最终质量尚未验收。以下为历史记录。

2026-10-08最新周期初始度量阶段：源码周期算法6复用同一delaunayizeBDS，自研直接复用InitialMeshLegalization/BdsSwapQualification/BdsTopology，CAD适配从同一BREP独立读取支撑面D1，不冻结度量值。40actual全前缀的3654查询/246交换/63扫描/124资格拒绝逐值一致；80嵌入/80图表与40初始/转换/恢复/域/清理也逐值通过。原32案例0交换的覆盖缺口以圆柱/球面各15嵌入点补齐，测试要求两类均有成功交换。证据periodic-bds-metric-final-oracle；仍冻结1D/拓扑/副本序/曲线物理链。下一步周期缝临时副本与BDS→面网格转换、失败1D重建及前沿等，完整生成未完成。下文为历史。

2026-10-08最新周期阶段：classifyPeriodicBdsDomain保留deleted负点种子跳过、外域(3,2)、无普通负对点补种子；prepareBdsInterior共享源码相同的两次cleanup。新单元RED4→GREEN，32actual独立CAD/图表/初始/BDS/恢复/域/清理全前缀逐值一致，4296重复pop、2140外域片、2268孤边和128外围点删除。证据periodic-bds-domain-interior-final-oracle。1D/拓扑/副本序/物理曲线链仍冻结，周期度量/重试/完整前沿及整模型验收未完成。下文较早状态按本段更新。

职责、数据所有权和全部函数命名见 [模块设计](../../../../docs/superpowers/specs/2026-10-07-gmsh-surface-reproduction-modules-design.md)，任务和验收见 [实施计划](../../../../docs/superpowers/plans/2026-10-07-gmsh-surface-reproduction-implementation.md)。

已落地的职责：

| 模块 | 只负责 | 主要函数 |
|---|---|---|
| MeshSizeEvaluator | 组合尺寸源、限制、倍率 | evaluateMeshSize |
| CurveDiscretization | 密度积分、尺寸扫平滑、计数、参数反解、节点元数据和过近内点过滤 | integrateCurveDensity / smoothCurveSizes / computeMinimumCurveSegments / computeCurvePointCount / distributeCurveParameters / computeStoredCurveSize / filterCurveNodes |
| BoundaryRegistry | 模型级物理节点、每条 CAD 边唯一离散、边恢复重试与版本 | buildBoundaryMesh / refineUnrecoveredEdges / findIncidentFaces |
| FaceChart（部分实现） | 普通面物理闭合/UV/分类/约束/初始插点顺序；周期边界环的方向/缝分支选择和闭合 | buildFaceChart / assemblePeriodicBoundaryLoop |
| PeriodicFaceChart | 真边界采样、边界 BDS/早退、嵌入 BDS/约束；按原顺序组装全部初始点列表 | sampleTrueBoundary / buildPeriodicBoundaryChart / appendPeriodicEmbeddedChart / buildPeriodicFaceChart |
| DivideAndConquerTriangulation | 已扰动 UV 的排序、递归切分、切线合并、有序环邻接和三角形输出 | buildDivideAndConquerTriangulation |
| InitialTriangulation | 普通入口早退/1.1 扩盒；周期入口 makeCube/3.5 扩盒、外围 CAD 点和连接映射 | buildInitialTriangulation / buildPeriodicInitialTriangulation |
| WalkTriangulation | 普通初始矩形、Walk、逐点分裂和局部翻边；保存稳定半边/面池 | buildWalkTriangulation |
| HalfEdgeTopology | 有限UV/池索引验证、三角循环改写和对角线交换；不判断翻边资格 | validateHalfEdgeTopology / rewriteHalfEdgeTriangle / swapHalfEdgeDiagonal |
| ConstraintRecovery（普通 PolyMesh） | 图表身份映射、严格穿越追踪/FIFO 翻边、原生诊断与双侧标签 | prepareInitialConstraints / intersectRecoverySegments / recoverConstraintEdges |
| DomainClassification（普通 PolyMesh） | 外域/内域两次原生 Color，残余未知域/孔保留 | classifyFaceDomain |
| InitialUvLegalization | 普通 PolyMesh 恢复/着色后的池序 UV 翻边；不执行 BDS 度量规则 | legalizeInitialUvMesh |
| BdsTypes / BdsTopology | BDS 分类身份、稳定节点ID、边/面池、原序双向邻接与连接交换；不决定几何资格 | insertBdsPoint / findBdsEdge / appendBdsTriangle / getBdsTriangleNodes / validateBdsTopology / getBdsEdgeNeighborhood / swapBdsEdgeTopology |
| BdsMeshConversion | 普通全 PolyMesh 面/周期 DocRecord 连接转换；保留独立 BDS 坐标和分类登记 | convertOrdinaryInitialMeshToBds / convertPeriodicInitialMeshToBds |
| BdsConstraintRecovery | BDS数值求交、恢复翻边资格、单条约束的原池扫描和失败诊断；不分配CAD分类或编排曲线pass | intersectBdsRecoverySegments / canSwapBdsRecoveryEdge / recoverBdsConstraintEdge |
| BdsCurveRecovery | 普通boundary/embedded登记、域前boundary与域后embedded曲线恢复、原生端点分类和失败诊断；不着域或清理池 | buildOrdinaryBdsRecoveryRegistry / recoverOrdinaryBdsBoundary / recoverOrdinaryBdsEmbedded |
| PeriodicBdsRecovery | 周期embedded先恢复、边界环恢复/临时CLASS_E和重试诊断；不标域、补端点分类或重建1D | recoverPeriodicBdsConstraints |
| BdsDomainClassification | 普通/周期BDS外域/内域的原序LIFO标记；保留不同种子规则和临时分类身份，不恢复或清理 | classifyOrdinaryBdsDomain / classifyPeriodicBdsDomain |
| BdsNodeSizeInitialization | 原MVertex实体/物理XYZ/曲线参数的BGM查询与BDS双尺寸赋值；不做前沿尺寸插值或清理 | initializeOrdinaryBdsNodeSizes |
| BdsInteriorPreparation | 普通/周期共有的空域面删除、两次非稳定池清理、孤边删除/分类传播与四外围点删除；不恢复/翻边/求尺寸 | prepareBdsInterior（prepareOrdinaryBdsInterior兼容入口） |
| BdsSwapQualification | 原始参数面积与初始Quality(false,true)资格；不求度量或改连接 | computeBdsParameterTriangleArea / canSwapInitialBdsEdge |
| InitialMeshLegalization | 动态活边池扫描、跨轮四节点配置去重、边中点度量圆判据；连接改写委托BdsTopology | legalizeInitialMesh / shouldFlipInitialEdge |
| FaceMeshTypes | 前沿前原节点/临时副本/新面节点身份、局部映射、UV与三角连接；不等同最终物理编号 | FaceVertex / FaceMesh |
| PeriodicSeamReconstruction | 按局部ID序复制重复0D/1D原节点、记录等价身份与副本UV；不改BDS或折叠副本 | reconstructPeriodicSeamVertices |
| FaceMeshConversion | 原节点元数据绑定、活面池顺序转换、先分配节点再按身份过滤；不查询CAD或求尺寸 | initializeFaceMeshVertices / validateFaceMeshVertices / convertBdsToFaceMesh |
| NodeSizeInitialization | 前沿物理邻边尺寸、embedded双尺寸/约束、boundary半段下限和UV覆盖；不构造MTri3或插点 | initializeNodeSizes |
| FrontMeshTypes | 前沿运行期三角创建池、物理R/h、原边号邻接及初始AllTris成员；不保存活动策略 | FrontTriangle / FrontMesh |
| FrontTriangleInitialization | 双尺寸均值、物理R/h、同半径身份排序/set拒绝及成对邻接；不选活动边或插点 | initializeFrontTriangles |
| FrontTriangleOrdering | 共享compareTri3Ptr不可变键：R/h降序、排序节点身份决胜；池索引不参与比较 | makeFrontTriangleKey / FrontTriangleKeyLess |
| FrontQueue | 初始活动扫描、首活动边/pop、AllTris登记、局部活动更新与失败半径转换；不做候选/空腔 | initializeFrontQueue / findActiveEdge / popActiveTriangle / registerFrontTriangles / updateActiveTriangles / markInsertionFailure |
| FrontPointPlacement | UV重心度量/活动边双尺寸候选；普通CAD物理圆修正和原失败保留，保留有符号高度；不定位/插点 | computeFrontPoint / correctFrontPointOnSurface |
| FrontPointLocation | 原UV反映射、到重心的首跨边搜索、显式force扫描及实际包含片mesh尺寸插值；不求CAD/BGM或修改空腔 | mapFrontPointToTriangle / intersectFrontSearchSegments / findContainingFrontTriangle / interpolateNodeMeshSize |
| FrontCavity | 固定本次metric的原DFS、内部边保护、壳边原序和包含片；只修改/恢复deleted，不分配节点或提交连接 | buildFrontCavity / rollbackFrontCavity |
| FrontInsertion | 验收/局部提交与单点生命周期分函数：前三个函数只处理已分配节点；insertFrontPoint组合空腔与显式CAD/BGM源，管理身份、分配槽、发布/正常拒绝。均不选片、执行参数域门控或调度全循环 | validateFrontInsertion / commitFrontInsertion / insertFrontTriangles / insertFrontPoint |
| SurfaceMetric | 第一基本形式、度量外心、物理半径 | evaluateSurfaceMetric / computeMetricCircumcircle / computePhysicalCircumradius |
| LinearSystem3 | 固定三阶列主元 QR | solveColumnPivotedQr |
| CurveSurfaceIntersection | 原生有限差分 Newton 和圆修正 | solveCurveSurfaceIntersection / intersectCircleWithSurface |

几何谓词仍属于 `../GeomPredicates`。`orient2dValue` 提供自适应有向面积数值，`incircleOriented` 保留原始朝向；旧 `orient2d/incircle` 语义保持，不能把 ±1 用于 Walk 面积大小比较。

初始度量合法化已通过44个actual普通面全前缀对照：4131次边中点度量查询、776次交换、82轮动态扫描及4次圆内但资格拒绝，全池/配置/事件逐值一致。纯重放冻结CAD导数；CAD重放通过`OccCadGeometry::faceSurfaceDerivatives`从同一BREP独立调用支撑面D1，也逐值一致。BRepAdaptor的B样条求导末位曾导致对照失败，失败记录保留；通用faceDerivatives仍保留原接口。270面积/725度量圆/42实际Quality(false)叶函数对照一致。普通CAD图表/1D/曲线元数据仍冻结；这些结果不代表周期恢复驱动、完整前沿或最终三角形质量已验收。

公共计算返回状态；取消、预算、非法 CAD 查询与正常未收敛分开。失败不发布半成品输出。曲面求交的非有限算术提前退出与原生迭代次数不同，单独记录，不能称为全部路径逐步等价。

周期恢复正常路径已通过32actual完整CAD/图表/初始/BDS/恢复前缀：80次embedded调用/2012条边界环边处理，全池/临时CLASS_E来源/逐次调用一致。只冻结1D/拓扑枚举/副本序及CAD边物理线段；恢复输入的局部副本和初始连接由自研计算。本矩阵没有缺边翻转/失败/重试，相关控制仅单元RED9→GREEN，不能称完整周期失败路径通过。分类新增FaceBoundary来源，临时(1,1)与同值注册CAD分类区别，不放入geometry注册表。

BoundaryRegistry 接收几何层提供的不可变精确查询视图，不生成 UV。0D 先分配，边内点过滤只标记节点失效，保留原分配编号空洞；闭环起终点身份相同但参数不同。恢复请求由 FaceChart 转换物理身份后按原生顺序传入，模块返回受影响面，编排层负责清理和重新生成。

8 个冻结 BREP 的最终 0D/1D 节点、参数、坐标及有向线段通过常尺寸原生对照；ordinary-map 恢复在 8 BREP 首/末/内段连续重试，参数/分类/XYZ/getLc 与实际原生函数一致。18 个变化尺寸案例的生成期边链/分配数通过，其中 10 例内点删除的身份空洞一致。最终 Gmsh API 的统一 Renumber 与生成期身份分开，测试端原生规则映射后输出一致，正式编号步骤留给 MeshAssembly。

FaceChart 普通路径已有单元和 6 个面实际 getNodeCopies 冻结 UV 重放证据，节点身份/顺序/UV/分类/面求值坐标及约束一致。周期环组装按原版选择严格唯一最近的方向和缝分支；6 BREP、9 个环、6 档原生容差和两种首缝选择共 108 次原生函数对照，顺序/物理身份/UV 一致。纯环函数不调用 CAD、不创建 BDS、不做跨环复用或 embedded；边界和嵌入 BDS 由 PeriodicFaceChart 接管。冻结几何输入的重放均不替代自研 CAD 适配器验收。

PeriodicFaceChart 的 buildPeriodicBoundaryChart 已补上六档重试和边界 BDS 初始化；只复用此前环的同物理 0D/近 UV 点，同一环和 1D 不复用，按环位置编号，counterpart 按原版单对应覆盖规则更新。7 BREP（含双向周期 torus）、两尺寸场、三次环重复共 54 组原生 BDS 对照，编号/分类/UV/面坐标/双尺寸/包围盒/环引用及容差选择一致。18 组为原 CAD 环序，36 组重复环是跨环复用压力输入，不是有效面网格生成证据；该重放仍未验证自己的 CAD 求值。

sampleTrueBoundary 按原版每边/分支均匀查询 300 次（seam 先 0 后 1），输出 299 条相邻 UV 线段，包含退化边。真实 OccCadGeometry 的精确 pcurve 接入已在 9 面、9269 条原生线段零容差对照通过，CAD 值由相同 BREP 独立计算；唯一边的原生指针遍历序由参考拓扑元数据显式提供，不能称全部拓扑适配已完成。缺失 pcurve/robust 反求回退仍待实现。

周期边界新增生产几何对照 GmshPeriodicBoundaryChartCadReplay：从同一 BREP 独立求范围、两套 pcurve UV 和面坐标，54 组原生 BDS 对照零容差通过。仍冻结 1D 网格/分类/参数和参考 CAD 环序，18 组原环序、36 组压力重复的证据边界保持。首轮圆柱容差 1 ULP 差异按源码定位到 ShapeAnalysis 与 BRepAdaptor 范围不同；新增 faceTrimParameterBounds 对齐图表尺度，不修改旧求导范围或放宽检查。

生产几何新增 projectOnFaceSurface，按 OCCFace 的支撑曲面直接投影/非周期轴扩展/惰性 projector 语义实现；不使用旧裁剪域最近点查询替代。7 BREP/9 面、两初始种子共 108 次实际原生投影零容差一致。投影落在孔内是支撑曲面语义，generic 反求仍待复现。

appendPeriodicEmbeddedChart 只产生新嵌入 BDS/约束/DocRecord 前缀，冻结原边界 inverse map，保留独立嵌入点与边端点的重复物理身份、负分类 tag、原分类 BGM 以及 seam 首副本选择/空覆盖诊断。buildPeriodicFaceChart 串联真边界、边界、嵌入和所有 DocRecord 点，预算累计且失败原子发布；raw UV 与扰动坐标分开，General 路线必须显式提供运行扰动流，零扰动也不跳随机调用。原生 seam 失败造成点数超过预分配时明确拒绝，不复刻越界写。

64 组实际 meshGeneratorPeriodic 嵌入阶段及 64 组完整图表串联对照逐值一致。叶阶段冻结边界 BDS；完整模式不冻结边界 UV/面坐标/范围/尺寸/编号，全部 CAD 值从同 BREP 独立计算，但仍冻结 1D 网格、CAD 拓扑序及副本指针序。24 组正常圆柱/球/环面周期输入、24 组平面强制周期私有入口、16 组圆柱/球端点 seam 压力分别记录；不代表初始三角化或全模型验收。

周期初始剖分已实现原生 makeCube 的对角线长度、3.5 扩盒、-1..-4 面求值/分类/双尺寸初值和 DocRecord 分治排序/切线/环邻接/三角形序；不裁域或删除外围点，交给后续约束/域模块。48 组实际 DocRecord 重放逐值通过；默认扰动 1e-9 的 32 组实际周期入口整阶段对照通过，包含 12 正常周期面、12 平面强制周期入口和 8 端点 seam 压力，仍冻结 1D/拓扑/副本指针序。BDS raw UV 不被扰动后的排序坐标覆盖，核心无 CAD/Gmsh 链接。

零扰动诊断发现原生 ConvertDListToTriangles 的实际 count 可少于 Euler 预估 count2（实际 137、声明 152），未初始化尾部使周期入口读取异常；自研显式 InvalidTopology。默认路径不受参考诊断钩子改变，8 原版/off/on 生成指纹相同。零扰动嵌入线另见原生 -5，尚未完整定位；全部零扰动初始分支不称通过。原生 Qtest 的行列式乘积上下溢/所有浮点指数跨度仍待数值核对，不以当前符号决胜样例推断任意输入等价。

普通初始入口已分为两个模块：InitialTriangulation 只做早退/身份校验/原次序/1.1 扩盒；WalkTriangulation 用稳定索引对应 PolyMesh 对象池，保存可变面首边、点入射边、循环/对边和标签。内部 locateInitialTriangle 使用有向面积数值比较，splitInitialTriangle 保留六半边追加序和 LIFO/已处理边规则，swapInitialEdge 保留原始朝向 incircle 与共圆不翻；不接旧 CDT。52 组纯点集及 24 次实际 GFaceInitialMesh(recover=0) 零容差逐池一致；含两尺寸上限/两尺寸场及嵌入点。CAD UV/1D/节点枚举顺序仍冻结，没有验证自己的普通面 CAD 反求。连接更新与池验证归 HalfEdgeTopology 共用。

ConstraintRecovery 只复现普通 PolyMesh 恢复前缀，不替代后续 BDS::recover_edge 或 BoundaryRegistry 的 1D 加密。prepareInitialConstraints 按 BDS localId→物理身份→顶点池映射并保留约束序；恢复按原生顶点星/穿越路径/FIFO/凸性/仍穿越规则，CAD 标签在此阶段不是翻边锁。已有边为0，正码为初始穿越数，-1/-2/-3 明确记录并按原版继续；成功标记 getEdge 找到的两侧，不将循环结束称为完整约束验收。静止队列/取消/预算/非法拓扑显式失败，不替换输入布局。

780 次实际原生严格相交、33 点集恢复（219 次需要翻边）及 28 次实际 GFaceInitialMesh 恢复前缀，穿越序/返回码/标签/全池逐值一致。新增 slit-embedded-point CAD 的四种尺寸输入实际有6次需翻边恢复、21条初始穿越，补上原24CAD全为已有边的覆盖缺口。最终实际CAD输入冻结1D/UV/枚举及物理约束链，但池端点映射由自研计算；BDS转换、普通多UV副本和后续BDS恢复未验收。-1失败后继续已有边有原生对照；-2/-3及原版无限循环的全部退化路径仍待额外对照。

DomainClassification 保留两次 Color 的 pop 时着色、三边原序、LIFO 重复入栈/访问和最后 other_side 覆盖。先从外围点0着外域-2，返回另一侧种子后着内域faceTag，残余-1按原生保留；不删除外围/孔内片。缺种子显式失败，避免原版空指针路径。InitialUvLegalization 只按两面同faceTag、raw incircle<0和原生凸性规则整池扫描，最多100次；末次空扫描保留，达到原生上限显式标记，不伪称收敛。这一轮仍不锁CAD标签，不能与后续BDS度量合法化合并。

72次actual GFaceInitialMesh阶段调用（36域+36UV）访问信息/种子/标签/全池/扫描/翻边序零容差一致，全部出现重复Color访问，含窄孔、双孔和嵌入线压力。4次真实UV翻边将先前恢复的嵌入线段换成近邻点对角线，CAD标签仍留在半边池对象；这说明后续BDS恢复不能删除。输入仍冻结1D/UV/枚举/物理约束链，结果未冻结。普通additional分支、多UV/全部失败退化、BDS转换/恢复/度量规则及整模型质量仍未完成。

边界/embedded seam 在普通图表入口仍显式返回 PeriodicRequired；多 UV 最终恢复和普通 embedded seam 的 getNodeCopies 分支待实现，不能称图表/初始模块全部分支已完整复现。BoundaryRegistry 的 periodic multimap/all 恢复、通用反求回退和完整产品几何接入也未完成。测试端 OCC 反求失败明确退出，不使用未经对照的近似结果。

BdsTopology使用ID有序节点表、稳定追加边/面池以及原生点边/边面顺序，不按坐标合并身份，不去重三角形。BdsMeshConversion普通路线保留独立reparam后的BDS UV/XYZ，四外围节点-1..-4在曲面求值；按PolyMesh面首边序转全部面，不传域/半边标签，不删除孔内/外围片。周期路线保留raw点、双尺寸、counterpart、负分类tag和未被节点引用的嵌入边登记。原生嵌入边在遍历线段前无条件add_geom，不能仅从新增点推导完整分类表。缺物理映射按原版跳片并显式记录；重复ID/非法数值/取消/预算失败不发布工作副本。

24组实际BDS原位操作、36次实际普通转换和32次实际周期转换全池/节点ID/原序邻接/三角形方向/分类逐值一致。普通整前缀由自研计算，但1D、UV枚举、独立BDS图表和四CAD查询值仍冻结；周期通过真实OccCadGeometry/完整图表/初始剖分串联，1D/拓扑序/副本指针序仍冻结。周期全端点复用压力曾暴露未引用分类丢失，补传递后复核，不放宽分类表比较。BDS恢复、度量规则及整模型未在这项证据中验收。

单条BDS恢复已实现：求交使用原生sys2x2及闭区间端点规则，与PolyMesh严格穿越分开；每轮整边池扫描，第一个合格翻边后重扫。曲线分类受保护，force只绕过config_modified；CAD登记自交记录双方为非fatal失败，无可翻边等为fatal。Complete只表示调用结束，调用方须检查recoveredEdge/nativeFatal/notRecovered。原生正常失败发布之前成功翻边的拓扑；取消/预算/非法输入保留输出。拓扑交换继承原边/两面分类，旧记录标deleted并移除活邻接，新边/两面按原序追加。

720数值+136恢复叶调用零容差对照通过：50合成、86来自已冻结周期CAD初始BDS。包括反向/六角点排列、config_modified、counterpart不阻挡恢复、CAD自交、多次翻边和两次成功交换后fatal；比较全池/邻接/分类及每轮扫描/尝试。这里冻结BDS点/连接/分类，不验证实际CAD外层恢复驱动或pass标记。完整源码顺序为pass1登记boundary+embedded、域前pass2恢复boundary、域后另对embedded调用pass2；上一轮只读到域前pass2，现已补全顺序，不能合并这两阶段。

BdsCurveRecovery已按物理身份独立建立ordinary inverse映射及首登记去重约束表；退化边跳过，缺映射跳段，曲线提前返回时不标端点，fatal终止boundary，非fatal继续其它曲线并交给重试编排。空曲线仍add_geom，空分类端点用(localID,0)，已分类点不改。36次实际meshGenerator域前boundary阶段、2445段调用的登记/分类/全BDS池和叶扫描/尝试逐值一致；初始/恢复/Color/UV/转换前缀由自研计算，不冻结结果连接。仍冻结1D/UV/副本序/独立BDS图表/四CAD值、曲线拓扑/物理链/退化及端点元数据。这批边界均为已有边；不能据此称boundary所有翻边/失败、域后embedded、周期驱动或全模型已验收。

BdsDomainClassification保留原生LIFO的重复pop、三边原序push、外域清空和负外围点对边补种子；不稳定过滤或补猜孔内域。BdsClassificationOrigin区分登记对象、临时内域和临时外域；即使tag/degree相同，原生指针身份也不同。普通外域(1,3)与周期外域(3,2)分开。36次实际普通域阶段全池/身份/访问/种子零容差一致，9588次pop中4886次为重复访问，2181片清空外域分类、2521片活内域；未清理池。

recoverOrdinaryBdsEmbedded在域后调用，与boundary共用逐曲线规则，但原版外层忽略curve bool，fatal只记录并继续其它embedded。36次实际嵌入阶段、38段调用全池/身份/扫描/尝试一致，其中4次真实翻边恢复之前UV轮换掉的嵌入线。初始到此的连接由自研计算；1D/UV/副本序/独立BDS图表/四CAD值与曲线拓扑元数据仍冻结。这不是全部失败/周期恢复或完整CAD生成验收。

BdsNodeSizeInitialization按BDS ID序查询原recoverMap中的MVertex元数据，0D用(0,0)，1D用(u,0)，同时赋lcBGM/lc；原高维实体直接MAX_LC，未映射点保持旧双尺寸。onlyInitialMesh跳过此段，Algo6实际为false。查询失败/预算/取消不发布半成品，原生非正尺寸回退保留evaluateMeshSize诊断。36actual普通全前缀、2503次BGM查询及全池零容差一致，尺寸值由自研独立计算；原实体/物理XYZ/曲线参数和模型特征长度仍作为输入冻结，整批实际没有高维MAX_LC分支，该分支只有单元和源码证据。

BdsInteriorPreparation按原序删除空分类面，再cleanup、删除孤边/边及端点分类降维、第二次cleanup，最后删除-1..-4；其它孤点和最大编号保持。std::partition非稳定排列及全部池引用映射保留，邻接相对顺序不改。36actual普通全前缀对照逐值一致：2557片空分类面、2685孤边、144外围点删除；36组面池与边池均确实重排，最终2521片，不能稳定过滤。分类身份/双尺寸/注册表/编号/邻接/两次重映射全值对照，输出连接由自研计算，前缀CAD/1D/原MVertex查询元数据仍冻结。外围点仍有活边时显式失败，避免原版悬空指针；该诊断仅单元覆盖，普通正常矩阵未触发。

后续先接度量合法化与周期BDS恢复驱动，再完整几何接入、图表剩余分支、前沿、原生平滑、失败回退及组装。BDS双尺寸与后续GFace/MTri3前沿尺寸初始化分阶段。未完成模块不预建空实现。CTest 标签为 `gmsh_reproduction`，参考工具/补丁说明见 [对照说明](../../../../tests/GmshReproduction.md)。
