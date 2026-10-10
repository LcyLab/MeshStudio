# 当前模块导航（2026-10-10）

源代码现按 ui、app、cad、project、geometry、mesh 六类目录组织；日常工程仍为 MeshStudio 应用和 mesher 算法库。准确的当前入口、目录职责和调用关系见[架构导航](docs/architecture-overview.md)。下面保留此前阶段记录，其中旧文件路径为历史路径。

# CAD 表面网格架构

## 独立Algorithm 6普通/周期面入口（2026-10-09）

新增OccAlgorithm6Adapter在core层持有OCC实例并提供BREP indexed实体/参数/pcurve/Value/D1回调；SurfaceGenerator和MeshAssembly编入纯mesher静态库。运行数据为Algorithm6ModelInput、FaceMesh和共享BoundaryMesh，生成状态与独立质量审计分开。原始节点存储参数、前沿UV、最终逐角UV分别保存；共享原节点按部件/物理身份复用，面内新节点按部件/面/局部身份装配。

2026-10-09球面数值兼容：固定Windows64 SDK的球面Value/D1使用x87成对sin/cos，其末位差会影响圆／曲面Newton分支。OccAlgorithm6Adapter仅为前沿球面回调读取含Location的BREP支撑面位置和半径，以NativeSinCos独立求值；初始图表/度量、原平滑、通用OccCadGeometry和独立审计仍走原路径。NativeSinCos编入纯mesher，MSVC x64使用MASM，其他平台使用标准库且不承诺与该Windows基线逐值相同。生成器没有Gmsh依赖或参考网格输入。

普通前沿不调用周期true-boundary门控；周期前沿保留该门控。MeshAssembly错误显式返回CAD面与局部片上下文，SurfaceGenerator保留该面失败状态并禁止完整模型发布。对照端GmshReferenceParameters仅处理参考导出：存储面UV/直接反求以XYZ前向残差选择，遗漏的INTERNAL节点按三角形节点身份补齐并校验；它不向自研入口提供数据，独立审计继续决定最终质量。

generateSurfaceMesh按CAD支撑周期性/seam选择普通或周期分支，顺序连续生成、转移、原平滑及装配；缝副本平滑后清理，每角UV独立保存。恢复细分采用局部10次重试及串行PENDING模型重访，只失效共享边关联面。取消/累计预算/OCC查询失败不发布完整模型，全部面状态保留。CLI与可选SurfaceMeshEngine实验入口编译OCC适配器，纯mesher不链接OCC/Gmsh；Gmsh只在独立对照进程运行。GUI使用独立Algorithm6ExperimentalSettings及明确未审计标识，工程保存配置/逐角UV/标签；legacy生产默认保留。阶段A报告和2026-10-09 B/C/D检查点分别记录已验证结果与未通过项，三阶段尚未整体验收。下文含旧研究检查点与产品架构。

2026-10-08连续平滑边界：普通/周期测试驱动在同一FaceMesh依次调用generateFrontMesh、transferFrontMesh、smoothSurfaceMesh；两extension共168组与独立原生阶段逐值一致。makeOccSurfaceSmoothingSource组合原始节点参数、支撑面D1/Value；参数选择在MeshVertexParameters，移动/验收/发布在SurfaceSmoothing。原节点stored参数独立于前沿UV槽，周期临时对象保留至平滑后。geometry须长于源，OCC适配仅CAD入口链接，纯mesher无OCC/Gmsh依赖。较早1D/拓扑枚举/raw CAD图表/前沿尺寸UV仍冻结；generic失败、最终周期清理/装配及完整验收未完成。以下为历史。

2026-10-08当前平滑职责：SurfaceSmoothing只计算度量加权位置、临时参数/XYZ的UV面积与最差eta验收、恢复和五次试探。几何由metric/surfacePoint/reparameterizeVertex/reparameterizeEdge显式提供，面积单点参数与邻边候选不能共用。smoothSurfaceMesh按节点身份/片向量原序处理；computeSmoothedPosition不管理拓扑，acceptSmoothedPosition不永久发布；发布再独立求GPoint。控制预算跨阶段累计，失败保留此前完整发布、不遗留临时改写。180同面MFaceVertex actual叶通过；0D/1D/seam适配及转移→平滑连续驱动待补，完整任务active。下文转移检查点为历史，“平滑未实现”为当时状态。

2026-10-08当前转移边界：普通/周期测试驱动将自研初始化/BDS/节点/前沿前缀接generateFrontMesh，再接FrontMeshTransfer::transferFrontMesh。转移读FrontMesh/FrontQueueState/NodeSizeState，按运行All的顺序替换FaceMesh连接；以首活片原double法线统一朝向，诊断等价重复后做单次映射/身份退化过滤。模块不清空运行池、不删除临时节点或ghost、不平滑；编排层持有并释放池，周期临时节点清理在原版平滑之后。146叶和两extension共168连续驱动逐值通过，含64折叠/8对等价诊断。SurfaceSmoothing仍未实现；固定五次试探即使成功也继续，不能实现为首次接受退出。原1D/拓扑/反求输入仍冻结，生产独立CAD入口和全模型验收未完成。以下历史。

2026-10-08动态编排：ParametricDomain负责原FAR范围公式/严格射线穿越；FrontGeneration::generateFrontMesh只组合队列、候选/圆修正、可选域门控和insertFrontPoint，原局部结果立即回到同一活动集，不重建全图。几何源明确区分只供圆求交坐标的surface与保留成功标志的新节点surfacePoint；正常失败ghost/号消费由FrontInsertion处理，循环累计预算并恢复当前未完成活动头。280同BREP冻结初始池actual动态循环、504域叶逐值通过；核心只连自研算法，CAD重放只另连OCC。最终转移/周期等价折叠/平滑/回退属于后续独立模块，完整自研前缀到动态循环尚未衔接；完整目标active，以下历史。

2026-10-08当前单点生命周期：FrontInsertion::insertFrontPoint组合FrontCavity/显式CAD与BGM源/只读验收/局部提交，管理分配记录与可引用节点。普通拒绝保留参数/尺寸槽和已消费ID，成功才发布FaceVertex/interiorVertices/live索引；取消/预算/查询异常恢复本次入口，不全图复制。validateFrontInsertion、commitFrontInsertion仍可独立调用，几何计算不藏在队列或拓扑模块。1560同BREP actual insertAPoint叶、extension0/1普通44+周期40共168自研前缀首候选逐值一致；57目标构建/58CTest/10Python和最终7报告SHA通过。完整动态队列反馈和外循环真参数域门控未完成，详细合同见front-generation-source-contract.md；以下历史检查点。

当前插点职责（2026-10-08）：FrontCavity只构建空腔/定位包含片与标记恢复，FrontInsertion独立对应insertVertexB。validateFrontInsertion只读计算原尺寸/物理R/h/UV面积/距离和角度；commitFrontInsertion只追加验收后的三角形，按[new,otherSide,...]原局部交错序重连、登记AllTris和更新ActiveTris，不清理旧deleted槽、不改FaceMesh节点/初始快照。insertFrontTriangles保留原早退-1/-2不恢复标记、几何拒绝-3/-4/-5恢复的差别；外层正常拒绝半径/节点销毁/发布仍待insertFrontPoint负责。取消/异常只恢复局部邻接/新增片/队列增量，不复制全图。204冻结叶通过，实际新节点CAD/BGM/append和动态循环未验收；以下旧检查点为历史。

当前包含片/空腔边界（2026-10-08）：FrontPointLocation只做invMapUV、首跨边搜索/显式force扫描和包含片mesh尺寸插值，不求CAD/BGM、不改网格；FrontCavity只做固定metric圆内/DFS、内部边保护、原壳边序与包含片，原位修改deleted，不创建新节点/三角连接。正常rollback按原版把空腔成员deleted=false（含搜索进入的旧deleted起点）；取消/异常恢复调用前状态。FrontQueue、候选、定位、空腔没有职责混用。普通/周期自研前缀已接独立初始候选空腔对照；外循环参数域门控、新节点分配和真实插点反馈尚未接入。下一步按原insertAPoint/insertVertexB补齐，不把参数/尺寸槽紧缩全回滚或全图重连引入复刻路径。以下旧检查点为历史。

当前候选阶段（2026-10-08）：FrontQueue选片/活动边；FrontPointPlacement::computeFrontPoint只做UV重心度量/外心/端点双尺寸候选，correctFrontPointOnSurface只构造物理圆并调用CurveSurfaceIntersection；下一步定位、包含片尺寸插值、FrontCavity及FrontGeneration。OccCadGeometry::faceSurfacePoint与faceSurfaceDerivatives共享直接BRep支撑面懒缓存，用于复刻OCCFace的Value/D1；通用适配器入口保持。普通/周期自研前缀已接候选，实际插点反馈及原NaN失败路径尚未完成；以下旧检查点为历史。

当前前沿调度（2026-10-08）：FrontTriangleOrdering共享R/h降序/节点身份并列键，FrontTriangleInitialization与FrontQueue都使用它；FrontQueueState独占运行期AllTris/ActiveTris，FrontMesh::orderedTriangles是初始快照。findActiveEdge只判首活动边，initializeFrontQueue只原序扫描，popActiveTriangle每次只消费一个头；registerFrontTriangles登记原创建序键，updateActiveTriangles处理局部邻域，不全图重排/删除陈旧项。markInsertionFailure只在目标已pop后移除AllTris键、R置0/-1、再插入，不负责几何/空腔回滚。不可变键避免随便改半径破坏集合；非法未移除键变更明确报错。61冻结状态叶和256前缀排空阶段通过，实际候选/插点反馈仍未接入；下一步FrontPointPlacement/FrontCavity与实际包含片插值，下文为历史。

最新前沿初始化（2026-10-08）：普通/周期驱动共有NodeSizeInitialization与FrontTriangleInitialization。前者仅计算节点参数/双尺寸/内部约束，后者仅将FaceMesh和NodeSizeState转为FrontMesh运行池，保存字面均值、物理R/h、初始AllTris排序成员和原边号邻接；不改CAD、BDS、节点尺寸或输入连接。R/h是生成尺度状态，不是形状质量分数。set拒绝保留创建槽，邻接仅连接排序集成员，保留非稳定边排序和成对连接规则；不按XYZ或物理等价合并。固定算法6/radiusNorm=2/无boundary layer。256actual对照通过，但重复set键拒绝只单元/源码覆盖，反求UV与指针遍历等元数据仍冻结。活动集/活动边、候选/空腔/插点与完整几何/失败/后处理/组装尚未完成；下文较早阶段为历史。

## 固定 Gmsh 流程复现（2026-10-07，当前任务）

最新前沿节点尺寸（2026-10-08）：普通驱动已在BDS度量后接共有FaceMeshConversion。新的NodeSizeInitialization仅对应buildMeshGenerationDataStructures的节点部分；以三角形物理邻边初始化meshSize，仅排除fake与其original的直接边，再依原序处理embedded规定尺寸/内部约束及boundary半段下限。每节点执行CAD反求，只有embedded原节点用面实体/原XYZ/raw UV查询BGM，普通节点background=mesh；周期UV覆盖在BGM之后。它不使用BDS lc或原曲线storedSize、不建MTri3/邻接/队列。输入显式vertexOrder，内边存无向身份对用于成员查询；原生指针遍历和当前反求UV仍冻结。44普通actual的3074节点/16BGM/38约束逐值通过，周期实际节点和完整前沿未验收；下文为历史。

最新接缝/面转换（2026-10-08）：`FaceMeshTypes`保存原节点、临时副本及新面节点身份；`initializeFaceMeshVertices`绑定原MVertex的实体/XYZ/参数/lc，不使用BDS域分类替换。周期专有`reconstructPeriodicSeamVertices`按localId序复制重复0D/1D节点，保留fake→original及每副本UV；dim>=2重复仅显式记录诊断，保留原身份/首次UV。共有`convertBdsToFaceMesh`按活面池/getNodes序先创建全部缺失节点，再按节点身份过滤片；不按XYZ/物理ID/面积过滤、不折叠副本、不查询CAD或尺寸。40actual周期前缀及24原生转换叶逐值一致，临时/新节点身份按创建序归一，未比较原生全局编号。普通完整转换驱动、完整前沿、失败重建和最终质量仍未验收，下文为历史。

2026-10-08周期初始度量也接同一`InitialMeshLegalization::legalizeInitialMesh`，调用在域/清理后、缝副本和BDS2GMSH前。独立CAD支撑面D1→SurfaceMetric→配置/圆内判断→BdsSwapQualification→BdsTopology；没有周期专用自有规则。40actual/3654查询/246交换/63扫描/124资格拒绝逐值一致。周期下一阶段必须按recoverMap局部ID序把重复物理MVertex拆成临时0D/1D副本，保留equivalence与每个副本的UV，再转换面三角形；不能在进入前沿前折叠。该转换/前沿/后处理/失败重建与最终验收尚未实现，下文为较早阶段。

2026-10-08新增周期域/共有清理：`BdsDomainClassification::classifyPeriodicBdsDomain`与普通入口共享LIFO，只保留源码的外域(3,2)、deleted种子跳过/无补种子差异；`BdsInteriorPreparation::prepareBdsInterior`负责两次非稳定清理、引用映射、分类传播和四负点删除。恢复不着域，域不删除，清理不求尺寸或翻边。32actual完整自研CAD前缀逐值一致，1D/拓扑/副本顺序/曲线物理链仍冻结。普通入口兼容函数委托同一清理实现；周期度量/失败重建/完整前沿与最终质量仍未完成，下文历史状态按本段更新。

周期恢复另由`PeriodicBdsRecovery::recoverPeriodicBdsConstraints`承担：图表选择的局部副本先逐embedded恢复，无普通CAD registry；失败立即停止。随后按环/位置找边，已有边不调leaf，缺边执行leaf后按原物理链查首CAD owner，成功缺边也登记原edgesNotRecovered。所有成功边用临时CLASS_E=(1,1)，不补0D分类、不加入注册geometry。分类来源新增FaceBoundary，避免与同值CAD曲线混淆。boundary失败继续各段，needsRetry/refineAll交后续失败编排；不着域/清理/重建1D。

32actual周期全CAD/图表/初始/BDS/恢复前缀全值一致，80embedded调用/2012边界环边；本矩阵0缺边/0交换/0失败/0重试，失败驱动仅单元RED9→GREEN，不能据此宣称全部周期路径通过。几何由现有封装独立求值，仅1D/枚举/副本序/CAD边物理链冻结；结果连接未冻结。周期域/清理/失败重建以及后续前沿/回退/最终质量仍未完成。

`mesher::gmshReference` 是独立的串行算法 6 复现模块，核心不链接 Gmsh，尚无完整生成入口。尺寸、曲线离散、共享身份、图表、初始剖分、前沿、平滑和回退分别拥有独立职责；全部函数名及依赖见 `docs/superpowers/specs/2026-10-07-gmsh-surface-reproduction-modules-design.md`。下文 reference-physical 属于前阶段实验，不能作为完整复现链路的实现或验收证据。

2026-10-08初始度量合法化已接普通BDS全前缀。`BdsSwapQualification`只计算原始参数面积及Quality(false,true)资格；`InitialMeshLegalization`只编排动态活边池扫描、跨轮四节点配置去重、端点modified检查与边中点度量圆判据，连接改写仍归`BdsTopology`。配置先于圆/资格登记，新追加边在同轮继续访问，直到零交换末轮；不重置modified、不清理deleted池。44actual普通面/4131查询/776交换/82扫描/4资格拒绝全池逐值一致，270面积/725圆/42实际交换叶函数对照一致。

CAD版本独立从同一BREP求导也通过44actual对照。`OccCadGeometry::faceSurfaceDerivatives`懒准备`BRep_Tool::Surface(face)`并直接D1，保留位置变换；Gmsh的OCCFace也是这条路径。通用`faceDerivatives`的BRepAdaptor B样条缓存曾造成末位差异，失败日志保留，通用接口未替换。前缀CAD图表/1D/曲线元数据仍冻结；完整周期恢复驱动、前沿尺寸/插点、原生后处理/回退/组装及最终形状/CAD/GUI/性能尚未验收。下文较早检查点按本段更新。

几何求值仍由 `GeometrySource/OccCadGeometry` 提供。复现路径新增可选能力 `edgeOnFaceRange`、`edgeOnFaceCoordinates`、`edgeSeamOnFace`、`faceTrimParameterBounds`、`projectOnFaceSurface`；没有能力则明确返回失败。精确 pcurve 的 reversed 相对全局边原朝向，不能与旧 `EdgeUse.branch` 混用。图表范围按 Gmsh 同样的 `ShapeAnalysis::GetFaceUVBounds` 计算，与原 `faceParameterBounds` 的导数求值器范围分开。直接投影按 OCCFace 的支撑曲面语义，不能用旧裁剪域最近点替代；惰性 projector 每实例串行复用。缺失 pcurve/generic/robust 反求回退、曲率和代理链的完整接入仍待复现。

`PeriodicFaceChart::sampleTrueBoundary` 独立做每分支 300 参数采样，分支为 0/1；`FaceChart::assemblePeriodicBoundaryLoop` 只做已求值 UV 的 +1/-1 方向/缝选择及闭合；`PeriodicFaceChart::buildPeriodicBoundaryChart` 分配边界 BDS 身份/尺寸、处理跨环复用和 counterpart。`appendPeriodicEmbeddedChart` 只产出新增嵌入点/约束/初始点前缀；`buildPeriodicFaceChart` 按原序串联上述阶段和全部初始点列表，保留 raw UV 与扰动坐标、原生重复身份及错误诊断。运行上下文提供同一扰动流，图表不自行 seed。

`InitialTriangulation::buildPeriodicInitialTriangulation` 已负责原 raw 盒 makeCube/3.5 扩盒、-1..-4 外围 CAD 点与图表身份映射；`DivideAndConquerTriangulation::buildDivideAndConquerTriangulation` 只按已扰动 UV 排序、递归切分、切线合并和有序环邻接输出，核心不调用 CAD。邻接向量保存原生可旋转环起点及插入/删除顺序，不改为统一旧 CDT。48 组分治和默认扰动 32 组实际周期入口逐值对照通过；原版零扰动尾部未初始化问题显式诊断，全部退化分支未验收。

`InitialTriangulation::buildInitialTriangulation` 负责普通入口的早退、身份/原插入序/1.1 扩盒；`WalkTriangulation::buildWalkTriangulation` 只负责纯 UV 的矩形、Walk、分裂/局部翻边。稳定池索引保存原生对象身份、可变首边、入射边和 prev/next/opposite/分类，供后续恢复/域模块接入，不借用旧 Mesh2D 的槽/首边规则。52 点集+24 实际原生 recover=0 调用逐池一致；普通 CAD UV/1D/枚举仍冻结。约束恢复、域着色、recover=1 内的 UV 翻边、BDS 转换和初始度量合法化仍待实现，外围点/域外片此时保留。生产默认 legacy 和前沿/质量规则保持。

普通约束恢复前缀已实现 `ConstraintRecovery::prepareInitialConstraints/recoverConstraintEdges`：先按 localId/物理ID转换为稳定池索引，再按原生顶点星/严格相交/穿越序/FIFO/凸性进行翻边，记录原生负返回码并继续；半边CAD标签在此阶段不是翻边锁。共同连接操作归 `HalfEdgeTopology`，不计算几何资格。780相交、33点集恢复和28实际CAD前缀逐值通过（219/6次分别需要翻边）。冻结物理约束链、1D、UV和枚举，池端点映射自研。域分类/后置UV翻边/BDS转换、多UV和后续BDS恢复仍未实现/验收，默认产品路径保持。

2026-10-08普通Color/后置UV两模块已接通：`DomainClassification::classifyFaceDomain` 保存原生LIFO/重复访问、从外域返回种子着内域和残余-1；`InitialUvLegalization::legalizeInitialUvMesh`只做半边池序/raw UV判据/凸性/100扫描，保存上限与空末轮。输入/池验证迁到HalfEdgeTopology共用。72 actual阶段调用（36域+36UV）全状态逐值一致，嵌入线压力4次确有UV翻边；标签跟随半边池对象，先前恢复的线段被换掉，所以后续BDS恢复仍需实现。BDS度量合法化、转换/恢复、additional/多UV/全部失败路径和完整几何/生成仍待验收；默认legacy保持。

固定复现链路在2026-10-08补上`BdsTypes/BdsTopology/BdsMeshConversion`：分类身份独立于节点引用，节点按ID而边/面及邻接按原追加序；普通转换不传PolyMesh域/边标签，保留全部外围/孔片，周期保留raw点/双尺寸/counterpart/未引用嵌入边登记。24原位、36普通和32周期实际BDS快照逐值一致；普通冻结图表CAD值，周期独立计算CAD/完整图表/初始剖分，1D/拓扑序/副本序仍冻结。恢复/域/度量合法化由后续独立BDS模块承担，完整复现尚未验收。

2026-10-08 `BdsConstraintRecovery`已承担BDS sys2x2闭区间求交、恢复资格、单边原池扫描/首个可翻边/失败诊断；`BdsTopology`新增邻域和纯连接交换。恢复与PolyMesh的严格相交/FIFO不同，曲线分类受保护、config_modified受原规则控制；原生正常失败可留下此前成功交换。720数值+136冻结BDS叶调用（50合成+86 CAD来源）全池与每轮尝试逐值一致。完整普通源码顺序为登记boundary+embedded、恢复boundary、域标记、恢复embedded，上一轮只读到域前pass2，现补全域后调用。周期恢复驱动、域删除和度量合法化仍待接通。核心纯重放不链接Gmsh/OCC；完整生成/质量/GUI/性能未验收。

`BdsCurveRecovery`负责普通约束登记、域前boundary和域后embedded曲线恢复/分类；`BdsDomainClassification`独立负责临时内外域标记。boundary遇fatal中止，embedded按原版继续其它曲线并另记fatal；正常曲线结束才补空端点(localID,0)。BdsClassificationOrigin保留注册对象与临时域对象身份。域阶段36actual全池/身份/LIFO访问一致，后置embedded36actual/38段/4次翻边一致；初始到此的连接自研，CAD/1D/图表值/曲线拓扑元数据仍冻结。后续按节点BGM双尺寸→池清理/分类传播→度量合法化→GFace转换→前沿尺寸的原顺序；std::partition非稳定池序也属于合同，完整生成仍未验收。

`BdsNodeSizeInitialization::initializeOrdinaryBdsNodeSizes`独立接原recoverMap查询视图，依据原MVertex实体/物理XYZ/曲线参数，按ID序求BGM并赋两个尺寸；不以BDS临时域分类/面求值坐标替代。36actual普通全前缀及2503次查询/全池一致，尺寸值自研计算，原查询元数据/模型特征长度仍冻结。高维MAX_LC分支当前只有源码/单元证据；前沿尺寸尚未接入，后续池清理见下条。

`BdsInteriorPreparation::prepareOrdinaryBdsInterior`后续已承担删空域面、两次std::partition清理/全池引用映射、孤边删除/分类降维和四负外围点删除；保留原邻接序/分类来源/双尺寸/登记表和ID上限。36actual全前缀全值一致，36面/边池均重排，2557空域面/2685孤边/144外围点删除后剩2521片。池索引只在当前池阶段有效，跨cleanup通过返回的旧->新映射更新。外围点仍有活边显式诊断，不生成悬空引用；完整几何/度量/前沿与整模型尚未验收。

## 自有内核上的参考生成路径（2026-10-07，实验）

PhysicalReferenceFrontal不再于生成后执行固定UV度量的全图合法化；必要插点仍使用原UV CDT/可见性事务，随后用四点共同切平面的圆谓词做局部翻边。四点排序构图保证同一四边形换对角线不改变物理判据；这不是一般曲面的全局内蕴Delaunay证明。原域/方向/硬限制及完整核验保留。

`SurfaceRefinementCandidates::surfaceBalancedEdgePoint`沿原UV边有界求等物理端距点；不投影到其他CAD分支。必要分边先验收UV子片方向。几乎共线的UV桥接片可能无法用double表示该分点，显式记录拒绝后尝试原UV中点；有限值/几何求解错误不使用此分支，实际CAD与子片仍逐项验收。

`SurfaceReferenceBoundary` 使用调用方曲线导数与尺寸函数，积分累计∫|C'|/h，再单调反解为共享参数节点；误差/预算/取消明确返回，失败不覆盖输出。GeometrySource新增非纯虚edgeDerivative，默认有界差分，OCC适配使用精确D1；缓存/分区/观察代理转发，不改变CAD身份。

`SurfaceReferenceFront` 独立提供种子尺寸、周期图表种子及前沿生成。mesh/background尺寸先均值再组合；物理R/h调度、UV重心度量候选、曲面圆修正与自有CDT事务连接。圆修正未收敛保留有限候选并计数，空腔Euler/面积/距离、方向、域/约束仍验收。physical模式以同一局部物理图表统一点高和空腔判定，作为单独对照策略。

参考产品绑定将逐边硬上界H换为生成目标上界H/sqrt(2)，边界与内部一致；参考核内部R/h阈值不变，实际曲面/变尺寸仍执行硬验收。每面首次参考CDT边界节点源按物理全局ID冻结，质量分点继承沿域邻接传播的源；Geometry/Quality显式分点目的决定关联面是否重建源。真正几何分点不复用旧图表种子，避免旧密集内部点成为新尺寸依据。此状态只存在单次生成器运行内，不是工程归档中的CAD数据。

CAD极点多个UV角共用物理节点，初始物理尺寸跳过等价点间边、物理生成跳过无面积桥接片并计数；若同身份内部边跨周期域，则先用UV中点在CAD上建立种子，保留受保护极点边。不同身份零长度边仍失败。生成正常结束只表示推进结束，拒绝/未满足片另报，最终CAD/拓扑与产品硬限制保持。

实验reference/reference-physical参数可归档；生产缺省仍legacy。当前仍接旧硬细化和有界优化，一般G1图表与超差区域重生成尚未接入，整模型质量/性能目标不以局部通过代替。

## 前沿生成和虚拟分区（2026-10-06，实验路线）

`SurfaceFrontalDelaunay` 在边界CDT/裁剪域已建立后，按活动边与局部物理metric布点，曲率/尺寸进入候选高度。局部事务/CDT连接和版本队列复用既有核心；后续硬几何验收保留。活动节点数量与分配池高水位分离，拒绝诊断有界。

候选在实际三维邻域切平面构造并有界反解原UV，空腔圆判定使用同一物理切图，避免非线性参数速度同时扭曲布点和连接。`insertPoint` 的可选circleTest仅覆盖圆判定，原UV方向/可见性/约束/裁剪域仍独立核验；非法谓词结果拒绝，异常传播，事务回滚。曲率估计另记录physical chord span，按chord<=2R换算前沿半径停止尺度，最终实际CAD采样仍是验收条件。工作量记录被选细化片的边长/偏差/法向/质量违规原因，原因可重叠。

`SurfaceMeshingPartition` 借用不可变 `GeometrySource`，OCC只提供公共解析chart、原曲面投影/周期/trim分类。可靠同实体同语义、未保护的共面/共圆柱分割边可移除；仿射mapping和周期分支先验证。孔/真实特征/接口保留，失败保留原分割并诊断；不同G1曲面局部atlas未实现。

输出区分partition和原CAD：partition UV/原支撑面列表、corners+edge midpoints+centroid的原trim face/UV采样。multi-face旧face=-1/UV NaN，不冒充完整footprint；refinement/interaction保留partition ID且原edge转换。VTK/工程往返保存新schema；独立审计面片→CAD距离测原trim并集，CAD→mesh限定公共patch。体转换尚不能保留此schema，明确拒绝。

7点支撑的位置来自实际线性facet，记录CAD UV、实际距离及projectedToTrim标志，仍受原maxDeviation控制。近邻查询遇到导入3D边与pcurve参数重建不一致时，求curve-on-surface极值；不放大容差、不标记域外UV为域内。准备、生成、来源标注共用取消状态，元数据在完整验收后提交。

初始Delaunay删除有限super点后核对真实凸包边覆盖；极窄域缺覆盖时使用真实凸包种子与内部空腔插入重建，保留原UV和输入索引，继续使用精确方向/度量圆判据。此修复解决前沿插点之前的CDT初始化问题，不代替最终形状或几何验收。


## 极点和有界内部重剖分（2026-10-03）

`SurfaceTangentChart` 以实际三维三角邻域建立切平面，通过已有 `GeometrySource` 坐标/D1/法向/参数范围做有界原 UV 反解；候选继续经过裁剪域与三维验收。它不替代原 UV 拓扑，不增加 OCC/Qt/VTK 依赖或纯虚接口，域外候选明确拒绝。

后置质量可请求共享棱真实曲线中点，检查新尺寸及形状后统一使相邻面失效。完整输出直到反馈结束才压缩节点池；已优化面缓存同步连接/UV/测量。确有边界重建试验才保留整网格快照并计数；恶化或未改善回退并停止反馈，硬条件失败/预算/取消仍明确返回。

`SurfaceLocalRemesh` 用最多 16 邻点的完整内部一环、可靠 UV 对角线判定和最小最差成本动态规划重剖分。CAD 原顶点、约束端点和同面多 UV 节点保护；不删除点槽/UV 索引，末端仅压缩无引用输出节点。局部拓扑及域事务支持验收拒绝/异常回滚。调度最多四轮，成本须改善局部最差形状并通过尺寸/偏差/法向检查；完成后再优化新连接。节点入射槽索引只在拓扑未变时复用，提交后必须重建；缺失/重复/过期槽拒绝。一般边收缩、大型空腔和任意极点拓扑图表仍未实现。

## 四面体边界与验收补充（2026-10-02）

`TetMeshEngine → validateVolumeBoundary → delaunay3D → enforceSurface3D/recoverVolumeBoundary → extractVolumeDomain → ruppertRefine3D → smooth3D → 边界/域/质量复验`。体边界检查独立于允许开放面的表面网格检查；约束面以真实共面面积覆盖形成屏障，邻接奇偶分类处理凹体、空腔和分离域。局部 2→3/3→2 恢复受预算及已有覆盖保护，不支持时明确失败。网格校验还要求共享面对顶点严格异侧。

索引表面入口保持节点编号，不几何焊接；源面/棱标签与分片角点 UV 由覆盖和插值映射。生成 `ok` 与 `qualityAccepted` 分开；细化未收敛、薄片或退化会形成可保存的状态和告警，旧工程缺少状态时表示未知。3D 谓词新增 filtered/exact 及精确零决胜，浮点极端范围与相交/覆盖容差仍有边界。

用户要求三角面片侧冻结，当前独立验证链为 Gmsh 固定 T3 → 自研体核心；Gmsh 不是生产依赖，也不生成体网格。此前表面快照/自动入口保留，最新完整 GUI/自研表面集成验收延期。23 项体相关回归包含已知拒绝与质量告警，不能解释为全面 CDT/质量保证。详细证据见 `docs/superpowers/plans/2026-10-02-tet-correctness.md`。以下表面章节保留其各阶段范围与历史记录。

## 调用边界

UI 表单统一使用 `ui/DialogFormFactory`：精确数值框、标题、参数分区/行、折叠区和操作栏，只依赖 Qt。`MeshSettingsDialog` 和 `ModelTransformDialog` 已复用；统一主题通过 formDialog 属性限定。数值非编辑时最多三位小数，编辑/悬停使用完整 round-trip 字符串，实际 double 不随简写舍入。模型变换显式保留原三位存储精度。调用示例见 `docs/dialog-form-factory.md`。

`MainWindow → SurfaceMeshEngine → OccCadGeometry → mesher::generateCadSurfaceMesh`

`MainWindow` 持有工程级 `SurfaceCriteria` 与 `TetMeshParams`；两个功能区按钮打开同一个 `MeshSettingsDialog` 的对应页。Dialog 编辑独立草稿，取消/关闭不提交；生成仅提交当前页并关闭弹窗，再启动后台引擎。旧网格参数 Dock 不再创建，表面取消按钮留在主窗口。工程保存/恢复不依赖 Dialog 生命周期，空工程也恢复参数并结束恢复状态。

- `OccCadGeometry` 负责 B-Rep 有效性、拓扑、曲线/曲面、p-curve 分支、法向与面方向；不负责三角化。
- `mesher` 只使用标准 C++，不包含 OCC、Qt、VTK。`GeometrySource` 和三维标量 `MeshSizeField` 是未来表面/体网格共用的输入契约；`MeshingProgress` 返回 false 取消工作。
- `SurfaceCriteria` 独立持有表面偏差、法向变化、半径边比等判据。未来的体判据应使用单独的 `VolumeCriteria`，本阶段没有替换四面体入口。
- `SurfaceMeshResult` 使用既有 `SurfaceMesh3D` 点与三角形，附带 CAD 面标签、棱段标签、逐三角形角点 UV、测量统计和诊断。周期缝节点具有一个三维 ID 与多个面内 UV。
- `SurfaceMeshEngine` 捕获 OCC/标准异常，转换 VTK 数据；界面处理显示和工程存储。

### 表面质量模块（2026-10-03）

`SurfaceBoundarySizeField` 保留首次几何采样作为尺寸源，沿有效面域邻接传播并在插点新增邻接时更新；质量切分不变成新的小尺寸源。`SurfaceQualityPolicy` 区分实际 CAD 角点保护与常规最小插点尺寸；`SurfaceRefinementCandidates` 只产生局部 D1 度量中的有限偏心/外心候选。初始内部粗片仍可用原廉价细化，普通边界在后置阶段重新检查。

`Delaunay2D::insertPoint` 的末尾可选验收回调在事务提交前执行。表面候选通过三维硬判据和局部质量才发布节点；拒绝/异常恢复原拓扑及 UV 索引。共同输入契约仍为纯 C++，没有添加 OCC/Qt/VTK 依赖；体算法调用保持兼容。

共享棱请求先在同版本完整验证，再批量登记参数并使相邻面失效；跨面统一处理减少反复重建。`CadSurfaceDiagnostics` 为 VTK FieldData 的字符串数组，携带核心测量残差与候选拒绝诊断，随现有 VTP 保存；旧工程缺少该可选字段不影响加载。后置质量的边界反馈与有界内部一环重剖分已补入，范围见上方 2026-10-03 章节。

## 独立装配部件与输入协调（2026-10-02）

用户确认各部件分别生成完整三角表面，保留接触与干涉。`GeometrySource::faceBodyId` 提供零基 CAD 实体标签，无实体归属返回 -1；`bodyContact` 提供接触、公共体积干涉、无测得接触或查询失败。核心默认 `AssemblySurfacePolicy::RejectBodyOverlap`，实际表面引擎显式使用 `IndependentBodies`。独立策略允许不同实体表面的交叉，记录 CAD 关系；同一实体的自交、方向/闭合、约束及硬几何要求仍检查。CAD 关系查询失败不表示无干涉；OCC 异常仍由引擎明确报告。

`SurfaceMeshResult` 增加逐片部件标签及 `bodyInteractions`。VTK 的 `CadBodyId` 属于 CellData，`CadAssemblyPolicy`/`CadBodyInteractions` 属于 FieldData，随工程保存。关系记录来自网格相交候选，不是全部实体对的完整 CAD 干涉审计。独立部件没有生成接触两侧的一致公共剖分，这些标签可复用，表面不能直接作为无重叠的多区域四面体边界。

OCC 适配默认严格有效性检查；实际表面引擎允许在深副本上有界协调共享端点附近的微小裁剪交叉容差。修复后完整有效性、拓扑数量和原始标签顺序必须通过；不改曲线、曲面、位置、拓扑和原文件。接受副本以 `CadAcceptedBRep` 保存到 VTP 字段，`CadInputReconciliation` 保存涉及面/棱/顶点及新旧容差，工程继续嵌入原输入字节。不是通用缺陷 B-Rep 修复。

## 算法

1. 检查输入与预算，用几何包围盒确定自动尺寸。
2. 每条 CAD 棱只生成一套物理节点，以三维尺寸、弦偏差和相邻面的法向变化细化。`SurfaceChart` 在局部 D1 度量中估计单位物理长度的法向导数，以最大奇异值估计曲率 κ；将尺寸收紧为 `min(h, 2 sin(θ/2)/κ, sqrt(8δ/κ))`。这是三维各向同性标量尺寸的采样估计，最终仍检查映射面片；它可以突破 minSize 并报告面/棱及估计值。普通闭合曲线至少保留四段初始采样；退化棱单独处理。
3. 在每个 face wire 上联合选择边走向及周期 p-curve 分支。面内节点保存自己的 UV，并引用公共三维节点。
4. 用图表中心的曲面导数构造正定 UV 度量；插点和合法化直接在原始 UV 上评价度量二次型圆判定，方向判定不经过投影舍入。一般裁剪域用无微扰二维 Delaunay、锁定约束的凸四边形翻转恢复及非约束边合法化；共线约束展开为子段链。矩形参数域按物理度量布点后合法化。域标记沿邻接传播，跨外环/孔翻转奇偶值，内部特征不改变标记。合法化以顶点对维护局部边队列，紧凑删除后重新查询半边。
5. 按映回三维的边长、曲面偏差、法向变化、质量和翻折检查细化。超长边/大法向跨度优先二分；二分通过局部事务验证 UV 方向与半边拓扑，再从新点一环传播 Lawson 翻边恢复 CDT，重新检查三维判据。翻折修复先试不会破坏正常邻片、且减少翻折数或剩余跨度的内部二分，避免反复切有效短边界。失败保留前 8 组片/邻片快照和最多 24 个点构造祖先。尖角、窄区、极点保留细节并报告尺寸/质量冲突。
6. 用局部曲面度量检查细化点是否侵入约束棱的直径域；侵入且棱段违反局部尺寸/几何要求、映射翻折涉及该约束边，或粗 UV 裁剪弦相交时，登记原曲线的新切分参数。`CadBoundaryRegistry` 保持每条 CAD 棱的公共采样链和参数节点 ID；只使该棱及关联面失效，无关面复用已验证的三角形/UV/统计。内部翻折分裂长边，避免重心加密保留原长边。面内不变的顶点组三角形复用 CAD 判据测量；点/法向查询按原 UV 缓存，输出阶段复用已测结果。切分诊断记录 CAD 棱、区间、源面、翻折/边界违规原因（前 64 次）。
7. 按 CAD 面的统一方向输出，检查物理边关联与方向、CAD 约束、采样判据和 BVH 相交。单面误差不足以保护窄间隙；检测到穿越且面片仍有可分辨的 CAD 近似误差时，按其误差的一半收紧相关面的偏差，重新离散关联共享棱并重建所有关联面；其他缓存保持。相交细化消耗共同预算，零近似误差、精度极限、预算或取消返回明确失败。当前收紧范围是整张关联面，尚未实现空间局部近邻尺寸场。
8. 几何细化完成后，固定全部共享裁剪/缝/极点节点，以三维半径边比做受限翻边和内部节点移动。移动使用邻点 UV 均值以及节点处 D1 局部度量映回 UV 的切向位移；候选逐片检查域、方向、最大边、偏差及法向，要求局部最坏值和质量和下降。最后重新测量及检查闭合/相交；失败回滚到已接受几何网格。剩余工作预算不足或达到 40 次扫描时明确报告软质量残留，不能提前消耗几何细化预算。

第 7 步的相交反馈只处理当前策略不接受的交叉；独立部件策略下跨实体交叉不触发细化，按上述关系记录。矩形完整网格每格用较短物理对角线分成两片，不额外加入中心点；仅从相应水平/竖直边提取 U/V 切分，原 p-curve UV 在非完整网格连接路径保留。

二维符号谓词使用 double 快速过滤与浮点展开回退；有限输入在中间计算不溢出/下溢的条件下保留精确符号。三维相交为容差检查，三维四面体谓词没有在本阶段升级。

`GeometrySource::faceParameterBounds` 是可选的纯 C++ 查询；OCC 适配器提供参数范围，曲率和默认 D1 的差分模板在边界使用单侧取样。p-curve 端点仅在 64 倍机器精度的相对舍入带内夹到范围端点；真正越界和非有限输入明确失败。零法向报告奇异取样，保留基础尺寸并继续依靠相邻正常样本与面片检查。参数连续区间、查询精度和通用图表切分尚未接入。

矩形的两侧分段不匹配时，按节点顺序连接两侧，并细分上下边对应的三角形以保留全部约束。避免中心扇形种子的大量翻边，也避免近共线 UV 边在投影点集凸包中丢失。当前 CDT 仍使用图表中心的固定度量，局部曲率尺寸只先接入公共棱采样，尚不等于变度量 CDT 或完整内域尺寸场。

批量约束恢复、增量插点、内部边二分和 Lawson 合法化通过 `Mesh2DTransaction` 记账局部三角形/邻接、原半边节点及点/片长度。失败逆向恢复，提交前验证受影响片及一环、保留约束；面完成仍全面检查。提交公布有序删除/尾片搬移/新增报告与单调拓扑版本，副本参照仅供测试。有限 UV 精确索引保持正负零相等；节点移动通过 `Mesh2D::relocatePoint` 同步索引及全部邻片版本；空腔访问代号溢出时重置。面重建期间保留 CAD 节点及仍有效的面引用，废弃内部节点槽确定性复用；成功输出前压缩弃用节点并同步三角形/边界/图表索引，不做几何焊接。节点预算按活动槽计数，分配池最高容量单独统计。

`SurfaceWorkStatistics` 分别记录边采样分裂、共享切分、相交反馈细化、面内插点、翻边、面构建和调度轮次；预算失败包含累计工作与重建次数。相交检查允许合法公共顶点/完整公共边接触，仍拒绝共面同侧重叠、重复三角形和穿过共同顶点的内部交叉。

`MeshingObserver` 是纯 C++、借用且可空的共同观察接口，默认参数保持原调用源码兼容（整仓重编，不保证旧二进制 ABI）。通过实体维度/ID 标注嵌套阶段和计数；回调不得抛异常或改变几何，取消仍由进度回调处理。RAII 在取消、失败和异常时闭合阶段，无观察器时核心不读计时时钟。`CadSurfaceBenchmark` 中的几何代理转发全部 `GeometrySource` 能力，统计 API 查询次数和时间；几何计数属于工具代理的外部请求，不包含 OCC 内部所有调用。工具维护包含/自身时间，避免父子阶段重复求和；全副本、访问、空腔与全域分类计数来自核心指定操作，不能解释为所有内存读写次数。体网格可后续复用观察契约，本阶段没有接入四面体生成。

## 参数与结果约束

`SurfaceDomainState` 在面构建初始分类、最终完整分类逐值核对；正常局部编辑仅继承源片标签并检查变化区域的一环奇偶关系。跨域空腔、内部二分或翻边明确拒绝。Lawson 与局部标签日志配对，逐次消费编辑报告；取消或错误同时恢复标签和拓扑。共享棱改变或相交反馈重建时丢弃该面标签，重新初始化。

`SurfaceRefinementQueue` 以槽位/拓扑/判据版本及稳定顶点键维护坏片。翻折和硬几何同级，按超限比处理，质量软目标随后；每次事务更新变化片和一环，搬移片复用几何测量。历史堆超过活动数的三倍加 128 时压缩；队列为空并完成合法化后重新测全部有效片，遗漏作为维护错误返回。共享棱请求独立保存边界版本、区间、源面及原因，中央调度优先去重处理有效请求，登记一次公共参数后重建关联面。

| 参数 | 自动值/内部值 | 规则 |
| --- | --- | --- |
| `targetSize` | 包围盒对角线 × 0.025 | 三维基础目标；Dialog 将其设为最大尺寸，API 可单独设置 |
| `maxSize` | API 默认 0；新 UI 默认为包围盒对角线 × 0.025 | >0 为三维三角边长上限，仅允许 1e-8 相对比较容差；0 保留旧目标尺寸 +5% |
| `minSize` | 目标尺寸 × 0.1 | 常规尺寸场及质量细化下限；几何特征可突破并诊断 |
| `maxDeviation` | 目标尺寸 × 0.1 | 曲线中点、三角形边中点与重心采样测得偏差 |
| `maxNormalAngleDeg` | 30° | 曲面法向采样点之间最大变化 |
| `maxRadiusEdgeRatio` | √2 | 三维外接圆半径/最短边；受特征和尺寸下限限制 |
| `growthRatio` | 1.3 | 采样目标尺寸在邻点之间的局部限制，未证明全局渐变上界 |
| `maxVertices/maxIterations` | 各 200000 | 硬几何细化耗尽时报错及位置；收敛后的软质量改善耗尽剩余工作预算则报告残留 |

Dialog 基础设置为最小/最大尺寸、几何偏差和法向角，高级设置为质量、边界渐变和预算。显式最大尺寸贯穿目标尺寸钳制、公共棱采样、面内坏片队列/分裂以及输出边长验收；超过上限返回失败。最小尺寸仍是软下限。旧工程缺少 maxSize 时读取 0，Dialog 按原允许值填入并提示重新生成才采用显式上限。API 的其他长度 0 采用自动值，负值或非有限值报错。

四面体页仅提供引擎实际消费的 targetSize 和 optimize，并说明 T4、自研 Delaunay 与细化；最小尺寸、HXT、T10 等未实现设置不显示为有效输入。原四面体算法及其目标尺寸语义不变。

`CadFaceId` 为每个三角形的 CAD 面 ID；`CadEdgeId` 的三个分量对应三条有向三角边，内部边为 -1；`CadCornerUV` 的六个分量对应三个角点的 UV。保存的是原始网格数据，显示法向处理不改变存储节点编号。

## 验证与已知限制

自动用例覆盖平面带孔、窄环、尖角、曲面共享棱一致性、粗曲线边界重建、三维尺寸场、STEP 立方体、圆柱缝、球面极点、参数收紧、交叉面拒绝、取消/预算及工程元数据往返。已有四面体测试用于回归。界面提供表面生成取消按钮。

复杂 `ball.brep` 已通过 targetSize=20、δ=2、θ=30°、预算 40000 的核心/引擎、闭合与 CAD 标签回归。独立审计以每面 16×16 偏移 UV 格点和每棱 33 个非中点采样查所属面网格最近距离；每面分层选至多 32 片，以三组 0.6/0.2/0.2 内部点查 OCC 裁剪面最近距离。不复用生成器偏差函数，仍仅是有限采样；结果见 `progress.md`。move/ball 实际 Qt 窗口已通过生成/取消/显示、保存后新窗口恢复；全部节点坐标/三角连接/标签逐值一致，并目视检查测试窗口图像，未声称用户手动验收。仅支持上述有界局部容差协调，不保证任意 CAD/预算成功，不宣称全局距离界或严格无自交证明。表面质量统计未持久化；仍需更多真实模型/参数覆盖。phone/凹球最新验证和质量残留见 `docs/superpowers/plans/2026-10-02-phone-concavaball-debug.md`。

GUI 的网格归类使用 `ModelSummary::isMesh/isSurfaceMesh`，不再按中文模型名猜类型；网格加载不回填 CAD 自动尺寸。属性单位在缺少已核对元数据时留空；长度条件使用导入后的模型坐标。外部 Gmsh 交换/面映射只属于测试审计工具，生产核心仍为自研。
