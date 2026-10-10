# Gmsh 4.15.2 原生对照

2026-10-08独立主流程阶段A：OccAlgorithm6Adapter独立从BREP提供1D/拓扑/参数/回调，SurfaceGenerator和MeshAssembly已接普通面从边界到平滑/最终装配。rectangle/hole/narrow_band h=.3、extension0/1共6组不读参考输入，独立Gmsh4.15.2（禁止切换）最终VTP与同一独立审计器形状/拓扑/标签对照通过；矩形多2片，不称逐连接等价。63相关CTest/24Python通过，最终证据algorithm6-stage-a-20261008-r3；完整入口重试未触发，周期/整模型/GUI/正式三轮速度未验收。详情docs/superpowers/plans/2026-10-08-algorithm6-stage-a-results.md。以下连续逐值矩阵为此前研究检查点。

2026-10-08当前连续平滑：`ordinary-smoothing-prefix-extend{0,1}-oracle`各44与`periodic-smoothing-prefix-extend{0,1}-oracle`各40，共168组自研初始化前缀→前沿生成→转移→平滑逐值零容差一致。实际参考独立生成网格，再调用原转移及laplaceSmoothing，不输入自研结果。较早1D/拓扑/raw CAD图表及前沿尺寸UV仍冻结；平滑BREP拓扑/pcurve/参数结果独立计算。当前64参数/120分类平滑回归、63 CTest/24 Python与8中立性通过；六报告/实际配对输出SHA见`smoothing-prefix-release-sha-audit.json`。extension1报告在摘要flag增加前启动，实际平滑配对输出完整审计，原报告不重写。完整目标active，generic反求、全恢复/回退、周期对象清理/组装、形状/CAD/GUI/性能未完成。下文为历史。

核心19预期RED→GREEN；发现原单点默认failOnSeam=true后新增1 RED→GREEN，面积原调用忽略正常false而使用UV。CAD适配stub实际入口RED→GREEN；首次set/TopoDS_Vertex缺include构建失败已纠正。球面退化边人为1D零参数触发原生“OpenCASCADE curve is null in projection”，失败报告保留；Unknown-support边非零stored节点仍对照通过，零参数反求不伪造成功。默认robust=0/genericClosestPoint=0直接路径，不据此宣称所有反求回退覆盖。

当前45复刻+18既有CTest/21Python通过；`surface-smoothing-kernel-raw-adapter-regression-oracle`180同面回归、`front-transfer-after-raw-parameters-oracle`146转移叶通过；`observer-raw-parameters-neutrality`8原版/off/on指纹同。231372字节/69导出/11文件补丁实际Git apply后SHA同，10pristine与tgz逐字同；新增geo/MVertex.cpp仅测试导出，原函数未改。当前报告/DLL/EXE及依赖边界见raw-parameters-release-sha-audit.json。下面为历史报告，不据旧SHA称当前重跑。

2026-10-08当前平滑核：`surface-smoothing-kernel-oracle/smoothing-report.json`为180次实际laplaceSmoothing与自研SurfaceSmoothing的零容差对照。五类CAD各36例，0/-1/1/2轮、片序/朝向反转、多内点顺序更新、首和第二次GPoint失败注入。1300访问/300可移动访问、1500试探（428接受/1072拒绝）、400发布、53816源查询；32首点失败、28发布求值失败。节点均为同面MFaceVertex，外圈用真实bl_data保护；连接/分类/入口UV+XYZ冻结，独立OCC D1/Value。同面stored-parameter路径不代表0D/1D/seam原始反求或前沿→平滑集成，任务9/总目标仍未完成，最终形状/CAD/GUI/性能未验收。

核心单元23预期RED→GREEN；2个自研Release目标和原生shared构建通过。44复刻+18既有CTest/21Python检查通过，`front-transfer-after-smoothing-oracle`146转移叶一致；`observer-surface-smoothing-neutrality`8原版/off/on指纹相同。当前补丁227090字节/67导出/10文件，9pristine与tgz逐字同，实际Git apply新副本后10SHA同。观察只记录原函数已计算状态，不额外查询几何；失败注入仅主动测试导出启用。首次导出宏拼写编译失败已纠正/保留日志。SHA和依赖审计见surface-smoothing-release-sha-audit.json；下文前缀报告是上一DLL的历史证据，不据旧SHA声称当前重跑。

2026-10-08当前连续生成/转移：`ordinary-front-transfer-prefix-final{0,1}-oracle`各44组、`periodic-front-transfer-prefix-final{0,1}-oracle`各40组，共168组逐值一致。自己的普通Walk/恢复/域/UV/BDS/度量/面转换/节点/初始前沿和周期CAD图表/BDS/恢复/缝副本前缀，在同一运行中进入generateFrontMesh再transferFrontMesh；候选、新CAD点/BGM、空腔、连接和反馈不冻结。172318pop/76637候选/75860尝试/74807成功/777域拒绝/875756几何查询，161输入确实pop新生成片。输出161870片，周期单次映射64身份折叠/8对等价诊断；普通面原外层无trueBoundary门控，周期真边界由独立CAD采样。

`front-transfer-leaf-final-oracle/front-transfer-report.json`另含146 actual transferDataStructure叶与168独立结束池重放，比较consumed/deleted/reversed/collapsed/equivalent和最终连接，覆盖首片退化、不使用robust符号替代double法线、物理法线分支、空指针/非空空equivalence、单次映射链和三份重复。原版splitEquivalentTriangles只报告，不去重。该结束池重放不等同连续入口证据，两者分别记录。

转移核心新增FrontMeshTransfer；单元RED20→GREEN，普通连续集成先RED缺front_transfer，再接驱动GREEN。5个自研受影响Release目标与参考shared构建通过；43复刻+18既有=61不同CTest及14Python通过。初次三个OCC CTest缺运行DLL（0xc0000135）已补原runtime PATH后全部通过；普通动态输出原30秒限时按60秒单例复核定位为测试观察耗时，完整普通驱动保护为300秒，不放宽算法判据。旧全池每轮观察曾超过64MB，现协议为初始/最终有序集合+逐轮成员增量+实际pop序，Python校验增量和最终集；280动态回归及1560单点回归通过。观察只记录已计算状态，不改变生成策略。

观察补丁216259字节/66导出/10文件、9pristine与固定tgz逐字同，真实Git apply重放10SHA一致；当前8原版/off/on生成指纹一致。二进制/报告与既有167源边界见`front-transfer-release-sha-audit.json`；纯转移重放仅CRT/KERNEL32，CAD驱动另连OCC，无Gmsh/Qt。1D元数据/拓扑与枚举/raw反求仍冻结；平滑、全部几何、重建回退、模型组装和形状/CAD/GUI/性能未验收，完整目标active。平滑源码合同另见`.planning/2026-10-07-gmsh-reproduction-research/surface-smoothing-source-contract.md`。下面为历史检查点，不用旧报告宣称当前SHA。

2026-10-08动态循环最终检查点：ParametricDomain和FrontGeneration已落地。`front-generation-final-oracle/comparison.json`为同一固定BREP上的280次actual bowyerWatsonFrontal完整动态运行（rectangle/hole/cylinder/sphere/torus，共7个面，extension0/1、常量/变化场、无门控/空域/全域/半域/孔/重复段/internal/ghost/deleted/空All）；7289pop、6231候选、5191实际插点尝试、2708成功、1040域拒绝、133282几何查询逐值一致。224输入确实消费了新生成片，不再只有初始队列排空。正常码1/-1/0/-4/-2/-3分别2708/1403/307/610/123/40。504实际pointInsideParametricDomain叶覆盖端点/共线/重复/孔/射线顶点与1e±90数值乘积上/下溢，全部零容差一致。

本矩阵明确冻结初始运行池、旧节点UV/XYZ/分类/双尺寸、internal边和All索引；自研独立求裁剪范围、D1/metric、每轮CAD圆求交/新XYZ/BGM、空腔、提交和动态反馈。原生leaf只注入初始化输入，调用实际原版初始活动扫描和循环，在transfer之前返回；没有重写一个参考while循环。每次新自动MVertex号按分配序归一到逻辑counter，拒绝ghost也消费。自己重放的初始队列调用现有initializeFrontQueue，不在测试代码再实现调度。最后运行池/槽/live索引/身份/XYZ全部比较；不表示完整CAD入口、普通/周期全自研前缀动态衔接、最终转移/平滑/装配或整模型质量已验收。

新节点source.surfacePoint显式保留GPoint成功标志，source.surface只提供圆求交坐标。循环failed GPoint传播测试先RED后GREEN；本动态原生矩阵没有自然/注入-9，对照不泛化。1560单点原生回归另含24显式注入-9（查询真实OCC后标记失败），204 insertVertexB和79空腔叶回归也通过。61Release构建，42复刻CTest+18既有CTest=60不同测试、10Python语义测试通过；控制测试涵盖跨阶段几何预算、取消保留既有提交、观察异常、空活动仍查一次范围和当前活动头恢复。

208161字节/65导出/10文件观察补丁已真实Git apply重放并逐SHA核对，9pristine原文件来自固定tgz；8原版/off/on生成指纹相同。纯FrontGeneration单元只有CRT/KERNEL32，CAD重放另有OCC、没有Gmsh/Qt。最终四报告DLL/EXE SHA及原167源边界见`front-generation-release-sha-audit.json`；初轮/修改前报告保留为历史，不用于当前SHA声明。构建/日志/数据都在仓库外MeshStudioValidation，legacy及正式EXE保持。完整目标active。以下旧检查点为历史。

2026-10-08单次insertAPoint：新增GmshFrontPointInsertionTest/CadReplay/Comparison及JSON输出。独立新节点CAD Value、包含片mesh插值、BGM查询、分配槽/ID消费、正常拒绝半径/成功局部池/AllTris/ActiveTris逐值比较；不把新点XYZ或BGM输出当输入。最终1560通过（400成功、648单片拒绝、288无包含片、200近点拒绝、24注入failed GPoint）。原自动MVertex计数器不重置，仅本次新号按创建序归一到冻结maximum+1。-9来自查询真实OCC后的测试代理标记，不是自然OCC失败。extension0/1普通44+周期40共168完整自研前缀首候选通过（140成功/18近点拒绝/2面积拒绝/8无包含片）；首提交隔离于后续初始排空，不证明完整动态反馈/参数域门控。57构建/58不同CTest（40复刻+18既有）/10Python、204验收/79空腔回归通过，192662字节/63导出/10文件補丁真实重放/8neutrality和7报告SHA通过。证据front-point-insertion-release-sha-audit.json，核心单元仅CRT/KERNEL32、CAD重放仅另连OCC且不连Gmsh；以下历史范围保持。

## 新片验收与局部提交（2026-10-08，当前）

FrontInsertion独立对应insertVertexB：validateFrontInsertion计算原新片尺寸/物理R/h、距离/极钝角与严格UV面积；commitFrontInsertion进行原局部交错连接/集合登记/活动更新，insertFrontTriangles保留-1/-2早退不恢复及-3/-4/-5恢复差别。调用方实际新节点CAD/BGM/append槽、失败半径/节点销毁/发布仍未接入。

单元RED16→GREEN，含严格面积等号、force与星形开关分工，以及两次集合merge后的取消/异常增量恢复。204冻结叶调用实际原生insertVertexB：126成功/504保留新片，72个-3、3个-4，-1/-2/-5各1；808诊断行和全池/AllTris/ActiveTris/局部序列逐值一致。覆盖extension开关、负双尺寸、boundary高度/face端点例外、极钝角、非平面物理半径、重复外侧片和set重复新片拒绝。原零距离cosine NaN以显式字符串诊断，保留比较语义；非有限输入/面积/半径的保护退出不冒充原生正常路径。

此叶明确冻结新节点XYZ/UV/分类/双尺寸、原池/队列和空腔/壳；`insert_apoint_lifecycle_validated=false`、`front_generation_feedback_validated=false`。没有用伪造已插入连接或原生验收输出驱动自研计算，也没有据叶通过声称真实新点或完整动态循环已验收。

55Release、49CTest（39复现+10既有）和10Python通过；79空腔叶回归。观察补丁181227字节/62导出/10文件真实重放，9原文件与固定发行包一致；8原版/off/on指纹同。2最终报告DLL/EXE SHA、观察源码/补丁SHA、167原源码边界与纯模块依赖复核通过。证据`front-insertion-leaf-release-oracle`、`front-insertion-cavity-regression-oracle`、`observer-front-insertion-neutrality`和`front-insertion-release-sha-audit.json`。下一步真实insertFrontPoint生命周期/自研前缀串联，再外循环及剩余完整流程；整体目标active。以下旧检查点为历史。

## 实际包含片与初始空腔（2026-10-08，当前）

`FrontPointLocation`负责原invMapUV、到重心的首跨边搜索、显式force扫描及包含片mesh插值；`FrontCavity`负责固定候选metric的原DFS/壳边/内部约束和deleted恢复，不分配新节点或提交连接。MTriangle圆内重载不减容差；已deleted邻片不贡献shell。原搜索可进入predeleted seed：正常失败清空腔成员标记；取消/异常则恢复入口状态，两者不能混同。

新增定位/空腔单元从RED到GREEN，79冻结原生叶通过（含strict圆边界、零度量/奇异反映射、内部边、旧deleted、显式force和循环搜索上限）。完整自研普通/周期CAD前缀串到初始候选后，对每个候选独立构造/恢复空腔：extension=0/1各普通44/周期40，共168阶段、10718候选、114850空腔成员及136054壳边逐值一致；132次无包含片和122次搜索也与参考一致。

参考leaf调用实际原生circle/DFS/invMap/search，但其initial insertAPoint控制仅镜像，未调用完整插点。报告`parametric_domain_gate_applied=false`及`actual_insertion_feedback_validated=false`；这是初始候选空腔验收，不是外循环实际序列或完整生成。原1D/拓扑/raw反求/指针序仍冻结，候选/度量/包含片/壳边由自研计算。

53Release目标、48CTest（38复现+10既有）及10Python通过；观察补丁173329字节/61导出/10文件真实重放，9原始文件与固定发行包一致；8原版/off/on指纹相同。5最终报告的DLL/EXE SHA及观察源码/补丁SHA已复核。167原源码边界仍仅原6文件变化，新增纯定位/空腔测试及重放只依赖CRT/KERNEL32，不链接Gmsh/OCC/Qt。

证据：仓库外`MeshStudioValidation/evidence-20261007-reproduction`中的`ordinary-cad-front-cavity-extend{0,1}-release-oracle`、`periodic-front-cavity-extend{0,1}-release-oracle`、`front-cavity-leaf-release-oracle`、`observer-front-cavity-neutrality`及`front-cavity-release-sha-audit.json`。下一步实际新节点CAD/BGM/append生命周期、insertVertexB验收/提交和FrontGeneration参数域门控/动态反馈；后处理/全几何/失败重建/组装与最终质量仍待完成。后续合同见`.planning/2026-10-07-gmsh-reproduction-research/front-insertion-source-contract.md`。

## 候选点/普通OCC圆修正（2026-10-08，当前）

FrontPointPlacement分computeFrontPoint和correctFrontPointOnSurface，对应optimalPointFrontal/B；保留原算术序、原边号、端点mesh/BGM均值、cosine=n2/sine=n1、norm<1e-12跳过、失败保留快候选与有符号L/半径/epsilon。直接CAD度量和支撑面Value独立求值；取消/预算不吞为普通失败。GmshFrontPointPlacementComparison实际调用两个native函数，最终front-point-cad-signed-release-oracle230叶含228有限逐值一致、2非有限诊断；native NaN/401查询未逐步复刻。

--check-front-points在普通/周期自研前缀初始化并排空队列后对每个允许插点的三角计算候选。参考在对应native前缀状态上调用真实候选叶；没有空腔/插点反馈。ordinary-cad-front-point-extend{0,1}-signed-release-oracle各44、periodic-front-point-extend{0,1}-signed-release-oracle各40，共168候选阶段/10718候选/82792实际曲面求值逐值一致，10568修正/150保留。自己的候选/度量/Value未冻结，1D/拓扑/raw反求/指针序仍冻结。前缀报告同时验证此前节点/三角/队列；不能替代完整生成/质量验收。

50相关Release目标与46CTest（36复现+10既有）/10Python通过，最后有符号修正后的5报告SHA与当前DLL/EXE一致，见front-point-signed-release-sha-audit.json。纯候选单元只依赖CRT/KERNEL32，167原源hash仍仅六必要变化。当前补丁164237字节/60导出真实重放、9pristine/archive及10观察源SHA一致，8原版/off/on neutrality同。较早first/release/controls目录为历史。任务8仅候选子项完成，实际包含片插值/空腔/插点/NaN失败处理、剩余几何/平滑/回退/组装及最终质量/性能/GUI未完成；整体active。以下旧检查点为历史。

## 初始前沿活动集与状态调度（2026-10-08，当前）

FrontTriangleOrdering提供初始与运行期共享compareTri3Ptr键；FrontQueue保存运行期AllTris/ActiveTris，FrontMesh::orderedTriangles只保留初始快照。findActiveEdge按0..2取首个无邻片或0<邻R<limit的边，不检查邻片deleted；目标deleted保留调用者edge。初始活动片即使自身R<=limit也入集，只有非活动且R<limit才break。pop一次只消耗一个头，分开记录deleted/活动检查/自身R>limit，保留跳过事件。registerFrontTriangles仅登记，updateActiveTriangles只按原邻域序把活动且自身R>limit的片入集，保留已有陈旧项；markInsertionFailure要求目标已pop，从AllTris移除后R=0/-1再插入，保留set冲突和创建池，不替代空腔回滚。

参考观察辅助编排逐行沿用bowyerWatsonFrontal的初始化/pop控制语句，调用真实native isActive/compareTri3Ptr/forceRadius/std::set；这些编排是显式oracle，不是整段生产前沿循环已运行。GmshFrontQueueComparison以冻结MTri3半径/邻接/删除状态做61组叶对照，含nextafter阈值与正subnormal、deleted邻片、原edge保留、初始break、失败0/-1与冲突、陈旧删除/邻接替换、局部注册/更新及空集。邻接替换与删除是测试输入操作；实际几何插点失败、new_cavity回滚/成功连接及反馈仍未对照。

--check-front-queue在真实buildMeshGenerationDataStructures之后取native活动集和排空序列，自己的普通/周期完整前缀计算三角状态和活动结果。最终ordinary[-cad]-front-queue-extend{0,1}-release-oracle各24原位+44阶段，periodic-front-queue-extend{0,1}-release-oracle各80embedded/80chart及初始至队列各40；共256阶段/16480pop/16298允许插点/182低半径跳过，逐值零容差一致。实际前缀排空没有拓扑变化/非活动陈旧项，不证明动态插点循环。队列/半径/连接不冻结，raw反求UV/节点指针序及1D/拓扑元数据仍冻结；独立CAD求导变体不代表全部反求通过。

模块stub真实RED12→GREEN，另增加取消/非法局部登记/陈旧连接/未pop键变更/非法失败类型原子性单元通过。48相关Release目标（front-queue-release-targets.txt）及随后4个重放的测试邻接操作重建通过；45CTest（35复现+10既有）/8Python通过。纯FrontQueueReplay只有CRT/KERNEL32，无Gmsh/OCC/Qt；167既有源hash仍仅六必要变化。最新158779字节/59导出/10文件补丁与9pristine/archive一致，gmsh-observed-front-queue-stale-patch-replay-20261008真实Git apply/10SHA一致；observer-front-queue-stale-neutrality8原版/off/on指纹同。当前release报告二进制SHA一致，first/final及旧neutrality报告为历史保留。

Task7仍缺实际包含片插值与动态插点反馈，下一步FrontPointPlacement/FrontCavity；源码普通CAD圆修正求交失败保留原候选并返回true，不应提前拒绝。完整目标、全部几何/失败重建/平滑/回退/组装和最终形状/CAD/GUI/性能验收未完成，以下为历史。

## 普通/周期前沿初始化（2026-10-08，当前）

新增FrontMeshTypes及FrontTriangleInitialization::initializeFrontTriangles，对应buildMeshGenerationDataStructures三角部分：保留字面0.3333333333均值和加法序，按实际Extend1dMeshIn2dSurfaces选择目标尺寸；MTri3 radiusNorm=2调用物理外接圆后除以该尺寸。AllTris降序R/h、同值按排序节点身份决胜；记录set拒绝但保留创建槽，不标deleted。connectTriangles只遍历AllTris成员，保留原边号、非稳定std::sort边键及相邻相等边成对连接。没有活动边/候选/插点或自有质量规则。

参考显式oracle现在能在实际节点阶段后继续执行原版MTri3构造/AllTris插入/connectTriangles，记录后清理所有创建对象；节点专用oracle继续在三角构造前返回，正常生成不截断。--check-front-triangles继续自己的完整普通或周期前缀，不输入原生均值、半径、排序或邻接。反求UV/原生节点指针遍历序、原1D/拓扑/副本元数据仍冻结；resolved extension是参考CAD选项元数据。固定一阶算法6/radiusNorm=2/无boundary layer，不能据此宣称其他norm/边界层或全部几何通过。

最终ordinary[-cad]-front-triangle-extend{0,1}-final-oracle各24原位+44actual；periodic-front-triangle-extend{0,1}-final-oracle各80embedded/80chart及初始至节点/三角阶段各40。共256actual/18472三角和18928个成对邻接逐值零容差一致。普通每矩阵3076三角/3106邻接，周期每矩阵3084三角/3252邻接；实际矩阵均无set重复拒绝，其覆盖仍只有源码/单元。周期节点40actual的2956节点/128embedded BGM/80内部约束/630副本/690次UV覆盖/128不同双尺寸一致。

单元先见stub真实RED10，再修正“数学同半径必然double同值”的测试假设（两个首边均沿坐标轴以构造真正并列），未修改物理圆算法。新增有限输入求和溢出在原生会形成非有限尺寸，自研显式NonConverged，先见RED再实现；退化有限哨兵、取消/预算/非法索引/非有限输入和输出原子性通过。非有限路径诊断不冒充原版继续执行等价。

46相关Release目标（front-triangle-release-targets.txt）及随后三重放目标最新注释/usage重建通过，44CTest（34复现+10既有）/8Python通过。ordinary-node[-cad]-front-triangle-regression-oracle、ordinary-metric[-cad]-front-triangle-regression-oracle各24+44及face-mesh-leaf-front-triangle-regression-oracle24通过。纯三角单元/普通重放仅CRT/KERNEL32，无Gmsh/OCC/Qt；167既有源hash仍仅六必要变化。151280字节/55导出/10文件观察补丁与9pristine/archive一致，gmsh-observed-front-triangle-patch-replay-20261008实际Git apply/10SHA一致；observer-front-triangle-neutrality8原版/off/on指纹同。当前报告DLL/EXE SHA一致，历史报告保留，SHA不代表当前构建。

下一步活动集/活动边→候选点→空腔/插点，继续源码逐阶段对照。Task7、完整目标、全部反求/失败重建/平滑/回退/组装和最终形状/CAD/GUI/性能验收仍未完成。以下为历史检查点。

## 普通面转换与前沿节点阶段（2026-10-08，最新）

普通重放新增ordinary-face-mesh，先观察真实案例以unknown BDS replay mode失败，再接共有initializeFaceMeshVertices/convertBdsToFaceMesh。纯/CAD求导各44actual全初始/恢复/域/尺寸/清理/度量前缀通过，1552条deleted记录次序一致；0新面节点/0身份过滤/0缝副本，相关创建/过滤仍由24转换叶覆盖。--check-face-mesh只截取实际BDS2GMSH三角分支，保留原1D MVertex元数据冻结及新身份归一合同。

新增NodeSizeInitialization，只对应meshGFaceOptimize.cpp:174–250节点部分，止于MTri3之前。单元RED12→GREEN覆盖半段覆盖embedded规定值、普通background=mesh、未引用embedded的map[]零初值、fake-original直接边排除而两fake同original仍保留零边、原生反求先于周期UV覆盖、degenerate曲线跳过、内部约束、预算/取消/非法顺序和几何失败原子性。实际前沿驱动先以未知模式RED，再复用该模块。

--check-front-node-sizes调用真实buildMeshGenerationDataStructures，仅显式oracle在data.addVertex之后返回，正常生成不截断。节点尺寸按自研物理边/embedded/边界链计算，不输入原生size结果；每节点原反求UV和原生vSizesMap指针遍历序显式冻结，embedded规定值及首MVertex身份作为CAD元数据冻结。每个embedded BGM由自己的evaluateMeshSize计算；--metric-cad只证明前面的度量D1来自自有CAD，不能称前沿反求适配通过。

普通纯/CAD求导各44actual的3074节点/反求记录、16embedded BGM、38内部约束和16不同双尺寸逐值零容差一致。report前沿反求适配false、完整前沿false；周期节点实际对照、MTri3/邻接/队列/候选/腔/插点、全几何/失败重建/平滑/回退/组装及最终形状/CAD/GUI/性能仍未完成。以下为历史。

最终证据ordinary-face-mesh-final-oracle、ordinary-face-mesh-cad-final-oracle、ordinary-front-node-size-final-oracle、ordinary-front-node-size-cad-final-oracle各24原位+44actual；ordinary-metric[-cad]-front-node-size-regression-oracle各24+44，periodic-front-node-size-regression-oracle保留80/80及初始至转换各40，face-mesh-leaf-front-node-size-regression-oracle24。45相关Release目标（列表front-node-size-release-targets.txt）构建后43CTest（33复现）/8Python通过；新增3个查询控制单元随后重建/通过，不改变计算逻辑。报告与当前DLL/EXE SHA一致，167既有源仅六必要变化。纯节点单元/普通重放仅CRT/KERNEL32，无Gmsh/OCC/Qt依赖。

观察补丁145852字节/52导出/10文件，9pristine逐字匹配固定tgz；gmsh-observed-front-node-size-patch-replay-20261008实际Git apply/10SHA一致。observer-front-node-size-neutrality8原版/off/on指纹同。新增显式front节点截取不构造MTri3，不能用这份通过报告证明半径/邻接/队列完成。

## 周期接缝与共有面转换（2026-10-08，最新）

新增FaceMeshTypes、PeriodicSeamReconstruction、FaceMeshConversion及独立单元/重放。单元真实RED10→GREEN。周期--face-mesh在自研CAD图表/初始/BDS/恢复/域/清理/度量之后绑定原MVertex元数据，再按源码recoverMap局部ID序重建临时0D/1D副本、UV和等价映射，最后执行BDS2GMSH三角转换；不折叠缝。dim>=2重复保留原身份与第一次UV，显式记录诊断。

periodic-face-mesh-final-oracle含80嵌入叶/80完整图表，以及初始、BDS转换、恢复、域、清理、度量、面转换各40actual对照；630临时副本/3084三角/492条deleted记录逐值零容差一致。该actual矩阵没有新面节点、身份退化过滤或dim>=2缝诊断，不能据此宣称这些实际CAD分支通过。face-mesh-conversion-leaf-final-oracle的24组实际原生BDS2GMSH对照补齐新节点创建/复用、六排列、相同XYZ不同身份、创建后过滤、deleted顺序、负局部ID和空池；叶证据不替代普通完整CAD驱动。unsupported缝分支目前只有源码和单元证据。

原1D元数据、拓扑枚举/副本序/物理边链仍冻结；CAD值、度量、UV副本映射和输出连接由自研前缀计算。原物理ID保留，临时/新原生指针以实际创建序归一身份，未对比原生进程全局MVertex编号。报告complete_front_generation_validated=false。19受影响Release目标构建、42CTest（32复现）及8Python通过；ordinary-face-mesh-metric[-cad]-regression-oracle各24原位+44actual回归通过，当前报告DLL/EXE SHA已复核。纯转换测试/重放只有CRT/KERNEL32依赖，无Gmsh/OCC/Qt。

观察补丁139523字节/50公开入口/10文件，真实Git apply及10SHA一致；9pristine源与固定tgz一致。observer-face-mesh-neutrality的8原版/关闭/开启完整生成指纹同。普通完整面转换、全几何/失败1D重建、前沿节点尺寸/MTri3/队列/候选/空腔/插点、平滑/回退/组装及整模型形状/CAD/GUI/性能仍未完成。以下为历史记录。

## 周期BDS恢复正常路径（2026-10-08）

最新周期初始度量（2026-10-08）：GmshPeriodicEmbeddedCadReplay新增--bds-metric，首先stub运行实际矩阵明确RED“periodic metric stage not implemented”，接同一legalizeInitialMesh后32actual全池/度量/决策一致，但0成功交换。增加圆柱/球面各15嵌入CAD点，在默认扰动下增加8actual压力例，40actual共3654查询/246交换/63扫描/124资格拒绝，全部零容差一致；测试要求两类压力均实际成功交换。周期图表/初始/转换/恢复/域/清理也都从同一BREP独立计算，源数据仅1D/拓扑/副本序/物理链冻结，没有冻结度量或输出连接。

periodic-bds-metric-final-oracle保存80嵌入叶/80CAD全图表和40各后阶段；metric_cad_derivatives_from_reference=false、periodic_metric_actual_swaps_validated=true。共享GmshBdsMetricJson仅测试序列化；普通纯和独立CAD各24原位+44actual度量回归通过。3受影响重放目标构建、41CTest（31复现）/8Python通过。129674字节/48导出/10文件补丁实际Git apply与10SHA一致，9pristine与固定tgz同；observer-periodic-bds-metric-neutrality8原版/off/on最终指纹同。周期缝MVertex临时副本、BDS转GFace、前沿尺寸/队列/布局/腔/平滑/失败重建和全部最终验收未完成，下文为较早阶段。

后续周期域/清理已接入（2026-10-08）：GmshPeriodicBdsDomainTest真实RED4→GREEN；deleted首负点种子、CLASS_E传播、重复LIFO、无内域时显式空域和取消原子性都有检查。classifyPeriodicBdsDomain独立入口，共享LIFO但保留周期外域(3,2)、跳过deleted、无普通负对点补填。prepareBdsInterior共享普通/周期同一清理算法，原普通入口兼容委托。

periodic-bds-domain-interior-final-oracle保存64叶/64CAD全图表/32初始/32转换/32恢复/32域/32清理原生对照。每个目标均从自研CAD/图表/初始连接/恢复完整计算，不输入原生域/清理输出；1D/拓扑/副本指针序/物理曲线链冻结。零容差比较全部池、邻接、临时分类来源、重复pop及两次cleanup映射；32域有4296重复pop、2124内域片/2140外域片，清理删除2268孤边/128外围点。此正常矩阵无恢复失败/重试，deleted种子差异为单元证据，不能称全部失败driver通过。

17受影响Release目标（含MeshStudio）已构建；41CTest（31复现）/8Python通过，普通域/清理各24原位+36actual、独立CAD普通度量24+44复核通过。当前观察补丁128376字节/47导出/10文件，9pristine与固定tgz一致；gmsh-observed-periodic-domain-interior-final-patch-replay-20261008真实Git apply和10SHA通过，8原版/off/on最终指纹一致。周期度量/失败1D重建/前沿/后处理/回退/组装及整模型/CAD/GUI/性能未完成。下文为较早恢复阶段记录。

新增独立PeriodicBdsRecovery::recoverPeriodicBdsConstraints，embedded先逐leaf，无registry；任意失败退出。之后逐闭合环，已有边跳leaf，缺边恢复后查首物理CAD链owner，成功也登记attemptedBoundaryCurves。全部成功边临时CLASS_E=(1,1)/FaceBoundary来源，不补0D分类或登记CAD实体。boundary失败继续并设needsRetry，refineAll只在embedded全成功后初始化，交后续1D重建/递归编排。单元RED9→GREEN，取消/非法保持输出。

periodic-bds-recovery-comment-final-oracle含64嵌入叶/64独立CAD全图表/32初始/32BDS转换/32actual恢复，全前缀连接自研；80embedded调用/2012环边，全池/邻接/临时分类来源/逐leaf扫描和调用事件逐值零容差一致。只冻结1D/拓扑/副本序与CAD物理线段，不冻结初始连接。矩阵没有缺边/翻转/失败/重试，不能外推全部失败驱动或周期域/清理验收。

16受影响Release目标含MeshStudio构建结束后40CTest（30复现）/8Python通过，注释后CAD重放/新单元重建并再次对照。普通六前阶段各24+36、度量纯和独立CAD各24+44、270/725/42度量叶、720/136恢复叶回归通过。观察补丁121198字节/45导出/10文件真实Git apply且SHA一致，8原版/off/on完整网格指纹同。完整前沿/后处理/失败回退/组装及最终形状/CAD/GUI/性能未完成。

## 普通BDS初始度量合法化（2026-10-08）

`BdsSwapQualification`和`InitialMeshLegalization`真实单元RED13→GREEN。原版Quality(false)仅关闭gamma改善，低连接度/缝/退化/参数面积守恒仍开启；面积为raw double，数组度量圆容差针对平方半径。四节点ID先排序登记，再查询度量/圆/资格；配置跨轮保留，动态扫描同轮追加边，不清理旧池或重置modified。

ordinary-bds-metric-first-oracle：24原位+44actual普通面，额外含剪切参数化/非平面B样条两BREP，4131查询/776交换/82扫描/4圆内资格拒绝，全池与每次决策零容差一致。这里只冻结CAD导数输入，自己的全初始/恢复/域/双尺寸/cleanup/度量圆/连接独立计算，不输入结果连接。

独立CAD第一次ordinary-bds-metric-cad-first-oracle明确失败：parameter-shear-0.3-constant-face1，原支撑面D1与BRepAdaptor B样条缓存求导的末位不同，外心x为0.7816685267857145/0.7816685267857146。沿OCCFace.cpp:262–269增加独立faceSurfaceDerivatives直接支撑面D1，不扩大比较容差；ordinary-bds-metric-cad-direct-oracle24+44全值通过。前缀CAD图表/1D/原MVertex元数据仍冻结，不冒充完整CAD适配器验收。

initial-metric-leaf-first-oracle：270面积（含轴1/2/未知flag/多极点/上下溢/顺序）、725数组度量圆（含奇异原点外心/平方半径容差门槛）、42actual Quality(false,true)交换（含保护/分类继承/全连接）逐值一致。度量模块对非有限输入显式失败，任意IEEE极端值完整等价、全部周期driver和最终质量仍未验收。

最终重建/复核：48相关Release目标顺序构建结束后39CTest（29复现）/8Python通过，包含新增支撑面求导的B样条/Location/方向/重复查询/非法ID。ordinary-bds-metric-final-oracle与ordinary-bds-metric-cad-final-oracle各24+44，initial-metric-leaf-final-oracle270/725/42；六旧普通矩阵各24+36、周期64/64/32/32、恢复720/136都通过。观察补丁114750字节/44导出/10文件真实Git apply及SHA同，8原版/off/on完整算法6指纹同；167既有源仍仅六必要变动，纯度量叶重放仅CRT/KERNEL32。使用转换对照的`--check-metric`，CAD变体另加`--metric-cad`和对应CAD EXE；叶接口使用GmshInitialMeshLegalizationComparison.py。所有构建/证据位于仓库外evidence-20261007-reproduction，未替换正式EXE。

## 普通BDS内域池准备阶段（2026-10-08）

`prepareOrdinaryBdsInterior`真实RED7→GREEN；删空域面/cleanup、孤边删除/空边临时CLASS_F/点分类只降维或填空、第二次cleanup、最后删-1..-4，保留std::partition非稳定排列与全池映射/原邻接序/分类来源/双尺寸/登记表/ID上限及其它孤点。活外围点仍有边时显式InvalidTopology避免原版悬空指针，该异常只有单元覆盖。

ordinary-bds-interior-release-oracle24拓扑+36actual全前缀逐值零容差通过：2557空域面、2685孤边、144外围点删除，面/边池36组全部重排，最终2521片。两次重映射与全池/身份/邻接全部比较；连接/尺寸自研计算，前缀CAD/1D/UV/原MVertex查询元数据仍冻结。尚未执行metric合法化，也不是完整CAD/周期/全部失败或最终形状质量验收。

44Release目标顺序构建结束后37CTest（27复现）/8Python通过。内域/尺寸/域/embedded/boundary/conversion六最终矩阵各24+36，bds-recovery-interior-regression-oracle720/136、periodic-bds-interior-regression-oracle64/64/32/32复核；观察补丁103710字节/40导出/10文件真实Git apply及SHA一致，observer-bds-interior-neutrality8原版/off/on指纹同。167既有源仅六必要改动，纯重放仅CRT/KERNEL32；整目标未完成。

## 普通BDS节点BGM双尺寸阶段（2026-10-08）

`initializeOrdinaryBdsNodeSizes`真实RED8→GREEN；原MVertex实体/物理XYZ/曲线参数按BDS ID序查询，区别于临时BDS域分类和面坐标。0D(0,0)/1D(u,0)求BGM同时赋两个尺寸；高维直接MAX_LC，未映射点保持，initial-only跳过。取消/预算/非法查询/几何失败不发布工作副本。

ordinary-bds-node-size-release-oracle：24拓扑+36实际普通meshGenerator全前缀、2503实际BGM调用的查询序/值及全池零容差一致。自研组合尺寸/显式回调公式，不冻结尺寸结果；原实体/XYZ/参数/模型特征长度、前缀1D/UV/图表/曲线元数据仍冻结。实际高维点0例，不能把MAX_LC单元称实际CAD覆盖，也不验证自己的完整几何适配、清理/度量/前沿/整模型质量。

43相关Release目标**顺序构建结束后**36CTest（26复现）/8Python通过；此前并发测试锁住EXE导致LNK1104，失败日志保留，重跑见bds-node-size-sequential-final-build/ctest.log。节点/域/embedded/boundary/conversion五最终矩阵各24+36，bds-recovery-node-size-regression-oracle720/136，periodic-bds-node-size-regression-oracle64/64/32/32通过，7报告SHA与当前DLL/EXE一致。补丁99124字节/39导出/10文件真实重放及SHA一致，observer-bds-node-size-neutrality8原版/off/on指纹同；167既有源仅六必要改动，纯重放仅CRT/KERNEL32。

## 普通BDS域/域后embedded阶段（2026-10-08）

`classifyOrdinaryBdsDomain`单元RED8→GREEN；`recoverOrdinaryBdsEmbedded`新增RED4→GREEN。域保留临时分类origin、重复LIFO访问和原种子顺序；embedded原版外层忽略curve bool，fatal另记而继续后续曲线。完整普通初始/转换/曲线恢复/域前缀由自研计算，不冻结结果连接；仍冻结1D/UV/副本序/独立BDS图表/四CAD查询值及曲线拓扑元数据。

ordinary-bds-domain-release-oracle：24拓扑+36actual，9588访问/4886重复/2181外域清空/2521活内域；ordinary-bds-embedded-release-oracle：24拓扑+36actual，38段/4实际翻边；全池/分类身份/访问与逐叶尝试零容差一致。两阶段均未执行尺寸/清理，不能据此称完整CAD或全部失败/周期驱动通过。

42Release目标、注释后受影响目标重建、35CTest（25复现）/8Python通过。ordinary-bds-domain-boundary-release-oracle与ordinary-bds-domain-conversion-release-oracle各24/36，bds-recovery-domain-regression-oracle720/136，periodic-bds-domain-regression-oracle64/64/32/32通过。95293字节/38导出/10文件补丁真实Git apply及SHA一致，observer-bds-domain-embedded-neutrality8原版/off/on指纹同；167既有源仅六必要变动，纯重放仅CRT/KERNEL32。失败构建/首轮汇总日志保留，整模型/质量/GUI/性能未验收。

核心实现不链接 Gmsh。这里的工具只在测试端显式加载仓库外的固定参考 DLL，不调用系统默认 Gmsh。版本、库 SHA256、构建选项、网格参数和实际算法随结果保存。

## 参考构建

从官方 `gmsh-4.15.2-source.tgz` 建独立源码副本，先构建并保留无观察 DLL，再在另一副本应用 `GmshObservedHooks.patch`。不得对研究用原始目录应用补丁。源包、补丁和当前 9 个相关文件（8 原源码+观察头）哈希在 `GmshObservedHooks.manifest.json`。

补丁包含观察头文件；外部头保留 UTF-8 BOM，避免当前 Gmsh Windows 编译器 CP936 解码中文注释。Git 应用后统一这 9 个外部文件为 LF，再核对 observed_sha256；已在独立副本逐字节重放验证。原始源包和原审查源码不修改。

参考选项：VS2022 x64 Release；`DEFAULT=0`、`ENABLE_MESH=1`、`ENABLE_OCC=1`、`ENABLE_BUILD_SHARED=1`、`ENABLE_PRIVATE_API=1`、`ENABLE_EIGEN=1`。OCC 路径显式传入 `OCC_INC` 和 `CMAKE_LIBRARY_PATH`。实际版本为 4.15.2-git，实际 BuildOptions 包括 Eigen、OpenCASCADE、Mesh、TetGen/BR、tinyobjloader；不按请求选项猜测构建结果。

当前 DLL 位于仓库外 `MeshStudioValidation/research/gmsh-observed-build-20261007/Release/`；几何核心和参考构建互不链接。

## 工具职责

| 工具 | 输入/检查 |
|---|---|
| GmshReproductionComparison.py | `run` 保存参考网格和阶段；`compare` 比较精确指纹；`compare-trace` 定位首次阶段/实体/字段分歧 |
| GmshObservationMatrix.py | 同一冻结 BREP，原版/观察关闭/观察开启各独立进程；8 个案例验证钩子无污染 |
| GmshCurveCadReplay | 从同一 BREP 用 OCC D1 重放自研曲线模块，输出累计表、计数和参数 |
| GmshBoundaryCadReplay / GmshBoundaryComparison.py | 同一 BREP、显式 modelLC，比较最终 0D/1D 节点身份/坐标/参数和有向线段；OCC 只在测试端链接 |
| GmshBoundaryRecoveryComparison.py | 调固定原生 ordinary-map remeshUnrecoveredEdges，对照首/末/内段连续恢复的节点/参数/分类/getLc |
| GmshBoundaryFieldComparison.py | endpoint/graded/step 回调场的生成期边链/分配数/删除空洞；独立记录最终 Renumber 差异 |
| GmshFaceChartComparison.py / GmshFaceChartReplay | 冻结实际 getNodeCopies 的 UV/顺序/面坐标，验证普通图表组装；纯重放程序不链接 CAD/Gmsh |
| GmshPeriodicBoundaryLoopComparison.py / GmshPeriodicBoundaryLoopReplay | 冻结边在面上的两套精确 pcurve；调用原生 buildConsecutiveListOfVertices 对照方向、缝分支、顺序和闭合 |
| GmshPeriodicBoundaryChartComparison.py / GmshPeriodicBoundaryChartReplay | 原生连续环/BDS 初始化、分类/双尺寸/编号/包围盒/counterpart；重复环显式标为复用压力输入 |
| GmshPeriodicBoundaryChartCadReplay | `--cad-executable` 额外用生产 OccCadGeometry 查询周期 UV/裁剪范围/面坐标；1D 网格和环拓扑次序仍冻结 |
| GmshTrueBoundaryComparison.py / GmshTrueBoundaryCadReplay | 同一 BREP 独立精确 pcurve 的固定 300 点采样，和实际原生 trueBoundary 逐值比较；参考仅提供唯一边拓扑序/朝向 |
| GmshOccPcurveTest | 生产几何接口的原朝向/反向 seam、退化范围、裁剪 UV 范围和失败输出保留 |
| GmshOccProjectionTest / GmshOccProjectionComparison.py / GmshOccProjectionCadReplay | 生产支撑曲面投影；同 BREP 实际 OCCFace::closestPoint 的 UV/最近 XYZ/重建 XYZ 对照 |
| GmshPeriodicEmbeddedChartTest / GmshPeriodicEmbeddedComparison.py / GmshPeriodicEmbeddedCadReplay | 实际 meshGeneratorPeriodic 截取新增嵌入 BDS/约束/DocRecord 前缀；`--check-full-chart` 另验证独立 CAD 的完整图表串联 |
| GmshDivideAndConquerTest / GmshDivideAndConquerComparison.py / GmshDivideAndConquerReplay | 实际 DocRecord 排序/递归切分/切线合并/三角形序；纯重放核心不依赖 CAD/Gmsh |
| GmshPeriodicInitialTriangulationTest / PeriodicEmbeddedComparison `--check-initial-triangulation` | 同 BREP 独立 CAD 图表→扩盒/四点→分治，有向连接及排序/外围节点逐值比较，约束恢复前截取 |
| GmshObservedPatch.py | 审核固定 tgz 与当前 8 原始源码、生成补丁，在新外部目录真实应用并检查 9 SHA；不改原审查目录 |
| GmshGeometryReference.py | 冻结数值输入，调用原生标量函数，再启动 GmshGeometryReplay 对照；同时记录失败诊断差异 |
| GmshGeometryReplay | 纯自研几何/数值重放程序，不依赖 Gmsh/OCC |

运行 Python 加 `-B`，构建、结果、缓存均放仓库外。CLI `--help` 给出必需的 API、DLL、runtime、输出路径。观察输出和矩阵目录要求新路径，保留失败证据。

BREP 导入使用 `highestDimOnly=False`，以保留独立点和低维几何。embedded 关系不属于 BREP 拓扑；固定 embedded 案例需显式 `--restore-fixture-embedding`，按唯一坐标/面身份恢复，缺失或歧义时失败。不能把丢约束的 BREP 运行当作原案例。

## 当前证据边界

8 案例的观察矩阵通过，曲线表/计数/参数已有常尺寸对照；底层数值案例涵盖度量圆、物理圆、符号判据、三阶 QR、曲线平滑、求交、数值方向接口及实际 filterPoints。过滤含常尺寸和变化 callback 尺寸，检查最终保留的物理身份；测试端临时节点只放在外部参考模型，核心不依赖该模型。构造按明确浮点容差比较，符号/身份严格比较；数值方向额外使用相对面积容差和严格符号，不用绝对 1e-12 掩盖小面积差异。

求交案例有一例原生在非有限算术后继续迭代，自研提前报错；双方失败并保留初值，但次数不等价，报告单列。此类诊断差异不会放宽有限算术路径的检查。

最终边链的 8 个常尺寸案例使用参考 generate(1)，包含 BREP 内的独立点和退化边端点；身份/分类/线连接严格比较，坐标与参数绝对容差 1e-12。modelLC 取参考模型包围盒对角线（OpenFile.cpp::SetBoundingBox）；完整查询调用次数不作为此项等价结论。参考默认最终编号与生成期空洞的差异，仍需在实际删除案例按阶段单独验证。

BoundaryRegistry/BoundaryRecovery 单元覆盖重合实体不合并、闭环移动时机、退化/零长、突变尺寸过滤后的编号空洞、参数中点、getLc==-1 的节点分类查询、周期端点分支、陈旧请求与预算/取消原子性。8 BREP 的 ordinary-map 恢复实际函数对照已通过；periodic multimap/all、面失效重建及 OCC 反求失败的通用回退尚未完成。

18 个变化 callback 场案例的生成期 boundary_chain 及分配数通过；10 例实际删除节点。10 例最终输出身份的差异由 GModel::mesh 后 renumberMeshVertices 的 full-save 分支解释，测试端按源码顺序映射后所有节点/参数/连接相同；未把生成期编号改成输出编号。

FaceChart 普通路径单元及 6 个面原生冻结 UV 重放一致，两个带 seam 的面明确留给周期入口。周期环组装共 108 个实际原生函数对照（6 BREP、9 环、6 档容差、两种首缝选择）一致，浮点 UV 按零容差比较，身份和顺序严格比较。原生测试使用每个环独立的空 BDS/recoverMap，仅验证环选择/闭合；跨环复用、BDS 尺寸/counterpart、embedded 和整个周期初始剖分不属于这 108 项。未闭合、等距候选、非有限输入和原子失败另有单元证据。自己的 CAD 求值适配器尚未验证，两类冻结输入重放不得称完整几何接入通过。

周期边界 BDS 增加 7 BREP、两尺寸场、一次/两次/三次环重复，共 54 项原生对照：18 项保留实际 CAD 环序，36 项为重复环复用压力输入，不能当完整有效面输入验收。边界节点编号（含复用后空洞）、分类、UV/面坐标、lc/lcBGM、raw 包围盒、环引用、所选容差/首缝及 counterpart 零容差一致。新增 torus 冻结于外部证据目录，OCC addTorus(0,0,0,2,.6) 生成，覆盖两方向 seam 和同一物理节点的四个 UV 副本。trueBoundary、embedded/初始剖分和自研 CAD adapter 不在此项范围。

这些证据尚不表示初始/周期/前沿/平滑/回退/组装全部实现，也不等于整模型形状质量、CAD 精度、GUI 或性能验收。

新增精确几何阶段：9 面 trueBoundary、9269 条线段与实际原生函数零容差一致；范围/UV 由同一 BREP 的生产 OccCadGeometry 查询，输入只固定参考唯一边的拓扑序/原朝向/seam。PeriodicBoundaryChartComparison 增加 --cad-executable，54 组额外重放独立求 pcurve UV/面坐标/裁剪范围，BDS 输出零容差一致；仍冻结参考 1D 网格和环次序。首次圆柱范围造成 1 ULP 容差差异，失败保留 periodic-boundary-production-adapter-oracle；按 OCCFace 的 ShapeAnalysis::GetFaceUVBounds 新增独立 faceTrimParameterBounds 后通过，最终证据 periodic-boundary-production-adapter-exact-bounds。精确 pcurve 接口曾在 default-false 下真实 RED，范围能力也先见 5 个 RED，再完成实现；GmshOccPcurveTest 验证 60 次 pcurve 查询、5 面范围及非法身份输出保留。generic/robust 反求、完整代理链/曲率/嵌入和初始剖分不属于这项通过。

支撑曲面投影新增 108 次实际 OCCFace::closestPoint 对照（7 BREP/9 面、两位置偏移/两种子），UV/最近 XYZ/面重建 XYZ 零容差一致；自己的投影不读取参考参数。孔内投影、周期及变换位置另有 54 次单元查询。default-false 接口先实际失败，再实现惰性独立 projector；OCC projector 不可拷贝，使用稳定 unique_ptr 所有权。generic 反求、启用 robust 模式的回退和完整生产代理链仍未验收。

嵌入叶阶段有 64 次实际 meshGeneratorPeriodic 对照，独立求嵌入投影/pcurve/面坐标/尺寸，仍冻结 1D/边界 BDS/副本指针序。带 `--check-full-chart` 的额外 64 次从同 BREP 独立组装边界 BDS 和所有 DocRecord 点，不输入参考 UV/XYZ/范围/尺寸/BDS 编号；trueBoundary、全 BDS、原生嵌入约束及全部初始点顺序/扰动逐值相同。两尺寸场、扰动 0/1e-9、种子 1/20261007 都覆盖。1D 网格、CAD 唯一边/环序及副本指针序仍是冻结拓扑输入。24 次为正常圆柱/球/环面周期输入（无嵌入），24 次平面强制调用周期私有入口，16 次圆柱/球 seam 压力仅一个端点线段；仍未验证完整算法 6 网格输出。

原生 seam 选择失败可能使实际新增点数超过 DocRecord 预估，且后续 facile 空覆盖会丢失约束。纯核心测试保留该诊断；完整入口明确返回 InvalidTopology，避免复制原版越界写。不运行会触发无效原生数组写入的含 seam 内点压力变体。完整 General 图表缺少扰动流显式返回 InvalidInput，零系数仍消耗随机调用，边界 <3/==3 早退先于嵌入/扰动要求。

最新串联证据在仓库外 `evidence-20261007-reproduction/periodic-full-chart-normal-periodic-oracle/periodic-embedded-report.json`（64 叶阶段+64 串联）；投影证据在 `occ-projection-final-oracle/occ-projection-report.json`。外围四点、扩盒、初始普通 Walk/周期 divide-conquer、前沿、平滑、回退、最终恢复和组装均不在上述通过范围。

新增周期初始阶段证据：`periodic-initial-default-random-oracle/periodic-embedded-report.json` 在 64 叶阶段/64 图表串联外增加默认 Mesh.RandomFactor=1e-9 的 32 次实际周期入口对照（12 正常周期、12 平面强制周期、8 端点 seam 压力）。独立 CAD 的 raw boundary 盒/原生 makeCube 舍入/3.5 扩盒、四个外围 UV/XYZ/分类/双尺寸、排序后的 DocRecord 点及有向 localId 连接序全部零容差一致；实际 BDS 接收数与 DocRecord 数也核对，但不宣称后续恢复/域/质量通过。所有 1D/拓扑/副本指针输入仍冻结。`divide-conquer-final-oracle` 另有 48 原生数值/随机点集，包含共圆方形、近共线和重复 localId/不同坐标；排序和连接严格相同。上述最终报告与当前DLL/EXE SHA已再次复核匹配。

零扰动初始运行首次 actual native 访问异常保留 `periodic-initial-cad-red-oracle` 和日志；新增只读 emitted/predicted 计数发现 `ConvertDListToTriangles` 实际 137、声明 152（graded embedded-point，另有常尺寸差异）。观察副本仅在显式初始 oracle 激活时阻止读取未初始化尾部，正常生成分支保持；自研返回 InvalidTopology。`periodic-initial-native-count-oracle` 保存 4 个同类零扰动诊断与 4 个默认扰动通过，下一零扰动嵌入线原生返回 -5，尚未定位。最终固定初始对照矩阵明确只验收默认扰动的 32 项，zero_perturbation_initial_stage_validated=false；不会将这些失败改为成功布局或删除证据。Qtest 行列式乘积上下溢的原生数值语义尚需复现验证。

普通初始阶段新增 `GmshWalkTriangulationReplay/Comparison.py`。只读 oracle 直接调用原版 Walk/split 及 GFaceInitialMesh(face,0,nullptr)，将实际 PolyMesh 指针池转为编号；原版算法不取自研连接作为输入。`ordinary-initial-pools-oracle/walk-triangulation-report.json` 保存 52 组纯点集和 24 次实际普通面调用，UV/物理身份/点入射边/面首边/半边 prev-next-opposite-face 及标签逐值零容差一致。点集包含精确共圆、共线、初始对角线、格点、近共线、多次序及 1e-8/1/1e8 尺度；实际面覆盖两尺寸上限和两尺寸场。普通 CAD UV、1D 和 getNodeCopies 枚举仍冻结，自己的扩盒/剖分独立计算；自己的普通 CAD 反求与约束/域/recover=1/BDS 转换未验收。

普通初始入口/纯半边模块分别先有真实运行 RED，再实现 GREEN。观察补丁新增两个显式池 oracle，55854 字节、26 导出、9 文件 SHA 真实 Git apply 重放一致；8 原版/off/on 生成指纹仍相同。核心重放不导入 Gmsh/OCC，默认产品入口保持。上述阶段证据不能代替完整算法 6 输出质量、CAD、GUI 或性能验收。

普通约束恢复新增 GmshConstraintRecoveryTest/Replay/Comparison：780实际intersect标量对照保留原生数值乘积的下溢负零/上溢无穷决胜，33点集实际recover_edge/getEdge（219次需要翻边）记录穿越列表/返回码/双侧标签/全池，包含-1失败后继续。观察副本显式前缀oracle调用实际GFaceInitialMesh(recover=1)，仅在全部恢复后、Color之前截取；正常生成不早退。28实际CAD同样逐值一致；原24CAD只有已有边，新增窄孔/嵌入点四尺寸场有6次确需翻边恢复、21条穿越，补上覆盖缺口。BREP保存在oracle目录，独立点的embedding关系显式恢复，窄孔上侧callback更密的规则可重放。

最终 `ordinary-constraint-final-oracle/constraint-recovery-report.json` 输入只冻结1D/UV/枚举和物理约束链；自研prepareInitialConstraints转换localId/物理ID/池索引，不输入参考已转换的池端点。输出按零容差比较所有内部状态。HalfEdgeTopology抽出共享连接写入后，`walk-constraint-final-oracle` 52点集+24actual初始仍一致。约束实现RED9后GREEN，身份映射另RED2后GREEN；取消/预算/非法拓扑原子性用保留标签哨兵验证。原版PolyMesh标签不是翻边锁；不将此行为推广到后续BDS阶段。

35目标Release/30CTest（20复现）/8Python语义测试通过；当前补丁62109字节/29导出/9文件实际重放SHA一致，8原版/off/on生成指纹仍同。纯恢复重放仅CRT/KERNEL32，无Gmsh/OCC/Qt。-2/-3/全部循环退化、多UV、域着色/后置UV翻边/BDS转换及后续恢复仍未完整验收，整模型/CAD/GUI/性能保持未完成。

2026-10-08新增 GmshInitialDomainComparison.py 和重放 domain/uv 模式，调用 actual GFaceInitialMesh(recover=1) 在 Color 后和 UV 轮后截取；不冻结原生连接/域/种子/访问/翻边。`ordinary-domain-final-oracle/initial-domain-report.json` 保存36域+36UV共72项，全池/标签/other_side两返回/重复LIFO访问及逐扫描翻边零容差一致。含双孔、窄孔及嵌入线两侧近点压力，两尺寸上限/两场；全部72有重复Color访问，UV4次真正翻边。原始输入仍冻结1D/UV/枚举/物理边链，池端点映射与后续计算自研。BDS转换/度量、ordinary additional/多UV/全部失败退化均不在此通过范围。

2026-10-08后续新增BdsTopology/BdsMeshConversion及GmshBdsMeshConversionTest/Replay/Comparison，原版观察副本显式调用actual meshGenerator/meshGeneratorPeriodic在BDS恢复前截取。新增JSON只按原生节点set、边/面池和原邻接序导出；正常生成不进入截取。BdsTopology用稳定局部ID和追加边/面索引；BdsMeshConversion普通独立BDS reparam坐标，转换全部PolyMesh面、不传域/边标签；周期保留raw点/双尺寸/counterpart及未被节点引用的分类登记。单元先RED7后GREEN，再由actual全端点复用seam压力暴露未引用负边分类遗漏，补独立登记传递后新增RED1→GREEN，不删分类表比较。

最终`ordinary-bds-final-oracle/bds-conversion-report.json`有24原生BDS append点集（全六角点排列/逆点插入/ID空洞/负外围/重复片）+36actual普通转换。输入冻结1D、PolyMesh UV/副本序、独立BDS图表ID/物理映射/UV/XYZ/分类及四CAD查询值，初始/恢复/Color/UV/转换全前缀自研；输出连接/方向/分类/有序邻接未冻结。`periodic-bds-final-oracle/periodic-embedded-report.json`有32actual周期BDS转换，来自真实OccCadGeometry/完整周期图表/分治初始串联；仍冻结1D/拓扑/副本指针序，CAD值和边界BDS不冻结。另64叶/64完整图表/32初始在同矩阵复核。全部零容差逐值一致，不覆盖零扰动/全部失败、additional/多UV、BDS恢复/度量或最终质量。

本阶段38相关目标Release构建通过，补非有限/未知CAD异常原子状态后受影响Main/测试/重放重新构建；32CTest（22复现+既有二维/几何/CAD）、8Python语义测试通过。最终92项BDS对照报告SHA与当前DLL/两重放EXE一致；74456字节/33导出/9文件观察补丁实际Git apply重放SHA一致，8原版/off/on指纹同。167既有源hash仍仅六必要改动；纯BDS重放仅CRT/KERNEL32，无Gmsh/OCC/Qt链接，默认legacy/正式EXE/GUI保持。

失败记录保留：普通脚本初次用了两个错误旧导出名，逐项核对已有InitialDomainComparison/manifest后纠正；两个空失败输出目录未删除。周期首个不一致在cylinder-seam-pressure的geometry表[-2,1]缺失，由原版无条件add_geom与自研只记节点分类差异定位。两次文档/文件头补丁未匹配，完整核查后重试，不存在部分应用。下一步BDS恢复/域和度量合法化，完整复现目标仍active。

嵌入线压力4次都把已恢复的线段翻成两侧嵌入点的对角线，原curveTag=5仍留在同一半边对象；普通后续BDS恢复不能因步骤看似重复而删除。单元另验证孔残余-1、真实UV翻边及标签迁移、无内域种子的显式NonConverged与取消/预算原子性。首次RED测试在空stub输出上写哨兵造成访问错误，先修正测试输出初值后看到6个真实RED，再实现GREEN，不把该访问错误当作需求证据。

最终36目标Release/31CTest（21复现）/8Python测试通过，780相交/33点集/28CAD恢复和52点集/24CAD初始已用当前DLL/EXE复核。当前补丁66643字节/30导出/9文件实际重放SHA一致；8原版/off/on指纹同，纯域/UV重放仍仅CRT/KERNEL32。全部新最终报告SHA与当前产物匹配；167既有源码仍仅六个必要文件变动。原版100扫描上限路径有明确诊断实现，但本72矩阵均未触及，不能称全退化验收；整模型/CAD/GUI/性能未完成。

## BDS 单次约束恢复（2026-10-08）

新增 `BdsConstraintRecovery` 和 Test/Replay/Comparison。实际原版 `Intersect_Edges_2d/sys2x2` 作为数值参考，实际 `BDS_Mesh::recover_edge` 作为恢复参考；观察仅在显式叶oracle中记录整边池扫描索引和真实swap尝试，不替换原生计算。BdsTopology只负责邻域和连接更新；恢复模块负责奇异阈值/闭区间求交、曲线/config_modified保护、原池首个可翻边和失败记录。保留旧deleted池、新边/面追加序及分类继承。Complete只表示叶调用结束，另查边索引/fatal/未恢复约束。

`bds-recovery-leaf-final-oracle/bds-recovery-report.json`：720数值（九尺度至1e±160，反向/互换、端点/共线/近奇异和上下溢）+136恢复调用零容差逐值一致。恢复含50合成与86冻结CAD初始BDS叶输入；全六首三角排列、曲线保护/modified/counterpart、已有边、登记CAD自交双方、18多交换fan和失败后发布拓扑。专门的failure-after-swaps原生成功2次后fatal，全池/3扫描/尝试与自研一致。比较包含全部节点/边/面/分类/有序邻接和每轮扫描，不是只比较最终三角数量。

恢复矩阵冻结BDS点、连接、分类、修改标记、登记约束及目标；CAD来源输入从前阶段32项周期BDS快照取约束/非已有边压力对，部分无可选请求。这不是实际CAD恢复驱动，不验证普通两pass/周期循环、分类标记、域删除、ix>300的全退化、重试或度量合法化。源码复核普通pass1登记boundary+embedded，pass2仅遍历boundary；旧嵌入线UV压力不能推断该线由普通BDS pass2重新恢复，完整后续链仍需核对。

单元真实RED11→GREEN；追加原生失败后发布拓扑/节点预算原子性通过。40相关Release目标构建，新增测试修改后单独重建，33CTest（23复现）和8Python测试通过。ordinary-bds-recovery-regression-oracle24原位+36普通转换，periodic-bds-recovery-runtime-regression-oracle64叶/64完整图表/32初始/32转换用当前DLL/EXE复核通过。首次周期回归返回0xC0000135（子进程缺OCC/Qt DLL路径），补PATH后在新目录重跑；失败目录与日志保留，没有算法容差修改。

当前观察补丁80324字节/35导出/10文件，9pristine文件与固定tgz逐字一致；observer-bds-recovery-patch-replay-20261008真实Git apply及10 SHA一致。observer-bds-recovery-neutrality8原版/off/on最终指纹同。纯BDS恢复重放仅CRT/KERNEL32，无Gmsh/OCC/Qt链接；完整生成/质量/CAD/GUI/性能未验收，总目标active。

## 普通域前BDS曲线登记/边界恢复（2026-10-08）

新BdsCurveRecovery负责buildOrdinaryBdsRecoveryRegistry与recoverOrdinaryBdsBoundary。按物理身份构建ordinary inverse，boundary再embedded登记首归属约束；域前只恢复boundary，curve分类先add_geom，正常处理全曲线才补空端点(localID,0)，原分类不改。缺映射跳段、非fatal提前返回该曲线后继续、fatal停止boundary；refineAll/未恢复交重试模块。单元真实RED11→GREEN，不把Complete解释为全部恢复成功。

外部只读oracle调用实际meshGenerator，在域前boundary结束/致命失败时截取；实际recoverEdge逐段记录原生BDS叶扫描/attempt。GmshBdsMeshConversionReplay新增ordinary-boundary，先独立计算Walk/PolyMesh恢复/Color/UV/BDS转换，再构建inverse/登记和恢复pass；不输入原生结果连接。`ordinary-bds-boundary-release-oracle/bds-conversion-report.json`保存24原位+36actual boundary阶段、2445段调用，全池/分类/登记/未恢复/顺序和尝试逐值一致。CAD1D/UV/副本序/独立BDS图表/四CAD值，以及curve拓扑/物理链/端点/退化/refineAll仍冻结。

本36矩阵boundary段都已有边，0交换。单元有自交/保护fatal/缺映射/空曲线等，但不能据此称driver所有需要交换/失败已native验收。域标记后embedded pass2、周期驱动、重试、尺寸/清理及完整生成不在此验收。源码完整顺序纠正上一节只读到域前pass2的表述：普通域前恢复boundary，域后另恢复embedded，meshGFace.cpp:1867–1874清楚保留调用。不得省略/提前合并。

41Release目标构建，转换重放注释修改后单独重建；34CTest（24复现）/8Python通过。ordinary-bds-curve-release-conversion-oracle原模式24/36、bds-recovery-curve-regression-oracle720/136、periodic-bds-curve-regression-oracle64/64/32/32用当前DLL/EXE复核；最终四报告SHA当前。补丁87350字节/36导出/10文件，9pristine与固定tgz一致，observer-bds-curve-patch-replay-20261008实际Git apply及10SHA同；observer-bds-curve-neutrality8原版/off/on指纹同。167既有源仍仅六必要变动，纯边界/转换重放仅CRT/KERNEL32，完整CAD/质量/GUI/性能目标未完成。
