# phone / 凹球问题的开源实现复查

状态：源码研究完成。用户要求仔细核对其他开源软件处理本次具体问题的方式；本轮未修改算法，未新增模型运行。

## 比较问题

- phone：132 个实体，用户确认各自保留完整表面、接触和干涉；仍有 15290/112066 片超质量目标，13858 片触及固定裁剪节点，最差半径边比约 477。不能把所有坏片证明为小特征必然限制。
- CAD 输入：共享端点附近微小裁剪交叉，当前只协调深副本的两个顶点容差，并完整验证。不等同通用修复。
- 凹球：物理质量后处理后 3040→23 片超标；中心固定度量仍参与初始 CDT，极点/缝和边界布点仍需专门核对。

## 研究范围

固定 Gmsh 4.15.2、Netgen v6.2.2603、CGAL v6.1.1，并核对 OCCT 的具体修复接口。固定版本便于复核，不宣称它们是当前最新发布。优先查看局部函数及调用链，不将函数存在当成同模型已验证。

已有记录：`2026-10-01-cad-surface-meshing-revision.md`、`2026-10-02-surface-performance-open-source.md`、`2026-10-02-phone-concavaball-debug.md`。

在线检索工具两次因连接失败未返回资料；使用此前官方源包缓存与官方仓库直接读取作为后备，保留来源及版本。

## 已核实要点

- Netgen `occgeom.cpp:256–258` 的 `MeshFace` 确实先调 `OCCMeshFace(...PARAMETERSPACE,true)`，失败后调用 `(...PLANESPACE,false)`，不是仅有未调用的接口。`occmeshsurf.cpp:115` 在局部基边处构造切平面/参数变换，约 170 行读取 D1 并检测 `D1ᵀD1` 奇异；不是整张面的固定中心度量。
- Gmsh `meshGFaceBDS.cpp:27` 将多个 UV 边界点恢复到同一三维顶点的边识别为退化；约 377 行有极点邻边专门分裂，约 453 行分裂时先对退化方向对齐参数，再取中点。`_periodicCounterpart` 参与局部处理；不能用普通二维点移动代替这些拓扑语义。
- Gmsh `DefaultOptions.h:1000–1027` 的自动 OCC 退化/小边/小面修复及缝合默认关闭；`GModelIO_OCC.cpp:5886` 的 healing 入口按开关调用。不能解释为 Gmsh 导入任意无效 B-Rep 都会自动修好。
- Netgen 优化文件名实际为 `smoothing2.cpp` / `smoothing2.5.cpp`；初次猜测 `optimize2d.cpp` 得到 404，已根据官方固定标签树纠正。已下载相应版本源码，继续核对实际调用和质量检查。
- Gmsh `meshGFaceDelaunayInsertion.cpp:333` 从曲面 D1 构造局部度量；约 365 行在待判定三角形重心取度量，约 1163 行的 Frontal 候选按活动边、局部尺寸及度量构造。当前代码 `CadSurfaceMesher.cpp:1171–1173` 对所有触及边界节点或最短边不超过 `2*h_min` 的三角形跳过质量驱动细化，范围比明确识别尖角的保护机制更宽。
- CGAL Mesh_2 的 `Refine_edges_with_clusters.h` 按约束边端点的尖角簇、插入半径和是否强制细化决定分裂或放弃当前质量候选；这不等于所有边界附近三角形都忽略质量。它是平面算法，不能直接移用其收敛结论到 CAD 度量场。
- Gmsh BDS 路径交替执行分裂、平滑、换边和收缩；`BDS.cpp:1082` 的收缩保护 CAD 顶点、CAD 边节点及周期关系。我们当前物理优化只换边和移动内部节点，尚无受保护的内部边收缩。
- OCCT `ShapeFix_Wire` 区分单边、相邻边和非相邻边自交；能提高容差，也可能改 pcurve、裁短或移除边。必须检查实际分支、修改开关及最大容差，不能把 `FixSelfIntersection()` 当作无损修复。

## 结论

继续采用自研 UV CDT 与三维判据细化。当前补丁已使 phone 默认参数成功、改善凹球质量，但质量改善主要发生在生成之后；下一轮应补生成前的边界尺寸传播、生成中的局部度量候选和极点图表，再扩展受保护的局部重网格操作。

phone 的 15290 个超标片中，13858 个触及固定裁剪节点、14531 个最短边不超过 `2*h_min`，两组有重叠。这些统计只说明关联，不能证明质量不可改善，也不能证明全部来自极点。需按 CAD 面/棱及具体几何位置分类。当前代码对这些片直接跳过质量驱动细化，是已确认的实现限制。

独立部件策略符合用户选择：每个部件保留自己的表面和标签，同部件自交仍拒绝。不同部件的接触和体积重叠不能靠表面细化解决。未来若需要多区域四面体的公共接口，需要另外构造共享拓扑和接口剖分；本轮不执行此操作。

## 1. 装配接触和干涉：拓扑操作与表面生成分开

### 源码事实

- Gmsh `Generator.cpp:575–635` 的二维流程对 CAD 面调用 `mesh(true)`，有失败重试及异常处理；这段流程没有把整个装配体先统一求布尔差。不能仅凭逐面调用证明任意相交装配默认均可通过所有后续检查。
- Gmsh 的 `gmsh/model/occ/fragment` 显式调用 General Fuse，返回新实体及输入到输出的映射，使交界共形。这是 CAD 拓扑加工步骤，不是简单保留原来每个相交实体的完整边界。[官方 API 说明](https://gmsh.info/doc/texinfo/gmsh.html#index-gmsh_002fmodel_002focc_002ffragment)
- Netgen `occgeom.cpp:343` 的 `GlueGeometry()` 调用 `BOPAlgo_Builder` 后重建面映射；`BuildFMap()` 约 1235–1280 行根据既有拓扑给面设置两侧 `domin/domout`。不能据此说默认导入会自动 glue，更不能说几何位置重合的两张独立面自动共享网格。[Netgen 源码](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/occ/occgeom.cpp#L343)

### 对当前代码的判断

`SurfaceMeshEngine.cpp:33` 已显式使用 `IndependentBodies`；`CadSurfaceMesher.cpp:714–715` 记录跨部件关系并允许独立边界。`OccCadGeometry.cpp:240` 缓存实体对的 OCC Common、面对距离查询。

这条方向保留。建议把“各实体内部验收”和“装配关系诊断”分成计时明确的阶段；独立生成不必等待布尔结果才能知道跨部件相交是否允许。关系记录仍要保存，查询失败要明确标记。当前记录由网格相交候选触发，不是所有实体对的完整干涉审计。OCC Common 是否占据主要耗时尚未逐阶段测量，不能承诺这一拆分提速多少。

## 2. 微小裁剪交叉：先分析，再选择有界修复

### 源码事实

- Gmsh 4.15.2 `DefaultOptions.h:1000–1027` 中 `OCCFixDegenerated/OCCFixSmallEdges/OCCFixSmallFaces/OCCMakeSolids/OCCSewFaces` 默认值均为 0；并非默认大修所有导入 CAD。启用小边修复后，`GModelIO_OCC.cpp:5983–6030` 会调用 `ShapeFix_Wire`，打开 `ModifyTopologyMode` 并调用多个修复函数，可能修改边和拓扑。[固定版本官方源包](https://gmsh.info/src/gmsh-4.15.2-source.tgz)
- OCCT `ShapeFix_Wire::FixIntersectingEdges(num)` 的相邻边分支（2574 起）先分析交点及误差；约 2662 行比较局部所需容差和 `MaxTolerance()`，可更新边/顶点容差；约 2723 行在允许拓扑修改的分支还可能裁短边。单边、相邻边、非相邻边是不同分支。[OCCT V7_9_0 源码](https://github.com/Open-Cascade-SAS/OCCT/blob/V7_9_0/src/ShapeFix/ShapeFix_Wire.cxx#L2574)
- Netgen `HealGeometry()` 从约 431 行开始，包含去除退化边及后续 ShapeFix 操作；不能直接视为保留所有小特征的容差协调。[Netgen healing 源码](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/occ/occgeom.cpp#L431)

### 对当前代码的判断

phone 的共享端点附近交叉使用深副本上的有限容差协调，且核对曲线、曲面、位置和拓扑。这个窄范围处理可以保留，不能升级成默认调用整套 ShapeFix。

建议 CAD 适配层先给出缺陷类型、涉及实体、交点的三维距离、输入容差和所需修复量；只允许已规定上限以内、明确不改几何/拓扑的自动协调。超出上限或需要裁边/删特征时返回具体诊断。`MaxTolerance` 也不能单独保证所有修复分支无损。OCCT 源码对照版本为 7.9，项目当前使用 7.6；若选用具体接口，必须先核对本机版本和回归结果。未执行 Gmsh/Netgen 对 phone 的 healing 对照，不能声称它们会无损修好同一模型。

## 3. 参数畸变和凹球极点：生成时使用局部映射

### 源码事实

- Gmsh `meshGFaceDelaunayInsertion.cpp:333` 用曲面 `firstDer` 得到 `JᵀJ`；约 365 行的各向异性 incircle 在当前三角形 UV 重心求局部度量。`optimalPointFrontal()`（1163 起）依据活动边、尺寸场和度量外心构造候选，约束候选到边的步长。[源码包](https://gmsh.info/src/gmsh-4.15.2-source.tgz)
- Gmsh BDS/MeshAdapt 路径中，`meshGFaceBDS.cpp:27` 从多个参数节点恢复到同一三维顶点识别退化边；约 377 行专门处理极点邻边；约 453 行在取 UV 中点前对齐退化方向的参数；周期对应点也参与操作。这些步骤不是每个 Gmsh 算法都无条件执行。
- Netgen `OCCGeometry::MeshFace()`（242 起）实际先尝试 `PARAMETERSPACE`，失败后尝试 `PLANESPACE`。`OCCSurface::DefineTangentialPlane()` 从局部基边/曲面导数构造变换，检测奇异矩阵；切平面路径的候选通过几何投影返回曲面。[调用链](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/occ/occgeom.cpp#L242)、[局部映射](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/occ/occmeshsurf.cpp#L115)

### 对当前代码的判断

`CadSurfaceMesher.cpp:1085–1089` 仍按整张 UV 包围盒中心构造一个度量用于初始 CDT 和主要换边；约 1265 行的普通细化候选仍取 UV 重心。`SurfaceChart.cpp` 已有局部 D1/曲率查询，物理后处理也使用局部切向移动，因此不是缺少所有局部几何能力，而是尚未把它们完整用于拓扑和布点阶段。

建议把候选生成独立为纯 C++ 模块：对待处理三角形/局部邻域求正定度量，优先尝试度量外心或沿有效前沿的偏心候选，经过裁剪域、约束侵入及三维硬判据筛选后插入。一次空腔/换边事务固定一致的度量及决胜规则，避免每个相邻判定用不同度量造成反复翻边；不能简单替换 incircle 的一个参数就宣称实现了稳定的变度量 CDT。

极点需明确“一个三维节点、多个合法 UV 分支、一个退化参数方向”，不能任意平均无定义的参数。局部切平面作为备选图表须保留裁剪环、周期缝和 CAD 节点对应，投影到裁剪面有效域后重新验收；不以失败后随意换坐标绕过检查。法向和映射一致性仍必须验证。

## 4. 短边附近瘦片：边界尺寸传播与尖角保护

### 源码事实

- Gmsh 官方说明 `MeshSizeExtendFromBoundary` 默认开启，会把边界尺寸延伸至面内部，也提醒过度细化时应调整它。BDS 路径 `computeNodalSizes()` 当前有效分支根据相邻 CAD 边网格段长设置节点尺寸；函数后的另一段传播代码因提前 `return` 不执行，不能作为有效实现引用。[尺寸场说明](https://gmsh.info/doc/texinfo/gmsh.html#Specifying-mesh-element-sizes)
- Netgen `occgenmesh.cpp:567` 用 CAD 边长度/分段控制限制局部尺寸，后续还按边/面曲率限制；区域和面属性可另设局部上限。[源码](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/occ/occgenmesh.cpp#L567)
- CGAL Mesh_2 `Refine_faces.h:179` 使用外心候选；`Refine_edges_with_clusters.h:186–246` 判断候选侵入约束边时，根据尖角簇、是否强制细化以及插入半径决定分裂边或放弃当前候选。放弃候选不等于删除三角形。[外心](https://github.com/CGAL/cgal/blob/v6.1.1/Mesh_2/include/CGAL/Mesh_2/Refine_faces.h#L179)、[尖角簇保护](https://github.com/CGAL/cgal/blob/v6.1.1/Mesh_2/include/CGAL/Mesh_2/Refine_edges_with_clusters.h#L186)

### 对当前代码的判断

`CadSurfaceMesher.cpp:1171–1173` 的质量筛选排除任意贴边三角形，以及最短边不超过 `2*h_min` 的三角形。`growthRatio` 已参与端点/中心目标比较，但没有将实际保留的短 CAD 段长传播成面内部的尺寸过渡带。短边旁边直接接大内部单元时，后处理只能有限改善。

建议先标记真正的尖角、窄区和受保护特征；从这些边界的实际离散尺寸构造有限范围的三维标量尺寸约束，并通过邻域限制尺寸渐变。不能把尺寸无界传播到整个装配体，也不能为了删除瘦片合并真实 CAD 短边。触及普通边界仍应接受质量驱动候选，只有明确的尖角保护、最小尺寸冲突、映射限制或预算耗尽才允许带原因保留残差。若保留真实小特征要求突破 `h_min`，须报告位置和原因；不能静默把面板最小尺寸改小。

这里借鉴 CGAL 的局部保护思想，不能把平面常度量算法的终止/质量结论直接推广到裁剪 CAD 曲面。

## 5. 后处理要能局部改拓扑，但保护 CAD 边界

- Gmsh `refineMeshBDS()`（726 起）交替做分裂、平滑、换边、收缩；`BDS.cpp:1082` 的普通 `collapse_edge_parametric` 保护 CAD 顶点、边界点、分类及周期对应。退化边处理存在专门的强制路径，不是普遍允许删除 CAD 特征。
- Netgen `basegeom.cpp:995` 在优化阶段调用 `OptimizeSurface()`；约 1221–1265 行按配置执行换边、`ImproveMesh`、`CombineImprove`。`smoothing2.cpp` 的质量函数兼顾形状与尺寸，候选移动使用 CAD 投影，投影失败会恢复节点。已核对真实调用，不只依据接口名判断。[优化调用](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/meshing/basegeom.cpp#L1221)、[移动实现](https://github.com/NGSolve/netgen/blob/v6.2.2603/libsrc/meshing/smoothing2.cpp#L976)
- 本次未把 Netgen 移动循环中注释所称的缩步当作有效回溯机制：局部变量 `fact` 虽缩小，所读版本的实际位移表达式没有使用它。参考机制仍须复核有效代码和返回状态。

我们当前优化只做换边/内部点移动，无法移除不合适的内部短边和点簇。建议最后补受保护的内部边收缩或局部重网格：保持约束、共享节点、逐面 UV 和标签；检查顶点邻域流形、裁剪域、翻折、偏差/法向/尺寸和相交；失败完整回滚。必须先完成局部候选和边界尺寸机制，再判断哪些残差值得更改拓扑，避免一次扩大太多。

## 推荐实施顺序与验收

以下是下一阶段建议，尚未实施。

| 顺序 | 工作包 | 完成标准 |
| --- | --- | --- |
| 1 | 将残差按面/棱/尖角/短边/极点定位；独立实体验收和关系诊断分开计时 | phone 每个差片可追溯；统计不把关联当原因；部件/关系标签不丢；同体自交仍拒绝；明确 CAD 查询耗时 |
| 2 | 短边附近局部尺寸带与显式尖角保护，收窄 blanket 质量跳过 | 普通边界可参与质量细化；尖角和 `h_min` 冲突有原因；预算受控；不删真实 CAD 特征 |
| 3 | 局部度量外心/前沿偏心候选、稳定的局部事务 | 同尺寸/偏差/法向条件下比较质量、片数、工作量；UV 拉伸/剪切等价测试；换边不循环；共享边一致 |
| 4 | 极点退化方向、周期对应和可切换局部图表 | 球面极点、圆柱缝、裁剪孔和窄区不折叠；一个三维节点的 UV 分支正确；无法处理时明确失败 |
| 5 | 受保护内部边收缩/局部重网格 | 不删除 CAD 特征，失败可回滚，闭合/方向/自交及所有硬判据复核 |

CAD 容差分析在适配层独立维护；不以默认 full healing 作为以上质量工作的前提。保持 `GeometrySource`、三维标量 `MeshSizeField`、进度/取消、诊断及结果标签契约；表面候选、图表、尖角保护属于表面实现，不把它们塞入未来体网格专用判据。

## 证据与限制

- 官方 Gmsh 4.15.2 源包 SHA256：`BE3F66F225D27BA9FA014F07E83169285DA8A051B0E8AB7103D88066B39BDD3E`；本轮再次核对哈希。官方手册本轮直接 HTTP 读取，未依赖搜索摘要。
- Netgen/CGAL/OCCT 使用上述固定官方标签。官方 GitHub raw/tree 可读取；猜测的 `optimize2d.cpp` / `optimize2d.hpp` 路径返回 404，未将其用作证据，优化证据来自实际 `basegeom.cpp` 与 `smoothing2.cpp`。
- 缓存目录：`C:/Users/30861/AppData/Local/Temp/MeshStudio-cad-research-20261001`。Gmsh 函数位置针对实际解压源包，不使用网页挑战页缓存。
- phone/凹球数值复用前一轮已验证记录：`2026-10-02-phone-concavaball-debug.md`。本轮没有新跑模型、构建、GUI 或同模型 Gmsh/Netgen 质量验收；不能证明参考方案已在这两个模型上有效，也不能报预期速度倍数。
- 本轮只更新研究与进度文档，保留当前算法、UI 及另一个任务的四面体改动。后续算法方案需要逐工作包实测，不能将上述源码机制等同为项目已完成的功能。
