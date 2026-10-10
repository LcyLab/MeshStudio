# 面向近等边三角形的 CAD 表面网格路线调研

2026-10-06。性质：研究与方向建议，尚未批准生产算法切换。只更新研究记录，没有修改算法、构建、生成新网格或进行性能测试。

## 结论与范围

**建议主线：保留原始 CAD 几何，以真实特征和界面定义网格分区；允许跨同实体内的光滑分割边；采用局部曲面度量下的前沿式布点与 Delaunay/局部重连接，再做有界重网格优化。**

这个建议不是“再给当前坏片修复流程加一个前沿模块”。需要首先解除“一个 CAD 面等于一个网格分区”的假设，然后改生成阶段的点位和处理顺序。CDT、几何查询、共享约束和事务回滚仍有复用价值。

前沿不是取得高质量三角形的唯一途径。CVT/RVD 和完整的局部各向同性重网格也能改善整体布局，应保留作对照。主线选择是结合精确 B-Rep 输入、自研 C++、现有模块复用和可解释生成过程作出的工程判断，不是已实测的性能排名。

用户已明确：主要追求三角形形状质量，并行暂不优先；允许跨光滑 CAD 分割边，但保留真实特征、材料界面和部件边界。该授权不包含删除物理小特征、修复任意 CAD 裂缝/重叠，或实施大范围重构。

本轮覆盖代表性经典方法、2022—2026 年相关进展及十个代表项目，不声称穷尽所有论文。公开全文、出版预览、官方文档和代表源码的读取程度在下文分开说明；作者报告的质量/速度不等于本项目验证结果。

## 1. 为什么只强调 Delaunay 或修坏片还不够

Delaunay 决定给定点集的连接关系，不能单独决定点集具有三角晶格式排列。最小角底线、几何误差满足，也不等于多数三角形接近等边。

当前静态证据见 [Gmsh 布点专项复核](2026-10-06-gmsh-point-layout-route-review.md)：

- `src/core/mesher/CadSurfaceMesher.cpp:795` 的矩形种子在理想均匀平面完整格点条件下产生直角等腰片。翻转方格对角线仍是直角片；内点已经在邻点平均位置时，平均平滑不必改变这种排列。这是几何推导，不是本轮运行结果。
- `SurfaceRefinementCandidates.cpp:15–29` 已有最短边偏心点思想；主要缺口是活动边身份、尺寸场与整片推进顺序，不能再说“完全没有等边高度公式”。
- `SurfaceRefinementQueue.cpp:13–19` 以违规修复为中心。当前也已有合格片的形状优化，不能沿用“合格片全部跳过”的旧描述。
- `CadSurfaceMesher.cpp:1287–1347` 的参数中点二分与 Gmsh 弧长/尺寸积分采样不同。差异影响需固定边界做消融，不能只凭源码量化收益。

因而调整方向应同时关注：约束是否合理、尺寸是否兼容、点位是否具有近等边倾向、是否允许改变节点数量和连接。仅提高质量阈值或平滑轮数，可能增加加密和耗时，却没有解决点阵来源。

## 2. 代表性论文：各自解决什么

| 路线与资料 | 主要输入/机制 | 对本项目的作用与边界 | 本轮读取程度 |
|---|---|---|---|
| [Rebay 1993，Delaunay 与 advancing-front 点位](https://www.sciencedirect.com/science/article/pii/S0021999183710971) | Delaunay 连接与前沿式选点结合 | 说明连接和布点可以结合，不要求彻底丢弃 Delaunay | 出版摘要，未取得全文 |
| [Ruppert 算法作者说明](https://www.cs.cmu.edu/~quake/tripaper/triangle3.html)、[Üngör off-centres](https://www.cise.ufl.edu/~ungor/papers/offCenter.pdf) | 平面约束域，按质量/尺寸细化；偏心点兼顾质量和点数 | 适合质量底线与候选回退参考；平面终止条件不能直接搬到一般 CAD 曲面 | 作者算法说明；偏心点论文选点和条件部分 |
| [Gmsh 作者预印本](https://gmsh.info/doc/preprints/gmsh_paper_preprint.pdf) | CAD 参数空间中使用物理曲面度量，结合多种生成算法 | 关键是物理形状质量，不是 UV 中画出规则等边片 | 曲面度量和表面生成章节，并核对 4.15.2 源码 |
| [Engwirda/Ivers，曲面 Frontal-Delaunay](https://arxiv.org/html/1501.04002)、[Engwirda 2016，分片光滑复形](https://arxiv.org/html/1606.01289) | 三维 restricted Delaunay，几何交点、前沿偏心点、特征保护与细化顺序 | 强参考：把“在哪里布点”和“如何保护曲线”作为生成核心；不是 Gmsh 算法 6，也不是逐面 UV CDT 的同一个理论框架 | 公开正文中选点、排序、特征保护和适用条件部分；核对 JIGSAW 代表源码 |
| [Alliez/Surazhsky 等 2003 各向同性重网格](https://www-sop.inria.fr/members/Pierre.Alliez/publications.html) | 已有表面网格，参数化/局部参数化与加权 Lloyd 布点 | 说明整体点位优化也是路线；需要另外处理原始 B-Rep 支撑 | 作者书目与摘要；未把相关书目全称为已精读 |
| [Botsch/Kobbelt 2004](https://www.graphics.rwth-aachen.de/publication/03101/) | 面向已有网格的局部重网格和点位调整 | 完整 split/collapse/flip/move 比单纯平滑更有重排能力；PMP 可核对实际操作 | 作者摘要与 PMP 实现；论文 PDF 入口读取失败 |
| [Persson/Strang，DistMesh](https://persson.berkeley.edu/distmesh/) | 距离函数几何、点集力学松弛与 Delaunay | 可作布点思想和简单域基线；不能直接代替 CAD 修剪、特征和接口体系 | 作者页面与此前研究记录，未审查新版本全部源码 |
| [Yu 等 2022，AFT 效率](https://www.sciencedirect.com/science/article/pii/S0010448522001373) | 连续曲面前沿生成，理想点和几何操作效率 | 提醒应降低生成环节的查询和定位成本，不应只靠并行掩盖过多修复 | 出版预览/摘要与贡献部分，非全文；未核实公开实现 |
| [Liu 等 2024，组合曲面 AFT](https://www.sciencedirect.com/science/article/pii/S0010448524000101) | 虚拟拓扑组合 CAD 曲面；连续几何布点，离散辅助参数化检验有效性 | 与跨光滑分割需求最贴近。借鉴网格分区独立于 CAD 面、连续几何与辅助表示分工；不照搬自动脏 CAD 修复 | 出版预览中的摘要、引言、流程与章节片段，全文入口 403；原文说明为内部实现 |
| [Zou 等 2024，CAD 独立网格与 NEFEM](https://cronfa.swan.ac.uk/Record/cronfa65064/Download/65064__29261__fdf69dec054047a9ae3395bad86cd58d.pdf) | 元素可跨 CAD 分割，精确几何依赖 NEFEM 的特殊曲面元素表示和几何支撑点 | 支持几何拓扑与网格拓扑解耦；普通平面 T3 跨面仍有逼近误差。原文明确没有以定义/改进 NEFEM 质量指标为目标，不能用来证明近等边质量 | 机构公开完整 PDF，读几何表示、重网格和结论章节；未核实公开实现 |
| [2024 metric-adapted embeddings](https://www.sciencedirect.com/science/article/pii/S0167839624000487) | 度量驱动的各向异性三角网格 | 度量计算有参考价值；各向异性目标不等于物理空间近等边 | 出版摘要/预览 |
| [2025 adaptive multiple facet-clipping CVT](https://arxiv.org/html/2505.14306) | 离散曲面、曲率自适应裁剪近似 CVT，并行计算 | 说明 CVT 的效率仍在发展，值得备用；精确 CAD 投影和特征约束不能自动获得 | 公开正文引言和方法概览，未完整复现、未核实公开代码 |
| [2025 RL Delaunay mesh](https://arxiv.org/html/2504.03610) | 二维多边形域，学习增删移动节点后重三角化 | 不能用二维实验直接推导三维 CAD 工业鲁棒性 | 公开正文任务定义及结果/结论部分 |
| [2026 NeuralCVT](https://arxiv.org/html/2609.08497) | 三角表面采样点云，学习 CVT 点位，RVD 输出连接 | 不是不存在的路线：已核实代码与模型入口。可研究加速/初始化；目前不是纯 C++ 精确 B-Rep 主链的直接替换 | 公开正文输入、特征损失和实验部分；官方仓库 README/依赖，未审查训练与推理全源码 |
| [2026 high-order CAD adaptation](https://www.sciencedirect.com/science/article/pii/S0010448525001654)、[2026 anisotropic boundary-layer AFT](https://www.sciencedirect.com/science/article/pii/S0045793026000745) | 高阶曲面单元或各向异性边界层 | 目标与当前线性近等边 T3 不同，本轮不选为主线 | 出版摘要/预览，非全文 |

组合曲面论文还提醒参数化存在拓扑限制。不能把任意闭合或高亏格表面虚拟合并后，假定存在一个无缝、无翻折的平面 UV 域。需要局部图表/图表集合或限定可参数化的网格分区；其实现成本必须计入方案。

## 3. 开源项目：从实际实现找可借鉴的机制

以下“源码核对”仅指列出的代表入口，不代表完整审计。下载位于系统 Temp，未向本仓库引入依赖。

| 项目与固定版本 | 实际核对 | 适用输入与可借鉴内容 |
|---|---|---|
| [Gmsh 4.15.2](https://gmsh.info/src/gmsh-4.15.2-source.tgz) | `meshGEdge.cpp` 积分布点；`meshGFaceDelaunayInsertion.cpp` 活动边、optimalPointFrontal/FrontalB、空腔插入；`meshGFaceOptimize.cpp` 平滑 | 原始 CAD 主参考。局部度量、目标高度、活动集合和增量连接；不是直接证明当前用户网格的实际配置或跨分割行为 |
| [Netgen v6.2.2603](https://github.com/NGSolve/netgen/tree/v6.2.2603) | `occgeom.cpp:248–258` 参数域/平面路径；`occmeshsurf.cpp:115–200` 局部切平面和映射；`meshing2.cpp` 基边选择、规则匹配及候选映回入口 | 原始 OCC CAD、纯前沿另一条成熟路线；局部坐标与投影值得参考。规则、前沿碰撞及收口需要完整体系，不能只提取一个理想点公式 |
| [JIGSAW d7859d7](https://github.com/dengwirda/jigsaw/tree/d7859d7e1b71e4ee1a33663a617ada9ffb4777b8) | `rdel_pred_delfront_3.hpp` 的 base_edge 检查受保护边或已合格邻面；`rdel_offh_delfront_3.inc:239–440` 高度限制、几何求交、邻点检查 | restricted Delaunay/Frontal-Delaunay 强参考。已读几何类是离散 R3 节点/面，另有解析椭球；原始 OCC B-Rep 需要几何 oracle 适配 |
| [CGAL 官方文档 6.2.1](https://doc.cgal.org/latest/Mesh_3/index.html) | Mesh_2、Surface_mesher、Mesh_3 特征/拓扑条件；PMP_Remeshing 操作流程；本轮未重下该版本全部源码 | 理论和几何 oracle 参考。Mesh_2 是平面域；Surface_mesher/ Mesh_3 是三维 restricted 框架。不能把平滑曲面保证套给任意尖角 CAD |
| [Geogram 3e2cd6e](https://github.com/BrunoLevy/geogram/tree/3e2cd6e5c6d91975a56d86025cd48fdfddbf9d96) | `mesh_remesh.cpp:62–149`：初始采样→Lloyd→Newton→compute_surface；下载 CVT.cpp，未逐函数完整审查 | 已有网格上的 CVT/RVD 整体布点；非常合适的竞争路线。精确 CAD、约束曲线和界面需额外设计，不是直接读取 B-Rep 即完成 |
| [MMG 8ed2259](https://github.com/MMGTools/mmg/tree/8ed2259164fa4c90be6301d247ecb1db7bd61228) | `mmgs1.c:1272–1318` 分裂/收缩/交换/移动；选读局部几何检查及 Bezier 计算 | MMGS 的已有三角表面度量适应；局部尺寸、特征和操作验收可借鉴。离散几何重建不等于原 CAD 投影 |
| [PMP 215975c](https://github.com/pmp-library/pmp-library/tree/215975ca1efafab97477b18187a4a23b87529036) | `remeshing.cpp:330–390` split/collapse/flip/tangential smoothing；`829–920` 倾向内点价数 6，保护锁定/特征边；官方 remeshing 文档 | 完整局部各向同性重网格基线，比继续加 Laplace 轮次更有意义；投影基准通常是原输入三角表面，需要替换/补充 CAD 支撑 |
| [Triangle](https://www.cs.cmu.edu/~quake/triangle.html) | 作者算法与接口说明，未下载本轮源码 | 二维约束三角化/质量细化，对底层拓扑和鲁棒谓词有参考价值，不是一般曲面生成器 |
| [DistMesh](https://persson.berkeley.edu/distmesh/) | 作者现行项目说明；MATLAB 与 Julia 入口 | 简洁布点对照；作者页面说明 Julia 当前仅 2D，不把它当成现成三维 CAD 方案 |
| [NeuralCVT f1df0dd](https://github.com/sig-pku/NeuralCVT/tree/f1df0dd732f478f0f9b39d1d8f2a75e0ecc55746) | README：公开代码/模型、Geogram 子模块、PyTorch/CUDA/PyG/PyTorch3D，测试平台说明 | 有真实开源入口；输入、依赖和运行条件与当前纯 C++ CAD 主链不同，不采纳作者速度数字作为本项目预期 |

CGAL 的 [各向同性重网格文档](https://doc.cgal.org/latest/PMP_Remeshing/index.html)明确采用分裂、收缩、翻边、平滑和投影。其 [Advancing Front Surface Reconstruction](https://doc.cgal.org/latest/Advancing_front_surface_reconstruction/index.html)则从点云重建表面，名字相似但不是这里的 CAD 前沿网格生成。

固定 SHA：

```text
JIGSAW    d7859d7e1b71e4ee1a33663a617ada9ffb4777b8
Geogram   3e2cd6e5c6d91975a56d86025cd48fdfddbf9d96
MMG       8ed2259164fa4c90be6301d247ecb1db7bd61228
PMP       215975ca1efafab97477b18187a4a23b87529036
NeuralCVT f1df0dd732f478f0f9b39d1d8f2a75e0ecc55746
Netgen    tag v6.2.2603
Gmsh     source 4.15.2, SHA256 BE3F66F225D27BA9FA014F07E83169285DA8A051B0E8AB7103D88066B39BDD3E
```

仓库快照不等于项目正式发布版；Netgen 标签不声称是最新版本。公开源码用于机制核对，自研代码应按论文/算法独立实现；若以后决定直接引入或移植代码，届时核对相应许可和版本，本轮未引入。

## 4. 三个可行方向，为什么推荐第一个

| 方案 | 优点 | 主要成本/风险 | 当前建议 |
|---|---|---|---|
| A：虚拟网格分区＋局部图表/物理度量＋前沿式布点＋Delaunay/局部重连接 | 从生成阶段建立近等边倾向；适配连续 CAD；可复用现有 2D 拓扑、事务与查询；有 Gmsh、Netgen、组合曲面论文参考 | 跨面支撑、图表转换、周期/极点、前沿汇合和候选有效性要专门设计；不是简单粘合各面的 UV | **首选原型方向**，先验证规则域和干净光滑组合面 |
| B：全物理空间 restricted Frontal-Delaunay | 不依赖全局 UV；曲线/曲面统一几何 oracle，有 JIGSAW/CGAL 理论与实现参考 | 通常维护三维 Delaunay/Voronoi 及受限子复形；精确 CAD 交点、多重近邻、薄层、尖角保护复杂；当前 3D 模块不能视为即插即用 | 保留为图表路线遇到系统性失败时的竞争方案；不因“3D 已有”就直接集成 |
| C：特征约束 CVT/RVD 全局重布点＋精确 CAD 支撑 | 强调整体均匀点位，符合形状目标；Geogram 与近年方法提供实际参考 | RVD/能量优化成本、特征约束和拓扑/几何投影体系较大；离散输入效果不能自动推广到 CAD；是否更慢需实测 | 保留高质量对照，必要时提升为主线；当前不先搭建 GPU/学习环境 |

纯 AFT 是 A 的另一种连接组织方式，而非必须再串行添加的第二个生成器。其收口/碰撞体系较复杂，初期更倾向保留现有 Delaunay 增量连接；若跨图表连接不能可靠表达，应评审物理空间局部重连接，而不是伪装成原逐面 CDT 已支持跨面。

完整 split/collapse/flip/move 重网格应作为所有方案的有限优化工具及独立基线。收缩允许改变节点密度和价数，可能解除当前点阵停滞；但必须保护特征、接口和几何支撑。不能假设它只有收尾作用，也不能未经对照就说它必然不如前沿。

## 5. 用户允许跨光滑分割后，必须先解决的表示问题

原 CAD 保持完整，另建网格约束/分区视图。光滑性是必要条件但不是唯一条件：还需同实体、相邻关系可靠、没有物理接口/边界条件约束。近邻、法向恰巧相同或边很短，都不能单独作为合并依据。

建议概念边界如下，尚未编写接口或实现：

1. **几何支撑**：保留 CAD face/edge、参数查询和稳定归属。网格域由多个 CAD 面组成，不丢弃底层几何。
2. **约束分类与虚拟分区**：真实棱边、接口、部件边界继续作为硬约束；经确认的光滑分割仅用于定位/追溯，不强制成为网格边。闭合/高亏格域按有效图表划分，图表边不自动升级为物理特征。
3. **尺寸与布点**：在物理空间定义近等边目标和渐变尺寸，图表负责有效性/定位。曲率误差可要求更小的局部尺度；不以强各向异性当作实现“物理等边”的捷径。真实窄带若仍有硬边，局部尺寸需兼容其宽度。
4. **候选与连接**：活动边推进，候选在相关原 CAD 支撑上定位；跨面的候选和面片必须有明确支撑覆盖，不能一律投影到最近任意面或任意实体。
5. **验收**：跨分割面片仍是平面三角形，需要对所覆盖的 CAD 片区采样检查偏差/法向和域覆盖，维持特征、接口匹配、无翻折/相交。

当前 `CadSurfaceMesher.h:117–121` 用单个 `triangleFaceIds` 和同一面上的 `triangleCornerUV` 表示三角形归属；`SurfaceMesh3D.h:20–24` 只存点和三角连接。跨 CAD 面三角形不能靠“取质心所在 faceId”完整表达支撑。实施前需审查输出、几何验收、归档/显示及下游依赖，区分网格域 ID、CAD 支撑集合与 body/interface 归属。这里只确认表示缺口，未审查/修改每个消费者。

## 6. 第一阶段怎样验证方向，而不是继续堆流程

先做小范围、可退出的原型。以下是拟议实验，不是已批准实现或已经达标：

- **分割不变性**：同一平面/圆柱几何，分别用单面和人为光滑分割的 B-Rep 表示。在几何、尺寸和物理约束相同条件下，比较是否因非物理分割而明显增加窄片、节点或质量损失。
- **拆开因素**：冻结边界节点，比较当前生成、前沿式布点和完整局部重网格；再单独比较积分边采样。保留生成结束、优化前的统计，判断质量从哪一环节得到。
- **适用域逐步扩展**：平面带孔/真实窄带→光滑组合曲面→圆柱接缝/极点→薄层和多部件/接口。先验证合法性和停止行为，再扩大模型规模。
- **统一对照**：Gmsh 算法 5/6，明确平滑开关、实际算法/回退、线程、版本及实际片数。尺寸参数名字相同不代表几何条件等价；需要同时报告 CAD 偏差和特征约束。
- **验收目标**：全角度直方图、三角形三个角均在 50°–70° 的比例、q 的 P5/P50、边长/目标尺寸、内部价数分布；边界/尖角/过渡/汇合区域分开报告。50°–70° 是比较口径，尚未承诺统一比例门槛。
- **几何和成本护栏**：原 CAD 双向采样偏差/法向、闭合/接口一致性、无越域与相交；分别记录查询、候选拒绝、重连接次数、耗时和内存。采样不构成全局几何证明。

若前沿式生成在相同边界下没有显著改善整体角度分布，或跨面图表/支撑造成持续合法性问题，就停止扩张并比较 B/C；若完整局部重网格已达到目标，则采用更简单且有证据的方案。不能以“最后又优化几轮”替代路线验证。

## 7. 对 Gmsh 快与流程数量的回答

Gmsh 同样有边界离散、初始三角化、约束恢复、插点和优化。它不是省略这些基础阶段。普通 Frontal-Delaunay 路径把目标布点、尺寸协调和增量局部连接组织在生成过程中，通常无需先形成一个不理想点阵再大量修复。该描述由固定源码入口支持，实际速度差距仍需公平计时。

当前多轮共享边反馈、面重建和几何验收分别有原因，但不应因已存在而全部固化为未来主线。新方案需要清晰职责，不需要把所有论文算法依次串起来。Gmsh 的漂亮输出也不是每片严格等边的保证；真实特征、边界汇合、曲率和尺寸变化会限制规则排列。

因此，本轮优先级是 **网格分区与 CAD 支撑解耦→生成布点对照→必要的有界优化→成本剖析**；并行、高阶、学习加速和四面体集成后置。

## 8. 本轮验证边界

已检查工作区状态并保留既有改动；核对官方文档、论文的上述读取范围、固定仓库快照代表函数及当前结果数据结构。研究记录的编码、链接路径、固定 SHA 与引用范围另行检查并记录在 progress.md。

本轮没有新网格、质量/性能结果，没有构建/回归/GUI 验收。结论是推荐方向和比较依据，不是生产方案已实现、已比 Gmsh 更好，或全部相关论文均已读完。
