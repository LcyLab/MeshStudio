# Algorithm 6 精读手册（源码级）

> 配套：`docs/Algorithm6-阅读路线与面试要点.md`（阅读顺序与面试要点）。
> 本文是**源码级精读**，每一条都带文件与行号，对着 `F:\Code\MeshStudio` 当前代码可核。
> 目标：读完之后，你能在白板上把每个阶段的**输入→处理→输出**和**为什么这么做**讲清楚。

---

## 0. 一页总览

```
TopoDS_Shape
  │  OccAlgorithm6Adapter::buildModelInput            (cad/occ/)
  ▼
Algorithm6ModelInput { BoundaryInput, faces[], GeometrySizeSource }
  │
  ├─ buildBoundaryMesh                                (boundary/BoundaryRegistry.cpp:97)
  │    ├─ MeshSizeEvaluator::evaluateMeshSize         (boundary/MeshSizeEvaluator.cpp:41)
  │    └─ CurveDiscretization::*                      (boundary/CurveDiscretization.cpp)
  ▼  BoundaryMesh（每条 CAD 边一份共享节点链）
  │
  ├─ 逐面 generateFaceMeshWithRandom                  (pipeline/SurfaceGenerator.cpp:130)
  │   普通分支（12 stage）：
  │     chart              buildFaceChart                        (chart/FaceChart.cpp:13)
  │     initial            buildInitialTriangulation             (initial/InitialTriangulation.cpp:14)
  │                        └ buildWalkTriangulation              (initial/WalkTriangulation.cpp)
  │                        prepareInitialConstraints             (initial/ConstraintRecovery.cpp:17)
  │                        recoverConstraintEdges                (initial/ConstraintRecovery.cpp:165)
  │                        classifyFaceDomain                    (initial/DomainClassification.cpp:11)
  │                        legalizeInitialUvMesh                 (initial/InitialUvLegalization.cpp:13)
  │     bds_conversion     convertOrdinaryInitialMeshToBds        (bds/BdsMeshConversion.cpp:33)
  │                        buildOrdinaryBdsRecoveryRegistry      (bds/BdsCurveRecovery.cpp:41)
  │     boundary_recovery  recoverOrdinaryBdsBoundary            (bds/BdsCurveRecovery.cpp:112)
  │                        └ recoverBdsConstraintEdge            (bds/BdsConstraintRecovery.cpp:42)
  │     bds_domain         classifyOrdinaryBdsDomain             (bds/BdsDomainClassification.cpp:64)
  │                        recoverOrdinaryBdsEmbedded            (bds/BdsCurveRecovery.cpp:116)
  │     bds_sizes          initializeOrdinaryBdsNodeSizes        (bds/BdsNodeSizeInitialization.cpp:10)
  │     initial_legalization prepareOrdinaryBdsInterior          (bds/BdsInteriorPreparation.cpp:90)
  │                        legalizeInitialMesh                  (bds/InitialMeshLegalization.cpp:23)
  │                        initializeFaceMeshVertices           (postprocess/FaceMeshConversion.cpp:63)
  │                        convertBdsToFaceMesh                 (postprocess/FaceMeshConversion.cpp:88)
  │     front_sizes        initializeNodeSizes                   (front/NodeSizeInitialization.cpp:12)
  │     front              initializeFrontTriangles              (front/FrontTriangleInitialization.cpp:14)
  │                        initializeFrontQueue                 (front/FrontQueue.cpp:48)
  │                        generateFrontMesh                    (front/FrontGeneration.cpp:10)
  │     transfer           transferFrontMesh                     (postprocess/FrontMeshTransfer.cpp:61)
  │     smoothing          smoothSurfaceMesh                     (postprocess/SurfaceSmoothing.cpp:148)
  │     final_coordinates  finalizeFaceMeshCoordinates           (pipeline/SurfaceGenerator.cpp:33)
  │   周期分支：见 pipeline/PeriodicSurfaceGeneration.cpp:22
  ▼
  └─ assembleSurfaceMesh                              (pipeline/MeshAssembly.cpp:17)
  ▼  SurfaceMeshResult（三角网格 + 面/体标签 + 逐角UV）
```

---

## 1. CAD 适配：把 B-Rep 变成"纯 C++ 能问的几何"

**文件**：`src/cad/occ/OccAlgorithm6Adapter.h/.cpp`

- `OccAlgorithm6Adapter` 持有一个 `OccCadGeometry`（OCC 查询器），构造时预准备拓扑/曲面适配对象。
- `buildModelInput(vertexSizes)` → `Algorithm6ModelInput`：一次给全模型的边界 + 每个面的 `Algorithm6FaceInput`（含 `chart`、`reparametrize`、`initialMetric`、`front`、`smoothing` 回调，以及周期面的 `periodicChart` 工厂）。
- `buildFaceInput(face)` → 单面输入（需要调用方自己提供共享 `BoundaryMesh`）。
- **BREP 里的 INTERNAL 边作为嵌入约束保留**；不推断 BREP 没保存的外部嵌入关系。

**面试追问**：为什么算法不直接调 OCC？
> 因为 `mesher` 是纯 C++ 库（不链接 OCC/Qt/VTK/Gmsh）。CAD 查询全部走 `std::function` 回调，适配器负责把 OCC 对象翻译成 `Point2D/Point3D + 状态`。这样算法可独立编译、可单测、可换几何内核。

---

## 2. 边界：每条 CAD 边只有一份离散

### 2.1 尺寸求值 `MeshSizeEvaluator::evaluateMeshSize`

`boundary/MeshSizeEvaluator.cpp:41`。顺序**严格**是（对应 Gmsh `BGM_MeshSizeWithoutScaling`）：

```
size = 1e22
if (dim < 2 && sizeFromPoints)  size = min(size, pointSize)
if (dim < 3 && curvature > 0)   size = min(size, 2π/κ/count)   // evaluateCurvatureSize
if (hasEntity)                  size = min(size, backgroundField, entitySize)
if (dim == 1)                   size = min(size, prescribedCurveSize)
if (sizeCallback)               size = sizeCallback(query, size)
size = min(modelCharacteristicLength, size)
size = max(size, sizeMin); size = min(size, sizeMax)
if (size <= 0) { usedInvalidSizeFallback = true; size = modelCharacteristicLength; }
if (entityFactor) size *= factor
result.size = size * options.sizeFactor
```

**关键点**：`sizeMin/sizeMax` 是在倍率**之前**钳制尺寸值，不是最终边长约束。非法尺寸**回退到 modelLength 并显式标记**，不静默。

### 2.2 曲线离散 `CurveDiscretization`

对应 Gmsh `meshGEdge.cpp`。五步：

| 函数 | 做什么 | 关键细节 |
|---|---|---|
| `integrateCurveDensity` (:51) | 递归梯形积分 `∫|C'(t)|/h(t) dt` | 接受条件 `error < precision && depth > 6` 或 `depth > 25`（:89）；每点算 `speed = |C'|`、`density = speed/h` |
| `smoothCurveSizes` (:113) | 前后向 Gauss-Seidel 限制相邻尺寸变化 | `slope = growthFactor - 1`，阈值 `slope * 1.01`；最多 2000 轮（:131,:149） |
| `computeCurvePointCount` (:167) | `max(minSegments+1, int(integral + 1.99))` | 两个端点也算节点 |
| `computeMinimumCurveSegments` (:177) | 圆弧 `0.99 + (minCircle-1)·angle/(2π)`；闭环 ≥4 段 | 按**参数跨度**，不是弦长反推 |
| `distributeCurveParameters` (:224) | 按累计密度等分，表内线性反解 | 保留两个原参数端点 |
| `filterCurveNodes` (:278) | 删除过近点 | **`distance < size * 0.3` 才删**（:328）；最少内点门槛可阻止整批删除 |

### 2.3 边界注册 `BoundaryRegistry::buildBoundaryMesh`

`boundary/BoundaryRegistry.cpp:97`。流程：

1. **先建全部 0D 节点**（含孤立点），再按显式边顺序生成边（:112-119）——对应原生"先 meshGVertex 再 meshGEdge"。
2. 每条边：
   - `geometryDegenerate` → 不积分、不建线，直接入表（:143-147）。
   - 闭环 Line 且中点距起点 < tolerance → `zeroClosedLine`，跳过长度积分（:150-154）。
   - 用 `sizeAt(t) = 1` 积分得**物理长度** `chain.length`（:157-161）。
   - 用 `sizeAt(t) = min(端点0D尺寸, 边1D尺寸)` 积分得**密度表**，再 `smoothCurveSizes(√curveSmoothRatio)`（:168-181）。
   - `computeStoredCurveSize` 存 `MEdgeVertex::getLc`（:202-206）——**这是元数据，不等于 BGM 尺寸**。
   - 过滤过近点（:210-217）。
   - 闭环单关联端点**在过滤之后**重定位（:221-222）——顺序不能反。
3. `chain.meshDegenerate = tooSmall || (closed && nodes.size() < 2)`（:223）。

**面试追问**：为什么每条边只离散一次？
> 相邻面共享同一条 CAD 边。如果逐面各算各的（像 OCC 内置三角化），浮点运算顺序不同会导致端点**不 bit 相同**，网格不水密。共享一份节点链从根上消除这个问题。

### 2.4 共享边重试 `refineUnrecoveredEdges`

`boundary/BoundaryRegistry.cpp:250`。对应 Gmsh `remeshUnrecoveredEdges`：

- 按请求顺序，匹配**无向物理线段**；命中就在参数中点插点（:309-336）。
- 端点参数：0D 端点用曲线 first/last，1D 内点用自己的参数（:313-319）。
- 闭环 0D 端点按"离哪端近"选分支（:322-325）。
- 新点尺寸 = 两端 `storedSize` 平均；若为 -1 才查 BGM（:326-331）。
- 有实际更新的请求 → `++chain.version`（:343）；未命中的记入 `unmatchedRequests`（:348）。
- 返回所有请求边的**关联面**（含未匹配），供编排层标记 PENDING（:296-299）。

---

## 3. 图表：CAD 身份 → 面内 UV

### 3.1 普通图表 `buildFaceChart`

`chart/FaceChart.cpp:13`。核心三件事：

**① 闭合性检查（奇偶法）**
```cpp
const auto toggle = [&](std::uint64_t id) {
    if(!parity.insert(id).second) parity.erase(id);   // 出现两次就抵消
};
...
if(!parity.empty()) throw ... "1D mesh does not form a closed face boundary";
```
（:58-81）——沿边界线段成对 toggle，全抵消才算闭合。

**② 路由判定**

| 条件 | 路由 |
|---|---|
| 任一 boundary/embedded curve `seam == true` | `PeriodicRequired`（:66-69） |
| `physicalNodes.size() <= 3` 且 == 3 | `DirectTriangle`（:113） |
| `physicalNodes.size() < 3` | `SkipInsufficientNodes`（:113） |
| 其它 | `General`（:117 之后继续） |

**③ 生成 UV / 约束 / 初始插入顺序**
- 每个节点查 `nodeCoordinates`（物理→UV）和 `surfacePoint`（UV→XYZ）（:125-131）。
- 约束 = 每条曲线 `samples` 的**相邻对**（:132-138）。
- `initialVertices` 用 `std::unordered_map` 枚举顺序（:141-161）——**这是原生 `getNodeCopies` 的合同**，不能换成排序。

### 3.2 周期环拼接 `assemblePeriodicBoundaryLoop`

`chart/FaceChart.cpp:171`。把每条边的 UV 点串拼成闭合环：
- 每条边有 2 或 4 种状态（正/反 × 主/副分支）（:207-217）。
- 续接时选**严格唯一最近**的候选，且距离 < tolerance（:232-237）——相等时**不能用首项或方向决胜**。
- 首边反向重试一次；闭合失败直接 `NonConverged`（:239-250）。

---

## 4. 初始网格（PolyMesh）

### 4.1 入口编排 `buildInitialTriangulation`

`initial/InitialTriangulation.cpp:14`。
- `SkipInsufficientNodes` → 直接返回空（:20）。
- `DirectTriangle` → 三个 localId 排序成一片（:21-28）。
- `General` → 算 initialVertices 包围盒，**扩盒 ×1.1**（:43-44），调 `buildWalkTriangulation`。

**周期入口** `buildPeriodicInitialTriangulation`（:52）：
- `du/dv < 1200` 时 `makeCube`：把盒变成 `center ± 1`，半边长 = **raw 盒对角线的一半**，再 `*3.5`（:95-102）。保留先 ±1 后 scale 的舍入次序。
- 四个外围角点做 CAD 求值，ID 为 **-1..-4**，尺寸 1e22（:104-118）。
- 调 `buildDivideAndConquerTriangulation`。

### 4.2 约束恢复（PolyMesh）`ConstraintRecovery`

`initial/ConstraintRecovery.cpp`。

**相交判据**（:49）：
```cpp
if(s0*s1 >= 0.) return false;          // 端点/共线不算相交
return !(t0*t1 >= 0.);                 // 保留负零/无穷语义
```
——**严格双方向面积乘积**，与 BDS 的 sys2x2 判据不同。

**追踪穿越边** `collectCrossedEdges`（:78）：先绕 `from` 的顶点星找第一条穿越边，再沿 `opposite→next` 严格追踪。返回码：0 已有、正数=穿越数、-1/-2/-3 失败。

**FIFO 翻边** `recoverOneConstraint`（:129）：
```cpp
std::deque<int> queue(crossed.begin(), crossed.end());
while(!queue.empty()) {
    edge = queue.front(); queue.pop_front();
    if (翻边后新对角线仍与约束相交) { swapHalfEdgeDiagonal(mesh,edge); queue.push_back(edge); }
    else { queue.push_back(edge); if(++withoutSwap >= queue.size()) throw NonConverged; }
}
```
- 成功后给该半边和它的 opposite 打 `curveEntity` 标签（:154-157）。

### 4.3 域分类（PolyMesh）`classifyFaceDomain`

`initial/DomainClassification.cpp:11`。**两次 LIFO 泛洪**：
1. 从 `vertices[0].incidentHalfEdge` 着 **-2（外域）**，返回跨约束边遇到的 `otherSide`（:38）。
2. 从 `otherSide` 着 **faceEntity（内域）**（:40）。
3. 泛洪只跨 `curveEntity == -1`（非约束边）的边（:30）——约束边是天然屏障。

### 4.4 UV 合法化（PolyMesh）`legalizeInitialUvMesh`

`initial/InitialUvLegalization.cpp:13`。**最多 100 轮**全池扫描：
```cpp
if(incircleOriented(a,b,c,d) < 0 && intersectRecoverySegments(a,b,c,d)) swapHalfEdgeDiagonal(...);
```
（:32）——注意前提是两个面 domain **都等于 faceEntity**（:27）。`passLimitReached` 明确标记"到上限仍没收敛"（:39）。

> ⚠️ 这个"UV incircle + 严格凸性"是 PolyMesh 语义；和后面 BDS 的"边中点度量圆"是**两回事**，顺序不能换。

---

## 5. BDS 阶段（最绕的一段）

### 5.1 转换 `convertOrdinaryInitialMeshToBds`

`bds/BdsMeshConversion.cpp:33`。
- `chart.vertices` → `BdsPoint`：**UV/XYZ 来自 chart，不拿 PolyMesh 的 UV 覆盖**（:45-49）。
- 四个外围点按 `initial.vertices[0..3]` 的 UV 在曲面上求值，ID = **-i-1**（:50-61）。
- PolyMesh 面按"可变首边"序转换；找不到物理身份 → 记 `skippedTriangles`（:63-74）。

### 5.2 约束登记 + 恢复 `BdsCurveRecovery`

- `buildOrdinaryBdsRecoveryRegistry`（:41）：把物理节点对 → BDS 局部 ID 对，**无向去重**（:46-54）。缺 inverse 映射的段跳过。
- `recoverCurves`（:61）：逐曲线逐段恢复。
  - 已有边 → 直接返回，不调恢复（:76-83）。
  - 失败段 → 记 `notRecovered`；fatal 则中止（boundary）或继续（embedded），由 `stopOnFatal` 控制（:94, :114, :119）。
  - 曲线**正常结束后**才给空端点赋 `{localID, 0}` 分类（:95-101）——顺序有语义。
  - `needsRefinement = !nativeFatal && (!notRecovered.empty() || refineAllEdges)`（:104）。

### 5.3 单边恢复 `recoverBdsConstraintEdge`

`bds/BdsConstraintRecovery.cpp:42`。
- **求交判据**（:13）：sys2x2 解参数，`|det|/norm < 1e-16` 奇异，闭区间 `[0,1]`（:20-25）。
- **翻边资格** `canSwapBdsRecoveryEdge`（:29）：`a*b > 0` 凸性（:37-38）+ sideEdges 去重（:39）+ 未删除 + 双面 + 非曲线分类。
- **主循环**（:60-88）：每轮扫**全边池**找相交边，取第一个可翻的翻掉再重扫；`iteration > 300` 或无可翻 → fatal。
- **CAD 自交**：目标边与另一条**已登记约束边**相交 → 两条都记 `notRecovered`（:66-71）。

### 5.4 域分类（BDS）`classifyOrdinaryBdsDomain`

`bds/BdsDomainClassification.cpp:11`。和 PolyMesh 版不同：
1. 清所有面分类（:34）。
2. 第一个**含负点**的面填外域（普通 `(1,3)`、周期 `(3,2)`）（:35-39）。
3. 分类双面边跨外域 → 填内域 `(1,2)`（:40-44）。
4. 清外域分类（:45-47）。
5. **普通额外**：负对点补一次内域种子（:49-55）；周期没有。

### 5.5 节点尺寸 `initializeOrdinaryBdsNodeSizes`

`bds/BdsNodeSizeInitialization.cpp:10`。
- 只有 `dimension < 2` 的节点查 BGM，其它直接 `1e22`（:32-40）。
- `dimension == 0` 用 `(u,v)=(0,0)`，`dimension == 1` 用 `(u,0)`（:35）。
- 用**原 MVertex 元数据**（实体/XYZ/曲线参数），不用 BDS 自己的 UV 或分类。

### 5.6 内部清理 `prepareBdsInterior`

`bds/BdsInteriorPreparation.cpp:50`。
1. 删所有空分类面 → `firstCleanup`（:56-62）。
2. 孤边（无面）删除；无分类边赋临时 `CLASS_F=(1,2)`；端点分类只降维（:64-77）。
3. `secondCleanup`（:78）。
4. 删 -1..-4 外围点；**若外围点还有活边则显式 `InvalidTopology`**，不复刻原生悬空指针（:79-83）。

> **`cleanup` 用 `std::partition`**（:26）——**非稳定排列**。所有池引用必须同步映射（:30-46）。这是原生行为，不能改成稳定排序。

### 5.7 度量合法化 `legalizeInitialMesh`

`bds/InitialMeshLegalization.cpp:23`。对应 Gmsh `delaunayizeBDS`。
```cpp
while(true) {
    for (i = 0; i < mesh.edges.size(); ++i) {        // 动态扫描：新追加边同轮继续访问
        if (端点都没改过) → EndpointsUnmodified;
        if (不是双面) → NotTwoFaces;
        configuration = 排序后的四点;
        if (!configurations.insert(configuration).second) → ConfigurationVisited;   // 配置在查询前登记
        midpoint = 边中点;
        metric = source(midpoint);                    // 只在这查 CAD
        circle = shouldFlipInitialEdge(四点, metric);
        if (!circle.inside) → OutsideCircle;
        if (!canSwapInitialBdsEdge(...)) → SwapRejected;
        swapBdsEdgeTopology(...); → Swapped;
    }
    if (!scan.swaps) break;
}
```
- 圆内判据容差按半径平方分档：`≤1e3 → 1e-12`，`≤1e5 → 1e-11`，否则 `1e-9`（:19）。
- **配置去重在度量查询之前**登记，圆外/拒绝的也不再重复查（:43）。

---

## 6. BDS → FaceMesh

### 6.1 `initializeFaceMeshVertices`（:63）

- 只选 `physicalId != 0` 的 BDS 点（外围点跳过）（:73）。
- 用 `boundary` 里 `alive` 的节点构造 `originals`（`SurfaceGenerator.cpp:111`）。
- `bdsToVertex` 记录 BDS 局部 ID → FaceMesh 身份（:81）。
- `maximumVertexId` 是所选原节点 ID 的上限（:79）。

### 6.2 `convertBdsToFaceMesh`（:88）

- 按 BDS 活面池序转换（:92）。
- 缺失节点创建 `FaceCreated`（`id = ++maximumVertexId`，dimension=2，entity=mesh.entity）（:96-107）。
- **先分配全部缺失节点，再判退化**（:108-110）——所以"身份相同的三角形"会被记为 `collapsedTriangles` 而不是提前跳过。
- 不查 CAD、不按坐标/面积过滤。

### 6.3 周期缝副本 `reconstructPeriodicSeamVertices`（:12）

按 `bdsToVertex` 的 **map（局部 ID）序**处理（:18）：
- 首次出现：`parametricCoordinates[id] = bds.uv`（:20）。
- 重复且 `dimension >= 2`：记 `unreconstructedSeams`，**不假造副本**（:21）。
- 重复且 `dimension <= 1`：创建 `PeriodicCopy`，`equivalence[fake] = original`，并**改 `bdsToVertex` 指向副本**（:24-27）。

**面试追问**：为什么不能在进前沿前折叠缝副本？
> 缝两侧在 UV 里是两个位置、物理上是同一个点。前沿要在 UV 里把它们当**两个独立节点**处理（否则 UV 拓扑断裂），等前沿+平滑全部结束后，才用 `equivalence` 映射回物理身份。提前折叠会让缝附近的前沿逻辑出错。

---

## 7. 前沿尺寸与初始化

### 7.1 `initializeNodeSizes`（:12）

对应 Gmsh `buildMeshGenerationDataStructures` 的节点部分。

1. 每个被三角形引用的节点 `sizes[id] = -1`（:50-54）。
2. **物理邻边最短长度**：对每片三角形，取三对顶点的物理距离，更新两端点的 min（:59-66）。
   - **只排除 fake 与其 original 的直接等价边**（`equivalent(a,b)`）（:56-62）——不按物理 ID 或坐标排除。
3. embedded 点取 `min(size, prescribedSize)`（:68-74）。
4. **boundary 半段下限**：`lower = 0.5 * distance(segment[0],segment[1])`，节点尺寸不得低于它（:84-90）。
5. 每个节点先 CAD 反求参数（:107），**仅 embedded** 才查 BGM（:110-116），最后周期 UV 覆盖（:117-118）。
6. 输出按**显式 vertexOrder** 索引（:92-97）。

### 7.2 `initializeFrontTriangles`（:14）

- `meanSizes` 用字面 **`.3333333333`**（:55-56）——⚠️ 与 `insertVertexB` 的 `1./3.` **不是同一个常数**，这是刻意的。
- `targetSize = extendBoundarySize ? min(mesh, background) : background`（:57）。
- `radius = 物理外接圆 R / targetSize`（:63）——**是 R/h，不是 R/最短边**。
- 邻接：把所有身份边键排序后**成对连接**（:72-91）——同一次 `std::sort` 后的相邻对，非流形重复边也照连。

---

## 8. 前沿主循环（算法 6 的灵魂）

### 8.1 队列 `FrontQueue`

| 函数 | 关键点 |
|---|---|
| `frontalRadiusLimit()`（:28） | 固定 `0.5 * √2 * 1` |
| `findActiveEdge`（:30） | 邻片 `-1`（边界）→ 活动；邻片 `radius < limit && > 0` → 活动 |
| `initializeFrontQueue`（:48） | AllTris 按 `orderedTriangles` 快照序插入；活动集只加"活动"的，遇非活动且 `R < limit` 就 **break**（:66） |
| `popActiveTriangle`（:72） | 取 set 头；校验键未变（:80）；`requiresInsertion = active && R > limit`（:88） |
| `registerFrontTriangles`（:94） | 新片登记 AllTris，重复键记 `rejectedDuplicates` |
| `updateActiveTriangles`（:109） | 邻域中活动且 `R > limit` 的加活动集；**不移除陈旧项** |
| `markInsertionFailure`（:127） | R 置 `0`（无包含片）或 `-1`（拒绝），再从 AllTris 移除重插（:141-143） |

**键不可变**：`FrontTriangleKey = (radius, 排序顶点身份)`，set 比较只用这两个（`FrontTriangleOrdering.h:14`）。要改半径必须先 `erase` 再 `insert`，否则 set 序被破坏。

### 8.2 候选 `computeFrontPoint`（`front/FrontPointPlacement.cpp:40`）

```cpp
candidate.planarCenter = planarCircumcenter(p);                       // 裸 UV 外心
candidate.metricQuery  = UV 重心;
candidate.metric       = source(metricQuery);                         // 唯一一次度量查询
circle                 = computeMetricCircumcircle(p, metric);        // 度量外心
a = (edge+2)%3, b = edge;
edgeMidpoint           = 0.5*(p[a]+p[b]);
du,dv                  = circle.center - edgeMidpoint;  (归一化)
metricRatio            = sqrt(du²·gUU + 2·du·dv·gUV + dv²·gVV);
edgeMeanSizes          = 两端点双尺寸的算术平均;
targetSize             = extend ? min(mesh,background) : background;
distanceToCenter       = 度量距离(edgeMidpoint → metricCenter);
height                 = min(targetSize * √3/2, distanceToCenter);   // ★ 近等边目标
fastPoint              = edgeMidpoint + height/metricRatio * (du,dv);
```
（:57-84）

**`height = min(h·√3/2, R_metric)` 的含义**：理想等边三角形的高 = 边长·√3/2；但不能超过"度量外心距离"。取两者较小 → 候选点落在"既接近等边、又不越过外心"的位置。

### 8.3 曲面修正 `correctFrontPointOnSurface`（:93）

- 以**原物理边**为基准构造圆：`center = 物理边中点`，`radius = height`（:115-126）。
- 法向 `norm < 1e-12` → 退化，**保留快候选**并标记（:120-124）。
- 调 `intersectCircleWithSurface`（有限差分 Newton，见 §10.2）。
- 求交失败（`NonConverged`/`GeometryQueryFailed`）→ **`retainedFastPoint = true`，仍返回有限候选**（:136-139）——对应 Gmsh 原生"失败仍返回 true"。

### 8.4 空腔 `buildFrontCavity`（`front/FrontCavity.cpp:12`）

1. 先测 `worst` 片是否在外接圆内（:45）。
2. 不在 → `findContainingFrontTriangle` 找包含片（:48-53）。
3. **DFS 显式栈**，按边 0..2 遍历（:55-73）：
   - 邻片 `-1` 或边在 `nodes.internalEdges`（内部约束边）→ 记为**壳边**（:69）。
   - 邻片未删且圆内 → 进入空腔（:71）。
4. `entryDeleted` 保存每个成员的原 `deleted`，供取消/异常恢复（:58）。
5. 若 worst 本身在圆内，在空腔内找包含片（:74-79）。

**圆内判据**：`distanceSquared < radiusSquared`，**无减容差**（:39）——与初始 BDS 重载不同，这是刻意的。

### 8.5 定位 `FrontPointLocation`

- `mapFrontPointToTriangle`（:29）：sys2x2 解重心坐标；`|det|/norm < 1e-16` 奇异 → `local = (0,0)` 但仍判 inside（:41-49）。
- `findContainingFrontTriangle`（:65）：从起点沿"到 UV 重心的首相交边"走（:88-89）；`iteration > all.size()` 停（:94）；`force` 才扫全 AllTris（:96）。
- `interpolateNodeMeshSize`（:108）：`(1-u-v)·h0 + u·h1 + v·h2`。

### 8.6 插点 `FrontInsertion`

**验收 `validateFrontInsertion`（:40）**，返回码语义：

| code | 条件 | 行号 |
|---|---|---|
| `-1` | 空腔只有 1 片 | :48 |
| `-2` | Euler 不成立：`shell.size() != triangles.size()+2` | :49 |
| `-3` | 面积不守恒：`|oldArea-newArea| > tol*oldArea` | :116 |
| `-4` | tooClose | :117 |
| `-5` | 其它面积等号 | :118 |
| `1` | 接受 | :115 |

- `relativeAreaTolerance = verifyStarShapeness ? 1e-12 : 1e12`（:75）。
- `tooClose = (d1 < L/2 || d2 < L/2 || boundaryHeight < L*0.4 || cos < -0.9999) && !force`（:107）——**保留零距离产生 NaN 的原比较结果**。
- 新片 `meanSizes` 用 **`1./3.`**（:90-91），和初始化阶段的 `.3333333333` 不同。

**提交 `commitFrontInsertion`（:126）**：
- 新片按 shell 序追加（:162）。
- 邻接重连：收集邻域所有边键 → 排序 → 相邻同键成对连接（:163-178）——保留**重复外侧片**的原局部序。

**单点生命周期 `insertFrontPoint`（:200）**，顺序严格：
```
空腔 → 无包含片? → rollback + markInsertionFailure(ContainingTriangleNotFound) → 返回
     → 查 GPoint（surfacePoint）
     → 分配身份 ++maximumVertexId
     → interpolateNodeMeshSize（meshSize）
     → 查 BGM（backgroundSize）
     → append 参数/尺寸槽
     → 若 surface.succeeded → validateFrontInsertion
          → code==1 → commit → 发布 FaceVertex/interiorVertices/索引
          → code∈[-5,-3] → rollback 空腔标记
     → 否则 nativeCode = -9（failed GPoint）
     → markInsertionFailure(PointInsertionRejected) → rollback
```
（:230-290）

**关键不变量**：正常拒绝**保留**已 append 的参数/尺寸槽和已消费的节点身份（:288）——不能紧缩回滚。只有取消/异常才走 `restore()` 全恢复（:206-218）。

### 8.7 主循环 `generateFrontMesh`（`front/FrontGeneration.cpp:10`）

```cpp
while(!queue.activeTriangles.empty()) {
    popActiveTriangle
    if (requiresInsertion) {
        computeFrontPoint(metric)                    // 每查询一次计一次预算
        correctFrontPointOnSurface(surface)
        if (trueBoundary) testParametricDomain(...)  // 周期才门控
        if (!domainTested || domain.inside) insertFrontPoint(...)
    }
    ++pops
}
```
- 预算：`stageControl.maximumGeometryEvaluations = SIZE_MAX`（:35）——**子阶段不再重复扣预算**，由外层 `query()` 统一累加。
- 异常恢复：只恢复**本次未完成轮次**的活动头（`savedHead`）（:21-22, :73）——已完成轮次保留。

---

## 9. 后处理与装配

### 9.1 转移 `transferFrontMesh`（`postprocess/FrontMeshTransfer.cpp:61`）

1. 按 `queue.allTriangles` 顺序输出（:69），校验键未变（:74）。
2. 跳过 `deleted`（:77）。
3. **统一朝向**：以**首活片**的法线为基准，`dot < 0` 就 `swap(v[1],v[2])`（:81-90）。
4. `equivalence` 存在时：先诊断重复对，再**单次**映射，丢弃身份退化片（:91-106）。

### 9.2 平滑 `smoothSurfaceMesh`（`postprocess/SurfaceSmoothing.cpp:148`）

- 只动 `dimension == 2 && entity == mesh.entity` 的自由节点（:170）。
- 邻点参数来自 `reparameterizeEdge`（:183-184）。
- `computeSmoothedPosition`（:93）：邻点方向**度量长度**加权平均（:107-112）。
- **固定五次试探**，`factor` 从 1 每次 `/1.4`（:190-202）——**即使接受也继续缩步**。
- `acceptSmoothedPosition`（:119）：比较 UV 面积（`(1+1e-10)·oldArea < newArea`）和最差 eta（:142）。
- 发布要**第二个 GPoint 也成功**（:195-200）。
- `computeTriangleEta`（:81）：`eta = 1 - |60 - minAngle|/60`。

### 9.3 装配 `assembleSurfaceMesh`（`pipeline/MeshAssembly.cpp:17`）

**节点合并键**（:50）：
```cpp
const Key key{face.bodyId, v.dimension<=1 ? 0 : mesh.entity, v.dimension<=1 ? v.physicalId : id};
```
——0D/1D 按 `(body, physicalId)` 共享；2D 按 `(body, face, id)` 区分。**不按坐标焊接**。

检查项：
- 共享身份坐标必须一致（:53）。
- 无重复片、无零面积片（:72-79）。
- 每边最多 2 面且**方向和为 0**（:92）——即每条边恰好两个反向面。
- CAD 边界段必须出现在最终网格里（:111）。
- 面内边：`incidence > 2` 或（`== 1` 且不是边界段）→ 错误（:116）。

输出：`triangleFaceIds`（CAD 面，零基）、`triangleBodyIds`、`triangleCornerUV`、`boundaryEdges`。

---

## 10. 关键数值细节（面试加分项）

### 10.1 两套"圆内"判据，不要混

| 位置 | 判据 | 容差 |
|---|---|---|
| `legalizeInitialMesh` / `shouldFlipInitialEdge` | 度量圆，`queryDist < radius² - tol` | 按半径²分档 1e-12/1e-11/1e-9 |
| `buildFrontCavity` / `circleTest` | 度量圆，`dist² < radius²` | **无减容差** |

### 10.2 Newton 求交 `CurveSurfaceIntersection.cpp`

- 有限差分 Jacobian：`delta = 1e-4 * |param|`（为 0 时取 `1e-4`）（:69-70）。
- 用**列主元 QR**（`solveColumnPivotedQr`）解 3×3（:79）。
- 收敛：`|step| < 1e-6`（:86）。
- 迭代上限 100（:55）；参数范数 `> 1e6` 停止（:56）。
- 圆求交时先检查初始残差，阈值 `radius * 1e-8`（:178）。
- **`residual == (0,0,0)` 精确零 → 返回 false**（:63-66）——原生行为，不是 bug。

### 10.3 三个"外接圆"

| 函数 | 空间 | 用途 | 退化行为 |
|---|---|---|---|
| `planarCircumcenter` | 裸 UV | 候选的初始外心 | 返回 `(-99999,-99999)` |
| `computeMetricCircumcircle` | 度量 UV | 候选/合法化/空腔 | `|det|/norm<1e-16` → center=(0,0) + 标记 |
| `computePhysicalCircumradius` | 3D | 前沿 R/h | 退化 → `-99999` 哨兵 |

### 10.4 两个不同的 1/3

| 位置 | 常数 | 行号 |
|---|---|---|
| `initializeFrontTriangles` | `.3333333333`（字面） | :55-56 |
| `validateFrontInsertion` | `1./3.` | :90-91 |

刻意保留原版差异，不能统一。

---

## 11. 与 Gmsh 源码对应表

| 模块 | Gmsh 对应 |
|---|---|
| `MeshSizeEvaluator` | `BackgroundMeshTools.cpp` `BGM_MeshSize` |
| `CurveDiscretization` | `meshGEdge.cpp` `meshGEdgeProcessing` |
| `BoundaryRegistry` | `meshGVertex` / `meshGEdge` / `remeshUnrecoveredEdges` |
| `FaceChart` | `meshGFace.cpp::meshGenerator` / `getNodeCopies` |
| `PeriodicFaceChart` | `meshGFace.cpp:2091-2303`（周期环） |
| `InitialTriangulation` / `WalkTriangulation` | `GFaceInitialMesh`（recover=0） |
| `DivideAndConquerTriangulation` | `DocRecord` 分治 |
| `ConstraintRecovery` | `GFaceInitialMesh`（recover=1）`recover_edge` |
| `DomainClassification` | `Color`（PolyMesh 域着色） |
| `BdsMeshConversion` / `BdsTopology` | `BDS` |
| `BdsConstraintRecovery` | `BDS::recover_edge` / `BDS_SwapEdgeTestRecover` |
| `BdsSwapQualification` | `swap_edge(..., BDS_SwapEdgeTestQuality(false))` |
| `InitialMeshLegalization` | `delaunayizeBDS` |
| `BdsInteriorPreparation` | 域后 `cleanup` |
| `FrontQueue` | `MTri3` / `compareTri3Ptr` / `findActiveEdge` |
| `FrontPointPlacement` | `optimalPointFrontal` / `optimalPointFrontalB` |
| `FrontCavity` / `FrontInsertion` | `insertAPoint` / `insertVertexB` |
| `FrontGeneration` | `bowyerWatsonFrontal` |
| `SurfaceSmoothing` | 原生平滑 / `_isItAGoodIdeaToMoveThatVertex` |
| `MeshAssembly` | 最终装配 |

---

## 12. 常见追问与标准答案（30 题）

**架构**
1. **项目里有几条表面链？** legacy（`generateCadSurfaceMesh`，产品默认）和 Algorithm 6（`gmshReference::generateSurfaceMesh`）。从 `SurfaceGenerator` 读的是后者。
2. **为什么在本项目中自行实现？** 为了掌握共享棱节点身份、CAD 标签、逐角 UV 和各阶段数据流，使生成核心不依赖 Gmsh，并通过独立对照定位差异。这不代表 Gmsh 无法提供这些能力，也不代表本项目已实现多区域公共剖分。
3. **算法链依赖 OCC 吗？** 不。`mesher` 是纯 C++，CAD 查询走 `std::function` 回调，适配器在 `src/cad/`。
4. **`RunControl` 三个字段？** `keepGoing`（取消）、`maximumGeometryEvaluations`（查询预算）、`maximumNodes`（节点预算）。
5. **为什么状态用返回值而不是异常？** 取消/预算不能被 `catch(...)` 吞成"未知几何错误"；`OperationFailure` 只用于跨模块提前退出。

**主流程**
6. **两层调度？** 单面内部 12 stage + 模型级 PENDING 遍历。
7. **面恢复失败怎么办？** `refineUnrecoveredEdges` 加密共享边 → 只失效关联面 → 重访，上限 `maximumRecoveryRetries`。
8. **stage 顺序？** chart → initial → bds_conversion → boundary_recovery → bds_domain → bds_sizes → initial_legalization → front_sizes → front → transfer → smoothing → final_coordinates。
9. **为什么域分类在约束恢复之后？** 域分类要跨"约束边"做屏障泛洪，约束边必须先存在。
10. **为什么只失效关联面？** 无关面的结果仍然有效，重建它们纯属浪费。

**数据结构**
11. **三套数据结构？** `FaceMesh`（前沿前面网格）、`BdsMesh`（三池+双向邻接）、`FrontMesh`（前沿运行池）。
12. **为什么 PolyMesh 和 BDS 各恢复一次？** 两套求交判据不同（严格面积乘积 vs sys2x2 闭区间），中间拓扑也不同。
13. **`equivalence` 干什么？** 周期缝副本 → 原节点映射，平滑后才折叠。
14. **FrontQueue 键为什么不可变？** set 序依赖键；改半径必须先 erase 再 insert。
15. **`BdsMesh` 为什么保留负 ID 和空洞？** 外围点 -1..-4 有语义；删点保留编号上限，跨 cleanup 用映射表更新引用。

**算法细节**
16. **候选 `height = min(h·√3/2, R)` 的含义？** 等边理想高度与外心距离取小，保证近等边且不越过外心。
17. **两套圆内判据差在哪？** 初始合法化减容差并按半径分档；前沿空腔不减容差。
18. **`validateFrontInsertion` 的返回码？** 1 接受；-1 单片、-2 Euler、-3 面积、-4 tooClose、-5 面积等号。
19. **正常拒绝要保留什么？** 已 append 的参数/尺寸槽、已消费的节点身份；只销毁 live 节点、恢复空腔标记。
20. **`failed GPoint`（-9）怎么处理？** 仍查 BGM 并 append 槽，再走 -9 拒绝路径。
21. **平滑为什么固定五次试探？** 原生行为：即使接受也继续缩步试探更小的移动。
22. **平滑发布为什么要第二个 GPoint？** 原生在验收成功后还会再查一次点，只有它也成功才发布。
23. **`tooClose` 四个条件？** `d1 < L/2`、`d2 < L/2`、`boundaryHeight < L*0.4`、`cos < -0.9999`。
24. **`filterCurveNodes` 删除阈值？** `distance < size * 0.3`；有最少内点门槛保护。
25. **`std::partition` 为什么重要？** cleanup 是非稳定排列，是原生行为；池引用必须同步映射。
26. **两个 1/3 常数为什么不同？** 初始化用 `.3333333333`，插点用 `1./3.`，刻意保留原生差异。
27. **Newton 求交用什么线性求解？** 列主元 QR（`solveColumnPivotedQR`），不是 LU。
28. **装配怎么合节点？** 0D/1D 按 `(body, physicalId)`，2D 按 `(body, face, id)`；不按坐标焊接。
29. **装配检查哪些？** 重复片、零面积片、每边 2 面且方向和 0、CAD 边界段完整、面内边非开边。
30. **生成成功等于质量合格吗？** 不等于。生成状态和独立质量审计分开报告。

---

## 13. 五分钟白板讲解脚本

> 建议按这个顺序讲，每段 30 秒：

1. **定位**：项目有两条表面链；`SurfaceGenerator` 是复刻 Gmsh Algorithm 6 的那条，纯 C++，OCC 只当查询器。
2. **入口**：`generateSurfaceMesh` 先建共享边界，再逐面生成，最后装配。面失败会加密共享边并只失效关联面。
3. **单面 12 步**：图表 → 初始剖分/约束恢复/域/UV 合法化 → 转 BDS 再恢复一遍 → 域/尺寸/清理/度量合法化 → 转面网格 → 前沿尺寸 → 前沿 → 转移 → 平滑 → 最终 UV。
4. **前沿**：按 R/h 降序取活动片 → 算近等边候选（`min(h·√3/2, R)`）→ 圆修正 → 空腔 → 验收（面积/Euler/距离/角度）→ 提交 → 队列局部更新。
5. **关键取舍**：状态显式返回、成功才原子发布、身份与坐标分离、两套恢复判据、两个尺寸状态、生成与审计分开。
6. **现状**：34 组相对验收通过、93 轮性能通过（最慢中位比 1.64），保留 3 个双方共有的绝对失败——如实报告，没有放宽门槛。

---

## 14. 跑一遍验证你的理解

```bash
# 端到端（矩形/圆柱含缝/球面含极点/失败原子性）
./build/Release/GmshSurfacePipelineTest.exe

# 逐阶段对照
./build/Release/GmshFaceChartTest.exe
./build/Release/GmshFrontGenerationTest.exe
./build/Release/GmshFrontPointInsertionTest.exe
./build/Release/GmshMeshAssemblyTest.exe
./build/Release/GmshInitialMeshLegalizationTest.exe
./build/Release/GmshSurfaceSmoothingTest.exe

# 打开诊断开关看真实内部状态
MESHSTUDIO_DEBUG_CDT=1 ./build/Release/GmshFrontGenerationTest.exe
```

对照着源码读输出，比只看代码快得多。
