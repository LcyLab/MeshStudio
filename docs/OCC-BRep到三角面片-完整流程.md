# 从 OCC 的 B-Rep 到三角面片：完整流程

> 面向"完全不知道这条路怎么走"的读者。
> 所有内容都对着 `F:\Code\MeshStudio` 的真实代码，带文件名与行号。
>
> **先记住一件事**：你项目里有**两条完全不同**的路径，它们的产物看起来一样（都是三角形），
> 但能力差很多。分不清这两条，后面全乱。

---

## 0. 先搞清 B-Rep 到底是什么

STEP / IGES / BRep 文件里存的是**边界表示（Boundary Representation, B-Rep）**。
它不是网格，里面**一个三角形都没有**。它存的是**数学定义**：

```
Solid（实体）
 └─ Shell（壳）
     └─ Face（面）  =  Surface（曲面） + Wire（裁剪环）
         └─ Wire（环）= 一圈有向的 Edge
             └─ Edge（棱） = Curve（曲线） + 两个 Vertex
                 └─ Vertex（顶点） = 一个三维点
```

**最容易搞混的一点**：

> **面（Face）≠ 曲面（Surface）**

一个曲面是无限大的（比如一个无限圆柱面）；**面 = 曲面 + 裁剪环**，
裁剪环在这个无限曲面上圈出一块"有效区域"。

所以：
- 一个圆柱侧面 = 圆柱面 + 两条环（上下两个圆）
- 一个带孔的法兰面 = 平面 + **两个**环（外环 + 孔环）

这就是为什么代码里到处是 `loops`（多个环）而不是一个 `loop`。

### 两种"参数"要分清

| 参数 | 属于谁 | 例子 |
|---|---|---|
| `t` | **曲线**参数（棱上的一维参数） | 棱 `[0, 1]` 或 `[0, 2π]` |
| `(u, v)` | **曲面**参数（面上的二维参数） | 圆柱面 `u∈[0,2π]`（角度），`v∈[0,h]`（高度） |

**同一条棱，在三维里是一条曲线（用 `t` 表示），在它相邻的每个面上还有各自的 `(u,v)` 表示**。
这个"棱在面里的 UV 表示"就叫 **p-curve**（`CurveOnSurface`）。
→ 这是理解后面一切的关键：**一条棱，一套三维节点，N 套 UV。**

---

## 1. 两条路径总览

```
                          ┌─────────────────────────────────────────┐
STEP/IGES/BRep 文件 ──►   │  ShapeLoader::load  →  TopoDS_Shape     │
                          └────────────────┬────────────────────────┘
                                           │
              ┌────────────────────────────┴────────────────────────────┐
              ▼                                                         ▼
  ┌───────────────────────────┐                        ┌────────────────────────────────┐
  │ 路径 A：OCC 自己三角化     │                        │ 路径 B：自研三角化              │
  │ ShapeTessellator          │                        │ OccCadGeometry + mesher        │
  │  → ShapeToPolyData        │                        │  → generateCadSurfaceMesh      │
  │  → BRepMesh_IncrementalMesh│                       │                                │
  │                           │                        │                                │
  │ 产物：三角汤（无拓扑）      │                        │ 产物：三角网格 + CAD 面标签     │
  │      + FaceId / SolidId   │                        │      + 棱标签 + 逐角 UV         │
  │                           │                        │      + 共享节点一致             │
  │ 用途：喂四面体引擎          │                        │ 用途：「三角面片」按钮           │
  └───────────────────────────┘                        └────────────────────────────────┘
```

| | 路径 A（OCC 内置） | 路径 B（自研） |
|---|---|---|
| 代码量 | ~200 行 | ~12 000 行（`mesher/`） |
| 共享棱节点 | ❌ 逐面独立，**近似相等但不 bit 相同** | ✅ 每条棱只有**一套**节点 |
| 逐角 UV | ❌ 没有 | ✅ 有（`CadCornerUV`） |
| 面 / 棱标签 | 只有 FaceId / SolidId | FaceId + **EdgeId** + BodyId |
| 可控性 | 只有一个 `deflection` 旋钮 | 尺寸 / 偏差 / 法向角 / 质量 / 预算 |
| 后续能做什么 | 只能喂体网格（还得先焊接） | 可当**多区域体网格的边界**（有标签） |

**一句话**：路径 A 是"能用的快路"，路径 B 是"可控的正路"。

---

## 2. 路径 A：OCC 自己三角化（5 步，很好懂）

**入口**：`src/core/ShapeTessellator.cpp` → `src/core/ShapeToPolyData.cpp`

### 步骤

**① 载入几何**

```cpp
ShapeLoader::load(filePath, shape, &err);     // 得到 TopoDS_Shape
```

**② ★ 先 Clean（这一步不能省）★**

`ShapeToPolyData.cpp:76`

```cpp
BRepTools::Clean(shape);
BRepMesh_IncrementalMesh mesher(shape, deflection, false, 0.5, true);
```

代码注释里写得很清楚：

> 形状可能已被更早的调用（视口加载）用**较粗**的弦高离散过，
> 而 `BRepMesh_IncrementalMesh` 在**已有三角化满足新弦高时会直接复用旧网格**。
> 先清掉才能保证按本次 deflection 重新离散。

→ **这是个真实的坑**：OCC 的三角化结果**存在 Shape 上**（`Poly_Triangulation`），
不清就会拿到上一次的旧结果，参数改了也不生效。

**③ 弦高（deflection）怎么定**

`ShapeToPolyData.cpp:56`

```cpp
double deflectionFor(shape, targetSize) {
    double base = diag * 0.001;                     // 包围盒对角线的千分之一
    if (targetSize <= 0) return base;
    return std::min(base, targetSize * 0.25);       // 弦高 ≤ 目标边长的 1/4
}
```

`BRepMesh_IncrementalMesh` 的参数含义：

| 参数 | 值 | 含义 |
|---|---|---|
| `deflection` | 弦高 | 三角面离真实曲面的**最大允许距离** |
| `isRelative` | `false` | 用**绝对**弦高，不用相对 |
| `angle` | `0.5` rad（≈28.6°） | 相邻三角面法向的最大允许夹角 |
| `parallel` | `true` | 并行离散 |

**④ 逐面取三角化结果**

`ShapeToPolyData.cpp:95-125` —— 遍历面表，对每个面：

```cpp
TopLoc_Location location;
const Handle(Poly_Triangulation)& tri = BRep_Tool::Triangulation(face, location);
for (i = 1..tri->NbNodes())
    points->InsertNextPoint(tri->Node(i).Transformed(location.Transformation()));
for (t = 1..tri->NbTriangles())
    triangles->InsertNextCell(3, {base+n1-1, base+n2-1, base+n3-1});
```

**⑤ 平铺成"三角汤"**

每个面有自己的顶点数组 → 全部**首尾相接**塞进一个 `vtkPoints`，
同时给每个三角形记上 `FaceId` 和 `SolidId`（`CellData`）。

### 路径 A 的两个后果

1. **共享棱的顶点只是"近似相等"**
   相邻两个面各自离散同一条棱，得到的点坐标**不保证 bit 相同**
   （浮点运算顺序不同）。所以三角汤里同一条棱会有**两份**顶点。
   → 直接判水密会失败 → **必须先焊接**（`mesher::weldSurfaceMesh`）。

2. **没有 UV、没有棱标签**
   所以路径 A 的结果**不能**直接当"多区域体网格的边界"用。

---

## 3. 路径 B：自研三角化（重点）

**入口**：`SurfaceMeshEngine::run`（`src/core/SurfaceMeshEngine.cpp:25`）

```cpp
ShapeLoader::load(filePath, shape, &err);          // ① 读文件
OccCadGeometry geometry(shape, true);              // ② OCC 适配（预准备）
mesher::generateCadSurfaceMesh(geometry, criteria, ...);  // ③ 自研核心
```

整个流程分 **7 个阶段**：

```
② OCC 适配  →  ③a 边界采样  →  ③b 逐面 CDT  →  ③c 域分类
            →  ③d 细化  →  ③e 输出/相交检查  →  ③f 后置质量
```

---

### ② OCC 适配：把 B-Rep 翻译成"纯 C++ 能问的几何"

**代码**：`OccCadGeometry::OccCadGeometry`（`src/core/OccCadGeometry.cpp:122`）

**为什么要这一层？** 因为自研算法库 `mesher/` **不依赖 OCC**。
它只认一个抽象接口 `GeometrySource`（`CadSurfaceMesher.h:23`），
`OccCadGeometry` 是这个接口的 OCC 实现。

`GeometrySource` 就是"算法想知道的全部事情"：

```cpp
virtual int vertexCount() / vertexPoint(v)          // 顶点
virtual int edgeCount()   / edgeVertices(e) / edgeRange(e) / edgePoint(e,t)   // 棱
virtual int faceCount()   / faceTopology(f)                                   // 面
virtual Point2D edgeUV(f, use, t)                    // ★ 棱在面上的 UV（p-curve）
virtual Point3D facePoint(f, uv) / faceNormal(f, uv) // 曲面求值
virtual void faceDerivatives(f, uv, du, dv)          // ★ 一阶导数（度量要用）
virtual bool faceParameterBounds(f, bounds)          // 参数范围
virtual int faceBodyId(f)                            // 实体归属
```

构造函数按顺序做这些事：

**a) 有效性检查**

```cpp
if (shape.IsNull() || !BRepCheck_Analyzer(shape).IsValid()) { m_error = ...; return; }
```

坏 B-Rep 直接拒绝（除非显式允许"有界容差协调"）。

**b) 建四张索引表（ID 全部从 0 开始）**

```cpp
TopExp::MapShapes(m_shape, TopAbs_VERTEX, m_vertices);
TopExp::MapShapes(m_shape, TopAbs_EDGE,   m_edges);
TopExp::MapShapes(m_shape, TopAbs_FACE,   m_faces);
TopExp::MapShapes(m_shape, TopAbs_SOLID,  m_solids);
```

**这一步很重要**：B-Rep 里对象是**指针**，算法里要用**整数 ID**。
`MapShapes` 就是"指针 → 稳定整数 ID"的映射表，整个生成过程都靠它。

**c) 顶点坐标预取** —— 一次性把 `BRep_Tool::Pnt` 全取出来。

**d) 每条棱准备几何**

```cpp
prepared.vertices = {起点ID, 终点ID};
prepared.range    = {first, last};        // BRep_Tool::Range(edge, first, last)
prepared.degenerate = BRep_Tool::Degenerated(edge);   // 退化棱（极点）
prepared.curve.emplace(edge);             // BRepAdaptor_Curve，只建一次
```

**e) 每个面准备曲面**

```cpp
m_surfaces.emplace_back(face);   // BRepAdaptor_Surface，Restriction=true，只建一次
```

> **★ 性能坑（真实踩过）★**
> OCC 的 `BRepAdaptor_Surface::Initialize` 默认会**重算裁剪 UVBounds、遍历面上所有棱**。
> 如果每次点/法向/导数查询都新建一个适配对象，`move.stp` 会推算出 **139 万次构造**，
> 核心耗时 **99.8 秒**。预建一次缓存后降到 **4.5 秒（22 倍）**。

**f) 解析每个面的 wire，得到 `EdgeUse` 列表**

```cpp
for (TopExp_Explorer wires(face, TopAbs_WIRE); ...)
  for (BRepTools_WireExplorer e(wire, face); ...) {
      const bool reversed = (e.Orientation() == TopAbs_REVERSED);
      loop.push_back({edgeId, reversed, 0});       // EdgeUse{棱ID, 是否反向, 周期分支}
  }
```

**g) 取 p-curve**

```cpp
trim.curve = BRep_Tool::CurveOnSurface(edge, face, trim.first, trim.last);
m_trimCurves.emplace({faceId, edgeId, branch}, trim);
```

这就是"棱在这个面上的 UV 表示"。**周期缝（seam）会有两条 p-curve**，所以要 `branch ∈ {0,1}`。

**h) ★ 用动态规划联合选择"走向 × 周期分支"★**

这是构造函数里最巧的一段（`OccCadGeometry.cpp:170` 附近）。

**问题**：`BRepTools_WireExplorer` 给的 `Orientation()` **不一定**是该棱在全局棱表里的方向；
周期缝还有两个分支。每个 `EdgeUse` 有 2 或 4 种可能状态，
一条 wire 上有 N 条棱 → **朴素枚举是 2^N**。

**做法**：把它当**最短路问题**，用 DP。

```cpp
// 代价：相邻两个 EdgeUse 在接缝处
//   ① 3D 端点必须是同一个顶点（不是 → 罚 1e30，直接淘汰）
//   ② UV 端点距离平方（越小越连贯）
auto transition = [&](const EdgeUse& left, const EdgeUse& right) {
    if (vertexEnd(left,false) != vertexEnd(right,true)) return 1e30;
    return uvEnd(left,false).SquareDistance(uvEnd(right,true));
};
// 对 loop 上每个位置枚举状态，逐位 DP，取全局最小代价，再回溯出选择
```

**为什么要这么麻烦**：如果选错方向/分支，这条 wire 的 UV 环会**接不上**
（在 UV 里断开或者绕反），后面的 CDT 必然失败。

**i) 记录实体归属**

```cpp
m_faceBodies[face] = solidId;   // 一个面只能属于一个实体，否则报错
```

> 装配体（`phone` 有 132 个实体）就靠这个标签区分。

---

### ③a 边界采样：先造"棱"，再造"面"

**代码**：`generateSurfacePass` 的 `MeshStage::BoundarySampling` 段（`CadSurfaceMesher.cpp:1150` 起）

**核心原则：每条 CAD 棱只生成一套三维节点，相邻面共享。**

这是自研路径与路径 A 最本质的区别。

**做法**：对每条棱维护一条"采样链" `CadBoundarySample{ t, global节点ID }`，
用**递归二分**加密：

```
split(a, pa, b, pb, depth):
    取中点 m = (a+b)/2
    pm = geo.edgePoint(edge, m)                 // 三维点
    对每个 incident face：
        ua = geo.edgeUV(face, use, a)           // 该棱在面上的 UV
        um = geo.edgeUV(face, use, m)
        nm = geo.faceNormal(face, um)
        检查三个判据：
          ① 弦偏差    lineDeviation(pa, pb, pm)   ≤ maxDeviation
          ② 法向角    angleDeg(na, nb)             ≤ maxNormalAngleDeg
          ③ 局部尺寸  |pb - pa|                    ≤ 尺寸场给的 h
    有任一不满足 → 继续二分（深度 ≤ 32）
```

**细节**：
- 子区间**复用左右端点的逐面 UV/法向，只查新中点**（性能优化）
- 闭合曲线（`v0 == v1`）**至少保留 4 段**，否则一个整圆可能只剩一条弦
- 退化棱（极点）单独处理

**这一阶段结束后**：每条棱有一串三维节点，且**每个节点都带每个相邻面的 UV**。

---

### ③b 逐面构造 CDT

**代码**：`MeshStage::InitialTriangulation` 段（`CadSurfaceMesher.cpp:1417` 起）

对每个 CAD 面：

**① 把棱采样拼成 UV 环（loop）**

```cpp
for (const auto& trimLoop : topology[f].loops)
  for (const EdgeUse& use : trimLoop) {
      const auto& seq = state.boundary->edge(use.edge).samples;
      for (k = 0 .. seq.size()-1) {
          const auto& s = use.reversed ? seq[seq.size()-1-k] : seq[k];  // 按走向取
          const Point2D uv = queries.edgeUV(f, use, s.t);               // ★ 转成 UV
          const int local = addLocal(s.global, uv);                     // ★ 记下 全局ID ↔ 局部UV
          loop.push_back(local);
      }
  }
```

**关键**：`w.global[local]` 记住"这个局部 UV 点对应哪个全局三维节点"。
这就是**面与面之间共享节点**的连接方式。

**② 检查 UV 弦是否交叉**

```cpp
if (orient2d(uv[a],uv[b],uv[p]) * orient2d(uv[a],uv[b],uv[q]) < 0 &&
    orient2d(uv[p],uv[q],uv[a]) * orient2d(uv[p],uv[q],uv[b]) < 0) {
    requestSharedSplit(...);   // 请求把这条共享棱再分细，然后重启这一轮
}
```

**为什么**：曲线在三维里不交叉，但它的**弦**（直线段）在 UV 里可能交叉。
此时 UV 多边形是自交的，CDT 做不了 → 只能把原棱分细。

**③ 算 UV 包围盒 + 在中心求度量**

```cpp
FaceMetric metric;
surfaceMetricAt(queries, f, {0.5*(u0+u1), 0.5*(v0+v1)}, metric, &err);
metric.origin = {u0, v0};
```

`FaceMetric` 就是 §2.3 说的 `M = JᵀJ/h²`（存成 `xx, xy, yy` 三个分量）。

**④ 两条初始化路线**

```cpp
const bool rectangular = rectangularGrid(...);      // 规则矩形域
if (rectangular) legalize(...);                     //   直接合法化
if (!rectangular && !synchronized) {                // 一般裁剪域
    delaunay2D(mesh, w.uv, true, &linear, observation);   // ① 带度量的 Delaunay
    enforceConstraints(mesh, w.constraints, &err);        // ② 恢复约束边
    legalize(mesh, w.constraints, ...);                   // ③ Lawson 合法化
}
```

| 路线 | 什么时候用 | 为什么 |
|---|---|---|
| `rectangularGrid` | 面是完整矩形（如圆柱侧面） | 按物理度量直接布点，比通用 CDT 稳、快 |
| `delaunay2D + enforceConstraints` | 一般裁剪域（带孔、任意环） | 通用 |

注意 `delaunay2D(..., preserveCoordinates=true, &linear, ...)`：
**不移动输入点**，且用**度量**而不是裸欧氏距离判断。

**⑤ 恢复约束边**

`enforceConstraints` 保证"棱采样之间的连线"真的出现在三角形网格里
（否则面片边界会有裂缝）。手段是**翻转凸四边形**或**插点**。

---

### ③c 域分类：哪些三角形"在面里"

```cpp
SurfaceDomainState domain;
domain.initialize(mesh, w.constraints, &err, observation);
```

**问题**：CDT 覆盖整个 UV 包围盒，但**带孔面的孔里也会长出三角形**，必须删掉。

**做法**：不是"算三角形重心在不在环里"（那样在凹域/多环时会错），
而是**沿三角形邻接传播**：从外环出发做 BFS，
跨过外环/孔时**翻转奇偶值**，内部特征不改变标记。

---

### ③d 细化：尺寸、偏差、法向、质量

**代码**：`MeshStage::Refinement`

和 §③a 的边界采样是同一套判据，只是这次作用在**面内部**的三角形上：

- 按映回三维的**边长**、**曲面偏差**、**法向变化**、**质量**（radius-edge ratio）、**翻折**挑坏片
- 超长边 / 大法向跨度 → **优先二分**
- 二分后**立即**从新点一环传播 Lawson 翻边恢复 CDT（这一条是踩坑换来的：
  延迟恢复会导致极薄翻折片死循环）

**共享边反馈**：如果细化点侵入某条约束棱的直径域 → 登记请求 →
`CadBoundaryRegistry` 只让**这条棱和它的关联面**失效，其余面**复用已验证结果**。

---

### ③e 输出与相交检查

- 按 CAD 面的**统一方向**输出（逐三角形按法向翻转会掩盖翻折，不允许）
- 检查物理边关联与方向、CAD 约束、采样判据
- **BVH 相交检查**：检测面片是否穿越其他面片。
  若检测到穿越且仍有可分辨的 CAD 近似误差 → 按误差一半**收紧相关面的偏差**，
  重新离散共享棱并重建关联面

---

### ③f 后置质量优化

固定全部共享裁剪/缝/极点节点，做**受限翻边**与**内部节点移动**，
要求局部最坏值下降；失败**回滚**到已接受几何网格。

---

### 最后：包装成 VTK

**代码**：`SurfaceMeshEngine.cpp:39-104`

```cpp
points   ← mesh.vertices
polygons ← mesh.triangles
CellData:
  CadFaceId    ← 每个三角形属于哪个 CAD 面
  CadBodyId    ← 属于哪个实体
  CadCornerUV  ← 6 个分量 = 3 个角点的 (u,v)      ★ 路径 A 没有这个
  CadEdgeId    ← 3 个分量 = 3 条边的 CAD 棱 ID（内部边 -1）
FieldData:
  CadSurfaceDiagnostics, CadAssemblyPolicy, CadBodyInteractions
  CadAcceptedBRep, CadInputReconciliation（仅在做过容差协调时）
```

---

## 4. 一张图看清数据流

```
TopoDS_Shape
   │
   │  OccCadGeometry 构造函数
   │  ├─ MapShapes → 顶点/棱/面/实体 的 整数ID 表
   │  ├─ 每条棱 → {两端点, 参数范围, 退化标记, BRepAdaptor_Curve}
   │  ├─ 每个面 → BRepAdaptor_Surface（只建一次！）
   │  ├─ 每个面 → WIRE → EdgeUse{棱ID, 反向?, 分支?}
   │  ├─ 每个 (面,棱,分支) → p-curve（CurveOnSurface）
   │  └─ DP 联合选 走向×分支（代价 = 3D端点不一致 + UV端点距离²）
   ▼
GeometrySource（纯 C++ 接口）
   │
   │  ① 边界采样：每条棱一条采样链，递归二分
   │     ├─ 三维弦偏差 ≤ maxDeviation
   │     ├─ 法向角 ≤ maxNormalAngleDeg
   │     └─ 边长 ≤ 尺寸场 h
   ▼
每条棱：一串全局节点 {t, globalId}，每个节点带各相邻面的 UV
   │
   │  ② 逐面：拼 UV 环 → 查弦交叉 → 求度量 M=JᵀJ/h²
   │     ├─ 矩形域 → rectangularGrid → legalize
   │     └─ 一般域 → delaunay2D → enforceConstraints → legalize
   ▼
Mesh2D（局部 UV 网格）  ←→  global[] 映射表  ←→  全局三维节点
   │
   │  ③ 域分类（沿邻接传播 + 奇偶翻转）
   │  ④ 细化（坏片队列 / 外心插点 / 侵入检查 / 共享边反馈）
   │  ⑤ 输出（统一方向 + BVH 相交检查）
   │  ⑥ 后置质量（受限翻边 + 内部移动，失败回滚）
   ▼
SurfaceMeshResult
   ├─ mesh.vertices / mesh.triangles        三维网格
   ├─ triangleFaceIds / triangleBodyIds     面 / 实体标签
   ├─ triangleCornerUV                      逐角 UV
   └─ boundaryEdges                         棱段 → CAD 棱 ID
```

---

## 5. 初学者最容易卡的 5 个点

| # | 卡点 | 解释 |
|---|---|---|
| 1 | **"面"和"曲面"不是一回事** | 面 = 曲面 + 裁剪环。一个曲面可以被多个面共用 |
| 2 | **一条棱为什么有"多套 UV"** | 三维只有一套节点（保证不裂），但每个相邻面里各有自己的 `(u,v)` 表示（p-curve） |
| 3 | **为什么要 DP 选走向和分支** | 方向选错 → UV 环接不上 → CDT 必失败；周期缝有两个分支 |
| 4 | **为什么路径 A 的顶点会重复** | OCC 是**逐面**离散的，相邻面各算各的，浮点结果不保证相同 |
| 5 | **为什么 UV 里还要算"度量"** | UV 的量纲可能是"角度"，直接量长度会得到错误的三角形 |

---

## 6. 建议的读代码顺序

对着上面的阶段编号走，**不要按文件行号顺序读**：

| 顺序 | 文件 | 读什么 | 对应阶段 |
|---|---|---|---|
| 1 | `src/core/SurfaceMeshEngine.cpp` | 全部（112 行） | 总览：谁调谁 |
| 2 | `src/core/ShapeToPolyData.cpp` | 全部（225 行） | 路径 A，建立"三角汤"概念 |
| 3 | `src/core/mesher/CadSurfaceMesher.h` | `GeometrySource` 接口（第 23~54 行） | 算法能问 OCC 什么 |
| 4 | `src/core/OccCadGeometry.h` | 全部（76 行） | 适配层的字段 |
| 5 | `src/core/OccCadGeometry.cpp:122` | 构造函数（约 150 行） | ★ ② OCC 适配 |
| 6 | `CadSurfaceMesher.cpp` 搜 `MeshStage::BoundarySampling` | 采样 + 递归二分 | ③a |
| 7 | `CadSurfaceMesher.cpp:1417` 起 | 逐面主循环 | ③b |
| 8 | `Constrained2D.h` + `Delaunay2D.h` | 两个头文件 | ③b 的算法细节 |
| 9 | `SurfaceDomainState.h` | 域分类 | ③c |
| 10 | `CadSurfaceMesher.cpp` 搜 `MeshStage::Refinement` | 细化 | ③d |

**跑起来看**：

```bash
cd F:/Code/MeshStudio
./build/Release/CadSurfaceBenchmark.exe --cad test_data/ball.brep --observe on --timeout 300
# 输出会按 MeshStage 逐阶段给耗时，直接看到每个阶段花多少时间
```

`test_data/cube2.stp` 是最小的 STEP 例子（24 个顶点 → 焊接后 8 个），适合先跑它。

---

## 7. 一句话总结

> **从 B-Rep 到三角面片，本质是把"数学定义的曲面 + 裁剪环"变成"离散的三角形"，
> 而自研路径比 OCC 内置路径多做三件事：**
>
> 1. **把每条棱的采样"拉出来共享"**（而不是逐面各算各的）→ 保证相邻面不裂缝；
> 2. **在 UV 参数域做约束三角化**（而不是直接按弦高折面）→ 保证边界被精确保留；
> 3. **给每个三角形记住它来自哪个 CAD 面/棱、每个角点的 UV** → 后面体网格才能用。

相关文档：
- 《三角面片求解器-算法原理与踩坑总结.md》—— 完整原理 + 五层坑
- 《初学者理解路线-三角面片求解器.md》—— 分步学习路径
- `ARCHITECTURE.md` —— 算法权威描述
