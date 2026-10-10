# gmsh 表面三角化源码导读

> 对象：`F:\Gmesh\gmsh-4.13.1-source`（4.13.1，2024）
> 所有行号都对着这个目录，可直接跳转。
> 目的：给 MeshStudio 的 CDT 求解器找一面"工业级参照镜"。

---

## 0. 一句话总览

gmsh 的表面三角化**不是在三维上做 CDT**，而是：

```
把曲面参数化到 (u,v) 平面
  → 在 (u,v) 上做「无约束 Delaunay」
  → 再用「翻转」把约束边恢复出来
  → 用「连通传播」把域外的三角形删掉
  → 最后在 (u,v) 上加密 + 光顺，每个新点再用 gf->point(u,v) 拉回三维
```

**整个流程 90% 在一个函数里**：`src/mesh/meshGFace.cpp` 的
`static bool meshGenerator(GFace *gf, ...)`（**1396 行**）。

---

## 1. 地图：文件与职责

| 文件 | 行数 | 职责 | 要不要精读 |
|---|---|---|---|
| `mesh/meshGFace.cpp` | 3413 | **总调度**：`meshGenerator()` 六步全在这 | ★★★ 必读 |
| `mesh/meshGFaceBDS.cpp` | 972 | **MeshAdapt 引擎**：split/swap/collapse/smooth 四件套 | ★★★ |
| `mesh/meshGFaceDelaunayInsertion.cpp` | 2024 | **Delaunay/Frontal 引擎**：Bowyer–Watson | ★★ |
| `mesh/BDS.cpp` | 1573 | **网格数据结构 + 边界恢复**：`recover_edge` / `swap_edge` | ★★★ |
| `mesh/BDS.h` | — | `BDS_Point` / `BDS_Edge` / `BDS_Face` 定义 | ★★★ |
| `mesh/meshTriangulation.cpp` | ~920 | **初始三角化（新）**：`PolyMesh` + `GFaceInitialMesh` | ★★ |
| `mesh/meshPolyMesh.h/.cpp` | ~1000 | 半边结构：`split_triangle` / `swap_edge` / `Walk` | ★★ |
| `mesh/meshGFaceOptimize.cpp` | 1484 | `laplaceSmoothing` / 重组合成四边形 | ★ |
| `mesh/meshGEdge.cpp` | 913 | **1D 边界离散**：曲线取点 | ★★ |
| `mesh/meshGFaceBamg.cpp` | 247 | BAMG 各向异性重构（调外部库） | 可跳过 |
| `mesh/DivideAndConquer.cpp` | — | 老初始三角化（Guibas–Stolfi） | 可跳过 |

---

## 2. 调用链（从上到下）

```
Generator.cpp:1351  GenerateMesh(GModel *m, int ask)
  ├─ Generator.cpp:349   Mesh1D()          → meshGEdge::operator()   边界离散
  └─ Generator.cpp:489   Mesh2D()
        └─ geo/GFace.cpp:1919  GFace::mesh(bool verbose)
              └─ mesh/meshGFace.cpp:3157  meshGFace::operator()(GFace*, bool)
                    └─ mesh/meshGFace.cpp:1396  meshGenerator(...)   ★主体★
```

`meshGFace::operator()` 只做三件事（3157–3271）：

1. 选算法名（`switch(gf->getMeshingAlgo())`，3199）
2. 转发给 `meshGenerator()`（3239）
3. **失败兜底**（3256–3268）：如果 `algoSwitchOnFailure`（默认开）且
   `isMeshValid(gf)` 为假 → 清空网格 → `setMeshingAlgo(1)`（MeshAdapt）→ 递归重跑。

> 这是 gmsh 最重要的一条工程哲学：**算法会失败，所以永远要有一条保底的路。**

---

## 3. `meshGenerator()` 的六个阶段

### 阶段 A · 参数化（把三维压成二维）

```cpp
// meshGFace.cpp:1525-1539
SPoint2 param;
reparamMeshVertexOnFace(here, gf, param);          // ← 三维点 → (u,v)
BDS_Point *pp = m->add_point(count, param[0], param[1], gf);
```

关键点：**`BDS_Point` 同时存两套坐标**（`BDS.h:56-57`）：

```cpp
double X, Y, Z;   // 三维实际位置
double u, v;      // 参数域坐标 —— 所有几何判定都用这一对
```

之后任何"取中点"都是先在 `(u,v)` 里取中点，再用 `gf->point(U,V)` 拉回三维
（见 `meshGFaceBDS.cpp:462-477`）。**这一步和你们 `SurfaceChart` 的思路一致。**

### 阶段 B · 收集边界点 + 建 BDS

- 遍历所有 `GEdge->lines`，把端点塞进 `all_vertices`，同时用异或法检查边界闭合：
  `meshGFace.cpp:1444-1451`（进 `boundary` 集合，第二次出现就 erase）
- 若 `boundary.size() != 0` → 报错 "The 1D mesh seems not to be forming a closed loop"（1465）
- `BDS_Mesh *m = new BDS_Mesh;`（1520）
- 特殊快路径：点数 `< 3` 直接跳过（1500）、`== 3` 直接建一个三角形返回（1507）

### 阶段 C · 初始三角化（两条路）

```cpp
// meshGFace.cpp:1546
if(CTX::instance()->mesh.oldInitialDelaunay2D) {   // 默认 0 = 关
    ... DocRecord doc(...);  doc.MakeMeshWithPoints();   // 老：Guibas–Stolfi 分治
} else {
    PolyMesh *pm = GFaceInitialMesh(gf->tag(), 1);       // 新（默认）
}
```

**新路（默认）**在 `meshTriangulation.cpp:786 GFaceInitialMesh()`：

1. `getNodeCopies()`（688）把边界点映射到 `(u,v)`，**处理 seam 边**（同一个顶点在参数域有两份拷贝）
2. `pm->initialize_rectangle(bb...)`（804）先造一个**包含所有点的大矩形**，外面套 4 个"无穷远"点
3. 逐点插入（807–821）：
   - `Walk(f, x, y)`（395）—— 从上一个三角形走过去定位（walking point location）
   - `pm->split_triangle(-1, x, y, 0, f, delaunayEdgeCriterionPlaneIsotropic, nullptr)`
4. 判据 `delaunayEdgeCriterionPlaneIsotropic`（288）：
   ```cpp
   double result = -robustPredicates::incircle(v0->position, v1->position,
                                                v2->position, v->position);
   return (result > 0) ? 1 : 0;
   ```
   → **用的是 Shewchuk 精确谓词**（`robustPredicates`），和你们 `GeomPredicates.h` 同源
5. 恢复模型边：对每条 `GEdge` 的每个 `line` 调 `recover_edge(pm, v0, v1)`（465）
6. `Color()`（548）**洪泛染色**：`-2` = 外部、`faceTag` = 内部、`-1` = 洞
7. 再翻转 100 轮把被 `recover_edge` 打乱的 Delaunay 性修回来（879–894）

最后无论是哪条路，都把三角形塞进 BDS（`m->add_triangle`，1607 / 1652）。

### 阶段 D · 边界恢复 —— 这是最值得看的部分

```cpp
// meshGFace.cpp:1668-1698
// pass 1: 只登记，不动网格 —— 先把所有约束边记进 edgesToRecover
recoverEdge(m, gf, *ite, recoverMapInv, &edgesToRecover, &edgesNotRecovered, 1);
// pass 2: 真正恢复
if(!recoverEdge(m, gf, *ite, recoverMapInv, &edgesToRecover, &edgesNotRecovered, 2))
    { delete m; gf->meshStatistics.status = GFace::FAILED; return false; }
```

真正的算法在 `BDS.cpp:355 BDS_Mesh::recover_edge()`：

```cpp
while(1) {
  // 1. 扫描全网格，找出所有与目标边 (p1,p2) 相交的边
  if (Intersect_Edges_2d(e->p1->u, e->p1->v, e->p2->u, e->p2->v,
                         p1->u, p1->v, p2->u, p2->v, x)) {
      // ★ 如果被撞的那条边本身也是约束边 → 1D 网格自交，救不了
      if (e2r->find(EdgeToRecover(e->p1->iD, e->p2->iD, nullptr)) != e2r->end()) {
          not_recovered->insert(...); selfIntersection = true;
      }
      intersected.push_back(e);
  }
  // 2. 没有交点了 → 检查是否真的出现了这条边
  if (!intersected.size() || ix > 300) { ... }

  // 3. 逐个尝试翻转被撞的边
  while(!success && ichoice < intersected.size())
      success = swap_edge(intersected[ichoice++], BDS_SwapEdgeTestRecover());
  ix++;
}
```

**核心不变量**在 `BDS.cpp:934 swap_edge()` 第 961 行：

```cpp
if(e->g && e->g->classif_degree == 1) return false;   // ← 约束边绝不允许被翻掉
```

`classif_degree`：`0` = 顶点、`1` = 边、`2` = 面、`3` = 外部
（`BDS_GeomEntity`，`BDS.h:31`）。**这条 guard 就是整个 CDT 的地基**——
普通边随便翻，约束边一根汗毛都不动。

**恢复失败时的逃生阀**（`meshGFace.cpp:1703-1745`）—— 这一段非常值得抄：

```cpp
if(edgesNotRecovered.size() || gf->meshStatistics.refineAllEdges) {
    Msg::Info(":-( There are %d intersections in the 1D mesh (curves%s)", ...);
    if(repairSelfIntersecting1dMesh) {
        Msg::Info("8-| Splitting those edges and trying again");
        remeshUnrecoveredEdges(recoverMapInv, edgesNotRecovered);  // ← 把 1D 网格加密
        gf->meshStatistics.refineAllEdges = false;
    }
    delete m;
    if(RECUR_ITER < 10)
        return meshGenerator(gf, RECUR_ITER + 1, ...);   // ← 整条流程重来，最多 10 次
    return false;
}
```

> **失败不是终点，是"把边界点加密后重来"。**
> 你们球面那 5254 条恢复不了的约束边，gmsh 的答案是：
> **不是去改恢复算法，而是回头把 1D 网格打散重做。**

`remeshUnrecoveredEdges` 定义在 `meshGFace.cpp:631`。

### 阶段 E · 域分类（把外面的三角形删掉）

`meshGFace.cpp:1753-1817`，三步：

```cpp
CLASS_F(1, 2);          // 1408: 内部面
CLASS_EXTERIOR(1, 3);   // 1409: 外部分类

// 1. 找一个含"假点"（iD<0，即包围盒 4 个点）的三角形 → 它一定在外部
//    然后 recur_tag() 递归把连成一片的外部三角形全打上 EXTERIOR
if(n[0]->iD < 0 || n[1]->iD < 0 || n[2]->iD < 0) { recur_tag(t, &CLASS_EXTERIOR); break; }

// 2. 找一条一边 EXTERIOR、一边还没标记的边 → 从它出发洪泛标记内部
if(e->faces(0)->g == &CLASS_EXTERIOR) recur_tag(e->faces(1), &CLASS_F);
```

**为什么必须洪泛而不是逐个判重心？** 因为非凸域/带洞的曲面（你们文档里
台阶 2 提过）——重心法会误判。gmsh 用**连通性**保证不会。

之后：`if(!t->g) m->del_face(t);`（1857）删掉所有没被标记的，最后删掉 4 个假点（1879）。

### 阶段 F · 加密 + 光顺 —— 两条**完全不同**的引擎

这是最容易读岔的地方。`meshGFace.cpp:1927` 起分叉：

#### 路线 1：MeshAdapt（曲面默认）

```cpp
// meshGFace.cpp:1927
if(!algoDelaunay2D(gf) && !onlyInitialMesh) {
    refineMeshBDS(gf, *m, CTX::instance()->mesh.refineSteps, true,  &recoverMapInv, nullptr);
    refineMeshBDS(gf, *m, CTX::instance()->mesh.refineSteps, false, &recoverMapInv, nullptr);
}
```

`algoDelaunay2D()`（730）在算法为 `MeshAdapt` 时返回 **false**，所以走这条。

`meshGFaceBDS.cpp:726 refineMeshBDS()` 的主循环（788–884）是**纯局部算子迭代**，
不是逐点插点：

```cpp
while(1) {
    splitEdgePass   (gf, m, 1.4, nb_split, ...);   // 边归一化长度 > 1.4 → 劈开
    smoothVertexPass(gf, m, nb_smooth, false, .5, ...);
    swapEdgePass    (gf, m, nb_swap, ...);         // 翻边让形状更 Delaunay
    smoothVertexPass(gf, m, nb_smooth, false, .5, ...);
    collapseEdgePass(gf, m, 0.7, MAXNP, nb_collaps, ...);  // 长度 < 0.7 → 塌缩
    smoothVertexPass(gf, m, nb_smooth, false, .5, ...);
    // 收敛判据：minL > 0.7 && maxL < 1.4，或者本轮没有 split/collapse
    if((minL > MINE_ && maxL < MAXE_) || IT > abs(NIT)) break;
    if(nb_split == 0 && nb_collaps == 0) break;
}
```

各算子位置：
- `splitEdgePass` — `meshGFaceBDS.cpp:405`（先按长度排序，长的先劈）
- `swapEdgePass` — `179`
- `collapseEdgePass` — `532`
- `smoothVertexPass` — `587` → `BDS_Mesh::smooth_point_centroid`
- 尺寸场来源：`computeNodalSizes`（603）—— **直接用 1D 网格的边长当尺寸场**

最后还有一段"**修坏单元**"的循环（892–954）：反复检测 `BDS_Face_Validity <= 0.2`
的三角形，只对它们做 swap + smooth，最多 10 轮；修不好就
`gf->meshStatistics.status = GFace::FAILED`。

#### 路线 2：Delaunay / Frontal / Pack / Bamg

```cpp
// meshGFace.cpp:1919
delaunayizeBDS(gf, *m, nb_swap);        // 先把恢复后的网格翻成 Delaunay
// meshGFace.cpp:1937
BDS2GMSH(m, gf, recoverMap);            // BDS → gmsh 的 MTriangle 结构（换数据结构！）
// meshGFace.cpp:1961
if(gf->getMeshingAlgo() == ALGO_2D_FRONTAL)       bowyerWatsonFrontal(gf);
else if(... == ALGO_2D_FRONTAL_QUAD)              bowyerWatsonFrontalLayers(gf, true);
else if(... == ALGO_2D_PACK_PRLGRMS)              bowyerWatsonParallelograms(gf);
else if(... == ALGO_2D_DELAUNAY || ALGO_2D_AUTO)  bowyerWatson(gf);
else { bowyerWatson(gf, 15000); meshGFaceBamg(gf); }   // BAMG
```

`meshGFaceDelaunayInsertion.cpp:1033 bowyerWatson()` 才是教科书式 Ruppert/Shewchuk 流程：

```cpp
std::set<MTri3 *, compareTri3Ptr> AllTris;    // 按外接圆半径排序的优先队列
buildMeshGenerationDataStructures(gf, AllTris, DATA);

while(1) {
    MTri3 *worst = *AllTris.begin();          // ★ 取"最坏"的三角形
    if(worst->isDeleted()) { ...erase...; continue; }
    if(worst->getRadius() < 0.5 * std::sqrt(2.0) ||   // ★ 半径够小就停
       (int)DATA.vSizes.size() > MAXPNT) break;

    double center[2], metric[3], r2;
    circUV(worst->tri(), DATA, center, gf);            // (u,v) 下的外接圆心
    buildMetric(gf, pa, metric);                       // 度量张量（各向异性）
    circumCenterMetric(worst->tri(), metric, DATA, center, r2);
    insertAPoint(gf, AllTris.begin(), center, metric, DATA, AllTris);  // 插点
}
```

> 注意：**`bowyerWatson` 操作的是 gmsh 自己的 `MTriangle` 结构，不是 BDS**。
> 中间靠 `BDS2GMSH`（`meshGFace.cpp:1335`）搬了一次家。

两条路最后都汇合到：

```cpp
// meshGFace.cpp:1984
laplaceSmoothing(gf, CTX::instance()->mesh.nbSmoothing, infty);   // meshGFaceOptimize.cpp:982
```

### 阶段 G · 收尾

- 调试输出：`outputScalarField(m->triangles, "surface%d-*.pos", ...)`
  （1657 / 1884 / 1987）
- 边界层后处理：`modifyInitialMeshForBoundaryLayers`（1957）
- 四边形重组：`recombineIntoQuads`（2020）/ `meshGFaceQuadrangulateBipartiteLabelling`（2013）
- 质量统计：`computeElementShapes`（2024）

---

## 4. 关键数据结构

### `BDS_Point`（`BDS.h:49`）

```cpp
double X, Y, Z;                 // 三维
double u, v;                    // 参数域（所有判定都用它）
double _lcBGM, _lcPTS;          // 两套尺寸场：背景网格 vs 点传播
bool   config_modified;         // 脏标记，驱动局部更新
short  degenerated;             // 0/1/2 三种退化（seam 上的点）
BDS_Point *_periodicCounterpart;// 周期面配对
int    iD;                      // <0 表示包围盒假点
BDS_GeomEntity *g;              // 几何归类（classif_tag/classif_degree）
std::vector<BDS_Edge *> edges;  // 邻接
```

`config_modified` 这个**脏标记 + 邻域传播**的写法（`neighboringModified`，
`meshGFaceBDS.cpp:166`）是性能关键：只处理动过的区域。

### 分类三元组（`BDS.h:31`）

| `classif_degree` | 含义 | 被翻转保护 |
|---|---|---|
| 0 | 几何顶点 | 保护 |
| 1 | 几何边（**约束边**） | **保护，绝不可翻** |
| 2 | 几何面 | 可翻 |
| 3 | 外部 | 会被删 |

### `PolyMesh`（`meshPolyMesh.h:15`）

标准半边结构，只有四个方法值得看：
`initialize_rectangle` / `split_triangle` / `swap_edge` / `getEdge`，
外加 `Walk`（`meshTriangulation.cpp:395`）做点定位。

---

## 5. 建议的阅读顺序（5 个半天）

| 时段 | 读什么 | 目标 |
|---|---|---|
| 半天 1 | `GmshDefines.h:239-248`（算法编号）+ `meshGFace.cpp:3157-3271`（入口） | 说清"算法编号 → 走哪条引擎" |
| 半天 2 | `BDS.h` 全文 + `meshGFace.cpp:1396-1560` | 说清 BDS 三件套 + 参数化怎么进来 |
| 半天 3 | **`BDS.cpp:355-446`（`recover_edge`）** + `934-1060`（`swap_edge`） | 说清"为什么约束边不会被翻掉" |
| 半天 4 | `meshGFaceBDS.cpp:405-601`（四个算子）+ `726-972`（主循环） | 说清 MeshAdapt 的收敛逻辑 |
| 半天 5 | `meshGFaceDelaunayInsertion.cpp:1033-1088` + `meshTriangulation.cpp:786-900` | 说清 Bowyer–Watson 与初始三角化 |

**不要一开始读**：`meshGFaceBAMG.cpp`（外部库封装）、`DivideAndConquer.cpp`（老路径）、
`meshGFacePack.cpp`（打包成平行四边形，很特殊）、`meshGFaceOptimize.cpp`（太长且是收尾）。

---

## 6. 与 MeshStudio 的对照

| 维度 | gmsh 4.13.1 | MeshStudio 当前 |
|---|---|---|
| 工作空间 | 参数域 `(u,v)` + 度量张量补偿 | `SurfaceChart` + `J=[S_u,S_v]` 度量 |
| 初始三角化 | 包围盒 + 逐点插入（`PolyMesh`）+ 精确谓词 `incircle` | `Delaunay2D.cpp` Bowyer–Watson |
| 边界恢复 | **翻转优先**，`classif_degree==1` 的边永不可翻 | `Constrained2D::enforceConstraints` |
| 恢复失败 | **回头加密 1D 网格，整条流程重来（最多 10 次）** | 球面上剩 5254 条边恢复不了 → 卡死 |
| 域分类 | `recur_tag` 洪泛（连通性保证） | `classifyConstrainedDomain` |
| 内部点插入 | 两条路：MeshAdapt 局部算子 / Bowyer–Watson 取最坏三角形 | 单一 CDT |
| 兜底 | `algoSwitchOnFailure`：Delaunay 失败 → 自动切 MeshAdapt | 无 |
| 尺寸场 | 直接用 **1D 网格边长**（`computeNodalSizes`） | 需确认 |
| 退化处理 | `randFactor=1e-9` 微扰动（**仅老路径 + 周期路径**） | 无 |

### 最值得抄的三件事

1. **`RECUR_ITER < 10` 的重试环 + `remeshUnrecoveredEdges`**
   （`meshGFace.cpp:1703-1745`）。
   你们球面问题的根因很可能不是"恢复算法不够强"，而是**边界点太密/太规则
   导致 1D 网格在参数域自交**。gmsh 的答案是加密后重来，而不是硬扛。

2. **`swap_edge` 里的 `classif_degree == 1` 早退**（`BDS.cpp:961`）。
   一个 O(1) 的判断就代替了整套"保护约束边"的复杂逻辑——
   因为**约束边压根不会被放进可翻候选集**。

3. **兜底算法切换**（`meshGFace.cpp:3256-3268`）。
   与其把一个算法调稳，不如承认它会失败，并留一条能出网格的路。
   你们的 `cube10.stl` minSICN=0 的 sliver，正是这条路的用武之地。

---

## 7. 动手实验

### 打开调试输出

```bash
# gmsh 内部有 debugSurface，指定面 tag 后会往当前目录吐 .pos 文件
gmsh -2 -v 99 -setnumber Mesh.DebugSurface 1 your_model.step
# 会在当前目录生成：
#   surface1-boundary-real.pos    阶段B 的边界
#   surface1-initial-param.pos    阶段C 的初始三角化（参数域）
#   surface1-recovered-param.pos  阶段D 恢复后
#   param1.pos / real1.pos        阶段F 完成后
```

在 ParaView / gmsh 里打开这些 `.pos`，**一帧一帧看网格是怎么长出来的**——
比读代码快十倍。

### 关掉兜底，看真实失败点

```bash
gmsh -2 -v 99 -setnumber Mesh.AlgorithmSwitchOnFailure 0 your_model.step
```

### 强制用老初始三角化（对比两条路）

```bash
gmsh -2 -setnumber Mesh.OldInitialDelaunay2D 1 your_model.step
```

### 加大随机扰动（专治共圆/共球点集）

```bash
# 默认 1e-9；球面这种共球点集可以试 1e-6
gmsh -2 -setnumber Mesh.RandomFactor 1e-6 ball.brep
```

### 强制 MeshAdapt（球面推荐先试这个）

```bash
gmsh -2 -setnumber Mesh.Algorithm 1 ball.brep     # 1 = MeshAdapt
gmsh -2 -setnumber Mesh.Algorithm 5 ball.brep     # 5 = Delaunay（对照）
```

### 想打断点

在 `F:\Gmesh\gmsh-4.13.1-build` 用 VS 打开 `gmsh.sln`（4.11.1 已配好），
在这几个位置下断点最有效率：

| 断点 | 看什么 |
|---|---|
| `meshGFace.cpp:1674` | 约束边有多少条 |
| `meshGFace.cpp:1703` | `edgesNotRecovered.size()` 是不是 0 |
| `BDS.cpp:411` | 边恢复是不是卡在 `ix > 300` |
| `BDS.cpp:961` | 有多少次翻转被约束边 guard 挡掉 |
| `meshGFaceBDS.cpp:874` | MeshAdapt 的收敛：`minL` / `maxL` |

---

## 8. 相关选项速查

| 选项 | 默认 | 位置 | 说明 |
|---|---|---|---|
| `Mesh.Algorithm` | 2 (Auto) | `GmshDefines.h:239` | 1=MeshAdapt 5=Delaunay 6=Frontal 7=BAMG 8=Frontal+Quad 9=Packing |
| `Mesh.AlgorithmSwitchOnFailure` | **1** | `DefaultOptions.h:1129` | 失败自动切 MeshAdapt |
| `Mesh.RandomFactor` | **1e-9** | `DefaultOptions.h:1518` | 微扰动，破退化 |
| `Mesh.RandomFactor3D` | 1e-12 | `DefaultOptions.h:1521` | 同上，三维 |
| `Mesh.OldInitialDelaunay2D` | **0** | `DefaultOptions.h:1421` | 0 = 用新 `PolyMesh` |
| `Mesh.RefineSteps` | — | — | MeshAdapt 迭代步数 |

`Auto` 的语义（`meshGFace.cpp:3202`）：**平面走 Delaunay，曲面走 MeshAdapt**。

---

## 附：一张图

```
                    GEdge::mesh()  ── 1D 边界离散（按尺寸场取点）
                            │
                            ▼
   ┌──────────────── meshGenerator() ────────────────┐
   │                                                  │
   │  A 参数化   reparamMeshVertexOnFace → (u,v)      │
   │  B 建 BDS   + 检查边界闭合                        │
   │  C 初始三角化  PolyMesh(包围盒+逐点插入)           │
   │                或 DocRecord(分治, 老)             │
   │  D 边界恢复  recover_edge = 扫交点 → swap_edge    │
   │             ★ classif_degree==1 的边不可翻        │
   │             失败 → 加密1D → 递归重来(≤10次)        │
   │  E 域分类    recur_tag 洪泛 → 删外部三角形         │
   │                                                  │
   │  F 分叉 ─┬─ MeshAdapt: split/swap/collapse/smooth │
   │          └─ Delaunay : delaunayizeBDS → BDS2GMSH  │
   │                       → bowyerWatson(最坏三角形)  │
   │          └────────────────┬───────────────────────┤
   │                           ▼                       │
   │                     laplaceSmoothing              │
   └──────────────────────────────────────────────────┘
                            │
                            ▼
              gf->point(u,v) 把每个点拉回三维
                            │
                 失败? → 切 MeshAdapt 重跑
```
