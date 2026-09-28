# MeshStudio 四面体网格生成设计方案

> 状态：定稿待实施（对应 README Phase 2/3/4）
> 日期：2026-09-12
> 依据：开源库调研（TetGen/Gmsh/CGAL/Netgen/TetWild 源码与官方文档）、商用 FEM 软件流程调研（ANSYS/Abaqus/COMSOL/STAR-CCM+）、四面体网格文献综述

---

## 1. 目标与定位

为 FEM 提供从 CAD 到四面体网格的全自动生成能力，同时保留用户对工程决策的控制权。

- **架构定位**：延续 README 的"薄 UI + 厚引擎"——引擎采用 **Gmsh 库模式**（已配齐），UI 只负责参数收集、流程编排、结果展示。
- **差异化分工**：Gmsh 包掉"算法活"（表面网格化、Delaunay 四面体化、边界恢复、质量优化）；MeshStudio 做三件 Gmsh 不包的事：
  1. **几何校验/修复入口**（生成前拦截坏输入）
  2. **边界组（物理组）可视化管理**（FEM 施加约束/载荷的必需品，开源生态公认的痛点）
  3. **质量验收报告 + 诊断联动**（对齐目标求解器阈值）

### 选型决策记录

| 候选 | 结论 | 理由 |
|---|---|---|
| **Gmsh 库（选定）** | ✅ | 一条龙（STEP→表面网格→体网格）、开源 FEM 生态主流做法（FEniCS/FreeFEM/Elmer/CalculiX/FreeCAD 全是调 Gmsh/TetGen/Netgen）、默认 Delaunay 内部即 TetGen/BR 边界恢复、参数选项体系成熟、项目已配齐 |
| TetGen | 备弃 | 单文件好集成、质量理论最强，但只做体网格（需自备表面网格），AGPL，对脏输入脆弱（Thingi10k 成功率 49.5%） |
| Netgen | 备选 | LGPL 更宽松、混合路线稳，但集成成本高一档，功能与 Gmsh 重叠 |
| CGAL / TetWild | 不采用 | 编译/依赖重；TetWild 表面是 ε 近似，不适合 FEM 精确边界 |

**许可**：Gmsh 为 GPL，链接后本软件分发物随 GPL —— 须补 LICENSE 文件（见 §10）。

---

## 2. 总体架构

> ⚠️ **本节为历史设计（Gmsh 方案）**。实际实现已把 Gmsh 换成自研算法库
> `mesher/`（见 §11），`core/TetMeshWorker` 改为 `QtConcurrent` 后台任务，
> 依赖里没有 `gmsh.lib`。保留本节用于对照算法选型与架构分层。

```
MeshStudio (Qt)
├── MeshSettingsPanel        参数面板（分组卡片，§5）
├── ModelTreePanel           模型树 + 边界组管理（§7）
├── OutputDock               日志 / 质量报告文本
├── ViewportWidget           几何 + 网格叠加显示（现成）
│
├── core/TetMeshEngine       [新增] Gmsh 封装：参数映射、导入、生成、转换、质量统计
├── core/TetMeshWorker       [新增] QThread 工作对象：串行执行整个 Gmsh 会话
├── core/MeshQualityChecker  [新增] 质量指标计算与阈值判定（含二面角自算）
└── core/GeometryValidator   [新增] 生成前校验（水密/朝向/自交）+ HealShapes 入口
```

依赖：现有 Qt 5.14.2 / VTK 9.3.1 / OCC 7.6.0 不变；新增链接 `gmsh.lib`（SDK 模式）。

---

## 3. 流水线与分工（六步）

```
① CAD 导入 ─────── Gmsh: occ::importShapes()          STEP/IGES/BRep 直接喂
② 几何校验/修复 ── ★MeshStudio: 体积/朝向检查 + healShapes 入口（§8）
③ 特征与尺寸场 ─── Gmsh: 特征线自动 + 曲率加密；MeshStudio 给参数
④ 表面网格 ─────── Gmsh: mesh::generate(2)
⑤ 体四面体 ─────── Gmsh: mesh::generate(3)（内部 = Delaunay + TetGen/BR 边界恢复 + 优化）
⑥ 质量验收/导出 ── ★MeshStudio: 质量统计 + 阈值判定 + 诊断联动 + 导出（§6）
```

前置条件（必须满足，否则 Gmsh/TetGen 类算法失败或产出空网格）：
表面闭合（水密）、无自相交、法向一致、无退化微小拓扑。这些由 ② 拦截，不由用户踩坑。

---

## 4. 核心调用链与线程模型

```cpp
// TetMeshWorker::run() —— 所有 gmsh:: 调用必须在本线程内串行完成
gmsh::initialize();                       // 与 finalize() 严格配对，每次生成一个会话
gmsh::option::setNumber("General.Terminal", 0);
gmsh::logger::start();                    // 日志重定向 → OutputDock

// —— 参数映射（§5 面板 → 选项）——
gmsh::option::setNumber("Mesh.MeshSizeMax", p.targetSize);
gmsh::option::setNumber("Mesh.MeshSizeMin", p.minSize);
gmsh::option::setNumber("Mesh.MeshSizeFromCurvature", 360.0 / p.curvatureAngle);
gmsh::option::setNumber("Mesh.ElementOrder", p.elementOrder);      // 1 或 2
gmsh::option::setNumber("Mesh.Algorithm3D", p.algo3D);             // 1=Delaunay, 10=HXT
gmsh::option::setNumber("Mesh.Optimize", p.optimize ? 1 : 0);
gmsh::option::setNumber("Mesh.OptimizeThreshold", p.optimizeThreshold);

// —— 几何 ——
gmsh::model::add("tet");
std::vector<std::pair<int,int>> errors;
gmsh::model::occ::importShapes(p.filePath, true, "", errors);
gmsh::model::occ::synchronize();
// [可选修复] gmsh::model::occ::healShapes(dimTags, tol, true,true,true,true,true);

// —— 边界组（§7，生成前重放用户定义）——
for (auto& g : p.boundaryGroups)
    gmsh::model::addPhysicalGroup(2, g.surfaceTags, g.tag); // + setPhysicalName

// —— 生成 ——
gmsh::model::mesh::generate(2);
gmsh::model::mesh::generate(3);

// —— 提取 → VTK（§6.3）—— 质量统计（§6.1）
gmsh::finalize();
```

**线程与进度约定**：
- `gmsh::initialize/finalize` 与全部 API 调用放在**同一个 worker 线程**串行执行（Gmsh API 非线程安全，禁止跨线程并发调用）。
- 进度：`gmsh::setCallback` 收到进度类消息后经信号转发主线程更新进度条（现有 `setGenerating()` 流程复用）。
- 取消：第一版不做中途取消（Gmsh 无安全取消点），"生成中"按钮置灰即可；后续可用异常 + 会话丢弃兜底。

---

## 5. 参数面板方案（分组卡片 + 表单行，对齐现有主题）

替换现有 MeshSettingsPanel 占位结构，分组与控件如下：

### 全局设置（现有卡片改造）
| 参数 | 控件 | 默认值 | Gmsh 选项 |
|---|---|---|---|
| 目标网格尺寸 | QDoubleSpinBox | 包围盒对角线×0.025（打开模型时自动填） | `Mesh.MeshSizeMax` |
| 最小网格尺寸 | QDoubleSpinBox | 目标尺寸×0.1 | `Mesh.MeshSizeMin` |
| 曲率细分角度 | QDoubleSpinBox（°） | 30 | `Mesh.MeshSizeFromCurvature` = 360/角度（每 2π 单元数） |
| 单元阶次 | QComboBox | 一阶（tooltip："应力分析推荐二阶 T10"） | `Mesh.ElementOrder`（1/2） |

### 质量与算法（新增卡片）
| 参数 | 控件 | 默认值 | Gmsh 选项 |
|---|---|---|---|
| 3D 算法 | QComboBox | Delaunay | `Mesh.Algorithm3D`（1=Delaunay，10=HXT 并行） |
| 生成后优化 | QCheckBox | 开 | `Mesh.Optimize` |
| 优化阈值 | QDoubleSpinBox | 0.3 | `Mesh.OptimizeThreshold` |

### 局部加密（沿用现有区域表格）
每行 = { 几何面/体, 尺寸 } → 生成前对所选实体调 `gmsh::model::mesh::setSize(dimTags, size)`。

### 共形设置（沿用）
薄片处理、判断容差。注：Gmsh 无顶层"渐变率"选项，第一版不暴露该参数（内部已有控制），后续如需可用背景尺寸场 Field 实现。

### 输出（新增卡片）
| 参数 | 控件 | 选项 |
|---|---|---|
| 导出格式 | QComboBox | VTK .vtu（默认）/ Abaqus .inp / Gmsh .msh |

> 说明：半径-边比、最小二面角是 TetGen/CGAL 层参数，Gmsh 未直接暴露，不进面板（避免无效 UI）；Gmsh 内部按 ρ=2.0 判据细分且默认优化。

---

## 6. 质量验收模块（MeshQualityChecker）

### 6.1 指标与角色

| 指标 | 获取方式 | 角色 | 判定 |
|---|---|---|---|
| **minSICN** | `gmsh::model::mesh::getElementQualities(tags, "minSICN")` | 哨兵 | < 0 → 翻转单元，**禁止导出**，报错并列出单元 |
| **minDetJac** | 同上（"minDetJac"） | 哨兵（与 SICN 互备） | < 0 → 翻转 |
| **gamma** = 3r/R | 同上（"gamma"），正四面体 = 1 | 主指标 | 全网格 min/avg + 分布直方图；< 0.3 计入"待优化" |
| **最小/最大二面角** | 自算（四对面法向夹角，Gmsh 不直接提供） | sliver 复核 | < 10° 或 > 170° 告警 |

底层依据：radius-edge 比（ρ≤2.0）抓整体形状但**抓不住 sliver**（薄片单元 ρ 可低至 0.707 却二面角≈0），必须尺寸比 + 角度双指标搭配；Jacobian 类指标（SICN/条件数）与插值误差、刚度矩阵条件数直接相关，且是唯一能可靠检出**翻转单元**的指标（求解器硬性要求正 Jacobian、朝向一致）。

### 6.2 阈值预设（对齐目标求解器）
- 通用底线：minSICN ≥ 0；min 二面角 ≥ 10°。
- Abaqus 预设：面角 5°~170°，aspect ratio < 10，shape factor > 0.0001。
- FLUENT 预设（导出 CFD 用途时提示）：skewness 最大 < 0.95、平均 < 0.33。
- 阈值随导出格式联动（选 .inp 时自动套 Abaqus 预设）。

### 6.3 数据转换与诊断联动
- 节点：`getNodes`（nodeTags 不连续，建 tag→index 映射）→ `vtkPoints`。
- 单元：`getElementsByType(4)`=TET4 → `VTK_TETRA`；阶次 2 时 `getElementsByType(11)`=TET10 → `VTK_QUADRATIC_TETRAHEDRON`。插入前按体积符号检查朝向，翻转发给 checker。
- 边界面：`getElementsByType(2 或 9)` + 物理组归属 → cell scalar（边界组 id），供视口按组着色/选择。
- 质量着色：gamma/SICN 写入 cell data 数组 → VTK scalar 着色（Phase 3）。
- 诊断面板（Phase 4）：最差 N 个单元列表 → `vtkExtractCells` 单独显示 + 双击定位。

### 6.4 报告（写入 OutputDock + 随导出落盘）
节点数 / 单元数（T4·T10 分列）/ min·avg gamma / minSICN / 翻转单元数 / 最差二面角 / 各阈值通过与否。

---

## 7. 边界组（物理组）管理

- **UI**：ModelTreePanel 面节点 → 右键"添加到边界组"→ 命名（如 `fixed`、`pressure`）。组列表存于工程文件。
- **实现**：生成前重放 `addPhysicalGroup(2, surfaceTags, tag)` + `setPhysicalName`；Gmsh 导出 .msh/.inp 时自动携带（.inp → *NSET/*ELSET）。
- **引用稳定性**：surface tag 依赖导入顺序，组记录同时保存几何指纹（面数 + 总面积 + 包围盒），生成前校验失配则提示用户重选。

---

## 8. 几何校验与修复（GeometryValidator）

生成前自动执行：
1. `occ::getMass(3, tag)`：体积 ≤ 0 → 疑似不闭合/朝向反，阻断并提示；
2. 表面自交 / 微小拓扑：Gmsh 会在生成时报错，日志捕获后引导用户走修复；
3. 修复按钮 → `occ::healShapes(dimTags, tol, fixDegenerated=true, fixSmallEdges=true, fixSmallFaces=true, sewFaces=true, makeSolids=true)` + 重新 synchronize；
4. STL 输入（离散几何）Phase 2 先不支持体网格（README 的 STL 走显示快照即可），Phase 3 经 `gmsh::merge + createGeometry` 路线补上。

---

## 9. 里程碑

| 阶段 | 内容 | 验收 |
|---|---|---|
| **M1（Phase 2a）** | gmsh 链接 CMake；TetMeshWorker 跑通 importShapes→generate(3)→vtkUnstructuredGrid→视口显示（默认参数） | 任意 STEP 出四面体并显示 |
| **M2（Phase 2b）** | 参数面板全量替换 + 选项映射 + 后台线程/进度 | 改参数结果可复现地变化 |
| **M3（Phase 2c）** | 质量报告（SICN/gamma/二面角）+ 边界组管理 | 报告数值正确、组随导出携带 |
| **M4（Phase 3）** | 导出 .vtu/.inp/.msh + 质量着色 | Abaqus 可读 .inp 且含 *NSET |
| **M5（Phase 4）** | 诊断面板（坏单元列表联动视口）+ healShapes 修复入口 | 翻转单元可定位 |
| **随行** | LICENSE 文件（GPL 声明） | M1 前完成 |

---

## 10. 风险与待决事项

1. **LICENSE**：项目当前无 LICENSE 文件。链接 Gmsh（GPL）后分发物随 GPL，须在首次发布前明确声明（若未来考虑宽松许可，引擎需换回 Netgen/LGPL，接口已按 TetMeshEngine 抽象隔离，可替换）。
2. **OCC 双通道**：MeshStudio 用 OCC 7.6.0 读几何显示，Gmsh SDK 内嵌自己的 OCC——预编译 SDK 符号隔离，无冲突；若改为源码编译 Gmsh，需统一 OCC 版本。
3. **Gmsh 版本**：本文 API 以 Gmsh 4.x 为准，实施时以所配 SDK 头文件签名核对。
4. **大模型内存**：Tet10 网格节点数约为 T4 的 6 倍量级，视口显示默认提供"仅显示外表面"选项（提取边界面渲染）。
5. **失败反馈**：Gmsh 生成失败（自交/不水密）的错误必须经 logger 如实报回 OutputDock，禁止静默空网格。

---

## 11. 输入几何：三角面片到底从哪来（2026-09-27 补充，实施后回填）

> ⚠️ 本节记录**实际实现**，与 §2/§3/§4 中的 Gmsh 方案不同：Gmsh 依赖已从
> CMake 移除，引擎换成自研算法库 `mesher/`（Delaunay → CDT → Ruppert → 平滑）。
> §3/§4 保留作为历史设计与算法选型对照。

### 11.1 问题

四面体网格引擎的输入是**三角形汤**（顶点 + 三角形索引，允许重复顶点、无拓扑）。
但输入格式分两类，性质完全不同：

| 格式 | 本质 | 是否自带三角面片 |
|---|---|---|
| STL | 三角网格（离散表示） | ✅ 自带 |
| **STEP / IGES / BRep** | **边界表示（B-Rep）**：面由 NURBS/解析曲面定义、棱由曲线定义 | ❌ **完全没有** |

因此 STEP/IGES/BRep **必须先做离散化（tessellation）** 把曲面折成三角片，
否则引擎拿不到任何可用的输入。这一步不是可选项，也不是"读文件时顺手做的"。

### 11.2 实施后的调用链

```
STL ─────────────── vtkSTLReader ─────────────────────────┐
                                                          ▼
STEP/IGES/BRep ─ ShapeLoader::load ─► TopoDS_Shape ─► TetMeshEngine::run(params, verts, tris)
                    (OCC 读文件)          │                        ▲
                                          ▼                        │
                       ShapeTessellator::tessellateFile            │
                        └─ ShapeToPolyData::deflectionFor          │
                             （弦高随 targetSize 收紧）             │
                        └─ ShapeToPolyData::tessellate             │
                             （BRepMesh_IncrementalMesh 逐面离散） ─┘
```

分层原则：**算法库 `mesher/` 与 `TetMeshEngine` 不依赖 OCC**。
离散化在 `ShapeTessellator`（依赖 OCC）里完成，由调用方
（`MainWindow::generateMesh` / `BRepInputTest`）把三角汤传进引擎。
这样只链接 VTK 的 `TetMeshEngineTest` 仍能直接测引擎本体。

### 11.3 两个必须处理的技术细节

**(a) 离散精度必须跟随目标尺寸** —— 原先 `ShapeToPolyData` 固定用
`diag · 1e-3` 作弦高，与 `targetSize` 无关。粗四面体挂在低精度折线上会
明显失真。现在 `deflectionFor(shape, targetSize)` 取
`min(diag·1e-3, targetSize·0.25)`：**默认设置下与从前完全一致（不退步）**，
用户把目标尺寸调细时弦高同步收紧。

**(b) 逐面离散的共享棱必须缝合** —— `BRep_Tool::Triangulation` 是**逐面**的，
每个面有独立的 `TopLoc_Location` 与顶点数组，相邻面在共享棱上的顶点
**只是近似相等，不保证 bit 相同**。若不缝合，一条棱会被 4 个面共享，
`isWatertight` 直接判不水密，STEP 路径根本走不通。

缝合由 `mesher::weldSurfaceMesh` 完成。本日修复了它的两个**真实缺陷**：

| 缺陷 | 症状 | 修法 |
|---|---|---|
| 只查精确格子、不探邻格 | 距离 < tol 但被格边界分开的点**静默不合并** | 扫 3×3×3 邻域 + 真实欧氏距离判据 |
| 坐标量化成 21 位整数（偏移 1048576） | 有效范围仅 `±1048575·tol`；引擎 tol = `diag·1e-7` → 范围仅 `±0.105·diag`，**大坐标全部被钳到同一格子**；负坐标被 clamp 到 0 | 改用 `floor(v/tol)` 的 64 位格子坐标，无偏移、无钳制 |

第二个缺陷的实际影响：`test_data/cube2.stp` 的坐标是
`x,y ∈ [500,1500]`、`z ∈ [2000,3000]`（diag 1732），旧实现会把 24 个顶点
全部塌成 1 个；`test_data/ball.brep` 的坐标含 `[-11.93, 11.93]` 负半轴，
旧实现同样塌陷。修复后两者都能正确焊接（cube2.stp: 24 → 8 点；
ball.brep: 4410 → 3850 点、合并 560 个重复点、剔除 3 个退化三角形），
且 `isWatertight` 通过。

### 11.4 新增回归

| 测试 | 覆盖 |
|---|---|
| `SurfaceMesh3DTest` 用例 4 | 近似重复点**跨格子边界**必须被合并（旧实现漏合并） |
| `SurfaceMesh3DTest` 用例 5 | 大坐标偏移 + 负坐标（旧实现塌成 1 个顶点） |
| `BRepInputTest --with-engine cube2.stp` | STEP → 离散 → 焊接 → **四面体全链路**（0.5 s） |
| `BRepInputTest ball.brep` | 曲面 + 负坐标的 B-Rep 的**离散 + 焊接**（球面 CDT 是分钟级，故不带 `--with-engine`） |

合计 `ctest -C Release` = **12/12 通过，约 12.5 s**。

### 11.5 顺带修掉的两个算法层缺陷（球面输入暴露）

`ball.brep` 是第一个"多边界点 + 曲面 + 负坐标"的真实输入，把两个此前
**从未被观测到**的缺陷逼了出来：

**(a) `delaunay3D` 的每次插入全网格自检 —— O(n·T) 二次方**

`Delaunay3D.cpp` 的插入循环里，每插一个点就调一次 `mesh.checkFaceHash()`
（遍历 4T 个面做哈希查找）。它只用于**定位**"第几步把拓扑弄坏了"，
却让整体复杂度退化成 O(n·T)：

| 输入 | 修复前 | 修复后 |
|---|---|---|
| `ball.brep`（3850 点） | **>135 s 仍未完成** | **0.82 s**（后含剪枝/校验共 6.7 s） |
| `Delaunay3DTest` | 3.89 s | **0.30 s** |

修法：改成**诊断开关**（默认关）—— `delaunay3D(mesh, pts, verifyEachStep)`
或环境变量 `MESHSTUDIO_DELAUNAY_VERIFY_EACH_STEP=1`。正确性兜底不变：
插入结束后的 `mesh.validate()` 始终执行。

**(b) `validate()` 与 `pruneDegenerateTets()` 的判据不带误差界 —— 谓词不一致**

构造阶段（`insertPoint3D`）在**归一化坐标**（量级 ~1）下用 `orient3dSoS` 定向，
而 `validate()` 在**还原后的原始坐标**下用朴素 `orient3dDet` 判符号。
行列式随缩放按 s³ 变化，对近退化薄片（sliver）其符号完全由浮点误差支配，
两个坐标系下可以给出**相反符号** → 假阳性 `tet not right-handed`。

实测证据（ball.brep，21824 个四面体）：

```
行列式分布: 正 21823 / 零 0 / 负 1
负者 |det| = 3.19e-12 ；全体 |det| 均值 2.275、最大 5.175e+02
→ 相对量级 6.2e-15，纯噪声，不是真翻转
```

同类问题：`pruneDegenerateTets` 的退化阈值是**绝对常数** `1e-12`。
单位立方体（|det| ~ O(1)）合理，但 `cube2.stp`（坐标 500~3000，|det| ~ O(1e9)）
形同虚设，而 ball 的那个噪声薄片恰好卡在 1e-12 之上被漏检。

修法：两处都改用尺度相对的 `orient3dEps(coordMag4(...))`（= 64·2⁻⁵²·s³），
与 `orient3dSoS` 内部判定退化用的是同一个界。

**修复后**：`delaunay3D` 对 ball 返回 `ok=1, tets=21822`（噪声薄片被正确剪除）；
STL golden baseline **完全未变**（cube 10/11/20、cube10 103/405/124）。

### 11.6 遗留

- **`enforceSurface3D`（CDT）在球面上是分钟级**：7692 个约束面、21822 个初始
  四面体，实测 >100 s 未完成。这是当前引擎最重的算法瓶颈（约束边恢复的
  实现是"逐边 + 局部翻转/插点"，缺空间索引与局部化）。
- **凸且近似共球的点集本质退化**：球面离散点全部落在凸包上且共球，
  Delaunay 剖分不唯一、腔体判定处处平局。这类输入需要**先插内部 Steiner 点**
  或改用"边界优先"的构造顺序，而不是"先 Delaunay 全部表面点再恢复约束面"。
- **曲面离散仍是"折面近似"**：引擎不做真实曲面投影（almost Delaunay 路线的
  固有取舍），离散误差直接等于几何误差。要根治需要曲面投影步。
- `minSICN = 0`（sliver）与 `refineBoundary` 打开后失败，仍受 almost Delaunay
  路线限制，见 `docs/references/README-shewchuk-failure-analysis.md`。
