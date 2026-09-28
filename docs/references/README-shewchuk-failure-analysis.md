# 保护球（Edge Protection）在本项目失败的文献依据

> 结论：本项目的失败**不是实现 bug，而是路线选择与前提条件不适配**。
> 依据：Shewchuk, *Constrained Delaunay Tetrahedralizations and Provably Good
> Boundary Recovery*, 全文见同目录 `shewchuk-cdt-boundary-recovery.pdf`
> （提取文本 `_shewchuk_clean.txt`）。页码为论文页码。

---

## 1. 保护球的真正定位：保证「CDT 存在性」，不是「恢复断边」

论文 **§3 Edge Protection**（p.5–6）开篇即给出定理：

> **Theorem 1** ([20]) *If X is **edge-protected**, then X has a CDT.*
>
> *A PLC X is **edge-protected** if every segment in X is **strongly Delaunay**.*

论文 §5 的两阶段算法（p.7，Figure 11）：
1. **保护球**：对每个「≥2 条段以 <90° 相交」的顶点 v_i 放球 S_i，
   对每条相邻段插入点 `s ∩ S_i`。目的：把段的「端」(ends) 切出来。
2. **中点二分**：对仍非强 Delaunay 的段（非 ends）插中点，
   直到**所有子段强 Delaunay** → PLC 达成 edge-protected → 算法终止。

**关键**：这套算法解决的是「**让 PLC 满足 Theorem 1 的前提**」，
即"CDT 存在性"的**预处理**。它**不是**"把断边变成网格边"的手段。
本项目把保护球误当作后者使用 —— 这是**第一处根本性错配**。

---

## 2. 本项目走的是论文明确警告的 "almost Delaunay" 路线

论文 §1（p.2）把三条路线并列，并**直接给出本项目的失败预言**：

> *The second approach, and the most common one in the engineering literature,
> might be called the **almost Delaunay** approach. Missing domain boundaries are
> recovered by inserting additional vertices where the boundaries intersect the
> faces or edges of the tetrahedralization. However, **the Delaunay property is
> not maintained during these vertex insertions (at least not fully)** ...
> The deviations occur at the boundaries—where obtaining high-quality elements is
> most difficult. **One consequence is that provably good Delaunay refinement
> procedures cannot be guaranteed to work correctly.** "Almost Delaunay" mesh
> generation programs do work well in many circumstances, but they are unlikely
> to be able to control the quality of the elements in circumstances like Figure 1.
> Furthermore, they are also **in danger of creating very short edges**, or
> inserting many more vertices than necessary, unless they are carefully designed.*

**本项目 `insertConstrainedPoint` 正是 almost Delaunay 路线**
（保护约束面不跨越 = 主动偏离 Delaunay 性）。
论文明确列出其推论：
- ✗ 可证明良好的细化**不能保证正确工作**
- ✗ **有产生过短边的危险**（← 对应 cube10 的 `minSICN=0`）
- ✗ 可能插入远超必要数量的顶点（← 对应保护球 36 点纯浪费）

`cavity volume not conserved`（(49,115)/(49,135)）不是偶然失手，
是这条路线的**理论宿命**。

---

## 3. ★ 论文原文指出：立方体用例下保护球【一个点都不该插】★

论文 §3（p.6）引入 **grazeable** 概念：

> *A segment is **grazeable** if two consecutive facets in the rotary order are
> separated by an interior angle of **180° or more**, or if the segment is
> included in fewer than two facets.*
>
> *Only the grazeable segments need to be strongly Delaunay to guarantee a CDT.
> A 3D PLC X is **weakly edge-protected** if each grazeable segment in X is
> strongly Delaunay. If X is weakly edge-protected, then X has a CDT.*
>
> *Segments that are not grazeable occur commonly in practice.
> **For instance, in a regular complex of cubical cells, no segment is
> grazeable.** The stronger result **excludes the non-grazeable segments from
> the need to be strongly Delaunay**.*

**`cube.stl` / `cube10.stl` 正是 "a regular complex of cubical cells"。**
几何核对：立方体每条棱处只有 2 个面，旋转顺序下的内角 = **90° < 180°**
→ **非 grazeable**。

而论文 §5（p.7）说 weak edge protection 的做法是：

> *the procedure considers **only the grazeable segments**, and **ignores the
> others as if they did not exist**.*

**本项目的 grazeable 集合为空 → 保护球本应插入 0 个点。**
实测 `candidates=36` 全部是多余的，且正是它们把网格搞坏。

---

## 4. 第二处独立诱因：五顶点共球

论文 §6（p.8）：

> *There are two known algorithms that can construct the CDT of any PLC X that
> has a CDT, including any weakly edge-protected PLC—**if no five vertices of X
> are cospherical**.*

论文 §9（p.9）进一步强调：

> *CDT construction algorithms are even **more sensitive to numerical error**
> than most geometric algorithms. Furthermore, PLCs in which **five or more
> vertices lie on a common sphere** can cause gift wrapping to fail if
> degeneracies are not handled carefully.*
>
> *Therefore, I recommend that **exact arithmetic** be used to implement the
> numerical predicates (orientation and insphere tests), and that all the
> algorithms use a modified insphere test that employs a simple **symbolic
> perturbation** technique to simulate the circumstance where no five vertices
> are cospherical.*

立方体（8 顶点）上「五顶点共球」极易发生（如 2×2×1 的四个角 + 任一其他点）。
本项目虽有 `orient3dSoS`，但 `insphereDet` 用的仍是带容差的浮点判据
（`insphereDet(...) > -1e-6`），**未做 SoS 扰动** —— 这与论文 §9 的要求不符。

---

## 5. 论文给出的正确行为：失败要「退回」，并用「段中点分裂」替代

论文 §8（p.8–9）—— 本条最值得实现：

> *[a Delaunay] **mesh generation program should test for this possibility, and
> back out when vertex insertion cannot succeed.***
>
> *A 3D Delaunay refinement algorithm ... handles this difficulty by considering
> the **diametral sphere** of each segment ... to be a **protected region**. Any
> effort to insert a vertex on or inside the sphere is **thwarted**; instead,
> **the segment is split at its center**, and the two resulting subsegments have
> smaller diametral spheres. Thus, every subsegment is kept strongly Delaunay at
> all times, and **the mathematical integrity of the CDT is never broached**.*

本项目当前是「插入失败 → 直接报死返回 false」。
论文要求「**插入失败 → 退回（back out）**」，并优先用「**段中点分裂**」达成目标。

---

## 6. 结论与本项目应采取的行动

| # | 偏差 | 论文依据 | 后果 | 状态 |
|---|---|---|---|---|
| 1 | 把「保证 CDT 存在性的预处理」当成「恢复断边的手段」 | §3 Thm 1 | 机制定位错误 | 认知已修正 |
| 2 | 走 almost Delaunay 路线 | §1 p.2 | 细化不可保证正确、易产短边 | 已加护栏（见 §8） |
| 3 | 对非 grazeable 段（立方体）也插保护球 | §3 p.6 | 36 个点全是浪费，反而破坏网格 | **✅ 已修复** |
| 4 | `insphereDet` 未做符号扰动 | §9 p.9 | 五顶点共球时判据失效 | **✅ 已修复** |
| 5 | 插入失败直接报死 | §8 p.8–9 | 应"退回"+改用段中点分裂 | **✅ 已实现** |

**行动建议（按性价比排序）**：
1. ~~**给保护球加 grazeable 过滤**~~ **✅ 已完成**（`Constrained3D.cpp` 的
   `isGrazeable` lambda）——只对「旋转顺序内角 ≥180° 或所在面数 <2」的段端点放球。
   立方体用例下 `segments=18 grazeable=0` → 保护球自动跳过 → 不再破坏网格。
2. ~~**把 `insphereDet` 换成 SoS 版本**~~ **✅ 已完成**
   （`Constrained3D.cpp` / `Flips3D.cpp` 均改用 `insphereSoS`）。
3. ~~**`insertConstrainedPoint` 失败改为「退回 + 段中点分裂」**~~ **✅ 已完成**
   （两处调用点：保护球点失败→改用段中点；细分中点失败→移出待处理列表继续）。
4. （大改造，超出「框架清晰 + 功能不退步」范围）
   实现严格增量 Delaunay 维护 + 真正的 CDT 构造（gift wrapping / sweep），
   彻底脱离 almost Delaunay 路线。

> 论文原文（值得贴墙，p.7）：
> **"if boundary recovery creates too-small tetrahedra, it might not be possible
> to fix them."**

---

## 7. 实施后的实测验证（2026-09-24）

### 7.1 grazeable 过滤：决定性验证

| | segments | grazeable | candidates | cube.stl 结果 |
|---|---|---|---|---|
| 修复前 | 18 | **18** | 36 | **FAIL** — `(49,115)`/`(49,135)` 体积不守恒 |
| 修复后 | 18 | **0** | 0 | **PASS** — 与关闭开关逐位一致 |

与论文原文完全吻合：*"in a regular complex of cubical cells, no segment is
grazeable."*

### 7.2 §8 back-out：用人工病态 PLC 验证可达

构造一个**含凹棱（reentrant edge，内角 270°）**的 PLC：绕该棱两面外法向夹角
> 90° → 内角 ≥180° → **grazeable** → 保护球确实生成。实测诊断输出：

```
[grazeable] segments=10 grazeable=8      ← 凹棱被正确识别为 grazeable
[protect]   candidates=16
[backout]   protect point on seg (10,11) failed: cavity volume not conserved
[backout]   protect point on seg (11,8)  failed: ...
[backout]   protect point on seg (11,10) failed: ...
[protect]   ok=13 fail=3 ends=13 validate=1   ← 3 次退让，网格仍合法
[backout]   skip (4,5): cavity volume not conserved
[backout]   skip (4,6): cavity volume not conserved
validate=1  vol=4.000000                  ← 全程体积守恒
```

**关键结论**：`diff/tol ≈ 10⁸`（如 `diff=0.148 tol=1.78e-09`），是**真实几何
退化**而非浮点噪声 —— 与第 4 节「五顶点共球」的预言一致。退让（back-out）
**确实按 §8 工作**：放弃该点的插入，网格分毫未损。

### 7.3 意外发现：almost Delaunay 路线的「无限细分」症状

上节那个病态 PLC 暴露出 §1 警告的**可观测形态**。实测迭代轨迹：

```
[iter 2]     split (9,10)  len=2        pending=8
[iter 19500] split (36,19526) len=0     pending=19506   ← 零长边！
```

每轮都「成功」插点，但 `pendingEdges` 从 8 **单调膨胀到 ~19500**，边长退化到
`1e-12`~`1e-5`，打满 20000 轮上限、耗时 **12.7s**，最终仍失败。

这正是论文 §1 那句 *"in danger of creating very short edges"* 的字面体现。
根因：本实现的 cavity 重建**不维持 Delaunay 性**，每次细分都会破坏先前已满足
的边，因此**不存在有限固定点**。

**已加护栏（两条，均在 `Constrained3D.cpp`）**：

| 护栏 | 判据 | 效果 |
|---|---|---|
| `edgeTooShort` 豁免 | 边长短于「原始约束边长中位数 × 1e-6」视为已满足 | 阻止零长边被无限二分 |
| `kMaxPending` 停滞检测 | `pendingEdges > 初始 + 64` 即报错退出 | **12.7s → 0.56s（23×）** |

报错信息也从含糊的 `did not converge after 20000 subdivisions` 变为**可诊断**的：

```
edge recovery diverged: pending constraint sub-edges grew to 75 (limit 74);
almost-Delaunay route cannot resolve this configuration
```

`Ruppert3DTest` 用例 3（已知待 M3.5 修复的回归护栏）同样受益：报错信息精确到
根因，且仍然 `validate=1 volumeOk=1`（安全底线不破）。

### 7.4 回归基线（全部通过，逐位不变）

- 单元测试 **8/8 全绿**（Delaunay2D/3D、Constrained3D、Flips3D、SurfaceMesh3D、
  ProjectArchive、Smooth3D、Ruppert3D）
- 端到端基线**逐位一致**：

| 用例 | 节点 | 体单元 | 表面三角形 | minSICN | iterations | insertedPoints |
|---|---|---|---|---|---|---|
| cube.stl | 10 | 11 | 20 | 0.577350 | 14 | 2 |
| cube10.stl | 103 | 405 | 124 | 0.000000 | 317 | 95 |

> 说明：cube/cube10 是**规则立方体 PLC**，grazeable 集为空 → 保护球不插点 →
> back-out 不触发。因此这两处修复对它们「零行为变化」是**正确**的（而非未生效）。
> 修复的价值体现在**含小角度/凹棱的真实 PLC** 上（见 §7.2）。

### 7.5 仍未解决（理论根治方案，超出当前目标范围）

`cube10.stl` 的 `minSICN = 0` 与 `Ruppert3DTest` 用例 3 的边界细化不收敛，
**根因是 almost Delaunay 路线本身**（§1、§2），不是任何单点 bug。要根治必须走
行动建议第 4 条：实现**严格增量 Delaunay 维护 + 真正的 CDT 构造**。这会让保护球
「段端自动成为网格边」的论文断言重新成立，也是 §7.3 无限细分的唯一出路。

当前实现的定位是：**在 almost Delaunay 路线下，把失败从「静默损坏 / 无界空转」
收敛为「可检测、可退让、可诊断」** —— 这是「框架清晰 + 功能不退步」目标下的
正确边界。

---

## 8. 翻转修复（restoreDelaunay）的「不恶化」护栏

> 本节与本论文的关联：§1 指出 almost Delaunay 路线 "the Delaunay property is
> not maintained during these vertex insertions"。工程上补这个缺口的手段是
> **局部翻转修复**（Lawson）。本节记录对该修复手段的一次完整核查。

### 8.1 核查方法

对 `restoreDelaunay` 做了「违规面计数」的独立验证：遍历每条内部面 (t, nb)，
用与实现**同一个谓词**（`insphereSoS`）判断对侧顶点是否落在邻居外接球内。

覆盖三类输入：

| 输入 | 退化程度 | 结果 |
|---|---|---|
| A 随机 200 点 | 一般位置 | `tets=1108`，违规 **0** → `flips=0` ✅ |
| B 精确 6³ 规则格点 | 大量 5+ 点共球 | `delaunay3D` **直接失败**并给出明确报错 ✅ |
| C 轻微扰动 5³ 格点 | 近退化（真实网格的 sliver 形态） | `tets=577`，违规 **0** → `flips=0` ✅ |

### 8.2 ★ 发现：3D 翻转修复会**恶化** Delaunay 性 ★

在退化输入（立方体六面各加 3×3 规则格点）上，未加护栏的 `restoreDelaunay`：

```
tets=130  初始违规面=61
restoreDelaunay flips=37  恢复后违规面=90      ← 单调恶化
```

逐面诊断（对每个违规面单独试翻并记录结果）：

```
违规面数=18  尝试翻转=18  成功=0
    16 x  拒绝：flip23: degenerate new tet
     2 x  拒绝：flip23: volume mismatch (new edge crosses existing edge)
```

且体积失配的实测值为 `rel=0.889` / `rel=0.25` —— 是**真实重叠**而非浮点噪声，
拒绝是正确的。结论：**单个 flip 的合法性判据没问题，问题在「该不该翻」**。

**理论根因**：2D 的 Lawson 翻转**保证**每次 incircle 翻转都严格改善 Delaunay 性，
故必然终止于 Delaunay 三角化；**3D 没有这个性质** —— 3D 翻转图存在环，
单次 flip23 可能让违规数增加。这是 3D 的固有困难（也是工程上做 3D Delaunay
修复普遍要配"插入/删除"等结构变更、而非纯翻转的原因）。

### 8.3 修复：试翻 + 只接受严格改善

`Flips3D.cpp::restoreDelaunay` 现把 flip23 当作**候选**：在副本上试翻，比较
全局违规数（`countLocalViolations`），**只有严格减少才应用**。

```
修复前：flips=37  违规 61 → 90   （恶化）
修复后：flips=1   违规 61 → 60   （改善，且保证不恶化）
```

同一改动对 `Smooth3DTest` 的效果：翻转数 `75 → 29`，而
`minDihedral / slivers / worstRho` **完全不变** —— 被剔除的 46 次翻转是纯空转。

**收益**：
- 违规数单调递减 → 循环必然终止，且**保证不恶化**；
- 与 §8（论文第 8 节）的 *"back out when vertex insertion cannot succeed"* 同一精神；
- 代价：每次候选翻转多一次 O(T) 计数 + 一次 O(T) 网格拷贝。当前规模
  （T 数百~数千）完全可接受；若将来 T 很大，可把计数改为只统计受影响区域。

### 8.4 同时修掉的三处隐患

| # | 隐患 | 修法 |
|---|---|---|
| 1 | `flip23` 未检查新边 `(d,e2)` 是否已存在（Joe 条件缺失）→ 会产生非流形边 | 加前置扫描，已存在则拒绝 |
| 2 | 硬编码容差 `1e-8` / `1e-6`（尺度相关，对 1e-3 或 1e4 量级的网格行为完全不同） | 改用尺度相对的 `orient3dEps(sc)` |
| 3 | `restoreDelaunay` 用 `insphereSoS` 决定翻转，`flip23` 却用 `orient3dDet` 校验结构 → 两套符号体系在退化处矛盾 | 结构判据统一到 `orient3dSoS` |

### 8.5 测试保真度修正

`Flips3DTest` 的违规计数器原先用 `insphereDet(...) > 1e-9`（带容差的朴素谓词），
而实现内部用 `insphereSoS(...) > 0` —— **测试在测一个与实现不同的判据**。
实测同一退化输入：朴素谓词只看到 2 个违规，SoS 谓词看到 61 个。
已统一到 `insphereSoS`，并新增用例 4（退化输入）把「不恶化」钉成硬断言：

```
== 4) restoreDelaunay on degenerate (cospherical) input ==
[PASS] degenerate: local Delaunay violations NOT increased (no-degradation guard)
  violations before=61 flips=1 after=60
```

该用例在未加护栏时会因 `61 → 90` 而失败，是一道有效的回归护栏。

### 8.6 回归基线（全部通过，逐位不变）

- 单元测试 **8/8 全绿**
- 端到端基线**逐位一致**：cube `10/11/0.577350/14/2`、
  cube10 `103/405/minSICN=0/317/95`

---

## 9. B-Rep 输入暴露的两个缺陷（2026-09-27）

`test_data/ball.brep`（球面 B-Rep）是本项目第一个「多边界点 + 曲面 + 负坐标」
的真实输入，把两个此前从未被观测到的缺陷逼了出来。两者都不是 almost Delaunay
路线的问题，而是**实现层面的判据/复杂度错误**。

### 9.1 `delaunay3D` 的每次插入全网格自检 → O(n·T)

`Delaunay3D.cpp` 插入循环里每插一点就调一次 `mesh.checkFaceHash()`
（遍历 4T 个面 + 哈希查找）。该自检只用于**定位**「第几步把拓扑弄坏了」，
却把整体复杂度从 O(n·T_avg) 退化成 O(n·T)：

| 输入 | 修复前 | 修复后 |
|---|---|---|
| ball.brep（3850 点） | >135 s 仍未完成 | **0.82 s** |
| Delaunay3DTest | 3.89 s | **0.30 s** |

修法：改成诊断开关（默认关），`delaunay3D(mesh, pts, verifyEachStep)` 或
环境变量 `MESHSTUDIO_DELAUNAY_VERIFY_EACH_STEP=1`。正确性兜底不变 ——
插入结束后的 `mesh.validate()` 始终执行。

### 9.2 `validate()` / `pruneDegenerateTets()` 判据不带误差界

**现象**：ball.brep 上 `delaunay3D` 报 `mesh invalid: tet not right-handed`。

**量化**（21824 个四面体）：

```
正 21823 / 零 0 / 负 1
负者 |det| = 3.19e-12 ；全体 |det| 均值 2.275、最大 5.175e+02
相对量级 6.2e-15  → 纯浮点噪声，不是真翻转
```

**根因**：构造阶段（`insertPoint3D`）在**归一化坐标**（平移+缩放，量级 ~1）下
用 `orient3dSoS` 定向；`validate()` 在**还原后的原始坐标**下用朴素 `orient3dDet`
判符号。行列式随缩放按 s³ 变化，对近退化薄片其符号由浮点误差支配 ——
两个坐标系下可以给出相反符号。这是**谓词不一致**，与 §9 论文强调的
「exact arithmetic + 符号扰动」正是同一类问题。

同类：`pruneDegenerateTets` 的退化阈值是**绝对常数** `1e-12`。
单位立方体（|det| ~ O(1)）合理；`cube2.stp`（坐标 500~3000，|det| ~ O(1e9)）
形同虚设；ball 的噪声薄片（3.19e-12）恰好卡在 1e-12 之上被漏检，
随后被 `validate` 报成假阳性。

**修法**：两处都改用尺度相对的 `orient3dEps(coordMag4(...))`（= 64·2⁻⁵²·s³），
与 `orient3dSoS` 内部判定退化用的是同一个界。

**修复后**：`delaunay3D` 对 ball 返回 `ok=1, tets=21822`（噪声薄片被正确剪除）。

### 9.3 回归基线（12/12，逐位不变）

- 单元测试 **8/8 全绿**（`Delaunay3DTest` 3.89 s → 0.30 s）
- 端到端**逐位一致**：cube `10/11/20/minSICN=0.577350`、
  cube10 `103/405/124/minSICN=0`
- 新增：`BRepInputTest --with-engine cube2.stp`（STEP 全链路）、
  `BRepInputTest ball.brep`（离散 + 焊接）
- 合计 `ctest -C Release` = **12/12，约 12.5 s**

### 9.4 未解决（算法层，非本次范围）

- **`enforceSurface3D`（CDT）在球面上是分钟级**：7692 个约束面、21822 个初始
  四面体，实测 >100 s 未完成。约束边恢复是「逐边 + 局部翻转/插点」，
  缺空间索引与局部化 —— 这是当前引擎最重的瓶颈。
- **凸且近似共球的点集本质退化**：球面离散点全在凸包上且共球，
  Delaunay 剖分不唯一、腔体判定处处平局。这类输入需要**先插内部 Steiner 点**
  或改用「边界优先」的构造顺序，而不是「先 Delaunay 全部表面点、再恢复约束面」。

---

## 10. 算法层治理（2026-09-27，接 §9）

§9 修的是「离散/焊接 + 谓词/复杂度」层面的缺陷。本节记录继续下钻到
**CDT 与细化**时发现的问题，以及一次被实测否定的"想当然修复"。

### 10.1 一类系统性缺陷：O(T) 全网格校验放进逐元素热循环

`validate()` / `checkFaceHash()` / `hasEdgeAny()` 都是 **O(T)**。项目里它们
被放进了三处逐元素循环，形成 **O(n·T) 二次方**：

| 位置 | 原状 | 修法 |
|---|---|---|
| `Delaunay3D` 插入循环 | 每插一点 `checkFaceHash()`（4T 次哈希查找） | 诊断开关（默认关），末尾 `validate()` 兜底 |
| `insertConstrainedPoint` 收尾 | **每次插入** `validate()` | 诊断开关；`enforceSurface3D` 收尾一次 |
| `Ruppert3D` 主循环 | **6 处**逐轮 `validate()` | 诊断开关；`ruppertRefine3D` 收尾一次 |
| `tryRemoveSliverByFlips` | 循环体内重算 `meanEdgeLength3D(m)` → **O(T²)** | 尺度量提到循环外；`worstBefore` 也提到面循环外 |
| `TetMesh3D::hasEdgeAny` | **O(T)** 全扫描（逐四面体查 6 条边） | 新增**边哈希** `m_edge`（增量维护引用计数）→ **O(1)** |

统一开关：`TetMesh3D::verifyEachStep()` / 环境变量
`MESHSTUDIO_VERIFY_EACH_STEP=1`。正确性兜底不变（各阶段收尾各做一次
全网格 `validate()`），只是失去"哪一步坏的"这个定位信息。

**边哈希**是本节唯一的**数据结构级**新增：`m_edge: key(无向边) → 引用计数`，
与既有 `m_face` 同构，在 `addTet` / `removeTet` 中同步维护
（swap-erase 搬家时四面体顶点集不变，故其 6 条边无需迁移）。
`validate()` 现在会**重算一遍引用计数比对** —— 因为这张表一旦失同步，
约束边恢复会**静默**判错，比"慢"危险得多。

**实测收益**：

| 项 | 前 | 后 |
|---|---|---|
| `Delaunay3DTest` | 3.89 s | **0.30 s** |
| 单元测试合计 | 12.76 s | **7.58 s** |
| `ball.brep` 全链路 | **>135 s 仍未完成** | **25.7 s**（且给出明确诊断） |

### 10.2 CDT 细分缺「约束面屏障」（真缺陷，已修）

`recoverConstraintEdges` 的细分循环调用 `insertConstrainedPoint` 时，
`excludeFaceKeys` 传的是**空集** —— 而同一个文件里保护球阶段的注释明确写着
"若传空集（这是最初的错误写法），cavity 会横穿约束面扩张……边界被破坏"。

后果（`MESHSTUDIO_DEBUG_CDT=1` 的 `[cdt-split]` 输出直接抓到）：

```
(4,87)   -> (4,3854)=1 (3854,87)=1     首次两个子边都成立
(4,3854) -> (4,3856)=1 (3856,3854)=0   第二子边没建立
(4,3856) -> ...                        随后又回到 (4,3856) 再细分
```

即：**刚建立的子边被后续插入破坏**，`pending` 单调增长、永不收敛。

修法：细分时也传 `constraintFaceKeys`（用 `MESHSTUDIO_CDT_NO_BARRIER=1` 可关闭做 A/B）。

| 输入 | 屏障关 | 屏障开 |
|---|---|---|
| `cube.stl` | 10/11/20, minSICN=0.577350 | **逐位一致** |
| `cube10.stl` | 103/405/124 | **逐位一致** |
| `ball.brep` 未满足约束边 | 11538（零进展，无限发散） | **5254**（恢复 6284 条 ≈ 55%） |

顺带修掉同型问题：`recoverEdgeByFlips` 原以 `{}` 作阻挡面调用，翻转会横穿
约束面破坏边界；现传入约束面集合，并加 `MESHSTUDIO_FLIP_RECOVERY=1` 开关。

### 10.3 发散判据选错了（已改）

旧判据是「待处理条目数 > 初始 + 64」。方向对，但**判据选错**：每次**合法**
细分都会让条目数 +1（1 条边换成 2 条子边），大输入在正常细分途中就会被误判。
实测 `ball.brep`（11538 条约束边）第 67 次细分即触发
`pending grew to 11603 (limit 11602)`。

正确判据是**未满足边数是否在进展**：
- 收敛路径：每细分一次就有边变成网格边 → `unsatisfied` 单调下降；
- 发散路径：只插点、`unsatisfied` 长期不降。

实测证据（放开旧护栏后）：`iter=37800, pending 11538→42546, points 3854→38258`，
38090 次插入、77 s，而 `unsatisfied` 始终不降 —— 确认是真发散。

新实现：每 2000 轮统计一次 `unsatisfied`，连续 3 个窗口未刷新历史最好值
即判发散并报错退出；同时把 `maxEdgeIters` / `kMaxPending` 改为与初始规模成比例
（`max(旧下限, 4×初始)`），既拦真发散又不误伤大输入。

### 10.4 一次被实测否定的"想当然修复"（★ 勿再改 ★）

`insertConstrainedPoint` 里「与 m 共面的边界面」的跳过阈值是**绝对常数** `1e-9`。
它看起来与 §9/§10.1 修过的"绝对阈值"是同族问题，理应改成尺度相对的
`orient3dEps(sc)`。**实测结果相反**：

| 判据 | `ball.brep` 未满足约束边 |
|---|---|
| 绝对 `1e-9` | **5254** |
| `orient3dEps(sc)` | 6717（**更差**） |

原因：两处判据**语义不同**。
- 「m 是否落在 nt 的面上」（决定 cavity 是否吞该 tet）——尺度相对是对的；
- 「这个边界面能不能省略重建」——省略的**代价**由紧随其后的体积守恒判据
  精确度量，阈值放宽只会让 cavity 边界上的"洞"变大 → 更多体积失配 → 更多退让。

结论：保持 `1e-9`。已在代码注释中留档，避免后人"顺手修"。

### 10.5 仍未解决：`ball.brep` 的 5254 条约束边

失败信息（现在 25.7 s 内给出，不再是无限挂起）：

```
edge recovery diverged: unsatisfied constraint edges made no progress
for 6000 subdivisions (unsatisfied=5254, pending=17977)
```

**根因**：球面点集**共球且凸**，`delaunay3D` 用顶点索引打破平局得到的边界
三角剖分与输入剖分**几乎处处不同**（同一批点、同一批面数，但近共面四边形的
**对角线取向不同**）。这类差异需要**翻转（对角交换）**才能消除，中点细分
做不到 —— 细分只是把断边换成两条更短的断边。

**为什么翻转路线目前不可用**：`recoverEdgeByFlips` 每次翻转尝试都要
**全扫描一遍四面体**（O(T)），单次调用即 O(maxFlips × T)；`ball` 上
T≈2.6 万，跑 200 s 一次插入都没发生。

**卡住边的失败形态**（`MESHSTUDIO_DEBUG_BACKOUT=1`）：
158 次退让全部是 `cavity volume not conserved`，且 `volNew > volCav`
（如 `volCav=12.3925 volNew=12.4966`，相对 +0.8%）—— 是**重叠**特征。
说明屏障开启后 cavity 对 m 不再满足星形性，棱锥互相重叠。

### 10.6 下一步（按性价比排序）

1. **让翻转路线可用**（最有希望，基础设施已就位）：把 `m_edge` 的值从
   "引用计数"扩展为 `{refcount, 一个所属 tet}`，再沿该 tet 绕边**扇面行走**
   （用既有面/邻居哈希，O(度) 即可枚举含该边的全部四面体），
   把 `recoverEdgeByFlips` 从 O(maxFlips × T) 降到 O(度)。
   然后对细分卡住的边启用路线 B（`MESHSTUDIO_FLIP_RECOVERY=1` 已可开）。
2. **pending 扫描改轮转**：现在「总是取索引 0 起第一条未满足边」，
   会对同一条边反复二分；改成轮转可把细分工作量摊到所有断边上。
3. **更远**：边界优先构造，或严格增量 Delaunay 维护（论文 §4 的前提）。

### 10.7 回归基线（12/12，逐位不变）

- 单元测试 **8/8**，合计 7.58 s（治理前 12.76 s）
- 端到端 **逐位一致**：cube `10/11/20/minSICN=0.577350`、
  cube10 `103/405/124/minSICN=0`
- B-Rep 链路：`cube2.stp` 全链路通过；`ball.brep` 离散 + 焊接通过
- 合计 `ctest -C Release` = **12/12，约 9.4 s**
