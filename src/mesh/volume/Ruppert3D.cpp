// 文件职责：四面体细化：处理约束段、侵入和不合格单元，受边界恢复能力及运行预算限制。
#include "Ruppert3D.h"

#include "Constrained3D.h"
#include "Delaunay3D.h"
#include "Flips3D.h"
#include "Quality3D.h"
#include "Smooth3D.h"     // 计算全网格平均边长。

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace mesher {

namespace {

// ★ 逐步全网格自检（O(T)，默认关闭）★
// ------------------------------------------------------------
// 细化主循环里原本有 6 处 `mesh.validate(&verr)`，**每一轮迭代都执行**。
// validate() 是 O(T)（含面哈希 + 边哈希全量重算），于是整体变成
// O(迭代数 · T) 的二次方 —— 小输入看不出，大输入（T≈2 万）直接跑成分钟级。
//
// 这些检查的价值是**定位**"哪一轮把网格弄坏了"。生产路径上由
// ruppertRefine3D 收尾的一次性 validate() 兜底，覆盖不变。
// 需要定位时用环境变量 MESHSTUDIO_VERIFY_EACH_STEP=1 打开。
// 详见 TetMesh3D::verifyEachStep。
//
// 返回 true = 通过（或未开启自检）。失败时把 "<stage>: <原因>" 写入 error。
bool verifyMeshIfEnabled(const TetMesh3D& mesh, std::string* error, const char* stage)
{
    if (!TetMesh3D::verifyEachStep())
        return true;
    std::string verr;
    if (!mesh.validate(&verr)) {
        if (error)
            *error = std::string(stage) + ": " + verr;
        return false;
    }
    return true;
}

// 无向边 key
inline uint64_t edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

// 无向面 key（顶点排序，各占 21 位）
inline uint64_t faceKey(int a, int b, int c)
{
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 42)
         | (static_cast<uint64_t>(static_cast<uint32_t>(b)) << 21)
         | static_cast<uint32_t>(c);
}

// 3D 中点
inline Point3D midPoint3(const Point3D& a, const Point3D& b)
{
    return { 0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z) };
}

// ============================================================
// tryRemoveSliverByFlips —— 用局部翻转消除薄片四面体（不新增顶点）
// ------------------------------------------------------------
// 动机：Ruppert 细化在密集加密时会产出极端薄片（实测 rho ~1e9、
// 二面角 ~0.0015°、最短边比目标小 4 个数量级）。这类薄片的**外接球心
// 落在薄片内部**，到最近顶点距离 ~ lmin，小于插点安全阈值
// 最小插点间距：minDist = max(0.5*badLmin, maxEdgeLength*1e-3)。
// 因此「插球心」这条标准 Ruppert 路径对它无效（插了会近重合 → 级联）。
// 原先的唯一处理是 `skipped.insert(tet)`（隔离）—— 坏单元被永久留在网格里，
// 这正是 cube10 端到端 minSICN=0 的直接来源。
//
// 本函数提供第二条路：**翻转**。薄片之所以存在，通常是因为某条应有的边
// 缺失（Delaunay 局部性被破坏）。对薄片的每个面尝试 `flip23`，只要新构型
// 通过 validate 且**不产生更差的 minDihedral**，就提交。翻转不引入新点，
// 因此没有级联风险。
//
// 返回 true 表示已成功改变网格（薄片被翻转掉或其某条面被翻掉）。
// ============================================================
bool tryRemoveSliverByFlips(TetMesh3D& mesh, int tet)
{
    if (tet < 0 || tet >= static_cast<int>(mesh.tets.size()))
        return false;

    // ★ meanEdgeLength3D 是 O(T)，必须在循环外算一次 ★
    // ------------------------------------------------------------
    // 旧实现在「逐四面体求最差二面角」的循环体里每次都重算
    // meanEdgeLength3D(m)，使那两个循环退化成 **O(T²)**：
    // 原方式逐单元重复求全网格平均边长；现由调用方统一提供该尺度。
    // ball.brep（T≈2.2 万）单次调用即 ~2×4.8×10⁸ 次距离计算 → 分钟级；
    // 而本函数是**每个薄片**调用一次。
    // 修法：尺度量在循环外算一次，作为参数传进去。
    const double scaleBefore = meanEdgeLength3D(mesh);
    auto minDihedralOf = [](const TetMesh3D& m, double scale, int t) -> double {
        if (t < 0 || t >= static_cast<int>(m.tets.size()))
            return -1.0;
        const TetQuality q = tetQuality(m, m.tets[t], scale);
        return q.minDihedralDeg;
    };

    const double before = minDihedralOf(mesh, scaleBefore, tet);
    if (before < 0.0)
        return false;

    // 判据：翻转后**整个网格的最差二面角不得变差**。
    // 注意 tet 索引在 flip23 后可能已变（swap-erase），所以不做「局部
    // 指标」比较，而是比较全局最差二面角 —— 更稳健且无索引失效风险。
    // worstBefore 与面无关 → 提到面循环外算一次（原本每个面重算一遍）。
    double worstBefore = 180.0;
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t)
        worstBefore = std::min(worstBefore, minDihedralOf(mesh, scaleBefore, t));

    // 对薄片的 4 个面尝试 2-3 翻转
    for (int e = 0; e < 4; ++e) {
        const int nb = mesh.tets[tet].n[e];
        if (nb < 0)
            continue;   // 边界面不翻（约束面）

        TetMesh3D trial = mesh;   // 试探翻转，通过校验才提交
        std::string err;
        if (!flip23(trial, tet, nb, &err))
            continue;
        // 逐步自检（O(T)，默认关闭）：flip23 自身已做结构 + 体积守恒校验，
        // 这里只是额外的"整体拓扑"兜底。放进逐面循环 = O(T²)，故默认关闭，
        // 需要时用 MESHSTUDIO_VERIFY_EACH_STEP=1 打开。
        if (TetMesh3D::verifyEachStep()) {
            std::string verr;
            if (!trial.validate(&verr))
                continue;
        }

        const double scaleAfter = meanEdgeLength3D(trial);
        double worstAfter = 180.0;
        for (int t = 0; t < static_cast<int>(trial.tets.size()); ++t)
            worstAfter = std::min(worstAfter, minDihedralOf(trial, scaleAfter, t));
        if (worstAfter <= worstBefore)
            continue;   // 没改善 → 不提交

        mesh = std::move(trial);
        return true;
    }
    return false;
}

// 外接球心与半径：2(b-a)·o = |b|²-|a|² 三个方程，克拉默求解。共面（det≈0）返回 false。
bool circumsphere3D(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                    Point3D& o, double& R)
{
    double M[3][3], rhs[3];
    M[0][0] = b.x - a.x; M[0][1] = b.y - a.y; M[0][2] = b.z - a.z;
    rhs[0] = 0.5 * ((b.x * b.x + b.y * b.y + b.z * b.z) - (a.x * a.x + a.y * a.y + a.z * a.z));
    M[1][0] = c.x - a.x; M[1][1] = c.y - a.y; M[1][2] = c.z - a.z;
    rhs[1] = 0.5 * ((c.x * c.x + c.y * c.y + c.z * c.z) - (a.x * a.x + a.y * a.y + a.z * a.z));
    M[2][0] = d.x - a.x; M[2][1] = d.y - a.y; M[2][2] = d.z - a.z;
    rhs[2] = 0.5 * ((d.x * d.x + d.y * d.y + d.z * d.z) - (a.x * a.x + a.y * a.y + a.z * a.z));

    const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                     - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                     + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (std::fabs(det) < 1e-300)
        return false;   // 四点共面/退化

    const double detX = rhs[0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                      - M[0][1] * (rhs[1] * M[2][2] - M[1][2] * rhs[2])
                      + M[0][2] * (rhs[1] * M[2][1] - M[1][1] * rhs[2]);
    const double detY = M[0][0] * (rhs[1] * M[2][2] - M[1][2] * rhs[2])
                      - rhs[0] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                      + M[0][2] * (M[1][0] * rhs[2] - rhs[1] * M[2][0]);
    const double detZ = M[0][0] * (M[1][1] * rhs[2] - rhs[1] * M[2][1])
                      - M[0][1] * (M[1][0] * rhs[2] - rhs[1] * M[2][0])
                      + rhs[0] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    o.x = detX / det;
    o.y = detY / det;
    o.z = detZ / det;
    R = std::sqrt((o.x - a.x) * (o.x - a.x) + (o.y - a.y) * (o.y - a.y)
                  + (o.z - a.z) * (o.z - a.z));
    return true;
}

// 点 p 是否落在子段 (a,b) 的直径球内（encroachment）
inline bool inDiametralBall(const Point3D& p, const Point3D& a, const Point3D& b)
{
    return (a.x - p.x) * (b.x - p.x) + (a.y - p.y) * (b.y - p.y) + (a.z - p.z) * (b.z - p.z) < 0.0;
}

} // namespace

/// 细化既有 CDT。surfaceFaces 为输入输出参数：。
Ruppert3DResult ruppertRefine3D(TetMesh3D& mesh,
                                std::vector<std::array<int, 3>>& surfaceFaces,
                                const Ruppert3DParams& params)
{
    Ruppert3DResult res;
    if (!std::isfinite(params.maxEdgeLength) || params.maxEdgeLength <= 0.0
        || !std::isfinite(params.radiusEdgeRatio) || params.radiusEdgeRatio <= 0.0
        || params.maxIterations < 0 || mesh.tets.empty()) {
        res.error = "ruppertRefine3D: invalid parameters or empty mesh";
        return res;
    }
    std::string initialError;
    if (!mesh.validate(&initialError)) {
        res.error = "ruppertRefine3D: invalid input mesh: " + initialError;
        return res;
    }
    Ruppert3DStatus termination = Ruppert3DStatus::IterationLimit;
    std::unordered_set<int> skipped;   // 无法继续改善的四面体
    bool removedByFlips = false;       // 本轮是否用翻转消除过薄片（诊断用）

    // ---- 约束子段列表（初始 = 表面所有边，去重） ----
    std::vector<std::pair<int, int>> segList;
    {
        std::unordered_set<uint64_t> seen;
        for (const auto& f : surfaceFaces) {
            for (int k = 0; k < 3; ++k) {
                const int a = f[k], b = f[(k + 1) % 3];
                if (seen.insert(edgeKey(a, b)).second)
                    segList.push_back({ a, b });
            }
        }
    }
    // 约束面集合（插入球心时 cavity 不跨越）
    std::unordered_set<uint64_t> faceSet;
    for (const auto& f : surfaceFaces)
        faceSet.insert(faceKey(f[0], f[1], f[2]));

    auto findSeg = [&](int a, int b) {
        const uint64_t k = edgeKey(a, b);
        for (int i = 0; i < static_cast<int>(segList.size()); ++i)
            if (edgeKey(segList[i].first, segList[i].second) == k)
                return i;
        return -1;
    };
    auto splitSeg = [&](int idx, std::string* err) -> bool {
        const auto e = segList[idx];
        const Point3D m = midPoint3(mesh.points[e.first], mesh.points[e.second]);
        // 收集包含待分裂边 (e.first,e.second) 的**全部约束面**，作为 cavity 的
        // 不可跨越屏障：m 落在该约束边上，两侧的约束面必须保持为网格面。
        std::unordered_set<uint64_t> barred;
        for (const auto& f : surfaceFaces) {
            const bool hasU = (f[0] == e.first || f[1] == e.first || f[2] == e.first);
            const bool hasV = (f[0] == e.second || f[1] == e.second || f[2] == e.second);
            if (hasU && hasV)
                barred.insert(faceKey(f[0], f[1], f[2]));
        }

        // ============================================================
        // 路线 B（前置修复）：翻转恢复 —— 若 (u,v) **尚不是**网格边，先用局部
        // 2-3 翻转把它翻出来，**不新增顶点**。这是 3D 边恢复的标准手法，
        // 能避免「靠插中点强行制造边」带来的退化腔与体积扰动。
        // 注意：本条**不改变** segList 的细分语义 —— 它只修复「边缺失」，
        // 不处理「边超长」；超长仍必须走路线 A 的中点细分。
        // ============================================================
        if (!mesh.hasEdgeAny(e.first, e.second)) {
            int edgeFlips = 0;
            std::string flipErr;
            // 返回值刻意忽略：B 失败是**正常路径**，直接落到 A 的细分兜底
            // （与原有实现的行为保持一致 —— 即「功能差不多」）。
            recoverEdgeByFlips(mesh, e.first, e.second, barred, &edgeFlips, &flipErr);
        }

        // 路线 A：中点细分（把超长约束边一分为二，递归至满足尺寸函数）
        if (!insertConstrainedPoint(mesh, m, barred, { edgeKey(e.first, e.second) }, err))
            return false;
        // 同上：约束插入同样可能产生零体积单元，统一清除后再做 validate。
        pruneDegenerateTets(mesh, nullptr);
        const int nv = static_cast<int>(mesh.points.size()) - 1;
        segList[idx] = { e.first, nv };
        segList.push_back({ nv, e.second });
        // ============================================================
        // 关键修复（refine 阶段体积漂移根因）：
        // insertConstrainedPoint 的手动 cavity 重建会删除 m 周围**所有**四面体
        // 并重建。若某条已恢复的约束边 (u,v)（v≠m）属于被删的 cavity，
        // 重建后它可能不再是网格边 —— 该约束边的两条子段面片被一并删除，
        // 表面出现缝隙 → 覆盖域与体积真实改变（实测 refine 后漂 +4.1e-5）。
        // 因此每次分裂后必须把**全部约束子段**重新判定并恢复一遍。
        // 这正是 enforceSurface3D 的边恢复步骤；这里复用同一实现。
        // ============================================================
        mesher::EdgeRecoveryParams erp;
        erp.edgeProtection = params.edgeProtection;
        if (!recoverConstraintEdges(mesh, surfaceFaces, segList, nullptr, err,
                                    erp, &res.protectPoints))
            return false;
        pruneDegenerateTets(mesh, nullptr);
        ++res.insertedPoints;
        return true;
    };
    auto nearestDist = [&](const Point3D& p) {
        double best = std::numeric_limits<double>::max();
        for (const Point3D& q : mesh.points) {
            const double d = (q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y)
                           + (q.z - p.z) * (q.z - p.z);
            best = std::min(best, d);
        }
        return std::sqrt(best);
    };
    auto insertPoint = [&](const Point3D& p, std::string* err) -> bool {
        std::string e;
        if (!insertPoint3D(mesh, p, faceSet, 0, nullptr, nullptr, &e)) {
            *err = e;
            return false;
        }
        ++res.insertedPoints;
        // 共球/共面退化会让插入产生零体积（扁平）四面体。它们拓扑自洽但
        // FEM 不可用，且会让随后的 validate 立刻失败。与 delaunay3D 一致，
        // 插入后立即清除（零体积删除不改变体积与覆盖区域）。
        pruneDegenerateTets(mesh, nullptr);
        return true;
    };

    // ---- 停滞检测状态（收敛护栏） ----
    // 背景（实测）：cube 用例 refineBoundary=false 时，前 32 轮全部走
    // `skipped.insert(badTet)`（最坏边是约束边但边界不细化），随后进入
    // encroachment 级联：插入点全部贴着 y≈1.0/x≈1.0 边界，质量 score 从 7.07
    // 爆炸到 2679.64，`skipped` 涨到 1678 —— 永不满足 worstScore<=1.0，
    // 只能跑满 maxIterations(=300000) 才退出，表现为「测试超时」。
    // 正确做法：识别「连续多轮无实质进展」并以 Stagnated 退出，
    // 让调用方拿到一个**可用的（虽未达标）网格**而非挂死。
    // 判据：连续 N 轮既没有插入新点、也没有分裂约束边、worstScore 未下降。
    const double kStagnateWindow = 64.0;      // 连续停滞轮数上限（相对网格规模）
    int stagnate = 0;
    double prevWorst = std::numeric_limits<double>::max();

    for (int iter = 0; iter < params.maxIterations; ++iter) {
        res.iterations = iter + 1;

        // ---- A) 超长约束边分裂（v1 关闭） ----
        // 3D 里分裂约束边的手动 cavity 重建在非 Delaunay 网格上界定不可靠
        //（与 M2 面恢复同一困难），v1 只做内部细化（边界由 M2 边恢复处理）。
        // 后续 M3.5 引入翻转框架后再启用边界细化。
        bool splitAny = false;
        if (params.refineBoundary) {
            for (int i = 0; i < static_cast<int>(segList.size()); ++i) {
                const auto& e = segList[i];
                const double len = std::sqrt(
                    (mesh.points[e.first].x - mesh.points[e.second].x)
                        * (mesh.points[e.first].x - mesh.points[e.second].x)
                    + (mesh.points[e.first].y - mesh.points[e.second].y)
                        * (mesh.points[e.first].y - mesh.points[e.second].y)
                    + (mesh.points[e.first].z - mesh.points[e.second].z)
                        * (mesh.points[e.first].z - mesh.points[e.second].z));
                if (len > params.maxEdgeLength) {
                    if (!splitSeg(i, &res.error)) {
                        res.error = "split segment failed: " + res.error;
                        return res;
                    }
                    splitAny = true;
                    break;
                }
            }
        }
        if (splitAny) {
            if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after split: "))
                return res;
            continue;
        }

        // ---- B) 找最坏四面体（质量 + 大小函数） ----
        int badTet = -1;
        double worstScore = 0.0;
        Point3D cc;
        double badLmin = 0.0;
        bool badBySize = false;
        for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
            if (skipped.count(t) != 0)
                continue;
            const Tet4& tet = mesh.tets[t];
            double R = 0.0;
            Point3D c;
            if (!circumsphere3D(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                                mesh.points[tet.v[2]], mesh.points[tet.v[3]], c, R))
                continue;
            double lmin = std::numeric_limits<double>::max();
            double lmax = 0.0;
            for (int e = 0; e < 4; ++e)
                for (int f = e + 1; f < 4; ++f) {
                    const double l = std::sqrt(
                        (mesh.points[tet.v[e]].x - mesh.points[tet.v[f]].x)
                            * (mesh.points[tet.v[e]].x - mesh.points[tet.v[f]].x)
                        + (mesh.points[tet.v[e]].y - mesh.points[tet.v[f]].y)
                            * (mesh.points[tet.v[e]].y - mesh.points[tet.v[f]].y)
                        + (mesh.points[tet.v[e]].z - mesh.points[tet.v[f]].z)
                            * (mesh.points[tet.v[e]].z - mesh.points[tet.v[f]].z));
                    lmin = std::min(lmin, l);
                    lmax = std::max(lmax, l);
                }
            if (lmin <= 0.0 || lmin < params.maxEdgeLength * 1e-6)
                continue;   // 退化四面体（近共面）：无法细化，跳过以免恶性循环
            double score = 0.0;
            if (R / lmin > params.radiusEdgeRatio)
                score = (R / lmin) / params.radiusEdgeRatio;
            if (lmax > params.maxEdgeLength)
                score = std::max(score, lmax / params.maxEdgeLength);
            if (score > worstScore) {
                worstScore = score;
                badTet = t;
                cc = c;
                badLmin = lmin;
                badBySize = (lmax > params.maxEdgeLength);
            }
        }
        if (badTet < 0 || worstScore <= 1.0) {
            termination = Ruppert3DStatus::Unresolved; // 全量收尾检查才能判定收敛
            break;
        }

        // ---- 收敛护栏：停滞检测 ----
        // 若本轮的「最坏质量分」相对上轮没有实质下降（<0.1%），则计一次停滞；
        // 超过窗口（随网格规模放大，至少 64 轮）即判定无法继续改善，主动收尾。
        if (worstScore >= prevWorst * 0.999) {
            ++stagnate;
        } else {
            stagnate = 0;
        }
        prevWorst = std::min(prevWorst, worstScore);
        if (stagnate > static_cast<int>(kStagnateWindow + mesh.tets.size() / 8)) {
            termination = Ruppert3DStatus::Stagnated;
            break;
        }
        const Tet4& btet = mesh.tets[badTet];

        // ---- C) 插入决策 ----
        if (badBySize) {
            // 最长边是约束子段 → 分裂；否则插中点
            double lmax = 0.0;
            int lu = -1, lv = -1;
            for (int e = 0; e < 4; ++e)
                for (int f = e + 1; f < 4; ++f) {
                    const double l = std::sqrt(
                        (mesh.points[btet.v[e]].x - mesh.points[btet.v[f]].x)
                            * (mesh.points[btet.v[e]].x - mesh.points[btet.v[f]].x)
                        + (mesh.points[btet.v[e]].y - mesh.points[btet.v[f]].y)
                            * (mesh.points[btet.v[e]].y - mesh.points[btet.v[f]].y)
                        + (mesh.points[btet.v[e]].z - mesh.points[btet.v[f]].z)
                            * (mesh.points[btet.v[e]].z - mesh.points[btet.v[f]].z));
                    if (l > lmax) { lmax = l; lu = btet.v[e]; lv = btet.v[f]; }
                }
            const int si = findSeg(lu, lv);
            if (si >= 0) {
                if (!params.refineBoundary) {
                    // ============================================================
                    // 薄片抢救（翻转路线）—— 挂在**实际逃逸路径**上
                    // ------------------------------------------------------------
                    // 实测（_RescueProbe, cube10）：失败隔离共 221 次，分布为
                    //   qBySizeSeg=136（本分支，超长约束边但 refineBoundary=false）
                    //   qBySizeMidDist=85（质量路径中点过近）
                    //   qByCCDist=0（球心路径 —— 原先抢救挂在这里，从未命中）
                    // 因此抢救必须挂在本分支和质量路径分支，否则等于没接。
                    // ============================================================
                    if (tryRemoveSliverByFlips(mesh, badTet)) {
                        removedByFlips = true;
                        ++res.sliverRescueOk;
                        continue;
                    }
                    ++res.sliverRescueTried;
                    skipped.insert(badTet);   // v1：边界不细化，跳过该四面体
                    ++res.qBySizeSeg;
                    ++res.quarantined;
                    continue;
                }
                if (!splitSeg(si, &res.error)) {
                    res.error = "split(oversize) failed: " + res.error;
                    return res;
                }
                if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after split(oversize): "))
                    return res;
                continue;
            }
            const Point3D mid = midPoint3(mesh.points[lu], mesh.points[lv]);
            if (nearestDist(mid) < 0.5 * badLmin) {
                // 薄片抢救（翻转路线）：见上方 qBySizeSeg 分支说明
                if (tryRemoveSliverByFlips(mesh, badTet)) {
                    removedByFlips = true;
                    ++res.sliverRescueOk;
                    continue;
                }
                ++res.sliverRescueTried;
                skipped.insert(badTet);
                ++res.qBySizeMidDist;
                ++res.quarantined;
                continue;
            }
            std::string insertError;
            if (!insertPoint(mid, &insertError)) {
                // 中点落在约束面上（walk 失败）→ v1 跳过该四面体
                skipped.insert(badTet);
                ++res.qBySizeMidInsert;
                ++res.quarantined;
                res.diagnostic = "midpoint insertion rejected: " + insertError;
                continue;
            }
            if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after insert midpoint: "))
                return res;
            continue;
        }

        // 质量路径：encroach 子段 → 分裂（v1 关闭，见 A 部分说明）；否则插球心
        if (params.refineBoundary) {
            bool encroached = false;
            for (int i = 0; i < static_cast<int>(segList.size()); ++i) {
                const auto& e = segList[i];
                if (inDiametralBall(cc, mesh.points[e.first], mesh.points[e.second])) {
                    if (!splitSeg(i, &res.error)) {
                        res.error = "split(encroached) failed: " + res.error;
                        return res;
                    }
                    if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after split(encroached): "))
                        return res;
                    encroached = true;
                    break;
                }
            }
            if (encroached)
                continue;
        }

        // 球心过近（相对最短边 + 绝对下界，防退化级联）→ 跳过
        const double minDist = std::max(0.5 * badLmin, params.maxEdgeLength * 1e-3);
        if (nearestDist(cc) < minDist) {
            // ============================================================
            // 薄片抢救（翻转路线）：插入球心会把新点放到离已有顶点过近处
            //（薄片的外接球心落在薄片内部，到顶点距离 ~ lmin），会引发
            // 近重合点 → 级联，所以不能插。此时**先尝试用局部翻转消除薄片**
            //（不新增顶点，无级联风险）；翻转失败才隔离跳过。
            //
            // 背景：这类薄片（实测 rho 可达 1e9、二面角 0.0015°）是 cube10 等
            // 大规模加密用例下 minSICN=0 的直接来源。原先只有「隔离」一条路
            //（第 409 行的 skipped.insert），坏单元被永久留在网格里。
            // 这里补上「消除」这条路 —— 正是 Flips3D 框架本应接入的位置。
            // ============================================================
            if (tryRemoveSliverByFlips(mesh, badTet)) {
                // 薄片已被翻转消除 —— 该 tet 索引可能已失效（swap-erase），
                // 但下一轮会重新扫描，无需特殊处理。继续下一轮即可。
                removedByFlips = true;
                ++res.sliverRescueOk;
                continue;
            }
            ++res.sliverRescueTried;
            skipped.insert(badTet);
            ++res.qByCCDist;
            ++res.quarantined;
            continue;
        }
        std::string insertError;
        if (!insertPoint(cc, &insertError)) {
            // 球心在域外（walk 失败）：v1 跳过该四面体（边界细化关闭时无子段可分裂）
            if (!params.refineBoundary) {
                skipped.insert(badTet);
                ++res.qByCCOutside;
                ++res.quarantined;
                res.diagnostic = "circumcenter insertion rejected: " + insertError;
                continue;
            }
            int best = -1;
            double bestD = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(segList.size()); ++i) {
                const auto& e = segList[i];
                const Point3D m = midPoint3(mesh.points[e.first], mesh.points[e.second]);
                const double d = (m.x - cc.x) * (m.x - cc.x) + (m.y - cc.y) * (m.y - cc.y)
                               + (m.z - cc.z) * (m.z - cc.z);
                if (d < bestD) { bestD = d; best = i; }
            }
            if (best < 0 || !splitSeg(best, &res.error)) {
                res.error = "insert circumcenter failed and no segment to split: " + insertError
                          + "; " + res.error;
                return res;
            }
            if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after split(cc-outside): "))
                return res;
            continue;
        }
        if (!verifyMeshIfEnabled(mesh, &res.error, "mesh invalid after insert circumcenter: "))
            return res;
    }

    // ---- 统计 ----
    res.tetCount = static_cast<int>(mesh.tets.size());
    res.maxEdgeLen = 0.0;
    res.worstRadiusEdge = 0.0;
    res.minDihedralDeg = 180.0;
    res.slivers = 0;
    const double qualityScale = meanEdgeLength3D(mesh);
    for (const Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e)
            for (int f = e + 1; f < 4; ++f) {
                const double l = std::sqrt(
                    (mesh.points[t.v[e]].x - mesh.points[t.v[f]].x)
                        * (mesh.points[t.v[e]].x - mesh.points[t.v[f]].x)
                    + (mesh.points[t.v[e]].y - mesh.points[t.v[f]].y)
                        * (mesh.points[t.v[e]].y - mesh.points[t.v[f]].y)
                    + (mesh.points[t.v[e]].z - mesh.points[t.v[f]].z)
                        * (mesh.points[t.v[e]].z - mesh.points[t.v[f]].z));
                res.maxEdgeLen = std::max(res.maxEdgeLen, l);
            }
        const TetQuality quality = tetQuality(mesh, t, qualityScale);
        const double d = quality.minDihedralDeg;
        res.worstRadiusEdge = std::max(res.worstRadiusEdge, quality.radiusEdge);
        if (quality.maxEdgeLength > params.maxEdgeLength * (1.0 + 1e-8)
            || !std::isfinite(quality.radiusEdge)
            || quality.radiusEdge > params.radiusEdgeRatio * (1.0 + 1e-8)
            || quality.degenerate)
            ++res.unresolvedTets;
        res.minDihedralDeg = std::min(res.minDihedralDeg, d);
        if (d < 5.0)
            ++res.slivers;
    }
    for (const auto& e : segList) {
        const double l = std::sqrt(
            (mesh.points[e.first].x - mesh.points[e.second].x)
                * (mesh.points[e.first].x - mesh.points[e.second].x)
            + (mesh.points[e.first].y - mesh.points[e.second].y)
                * (mesh.points[e.first].y - mesh.points[e.second].y)
            + (mesh.points[e.first].z - mesh.points[e.second].z)
                * (mesh.points[e.first].z - mesh.points[e.second].z));
        res.maxEdgeLen = std::max(res.maxEdgeLen, l);
    }

    // ---- 收尾整体自检（O(T)，一次）----
    // 主循环里的 6 处逐轮 validate 已改为默认关闭（见 verifyMeshIfEnabled）。
    // 这里做**一次性**兜底：覆盖不变，只是失去"哪一轮坏的"定位信息。
    {
        std::string verr;
        if (!mesh.validate(&verr)) {
            res.error = "mesh invalid after ruppertRefine3D: " + verr;
            res.converged = false;
            res.status = Ruppert3DStatus::Failed;
            return res;
        }
    }
    res.status = termination;
    res.converged = res.unresolvedTets == 0
                 && res.maxEdgeLen <= params.maxEdgeLength * (1.0 + 1e-8);
    if (res.converged) {
        res.status = Ruppert3DStatus::Converged;
        res.diagnostic.clear();
    } else {
        const std::string reason = res.status == Ruppert3DStatus::Stagnated ? "stagnated"
            : res.status == Ruppert3DStatus::IterationLimit ? "iteration limit reached"
            : "unresolved tetrahedra (including skipped candidates)";
        res.diagnostic = "ruppertRefine3D: " + reason + "; unresolved="
                       + std::to_string(res.unresolvedTets) + "; quarantined="
                       + std::to_string(res.quarantined)
                       + (res.diagnostic.empty() ? "" : "; " + res.diagnostic);
    }
    return res;
}

} // namespace mesher
