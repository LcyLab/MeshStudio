#include "Flips3D.h"

#include "GeomPredicates.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <set>

namespace mesher {

namespace {

// 按右手系添加四面体（定向错误则交换后两个顶点）
int addTetOriented(TetMesh3D& mesh, int a, int b, int c, int d)
{
    if (orient3dDet(mesh.points[a], mesh.points[b], mesh.points[c], mesh.points[d]) < 0.0)
        std::swap(c, d);
    return mesh.addTet(a, b, c, d);
}

} // namespace

bool flip23(TetMesh3D& mesh, int t1, int t2, std::string* error)
{
    const int n = static_cast<int>(mesh.tets.size());
    if (t1 == t2 || t1 < 0 || t2 < 0 || t1 >= n || t2 >= n) {
        if (error) *error = "flip23: invalid tet ids";
        return false;
    }
    const Tet4 T1 = mesh.tets[t1];   // 拷贝（removeTet 会改动数组）
    const Tet4 T2 = mesh.tets[t2];

    // 找 T1 的共享面索引 e（3 个顶点都在 T2 中）：d = v[e] 是 T1 独有顶点，
    // com 按该面的环序 (v[e+1],v[e+2],v[e+3]) 排列——必须用环序而非任意遍历顺序，
    // 否则 3 个新四面体的覆盖不自洽（会出现同向共享面 → 重叠）
    int se = -1, d = -1, e2 = -1;
    for (int i = 0; i < 4; ++i) {
        int cnt = 0;
        for (int m = 1; m <= 3; ++m) {
            const int v = T1.v[(i + m) % 4];
            for (int j = 0; j < 4; ++j)
                if (T2.v[j] == v) { ++cnt; break; }
        }
        if (cnt == 3) { se = i; d = T1.v[i]; break; }
    }
    if (se < 0) {
        if (error) *error = "flip23: tets do not share a face";
        return false;
    }
    const int com[3] = { T1.v[(se + 3) % 4], T1.v[(se + 2) % 4], T1.v[(se + 1) % 4] };
    // T2 的独有顶点 e2
    for (int j = 0; j < 4; ++j) {
        bool inT1 = false;
        for (int i = 0; i < 4; ++i)
            if (T2.v[j] == T1.v[i]) inT1 = true;
        if (!inT1) e2 = T2.v[j];
    }
    if (e2 < 0) {
        if (error) *error = "flip23: no opposite vertex in t2";
        return false;
    }
    // d、e2 必须在共享面两侧。
    // ★ 判据一律用 SoS（论文 §9）★
    // 关键：`restoreDelaunay` 用 insphereSoS 决定"要不要翻"，因此这里的结构
    // 校验**必须**用同一套符号体系（orient3dSoS），否则两套判据在共球/共面
    // 退化处会给出互相矛盾的结果 —— 出现"insphereSoS 说有违规、于是翻转，
    // 但 orient3d 说不合法/合法"的不一致，翻转就会破坏而非改善 Delaunay 性。
    // 这正是实测"违规面 61 → 90 单调恶化"的机制（见 tests/probe_restore.cpp）。
    //
    // 另外：不再用硬编码 1e-8/1e-6 绝对阈值（尺度相关 → 对不同尺寸的网格
    // 行为不可预测）。改用尺度相对的 orient3dEps（与 Constrained3D 同规格）。
    // ============================================================
    double sc = 1.0;
    {
        const Point3D* ps[4] = { &mesh.points[com[0]], &mesh.points[com[1]],
                                 &mesh.points[com[2]], &mesh.points[d] };
        for (int q = 0; q < 4; ++q) {
            sc = std::max(sc, std::fabs(ps[q]->x));
            sc = std::max(sc, std::fabs(ps[q]->y));
            sc = std::max(sc, std::fabs(ps[q]->z));
        }
    }
    const int s1 = orient3dSoS(mesh.points[com[0]], mesh.points[com[1]],
                               mesh.points[com[2]], mesh.points[d],
                               com[0], com[1], com[2], d);
    const int s2 = orient3dSoS(mesh.points[com[0]], mesh.points[com[1]],
                               mesh.points[com[2]], mesh.points[e2],
                               com[0], com[1], com[2], e2);
    if (s1 * s2 >= 0) {
        if (error) *error = "flip23: d/e2 on same side of shared face";
        return false;
    }
    // 预检查：两个旧四面体都不能是"零体积"（SoS 判定，绝不返回 0 → 用体积量级）
    const double volT1 = std::fabs(orient3dDet(mesh.points[T1.v[0]], mesh.points[T1.v[1]],
                                               mesh.points[T1.v[2]], mesh.points[T1.v[3]]));
    const double volT2 = std::fabs(orient3dDet(mesh.points[T2.v[0]], mesh.points[T2.v[1]],
                                               mesh.points[T2.v[2]], mesh.points[T2.v[3]]));
    const double epsT = orient3dEps(sc);
    if (volT1 <= epsT || volT2 <= epsT) {
        if (error) *error = "flip23: degenerate input tet";
        return false;
    }
    double volNew = 0.0;
    bool degenerateNew = false;
    for (int k = 0; k < 3; ++k) {
        const double v = orient3dDet(mesh.points[d], mesh.points[e2], mesh.points[com[k]],
                                     mesh.points[com[(k + 1) % 3]]);
        if (std::fabs(v) <= epsT) { degenerateNew = true; break; }
        volNew += std::fabs(v);
    }
    if (degenerateNew) {
        if (error) *error = "flip23: degenerate new tet";
        return false;
    }
    // 合法性：新四面体体积和必须等于旧两四面体体积和（否则新边 de 与外部边相交，
    // 翻转会超出/重叠 → 拓扑破坏）。这是 2-3 flip 的必要条件。
    const double volOld = volT1 + volT2;
    // 相对容差 1e-9（不是硬编码绝对值）：体积量级 ∝ 坐标^3，1e-9 相对是
    // 一个很松但足够紧的界，能吸收浮点误差而不会吞掉真实的重叠。
    if (std::fabs(volNew - volOld) > 1e-9 * volOld) {
        if (error) *error = "flip23: volume mismatch (new edge crosses existing edge)";
        return false;
    }

    // ★ 关键前置条件：新边 (d, e2) 在**删掉 T1/T2 之后**不得已存在 ★
    // ------------------------------------------------------------
    // 这是 2-3 flip（Joe 条件）的必备判据，旧实现遗漏了它 —— 后果正是
    // "翻转越翻越坏"：若 (d,e2) 已是一条网格边，翻转后该边会被**重复**，
    // 产生非流形拓扑，局部 Delaunay 性随之崩溃。
    // 实测（tests/probe_restore.cpp，立方体面上规则网格点 → 大量共球退化）：
    //   遗漏此判据时，restoreDelaunay 使违规面 61 → 90，**单调恶化**；
    //   每步单独观察也是 "违规面 61 -> 62 -> 63 -> 65 -> ..." 持续上升。
    //
    // 判定方式：扫一遍四面体，找一条含 (d,e2) 且**既不是 T1 也不是 T2** 的边。
    // 不能用 mesh.hasEdgeAny（它会把 T1/T2 自身的边也算进去 —— 但注意 T1/T2
    // 并不含 (d,e2)，d 只在 T1、e2 只在 T2，故实际上 T1/T2 不含该边；
    // 仍显式排除更稳妥，代价极低）。
    // ============================================================
    {
        const int lo = std::min(d, e2), hi = std::max(d, e2);
        for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
            if (t == t1 || t == t2)
                continue;
            const Tet4& T = mesh.tets[t];
            bool hasLo = false, hasHi = false;
            for (int k = 0; k < 4; ++k) {
                if (T.v[k] == lo) hasLo = true;
                if (T.v[k] == hi) hasHi = true;
            }
            if (hasLo && hasHi) {
                if (error)
                    *error = "flip23: new edge (d,e2) already exists (would create "
                             "non-manifold edge)";
                return false;
            }
        }
    }

    // 删除两个旧四面体（从大到小，swap-erase 安全）
    mesh.removeTet(std::max(t1, t2));
    mesh.removeTet(std::min(t1, t2));

    // 新建 3 个共享边 (d,e2) 的四面体：扇叶 (c0,c1)、(c1,c2)、(c2,c0)。
    // 定向时交换前两个顶点 (d,e2)（而非后两个）：保持扇叶顺序 (c_k,c_{k+1})，
    // 使 3 个四面体绕边 (d,e2) 环状一致；交换后两个会让扇叶顺序错乱 → 重叠。
    for (int k = 0; k < 3; ++k) {
        int a = d, b = e2, c = com[k], dd = com[(k + 1) % 3];
        const double o = orient3dDet(mesh.points[a], mesh.points[b], mesh.points[c], mesh.points[dd]);
        if (o < 0.0)
            std::swap(a, b);
        mesh.addTet(a, b, c, dd);
    }
    return true;
}

bool flip32(TetMesh3D& mesh, int t1, int t2, int t3, std::string* error)
{
    const int n = static_cast<int>(mesh.tets.size());
    if (t1 < 0 || t2 < 0 || t3 < 0 || t1 >= n || t2 >= n || t3 >= n
        || t1 == t2 || t2 == t3 || t1 == t3) {
        if (error) *error = "flip32: invalid tet ids";
        return false;
    }
    const Tet4 T[3] = { mesh.tets[t1], mesh.tets[t2], mesh.tets[t3] };

    // 找 3 个四面体的公共顶点（应为 2 个 = 共享边 (d,e)）
    int com[2] = { -1, -1 };
    int nc = 0;
    for (int i = 0; i < 4; ++i) {
        const int v = T[0].v[i];
        bool inAll = true;
        for (int k = 1; k < 3; ++k) {
            bool inTk = false;
            for (int j = 0; j < 4; ++j)
                if (T[k].v[j] == v) inTk = true;
            if (!inTk) inAll = false;
        }
        if (inAll) {
            if (nc < 2) com[nc] = v;
            ++nc;
        }
    }
    if (nc != 2) {
        if (error) *error = "flip32: tets do not share an edge";
        return false;
    }
    // 每个四面体的"扇叶"顶点（非共享边端点）
    int leaf[3] = { -1, -1, -1 };
    for (int k = 0; k < 3; ++k) {
        for (int j = 0; j < 4; ++j) {
            if (T[k].v[j] != com[0] && T[k].v[j] != com[1]) {
                leaf[k] = T[k].v[j];
                break;
            }
        }
        if (leaf[k] < 0) {
            if (error) *error = "flip32: degenerate fan";
            return false;
        }
    }
    // 删三个（从大到小）
    int ids[3] = { t1, t2, t3 };
    std::sort(ids, ids + 3, std::greater<int>());
    for (int k = 0; k < 3; ++k)
        mesh.removeTet(ids[k]);

    // 新建 2 个共享面 (leaf0,leaf1,leaf2) 的四面体
    addTetOriented(mesh, leaf[0], leaf[1], leaf[2], com[0]);
    addTetOriented(mesh, leaf[0], leaf[1], leaf[2], com[1]);
    return true;
}

// 局部 Delaunay 违规计数：每条内部面 (t, nb)，若对侧顶点落在对方外接球内即违规。
// 用 insphereSoS（论文 §9），与 restoreDelaunay 的判据同源，保证口径一致。
// 复杂度 O(T)。供 restoreDelaunay 的「不恶化」护栏使用。
int countLocalViolations(const TetMesh3D& mesh)
{
    int bad = 0;
    for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
        const Tet4& T = mesh.tets[t];
        for (int e = 0; e < 4; ++e) {
            const int nb = T.n[e];
            if (nb < 0 || nb < t)
                continue;   // 每个内部面只数一次
            const int d = T.v[e];
            const Tet4& N = mesh.tets[nb];
            int e2 = -1;
            for (int k = 0; k < 4; ++k) {
                bool shared = false;
                for (int m = 1; m <= 3; ++m)
                    if (N.v[k] == T.v[(e + m) % 4]) shared = true;
                if (!shared) e2 = N.v[k];
            }
            if (e2 < 0 || e2 == d)
                continue;
            if (insphereSoS(mesh.points[T.v[0]], mesh.points[T.v[1]], mesh.points[T.v[2]],
                            mesh.points[T.v[3]], mesh.points[e2],
                            T.v[0], T.v[1], T.v[2], T.v[3], e2) > 0)
                ++bad;
        }
    }
    return bad;
}

int restoreDelaunay(TetMesh3D& mesh, int maxFlips)
{
    // ============================================================
    // ★ 3D Lawson 迭代必须带「不恶化」护栏 ★
    // ------------------------------------------------------------
    // 理论背景：2D 的 Lawson 翻转**保证**每次 incircle 翻转都严格改善
    // Delaunay 性，因此必然终止于 Delaunay 三角化。**3D 没有这个性质** ——
    // 3D 翻转图存在环，单次 flip23 可能让局部 Delaunay 违规数**增加**。
    // 这不是本实现的 bug，而是 3D 的固有困难（也是为什么工程上做 3D
    // Delaunay 修复普遍要配"插入/删除"等结构变更，而非纯翻转）。
    //
    // 实测证据（退化输入：立方体面上规则格点，大量精确共球）：
    //   restoreDelaunay 做了 37 次**几何合法**的 flip23
    //   （每次都通过体积守恒 + validate），但局部违规面 61 → 90，
    //   **单调恶化**。逐面诊断显示 18 个违规面里 16 个被正确拒绝
    //   （"degenerate new tet"）、2 个被正确拒绝（体积失配 rel≥0.25），
    //   说明单个 flip 的合法性判据没问题，问题在**该不该翻**。
    //
    // 修法（本实现）：把 flip23 当作**候选**，在副本上试翻并比较全局
    // 违规数，只有**严格减少**才真正应用。效果：
    //   · 违规数单调递减 → 循环必然终止，且**保证不恶化**；
    //   · 与论文 §8 的 "back out when vertex insertion cannot succeed"
    //     同一精神：不可改善的操作要退让，而不是硬做。
    //   · 代价：每次候选翻转多一次 O(T) 违规计数 + 一次 O(T) 网格拷贝。
    //     当前网格规模（T 数百~数千）完全可接受；若将来 T 很大，
    //     可把计数改为只统计受影响区域（见注释末）。
    //
    // 注意：`rejected` 仍保留 —— 它避免对同一对反复试翻（试翻虽被回退，
    // 但拷贝+计数有成本）。
    // ============================================================
    int flips = 0;
    std::set<std::pair<int, int>> rejected;   // 已判定"翻转会破坏网格/不改善"的相邻对
    int curViol = countLocalViolations(mesh);
    for (int iter = 0; iter < maxFlips; ++iter) {
        bool done = true;
        for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
            const Tet4 tet = mesh.tets[t];   // 拷贝
            // 跳过退化四面体（细化在边界附近产生的近共面单元，翻转数值不稳）。
            // ★ 尺度相对容差（论文 §9 精神）★
            // 旧实现用硬编码 1e-8，对坐标量级 1e-3 或 1e4 的网格行为完全不同
            //（前者全部被跳过、后者全部被接受）。改用 orient3dEps(sc)：
            //   sc = 该四面体坐标的绝对量级，体积容差 ∝ 64·2^-52·sc³。
            // 与 flip23 内部、以及 Constrained3D 的判据口径统一。
            {
                double sc = 1.0;
                const Point3D* ps[4] = { &mesh.points[tet.v[0]], &mesh.points[tet.v[1]],
                                         &mesh.points[tet.v[2]], &mesh.points[tet.v[3]] };
                for (int q = 0; q < 4; ++q) {
                    sc = std::max(sc, std::fabs(ps[q]->x));
                    sc = std::max(sc, std::fabs(ps[q]->y));
                    sc = std::max(sc, std::fabs(ps[q]->z));
                }
                if (std::fabs(orient3dDet(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                                          mesh.points[tet.v[2]], mesh.points[tet.v[3]]))
                    <= orient3dEps(sc))
                    continue;
            }
            for (int e = 0; e < 4; ++e) {
                const int nb = tet.n[e];
                if (nb < 0 || nb < t)
                    continue;   // 每个内部面处理一次
                if (rejected.count({ std::min(t, nb), std::max(t, nb) }) != 0)
                    continue;
                // t 的该面 e 的对侧顶点 d；nb 的对侧顶点 e2
                const int d = tet.v[e];
                const Tet4 nt = mesh.tets[nb];
                int e2 = -1;
                for (int k = 0; k < 4; ++k) {
                    bool shared = false;
                    for (int m = 1; m <= 3; ++m)
                        if (nt.v[k] == tet.v[(e + m) % 4]) shared = true;
                    if (!shared) e2 = nt.v[k];
                }
                if (e2 < 0 || e2 == d)
                    continue;
                // e2 在 t 的外接球内（或 d 在 nb 球内）→ 非 Delaunay → 候选 flip23
                // 用 insphereSoS（论文 §9）：共球退化时按顶点索引确定性打破平局，
                // 绝不返回 0，避免固定浮点容差导致的判据不一致。
                if (insphereSoS(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                                mesh.points[tet.v[2]], mesh.points[tet.v[3]],
                                mesh.points[e2],
                                tet.v[0], tet.v[1], tet.v[2], tet.v[3], e2) > 0) {
                    // —— 试翻（副本上）+ 不恶化判据 ——
                    TetMesh3D trial = mesh;
                    std::string err;
                    bool accepted = false;
                    int after = curViol;
                    if (flip23(trial, t, nb, &err)) {
                        after = countLocalViolations(trial);
                        accepted = after < curViol;   // 只有**严格改善**才应用
                    }
                    if (accepted) {
                        mesh = std::move(trial);
                        curViol = after;
                        ++flips;
                        done = false;
                        break;
                    }
                    // 翻转不合法 或 不改善 → 记入 rejected，避免反复试
                    rejected.insert({ std::min(t, nb), std::max(t, nb) });
                }
            }
            if (!done)
                break;
        }
        if (done)
            break;
    }
    return flips;
}

// ============================================================
// recoverEdgeByFlips —— 约束边恢复（翻转路线，不新增顶点）
// ------------------------------------------------------------
// 3D 边恢复的标准思路（Shewchuk / Si 的 cavity 重三角化）：
//
//   目标：让 (u,v) 成为一条网格边。
//
//   定义 **cavity(u→v)**：所有「含 u **但不含** v」的四面体集合 C。
//   若 C 恰好构成一个**沿 (u,v) 的星形扇**（即 C 的边界面里，除 u 之外
//   都围绕 v 排布），则可以把整个 C 删掉、以 (u,v) 为轴心重新三角化
//   C 的边界面 —— 重三角化后 (u,v) 自然出现。
//
//   实操上的**渐进版**（本实现采用，更稳且易于失败回退）：
//     反复扫描 C，对每个 t ∈ C 找它「不含 u 的对侧面」f：
//       * 若 f 的邻居 nb ∉ C（即 nb 只由 f 一侧贴住腔），且 nb 含 v
//         —— 则翻转 t↔nb 后 (u,v) 逐步被引入，计一次 2-3 flip。
//       * 否则跳过，换下一个 t（或下一个面）。
//     直至 (u,v) 出现（成功）或无任何可翻转项（失败，调用方回退细分）。
//
//   为什么需要「nb 含 v」：2-3 翻转把共享面 (a,b,c) 换成共享边 (d,e)
//   （d=t 的对侧顶点=u 或其它，e=nb 的对侧顶点）。要让新边是 (u,v)，
//   必须确保 v 就在 nb 里 —— 即 v 是这条「待恢复边」的另一端且已在腔侧。
//
//   blockedFaces：约束面屏障。翻转会跨越该面 ⇒ 跳过（会毁掉已恢复表面）。
// ============================================================
bool recoverEdgeByFlips(TetMesh3D& mesh, int u, int v,
                        const std::unordered_set<uint64_t>& blockedFaces,
                        int* outFlips, std::string* error)
{
    if (outFlips) *outFlips = 0;
    if (u == v || u < 0 || v < 0
        || u >= static_cast<int>(mesh.points.size())
        || v >= static_cast<int>(mesh.points.size())) {
        if (error) *error = "recoverEdgeByFlips: invalid vertex ids";
        return false;
    }
    if (mesh.hasEdgeAny(u, v)) {
        if (outFlips) *outFlips = 0;
        return true;   // 已经是网格边
    }

    auto faceBlocked = [&](int a, int b, int c) -> bool {
        if (blockedFaces.empty())
            return false;
        int x = a, y = b, z = c;
        if (x > y) std::swap(x, y);
        if (y > z) std::swap(y, z);
        if (x > y) std::swap(x, y);
        const uint64_t fk = (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 42)
                          | (static_cast<uint64_t>(static_cast<uint32_t>(y)) << 21)
                          | static_cast<uint32_t>(z);
        return blockedFaces.count(fk) != 0;
    };

    const int maxFlips = 4096;
    int flips = 0;
    for (int guard = 0; guard < maxFlips; ++guard) {
        if (mesh.hasEdgeAny(u, v))
            break;

        bool didFlip = false;
        int bestT = -1, bestNb = -1;
        double bestQuality = -1.0;

        // 扫所有「含 u 不含 v」的四面体，挑一个「最可翻」的（体积最大 → 最稳）
        for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
            const Tet4 tet = mesh.tets[t];
            bool hasU = false, hasV = false;
            for (int k = 0; k < 4; ++k) {
                if (tet.v[k] == u) hasU = true;
                if (tet.v[k] == v) hasV = true;
            }
            if (!hasU || hasV)
                continue;   // 只处理 C = 「含 u 不含 v」

            for (int e = 0; e < 4; ++e) {
                const int a = tet.v[(e + 1) % 4];
                const int b = tet.v[(e + 2) % 4];
                const int c = tet.v[(e + 3) % 4];
                if (a == u || b == u || c == u)
                    continue;               // 只翻转「对 u 的对侧面」
                if (faceBlocked(a, b, c))
                    continue;               // 约束面屏障
                const int nb = tet.n[e];
                if (nb < 0)
                    continue;
                const Tet4& nt = mesh.tets[nb];
                bool nbHasU = false, nbHasV = false;
                for (int k = 0; k < 4; ++k) {
                    if (nt.v[k] == u) nbHasU = true;
                    if (nt.v[k] == v) nbHasV = true;
                }
                if (nbHasU)
                    continue;               // 邻居也在 C 内 → 翻转无益
                if (!nbHasV)
                    continue;               // 邻居不含 v → 翻转不会引入 (u,v)

                // 调用 flip23 试翻（其内部已有体积守恒 + 退化判定）
                TetMesh3D trial = mesh;     // 试探性翻转，成功才提交
                std::string err;
                if (!flip23(trial, t, nb, &err))
                    continue;
                std::string vv;
                if (!trial.validate(&vv))
                    continue;
                // 记录最优候选（本实现取第一个可用即可，留最优接口便于后续调优）
                const double vol = std::fabs(orient3dDet(mesh.points[tet.v[0]],
                                                         mesh.points[tet.v[1]],
                                                         mesh.points[tet.v[2]],
                                                         mesh.points[tet.v[3]]));
                if (vol > bestQuality) {
                    bestQuality = vol;
                    bestT = t;
                    bestNb = nb;
                }
            }
        }

        if (bestT >= 0) {
            std::string err;
            if (flip23(mesh, bestT, bestNb, &err)) {
                ++flips;
                didFlip = true;
            }
        }
        if (!didFlip)
            break;   // 无法继续（被阻挡 / 近退化 / 拓扑不允许）
    }

    if (outFlips) *outFlips = flips;
    const bool ok = mesh.hasEdgeAny(u, v);
    if (!ok && error)
        *error = "recoverEdgeByFlips: could not introduce edge ("
               + std::to_string(u) + "," + std::to_string(v) + ") in "
               + std::to_string(flips) + " flips";
    return ok;
}

} // namespace mesher
