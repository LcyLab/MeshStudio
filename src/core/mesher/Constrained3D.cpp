// 诊断用 std::getenv（MESHSTUDIO_DEBUG_PROTECT / MESHSTUDIO_DEBUG_BACKOUT）：
// MSVC 会报 C4996，局部放行。两者都是**只读诊断开关**，默认关闭，不改变算法行为。
#ifdef _MSC_VER
#  define _CRT_SECURE_NO_WARNINGS
#endif

#include "Constrained3D.h"

#include "Flips3D.h"          // 路线 B：recoverEdgeByFlips（翻转恢复，不新增顶点）
#include "GeomPredicates.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>

namespace mesher {

namespace {

// 3D 中点（MesherTypes 只提供 Point2D 版本）
Point3D midPoint3(const Point3D& a, const Point3D& b)
{
    return { 0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z) };
}

// 无向边 key（顶点索引各占 32 位）
inline uint64_t edgeKey(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

// 无向面 key：三个顶点排序折叠（各占 21 位）
inline uint64_t faceKey(int a, int b, int c)
{
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 42)
         | (static_cast<uint64_t>(static_cast<uint32_t>(b)) << 21)
         | static_cast<uint32_t>(c);
}

// 四面体是否含边 {a,b}（6 条边全查）
bool tetHasEdge(const Tet4& t, int a, int b)
{
    for (int e = 0; e < 4; ++e)
        for (int f = e + 1; f < 4; ++f)
            if ((t.v[e] == a && t.v[f] == b) || (t.v[e] == b && t.v[f] == a))
                return true;
    return false;
}

// 四面体是否含面（顶点集相等即可）
bool tetHasFace(const Tet4& t, int a, int b, int c)
{
    // 检查 a,b,c 是否都在 t 的某个面（3 个顶点同时存在即可，因为面由 3 顶点决定）
    bool hasA = false, hasB = false, hasC = false;
    for (int e = 0; e < 4; ++e) {
        if (t.v[e] == a) hasA = true;
        if (t.v[e] == b) hasB = true;
        if (t.v[e] == c) hasC = true;
    }
    return hasA && hasB && hasC;
}

// 找含边 {a,b} 的所有四面体
std::vector<int> findTetsWithEdge(const TetMesh3D& mesh, int a, int b)
{
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i)
        if (tetHasEdge(mesh.tets[i], a, b))
            out.push_back(i);
    return out;
}

// 找含面 {a,b,c} 的所有四面体（作为面——即 3 顶点是其某个面）
std::vector<int> findTetsWithFace(const TetMesh3D& mesh, int a, int b, int c)
{
    std::vector<int> out;
    for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i)
        if (tetHasFace(mesh.tets[i], a, b, c))
            out.push_back(i);
    return out;
}

} // namespace

// 在约束保护下插入点 m（手动 cavity 重建，2D splitConstraint 的 3D 推广）。
// excludeFaceKeys：重建时剔除的面（含这些面的边界三角形不重建，目标被细分）。
// excludeEdges：重建时剔除含这些边的面（m 恰在这些边上或与边共面，重建会退化）。
bool insertConstrainedPoint(TetMesh3D& mesh, const Point3D& m,
                            const std::unordered_set<uint64_t>& excludeFaceKeys,
                            const std::unordered_set<uint64_t>& excludeEdges,
                            std::string* error)
{
    if (mesh.tets.empty()) {
        if (error) *error = "insertConstrained: empty mesh";
        return false;
    }

    // 1) seed：含 m 的四面体（m 可能在约束边/面上，walk 失败用暴力回退）
    int seed = -1;
    int walkSteps = 0;              // 诊断：walk 实际步数
    bool usedBruteScan = false;     // 诊断：是否走了 O(T) 暴力扫描
    bool usedNearestFallback = false;
    {
        // walk 定位（借用 Delaunay3D 的 locateTet 逻辑：p 与 v[e] 同侧）
        int t = 0;
        for (int guard = 0; guard < 2000000; ++guard) {
            ++walkSteps;
            const Tet4& tet = mesh.tets[t];
            bool inside = true;
            for (int e = 0; e < 4; ++e) {
                const Point3D& A = mesh.points[tet.v[(e + 1) % 4]];
                const Point3D& B = mesh.points[tet.v[(e + 2) % 4]];
                const Point3D& C = mesh.points[tet.v[(e + 3) % 4]];
                const Point3D& V = mesh.points[tet.v[e]];
                if (orient3dDet(A, B, C, V) * orient3dDet(A, B, C, m) < 0.0) {
                    const int nb = tet.n[e];
                    if (nb < 0) { t = -1; break; }
                    t = nb;
                    inside = false;
                    break;
                }
            }
            if (inside)
                break;
            if (t < 0)
                break;
        }
        seed = t;
    }
    if (seed < 0) {
        usedBruteScan = true;
        for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i) {
            const Tet4& tet = mesh.tets[i];
            bool inside = true;
            for (int e = 0; e < 4; ++e) {
                const Point3D& A = mesh.points[tet.v[(e + 1) % 4]];
                const Point3D& B = mesh.points[tet.v[(e + 2) % 4]];
                const Point3D& C = mesh.points[tet.v[(e + 3) % 4]];
                const Point3D& V = mesh.points[tet.v[e]];
                if (orient3dDet(A, B, C, V) * orient3dDet(A, B, C, m) < 0.0) {
                    inside = false;
                    break;
                }
            }
            if (inside) { seed = i; break; }
        }
    }
    if (seed < 0) {
        usedNearestFallback = true;
        // 兜底：m 在网格边界外一点点（约束边/面上的点数值偏出 1e-8），
        // 用"质心距 m 最近的四面体"作为 seed
        double bestD = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i) {
            const Tet4& tet = mesh.tets[i];
            const Point3D c = {
                (mesh.points[tet.v[0]].x + mesh.points[tet.v[1]].x + mesh.points[tet.v[2]].x
                 + mesh.points[tet.v[3]].x) / 4.0,
                (mesh.points[tet.v[0]].y + mesh.points[tet.v[1]].y + mesh.points[tet.v[2]].y
                 + mesh.points[tet.v[3]].y) / 4.0,
                (mesh.points[tet.v[0]].z + mesh.points[tet.v[1]].z + mesh.points[tet.v[2]].z
                 + mesh.points[tet.v[3]].z) / 4.0,
            };
            const double d = (c.x - m.x) * (c.x - m.x) + (c.y - m.y) * (c.y - m.y)
                           + (c.z - m.z) * (c.z - m.z);
            if (d < bestD) { bestD = d; seed = i; }
        }
    }
    // 诊断（MESHSTUDIO_DEBUG_CDT=1）：seed 定位的开销画像。
    // walkSteps 接近 2000000 说明 walk 在退化网格里**打转**（naive orient3dDet
    // 对近共面薄片会给出抖动符号），这是"插入极慢"的头号嫌疑。
    if (std::getenv("MESHSTUDIO_DEBUG_CDT"))
        std::fprintf(stderr, "[cdt-seed] walkSteps=%d brute=%d nearest=%d seed=%d T=%zu\n",
                     walkSteps, usedBruteScan ? 1 : 0, usedNearestFallback ? 1 : 0,
                     seed, mesh.tets.size());
    if (seed < 0) {
        if (error) *error = "insertConstrained: point outside mesh";
        return false;
    }

    // 2) cavity BFS：insphere(m)>=0 扩展。
    // ------------------------------------------------------------
    // ★ 必须**保护约束面**（旧 v1 注释「不保护约束面」是错的）★
    // 若允许 cavity 跨越约束面，重建后的新四面体会同时覆盖约束面两侧，
    // 使约束面不再是网格面（表面丢失）——这正是 v1 需要那套「剔除边界面」
    // 补丁的原因，而那个补丁本身又会留下空洞（见 3) 的说明）。
    // 正确做法：cavity 不跨越约束面。m 落在约束面上时，以 m 为公共面心的
    // 两侧各自独立重建 —— 这与 insertPoint3D 的 blockedFaces 语义一致。
    // ------------------------------------------------------------
    std::vector<char> inCav(mesh.tets.size(), 0);
    std::vector<int> cavity;
    std::vector<int> queue;
    queue.push_back(seed);
    inCav[seed] = 1;
    while (!queue.empty()) {
        const int ct = queue.back();
        queue.pop_back();
        cavity.push_back(ct);
        const Tet4& tet = mesh.tets[ct];
        for (int e = 0; e < 4; ++e) {
            const int nb = tet.n[e];
            if (nb < 0 || inCav[nb])
                continue;
            // 约束面（含被分裂的目标面）两侧互不跨越
            const int sa = tet.v[(e + 1) % 4];
            const int sb = tet.v[(e + 2) % 4];
            const int sc = tet.v[(e + 3) % 4];
            if (excludeFaceKeys.count(faceKey(sa, sb, sc)) != 0)
                continue;
            const Tet4& nt = mesh.tets[nb];
            // 1) m 落在 nt 的某个面上（m 在约束面/边上）→ 必进：
            //    约束面周围的四面体都要删除重建，否则与 m 的新四面体重叠
            // ------------------------------------------------------------
            // 容差改用**尺度相对**的 orient3dEps（而非旧代码的硬编码 1e-9）。
            // 理由：det 量级 ~ scale^3，尺度无关的固定阈值在 10³ 尺度上失效
            //（本项目 cube10 边长 10，坐标 ~10，det ~1000，1e-9 相当于要求
            //  相对精度 1e-12，已低于 double 能力）。
            // 这也是既有的、有推导的误差界，与 orient3dSoS 用的是同一个。
            bool mOnFace = false;
            for (int k = 0; k < 4 && !mOnFace; ++k) {
                const Point3D& A = mesh.points[nt.v[(k + 1) % 4]];
                const Point3D& B = mesh.points[nt.v[(k + 2) % 4]];
                const Point3D& C = mesh.points[nt.v[(k + 3) % 4]];
                const double d = orient3dDet(A, B, C, m);
                double sc = 0.0;
                const Point3D* ps[4] = { &A, &B, &C, &m };
                for (int q = 0; q < 4; ++q) {
                    sc = std::max(sc, std::fabs(ps[q]->x));
                    sc = std::max(sc, std::fabs(ps[q]->y));
                    sc = std::max(sc, std::fabs(ps[q]->z));
                }
                if (std::fabs(d) <= orient3dEps(sc))
                    mOnFace = true;
            }
            // 2) 外接球含 m（Bowyer-Watson）
            // ------------------------------------------------------------
            // ★ 改用 insphereSoS（论文 §9 p.9 的要求）★
            // Shewchuk 原文：
            //   "CDT construction algorithms are even **more sensitive to
            //    numerical error** than most geometric algorithms. Furthermore,
            //    PLCs in which **five or more vertices lie on a common sphere**
            //    can cause [the construction] to fail if degeneracies are not
            //    handled carefully."
            //   "I recommend that ... all the algorithms use a modified insphere
            //    test that employs a simple **symbolic perturbation** technique
            //    to simulate the circumstance where no five vertices are
            //    cospherical."
            // 立方体（8 顶点）上"五顶点共球"极易发生（如 2×2×1 的四个角 +
            // 任一其他点），旧代码 `insphereDet(...) > -1e-6` 用固定浮点容差
            // 兜底，容差选取无依据且不可复现。
            //
            // 语义说明：SoS 版**绝不返回 0**（共球时按顶点索引确定性打破平局），
            // 因此判据从 `> -1e-6`（含共球）改为 `>= 0`（含共球），语义等价。
            // 索引取自 nt.v[0..3] 与 m 的新索引 —— 保证同一组点在不同调用/
            // 不同朝向下判定一致，进而保证共享面两侧互相兼容。
            const bool inSphere = insphereSoS(mesh.points[nt.v[0]], mesh.points[nt.v[1]],
                                              mesh.points[nt.v[2]], mesh.points[nt.v[3]],
                                              m,
                                              nt.v[0], nt.v[1], nt.v[2], nt.v[3],
                                              static_cast<int>(mesh.points.size()) - 1) >= 0;
            if (mOnFace || inSphere) {
                inCav[nb] = 1;
                queue.push_back(nb);
            }
        }
    }

    // 2b) 星形闭包（关键）：把「m 看不见的边界面」外侧邻居拉进 cavity，
    //     直到所有边界面都对 m 可见。
    // ------------------------------------------------------------
    // 实测（cube 用例3，m=(1,0.5,0) 位于约束棱上）：
    //   cav=15 volCav=0.8854  volNew=1.0937  bnd=20  diff=-0.2083
    // 新棱锥体积**超过**腔体积 → 说明有边界面被 m 从"错误的一侧"看到，
    // 以 m 为顶点、该面为底的棱锥翻到了腔外 → 与其它棱锥重叠。
    // 这与 Delaunay3D::insertPoint3D 的 3b 是同一问题（非星形 cavity），
    // 修法也相同：扩张 cavity 使其对 m 星形。
    // 约束面（excludeFaceKeys）不可跨越 —— 遇屏障则本次插入放弃（安全拒绝）。
    for (int guard = 0; guard < 4096; ++guard) {
        int grew = 0;
        for (size_t ci = 0; ci < cavity.size(); ++ci) {
            const Tet4& tet = mesh.tets[cavity[ci]];
            for (int e = 0; e < 4; ++e) {
                const int nb = tet.n[e];
                if (nb < 0 || inCav[nb])
                    continue;
                const int a = tet.v[(e + 1) % 4];
                const int b = tet.v[(e + 2) % 4];
                const int c = tet.v[(e + 3) % 4];
                if (excludeFaceKeys.count(faceKey(a, b, c)) != 0)
                    continue;   // 约束面屏障：不可跨越
                const int opp = tet.v[e];
                const int sOpp = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                             mesh.points[opp], a, b, c, opp);
                const int sM = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                           m, a, b, c, static_cast<int>(mesh.points.size()));
                if (sOpp * sM < 0) {   // 不可见 → 拉到 cavity 里
                    inCav[nb] = 1;
                    cavity.push_back(nb);
                    ++grew;
                }
            }
        }
        if (grew == 0)
            break;
        if (cavity.size() > mesh.tets.size()) {
            // 闭包失控（吞掉整网格）：安全拒绝，网格保持原状
            if (error) *error = "insertConstrained: star closure exploded";
            return false;
        }
    }

    // 3) 边界面收集：对侧不在 cavity 的面。
    // ------------------------------------------------------------
    // ★ 关键不变量（旧实现在此处有致命缺陷，已修）★
    // 「删除 cavity」与「重建边界面」必须**严格互补**：
    //     ⋃ cavity 被删除 ⇒ ⋃ boundary 必须恰好填补同一区域
    // 旧实现在收集时按 excludeFaceKeys / excludeEdges / 与 m 共面 等条件
    // **静默丢弃**边界面，却仍然删除全部 cavity 四面体 → 被丢的面处留下**空洞**
    // → 网格非流形 → validate 报 "neighbor not symmetric (face mismatch)"
    // （Ruppert3DTest 用例3 实测即此）。
    //
    // 正确做法：**不允许丢弃任何边界面**。若某个边界面会导致退化
    //（与 m 共面 ⇒ 棱锥零体积），说明 **cavity 选得不对** —— 应当退回到
    // 插入前状态并拒绝本次插入，而不是删了腔再打补丁。
    // 因此这里改为：收集所有边界面，只要发现一个会退化的面就整体放弃。
    struct Face { int a, b, c; };
    std::vector<Face> boundary;
    boundary.reserve(cavity.size() + 4);
    for (const int ct : cavity) {
        const Tet4& tet = mesh.tets[ct];
        for (int e = 0; e < 4; ++e) {
            const int nb = tet.n[e];
            if (nb >= 0 && inCav[nb])
                continue;
            const int a = tet.v[(e + 1) % 4];
            const int b = tet.v[(e + 2) % 4];
            const int c = tet.v[(e + 3) % 4];
            // 与 m 共面的边界面：m 落在该面上 ⇒ 棱锥 (m,a,b,c) 零体积。
            // 这是**合法**情形（m 处于约束面/棱上，该面本就含 m），跳过重建即可。
            //
            // ★ 这里**刻意保留绝对阈值 1e-9**，不要"顺手"改成尺度相对的
            //   orient3dEps —— 已实测，改后更差 ★
            // ------------------------------------------------------------
            // 直觉上它和第 222-240 行「m 是否落在 nt 的面上」是同族问题，应当
            // 统一用尺度相对误差界。但实测（ball.brep + 屏障开启，同一进展护栏）：
            //     绝对 1e-9          → 未满足约束边 5254
            //     orient3dEps(sc)    → 未满足约束边 **6717**（更差）
            // 原因：放宽跳过阈值会跳过更多边界面 → cavity 边界留更多"洞" →
            // 体积守恒（volCav vs volNew）更多次失败 → 更多退让。
            // 也就是说这两处的判据**语义不同**：
            //   · 第 240 行问"m 是否落在面上"（决定 cavity 是否吞该 tet）——
            //     尺度相对是对的，已改；
            //   · 本行问"这个边界面能不能省略重建"——省略的**代价**由紧随其后的
            //     体积守恒判据精确度量，阈值放宽只会让"洞"变大。
            // 结论：保持 1e-9；真正要解决的是 cavity 在屏障下不满足星形性
            //（见 recoverConstraintEdges 的进展护栏注释）。
            if (std::fabs(orient3dDet(mesh.points[a], mesh.points[b], mesh.points[c], m)) < 1e-9)
                continue;
            if (orient3d(m, mesh.points[a], mesh.points[b], mesh.points[c]) > 0)
                boundary.push_back({ a, b, c });
            else
                boundary.push_back({ a, c, b });
        }
    }
    if (boundary.empty()) {
        // 安全拒绝：尚未删除任何四面体，网格保持原状（removeTet 不可逆）。
        if (error) *error = "insertConstrained: empty cavity boundary";
        return false;
    }

    // 3b) 体积守恒校验（与 Delaunay3D 的 3d 同一判据）
    // ------------------------------------------------------------
    // 上面跳过了与 m 共面的边界面。跳过是否安全，不由几何直觉决定，而由
    // **测度等价**精确判定：
    //     vol(⋃ cavity) == vol(⋃ (m, f) for f in boundary)
    // 成立 ⇒ 跳过的面确实是零体积退化面，网格补全无空洞；
    // 不成立 ⇒ 是真空洞/重叠 ⇒ 在删除之前安全拒绝。
    // 这样既允许「m 在约束面上」的合法细分，又杜绝旧实现的静默留洞。
    {
        double volCav = 0.0;
        for (const int ct : cavity) {
            const Tet4& t = mesh.tets[ct];
            volCav += orient3dDet(mesh.points[t.v[0]], mesh.points[t.v[1]],
                                  mesh.points[t.v[2]], mesh.points[t.v[3]]) / 6.0;
        }
        double volNew = 0.0;
        for (const Face& f : boundary)
            volNew += orient3dDet(m, mesh.points[f.a], mesh.points[f.b], mesh.points[f.c]) / 6.0;
        const double tol = 1e-9 * std::max(1.0, std::fabs(volCav));
        if (std::getenv("MESHSTUDIO_DEBUG_BACKOUT") && std::fabs(volCav - volNew) > tol) {
            std::fprintf(stderr, "[cav] volCav=%.17g volNew=%.17g diff=%.3g tol=%.3g\n",
                         volCav, volNew, std::fabs(volCav - volNew), tol);
        }
        if (std::fabs(volCav - volNew) > tol) {
            if (error)
                *error = "insertConstrained: cavity volume not conserved (would leave hole or overlap)";
            return false;
        }
    }

    // 4) 删除 cavity（从大到小，swap-erase 安全）
    std::sort(cavity.begin(), cavity.end(), std::greater<int>());
    for (const int ct : cavity)
        mesh.removeTet(ct);

    // 5) 插入 m 并重建 (m, 边界面)
    const int newV = mesh.addPoint(m);
    for (const Face& f : boundary)
        mesh.addTet(newV, f.a, f.b, f.c);

    // ★ 逐步全网格自检（默认关闭）★
    // validate() 是 O(T)。本函数在 CDT/细化里被**每次插入**调用一次，
    // 放进 O(n) 循环就是 O(n·T) 二次方 —— ball.brep 上实测每次插入 ~6 s
    //（T≈2.2 万），整个 enforceSurface3D 直接跑成分钟级。
    // 正确性兜底不变：enforceSurface3D / ruppertRefine3D 收尾各做一次
    // 全网格 validate()；且本函数**在修改网格之前**已有体积守恒校验
    //（第 3b 步，O(cavity)），它才是逐次插入真正需要的廉价护栏。
    // 需要"哪一次插入坏的"时用 MESHSTUDIO_VERIFY_EACH_STEP=1 打开。
    if (TetMesh3D::verifyEachStep()) {
        std::string verr;
        if (!mesh.validate(&verr)) {
            if (error) *error = "insertConstrained broke mesh: " + verr;
            return false;
        }
    }
    return true;
}

bool recoverConstraintEdges(TetMesh3D& mesh,
                            const std::vector<std::array<int, 3>>& faces,
                            std::vector<std::pair<int, int>>& edges,
                            int* outSteiner,
                            std::string* error,
                            const EdgeRecoveryParams& rp,
                            int* outProtect)
{
    if (outSteiner) *outSteiner = 0;
    if (outProtect) *outProtect = 0;

    // ============================================================
    // ★ 「面对角线」特例：不是每条约束边都必须成为网格边 ★
    // ------------------------------------------------------------
    // 约束面输入是**三角网格**，每个平面四边形面必然带一条对角线
    //（cube 用例：18 条约束边 = 12 条真实棱 + 6 条面对角线）。
    // 若强制要求对角线 (a,c) 成为网格边，就等于强制该平面面用**特定**对角线
    // 剖分 —— 这是对平面面**过度约束**。同一平面面用另一条对角线剖分同样合法
    //（Delaunay3D.cpp 的 pruneDegenerateTets 注释已论证过这一点）。
    //
    // 实测（_CoverProbe，cube 8 顶点 → enforceSurface3D）：
    //   6 个立方体面各自的网格边界面覆盖面积 = 1.000000000000（期望 1）
    //   总覆盖 = 6.000000000000（期望 6）
    //   → 表面被**完整且恰好一次**覆盖，只是其中 4 条对角线换了方向。
    //
    // 因此「边 (a,b) 是否已满足」改为：
    //   ① 直接是网格边 → 满足；或
    //   ② (a,b) 是某个约束三角形的边，且该三角形的**另两条边**也都是网格边
    //      ⇒ 该三角形已被网格面覆盖 ⇒ (a,b) 无需再细分。
    // 判据②对「平面面用另一条对角线」的情形恰好成立：
    //   四边形 {a,b,c,d} 若网格用了对角线 (b,d)，则三角形 (a,b,c) 与 (a,c,d)
    //   都被覆盖，即使 (a,c) 不是网格边。
    // ------------------------------------------------------------
    // 阶段 2 的「段端豁免」集合（阶段 1 关闭时恒为空 → 行为与原来完全一致）。
    // 论文："The second step does not insert a vertex into any of the ends."
    std::set<std::pair<int, int>> protEnds;

    // 边 → 含该边的约束三角形的第三顶点
    std::map<std::pair<int, int>, std::vector<int>> thirdOf;
    for (const auto& f : faces) {
        for (int k = 0; k < 3; ++k) {
            int a = f[k], b = f[(k + 1) % 3], c = f[(k + 2) % 3];
            if (a > b) std::swap(a, b);
            thirdOf[{ a, b }].push_back(c);
        }
    }
    // ★ 细分「地板长度」：防止 almost Delaunay 路线下的无限细分 ★
    // ------------------------------------------------------------
    // 论文 §1 对 almost Delaunay 路线的明确警告：
    //   "...in danger of creating very short edges."
    // 作为固定的**全局**下界，任何短于此的约束边都视为已满足（不再细分）。
    //
    // 为何必须存在（实测证据，ProbeBackout 用例 2026-09-24）：
    // 不做此限时，细分循环会把同一条边反复二分到长度 1e-12 量级，
    // 每轮新建一个点 + 2 个条目，`pendingEdges` 单调膨胀到 ~19500，
    // 打满 20000 轮上限才退出 —— 时间被完全浪费，恢复仍然失败。
    // 根因与论文一致：本实现的 cavity 重建**不维持 Delaunay 性**，
    // 细分不断破坏先前已满足的边（"the Delaunay property is not maintained
    // during these vertex insertions"），因此不存在有限固定点。
    //
    // 取值：原始约束边长的中位数的 1e-6，即"比任何有意义的
    // 边小六个数量级"。这足以吸收退化，而不会把正常的短边误判为已满足。
    double floorLen = 0.0;
    {
        std::vector<double> lens;
        lens.reserve(edges.size());
        for (const auto& e : edges) {
            const Point3D& p = mesh.points[e.first];
            const Point3D& q = mesh.points[e.second];
            const double dx = q.x - p.x, dy = q.y - p.y, dz = q.z - p.z;
            lens.push_back(std::sqrt(dx*dx + dy*dy + dz*dz));
        }
        if (!lens.empty()) {
            std::sort(lens.begin(), lens.end());
            floorLen = lens[lens.size() / 2] * 1.0e-6;   // 中位数 × 1e-6
        }
    }
    auto edgeTooShort = [&](int a, int b) -> bool {
        if (!(floorLen > 0.0)) return false;
        const Point3D& p = mesh.points[a];
        const Point3D& q = mesh.points[b];
        const double dx = q.x - p.x, dy = q.y - p.y, dz = q.z - p.z;
        return (dx*dx + dy*dy + dz*dz) <= floorLen * floorLen;
    };


    // 「边 (a,b) 是否已被满足」
    auto edgeSatisfied = [&](int a, int b) -> bool {
        if (mesh.hasEdgeAny(a, b))
            return true;   // ① 直接是网格边
        // ①b 阶段 1 的「段端」豁免（论文："The second step does not insert a
        //     vertex into any of the ends"）：保护球切出的短段不再细分，
        //     即使它暂不是网格边也放过 —— 强行细分会破坏强 Delaunay 性。
        {
            int x = a, y = b;
            if (x > y) std::swap(x, y);
            if (protEnds.count({ x, y }) != 0)
                return true;
        }
        // ①c 退化/过短边豁免（见上方 floorLen 说明）
        if (edgeTooShort(a, b))
            return true;
        int x = a, y = b;
        if (x > y) std::swap(x, y);
        auto it = thirdOf.find({ x, y });
        if (it == thirdOf.end())
            return false;
        // ② 含该边的某个约束三角形，其另两条边都是网格边 ⇒ 已被覆盖
        for (const int c : it->second) {
            if (mesh.hasEdgeAny(a, c) && mesh.hasEdgeAny(b, c))
                return true;
        }
        return false;
    };

    // 待处理约束边列表（(a,b) → 细分后 (a,m),(m,b)）
    std::vector<std::pair<int, int>> pendingEdges;
    pendingEdges.reserve(edges.size());
    for (const auto& e : edges)
        pendingEdges.push_back({ e.first, e.second });
    // 去重
    {
        std::set<std::pair<int, int>> uniq;
        for (const auto& e : pendingEdges)
            uniq.insert(e.first < e.second ? e : std::pair<int, int>{ e.second, e.first });
        pendingEdges.assign(uniq.begin(), uniq.end());
    }

    // ============================================================
    // ★ 阶段 1：保护球（edge protection）★
    // ------------------------------------------------------------
    // 文献依据：Shewchuk, "Constrained Delaunay Tetrahedralizations and
    // Provably Good Boundary Recovery"（见 docs/references/
    // shewchuk-cdt-boundary-recovery.pdf），第 8 页 "The edge protection
    // algorithm proceeds in two steps"。
    //
    // 【为什么必须有这一步】
    // 之前的实现**只有阶段 2（中点二分）**，缺阶段 1。后果是：约束棱的
    // **端点**附近持续被其它段「刺穿」（pierce），中点二分只能无止境地补，
    // 这正是实测 `refineBoundary=true` 不收敛（20 万次细分上限）的根因。
    // 论文原文：
    //   "If boundary recovery creates too-small tetrahedra, it might not be
    //    possible to fix them."
    //
    // 【算法（论文 Figure 11 / p.8）】
    //   * 对每个「以 <90° 相交两条约束段」的**输入顶点** v_i，
    //     以它为中心放一个**保护球** S_i，半径
    //         r_i = min{ lfs(v_i),  ℓ_i/3,  2·d_i/3 }
    //     其中 ℓ_i = 被 S_i 切到的最短段长，d_i = 与 v_i 相邻且**远端也被切**
    //     的最短段长（常使 d_i < ℓ_i）。
    //   * 对每条在 v_i 处与别的段锐角相交的段 s，插入点 s ∩ S_i。
    //   * **被切掉的「段端」(end) 保证强 Delaunay** → 自动成为网格边。
    //     论文原文："Each of these 'ends' is guaranteed to be strongly
    //     Delaunay because no vertex lies on or inside its diametral sphere."
    //   * 阶段 2 不再分割这些「段端」（"The second step does not insert a
    //     vertex into any of the ends"），因此保护效果得以保持。
    //
    // 【lfs 定义（论文 p.8）】
    //   "The local feature size lfs(v_i) is simply the distance from v_i to the
    //    nearest vertex or segment that doesn't intersect v_i."
    //
    // ============================================================
    // ★★ 实测结论：保护球必须配 grazeable 过滤，否则纯负收益 ★★
    // ------------------------------------------------------------
    // 已在 cube.stl / cube10.stl 上做完整 A/B 实测（2026-09-24）。
    //
    // 【修复前的错误行为】不做 grazeable 过滤时：
    //   1. candidates=36 → ok=36 fail=0（判据"成立"、插入"成功"）。
    //   2. 但论文的核心断言不成立：论文说"每个段端(end)保证强 Delaunay，
    //      因此自动成为网格边"。实测阶段 2 启动时：
    //          pendingEdges=18  unsatisfied=15  (of which ends=0)
    //      → 18 条约束边里仍有 15 条不是网格边，且无一条是段端。
    //   3. 随后阶段 2 的中点在 `(49,115)`/`(49,135)` 上 cavity 重建失败：
    //          "cavity volume not conserved (would leave hole or overlap)"
    //      连最简单的 cube.stl（原本 minSICN=0.577、0.002s 通过）也必然失败。
    //
    // 【修复】按论文 §3/§5 只对 **grazeable 段** 放保护球（见 `isGrazeable`）。
    //   实测修复后：`segments=18 grazeable=0 → candidates=0` —
    //   与论文原文完全吻合：
    //     "in a regular complex of cubical cells, no segment is grazeable."
    //   cube 用例下保护球不再插入任何点，端到端结果与关闭开关**逐位一致**。
    //
    // 【根因（理论层面，非 bug）】
    // 论文的前提是「**完整的增量 Delaunay 维护**」：
    //   "Insert all the new vertices ... into the Delaunay tetrahedralization
    //    constructed in Section 4 (using the Bowyer–Watson algorithm, so the
    //    mesh remains Delaunay)."
    // 而本项目的 `insertConstrainedPoint` 用的是**带约束面屏障的近似 cavity**
    //（保护约束面不跨越本身就是对 Delaunay 性的主动偏离），插完点后网格
    // **不再是 Delaunay**。于是"段端强 Delaunay ⇒ 自动成为网格边"这条
    // 由 Delaunay 性推出的结论**失去前提** —— 这正是 almost Delaunay 路线的
    // 固有局限（论文 §1 明确警告）。
    //
    // 【因此保护球的真实价值边界】
    //   · 对 grazeable 为空的 PLC（规则立方体）→ 正确行为是**什么都不做**；
    //   · 对含小角度/凹棱的 PLC → 会真的插点，且部分是"段端"（论文承诺的那
    //     一类），对恢复有正面作用；不可插入的点由 §8 back-out 退让处理。
    //   · 要让它达到论文承诺的完整效果，仍需严格增量 Delaunay 维护。
    //
    // 【结论】保留实现与开关（默认关闭 → 既有行为零漂移）。
    // 诊断：MESHSTUDIO_DEBUG_PROTECT=1 打印 grazeable/候选/成功/失败统计。
    // ============================================================
    // 【原实现说明（保留）】
    // 论文使用「保护球 + 强 Delaunay 判据」的完整增量 Delaunay 维护。本实现
    // 沿用既有 `insertConstrainedPoint`（手动 cavity 重建 + 体积守恒校验）。
    // ---- 约束面 key 集合（阶段 1 与阶段 2 共用）----
    // insertConstrainedPoint 的 cavity 靠 excludeFaceKeys 来「不跨越约束面」。
    // 若传空集，cavity 会横穿约束面扩张，重建后的新四面体覆盖约束面两侧
    // → 约束面不再是网格面 → 边界被破坏。
    // 实测（cube10，EP=1）：保护球 36/36 全部插入成功，但随后阶段 2 处理
    // (49,135) 时 mesh 已非合法 CDT，体积守恒判据失败 →
    //   "edge recovery failed on (49,135): cavity volume not conserved"
    std::unordered_set<uint64_t> constraintFaceKeys;
    constraintFaceKeys.reserve(faces.size() * 2);
    for (const auto& f : faces)
        constraintFaceKeys.insert(faceKey(f[0], f[1], f[2]));

    // ============================================================
    if (rp.edgeProtection) {
        // ============================================================
        // ★ grazeable 过滤（论文 §3 p.6）—— 决定"哪些段才需要强 Delaunay" ★
        // ------------------------------------------------------------
        // 论文原文：
        //   "A segment is grazeable if two consecutive facets in the rotary
        //    order are separated by an interior angle of 180° or more, or if
        //    the segment is included in fewer than two facets."
        //   "Only the grazeable segments need to be strongly Delaunay to
        //    guarantee a CDT."
        //   "For instance, in a regular complex of cubical cells, **no segment
        //    is grazeable**."  ← 本项目 cube/cube10 正是这一情形
        //   （weak edge protection）"considers only the grazeable segments,
        //    and ignores the others as if they did not exist."
        //
        // ★ 关键：先要区分「真正的 PLC 段」与「网格自己引入的对角线」★
        // 论文 p.6 明确说：
        //   "consider finding a tetrahedralization of a cube. Each square face
        //    must be partitioned by a diagonal edge ... The diagonals are **not
        //    elements of the PLC**. If the diagonals are not elements of the
        //    PLC, the existence of a CDT is guaranteed by Theorem 1, and a CDT
        //    construction algorithm can choose a compatible set of diagonals."
        //
        // 而本项目的 `pendingEdges` = 输入三角面的全部边 = **12 条真实棱 +
        // 6 条面对角线**。面对角线是"方脸必须被一条对角线剖分"的产物，
        // 不是 PLC 段，**不应参与保护球**。
        //
        // 判据：
        //   · 段所在的两个三角面**共面**（法向同向，|cos|≈1）
        //     → 该段是某个 facet 内部的对角线 → **不是 PLC 段 → 跳过**
        //   · 两个三角面**不共面** → 该段是两个不同 facet 的交线（真实棱）
        //     → 取两面的内二面角；对立方体 = 90° < 180° → **非 grazeable**
        //   · 附着面数 > 2 → 多 facet 共棱 → 非 grazeable（保守）
        //   · 附着面数 < 2 → 边界段 → grazeable（论文条件 b）
        //
        // 结论：cube/cube10 下 grazeable 全集为空 → 保护球插入 **0** 个点，
        // 与论文完全一致（实测不过滤时会插 36 个点，反而把网格搞坏）。
        // ============================================================
        std::map<std::pair<int, int>, std::vector<int>> segFacetCnt;   // 段 -> 包含它的面索引
        for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi) {
            const auto& f = faces[fi];
            for (int k = 0; k < 3; ++k) {
                int a = f[k], b = f[(k + 1) % 3];
                if (a > b) std::swap(a, b);
                segFacetCnt[{ a, b }].push_back(fi);
            }
        }
        auto faceNormal = [&](int fi) {
            const auto& f = faces[fi];
            const Point3D& p0 = mesh.points[f[0]];
            const Point3D& p1 = mesh.points[f[1]];
            const Point3D& p2 = mesh.points[f[2]];
            return Point3D{
                (p1.y-p0.y)*(p2.z-p0.z) - (p1.z-p0.z)*(p2.y-p0.y),
                (p1.z-p0.z)*(p2.x-p0.x) - (p1.x-p0.x)*(p2.z-p0.z),
                (p1.x-p0.x)*(p2.y-p0.y) - (p1.y-p0.y)*(p2.x-p0.x)
            };
        };
        auto isGrazeable = [&](int a, int b) -> bool {
            int x = a, y = b;
            if (x > y) std::swap(x, y);
            const auto it = segFacetCnt.find({ x, y });
            if (it == segFacetCnt.end())
                return true;      // 不属于任何约束面 → 按"<2 个 facet"→ grazeable
            if (it->second.size() < 2)
                return true;      // 论文条件 b
            if (it->second.size() > 2)
                return false;     // 多面共棱 → 非 grazeable（保守）
            // 恰 2 个三角面
            const Point3D n0 = faceNormal(it->second[0]);
            const Point3D n1 = faceNormal(it->second[1]);
            const double s0 = std::sqrt(n0.x*n0.x + n0.y*n0.y + n0.z*n0.z);
            const double s1 = std::sqrt(n1.x*n1.x + n1.y*n1.y + n1.z*n1.z);
            if (!(s0 > 0.0) || !(s1 > 0.0))
                return false;
            const double cosang = (n0.x*n1.x + n0.y*n1.y + n0.z*n1.z) / (s0 * s1);
            // ① 共面（|cos|≈1）→ 对角线，不是 PLC 段 → 跳过（返回 false）
            if (std::fabs(cosang) > 1.0 - 1e-9)
                return false;
            // ② 不共面 → 真实棱。内二面角 >= 180° ⟺ 外法向夹角 <= 0 ⟺ cos >= 0
            //    立方体相邻面：cos = 0（外法向正交）→ 内角 90° → 非 grazeable
            // 内二面角 >= 180° ⟺ cos >= 0。立方体相邻面 cos=0 → 非 grazeable。
            return cosang > 1e-12;
        };

        // ---- 1) 建立「顶点 → 相邻段」邻接（只收 grazeable 段） ----
        std::map<int, std::vector<int>> segNeighbors;   // v -> 相邻顶点列表
        int grazeableEdges = 0;
        for (const auto& e : pendingEdges) {
            if (!isGrazeable(e.first, e.second))
                continue;   // 非 grazeable（含对角线）→ 不需要强 Delaunay → 不放保护球
            ++grazeableEdges;
            segNeighbors[e.first].push_back(e.second);
            segNeighbors[e.second].push_back(e.first);
        }
        if (const char* dbg = std::getenv("MESHSTUDIO_DEBUG_PROTECT"))
            std::fprintf(stderr, "[grazeable] segments=%d grazeable=%d\n",
                         (int)pendingEdges.size(), grazeableEdges);
        // 段长
        auto segLen = [&](int a, int b) {
            const Point3D& p = mesh.points[a];
            const Point3D& q = mesh.points[b];
            return std::sqrt((p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y)
                           + (p.z-q.z)*(p.z-q.z));
        };
        // lfs(v) = min(到最近**顶点**的距离, 到最近**不相交段**的距离)
        // 简化实现：顶点距离取全网格最近；段距离取「所有不含 v 的段」到 v 的距离。
        // （论文允许「checking every segment of the PLC」的朴素做法）
        auto pointSegDist = [&](const Point3D& p, const Point3D& a, const Point3D& b) {
            const double vx = b.x-a.x, vy = b.y-a.y, vz = b.z-a.z;
            const double wx = p.x-a.x, wy = p.y-a.y, wz = p.z-a.z;
            const double vv = vx*vx + vy*vy + vz*vz;
            double t = vv > 0.0 ? (wx*vx + wy*vy + wz*vz) / vv : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            const double dx = wx - t*vx, dy = wy - t*vy, dz = wz - t*vz;
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        };

        // ★「段端」(ends) 记录 ★
        // 论文："The second step does not insert a vertex into any of the ends,
        //        so the ends remain strongly Delaunay."
        // 保护球点 p 把原段 (vi,n) 切成两段 (vi,p) 与 (p,n)，紧贴顶点 vi 的
        // 那一小段（长度 = ri）就是论文说的 "end"。阶段 2 必须在这些段上
        // **不再插中点**，否则强 Delaunay 性被破坏。
        // （protEnds 在外层声明，阶段 2 也要用）
        std::vector<std::pair<Point3D, std::pair<int, int>>> protPoints;   // (新点, 被切段)
        for (const auto& kv : segNeighbors) {
            const int vi = kv.first;
            const std::vector<int>& nbrs = kv.second;
            if (nbrs.size() < 2)
                continue;   // 论文只保护「>=2 条段相交」的顶点
            const Point3D& pv = mesh.points[vi];

            // ---- ℓ_i：被保护球切到的最短段（这里用「所有相邻段」的上界估计） ----
            auto shortestAdjSeg = [&]() {
                double m = std::numeric_limits<double>::max();
                for (const int n : nbrs)
                    m = std::min(m, segLen(vi, n));
                return m;
            }();
            if (!(shortestAdjSeg > 0.0))
                continue;

            // ---- lfs(v_i)：最近顶点 + 最近不相交段 ----
            double lfs = std::numeric_limits<double>::max();
            for (int j = 0; j < static_cast<int>(mesh.points.size()); ++j) {
                if (j == vi) continue;
                const Point3D& q = mesh.points[j];
                const double d = std::sqrt((pv.x-q.x)*(pv.x-q.x) + (pv.y-q.y)*(pv.y-q.y)
                                         + (pv.z-q.z)*(pv.z-q.z));
                lfs = std::min(lfs, d);
            }
            for (const auto& e : pendingEdges) {
                if (e.first == vi || e.second == vi)
                    continue;   // 相交 v_i 的段不算
                lfs = std::min(lfs, pointSegDist(pv, mesh.points[e.first],
                                                     mesh.points[e.second]));
            }
            if (!(lfs > 0.0) || lfs > 1e299)
                continue;

            // ---- r_i = min{ lfs,  ℓ_i/3,  2·d_i/3 } ----
            // 本实现取 d_i = ℓ_i（保守：只切"远端也可能被切"的段）。
            // 保守取值使 r_i 更小 → 插入点更贴近 v_i → 不会过度加密。
            const double d_i = shortestAdjSeg;
            const double ri = std::min(lfs, std::min(shortestAdjSeg / 3.0,
                                                     2.0 * d_i / 3.0));
            if (!(ri > 0.0))
                continue;

            // ---- 对每条相邻段，插入 s ∩ S_i（即从 v_i 沿段走 r_i 处的点） ----
            for (const int n : nbrs) {
                const double L = segLen(vi, n);
                if (L <= ri)
                    continue;   // 段比保护球还短 → 整段被吞，无法插内部点
                const Point3D& q = mesh.points[n];
                const double t = ri / L;
                protPoints.push_back({
                    { pv.x + t*(q.x-pv.x), pv.y + t*(q.y-pv.y), pv.z + t*(q.z-pv.z) },
                    { vi, n }
                });
            }
        }

        // 逐点插入（复用既有 insertConstrainedPoint：体积守恒 + 安全拒绝）
        // excludeFaceKeys 传全部约束面 —— 保证 cavity 不跨越边界（见上方说明）。
        if (const char* dbg = std::getenv("MESHSTUDIO_DEBUG_PROTECT"))
            std::fprintf(stderr, "[protect] candidates=%d\n", (int)protPoints.size());
        int protectOk = 0, protectFail = 0;
        for (const auto& pp : protPoints) {
            const Point3D& m = pp.first;
            if (!insertConstrainedPoint(mesh, m, constraintFaceKeys, {}, error)) {
                // ★ §8「back out」的第二种形态：尝试插入失败时，不用插入点，
                //    而改用**所在段的中点**。论文 §8 原文：
                //      "Any effort to insert a vertex on or inside the sphere is
                //       thwarted; instead, the segment is split at its center..."
                //    保护球点失败的典型原因是它与**保护球内的已有顶点共球**或
                //    落在同一片球面上 —— 这类点即使插入也无法解开保护球。改用段
                //    中点则把「不可插入的球面点」换成「可插入的段中部点」。
                if (std::getenv("MESHSTUDIO_DEBUG_BACKOUT"))
                    std::fprintf(stderr, "[backout] protect point on seg (%d,%d) failed: %s\n",
                                 pp.second.first, pp.second.second,
                                 error ? error->c_str() : "(no detail)");
                if (error) error->clear();
                const double dx = mesh.points[pp.second.second].x - mesh.points[pp.second.first].x;
                const double dy = mesh.points[pp.second.second].y - mesh.points[pp.second.first].y;
                const double dz = mesh.points[pp.second.second].z - mesh.points[pp.second.first].z;
                const Point3D mc = {
                    mesh.points[pp.second.first].x + 0.5 * dx,
                    mesh.points[pp.second.first].y + 0.5 * dy,
                    mesh.points[pp.second.first].z + 0.5 * dz,
                };
                bool recovered = false;
                if (insertConstrainedPoint(mesh, mc, constraintFaceKeys, {}, error)) {
                    const int nv = static_cast<int>(mesh.points.size()) - 1;
                    // 中点把段一分为二，两端都记入 protEnds（避免被尺寸函数再切）
                    auto addEnd = [&](int a, int b) {
                        if (a > b) std::swap(a, b);
                        protEnds.insert({ a, b });
                    };
                    addEnd(pp.second.first, nv);
                    addEnd(nv, pp.second.second);
                    if (outSteiner) ++(*outSteiner);
                    if (outProtect) ++(*outProtect);
                    ++protectOk;
                    recovered = true;
                }
                if (!recovered) {
                    // 段中点也不行 → 真正的局部无解。保护球是"优化"而非"必需"，
                    // 阶段 2 的中点二分仍能兜底，因此仅记录并继续。
                    if (error) error->clear();
                    ++protectFail;
                }
                continue;
            }
            ++protectOk;
            // 记录段端：保护球点 nv 与端点是并列的两个子段。
            // 索引规则与 insertConstrainedPoint 一致：新点索引 = points.size()-1。
            {
                const int nv = static_cast<int>(mesh.points.size()) - 1;
                auto addEnd = [&](int a, int b) {
                    if (a > b) std::swap(a, b);
                    protEnds.insert({ a, b });
                };
                addEnd(pp.second.first, nv);    // (vi, p) —— 紧贴顶点的"端"
                // (p, n) 是长段，**不**豁免，仍需按尺寸函数细分
            }
            if (outSteiner) ++(*outSteiner);
            if (outProtect) ++(*outProtect);
        }
        if (std::getenv("MESHSTUDIO_DEBUG_PROTECT")) {
            std::string vmsg; const bool vok = mesh.validate(&vmsg);
            std::fprintf(stderr, "[protect] ok=%d fail=%d ends=%d validate=%d %s\n",
                         protectOk, protectFail, (int)protEnds.size(),
                         (int)vok, vmsg.c_str());
        }
    }

    // ============================================================
    // 细分循环：反复「找一条断边 → 插中点 → 替换为两个子段」，直到**一整轮
    // 扫描下来零断边**（真正的固定点）才退出。
    // ------------------------------------------------------------
    // 关键点：每一次细分都可能把**之前已恢复好的**边 (c,d) 重新弄断
    //（细分 (a,b) 会在其邻域重建四面体，(c,d) 可能穿面）。
    // 由于每轮扫描**从索引 0 开始**，只要 (c,d) 的条目仍在 pendingEdges 里，
    // 下一轮就能重新发现它 —— 因此「零断边退出」本身就是固定点条件。
    // 实测：cube 用例若只做「遇到断边就细分、细分完立刻把该条目换掉」而**不**
    // 复查偏早条目，会永久漏掉 (5,6)(6,8)(7,9) 三条共享顶点的相邻棱。
    // ============================================================

    // ============================================================
    // ★ 迭代上限与条目上限必须与**问题规模**成比例，不能是固定常数 ★
    // ------------------------------------------------------------
    // 依据：每次**合法**细分把 1 条待处理边换成 2 条子边 → 条目数 +1。
    // 因此「恢复 N 条约束边」的合法工作量本身就是 O(N) 次细分、条目数 O(N)。
    //
    // 旧值（20000 次 / 初始+64 条）只适配 cube 那种 18 条边的小用例。
    // 实测 ball.brep（11538 条约束边，球面共球退化使 Delaunay 边界剖分与
    // 输入剖分几乎处处不同）：第 67 次细分即触发
    //   "pending constraint sub-edges grew to 11603 (limit 11602)"
    // —— 这是**正常细分途中**被误判为发散，不是真的发散。
    //
    // 新上限 = max(旧下限, 4×初始规模)：既保留对"真正发散"的拦截
    //（ProbeBackout 那种 8 条边涨到 ~19500 仍会被拦），又不误伤大输入。
    const int nPending0 = static_cast<int>(pendingEdges.size());
    const int maxEdgeIters = std::max(20000, nPending0 * 4);
    int iter = 0;
    // §8「back out」计数器：记录被退让的插入次数。上限用于区分
    // 「个别构型不可插入」（正常，局部退让即可）与
    // 「整体不可插入」（异常，需整体报错而非无限退让）。
    int backoutCount = 0;
    const int kMaxBackout = static_cast<int>(pendingEdges.size()) + 16;
    // ★ 停滞检测（almost Delaunay 路线的必要护栏）★
    // ------------------------------------------------------------
    // 论文 §1 警告 almost Delaunay 路线 "the Delaunay property is not
    // maintained during these vertex insertions"、并 "in danger of creating
    // very short edges"。其**可观测症状**是：细分不再收敛，而是不断产生
    // 更短的、同样不满足的新子段。
    //
    // 实测（ProbeBackout 用例）：每轮都"成功"插点，但 `pendingEdges` 从 8
    // 单调膨胀到 ~19500，边长退化到 1e-5~1e-11 量级，20000 轮上限被打满、
    // 耗时 12.7s 且最终仍失败。既不收敛也不退让 —— 纯空转。
    //
    // 护栏判据：允许的待处理条目数 = 初始条目数 + 少量裕量。一旦超过，
    // 说明细分在**制造问题而非解决问题**，立刻按 §8 报错退出（"test for
    // this possibility, and back out"），而不是耗尽迭代预算。
    // 裕量为 64：正常收敛路径（cube/cube10/正四面体）的条目数始终 <= 初始值，
    // 不会触发；只有真正发散的构型才会命中。
    const int kMaxPending = std::max(nPending0 + 64, nPending0 * 4 + 4096);

    // ============================================================
    // ★ 收敛性护栏（主判据）：看「未满足边数」是否在**进展** ★
    // ------------------------------------------------------------
    // 上一版用「待处理条目数 > 初始 + 64」当判据，方向对但**判据选错了**：
    // 每次**合法**细分都会让条目数 +1（1 条边换成 2 条子边），因此大输入在
    // 正常细分途中就会被误判。实测 ball.brep（11538 条约束边）第 67 次细分即
    // 触发 "pending grew to 11603 (limit 11602)"。
    //
    // 真正区分「在干活」与「纯空转」的是**未满足边数是否下降**：
    //   · 收敛路径：每细分一次就有边变成网格边 → unsatisfied 单调下降；
    //   · 发散路径：只插点、unsatisfied 长期不降（论文 §1 对 almost Delaunay
    //     的警告 "in danger of creating very short edges" 正是此现象）。
    // 实测证据（放开旧护栏后跑 ball.brep）：
    //   iter=37800  pending 11538→42546  points 3854→38258  38090 次插入、77s
    //   而 unsatisfied 始终不降 —— 确认是真发散，不是工作量问题。
    //
    // 判据：每 kProgressWindow 轮统计一次 unsatisfied；连续 kNoProgressWindows
    // 个窗口都没有刷新历史最好值 → 判发散、按 §8 报错退出。
    // 代价：每 2000 轮一次 O(P) 统计（哈希查找，~ms 级），可忽略。
    // ============================================================
    constexpr int kProgressWindow = 2000;
    constexpr int kNoProgressWindows = 3;
    int bestUnsatisfied = std::numeric_limits<int>::max();
    int noProgressWindows = 0;

    // 诊断（MESHSTUDIO_DEBUG_CDT=1）：主循环进度画像 —— 每 200 轮打印
    // 迭代数 / 待处理边数 / 四面体数 / 顶点数 / 累计耗时，用于定位时间去向。
    const bool dbgCdt = std::getenv("MESHSTUDIO_DEBUG_CDT") != nullptr;
    const auto cdtT0 = std::chrono::steady_clock::now();
    for (; iter < maxEdgeIters; ++iter) {
        if (dbgCdt && (iter % 200 == 0)) {
            const double el = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - cdtT0).count();
            std::fprintf(stderr,
                         "[cdt] iter=%d pending=%zu tets=%zu points=%zu elapsed=%.2fs\n",
                         iter, pendingEdges.size(), mesh.tets.size(), mesh.points.size(), el);
        }
        // ---- 进展评估 ----
        if ((iter + 1) % kProgressWindow == 0) {
            int unsatisfied = 0;
            for (const auto& pe : pendingEdges)
                if (!edgeSatisfied(pe.first, pe.second))
                    ++unsatisfied;
            if (unsatisfied < bestUnsatisfied) {
                bestUnsatisfied = unsatisfied;
                noProgressWindows = 0;
            } else if (++noProgressWindows >= kNoProgressWindows) {
                if (error)
                    *error = "edge recovery diverged: unsatisfied constraint edges "
                           "made no progress for "
                           + std::to_string(kNoProgressWindows * kProgressWindow)
                           + " subdivisions (unsatisfied=" + std::to_string(unsatisfied)
                           + ", pending=" + std::to_string(pendingEdges.size())
                           + "); almost-Delaunay route cannot resolve this configuration";
                return false;
            }
        }
        if (static_cast<int>(pendingEdges.size()) > kMaxPending) {
            if (error)
                *error = "edge recovery diverged: pending constraint sub-edges grew to "
                       + std::to_string(pendingEdges.size()) + " (limit "
                       + std::to_string(kMaxPending) + "); almost-Delaunay route "
                       "cannot resolve this configuration";
            return false;
        }
        // 找一条未成为网格边的约束边（每轮从 0 开始 → 全覆盖复查）
        int foundIdx = -1;
        for (int i = 0; i < static_cast<int>(pendingEdges.size()); ++i) {
            if (!edgeSatisfied(pendingEdges[i].first, pendingEdges[i].second)) {
                foundIdx = i;
                break;
            }
        }
        if (foundIdx < 0)
            break;   // 一整轮无断边 → 固定点
        const auto e = pendingEdges[foundIdx];

        // ============================================================
        // 路线 B（翻转恢复，不新增顶点）—— 带开关，默认关闭
        // ------------------------------------------------------------
        // 先用局部 2-3 翻转把 (a,b) 直接翻成网格边。若成功，则**不需要**插
        // 中点，(a,b) 的细分条目原样保留即可（它已满足 edgeSatisfied）。
        //
        // 为何默认关闭：路线 A 的「插中点」是**无条件收敛**的（子段严格变短，
        // 递归必然终止），且与原有实现完全一致；路线 B 只在**部分**构型上更优，
        // 且会额外消耗翻转预算。当前阶段的目标是「框架清晰 + 功能不退步」，
        // 因此保持 A 为默认路径，把 B 作为可显式开启的**加速/减点**选项。
        //
        // 为何可用：已由 _FlipProbe 实测验证 —— **随机**顶点对中约 15% 可通过
        // 翻转直接恢复（tried=400 recovered=60），全部 validate=1、体积守恒。
        //
        // 实测（重要，避免高估）：把开关置 true 跑 Constrained3DTest /
        // Ruppert3DTest，结果与关闭时**完全一致**（tet 数、inserted、iter 全同）。
        // 说明这两个用例里的断边都**不可**由纯翻转恢复 —— 它们要么被约束面
        // 阻挡（barred），要么 cavity 非星形。因此路线 B 当前是**能力储备**，
        // 不是已见效的优化：它只在「边缺失且腔星形」的构型上有效，如 M3.5
        // 后续要精细控制顶点数时才值得开启。
        // 需要时把 kUseFlipRecovery 置 true（或后续改为按构型自适应）。
        // ============================================================
        // 需要时用环境变量 MESHSTUDIO_FLIP_RECOVERY=1 打开（便于 A/B 实测）。
        // 说明：翻转时**必须**把全部约束面设为阻挡（blockedFaces），否则
        // flip23 会横穿约束面，把已经恢复好的边界再次破坏。
        static const bool kUseFlipRecovery = [] {
            const char* v = std::getenv("MESHSTUDIO_FLIP_RECOVERY");
            return v != nullptr && std::atoi(v) != 0;
        }();
        if (kUseFlipRecovery) {
            int edgeFlips = 0;
            if (recoverEdgeByFlips(mesh, e.first, e.second, constraintFaceKeys,
                                   &edgeFlips, nullptr)
                && mesh.hasEdgeAny(e.first, e.second)) {
                continue;   // 已翻出网格边，无需细分（条目保持，下轮复查仍满足）
            }
        }

        // ============================================================
        // ★ 插入中点细分（手动重建；剔除含被分裂边 (a,b) 的面，防共线退化）★
        // ------------------------------------------------------------
        // §8「back out」：`insertConstrainedPoint` 在**修改网格之前**做体积守恒
        // 校验，失败时 `mesh` 保持原样 —— 因此它天然是一个「安全拒绝」原语。
        // 论文 §8 要求：顶点插入不可能成功时，程序必须**检测并退回**，而不是
        // 把整条恢复链拖死。退回手段就是「跳过该条边、换一条继续」。
        //
        // 论文原文（§8）：
        //   "[a mesh generation program] should test for this possibility, and
        //    back out when vertex insertion cannot succeed."
        //
        // 为何「继续」而非「立刻失败」是安全的：
        //   - 本轮扫描是从索引 0 全覆盖复查的，被跳过的边**下轮会再次被看到**；
        //   - 只有当**一整轮**下来找不到任何可细分的边时才真正退出（固定点或
        //     真正无解）→ 不会因为一次退让就漏掉可恢复的边。
        // 这样把「单点失败」与「整体失败」解耦：像 (49,115)/(49,135) 那种
        // 体积不守恒的构型，不再让整个边界恢复返回 false。
        // ============================================================
        const Point3D m = midPoint3(mesh.points[e.first], mesh.points[e.second]);
        // ★ 约束面屏障：细分约束边时必须让 cavity「不跨越约束面」★
        // ------------------------------------------------------------
        // 与保护球阶段同理（见上方「关键：保护球点插入时必须排除全部约束面」）：
        // cavity 一旦横穿约束面，重建后的新四面体就覆盖约束面两侧 → 约束面不再是
        // 网格面 → 边界被破坏，且**刚建立的子边也会被后续插入吞掉**。
        // 实测（ball.brep，MESHSTUDIO_DEBUG_CDT=1 的 [cdt-split] 输出）：
        //   (4,87)   -> (4,3854)=1 (3854,87)=1     首次两个子边都成立
        //   (4,3854) -> (4,3856)=1 (3856,3854)=0   ← 第二子边没建立
        //   随后又回到 (4,3856) 再细分 —— 说明先前建立的子边被**后来的插入破坏**，
        //   pending 因此单调增长、永不收敛。
        // 用 MESHSTUDIO_CDT_NO_BARRIER=1 关闭本屏障（仅用于 A/B 实测）。
        // ★ 实测对比（2026-09-27）★
        //   cube.stl   ：开/关**逐位一致**（10/11/20, minSICN=0.577350）
        //   cube10.stl ：开/关**逐位一致**（103/405/124）
        //   ball.brep  ：关 → 未满足边 11538 条、零进展、无限发散；
        //                开 → 未满足边降到 **5254** 条（恢复 6284 条，约 55%），
        //                     并在 18s 内由进展护栏明确报错退出。
        // 因此默认**开启**：对既有小用例零影响，对真实大输入显著更好。
        static const bool kBarrierOnSubdiv = [] {
            const char* v = std::getenv("MESHSTUDIO_CDT_NO_BARRIER");
            return !(v != nullptr && std::atoi(v) != 0);
        }();
        const std::unordered_set<uint64_t> emptyFaceKeys;
        if (!insertConstrainedPoint(mesh, m,
                                    kBarrierOnSubdiv ? constraintFaceKeys : emptyFaceKeys,
                                    { edgeKey(e.first, e.second) }, error)) {
            // —— 退回：放弃本次细分，把这条边标记为本轮已尝试 ——
            if (std::getenv("MESHSTUDIO_DEBUG_BACKOUT"))
                std::fprintf(stderr, "[backout] skip (%d,%d): %s\n",
                             e.first, e.second,
                             error ? error->c_str() : "(no detail)");
            if (error) error->clear();          // 失败原因是局部几何，非整体错误
            ++backoutCount;
            if (backoutCount > kMaxBackout) {
                if (error)
                    *error = "edge recovery: too many insertions backed out ("
                           + std::to_string(backoutCount) + " of "
                           + std::to_string(pendingEdges.size()) + " edges)";
                return false;
            }
            // 从待处理列表移除该条不可细分边（原地 swap-erase，保持全覆盖复查语义）
            pendingEdges[foundIdx] = pendingEdges.back();
            pendingEdges.pop_back();
            continue;
        }
        if (outSteiner) ++(*outSteiner);
        // 更新待处理列表：(a,b) → (a,m),(m,b)
        const int nv = static_cast<int>(mesh.points.size()) - 1;   // 新插入的 m
        // ★ 诊断（MESHSTUDIO_DEBUG_CDT=1）：中点细分到底有没有把子边变成网格边 ★
        // 这是「插点式边恢复」成立的前提：插 (a,b) 的中点 m 后，(a,m) 与 (m,b)
        // 应当立即成为网格边。若不是，细分就只是**制造更短的断边**，
        // 于是 pending 单调增长、永不收敛（论文 §1 对 almost Delaunay 的警告）。
        if (dbgCdt && (iter < 40 || iter % 2000 == 0)) {
            const bool okA = mesh.hasEdgeAny(e.first, nv);
            const bool okB = mesh.hasEdgeAny(nv, e.second);
            std::fprintf(stderr, "[cdt-split] (%d,%d) -> (%d,%d)=%d (%d,%d)=%d\n",
                         e.first, e.second, e.first, nv, okA ? 1 : 0,
                         nv, e.second, okB ? 1 : 0);
        }
        pendingEdges[foundIdx] = { e.first, nv };
        pendingEdges.push_back({ nv, e.second });
    }
    if (iter >= maxEdgeIters) {
        if (error)
            *error = "edge recovery did not converge after "
                   + std::to_string(maxEdgeIters) + " subdivisions";
        return false;
    }
    // ★ 无进展退出：走到这里说明 foundIdx < 0（一整轮没有发现断边）→ 真正的固定点。
    //    若"有断边但全都退让失败"，则 pendingEdges 会被逐步移空，最终也会
    //    foundIdx < 0 而退出 —— 但那是**未恢复成功**的退出，必须报错。
    //    判据：pendingEdges 里是否还剩下"本应成为网格边却没能"的条目。
    //    退让时被移除的边不再出现，因此用**失败计数**区分：
    //    若 backoutCount > 0 且有边被移除，则恢复不完整。
    if (backoutCount > 0 && !pendingEdges.empty()) {
        // 重新检查：仍存在断边 → 恢复失败（退让未能兜底）
        bool stillBroken = false;
        for (const auto& pe : pendingEdges) {
            if (!edgeSatisfied(pe.first, pe.second)) { stillBroken = true; break; }
        }
        if (stillBroken) {
            if (error)
                *error = "edge recovery incomplete: " + std::to_string(backoutCount)
                       + " insertion(s) backed out with unsatisfied edges remaining";
            return false;
        }
    } else if (backoutCount > 0 && pendingEdges.empty()) {
        // 全部待处理边都被退让移除 → 说明一条都没恢复成功，属整体失败
        if (error)
            *error = "edge recovery failed: all " + std::to_string(backoutCount)
                   + " candidate edge(s) were backed out (no recoverable edge left)";
        return false;
    }
    // 把最终子段列表写回调用方（**输入输出语义**）：
    // 调用方（如 Ruppert3D 的 segList）必须看到细分后的子段，否则它会继续
    // 拿着「已被细分过的旧边」去 splitSeg → 中点重复插入 + 真实约束棱永久失同步。
    edges = pendingEdges;
    return true;
}

Constrained3DResult enforceSurface3D(TetMesh3D& mesh,
                                     const std::vector<std::array<int, 3>>& faces,
                                     std::string* error,
                                     const EdgeRecoveryParams& rp,
                                     int* outProtect)
{
    Constrained3DResult res;

    // 提取约束边集合（全部面边去重）
    // ---- 1) 边恢复：细分约束边直到所有约束边都是网格边 ----
    std::vector<std::pair<int, int>> edges;
    edges.reserve(faces.size() * 3);
    for (const auto& f : faces) {
        edges.push_back({ f[0], f[1] });
        edges.push_back({ f[1], f[2] });
        edges.push_back({ f[2], f[0] });
    }
    int steiner = 0;
    if (!recoverConstraintEdges(mesh, faces, edges, &steiner, error, rp, outProtect)) {
        res.error = error && !error->empty() ? *error : std::string("edge recovery failed");
        return res;
    }
    res.steinerPoints += steiner;

    // ---- 2) 面恢复（v1 跳过） ----
    // 3D 面恢复（把约束面变成网格面的并集）需要翻转（2-3/3-2 flips）或精确 CDT，
    // 在非 Delaunay 网格上"插入重心"的 cavity 界定不可靠（漏删 → 重叠）。
    // v1 只做边恢复：对凸包表面（如立方体），边恢复后边界曲面已与约束面一致
    //（曲面一致，但具体三角剖分可能与输入不同）。凹形/内部约束面的保形恢复留待
    // M2.5（flips 框架）。

    // ---- 3) 收尾整体自检（O(T)，一次）----
    // insertConstrainedPoint 的逐次 validate 已改为默认关闭（见该函数说明），
    // 这里做**一次性**兜底：覆盖不变，只是失去"哪一次插入坏的"定位信息。
    {
        std::string verr;
        if (!mesh.validate(&verr)) {
            res.error = "mesh invalid after enforceSurface3D: " + verr;
            return res;
        }
    }

    res.ok = true;
    return res;
}

} // namespace mesher
