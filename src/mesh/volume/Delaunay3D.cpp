// 文件职责：三维 Delaunay 插点：用球内判定寻找空腔，替换受影响四面体并重建邻接。
#include "Delaunay3D.h"

#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <unordered_set>
#include <utility>

namespace mesher {

namespace {

// 步行定位：从 start 出发，沿"点在有向面外侧"跨越邻居，直到 p 在四面体所有面内侧。
// 返回包含 p 的四面体 id；p 落在网格覆盖区外或步数超限返回 -1。
int locateTet(const TetMesh3D& mesh, const Point3D& p, int start,int pointId)
{
    if (start < 0 || start >= static_cast<int>(mesh.tets.size()))
        start = 0;
    int t = start;
    const int maxSteps = 2000000;
    for (int step = 0; step < maxSteps; ++step) {
        const Tet4& tet = mesh.tets[t];
        bool inside = true;
        for (int e = 0; e < 4; ++e) {
            // 面 (v[e+1],v[e+2],v[e+3])，v[e] 是面外侧顶点。
            // orient3d 对循环置换变号、偶置换不变号，不同面的 v[e] 侧符号不同，
            // 故用"p 与 v[e] 同侧 → 内侧"的同号判定（与符号约定无关，绝对正确）。
            const Point3D& A = mesh.points[tet.v[(e + 1) % 4]];
            const Point3D& B = mesh.points[tet.v[(e + 2) % 4]];
            const Point3D& C = mesh.points[tet.v[(e + 3) % 4]];
            const Point3D& V = mesh.points[tet.v[e]];
            const int a=tet.v[(e+1)%4],b=tet.v[(e+2)%4],c=tet.v[(e+3)%4];
            if (orient3dSoS(A,B,C,V,a,b,c,tet.v[e]) * orient3dSoS(A,B,C,p,a,b,c,pointId) < 0) {
                const int nb = tet.n[e];
                if (nb < 0)
                    return -1;   // 越界：p 在网格覆盖区外
                t = nb;
                inside = false;
                break;
            }
        }
        if (inside)
            return t;
    }
    return -1;
}

} // namespace

/// 单点插入既有网格（细化阶段复用，坐标用原始坐标系）。
bool insertPoint3D(TetMesh3D& mesh, const Point3D& p,
                   const std::unordered_set<uint64_t>& blockedFaces,
                   int startTet, int* outNewVertex, int* outTet,
                   std::string* error)
{
    if (mesh.tets.empty()) {
        if (error) *error = "insertPoint3D: empty mesh";
        return false;
    }

    const int newV = mesh.addPoint(p);
    if (outNewVertex) *outNewVertex = newV;
    if (outTet) *outTet = -1;

    // 1) 定位包含 p 的四面体（walk 失败则换起点重试，再暴力搜索回退）
    int t = locateTet(mesh, p, startTet,newV);
    if (t < 0)
        t = locateTet(mesh, p, 0,newV);
    if (t < 0) {
        for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i) {
            const Tet4& tet = mesh.tets[i];
            bool inside = true;
            for (int e = 0; e < 4; ++e) {
                const Point3D& A = mesh.points[tet.v[(e + 1) % 4]];
                const Point3D& B = mesh.points[tet.v[(e + 2) % 4]];
                const Point3D& C = mesh.points[tet.v[(e + 3) % 4]];
                const Point3D& V = mesh.points[tet.v[e]];
                const int a=tet.v[(e+1)%4],b=tet.v[(e+2)%4],c=tet.v[(e+3)%4];
                if (orient3dSoS(A,B,C,V,a,b,c,tet.v[e]) * orient3dSoS(A,B,C,p,a,b,c,newV) < 0) {
                    inside = false;
                    break;
                }
            }
            if (inside) { t = i; break; }
        }
    }
    if (t < 0) {
        mesh.points.pop_back();
        if (outNewVertex) *outNewVertex = -1;
        if (error) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "insertPoint3D: point (%.6f, %.6f, %.6f) outside region (tets=%zu, start=%d)",
                          p.x, p.y, p.z, mesh.tets.size(), startTet);
            *error = buf;
        }
        return false;
    }

    // 3) **星形性闭包**（关键修复）：
    //    Bowyer-Watson 要求 cavity 关于 p 星形，即每条边界面 (a,b,c) 都能被 p
    //    "看见"（cavity 侧顶点 opp 与 p 在该面同侧）。若按外接球判据 (insphere>0)
    //    收集的 cavity 不满足该条件（实测：立方体第 5 个角点插入时出现），
    //    重建会在该面同侧生成重叠四面体 → 面哈希被 addTetFaces 的
    //    `m_face[key] = ...` 静默覆盖 → 不变量 4·T 破坏。
    //    处理：把不可见面外侧的邻居也纳入 cavity，迭代直到全部边界面可见。
    //    这等价于把 cavity 扩张到「p 的星形闭包」，是标准的 cavity 修复手段。
    //    （注：约束面（blockedFaces）不可跨越，遇阻挡则放弃本次插入。）
    std::vector<char> inCav(mesh.tets.size(), 0);
    std::vector<int> cavity;
    cavity.push_back(t);
    inCav[t] = 1;

    // 约束面判定（无序 key），供扩张与 BFS 共用
    auto isBlocked = [&](int x, int y, int z) -> bool {
        if (blockedFaces.empty()) return false;
        if (x > y) std::swap(x, y);
        if (y > z) std::swap(y, z);
        if (x > y) std::swap(x, y);
        const uint64_t fk = (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 42)
                          | (static_cast<uint64_t>(static_cast<uint32_t>(y)) << 21)
                          | static_cast<uint32_t>(z);
        return blockedFaces.count(fk) != 0;
    };

    // 3a) 外接球扩张（BFS）：p 严格在外接球内部的相邻四面体纳入 cavity
    {
        std::vector<int> queue;
        queue.push_back(t);
        while (!queue.empty()) {
            const int ct = queue.back();
            queue.pop_back();
            const Tet4& tet = mesh.tets[ct];
            for (int e = 0; e < 4; ++e) {
                const int nb = tet.n[e];
                if (nb < 0 || inCav[nb])
                    continue;
                if (isBlocked(tet.v[(e + 1) % 4], tet.v[(e + 2) % 4], tet.v[(e + 3) % 4]))
                    continue;
                const Tet4& nt = mesh.tets[nb];
                if (insphereSoS(mesh.points[nt.v[0]], mesh.points[nt.v[1]],
                                mesh.points[nt.v[2]], mesh.points[nt.v[3]], p,
                                nt.v[0], nt.v[1], nt.v[2], nt.v[3], newV) > 0) {
                    inCav[nb] = 1;
                    cavity.push_back(nb);   // 关键：必须同步登记进 cavity 列表
                    queue.push_back(nb);
                }
            }
        }
    }

    // 3b) 星形性闭包：反复把「不可见边界面」外侧的邻居纳入 cavity，直到稳定
    //     安全护栏：闭包体积不得超过网格的 60%（病态情形下闭包会失控吞掉整网格）
    const size_t cavLimit = static_cast<size_t>(mesh.tets.size() * 3 / 5) + 8;
    for (int guard = 0; guard < 4096; ++guard) {
        int grew = 0;
        for (size_t ci = 0; ci < cavity.size(); ++ci) {
            const int ct = cavity[ci];
            const Tet4& tet = mesh.tets[ct];
            for (int e = 0; e < 4; ++e) {
                const int nb = tet.n[e];
                if (nb < 0 || inCav[nb])
                    continue;
                const int a = tet.v[(e + 1) % 4], b = tet.v[(e + 2) % 4], c = tet.v[(e + 3) % 4];
                if (isBlocked(a, b, c))
                    continue;
                // 面 (a,b,c) 是否被 p 看见：opp 侧与 p 侧同号即"可见"
                const int opp = tet.v[e];
                const int sOpp = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                             mesh.points[opp], a, b, c, opp);
                const int sP = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                           p, a, b, c, newV);
                if (sOpp * sP < 0) {
                    // 不可见 → 把外侧邻居拉进 cavity
                    inCav[nb] = 1;
                    cavity.push_back(nb);
                    ++grew;
                }
            }
        }
        if (grew == 0)
            break;
        if (cavity.size() > cavLimit) {
            // 闭包失控：放弃本次插入（安全拒绝），网格保持原状
            mesh.points.pop_back();
            if (outNewVertex) *outNewVertex = -1;
            if (outTet) *outTet = -1;
            return true;
        }
    }

    // ============================================================
    // 3b-2) 简单连通 + 星形完整性校验（防重叠覆盖）
    // ------------------------------------------------------------
    // 背景：cube 用例中曾出现**体积增大** +4.1e-5 —— 经 _OverlapProbe 定位为
    // 2 个四面体被 3 个四面体覆盖（重叠），且其体积之和恰等于漂移量。
    // 成因：上述 BFS + 星形闭包在退化/共球情形下可能产生**非简单连通**的
    // cavity（例如两块区域仅通过一条边/一个点相接，或闭包后仍有内部小腔），
    // 此时「cavity 边界面」构成的曲面不是**单连通闭曲面**：重建 (p, 边界面)
    // 会在该处生成**重叠**的新四面体。
    // 这种重叠的面哈希仍然互补（拓扑自洽）、顶点也全在域内，故 validate
    // 与体积外的几何检查都抓不到 —— 只有逐点覆盖计数（本探针）能发现。
    //
    // 判据：对 cavity 内每个四面体 t，其「朝向 p 的三个侧面」必须**每个都**
    // 出现在边界面集合中（即这三个面都对 cavity 外部开放）。若某个侧面
    // 的对侧仍在 cavity 内，则 t 是 cavity 的"内部"四面体 —— 这本身正常
    // （简单连通的星形 cavity 允许内部 t）。因此真正可靠的判据是：
    // **边界面集合必须恰好构成一个封闭曲面，即每条有向边出现且仅出现一次**。
    // 重叠/非单连通腔会使某条边出现次数 >1 或 <1（漏面）。
    // ============================================================

    // 3b-2) 边界面「对 p 可见性」校验（防重叠覆盖）—— **可跳过**判据
    // ------------------------------------------------------------
    // 事实基础（_OverlapProbe 实测）：cube 用例 refine 后出现 2 个四面体被 3 个
    // 覆盖（重叠），其体积之和恰等于体积漂移 +4.1e-5。成因是某次插入的 cavity
    // 给 p 留下了**看不见**的边界面 → 以 p 与该面组成的棱锥翻到 cavity 之外，
    // 与其它新增/既有四面体重叠。
    //
    // 判据：对每个边界面 f=(a,b,c)，其**原对侧顶点** opp 与 p 必须位于 f 的
    //       同侧（同侧 ⇒ p 看得见 f ⇒ 棱锥落在 cavity 内 ⇒ 不与其它棱锥相交）。
    //
    // ===== 但是：为什么「不可见」不再等价于「拒绝」=====
    // _RejectProbe 实测（cube + 30 点，插入 p=(0.75,0.75,0.0)，z=0 边界面上）：
    // 历史诊断：定位到的起始四面体编号为 42。
    // 外接球邻域搜索后，空腔含 11 个四面体。
    //   after star closure: cavity=11   ← 闭包一轮未增长
    // 历史诊断：边界面 18 个，其中 2 个不满足星形条件。
    // 异常边界面：t42/e2，顶点 (7,42,6)，对顶点 32，方向符号 -1/+1，原方向值 -1.288e-01。
    // 异常边界面：t157/e2，顶点 (5,42,7)，对顶点 24，方向符号 -1/+1，原方向值 -1.250e-01。
    // 关键事实：detOpp = -1.288e-01 / -1.250e-01，**强非退化**（远不是平局）。
    // 且两个坏面的对侧顶点 (32, 24) **不在** cavity 内，故 3b 的闭包无法通过
    // 「把外侧邻居拉进来」消除它们 —— 星形闭包对这种情况本质无效。
    //
    // 根因：p 落在**约束面（z=0）上**，3a 的外接球 BFS 因 isBlocked 在 z=0 处
    // 被截断，得到的 cavity 只覆盖边界面的**内侧一半**。此时 p 与那个只从单侧
    // 贴住它的边界面（顶点环里带着 v42 这样的同伦顶点）**几何上本就不同侧**：
    // 棱锥 (p,f) 是该腔的**正常**组成部分，不是重叠。把它当错误拒掉，就产生
    // 了实测的拒绝风暴（ins=299847、T 停滞在 350）—— 因为 Ruppert 的 B 段
    // 每轮都会重新选中同一个最坏四面体、算出同一个球心、再次被拒，死循环。
    //
    // 因此可见性只应作为**首选判据**：不满足时不能直接拒，而要走下面的
    // 3b-3「体积守恒保底」——真正能判定重叠的是「新棱锥体积之和 == 被删
    // cavity 体积之和」，它对合法开腔成立、对真重叠不成立。
    //
    // 用 SoS 谓词（与 validate / faceVertex 同一约定）保证退化平局也确定性判号。
    // 注意：必须用与下面 3c 相同的定向约定取「f 的原始朝向」，故这里直接取
    // cavity 内该四面体的面环序 (e+1,e+2,e+3) 与对侧顶点 v[e]。
    // ============================================================
    bool visibilityOk = true;
    {
        for (const int ct : cavity) {
            const Tet4& tet = mesh.tets[ct];
            for (int e = 0; e < 4; ++e) {
                const int nb = tet.n[e];
                if (nb >= 0 && inCav[nb])
                    continue;   // 内部面，不是边界面
                const int a = tet.v[(e + 1) % 4];
                const int b = tet.v[(e + 2) % 4];
                const int c = tet.v[(e + 3) % 4];
                const int opp = tet.v[e];
                const int sOpp = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                             mesh.points[opp], a, b, c, opp);
                const int sP = orient3dSoS(mesh.points[a], mesh.points[b], mesh.points[c],
                                           p, a, b, c, newV);
                if (sOpp * sP < 0) {   // 异侧 → p 看不见该面 → 棱锥穿出 → 可能重叠
                    visibilityOk = false;
                    break;
                }
            }
            if (!visibilityOk)
                break;
        }
    }

    // 3c) 收集 cavity 边界面（对侧不在 cavity），定向为 (p,a,b,c) 右手系
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
            // 定向：使新四面体 (p,a,b,c) 为右手系。
            // 用 **SoS 谓词**（orient3dSoS）：p 与面 (a,b,c) 共面时按顶点索引
            // 确定性打破平局（绝不返回 0），与 TetMesh3D::faceVertex/validate
            // 共用同一套约定 → 即便生成零体积单元，其面哈希也与邻居严格互补
            // （见 tests/_FaceHashProbe.cpp）→ 拓扑不变量不被破坏。
            // 零体积单元由 pruneDegenerateTets() 在插入完成后统一清除。
            const int s = orient3dSoS(p, mesh.points[a], mesh.points[b], mesh.points[c],
                                      newV, a, b, c);
            if (s > 0)
                boundary.push_back({ a, b, c });
            else
                boundary.push_back({ a, c, b });
        }
    }

    // 3d) 体积守恒保底校验（visibilityOk==false 时的最终裁决）
    // ------------------------------------------------------------
    // 原理：Bowyer-Watson 的替换必须是**测度等价**的 ——
    // 要求原空腔体积与新节点连接全部边界面后形成的体积相等。
    // 对每个**合法** cavity（含 p 在约束面上的开腔）该等式在浮点精度内成立；
    // 一旦发生重叠覆盖，新棱锥之和会**超过** cavity 之和，差值恰为重叠量
    //（_OverlapProbe 实测的 +4.1e-5 即此）。
    // 这是唯一一个对「开腔/边界点」也成立的精确判据，故用它做最后兜底，
    // 避免可见性判据在约束面附近过严导致拒绝风暴。
    // 阈值取相对量：1e-9 * max(1, |volCav|)。合法插入的浮点差在 1e-16 量级，
    // 真重叠的量级是 1e-5，两者相差 7 个数量级，阈值极安全。
    // 注意：这里**不影响已提交的网格** —— 若判定失败则在删 cavity 之前退出。
    // ============================================================
    if (!visibilityOk) {
        double volCav = 0.0;
        for (const int ct : cavity) {
            const Tet4& tet = mesh.tets[ct];
            volCav += orient3dDet(mesh.points[tet.v[0]], mesh.points[tet.v[1]],
                                  mesh.points[tet.v[2]], mesh.points[tet.v[3]]) / 6.0;
        }
        double volNew = 0.0;
        for (const Face& f : boundary) {
            volNew += orient3dDet(p, mesh.points[f.a], mesh.points[f.b], mesh.points[f.c]) / 6.0;
        }
        const double tol = 1e-9 * std::max(1.0, std::fabs(volCav));
        if (std::fabs(volCav - volNew) > tol) {
            // 真重叠（或腔度量不等价）→ 安全拒绝，网格保持原状
            mesh.points.pop_back();
            if (outNewVertex) *outNewVertex = -1;
            if (outTet) *outTet = -1;
            return true;
        }
    }

    // 4) 孤立点防护（预防式）：若所有边界面都与 p 共面而被丢弃（极端退化），
    //    插入将不产生任何四面体 → newV 成为孤立点，破坏 Euler 计数。
    //    此时**在删除 cavity 之前**就放弃本次插入（安全拒绝），
    //    网格保持原状（removeTet 是 swap-erase 不可逆，故不能事后回滚）。
    if (boundary.empty()) {
        mesh.points.pop_back();
        if (outNewVertex) *outNewVertex = -1;
        if (outTet) *outTet = -1;
        return true;   // 本点无法插入（已在网格中或完全退化），不算失败
    }

    // 5) 删除 cavity（从大到小，swap-erase 安全）
    std::sort(cavity.begin(), cavity.end(), std::greater<int>());
    for (const int ct : cavity)
        mesh.removeTet(ct);

    // 6) 新四面体 (newV, a, b, c)，addTet 自动与相邻四面体互连
    int newTet = -1;
    for (const Face& f : boundary)
        newTet = mesh.addTet(newV, f.a, f.b, f.c);

    if (outTet) *outTet = newTet;
    return true;
}

// ============================================================
// ============================================================
// ============================================================
// ============================================================
// pruneDegenerateTets —— 清除零体积四面体（共面/共球退化残留）
// ------------------------------------------------------------
// 背景：立方体 8 顶点共球（球心 (0.5,0.5,0.5)）。Delaunay 剖分不唯一，
// Bowyer-Watson 会插入零体积（扁平）四面体。cube 用例最终剩 2 个：
//   A: v=[11 9 7 5]  共面于 x = 1
//   B: v=[11 8 10 9] 共面于 z = 1
//
// 本例中它们**必须删除**，而不是保留：
//   以 B 为例（v=[11 8 10 9]，正方形 8-10-11-9 于 z=1）：
//     e0 (8,10, 9) → 邻 t1 (下侧)
//     e1 (10, 9,11) → 邻 t5 (下侧)
//     e2 ( 9,11, 8) → 无邻居
//     e3 (11, 8,10) → 无邻居
//   下侧邻居 t1、t5 **已经可以直接拼出 z=1 顶面**：
//     t1 的面 (8,10,9) 与 t5 的面 (10,9,11) 沿对角线 9-10 完整覆盖正方形。
//   B 只是一个**零体积占位薄片**：把它「顶面用另一条对角线（8-11）剖分」的
//   冗余信息挂在外面，导致该表面上出现两套不一致的三角形。
//   删除 B 后，t1 的 (8,10,9) 与 t5 的 (10,9,11) 自然晋升为边界面（表面），
//   恰好补全顶面；而 B 自带的冗余两片 (9,11,8)、(11,8,10) 一并消失。
//
// 因此删除是**充分且必要**的：
//   * 体积为 0 ⇒ 删除不改变总体积、覆盖区域；
//   * removeTetFaces 把仍被下侧邻居使用的两个面（e0/e1）的哈希项
//     转让给邻居，把两个冗余面（e2/e3）擦除 —— 语义完全正确；
//   * 不存在「留洞」：因为「洞」的那两片本来就被 t1/t5 占着。
//
// 早先版本曾加过「退化 tet 若有边界面就保留」的护栏，那是**错误**的：
// 它恰好保留了这两个必须删除的占位薄片，导致 validate 一直报
// "tet is degenerate (zero volume)"。现已移除。
// ============================================================

int pruneDegenerateTets(TetMesh3D& mesh, std::string* error)
{
    (void)error;
    int removed = 0;
    for (int pass = 0; pass < 1000; ++pass) {
        // 找第一个退化（零体积）四面体
        //
        // 精确零才可删除；正体积薄片留给质量诊断，不能通过清理丢失实体。
        int bad = -1;
        for (int t = 0; t < static_cast<int>(mesh.tets.size()); ++t) {
            const Tet4& tet = mesh.tets[t];
            const Point3D& a = mesh.points[tet.v[0]];
            const Point3D& b = mesh.points[tet.v[1]];
            const Point3D& c = mesh.points[tet.v[2]];
            const Point3D& d = mesh.points[tet.v[3]];
            if (orient3d(a,b,c,d)==0) {
                bad = t;
                break;
            }
        }
        if (bad < 0)
            break;   // 已无退化四面体

        // 直接删除。removeTet 会：
        //   * removeTetFaces：把「仍被邻居使用」的面的哈希项转让给邻居；
        //     把「无人使用」的面擦除；
        //   * 清除邻居对 bad 的引用；
        //   * swap-erase 迁移最后一个 tet 并重建其邻居引用与哈希项。
        mesh.removeTet(bad);

        ++removed;
    }
    return removed;
}

/// 对 point 集合生成 Delaunay 四面体化。点被追加进 mesh.points。
Delaunay3DResult delaunay3D(TetMesh3D& mesh, std::vector<Point3D> points,
                            bool verifyEachStep)
{
    Delaunay3DResult res;

    // 环境变量兜底开关（不改代码即可打开逐步自检，用于定位拓扑被哪一步弄坏）。
    // 两个变量都认：本模块专用的，以及全局统一的 MESHSTUDIO_VERIFY_EACH_STEP。
    if (!verifyEachStep) {
        const char* env = std::getenv("MESHSTUDIO_DELAUNAY_VERIFY_EACH_STEP");
        verifyEachStep = (env != nullptr) && (std::atoi(env) != 0);
        if (!verifyEachStep)
            verifyEachStep = TetMesh3D::verifyEachStep();
    }

    // ---- 去重（精确重合点会让 Bowyer-Watson 产生退化），保持输入顺序 ----
    {
        std::set<std::pair<double, std::pair<double, double>>> seen;   // (x,(y,z))
        std::vector<Point3D> unique;
        unique.reserve(points.size());
        for (const Point3D& p : points) {
            if (seen.insert({ p.x, { p.y, p.z } }).second)
                unique.push_back(p);
        }
        points = std::move(unique);
    }
    if (points.size() < 4) {
        res.error = "delaunay3D: need at least 4 distinct points";
        return res;
    }

    // ---- 归一化：平移到 bbox 中心、缩放到半尺寸 1 ----
    double minX = points[0].x, maxX = points[0].x;
    double minY = points[0].y, maxY = points[0].y;
    double minZ = points[0].z, maxZ = points[0].z;
    for (const Point3D& p : points) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
        minZ = std::min(minZ, p.z); maxZ = std::max(maxZ, p.z);
    }
    const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY), cz = 0.5 * (minZ + maxZ);
    const double scale = std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ)) * 0.5;
    const auto originalPoints=points;
    for (Point3D& p : points) {
        p.x = (p.x - cx) / scale;
        p.y = (p.y - cy) / scale;
        p.z = (p.z - cz) / scale;
    }

    // ---- 共面/共球退化处理：**不再扰动坐标** ----
    // 旧实现给坐标加 ±1e-8 随机扰动来破坏共球退化，代价是把精确的凸包顶点打散
    // （如立方体 8 顶点共球，扰动后棱被判为"非网格边"，边界恢复被迫插入 Steiner
    //  点并在非 Delaunay 网格上失配），且不可复现。
    // 现改为在**谓词层**做符号扰动（GeomPredicates.h 的 orient3dSoS/insphereSoS）：
    // 坐标保持精确，退化平局由确定性索引扰动打破。故此处不再改动 points。

    // ---- super 四面体：包裹 [-1,1]^3 的点集 ----
    const double S = 10.0;
    const int s0 = mesh.addPoint({ 0.0, 0.0, S });
    const int s1 = mesh.addPoint({ -S, -S, -S });
    int s2 = mesh.addPoint({ S, -S, -S });
    int s3 = mesh.addPoint({ 0.0, S, -S });
    if (orient3dDet(mesh.points[s0], mesh.points[s1], mesh.points[s2], mesh.points[s3]) < 0.0)
        std::swap(s2, s3);
    mesh.addTet(s0, s1, s2, s3);

    // ---- 逐个插入（按输入顺序：保证第 i 个输入点的网格索引为 i+4） ----
    // 说明：共球退化会临时产生零体积四面体。实测其拓扑（面哈希/邻居对称）
    // 完全自洽，故插入过程只校验面哈希不变量（快速），零体积单元由
    // 插入完成后的 pruneDegenerateTets() 统一清除，再做完整 validate。
    int startTet = 0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        std::string err;
        int inserted=-1;
        if (!insertPoint3D(mesh, points[i], {}, startTet, &inserted, &startTet, &err)) {
            res.error = "delaunay3D: step " + std::to_string(res.insertSteps)
                        + " (point #" + std::to_string(i) + "): " + err;
            return res;
        }
        if(inserted!=i+4) {
            res.error="delaunay3D: input point "+std::to_string(i)+" was rejected; original node identities cannot be preserved";
            return res;
        }
        ++res.insertSteps;
        // 快速拓扑不变量：面哈希与邻居声明必须自洽。
        // ★ 默认跳过 ★：这是 O(T) 的全网格自检，放在每次插入后会让整体
        // 复杂度退化成 O(n·T)（实测 3850 点从秒级 → 分钟级）。
        // 它只用于定位"哪一步坏了"，正确性由末尾的 mesh.validate() 兜底。
        // 需要时用 verifyEachStep / MESHSTUDIO_DELAUNAY_VERIFY_EACH_STEP 打开。
        if (verifyEachStep) {
            std::string ferr;
            if (!mesh.checkFaceHash(&ferr)) {
                res.error = "delaunay3D: face hash broken after step "
                            + std::to_string(res.insertSteps) + " (" + ferr + ")";
                return res;
            }
        }
    }

    // ---- 剔除 super 相关四面体 ----
    std::vector<int> toDelete;
    for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i) {
        const Tet4& tet = mesh.tets[i];
        for (int k = 0; k < 4; ++k) {
            if (tet.v[k] == s0 || tet.v[k] == s1 || tet.v[k] == s2 || tet.v[k] == s3) {
                toDelete.push_back(i);
                break;
            }
        }
    }
    std::sort(toDelete.begin(), toDelete.end(), std::greater<int>());
    for (const int id : toDelete)
        mesh.removeTet(id);

    // ---- 还原坐标（super 顶点无四面体引用，还原后成为孤立点，可忽略） ----
    for (Point3D& p : mesh.points) {
        p.x = p.x * scale + cx;
        p.y = p.y * scale + cy;
        p.z = p.z * scale + cz;
    }
    for(size_t i=0;i<originalPoints.size();++i) mesh.points[i+4]=originalPoints[i];

    // ---- 清除零体积（共面/共球退化）四面体 ----
    // 在原始坐标上以精确符号判断零体积。
    // 体积为 0 的单元删除不改变总体积（由 test 的体积守恒断言守护）。
    {
        std::string perr;
        const int pruned = pruneDegenerateTets(mesh, &perr);
        if (pruned > 0 && !perr.empty()) {
            res.error = "delaunay3D: pruneDegenerateTets failed: " + perr;
            return res;
        }
    }

    // ---- 收集有效四面体 ----
    for (int i = 0; i < static_cast<int>(mesh.tets.size()); ++i) {
        const Tet4& tet = mesh.tets[i];
        bool hasSuper = false;
        for (int k = 0; k < 4; ++k) {
            if (tet.v[k] == s0 || tet.v[k] == s1 || tet.v[k] == s2 || tet.v[k] == s3) {
                hasSuper = true;
                break;
            }
        }
        if (!hasSuper)
            res.tetIds.push_back(i);
    }

    std::string err;
    if (!mesh.validate(&err)) {
        res.error = "mesh invalid after delaunay3D: " + err;
        return res;
    }
    res.ok = true;
    return res;
}

} // namespace mesher
