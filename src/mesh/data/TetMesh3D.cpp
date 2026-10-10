// 文件职责：体网格容器：维护四面体连接、邻接与一致性检查；插点、恢复和细化策略由上层算法提供。
#include "TetMesh3D.h"

#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace mesher {

namespace {

/// 无向边 key：两个顶点索引各占 32 位（min<<32 | max），与顺序无关。
inline uint64_t edgeKeyOf(int a, int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}
// 有向面 key：三个顶点索引各占 21 位（顶点数 < 2^21 ≈ 209 万，超出会冲突）
inline uint64_t faceKey(int a, int b, int c)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 42)
         | (static_cast<uint64_t>(static_cast<uint32_t>(b)) << 21)
         | static_cast<uint32_t>(c);
}

// **规范（无向）面 key**：先排序三顶点，再取"最小循环旋转"作为环序首元素，
// 从而对齐到唯一表示。
//
// 为什么必须这样做：面哈希 m_face 要求「共享同一面的两个四面体算出**同一个**
// key」，否则无法配对。此前用「外法向环序」（依赖对侧顶点 opp 的 orient3d 符号）
// 来生成 key，在**退化情形**下失效 —— 当四面体零体积（四点共面）时，
// orient3d(a,b,c,opp)==0，落入 SoS 分支；而 SoS 的符号依赖完整的 4 元组
// (a,b,c,opp)，两个相邻四面体的 opp 不同 → 同一面得到不同符号 →
// key 不互补也不相同 → m_face 少项 → 不变量 4T 破坏（实测 cube 用例）。
//
// 规范 key 只依赖面自身的 3 个顶点，与 opp 无关，故对任意四面体、
// 任意退化程度都严格一致。邻居查找用规范 key 的逆序即可。
inline uint64_t canonicalFaceKey(int a, int b, int c)
{
    // 三次比较交换 → 升序 (a,b,c)
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return faceKey(a, b, c);
}
} // namespace

/// 按四面体局部面及面内位置查询顶点编号。
int TetMesh3D::faceVertex(int faceId, int k) const
{
    // 规范环序：仅由面自身三顶点决定（与对侧顶点无关），保证共享面的
    // 两个四面体得到完全相同的三元组。取「最小循环旋转」作为唯一表示，
    // 使 key 与四面体内部的 e 序号、顶点遍历顺序都无关。
    const int tetId = faceId / 4;
    const int e = faceId % 4;
    const Tet4& t = tets[tetId];
    int r[3] = { t.v[(e + 1) % 4], t.v[(e + 2) % 4], t.v[(e + 3) % 4] };
    // 最小旋转：在三个循环位移中取字典序最小者
    int best = 0;
    for (int s = 1; s < 3; ++s) {
        if (r[s] < r[best]) best = s;
        else if (r[s] == r[best]) {
            if (r[(s + 1) % 3] < r[(best + 1) % 3]) best = s;
            else if (r[(s + 1) % 3] == r[(best + 1) % 3]
                     && r[(s + 2) % 3] < r[(best + 2) % 3]) best = s;
        }
    }
    return r[(best + k) % 3];
}

/// 追加四面体并建立面索引和邻接关系。
int TetMesh3D::addTet(int a, int b, int c, int d)
{
    Tet4 t;
    t.v[0] = a;
    t.v[1] = b;
    t.v[2] = c;
    t.v[3] = d;
    const int id = static_cast<int>(tets.size());
    tets.push_back(t);

    // 逐面处理：**先查对侧**（查的是加入本四面体之前的哈希表状态），
    // 再登记本面。这样共享面能正确配对上已存在的邻居；
    // 边界面则查不到（或查到自己——本面尚未登记，绝不会查到自己）。
    for (int e = 0; e < 4; ++e) {
        const int face = id * 4 + e;
        const int x = t.v[(e + 1) % 4], y = t.v[(e + 2) % 4], z = t.v[(e + 3) % 4];
        const uint64_t key = canonicalFaceKey(x, y, z);
        auto it = m_face.find(key);
        if (it != m_face.end() && it->second / 4 != id) {
            // 已有邻居：双向建立邻居关系
            setNeighbor(face, it->second);
        }
        m_face[key] = face;
    }
    addTetEdges(id);
    return id;
}

/// 登记四面体的六条边。
void TetMesh3D::addTetEdges(int tetId)
{
    const Tet4& t = tets[tetId];
    for (int e = 0; e < 4; ++e)
        for (int f = e + 1; f < 4; ++f)
            ++m_edge[edgeKeyOf(t.v[e], t.v[f])];   // 默认 0 → 1
}

/// 撤销四面体的边登记。
void TetMesh3D::removeTetEdges(int tetId)
{
    const Tet4& t = tets[tetId];
    for (int e = 0; e < 4; ++e) {
        for (int f = e + 1; f < 4; ++f) {
            const auto it = m_edge.find(edgeKeyOf(t.v[e], t.v[f]));
            if (it == m_edge.end())
                continue;
            if (--(it->second) <= 0)
                m_edge.erase(it);   // 最后一个使用者消失 → 该边不再是网格边
        }
    }
}

/// 删除四面体并更新相邻单元和索引。
void TetMesh3D::removeTet(int tetId)
{
    const Tet4& t = tets[tetId];

    // 1) 擦除被删四面体贡献的有向面与边
    removeTetFaces(tetId);
    removeTetEdges(tetId);

    // 2) 被删四面体的邻居中，指回 tetId 的引用置 -1
    for (int e = 0; e < 4; ++e) {
        const int nb = t.n[e];
        if (nb < 0) continue;
        for (int e2 = 0; e2 < 4; ++e2) {
            if (tets[nb].n[e2] == tetId) { tets[nb].n[e2] = -1; break; }
        }
    }

    // 3) swap-erase：最后一个四面体搬到 tetId 位置，迁移邻居引用与面哈希
    const int lastId = static_cast<int>(tets.size()) - 1;
    if (lastId != tetId) {
        tets[tetId] = tets[lastId];
        const Tet4& mv = tets[tetId];
        for (int e = 0; e < 4; ++e) {
            const int oldFace = lastId * 4 + e;
            const int newFace = tetId * 4 + e;
            const int nb = mv.n[e];
            if (nb >= 0) {
                for (int e2 = 0; e2 < 4; ++e2) {
                    if (tets[nb].n[e2] == lastId) { tets[nb].n[e2] = tetId; break; }
                }
            }
            // 面 key 用规范环序（与 addTetFaces 一致），否则迁移找不到导致面哈希污染
            const uint64_t key = canonicalFaceKey(faceVertex(newFace, 0),
                                                  faceVertex(newFace, 1),
                                                  faceVertex(newFace, 2));
            auto it = m_face.find(key);
            if (it != m_face.end() && it->second == oldFace)
                it->second = newFace;
        }
    }
    tets.pop_back();
}

/// 查询指定四面体面的反向邻接面。
int TetMesh3D::oppositeFace(int faceId) const
{
    // faceVertex 返回**规范环序**（只由面自身三顶点决定，与对侧无关），
    // 因此共享同一面的两个四面体算出**完全相同**的 key。
    // 注意：命中项可能是**本面自己**（若本面是边界面，或查询时本面占用了哈希项）。
    // 调用方需自行判断 opp/4 是否等于本 tet。为便于使用，这里返回 -1 表示
    // 「没有对侧」——即命中的是本面自身。
    const int a = faceVertex(faceId, 0);
    const int b = faceVertex(faceId, 1);
    const int c = faceVertex(faceId, 2);
    auto it = m_face.find(canonicalFaceKey(a, b, c));
    if (it == m_face.end() || it->second == faceId)
        return -1;
    return it->second;
}

/// 查询共享边或共享面的反向邻接单元。
int TetMesh3D::findOpposite(int tetId, int e) const
{
    const int opp = oppositeFace(tetId * 4 + e);
    return (opp >= 0) ? opp / 4 : -1;
}

/// 检查三个节点是否组成已有四面体面。
bool TetMesh3D::hasFaceAny(int a, int b, int c) const
{
    // 规范 key 只依赖面三顶点集合，故一次查找即可（无需枚举 6 个排列）
    return m_face.count(canonicalFaceKey(a, b, c)) != 0;
}

/// 检查无向节点对是否组成已有网格边。
bool TetMesh3D::hasEdgeAny(int a, int b) const
{
    // O(1)：边哈希（增量维护，见 m_edge 说明）。
    // 注意：swap-erase 把最后一个四面体搬到被删位置时，其**顶点集合不变**，
    // 因此它的 6 条边无需迁移/重登记 —— 引用计数天然保持正确。
    return m_edge.find(edgeKeyOf(a, b)) != m_edge.end();
}

/// 查询网格边关联的四面体数量。
int TetMesh3D::edgeDegree(int a, int b) const
{
    const auto it = m_edge.find(edgeKeyOf(a, b));
    return it == m_edge.end() ? 0 : it->second;
}

/// 从面索引中移除四面体的各个面。
void TetMesh3D::removeTetFaces(int tetId)
{
    const Tet4& t = tets[tetId];
    for (int e = 0; e < 4; ++e) {
        const int face = tetId * 4 + e;
        const uint64_t key = canonicalFaceKey(t.v[(e + 1) % 4], t.v[(e + 2) % 4],
                                              t.v[(e + 3) % 4]);
        auto it = m_face.find(key);
        if (it == m_face.end() || it->second != face)
            continue;
        // 该面的哈希项属于本面 → 若存在邻居（该面被两个 tet 共享），
        // 把哈希项改指向邻居的那个面（否则邻居的面会"消失"）；
        // 若为边界面则直接擦除。
        const int nb = t.n[e];
        if (nb >= 0) {
            int nbFace = -1;
            for (int e2 = 0; e2 < 4; ++e2) {
                const Tet4& u = tets[nb];
                if (canonicalFaceKey(u.v[(e2 + 1) % 4], u.v[(e2 + 2) % 4],
                                     u.v[(e2 + 3) % 4]) == key) {
                    nbFace = nb * 4 + e2;
                    break;
                }
            }
            if (nbFace >= 0) {
                it->second = nbFace;
                continue;
            }
        }
        m_face.erase(it);
    }
}

/// 设置单元对应边或面的邻接编号。
void TetMesh3D::setNeighbor(int faceA, int faceB)
{
    // 调用方保证 faceA 与 faceB 是同一无向面的两个相反方向
    tets[faceA / 4].n[faceA % 4] = faceB / 4;
    tets[faceB / 4].n[faceB % 4] = faceA / 4;
}

/// 查询网格面关联的四面体数量。
int TetMesh3D::faceDegree(int a, int b, int c) const
{
    const uint64_t key = canonicalFaceKey(a, b, c);
    int deg = 0;
    for (const Tet4& t : tets) {
        for (int e = 0; e < 4; ++e) {
            if (canonicalFaceKey(t.v[(e + 1) % 4], t.v[(e + 2) % 4], t.v[(e + 3) % 4]) == key)
                ++deg;
        }
    }
    return deg;
}

/// 核对面哈希表与四面体面记录是否一致。
bool TetMesh3D::checkFaceHash(std::string* error) const
{
    const auto fail = [error](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    const int T = static_cast<int>(tets.size());

    // 遍历每个四面体的 4 个面（共 4T 个「面实例」）：
    //   * 每个实例的度数（被多少个 tet 共享）只能是 1 或 2；
    //   * 度数必须与该面的邻居声明一致（deg==2 ⇔ n[e]>=0）；
    //   * 4T 个实例的度数之和必须恰为 4T（因为每个实例贡献 1，
    //     而 deg 是把「共享」计了两次——等价地：distinct 面数 + 共享面数 == 2T）。
    // 这里用更直接的等价判据：所有**去重后**的面，其度数之和 == 4T。
    std::unordered_map<uint64_t, int> seenDeg;
    for (int id = 0; id < T; ++id) {
        const Tet4& t = tets[id];
        for (int e = 0; e < 4; ++e) {
            const int a = t.v[(e + 1) % 4], b = t.v[(e + 2) % 4], c = t.v[(e + 3) % 4];
            const uint64_t key = canonicalFaceKey(a, b, c);
            if (seenDeg.find(key) != seenDeg.end())
                continue;   // 该面已判定过
            const int deg = faceDegree(a, b, c);
            seenDeg[key] = deg;
            if (deg > 2)
                return fail("face shared by more than two tetrahedra (overlap)");
            if (deg < 1)
                return fail("face missing from hash");

            // 与邻居声明一致性：deg==2 ⇔ 该面（在某个 tet 中）有邻居
            const bool hasNb = (t.n[e] >= 0);
            if (hasNb != (deg == 2))
                return fail("face degree inconsistent with neighbor declaration");

            // 哈希归属：该 key 必须指向使用该面的某个面 id
            auto it = m_face.find(key);
            if (it == m_face.end())
                return fail("face key absent from hash");
            if (deg == 1 && it->second != id * 4 + e)
                return fail("boundary face not owned by this face id");
        }
    }

    // 去重面数必须等于哈希表大小
    if (static_cast<int>(seenDeg.size()) != static_cast<int>(m_face.size()))
        return fail("face hash size mismatch vs distinct faces");
    // 度数之和（每个共享面计 2）必须等于 2·(去重面数) + 共享面数……等价判据：
    // 直接验证 Σ deg == 4T —— 每个 tet 贡献 4 个面实例，每个实例恰被计数一次。
    long long sumDeg = 0;
    for (const auto& kv : seenDeg)
        sumDeg += kv.second;
    // 注意：Σ（不同面的关联单元数） == 4T 恒成立（每个 tet 的 4 个面各被计一次）
    if (sumDeg != 4LL * T)
        return fail("face degree sum mismatch");
    return true;
}

/// 读取逐步校验开关，决定是否检查每次拓扑操作。
bool TetMesh3D::verifyEachStep()
{
    // 读一次并缓存：热循环里不能反复 getenv（既慢又非线程安全）。
    static const bool on = [] {
        const char* v = std::getenv("MESHSTUDIO_VERIFY_EACH_STEP");
        return v != nullptr && std::atoi(v) != 0;
    }();
    return on;
}

/// 拓扑一致性自检（邻居对称、面哈希完整性、边哈希引用计数、右手系、索引范围）。
bool TetMesh3D::validate(std::string* error) const
{
    const auto fail = [error](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    const int V = static_cast<int>(points.size());

    for (const Tet4& t : tets) {
        for (int e = 0; e < 4; ++e) {
            if (t.v[e] < 0 || t.v[e] >= V)
                return fail("vertex index out of range");
            if (t.n[e] < -1 || t.n[e] >= static_cast<int>(tets.size()))
                return fail("neighbor index out of range");
        }
        // 精确符号检查物理单元，不用 SoS 把零体积单元当作正体积。
        const Point3D& pa = points[t.v[0]];
        const Point3D& pb = points[t.v[1]];
        const Point3D& pc = points[t.v[2]];
        const Point3D& pd = points[t.v[3]];
        for(const auto* p:{&pa,&pb,&pc,&pd}) if(!std::isfinite(p->x)||!std::isfinite(p->y)||!std::isfinite(p->z))
            return fail("nonfinite tet vertex");
        const int od = orient3d(pa, pb, pc, pd);
        if (od == 0)
            return fail("tet is degenerate (zero volume)");
        if (od < 0.0)
            return fail("tet not right-handed");
    }

    for (int id = 0; id < static_cast<int>(tets.size()); ++id) {
        const Tet4& t = tets[id];
        for (int e = 0; e < 4; ++e) {
            const int nb = t.n[e];
            if (nb >= 0) {
                const Tet4& tn = tets[nb];
                bool found = false;
                for (int e2 = 0; e2 < 4; ++e2) {
                    if (tn.n[e2] != id)
                        continue;
                    // 共享面的判定必须用**无向顶点集**：两个相邻四面体从各自
                    // 一侧看同一个面，环序必然互为**反向**（一个逆时针、一个
                    // 顺时针）。faceVertex 给的是「有向最小循环旋转」，只保证
                    // 两侧的**顶点集合**相同（这也是 canonicalFaceKey 成立的基础），
                    // 不保证三元组逐项相等。故这里比对排序后的集合。
                    int m[3] = { faceVertex(id * 4 + e, 0),
                                 faceVertex(id * 4 + e, 1),
                                 faceVertex(id * 4 + e, 2) };
                    int n[3] = { faceVertex(nb * 4 + e2, 0),
                                 faceVertex(nb * 4 + e2, 1),
                                 faceVertex(nb * 4 + e2, 2) };
                    std::sort(m, m + 3);
                    std::sort(n, n + 3);
                    if (m[0] == n[0] && m[1] == n[1] && m[2] == n[2]) {
                        const int side=orient3d(points[m[0]],points[m[1]],points[m[2]],points[t.v[e]]);
                        const int otherSide=orient3d(points[m[0]],points[m[1]],points[m[2]],points[tn.v[e2]]);
                        if(side==0 || otherSide==0 || side==otherSide)
                            return fail("adjacent tetrahedra fold across shared face (opposite vertices not on opposite sides)");
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return fail("neighbor not symmetric (face mismatch)");
            } else {
                // 声明无邻居（边界面）的面，不允许存在**另一个** tet 使用同一面
                const int a = faceVertex(id * 4 + e, 0);
                const int b = faceVertex(id * 4 + e, 1);
                const int c = faceVertex(id * 4 + e, 2);
                auto it = m_face.find(canonicalFaceKey(a, b, c));
                if (it != m_face.end() && it->second != id * 4 + e)
                    return fail("boundary face has hidden opposite face");
            }
        }
    }

    if (!checkFaceHash(error))
        return false;

    // ---- 边哈希自检（O(6T)）----
    // hasEdgeAny 现在是 O(1) 查表，因此这张表一旦与四面体失同步，
    // 约束边恢复会**静默**判断错误（比"慢"危险得多）。这里重算一遍引用计数比对。
    {
        std::unordered_map<uint64_t, int> expect;
        for (const Tet4& t : tets)
            for (int e = 0; e < 4; ++e)
                for (int f = e + 1; f < 4; ++f)
                    ++expect[edgeKeyOf(t.v[e], t.v[f])];
        if (expect.size() != m_edge.size())
            return fail("edge hash size mismatch vs distinct edges");
        for (const auto& kv : expect) {
            const auto it = m_edge.find(kv.first);
            if (it == m_edge.end() || it->second != kv.second)
                return fail("edge hash refcount inconsistent");
        }
    }
    return true;
}

/// 累计四面体网格的体积。
double TetMesh3D::totalVolume() const
{
    double sum = 0.0;
    for (const Tet4& t : tets)
        sum += orient3dDet(points[t.v[0]], points[t.v[1]], points[t.v[2]], points[t.v[3]]) / 6.0;
    return sum;
}

} // namespace mesher
