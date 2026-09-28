#pragma once
// ============================================================
// TetMesh3D —— 3D 四面体网格容器（M1 数据结构模块）
// 职责：管理顶点/四面体数组，维护四要素的一致性：
//   1. 四面体邻居 n[4]：n[e] 对边 v[e]（即面 (v[(e+1)%4], v[(e+2)%4], v[(e+3)%4]) 外侧的四面体）
//   2. 有向面哈希表：key=(环序三顶点) → 面 id = tetId*4+e，O(1) 查找对侧四面体
//   3. 顶点索引合法性
//   4. 右手系定向（orient3d(v0,v1,v2,v3) > 0）
// 本模块只保证拓扑数据结构的正确性，不含任何四面体化算法。
// 结构完全对应 2D 的 Mesh2D（邻居 + 半边表），是 3D 阶段的地基。
// ============================================================

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "MesherTypes.h"

namespace mesher {

struct Tet4
{
    int v[4] = { -1, -1, -1, -1 };   ///< 顶点索引（右手系）
    int n[4] = { -1, -1, -1, -1 };   ///< 邻居四面体 id，n[e] 对边 v[e]（即面 (v[e+1],v[e+2],v[e+3]) 外侧）
};

class TetMesh3D
{
public:
    std::vector<Point3D> points;
    std::vector<Tet4>    tets;

    void clear() { points.clear(); tets.clear(); m_face.clear(); m_edge.clear(); }

    int addPoint(const Point3D& p)
    {
        points.push_back(p);
        return static_cast<int>(points.size()) - 1;
    }

    /// 新增四面体（顶点需已存在且右手系 orient3d(a,b,c,d)>0），自动建邻居与面/边哈希。
    int addTet(int a, int b, int c, int d);

    /// 删除四面体：swap-erase 紧凑删除，同步修正邻居引用与面/边哈希。
    void removeTet(int tetId);

    /// 四面体 tetId 的面 e 对侧的四面体 id（-1 表示边界面）。
    int findOpposite(int tetId, int e) const;

    /// 无向三角面 {a,b,c} 是否已被网格中的四面体使用。O(1)（查面哈希）。
    bool hasFaceAny(int a, int b, int c) const;

    /// 无向边 {a,b} 是否已是网格中的某条四面体边。
    ///
    /// ★ O(1) —— 查边哈希，**不是**全网格扫描 ★
    /// 旧实现是 `for (每个四面体) 查它的 6 条边`，即 O(T)。它被
    /// `Constrained3D::edgeSatisfied` 在**每轮、对每条待处理约束边**调用，
    /// 于是单轮就是 O(P·T)：ball.brep 上 P=7692、T=21822 → 1.7×10⁸ 次比较/轮，
    /// 多轮下来 enforceSurface3D 直接跑成分钟级（实测 >100 s 未完成）。
    /// 改成边哈希后降为 O(1)，该阶段回到秒级。
    bool hasEdgeAny(int a, int b) const;

    /// 边 {a,b} 被多少个四面体包含（正常为 1~N；0 表示不是网格边）。
    /// 与 hasEdgeAny 同源，供诊断/自检使用。
    int edgeDegree(int a, int b) const;

    /// 拓扑一致性自检（邻居对称、面哈希完整性、边哈希引用计数、右手系、索引范围）。
    bool validate(std::string* error = nullptr) const;

    /// ★「逐步自检」总开关（默认关闭）★
    /// ------------------------------------------------------------
    /// validate() / checkFaceHash() 都是 **O(T)**。把它们放进 O(n) 的逐元素
    /// 热循环（逐点插入 / 逐轮细化 / 逐面试翻）就是 **O(n·T) 二次方**，
    /// 小输入看不出、大输入直接跑成分钟级。
    ///
    /// 这些检查的唯一价值是**定位**"哪一步把网格弄坏了"（错误信息里带步号）。
    /// 生产路径上由各阶段收尾的**一次性** validate() 兜底，覆盖不变，
    /// 只是失去"哪一步坏的"这个定位信息。
    ///
    /// 打开方式：环境变量 `MESHSTUDIO_VERIFY_EACH_STEP=1`（进程级，读一次后缓存；
    /// 热循环里反复 getenv 既慢又非线程安全，故缓存）。
    static bool verifyEachStep();

    /// 全部四面体体积之和
    double totalVolume() const;

    /// 有向面哈希表项数（自检用；规范 key 下 = 去重后的面数）
    size_t faceHashSize() const { return m_face.size(); }

    /// 边哈希表项数（自检用；= 去重后的网格边数）
    size_t edgeHashSize() const { return m_edge.size(); }

    /// 面哈希自检（O(F)）：
    ///   1) 每个面 key 恰由 1~2 个四面体引用（>2 → 拓扑重叠，非法）；
    ///   2) 「1 个引用」必须与「该面所在 tet 声明为边界面(n[e]<0)」一致；
    ///   3) 「2 个引用」必须与「两个 tet 互指为邻居」一致。
    /// 返回 true 表示面哈希与邻居结构完全自洽。
    bool checkFaceHash(std::string* error = nullptr) const;

    /// 面 {a,b,c} 被多少个四面体的面引用（1 或 2 为正常）。
    int faceDegree(int a, int b, int c) const;

    /// 面 id（tetId*4+e）的**规范环序**第 k 个顶点（k=0..2）。
    /// 环序仅由该面的 3 个顶点决定（三次比较得升序集合，再取最小循环旋转），
    /// 与对侧顶点、四面体内部的 e 序号都无关。因此共享同一面的两个四面体
    /// 会算出**完全相同**的三元组 → 面哈希可以可靠配对，即使其中一个是
    /// 零体积退化单元也不会污染哈希表。
    int faceVertex(int faceId, int k) const;

private:
    // （addTet 内部逐面完成「查对侧 + 登记」两步，无需单独的 addTetFaces）
    void removeTetFaces(int tetId);  // 从面哈希擦除 tetId 四个有向面
    void addTetEdges(int tetId);     // 把 tetId 的 6 条边计入边哈希（引用计数 +1）
    void removeTetEdges(int tetId);  // 把 tetId 的 6 条边从边哈希摘除（计数 -1）
    void setNeighbor(int faceA, int faceB); // faceA 与 faceB 互为反向面 → 双向邻居
    int  oppositeFace(int faceId) const;    // 反向有向面 id（-1 无）

    // key: 三个顶点索引折叠成 uint64（各占 21 位）
    std::unordered_map<uint64_t, int> m_face;

    // 边哈希：key = 无向边（min<<32 | max），value = **被多少个四面体包含**
    // （引用计数）。与 m_face 同步增量维护：addTet +1 / removeTet -1，减到 0 擦除。
    // 用途：把 hasEdgeAny 从 O(T) 全扫描降到 O(1)（见 hasEdgeAny 的说明）。
    std::unordered_map<uint64_t, int> m_edge;
};

} // namespace mesher
