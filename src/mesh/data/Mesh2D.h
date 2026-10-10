#pragma once
// ============================================================
// Mesh2D —— 2D 三角形网格容器（M0 数据结构模块）
// 职责：管理顶点/三角形数组，维护三要素的一致性：
//   1. 三角形邻居 n[3]：n[e] 位于有向边 (v[e],v[(e+1)%3]) 的另一侧
//   2. 有向半边表：key=(起点,终点) → 半边 id = triId*3+e，用于 O(1) 查找对侧三角形
//   3. 顶点索引合法性
// 本模块不包含任何三角化/细化算法，只保证拓扑数据结构的正确性。
// 2D 原型中这套"邻接维护 + 紧凑删除"的骨架会在 3D 四面体阶段沿用。
// ============================================================

#include <cstdint>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>

#include "geometry/types/MesherTypes.h"
#include "mesh/runtime/MeshingObserver.h"

namespace mesher {
class Mesh2DTransaction;
struct MeshEditDelta;

struct Tri2D
{
    int v[3] = { -1, -1, -1 };   ///< 顶点索引（逆时针）
    int n[3] = { -1, -1, -1 };   ///< 邻居三角形 id，-1 表示无（边界）
};

class Mesh2D
{
public:
    /// 构造二维网格；复制或移动时同步其节点、拓扑及索引状态。
    Mesh2D()=default;
    /// 构造二维网格；复制或移动时同步其节点、拓扑及索引状态。
    Mesh2D(const Mesh2D&);
    /// 替换二维网格内容并同步内部索引状态。
    Mesh2D& operator=(const Mesh2D&);
    /// 构造二维网格；复制或移动时同步其节点、拓扑及索引状态。
    Mesh2D(Mesh2D&&);
    /// 替换二维网格内容并同步内部索引状态。
    Mesh2D& operator=(Mesh2D&&);
    std::vector<Point2D> points;
    std::vector<Tri2D>   tris;

    /// 清空二维网格数据、索引和访问标记。
    void clear();
    /// 追加网格节点并返回其编号。
    int addPoint(const Point2D& p);
    /// 撤销最近追加的节点并更新节点索引。
    void discardLastPoint();
    /// 精确有限坐标查重，+0/-0 相同；直接批量改 points 后须使索引失效。
    int exactPoint(const Point2D& p) const;
    /// 已验收内部点的位置更新；调用方给出完整关联片，禁止事务中移动。
    /// 保持精确点索引，并更新关联片版本；不代替几何/方向检查。
    bool relocatePoint(int vertex,Point2D position,const std::vector<int>& incident);
    /// 标记节点坐标索引失效，等待后续重建。
    void invalidatePointIndex() { m_pointIndexReady=false; }
    /// 读取三角形的拓扑版本，用于识别过期条目。
    std::uint64_t triangleVersion(int slot) const { return m_versions.at(slot); }
    /// 开始新一轮空腔访问并返回标记代次。
    std::uint64_t beginCavityVisit(MeshObservation observation={});
    /// 判断三角形是否已在本轮空腔搜索中访问。
    bool cavityVisited(int slot,std::uint64_t epoch) const { return m_cavityMarks[slot]==epoch; }
    /// 记录三角形已被本轮空腔搜索访问。
    void markCavity(int slot,std::uint64_t epoch) { m_cavityMarks[slot]=epoch; }
    /// 检查局部编辑涉及的三角形、半边和邻接关系。
    bool validateAffected(const std::vector<int>& slots,std::string* error=nullptr,MeshObservation observation={}) const;

    /// 新增三角形（顶点需已存在且不共线），自动建立邻居与半边表，返回 id。
    int addTri(int a, int b, int c);

    /// 删除三角形：swap-erase 紧凑删除，同步修正邻居引用与半边表。
    void removeTri(int triId);

    /// 三角形 triId 的边 e 对侧的三角形 id（-1 表示该边无邻居）。
    int findOpposite(int triId, int e) const;

    /// 无向边 {a,b} 是否已被网格中的三角形使用。
    bool hasEdgeAny(int a, int b) const;

    /// 有向边 a→b 的半边 id（triId*3+e）；不存在返回 -1。
    /// 紧凑删除后需重新查询，不保留旧 id。
    int findHalfedge(int a,int b) const;

    /// 拓扑一致性自检（邻居对称性、半边表完整性、索引范围）。
    /// 通过返回 true；否则把原因写入 error。
    bool validate(std::string* error = nullptr,MeshObservation observation = {}) const;

    /// 全部三角形面积之和
    double totalArea() const;

private:
    friend class Mesh2DTransaction;
    friend struct Mesh2DTestAccess;
    Mesh2DTransaction* m_transaction=nullptr;
    std::vector<std::uint64_t> m_versions,m_cavityMarks;
    std::uint64_t m_nextVersion=1,m_visitEpoch=0;
    mutable std::map<std::pair<double,double>,int> m_pointIndex;
    mutable bool m_pointIndexReady=false;
    /// 按需建立精确坐标到节点编号的查找索引。
    void ensurePointIndex() const;
    /// 记录三角形被修改，供版本更新或事务回滚使用。
    void touchTriangle(int slot);
    /// 写入有向半边索引。
    void writeHalfedge(std::uint64_t key,int value);
    /// 移除有向半边索引。
    void eraseHalfedge(std::uint64_t key);
    /// 读取半边的起点编号。
    int  srcOf(int he) const { return tris[he / 3].v[he % 3]; }
    /// 读取半边的终点编号。
    int  dstOf(int he) const { return tris[he / 3].v[(he % 3 + 1) % 3]; }
    int  oppositeHalfedge(int he) const; // 反向有向半边 id（-1 无）
    void addTriEdges(int triId);     // 把 triId 三条有向边写入半边表
    void removeTriEdges(int triId);  // 从半边表擦除 triId 三条有向边
    void setNeighbor(int heA, int heB); // heA 与 heB 互为反向半边 → 双向邻居

    // key: ((uint64)起点 << 32) | 终点
    std::unordered_map<uint64_t, int> m_halfedge;
};

} // namespace mesher
