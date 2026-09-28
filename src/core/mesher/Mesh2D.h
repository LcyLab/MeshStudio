#pragma once
// ============================================================
// Mesh2D —— 2D 三角形网格容器（M0 数据结构模块）
// 职责：管理顶点/三角形数组，维护三要素的一致性：
//   1. 三角形邻居 n[3]：n[e] 对边 v[e]（即 v[(e+1)%3] 与 v[(e+2)%3] 的连边）外侧的三角形
//   2. 有向半边表：key=(起点,终点) → 半边 id = triId*3+e，用于 O(1) 查找对侧三角形
//   3. 顶点索引合法性
// 本模块不包含任何三角化/细化算法，只保证拓扑数据结构的正确性。
// 2D 原型中这套"邻接维护 + 紧凑删除"的骨架会在 3D 四面体阶段沿用。
// ============================================================

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "MesherTypes.h"

namespace mesher {

struct Tri2D
{
    int v[3] = { -1, -1, -1 };   ///< 顶点索引（逆时针）
    int n[3] = { -1, -1, -1 };   ///< 邻居三角形 id，-1 表示无（边界）
};

class Mesh2D
{
public:
    std::vector<Point2D> points;
    std::vector<Tri2D>   tris;

    void clear() { points.clear(); tris.clear(); m_halfedge.clear(); }

    int addPoint(const Point2D& p)
    {
        points.push_back(p);
        return static_cast<int>(points.size()) - 1;
    }

    /// 新增三角形（顶点需已存在且不共线），自动建立邻居与半边表，返回 id。
    int addTri(int a, int b, int c);

    /// 删除三角形：swap-erase 紧凑删除，同步修正邻居引用与半边表。
    void removeTri(int triId);

    /// 三角形 triId 的边 e 对侧的三角形 id（-1 表示该边无邻居）。
    int findOpposite(int triId, int e) const;

    /// 无向边 {a,b} 是否已被网格中的三角形使用。
    bool hasEdgeAny(int a, int b) const;

    /// 拓扑一致性自检（邻居对称性、半边表完整性、索引范围）。
    /// 通过返回 true；否则把原因写入 error。
    bool validate(std::string* error = nullptr) const;

    /// 全部三角形面积之和
    double totalArea() const;

private:
    int  srcOf(int he) const { return tris[he / 3].v[he % 3]; }
    int  dstOf(int he) const { return tris[he / 3].v[(he % 3 + 1) % 3]; }
    int  oppositeHalfedge(int he) const; // 反向有向半边 id（-1 无）
    void addTriEdges(int triId);     // 把 triId 三条有向边写入半边表
    void removeTriEdges(int triId);  // 从半边表擦除 triId 三条有向边
    void setNeighbor(int heA, int heB); // heA 与 heB 互为反向半边 → 双向邻居

    // key: ((uint64)起点 << 32) | 终点
    std::unordered_map<uint64_t, int> m_halfedge;
};

} // namespace mesher
