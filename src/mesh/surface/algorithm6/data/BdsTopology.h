#pragma once
/** @file BdsTopology.h
 * @brief BDS 稳定节点身份、边/面池和双向邻接；不决定几何操作资格。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "SurfaceTypes.h"
#include "BdsTypes.h"
#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace mesher::gmshReference {
struct BdsPoint {
    int id = 0;
    std::uint64_t physicalId = 0; // 外围点为0；不与局部负编号混用。
    Point2D uv;
    Point3D point;
    NodeSize sizes{1.e22,1.e22};
    BdsClassification classification;
    bool configurationModified = true;
    short degenerated = 0;
    std::optional<int> periodicCounterpart;
    std::vector<int> edges; // 原生追加序，不能按邻点编号排序。
};
struct BdsEdge {
    int first = 0, second = 0; // 节点局部ID，始终 first < second。
    bool deleted = false;
    BdsClassification classification;
    std::vector<int> faces; // 原生 BDS_Edge::_faces 追加序。
};
struct BdsTriangle {
    std::array<int,3> edges{}; // 边池索引，保留 e1/e2/e3 顺序。
    bool deleted = false;
    BdsClassification classification;
};
struct BdsMesh {
    std::map<int,BdsPoint> points; // 对应原生按 iD 排列的 set，不压缩负ID或空洞。
    std::set<BdsClassification> geometry; // 只登记Registered来源；临时域标记不加入m.geom。
    std::vector<BdsEdge> edges;
    std::vector<BdsTriangle> triangles;
    int maximumPointId = 0;
};

/** 原位拓扑操作；调用方拥有工作副本，算法入口负责失败原子发布。
 * 不调用 CAD、不着域、不过滤三角形或按几何容差合并节点。
 * insert 拒绝重复ID，避免原版 set 插入失败仍返回泄漏点的未定义身份。
 * append 保留重复三角形/多面邻接的原生记录语义，但拒绝缺点和重复端点。
 * 失败抛 OperationFailure；get 返回由三对相邻边共同顶点恢复的原方向。
 */
void insertBdsPoint(BdsMesh& mesh, BdsPoint point);
/// 按无向节点对查找未删除的 BDS 边。
int findBdsEdge(const BdsMesh& mesh, int first, int second);
/// 追加 BDS 三角形并建立边、节点和三角形的关联。
int appendBdsTriangle(BdsMesh& mesh, const std::array<int,3>& nodes);
/// 由 BDS 三角形的三条边还原节点编号。
std::array<int,3> getBdsTriangleNodes(const BdsMesh& mesh, int triangle);

struct BdsEdgeNeighborhood {
    int first = 0, second = 0;
    std::array<int,2> oppositeNodes{}, faces{};
    std::array<int,4> sideEdges{}; // p1-op0, op0-p2, p1-op1, op1-p2；在各原面内查找。
    int orientation = 0;
};
/** 获取原生双三角邻域和首面方向；无双面活边时返回空。
 * 不检查CAD分类/config_modified/凸性，它们属于具体几何操作资格。
 */
std::optional<BdsEdgeNeighborhood> getBdsEdgeNeighborhood(const BdsMesh& mesh,int edge);
/** 已通过调用方资格检查的BDS翻边，删除旧边/两面并按原序追加新边/两面。
 * 不去重已有另一对角线；继承各面及旧边分类，四点config_modified置真。
 * 旧池记录保留，只从活双向邻接移除；邻域须来自当前未修改的mesh。
 * 原位操作、异常合同同append；不判断几何资格，不执行cleanup。
 */
void swapBdsEdgeTopology(BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& neighborhood);

/** 校验有限数值、注册/临时分类来源身份、池引用及双向邻接；不判断域/朝向/形状质量。
 * 不要求边最多邻接两面；阶段性重复片按原生允许保存。
 * 已删除边/面须从活邻接中移除，但稳定池记录保留到 cleanup 阶段。
 */
OperationStatus validateBdsTopology(const BdsMesh& mesh, const RunControl& control = {});
}
