// 文件职责：局部编辑事务：记录三角形和半边的修改，失败恢复原状态，提交后公布槽位搬移和编辑事件。
#include "Mesh2DTransaction.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
namespace mesher {
/// 建立局部网格编辑事务；禁止复制以保持唯一回滚责任。
Mesh2DTransaction::Mesh2DTransaction(Mesh2D& m,MeshObservation o):mesh(m),observation(o) {
    if (mesh.m_transaction) throw std::logic_error("nested Mesh2D transaction");
    mesh.ensurePointIndex();
    delta.initialPoints=mesh.points.size(); delta.initialTriangles=mesh.tris.size();
    triangles.reserve(32); changed.reserve(32); halfedges.reserve(64); seenEdges.reserve(64);
    addedPointKeys.reserve(4); delta.events.reserve(32); delta.changedSlots.reserve(32);
    mesh.m_transaction=this; observation.count(MeshCounter::LocalMeshTransactions,1);
}
/// 事务未提交时恢复修改前的网格状态。
Mesh2DTransaction::~Mesh2DTransaction() { if (!finished) rollback(); }
/// 记录三角形被修改，供版本更新或事务回滚使用。
void Mesh2DTransaction::touchTriangle(int slot) {
    if (slot<int(delta.initialTriangles) && !triangles.count(slot)) {
        triangles.emplace(slot,TriUndo{mesh.tris[slot],mesh.m_versions[slot]});
        observation.count(MeshCounter::TransactionTrianglesLogged,1);
    }
    if (changed.insert(slot).second) delta.changedSlots.push_back(slot);
}
/// 在修改前保存半边索引，供事务回滚使用。
void Mesh2DTransaction::touchHalfedge(std::uint64_t key) {
    if (seenEdges.count(key)) return;
    halfedges.push_back({key,{},false});
    seenEdges.insert(key);
    auto& undo=halfedges.back(); undo.original=mesh.m_halfedge.extract(key); undo.captured=true;
    observation.count(MeshCounter::TransactionHalfedgesLogged,1);
}
/// 记录事务中新建的三角形。
void Mesh2DTransaction::created(int slot) {
    delta.events.push_back({MeshEditEvent::Kind::Created,slot,-1}); delta.createdSlots.push_back(slot);
}
/// 记录事务中被移除的三角形及其原状态。
void Mesh2DTransaction::removed(int slot,int last) {
    const auto& tri=mesh.tris[slot];delta.removedTriangles.push_back({tri.v[0],tri.v[1],tri.v[2]});
    delta.events.push_back({MeshEditEvent::Kind::Removed,slot,last}); delta.deletedSlots.push_back(slot);
    if (slot!=last) {
        delta.events.push_back({MeshEditEvent::Kind::Moved,slot,last}); delta.moves.push_back({last,slot});
    }
}
/// 编辑前网格须有效，编辑期间只通过 Mesh2D 方法改存储；不嵌套事务。
bool Mesh2DTransaction::validate(std::string* error,const std::vector<std::pair<int,int>>& protectedEdges) const {
    if (!mesh.validateAffected(delta.changedSlots,error,observation)) return false;
    for (auto edge:protectedEdges) if (!mesh.hasEdgeAny(edge.first,edge.second)) {
        if (error) *error="local edit removed a protected constraint"; return false;
    }
    return true;
}
/// 发布网格变化并结束事务，保留已经完成的修改。
bool Mesh2DTransaction::commit(std::string* error,MeshEditDelta* published,const std::vector<std::pair<int,int>>& protectedEdges) {
    if (finished) throw std::logic_error("finished Mesh2D transaction");
    if (!validate(error,protectedEdges)) return false;
    auto& slots=delta.changedSlots;
    slots.erase(std::remove_if(slots.begin(),slots.end(),[&](int s){return s<0 || s>=int(mesh.tris.size());}),slots.end());
    if (std::numeric_limits<std::uint64_t>::max()-mesh.m_nextVersion<slots.size())
        throw std::overflow_error("Mesh2D topology version exhausted");
    for (int s:slots) mesh.m_versions[s]=mesh.m_nextVersion++;
    mesh.m_transaction=nullptr; finished=true;
    if (published) *published=std::move(delta);
    return true;
}
/// 保留旧 map 节点、向量容量；回滚不重新分配节点，版本序号不倒退。
void Mesh2DTransaction::rollback() noexcept {
    if (finished) return;
    // 新节点先摘除，再移回保存的旧节点；bucket 容量不缩小，恢复无需分配。
    for (auto it=halfedges.rbegin();it!=halfedges.rend();++it) if (it->captured) {
        mesh.m_halfedge.erase(it->key);
        if (!it->original.empty()) mesh.m_halfedge.insert(std::move(it->original));
    }
    mesh.tris.resize(delta.initialTriangles); mesh.m_versions.resize(delta.initialTriangles);
    for (const auto& p:triangles) { mesh.tris[p.first]=p.second.triangle; mesh.m_versions[p.first]=p.second.version; }
    for (const auto& key:addedPointKeys) mesh.m_pointIndex.erase(key);
    mesh.points.resize(delta.initialPoints);
    mesh.m_transaction=nullptr; finished=true; observation.count(MeshCounter::RolledBackTransactions,1);
}
} // namespace mesher
