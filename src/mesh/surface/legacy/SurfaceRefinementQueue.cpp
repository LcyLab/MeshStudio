// 文件职责：增量细化调度：按三角形版本管理待处理任务，丢弃局部编辑后失效的队列记录。
#include "SurfaceRefinementQueue.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace mesher {
namespace {
/// 判断两个细化任务是否包含相同槽位、版本和优先级信息。
bool same(const SurfaceTriangleTicket& a,const SurfaceTriangleTicket& b) {
    return a.face==b.face && a.slot==b.slot && a.topologyVersion==b.topologyVersion &&
        a.criteriaVersion==b.criteriaVersion && a.priorityClass==b.priorityClass &&
        a.violation==b.violation && a.vertexKey==b.vertexKey;
}
}
/// 按任务类别、违反程度和稳定身份比较细化优先级。
bool SurfaceRefinementQueue::LowerPriority::operator()(const SurfaceTriangleTicket& a,const SurfaceTriangleTicket& b) const {
    if (a.priorityClass!=b.priorityClass) return a.priorityClass<b.priorityClass;
    if (a.violation!=b.violation) return a.violation<b.violation;
    if (a.face!=b.face) return a.face>b.face;
    if (a.vertexKey!=b.vertexKey) return a.vertexKey>b.vertexKey;
    if (a.slot!=b.slot) return a.slot>b.slot;
    return a.topologyVersion<b.topologyVersion;
}
/// 绑定 CAD 面和准则版本，建立细化任务队列。
SurfaceRefinementQueue::SurfaceRefinementQueue(int f,std::uint64_t v,MeshObservation o):face(f),criteria(v),observation(o) {
    if (f<0 || !v) throw std::invalid_argument("invalid surface queue face/criteria version");
}
/// 校验并登记细化任务，同时更新槽位的有效任务。
bool SurfaceRefinementQueue::offer(const SurfaceTriangleTicket& t,std::string* error) {
    if (t.face!=face || t.slot<0 || !t.topologyVersion || t.criteriaVersion!=criteria ||
        t.priorityClass<1 || t.priorityClass>2 || !std::isfinite(t.violation) || t.violation<=0 ||
        t.vertexKey[0]<0 || !(t.vertexKey[0]<t.vertexKey[1] && t.vertexKey[1]<t.vertexKey[2])) {
        if (error) *error="invalid refinement queue ticket"; return false;
    }
    auto prior=active.find(t.slot);
    if (prior!=active.end() && same(prior->second,t)) return true;
    heap.push(t); active[t.slot]=t;
    observation.count(MeshCounter::RefinementQueuePushes,1);
    if (heap.size()>peak) {
        observation.count(MeshCounter::RefinementQueuePeakEntries,heap.size()-peak); peak=heap.size();
    }
    compact(); return true;
}
/// 取出最高优先级的有效任务，跳过版本过期条目。
bool SurfaceRefinementQueue::pop(const Mesh2D& mesh,SurfaceTriangleTicket& result) {
    while (!heap.empty()) {
        const auto t=heap.top(); heap.pop(); const auto found=active.find(t.slot);
        if (found==active.end() || !same(found->second,t)) {
            observation.count(MeshCounter::RefinementQueueStaleEntries,1); continue;
        }
        active.erase(found);
        bool valid=t.criteriaVersion==criteria && t.slot<int(mesh.tris.size()) && mesh.triangleVersion(t.slot)==t.topologyVersion;
        if (valid) {
            const auto tri=mesh.tris[t.slot]; std::array<int,3> key{tri.v[0],tri.v[1],tri.v[2]}; std::sort(key.begin(),key.end());
            valid=key==t.vertexKey;
        }
        if (!valid) { observation.count(MeshCounter::RefinementQueueStaleEntries,1); continue; }
        result=t; return true;
    }
    return false;
}
/// 在陈旧条目过多时由有效任务重建队列。
void SurfaceRefinementQueue::compact() {
    if (heap.size()<=active.size()*3+128) return;
    std::vector<SurfaceTriangleTicket> live; live.reserve(active.size());
    for (const auto& entry:active) live.push_back(entry.second);
    decltype(heap) candidate(LowerPriority{},std::move(live)); heap=std::move(candidate);
    ++compressed; observation.count(MeshCounter::RefinementQueueCompactions,1);
}
/// 移除局部拓扑编辑影响的任务并按需压缩队列。
void SurfaceRefinementQueue::invalidate(const MeshEditDelta& delta) {
    for (const auto& e:delta.events) {
        active.erase(e.slot); if (e.other>=0) active.erase(e.other);
    }
    for (int slot:delta.changedSlots) active.erase(slot);
    compact();
}
/// 更新细化准则版本并清空原有效任务。
void SurfaceRefinementQueue::resetCriteria(std::uint64_t version) {
    if (!version) throw std::invalid_argument("zero criteria version");
    criteria=version; active.clear(); compact();
}
} // namespace mesher
