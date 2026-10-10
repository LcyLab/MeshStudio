// 文件职责：面内域记账：局部拓扑编辑时继承域标签，最终与完整边界分类核对，防止孔洞或外域被误填。
#include "SurfaceDomainState.h"
#include "mesh/planar/Constrained2D.h"
#include <stdexcept>
namespace mesher {
namespace {
/// 将有序节点对编码为边查找键。
std::uint64_t edgeKey(int a,int b) {
    if (a>b) std::swap(a,b); return (std::uint64_t(std::uint32_t(a))<<32)|std::uint32_t(b);
}
/// 写入失败原因并返回失败状态。
bool fail(std::string* error,const char* why) { if (error) *error=why; return false; }
}
/// 完整分类约束域，建立与三角形槽位对应的标签。
bool SurfaceDomainState::initialize(const Mesh2D& mesh,const std::vector<std::pair<int,int>>& edges,
                                   std::string* error,MeshObservation observation) {
    if (editing) throw std::logic_error("domain initialized during edit");
    std::vector<bool> candidate;
    if (!classifyConstrainedDomain(mesh,edges,candidate,error,observation)) return false;
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,edges,segments,error,observation)) return false;
    std::unordered_set<std::uint64_t> keys;
    for (auto e:segments) keys.insert(edgeKey(e.first,e.second));
    labels=std::move(candidate); boundary=std::move(keys); return true;
}
/// 检查局部修改涉及的三角形是否具有一致域标签。
bool SurfaceDomainState::uniform(const std::vector<int>& slots,std::string* error) const {
    int tag=-1;
    for (int t:slots) {
        if (t<0 || t>=int(labels.size())) return fail(error,"domain edit has an invalid source slot");
        if (tag>=0 && tag!=int(labels[t])) return fail(error,"local edit crosses different domain tags");
        tag=int(labels[t]);
    }
    return tag>=0 || fail(error,"domain edit has no source triangle");
}
/// 建立域标签编辑范围；禁止复制以保持唯一回滚责任。
SurfaceDomainState::Edit::Edit(SurfaceDomainState& s):state(s),originalSize(s.labels.size()) {
    if (state.editing) throw std::logic_error("nested domain edit");
    undo.reserve(32); state.editing=true;
}
/// 在修改前保存原槽位的域标签。
void SurfaceDomainState::Edit::touch(int slot) {
    if (slot<int(originalSize) && !undo.count(slot)) undo.emplace(slot,state.labels.at(slot));
}
/// 未提交时恢复原域标签和编辑状态。
SurfaceDomainState::Edit::~Edit() {
    if (!finished) {
        state.labels.resize(originalSize);
        for (auto p:undo) state.labels[p.first]=p.second;
        state.editing=false;
    }
}
/// 按局部网格变化顺序迁移三角形域标签。
bool SurfaceDomainState::Edit::consume(const MeshEditDelta& delta,std::string* error) {
    if (delta.initialTriangles!=originalSize || cursor>delta.events.size())
        return fail(error,"domain delta does not match initial triangle count");
    for (;cursor<delta.events.size();++cursor) {
        const auto e=delta.events[cursor];
        if (e.kind==MeshEditEvent::Kind::Moved) {
            if (e.slot!=expectedMoveTo || e.other!=expectedMoveFrom || expectedMoveFrom<0)
                return fail(error,"domain delta has an unexpected move");
            expectedMoveFrom=expectedMoveTo=-1; continue;
        }
        if (expectedMoveFrom>=0) return fail(error,"domain delta omitted a tail move");
        if (e.kind==MeshEditEvent::Kind::Removed) {
            if (e.slot<0 || e.slot>=int(state.labels.size()) || e.other!=int(state.labels.size())-1)
                return fail(error,"domain delta has an invalid removal");
            if (createdGroup) { inherited=-1; createdGroup=false; }
            const int tag=int(state.labels[e.slot]);
            if (inherited>=0 && inherited!=tag) return fail(error,"domain delta mixes source tags");
            inherited=tag;
            touch(e.slot); touch(e.other);
            if (e.slot!=e.other) {
                state.labels[e.slot]=state.labels.back(); expectedMoveFrom=e.other; expectedMoveTo=e.slot;
            }
            state.labels.pop_back();
        } else {
            if (inherited<0 || e.slot!=int(state.labels.size()))
                return fail(error,"domain delta has no valid inheritance source");
            // 被删的原槽位已在删除时记账；新尾槽位不属于旧状态。
            state.labels.push_back(inherited!=0); createdGroup=true;
        }
    }
    return expectedMoveFrom<0 || fail(error,"domain delta ended before a tail move");
}
/// 检查受影响槽位的标签数量和局部域一致性。
bool SurfaceDomainState::Edit::validate(const Mesh2D& mesh,const std::vector<int>& changed,std::string* error) const {
    if (state.labels.size()!=mesh.tris.size()) return fail(error,"domain label count differs from triangle count");
    std::unordered_set<int> affected;
    for (int t:changed) if (t>=0 && t<int(mesh.tris.size())) {
        affected.insert(t); for (int n:mesh.tris[t].n) if (n>=0) affected.insert(n);
    }
    for (int t:affected) {
        if (t>=int(mesh.tris.size())) return fail(error,"domain neighbor out of range");
        const auto& tri=mesh.tris[t];
        for (int k=0;k<3;++k) {
            const int n=tri.n[k];
            if (n<-1 || n>=int(mesh.tris.size())) return fail(error,"domain neighbor out of range");
            const bool opposite=n<0?false:state.labels[n];
            if ((state.labels[t]!=opposite)!=bool(state.boundary.count(edgeKey(tri.v[k],tri.v[(k+1)%3]))))
                return fail(error,"local domain labels disagree across a boundary");
        }
    }
    return true;
}
/// 提交域标签编辑并结束编辑状态。
void SurfaceDomainState::Edit::commit() { finished=true; state.editing=false; }
/// 应用网格变化并校验局部域标签，失败时回滚。
bool SurfaceDomainState::apply(const Mesh2D& mesh,const MeshEditDelta& delta,std::string* error) {
    Edit edit(*this);
    if (!edit.consume(delta,error) || !edit.validate(mesh,delta.changedSlots,error)) return false;
    edit.commit(); return true;
}
/// 将当前域标签与完整重新分类的结果对照。
bool SurfaceDomainState::verify(const Mesh2D& mesh,const std::vector<std::pair<int,int>>& edges,
                               std::string* error,MeshObservation observation) const {
    std::vector<bool> full;
    if (!classifyConstrainedDomain(mesh,edges,full,error,observation)) return false;
    return full==labels || fail(error,"local domain labels differ from full classification");
}
} // namespace mesher
