// 文件职责：默认表面路线的共享边登记：保持 CAD 曲线节点身份一致，向各面提供同一份离散边界。
#include "CadBoundaryRegistry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <set>

namespace mesher {

/// 登记共享 CAD 边关联的面，避免重复登记。
void CadBoundaryRegistry::addUse(int id,int face) {
    auto& uses=m_edges.at(id).incidentFaces;
    if (std::find(uses.begin(),uses.end(),face)==uses.end()) uses.push_back(face);
}

/// 请求在边界参数区间内增加中点采样。
bool CadBoundaryRegistry::requestSplit(int id,double left,double right,
                                       double& parameter,std::string* error) {
    return registerSplit(id,left,right,left+(right-left)*.5,parameter,error);
}
/// 校验新增参数并登记边界细分，使对应边的缓存失效。
bool CadBoundaryRegistry::registerSplit(int id,double left,double right,double selected,
                                       double& parameter,std::string* error) {
    auto& e=m_edges.at(id);
    if (e.version==std::numeric_limits<std::uint64_t>::max()) {
        if (error) *error="shared boundary version exhausted"; return false;
    }
    if (!std::isfinite(selected)||!(selected>left && selected<right) || e.nodeIds.count(selected)) {
        if (error) *error="shared edge split has no new representable parameter";
        return false;
    }
    e.nodeIds[selected]=-1; // 只有边采样成功后才分配节点。
    invalidate(id);
    parameter=selected;
    return true;
}

/// 更新共享边版本并使采样状态失效。
void CadBoundaryRegistry::invalidate(int id) {
    auto& e=m_edges.at(id);
    if (e.version==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("shared boundary version exhausted");
    ++e.version; e.ready=false;
}
/// 检查边界细分请求是否仍匹配当前边版本和参数区间。
bool CadBoundaryRegistry::current(const SplitRequest& request) const {
    if (request.edge<0 || request.edge>=int(m_edges.size()) || !std::isfinite(request.left) ||
        !std::isfinite(request.right) || !(request.left<request.right) ||
        (request.purpose!=CadBoundarySplitPurpose::Geometry&&request.purpose!=CadBoundarySplitPurpose::Quality)) return false;
    const auto& e=m_edges[request.edge];
    if (!e.ready || request.boundaryVersion!=e.version) return false;
    const auto found=std::lower_bound(e.samples.begin(),e.samples.end(),request.left,
        [](const CadBoundarySample& p,double t){return p.t<t;});
    return found!=e.samples.end() && found->t==request.left && found+1!=e.samples.end() && (found+1)->t==request.right;
}
/// 默认中点；显式参数须为未登记的有限内部参数，同区间跨面请求沿用首次选择。
bool CadBoundaryRegistry::enqueueSplit(int id,double left,double right,int face,const std::string& reason,
                                       std::uint64_t& requestId,std::string* error,std::optional<double> parameter,
                                       CadBoundarySplitPurpose purpose) {
    if (id<0 || id>=int(m_edges.size()) || face<0 ||
        (purpose!=CadBoundarySplitPurpose::Geometry&&purpose!=CadBoundarySplitPurpose::Quality)) {
        if (error) *error="invalid shared split edge/source face"; return false;
    }
    SplitRequest request{m_nextRequest,m_edges[id].version,id,face,left,right,reason,parameter.value_or(left+(right-left)*.5)};
    request.purpose=purpose;
    if (!current(request)) { if (error) *error="shared split interval is stale or not adjacent"; return false; }
    if(parameter&&(!std::isfinite(*parameter)||!(*parameter>left&&*parameter<right)||m_edges[id].nodeIds.count(*parameter))){
        if(error)*error="explicit shared split parameter is invalid or already registered";return false;}
    const RequestKey key{id,request.boundaryVersion,left,right};
    const auto prior=m_requests.find(key);
    if (prior!=m_requests.end()) {
        // 任一邻面要求几何分边，就不能把同一区间当作纯质量分点冻结尺寸。
        if(purpose==CadBoundarySplitPurpose::Geometry)prior->second.purpose=purpose;
        requestId=prior->second.id; return true;
    }
    if (m_nextRequest==std::numeric_limits<std::uint64_t>::max()) {
        if (error) *error="shared split request id exhausted"; return false;
    }
    m_requests.emplace(key,request); ++m_nextRequest; requestId=request.id;
    MeshObservation{m_observer,{1,id}}.count(MeshCounter::BoundaryQueueRequests,1);
    if (m_requests.size()>m_peakRequests) {
        MeshObservation{m_observer,{}}.count(MeshCounter::BoundaryQueuePeakEntries,m_requests.size()-m_peakRequests);
        m_peakRequests=m_requests.size();
    }
    return true;
}
/// 取出仍有效的边界细分请求，丢弃过期项。
bool CadBoundaryRegistry::popSplitRequest(SplitRequest& result) {
    while (!m_requests.empty()) {
        auto it=m_requests.begin(); auto request=std::move(it->second); m_requests.erase(it);
        if (!current(request)) {
            MeshObservation{m_observer,{1,request.edge}}.count(MeshCounter::BoundaryQueueStaleRequests,1); continue;
        }
        result=std::move(request); return true;
    }
    return false;
}
/// 应用一条与当前版本一致的边界细分请求。
bool CadBoundaryRegistry::applySplit(const SplitRequest& request,double& parameter,std::string* error) {
    if (!current(request)) { if (error) *error="shared split request boundary version is stale"; return false; }
    return registerSplit(request.edge,request.left,request.right,request.parameter,parameter,error);
}
/// 先校验同一边界快照上的全部请求，再统一登记细分。
bool CadBoundaryRegistry::applySplitBatch(const std::vector<SplitRequest>& requests,
    std::vector<double>& parameters,std::string* error) {
    std::vector<double> pending;std::set<std::pair<int,double>> nodes;std::set<int> affected;
    for(const auto& request:requests) {
        if(!current(request)){if(error)*error="shared split batch contains a stale interval";return false;}
        const auto& edge=m_edges[request.edge];
        const double mid=request.parameter;
        if(!std::isfinite(mid)||!(mid>request.left && mid<request.right) || edge.nodeIds.count(mid) ||
            !nodes.emplace(request.edge,mid).second || edge.version==std::numeric_limits<std::uint64_t>::max()) {
            if(error)*error="shared split batch has no new representable parameter or exhausted version";return false;
        }
        affected.insert(request.edge);pending.push_back(mid);
    }
    for(size_t i=0;i<requests.size();++i)m_edges[requests[i].edge].nodeIds[pending[i]]=-1;
    for(int edge:affected)invalidate(edge);
    parameters=std::move(pending);return true;
}

} // namespace mesher
