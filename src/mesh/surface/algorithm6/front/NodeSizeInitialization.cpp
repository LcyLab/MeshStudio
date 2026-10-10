/** @file NodeSizeInitialization.cpp
 * @brief 前沿节点尺寸阶段。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "NodeSizeInitialization.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
/// 为前沿节点初始化曲面参数、网格尺寸和背景尺寸。
NodeSizeInitializationResult initializeNodeSizes(const FaceMesh& mesh,
    const NodeSizeInitializationInput& input,const NodeReparameterizationSource& reparameterize,
    double modelCharacteristicLength,const SurfaceMeshingOptions& options,
    const GeometrySizeSource& source,NodeSizeState& output,const RunControl& control)
{
    NodeSizeInitializationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"front node size initialization cancelled"}}; };
        auto fail = [](MeshingStatus code,const char* message) { throw OperationFailure{{code,message}}; };
        check();
        if(mesh.entity <= 0) fail(MeshingStatus::InvalidInput,"invalid front face entity");
        for(const auto& entry : mesh.vertices) {
            check(); const auto& v = entry.second;
            if(!entry.first || entry.first != v.id || v.dimension < 0 || v.dimension > 2 || v.entity <= 0 ||
               !std::isfinite(v.point.x) || !std::isfinite(v.point.y) || !std::isfinite(v.point.z))
                fail(MeshingStatus::InvalidInput,"invalid front vertex metadata");
        }
        auto vertex = [&](std::uint64_t id) -> const FaceVertex& {
            const auto found = mesh.vertices.find(id);
            if(found == mesh.vertices.end()) fail(MeshingStatus::InvalidTopology,"front mesh references missing vertex");
            return found->second;
        };
        for(const auto& e : mesh.equivalence) {
            check(); const auto& copy = vertex(e.first); const auto& original = vertex(e.second);
            if(copy.origin != FaceVertexOrigin::PeriodicCopy || original.origin != FaceVertexOrigin::Original || copy.dimension > 1 || copy.physicalId != original.physicalId)
                fail(MeshingStatus::InvalidTopology,"invalid front periodic equivalence");
        }
        for(const auto& e : mesh.parametricCoordinates) {
            check(); vertex(e.first);
            if(!std::isfinite(e.second.x) || !std::isfinite(e.second.y)) fail(MeshingStatus::InvalidInput,"invalid front periodic parameter override");
        }
        auto distance = [&](std::uint64_t a,std::uint64_t b) {
            const auto& p = vertex(a).point; const auto& q = vertex(b).point;
            const double dx = p.x-q.x, dy = p.y-q.y, dz = p.z-q.z;
            const double length = std::sqrt(dx*dx+dy*dy+dz*dz);
            if(!std::isfinite(length)) fail(MeshingStatus::InvalidInput,"nonfinite front edge length");
            return length;
        };
        std::map<std::uint64_t,double> sizes;
        for(const auto& t : mesh.triangles) {
            check();
            for(auto id : t) { vertex(id); sizes[id] = -1.; }
        }
        // 只排除fake与其original的直接等价关系，不按物理ID或坐标排除边。
        auto equivalent = [&](std::uint64_t a,std::uint64_t b) {
            const auto e = mesh.equivalence.find(a); return e != mesh.equivalence.end() && e->second == b;
        };
        for(const auto& t : mesh.triangles) {
            check();
            for(int i = 0; i < 3; ++i) for(int j = i+1; j < 3; ++j) {
                if(equivalent(t[i],t[j]) || equivalent(t[j],t[i])) continue;
                const double length = distance(t[i],t[j]);
                for(auto id : {t[i],t[j]}) if(sizes.at(id) < 0. || sizes.at(id) > length) sizes.at(id) = length;
            }
        }
        std::set<std::uint64_t> embeddedVertices;
        for(const auto& entry : input.embeddedVertices) {
            check(); vertex(entry.first);
            if(!std::isfinite(entry.second)) fail(MeshingStatus::InvalidInput,"nonfinite embedded prescribed size");
            // 未被三角形引用的embedded节点，原版map[]先插入0再取min。
            sizes[entry.first] = std::min(sizes[entry.first],entry.second);
            embeddedVertices.insert(entry.first);
        }
        NodeSizeState state;
        for(const auto& curve : input.embeddedCurves) {
            check(); if(curve.meshDegenerated) continue;
            for(auto segment : curve.segments) {
                check(); vertex(segment[0]); vertex(segment[1]);
                if(segment[1] < segment[0]) std::swap(segment[0],segment[1]);
                state.internalEdges.insert(segment);
            }
        }
        for(const auto& curve : input.boundaryCurves) {
            check(); if(curve.meshDegenerated) continue;
            for(const auto& segment : curve.segments) {
                check(); const double lower = .5*distance(segment[0],segment[1]);
                for(auto id : segment) if(sizes[id] < lower) sizes[id] = lower;
            }
        }
        if(sizes.size() > control.maximumNodes) fail(MeshingStatus::BudgetExceeded,"front node budget exceeded");
        if(input.vertexOrder.size() != sizes.size()) fail(MeshingStatus::InvalidInput,"front vertex order must cover initialized nodes");
        std::set<std::uint64_t> ordered;
        for(auto id : input.vertexOrder) {
            check();
            if(!sizes.count(id) || !ordered.insert(id).second) fail(MeshingStatus::InvalidInput,"invalid or duplicate front vertex order");
        }
        std::size_t evaluations = 0;
        auto queryBudget = [&] {
            check();
            if(evaluations >= control.maximumGeometryEvaluations) fail(MeshingStatus::BudgetExceeded,"front geometry query budget exceeded");
            ++evaluations;
        };
        for(auto id : input.vertexOrder) {
            queryBudget();
            if(!reparameterize) fail(MeshingStatus::GeometryQueryFailed,"front CAD reparameterization source missing");
            const auto& v = vertex(id); const Point2D raw = reparameterize(v);
            if(!std::isfinite(raw.x) || !std::isfinite(raw.y)) fail(MeshingStatus::GeometryQueryFailed,"nonfinite front CAD parameters");
            result.reparameterizations.push_back({id,raw}); double background = sizes.at(id);
            if(embeddedVertices.count(id)) {
                queryBudget(); const MeshSizeQuery query{2,mesh.entity,raw.x,raw.y,v.point};
                auto evaluation = evaluateMeshSize(query,modelCharacteristicLength,options,source);
                result.embeddedQueries.push_back({id,query,evaluation});
                if(!evaluation.status.complete()) throw OperationFailure{evaluation.status};
                background = evaluation.size;
            }
            Point2D uv = raw; const auto override = mesh.parametricCoordinates.find(id);
            if(override != mesh.parametricCoordinates.end()) uv = override->second;
            state.indices.emplace(id,state.nodes.size()); state.nodes.push_back({id,uv,{sizes.at(id),background}});
        }
        check(); output = std::move(state);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::GeometryQueryFailed,error.what()}; }
    return result;
}
}
