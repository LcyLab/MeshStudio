/** @file BdsDomainClassification.cpp
 * @brief 普通/周期BDS域标记实现。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsDomainClassification.h"
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 按普通或周期图表规则标记 BDS 网格的内部区域。
BdsDomainClassificationResult classifyBdsDomain(const BdsMesh& input,BdsMesh& output,bool periodic,const RunControl& control)
{
    BdsDomainClassificationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS domain classification cancelled"}}; };
        auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
        check(); require(validateBdsTopology(input,control)); BdsMesh mesh = input;
        const BdsClassification interior{1,2,BdsClassificationOrigin::FaceInterior};
        const BdsClassification exterior{periodic ? 3 : 1,periodic ? 2 : 3,BdsClassificationOrigin::FaceExterior};
        auto isExterior = [&](BdsClassification value) { return value.origin == exterior.origin && value.tag == exterior.tag && value.dimension == exterior.dimension; };
        auto fill = [&](int seed,BdsClassification marker) {
            result.fills.push_back({seed,marker,{}}); auto& visits = result.fills.back().visits; std::vector<int> stack{seed};
            while(!stack.empty()) {
                check(); const int index = stack.back(); stack.pop_back(); auto& face = mesh.triangles[index];
                const bool empty = face.classification.dimension < 0; visits.emplace_back(index,empty);
                if(!empty) continue;
                face.classification = marker;
                for(int edge : face.edges) {
                    const auto& e = mesh.edges[edge];
                    if(e.classification.dimension < 0 && e.faces.size() == 2) stack.push_back(e.faces[0] == index ? e.faces[1] : e.faces[0]);
                }
            }
        };
        for(auto& face : mesh.triangles) { check(); face.classification = {}; }
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            check(); if(periodic && mesh.triangles[i].deleted) continue;
            const auto nodes = getBdsTriangleNodes(mesh,int(i));
            if(nodes[0] < 0 || nodes[1] < 0 || nodes[2] < 0) { fill(int(i),exterior); break; }
        }
        for(const auto& edge : mesh.edges) {
            check(); if(edge.classification.dimension < 0 || edge.faces.size() != 2) continue;
            if(isExterior(mesh.triangles[edge.faces[0]].classification)) { fill(edge.faces[1],interior); break; }
            if(isExterior(mesh.triangles[edge.faces[1]].classification)) { fill(edge.faces[0],interior); break; }
        }
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            check(); if(isExterior(mesh.triangles[i].classification)) { result.clearedExteriorTriangles.push_back(int(i)); mesh.triangles[i].classification = {}; }
        }
        // 原版普通面还有负对顶点补种子；周期面没有这一次填充。
        if(!periodic) for(std::size_t i = 0; i < mesh.edges.size(); ++i) {
            check(); const auto& edge = mesh.edges[i]; if(edge.classification.dimension < 0 || edge.faces.size() != 2) continue;
            const auto neighborhood = getBdsEdgeNeighborhood(mesh,int(i));
            if(!neighborhood) continue;
            if(neighborhood->oppositeNodes[0] < 0) { fill(edge.faces[1],interior); break; }
            if(neighborhood->oppositeNodes[1] < 0) { fill(edge.faces[0],interior); break; }
        }
        for(const auto& face : mesh.triangles) if(!face.deleted && face.classification.origin == BdsClassificationOrigin::FaceInterior) ++result.interiorTriangles;
        check(); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
/// 对普通面 BDS 网格分类，去除计算域外部分。
BdsDomainClassificationResult classifyOrdinaryBdsDomain(const BdsMesh& input,BdsMesh& output,const RunControl& control)
{
    return classifyBdsDomain(input,output,false,control);
}
/// 按周期边界和参数分支分类 BDS 计算域。
BdsDomainClassificationResult classifyPeriodicBdsDomain(const BdsMesh& input,BdsMesh& output,const RunControl& control)
{
    return classifyBdsDomain(input,output,true,control);
}
}
