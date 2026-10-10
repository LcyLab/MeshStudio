/** @file DomainClassification.cpp
 * @brief 保留两次 Color 的 LIFO/重复访问/other_side 覆盖。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "DomainClassification.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include <exception>
namespace mesher::gmshReference {
/// 依据闭合边界和孔洞对初始三角形分类。
DomainClassificationResult classifyFaceDomain(const WalkTriangulation& input, int faceEntity,
    WalkTriangulation& output, const RunControl& control)
{
    DomainClassificationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial domain classification cancelled"}}; };
        check(); const auto valid = validateHalfEdgeTopology(input,control);
        if(!valid.complete()) throw OperationFailure{valid};
        if(faceEntity <= 0 || input.vertices.size() < 4) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid ordinary Color input"}};
        for(const auto& f : input.faces) if(f.domain != -1) throw OperationFailure{{MeshingStatus::InvalidInput,"ordinary Color requires unknown input domains"}};
        WalkTriangulation mesh = input;
        auto colorRegion = [&](int seed, int color, std::vector<int>& visits) {
            std::vector<int> stack{mesh.halfEdges[seed].face}; int otherSide = -1;
            while(!stack.empty()) {
                check(); const int face = stack.back(); stack.pop_back();
                visits.push_back(face); mesh.faces[face].domain = color;
                int edge = mesh.faces[face].halfEdge;
                for(int i = 0; i < 3; ++i) {
                    const auto& e = mesh.halfEdges[edge];
                    if(e.curveEntity == -1 && e.opposite >= 0 && mesh.faces[mesh.halfEdges[e.opposite].face].domain == -1)
                        stack.push_back(mesh.halfEdges[e.opposite].face);
                    else if(e.curveEntity != -1 && e.opposite >= 0) otherSide = e.opposite;
                    edge = e.next;
                }
            }
            return otherSide;
        };
        result.interiorSeedHalfEdge = colorRegion(mesh.vertices[0].incidentHalfEdge,-2,result.exteriorVisits);
        if(result.interiorSeedHalfEdge < 0) throw OperationFailure{{MeshingStatus::NonConverged,"native Color has no interior seed"}};
        result.finalOppositeHalfEdge = colorRegion(result.interiorSeedHalfEdge,faceEntity,result.interiorVisits);
        check(); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
