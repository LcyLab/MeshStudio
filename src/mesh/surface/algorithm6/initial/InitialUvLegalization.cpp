/** @file InitialUvLegalization.cpp
 * @brief 原生普通初始内域的有界 UV 翻边，不执行 BDS 度量合法化。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "InitialUvLegalization.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include "ConstraintRecovery.h"
#include "geometry/predicates/GeomPredicates.h"
#include <exception>
namespace mesher::gmshReference {
/// 在保留约束边的条件下合法化初始 UV 网格。
InitialUvLegalizationResult legalizeInitialUvMesh(const WalkTriangulation& input, int faceEntity,
    WalkTriangulation& output, const RunControl& control)
{
    InitialUvLegalizationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"initial UV legalization cancelled"}}; };
        check(); const auto valid = validateHalfEdgeTopology(input,control);
        if(!valid.complete()) throw OperationFailure{valid};
        if(faceEntity <= 0) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid initial UV face entity"}};
        WalkTriangulation mesh = input;
        for(int pass = 0; pass < 100; ++pass) {
            std::vector<int> flips;
            for(std::size_t i = 0; i < mesh.halfEdges.size(); ++i) {
                check(); const auto& e = mesh.halfEdges[i];
                if(e.opposite < 0 || mesh.faces[e.face].domain != faceEntity || mesh.faces[mesh.halfEdges[e.opposite].face].domain != faceEntity) continue;
                const int e1 = e.next, e2 = mesh.halfEdges[e1].next;
                const int o1 = mesh.halfEdges[e.opposite].next, o2 = mesh.halfEdges[o1].next;
                const auto a = mesh.vertices[e.origin].uv, b = mesh.vertices[mesh.halfEdges[e1].origin].uv;
                const auto c = mesh.vertices[mesh.halfEdges[e2].origin].uv, d = mesh.vertices[mesh.halfEdges[o2].origin].uv;
                if(incircleOriented(a,b,c,d) < 0 && intersectRecoverySegments(a,b,c,d)) {
                    swapHalfEdgeDiagonal(mesh,int(i)); flips.push_back(int(i));
                }
            }
            result.flippedHalfEdges.push_back(std::move(flips));
            if(result.flippedHalfEdges.back().empty()) break;
        }
        result.passLimitReached = result.flippedHalfEdges.size() == 100 && !result.flippedHalfEdges.back().empty();
        check(); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
