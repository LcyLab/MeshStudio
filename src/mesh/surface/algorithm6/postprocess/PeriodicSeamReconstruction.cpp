/** @file PeriodicSeamReconstruction.cpp
 * @brief 周期临时副本和各副本UV身份。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "PeriodicSeamReconstruction.h"
#include <exception>
#include <limits>
#include <set>
namespace mesher::gmshReference {
/// 恢复周期接缝的节点等价关系及面参数分支。
PeriodicSeamReconstructionResult reconstructPeriodicSeamVertices(const BdsMesh& bds,const FaceMesh& input,FaceMesh& output,const RunControl& control) {
    PeriodicSeamReconstructionResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"periodic seam reconstruction cancelled"}}; };
        check(); const auto valid = validateFaceMeshVertices(bds,input,control); if(!valid.complete()) throw OperationFailure{valid};
        FaceMesh mesh = input; std::set<std::uint64_t> seen;
        for(auto& entry : mesh.bdsToVertex) {
            check(); const auto id = entry.second; const auto& original = mesh.vertices.at(id);
            if(seen.insert(id).second) { mesh.parametricCoordinates[id] = bds.points.at(entry.first).uv; continue; }
            if(original.dimension >= 2) { mesh.unreconstructedSeams.push_back(entry.first); continue; }
            if(mesh.vertices.size() >= control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"periodic copy vertex budget exceeded"}};
            if(mesh.maximumVertexId == std::numeric_limits<std::uint64_t>::max()) throw OperationFailure{{MeshingStatus::InvalidInput,"periodic copy identity overflow"}};
            FaceVertex fake = original; fake.id = ++mesh.maximumVertexId; fake.origin = FaceVertexOrigin::PeriodicCopy;
            mesh.vertices.emplace(fake.id,fake); entry.second = fake.id;
            mesh.equivalence.emplace(fake.id,id); mesh.parametricCoordinates[fake.id] = bds.points.at(entry.first).uv;
            seen.insert(fake.id); result.duplicatedLocalNodes.push_back(entry.first);
        }
        check(); const auto final = validateFaceMeshVertices(bds,mesh,control); if(!final.complete()) throw OperationFailure{final};
        output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
