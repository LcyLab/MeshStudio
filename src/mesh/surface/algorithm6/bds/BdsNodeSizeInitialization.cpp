/** @file BdsNodeSizeInitialization.cpp
 * @brief 普通BDS节点BGM尺寸阶段。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsNodeSizeInitialization.h"
#include <exception>
namespace mesher::gmshReference {
/// 初始化普通面 BDS 节点的网格尺寸和背景尺寸。
BdsNodeSizeInitializationResult initializeOrdinaryBdsNodeSizes(const BdsMesh& input,
    const BdsRecoveredVertexQueries& recoveredVertices,double modelCharacteristicLength,
    const SurfaceMeshingOptions& options,const GeometrySizeSource& source,BdsMesh& output,
    bool onlyInitialMesh,const RunControl& control)
{
    BdsNodeSizeInitializationResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"BDS node size initialization cancelled"}}; };
        auto require = [](OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; };
        check(); require(validateBdsTopology(input,control)); BdsMesh mesh = input;
        result.skippedInitialOnly = onlyInitialMesh;
        if(!onlyInitialMesh) {
            for(const auto& vertex : recoveredVertices) {
                check();
                if(mesh.points.count(vertex.first) == 0 || vertex.second.dimension < 0 || vertex.second.dimension > 3)
                    throw OperationFailure{{MeshingStatus::InvalidInput,"invalid recovered BDS vertex metadata"}};
            }
            for(auto& entry : mesh.points) {
                check(); const auto recovered = recoveredVertices.find(entry.first);
                if(recovered == recoveredVertices.end()) continue;
                MeshSizeQuery query = recovered->second;
                double size = 1.e22;
                if(query.dimension < 2) {
                    if(result.queries.size() >= control.maximumGeometryEvaluations)
                        throw OperationFailure{{MeshingStatus::BudgetExceeded,"BDS background size query budget exceeded"}};
                    query.v = 0.; if(query.dimension == 0) query.u = 0.;
                    auto evaluation = evaluateMeshSize(query,modelCharacteristicLength,options,source);
                    result.queries.push_back({entry.first,query,evaluation}); require(evaluation.status);
                    size = evaluation.size;
                } else result.unconstrainedPoints.push_back(entry.first);
                entry.second.sizes = {size,size};
            }
        }
        check(); require(validateBdsTopology(mesh,control)); output = std::move(mesh);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
}
