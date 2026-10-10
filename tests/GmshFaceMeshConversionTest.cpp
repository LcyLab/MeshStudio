/** @file GmshFaceMeshConversionTest.cpp
 * @brief 周期副本不折叠、原物理XYZ/参数/lc保留及BDS创建/过滤/控制边界。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/PeriodicSeamReconstruction.h"
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* text) { if(!ok) { ++failures; std::cerr << "FAIL " << text << '\n'; } }
BdsMesh fixture() {
    BdsMesh mesh;
    for(int i = 1; i <= 6; ++i) {
        BdsPoint p; p.id = i; p.physicalId = i <= 2 ? 10 : i == 3 ? 11 : i == 4 ? 0 : 13;
        p.uv = {double(i),double(i*i)}; p.point = {double(i),double(i+1),double(i+2)}; insertBdsPoint(mesh,p);
    }
    appendBdsTriangle(mesh,{1,2,3}); appendBdsTriangle(mesh,{1,3,4}); return mesh;
}
std::vector<FaceVertex> originals() {
    FaceVertex a; a.id = a.physicalId = 10; a.dimension = 0; a.entity = 21; a.point = {9.,8.,7.};
    FaceVertex b = a; b.id = b.physicalId = 11; b.dimension = 1; b.entity = 22; b.parameters = {.3,0.}; b.storedSize = 2.5;
    FaceVertex c = b; c.id = c.physicalId = 13; c.parameters = {.7,0.}; c.storedSize = 3.5;
    return {a,b,c};
}
}
int main() {
    const auto bds = fixture(); FaceMesh initial; auto status = initializeFaceMeshVertices(bds,5,originals(),initial);
    check(status.complete() && initial.vertices.size() == 3 && initial.bdsToVertex.size() == 5 && initial.maximumVertexId == 13,"initialize selected originals and local map without coordinate welding");
    FaceMesh reconstructed;
    // 手工原视图使seam/转换RED不被初始化stub遮住。
    initial.entity = 5; initial.maximumVertexId = 13; for(auto p : originals()) initial.vertices[p.id] = p;
    initial.bdsToVertex = {{1,10},{2,10},{3,11},{5,13},{6,13}};
    auto seam = reconstructPeriodicSeamVertices(bds,initial,reconstructed);
    check(seam.status.complete() && seam.duplicatedLocalNodes == std::vector<int>({2,6}) && reconstructed.vertices.size() == 5,"copy 0D and orphan 1D duplicates in local ID order");
    if(seam.status.complete()) {
        check(reconstructed.equivalence.at(14) == 10 && reconstructed.equivalence.at(15) == 13 && reconstructed.bdsToVertex.at(2) == 14 && reconstructed.bdsToVertex.at(6) == 15,"seam copies retain separate identity and fake-to-original mapping");
        const auto& fake = reconstructed.vertices.at(15);
        check(fake.point.x == 9. && fake.point.y == 8. && fake.parameters.x == .7 && fake.storedSize == 3.5 && fake.dimension == 1 && fake.entity == 22,"copy original physical XYZ and edge parameters/lc rather than BDS coordinates");
        check(reconstructed.parametricCoordinates.at(14).x == 2. && reconstructed.parametricCoordinates.at(10).x == 1. && reconstructed.triangles.empty(),"separate UV per seam identity and no triangle conversion");
        FaceMesh converted; const auto result = convertBdsToFaceMesh(bds,reconstructed,converted);
        check(result.status.complete() && converted.triangles.size() == 2 && result.createdLocalNodes == std::vector<int>({4}) && converted.interiorVertices == std::vector<std::uint64_t>({16}),"conversion retains seam triangles and appends unmapped face node");
        if(result.status.complete()) {
            const auto& p = converted.vertices.at(16);
            check(p.origin == FaceVertexOrigin::FaceCreated && !p.physicalId && p.dimension == 2 && p.entity == 5 && p.point.x == 4. && p.parameters.x == 4. && p.parameters.y == 16.,"new face vertex directly stores BDS XYZ and UV");
        }
    }
    FaceMesh ordinary; auto conversion = convertBdsToFaceMesh(bds,initial,ordinary);
    check(conversion.status.complete() && ordinary.triangles.size() == 1 && conversion.collapsedTriangles == std::vector<int>({0}),"ordinary conversion filters repeated MVertex identity only");
    auto sameXYZ = fixture(); sameXYZ.points.at(2).physicalId = 13;
    conversion = convertBdsToFaceMesh(sameXYZ,FaceMesh{},ordinary);
    check(conversion.status.code == MeshingStatus::InvalidInput,"invalid face view rejected");
    auto unsupported = initial; unsupported.vertices.at(10).dimension = 2; unsupported.vertices.at(10).entity = 5;
    seam = reconstructPeriodicSeamVertices(bds,unsupported,reconstructed);
    check(seam.status.complete() && reconstructed.unreconstructedSeams == std::vector<int>({2}) && reconstructed.bdsToVertex.at(2) == 10 && reconstructed.parametricCoordinates.at(10).x == 1.,"unsupported repeated face vertex diagnosed without invented copy or second UV overwrite");
    FaceMesh sentinel; sentinel.entity = 999; RunControl cancel; cancel.keepGoing = [] { return false; };
    check(initializeFaceMeshVertices(bds,5,originals(),sentinel,cancel).code == MeshingStatus::Cancelled && sentinel.entity == 999,"initialization cancellation atomic");
    check(reconstructPeriodicSeamVertices(bds,initial,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.entity == 999,"seam cancellation atomic");
    check(convertBdsToFaceMesh(bds,initial,sentinel,cancel).status.code == MeshingStatus::Cancelled && sentinel.entity == 999,"conversion cancellation atomic");
    RunControl budget; budget.maximumNodes = 3;
    check(reconstructPeriodicSeamVertices(bds,initial,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.entity == 999,"copy allocation budget keeps output");
    budget.maximumNodes = 3;
    check(convertBdsToFaceMesh(bds,initial,sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.entity == 999,"new face allocation budget keeps output");
    return failures ? 1 : 0;
}
