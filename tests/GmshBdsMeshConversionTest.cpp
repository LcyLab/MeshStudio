/** @file GmshBdsMeshConversionTest.cpp
 * @brief BDS 身份/邻接序、域标签隔离和转换失败原子性。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/bds/BdsMeshConversion.h"
#include "mesh/surface/algorithm6/data/HalfEdgeTopology.h"
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
BdsPoint point(int id) { BdsPoint p; p.id = id; p.uv = {double(id),double(id+1)}; p.point = {double(id),double(id+1),2.}; return p; }
bool throwsOperation(const std::function<void()>& call) { try { call(); return false; } catch(const OperationFailure&) { return true; } }
}
int main()
{
    BdsMesh topology;
    for(int id : {4,0,-1,7}) insertBdsPoint(topology,point(id));
    check(topology.points.size() == 4 && topology.maximumPointId == 7,"stable point identities and maximum");
    if(topology.points.size() == 4) {
        const int first = appendBdsTriangle(topology,{4,0,-1});
        const int second = appendBdsTriangle(topology,{0,4,7});
        check(first == 0 && second == 1 && topology.edges.size() == 5,"triangle append and shared edge reuse");
        if(topology.edges.size() == 5) {
            check(topology.edges[0].first == 0 && topology.edges[0].second == 4 && topology.edges[0].faces == std::vector<int>{0,1},"ID endpoint order and face append order");
            check(topology.points.at(4).edges == std::vector<int>{0,2,3},"point edge append order");
            check(getBdsTriangleNodes(topology,0) == std::array<int,3>{4,0,-1} && getBdsTriangleNodes(topology,1) == std::array<int,3>{0,4,7},"oriented nodes from unordered edge endpoints");
            check(findBdsEdge(topology,4,0) == 0 && findBdsEdge(topology,0,4) == 0,"edge lookup both directions");
            const int duplicate = appendBdsTriangle(topology,{4,0,-1});
            check(duplicate == 2 && topology.edges.size() == 5 && topology.edges[0].faces.size() == 3 && validateBdsTopology(topology).complete(),"native duplicate triangle and multi-face record");
            auto bad = topology; bad.points.at(4).edges.clear();
            check(validateBdsTopology(bad).code == MeshingStatus::InvalidTopology,"bidirectional adjacency validation");
        }
        check(throwsOperation([&] { insertBdsPoint(topology,point(4)); }),"duplicate node identity rejected");
        const auto edgeCount = topology.edges.size();
        check(throwsOperation([&] { appendBdsTriangle(topology,{4,0,123}); }) && topology.edges.size() == edgeCount,"missing point rejected before mutation");
    }
    FaceChart chart; chart.entity = 19;
    chart.initialVertices = {{10,{0.,0.}},{20,{2.,0.}},{30,{2.,2.}},{40,{0.,2.}},{50,{1.,.7}}};
    // BDS reparam 枚举与 PolyMesh getNodeCopies 顺序独立，且 UV 也可不同。
    const int insertion[] = {3,1,4,0,2};
    for(int i = 0; i < 5; ++i) {
        const auto& p = chart.initialVertices[insertion[i]];
        chart.vertices.push_back({i,p.physicalId,1,i+3,{p.uv.x+.125,p.uv.y+.25},{p.uv.x,p.uv.y,3.}});
    }
    OrdinaryInitialTriangulation initial; check(buildInitialTriangulation(chart,initial).complete(),"ordinary input built");
    for(auto& f : initial.mesh.faces) f.domain = -2;
    for(auto& e : initial.mesh.halfEdges) e.curveEntity = 99;
    BdsMesh converted; int calls = 0;
    const auto result = convertOrdinaryInitialMeshToBds(chart,initial.mesh,[&](Point2D uv) { ++calls; return Point3D{uv.x*2.,uv.y*3.,5.}; },converted);
    check(result.status.complete() && result.geometryEvaluations == 4 && calls == 4 && converted.points.size() == 9,"ordinary four CAD outer points");
    check(converted.triangles.size() == initial.mesh.faces.size() && !converted.triangles.empty(),"all ghost and exterior faces retained");
    if(converted.points.size() == 9 && !converted.triangles.empty()) {
        check(converted.points.at(0).physicalId == 40 && converted.points.at(0).uv.x == .125 && converted.points.at(0).sizes.meshSize == 1.e22,"independent BDS coordinates and initial sizes");
        check(converted.points.at(-1).physicalId == 0 && converted.points.at(-1).classification.dimension == 2 && converted.points.at(-1).classification.tag == 19,"negative outer identities and CAD classification");
        check(converted.triangles[0].classification.dimension == -1 && converted.edges[0].classification.dimension == -1,"PolyMesh domain and edge tags not transferred");
        check(validateBdsTopology(converted).complete(),"converted ordinary topology valid");
        auto missing = chart; missing.vertices.erase(missing.vertices.begin());
        BdsMesh skipped; const auto partial = convertOrdinaryInitialMeshToBds(missing,initial.mesh,[](Point2D uv){ return Point3D{uv.x,uv.y,0.}; },skipped);
        check(partial.status.complete() && !partial.skippedTriangles.empty() && skipped.triangles.size()+partial.skippedTriangles.size() == initial.mesh.faces.size(),"native missing aaa endpoint skip explicitly reported");
    }
    BdsMesh sentinel; sentinel.maximumPointId = 777;
    RunControl cancelled; cancelled.keepGoing = [] { return false; };
    check(convertOrdinaryInitialMeshToBds(chart,initial.mesh,{},sentinel,cancelled).status.code == MeshingStatus::Cancelled && sentinel.maximumPointId == 777,"cancel preserves output");
    RunControl budget; budget.maximumGeometryEvaluations = 3;
    check(convertOrdinaryInitialMeshToBds(chart,initial.mesh,[](Point2D uv){ return Point3D{uv.x,uv.y,0.}; },sentinel,budget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 777,"geometry budget preserves output");
    check(convertOrdinaryInitialMeshToBds(chart,initial.mesh,[](Point2D){ return Point3D{std::numeric_limits<double>::quiet_NaN(),0.,0.}; },sentinel).status.code == MeshingStatus::GeometryQueryFailed && sentinel.maximumPointId == 777,"non-finite geometry preserves output");
    check(convertOrdinaryInitialMeshToBds(chart,initial.mesh,[](Point2D) -> Point3D { throw 17; },sentinel).status.code == MeshingStatus::GeometryQueryFailed && sentinel.maximumPointId == 777,"unknown geometry failure explicit");
    RunControl nodesBudget; nodesBudget.maximumNodes = 8;
    check(convertOrdinaryInitialMeshToBds(chart,initial.mesh,[](Point2D uv){ return Point3D{uv.x,uv.y,0.}; },sentinel,nodesBudget).status.code == MeshingStatus::BudgetExceeded && sentinel.maximumPointId == 777,"outer nodes included in budget");
    InitialTriangulation periodic;
    for(int id : {1,3,8}) {
        PeriodicBoundaryVertex p; p.vertex = {id,std::uint64_t(id),0,-id,{double(id),0.},{double(id),0.,1.}};
        p.sizes = {double(id)+.5,double(id)+.75}; p.periodicCounterpart = id == 1 ? 3 : id == 3 ? 1 : -1;
        periodic.vertices.push_back(p);
    }
    periodic.triangles = {{1,3,8}};
    periodic.geometry.push_back({-77,1}); // 所有端点复用时仍登记该嵌入边。
    BdsMesh pmesh; auto presult = convertPeriodicInitialMeshToBds(periodic,pmesh);
    check(presult.status.complete() && pmesh.maximumPointId == 8 && pmesh.points.size() == 3 && pmesh.triangles.size() == 1,"periodic conversion preserves numbering gaps");
    check(pmesh.geometry.count({-77,1}) == 1,"unreferenced embedded curve classification retained");
    if(pmesh.points.size() == 3) check(pmesh.points.at(1).periodicCounterpart == 3 && pmesh.points.at(3).periodicCounterpart == 1 && pmesh.points.at(8).sizes.backgroundSize == 8.75 && pmesh.points.at(1).classification.tag == -1,"periodic counterpart size and negative classification");
    periodic.vertices[0].periodicCounterpart = 99;
    check(convertPeriodicInitialMeshToBds(periodic,sentinel).status.code == MeshingStatus::InvalidTopology && sentinel.maximumPointId == 777,"invalid counterpart atomic failure");
    return failures ? 1 : 0;
}
