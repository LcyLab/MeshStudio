/** @file GmshMeshVertexParametersTest.cpp
 * @brief 原seam候选序、参数0语义、单点与邻边差异、首候选参照和共享预算。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/geometry/MeshVertexParameters.h"
#include <iostream>
#include <stdexcept>
using namespace mesher; using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
bool equal(Point2D a,Point2D b) { return a.x == b.x && a.y == b.y; }
MeshVertexParameterInput node(int dimension,int entity,Point2D uv = {}) {
    MeshVertexParameterInput input; input.vertex.dimension = dimension; input.vertex.entity = entity;
    input.vertex.id = entity+100; input.vertex.parameters = uv; input.vertex.point = {double(entity),2.,3.}; return input;
}
}
int main() {
    MeshVertexParameterSource source; source.face = 10; std::vector<std::array<double,3>> calls; int projections = 0;
    source.curves[5] = {1,1,2.,9.,true,true,true}; source.curves[3] = {1,2,4.,8.,true,true,true};
    source.vertexIncidentCurves[1] = {5,3};
    source.edgeCoordinates = [&](int edge,double t,int dir) { calls.push_back({double(edge),t,double(dir)}); return Point2D{t,double(dir)}; };
    source.vertexCoordinates = [](int entity,int dir) { return Point2D{double(entity),double(dir)}; };
    source.edgeParameterFromPoint = [&](int,Point3D) { ++projections; return 6.; };
    source.surfaceCoordinates = [](Point3D p,bool onSurface) { return Point2D{p.x,onSurface ? p.y : p.z}; };
    auto v = node(0,1); auto all = collectMeshVertexParameters(v,source);
    check(all.status.complete() && all.seamPresent && all.parameters.size() == 6 && all.geometryEvaluations == 6,"seam endpoint candidates preserve both endpoints and source edge order");
    check(calls == std::vector<std::array<double,3>>{{5.,2.,-1.},{5.,2.,1.},{5.,9.,-1.},{5.,9.,1.},{3.,4.,-1.},{3.,4.,1.}},"closed seam adds low and high without deduplication");
    source.curves[5].seam = source.curves[3].seam = false; all = collectMeshVertexParameters(v,source);
    check(all.status.complete() && all.parameters.size() == 1 && equal(all.parameters[0],{1.,1.}),"ordinary CAD vertex uses vertex reparam rather than edge candidate enumeration");
    source.curves[5].seam = true;
    auto single = reparameterizeMeshVertexOnFace(v,source,{true,true,-1});
    check(single.status.complete() && !single.succeeded && equal(single.parameter,{1.,-1.}),"single seam rejection retains already computed vertex parameter");
    single = reparameterizeMeshVertexOnFace(v,source);
    check(single.status.complete() && !single.succeeded && equal(single.parameter,{1.,1.}),"native single-point default reports seam after computing UV");
    v = node(1,5,{0.,7.}); calls.clear(); projections = 0; all = collectMeshVertexParameters(v,source);
    check(all.status.complete() && projections == 1 && calls.size() == 2 && calls[0][1] == 6. && all.geometryEvaluations == 3,"edge candidate parameter zero invokes inverse then +1/-1");
    projections = 0; calls.clear(); single = reparameterizeMeshVertexOnFace(v,source,{true,false,1});
    check(single.status.complete() && single.succeeded && projections == 0 && calls[0][1] == 0.,"single edge parameter zero is stored value and not inverse projected");
    source.curves[5].parametrized = false; all = collectMeshVertexParameters(v,source);
    check(all.status.complete() && all.parameters.empty() && all.geometryEvaluations == 0,"curve without parametrization has no candidates");
    source.curves[5].parametrized = true; v.hasParameters[0] = false; single = reparameterizeMeshVertexOnFace(v,source);
    check(!single.status.complete() && !single.succeeded,"missing edge parameter explicitly rejects undefined native input");
    auto a = node(2,10,{2.,3.}),b = node(2,11,{8.,9.});
    auto pair = reparameterizeMeshEdgeOnFace(a,b,source);
    check(pair.status.complete() && pair.usedSurfaceFallback && equal(pair.parameters[0],{10.,2.}) && equal(pair.parameters[1],{11.,2.}) && pair.geometryEvaluations == 2,"empty second candidates causes both points to use surface inverse");
    single = reparameterizeMeshVertexOnFace(a,source);
    check(single.status.complete() && single.succeeded && equal(single.parameter,{2.,3.}) && single.geometryEvaluations == 0,"same-face stored UV does not query CAD");
    a.hasParameters[1] = false; single = reparameterizeMeshVertexOnFace(a,source,{false,false,1});
    check(single.status.complete() && single.succeeded && equal(single.parameter,{10.,3.}),"incomplete face parameters uses inverse with onSurface flag");
    source.discreteSurface = true; all = collectMeshVertexParameters(node(0,1),source);
    check(all.status.complete() && all.parameters.size() == 1 && equal(all.parameters[0],{1.,2.}),"discrete surface overrides candidate CAD classification"); source.discreteSurface = false;
    a = node(1,5); a.kind = ParameterEntityKind::DiscreteCurve; single = reparameterizeMeshVertexOnFace(a,source,{false,true,-1});
    check(single.status.complete() && single.succeeded && equal(single.parameter,{5.,3.}),"discrete curve single path bypasses seam and uses requested surface inverse");
    a = node(0,1); a.kind = ParameterEntityKind::GmshVertex; source.plane = true;
    single = reparameterizeMeshVertexOnFace(a,source,{false,false,1});
    check(single.status.complete() && equal(single.parameter,{1.,3.}),"native Gmsh plane vertex single hack uses surface inverse"); source.plane = false;
    // Each side chooses against the other's first candidate: not nearest global pair.
    source.curves[5].seam = true; source.curves[3].seam = true;
    source.edgeCoordinates = [](int edge,double,int dir) { return Point2D{edge == 5 ? (dir == 1 ? 0. : 100.) : (dir == 1 ? 90. : 10.),0.}; };
    a = node(1,5,{1.,0.}); b = node(1,3,{1.,0.}); pair = reparameterizeMeshEdgeOnFace(a,b,source);
    check(pair.status.complete() && equal(pair.parameters[0],{100.,0.}) && equal(pair.parameters[1],{10.,0.}) && pair.selectedCandidates == std::array<std::size_t,2>{1,1},"pair choices independently reference first candidate of opposite endpoint");
    source.edgeCoordinates = [](int edge,double,int dir) { return Point2D{edge == 5 ? (dir == 1 ? -1. : 1.) : 0.,0.}; };
    pair = reparameterizeMeshEdgeOnFace(a,b,source);
    check(pair.status.complete() && pair.selectedCandidates == std::array<std::size_t,2>{0,0},"strict nearest tie keeps original first candidates");
    RunControl control; control.maximumGeometryEvaluations = 3; pair = reparameterizeMeshEdgeOnFace(a,b,source,control);
    check(pair.status.code == MeshingStatus::BudgetExceeded && pair.geometryEvaluations == 3,"two endpoint collection shares one geometry budget");
    control = {}; control.keepGoing = [] { return false; }; all = collectMeshVertexParameters(a,source,control);
    check(all.status.code == MeshingStatus::Cancelled && all.geometryEvaluations == 0,"entry cancellation does not query geometry");
    source.edgeCoordinates = [](int,double,int)->Point2D { throw std::runtime_error("injected edge query failure"); };
    pair = reparameterizeMeshEdgeOnFace(a,b,source);
    check(pair.status.code == MeshingStatus::GeometryQueryFailed && pair.status.message == "injected edge query failure","geometry exceptions remain explicit");
    a.classified = false; single = reparameterizeMeshVertexOnFace(a,source);
    check(single.status.code == MeshingStatus::InvalidInput && !single.succeeded,"unclassified single node is explicit failure");
    return failures ? 1 : 0;
}
