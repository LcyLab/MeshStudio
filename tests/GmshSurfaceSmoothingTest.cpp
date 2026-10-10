/** @file GmshSurfaceSmoothingTest.cpp
 * @brief 原eta、加权顺序、临时恢复、五次试探、分类保护与跨阶段预算。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/postprocess/SurfaceSmoothing.h"
#include <cmath>
#include <iostream>
using namespace mesher;
using namespace mesher::gmshReference;
namespace {
int failures = 0;
void check(bool ok,const char* message) { if(!ok) { ++failures; std::cerr << "FAIL " << message << '\n'; } }
bool close(double a,double b) { return std::abs(a-b) < 1.e-13; }
FaceMesh fan() {
    FaceMesh mesh; mesh.entity = 1; mesh.maximumVertexId = 99;
    const Point2D points[] = {{0.,0.},{1.,0.},{1.,1.},{0.,1.},{.2,.3}};
    for(int i = 0; i < 5; ++i) { FaceVertex v; v.id = i+1; v.dimension = i == 4 ? 2 : 0; v.entity = i == 4 ? 1 : i+1; v.point = {points[i].x,points[i].y,0.}; v.parameters = points[i]; mesh.vertices.emplace(v.id,v); }
    mesh.triangles = {{1,2,5},{2,3,5},{3,4,5},{4,1,5}}; mesh.interiorVertices = {5}; return mesh;
}
SurfaceSmoothingSource source() {
    SurfaceSmoothingSource s; s.metric = [](Point2D) { return SurfaceMetric{1.,0.,1.}; };
    s.surfacePoint = [](Point2D uv) { return SmoothingSurfacePoint{{uv.x,uv.y,0.},true}; };
    s.reparameterizeVertex = [](const FaceVertex& v,const FaceMesh&) { return v.parameters; };
    s.reparameterizeEdge = [](const FaceVertex& a,const FaceVertex& b,const FaceMesh&) { return std::array<Point2D,2>{a.parameters,b.parameters}; };
    return s;
}
bool originalCenter(const FaceMesh& mesh) { const auto& v = mesh.vertices.at(5); return v.parameters.x == .2 && v.parameters.y == .3 && v.point.x == .2 && v.point.y == .3; }
}
int main() {
    auto quality = computeTriangleEta({Point3D{0.,0.,0.},{1.,0.,0.},{.5,std::sqrt(3.)*.5,0.}});
    check(quality.status.complete() && close(quality.eta,1.) && close(quality.minimumAngle,60.),"eta equilateral uses physical minimum angle");
    quality = computeTriangleEta({Point3D{0.,0.,0.},{1.,0.,0.},{0.,1.,0.}});
    check(quality.status.complete() && close(quality.eta,.75),"eta right triangle differs from gamma");
    quality = computeTriangleEta({Point3D{0.,0.,0.},{1.,0.,0.},{2.,0.,0.}});
    check(quality.status.complete() && quality.eta == 0.,"eta finite collinear triangle retains zero");
    std::vector<Point2D> metricQueries;
    auto position = computeSmoothedPosition({0.,0.},{{3,{1.,0.}},{1,{0.,1.}}},[&](Point2D uv) { metricQueries.push_back(uv); return SurfaceMetric{4.,0.,1.}; });
    check(position.status.complete() && close(position.position.x,2./3.) && close(position.position.y,1./3.) && position.totalWeight == 3. && position.geometryEvaluations == 2,"metric directional weighting, not uniform or length weighting");
    check(metricQueries.size() == 2 && metricQueries[0].x == 0. && position.neighbors.size() == 2 && position.neighbors[0].vertex == 1,"neighbors and metric requests in identity order");
    position = computeSmoothedPosition({0.,0.},{{1,{0.,0.}},{2,{1.,0.}}},[](Point2D) { return SurfaceMetric{1.,0.,1.}; });
    check(position.status.complete() && position.totalWeight == 1. && position.position.x == 1. && position.neighbors[0].weight == 0.,"coincident neighbor still queried and contributes native zero direction weight");
    position = computeSmoothedPosition({0.,0.},{},[](Point2D) { return SurfaceMetric{1.,0.,1.}; });
    check(position.status.code == MeshingStatus::NonConverged,"zero total weight explicit, no fabricated mean");
    auto mesh = fan(); auto s = source(); const std::vector<int> incident{0,1,2,3};
    auto acceptance = acceptSmoothedPosition(mesh,5,incident,{.2,.3},{.5,.5},s);
    check(acceptance.status.complete() && acceptance.accepted && acceptance.newMinimumEta > acceptance.oldMinimumEta && close(acceptance.oldArea,1.) && close(acceptance.newArea,1.),"improved eta and unchanged UV area accepted");
    check(originalCenter(mesh) && acceptance.geometryEvaluations == 25,"acceptance restores temporary point, keeps all old/new UV queries");
    acceptance = acceptSmoothedPosition(mesh,5,incident,{.2,.3},{2.,2.},s);
    check(acceptance.status.complete() && !acceptance.accepted && acceptance.newArea > acceptance.oldArea && originalCenter(mesh),"increased UV area rejected and restored");
    s.surfacePoint = [](Point2D) { return SmoothingSurfacePoint{}; };
    acceptance = acceptSmoothedPosition(mesh,5,incident,{.2,.3},{.5,.5},s);
    check(acceptance.status.complete() && !acceptance.accepted && acceptance.geometryEvaluations == 1 && originalCenter(mesh),"failed GPoint rejects before area queries");
    s = source(); RunControl control; control.maximumGeometryEvaluations = 13;
    acceptance = acceptSmoothedPosition(mesh,5,incident,{.2,.3},{.5,.5},s,control);
    check(acceptance.status.code == MeshingStatus::BudgetExceeded && acceptance.geometryEvaluations == 13 && originalCenter(mesh),"budget during temporary modified vertex restores entry");
    std::vector<SmoothingVertexEvent> events; control = {};
    auto result = smoothSurfaceMesh(mesh,s,{},control,[&](const SmoothingVertexEvent& e) { events.push_back(e); });
    check(result.status.complete() && result.vertexVisits == 5 && result.eligibleVertices == 1 && result.acceptedWrites == 1 && result.geometryEvaluations == 138,"one round visits classification order and preserves query sequence");
    check(events.size() == 5 && events.back().vertex == 5 && events.back().trials.size() == 5 && events.back().trials[0].published && !events.back().trials[1].published && close(mesh.vertices.at(5).point.x,.5),"success does not stop later four trials");
    if(events.size() == 5 && events.back().trials.size() == 5) {
        const auto& e = events.back(); const double factor = 1./1.4;
        check(close(e.trials[1].trial.x,.5*factor+.2*(1.-factor)) && close(e.trials[1].actual.x,.5),"trial uses fixed before/after but acceptance actual follows commit");
    }
    check(mesh.triangles == fan().triangles && mesh.maximumVertexId == 99 && mesh.interiorVertices == std::vector<std::uint64_t>{5} && mesh.vertices.at(1).point.x == 0.,"smoothing leaves topology, identity and fixed boundary unchanged");
    mesh = fan(); events.clear(); int pointQueries = 0; s = source();
    s.surfacePoint = [&](Point2D uv) { return SmoothingSurfacePoint{{uv.x,uv.y,0.},++pointQueries != 2}; };
    result = smoothSurfaceMesh(mesh,s,{}, {},[&](const SmoothingVertexEvent& e) { events.push_back(e); });
    check(result.status.complete() && events.size() == 5 && events.back().trials.size() == 5 && events.back().trials[0].acceptance.accepted && !events.back().trials[0].published && events.back().trials[1].published && pointQueries == 7,"second independent GPoint failure prevents first publication, next trial still runs");
    mesh = fan(); s = source(); s.surfacePoint = [](Point2D) { return SmoothingSurfacePoint{}; };
    result = smoothSurfaceMesh(mesh,s);
    check(result.status.complete() && result.acceptedWrites == 0 && result.geometryEvaluations == 17 && originalCenter(mesh),"all five failed points leave original position");
    mesh = fan(); SurfaceSmoothingOptions options; options.iterations = 0;
    result = smoothSurfaceMesh(mesh,{},options);
    check(result.status.complete() && result.geometryEvaluations == 0 && result.vertexVisits == 0 && originalCenter(mesh),"zero smoothing iterations needs no callbacks");
    options = {}; options.boundaryLayerVertices.insert(5); result = smoothSurfaceMesh(mesh,{},options);
    check(result.status.complete() && result.eligibleVertices == 0 && originalCenter(mesh),"boundary-layer membership fixes vertex before geometry queries");
    options = {}; options.faceVerticesWithBoundaryLayerData.insert(5); result = smoothSurfaceMesh(mesh,{},options);
    check(result.status.complete() && result.eligibleVertices == 0 && originalCenter(mesh),"MFaceVertex boundary-layer data fixes free classified vertex");
    mesh.vertices.at(5).entity = 2; result = smoothSurfaceMesh(mesh,{});
    check(result.status.complete() && result.eligibleVertices == 0 && originalCenter(mesh),"dimension two on another face is fixed");
    mesh = fan(); control = {}; control.maximumGeometryEvaluations = 38;
    result = smoothSurfaceMesh(mesh,source(),{},control);
    check(result.status.code == MeshingStatus::BudgetExceeded && result.geometryEvaluations == 38 && result.acceptedWrites == 1 && close(mesh.vertices.at(5).point.x,.5),"shared budget spans position, trial and publication; prior complete trial kept");
    mesh = fan(); control = {}; control.keepGoing = [] { return false; };
    result = smoothSurfaceMesh(mesh,source(),{},control);
    check(result.status.code == MeshingStatus::Cancelled && originalCenter(mesh),"entry cancellation explicit");
    mesh = fan(); result = smoothSurfaceMesh(mesh,source(),{}, {},[](const SmoothingVertexEvent& e) { if(e.vertex == 5) throw std::runtime_error("observer failed"); });
    check(result.status.code == MeshingStatus::NonConverged && close(mesh.vertices.at(5).point.x,.5),"observer exception retains already committed trials, not false success");
    mesh = fan(); events.clear(); s = source();
    s.reparameterizeEdge = [](const FaceVertex& a,const FaceVertex& b,const FaceMesh&) { auto p = std::array<Point2D,2>{a.parameters,b.parameters}; if(a.id == 5) p[1].x += 1.; return p; };
    result = smoothSurfaceMesh(mesh,s,{}, {},[&](const SmoothingVertexEvent& e) { events.push_back(e); });
    check(result.status.complete() && events.size() == 5 && events.back().position.neighbors.size() == 4 && events.back().position.neighbors[1].vertex == 2 && events.back().position.neighbors[1].parameters.x == 2.,"last edge-oriented neighbor UV overwrites first, kept in identity order");
    return failures ? 1 : 0;
}
