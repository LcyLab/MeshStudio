#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include <iostream>
#include <stdexcept>

#include "mesh/runtime/MeshingObserver.h"
#include <vector>
using namespace mesher;
namespace {
struct Patch : GeometrySource {
    bool throws=false;
    int vertexCount() const override { return 8; }
    Point3D vertexPoint(int i) const override {
        const Point3D p[]={{0,0,0},{4,0,0},{4,4,0},{0,4,0},{1,1,0},{1,3,0},{3,3,0},{3,1,0}};
        return p[i];
    }
    int edgeCount() const override { return 8; }
    std::pair<int,int> edgeVertices(int e) const override { return {e,e<4?(e+1)%4:4+(e-3)%4}; }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int e,double t) const override {
        const auto ends=edgeVertices(e); const auto a=vertexPoint(ends.first),b=vertexPoint(ends.second);
        return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),0};
    }
    int faceCount() const override { return 1; }
    FaceTopology faceTopology(int) const override { return {{{{0},{1},{2},{3}},{{4},{5},{6},{7}}}}; }
    Point2D edgeUV(int,const EdgeUse& u,double t) const override { const auto p=edgePoint(u.edge,t); return {p.x,p.y}; }
    Point3D facePoint(int,Point2D p) const override { return {p.x,p.y,0}; }
    Point3D faceNormal(int,Point2D) const override {
        if (throws) throw std::runtime_error("fixture CAD error");
        return {0,0,1};
    }
};
struct Observer : MeshingObserver {
    std::vector<std::pair<MeshStage,MeshEntityRef>> stack;
    int begins=0,ends=0; bool valid=true; std::uint64_t copies=0,classifications=0,transactions=0,audits=0,shapeTrials=0;
    void stageBegin(MeshStage s,MeshEntityRef e) noexcept override { stack.push_back({s,e}); ++begins; }
    void stageEnd(MeshStage s,MeshEntityRef e) noexcept override {
        if (stack.empty() || stack.back().first!=s || stack.back().second.id!=e.id || stack.back().second.dimension!=e.dimension) valid=false;
        else stack.pop_back();
        ++ends;
    }
    void addCount(MeshCounter c,MeshEntityRef,std::uint64_t n) noexcept override {
        if (c==MeshCounter::WholeMeshCopies) copies+=n;
        if (c==MeshCounter::WholeDomainClassifications) classifications+=n;
        if (c==MeshCounter::LocalMeshTransactions) transactions+=n;
        if (c==MeshCounter::FullCriterionAudits) audits+=n;
        if (c==MeshCounter::ShapeCandidateTrials) shapeTrials+=n;
    }
    bool balanced() const { return valid && begins>0 && begins==ends && stack.empty(); }
};
bool same(const SurfaceMeshResult& a,const SurfaceMeshResult& b) {
    if (a.ok!=b.ok || a.error!=b.error || a.diagnostics!=b.diagnostics || a.mesh.triangles!=b.mesh.triangles ||
        a.triangleFaceIds!=b.triangleFaceIds || a.mesh.vertices.size()!=b.mesh.vertices.size() ||
        a.triangleCornerUV.size()!=b.triangleCornerUV.size() || a.boundaryEdges.size()!=b.boundaryEdges.size() ||
        a.work.totalWork!=b.work.totalWork || a.measuredMaxDeviation!=b.measuredMaxDeviation ||
        a.measuredMaxNormalAngleDeg!=b.measuredMaxNormalAngleDeg || a.measuredMaxEdgeLength!=b.measuredMaxEdgeLength ||
        a.measuredWorstRadiusEdgeRatio!=b.measuredWorstRadiusEdgeRatio) return false;
    const auto work=[](const SurfaceWorkStatistics& w) { return std::array<int,10>{w.totalWork,w.boundarySamplingSplits,
        w.sharedEdgeSplits,w.intersectionRefinements,w.insertedSurfacePoints,w.delaunayFlips,w.faceBuilds,w.schedulingPasses,
        w.violationsAfterLegalization,w.allocatedVertices}; };
    if (work(a.work)!=work(b.work)) return false;
    for (size_t i=0;i<a.mesh.vertices.size();++i) { const auto p=a.mesh.vertices[i],q=b.mesh.vertices[i]; if (p.x!=q.x || p.y!=q.y || p.z!=q.z) return false; }
    for (size_t i=0;i<a.triangleCornerUV.size();++i) for (int k=0;k<3;++k) {
        const auto p=a.triangleCornerUV[i][k],q=b.triangleCornerUV[i][k]; if (p.x!=q.x || p.y!=q.y) return false;
    }
    for (size_t i=0;i<a.boundaryEdges.size();++i) {
        const auto p=a.boundaryEdges[i],q=b.boundaryEdges[i]; if (p.a!=q.a || p.b!=q.b || p.cadEdge!=q.cadEdge) return false;
    }
    return true;
}
}
int main() {
    Patch patch; SurfaceCriteria c; c.targetSize=0.8; c.minSize=0.08; c.maxDeviation=0.08;
    const auto plain=generateCadSurfaceMesh(patch,c);
    Observer observer; const auto measured=generateCadSurfaceMesh(patch,c,nullptr,{},&observer);
    if (!plain.ok || !same(plain,measured) || !observer.balanced() || observer.copies<2 || observer.copies>4 || !observer.transactions ||
        observer.classifications!=2*measured.work.faceBuilds || observer.audits!=measured.work.faceBuilds || !observer.shapeTrials) {
        std::cerr<<"FAIL observation must preserve the hole mesh and bound accepted-surface snapshots independently of local trials"
            <<" same="<<same(plain,measured)<<" classifications="<<observer.classifications
            <<" audits="<<observer.audits<<" faceBuilds="<<measured.work.faceBuilds<<'\n'; return 1;
    }
    Observer cancelled; const auto stopped=generateCadSurfaceMesh(patch,c,nullptr,[](int,int){return false;},&cancelled);
    if (stopped.ok || stopped.error.find("cancelled")==std::string::npos || !cancelled.balanced()) return 1;
    patch.throws=true; Observer failed; bool caught=false;
    try { generateCadSurfaceMesh(patch,c,nullptr,{},&failed); } catch (const std::runtime_error&) { caught=true; }
    if (!caught || !failed.balanced()) { std::cerr<<"FAIL CAD exception must close observation stages\n"; return 1; }
    std::cout<<"PASS observational equivalence, nested scopes, cancellation and CAD exception\n"; return 0;
}
