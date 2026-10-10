// Audit CAD queries independently of triangulation; IDs match OccCadGeometry.
#include "cad/occ/OccCadGeometry.h"
#include "cad/import/ShapeLoader.h"
#include "mesh/validation/SurfaceMeshValidation.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TColStd_Array1OfReal.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>

using mesher::Point2D;
using mesher::Point3D;
namespace {
double norm(Point3D p) { return std::hypot(p.x,p.y,p.z); }
double distance(Point3D p,Point3D q) { return norm({p.x-q.x,p.y-q.y,p.z-q.z}); }
double angle(Point3D p,Point3D q) {
    const double a=norm(p),b=norm(q);
    if (!(a>0 && b>0)) return -1;
    return std::acos(std::clamp((p.x/a)*(q.x/b)+(p.y/a)*(q.y/b)+(p.z/a)*(q.z/b),-1.0,1.0))*180/std::acos(-1.0);
}
struct Sample { int face; mesher::EdgeUse use; double t; };
// A local repro keeps the original CAD objects and p-curve branches; only IDs
// exposed to the mesher are remapped, with the source IDs printed by the tool.
class SelectedFace final : public mesher::GeometrySource {
    const OccCadGeometry& source;
    std::vector<int> faces;
    std::vector<int> vertices,edges;
    std::vector<mesher::FaceTopology> topology;
    int localVertex(int v) {
        auto it=std::find(vertices.begin(),vertices.end(),v);
        if (it!=vertices.end()) return int(it-vertices.begin());
        vertices.push_back(v); return int(vertices.size()-1);
    }
public:
    SelectedFace(const OccCadGeometry& g,std::vector<int> selected):source(g),faces(std::move(selected)) {
        for (int f:faces) topology.push_back(g.faceTopology(f));
        for (auto& face:topology) for (auto& loop:face.loops) for (auto& use:loop) {
            auto it=std::find(edges.begin(),edges.end(),use.edge);
            if (it==edges.end()) { edges.push_back(use.edge); it=edges.end()-1; }
            use.edge=int(it-edges.begin());
        }
        for (int e:edges) { const auto ends=g.edgeVertices(e); localVertex(ends.first); localVertex(ends.second); }
        for (size_t e=0;e<edges.size();++e) std::cout<<"localEdge="<<e<<" sourceEdge="<<edges[e]<<'\n';
    }
    int vertexCount() const override { return int(vertices.size()); }
    Point3D vertexPoint(int v) const override { return source.vertexPoint(vertices[v]); }
    int edgeCount() const override { return int(edges.size()); }
    std::pair<int,int> edgeVertices(int e) const override {
        const auto p=source.edgeVertices(edges[e]);
        return {int(std::find(vertices.begin(),vertices.end(),p.first)-vertices.begin()),
                int(std::find(vertices.begin(),vertices.end(),p.second)-vertices.begin())};
    }
    std::pair<double,double> edgeRange(int e) const override { return source.edgeRange(edges[e]); }
    Point3D edgePoint(int e,double t) const override { return source.edgePoint(edges[e],t); }
    Point3D edgeDerivative(int e,double t) const override { return source.edgeDerivative(edges[e],t); }
    bool edgeDegenerate(int e) const override { return source.edgeDegenerate(edges[e]); }
    int faceCount() const override { return int(faces.size()); }
    mesher::FaceTopology faceTopology(int f) const override { return topology[f]; }
    Point2D edgeUV(int f,const mesher::EdgeUse& use,double t) const override {
        auto original=use; original.edge=edges[use.edge]; return source.edgeUV(faces[f],original,t);
    }
    Point3D facePoint(int f,Point2D uv) const override { return source.facePoint(faces[f],uv); }
    Point3D faceNormal(int f,Point2D uv) const override { return source.faceNormal(faces[f],uv); }
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override { source.faceDerivatives(faces[f],uv,du,dv); }
    bool faceParameterBounds(int f,mesher::SurfaceParameterBounds& bounds) const override { return source.faceParameterBounds(faces[f],bounds); }
    bool faceReversed(int f) const override { return source.faceReversed(faces[f]); }
    bool normalUsesOrientedDerivatives() const override { return source.normalUsesOrientedDerivatives(); }
};
class TracedGeometry final : public mesher::GeometrySource {
public:
    const OccCadGeometry& source;
    mutable std::deque<Sample> recent;
    mutable std::vector<int> curveQueries;
    mutable bool enteredFaces=false;
    mutable int passes=0,lastFace=-1;
    explicit TracedGeometry(const OccCadGeometry& g):source(g),curveQueries(g.edgeCount(),0) {}
    int vertexCount() const override {
        if (passes) std::cout<<"rebuild after pass="<<passes<<" lastFace="<<lastFace<<'\n';
        ++passes; enteredFaces=false; lastFace=-1;
        return source.vertexCount();
    }
    Point3D vertexPoint(int v) const override { return source.vertexPoint(v); }
    int edgeCount() const override { return source.edgeCount(); }
    std::pair<int,int> edgeVertices(int e) const override { return source.edgeVertices(e); }
    std::pair<double,double> edgeRange(int e) const override { return source.edgeRange(e); }
    Point3D edgePoint(int e,double t) const override { ++curveQueries[e]; return source.edgePoint(e,t); }
    Point3D edgeDerivative(int e,double t) const override { ++curveQueries[e]; return source.edgeDerivative(e,t); }
    bool edgeDegenerate(int e) const override { return source.edgeDegenerate(e); }
    int faceCount() const override { return source.faceCount(); }
    mesher::FaceTopology faceTopology(int f) const override { return source.faceTopology(f); }
    Point2D edgeUV(int f,const mesher::EdgeUse& u,double t) const override {
        recent.push_back({f,u,t});
        if (recent.size()>18) recent.pop_front();
        return source.edgeUV(f,u,t);
    }
    Point3D facePoint(int f,Point2D uv) const override { return source.facePoint(f,uv); }
    Point3D faceNormal(int f,Point2D uv) const override { return source.faceNormal(f,uv); }
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override {
        enteredFaces=true; source.faceDerivatives(f,uv,du,dv);
        lastFace=f;
    }
    bool faceReversed(int f) const override { return source.faceReversed(f); }
    bool normalUsesOrientedDerivatives() const override { return source.normalUsesOrientedDerivatives(); }
    bool faceParameterBounds(int f,mesher::SurfaceParameterBounds& bounds) const override { return source.faceParameterBounds(f,bounds); }
    bool bounds(Point3D& a,Point3D& b) const override { return source.bounds(a,b); }
    bool expectsClosedSurface() const override { return source.expectsClosedSurface(); }
    std::string validationError() const override { return source.validationError(); }
};
void dump(const OccCadGeometry& g,const Sample& s) {
    const auto uv=g.edgeUV(s.face,s.use,s.t);
    const auto p=g.edgePoint(s.use.edge,s.t);
    Point3D du,dv;
    g.faceDerivatives(s.face,uv,du,dv);
    const auto n=g.faceNormal(s.face,uv);
    const double scale=norm(du)*norm(dv);
    std::cout << "sample edge="<<s.use.edge<<" face="<<s.face<<" branch="<<s.use.branch
              <<" t="<<s.t<<" uv=("<<uv.x<<","<<uv.y<<")"
              <<" residual="<<distance(p,g.facePoint(s.face,uv))
              <<" |du|="<<norm(du)<<" |dv|="<<norm(dv)
              <<" sinJacobian="<<(scale>0?norm(n)/scale:0)
              <<" normal=("<<n.x<<","<<n.y<<","<<n.z<<")\n";
}
}
int main(int argc,char** argv) {
    if (argc<2 || argc>6) return 2;
    const auto loadStarted=std::chrono::steady_clock::now();
    TopoDS_Shape shape;
    QString error;
    if (!ShapeLoader::load(QString::fromLocal8Bit(argv[1]),shape,&error)) {
        std::cerr<<error.toStdString()<<'\n'; return 1;
    }
    const auto loadFinished=std::chrono::steady_clock::now();
    OccCadGeometry geo(shape);
    const auto geometryFinished=std::chrono::steady_clock::now();
    if (!geo.validationError().empty()) { std::cerr<<geo.validationError()<<'\n'; return 1; }
    if (argc>=5 && std::string(argv[4])=="benchmark") {
        const int timeout=argc==6?std::stoi(argv[5]):60;
        if (timeout<1 || timeout>600) { std::cerr<<"benchmark timeout must be 1..600 seconds\n"; return 2; }
        mesher::SurfaceCriteria c; c.targetSize=std::stod(argv[2]); c.maxIterations=std::stoi(argv[3]);
        Point3D lo,hi;
        if (!geo.bounds(lo,hi)) { std::cerr<<"geometry bounds unavailable\n"; return 1; }
        const double diagonal=distance(lo,hi);
        if (c.targetSize==0) c.targetSize=diagonal*0.025;
        c.minSize=c.targetSize*0.1; c.maxDeviation=c.targetSize*0.1;
        std::cout<<std::setprecision(17)<<"benchmark faces="<<geo.faceCount()<<" edges="<<geo.edgeCount()
                 <<" vertices="<<geo.vertexCount()<<" closed="<<geo.expectsClosedSurface()<<" diagonal="<<diagonal<<'\n'
                 <<"criteria target="<<c.targetSize<<" min="<<c.minSize<<" deviation="<<c.maxDeviation
                 <<" angle="<<c.maxNormalAngleDeg<<" iterations="<<c.maxIterations<<" vertices="<<c.maxVertices
                 <<" timeout="<<timeout<<std::endl;
        const auto started=std::chrono::steady_clock::now(); auto reported=started; int lastFace=0;
        double firstCompletedFace=-1;
        const auto result=mesher::generateCadSurfaceMesh(geo,c,nullptr,[&](int done,int total) {
            const auto now=std::chrono::steady_clock::now();
            lastFace=done;
            if (done>0 && firstCompletedFace<0) firstCompletedFace=std::chrono::duration<double>(now-started).count();
            if (now-reported>std::chrono::seconds(10)) {
                std::cout<<"benchmark progress face="<<done<<'/'<<total<<" seconds="
                         <<std::chrono::duration<double>(now-started).count()<<std::endl; reported=now;
            }
            return now-started<std::chrono::seconds(timeout);
        });
        const auto finished=std::chrono::steady_clock::now();
        const auto seconds=[](auto a,auto b) { return std::chrono::duration<double>(b-a).count(); };
        std::cout<<"benchmark ok="<<result.ok<<" error="<<result.error<<'\n'
                 <<"timing import="<<seconds(loadStarted,loadFinished)<<" adapter="<<seconds(loadFinished,geometryFinished)
                 <<" meshing="<<seconds(started,finished)<<" total="<<seconds(loadStarted,finished)
                 <<" firstCompletedFace="<<firstCompletedFace<<" lastFace="<<lastFace<<'\n'
                 <<"output points="<<result.mesh.vertices.size()<<" triangles="<<result.mesh.triangles.size()
                 <<" maxEdge="<<result.measuredMaxEdgeLength<<" deviation="<<result.measuredMaxDeviation
                 <<" normalAngle="<<result.measuredMaxNormalAngleDeg<<'\n'
                 <<"work="<<result.work.totalWork<<" faceBuilds="<<result.work.faceBuilds<<" passes="<<result.work.schedulingPasses
                 <<" samplingSplits="<<result.work.boundarySamplingSplits<<" sharedSplits="<<result.work.sharedEdgeSplits
                 <<" insertions="<<result.work.insertedSurfacePoints<<" flips="<<result.work.delaunayFlips
                 <<" intersectionRefinements="<<result.work.intersectionRefinements
                 <<" allocatedVertices="<<result.work.allocatedVertices<<'\n';
        std::cout<<"diagnosticCount="<<result.diagnostics.size()<<'\n';
        for (size_t i=0;i<std::min<size_t>(12,result.diagnostics.size());++i)
            std::cout<<"diagnostic="<<result.diagnostics[i]<<'\n';
        return result.ok?0:1;
    }
    const int edge=argc>=3?std::stoi(argv[2]):30;
    if (edge<0 || edge>=geo.edgeCount()) return 2;
    std::cout<<std::setprecision(17)<<"faces="<<geo.faceCount()<<" edges="<<geo.edgeCount()
             <<" vertices="<<geo.vertexCount()<<" closed="<<geo.expectsClosedSurface()<<'\n';
    TopTools_IndexedMapOfShape faces,edges;
    TopExp::MapShapes(shape,TopAbs_FACE,faces);
    TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    const auto range=geo.edgeRange(edge);
    const auto rawEdge=TopoDS::Edge(edges(edge+1));
    std::cout<<"edge="<<edge<<" range=["<<range.first<<","<<range.second
             <<"] sameParameter="<<BRep_Tool::SameParameter(rawEdge)
             <<" tolerance="<<BRep_Tool::Tolerance(rawEdge)<<'\n';
    for (int f=0;f<geo.faceCount();++f) for (const auto& loop:geo.faceTopology(f).loops)
        for (const auto& use:loop) if (use.edge==edge) {
            BRepAdaptor_Surface surf(TopoDS::Face(faces(f+1)));
            std::cout<<"face="<<f<<" surfaceType="<<int(surf.GetType())
                     <<" uPeriodic="<<surf.IsUPeriodic()<<" vPeriodic="<<surf.IsVPeriodic()<<'\n';
            TColStd_Array1OfReal ui(1,surf.NbUIntervals(GeomAbs_C1)+1),vi(1,surf.NbVIntervals(GeomAbs_C1)+1);
            surf.UIntervals(ui,GeomAbs_C1); surf.VIntervals(vi,GeomAbs_C1);
            std::cout<<"U C1 breaks:"; for (int i=ui.Lower();i<=ui.Upper();++i) std::cout<<' '<<ui(i);
            std::cout<<"\nV C1 breaks:"; for (int i=vi.Lower();i<=vi.Upper();++i) std::cout<<' '<<vi(i);
            std::cout<<'\n';
            for (int k=0;k<=8;++k) dump(geo,{f,use,range.first+(range.second-range.first)*k/8});
        }
    TracedGeometry trace(geo);
    mesher::SurfaceCriteria criteria;
    criteria.targetSize=20; criteria.maxDeviation=2;
    criteria.maxIterations=argc>=4?std::stoi(argv[3]):1000;
    if (argc==6) {
        std::vector<int> selected;
        std::string list=argv[5]; size_t offset=0;
        do {
            const size_t end=list.find(',',offset);
            const int f=std::stoi(list.substr(offset,end-offset));
            if (f<0 || f>=geo.faceCount() || std::find(selected.begin(),selected.end(),f)!=selected.end()) return 2;
            selected.push_back(f);
            if (end==std::string::npos) break;
            offset=end+1;
        } while (offset<list.size());
        SelectedFace local(geo,selected);
        double u0=1e300,u1=-1e300,v0=1e300,v1=-1e300;
        for (const auto& loop:local.faceTopology(0).loops) for (const auto& use:loop) {
            const auto range=local.edgeRange(use.edge);
            for (double t:{range.first,range.second}) {
                const auto uv=local.edgeUV(0,use,t);
                u0=std::min(u0,uv.x); u1=std::max(u1,uv.x);
                v0=std::min(v0,uv.y); v1=std::max(v1,uv.y);
            }
        }
        std::cout<<"localBounds U=["<<u0<<','<<u1<<"] V=["<<v0<<','<<v1<<"]\n";
        for (int j=0;j<3;++j) {
            const double v=v0+(v1-v0)*j/2;
            Point3D du,dv;
            local.faceDerivatives(0,{(u0+u1)/2,v},du,dv);
            const auto n=local.faceNormal(0,{u0,v});
            std::cout<<"row v="<<v<<" |du|="<<norm(du)<<" |dv|="<<norm(dv)<<" anglesFromU0:";
            for (int i=1;i<=4;++i) std::cout<<' '<<angle(n,local.faceNormal(0,{u0+(u1-u0)*i/4,v}));
            std::cout<<'\n';
        }
        const auto start=std::chrono::steady_clock::now();
        const auto result=mesher::generateCadSurfaceMesh(local,criteria,nullptr,[&](int,int) {
            return std::chrono::steady_clock::now()-start<std::chrono::seconds(10);
        });
        std::cout<<"sourceFaces="<<list<<" ok="<<result.ok<<" error="<<result.error
                 <<" vertices="<<result.mesh.vertices.size()<<" triangles="<<result.mesh.triangles.size()<<'\n';
        std::cout<<"work="<<result.work.totalWork<<" faceBuilds="<<result.work.faceBuilds
                 <<" intersectionRefinements="<<result.work.intersectionRefinements
                 <<" insertions="<<result.work.insertedSurfacePoints<<" flips="<<result.work.delaunayFlips
                 <<" violationsAfterLegalization="<<result.work.violationsAfterLegalization<<'\n';
        for (const auto& sample:result.refinementSamples) {
            const auto& s=sample.facet;
            std::cout<<"localSnapshot face="<<sample.face<<" sharedArea="<<s.sharedSignedArea
                     <<" surfaceArea="<<s.surfaceSignedArea<<'\n';
            for (int k=0;k<3;++k) std::cout<<"localCorner uv="<<s.uv[k].x<<','<<s.uv[k].y
                <<" shared="<<s.sharedPoints[k].x<<','<<s.sharedPoints[k].y<<','<<s.sharedPoints[k].z<<'\n';
        }
        for (const auto& d:result.diagnostics) std::cout<<"diagnostic="<<d<<'\n';
        std::vector<double> columns;
        for (const auto& tri:result.triangleCornerUV) for (auto uv:tri) columns.push_back(uv.x);
        std::sort(columns.begin(),columns.end());
        columns.erase(std::unique(columns.begin(),columns.end()),columns.end());
        if (!columns.empty()) {
            std::cout<<"output U coordinates (first 28 of "<<columns.size()<<"):";
            for (size_t i=0;i<std::min<size_t>(28,columns.size());++i) std::cout<<' '<<columns[i];
            std::cout<<'\n';
        }
        if (!result.ok && result.error.find("intersect")!=std::string::npos) {
            mesher::SurfaceIntersection intersection;
            std::string message;
            mesher::validateSurfaceIntersections(result.mesh,&message,0,&intersection);
            for (int t:{intersection.firstTriangle,intersection.secondTriangle}) if (t>=0)
                for (int k=0;k<3;++k) {
                    const int id=result.mesh.triangles[t][k];
                    const auto p=result.mesh.vertices[id];
                    const auto uv=result.triangleCornerUV[t][k];
                    std::cout<<"facet="<<t<<" node="<<id<<" UV=("<<uv.x<<','<<uv.y
                             <<") XYZ=("<<p.x<<','<<p.y<<','<<p.z<<")\n";
                }
        }
        // verify 是成功回归模式；普通 audit 的退出码只表示报告工具运行成功。
        if (std::string(argv[4])=="verify") return result.ok?0:1;
        return 0;
    }
    const bool full=argc==5 && (std::string(argv[4])=="full" || std::string(argv[4])=="full-long");
    const int timeout=full && std::string(argv[4])=="full-long"?120:30;
    const auto start=std::chrono::steady_clock::now();
    const auto result=mesher::generateCadSurfaceMesh(trace,criteria,nullptr,[&](int,int) {
        return (full || !trace.enteredFaces) && std::chrono::steady_clock::now()-start<std::chrono::seconds(timeout);
    });
    std::cout<<"mesher ok="<<result.ok<<" error="<<result.error<<'\n';
    std::cout<<"work="<<result.work.totalWork<<" samplingSplits="<<result.work.boundarySamplingSplits
             <<" intersectionRefinements="<<result.work.intersectionRefinements
             <<" sharedSplits="<<result.work.sharedEdgeSplits<<" faceBuilds="<<result.work.faceBuilds
             <<" insertions="<<result.work.insertedSurfacePoints<<" flips="<<result.work.delaunayFlips
             <<" violationsAfterLegalization="<<result.work.violationsAfterLegalization
             <<" allocatedVertices="<<result.work.allocatedVertices<<'\n';
    std::cout<<"elapsedSeconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()
             <<" vertices="<<result.mesh.vertices.size()<<" triangles="<<result.mesh.triangles.size()
             <<" maxEdge="<<result.measuredMaxEdgeLength<<" maxDeviation="<<result.measuredMaxDeviation
             <<" maxAngle="<<result.measuredMaxNormalAngleDeg<<'\n';
    std::cout<<"passes="<<trace.passes<<" lastFace="<<trace.lastFace<<'\n';
    if (!result.ok && result.error.find("intersect")!=std::string::npos) {
        mesher::SurfaceIntersection intersection; std::string message;
        mesher::validateSurfaceIntersections(result.mesh,&message,0,&intersection);
        for (int t:{intersection.firstTriangle,intersection.secondTriangle}) if (t>=0) {
            const int f=result.triangleFaceIds[t];
            BRepAdaptor_Surface surf(TopoDS::Face(faces(f+1)));
            std::cout<<"intersectionFacet="<<t<<" face="<<f<<" surfaceType="<<int(surf.GetType())<<'\n';
            if (surf.GetType()==GeomAbs_Sphere) {
                const auto sphere=surf.Sphere(); const auto center=sphere.Location();
                std::cout<<"intersectionSphere face="<<f<<" radius="<<sphere.Radius()
                         <<" center="<<center.X()<<','<<center.Y()<<','<<center.Z()<<'\n';
            }
            for (int k=0;k<3;++k) {
                const int id=result.mesh.triangles[t][k];
                const auto p=result.mesh.vertices[id],q=geo.facePoint(f,result.triangleCornerUV[t][k]);
                const auto uv=result.triangleCornerUV[t][k];
                std::cout<<"intersectionCorner="<<t<<" node="<<id<<" UV="<<uv.x<<','<<uv.y
                         <<" XYZ="<<p.x<<','<<p.y<<','<<p.z<<" residual="<<distance(p,q)<<'\n';
            }
        }
    }
    for (size_t i=0;i<result.refinementSamples.size();++i) {
        const auto& sample=result.refinementSamples[i];
        std::cout<<"snapshot="<<i<<" face="<<sample.face<<" edge="<<sample.cadEdge
                 <<" interval=["<<sample.parameterStart<<','<<sample.parameterEnd<<"]\n";
        const auto dumpFacet=[&](const mesher::SurfaceFacetSnapshot& s,int neighbor) {
            std::cout<<"snapshotFacet="<<i<<" neighbor="<<neighbor<<" sharedArea="<<s.sharedSignedArea
                     <<" surfaceArea="<<s.surfaceSignedArea<<'\n';
            for (int k=0;k<3;++k) {
                const auto uv=s.uv[k]; const auto p=s.sharedPoints[k],q=s.surfacePoints[k];
                std::cout<<"snapshotCorner="<<i<<" neighbor="<<neighbor<<" k="<<k
                         <<" uv="<<uv.x<<','<<uv.y<<" shared="<<p.x<<','<<p.y<<','<<p.z
                         <<" surface="<<q.x<<','<<q.y<<','<<q.z<<" residual="<<distance(p,q)<<'\n';
            }
        };
        dumpFacet(sample.facet,-1);
        for (size_t j=0;j<sample.neighbors.size();++j) dumpFacet(sample.neighbors[j],int(j));
    }
    for (const auto& d:result.diagnostics) std::cout<<"diagnostic="<<d<<'\n';
    for (int e=0;e<geo.edgeCount();++e) if (trace.curveQueries[e])
        std::cout<<"curveQueries edge="<<e<<" count="<<trace.curveQueries[e]<<'\n';
    for (const auto& s:trace.recent) dump(geo,s);
    if (trace.recent.size()>=3) {
        const auto a=trace.recent[trace.recent.size()-3];
        const auto b=trace.recent[trace.recent.size()-2];
        const auto m=trace.recent.back();
        if (a.face==b.face && b.face==m.face && a.use.edge==b.use.edge && b.use.edge==m.use.edge) {
            const auto na=geo.faceNormal(a.face,geo.edgeUV(a.face,a.use,a.t));
            const auto nb=geo.faceNormal(b.face,geo.edgeUV(b.face,b.use,b.t));
            std::cout<<"last interval=["<<a.t<<","<<b.t<<"] endpointAngle="<<angle(na,nb)<<'\n';
            for (int k=1;k<=24;++k) {
                const double t=a.t+(b.t-a.t)*std::ldexp(1.0,-k);
                const auto n=geo.faceNormal(a.face,geo.edgeUV(a.face,a.use,t));
                std::cout<<"approach k="<<k<<" t="<<t<<" angleToStart="<<angle(na,n)<<'\n';
                if (k==24) dump(geo,{a.face,a.use,t});
            }
        }
    }
    return 0;
}
