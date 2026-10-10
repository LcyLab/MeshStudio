// 独立采样审计：CAD 网格点最近距离与 OCC 裁剪面最近距离，不复用生成器误差函数。
#include "cad/occ/OccCadGeometry.h"
#include "cad/import/ShapeLoader.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "CadBenchmarkOptions.h"
#include "CadSurfaceQualityReport.h"
#include "CadExternalMesh.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepTools.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <Standard_Failure.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>

using mesher::Point3D;
namespace {
Point3D sub(Point3D a,Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
double dot(Point3D a,Point3D b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
double sq(Point3D a) { return dot(a,a); }
Point3D point(gp_Pnt p) { return {p.X(),p.Y(),p.Z()}; }
Point3D blend(Point3D a,Point3D b,double t) { return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z)}; }
double segmentDistance(Point3D p,Point3D a,Point3D b) {
    const auto d=sub(b,a); const double length=sq(d);
    return sq(sub(p,blend(a,b,length>0?std::clamp(dot(sub(p,a),d)/length,0.0,1.0):0.0)));
}
double triangleDistance(Point3D p,Point3D a,Point3D b,Point3D c) {
    double distance=std::min({segmentDistance(p,a,b),segmentDistance(p,b,c),segmentDistance(p,c,a)});
    const auto u=sub(b,a),v=sub(c,a),w=sub(p,a);
    const double uu=sq(u),vv=sq(v),uv=dot(u,v),uw=dot(u,w),vw=dot(v,w),det=uu*vv-uv*uv;
    if (det>0) {
        const double x=(uw*vv-vw*uv)/det,y=(vw*uu-uw*uv)/det;
        if (x>=0 && y>=0 && x+y<=1) {
            const Point3D q{a.x+x*u.x+y*v.x,a.y+x*u.y+y*v.y,a.z+x*u.z+y*v.z};
            distance=std::min(distance,sq(sub(p,q)));
        }
    }
    return distance;
}
double coordinate(Point3D p,int axis) { return axis==0?p.x:axis==1?p.y:p.z; }
struct Box {
    Point3D lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};
    void add(Point3D p) {
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    double distance(Point3D p) const {
        double d=0;
        for (int k=0;k<3;++k) {
            const double v=coordinate(p,k),a=coordinate(lo,k),b=coordinate(hi,k);
            const double e=std::max({a-v,0.0,v-b}); d+=e*e;
        }
        return d;
    }
};
class FaceMeshQuery {
    const mesher::SurfaceMesh3D& mesh;
    struct Node { Box box; int begin=0,end=0,left=-1,right=-1; };
    std::vector<int> order;
    std::vector<Node> nodes;
    Box triangleBox(int t) const {
        Box box; for (int v:mesh.triangles[t]) box.add(mesh.vertices[v]); return box;
    }
    int build(int begin,int end) {
        Node node; node.begin=begin; node.end=end;
        for (int i=begin;i<end;++i) { const auto b=triangleBox(order[i]); node.box.add(b.lo); node.box.add(b.hi); }
        const int id=int(nodes.size()); nodes.push_back(node);
        if (end-begin>8) {
            const auto span=sub(node.box.hi,node.box.lo);
            const int axis=span.x>=span.y && span.x>=span.z?0:span.y>=span.z?1:2;
            const int mid=begin+(end-begin)/2;
            std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](int a,int b) {
                const auto ba=triangleBox(a),bb=triangleBox(b);
                return coordinate(ba.lo,axis)+coordinate(ba.hi,axis)<coordinate(bb.lo,axis)+coordinate(bb.hi,axis);
            });
            node.left=build(begin,mid); node.right=build(mid,end); nodes[id]=node;
        }
        return id;
    }
    void visit(int id,Point3D p,double& best) const {
        const auto& node=nodes[id]; if (node.box.distance(p)>best) return;
        if (node.left<0) {
            for (int i=node.begin;i<node.end;++i) {
                const auto& t=mesh.triangles[order[i]];
                best=std::min(best,triangleDistance(p,mesh.vertices[t[0]],mesh.vertices[t[1]],mesh.vertices[t[2]]));
            }
        } else {
            int a=node.left,b=node.right;
            if (nodes[b].box.distance(p)<nodes[a].box.distance(p)) std::swap(a,b);
            visit(a,p,best); visit(b,p,best);
        }
    }
public:
    FaceMeshQuery(const mesher::SurfaceMesh3D& surface,std::vector<int> triangles):mesh(surface),order(std::move(triangles)) {
        if (!order.empty()) build(0,int(order.size()));
    }
    double distance(Point3D p) const {
        double best=std::numeric_limits<double>::infinity(); if (!nodes.empty()) visit(0,p,best); return std::sqrt(best);
    }
};
}

int main(int argc,char** argv) {
    try {
        auto args=cadbench::arguments(argc,argv); std::string external;
        for (size_t i=0;i<args.size();++i) if (args[i]=="--external") {
            if (!external.empty() || i+1>=args.size()) throw std::invalid_argument("duplicate/missing --external");
            external=args[i+1]; args.erase(args.begin()+i,args.begin()+i+2); --i;
        }
        auto options=cadbench::parse(args,true);
        // 最近距离算子的解析检查，防止审计工具把距离始终算成零。
        if (std::abs(triangleDistance({0.2,0.2,2},{0,0,0},{1,0,0},{0,1,0})-4)>1e-12 ||
            std::abs(triangleDistance({2,0,0},{0,0,0},{1,0,0},{0,1,0})-1)>1e-12) {
            std::cerr<<"independent triangle distance analytic checks failed\n"; return 1;
        }
        TopoDS_Shape shape; QString error;
        if (!ShapeLoader::load(QString::fromLocal8Bit(options.cad.c_str()),shape,&error)) { std::cerr<<error.toStdString()<<'\n'; return 1; }
        OccCadGeometry geo(shape,true);
        for(const auto& diagnostic:geo.inputDiagnostics())std::cout<<diagnostic<<'\n';
        cadbench::resolve(options,geo); const auto c=options.criteria;
        std::cout<<std::setprecision(17)<<"criteria target="<<c.targetSize<<" min="<<c.minSize<<" max="<<c.maxSize<<" deviation="<<c.maxDeviation
            <<" angle="<<c.maxNormalAngleDeg<<" work="<<c.maxIterations<<" vertices="<<c.maxVertices
            <<" timeoutPerPhase="<<options.timeout<<std::endl;
        const auto started=std::chrono::steady_clock::now();
        TopTools_IndexedMapOfShape faces; TopExp::MapShapes(shape,TopAbs_FACE,faces);
        mesher::Point3D lo,hi; geo.bounds(lo,hi);
        const auto result=external.empty()?mesher::generateCadSurfaceMesh(geo,c,nullptr,[&](int,int) {
            return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<options.timeout;
        },nullptr,mesher::AssemblySurfacePolicy::IndependentBodies):readExternalSurface(external,faces,std::hypot(hi.x-lo.x,hi.y-lo.y,hi.z-lo.z));
        if (!result.ok) { std::cerr<<result.error<<'\n'; return 1; }
        const auto quality=cadquality::analyze(result,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"independent quality bad="<<quality.bad<<" minAngle="<<quality.minAngle
            <<" worst="<<quality.worst<<" p95="<<quality.p95<<" p99="<<quality.p99<<'\n';
        if(quality.invalid) {std::cerr<<"independent quality detected degenerate facets\n";return 1;}
        if (result.triangleFaceIds.size()!=result.mesh.triangles.size() ||
            result.triangleCornerUV.size()!=result.mesh.triangles.size()) {
            std::cerr<<"surface triangle metadata count mismatch\n"; return 1;
        }
        std::vector<std::vector<int>> faceTriangles(faces.Extent());
        const bool partitioned=!result.trianglePartitionIds.empty();
        if(partitioned && (result.trianglePartitionIds.size()!=result.mesh.triangles.size()||
           result.trianglePartitionUV.size()!=result.mesh.triangles.size()))throw std::runtime_error("partition metadata count mismatch");
        for (size_t t=0;t<result.mesh.triangles.size();++t) {
            if(partitioned) {
                const int patch=result.trianglePartitionIds[t];
                if(patch<0||patch>=int(result.partitionSourceFaces.size()))throw std::runtime_error("invalid patch label");
                // Candidate query set for the original face's common surface
                // patch. This is not a claim that every triangle clips that face.
                for(int f:result.partitionSourceFaces[patch]) {
                    if(f<0||f>=faces.Extent())throw std::runtime_error("invalid patch source face");
                    faceTriangles[f].push_back(int(t));
                }
                continue;
            }
            const int f=result.triangleFaceIds[t];
            if (f<0 || f>=faces.Extent()) { std::cerr<<"invalid CAD face label on triangle "<<t<<'\n'; return 1; }
            faceTriangles[f].push_back(int(t));
        }
        const auto auditStarted=std::chrono::steady_clock::now();
        const auto checkTime=[&] {
            if (std::chrono::duration<double>(std::chrono::steady_clock::now()-auditStarted).count()>=options.timeout)
                throw std::runtime_error("independent audit timed out; coverage is incomplete");
        };
        double cadToMesh=0,meshToCad=0; int cadSamples=0,meshSamples=0,interiorSamples=0,supplementedFaces=0;
        for (int f=0;f<faces.Extent();++f) {
            checkTime();
            const auto face=TopoDS::Face(faces(f+1)); BRepAdaptor_Surface surface(face);
            FaceMeshQuery query(result.mesh,faceTriangles[f]);
            double u0,u1,v0,v1; BRepTools::UVBounds(face,u0,u1,v0,v1);
            if (!std::isfinite(u0)||!std::isfinite(u1)||!std::isfinite(v0)||!std::isfinite(v1) || !(u1>u0 && v1>v0)) {
                std::cerr<<"invalid CAD UV bounds on face "<<f<<'\n'; return 1;
            }
            int faceCadSamples=0,faceMeshSamples=0,faceInteriorSamples=0; double faceForward=0,faceBackward=0;
            // 固定 UV 格点与曲线分点，不依赖生成器的边中点/重心布局；只查所属面网格。
            for (int i=0;i<16;++i) for (int j=0;j<16;++j) {
                const double u=u0+(u1-u0)*(i+0.37)/16,v=v0+(v1-v0)*(j+0.61)/16;
                const BRepClass_FaceClassifier classifier(face,gp_Pnt2d(u,v),1e-9);
                if (classifier.State()!=TopAbs_IN && classifier.State()!=TopAbs_ON) continue;
                faceForward=std::max(faceForward,query.distance(point(surface.Value(u,v)))); ++faceCadSamples;
                ++faceInteriorSamples;
            }
            for (const auto& loop:geo.faceTopology(f).loops) for (const auto& use:loop) {
                if (geo.edgeDegenerate(use.edge)) continue;
                const auto range=geo.edgeRange(use.edge);
                for (int k=0;k<33;++k) {
                    faceForward=std::max(faceForward,query.distance(geo.edgePoint(use.edge,range.first+(range.second-range.first)*(k+0.29)/33)));
                    ++faceCadSamples;
                }
            }
            // 每面分层选至多 32 个面片，三个内部重心权值轮换；OCC 查询裁剪面最近距离。
            const auto& triangles=faceTriangles[f]; const size_t count=std::min<size_t>(32,triangles.size());
            if (!faceInteriorSamples) {
                if (!external.empty()) throw std::runtime_error("external narrow face has no interior coverage; OCC UV export is required");
                ++supplementedFaces;
                // 极窄裁剪面补点：只用输出 UV 定位，精确 CAD 分类/距离仍独立查询。
                // 与生成器中点/重心取样不同；不是整面覆盖或误差上界证明。
                for (size_t i=0;i<count;++i) {
                    const int t=triangles[i*triangles.size()/count];
                    const auto& uv=partitioned?result.trianglePartitionUV[t]:result.triangleCornerUV[t];
                    double u=0.17*uv[0].x+0.29*uv[1].x+0.54*uv[2].x;
                    double v=0.17*uv[0].y+0.29*uv[1].y+0.54*uv[2].y;
                    if(partitioned) {
                        const int reference=result.partitionSourceFaces[result.trianglePartitionIds[t]].front();
                        const auto p=geo.facePoint(reference,{u,v});mesher::Point2D projected;
                        if(reference==f)projected={u,v};
                        else if(!geo.faceCoordinates(f,p,{(u0+u1)*.5,(v0+v1)*.5},projected))continue;
                        u=projected.x;v=projected.y;
                    }
                    const BRepClass_FaceClassifier classifier(face,gp_Pnt2d(u,v),1e-9);
                    if (classifier.State()!=TopAbs_IN && classifier.State()!=TopAbs_ON) continue;
                    faceForward=std::max(faceForward,query.distance(point(surface.Value(u,v))));
                    ++faceCadSamples; ++faceInteriorSamples;
                }
                if (!faceInteriorSamples) {
                    std::cerr<<"no CAD interior sample coverage on narrow face "<<f<<'\n'; return 1;
                }
            }
            BRepExtrema_DistShapeShape extrema; extrema.LoadS2(face);
            for (size_t i=0;i<count;++i) {
                checkTime();
                const auto& tri=result.mesh.triangles[triangles[i*triangles.size()/count]];
                for (int k=0;k<3;++k) {
                    Point3D p{};
                    for (int j=0;j<3;++j) {
                        const double weight=j==k?0.6:0.2; const auto q=result.mesh.vertices[tri[j]];
                        p.x+=weight*q.x; p.y+=weight*q.y; p.z+=weight*q.z;
                    }
                    const auto vertex=BRepBuilderAPI_MakeVertex(gp_Pnt(p.x,p.y,p.z)).Vertex();
                    double sampled=std::numeric_limits<double>::infinity();
                    const int t=triangles[i*triangles.size()/count];
                    const std::vector<int> originalFaces=partitioned?result.partitionSourceFaces[result.trianglePartitionIds[t]]:std::vector<int>{f};
                    // Reverse distance is to the actual trimmed-face union;
                    // crossing facets must not be measured against one half.
                    for(int original:originalFaces) {
                        checkTime();extrema.LoadS2(faces(original+1));extrema.LoadS1(vertex);
                        if(!extrema.Perform()||!extrema.IsDone()||!std::isfinite(extrema.Value()))
                            throw std::runtime_error("OCC original trimmed-face union distance query failed");
                        sampled=std::min(sampled,extrema.Value());
                    }
                    faceBackward=std::max(faceBackward,sampled); ++faceMeshSamples;
                }
            }
            std::cout<<"face="<<f<<" cadSamples="<<faceCadSamples<<" meshSamples="<<faceMeshSamples
                     <<" interiorSamples="<<faceInteriorSamples<<" cadToMesh="<<faceForward<<" meshToCad="<<faceBackward<<'\n';
            if (faceCadSamples==0 || faceMeshSamples==0 || faceForward>c.maxDeviation*(1+1e-8) ||
                faceBackward>c.maxDeviation*(1+1e-8)) {
                std::cerr<<"independent sampled coverage/distance criterion failed on face "<<f<<'\n'; return 1;
            }
            cadToMesh=std::max(cadToMesh,faceForward); meshToCad=std::max(meshToCad,faceBackward);
            cadSamples+=faceCadSamples; meshSamples+=faceMeshSamples;
            interiorSamples+=faceInteriorSamples;
        }
        std::cout<<"PASS independent sampled distances cadToMesh="<<cadToMesh<<" meshToCad="<<meshToCad
                 <<" cadSamples="<<cadSamples<<" meshSamples="<<meshSamples<<" interiorSamples="<<interiorSamples
                 <<" supplementedFaces="<<supplementedFaces<<" faces="<<faces.Extent()
                 <<(external.empty()?" generationSeconds=":" externalMappingAndValidationSeconds=")<<std::chrono::duration<double>(auditStarted-started).count()
                 <<" auditSeconds="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-auditStarted).count()<<'\n';
        return 0;
    } catch (const Standard_Failure& failure) { std::cerr<<failure.GetMessageString()<<'\n'; return 1; }
      catch (const std::invalid_argument& failure) { std::cerr<<failure.what()<<'\n'; return 2; }
      catch (const std::exception& failure) { std::cerr<<failure.what()<<'\n'; return 1; }
}
