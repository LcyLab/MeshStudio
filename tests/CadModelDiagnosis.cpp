// Manual real-model probe: input validity and physical triangle quality.
#include "cad/import/ShapeLoader.h"
#include "cad/occ/OccCadGeometry.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "CadBenchmarkOptions.h"
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Result.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRep_Tool.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepCheck_ListIteratorOfListOfStatus.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <Standard_Failure.hxx>
#include <QCoreApplication>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <map>

namespace {
const char* statusName(BRepCheck_Status s) {
    static const char* names[]={"NoError","InvalidPointOnCurve","InvalidPointOnCurveOnSurface","InvalidPointOnSurface",
        "No3DCurve","Multiple3DCurve","Invalid3DCurve","NoCurveOnSurface","InvalidCurveOnSurface",
        "InvalidCurveOnClosedSurface","InvalidSameRangeFlag","InvalidSameParameterFlag","InvalidDegeneratedFlag",
        "FreeEdge","InvalidMultiConnexity","InvalidRange","EmptyWire","RedundantEdge","SelfIntersectingWire",
        "NoSurface","InvalidWire","RedundantWire","IntersectingWires","InvalidImbricationOfWires","EmptyShell",
        "RedundantFace","InvalidImbricationOfShells","UnorientableShape","NotClosed","NotConnected","SubshapeNotInShape",
        "BadOrientation","BadOrientationOfSubshape","InvalidPolygonOnTriangulation","InvalidToleranceValue","EnclosedRegion","CheckFail"};
    return int(s)>=0 && int(s)<37?names[int(s)]:"Unknown";
}
void validity(const TopoDS_Shape& shape) {
    BRepCheck_Analyzer check(shape); TopTools_IndexedMapOfShape all,faces,edges;
    TopExp::MapShapes(shape,all); TopExp::MapShapes(shape,TopAbs_FACE,faces); TopExp::MapShapes(shape,TopAbs_EDGE,edges);
    std::cout<<"valid="<<check.IsValid()<<" subshapes="<<all.Extent()<<'\n';
    for (int i=1;i<=all.Extent();++i) {
        const auto r=check.Result(all(i)); if (r.IsNull()) continue;
        auto print=[&](const BRepCheck_ListOfStatus& list,int context) {
            for (BRepCheck_ListIteratorOfListOfStatus it(list);it.More();it.Next()) if (it.Value()!=BRepCheck_NoError)
                std::cout<<"shape="<<i<<" type="<<int(all(i).ShapeType())<<" context="<<context<<" status="<<statusName(it.Value())<<'\n';
        };
        print(r->Status(),0);
        for (r->InitContextIterator();r->MoreShapeInContext();r->NextShapeInContext())
            print(r->StatusOnShape(),all.FindIndex(r->ContextualShape()));
    }
    for(int f=1;f<=faces.Extent();++f) if(!check.IsValid(faces(f))) {
        auto face=TopoDS::Face(faces(f)); BRepAdaptor_Surface surface(face);
        double u0,u1,v0,v1; BRepTools::UVBounds(face,u0,u1,v0,v1);
        std::cout<<std::setprecision(17)<<"invalidFace="<<f-1<<" surfaceType="<<int(surface.GetType())
            <<" tolerance="<<BRep_Tool::Tolerance(face)<<" uvBounds="<<u0<<','<<u1<<','<<v0<<','<<v1<<'\n';
        for(TopExp_Explorer it(face,TopAbs_WIRE);it.More();it.Next()) {
            BRepCheck_Wire w(TopoDS::Wire(it.Current())); TopoDS_Edge a,b;
            if(w.SelfIntersect(face,a,b)==BRepCheck_NoError) continue;
            std::cout<<"intersectingEdges="<<edges.FindIndex(a)-1<<','<<edges.FindIndex(b)-1<<'\n';
            for(auto edge:{a,b}) if(!edge.IsNull()) {
                double first,last; auto curve=BRep_Tool::CurveOnSurface(edge,face,first,last);
                std::cout<<"edge="<<edges.FindIndex(edge)-1<<" tolerance="<<BRep_Tool::Tolerance(edge)<<" range="<<first<<','<<last<<'\n';
                TopoDS_Vertex v0,v1; TopExp::Vertices(edge,v0,v1);
                for(auto v:{v0,v1}) if(!v.IsNull()) {
                    auto p=BRep_Tool::Pnt(v);
                    std::cout<<"vertex="<<all.FindIndex(v)<<" tolerance="<<BRep_Tool::Tolerance(v)<<" xyz="<<p.X()<<','<<p.Y()<<','<<p.Z()<<'\n';
                }
                if(!curve.IsNull()) for(int k=0;k<=4;++k) {
                    auto p=curve->Value(first+(last-first)*k/4); auto q=surface.Value(p.X(),p.Y());
                    std::cout<<"sample="<<k<<" uv="<<p.X()<<','<<p.Y()<<" xyz="<<q.X()<<','<<q.Y()<<','<<q.Z()<<'\n';
                }
            }
            if(!a.IsNull() && !b.IsNull()) {
                double a0,a1,b0,b1;
                auto ca=BRep_Tool::CurveOnSurface(a,face,a0,a1),cb=BRep_Tool::CurveOnSurface(b,face,b0,b1);
                Geom2dAPI_InterCurveCurve intersections(new Geom2d_TrimmedCurve(ca,a0,a1),new Geom2d_TrimmedCurve(cb,b0,b1),1e-10);
                std::cout<<"curveIntersections="<<intersections.NbPoints()<<" overlaps="<<intersections.NbSegments()<<'\n';
                for(int k=1;k<=intersections.NbPoints();++k) {
                    auto p=intersections.Point(k); std::cout<<"crossingUV="<<p.X()<<','<<p.Y()<<'\n';
                }
                for(int k=1;k<=intersections.NbSegments();++k) {
                    Handle(Geom2d_Curve) x,y; intersections.Segment(k,x,y);
                    for(auto curve:{x,y}) {
                        auto p=curve->Value(curve->FirstParameter()),q=curve->Value(curve->LastParameter());
                        std::cout<<"overlap="<<p.X()<<','<<p.Y()<<" to "<<q.X()<<','<<q.Y()<<" span="<<p.Distance(q)<<'\n';
                    }
                }
            }
        }
    }
}
mesher::Point3D sub(mesher::Point3D a,mesher::Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
double norm(mesher::Point3D a) { return std::hypot(a.x,a.y,a.z); }
double quality(const mesher::SurfaceMesh3D& m,std::array<int,3> t) {
    auto ab=sub(m.vertices[t[1]],m.vertices[t[0]]),ac=sub(m.vertices[t[2]],m.vertices[t[0]]);
    double a=norm(ab),b=norm(sub(m.vertices[t[2]],m.vertices[t[1]])),c=norm(ac);
    double area=norm({ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x});
    return a*b*c/(2*area*std::min({a,b,c}));
}
void flipCandidates(const mesher::SurfaceMeshResult& m,const mesher::SurfaceCriteria& criteria) {
    std::map<std::pair<int,int>,std::vector<int>> adjacency; std::set<std::pair<int,int>> constrained;
    for(auto e:m.boundaryEdges) constrained.insert(std::minmax(e.a,e.b));
    for(size_t t=0;t<m.mesh.triangles.size();++t) for(int k=0;k<3;++k)
        adjacency[std::minmax(m.mesh.triangles[t][k],m.mesh.triangles[t][(k+1)%3])].push_back(int(t));
    int candidates=0; std::set<int> badTriangles;
    for(const auto& [edge,triangles]:adjacency) {
        if(triangles.size()!=2 || constrained.count(edge)) continue;
        int a=triangles[0],b=triangles[1]; if(m.triangleFaceIds[a]!=m.triangleFaceIds[b]) continue;
        std::map<int,mesher::Point2D> uv; bool coherent=true; int c=-1,d=-1;
        for(int t:{a,b}) for(int k=0;k<3;++k) {
            int v=m.mesh.triangles[t][k]; auto p=m.triangleCornerUV[t][k]; auto it=uv.find(v);
            if(it!=uv.end() && (std::abs(it->second.x-p.x)>1e-12 || std::abs(it->second.y-p.y)>1e-12)) coherent=false;
            uv[v]=p;
            if(v!=edge.first && v!=edge.second) (t==a?c:d)=v;
        }
        if(!coherent || c<0 || d<0 || c==d || adjacency.count(std::minmax(c,d))) continue;
        auto orient=[&](int i,int j,int k) { auto p=uv[i],q=uv[j],r=uv[k]; return (q.x-p.x)*(r.y-p.y)-(q.y-p.y)*(r.x-p.x); };
        if(orient(edge.first,edge.second,c)*orient(edge.first,edge.second,d)>=0 ||
           orient(c,d,edge.first)*orient(c,d,edge.second)>=0) continue;
        if(norm(sub(m.mesh.vertices[c],m.mesh.vertices[d]))>criteria.maxSize*(1+1e-8)) continue;
        double before=std::max(quality(m.mesh,m.mesh.triangles[a]),quality(m.mesh,m.mesh.triangles[b]));
        double after=std::max(quality(m.mesh,{c,d,edge.first}),quality(m.mesh,{d,c,edge.second}));
        if(before>criteria.maxRadiusEdgeRatio && after<0.8*before) {
            ++candidates;
            for(int t:{a,b}) if(quality(m.mesh,m.mesh.triangles[t])>criteria.maxRadiusEdgeRatio) badTriangles.insert(t);
        }
    }
    // Candidate evidence only. This probe does not apply edits or validate a replacement mesh.
    std::cout<<"potentialPhysicalFlipPairs="<<candidates<<" affectedBadTriangles="<<badTriangles.size()<<'\n';
}
TopoDS_Shape localToleranceProbe(const TopoDS_Shape& original) {
    BRepBuilderAPI_Copy copy(original,true,false);TopoDS_Shape shape=copy.Shape();
    BRep_Builder builder;
    for(int pass=0;pass<8;++pass) {
        BRepCheck_Analyzer check(shape); if(check.IsValid())return shape;
        bool changed=false;
        for(TopExp_Explorer faces(shape,TopAbs_FACE);faces.More();faces.Next()) {
            auto face=TopoDS::Face(faces.Current()); if(check.IsValid(face))continue;
            BRepAdaptor_Surface surface(face);
            for(TopExp_Explorer wires(face,TopAbs_WIRE);wires.More();wires.Next()) {
                BRepCheck_Wire wire(TopoDS::Wire(wires.Current()));TopoDS_Edge a,b;
                if(wire.SelfIntersect(face,a,b)!=BRepCheck_SelfIntersectingWire || a.IsNull() || b.IsNull())continue;
                TopoDS_Vertex shared;
                for(TopExp_Explorer va(a,TopAbs_VERTEX);va.More();va.Next())for(TopExp_Explorer vb(b,TopAbs_VERTEX);vb.More();vb.Next())
                    if(va.Current().IsSame(vb.Current()))shared=TopoDS::Vertex(va.Current());
                if(shared.IsNull())continue;
                const double old=BRep_Tool::Tolerance(shared),limit=100*old;double needed=0;
                double a0,a1,b0,b1;auto ca=BRep_Tool::CurveOnSurface(a,face,a0,a1),cb=BRep_Tool::CurveOnSurface(b,face,b0,b1);
                Geom2dAPI_InterCurveCurve intersections(new Geom2d_TrimmedCurve(ca,a0,a1),new Geom2d_TrimmedCurve(cb,b0,b1),1e-10);
                auto vertex=BRep_Tool::Pnt(shared);
                auto sample=[&](gp_Pnt2d p){needed=std::max(needed,vertex.Distance(surface.Value(p.X(),p.Y())));};
                for(int k=1;k<=intersections.NbPoints();++k)sample(intersections.Point(k));
                for(int k=1;k<=intersections.NbSegments();++k) {
                    Handle(Geom2d_Curve) x,y;intersections.Segment(k,x,y);
                    for(auto c:{x,y})for(int j=0;j<=8;++j)sample(c->Value(c->FirstParameter()+(c->LastParameter()-c->FirstParameter())*j/8));
                }
                if(needed>old && needed*1.1<=limit) {
                    builder.UpdateVertex(shared,needed*1.1);changed=true;
                    std::cout<<"PROBE ONLY tolerance reconciliation old="<<old<<" new="<<needed*1.1<<" vertex="<<vertex.X()<<','<<vertex.Y()<<','<<vertex.Z()<<'\n';
                }
            }
        }
        if(!changed)break;
    }
    return shape;
}
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        auto args=cadbench::arguments(argc,argv); std::string csv; int repairVertex=-1;double repairTolerance=0;bool probeLocal=false;int pairA=-1,pairB=-1,bodyA=-1,bodyB=-1;
        std::vector<int> inspectFaces;
        for(size_t i=0;i<args.size();)if(args[i]=="--inspect-face"){
            if(i+1>=args.size())throw std::invalid_argument("missing inspect face");
            inspectFaces.push_back(std::stoi(args[i+1]));args.erase(args.begin()+i,args.begin()+i+2);
        }else ++i;
        std::cout<<std::setprecision(17);
        auto bodyPair=std::find(args.begin(),args.end(),"--body-pair");if(bodyPair!=args.end()) {
            auto i=bodyPair-args.begin();if(i+2>=int(args.size()))throw std::invalid_argument("missing body pair");
            bodyA=std::stoi(args[i+1]);bodyB=std::stoi(args[i+2]);args.erase(args.begin()+i,args.begin()+i+3);
        }
        auto pair=std::find(args.begin(),args.end(),"--face-pair");if(pair!=args.end()) {
            auto i=pair-args.begin();if(i+2>=int(args.size()))throw std::invalid_argument("missing pair");
            pairA=std::stoi(args[i+1]);pairB=std::stoi(args[i+2]);args.erase(args.begin()+i,args.begin()+i+3);
        }
        auto local=std::find(args.begin(),args.end(),"--probe-local");if(local!=args.end()){probeLocal=true;args.erase(local);}
        for(size_t i=0;i<args.size();++i) if(args[i]=="--probe-tolerance") {
            if(i+2>=args.size())throw std::invalid_argument("--probe-tolerance needs original all-shape index and tolerance");
            repairVertex=std::stoi(args[i+1]);repairTolerance=std::stod(args[i+2]);
            args.erase(args.begin()+i,args.begin()+i+3);break;
        }
        for (size_t i=0;i<args.size();++i) if (args[i]=="--csv") {
            if (i+1>=args.size()) throw std::invalid_argument("missing --csv path");
            csv=args[i+1]; args.erase(args.begin()+i,args.begin()+i+2); break;
        }
        auto options=cadbench::parse(args,true); TopoDS_Shape shape; QString error;
        if(options.cad=="sphere")shape=BRepPrimAPI_MakeSphere(100).Shape();
        else if (!ShapeLoader::load(QString::fromLocal8Bit(options.cad.c_str()),shape,&error)) throw std::runtime_error(error.toStdString());
        validity(shape);
        if(probeLocal) {shape=localToleranceProbe(shape);validity(shape);}
        if(repairVertex>0) {
            TopTools_IndexedMapOfShape original;TopExp::MapShapes(shape,original);
            auto vertex=TopoDS::Vertex(original(repairVertex));double oldTolerance=BRep_Tool::Tolerance(vertex);
            BRepBuilderAPI_Copy copy(shape,true,false);auto copied=TopoDS::Vertex(copy.ModifiedShape(vertex));
            BRep_Builder builder;builder.UpdateVertex(copied,repairTolerance);
            shape=copy.Shape();
            std::cout<<"PROBE ONLY localTolerance old="<<oldTolerance<<" new="<<BRep_Tool::Tolerance(copied)
                <<" originalAfter="<<BRep_Tool::Tolerance(vertex)<<'\n';validity(shape);
        }
        OccCadGeometry geo(shape,true);
        for(const auto& d:geo.inputDiagnostics())std::cout<<d<<'\n';
        if(!inspectFaces.empty()){
            if(!geo.validationError().empty())throw std::runtime_error(geo.validationError());
            TopTools_IndexedMapOfShape faces;TopExp::MapShapes(geo.acceptedShape(),TopAbs_FACE,faces);
            for(int f:inspectFaces){
                if(f<0||f>=geo.faceCount())throw std::invalid_argument("inspect face out of range");
                BRepAdaptor_Surface surface(TopoDS::Face(faces(f+1)));
                mesher::SurfaceParameterBounds bounds;geo.faceParameterBounds(f,bounds);
                std::cout<<"INSPECT face="<<f<<" body="<<geo.faceBodyId(f)<<" type="<<int(surface.GetType())
                    <<" bounds="<<bounds.uMin<<','<<bounds.uMax<<','<<bounds.vMin<<','<<bounds.vMax<<'\n';
                mesher::Point3D du,dv;geo.faceDerivatives(f,{(bounds.uMin+bounds.uMax)/2,(bounds.vMin+bounds.vMax)/2},du,dv);
                std::cout<<"D1 du="<<norm(du)<<" dv="<<norm(dv)<<" dot="<<du.x*dv.x+du.y*dv.y+du.z*dv.z<<'\n';
                int loopId=0;for(const auto& loop:geo.faceTopology(f).loops){
                    for(auto use:loop){const auto range=geo.edgeRange(use.edge);const auto endpoints=geo.edgeVertices(use.edge);
                        const auto a=geo.edgeUV(f,use,use.reversed?range.second:range.first);
                        const auto b=geo.edgeUV(f,use,use.reversed?range.first:range.second);
                        double sampledLength=0;auto p=geo.edgePoint(use.edge,range.first);
                        for(int k=1;k<=32;++k){const auto q=geo.edgePoint(use.edge,range.first+(range.second-range.first)*k/32);
                            sampledLength+=norm(sub(p,q));p=q;}
                        std::cout<<"TRIM loop="<<loopId<<" edge="<<use.edge<<" vertices="<<endpoints.first<<','<<endpoints.second
                            <<" reversed="<<use.reversed<<" degenerate="<<geo.edgeDegenerate(use.edge)
                            <<" length32="<<sampledLength<<" uv="<<a.x<<','<<a.y<<" -> "<<b.x<<','<<b.y<<'\n';
                    }++loopId;
                }
            }return 0;
        }
        if(bodyA>=0)for(int f=0;f<geo.faceCount();++f) {
            if(pairA<0 && geo.faceBodyId(f)==bodyA)pairA=f;
            if(pairB<0 && geo.faceBodyId(f)==bodyB)pairB=f;
        }
        if(bodyA>=0 && (pairA<0 || pairB<0))throw std::invalid_argument("body pair out of range");
        if(pairA>=0) {
            TopTools_IndexedMapOfShape faces,edges,solids;TopExp::MapShapes(geo.acceptedShape(),TopAbs_FACE,faces);
            TopExp::MapShapes(geo.acceptedShape(),TopAbs_EDGE,edges);TopExp::MapShapes(geo.acceptedShape(),TopAbs_SOLID,solids);
            std::cout<<"solids="<<solids.Extent()<<'\n';
            for(int f:{pairA,pairB}) {
                auto face=TopoDS::Face(faces(f+1));BRepAdaptor_Surface surface(face);double u0,u1,v0,v1;BRepTools::UVBounds(face,u0,u1,v0,v1);
                std::cout<<"FACE "<<f<<" type="<<int(surface.GetType())<<" bounds="<<u0<<','<<u1<<','<<v0<<','<<v1<<" tol="<<BRep_Tool::Tolerance(face)<<'\n';
                if(surface.GetType()==GeomAbs_Cylinder) {auto c=surface.Cylinder();auto p=c.Location();auto d=c.Axis().Direction();
                    std::cout<<"cylinder radius="<<c.Radius()<<" origin="<<p.X()<<','<<p.Y()<<','<<p.Z()<<" axis="<<d.X()<<','<<d.Y()<<','<<d.Z()<<'\n';}
                for(TopExp_Explorer e(face,TopAbs_EDGE);e.More();e.Next())std::cout<<"edge="<<edges.FindIndex(e.Current())-1<<' ';
                std::cout<<'\n';
                for(int s=1;s<=solids.Extent();++s) {TopTools_IndexedMapOfShape fs;TopExp::MapShapes(solids(s),TopAbs_FACE,fs);if(fs.Contains(face))std::cout<<"solid="<<s-1<<'\n';}
            }
            BRepExtrema_DistShapeShape distance(faces(pairA+1),faces(pairB+1));distance.Perform();
            std::cout<<"distanceDone="<<distance.IsDone();if(distance.IsDone())std::cout<<" distance="<<distance.Value()<<" solutions="<<distance.NbSolution();std::cout<<'\n';
            std::vector<int> owners;
            for(int f:{pairA,pairB})for(int s=1;s<=solids.Extent();++s) {
                TopTools_IndexedMapOfShape fs;TopExp::MapShapes(solids(s),TopAbs_FACE,fs);if(fs.Contains(faces(f+1)))owners.push_back(s);
            }
            if(owners.size()==2) {
                BRepAlgoAPI_Common common(solids(owners[0]),solids(owners[1]));common.Build();
                std::cout<<"bodyCommonDone="<<common.IsDone();
                if(common.IsDone()) {
                    TopTools_IndexedMapOfShape sharedSolids,sharedFaces;TopExp::MapShapes(common.Shape(),TopAbs_SOLID,sharedSolids);TopExp::MapShapes(common.Shape(),TopAbs_FACE,sharedFaces);
                    GProp_GProps props;BRepGProp::VolumeProperties(common.Shape(),props);
                    std::cout<<" solids="<<sharedSolids.Extent()<<" faces="<<sharedFaces.Extent()<<" volume="<<props.Mass();
                    auto center=props.CentreOfMass();std::cout<<" center="<<center.X()<<','<<center.Y()<<','<<center.Z();
                    for(int body:owners) {GProp_GProps p;BRepGProp::VolumeProperties(solids(body),p);std::cout<<" bodyVolume="<<p.Mass();}
                }
                std::cout<<'\n';
            }
            return 0;
        }
        if (!geo.validationError().empty()) return 1;
        cadbench::resolve(options,geo);
        // Match the actual surface dialog defaults, including its explicit edge ceiling.
        if (options.criteria.maxSize==0) options.criteria.maxSize=options.criteria.targetSize;
        auto start=std::chrono::steady_clock::now();
        auto mesh=mesher::generateCadSurfaceMesh(geo,options.criteria,nullptr,[&](int,int) {
            return std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<options.timeout;
        },nullptr,mesher::AssemblySurfacePolicy::IndependentBodies);
        std::cout<<std::setprecision(17)<<"ok="<<mesh.ok<<" error="<<mesh.error<<" seconds="
            <<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<" qualityFlips="<<mesh.work.qualityFlips<<" qualityMoves="<<mesh.work.qualityMoves<<" target="<<options.criteria.targetSize
            <<" min="<<options.criteria.minSize<<" max="<<options.criteria.maxSize<<'\n';
        for (const auto& d:mesh.diagnostics) std::cout<<d<<'\n';
        if (!mesh.ok) return 1;
        std::set<int> boundary; std::set<std::pair<int,int>> boundaryEdges;
        for (auto e:mesh.boundaryEdges) { boundary.insert(e.a); boundary.insert(e.b); boundaryEdges.insert(std::minmax(e.a,e.b)); }
        std::ofstream file; if (!csv.empty()) { file.open(csv); if(!file) throw std::runtime_error("cannot open CSV"); }
        if(file) file<<"triangle,face,ratio,minAngle,minEdge,maxEdge,boundaryVertices,boundaryEdges,u,v,x,y,z\n"<<std::setprecision(17);
        std::vector<double> ratios; int bad=0,nearBoundary=0,small=0;
        double worst=0; int worstId=-1;
        for(size_t i=0;i<mesh.mesh.triangles.size();++i) {
            auto t=mesh.mesh.triangles[i]; auto a=mesh.mesh.vertices[t[0]],b=mesh.mesh.vertices[t[1]],c=mesh.mesh.vertices[t[2]];
            auto ab=sub(b,a),ac=sub(c,a); double l[3]={norm(ab),norm(sub(c,b)),norm(ac)};
            double twiceArea=norm({ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x});
            double lo=*std::min_element(l,l+3),hi=*std::max_element(l,l+3);
            double r=l[0]*l[1]*l[2]/(2*twiceArea*lo);
            double angle=std::asin(std::min(1.0,0.5/r))*180/3.14159265358979323846;
            int bv=0,be=0; for(int j=0;j<3;++j) { bv+=int(boundary.count(t[j])); be+=int(boundaryEdges.count(std::minmax(t[j],t[(j+1)%3]))); }
            ratios.push_back(r); if(r>worst){worst=r;worstId=int(i);}
            if(r>options.criteria.maxRadiusEdgeRatio) { ++bad; if(bv)++nearBoundary; if(lo<=2*options.criteria.minSize)++small; }
            auto uv=mesh.triangleCornerUV[i];
            if(file) file<<i<<','<<mesh.triangleFaceIds[i]<<','<<r<<','<<angle<<','<<lo<<','<<hi<<','<<bv<<','<<be<<','
                <<(uv[0].x+uv[1].x+uv[2].x)/3<<','<<(uv[0].y+uv[1].y+uv[2].y)/3<<','
                <<(a.x+b.x+c.x)/3<<','<<(a.y+b.y+c.y)/3<<','<<(a.z+b.z+c.z)/3<<'\n';
        }
        std::sort(ratios.begin(),ratios.end());
        std::cout<<"points="<<mesh.mesh.vertices.size()<<" triangles="<<ratios.size()<<" bad="<<bad<<" boundaryBad="<<nearBoundary
            <<" smallBad="<<small<<" worst="<<worst<<" worstTriangle="<<worstId<<" p95="<<ratios[ratios.size()*95/100]
            <<" p99="<<ratios[ratios.size()*99/100]<<'\n';
        flipCandidates(mesh,options.criteria);
        return 0;
    } catch(const Standard_Failure& e) { std::cerr<<e.GetMessageString()<<'\n'; return 1; }
      catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
