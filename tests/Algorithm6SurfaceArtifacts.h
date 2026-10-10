#pragma once
/** @file Algorithm6SurfaceArtifacts.h
 * @brief 算法6工具的VTP交换与独立质量/CAD采样审计，不参与网格生成。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "CadSurfaceQualityReport.h"
#include "Algorithm6CadAuditQueries.h"
#include "cad/occ/OccCadGeometry.h"
#include "mesh/validation/SurfaceMeshValidation.h"
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkNew.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
#include "mesh/surface/algorithm6/boundary/BoundaryRegistry.h"
namespace algorithm6artifacts {
using namespace mesher;
inline void writeVtp(const SurfaceMeshResult& mesh,const std::filesystem::path& path) {
    vtkNew<vtkPolyData> data;vtkNew<vtkPoints> points;points->SetDataTypeToDouble();
    for(const auto& p:mesh.mesh.vertices)points->InsertNextPoint(p.x,p.y,p.z);data->SetPoints(points);
    vtkNew<vtkCellArray> triangles;
    vtkNew<vtkIntArray> faces,bodies;faces->SetName("CadFaceId");bodies->SetName("CadBodyId");
    vtkNew<vtkDoubleArray> uv,angles;uv->SetName("CadCornerUV");uv->SetNumberOfComponents(6);angles->SetName("MinimumAngleDegrees");
    for(std::size_t k=0;k<mesh.mesh.triangles.size();++k) {
        const auto t=mesh.mesh.triangles[k];vtkIdType ids[3]={t[0],t[1],t[2]};triangles->InsertNextCell(3,ids);
        faces->InsertNextValue(mesh.triangleFaceIds.at(k));bodies->InsertNextValue(mesh.triangleBodyIds.at(k));
        const auto u=mesh.triangleCornerUV.at(k);double v[6]={u[0].x,u[0].y,u[1].x,u[1].y,u[2].x,u[2].y};uv->InsertNextTuple(v);
        angles->InsertNextValue(cadquality::measure(mesh.mesh.vertices[t[0]],mesh.mesh.vertices[t[1]],mesh.mesh.vertices[t[2]]).minAngleDeg);
    }
    data->SetPolys(triangles);data->GetCellData()->AddArray(faces);data->GetCellData()->AddArray(bodies);data->GetCellData()->AddArray(uv);data->GetCellData()->AddArray(angles);
    vtkNew<vtkCellArray> lines;vtkNew<vtkIntArray> edges;edges->SetName("CadEdgeId");
    // XML PolyData cell顺序为lines后polys，所有标签数组都按此顺序对齐。
    for(const auto& e:mesh.boundaryEdges){vtkIdType ids[2]={e.a,e.b};lines->InsertNextCell(2,ids);edges->InsertNextValue(e.cadEdge);}
    data->SetLines(lines);
    if(!mesh.boundaryEdges.empty()) {
        const auto n=mesh.boundaryEdges.size();vtkNew<vtkIntArray> allFaces,allBodies;allFaces->SetName("CadFaceId");allBodies->SetName("CadBodyId");
        vtkNew<vtkDoubleArray> allUv,allAngles;allUv->SetName("CadCornerUV");allUv->SetNumberOfComponents(6);allAngles->SetName("MinimumAngleDegrees");
        for(std::size_t k=0;k<n;++k){allFaces->InsertNextValue(-1);allBodies->InsertNextValue(-1);double z[6]={};allUv->InsertNextTuple(z);allAngles->InsertNextValue(0);}
        for(vtkIdType k=0;k<faces->GetNumberOfTuples();++k){allFaces->InsertNextValue(faces->GetValue(k));allBodies->InsertNextValue(bodies->GetValue(k));allUv->InsertNextTuple(uv->GetTuple(k));allAngles->InsertNextValue(angles->GetValue(k));edges->InsertNextValue(-1);}
        data->GetCellData()->RemoveArray("CadFaceId");data->GetCellData()->RemoveArray("CadBodyId");data->GetCellData()->RemoveArray("CadCornerUV");data->GetCellData()->RemoveArray("MinimumAngleDegrees");
        data->GetCellData()->AddArray(allFaces);data->GetCellData()->AddArray(allBodies);data->GetCellData()->AddArray(allUv);data->GetCellData()->AddArray(allAngles);
    }
    data->GetCellData()->AddArray(edges);
    vtkNew<vtkXMLPolyDataWriter> writer;writer->SetFileName(path.string().c_str());writer->SetInputData(data);writer->SetDataModeToAscii();if(!writer->Write())throw std::runtime_error("VTP export failed");
}
inline SurfaceMeshResult readVtp(const std::filesystem::path& path) {
    vtkNew<vtkXMLPolyDataReader> reader;reader->SetFileName(path.string().c_str());reader->Update();const auto data=reader->GetOutput();
    if(!data->GetNumberOfPoints()||!data->GetNumberOfPolys())throw std::runtime_error("VTP has no surface mesh");
    SurfaceMeshResult mesh;
    for(vtkIdType k=0;k<data->GetNumberOfPoints();++k){double p[3];data->GetPoint(k,p);mesh.mesh.vertices.push_back({p[0],p[1],p[2]});}
    const auto faces=data->GetCellData()->GetArray("CadFaceId"),bodies=data->GetCellData()->GetArray("CadBodyId"),uv=data->GetCellData()->GetArray("CadCornerUV"),edges=data->GetCellData()->GetArray("CadEdgeId");
    if(!faces||!bodies||!uv||!edges||uv->GetNumberOfComponents()!=6)throw std::runtime_error("VTP CAD labels or corner UV missing");
    vtkIdType count;const vtkIdType* ids;vtkIdType cell=0;
    for(data->GetLines()->InitTraversal();data->GetLines()->GetNextCell(count,ids);++cell){if(count!=2)throw std::runtime_error("nonsegment CAD edge");mesh.boundaryEdges.push_back({int(ids[0]),int(ids[1]),int(edges->GetComponent(cell,0))});}
    for(data->GetPolys()->InitTraversal();data->GetPolys()->GetNextCell(count,ids);++cell){if(count!=3)throw std::runtime_error("nontriangle surface");mesh.mesh.triangles.push_back({int(ids[0]),int(ids[1]),int(ids[2])});mesh.triangleFaceIds.push_back(int(faces->GetComponent(cell,0)));mesh.triangleBodyIds.push_back(int(bodies->GetComponent(cell,0)));std::array<Point2D,3> u;for(int k=0;k<3;++k)u[k]={uv->GetComponent(cell,2*k),uv->GetComponent(cell,2*k+1)};mesh.triangleCornerUV.push_back(u);}
    mesh.ok=true;return mesh;
}
// 阶段网格保留运行节点，包括缝副本/极点临时零物理面积片；不冒充最终装配结果。
inline void writeFaceVtp(const gmshReference::FaceMesh& face,const gmshReference::BoundaryMesh& boundary,
    int body,const std::filesystem::path& path) {
    SurfaceMeshResult stage;std::map<std::uint64_t,int> ids;
    for(const auto& v:face.vertices){ids[v.first]=int(stage.mesh.vertices.size());stage.mesh.vertices.push_back(v.second.point);}
    std::map<std::array<std::uint64_t,2>,int> physicalEdges;
    for(const auto& c:boundary.curves)if(!c.meshDegenerate&&std::find(c.incidentFaces.begin(),c.incidentFaces.end(),face.entity)!=c.incidentFaces.end())
        for(std::size_t k=1;k<c.samples.size();++k){auto e=std::array<std::uint64_t,2>{c.samples[k-1].node,c.samples[k].node};std::sort(e.begin(),e.end());physicalEdges[e]=c.entity-1;}
    std::map<std::array<int,2>,int> tagged;
    for(std::size_t i=0;i<face.triangles.size();++i) {
        const auto t=face.triangles[i];std::array<int,3> triangle;std::array<Point2D,3> uv;
        for(int k=0;k<3;++k) {
            const auto& v=face.vertices.at(t[k]);triangle[k]=ids.at(t[k]);
            const auto p=face.parametricCoordinates.find(t[k]);
            uv[k]=!face.triangleCornerUV.empty()?face.triangleCornerUV.at(i)[k]:(p==face.parametricCoordinates.end()?v.parameters:p->second);
            auto e=std::array<std::uint64_t,2>{v.physicalId,face.vertices.at(t[(k+1)%3]).physicalId};std::sort(e.begin(),e.end());
            const auto found=physicalEdges.find(e);
            if(found!=physicalEdges.end()){auto edge=std::array<int,2>{ids.at(t[k]),ids.at(t[(k+1)%3])};std::sort(edge.begin(),edge.end());tagged[edge]=found->second;}
        }
        stage.mesh.triangles.push_back(triangle);stage.triangleCornerUV.push_back(uv);stage.triangleFaceIds.push_back(face.entity-1);stage.triangleBodyIds.push_back(body);
    }
    for(const auto& e:tagged)stage.boundaryEdges.push_back({e.first[0],e.first[1],e.second});
    writeVtp(stage,path);
}
inline Point3D subtract(Point3D a,Point3D b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline double dot(Point3D a,Point3D b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline double segmentDistance(Point3D p,Point3D a,Point3D b) {
    const auto d=subtract(b,a);const double length=dot(d,d),t=length>0?std::clamp(dot(subtract(p,a),d)/length,0.,1.):0.;
    const auto r=subtract(p,{a.x+t*d.x,a.y+t*d.y,a.z+t*d.z});return dot(r,r);
}
inline double triangleDistance(Point3D p,Point3D a,Point3D b,Point3D c) {
    double best=std::min({segmentDistance(p,a,b),segmentDistance(p,b,c),segmentDistance(p,c,a)});
    const auto u=subtract(b,a),v=subtract(c,a),w=subtract(p,a);const double uu=dot(u,u),vv=dot(v,v),uv=dot(u,v),uw=dot(u,w),vw=dot(v,w),det=uu*vv-uv*uv;
    if(det>0){const double x=(uw*vv-vw*uv)/det,y=(vw*uu-uw*uv)/det;if(x>=0&&y>=0&&x+y<=1){const auto r=subtract(p,{a.x+x*u.x+y*v.x,a.y+x*u.y+y*v.y,a.z+x*u.z+y*v.z});best=std::min(best,dot(r,r));}}
    return best;
}
inline double percentile(std::vector<double> values,double q){if(values.empty())return 0;std::sort(values.begin(),values.end());return values.at(std::size_t(q*(values.size()-1)));}
inline void shapeJson(std::ostream& out,const std::vector<cadquality::Metric>& values) {
    std::vector<double> angles,edges;std::size_t nearCount=0,low=0;
    for(const auto& m:values){angles.push_back(m.minAngleDeg);edges.push_back(m.maxEdge);nearCount+=m.minAngleDeg>=50&&m.maxAngleDeg<=70;low+=m.minAngleDeg<25;}
    out<<"{\"triangles\":"<<values.size()<<",\"near_fraction\":"<<(values.empty()?0.:double(nearCount)/values.size())<<",\"angle_p1\":"<<percentile(angles,.01)<<",\"angle_p5\":"<<percentile(angles,.05)<<",\"below25_fraction\":"<<(values.empty()?0.:double(low)/values.size())<<",\"max_edge\":"<<percentile(edges,1)<<'}';
}
inline void distanceJson(std::ostream& out,const std::vector<double>& values) {out<<"{\"samples\":"<<values.size()<<",\"p50\":"<<percentile(values,.5)<<",\"p95\":"<<percentile(values,.95)<<",\"max\":"<<percentile(values,1)<<'}';}
inline void audit(const OccCadGeometry& geometry,const SurfaceMeshResult& mesh,const std::filesystem::path& output) {
    TopTools_IndexedMapOfShape faces;TopExp::MapShapes(geometry.acceptedShape(),TopAbs_FACE,faces);
    std::vector<std::unique_ptr<algorithm6audit::CadFaceQueries>> queries;std::size_t exactLinearPlanes=0,analyticPlanes=0;
    for(int i=1;i<=faces.Extent();++i){queries.push_back(std::make_unique<algorithm6audit::CadFaceQueries>(TopoDS::Face(faces(i))));exactLinearPlanes+=queries.back()->usesExactLinearPlane();analyticPlanes+=queries.back()->usesAnalyticPlane();}
    std::vector<cadquality::Metric> all,boundary,interior;std::vector<double> meshToCad,cadToMesh;
    std::set<int> boundaryNodes;std::set<std::array<int,2>> declaredEdges;
    for(const auto& e:mesh.boundaryEdges){boundaryNodes.insert(e.a);boundaryNodes.insert(e.b);auto k=std::array<int,2>{e.a,e.b};std::sort(k.begin(),k.end());declaredEdges.insert(k);}
    TopTools_IndexedMapOfShape cadEdges;TopExp::MapShapes(geometry.acceptedShape(),TopAbs_EDGE,cadEdges);
    Point3D lo,hi;if(!geometry.bounds(lo,hi))throw std::runtime_error("CAD audit bounding box missing");
    const double labelTolerance=std::max(1.e-10,std::hypot(hi.x-lo.x,hi.y-lo.y,hi.z-lo.z)*1.e-8);
    for(const auto& e:mesh.boundaryEdges) {
        if(e.cadEdge<0||e.cadEdge>=cadEdges.Extent())throw std::runtime_error("CAD edge label outside model");
        BRepExtrema_DistShapeShape distance;distance.LoadS2(cadEdges(e.cadEdge+1));
        for(int id:{e.a,e.b}){const auto p=mesh.mesh.vertices.at(id);distance.LoadS1(BRepBuilderAPI_MakeVertex(gp_Pnt(p.x,p.y,p.z)).Vertex());if(!distance.Perform()||!distance.IsDone()||!std::isfinite(distance.Value())||distance.Value()>labelTolerance)throw std::runtime_error("boundary endpoint disagrees with CAD edge label");}
    }
    std::map<std::array<int,2>,int> incidence,orientation;std::set<std::array<int,3>> unique;
    std::vector<std::vector<std::size_t>> faceTriangles(geometry.faceCount());
    std::size_t outside=0,wrongOrientation=0,wrongUV=0;std::vector<std::string> violations;
    auto violation=[&](std::string message){if(violations.size()<100)violations.push_back(std::move(message));};
    for(std::size_t i=0;i<mesh.mesh.triangles.size();++i) {
        const auto t=mesh.mesh.triangles[i];auto key=t;std::sort(key.begin(),key.end());if(!unique.insert(key).second)throw std::runtime_error("duplicate triangle");
        const auto a=mesh.mesh.vertices.at(t[0]),b=mesh.mesh.vertices.at(t[1]),c=mesh.mesh.vertices.at(t[2]);const auto m=cadquality::measure(a,b,c);if(!m.valid)throw std::runtime_error("invalid triangle");all.push_back(m);
        (boundaryNodes.count(t[0])||boundaryNodes.count(t[1])||boundaryNodes.count(t[2])?boundary:interior).push_back(m);
        const int face=mesh.triangleFaceIds.at(i);if(face<0||face>=faces.Extent())throw std::runtime_error("face label outside CAD");
        if(mesh.triangleBodyIds.at(i)!=geometry.faceBodyId(face))throw std::runtime_error("body label disagrees with CAD face ownership");
        faceTriangles[face].push_back(i);
        const auto u=mesh.triangleCornerUV.at(i);const Point2D centroid{(u[0].x+u[1].x+u[2].x)/3,(u[0].y+u[1].y+u[2].y)/3};
        const auto state=queries.at(face)->classify(gp_Pnt2d(centroid.x,centroid.y),1.e-10);
        if(state!=TopAbs_IN&&state!=TopAbs_ON){++outside;const auto p=geometry.faceSurfacePoint(face,centroid);const auto distance=queries.at(face)->distance(gp_Pnt(p.x,p.y,p.z));SurfaceParameterBounds bounds;geometry.faceTrimParameterBounds(face,bounds);std::ostringstream message;message<<std::setprecision(17)<<"triangle centroid outside trim/hole: triangle="<<i<<" face="<<face<<" uv="<<centroid.x<<','<<centroid.y<<" xyz="<<p.x<<','<<p.y<<','<<p.z<<" cad_tolerance="<<BRep_Tool::Tolerance(TopoDS::Face(faces(face+1)))<<" trim_distance="<<distance<<" bounds="<<bounds.uMin<<','<<bounds.uMax<<','<<bounds.vMin<<','<<bounds.vMax;violation(message.str());}
        const auto ab=subtract(b,a),ac=subtract(c,a),normal=geometry.faceNormal(face,centroid);
        const Point3D cross{ab.y*ac.z-ab.z*ac.y,ab.z*ac.x-ab.x*ac.z,ab.x*ac.y-ab.y*ac.x};
        if(!(dot(cross,normal)>0)){++wrongOrientation;violation("triangle direction disagrees with CAD orientation: triangle="+std::to_string(i)+" face="+std::to_string(face));}
        for(int k=0;k<3;++k){const auto support=geometry.faceSurfacePoint(face,u[k]);const auto residual=subtract(support,mesh.mesh.vertices.at(t[k]));if(std::sqrt(dot(residual,residual))>labelTolerance){++wrongUV;std::ostringstream message;message<<std::setprecision(17)<<"corner UV disagrees with XYZ/CAD face: triangle="<<i<<" face="<<face<<" corner="<<k<<" node="<<t[k]<<" uv="<<u[k].x<<','<<u[k].y<<" residual="<<std::sqrt(dot(residual,residual))<<" tolerance="<<labelTolerance;violation(message.str());}}
        for(int k=0;k<3;++k){auto e=std::array<int,2>{t[k],t[(k+1)%3]};const int sign=e[0]<e[1]?1:-1;std::sort(e.begin(),e.end());++incidence[e];orientation[e]+=sign;}
        const std::array<Point3D,7> samples{a,b,c,Point3D{(a.x+b.x)/2,(a.y+b.y)/2,(a.z+b.z)/2},Point3D{(b.x+c.x)/2,(b.y+c.y)/2,(b.z+c.z)/2},Point3D{(c.x+a.x)/2,(c.y+a.y)/2,(c.z+a.z)/2},Point3D{(a.x+b.x+c.x)/3,(a.y+b.y+c.y)/3,(a.z+b.z+c.z)/3}};
        for(const auto p:samples)meshToCad.push_back(queries.at(face)->distance(gp_Pnt(p.x,p.y,p.z)));
    }
    std::size_t open=0;
    for(const auto& e:incidence){if(e.second>2||(e.second==2&&orientation[e.first]!=0))throw std::runtime_error("nonmanifold or inconsistent triangle orientation");if(e.second==1){++open;if(!declaredEdges.count(e.first))throw std::runtime_error("undeclared open edge");}}
    for(const auto& e:declaredEdges)if(!incidence.count(e))throw std::runtime_error("missing declared CAD segment");
    std::map<std::pair<int,int>,CadBodyContact> contacts;
    // 沿用工程已有IndependentBodies语义；只有CAD证实的跨部件接触/干涉才分列。
    // 同部件相交与无CAD接触证据的跨部件相交仍严格拒绝，不合并部件节点。
    auto acceptedContact=[&](int first,int second){const int a=mesh.triangleFaceIds.at(first),b=mesh.triangleFaceIds.at(second);if(geometry.faceBodyId(a)<0||geometry.faceBodyId(b)<0||geometry.faceBodyId(a)==geometry.faceBodyId(b))return false;const auto contact=geometry.bodyContact(a,b);if(contact==CadBodyContact::QueryFailed)throw std::runtime_error("CAD body contact query failed");if(contact==CadBodyContact::BoundaryContact||contact==CadBodyContact::VolumeOverlap){contacts.emplace(std::make_pair(std::min(a,b),std::max(a,b)),contact);return true;}return false;};
    std::string error;const bool intersectionFree=validateSurfaceIntersections(mesh.mesh,&error,0,nullptr,acceptedContact);if(!intersectionFree)violation("surface intersections: "+error);
    const auto sampleCadPoint=[&](int face,Point3D p){double best=std::numeric_limits<double>::infinity();for(auto i:faceTriangles.at(face)){const auto t=mesh.mesh.triangles[i];best=std::min(best,triangleDistance(p,mesh.mesh.vertices[t[0]],mesh.mesh.vertices[t[1]],mesh.mesh.vertices[t[2]]));}if(!std::isfinite(best))throw std::runtime_error("CAD face has no mesh");cadToMesh.push_back(std::sqrt(best));};
    const auto sampleCad=[&](int face,Point2D uv){sampleCadPoint(face,geometry.faceSurfacePoint(face,uv));};
    for(int f=0;f<geometry.faceCount();++f) {
        SurfaceParameterBounds b;if(!geometry.faceTrimParameterBounds(f,b))throw std::runtime_error("CAD audit bounds failed");
        for(int i=0;i<=80;++i)for(int j=0;j<=80;++j){Point2D uv{b.uMin+(b.uMax-b.uMin)*i/80.,b.vMin+(b.vMax-b.vMin)*j/80.};const auto state=queries.at(f)->classify(gp_Pnt2d(uv.x,uv.y),1.e-10);if(state==TopAbs_IN||state==TopAbs_ON)sampleCad(f,uv);}
        // BREP的pcurve可能与物理曲线参数不一致；边界CAD样本必须来自真实3D曲线。
        const auto topology=geometry.faceTopology(f);for(const auto& loop:topology.loops)for(const auto& edge:loop){const auto range=geometry.edgeRange(edge.edge);for(int k=0;k<=300;++k)sampleCadPoint(f,geometry.edgePoint(edge.edge,range.first+(range.second-range.first)*k/300.));}
    }
    const bool complete=outside==0&&wrongOrientation==0&&wrongUV==0&&intersectionFree;
    std::ofstream out(output);if(!out)throw std::runtime_error("cannot write audit");out<<std::setprecision(17)<<"{\"complete\":"<<(complete?"true":"false")<<",\"sample_contract\":\"v2: trimmed UV 81x81 + 301 physical 3D curve samples per CAD boundary use; triangle 7 barycentric samples\",\"shape\":{\"all\":";shapeJson(out,all);out<<",\"boundary\":";shapeJson(out,boundary);out<<",\"interior\":";shapeJson(out,interior);out<<"},\"mesh_to_cad\":";distanceJson(out,meshToCad);out<<",\"cad_to_mesh\":";distanceJson(out,cadToMesh);out<<",\"topology\":{\"open_edges\":"<<open<<",\"duplicate_triangles\":0,\"nonmanifold_edges\":0,\"wrong_orientation\":"<<wrongOrientation<<",\"outside_centroids\":"<<outside<<",\"wrong_corner_uv\":"<<wrongUV<<",\"intersections\":"<<(intersectionFree?0:1)<<"},\"body_contact_policy\":\"independent CAD bodies; CAD contacts and volume interference reported separately\",\"cad_body_contacts\":[";
    bool comma=false;for(const auto& c:contacts){if(comma)out<<',';comma=true;out<<"{\"first_face\":"<<c.first.first<<",\"second_face\":"<<c.first.second<<",\"first_body\":"<<geometry.faceBodyId(c.first.first)<<",\"second_body\":"<<geometry.faceBodyId(c.first.second)<<",\"contact\":"<<int(c.second)<<"}";}
    out<<"],\"violations\":[";for(std::size_t i=0;i<violations.size();++i){if(i)out<<',';out<<std::quoted(violations[i]);}out<<"],\"violation_details_limit\":100,\"cad_query_contract\":\"v3: original shared CAD vertex closure and roundoff consistency checks; analytic line/circle planar trims and physical distances; other faces retain OCC; UV tolerance 1e-10; analytic distance has no ON snapping\",\"exact_planar_linear_faces\":"<<exactLinearPlanes<<",\"analytic_planar_arc_faces\":"<<analyticPlanes<<"}\n";out.close();
    if(!complete)throw std::runtime_error("final audit failed; full metrics and violation locations saved to "+output.string());
}
}
