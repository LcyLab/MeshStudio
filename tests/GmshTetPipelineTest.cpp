// 独立 Gmsh 面网格 → 自研四面体测试；不调用 SurfaceMeshEngine/CadSurfaceMesher。
#include "app/meshing/TetMeshEngine.h"
#include "mesh/validation/VolumeBoundaryValidation.h"
#include "mesh/volume/Delaunay3D.h"
#include "mesh/volume/Constrained3D.h"
#include "mesh/volume/VolumeDomain3D.h"
#include "geometry/predicates/GeomPredicates.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileInfo>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <algorithm>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    if(argc!=5) {std::cerr<<"GmshTetPipelineTest <surface.gmstet> <volumeTarget> <case> <report.json>\n";return 2;}
    mesher::SurfaceMeshResult boundary;
    std::ifstream input(argv[1]);std::string magic;
    int nodes=0,triangles=0,edges=0;double expected=0;
    input>>magic>>nodes>>triangles>>edges>>expected;
    if(!input || magic!="GMSTET1" || nodes<4 || nodes>=(1<<21)-4 || triangles<4 || triangles>2000000 ||
       edges<0 || edges>triangles*3 || !std::isfinite(expected) || expected<=0) return 2;
    boundary.mesh.vertices.resize(nodes);boundary.mesh.triangles.resize(triangles);
    boundary.triangleFaceIds.resize(triangles);boundary.triangleCornerUV.resize(triangles);
    boundary.boundaryEdges.resize(edges);
    for(auto& p:boundary.mesh.vertices) input>>p.x>>p.y>>p.z;
    for(int t=0;t<triangles;++t) {
        auto& f=boundary.mesh.triangles[t];input>>boundary.triangleFaceIds[t]>>f[0]>>f[1]>>f[2];
        for(auto& uv:boundary.triangleCornerUV[t]) input>>uv.x>>uv.y;
    }
    for(auto& e:boundary.boundaryEdges) input>>e.a>>e.b>>e.cadEdge;
    if(!input) return 2;
    std::string extra;if(input>>extra) return 2;
    boundary.ok=true;
    const auto checked=mesher::validateVolumeBoundary(boundary.mesh);
    TetMeshParams params;params.optimize=true;params.targetSize=QString::fromLocal8Bit(argv[2]).toDouble();
    const auto result=TetMeshEngine::run(params,boundary);
    const bool expectedRejection=std::string(argv[3])=="sphere-unsupported";
    int failures=0;
    auto check=[&](bool ok,const char* text) {std::cout<<(ok?"PASS ":"FAIL ")<<text<<'\n';failures+=!ok;};
    check(checked.ok,"raw Gmsh surface passes independent volume boundary gate");
    check(expectedRejection ? !result.ok && result.error.contains("volume boundary face not recovered") : result.ok,
          expectedRejection ? "known unsupported coarse curved boundary is explicitly rejected" : "self-developed tetrahedral engine accepts Gmsh surface");
    double volume=0;bool positive=true,domain=true;
    vtkNew<vtkIdList> ids;
    const std::string kind=argv[3];
    if(result.volume) for(vtkIdType t=0;t<result.volume->GetNumberOfCells();++t) {
        result.volume->GetCellPoints(t,ids);
        if(ids->GetNumberOfIds()!=4) {positive=false;continue;}
        double p[4][3],d[3][3],center[3]={};
        for(int k=0;k<4;++k) {result.volume->GetPoint(ids->GetId(k),p[k]);for(int j=0;j<3;++j) center[j]+=p[k][j]/4;}
        for(int k=0;k<3;++k) for(int j=0;j<3;++j) d[k][j]=p[k+1][j]-p[0][j];
        const double det=d[0][0]*(d[1][1]*d[2][2]-d[1][2]*d[2][1])-d[0][1]*(d[1][0]*d[2][2]-d[1][2]*d[2][0])+d[0][2]*(d[1][0]*d[2][1]-d[1][1]*d[2][0]);
        positive=positive && det>0 && std::isfinite(det);volume+=det/6;
        if(kind=="concave") domain=domain && !(center[0]>1+1e-9 && center[1]>1+1e-9);
        if(kind=="cavity") domain=domain && !(center[0]>1+1e-9 && center[0]<2-1e-9 && center[1]>1+1e-9 && center[1]<2-1e-9 && center[2]>1+1e-9 && center[2]<2-1e-9);
        if(kind=="disconnected") domain=domain && !(center[0]>1+1e-9 && center[0]<2-1e-9);
    }
    if(result.ok) {
        check(result.weldMergedVertices==0,"Gmsh node identities preserved without welding");
        check(positive && std::abs(volume-expected)<=expected*1e-8,"independent tet determinants match Gmsh boundary integral");
        check(domain,"no tet centroids in known concavity/cavity/separating gap");
        check(result.stats.minGamma>=0 && result.stats.minGamma<=1+1e-8 && result.stats.avgGamma<=1+1e-8,"true gamma stays within physical bounds");
        check(result.qualityAccepted || !result.warnings.empty(),"quality failure has an explicit warning");
        check(!result.qualityAccepted || (result.refineConverged && result.stats.degenerateCount==0 &&
              result.stats.sliverCount==0 && result.stats.invertedCount==0 && result.stats.maxEdgeLength<=params.targetSize*(1+1e-8) &&
              result.stats.worstRadiusEdge<=2*(1+1e-8)),"accepted result never hides unresolved size, radius, slivers or degeneracy");
        auto output=TetMeshEngine::boundarySnapshot(result.surface);
        check(output.ok && output.triangleFaceIds.size()==output.mesh.triangles.size() &&
              output.triangleCornerUV.size()==output.mesh.triangles.size(),"Gmsh face tags and corner UV survive volume output");
        bool curvesCovered=true;
        for(const auto& original:boundary.boundaryEdges) {
            const auto a=boundary.mesh.vertices[original.a],b=boundary.mesh.vertices[original.b];
            const double d[3]={b.x-a.x,b.y-a.y,b.z-a.z},length2=d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
            std::vector<std::pair<double,double>> intervals;
            for(const auto& edge:output.boundaryEdges) if(edge.cadEdge==original.cadEdge) {
                double s[2];bool collinear=true;
                const mesher::Point3D p[2]={output.mesh.vertices[edge.a],output.mesh.vertices[edge.b]};
                for(int k=0;k<2;++k) {
                    const double q[3]={p[k].x-a.x,p[k].y-a.y,p[k].z-a.z};
                    s[k]=(q[0]*d[0]+q[1]*d[1]+q[2]*d[2])/length2;
                    for(int j=0;j<3;++j) collinear=collinear && std::abs(q[j]-s[k]*d[j])<1e-9;
                }
                if(collinear) intervals.emplace_back(std::max(0.0,std::min(s[0],s[1])),std::min(1.0,std::max(s[0],s[1])));
            }
            std::sort(intervals.begin(),intervals.end());double end=0;
            for(auto interval:intervals) if(interval.first<=end+1e-8 && interval.second>interval.first) end=std::max(end,interval.second);
            curvesCovered=curvesCovered && end>=1-1e-8;
        }
        check(curvesCovered,"every Gmsh curve segment retains labeled boundary coverage");
    } else {
        check(!result.error.isEmpty() && !result.volume && !result.surface,"unsupported boundary fails explicitly without partial output");
        mesher::TetMesh3D probe;
        const auto initial=mesher::delaunay3D(probe,boundary.mesh.vertices);
        if(initial.ok) {
            std::vector<std::array<int,3>> faces;
            for(const auto& f:boundary.mesh.triangles) faces.push_back({f[0]+4,f[1]+4,f[2]+4});
            std::string error;mesher::enforceSurface3D(probe,faces,&error);
            const auto coverage=mesher::measureVolumeBoundaryCoverage(probe,faces);
            for(int i:coverage.missingFaces) {
                const auto f=faces[i];int positive=0,negative=0;
                for(size_t v=4;v<probe.points.size();++v) {
                    const int s=mesher::orient3d(probe.points[f[0]],probe.points[f[1]],probe.points[f[2]],probe.points[v]);
                    positive+=s>0;negative+=s<0;
                }
                std::cout<<"missingInputFace="<<i<<" exactFacePresent="<<probe.hasFaceAny(f[0],f[1],f[2])<<" pointSides="<<positive<<'/'<<negative<<'\n';
            }
        }
    }
    QJsonObject report;
    report["case"]=QString::fromLocal8Bit(argv[3]);report["input"]=QString::fromLocal8Bit(argv[1]);
    report["source"]=QStringLiteral("Gmsh generate(2), own tetrahedral engine; self-developed surface generator not called");
    report["targetSize"]=params.targetSize;report["inputPoints"]=nodes;report["inputTriangles"]=triangles;
    report["boundaryAccepted"]=checked.ok;report["boundaryError"]=QString::fromStdString(checked.error);
    report["ok"]=result.ok;report["error"]=result.error;report["qualityAccepted"]=result.qualityAccepted;
    report["refineConverged"]=result.refineConverged;report["refineDiagnostic"]=result.refineDiagnostic;
    report["tetCount"]=qint64(result.stats.tetCount);report["surfaceVolume"]=expected;report["tetVolume"]=volume;
    report["minGamma"]=result.stats.minGamma;report["maxEdgeLength"]=result.stats.maxEdgeLength;
    report["minDihedralDeg"]=result.stats.minDihedralDeg;report["slivers"]=qint64(result.stats.sliverCount);
    if(std::isfinite(result.stats.worstRadiusEdge)) report["worstRadiusEdge"]=result.stats.worstRadiusEdge;
    QJsonArray warnings;for(const auto& warning:result.warnings) warnings.append(warning);report["warnings"]=warnings;
    report["assertionFailures"]=failures;
    if(result.ok) {
        const auto prefix=QFileInfo(QString::fromLocal8Bit(argv[4])).absolutePath()+"/"+QFileInfo(QString::fromLocal8Bit(argv[4])).completeBaseName();
        vtkNew<vtkXMLUnstructuredGridWriter> volumeWriter;volumeWriter->SetFileName((prefix+".vtu").toUtf8().constData());volumeWriter->SetInputData(result.volume);
        vtkNew<vtkXMLPolyDataWriter> surfaceWriter;surfaceWriter->SetFileName((prefix+".vtp").toUtf8().constData());surfaceWriter->SetInputData(result.surface);
        if(!volumeWriter->Write() || !surfaceWriter->Write()) return 2;
        report["volumeArtifact"]=prefix+".vtu";report["surfaceArtifact"]=prefix+".vtp";
    }
    QFile out(QString::fromLocal8Bit(argv[4]));if(!out.open(QIODevice::WriteOnly)) return 2;
    out.write(QJsonDocument(report).toJson());out.close();
    std::cout<<QJsonDocument(report).toJson(QJsonDocument::Compact).constData()<<'\n';
    return failures?1:0;
}
