#include "app/meshing/TetMeshEngine.h"
#include <QCoreApplication>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <cmath>
#include <iostream>
#include <limits>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    int failures=0;
    auto check=[&](bool ok,const char* name) {std::cout<<(ok?"PASS ":"FAIL ")<<name<<'\n';failures+=!ok;};
    TetMeshParams params;params.targetSize=100;params.optimize=false;
    mesher::SurfaceMeshResult boundary;boundary.ok=true;
    boundary.mesh.vertices={{0,0,0},{1,0,0},{0,1,0},{0,0,1}};
    boundary.mesh.triangles={{0,2,1},{0,1,3},{1,2,3},{2,0,3}};
    boundary.triangleFaceIds={0,1,2,3};
    for(const auto& f:boundary.mesh.triangles) {
        std::array<mesher::Point2D,3> uv;
        for(int k=0;k<3;++k) {auto p=boundary.mesh.vertices[f[k]];uv[k]={p.x+2*p.z,p.y+3*p.z};}
        boundary.triangleCornerUV.push_back(uv);
    }
    boundary.boundaryEdges={{0,1,10},{0,2,11},{0,3,12},{1,2,13},{1,3,14},{2,3,15}};
    const auto result=TetMeshEngine::run(params,boundary);
    check(result.ok && result.qualityAccepted && result.weldMergedVertices==0,"indexed input preserves identity and acceptance");
    auto snapshot=TetMeshEngine::boundarySnapshot(result.surface);
    check(snapshot.ok && snapshot.triangleFaceIds.size()==4 && snapshot.boundaryEdges.size()==12,"CAD face and edge labels survive volume output");
    bool uvOk=snapshot.triangleCornerUV.size()==4;
    for(size_t t=0;uvOk && t<snapshot.mesh.triangles.size();++t) for(int k=0;k<3;++k) {
        auto p=snapshot.mesh.vertices[snapshot.mesh.triangles[t][k]];auto uv=snapshot.triangleCornerUV[t][k];
        uvOk=uvOk && std::abs(uv.x-p.x-2*p.z)<1e-12 && std::abs(uv.y-p.y-3*p.z)<1e-12;
    }
    check(uvOk,"corner UV follows outward boundary vertex order");
    check(TetMeshEngine::run(params,snapshot).ok,"volume boundary snapshot can be reused");
    boundary.boundaryEdges[0].a=99;
    auto invalid=TetMeshEngine::run(params,boundary);
    check(!invalid.ok && !invalid.volume && !invalid.surface,"invalid CAD edge index rejected without partial output");
    boundary.boundaryEdges[0].a=0;
    boundary.triangleCornerUV[0][0].x=std::numeric_limits<double>::quiet_NaN();
    check(!TetMeshEngine::run(params,boundary).ok,"nonfinite corner UV rejected");
    if(result.surface) {
        vtkNew<vtkPolyData> bad;bad->DeepCopy(result.surface);
        vtkNew<vtkDoubleArray> labels;labels->SetName("CadFaceId");
        for(int i=0;i<4;++i) labels->InsertNextValue(i==0?0.5:i);
        bad->GetCellData()->AddArray(labels);
        check(!TetMeshEngine::boundarySnapshot(bad).ok,"fractional CAD ID rejected before integer conversion");
    }
    mesher::SurfaceMeshResult concave;concave.ok=true;
    const double xy[6][2]={{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}};
    for(int z=0;z<2;++z) for(auto& p:xy) concave.mesh.vertices.push_back({p[0],p[1],double(z)});
    for(auto f:std::vector<std::array<int,3>>{{0,1,3},{1,2,3},{0,3,5},{3,4,5}}) {
        concave.mesh.triangles.push_back({f[0],f[2],f[1]});concave.mesh.triangles.push_back({f[0]+6,f[1]+6,f[2]+6});
    }
    for(int i=0;i<6;++i) {int j=(i+1)%6;concave.mesh.triangles.push_back({i,j,j+6});concave.mesh.triangles.push_back({i,j+6,i+6});}
    const auto l=TetMeshEngine::run(params,concave);
    double volume=0;bool outside=false;
    if(l.volume) {
        vtkNew<vtkIdList> ids;
        for(vtkIdType t=0;t<l.volume->GetNumberOfCells();++t) {
            l.volume->GetCellPoints(t,ids);mesher::Point3D p[4];double cx=0,cy=0;
            for(int k=0;k<4;++k) {double q[3];l.volume->GetPoint(ids->GetId(k),q);p[k]={q[0],q[1],q[2]};cx+=q[0]/4;cy+=q[1]/4;}
            auto delta=[](mesher::Point3D a,mesher::Point3D b) {return mesher::Point3D{a.x-b.x,a.y-b.y,a.z-b.z};};
            auto a=delta(p[1],p[0]),b=delta(p[2],p[0]),c=delta(p[3],p[0]);
            volume+=std::abs(a.x*(b.y*c.z-b.z*c.y)-a.y*(b.x*c.z-b.z*c.x)+a.z*(b.x*c.y-b.y*c.x))/6;
            outside=outside || (cx>1+1e-10 && cy>1+1e-10);
        }
    }
    check(l.ok && std::abs(volume-3)<1e-10 && !outside,"full engine excludes concave exterior and preserves volume 3");
    if(!l.ok) std::cout<<l.error.toStdString()<<'\n';
    auto duplicate=boundary;duplicate.triangleCornerUV.clear();duplicate.boundaryEdges.clear();
    duplicate.mesh.vertices.push_back(duplicate.mesh.vertices[0]);
    for(auto& f:duplicate.mesh.triangles) for(int& v:f) if(v==0) v=4;
    const auto repeated=TetMeshEngine::run(params,duplicate);
    check(!repeated.ok && !repeated.volume,"coincident indexed nodes rejected before Delaunay deduplication");
    return failures?1:0;
}
