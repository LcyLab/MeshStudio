#include "cad/occ/OccCadGeometry.h"
#include "cad/import/ShapeLoader.h"
#include "app/meshing/SurfaceMeshEngine.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "CadSurfaceQualityReport.h"
#include "mesh/surface/legacy/SurfaceMesh3D.h"

#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <set>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <QTemporaryDir>
#include <BRepTools.hxx>
#include "mesh/validation/SurfaceMeshValidation.h"
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Circ.hxx>

int main(int argc,char** argv) {
    if (argc<2 || argc>5) return 2;
    TopoDS_Shape shape;
    QString error;
    const std::string input=argv[1];
    const bool sphere=input=="sphere" || input=="sphere-tight" || input=="sphere-quality" || input=="sphere-reference" || input=="sphere-reference-physical";
    const bool cylinder=input=="cylinder";
    const bool annulus=input=="annulus";
    const bool assembly=input=="assembly-contact" || input=="assembly-overlap" || input=="assembly-independent";
    if (sphere) shape=BRepPrimAPI_MakeSphere(100.0).Shape();
    else if (std::string(argv[1])=="cylinder") shape=BRepPrimAPI_MakeCylinder(100.0,200.0).Shape();
    else if (annulus) {
        const gp_Ax2 axes(gp_Pnt(0,0,0),gp_Dir(0,0,1));
        const TopoDS_Wire outer=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,100.0)));
        TopoDS_Wire inner=BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(axes,99.0)));
        inner.Reverse();
        BRepBuilderAPI_MakeFace face(outer);
        face.Add(inner); shape=face.Shape();
    }
    else if(assembly) {
        TopoDS_Compound compound;BRep_Builder builder;builder.MakeCompound(compound);
        builder.Add(compound,BRepPrimAPI_MakeBox(1,1,1).Shape());
        builder.Add(compound,BRepPrimAPI_MakeBox(gp_Pnt(input=="assembly-contact"?1:0.5,0,0),1,1,1).Shape());
        shape=compound;
    }
    else if (!ShapeLoader::load(QString::fromLocal8Bit(argv[1]),shape,&error)) {
        std::cerr << error.toStdString() << '\n'; return 1;
    }
    if (!BRepCheck_Analyzer(shape).IsValid()) {
        std::cerr << "input B-Rep is invalid\n"; return 1;
    }
    OccCadGeometry geo(shape);
    mesher::SurfaceCriteria c;
    c.targetSize=sphere || cylinder || annulus ? 50.0 : 250.0;
    c.maxDeviation=c.targetSize*0.1;
    c.maxIterations=1000;
    if (annulus) c.maxIterations=10000;
    if(input=="sphere-reference"||input=="sphere-reference-physical") {
        c.pointPlacement=input=="sphere-reference"?mesher::SurfacePointPlacement::ReferenceFrontal:mesher::SurfacePointPlacement::PhysicalReferenceFrontal;
        c.maxIterations=5000;
    }
    if (argc>=3) c.targetSize=std::stod(argv[2]);
    if (argc>=4) c.maxIterations=std::stoi(argv[3]);
    c.maxDeviation=c.targetSize*0.1;
    if(argc==5){c.maxSize=c.targetSize;c.minSize=c.targetSize*0.1;}
    if (input=="sphere-tight") {
        c.maxDeviation=0.8;
        c.maxNormalAngleDeg=15;
        c.maxIterations=5000;
    }
    if(input=="sphere-quality") {
        c.targetSize=c.maxSize=std::sqrt(3.0)*200*0.025;
        c.minSize=c.maxDeviation=c.targetSize*0.1;
        c.maxIterations=200000;
    }
    if(assembly) {c.targetSize=c.maxSize=0.5;c.minSize=c.maxDeviation=0.05;c.maxIterations=10000;}
    struct RingSizeField final : mesher::MeshSizeField {
        double sizeAt(const mesher::Point3D& p) const override {
            return std::hypot(p.x,p.y)>99.5?50.0:20.0;
        }
    } ringField;
    const auto started=std::chrono::steady_clock::now();
    const auto r=mesher::generateCadSurfaceMesh(geo,c,annulus?&ringField:nullptr,[started,input](int,int) {
        return std::chrono::steady_clock::now()-started<std::chrono::seconds(input=="sphere-quality"?55:30);
    },nullptr,input=="assembly-independent" || (!assembly && !sphere && !cylinder && !annulus)
        ?mesher::AssemblySurfacePolicy::IndependentBodies:mesher::AssemblySurfacePolicy::RejectBodyOverlap);
    if(input=="assembly-overlap") {
        if(r.ok || r.error.find("volume overlap")==std::string::npos) {std::cerr<<"overlapping CAD bodies were not rejected: "<<r.error<<'\n';return 1;}
        std::cout<<"PASS positive common volume rejected: "<<r.error<<'\n';return 0;
    }
    if (!r.ok) { std::cerr << r.error << '\n'; return 1; }
    if(argc==5){
        const auto report=cadquality::analyze(r,c.maxRadiusEdgeRatio,c.minSize);
        std::cout<<"independent quality worst="<<report.worst<<" minAngle="<<report.minAngle<<" bad="<<report.bad<<'\n';
        if(report.invalid || report.worst>std::stod(argv[4])){
            std::cerr<<"avoidable worst facet exceeded frozen quality regression ceiling\n";return 1;}
    }
    if(assembly) {
        if(std::set<int>(r.triangleBodyIds.begin(),r.triangleBodyIds.end())!=std::set<int>{0,1}) {std::cerr<<"assembly regions lost\n";return 1;}
        std::set<int> nodes[2];
        for(size_t i=0;i<r.mesh.triangles.size();++i)for(int v:r.mesh.triangles[i])nodes[r.triangleBodyIds[i]].insert(v);
        for(int v:nodes[0])if(nodes[1].count(v)) {std::cerr<<"separate CAD body nodes were welded\n";return 1;}
        std::string intersections;
        if(mesher::validateSurfaceIntersections(r.mesh,&intersections)) {std::cerr<<"strict validator did not detect independent coincident interfaces\n";return 1;}
        if(input=="assembly-independent" && std::none_of(r.bodyInteractions.begin(),r.bodyInteractions.end(),[](auto i){return i.contact==mesher::CadBodyContact::VolumeOverlap;})) {
            std::cerr<<"retained interference not recorded\n";return 1;
        }
        QTemporaryDir directory;const auto path=directory.filePath("assembly.brep");
        if(!directory.isValid() || !BRepTools::Write(shape,path.toLocal8Bit().constData()))return 1;
        const auto vtkResult=SurfaceMeshEngine::run(path,c);
        if(!vtkResult.ok || !vtkResult.surface || !vtkResult.surface->GetCellData()->GetArray("CadBodyId")) {
            std::cerr<<"assembly VTK labels missing\n";return 1;
        }
    }
    if(input=="sphere-quality") {
        int bad=0; double worst=0,interiorWorst=0;std::set<int> trim;
        for(auto e:r.boundaryEdges){trim.insert(e.a);trim.insert(e.b);}
        for(auto t:r.mesh.triangles) {
            auto a=r.mesh.vertices[t[0]],b=r.mesh.vertices[t[1]],d=r.mesh.vertices[t[2]];
            const auto len=[](mesher::Point3D a,mesher::Point3D b){return std::hypot(a.x-b.x,a.y-b.y,a.z-b.z);};
            const double x=len(a,b),y=len(b,d),z=len(d,a);
            const double ux=b.x-a.x,uy=b.y-a.y,uz=b.z-a.z,vx=d.x-a.x,vy=d.y-a.y,vz=d.z-a.z;
            const double twiceArea=std::hypot(uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx);
            const double q=x*y*z/(2*twiceArea*std::min({x,y,z}));
            if(q>c.maxRadiusEdgeRatio)++bad; worst=std::max(worst,q);
            if(!trim.count(t[0]) && !trim.count(t[1]) && !trim.count(t[2]))interiorWorst=std::max(interiorWorst,q);
        }
        std::cout<<"quality bad="<<bad<<" triangles="<<r.mesh.triangles.size()<<" worst="<<worst<<'\n';
        if(bad>r.mesh.triangles.size()/20 || interiorWorst>5.74) {
            std::cerr<<"smooth sphere contains avoidable skinny physical triangles (interior min angle below 5 degrees or over 5% bad)\n";
            return 1;
        }
    }
    if (r.measuredMaxDeviation>c.maxDeviation*(1+1e-8) ||
        r.measuredMaxNormalAngleDeg>c.maxNormalAngleDeg*(1+1e-8) ||
        (input=="sphere-tight" && r.mesh.triangles.size()<=600)) {
        std::cerr << "surface criteria tightening was not respected\n"; return 1;
    }
    std::string water;
    if (geo.expectsClosedSurface() && !mesher::isWatertight(r.mesh,&water)) {
        std::cerr << water << '\n'; return 1;
    }
    if (annulus && r.diagnostics.empty()) { std::cerr << "narrow CAD ring conflict not reported\n"; return 1; }
    if (annulus) {
        bool rebuilt=false;
        for (const auto& d:r.diagnostics) rebuilt|=d.find("incident faces rebuilt")!=std::string::npos;
        if (!rebuilt) { std::cerr << "coarse curved trim intersections did not trigger boundary rebuilding\n"; return 1; }
    }
    if (!sphere && !cylinder && !annulus && !assembly) {
        const auto vtkResult=SurfaceMeshEngine::run(QString::fromLocal8Bit(argv[1]),c);
        if (!vtkResult.ok || !vtkResult.surface ||
            vtkResult.surface->GetNumberOfCells()!=static_cast<vtkIdType>(r.mesh.triangles.size()) ||
            !vtkResult.surface->GetCellData()->GetArray("CadFaceId") ||
            !vtkResult.surface->GetCellData()->GetArray("CadCornerUV") ||
            !vtkResult.surface->GetCellData()->GetArray("CadEdgeId")) {
            std::cerr << "VTK surface metadata missing\n"; return 1;
        }
    }
    std::cout << "PASS triangles=" << r.mesh.triangles.size()
              << " vertices=" << r.mesh.vertices.size()
              << " maxEdge="<<r.measuredMaxEdgeLength
              << " maxDeviation="<<r.measuredMaxDeviation
              << " maxAngle="<<r.measuredMaxNormalAngleDeg<< '\n';
    return 0;
}
