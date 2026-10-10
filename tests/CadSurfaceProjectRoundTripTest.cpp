// 真实 CAD 生成结果的工程保存往返：坐标、连接、标签逐值不变。
#include "project/ProjectArchive.h"
#include "app/meshing/SurfaceMeshEngine.h"
#include "OccSplitSurfaceFixtures.h"
#include <BRepTools.hxx>

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkStringArray.h>
#include <vtkDataArray.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkXMLPolyDataReader.h>

#include <chrono>
#include <cmath>
#include <iostream>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    if (argc!=2 && (argc!=3 || std::string(argv[2])!="--algorithm6")) { std::cerr<<"usage: CadSurfaceProjectRoundTripTest CAD-file [--algorithm6]\n"; return 2; }
    const bool algorithm6=argc==3;
    QTemporaryDir fixtureDirectory;
    const bool partitionFixture=QString::fromLocal8Bit(argv[1]).startsWith("split-");
    QString source=QString::fromLocal8Bit(argv[1]);
    if(partitionFixture) {
        source=fixtureDirectory.filePath("original-split.brep");
        const auto shape=QString::fromLocal8Bit(argv[1])=="split-cylinder"?occsplit::cylinder(true):occsplit::plane(true);
        if(!fixtureDirectory.isValid()||!BRepTools::Write(shape,QFile::encodeName(source).constData()))return 1;
    }
    mesher::SurfaceCriteria c; c.targetSize=20; c.maxDeviation=2; c.maxIterations=40000;
    if(partitionFixture){c.targetSize=c.maxSize=1;c.minSize=.1;c.maxDeviation=.02;
        c.pointPlacement=mesher::SurfacePointPlacement::FrontalDelaunay;c.crossSmoothCadSplits=true;}
    if(algorithm6) {
        c.pointPlacement=mesher::SurfacePointPlacement::Algorithm6Experimental;c.crossSmoothCadSplits=false;
        c.algorithm6.targetSize=1;c.algorithm6.extendBoundarySize=0;c.algorithm6.smoothingIterations=1;
        c.algorithm6.maximumNodes=50000;c.algorithm6.maximumGeometryEvaluations=1000000;
        const auto cancelled=SurfaceMeshEngine::run(source,c,[](int,int){return false;});
        if(cancelled.ok||cancelled.surface||!cancelled.error.contains("cancel",Qt::CaseInsensitive)) {
            std::cerr<<"Algorithm 6 cancellation not returned: "<<cancelled.error.toStdString()<<'\n';return 1;
        }
        auto limited=c;limited.algorithm6.maximumGeometryEvaluations=1;
        const auto exhausted=SurfaceMeshEngine::run(source,limited);
        if(exhausted.ok||exhausted.surface||!exhausted.error.contains("budget",Qt::CaseInsensitive)) {
            std::cerr<<"Algorithm 6 query budget not returned: "<<exhausted.error.toStdString()<<'\n';return 1;
        }
    }
    const auto started=std::chrono::steady_clock::now();
    const auto generated=SurfaceMeshEngine::run(source,c,[started](int,int) {
        return std::chrono::steady_clock::now()-started<std::chrono::seconds(30);
    });
    if (!generated.ok || !generated.surface) { std::cerr<<generated.error.toStdString()<<'\n'; return 1; }
    if(partitionFixture && !algorithm6 && (!generated.surface->GetCellData()->GetArray("SurfacePatchId")||
       !generated.surface->GetCellData()->GetArray("CadSupportFaceIds"))) {
        std::cerr<<"cross-face support schema absent\n";return 1;
    }
    if(algorithm6) {
        auto* algorithm=vtkStringArray::SafeDownCast(generated.surface->GetFieldData()->GetAbstractArray("SurfaceGenerationAlgorithm"));
        if(!algorithm||algorithm->GetValue(0)!="algorithm6-experimental; first_order; serial; switch_on_failure=0; quality_not_audited")return 1;
    }
    auto* diagnosticArray=vtkStringArray::SafeDownCast(generated.surface->GetFieldData()->GetAbstractArray("CadSurfaceDiagnostics"));
    if(!diagnosticArray || diagnosticArray->GetNumberOfValues()!=generated.detail.diagnostics.size()) {
        std::cerr<<"surface diagnostics absent from the saved mesh payload\n";return 1;
    }
    for(vtkIdType i=0;i<diagnosticArray->GetNumberOfValues();++i)if(diagnosticArray->GetValue(i)!=generated.detail.diagnostics[i])return 1;
    MeshProject project; project.projectName=QStringLiteral("real CAD surface roundtrip"); project.surfaceCriteria=c;
    ProjectModelEntry geometry; geometry.displayName=QStringLiteral("input CAD");
    geometry.sourceFile=QStringLiteral("geometry/")+QFileInfo(source).fileName();
    ProjectModelEntry surface; surface.displayName=QStringLiteral("surface"); surface.isMesh=true; surface.isSurfaceMesh=true;
    surface.surfaceFile=QStringLiteral("mesh/surface.vtp");
    project.models={geometry,surface};
    QTemporaryDir directory,unpack;
    if (!directory.isValid() || !unpack.isValid()) { std::cerr<<"cannot create temporary archive directories\n"; return 1; }
    const QString package=directory.filePath(QStringLiteral("real-surface.msp"));
    QHash<QString,QString> geometryFiles; geometryFiles.insert(geometry.sourceFile,source);
    QHash<QString,vtkPolyData*> surfaces; surfaces.insert(surface.surfaceFile,generated.surface);
    QString error;
    if (!ProjectArchive::save(project,package,QImage(),QString(),geometryFiles,{},surfaces,&error)) {
        std::cerr<<"archive save failed: "<<error.toStdString()<<'\n'; return 1;
    }
    MeshProject restored;
    if (!ProjectArchive::load(package,restored,unpack,&error)) { std::cerr<<"archive load failed: "<<error.toStdString()<<'\n'; return 1; }
    const auto& d=restored.surfaceCriteria;
    if (restored.models.size()!=2 || restored.models[0].sourceFile!=geometry.sourceFile ||
        !restored.models[1].isSurfaceMesh || !restored.models[1].isMesh ||
        restored.models[1].surfaceFile!=surface.surfaceFile || restored.models[1].meshFile!=QString() ||
        d.targetSize!=c.targetSize || d.minSize!=c.minSize || d.maxDeviation!=c.maxDeviation ||
        d.maxNormalAngleDeg!=c.maxNormalAngleDeg || d.maxRadiusEdgeRatio!=c.maxRadiusEdgeRatio ||
        d.growthRatio!=c.growthRatio || d.maxVertices!=c.maxVertices || d.maxIterations!=c.maxIterations ||
        d.pointPlacement!=c.pointPlacement || d.crossSmoothCadSplits!=c.crossSmoothCadSplits ||
        (algorithm6 && (d.algorithm6.targetSize!=c.algorithm6.targetSize || d.algorithm6.extendBoundarySize!=c.algorithm6.extendBoundarySize ||
        d.algorithm6.smoothingIterations!=c.algorithm6.smoothingIterations || d.algorithm6.maximumNodes!=c.algorithm6.maximumNodes ||
        d.algorithm6.maximumGeometryEvaluations!=c.algorithm6.maximumGeometryEvaluations))) {
        std::cerr<<"restored surface metadata/criteria differ\n"; return 1;
    }
    QFile original(source),copy(unpack.filePath(geometry.sourceFile));
    if (!original.open(QIODevice::ReadOnly) || !copy.open(QIODevice::ReadOnly) || original.readAll()!=copy.readAll()) {
        std::cerr<<"embedded CAD bytes differ\n"; return 1;
    }
    vtkNew<vtkXMLPolyDataReader> reader;
    const QByteArray diskPath=QFile::encodeName(unpack.filePath(surface.surfaceFile)); reader->SetFileName(diskPath.constData()); reader->Update();
    auto* before=generated.surface.GetPointer(); auto* after=reader->GetOutput();
    if (!after || after->GetNumberOfPoints()!=before->GetNumberOfPoints() || after->GetNumberOfCells()!=before->GetNumberOfCells()) {
        std::cerr<<"restored mesh point/cell counts differ\n"; return 1;
    }
    auto* restoredDiagnostics=vtkStringArray::SafeDownCast(after->GetFieldData()->GetAbstractArray("CadSurfaceDiagnostics"));
    if(!restoredDiagnostics || restoredDiagnostics->GetNumberOfValues()!=diagnosticArray->GetNumberOfValues())return 1;
    for(vtkIdType i=0;i<diagnosticArray->GetNumberOfValues();++i)if(diagnosticArray->GetValue(i)!=restoredDiagnostics->GetValue(i))return 1;
    for (vtkIdType i=0;i<before->GetNumberOfPoints();++i) {
        double a[3],b[3]; before->GetPoint(i,a); after->GetPoint(i,b);
        for (int k=0;k<3;++k) if (a[k]!=b[k]) { std::cerr<<"coordinate changed at node "<<i<<'\n'; return 1; }
    }
    vtkNew<vtkIdList> a,b;
    for (vtkIdType i=0;i<before->GetNumberOfCells();++i) {
        before->GetCellPoints(i,a); after->GetCellPoints(i,b);
        if (a->GetNumberOfIds()!=3 || b->GetNumberOfIds()!=3) { std::cerr<<"restored cell is not a triangle\n"; return 1; }
        for (int k=0;k<3;++k) if (a->GetId(k)!=b->GetId(k)) { std::cerr<<"node connectivity changed at triangle "<<i<<'\n'; return 1; }
    }
    for (const char* name: {"CadFaceId","CadEdgeId","CadCornerUV","CadBodyId"}) {
        auto* a=before->GetCellData()->GetArray(name); auto* b=after->GetCellData()->GetArray(name);
        if (!a || !b || a->GetDataType()!=b->GetDataType() || a->GetNumberOfComponents()!=b->GetNumberOfComponents() ||
            a->GetNumberOfTuples()!=b->GetNumberOfTuples() || a->GetNumberOfTuples()!=before->GetNumberOfCells()) {
            std::cerr<<"restored array shape differs: "<<name<<'\n'; return 1;
        }
        for (vtkIdType i=0;i<a->GetNumberOfTuples();++i) for (int k=0;k<a->GetNumberOfComponents();++k)
            if (a->GetComponent(i,k)!=b->GetComponent(i,k)&&!(std::isnan(a->GetComponent(i,k))&&std::isnan(b->GetComponent(i,k)))) {
                std::cerr<<"restored array value differs: "<<name<<" at "<<i<<','<<k<<'\n'; return 1; }
    }
    if(partitionFixture && !algorithm6)for(const char* name:{"SurfacePatchId","SurfacePatchCornerUV","CadSupportFaceIds","CadSupportUV","CadSupportDistance","CadSupportProjectedToTrim"}) {
        auto* a=before->GetCellData()->GetArray(name);auto* b=after->GetCellData()->GetArray(name);
        if(!a||!b||a->GetNumberOfComponents()!=b->GetNumberOfComponents()||a->GetNumberOfTuples()!=b->GetNumberOfTuples())return 1;
        for(vtkIdType i=0;i<a->GetNumberOfTuples();++i)for(int k=0;k<a->GetNumberOfComponents();++k)
            if(a->GetComponent(i,k)!=b->GetComponent(i,k))return 1;
    }
    if(partitionFixture && !algorithm6) {
        auto* a=vtkStringArray::SafeDownCast(before->GetFieldData()->GetAbstractArray("SurfacePatchSourceFaces"));
        auto* b=vtkStringArray::SafeDownCast(after->GetFieldData()->GetAbstractArray("SurfacePatchSourceFaces"));
        if(!a||!b||a->GetNumberOfValues()!=b->GetNumberOfValues())return 1;
        for(vtkIdType i=0;i<a->GetNumberOfValues();++i)if(a->GetValue(i)!=b->GetValue(i))return 1;
    }
    std::cout<<"PASS real CAD archive roundtrip points="<<after->GetNumberOfPoints()<<" triangles="<<after->GetNumberOfCells()<<'\n';
    return 0;
}
