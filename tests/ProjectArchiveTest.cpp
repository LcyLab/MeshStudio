// 工程存储层自测：save → load → 逐项比对（ZIP 包 / JSON / 几何嵌入 / vtu+vtp 网格）
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <QJsonObject>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkCellType.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLUnstructuredGridReader.h>

#include "project/ProjectArchive.h"
#include "project/ProjectPackage.h"

#include <iostream>

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    if (condition) {
        std::cout << "[PASS] " << what << std::endl;
    } else {
        std::cerr << "[FAIL] " << what << std::endl;
        ++g_failures;
    }
}

vtkSmartPointer<vtkUnstructuredGrid> makeSampleVolume()
{
    vtkNew<vtkPoints> points;
    points->InsertNextPoint(0, 0, 0);
    points->InsertNextPoint(10, 0, 0);
    points->InsertNextPoint(0, 10, 0);
    points->InsertNextPoint(0, 0, 10);
    auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->SetPoints(points);
    vtkIdType tet[4] = { 0, 1, 2, 3 };
    grid->InsertNextCell(VTK_TETRA, 4, tet);
    return grid;
}

vtkSmartPointer<vtkPolyData> makeSampleSurface()
{
    vtkNew<vtkPoints> points;
    points->InsertNextPoint(0, 0, 0);
    points->InsertNextPoint(10, 0, 0);
    points->InsertNextPoint(0, 10, 0);
    auto poly = vtkSmartPointer<vtkPolyData>::New();
    poly->SetPoints(points);
    vtkNew<vtkCellArray> tris;
    vtkIdType tri[3] = { 0, 1, 2 };
    tris->InsertNextCell(3, tri);
    poly->SetPolys(tris);
    vtkNew<vtkIntArray> faces;
    faces->SetName("CadFaceId"); faces->InsertNextValue(7);
    vtkNew<vtkDoubleArray> uv;
    uv->SetName("CadCornerUV"); uv->SetNumberOfComponents(6);
    const double corners[6]={0,0,1,0,0,1}; uv->InsertNextTuple(corners);
    vtkNew<vtkIntArray> edges;
    edges->SetName("CadEdgeId"); edges->SetNumberOfComponents(3);
    edges->InsertNextTuple3(2,3,-1);
    poly->GetCellData()->AddArray(faces);
    poly->GetCellData()->AddArray(uv);
    poly->GetCellData()->AddArray(edges);
    return poly;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    // ---- 构造待保存的工程（几何 + 网格 + 参数 + 统计） ----
    MeshProject out;
    out.projectName = QStringLiteral("selftest");

    ProjectModelEntry geo;
    geo.displayName = QStringLiteral("cube2.stp");
    geo.isMesh = false;
    geo.sourceFile = QStringLiteral("geometry/cube2.stp");
    geo.color = QColor(0x12, 0x34, 0x56);
    geo.opacity = 0.75;
    geo.visible = false;
    geo.transform[0] = 1.5;
    geo.transform[6] = 2.0;
    out.models.append(geo);

    ProjectModelEntry mesh;
    mesh.displayName = QStringLiteral("cube2（四面体网格）");
    mesh.isMesh = true;
    mesh.meshFile = QStringLiteral("mesh/mesh.vtu");
    mesh.surfaceFile = QStringLiteral("mesh/mesh.vtp");
    mesh.color = QColor(0xAA, 0xBB, 0xCC);
    out.models.append(mesh);

    ProjectModelEntry triangleMesh;
    triangleMesh.displayName = QStringLiteral("cube2（表面三角网格）");
    triangleMesh.isMesh = true;
    triangleMesh.isSurfaceMesh = true;
    triangleMesh.surfaceFile = QStringLiteral("mesh/surface-only.vtp");
    out.models.append(triangleMesh);

    out.meshParams.targetSize = 5.5;
    out.meshParams.minSize = 0.55;
    out.meshParams.curvatureAngle = 25.0;
    out.meshParams.elementOrder = 2;
    out.meshParams.algo3D = 10;
    out.meshParams.optimize = false;
    out.meshParams.optimizeThreshold = 0.42;
    out.surfaceCriteria.targetSize = 3.5;
    out.surfaceCriteria.minSize = 0.35;
    out.surfaceCriteria.maxSize = 3.75;
    out.surfaceCriteria.maxDeviation = 0.123;
    out.surfaceCriteria.maxNormalAngleDeg = 26.0;
    out.surfaceCriteria.pointPlacement=mesher::SurfacePointPlacement::FrontalDelaunay;
    if(argc==2)out.surfaceCriteria.pointPlacement=QString::fromLocal8Bit(argv[1])=="reference"?mesher::SurfacePointPlacement::ReferenceFrontal:mesher::SurfacePointPlacement::PhysicalReferenceFrontal;
    out.surfaceCriteria.crossSmoothCadSplits=true;

    out.hasMeshStats = true;
    out.meshStats.nodeCount = 4;
    out.meshStats.tetCount = 15968;
    out.meshStats.surfaceTriCount = 4;
    out.meshStats.quadratic = true;
    out.meshStats.minSICN = 0.123;
    out.meshStats.minGamma = 0.30;
    out.meshStats.avgGamma = 0.77;
    out.meshStats.minDihedralDeg = 12.5;
    out.meshStats.maxDihedralDeg = 168.0;
    out.meshStats.invertedCount = 0;
    out.meshStats.minScaledJacobian = 0.123;
    out.meshStats.degenerateCount=2;
    out.meshStats.sliverCount=7;
    out.meshStats.maxEdgeLength=6.25;
    out.meshStats.worstRadiusEdge=9.5;
    out.meshStats.acceptanceKnown=true;
    out.meshStats.qualityAccepted=false;

    QTemporaryDir outDir;
    const QString packagePath = outDir.filePath(QStringLiteral("selftest.msp"));
    const QString geometryDisk = QStringLiteral("F:/Code/MeshStudio/test_data/cube2.stp");

    QHash<QString, QString> geometryFiles;
    geometryFiles.insert(geo.sourceFile, geometryDisk);
    auto volume = makeSampleVolume();
    auto surface = makeSampleSurface();
    QHash<QString, vtkUnstructuredGrid*> volumes;
    volumes.insert(mesh.meshFile, volume);
    QHash<QString, vtkPolyData*> surfaces;
    surfaces.insert(mesh.surfaceFile, surface);
    surfaces.insert(triangleMesh.surfaceFile, surface);

    QString error;
    const bool saved = ProjectArchive::save(out, packagePath, QImage(), QString(),
                                            geometryFiles, volumes, surfaces, &error);
    check(saved, "save() 成功");
    if (!saved)
        std::cerr << "  error: " << error.toLocal8Bit().constData() << std::endl;
    check(QFileInfo::exists(packagePath), "工程包文件已生成");

    // ---- 重新加载并逐项比对 ----
    MeshProject in;
    QTemporaryDir unpackDir;
    const bool loaded = ProjectArchive::load(packagePath, in, unpackDir, &error);
    check(loaded, "load() 成功");
    if (!loaded)
        std::cerr << "  error: " << error.toLocal8Bit().constData() << std::endl;

    check(in.projectName == out.projectName, "工程名一致");
    check(in.models.size() == 3, "模型条目数 = 3");
    if (in.models.size() == 3) {
        check(in.models[0].displayName == out.models[0].displayName, "几何显示名一致");
        check(in.models[0].color == out.models[0].color, "几何颜色一致");
        check(in.models[0].opacity == 0.75, "几何透明度一致");
        check(!in.models[0].visible, "几何可见性一致");
        check(in.models[0].transform[0] == 1.5 && in.models[0].transform[6] == 2.0, "几何变换一致");
        check(in.models[1].isMesh, "网格条目 isMesh");
        check(in.models[1].surfaceFile == mesh.surfaceFile, "网格外表面路径一致");
        check(in.models[2].isMesh && in.models[2].isSurfaceMesh &&
              in.models[2].meshFile.isEmpty() &&
              in.models[2].surfaceFile == triangleMesh.surfaceFile,
              "表面网格条目与路径一致");
    }
    check(in.meshParams.targetSize == 5.5 && in.meshParams.elementOrder == 2 &&
              in.meshParams.algo3D == 10 && !in.meshParams.optimize &&
              in.meshParams.optimizeThreshold == 0.42,
          "网格参数一致");
    check(in.surfaceCriteria.maxSize==3.75,"表面最大单元尺寸保存恢复");
    check(in.surfaceCriteria.pointPlacement==out.surfaceCriteria.pointPlacement &&
          in.surfaceCriteria.crossSmoothCadSplits==out.surfaceCriteria.crossSmoothCadSplits,
          "surface generation strategy and smooth partition policy survive archive");
    check(in.surfaceCriteria.targetSize == 3.5 &&
          in.surfaceCriteria.minSize == 0.35 &&
          in.surfaceCriteria.maxDeviation == 0.123 &&
          in.surfaceCriteria.maxNormalAngleDeg == 26.0,
          "表面三角参数一致");
    check(in.meshStats.minScaledJacobian==0.123 && in.meshStats.degenerateCount==2 &&
          in.meshStats.sliverCount==7 && in.meshStats.maxEdgeLength==6.25 && in.meshStats.worstRadiusEdge==9.5 &&
          in.meshStats.acceptanceKnown && !in.meshStats.qualityAccepted,
          "volume quality acceptance measurements survive project round trip");
    check(in.hasMeshStats && in.meshStats.tetCount == 15968 &&
              in.meshStats.minGamma == 0.30 && in.meshStats.minDihedralDeg == 12.5,
          "质量统计一致");

    // 嵌入的几何源文件字节级一致
    const QString unpackedGeometry = unpackDir.filePath(geo.sourceFile);
    check(QFileInfo::exists(unpackedGeometry), "几何源文件已嵌入包内");
    if (QFileInfo::exists(unpackedGeometry)) {
        QFile original(geometryDisk);
        QFile copied(unpackedGeometry);
        check(original.open(QIODevice::ReadOnly) && copied.open(QIODevice::ReadOnly) &&
                  original.readAll() == copied.readAll(),
              "嵌入几何与源文件逐字节一致");
    }

    // 网格数据可读回
    const QString unpackedVtu = unpackDir.filePath(mesh.meshFile);
    const QString unpackedVtp = unpackDir.filePath(mesh.surfaceFile);
    check(QFileInfo::exists(unpackedVtu), "体网格 .vtu 已嵌入包内");
    check(QFileInfo::exists(unpackedVtp), "网格外表面 .vtp 已嵌入包内");
    check(QFileInfo::exists(unpackDir.filePath(triangleMesh.surfaceFile)),
          "表面专用 .vtp 已嵌入包内");
    vtkNew<vtkXMLUnstructuredGridReader> volumeReader;
    volumeReader->SetFileName(unpackedVtu.toLocal8Bit().constData());
    volumeReader->Update();
    check(volumeReader->GetOutput()->GetNumberOfCells() == 1, ".vtu 读回单元数 = 1");
    vtkNew<vtkXMLPolyDataReader> surfaceReader;
    surfaceReader->SetFileName(unpackedVtp.toLocal8Bit().constData());
    surfaceReader->Update();
    check(surfaceReader->GetOutput()->GetNumberOfCells() == 1, ".vtp 读回三角形数 = 1");
    auto* cells=surfaceReader->GetOutput()->GetCellData();
    check(cells->GetArray("CadFaceId") && cells->GetArray("CadFaceId")->GetComponent(0,0)==7 &&
          cells->GetArray("CadCornerUV") && cells->GetArray("CadCornerUV")->GetNumberOfComponents()==6 &&
          cells->GetArray("CadEdgeId") && cells->GetArray("CadEdgeId")->GetComponent(0,2)==-1,
          "CAD 面/棱标签与逐角 UV 保存后保持一致");

    // 在测试自己的临时解包目录模拟新增 maxSize 字段前的旧工程。
    QFile settings(unpackDir.filePath("state/mesh_settings.json"));
    check(settings.open(QIODevice::ReadOnly),"读取测试参数 JSON");
    auto settingsObject=QJsonDocument::fromJson(settings.readAll()).object(); settings.close();
    auto oldSurface=settingsObject.value("surface").toObject(); oldSurface.remove("maxSize");
    settingsObject.insert("surface",oldSurface);
    check(settings.open(QIODevice::WriteOnly|QIODevice::Truncate),"写入旧格式测试参数");
    settings.write(QJsonDocument(settingsObject).toJson()); settings.close();
    QFile report(unpackDir.filePath("state/mesh_report.json"));
    check(report.open(QIODevice::ReadOnly),"read saved quality report for legacy compatibility test");
    auto oldReport=QJsonDocument::fromJson(report.readAll()).object();report.close();
    for(const char* key:{"minScaledJacobian","acceptanceKnown","qualityAccepted","degenerateCount","sliverCount","maxEdgeLength","worstRadiusEdge"}) oldReport.remove(key);
    check(report.open(QIODevice::WriteOnly|QIODevice::Truncate),"write legacy quality report");
    report.write(QJsonDocument(oldReport).toJson());report.close();
    const QString legacyPath=outDir.filePath("legacy.msp");
    check(ProjectPackage::packDirectory(unpackDir.path(),legacyPath,&error),"打包旧格式工程");
    MeshProject legacy; QTemporaryDir legacyDir;
    check(ProjectArchive::load(legacyPath,legacy,legacyDir,&error),"旧格式工程可加载");
    check(legacy.surfaceCriteria.maxSize==0 && legacy.surfaceCriteria.targetSize==out.surfaceCriteria.targetSize,
          "缺少最大尺寸的旧工程保留目标及旧 5% 语义");
    check(!legacy.meshStats.acceptanceKnown && !legacy.meshStats.qualityAccepted &&
          legacy.meshStats.minScaledJacobian==legacy.meshStats.minSICN,"legacy report never invents quality acceptance");

    std::cout << (g_failures == 0 ? "== ALL TESTS PASSED ==" : "== TESTS FAILED ==")
              << std::endl;
    return g_failures == 0 ? 0 : 1;
}
