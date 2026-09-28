// 工程存储层自测：save → load → 逐项比对（ZIP 包 / JSON / 几何嵌入 / vtu+vtp 网格）
#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <vtkCellArray.h>
#include <vtkCellType.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLUnstructuredGridReader.h>

#include "core/ProjectArchive.h"

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

    out.meshParams.targetSize = 5.5;
    out.meshParams.minSize = 0.55;
    out.meshParams.curvatureAngle = 25.0;
    out.meshParams.elementOrder = 2;
    out.meshParams.algo3D = 10;
    out.meshParams.optimize = false;
    out.meshParams.optimizeThreshold = 0.42;

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
    check(in.models.size() == 2, "模型条目数 = 2");
    if (in.models.size() == 2) {
        check(in.models[0].displayName == out.models[0].displayName, "几何显示名一致");
        check(in.models[0].color == out.models[0].color, "几何颜色一致");
        check(in.models[0].opacity == 0.75, "几何透明度一致");
        check(!in.models[0].visible, "几何可见性一致");
        check(in.models[0].transform[0] == 1.5 && in.models[0].transform[6] == 2.0, "几何变换一致");
        check(in.models[1].isMesh, "网格条目 isMesh");
        check(in.models[1].surfaceFile == mesh.surfaceFile, "网格外表面路径一致");
    }
    check(in.meshParams.targetSize == 5.5 && in.meshParams.elementOrder == 2 &&
              in.meshParams.algo3D == 10 && !in.meshParams.optimize &&
              in.meshParams.optimizeThreshold == 0.42,
          "网格参数一致");
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
    vtkNew<vtkXMLUnstructuredGridReader> volumeReader;
    volumeReader->SetFileName(unpackedVtu.toLocal8Bit().constData());
    volumeReader->Update();
    check(volumeReader->GetOutput()->GetNumberOfCells() == 1, ".vtu 读回单元数 = 1");
    vtkNew<vtkXMLPolyDataReader> surfaceReader;
    surfaceReader->SetFileName(unpackedVtp.toLocal8Bit().constData());
    surfaceReader->Update();
    check(surfaceReader->GetOutput()->GetNumberOfCells() == 1, ".vtp 读回三角形数 = 1");

    std::cout << (g_failures == 0 ? "== ALL TESTS PASSED ==" : "== TESTS FAILED ==")
              << std::endl;
    return g_failures == 0 ? 0 : 1;
}
