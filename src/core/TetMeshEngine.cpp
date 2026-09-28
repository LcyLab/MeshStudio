// 自研四面体网格引擎（M5 集成）：表面三角形汤 → 自研算法链路 → vtkUnstructuredGrid
// 链路：顶点焊接/水密检查 → Delaunay 四面体化 → 约束边恢复 → Ruppert 细化
//       → Laplacian 平滑 → 质量统计
// 取代此前的 Gmsh 库调用（Gmsh 依赖已从 CMake 移除）。
//
// 两个入口：
//   run(params)                      —— 读 STL 文件后转调下面的重载（快捷入口）
//   run(params, soupVerts, soupTris) —— 统一主链路，接受任意来源的三角形汤
// 本文件**刻意不依赖 OCC**：STEP/IGES/BRep 的离散化在 ShapeTessellator 里做，
// 由调用方（MainWindow / BRepInputTest）完成后把三角汤传进来。这样只链接
// VTK 的 TetMeshEngineTest 也能直接测引擎本体。
#include "TetMeshEngine.h"

#include <QCoreApplication>
#include <QFileInfo>

#include <algorithm>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <vtkCellArray.h>
#include <vtkCellType.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "mesher/Constrained3D.h"
#include "mesher/Delaunay3D.h"
#include "mesher/GeomPredicates.h"
#include "mesher/Quality3D.h"
#include "mesher/Ruppert3D.h"
#include "mesher/Smooth3D.h"
#include "mesher/SurfaceMesh3D.h"

namespace {

double dist3(const mesher::Point3D& a, const mesher::Point3D& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)
                     + (a.z - b.z) * (a.z - b.z));
}

// 读 STL（二进制/ASCII 由 VTK 自动识别）→ 三角形汤
bool readStlSoup(const QString& path,
                 std::vector<mesher::Point3D>& vertices,
                 std::vector<std::array<int, 3>>& triangles,
                 QString* error)
{
    vtkNew<vtkSTLReader> reader;
    reader->SetFileName(path.toUtf8().constData());
    reader->Update();
    vtkPolyData* poly = reader->GetOutput();
    if (!poly || poly->GetNumberOfPoints() == 0) {
        *error = QCoreApplication::tr("STL 读取失败或无有效三角形：%1").arg(path);
        return false;
    }
    const vtkIdType nPts = poly->GetNumberOfPoints();
    vertices.resize(static_cast<std::size_t>(nPts));
    for (vtkIdType i = 0; i < nPts; ++i) {
        double p[3];
        poly->GetPoint(i, p);
        vertices[static_cast<std::size_t>(i)] = { p[0], p[1], p[2] };
    }
    vtkCellArray* polys = poly->GetPolys();
    polys->InitTraversal();
    vtkIdType npts = 0;
    const vtkIdType* ids = nullptr;
    while (polys->GetNextCell(npts, ids)) {
        if (npts != 3)
            continue;   // 只接受三角面
        triangles.push_back({ static_cast<int>(ids[0]), static_cast<int>(ids[1]),
                              static_cast<int>(ids[2]) });
    }
    if (triangles.empty()) {
        *error = QCoreApplication::tr("STL 中没有三角形面片。");
        return false;
    }
    return true;
}

// 质量统计（自研）：SICN / gamma / 二面角 / 翻转单元
void computeStats(const mesher::TetMesh3D& mesh, TetMeshQualityStats& stats)
{
    stats.quadratic = false;
    stats.tetCount = static_cast<long long>(mesh.tets.size());
    stats.nodeCount = 0;
    stats.surfaceTriCount = 0;
    stats.minSICN = 1.0;
    stats.minGamma = 1.0;
    stats.avgGamma = 0.0;
    stats.minDihedralDeg = 180.0;
    stats.maxDihedralDeg = 0.0;
    stats.invertedCount = 0;

    std::vector<char> used(mesh.points.size(), 0);
    for (const mesher::Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            used[t.v[e]] = 1;
            if (t.n[e] < 0)
                ++stats.surfaceTriCount;
        }
    }
    for (const char u : used)
        stats.nodeCount += u ? 1 : 0;

    const double edgeScale = mesher::meanEdgeLength3D(mesh);
    double gammaSum = 0.0;
    int gammaCount = 0;
    for (const mesher::Tet4& t : mesh.tets) {
        const mesher::Point3D& A = mesh.points[t.v[0]];
        const mesher::Point3D& B = mesh.points[t.v[1]];
        const mesher::Point3D& C = mesh.points[t.v[2]];
        const mesher::Point3D& D = mesh.points[t.v[3]];
        const double vol = mesher::orient3dDet(A, B, C, D) / 6.0;   // 带符号体积
        if (vol <= 0.0)
            ++stats.invertedCount;

        // SICN = 6√2·V / (l1·l2·l3)（同一顶点出发三边），取 4 顶点最小；带符号。
        // 正四面体 = 1，退化 → 0，翻转 → 负。
        double sicn = 1e30;
        for (int i = 0; i < 4; ++i) {
            const mesher::Point3D& P = mesh.points[t.v[i]];
            const double l1 = dist3(P, mesh.points[t.v[(i + 1) % 4]]);
            const double l2 = dist3(P, mesh.points[t.v[(i + 2) % 4]]);
            const double l3 = dist3(P, mesh.points[t.v[(i + 3) % 4]]);
            if (l1 > 0.0 && l2 > 0.0 && l3 > 0.0)
                sicn = std::min(sicn, 6.0 * std::sqrt(2.0) * vol / (l1 * l2 * l3));
        }
        if (sicn < 1e29)
            stats.minSICN = std::min(stats.minSICN, sicn);

        // gamma = 3·rin/rcirc（Verdict 定义，正四面体 = 1）
        const mesher::TetQuality q = mesher::tetQuality(mesh, t, edgeScale);
        stats.minDihedralDeg = std::min(stats.minDihedralDeg, q.minDihedralDeg);
        stats.maxDihedralDeg = std::max(stats.maxDihedralDeg, 180.0 - q.minDihedralDeg);
        const double lmin = std::min(std::min(dist3(A, B), dist3(B, C)), dist3(C, A));
        if (q.radiusEdge > 0.0 && vol > 0.0 && lmin > 0.0) {
            const double area = 0.5 * (dist3(A, B) * dist3(B, C) * dist3(C, A)
                                       + dist3(A, B) * dist3(A, D) * dist3(B, D)
                                       + dist3(B, C) * dist3(B, D) * dist3(C, D)
                                       + dist3(C, A) * dist3(C, D) * dist3(A, D)) * 0.5;
            if (area > 0.0) {
                const double rin = 3.0 * vol / area;               // 内切半径
                const double rcirc = q.radiusEdge * lmin;          // 外接半径
                if (rcirc > 0.0) {
                    const double gamma = 3.0 * rin / rcirc;
                    gammaSum += gamma;
                    ++gammaCount;
                    stats.minGamma = std::min(stats.minGamma, gamma);
                }
            }
        }
    }
    stats.avgGamma = gammaCount > 0 ? gammaSum / gammaCount : 0.0;
}

} // namespace

TetMeshResult TetMeshEngine::run(const TetMeshParams& params)
{
    TetMeshResult result;
    if (params.geometryFilePath.isEmpty()) {
        result.error = QCoreApplication::tr("未指定几何文件。");
        return result;
    }
    const QString suffix = QFileInfo(params.geometryFilePath).suffix().toLower();
    if (suffix != QStringLiteral("stl")) {
        result.error = QCoreApplication::tr(
            "本入口只读 STL 表面网格（当前格式：%1）。STEP/IGES/BRep 是边界表示、"
            "不携带三角面片，需先经 ShapeTessellator 离散化，再调用三角汤重载 "
            "run(params, vertices, triangles)。").arg(suffix);
        return result;
    }

    // STL 本身就是三角形汤：读进来后交给统一的重载
    std::vector<mesher::Point3D> soupVerts;
    std::vector<std::array<int, 3>> soupTris;
    QString readErr;
    if (!readStlSoup(params.geometryFilePath, soupVerts, soupTris, &readErr)) {
        result.error = readErr;
        return result;
    }
    return run(params, soupVerts, soupTris);
}

TetMeshResult TetMeshEngine::run(const TetMeshParams& params,
                                 const std::vector<mesher::Point3D>& soupVerts,
                                 const std::vector<std::array<int, 3>>& soupTris)
{
    TetMeshResult result;
    if (soupVerts.empty() || soupTris.empty()) {
        result.error = QCoreApplication::tr("三角汤为空，无法生成四面体网格。");
        return result;
    }
    result.inputPointCount = static_cast<long long>(soupVerts.size());
    result.inputTriCount = static_cast<long long>(soupTris.size());

    // ---- 1) 焊接 → 水密检查 ----
    double bbMin[3] = { 1e300, 1e300, 1e300 };
    double bbMax[3] = { -1e300, -1e300, -1e300 };
    for (const mesher::Point3D& p : soupVerts) {
        bbMin[0] = std::min(bbMin[0], p.x); bbMax[0] = std::max(bbMax[0], p.x);
        bbMin[1] = std::min(bbMin[1], p.y); bbMax[1] = std::max(bbMax[1], p.y);
        bbMin[2] = std::min(bbMin[2], p.z); bbMax[2] = std::max(bbMax[2], p.z);
    }
    const double diag = std::sqrt((bbMax[0] - bbMin[0]) * (bbMax[0] - bbMin[0])
                                  + (bbMax[1] - bbMin[1]) * (bbMax[1] - bbMin[1])
                                  + (bbMax[2] - bbMin[2]) * (bbMax[2] - bbMin[2]));
    if (diag <= 0.0) {
        result.error = QCoreApplication::tr("几何退化（包围盒为零）。");
        return result;
    }

    const mesher::WeldResult weld = mesher::weldSurfaceMesh(soupVerts, soupTris, diag * 1e-7);
    result.weldedPointCount = static_cast<long long>(weld.surface.vertices.size());
    result.weldMergedVertices = weld.mergedVertices;
    result.weldDroppedTriangles = weld.droppedTriangles;
    std::string wtErr;
    if (!mesher::isWatertight(weld.surface, &wtErr)) {
        result.error = QCoreApplication::tr(
                           "表面网格不闭合，无法生成四面体：%1\n"
                           "（焊接前 %2 点 / %3 三角形 → 焊接后 %4 点，"
                           "合并 %5 个重复点、剔除 %6 个退化三角形）")
                           .arg(QString::fromStdString(wtErr))
                           .arg(result.inputPointCount)
                           .arg(result.inputTriCount)
                           .arg(result.weldedPointCount)
                           .arg(result.weldMergedVertices)
                           .arg(result.weldDroppedTriangles);
        return result;
    }

    // ---- 2) Delaunay 四面体化（super 顶点占索引 0-3，输入点从 4 开始） ----
    mesher::TetMesh3D mesh;
    mesher::Delaunay3DResult delaunay = mesher::delaunay3D(mesh, weld.surface.vertices);
    if (!delaunay.ok) {
        result.error = QCoreApplication::tr("Delaunay 四面体化失败：%1")
                           .arg(QString::fromStdString(delaunay.error));
        return result;
    }
    std::vector<std::array<int, 3>> faces;
    faces.reserve(weld.surface.triangles.size());
    for (const auto& t : weld.surface.triangles)
        faces.push_back({ t[0] + 4, t[1] + 4, t[2] + 4 });

    // ---- 3) 约束边恢复（CDT） ----
    // 实验开关（默认关闭）：MESHSTUDIO_EDGE_PROTECTION=1 打开「阶段 1 保护球」。
    // 保护球的设计意图是在**边界恢复一开始**保护段端，因此这里也要传下去
    //（只传给 refine 阶段是不够的 —— 那时锐角端已被反复二分，尺度已塌）。
    const bool useEdgeProtection =
        qEnvironmentVariableIsSet("MESHSTUDIO_EDGE_PROTECTION")
        && qEnvironmentVariableIntValue("MESHSTUDIO_EDGE_PROTECTION") != 0;
    mesher::EdgeRecoveryParams initErp;
    initErp.edgeProtection = useEdgeProtection;

    std::string err;
    int initProtect = 0;
    mesher::Constrained3DResult constrained =
        mesher::enforceSurface3D(mesh, faces, &err, initErp, &initProtect);
    if (!constrained.ok) {
        result.error = QCoreApplication::tr("边界恢复失败：%1")
                           .arg(QString::fromStdString(constrained.error));
        return result;
    }

    // ---- 4) Ruppert 细化（大小函数；targetSize 未指定时用对角线 5%） ----
    mesher::Ruppert3DParams rp;
    rp.maxEdgeLength = params.targetSize > 0.0 ? params.targetSize : diag * 0.05;
    rp.radiusEdgeRatio = 2.0;
    // 实验开关（默认关闭，不影响原有行为）：MESHSTUDIO_EDGE_PROTECTION=1
    // 打开「阶段 1 保护球」（Shewchuk CDT 边界恢复第一阶段）。
    if (qEnvironmentVariableIsSet("MESHSTUDIO_EDGE_PROTECTION"))
        rp.edgeProtection = qEnvironmentVariableIntValue("MESHSTUDIO_EDGE_PROTECTION") != 0;
    if (qEnvironmentVariableIsSet("MESHSTUDIO_REFINE_BOUNDARY"))
        rp.refineBoundary = qEnvironmentVariableIntValue("MESHSTUDIO_REFINE_BOUNDARY") != 0;
    const mesher::Ruppert3DResult rr = mesher::ruppertRefine3D(mesh, faces, rp);
    result.refineConverged = rr.converged;
    result.refineIterations = rr.iterations;
    result.refineInserted = rr.insertedPoints;
    result.quarantined = rr.quarantined;
    result.qBySizeSeg = rr.qBySizeSeg;
    result.qBySizeMidDist = rr.qBySizeMidDist;
    result.qByEncroach = rr.qByEncroach;
    result.qByCCDist = rr.qByCCDist;
    result.qByCCOutside = rr.qByCCOutside;
    result.sliverRescueTried = rr.sliverRescueTried;
    result.protectPointsInit = initProtect;
    result.protectPoints = rr.protectPoints;
    result.protectPointsRefine = rr.protectPoints;
    result.refineWorstRadiusEdge = rr.worstRadiusEdge;
    result.refineMaxEdgeLen = rr.maxEdgeLen;
    result.refineSlivers = rr.slivers;

    // ---- 5) 平滑（optimize 开关） ----
    if (params.optimize) {
        mesher::Smooth3DParams sp;
        sp.iterations = 10;
        sp.relaxation = 0.5;
        mesher::smoothMesh3D(mesh, faces, sp);
    }

    // ---- 6) 转 VTK：活跃顶点紧凑重编号 ----
    std::unordered_map<int, vtkIdType> index;
    vtkNew<vtkPoints> points;
    for (const mesher::Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            const int v = t.v[e];
            if (index.find(v) == index.end()) {
                const mesher::Point3D& p = mesh.points[v];
                const vtkIdType id = points->InsertNextPoint(p.x, p.y, p.z);
                index.emplace(v, id);
            }
        }
    }
    vtkNew<vtkCellArray> cells;
    for (const mesher::Tet4& t : mesh.tets) {
        vtkIdType tet[4] = { index[t.v[0]], index[t.v[1]], index[t.v[2]], index[t.v[3]] };
        cells->InsertNextCell(4, tet);
    }
    auto grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
    grid->SetPoints(points);
    grid->SetCells(VTK_TETRA, cells);
    result.volume = grid;

    // 表面（边界面）用于视口显示
    vtkNew<vtkPoints> sPoints;
    std::unordered_map<int, vtkIdType> sIndex;
    const auto surfaceId = [&](int v) {
        const auto it = sIndex.find(v);
        if (it != sIndex.end())
            return it->second;
        const mesher::Point3D& p = mesh.points[v];
        const vtkIdType id = sPoints->InsertNextPoint(p.x, p.y, p.z);
        sIndex.emplace(v, id);
        return id;
    };
    vtkNew<vtkCellArray> sPolys;
    for (const mesher::Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            if (t.n[e] >= 0)
                continue;
            vtkIdType tri[3] = { surfaceId(t.v[(e + 1) % 4]), surfaceId(t.v[(e + 2) % 4]),
                                 surfaceId(t.v[(e + 3) % 4]) };
            sPolys->InsertNextCell(3, tri);
        }
    }
    auto surface = vtkSmartPointer<vtkPolyData>::New();
    surface->SetPoints(sPoints);
    surface->SetPolys(sPolys);
    result.surface = surface;

    // ---- 7) 质量统计 ----
    computeStats(mesh, result.stats);
    result.ok = true;
    return result;
}
