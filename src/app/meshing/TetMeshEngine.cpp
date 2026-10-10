// 自研体网格链路：索引表面（或 STL 焊接）→ 体边界检查 → Delaunay → 边/面恢复
// → 屏障奇偶域提取 → 细化/受限平滑 → 最终边界、体积和质量验收 → VTK。
// CAD 的 SurfaceMeshEngine 由调用方运行，本文件不依赖 OCC。
// 面恢复能力受限，无法恢复时明确失败；可显示结果与质量验收通过分别表示。
#include "TetMeshEngine.h"

#include <QCoreApplication>
#include <QFileInfo>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <vector>

#include <vtkCellArray.h>
#include <vtkCellType.h>
#include <vtkNew.h>
#include <vtkCellData.h>
#include <vtkIntArray.h>
#include <vtkDoubleArray.h>
#include <vtkDataArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "mesh/volume/Constrained3D.h"
#include "mesh/volume/Delaunay3D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/volume/Quality3D.h"
#include "mesh/volume/Ruppert3D.h"
#include "mesh/volume/Smooth3D.h"
#include "mesh/surface/legacy/SurfaceMesh3D.h"
#include "mesh/validation/VolumeBoundaryValidation.h"
#include "mesh/volume/VolumeDomain3D.h"

namespace {

// 在原三角面片内插值；这是离散表面 UV，不声称新增点位于精确 CAD 曲面。
bool cornerWeights(const mesher::Point3D& p,const std::array<int,3>& f,
                   const std::vector<mesher::Point3D>& points,double (&w)[3]) {
    const auto a=points[f[0]],b=points[f[1]],c=points[f[2]];
    const double u[3]={b.x-a.x,b.y-a.y,b.z-a.z},v[3]={c.x-a.x,c.y-a.y,c.z-a.z};
    const double n[3]={u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
    int axis=0;for(int i=1;i<3;++i) if(std::abs(n[i])>std::abs(n[axis])) axis=i;
    const int x=(axis+1)%3,y=(axis+2)%3;
    const double q[3]={p.x-a.x,p.y-a.y,p.z-a.z};
    const double det=u[x]*v[y]-u[y]*v[x];if(det==0) return false;
    w[1]=(q[x]*v[y]-q[y]*v[x])/det;w[2]=(u[x]*q[y]-u[y]*q[x])/det;w[0]=1-w[1]-w[2];
    return w[0]>=-1e-9 && w[1]>=-1e-9 && w[2]>=-1e-9;
}

/// 判断点是否落在指定线段范围内。
bool onSegment(const mesher::Point3D& p,const mesher::Point3D& a,const mesher::Point3D& b) {
    const double d[3]={b.x-a.x,b.y-a.y,b.z-a.z},q[3]={p.x-a.x,p.y-a.y,p.z-a.z};
    const double scale=std::max({std::abs(d[0]),std::abs(d[1]),std::abs(d[2])});
    if(!(scale>0)) return false;
    double length=0,dot=0;for(int i=0;i<3;++i) {length+=(d[i]/scale)*(d[i]/scale);dot+=(q[i]/scale)*(d[i]/scale);}
    const double t=dot/length;
    const double magnitude=std::max({scale,std::abs(a.x),std::abs(a.y),std::abs(a.z),std::abs(b.x),std::abs(b.y),std::abs(b.z)});
    const double tolerance=256*std::numeric_limits<double>::epsilon()*magnitude;
    if(t < -tolerance/scale || t > 1+tolerance/scale) return false;
    for(int i=0;i<3;++i) if(std::abs(q[i]-t*d[i])>tolerance) return false;
    return true;
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

// 质量统计统一由 Quality3D 计算，避免引擎与细化采用不同度量。
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

    const auto quality = mesher::summarizeQuality(mesh, mesher::meanEdgeLength3D(mesh));
    stats.minGamma = quality.minGamma;
    stats.avgGamma = quality.avgGamma;
    stats.minScaledJacobian = quality.minScaledJacobian;
    stats.minSICN = stats.minScaledJacobian; // 旧存档字段兼容
    stats.minDihedralDeg = quality.minDihedralDeg;
    stats.maxDihedralDeg = quality.maxDihedralDeg;
    stats.invertedCount = quality.invertedCount;
    stats.degenerateCount = quality.degenerateCount;
    stats.sliverCount = quality.sliverCount;
    stats.maxEdgeLength = quality.maxEdgeLength;
    stats.worstRadiusEdge = quality.worstRadiusEdge;
}

} // namespace

/// 读取 STL 三角表面并生成四面体网格，其他文件格式明确报错。
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
            "不携带三角面片，需先经 SurfaceMeshEngine 生成自研表面，再调用索引表面重载 "
            "run(params, boundary)。").arg(suffix);
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

namespace {
/// 校验输入边界，构造约束四面体网格并汇总结果与质量统计。
TetMeshResult runBoundary(const TetMeshParams& params,
                                 const std::vector<mesher::Point3D>& soupVerts,
                                 const std::vector<std::array<int, 3>>& soupTris,
                                 const mesher::SurfaceMeshResult* indexedBoundary)
{
    TetMeshResult result;
    if (soupVerts.empty() || soupTris.empty()) {
        result.error = QCoreApplication::tr("三角汤为空，无法生成四面体网格。");
        return result;
    }
    result.inputPointCount = static_cast<long long>(soupVerts.size());
    result.inputTriCount = static_cast<long long>(soupTris.size());
    if(!std::isfinite(params.targetSize) || params.targetSize<0 || soupVerts.size()>=(1u<<21)-4) {
        result.error=QStringLiteral("Invalid target size or boundary exceeds face index capacity");return result;
    }
    for(const auto& p:soupVerts) if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) {
        result.error=QStringLiteral("Non-finite input boundary vertex");return result;
    }
    for(const auto& f:soupTris) for(int v:f) if(v<0 || v>=int(soupVerts.size())) {
        result.error=QStringLiteral("Invalid input boundary triangle index");return result;
    }

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
    if (!(diag > 0.0) || !std::isfinite(diag)) {
        result.error = QCoreApplication::tr("几何退化（包围盒为零）。");
        return result;
    }

    mesher::WeldResult weld;
    if(indexedBoundary) weld.surface=indexedBoundary->mesh;
    else weld=mesher::weldSurfaceMesh(soupVerts,soupTris,diag*1e-7);
    result.weldedPointCount = static_cast<long long>(weld.surface.vertices.size());
    result.weldMergedVertices = weld.mergedVertices;
    result.weldDroppedTriangles = weld.droppedTriangles;
    std::string wtErr;
    const auto validated=mesher::validateVolumeBoundary(weld.surface);
    wtErr=validated.error;
    if (weld.droppedTriangles>0 || !validated.ok) {
        if(weld.droppedTriangles>0) wtErr="input triangles collapsed during welding; boundary cannot be accepted";
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

    // Delaunay 会精确去重，体约束却引用原索引；索引入口不能静默失去节点身份。
    std::set<std::array<double,3>> uniquePoints;
    for(const auto& p:weld.surface.vertices) if(!uniquePoints.insert({p.x,p.y,p.z}).second) {
        result.error=QStringLiteral("Coincident indexed boundary vertices: distinct node identities cannot be merged for volume meshing");return result;
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
    const auto originalFaces=faces;

    // ---- 3) 约束边恢复及实际面覆盖 ----
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

    const auto domain=mesher::extractVolumeDomain(mesh,originalFaces);
    if(!domain.ok) {
        result.error=QStringLiteral("Volume domain classification failed: ")+QString::fromStdString(domain.error);return result;
    }
    const double expectedVolume=domain.volume;
    // 原输入片可能换过共面对角线；细化屏障必须使用真实网格边界面。
    faces.clear();
    for(const auto& t:mesh.tets) for(int e=0;e<4;++e) if(t.n[e]<0) {
        std::array<int,3> f={t.v[(e+1)%4],t.v[(e+2)%4],t.v[(e+3)%4]};
        if(mesher::orient3d(mesh.points[f[0]],mesh.points[f[1]],mesh.points[f[2]],mesh.points[t.v[e]])>0) std::swap(f[1],f[2]);
        faces.push_back(f);
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
    result.refineStatus = rr.status;
    result.refineDiagnostic = QString::fromStdString(rr.diagnostic);
    result.refineUnresolvedTets = rr.unresolvedTets;
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
    if (rr.status == mesher::Ruppert3DStatus::Failed || !rr.error.empty()) {
        result.error = QCoreApplication::tr("四面体细化失败：%1").arg(QString::fromStdString(rr.error));
        return result;
    }
    if (!rr.converged)
        result.warnings.append(QCoreApplication::tr("细化未收敛：%1")
                                   .arg(QString::fromStdString(rr.diagnostic)));

    // ---- 5) 平滑（optimize 开关） ----
    if (params.optimize) {
        mesher::Smooth3DParams sp;
        sp.iterations = 10;
        sp.relaxation = 0.5;
        const auto smoothing = mesher::smoothMesh3D(mesh, faces, sp);
        result.smoothSucceeded = smoothing.ok;
        if (!smoothing.ok) {
            result.error = QCoreApplication::tr("四面体平滑失败：%1")
                               .arg(QString::fromStdString(smoothing.error));
            return result;
        }
    }

    // 平滑可改变尺寸/质量，验收必须在全部阶段之后重新测量。
    computeStats(mesh, result.stats);
    const auto& quality = result.stats;
    const bool sizeAccepted = quality.maxEdgeLength <= rp.maxEdgeLength * (1.0 + 1e-8);
    const bool radiusAccepted = std::isfinite(quality.worstRadiusEdge)
        && quality.worstRadiusEdge <= rp.radiusEdgeRatio * (1.0 + 1e-8);
    result.qualityAccepted = rr.converged && sizeAccepted && radiusAccepted
        && quality.degenerateCount == 0 && quality.sliverCount == 0
        && quality.invertedCount == 0 && quality.minGamma > 0.0;
    result.stats.acceptanceKnown = true;
    result.stats.qualityAccepted = result.qualityAccepted;
    if (!result.qualityAccepted)
        result.warnings.append(QCoreApplication::tr(
            "质量未验收：最大边长 %1（目标 %2），最坏半径边比 %3（目标 %4），"
            "退化单元 %5，内二面角小于5°单元 %6。")
            .arg(quality.maxEdgeLength).arg(rp.maxEdgeLength).arg(quality.worstRadiusEdge)
            .arg(rp.radiusEdgeRatio).arg(quality.degenerateCount).arg(quality.sliverCount));

    // 细化/平滑不能丢失输入边界或改变实体体积，验收失败不提供体网格。
    const auto finalDomain=mesher::extractVolumeDomain(mesh,originalFaces);
    if(!finalDomain.ok || finalDomain.removedTets>0 ||
       std::abs(mesh.totalVolume()-expectedVolume)>expectedVolume*1e-8) {
        result.error=QStringLiteral("Final boundary/volume acceptance failed: ")+QString::fromStdString(finalDomain.error);
        return result;
    }
    const auto boundaryCoverage=mesher::measureVolumeBoundaryCoverage(mesh,originalFaces);
    // ---- 6) 转 VTK：活跃顶点紧凑重编号 ----
    std::unordered_map<int, vtkIdType> index;
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
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

    // 表面（边界面）用于视口显示
    vtkNew<vtkPoints> sPoints;
    sPoints->SetDataTypeToDouble();
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
    vtkNew<vtkIntArray> cadFaceIds;cadFaceIds->SetName("CadFaceId");
    vtkNew<vtkIntArray> cadEdgeIds;cadEdgeIds->SetName("CadEdgeId");cadEdgeIds->SetNumberOfComponents(3);
    vtkNew<vtkDoubleArray> cornerUV;cornerUV->SetName("CadCornerUV");cornerUV->SetNumberOfComponents(6);
    vtkNew<vtkIntArray> bodyIds;bodyIds->SetName("CadBodyId");
    std::unordered_map<uint64_t,int> cadEdges;
    const auto edgeKey=[](int a,int b) {if(a>b) std::swap(a,b);return (uint64_t(a)<<32)|uint64_t(b);};
    if(indexedBoundary) for(const auto& e:indexedBoundary->boundaryEdges) cadEdges[edgeKey(e.a+4,e.b+4)]=e.cadEdge;
    for (const mesher::Tet4& t : mesh.tets) {
        for (int e = 0; e < 4; ++e) {
            if (t.n[e] >= 0)
                continue;
            std::array<int,3> f={t.v[(e+1)%4],t.v[(e+2)%4],t.v[(e+3)%4]};
            if(mesher::orient3d(mesh.points[f[0]],mesh.points[f[1]],mesh.points[f[2]],mesh.points[t.v[e]])>0) std::swap(f[1],f[2]);
            vtkIdType tri[3]={surfaceId(f[0]),surfaceId(f[1]),surfaceId(f[2])};
            sPolys->InsertNextCell(3, tri);
            if(indexedBoundary && !indexedBoundary->triangleFaceIds.empty()) {
                auto sorted=f;std::sort(sorted.begin(),sorted.end());
                const uint64_t key=(uint64_t(sorted[0])<<42)|(uint64_t(sorted[1])<<21)|uint64_t(sorted[2]);
                const auto& sources=boundaryCoverage.sourceFaces.at(key);
                int label=indexedBoundary->triangleFaceIds[sources.front()];
                int body=indexedBoundary->triangleBodyIds.empty()?-1:indexedBoundary->triangleBodyIds[sources.front()];
                for(int source:sources) {
                    const int sourceLabel=indexedBoundary->triangleFaceIds[source];
                    if(label!=sourceLabel || (!indexedBoundary->triangleBodyIds.empty() && body!=indexedBoundary->triangleBodyIds[source])) {
                        result.error=QStringLiteral("Recovered boundary triangle crosses different CAD face labels");return result;
                    }
                }
                cadFaceIds->InsertNextValue(label);
                if(!indexedBoundary->triangleBodyIds.empty()) bodyIds->InsertNextValue(body);
                int labels[3]={-1,-1,-1};
                for(int k=0;k<3;++k) for(int source:sources) {
                    const auto& sf=originalFaces[source];
                    for(int j=0;j<3;++j) {
                        const auto it=cadEdges.find(edgeKey(sf[j],sf[(j+1)%3]));
                        if(it==cadEdges.end()) continue;
                        if(onSegment(mesh.points[f[k]],mesh.points[sf[j]],mesh.points[sf[(j+1)%3]]) &&
                           onSegment(mesh.points[f[(k+1)%3]],mesh.points[sf[j]],mesh.points[sf[(j+1)%3]])) {
                            if(labels[k]>=0 && labels[k]!=it->second) {result.error=QStringLiteral("Conflicting CAD segment labels");return result;}
                            labels[k]=it->second;
                        }
                    }
                }
                cadEdgeIds->InsertNextTuple3(labels[0],labels[1],labels[2]);
                if(!indexedBoundary->triangleCornerUV.empty()) {
                    double uv[6];
                    for(int k=0;k<3;++k) {
                        bool found=false;
                        for(int source:sources) {
                            double w[3];if(!cornerWeights(mesh.points[f[k]],originalFaces[source],mesh.points,w)) continue;
                            uv[2*k]=uv[2*k+1]=0;
                            for(int j=0;j<3;++j) {const auto p=indexedBoundary->triangleCornerUV[source][j];uv[2*k]+=w[j]*p.x;uv[2*k+1]+=w[j]*p.y;}
                            found=true;break;
                        }
                        if(!found) {result.error=QStringLiteral("Cannot interpolate recovered boundary corner UV");return result;}
                    }
                    cornerUV->InsertNextTuple(uv);
                }
            }
        }
    }
    auto surface = vtkSmartPointer<vtkPolyData>::New();
    surface->SetPoints(sPoints);
    surface->SetPolys(sPolys);
    if(cadFaceIds->GetNumberOfTuples()>0) {
        surface->GetCellData()->AddArray(cadFaceIds);surface->GetCellData()->AddArray(cadEdgeIds);
        if(cornerUV->GetNumberOfTuples()>0) surface->GetCellData()->AddArray(cornerUV);
        if(bodyIds->GetNumberOfTuples()>0) surface->GetCellData()->AddArray(bodyIds);
    }
    result.surface = surface;
    result.volume = grid;

    // ---- 7) 质量统计 ----
    result.ok = true;
    return result;
}
} // namespace

/// 焊接三角形汤的重复节点，再从闭合边界生成四面体网格。
TetMeshResult TetMeshEngine::run(const TetMeshParams& params,
    const std::vector<mesher::Point3D>& vertices,const std::vector<std::array<int,3>>& triangles) {
    return runBoundary(params,vertices,triangles,nullptr);
}

/// 从保留节点身份和 CAD 标签的索引边界生成四面体网格。
TetMeshResult TetMeshEngine::run(const TetMeshParams& params,const mesher::SurfaceMeshResult& boundary) {
    if(std::any_of(boundary.partitionSourceFaces.begin(),boundary.partitionSourceFaces.end(),[](const auto& faces){return faces.size()>1;})) {
        TetMeshResult failed;failed.error=QStringLiteral("Volume meshing does not yet support virtual patch CAD provenance");return failed;
    }
    auto invalid=[&]() {TetMeshResult failed;failed.error=QStringLiteral("Invalid structured surface boundary or CAD metadata");return failed;};
    const auto count=boundary.mesh.triangles.size();
    if(!boundary.ok || (!boundary.boundaryEdges.empty() && boundary.triangleFaceIds.empty()) ||
       (!boundary.triangleFaceIds.empty() && boundary.triangleFaceIds.size()!=count) ||
       (!boundary.triangleCornerUV.empty() && (boundary.triangleCornerUV.size()!=count || boundary.triangleFaceIds.empty())) ||
       (!boundary.triangleBodyIds.empty() && (boundary.triangleBodyIds.size()!=count || boundary.triangleFaceIds.empty()))) {
        TetMeshResult failed;failed.error=QStringLiteral("Invalid structured surface boundary or CAD labels");return failed;
    }
    for(int id:boundary.triangleFaceIds) if(id < -1) return invalid();
    for(int id:boundary.triangleBodyIds) if(id < -1) return invalid();
    for(const auto& uv:boundary.triangleCornerUV) for(auto p:uv) if(!std::isfinite(p.x)||!std::isfinite(p.y)) return invalid();
    std::unordered_map<uint64_t,int> edgeLabels;
    for(const auto& e:boundary.boundaryEdges) {
        if(e.a<0 || e.b<0 || e.a==e.b || e.a>=int(boundary.mesh.vertices.size()) || e.b>=int(boundary.mesh.vertices.size()) || e.cadEdge<0) return invalid();
        const auto key=(uint64_t(std::min(e.a,e.b))<<32)|uint64_t(std::max(e.a,e.b));
        auto inserted=edgeLabels.emplace(key,e.cadEdge);
        if(!inserted.second && inserted.first->second!=e.cadEdge) return invalid();
    }
    return runBoundary(params,boundary.mesh.vertices,boundary.mesh.triangles,&boundary);
}

/// 将已索引的 VTK 表面复制为纯数据快照，拒绝非三角形/错误标签；不焊接。
mesher::SurfaceMeshResult TetMeshEngine::boundarySnapshot(vtkPolyData* surface) {
    mesher::SurfaceMeshResult out;
    if(surface&&surface->GetCellData()->GetArray("SurfacePatchId")) {
        out.error="volume boundary conversion does not yet support virtual patch CAD provenance";return out;
    }
    if(!surface || surface->GetNumberOfPoints()>=(1u<<21)-4 || !surface->GetPolys() ||
       surface->GetNumberOfCells()!=surface->GetNumberOfPolys()) {out.error="invalid triangle-only volume boundary";return out;}
    for(vtkIdType i=0;i<surface->GetNumberOfPoints();++i) {
        double p[3];surface->GetPoint(i,p);out.mesh.vertices.push_back({p[0],p[1],p[2]});
    }
    auto* faces=surface->GetCellData()->GetArray("CadFaceId");
    auto* edges=surface->GetCellData()->GetArray("CadEdgeId");
    auto* uv=surface->GetCellData()->GetArray("CadCornerUV");
    auto* bodies=surface->GetCellData()->GetArray("CadBodyId");
    for(auto array:{faces,edges,uv,bodies}) if(array && array->GetNumberOfTuples()!=surface->GetNumberOfPolys()) {
        out.error="volume boundary CAD metadata count mismatch";return out;
    }
    if((faces && faces->GetNumberOfComponents()!=1) || (edges && edges->GetNumberOfComponents()!=3) ||
       (uv && uv->GetNumberOfComponents()!=6) || (bodies && bodies->GetNumberOfComponents()!=1)) {out.error="volume boundary CAD metadata components mismatch";return out;}
    if(!faces && (edges || uv || bodies)) {out.error="CAD metadata requires face labels";return out;}
    for(auto array:{faces,edges,bodies}) if(array) for(vtkIdType i=0;i<array->GetNumberOfTuples();++i) for(int k=0;k<array->GetNumberOfComponents();++k) {
        const double id=array->GetComponent(i,k);
        if(!std::isfinite(id) || id < -1 || id>std::numeric_limits<int>::max() || std::floor(id)!=id) {out.error="invalid CAD integer label";return out;}
    }
    if(uv) for(vtkIdType i=0;i<uv->GetNumberOfTuples();++i) for(int k=0;k<6;++k) if(!std::isfinite(uv->GetComponent(i,k))) {out.error="nonfinite CAD corner UV";return out;}
    auto* polys=surface->GetPolys();polys->InitTraversal();vtkIdType n=0;const vtkIdType* ids=nullptr;
    vtkIdType t=0;
    while(polys->GetNextCell(n,ids)) {
        if(n!=3) {out.error="volume boundary contains a non-triangle cell";return out;}
        for(int k=0;k<3;++k) if(ids[k]<0 || ids[k]>=surface->GetNumberOfPoints()) {out.error="invalid boundary cell index";return out;}
        out.mesh.triangles.push_back({int(ids[0]),int(ids[1]),int(ids[2])});
        if(faces) out.triangleFaceIds.push_back(int(faces->GetComponent(t,0)));
        if(bodies) out.triangleBodyIds.push_back(int(bodies->GetComponent(t,0)));
        if(edges) for(int k=0;k<3;++k) {
            const int edge=int(edges->GetComponent(t,k));
            if(edge>=0) out.boundaryEdges.push_back({int(ids[k]),int(ids[(k+1)%3]),edge});
        }
        if(uv) out.triangleCornerUV.push_back({mesher::Point2D{uv->GetComponent(t,0),uv->GetComponent(t,1)},
            mesher::Point2D{uv->GetComponent(t,2),uv->GetComponent(t,3)},mesher::Point2D{uv->GetComponent(t,4),uv->GetComponent(t,5)}});
        ++t;
    }
    out.ok=true;return out;
}
