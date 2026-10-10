// ============================================================
// 自研四面体引擎端到端自测（M5 收尾验证，控制台程序）
// 验证：STL → 焊接/水密 → Delaunay → 约束恢复 → 细化 → 平滑 → 质量统计
// 用法：TetMeshEngineTest <input.stl> [targetSize]
// ============================================================
#include <QCoreApplication>
#include <QFileInfo>
#include <QString>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkCellData.h>
#include <vtkIntArray.h>

#include "app/meshing/TetMeshEngine.h"
#include "app/meshing/TetMeshParams.h"

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    if(args.size()==2&&args[1]==QStringLiteral("--reject-virtual-patch")) {
        mesher::SurfaceMeshResult boundary;boundary.ok=true;boundary.partitionSourceFaces={{0,1}};
        const auto result=TetMeshEngine::run(TetMeshParams{},boundary);
        vtkNew<vtkPolyData> surface;vtkNew<vtkIntArray> patch;patch->SetName("SurfacePatchId");
        surface->GetCellData()->AddArray(patch);
        const auto snapshot=TetMeshEngine::boundarySnapshot(surface);
        if(result.ok||!result.error.contains(QStringLiteral("virtual patch"))||snapshot.ok||
           snapshot.error.find("virtual patch")==std::string::npos) {
            std::printf("[FAIL] unsupported cross-face metadata must fail explicitly before volume meshing\n");return 1;
        }
        std::printf("[PASS] unsupported virtual patch volume input rejected explicitly\n");return 0;
    }

    if (args.size() < 2) {
        std::printf("用法: TetMeshEngineTest <input.stl> [targetSize]\n");
        return 2;
    }

    TetMeshParams p;
    p.geometryFilePath = args.at(1);
    if (args.size() >= 3)
        p.targetSize = args.at(2).toDouble();

    std::printf("== 端到端测试 ==\n");
    std::printf("输入: %s\n", p.geometryFilePath.toUtf8().constData());
    std::printf("目标尺寸: %.6g (%s)\n", p.targetSize,
                p.targetSize > 0.0 ? "指定" : "默认=对角线5%");
    std::printf("优化(平滑): %s\n\n", p.optimize ? "开" : "关");

    // 计时
    const auto t0 = std::chrono::steady_clock::now();
    const TetMeshResult r = TetMeshEngine::run(p);
    const auto t1 = std::chrono::steady_clock::now();
    const double sec = std::chrono::duration<double>(t1 - t0).count();

    if (!r.ok) {
        std::printf("[FAIL] 生成失败: %s\n", r.error.toUtf8().constData());
        return 1;
    }

    const TetMeshQualityStats& s = r.stats;
    std::printf("[PASS] 端到端链路跑通\n\n");
    std::printf("耗时            : %.3f s\n", sec);
    std::printf("节点            : %lld\n", s.nodeCount);
    std::printf("体单元          : %lld\n", s.tetCount);
    std::printf("表面三角形      : %lld\n", s.surfaceTriCount);
    std::printf("单元阶次        : %s\n", s.quadratic ? "T10" : "T4");
    std::printf("\n-- 质量 --\n");
    std::printf("scaled Jacobian : %.6f  %s\n", s.minScaledJacobian,
                s.minScaledJacobian < 0.0 ? "<< 存在翻转单元!" : "ok");
    std::printf("qualityAccepted : %s\n", r.qualityAccepted ? "yes" : "no");
    for (const auto& warning : r.warnings)
        std::printf("[WARNING] %s\n", warning.toUtf8().constData());
    std::printf("gamma 最小/平均 : %.6f / %.6f\n", s.minGamma, s.avgGamma);
    std::printf("二面角范围      : [%.2f°, %.2f°]\n", s.minDihedralDeg, s.maxDihedralDeg);
    std::printf("翻转单元数      : %lld\n", s.invertedCount);

    // ---- 细化诊断（调优用） ----
    std::printf("\n-- 细化诊断 --\n");
    std::printf("converged       : %s\n", r.refineConverged ? "yes" : "no");
    std::printf("iterations      : %d\n", r.refineIterations);
    std::printf("insertedPoints  : %d\n", r.refineInserted);
    std::printf("protectPoints   : %d  (init=%d refine=%d)\n",
                r.protectPoints, r.protectPointsInit, r.protectPointsRefine);
    std::printf("worstRadiusEdge : %.6f\n", r.refineWorstRadiusEdge);
    std::printf("maxEdgeLen      : %.6f\n", r.refineMaxEdgeLen);
    std::printf("slivers(<5deg)  : %d\n", r.refineSlivers);
    std::printf("quarantined     : %d  (bySizeSeg=%d bySizeMidDist=%d byEncroach=%d"
                " byCCDist=%d byCCOutside=%d)\n",
                r.quarantined, r.qBySizeSeg, r.qBySizeMidDist, r.qByEncroach,
                r.qByCCDist, r.qByCCOutside);
    std::printf("sliverRescueTried: %d\n", r.sliverRescueTried);

    // 检查项
    int fails = 0;
    auto check = [&](bool ok, const char* name) {
        std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
        if (!ok)
            ++fails;
    };
    std::printf("\n-- 检查 --\n");
    check(r.volume != nullptr && s.tetCount > 0, "生成了体网格");
    check(r.surface != nullptr && s.surfaceTriCount > 0, "生成了外表面");
    check(s.invertedCount == 0, "无翻转单元");
    check(s.nodeCount > 0, "节点数 > 0");
    check(!r.refineConverged || (r.refineWorstRadiusEdge <= 2.0 * (1.0 + 1e-8)
        && r.refineUnresolvedTets == 0), "细化收敛不得隐藏未达标单元");
    check(r.qualityAccepted || !r.warnings.empty(), "未验收结果有明确警告");
    check(s.minSICN == s.minScaledJacobian, "旧存档兼容字段与实际指标一致");
    if (QFileInfo(p.geometryFilePath).fileName() == QStringLiteral("cube10.stl") && p.targetSize<10)
        check(!r.qualityAccepted, "cube10 已知未达标网格不能报告质量验收通过");

    // 对固定立方体输入验证实体体积、边界面积、域范围及朝向；剖分数量可合法改变。
    const QString base=QFileInfo(p.geometryFilePath).fileName();
    if(base==QStringLiteral("cube.stl") || base==QStringLiteral("cube10.stl")) {
        const double side=base==QStringLiteral("cube.stl")?1.0:10.0;
        double bounds[6];r.volume->GetBounds(bounds);
        bool domainValid=true,cellsValid=true;
        double totalVolume=0,areas[6]={};
        vtkNew<vtkIdList> ids;
        for(vtkIdType t=0;t<r.volume->GetNumberOfCells();++t) {
            r.volume->GetCellPoints(t,ids);
            if(ids->GetNumberOfIds()!=4) {cellsValid=false;continue;}
            double q[4][3],d[3][3];
            for(int k=0;k<4;++k) {
                r.volume->GetPoint(ids->GetId(k),q[k]);
                for(int a=0;a<3;++a) domainValid=domainValid && q[k][a]>=bounds[2*a]-1e-9 && q[k][a]<=bounds[2*a]+side+1e-9;
            }
            for(int k=0;k<3;++k) for(int a=0;a<3;++a) d[k][a]=q[k+1][a]-q[0][a];
            const double det=d[0][0]*(d[1][1]*d[2][2]-d[1][2]*d[2][1])-d[0][1]*(d[1][0]*d[2][2]-d[1][2]*d[2][0])+d[0][2]*(d[1][0]*d[2][1]-d[1][1]*d[2][0]);
            cellsValid=cellsValid && det>0;totalVolume+=det/6;
        }
        bool boundaryValid=true;
        for(vtkIdType t=0;t<r.surface->GetNumberOfCells();++t) {
            r.surface->GetCellPoints(t,ids);double q[3][3],u[3],v[3],n[3];
            for(int k=0;k<3;++k) r.surface->GetPoint(ids->GetId(k),q[k]);
            for(int a=0;a<3;++a) {u[a]=q[1][a]-q[0][a];v[a]=q[2][a]-q[0][a];}
            for(int a=0;a<3;++a) n[a]=u[(a+1)%3]*v[(a+2)%3]-u[(a+2)%3]*v[(a+1)%3];
            int plane=-1;
            for(int a=0;a<3;++a) for(int b=0;b<2;++b) {
                bool on=true;for(int k=0;k<3;++k) on=on && std::abs(q[k][a]-(bounds[2*a]+b*side))<1e-9;
                if(on) plane=2*a+b;
            }
            if(plane<0) {boundaryValid=false;continue;}
            boundaryValid=boundaryValid && n[plane/2]*(plane%2?1:-1)>0;
            areas[plane]+=std::hypot(n[0],n[1],n[2])/2;
        }
        for(double area:areas) boundaryValid=boundaryValid && std::abs(area-side*side)<side*side*1e-9;
        check(cellsValid && domainValid,"cube: positive tetrahedra remain in prescribed domain");
        check(std::abs(totalVolume-side*side*side)<side*side*side*1e-9,"cube: independent determinant volume equals solid volume");
        check(boundaryValid,"cube: six prescribed planes fully covered with outward normals");
        check(s.minGamma>0 && s.minGamma<=1+1e-10 && s.avgGamma<=1+1e-10 && s.degenerateCount==0,"cube: true gamma is bounded and no degenerate cells");
    }

    std::printf("\n===== 结果: %s (failures=%d) =====\n",
                fails == 0 ? "全部通过" : "有失败", fails);
    return fails == 0 ? 0 : 1;
}
