#pragma once

#include <array>
#include <vector>

#include <QString>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>

#include "TetMeshParams.h"
#include "mesher/MesherTypes.h"

/// 四面体网格质量统计（度量依据见 docs/tet-mesh-design.md §6）
struct TetMeshQualityStats
{
    long long nodeCount = 0;        ///< 网格节点总数
    long long tetCount = 0;         ///< 体单元数（T4 或 T10，随阶次）
    long long surfaceTriCount = 0;  ///< 外表面三角形数
    bool quadratic = false;         ///< 是否二阶单元（T10）

    double minSICN = 1.0;           ///< 有符号逆条件数最小值（<0 即翻转单元）
    double minGamma = 1.0;          ///< gamma=3·内切半径/外接半径 最小值（正四面体=1）
    double avgGamma = 0.0;          ///< gamma 平均值
    double minDihedralDeg = 180.0;  ///< 全网格最小二面角（自算，sliver 复核）
    double maxDihedralDeg = 0.0;    ///< 全网格最大二面角
    long long invertedCount = 0;    ///< 翻转单元数（minSICN < 0）
};

/// 四面体生成结果：体网格用于后续导出/诊断，外表面对象直接供视口显示
struct TetMeshResult
{
    bool ok = false;
    QString error;
    TetMeshQualityStats stats;
    vtkSmartPointer<vtkUnstructuredGrid> volume;
    vtkSmartPointer<vtkPolyData> surface;
    // ---- 输入诊断（B-Rep 离散输入质量排查用，UI 只记日志） ----
    long long inputPointCount = 0;   ///< 焊接前顶点数（含逐面离散产生的重复点）
    long long inputTriCount = 0;     ///< 焊接前三角形数
    long long weldedPointCount = 0;  ///< 焊接后顶点数
    int weldMergedVertices = 0;      ///< 被合并掉的重复顶点数
    int weldDroppedTriangles = 0;    ///< 焊接后退化被剔除的三角形数
    // ---- 细化诊断（M3 调优用，UI 不消费） ----
    bool refineConverged = false;
    int refineIterations = 0;
    int refineInserted = 0;
    int quarantined = 0;
    int qBySizeSeg = 0;
    int qBySizeMidDist = 0;
    int qByEncroach = 0;
    int qByCCDist = 0;
    int qByCCOutside = 0;
    int sliverRescueTried = 0;
    /// 阶段 1 保护球插入点数：初始终（enforceSurface3D）/ refine 阶段 分开计
    int protectPoints = 0;
    int protectPointsInit = 0;
    int protectPointsRefine = 0;
    double refineWorstRadiusEdge = 0.0;
    double refineMaxEdgeLen = 0.0;
    int refineSlivers = 0;
};

/// 自研四面体网格引擎（M5 集成，取代 Gmsh）。
/// 链路：表面三角形汤 → 焊接/水密检查 → Delaunay 四面体化 → 约束边恢复
///       → Ruppert 细化 → Laplacian 平滑 → 质量统计（自研 gamma/SICN/二面角）。
/// run() 在后台线程执行（纯计算、无全局状态）。
///
/// ★ 输入来源无关 ★
/// 引擎只认「三角形汤」（顶点 + 三角形索引）。STL 天然就是三角形汤；
/// STEP / IGES / BRep 是边界表示、**不带三角面片**，需先经
/// ShapeTessellator / ShapeToPolyData 离散化后再走本重载。因此引擎本身
/// 不依赖 OCC，可被只链接 VTK 的测试目标复用。
namespace TetMeshEngine {

/// 从 params.geometryFilePath 读 STL 并生成网格（STL 快捷入口）。
/// 非 STL 后缀会返回明确的错误，提示改用三角汤重载。
TetMeshResult run(const TetMeshParams& params);

/// 直接以「三角形汤」为输入生成网格（跳过文件读取）。
/// soupVertices 允许包含重复顶点（逐面离散的 B-Rep 输出就是如此），
/// 内部会按 diag·1e-7 的容差焊接。
TetMeshResult run(const TetMeshParams& params,
                  const std::vector<mesher::Point3D>& soupVertices,
                  const std::vector<std::array<int, 3>>& soupTriangles);

} // namespace TetMeshEngine
