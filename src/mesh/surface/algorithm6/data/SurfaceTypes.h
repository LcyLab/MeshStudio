#pragma once
/** @file SurfaceTypes.h
 * @brief 独立复现路线的数据合同；不导入 SurfaceCriteria 产品细化默认。
 * @date 2026-10-07
 * @modified 2026-10-10
 */
#include "geometry/types/MesherTypes.h"
#include <cstddef>
#include <functional>
#include <limits>
#include <string>

namespace mesher::gmshReference {
enum class MeshingStatus {
    Complete, Cancelled, BudgetExceeded, InvalidInput,
    GeometryQueryFailed, InvalidTopology, NonConverged
};

struct OperationStatus {
    MeshingStatus code = MeshingStatus::Complete;
    std::string message;
    /// 判断操作状态是否为完整成功。
    bool complete() const { return code == MeshingStatus::Complete; }
};

// 查询回调跨模块提前退出时携带明确状态；普通 CAD 异常仍报 GeometryQueryFailed。
// 不能由 catch(...) 将取消/预算吞成未知几何错误。算法入口捕获并返回 status。
struct OperationFailure { OperationStatus status; };

// 对应 4.15.2 DefaultOptions 的算法 6、一阶、串行基线。
// sizeMin/sizeMax 在倍率之前限制尺寸值，不是最终网格边长约束。
struct SurfaceMeshingOptions {
    double sizeMin = 0.;
    double sizeMax = 1.e22;
    double sizeFactor = 1.;
    bool sizeFromPoints = true;
    double elementsPerCurvatureCircle = 0.;
    int extendBoundarySize = 1;
    double curveIntegrationPrecision = 1.e-9;
    double curveSmoothRatio = 1.8;
    int minimumLinePoints = 2;
    int minimumCurvePoints = 3;
    int minimumCirclePoints = 7;
    bool oldInitialDelaunay = false;
    double initialRandomFactor = 1.e-9; // 周期初始UV扰动；只改变剖分坐标，不覆盖原始UV。
    int smoothingIterations = 1;
    int maximumRecoveryRetries = 10;
    int adaptationIterations = 10;
    bool switchOnFailure = true;
};

// 两个状态均为物理长度；面内插点只插值 meshSize，backgroundSize 独立查询。
struct NodeSize {
    double meshSize = 0.;
    double backgroundSize = 0.;
};

// 限制不是收敛条件；算法模块自行检查并显式返回取消或预算状态。
struct RunControl {
    std::function<bool()> keepGoing;
    std::size_t maximumGeometryEvaluations = std::numeric_limits<std::size_t>::max();
    std::size_t maximumNodes = std::numeric_limits<std::size_t>::max();
};
} // namespace mesher::gmshReference
