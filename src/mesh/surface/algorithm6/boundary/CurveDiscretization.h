#pragma once
/** @file CurveDiscretization.h
 * @brief 对应 meshGEdge.cpp 的积分、原生尺寸平滑、计数和参数分布。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"
#include <cstdint>
#include <vector>

namespace mesher::gmshReference {
enum class CurveKind { Line, Circle, Ellipse, Other };
struct CurveDensitySample {
    double parameter = 0.;
    double density = 0.;           // |C'(t)| / h(t)，无量纲密度/参数。
    double cumulativeDensity = 0.;
    double speed = 0.;             // |C'(t)|，物理长度/参数。
};

struct CurveDensityTable {
    std::vector<CurveDensitySample> samples;
    std::size_t densityEvaluations = 0;
    bool nativeIntegrationDepthLimitReached = false;
    int smoothingIterations = 0;
    bool nativeSmoothingLimitReached = false;
};

/** Integration/RecursiveIntegration：递归梯形绝对误差、depth>6、depth>25。
 * 参数必须递增且有限；derivative/sizeAt 本次运行不可变。
 * 成功写入完整累计表，失败/取消/预算耗尽保留 output。不创建 CAD 节点。
 * 达到原生深度停止条件单独标记，不宣称积分已达到误差目标。
 */
OperationStatus integrateCurveDensity(double first, double last,
    const std::function<Point3D(double)>& derivative,
    const std::function<double(double)>& sizeAt,
    CurveDensityTable& output, double absolutePrecision = 1.e-9,
    const RunControl& control = {});

/** smoothPrimitive：前后 Gauss-Seidel 扫尺寸后重算梯形累计表。
 * growthFactor 对应 sqrt(Mesh.SmoothRatio)，至少为 1。
 * 保留原生反向扫使用右端速度的公式。成功原子替换 table，失败保留原值。
 */
OperationStatus smoothCurveSizes(CurveDensityTable& table, double growthFactor,
                                const RunControl& control = {});

/** max(minimumSegments+1, int(integral+1.99))，含两个端点的节点数。
 * 非法/溢出输入分别抛 invalid_argument/overflow_error，不静默截断。
 */
int computeCurvePointCount(double integral, int minimumSegments);

/** OCCEdge::minimumMeshSegments：圆弧参数跨度、闭环至少 4 段和实体最少段数。
 * arcSpan 对 Circle/Ellipse 为弧度；不通过物理弦长反推分段。
 * 非法/溢出输入分别抛 invalid_argument/overflow_error。
 */
int computeMinimumCurveSegments(CurveKind kind, double arcSpan, bool closed,
    int prescribedMinimum, const SurfaceMeshingOptions& options);

struct StoredCurveSizeResult {
    OperationStatus status;
    double size = -1.;
    bool finiteResult = false;
};
/** meshGEdge.cpp 节点创建时存入 MEdgeVertex 的 lc 元数据。
 * 公式中的 d 为内层曲线速度，保留其对外层累计量的变量遮蔽行为。
 * 此值不等于过滤时的 BGM 尺寸，也不等于面内 mesh/background。
 * 非正/非有限结果不修正；finiteResult 明确诊断，调用方不能按有效尺寸使用。
 */
StoredCurveSizeResult computeStoredCurveSize(const CurveDensitySample& left,
    const CurveDensitySample& right, double speed);

/** 将累计密度分成 pointCount-1 段，表内线性反解，保留两个原参数端点。
 * 对应 meshGEdge 分布循环；不使用 Hermite 或额外曲率细化。
 * 成功替换 parameters，失败保留原值；最少段数/闭环身份由上层决定。
 */
OperationStatus distributeCurveParameters(const CurveDensityTable& table, int pointCount,
    std::vector<double>& parameters, const RunControl& control = {});

struct CurveNode {
    std::uint64_t id = 0;
    double parameter = 0.;
    Point3D point;
    double storedSize = -1.; // MEdgeVertex::getLc；过滤不使用此值，不等于当前 BGM 尺寸。
};
struct CurveNodeFilteringResult {
    OperationStatus status;
    std::size_t sizeEvaluations = 0;
    std::vector<std::uint64_t> removedNodeIds;
    bool minimumNodeCountPreventedRemoval = false;
};

/** filterPoints：普通算法 6、无 transfinite/重组/边界层的内点过滤。
 * beginParameter 是 CAD 反求起点得到的参数，不能暗用周期分支的上/下限。
 * 首点按自身参数查询 1D 尺寸，后续按最后保留点的参数中点查询；d<0.3h 删除。
 * 不检查最后内点到终点。最少内点门槛阻止整批删除，不截取部分候选。
 * 成功原子更新内点列表（身份/顺序保留），失败/取消/预算保留原列表。
 * 一次 1D 尺寸查询计一个预算单位；storedSize 只透传，不参与过滤。
 */
CurveNodeFilteringResult filterCurveNodes(std::vector<CurveNode>& interiorNodes,
    Point3D beginPoint, double beginParameter, int minimumInteriorNodes,
    const std::function<double(double)>& curveSizeAt, const RunControl& control = {});

/** 仅当 i>0 且前一保留点仍为 0D 起点时调用 inverseBeginParameter。
 * 对应 filterPoints 的 CAD 反求时机；每次调用，不提前查询或缓存。
 * 查询失败原子保留 interiorNodes，其余合同同上。
 */
CurveNodeFilteringResult filterCurveNodes(std::vector<CurveNode>& interiorNodes,
    Point3D beginPoint, const std::function<double()>& inverseBeginParameter,
    int minimumInteriorNodes, const std::function<double(double)>& curveSizeAt,
    const RunControl& control = {});
} // namespace mesher::gmshReference
