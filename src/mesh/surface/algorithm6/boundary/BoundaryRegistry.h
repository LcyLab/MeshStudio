#pragma once
/** @file BoundaryRegistry.h
 * @brief 模型级物理节点和 CAD 边离散；不负责面上的 UV 分支。
 * @date 2026-10-07
 */
#include "CurveDiscretization.h"
#include "MeshSizeEvaluator.h"

namespace mesher::gmshReference {
struct BoundaryVertexInput {
    int entity = -1;
    Point3D point;
};

// 从 GeometrySource/OccCadGeometry 取得的本次运行只读视图。
// point/derivative 必须为精确 CAD 求值；不能暗用有限差分 D1。
// inverseParameter 对应 OCCEdge::parFromPoint，不能直接返回参数下限。
struct BoundaryCurveInput {
    int entity = -1;
    int beginVertex = -1, endVertex = -1;
    double first = 0., last = 1.;
    CurveKind kind = CurveKind::Other;
    bool geometryDegenerate = false;
    int prescribedMinimumSegments = 1;
    std::vector<int> incidentFaces;
    std::function<Point3D(double)> point;
    std::function<Point3D(double)> derivative;
    std::function<double(Point3D)> inverseParameter;
};

struct BoundaryInput {
    // 显式保留原生实体遍历顺序；不同实体不按坐标合并。
    std::vector<BoundaryVertexInput> vertices;
    std::vector<BoundaryCurveInput> curves;
    double modelCharacteristicLength = 1.;
    double geometryTolerance = 1.e-8;
    double edgeLengthTolerance = 0.;
};

struct BoundaryNode {
    std::uint64_t id = 0; // 从 1 分配，nodes[id-1]；删除内点保留编号空洞。
    Point3D point;
    int dimension = -1, entity = -1;
    double parameter = 0., storedSize = -1.;
    bool alive = true;
};
struct BoundarySample {
    std::uint64_t node = 0;
    double parameter = 0.; // 闭环起终点物理身份相同，参数不同。
};
struct BoundaryCurve {
    int entity = -1;
    std::uint64_t beginNode = 0, endNode = 0;
    std::vector<BoundarySample> samples; // 相邻样本是一条有向线段。
    std::vector<int> incidentFaces;
    std::uint64_t version = 0;
    double length = 0.;
    bool geometryDegenerate = false, tooSmall = false, meshDegenerate = false;
    bool integrationDepthLimitReached = false, smoothingLimitReached = false;
    bool minimumNodeCountPreventedRemoval = false;
};
struct BoundaryMesh {
    std::vector<BoundaryNode> nodes;
    std::vector<BoundaryCurve> curves;
    std::size_t invalidSizeFallbacks = 0;
};
struct BoundaryOperationResult {
    OperationStatus status;
    // 本模块每次 CAD point/D1/inverse 或尺寸求值计一次；跨所有边累计。
    // 与原生查询次数无等价承诺，不作为收敛条件。
    std::size_t geometryEvaluations = 0;
};

/** meshGVertex/meshGEdge 普通 OCC 算法 6：0D 身份、物理长度积分、
 * 端点 0D+1D 尺寸、平滑/分布/过滤和闭环端点重定位。
 * 每条 CAD 边一份有向物理链；退化几何边不生成线段，保留端点身份。
 * 仅成功原子替换 output；取消、预算和几何失败不发布部分链。
 * maximumNodes 限制总分配数（包含过滤后空洞），不重新编号。
 */
BoundaryOperationResult buildBoundaryMesh(const BoundaryInput& input,
    const SurfaceMeshingOptions& options, const GeometrySizeSource& sizes,
    BoundaryMesh& output, const RunControl& control = {});

struct UnrecoveredEdge {
    int curveEntity = -1;
    std::uint64_t from = 0, to = 0; // FaceChart 先将 BDS/UV 身份转为物理身份。
};
struct BoundaryRefinementResult : BoundaryOperationResult {
    std::vector<int> incidentFaces;
    std::vector<std::size_t> unmatchedRequests; // 陈旧/重复请求的原输入索引。
    std::size_t splitSegments = 0;
};

/** meshGFace.cpp::remeshUnrecoveredEdges：按调用方给定的原生请求顺序，
 * 匹配无向物理线段，参数中点插点，getLc==-1 才按节点分类查询 BGM。
 * lc 为两端元数据平均；闭环 0D 端点按另一端距参数上下限选分支。
 * 成功原子更新 mesh，每次有实际更新的请求递增边版本；长度状态保留。
 * 返回所有请求边关联面（包括未匹配请求），编排层负责清理/标记 PENDING。
 * 重复或陈旧线段明确列于 unmatchedRequests；失败保留 mesh 并清空更新结果。
 */
BoundaryRefinementResult refineUnrecoveredEdges(const BoundaryInput& input,
    const SurfaceMeshingOptions& options, const GeometrySizeSource& sizes,
    const std::vector<UnrecoveredEdge>& requests, BoundaryMesh& mesh,
    const RunControl& control = {});

/** 查指定 CAD 边的关联面，去重后保留输入次序；未知边返回 InvalidInput。
 * 不修改或清理面网格；SurfaceGenerator 负责失效处理。
 */
OperationStatus findIncidentFaces(const BoundaryMesh& mesh, int curveEntity,
                                 std::vector<int>& output);
} // namespace mesher::gmshReference
