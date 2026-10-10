#pragma once
/** @file SurfaceSmoothing.h
 * @brief 原生一阶三角面平滑的加权位置、临时验收和五次试探；不翻边或改变尺寸场。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "mesh/surface/algorithm6/data/FaceMeshTypes.h"
#include "mesh/surface/algorithm6/geometry/SurfaceMetric.h"
#include <set>
namespace mesher::gmshReference {
struct SmoothingSurfacePoint { Point3D point; bool succeeded = false; };
using SmoothingMetricSource = std::function<SurfaceMetric(Point2D)>;
struct SurfaceSmoothingSource {
    SmoothingMetricSource metric;
    std::function<SmoothingSurfacePoint(Point2D)> surfacePoint;
    std::function<Point2D(const FaceVertex&,const FaceMesh&)> reparameterizeVertex;
    std::function<std::array<Point2D,2>(const FaceVertex&,const FaceVertex&,const FaceMesh&)> reparameterizeEdge;
};
struct SmoothingTriangleQuality {
    OperationStatus status;
    std::array<double,3> anglesDegrees{};
    double minimumAngle = 0., eta = 0.;
};
/// 由三角形内角计算平滑使用的形状质量指标。
SmoothingTriangleQuality computeTriangleEta(const std::array<Point3D,3>& triangle);
struct SmoothingNeighbor {
    std::uint64_t vertex = 0;
    Point2D parameters, direction;
    SurfaceMetric metric;
    double weight = 0.;
};
struct SmoothedPositionResult {
    OperationStatus status;
    Point2D position;
    double totalWeight = 0.;
    std::vector<SmoothingNeighbor> neighbors;
    std::size_t geometryEvaluations = 0;
};
/** 按节点身份升序，在邻点UV查询度量，以方向的度量长度加权平均；不求邻点图表。
 * 零总权重/非有限算术明确返回未收敛，不伪造平均点或跳过奇异邻点。
 */
SmoothedPositionResult computeSmoothedPosition(Point2D before,const std::map<std::uint64_t,Point2D>& neighbors,
    const SmoothingMetricSource& metric,const RunControl& control = {});
struct SmoothedPositionAcceptance {
    OperationStatus status;
    bool accepted = false;
    SmoothingSurfacePoint surface;
    double oldArea = 0., newArea = 0., oldMinimumEta = 1., newMinimumEta = 1.;
    std::size_t geometryEvaluations = 0;
};
/** 原_isItAGoodIdeaToMoveThatVertex：先查询trial的GPoint，再以临时XYZ/参数验收UV面积和最差eta。
 * 正常完成恢复到actual参数和入口XYZ；取消/异常恢复整个入口节点。函数不永久发布移动。
 * incidentTriangles保留关联片向量的原序与重复。面积逐顶点reparameterizeVertex，不用邻边图表。
 */
SmoothedPositionAcceptance acceptSmoothedPosition(FaceMesh& mesh,std::uint64_t vertex,
    const std::vector<int>& incidentTriangles,Point2D actual,Point2D trial,
    const SurfaceSmoothingSource& source,const RunControl& control = {});
struct SurfaceSmoothingOptions {
    int iterations = 1;
    std::set<std::uint64_t> boundaryLayerVertices, faceVerticesWithBoundaryLayerData;
};
enum class SmoothingSkip { None, NotOnFace, ProtectedBoundaryLayer, FaceVertexBoundaryLayer };
struct SmoothingTrial {
    Point2D trial, actual;
    double factor = 0.;
    SmoothedPositionAcceptance acceptance;
    bool published = false;
    SmoothingSurfacePoint publicationPoint;
};
struct SmoothingVertexEvent {
    int iteration = 0;
    std::uint64_t vertex = 0;
    SmoothingSkip skip = SmoothingSkip::None;
    Point2D before;
    SmoothedPositionResult position;
    std::vector<SmoothingTrial> trials;
};
using SurfaceSmoothingObserver = std::function<void(const SmoothingVertexEvent&)>;
struct SurfaceSmoothingResult {
    OperationStatus status;
    std::size_t vertexVisits = 0, eligibleVertices = 0, acceptedWrites = 0, geometryEvaluations = 0;
};
/** 按节点身份升序、转移后片序收集邻点；每节点固定五次试探，即使接受也继续缩步。
 * 只移动dimension=2且entity=mesh.entity的自由节点，保持边界/embedded及显式BL保护。
 * 后续节点看到前序已发布位置。取消/查询失败保持此前完整试探提交，临时验收不遗留改写。
 * 原查询次数保留：验收成功后还查询第二个GPoint，只有它也成功才发布。
 * geometryEvaluations统计源入口请求（边双参数/单节点参数/度量/点），跨轮共用预算。
 * 不改变连接、身份/最大号、尺寸/前沿槽，不清理周期临时节点；0轮无需源回调。
 */
SurfaceSmoothingResult smoothSurfaceMesh(FaceMesh& mesh,const SurfaceSmoothingSource& source,
    const SurfaceSmoothingOptions& options = {},const RunControl& control = {},
    const SurfaceSmoothingObserver& observer = {});
}
