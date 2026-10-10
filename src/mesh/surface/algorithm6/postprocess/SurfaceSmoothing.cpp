/** @file SurfaceSmoothing.cpp
 * @brief 原生面平滑。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "SurfaceSmoothing.h"
#include <algorithm>
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace {
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); }
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
class SmoothingQueries {
public:
    /// 保存平滑阶段的查询控制和共享计数。
    SmoothingQueries(const RunControl& control,std::size_t& count) : control_(control),count_(count) {}
    /// 检查运行取消状态，取消时停止当前阶段。
    void check() const {
        if(control_.keepGoing && !control_.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"surface smoothing cancelled"}};
    }
    /// 检查预算后执行平滑几何查询，并统一报告异常。
    template<class Function> auto request(Function function) {
        check();
        if(count_ >= control_.maximumGeometryEvaluations) throw OperationFailure{{MeshingStatus::BudgetExceeded,"surface smoothing query budget exceeded"}};
        ++count_;
        try { return function(); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
        catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"surface smoothing source failed"}}; }
    }
private:
    const RunControl& control_; std::size_t& count_;
};
/// 用叉积、点积和反正切计算三角形指定顶点的角度。
double triangleAngle(Point3D p1,Point3D p2,Point3D p3) {
    const Point3D a{p1.x-p2.x,p1.y-p2.y,p1.z-p2.z},b{p3.x-p2.x,p3.y-p2.y,p3.z-p2.z};
    // 保留SVector3::crossprod/norm/dot和atan2；不以acos或面积公式替代。
    const Point3D c{a.y*b.z-b.y*a.z,-(a.x*b.z-b.x*a.z),a.x*b.y-b.x*a.y};
    const double sine = std::sqrt(c.x*c.x+c.y*c.y+c.z*c.z);
    const double cosine = a.x*b.x+a.y*b.y+a.z*b.z;
    return std::atan2(sine,cosine);
}
/// 按编号查询平滑节点关联的三角形。
const std::array<std::uint64_t,3>& incidentTriangle(const FaceMesh& mesh,int index) {
    if(index < 0 || std::size_t(index) >= mesh.triangles.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid smoothing incident triangle"}};
    return mesh.triangles[index];
}
/// 按四面体局部面及面内位置查询顶点编号。
const FaceVertex& faceVertex(const FaceMesh& mesh,std::uint64_t id) {
    const auto found = mesh.vertices.find(id);
    if(found == mesh.vertices.end() || found->second.id != id) throw OperationFailure{{MeshingStatus::InvalidTopology,"smoothing face vertex is missing"}};
    return found->second;
}
/// 反求三角形顶点 UV 并计算参数平面的有向面积。
double parametricTriangleArea(const FaceMesh& mesh,const std::array<std::uint64_t,3>& triangle,
    const SurfaceSmoothingSource& source,SmoothingQueries& queries) {
    if(!source.reparameterizeVertex) throw OperationFailure{{MeshingStatus::InvalidInput,"smoothing single-vertex reparameterization source missing"}};
    std::array<Point2D,3> uv;
    for(int i = 0; i < 3; ++i) {
        const auto& v = faceVertex(mesh,triangle[i]);
        uv[i] = queries.request([&] { return source.reparameterizeVertex(v,mesh); });
        if(!finite(uv[i])) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"smoothing reparameterization is nonfinite"}};
    }
    const double area = .5*std::fabs((uv[1].x-uv[0].x)*(uv[2].y-uv[0].y)-(uv[2].x-uv[0].x)*(uv[1].y-uv[0].y));
    if(!std::isfinite(area)) throw OperationFailure{{MeshingStatus::NonConverged,"smoothing parametric area overflow"}};
    return area;
}
/// 计算面网格中指定三角形的形状质量。
double triangleEta(const FaceMesh& mesh,const std::array<std::uint64_t,3>& triangle) {
    std::array<Point3D,3> points;
    for(int i = 0; i < 3; ++i) points[i] = faceVertex(mesh,triangle[i]).point;
    const auto result = computeTriangleEta(points); require(result.status); return result.eta;
}
}
/// 由三角形内角计算平滑使用的形状质量指标。
SmoothingTriangleQuality computeTriangleEta(const std::array<Point3D,3>& triangle) {
    SmoothingTriangleQuality result;
    for(auto p : triangle) if(!finite(p)) { result.status = {MeshingStatus::InvalidInput,"nonfinite triangle quality coordinate"}; return result; }
    constexpr double pi = 3.14159265358979323846, inverse60 = 1.0/60.0;
    result.anglesDegrees = {180*triangleAngle(triangle[0],triangle[1],triangle[2])/pi,
                           180*triangleAngle(triangle[1],triangle[2],triangle[0])/pi,
                           180*triangleAngle(triangle[2],triangle[0],triangle[1])/pi};
    for(double angle : result.anglesDegrees) if(!std::isfinite(angle)) { result.status = {MeshingStatus::NonConverged,"nonfinite triangle angle arithmetic"}; return result; }
    result.minimumAngle = std::min(std::min(result.anglesDegrees[0],result.anglesDegrees[1]),result.anglesDegrees[2]);
    result.eta = 1.-std::abs(60.-result.minimumAngle)*inverse60; return result;
}
/// 按节点身份升序，在邻点UV查询度量，以方向的度量长度加权平均；不求邻点图表。
SmoothedPositionResult computeSmoothedPosition(Point2D before,const std::map<std::uint64_t,Point2D>& neighbors,
    const SmoothingMetricSource& metric,const RunControl& control) {
    SmoothedPositionResult result; SmoothingQueries queries(control,result.geometryEvaluations);
    try {
        queries.check();
        if(!finite(before) || !metric) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid smoothing position or metric source"}};
        for(const auto& neighbor : neighbors) {
            queries.check();
            if(!neighbor.first || !finite(neighbor.second)) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid smoothing neighbor parameters"}};
            SmoothingNeighbor row; row.vertex = neighbor.first; row.parameters = neighbor.second;
            row.direction = {row.parameters.x-before.x,row.parameters.y-before.y};
            const double length = std::sqrt(row.direction.x*row.direction.x+row.direction.y*row.direction.y);
            if(length != 0.) { const double reciprocal = 1./length; row.direction.x *= reciprocal; row.direction.y *= reciprocal; }
            row.metric = queries.request([&] { return metric(row.parameters); });
            row.weight = std::sqrt(row.metric.gUU*row.direction.x*row.direction.x+2*row.metric.gUV*row.direction.x*row.direction.y+row.metric.gVV*row.direction.y*row.direction.y);
            if(!finite(row.direction) || !std::isfinite(row.weight)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite smoothing metric weight"}};
            result.position.x += row.parameters.x*row.weight; result.position.y += row.parameters.y*row.weight; result.totalWeight += row.weight;
            result.neighbors.push_back(row);
        }
        const double reciprocal = 1./result.totalWeight; result.position.x *= reciprocal; result.position.y *= reciprocal;
        if(!finite(result.position)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite smoothing weighted position"}};
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::NonConverged,error.what()}; }
    return result;
}
/// 原_isItAGoodIdeaToMoveThatVertex：先查询trial的GPoint，再以临时XYZ/参数验收UV面积和最差eta。
SmoothedPositionAcceptance acceptSmoothedPosition(FaceMesh& mesh,std::uint64_t vertex,
    const std::vector<int>& incidentTriangles,Point2D actual,Point2D trial,const SurfaceSmoothingSource& source,const RunControl& control) {
    SmoothedPositionAcceptance result; SmoothingQueries queries(control,result.geometryEvaluations);
    FaceVertex entry; bool saved = false;
    try {
        queries.check(); entry = faceVertex(mesh,vertex); saved = true;
        if(!finite(actual) || !finite(trial) || !source.surfacePoint) throw OperationFailure{{MeshingStatus::InvalidInput,"invalid smoothing trial or point source"}};
        result.surface = queries.request([&] { return source.surfacePoint(trial); });
        if(!result.surface.succeeded) return result;
        if(!finite(result.surface.point)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"smoothing surface point is nonfinite"}};
        for(int i : incidentTriangles) {
            queries.check(); const auto& triangle = incidentTriangle(mesh,i);
            result.oldArea += parametricTriangleArea(mesh,triangle,source,queries);
            result.oldMinimumEta = std::min(triangleEta(mesh,triangle),result.oldMinimumEta);
        }
        auto& changed = mesh.vertices.at(vertex); changed.parameters = trial; changed.point = result.surface.point;
        for(int i : incidentTriangles) {
            queries.check(); const auto& triangle = incidentTriangle(mesh,i);
            result.newArea += parametricTriangleArea(mesh,triangle,source,queries);
            result.newMinimumEta = std::min(triangleEta(mesh,triangle),result.newMinimumEta);
        }
        changed.parameters = actual; changed.point = entry.point;
        if(!std::isfinite(result.oldArea) || !std::isfinite(result.newArea)) throw OperationFailure{{MeshingStatus::NonConverged,"smoothing incident area sum overflow"}};
        result.accepted = !((1.+1.e-10)*result.oldArea < result.newArea || result.newMinimumEta < result.oldMinimumEta);
    } catch(const OperationFailure& error) { if(saved) mesh.vertices.at(vertex) = entry; result.status = error.status; }
      catch(const std::exception& error) { if(saved) mesh.vertices.at(vertex) = entry; result.status = {MeshingStatus::InvalidTopology,error.what()}; }
    return result;
}
/// 在固定 CAD 边界和几何约束下执行指定轮数的原版平滑。
SurfaceSmoothingResult smoothSurfaceMesh(FaceMesh& mesh,const SurfaceSmoothingSource& source,
    const SurfaceSmoothingOptions& options,const RunControl& control,const SurfaceSmoothingObserver& observer) {
    SurfaceSmoothingResult result; SmoothingQueries queries(control,result.geometryEvaluations);
    try {
        queries.check(); if(!options.iterations) return result;
        if(mesh.vertices.size() > control.maximumNodes) throw OperationFailure{{MeshingStatus::BudgetExceeded,"smoothing face vertex budget exceeded"}};
        std::map<std::uint64_t,std::vector<int>> incident;
        for(std::size_t i = 0; i < mesh.triangles.size(); ++i) {
            queries.check();
            for(auto id : mesh.triangles[i]) { faceVertex(mesh,id); incident[id].push_back(int(i)); }
        }
        // 每个子阶段保留自己的观察计数，所有源入口共用此完整运行的预算。
        RunControl stageControl = control; stageControl.maximumGeometryEvaluations = std::numeric_limits<std::size_t>::max();
        auto wrapped = source;
        wrapped.metric = [&](Point2D uv) { return queries.request([&] { return source.metric(uv); }); };
        wrapped.surfacePoint = [&](Point2D uv) { return queries.request([&] { return source.surfacePoint(uv); }); };
        wrapped.reparameterizeVertex = [&](const FaceVertex& v,const FaceMesh& current) { return queries.request([&] { return source.reparameterizeVertex(v,current); }); };
        for(int iteration = 0; iteration < options.iterations; ++iteration) {
            for(const auto& item : incident) {
                queries.check(); SmoothingVertexEvent event; event.iteration = iteration; event.vertex = item.first;
                const auto& vertex = faceVertex(mesh,item.first);
                if(options.boundaryLayerVertices.count(item.first)) event.skip = SmoothingSkip::ProtectedBoundaryLayer;
                else if(vertex.dimension != 2 || vertex.entity != mesh.entity) event.skip = SmoothingSkip::NotOnFace;
                else if(options.faceVerticesWithBoundaryLayerData.count(item.first)) event.skip = SmoothingSkip::FaceVertexBoundaryLayer;
                if(event.skip == SmoothingSkip::None) {
                    if(!source.metric || !source.surfacePoint || !source.reparameterizeVertex || !source.reparameterizeEdge)
                        throw OperationFailure{{MeshingStatus::InvalidInput,"surface smoothing source missing"}};
                    ++result.eligibleVertices; event.before = vertex.parameters;
                    std::map<std::uint64_t,Point2D> neighbors;
                    for(int index : item.second) {
                        queries.check(); const auto& triangle = incidentTriangle(mesh,index);
                        for(int j = 0; j < 3; ++j) {
                            const auto first = triangle[j],second = triangle[(j+1)%3];
                            if(first == item.first || second == item.first) {
                                const auto& a = faceVertex(mesh,first); const auto& b = faceVertex(mesh,second);
                                const auto parameters = queries.request([&] { return source.reparameterizeEdge(a,b,mesh); });
                                if(first == item.first) neighbors[second] = parameters[1]; else neighbors[first] = parameters[0];
                            }
                        }
                    }
                    event.position = computeSmoothedPosition(event.before,neighbors,wrapped.metric,stageControl); require(event.position.status);
                    double factor = 1.; Point2D actual = event.before;
                    for(int trialIndex = 0; trialIndex < 5; ++trialIndex) {
                        queries.check(); SmoothingTrial trial; trial.factor = factor;
                        trial.trial = {event.position.position.x*factor+event.before.x*(1.-factor),event.position.position.y*factor+event.before.y*(1.-factor)};
                        trial.acceptance = acceptSmoothedPosition(mesh,item.first,item.second,actual,trial.trial,wrapped,stageControl); require(trial.acceptance.status);
                        if(trial.acceptance.accepted) {
                            trial.publicationPoint = queries.request([&] { return source.surfacePoint(trial.trial); });
                            if(trial.publicationPoint.succeeded) {
                                if(!finite(trial.publicationPoint.point)) throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"smoothing publication point is nonfinite"}};
                                auto& moved = mesh.vertices.at(item.first); moved.parameters = actual = trial.trial; moved.point = trial.publicationPoint.point;
                                trial.published = true; ++result.acceptedWrites;
                            }
                        }
                        trial.actual = actual; event.trials.push_back(trial); factor /= 1.4;
                    }
                }
                ++result.vertexVisits; if(observer) observer(event);
            }
        }
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::NonConverged,error.what()}; }
      catch(...) { result.status = {MeshingStatus::NonConverged,"unknown surface smoothing failure"}; }
    return result;
}
}
