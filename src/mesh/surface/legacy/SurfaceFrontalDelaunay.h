#pragma once
/** @file SurfaceFrontalDelaunay.h
 * @brief 从约束边或尺寸已满足的邻片推进物理度量布点，复用增量 CDT。
 * @date 2026-10-06
 */
#include "CadSurfaceMesher.h"
#include "SurfaceDomainState.h"
#include <functional>

namespace mesher {
enum class SurfaceFrontalEdgeSource { ProtectedConstraint, DomainBoundary, SettledNeighbor };
struct SurfaceFrontalCandidateSample {
    std::array<Point2D,2> baseUV{};
    Point2D candidateUV{};
    double target=0;
    SurfaceFrontalEdgeSource source=SurfaceFrontalEdgeSource::DomainBoundary;
};
struct SurfaceFrontalStatistics {
    int attempted=0,inserted=0,rejected=0;
    std::vector<std::string> refusals; // 最多八个拒绝原因，非每次候选日志。
    std::vector<SurfaceFrontalCandidateSample> samples; // 最多八个构造坐标快照，无可复用局部槽编号。
};

/** 在已分类的边界 CDT 中生成面内点；保留原 CAD UV 和固定约束。
 * global 将局部点映射到 xyz；allocate 分配或复用 CAD 点槽并返回全局编号。
 * targetAt 返回物理目标边长，geometry 在本轮保持不变。
 * 点位在真实邻域切平面中构造，再有界反解原CAD UV；非线性参数速度不直接作为物理高度。
 * work 与调用方共享候选/细化预算；失败明确返回 error，取消不当作完成。
 * 候选拒绝由局部插点事务回滚；已接受点在后续失败时不整体撤销。
 * activeVertexCount 非空时提供整个运行的活动点数；空时使用 xyz.size()。
 * 本阶段不代替调用方最终偏差、法向、边长、拓扑及相交验收。
 */
bool generateSurfaceFrontalPoints(Mesh2D& mesh,std::vector<int>& global,
    const std::vector<Point3D>& xyz,SurfaceDomainState& domain,
    const std::vector<std::pair<int,int>>& constraints,const GeometrySource& geometry,int face,
    const SurfaceCriteria& criteria,const std::function<double(Point3D)>& targetAt,
    const std::function<int(Point3D)>& allocate,int& work,SurfaceFrontalStatistics& statistics,
    std::string* error,MeshingProgress progress={},MeshObservation observation={},
    const std::function<size_t()>& activeVertexCount={});
}
