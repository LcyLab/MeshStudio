#pragma once
/** @file SurfaceSizingPolicy.h
 * @brief 固定几何来源的质量尺寸协调与局部下限，保留常规密度参数。
 * @date 2026-10-05
 * @modified 2026-10-10
 */
#include "SurfaceBoundarySizeField.h"
#include "CadBoundaryRegistry.h"
#include "CadSurfaceMesher.h"
namespace mesher {
// 质量阶段的目标尺寸和新增边下限共享固定几何来源，图表不跨独立部件。
class SurfaceSizingPolicy {
    SurfaceBoundarySizeField targets,floors,regularTargets;
    double slope=0,regularFloor=0;
    size_t count=0;
public:
    // 下限是几何目标的半尺度，其长度传播斜率也取一半，避免过早恢复常规 min。
    static double floorGrowthRatio(double growth){return 1+(growth-1)*.5;}
    /// 协调目标尺寸、最小尺寸、边界种子及增长率。
    bool initialize(const std::vector<Point3D>& points,const std::vector<std::array<int,3>>& triangles,
        const std::vector<std::pair<int,int>>& boundary,const std::vector<CadBoundarySizingAnchor>& anchors,
        const SurfaceCriteria& criteria,std::string* error);
    /// 查询指定节点的目标尺寸。
    double target(int v) const {return targets.at(v);}
    /// 查询指定节点的尺寸下限。
    double floor(int v) const {return floors.at(v);}
    /// 读取候选三角形协调后的目标尺寸。
    double candidateTarget(std::array<int,3> ids) const {return targets.candidateSize(ids);}
    /// 读取候选三角形常规传播得到的目标尺寸。
    double regularCandidateTarget(std::array<int,3> ids) const {return regularTargets.candidateSize(ids);}
    /// 读取候选三角形协调后的尺寸下限。
    double candidateFloor(std::array<int,3> ids) const {return floors.candidateSize(ids);}
    /// 结合邻域距离计算新节点允许的最小尺寸。
    double insertionFloor(const std::vector<std::pair<int,double>>& neighbors) const;
    /// 按邻域尺寸和距离追加新节点的尺寸状态。
    bool append(const std::vector<std::pair<int,double>>& neighbors,std::string* error);
};
}
