#pragma once
/** @file SurfaceBoundarySizeField.h
 * @brief 同面有效邻接上的三维长度尺寸传播，不穿过孔或独立区域。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "geometry/types/MesherTypes.h"
#include <array>
#include <string>
#include <vector>
namespace mesher {
class SurfaceBoundarySizeField {
    std::vector<double> sizes;
    std::vector<std::vector<std::pair<int,double>>> graph;
    double targetSize=0,slope=0;
public:
    /// 沿有效网格路径传播固定边界尺寸并初始化节点尺寸。
    bool initialize(const std::vector<Point3D>& points,const std::vector<std::array<int,3>>& triangles,
                    const std::vector<std::pair<int,int>>& boundary,double target,double minSize,double growth,std::string* error,
                    const std::vector<std::pair<int,double>>& seedSizes={});
    /// 查询指定节点的边界传播尺寸。
    double at(int vertex) const {return sizes.at(vertex);}
    /// 取候选三角形各节点尺寸的最小值。
    double candidateSize(std::array<int,3> vertices) const;
    // 新点继承合法源邻域的尺寸；不重新扫描 CAD 边。
    bool append(const std::vector<std::pair<int,double>>& neighbors,std::string* error);
};
}
