#pragma once

#include "CadSurfaceMesher.h"
#include <limits>

namespace mesher {

// 局部第一基本形式 J^T J 的 Cholesky 分解；映射坐标采用物理长度单位，原 CAD UV 保持不变。
struct SurfaceMetric {
    Point2D origin{};
    double xx=1,xy=0,yy=1;
    /// 将原 UV 坐标映射到局部物理长度度量平面。
    Point2D project(Point2D p) const;
    /// 将局部度量平面坐标还原为原 UV 参数。
    Point2D unproject(Point2D p) const;
};

/// 由曲面导数构造局部第一基本形式及其分解。
bool surfaceMetricAt(const GeometrySource& geometry,int face,Point2D uv,
                     SurfaceMetric& metric,std::string* error);

struct SurfaceSizeSample {
    bool valid=false,singular=false;
    double target=0,maxCurvature=0;
    // 独立于目标尺寸的局部物理弦跨度曲率估计。
    double curvatureSpan=std::numeric_limits<double>::infinity();
    std::string error;
};

// 估计单位物理切向长度上的最大法向导数，形成三维各向同性目标尺寸；几何要求可使它小于最小尺寸，实际面片测量仍决定验收。
/// 结合曲率和几何偏差要求估计局部目标尺寸。
SurfaceSizeSample surfaceSizeAt(const GeometrySource& geometry,int face,Point2D uv,
                               double target,const SurfaceCriteria& criteria);

} // namespace mesher
