#pragma once
/** @file SurfaceTangentChart.h
 * @brief 曲面局部切平面候选及有界原参数反解；不修改网格拓扑。
 * @date 2026-10-03
 * @modified 2026-10-10
 */
#include "CadSurfaceMesher.h"

namespace mesher {
struct SurfaceTangentChart {
    Point3D origin{},axisU{},axisV{},normal{};
    Point2D seed{};
    SurfaceParameterBounds window;
    double tolerance=0;
    /// 将三维点投影到局部正交切平面坐标。
    Point2D project(Point3D point) const;
};
/** 建立实际三维邻域的切平面及原 UV 分支窗口。
 * 窗口只限制反解步长，不代表有效裁剪域；调用方仍检查裁剪与约束。
 */
bool surfaceTangentChartAt(const GeometrySource& geometry,int face,
    const std::array<Point2D,3>& uv,const std::array<Point3D,3>& xyz,
    SurfaceTangentChart& chart,std::string* error);
/** 四边形空圆判断专用：四点排序后构造相同物理平面，换对角线不改变判据。
 * 不提供反解窗口；仅使用返回chart的project、origin和轴。
 */
bool surfaceTangentQuadChartAt(const GeometrySource& geometry,int face,
    const std::array<Point2D,4>& uv,const std::array<Point3D,4>& xyz,
    SurfaceTangentChart& chart,std::string* error);
struct SurfaceChartInverse {
    bool valid=false;
    Point2D uv{};
    Point3D point{};
    double residual=0;
    int iterations=0;
    std::string error;
};
/** 有界 Newton/缩步反解切平面坐标，使用原曲面坐标与 D1 查询。
 * 返回原曲面上的点；不是到任意底层曲面的全局最近点投影。
 * 不换周期分支，最多 20 步，每步最多 8 次缩步；几何查询异常原样传播。
 */
SurfaceChartInverse invertSurfaceTangentChart(const GeometrySource& geometry,int face,
    const SurfaceTangentChart& chart,Point2D point);
struct SurfaceTangentCandidates {
    std::vector<Point2D> uv;
    std::vector<std::string> refusals;
};
// 最多两个物理偏心/外心候选；成功反解后仍由调用方验收有效裁剪域和三维质量。
SurfaceTangentCandidates surfaceTangentRefinementCandidates(const GeometrySource& geometry,int face,
    const std::array<Point2D,3>& uv,const std::array<Point3D,3>& xyz,double target);
} // namespace mesher
