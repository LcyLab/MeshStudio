#pragma once
/** @file SurfaceReferenceBoundary.h
 * @brief 尺寸加权弧长积分及累计度量反解，不分配CAD节点。
 * @date 2026-10-06
 */
#include "geometry/types/MesherTypes.h"
#include <functional>
#include <string>
#include <vector>
namespace mesher {
struct ReferenceCurveSamplingOptions {
    double relativeTolerance=1e-6;
    int maxDepth=24,maxEvaluations=65536,maxSegments=200000;
};
/** 自适应积分 |C'(t)|/h(t)，反解等累计度量参数，包含两个端点。
 * sizeAt、derivative 在本轮不可变；sizeAt 须为正有限值，零导数允许。
 * 失败不覆盖 parameters；取消、数值停滞、预算与非法查询均明确返回错误。
 * 这是密度布点，实际弦误差/法向变化由调用方继续验收。
 */
bool sampleReferenceCurve(double first,double last,
    const std::function<Point3D(double)>& derivative,const std::function<double(double)>& sizeAt,
    std::vector<double>& parameters,std::string* error,ReferenceCurveSamplingOptions={},
    const std::function<bool()>& keepGoing={});
}
