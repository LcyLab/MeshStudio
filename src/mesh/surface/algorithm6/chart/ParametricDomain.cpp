/** @file ParametricDomain.cpp
 * @brief 参数域严格穿越计数。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "ParametricDomain.h"
#include "geometry/predicates/GeomPredicates.h"
#include <cmath>
#include <exception>
namespace mesher::gmshReference {
namespace { /* 检查输入坐标或数值是否有限。 */ bool finite(Point2D p) { return std::isfinite(p.x) && std::isfinite(p.y); } }
/// 计算参数域射线判断所需的远点。
OperationStatus computeParametricFarPoint(Point2D lower,Point2D upper,Point2D& output) {
    if(!finite(lower) || !finite(upper) || lower.x > upper.x || lower.y > upper.y) return {MeshingStatus::InvalidInput,"invalid parametric bounds"};
    const Point2D far{upper.x+(upper.x-lower.x),upper.y+(upper.y-lower.y)};
    if(!finite(far)) return {MeshingStatus::NonConverged,"parametric FAR overflow"};
    output = far; return {};
}
/// 判断 UV 点在真实边界环中的域归属。
ParametricDomainResult testParametricDomain(const std::vector<TrueBoundarySegment>& boundary,Point2D point,Point2D far,const RunControl& control) {
    ParametricDomainResult result;
    try {
        auto check = [&] { if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"parametric domain cancelled"}}; };
        check(); if(!finite(point) || !finite(far)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite parametric domain query"}};
        for(const auto& segment : boundary) {
            check(); if(!finite(segment.from) || !finite(segment.to)) throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite true boundary segment"}};
            double a = orient2dValue(segment.from,segment.to,point),b = orient2dValue(segment.from,segment.to,far);
            // 原实现比较数值乘积，不用独立符号判断替代上下溢结果。
            if(a*b < 0.) {
                a = orient2dValue(point,far,segment.from); b = orient2dValue(point,far,segment.to);
                if(a*b < 0.) ++result.crossingCount;
            }
        }
        check(); result.inside = result.crossingCount%2 != 0;
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidInput,error.what()}; }
      catch(...) { result.status = {MeshingStatus::InvalidInput,"unknown parametric domain failure"}; }
    return result;
}
}
