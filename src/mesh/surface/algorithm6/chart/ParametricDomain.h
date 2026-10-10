#pragma once
/** @file ParametricDomain.h
 * @brief 原pointInsideParametricDomain的严格采样段穿越计数；不查询CAD或判断裁剪距离。
 * @date 2026-10-08
 */
#include "PeriodicFaceChart.h"
namespace mesher::gmshReference {
struct ParametricDomainResult { OperationStatus status; bool inside = false; std::size_t crossingCount = 0; };
/** FAR=high+(high-low)，保留原算术序；非有限/倒置范围不发布output。 */
OperationStatus computeParametricFarPoint(Point2D lower,Point2D upper,Point2D& output);
/** 按采样原序比较两组orient2dValue的数值乘积<0；端点/共线不算严格穿越。
 * 空边界返回false，和上层trueBoundary=nullptr不门控不同。
 * 保留乘积上/下溢的原比较语义，不替换为符号相乘或加容差。
 */
ParametricDomainResult testParametricDomain(const std::vector<TrueBoundarySegment>& boundary,
    Point2D point,Point2D far,const RunControl& control = {});
}
