#pragma once
/** @file BdsSwapQualification.h
 * @brief BDS初始Quality(false,true)翻边资格；不计算度量或改连接。
 * @date 2026-10-08
 */
#include "mesh/surface/algorithm6/data/BdsTopology.h"
namespace mesher::gmshReference {
/** 原生surface_triangle_param的半面积：原始double叉积，退化轴1/2另算，
 * 多个退化点为0；不以自适应orient2d替代，不限制IEEE算术结果。
 */
double computeBdsParameterTriangleArea(const BdsPoint& first,const BdsPoint& second,const BdsPoint& third);
/** 对应swap_edge(...,BDS_SwapEdgeTestQuality(false))：普通保护、对角两点
 * 退化/counterpart保护、旧端点分类/连接度和1e-12相对面积守恒，再排重侧边。
 * false只关三角形gamma改善，testSmallTriangles仍为true；不套Recover凸性。
 */
bool canSwapInitialBdsEdge(const BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& neighborhood);
}
