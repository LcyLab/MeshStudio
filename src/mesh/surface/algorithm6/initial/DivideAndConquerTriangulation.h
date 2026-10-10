#pragma once
/** @file DivideAndConquerTriangulation.h
 * @brief 原生周期初始剖分的纯 UV 分治/有序环邻接；不求 CAD 或恢复约束。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"
#include <array>
#include <vector>

namespace mesher::gmshReference {
struct InitialPoint {
    int localId = -1;
    Point2D coordinates; // 已由图表消费运行扰动流；此模块不再扰动。
};
struct DivideAndConquerTriangulation {
    std::vector<InitialPoint> sortedPoints; // x/y 升序，localId 可重复但坐标不得完全重合。
    std::vector<std::array<int,3>> triangles; // 三个 sortedPoints 索引，保留原生输出序和顺时针。
    std::size_t hullPointCount = 0;
};

/** 对应 DocRecord::MakeMeshWithPoints：按坐标排序、递归合并及按环邻接输出三角形。
 * 不插外围点、不移点、不归一三角形方向、不压缩重复物理身份。
 * 原生 atan2/1e-15 邻接决胜、严格左右谓词及共圆不算 outside 的 Qtest 均保留。
 * Qtest 当前组合鲁棒符号；原版行列式数值乘积上下溢分支仍待核对，不保证任意指数输入等价。
 * 输入完全重合或三角形实际数与 Euler 预估不同明确失败，不读取原生未初始化数组。
 * 少于三点保留原输入序并返回空三角形；失败/取消/预算保留 output。
 * maximumNodes 限制输入位置数，keepGoing 在合并/邻接遍历中检查；不调用 CAD。
 */
OperationStatus buildDivideAndConquerTriangulation(const std::vector<InitialPoint>& input,
    DivideAndConquerTriangulation& output, const RunControl& control = {});
} // namespace mesher::gmshReference
