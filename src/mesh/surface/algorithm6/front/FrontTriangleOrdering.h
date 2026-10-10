#pragma once
/** @file FrontTriangleOrdering.h
 * @brief compareTri3Ptr的不可变键；共享初始与运行期set规则，不添加池索引决胜。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "mesh/surface/algorithm6/data/FrontMeshTypes.h"
namespace mesher::gmshReference {
struct FrontTriangleKey {
    double radius = 0.;
    std::array<std::uint64_t,3> vertices{}; // 排序节点身份；不是物理等价身份。
    int triangle = -1; // 值，不参与set比较。
};
struct FrontTriangleKeyLess {
    /// 优先比较较大的半径，再按顶点键确定稳定次序。
    bool operator()(const FrontTriangleKey& first,const FrontTriangleKey& second) const;
};
/// 由半径和排序后的顶点编号构造前沿队列键。
FrontTriangleKey makeFrontTriangleKey(const FrontTriangle& triangle,int index);
}
