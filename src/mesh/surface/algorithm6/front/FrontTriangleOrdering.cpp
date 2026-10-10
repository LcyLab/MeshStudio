/** @file FrontTriangleOrdering.cpp
 * @brief compareTri3Ptr半径降序/MFaceLessThan身份并列键。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontTriangleOrdering.h"
#include <algorithm>
namespace mesher::gmshReference {
/// 优先比较较大的半径，再按顶点键确定稳定次序。
bool FrontTriangleKeyLess::operator()(const FrontTriangleKey& first,const FrontTriangleKey& second) const
{
    if(first.radius > second.radius) return true;
    if(first.radius < second.radius) return false;
    return first.vertices < second.vertices;
}
/// 由半径和排序后的顶点编号构造前沿队列键。
FrontTriangleKey makeFrontTriangleKey(const FrontTriangle& triangle,int index)
{
    FrontTriangleKey key{triangle.radius,triangle.vertices,index};
    std::sort(key.vertices.begin(),key.vertices.end());
    return key;
}
}
