/** @file BdsSwapQualification.cpp
 * @brief 初始BDS翻边资格实现。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "BdsSwapQualification.h"
#include <cmath>
namespace mesher::gmshReference {
/// 计算 BDS 参数三角形的有向面积。
double computeBdsParameterTriangleArea(const BdsPoint& first,const BdsPoint& second,const BdsPoint& third) {
    if(int(first.degenerated != 0)+int(second.degenerated != 0)+int(third.degenerated != 0) > 1) return 0.;
    auto poleArea = [](const BdsPoint& pole,const BdsPoint& a,const BdsPoint& b,int axis) {
        const double delta = axis == 1 ? std::abs(b.uv.x-a.uv.x) : std::abs(b.uv.y-a.uv.y);
        const double center = axis == 1 ? .5*(b.uv.y+a.uv.y)-pole.uv.y : .5*(b.uv.x+a.uv.x)-pole.uv.x;
        return 2*std::abs(center)*delta;
    };
    double cross;
    if(first.degenerated == 1) cross = poleArea(first,second,third,1);
    else if(second.degenerated == 1) cross = poleArea(second,first,third,1);
    else if(third.degenerated == 1) cross = poleArea(third,first,second,1);
    else if(first.degenerated == 2) cross = poleArea(first,second,third,2);
    else if(second.degenerated == 2) cross = poleArea(second,first,third,2);
    else if(third.degenerated == 2) cross = poleArea(third,first,second,2);
    else {
        const double ax = first.uv.x-second.uv.x, ay = first.uv.y-second.uv.y;
        const double bx = first.uv.x-third.uv.x, by = first.uv.y-third.uv.y;
        cross = ax*by-ay*bx;
    }
    return .5*cross;
}
/// 根据分类、面积和邻域条件判断初始 BDS 边能否交换。
bool canSwapInitialBdsEdge(const BdsMesh& mesh,int edge,const BdsEdgeNeighborhood& n) {
    if(edge < 0 || std::size_t(edge) >= mesh.edges.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid initial BDS edge"}};
    const auto& e = mesh.edges[edge];
    if(e.deleted || e.faces.size() != 2 || e.classification.dimension == 1) return false;
    const auto& p1 = mesh.points.at(n.first); const auto& p2 = mesh.points.at(n.second);
    const auto& op1 = mesh.points.at(n.oppositeNodes[0]); const auto& op2 = mesh.points.at(n.oppositeNodes[1]);
    if(!p1.configurationModified && !p2.configurationModified && !op1.configurationModified && !op2.configurationModified) return false;
    if((op1.degenerated && op2.degenerated) || (op1.periodicCounterpart && op2.periodicCounterpart)) return false;
    for(const auto* p : {&p1,&p2}) {
        if(p->classification.dimension == 2 && p->edges.size() <= 4) return false;
        if(p->classification.dimension >= 0 && p->classification.dimension < 2 && p->edges.size() <= 3) return false;
    }
    const double s1 = std::abs(computeBdsParameterTriangleArea(p1,p2,op1)), s2 = std::abs(computeBdsParameterTriangleArea(p1,p2,op2));
    const double s3 = std::abs(computeBdsParameterTriangleArea(p1,op1,op2)), s4 = std::abs(computeBdsParameterTriangleArea(p2,op1,op2));
    if(std::abs(s1+s2-s3-s4) > 1.e-12*(s3+s4)) return false;
    return n.sideEdges[0] != n.sideEdges[2] && n.sideEdges[1] != n.sideEdges[3];
}
}
