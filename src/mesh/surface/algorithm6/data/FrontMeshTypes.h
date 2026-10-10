#pragma once
/** @file FrontMeshTypes.h
 * @brief 前沿运行期三角形、R/h排序成员和原边编号邻接；不保存活动队列策略。
 * @date 2026-10-08
 */
#include "FaceMeshTypes.h"
namespace mesher::gmshReference {
struct FrontTriangle {
    std::array<std::uint64_t,3> vertices{};
    NodeSize meanSizes;
    double targetSize = 0., radius = 0.; // radius为物理外接圆R/targetSize，不是R/最短边。
    std::array<int,3> neighbors{-1,-1,-1}; // 边i连接顶点(i+2)%3与i；-1为无邻片。
    bool deleted = false;
};
struct FrontMesh {
    std::vector<FrontTriangle> triangles; // 原输入/创建序，索引稳定；包含未入set的重复片。
    std::vector<int> orderedTriangles; // 初始AllTris快照；后续运行集合由FrontQueueState维护。
};
}
