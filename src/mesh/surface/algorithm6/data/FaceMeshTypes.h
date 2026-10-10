#pragma once
/** @file FaceMeshTypes.h
 * @brief 前沿前的节点身份、周期临时副本与面连接；不等同最终物理节点编号。
 * @date 2026-10-08
 */
#include "SurfaceTypes.h"
#include <array>
#include <cstdint>
#include <map>
#include <vector>
namespace mesher::gmshReference {
enum class FaceVertexOrigin { Original, PeriodicCopy, FaceCreated };
struct FaceVertex {
    std::uint64_t id = 0, physicalId = 0;
    FaceVertexOrigin origin = FaceVertexOrigin::Original;
    Point3D point;
    int dimension = -1, entity = -1; // 原MVertex实体，不用BDS临时CLASS_F替换。
    Point2D parameters; // 1D为(u,0)，2D为(u,v)，0D为(0,0)。
    double storedSize = -1.; // 仅1D MEdgeVertex持有lc；不在此求前沿双尺寸。
};
struct FaceMesh {
    int entity = -1;
    // 临时/新节点按本阶段创建序分配身份，不复刻进程全局MVertex自动编号。
    std::uint64_t maximumVertexId = 0;
    std::map<std::uint64_t,FaceVertex> vertices;
    std::map<int,std::uint64_t> bdsToVertex;
    std::map<std::uint64_t,std::uint64_t> equivalence; // 临时副本->原节点；此阶段不折叠。
    std::map<std::uint64_t,Point2D> parametricCoordinates;
    std::vector<int> unreconstructedSeams; // 原版dim>=2重复只报错，保留第一次UV和原身份。
    std::vector<std::uint64_t> interiorVertices; // BDS2GMSH创建MFaceVertex的追加序。
    std::vector<std::array<std::uint64_t,3>> triangles;
    // 最终周期连接折叠前保存每角UV；空表示普通节点参数视图。
    // 平滑后仅更新内部节点角UV，边界角保留各自缝分支。
    std::vector<std::array<Point2D,3>> triangleCornerUV;
};
}
