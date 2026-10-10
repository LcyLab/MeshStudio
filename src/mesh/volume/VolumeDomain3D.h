#pragma once
// 文件职责：体域处理：核验约束面覆盖，尝试安全局部恢复，再沿边界屏障的奇偶关系移除外域和内腔。

#include "mesh/data/TetMesh3D.h"
#include <array>
#include <unordered_set>
#include <unordered_map>

namespace mesher {

/// 实际共面面片覆盖，不以约束边存在推断面已恢复。
struct VolumeBoundaryCoverage {
    bool ok=false;
    std::string error;
    std::unordered_set<uint64_t> barriers;
    std::unordered_map<uint64_t,std::vector<int>> sourceFaces;
    std::vector<int> missingFaces;
};

/// faces 使用 mesh.points 索引；允许同一平面片上的不同对角线。
/// 采用归一化投影面积与尺度舍入容差；不是任意精度几何保证。
/// 输入须是无自交、不重叠的闭合表面；不能将装配接触/干涉当作公共界面。
VolumeBoundaryCoverage measureVolumeBoundaryCoverage(
    const TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces);

struct VolumeDomainResult {
    bool ok=false;
    std::string error;
    int recoveredFlips=0;
    int removedTets=0;
    double volume=0;
};

/// 尝试受现有覆盖面保护的 2→3/3→2 翻转恢复；预算耗尽/不可恢复明确失败。
/// 整个操作在副本完成，失败保留输入网格。
VolumeDomainResult recoverVolumeBoundary(
    TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces,int maxFlips=256);

/// 边界完全覆盖后，沿邻接跨屏障翻转奇偶值，剔除外域及内腔。
/// 不使用重心投票/射线多数判定；失败时保留原网格。
VolumeDomainResult extractVolumeDomain(
    TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces);
}
