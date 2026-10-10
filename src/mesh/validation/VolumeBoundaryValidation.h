#pragma once

#include "mesh/surface/legacy/SurfaceMesh3D.h"

namespace mesher {

struct VolumeBoundaryValidationResult {
    bool ok = false;
    std::string error;
    int shellCount = 0;
    // 输入各壳有符号体积之和；允许反向壳抵消，不代表按奇偶填充后的材料体积。
    double signedVolume = 0;
};

/// 体网格入口的只读验收：闭合、朝向一致、顶点流形、逐壳非零体积。
/// 不焊接、不删除面片、不改索引；统一反向及多个独立/嵌套壳均允许。
/// 多壳的内外/空腔由后续奇偶分类解释，不强制壳之间采用同一朝向。
/// 默认复用表面网格的容差相交检查；关闭时仅检查拓扑和逐壳体积。
/// 普通表面网格仍可开放，本验收仅用于进入体网格的边界。
VolumeBoundaryValidationResult validateVolumeBoundary(const SurfaceMesh3D& surface,
                                                       bool checkIntersections = true);

} // namespace mesher
