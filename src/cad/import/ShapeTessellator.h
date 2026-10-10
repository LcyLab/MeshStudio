#pragma once
// 文件职责：OCC 离散兼容工具：读取 CAD 并输出三角汤；当前 CAD 体网格入口使用 SurfaceMeshEngine 的索引表面。

#include <array>
#include <vector>

#include <QString>

#include "geometry/types/MesherTypes.h"

/// B-Rep 类几何（STEP / IGES / BRep）→ 三角形汤。
///
/// CAD 几何由曲面和曲线定义；本工具通过 OCC 离散取得三角片，输出允许顶点重复。
/// 这是兼容及独立测试入口，不是当前 GUI 的 CAD 体网格生成主链。
/// 当前主链先由 SurfaceMeshEngine 生成保留共享节点和 CAD 标签的索引表面；
/// MainWindow 只复用本模块的 isBRepFormat 来识别输入格式。
///
/// 离散精度由 `targetSize` 决定（见 ShapeToPolyData::deflectionFor）：
/// 目标单元尺寸越细，弦高越小，避免粗四面体挂在低精度折线上。
///
/// 与 ShapeLoader 的分工：ShapeLoader 只负责「文件 → TopoDS_Shape」，
/// 本模块负责「CAD 文件 → OCC 离散三角汤」；算法库 mesher 与 TetMeshEngine 不依赖 OCC。
namespace ShapeTessellator {

/// 后缀是否属于需要先离散的 B-Rep 类格式（step/stp/iges/igs/brep）。
bool isBRepFormat(const QString& filePath);

/// 读取并离散。vertices / triangles 为输出（调用前会被清空）。
/// 失败时把原因写入 error（非空时）。
bool tessellateFile(const QString& filePath,
                    double targetSize,
                    std::vector<mesher::Point3D>& vertices,
                    std::vector<std::array<int, 3>>& triangles,
                    QString* error);

} // namespace ShapeTessellator
