#pragma once

#include <array>
#include <vector>

#include <QString>

#include "mesher/MesherTypes.h"

/// B-Rep 类几何（STEP / IGES / BRep）→ 三角形汤。
///
/// ★ 为什么需要这一层 ★
/// STEP / IGES / BRep 是**边界表示**（面由 NURBS/解析曲面定义，棱由曲线定义），
/// 文件里**不携带任何三角面片**。而自研四面体引擎的输入是「三角形汤」
/// （顶点 + 三角形索引，允许顶点重复、无拓扑）。因此必须先经 OCC 的
/// `BRepMesh_IncrementalMesh` 做**离散化（tessellation）**把曲面折成三角片，
/// 才能喂给引擎。
///
/// 离散精度由 `targetSize` 决定（见 ShapeToPolyData::deflectionFor）：
/// 目标单元尺寸越细，弦高越小，避免粗四面体挂在低精度折线上。
///
/// 与 ShapeLoader 的分工：ShapeLoader 只负责「文件 → TopoDS_Shape」，
/// 本模块负责「TopoDS_Shape → 三角汤」，两者都只在需要 OCC 的调用方使用
///（MainWindow / BRepInputTest）；算法库 mesher 与 TetMeshEngine 保持 OCC 无关。
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
