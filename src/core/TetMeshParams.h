#pragma once

#include <QString>

/// 四面体网格生成参数：参数面板 → 自研网格引擎的传递结构
struct TetMeshParams
{
    QString geometryFilePath;       ///< 输入文件（v1 仅支持 STL 表面网格）
    /// 全局目标尺寸 → 细化目标边长 maxEdgeLength。
    /// 语义：>0 用该值；<=0（含默认 0）表示**自动**，由引擎按包围盒对角线
    /// 的 5% 计算（见 TetMeshEngine::run）。UI 若要表示"不限制"，需传一个
    /// 足够大的正数，不能用 0（0 已被"自动"占用）。
    double targetSize = 0.0;
    double minSize = 0.0;           ///< 最小网格尺寸 → 细化下界（0 = 不限制）
    double curvatureAngle = 30.0;   ///< 曲率细分角度(°)（v1 未实现，预留）
    int elementOrder = 1;           ///< 单元阶次：1=T4（线性）/ 2=T10（二次，v1 未实现）
    int algo3D = 1;                 ///< 3D 算法（v1 仅自研 Delaunay 细化，字段保留兼容）
    bool optimize = true;           ///< 生成后质量优化 → Laplacian 平滑
    double optimizeThreshold = 0.3; ///< 优化触发阈值（v1 未实现，预留）
};
