#pragma once

#include <QColor>
#include <QVector>

#include "app/meshing/TetMeshEngine.h"
#include "app/meshing/TetMeshParams.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"

/// 工程包内单个模型条目（.msp 的 state/models.json 数组元素）
struct ProjectModelEntry
{
    QString displayName;       ///< 模型树显示名（几何=文件名；网格=xxx（四面体网格））
    bool isMesh = false;       ///< true=四面体网格模型 / false=几何模型
    bool isSurfaceMesh = false; ///< true=仅有三角面片，无四面体体网格
    QString sourceFile;        ///< 几何：包内相对路径 geometry/xxx
    QString meshFile;          ///< 网格：包内相对路径 mesh/xxx.vtu
    QString surfaceFile;       ///< 网格：包内相对路径 mesh/xxx.vtp（外表面）
    QColor color{0x7F, 0x96, 0xA8};
    double opacity = 1.0;
    bool visible = true;
    double transform[7] = { 0, 0, 0, 0, 0, 0, 1.0 }; ///< 平移/旋转/缩放
};

/// MeshStudio 工程数据模型（对齐 EMStudio 的 Document，轻量版）
struct MeshProject
{
    QString projectName;
    QVector<ProjectModelEntry> models;
    TetMeshParams meshParams;          ///< 最近一次网格参数
    mesher::SurfaceCriteria surfaceCriteria; ///< 表面三角生成参数
    bool hasMeshStats = false;         ///< 是否携带网格质量统计
    TetMeshQualityStats meshStats;

    /// 判断工程是否具有有效名称。
    bool isValid() const { return !projectName.isEmpty(); }
};
