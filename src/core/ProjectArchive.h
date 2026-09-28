#pragma once

#include <QHash>
#include <QImage>
#include <QString>
#include <QTemporaryDir>
#include <vtkSmartPointer.h>

#include "MeshProject.h"

class vtkPolyData;
class vtkUnstructuredGrid;

/**
 * @class ProjectArchive
 * @brief MeshStudio 工程保存/打开入口（.msp 单文件工程包）
 *
 * 包内结构（ZIP，由 ProjectPackage 打包）：
 *   project.json               工程元数据
 *   preview/preview.png        视口预览图
 *   state/models.json          模型列表（颜色/透明度/可见性/变换）
 *   state/mesh_settings.json   网格参数
 *   state/mesh_report.json     网格质量统计（有网格时）
 *   geometry/<文件名>          几何源文件按原格式嵌入（自包含）
 *   mesh/<名称>.vtu            体四面体网格（已生成时，二进制 XML）
 *   mesh/<名称>.vtp            网格外表面（线性三角形，供恢复显示）
 */
class ProjectArchive
{
public:
    /**
     * @brief 保存工程
     * @param geometrySourceFiles 包内相对路径(geometry/...) → 磁盘上的几何源文件
     * @param meshVolumes 包内相对路径(mesh/....vtu) → 体网格数据
     * @param meshSurfaces 包内相对路径(mesh/....vtp) → 网格外表面
     * @param fallbackPackagePath 保存前的旧包路径：预览图缺失时复用旧图、创建时间继承
     */
    static bool save(
        const MeshProject& project,
        const QString& packagePath,
        const QImage& previewImage,
        const QString& fallbackPackagePath,
        const QHash<QString, QString>& geometrySourceFiles,
        const QHash<QString, vtkUnstructuredGrid*>& meshVolumes,
        const QHash<QString, vtkPolyData*>& meshSurfaces,
        QString* error = nullptr);

    /// 加载工程；unpackDir 由调用方持有保活，条目路径从 project.models 拼接
    static bool load(
        const QString& packagePath,
        MeshProject& project,
        QTemporaryDir& unpackDir,
        QString* error = nullptr);
};
