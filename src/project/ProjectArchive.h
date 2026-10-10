#pragma once
// 文件职责：工程序列化入口：把模型状态、参数、网格和几何源文件组织为 .msp 内容，加载时恢复元数据。

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
     * @return 所需条目和打包都成功才返回 true；失败原因写入 error（非空时）。
     * 网格指针仅在本次同步保存期间借用；调用方须保证数据完整且不被并发修改。
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

    /// 加载元数据并解包；unpackDir 由调用方保活，条目路径从 project.models 拼接。
    /// 这里只恢复工程数据，不创建视口 actor；几何和网格显示由主窗口随后恢复。
    static bool load(
        const QString& packagePath,
        MeshProject& project,
        QTemporaryDir& unpackDir,
        QString* error = nullptr);
};
