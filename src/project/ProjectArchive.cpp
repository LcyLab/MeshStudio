// 文件职责：工程序列化入口：把模型状态、参数、网格和几何源文件组织为 .msp 内容，加载时恢复元数据。
#include "ProjectArchive.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridReader.h>
#include <vtkXMLUnstructuredGridWriter.h>

#include "ProjectPackage.h"

namespace {

constexpr int kFormatVersion = 1;
constexpr int kPreviewWidth = 480;
constexpr int kPreviewHeight = 320;
constexpr qint64 kJsonMaxBytes = 8 * 1024 * 1024;

const char* kProjectMetadataEntry = "project.json";
const char* kPreviewEntry = "preview/preview.png";
const char* kModelsEntry = "state/models.json";
const char* kMeshSettingsEntry = "state/mesh_settings.json";
const char* kMeshReportEntry = "state/mesh_report.json";

/// 存在错误输出指针时写入错误说明。
void setError(QString* error, const QString& text)
{
    if (error)
        *error = text;
}

/// 将 JSON 文档写入临时文件并提交到目标路径。
bool writeJsonFile(const QString& path, const QJsonDocument& document, QString* error)
{
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        setError(error, QCoreApplication::tr("无法写入工程文件：%1").arg(path));
        return false;
    }
    file.write(document.toJson(QJsonDocument::Indented));
    if (!file.commit()) {
        setError(error, QCoreApplication::tr("工程文件写入未完成：%1").arg(path));
        return false;
    }
    return true;
}

/// 读取工程包中的 JSON 条目并检查其对象格式。
bool readJsonEntry(const QString& packagePath, const QString& entry, QJsonDocument& document, QString* error)
{
    QByteArray data;
    if (!ProjectPackage::readFile(packagePath, entry, data, kJsonMaxBytes, error))
        return false;
    document = QJsonDocument::fromJson(data);
    if (!document.isObject()) {
        setError(error, QCoreApplication::tr("工程包 JSON 条目无效：%1").arg(entry));
        return false;
    }
    return true;
}

/// 将模型颜色序列化为 RGB 字段。
QJsonObject colorToJson(const QColor& color)
{
    QJsonObject object;
    object.insert(QStringLiteral("r"), color.red());
    object.insert(QStringLiteral("g"), color.green());
    object.insert(QStringLiteral("b"), color.blue());
    return object;
}

/// 从 RGB 字段还原颜色，缺失时使用备用值。
QColor colorFromJson(const QJsonObject& object, const QColor& fallback)
{
    if (!object.contains(QStringLiteral("r")))
        return fallback;
    return QColor(object.value(QStringLiteral("r")).toInt(),
                  object.value(QStringLiteral("g")).toInt(),
                  object.value(QStringLiteral("b")).toInt());
}

/// 将平移、旋转和缩放参数序列化为数组。
QJsonArray transformToJson(const double transform[7])
{
    QJsonArray array;
    for (int i = 0; i < 7; ++i)
        array.append(transform[i]);
    return array;
}

/// 从数组恢复七个显示变换参数，缺失值使用默认值。
void transformFromJson(const QJsonArray& array, double out[7])
{
    static const double kDefaults[7] = { 0, 0, 0, 0, 0, 0, 1.0 };
    for (int i = 0; i < 7; ++i)
        out[i] = array.size() > i ? array.at(i).toDouble(kDefaults[i]) : kDefaults[i];
}

/// 将工程模型及其文件、外观信息序列化。
QJsonDocument modelsToJson(const QVector<ProjectModelEntry>& models)
{
    QJsonArray array;
    for (const ProjectModelEntry& entry : models) {
        QJsonObject object;
        object.insert(QStringLiteral("name"), entry.displayName);
        object.insert(QStringLiteral("isMesh"), entry.isMesh);
        object.insert(QStringLiteral("isSurfaceMesh"), entry.isSurfaceMesh);
        if (!entry.sourceFile.isEmpty())
            object.insert(QStringLiteral("sourceFile"), entry.sourceFile);
        if (!entry.meshFile.isEmpty())
            object.insert(QStringLiteral("meshFile"), entry.meshFile);
        if (!entry.surfaceFile.isEmpty())
            object.insert(QStringLiteral("surfaceFile"), entry.surfaceFile);
        object.insert(QStringLiteral("color"), colorToJson(entry.color));
        object.insert(QStringLiteral("opacity"), entry.opacity);
        object.insert(QStringLiteral("visible"), entry.visible);
        object.insert(QStringLiteral("transform"), transformToJson(entry.transform));
        array.append(object);
    }
    QJsonObject root;
    root.insert(QStringLiteral("models"), array);
    return QJsonDocument(root);
}

/// 读取工程中的模型列表及显示状态。
bool modelsFromJson(const QJsonDocument& document, QVector<ProjectModelEntry>& models)
{
    models.clear();
    const QJsonArray array = document.object().value(QStringLiteral("models")).toArray();
    for (const QJsonValue& value : array) {
        const QJsonObject object = value.toObject();
        ProjectModelEntry entry;
        entry.displayName = object.value(QStringLiteral("name")).toString();
        entry.isMesh = object.value(QStringLiteral("isMesh")).toBool();
        entry.isSurfaceMesh = object.value(QStringLiteral("isSurfaceMesh")).toBool();
        entry.sourceFile = object.value(QStringLiteral("sourceFile")).toString();
        entry.meshFile = object.value(QStringLiteral("meshFile")).toString();
        entry.surfaceFile = object.value(QStringLiteral("surfaceFile")).toString();
        entry.color = colorFromJson(object.value(QStringLiteral("color")).toObject(),
                                    QColor(0x7F, 0x96, 0xA8));
        entry.opacity = object.value(QStringLiteral("opacity")).toDouble(1.0);
        entry.visible = object.value(QStringLiteral("visible")).toBool(true);
        transformFromJson(object.value(QStringLiteral("transform")).toArray(), entry.transform);
        if (entry.displayName.isEmpty())
            return false;
        models.append(entry);
    }
    return true;
}

/// 序列化体网格参数和表面生成设置。
QJsonDocument meshSettingsToJson(const TetMeshParams& params,
                                 const mesher::SurfaceCriteria& surface)
{
    QJsonObject object;
    object.insert(QStringLiteral("targetSize"), params.targetSize);
    object.insert(QStringLiteral("minSize"), params.minSize);
    object.insert(QStringLiteral("curvatureAngle"), params.curvatureAngle);
    object.insert(QStringLiteral("elementOrder"), params.elementOrder);
    object.insert(QStringLiteral("algo3D"), params.algo3D);
    object.insert(QStringLiteral("optimize"), params.optimize);
    object.insert(QStringLiteral("optimizeThreshold"), params.optimizeThreshold);
    QJsonObject triangle;
    triangle.insert(QStringLiteral("targetSize"),surface.targetSize);
    triangle.insert(QStringLiteral("minSize"),surface.minSize);
    triangle.insert(QStringLiteral("maxSize"),surface.maxSize);
    triangle.insert(QStringLiteral("maxDeviation"),surface.maxDeviation);
    triangle.insert(QStringLiteral("maxNormalAngleDeg"),surface.maxNormalAngleDeg);
    triangle.insert(QStringLiteral("maxRadiusEdgeRatio"),surface.maxRadiusEdgeRatio);
    triangle.insert(QStringLiteral("growthRatio"),surface.growthRatio);
    triangle.insert(QStringLiteral("maxVertices"),surface.maxVertices);
    triangle.insert(QStringLiteral("maxIterations"),surface.maxIterations);
    triangle.insert(QStringLiteral("pointPlacement"),QString::fromLatin1(mesher::surfacePointPlacementName(surface.pointPlacement)));
    triangle.insert(QStringLiteral("crossSmoothCadSplits"),surface.crossSmoothCadSplits);
    QJsonObject algorithm6;
    algorithm6.insert(QStringLiteral("targetSize"),surface.algorithm6.targetSize);
    algorithm6.insert(QStringLiteral("extendBoundarySize"),surface.algorithm6.extendBoundarySize);
    algorithm6.insert(QStringLiteral("smoothingIterations"),surface.algorithm6.smoothingIterations);
    algorithm6.insert(QStringLiteral("maximumNodes"),surface.algorithm6.maximumNodes);
    algorithm6.insert(QStringLiteral("maximumGeometryEvaluations"),surface.algorithm6.maximumGeometryEvaluations);
    triangle.insert(QStringLiteral("algorithm6"),algorithm6);
    object.insert(QStringLiteral("surface"),triangle);
    return QJsonDocument(object);
}

/// 读取表面生成设置，并兼容旧工程参数格式。
mesher::SurfaceCriteria surfaceSettingsFromJson(const QJsonDocument& document)
{
    mesher::SurfaceCriteria result;
    const QJsonObject object=document.object().value(QStringLiteral("surface")).toObject();
    if (object.isEmpty()) {
        const QJsonObject legacy=document.object();
        result.targetSize=legacy.value(QStringLiteral("targetSize")).toDouble(0);
        result.minSize=legacy.value(QStringLiteral("minSize")).toDouble(0);
        result.maxNormalAngleDeg=legacy.value(QStringLiteral("curvatureAngle")).toDouble(30);
        result.maxDeviation=result.targetSize>0?result.targetSize*0.1:0;
        return result;
    }
    result.targetSize=object.value(QStringLiteral("targetSize")).toDouble(0);
    result.minSize=object.value(QStringLiteral("minSize")).toDouble(0);
    // 缺少最大尺寸的旧工程保留 targetSize + 5% 语义，Dialog 显示迁移说明。
    result.maxSize=object.value(QStringLiteral("maxSize")).toDouble(0);
    result.maxDeviation=object.value(QStringLiteral("maxDeviation")).toDouble(0);
    result.maxNormalAngleDeg=object.value(QStringLiteral("maxNormalAngleDeg")).toDouble(30);
    result.maxRadiusEdgeRatio=object.value(QStringLiteral("maxRadiusEdgeRatio")).toDouble(result.maxRadiusEdgeRatio);
    result.growthRatio=object.value(QStringLiteral("growthRatio")).toDouble(result.growthRatio);
    result.maxVertices=object.value(QStringLiteral("maxVertices")).toInt(result.maxVertices);
    result.maxIterations=object.value(QStringLiteral("maxIterations")).toInt(result.maxIterations);
    const auto placement=object.value(QStringLiteral("pointPlacement")).toString(QStringLiteral("legacy"));
    result.pointPlacement=placement==QStringLiteral("frontal")?mesher::SurfacePointPlacement::FrontalDelaunay:
        placement==QStringLiteral("reference")?mesher::SurfacePointPlacement::ReferenceFrontal:
        placement==QStringLiteral("reference-physical")?mesher::SurfacePointPlacement::PhysicalReferenceFrontal:
        placement==QStringLiteral("algorithm6-experimental")?mesher::SurfacePointPlacement::Algorithm6Experimental:
        placement==QStringLiteral("legacy")?mesher::SurfacePointPlacement::LegacyRefinement:static_cast<mesher::SurfacePointPlacement>(-1);
    result.crossSmoothCadSplits=object.value(QStringLiteral("crossSmoothCadSplits")).toBool(false);
    const auto algorithm6=object.value(QStringLiteral("algorithm6")).toObject();
    result.algorithm6.targetSize=algorithm6.value(QStringLiteral("targetSize")).toDouble(result.algorithm6.targetSize);
    result.algorithm6.extendBoundarySize=algorithm6.value(QStringLiteral("extendBoundarySize")).toInt(result.algorithm6.extendBoundarySize);
    result.algorithm6.smoothingIterations=algorithm6.value(QStringLiteral("smoothingIterations")).toInt(result.algorithm6.smoothingIterations);
    result.algorithm6.maximumNodes=algorithm6.value(QStringLiteral("maximumNodes")).toInt(result.algorithm6.maximumNodes);
    result.algorithm6.maximumGeometryEvaluations=algorithm6.value(QStringLiteral("maximumGeometryEvaluations")).toInt(result.algorithm6.maximumGeometryEvaluations);
    return result;
}

/// 从工程 JSON 恢复体网格参数。
TetMeshParams meshSettingsFromJson(const QJsonDocument& document)
{
    TetMeshParams params;
    const QJsonObject object = document.object();
    params.targetSize = object.value(QStringLiteral("targetSize")).toDouble(0.0);
    params.minSize = object.value(QStringLiteral("minSize")).toDouble(0.0);
    params.curvatureAngle = object.value(QStringLiteral("curvatureAngle")).toDouble(30.0);
    params.elementOrder = object.value(QStringLiteral("elementOrder")).toInt(1);
    params.algo3D = object.value(QStringLiteral("algo3D")).toInt(1);
    params.optimize = object.value(QStringLiteral("optimize")).toBool(true);
    params.optimizeThreshold = object.value(QStringLiteral("optimizeThreshold")).toDouble(0.3);
    return params;
}

/// 序列化体网格数量和质量统计。
QJsonDocument meshReportToJson(const TetMeshQualityStats& stats)
{
    QJsonObject object;
    object.insert(QStringLiteral("nodeCount"), static_cast<qint64>(stats.nodeCount));
    object.insert(QStringLiteral("tetCount"), static_cast<qint64>(stats.tetCount));
    object.insert(QStringLiteral("surfaceTriCount"), static_cast<qint64>(stats.surfaceTriCount));
    object.insert(QStringLiteral("quadratic"), stats.quadratic);
    object.insert(QStringLiteral("minSICN"), stats.minSICN);
    object.insert(QStringLiteral("minScaledJacobian"), stats.minScaledJacobian);
    object.insert(QStringLiteral("acceptanceKnown"), stats.acceptanceKnown);
    object.insert(QStringLiteral("qualityAccepted"), stats.qualityAccepted);
    object.insert(QStringLiteral("degenerateCount"), static_cast<qint64>(stats.degenerateCount));
    object.insert(QStringLiteral("sliverCount"), static_cast<qint64>(stats.sliverCount));
    object.insert(QStringLiteral("maxEdgeLength"), stats.maxEdgeLength);
    object.insert(QStringLiteral("worstRadiusEdge"), stats.worstRadiusEdge);
    object.insert(QStringLiteral("minGamma"), stats.minGamma);
    object.insert(QStringLiteral("avgGamma"), stats.avgGamma);
    object.insert(QStringLiteral("minDihedralDeg"), stats.minDihedralDeg);
    object.insert(QStringLiteral("maxDihedralDeg"), stats.maxDihedralDeg);
    object.insert(QStringLiteral("invertedCount"), static_cast<qint64>(stats.invertedCount));
    return QJsonDocument(object);
}

/// 从工程 JSON 恢复体网格统计。
TetMeshQualityStats meshReportFromJson(const QJsonDocument& document)
{
    TetMeshQualityStats stats;
    const QJsonObject object = document.object();
    stats.nodeCount = static_cast<qint64>(object.value(QStringLiteral("nodeCount")).toDouble());
    stats.tetCount = static_cast<qint64>(object.value(QStringLiteral("tetCount")).toDouble());
    stats.surfaceTriCount = static_cast<qint64>(object.value(QStringLiteral("surfaceTriCount")).toDouble());
    stats.quadratic = object.value(QStringLiteral("quadratic")).toBool();
    stats.minSICN = object.value(QStringLiteral("minSICN")).toDouble(1.0);
    stats.minScaledJacobian = object.value(QStringLiteral("minScaledJacobian")).toDouble(stats.minSICN);
    stats.acceptanceKnown = object.value(QStringLiteral("acceptanceKnown")).toBool(false);
    stats.qualityAccepted = stats.acceptanceKnown && object.value(QStringLiteral("qualityAccepted")).toBool(false);
    stats.degenerateCount = static_cast<qint64>(object.value(QStringLiteral("degenerateCount")).toDouble());
    stats.sliverCount = static_cast<qint64>(object.value(QStringLiteral("sliverCount")).toDouble());
    stats.maxEdgeLength = object.value(QStringLiteral("maxEdgeLength")).toDouble();
    stats.worstRadiusEdge = object.value(QStringLiteral("worstRadiusEdge")).toDouble();
    stats.minGamma = object.value(QStringLiteral("minGamma")).toDouble(1.0);
    stats.avgGamma = object.value(QStringLiteral("avgGamma")).toDouble(0.0);
    stats.minDihedralDeg = object.value(QStringLiteral("minDihedralDeg")).toDouble(180.0);
    stats.maxDihedralDeg = object.value(QStringLiteral("maxDihedralDeg")).toDouble(0.0);
    stats.invertedCount = static_cast<qint64>(object.value(QStringLiteral("invertedCount")).toDouble());
    return stats;
}

/// 将截图调整为工程预览图的固定尺寸。
QImage normalizedPreviewImage(const QImage& captured)
{
    if (captured.isNull())
        return {};
    const QSize previewSize(kPreviewWidth, kPreviewHeight);
    return captured.size() == previewSize
        ? captured
        : captured.scaled(previewSize, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
}

/// 从旧工程包读取可复用的预览图。
QImage readFallbackPreviewImage(const QString& fallbackPackagePath)
{
    if (fallbackPackagePath.isEmpty() || !QFileInfo::exists(fallbackPackagePath))
        return {};
    QByteArray data;
    if (!ProjectPackage::readFile(fallbackPackagePath, QString::fromLatin1(kPreviewEntry), data, 16 * 1024 * 1024))
        return {};
    return QImage::fromData(data, "PNG");
}

} // namespace

/// 保存工程。
bool ProjectArchive::save(
    const MeshProject& project,
    const QString& packagePath,
    const QImage& previewImage,
    const QString& fallbackPackagePath,
    const QHash<QString, QString>& geometrySourceFiles,
    const QHash<QString, vtkUnstructuredGrid*>& meshVolumes,
    const QHash<QString, vtkPolyData*>& meshSurfaces,
    QString* error)
{
    QTemporaryDir stagingDir;
    if (!stagingDir.isValid()) {
        setError(error, QCoreApplication::tr("无法创建工程打包临时目录"));
        return false;
    }
    const QDir root(stagingDir.path());

    // 元数据：创建时间继承旧包
    QString createdAt = QDateTime::currentDateTime().toString(Qt::ISODate);
    if (!fallbackPackagePath.isEmpty() && QFileInfo::exists(fallbackPackagePath)) {
        QJsonDocument oldMeta;
        if (readJsonEntry(fallbackPackagePath, QString::fromLatin1(kProjectMetadataEntry), oldMeta, nullptr))
            createdAt = oldMeta.object().value(QStringLiteral("createdAt")).toString(createdAt);
    }
    QJsonObject meta;
    meta.insert(QStringLiteral("app"), QStringLiteral("MeshStudio"));
    meta.insert(QStringLiteral("formatVersion"), kFormatVersion);
    meta.insert(QStringLiteral("projectName"), project.projectName);
    meta.insert(QStringLiteral("createdAt"), createdAt);
    if (!writeJsonFile(root.filePath(QString::fromLatin1(kProjectMetadataEntry)),
                       QJsonDocument(meta), error))
        return false;

    // 预览图：本次捕获优先，缺失时复用旧包
    QImage preview = normalizedPreviewImage(previewImage);
    if (preview.isNull())
        preview = normalizedPreviewImage(readFallbackPreviewImage(fallbackPackagePath));
    if (!preview.isNull()) {
        if (!root.mkpath(QStringLiteral("preview"))) {
            setError(error, QCoreApplication::tr("无法创建预览目录"));
            return false;
        }
        if (!preview.save(root.filePath(QString::fromLatin1(kPreviewEntry)), "PNG")) {
            setError(error, QCoreApplication::tr("无法写入预览图"));
            return false;
        }
    }

    // 状态 JSON
    if (!root.mkpath(QStringLiteral("state"))) {
        setError(error, QCoreApplication::tr("无法创建状态目录"));
        return false;
    }
    if (!writeJsonFile(root.filePath(QString::fromLatin1(kModelsEntry)), modelsToJson(project.models), error))
        return false;
    if (!writeJsonFile(root.filePath(QString::fromLatin1(kMeshSettingsEntry)),
                       meshSettingsToJson(project.meshParams,project.surfaceCriteria), error))
        return false;
    if (project.hasMeshStats &&
        !writeJsonFile(root.filePath(QString::fromLatin1(kMeshReportEntry)), meshReportToJson(project.meshStats), error))
        return false;

    // 几何源文件按原格式拷入（自包含）
    for (auto it = geometrySourceFiles.constBegin(); it != geometrySourceFiles.constEnd(); ++it) {
        const QString targetPath = root.filePath(it.key());
        QFileInfo targetInfo(targetPath);
        if (!root.mkpath(targetInfo.path())) {
            setError(error, QCoreApplication::tr("无法创建几何目录"));
            return false;
        }
        QFile::remove(targetPath);
        if (!QFile::copy(it.value(), targetPath)) {
            setError(error, QCoreApplication::tr("无法嵌入几何文件：%1").arg(it.value()));
            return false;
        }
    }

    // 体网格写 .vtu（二进制 XML）
    for (auto it = meshVolumes.constBegin(); it != meshVolumes.constEnd(); ++it) {
        const QString targetPath = root.filePath(it.key());
        QFileInfo targetInfo(targetPath);
        if (!root.mkpath(targetInfo.path())) {
            setError(error, QCoreApplication::tr("无法创建网格目录"));
            return false;
        }
        vtkNew<vtkXMLUnstructuredGridWriter> writer;
        writer->SetFileName(targetPath.toLocal8Bit().constData());
        writer->SetInputData(it.value());
        writer->SetDataModeToBinary();
        if (!writer->Write()) {
            setError(error, QCoreApplication::tr("无法写入体网格：%1").arg(it.key()));
            return false;
        }
    }

    // 网格外表面写 .vtp（线性三角形，恢复显示用）
    for (auto it = meshSurfaces.constBegin(); it != meshSurfaces.constEnd(); ++it) {
        const QString targetPath = root.filePath(it.key());
        QFileInfo targetInfo(targetPath);
        if (!root.mkpath(targetInfo.path())) {
            setError(error, QCoreApplication::tr("无法创建网格目录"));
            return false;
        }
        vtkNew<vtkXMLPolyDataWriter> writer;
        writer->SetFileName(targetPath.toLocal8Bit().constData());
        writer->SetInputData(it.value());
        writer->SetDataModeToBinary();
        if (!writer->Write()) {
            setError(error, QCoreApplication::tr("无法写入网格外表面：%1").arg(it.key()));
            return false;
        }
    }

    return ProjectPackage::packDirectory(stagingDir.path(), packagePath, error);
}

/// 加载元数据并解包；unpackDir 由调用方保活，条目路径从 project.models 拼接。
bool ProjectArchive::load(
    const QString& packagePath,
    MeshProject& project,
    QTemporaryDir& unpackDir,
    QString* error)
{
    project = MeshProject{};

    if (!ProjectPackage::unpackToTemporaryDirectory(packagePath, unpackDir, error))
        return false;

    QJsonDocument meta;
    if (!readJsonEntry(packagePath, QString::fromLatin1(kProjectMetadataEntry), meta, error))
        return false;
    project.projectName = meta.object().value(QStringLiteral("projectName")).toString();

    QJsonDocument models;
    if (!readJsonEntry(packagePath, QString::fromLatin1(kModelsEntry), models, error))
        return false;
    if (!modelsFromJson(models, project.models)) {
        setError(error, QCoreApplication::tr("工程包模型列表无效"));
        return false;
    }

    QJsonDocument settings;
    if (readJsonEntry(packagePath, QString::fromLatin1(kMeshSettingsEntry), settings, nullptr)) {
        project.meshParams = meshSettingsFromJson(settings);
        project.surfaceCriteria = surfaceSettingsFromJson(settings);
    }

    QJsonDocument report;
    if (readJsonEntry(packagePath, QString::fromLatin1(kMeshReportEntry), report, nullptr)) {
        project.hasMeshStats = true;
        project.meshStats = meshReportFromJson(report);
    }

    return true;
}
