// 文件职责：VTK 视口管理：后台加载几何数据，GUI 线程创建渲染对象，并管理模型显示、变换和工程快照。
#include "ViewportWidget.h"

#include "cad/import/ShapeLoader.h"
#include "cad/import/ShapeToPolyData.h"

#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFileInfo>
#include <QMimeData>
#include <QSet>
#include <QUrl>
#include <QtConcurrent/QtConcurrentRun>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCornerAnnotation.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkIdList.h>
#include <vtkIdTypeArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPlane.h>
#include <vtkTransform.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSTLReader.h>
#include <vtkTextProperty.h>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <algorithm>

namespace {

// 默认调色板：多模型导入时自动区分色相
const QColor kModelPalette[] = {
    QColor(0x7F, 0x96, 0xA8), // 钢蓝
    QColor(0x5E, 0x9C, 0x8F), // 青绿
    QColor(0xC0, 0x85, 0x52), // 琥珀
    QColor(0x8E, 0x7C, 0xB0), // 紫灰
    QColor(0xB0, 0x65, 0x65), // 砖红
    QColor(0x6B, 0x8E, 0xAE), // 湖蓝
};

/// 根据后缀判断输入是否为 STL 三角表面。
bool isStlFile(const QString& path)
{
    return QFileInfo(path).suffix().compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0;
}

// 在工作线程执行: 只做数据对象(OCC/VTK 数据), 不碰渲染
ViewportLoadResult computeLoad(const QString& filePath)
{
    ViewportLoadResult result;

    if (isStlFile(filePath)) {
        auto reader = vtkSmartPointer<vtkSTLReader>::New();
        reader->SetFileName(filePath.toLocal8Bit().constData());
        reader->Update();
        result.surface = reader->GetOutput();
        if (!result.surface || result.surface->GetNumberOfPoints() == 0) {
            result.error = QObject::tr("STL 文件为空或无法解析: %1").arg(filePath);
            return result;
        }
        result.faces = int(result.surface->GetNumberOfCells()); // 三角形数
        result.ok = true;
        return result;
    }

    TopoDS_Shape shape;
    QString error;
    if (!ShapeLoader::load(filePath, shape, &error)) {
        result.error = error;
        return result;
    }

    TopTools_IndexedMapOfShape solidMap, faceMap;
    TopExp::MapShapes(shape, TopAbs_SOLID, solidMap);
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

    result.surface = ShapeToPolyData::surfaceMesh(shape);
    result.edgeLines = ShapeToPolyData::edgeLines(shape);
    if (!result.surface || result.surface->GetNumberOfPoints() == 0) {
        result.error = QObject::tr("几何中没有可显示的面: %1").arg(filePath);
        return result;
    }
    result.solids = solidMap.Extent();
    result.faces = faceMap.Extent();
    result.edges = result.edgeLines ? int(result.edgeLines->GetNumberOfLines()) : 0;
    result.ok = true;
    return result;
}

} // namespace

/// 初始化 VTK 渲染场景及后台几何加载观察器。
ViewportWidget::ViewportWidget(QWidget* parent)
    : QVTKOpenGLNativeWidget(parent)
{
    setAcceptDrops(true);
    setFormat(QVTKOpenGLNativeWidget::defaultFormat());

    m_renderWindow->AddRenderer(m_renderer);
    setRenderWindow(m_renderWindow);

    vtkNew<vtkInteractorStyleTrackballCamera> style;
    interactor()->SetInteractorStyle(style);

    buildScene();

    connect(&m_loadWatcher, &QFutureWatcher<ViewportLoadResult>::finished,
            this, &ViewportWidget::onLoadFinished);
}

/// 建立渲染器、坐标辅助和默认交互环境。
void ViewportWidget::buildScene()
{
    // 透明模型渲染: 深度剥离要求关闭多重采样(MSAA), 以 FXAA 替代抗锯齿
    m_renderWindow->SetMultiSamples(0);
    m_renderer->SetUseFXAA(true);
    m_renderWindow->SetAlphaBitPlanes(1);
    m_renderer->SetUseDepthPeeling(1);
    m_renderer->SetOcclusionRatio(0.1);
    m_renderer->SetMaximumNumberOfPeels(6);

    // 视口背景：带一丝紫调的浅灰渐变，与深茄紫主题同一色温
    m_renderer->SetBackground(0.965, 0.961, 0.969);   // #F6F5F7
    m_renderer->SetBackground2(0.902, 0.886, 0.914);  // #E6E2E9
    m_renderer->GradientBackgroundOn();

    // 地面参考网格（随模型尺寸适配）
    m_gridSource->SetOrigin(-2.5, -2.5, 0.0);
    m_gridSource->SetPoint1(2.5, -2.5, 0.0);
    m_gridSource->SetPoint2(-2.5, 2.5, 0.0);
    m_gridSource->SetXResolution(25);
    m_gridSource->SetYResolution(25);

    vtkNew<vtkPolyDataMapper> gridMapper;
    gridMapper->SetInputConnection(m_gridSource->GetOutputPort());

    m_gridActor = vtkSmartPointer<vtkActor>::New();
    m_gridActor->SetMapper(gridMapper);
    m_gridActor->GetProperty()->SetRepresentationToWireframe();
    m_gridActor->GetProperty()->SetColor(0.635, 0.615, 0.655);
    m_gridActor->GetProperty()->SetLighting(false);
    m_gridActor->SetPickable(false);
    m_renderer->AddActor(m_gridActor);

    // 全局剖切面
    m_clipPlane = vtkSmartPointer<vtkPlane>::New();

    // 左下角坐标轴指示器
    vtkNew<vtkAxesActor> axes;
    axes->SetShaftTypeToCylinder();
    axes->SetCylinderRadius(0.02);
    m_axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_axesWidget->SetOrientationMarker(axes);
    m_axesWidget->SetInteractor(interactor());
    m_axesWidget->SetViewport(0.0, 0.0, 0.18, 0.18);
    m_axesWidget->SetZoom(1.6);
    // 先 Enabled 再关交互：vtkOrientationMarkerWidget 要求 interactor+enabled 就绪后才能改交互
    m_axesWidget->EnabledOn();
    m_axesWidget->InteractiveOff();

    // 视口保持干净：不放应用名与提示文字
    m_annotation->GetTextProperty()->SetColor(0.545, 0.525, 0.565);
    m_annotation->GetTextProperty()->SetFontSize(11);
    m_annotation->GetTextProperty()->ItalicOff();
    m_annotation->GetTextProperty()->BoldOff();
    m_renderer->AddViewProp(m_annotation);

    applyDefaultCamera();
    m_renderWindow->Render();
    qDebug("MSTAGE: depth peeling used=%d (multisamples=%d)",
           m_renderer->GetLastRenderingUsedDepthPeeling(),
           m_renderWindow->GetMultiSamples());
}

/// 按场景范围恢复默认相机并重绘。
void ViewportWidget::applyDefaultCamera()
{
    // 先给定一个非平行的观察方向，ResetCamera 只适配距离不改变朝向
    if (auto* cam = m_renderer->GetActiveCamera()) {
        cam->SetPosition(6.0, -7.0, 4.0);
        cam->SetFocalPoint(0.0, 0.0, 0.3);
        cam->SetViewUp(0.0, 0.0, 1.0);
    }
    // 只按模型包围盒适配：ResetCamera() 会把地面网格(1.6 倍模型)算进去，模型被缩得过小
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
    if (sceneBounds(lo, hi)) {
        const double bounds[6] = { lo[0], hi[0], lo[1], hi[1], lo[2], hi[2] };
        m_renderer->ResetCamera(bounds);
    } else {
        m_renderer->ResetCamera();
    }
    m_renderer->ResetCameraClippingRange();
    m_renderWindow->Render();
}

/// 合并当前场景模型的三维包围范围。
bool ViewportWidget::sceneBounds(double lo[3], double hi[3])
{
    bool any = false;
    for (const LoadedGeometryModel& model : m_models) {
        if (!model.visible || !model.surfaceActor)
            continue;
        double b[6];
        model.surfaceActor->GetBounds(b);
        if (!any) {
            for (int i = 0; i < 3; ++i) {
                lo[i] = b[2 * i];
                hi[i] = b[2 * i + 1];
            }
            any = true;
        } else {
            for (int i = 0; i < 3; ++i) {
                lo[i] = std::min(lo[i], b[2 * i]);
                hi[i] = std::max(hi[i], b[2 * i + 1]);
            }
        }
    }
    return any;
}

/// 按模型范围调整地面参考网格。
void ViewportWidget::fitGroundGrid()
{
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
    const bool any = sceneBounds(lo, hi);

    if (!any) {
        m_gridSource->SetOrigin(-2.5, -2.5, 0.0);
        m_gridSource->SetPoint1(2.5, -2.5, 0.0);
        m_gridSource->SetPoint2(-2.5, 2.5, 0.0);
        m_gridSource->Modified();
        return;
    }

    const double dx = hi[0] - lo[0];
    const double dy = hi[1] - lo[1];
    const double half = 0.75 * std::max(dx, dy) * 1.6 + 1e-6;
    const double cx = 0.5 * (lo[0] + hi[0]);
    const double cy = 0.5 * (lo[1] + hi[1]);
    const double z = lo[2] - std::max(1e-3 * half, half * 0.01);
    m_gridSource->SetOrigin(cx - half, cy - half, z);
    m_gridSource->SetPoint1(cx + half, cy - half, z);
    m_gridSource->SetPoint2(cx - half, cy + half, z);
    m_gridSource->Modified();
}

/// 按模型编号查找视口持有的模型。
LoadedGeometryModel* ViewportWidget::findModel(int modelId)
{
    for (LoadedGeometryModel& model : m_models) {
        if (model.id == modelId)
            return &model;
    }
    return nullptr;
}

/// 按模型编号查找视口持有的模型。
const LoadedGeometryModel* ViewportWidget::findModel(int modelId) const
{
    for (const LoadedGeometryModel& model : m_models) {
        if (model.id == modelId)
            return &model;
    }
    return nullptr;
}

/// 复制模型原始三角表面，供后台计算独立使用。
vtkSmartPointer<vtkPolyData> ViewportWidget::surfaceMeshSnapshot(int modelId) const
{
    const auto* model=findModel(modelId);
    if(!model || !model->isSurfaceMesh || !model->surface) return {};
    auto snapshot=vtkSmartPointer<vtkPolyData>::New();
    snapshot->DeepCopy(model->surface);
    return snapshot;
}

/// 读取指定模型的显示名称、文件和范围摘要。
ModelSummary ViewportWidget::modelSummary(int modelId) const
{
    ModelSummary summary;
    const LoadedGeometryModel* model = findModel(modelId);
    if (!model)
        return summary;
    summary.valid = true;
    summary.isMesh = model->isMesh;
    summary.isSurfaceMesh = model->isSurfaceMesh;
    summary.name = model->name;
    summary.filePath = model->filePath;
    summary.solids = model->solids;
    summary.faces = model->faces;
    summary.edges = model->edges;
    summary.visible = model->visible;
    // 加载结果未携带已核对的单位元数据；文件扩展名不能证明坐标单位。
    summary.unitHint.clear();

    if (!model->surfaceActor)
        return summary;
    double b[6];
    model->surfaceActor->GetBounds(b);
    for (int i = 0; i < 6; ++i)
        summary.range[i] = b[i];
    summary.rangeValid = true;
    return summary;
}

/// 将生成的网格及其 VTK 数据加入视口。
int ViewportWidget::addMeshModel(vtkPolyData* meshSurface, const QString& name,
                                  bool surfaceOnly)
{
    if (!meshSurface || meshSurface->GetNumberOfPoints() == 0)
        return -1;

    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(meshSurface);
    normals->SetFeatureAngle(30.0);
    if (surfaceOnly) {
        normals->SplittingOff();
        normals->ConsistencyOff();
    }

    vtkNew<vtkPolyDataMapper> meshMapper;
    meshMapper->SetInputConnection(normals->GetOutputPort());

    LoadedGeometryModel model;
    model.id = m_nextModelId++;
    model.name = name;
    model.isMesh = true;
    model.isSurfaceMesh = surfaceOnly;
    model.solids = 0;
    model.faces = int(meshSurface->GetNumberOfCells());
    model.edges = 0;
    model.color = kModelPalette[(model.id - 1)
                                % (sizeof(kModelPalette) / sizeof(kModelPalette[0]))];

    vtkSmartPointer<vtkActor> surface = vtkSmartPointer<vtkActor>::New();
    surface->SetMapper(meshMapper);
    surface->GetProperty()->SetColor(model.color.redF(), model.color.greenF(), model.color.blueF());
    surface->GetProperty()->SetAmbient(0.15);
    surface->GetProperty()->SetDiffuse(0.85);
    surface->GetProperty()->SetSpecular(0.25);
    surface->GetProperty()->SetSpecularPower(40);
    // 画出三角形边线以显示生成的网格。
    surface->GetProperty()->SetEdgeVisibility(1);
    surface->GetProperty()->SetEdgeColor(0.16, 0.18, 0.21);
    surface->GetProperty()->SetLineWidth(0.6);
    m_renderer->AddActor(surface);

    model.surface = meshSurface;
    model.edgeLines = nullptr;
    model.surfaceActor = surface; // 必须赋值：显示模式/变换/裁剪/显隐判定都依赖它
    model.edgeActor = nullptr;
    applyDisplayModeToModel(model);
    applyTransformToModel(model);
    if (m_clipping)
        applyClippingToModel(model, true);
    m_models.push_back(model);

    fitGroundGrid();
    if (m_clipping)
        updateClippingPlane();
    resetView();
    emit modelLoaded(model.id, model.name, model.solids, model.faces, model.edges);
    return model.id;
}

/// 校验文件格式并启动或排队处理几何加载。
void ViewportWidget::loadFile(const QString& filePath)
{
    if (!isStlFile(filePath) && !ShapeLoader::isSupported(filePath)) {
        emit loadFailed(filePath, tr("不支持的文件格式: %1")
                                     .arg(QFileInfo(filePath).suffix()));
        return;
    }
    // 正在加载时排队，导入完成自动接着处理
    if (m_loading) {
        m_loadQueue.append(filePath);
        return;
    }
    startLoad(filePath);
}

/// 提交一次后台文件读取任务。
void ViewportWidget::startLoad(const QString& filePath)
{
    // 后台只准备数据对象；完成通知返回 GUI 线程后再创建和挂接渲染对象。
    m_loading = true;
    m_pendingPath = filePath;
    emit loadStarted();
    m_loadWatcher.setFuture(
        QtConcurrent::run([path = filePath]() { return computeLoad(path); }));
}

/// 接收加载结果，创建显示模型并继续处理队列。
void ViewportWidget::onLoadFinished()
{
    m_loading = false;
    const ViewportLoadResult result = m_loadWatcher.result();
    if (!result.ok) {
        emit loadFailed(m_pendingPath, result.error);
        // 失败不能中断排队中的后续文件
        if (!m_loadQueue.isEmpty())
            startLoad(m_loadQueue.takeFirst());
        return;
    }

    // 已回到视口线程：法线过滤器、mapper、actor 和 renderer 的连接在这里建立。
    vtkNew<vtkPolyDataNormals> normals;
    normals->SetInputData(result.surface);
    normals->SetFeatureAngle(30.0);

    vtkNew<vtkPolyDataMapper> surfaceMapper;
    surfaceMapper->SetInputConnection(normals->GetOutputPort());

    LoadedGeometryModel model;
    model.id = m_nextModelId++;
    model.name = QFileInfo(m_pendingPath).fileName();
    model.filePath = m_pendingPath;
    model.solids = result.solids;
    model.faces = result.faces;
    model.edges = result.edges;
    model.color = kModelPalette[(model.id - 1)
                                % (sizeof(kModelPalette) / sizeof(kModelPalette[0]))];

    vtkSmartPointer<vtkActor> surface = vtkSmartPointer<vtkActor>::New();
    surface->SetMapper(surfaceMapper);
    surface->GetProperty()->SetColor(model.color.redF(), model.color.greenF(), model.color.blueF());
    surface->GetProperty()->SetAmbient(0.15);
    surface->GetProperty()->SetDiffuse(0.85);
    surface->GetProperty()->SetSpecular(0.25);
    surface->GetProperty()->SetSpecularPower(40);

    vtkSmartPointer<vtkActor> edges;
    if (result.edgeLines && result.edgeLines->GetNumberOfLines() > 0) {
        vtkNew<vtkPolyDataMapper> edgeMapper;
        edgeMapper->SetInputData(result.edgeLines);

        edges = vtkSmartPointer<vtkActor>::New();
        edges->SetMapper(edgeMapper);
        edges->GetProperty()->SetColor(model.color.darker(160).redF(), model.color.darker(160).greenF(), model.color.darker(160).blueF());
        edges->GetProperty()->SetLineWidth(1.5);
        edges->GetProperty()->SetLighting(false);
        m_renderer->AddActor(edges);
    }
    m_renderer->AddActor(surface);

    model.surface = result.surface;
    model.edgeLines = result.edgeLines;
    model.surfaceActor = surface;
    model.edgeActor = edges;
    applyDisplayModeToModel(model);
    applyTransformToModel(model);
    if (m_clipping)
        applyClippingToModel(model, true);
    m_models.push_back(model);

    fitGroundGrid();
    // 剖切开启时新模型的边界要参与剖切面定位，否则剖切面停留在默认位置
    if (m_clipping)
        updateClippingPlane();
    resetView();
    emit modelLoaded(model.id, model.name, model.solids, model.faces, model.edges);

    if (!m_loadQueue.isEmpty())
        startLoad(m_loadQueue.takeFirst());
}

/// 修改模型显示颜色并重绘视口。
void ViewportWidget::setModelColor(int modelId, const QColor& color)
{
    if (LoadedGeometryModel* model = findModel(modelId)) {
        model->color = color;
        if (model->surfaceActor)
            model->surfaceActor->GetProperty()->SetColor(color.redF(), color.greenF(), color.blueF());
        if (model->edgeActor)
            model->edgeActor->GetProperty()->SetColor(color.darker(160).redF(), color.darker(160).greenF(), color.darker(160).blueF());
        m_renderWindow->Render();
    }
}

/// 查询指定模型当前的显示颜色。
QColor ViewportWidget::modelColor(int modelId) const
{
    const LoadedGeometryModel* model = findModel(modelId);
    return model ? model->color : QColor();
}

/// 返回当前加载的模型数量。
int ViewportWidget::modelCount() const
{
    return m_models.size();
}

/// 收集模型元数据、几何源路径和网格数据供工程保存。
void ViewportWidget::collectProjectModels(QVector<ProjectModelEntry>& entries,
                                          QHash<QString, QString>& sourceFiles,
                                          QHash<QString, vtkPolyData*>& meshSurfaces) const
{
    entries.clear();
    sourceFiles.clear();
    meshSurfaces.clear();
    QSet<QString> usedNames;
    for (const LoadedGeometryModel& model : m_models) {
        ProjectModelEntry entry;
        // 同名模型去重：包内文件名/显示名必须唯一，否则恢复匹配会错位
        QString name = model.name;
        int copy = 2;
        while (usedNames.contains(name)) {
            const QFileInfo info(model.name);
            name = QStringLiteral("%1(%2).%3").arg(info.completeBaseName())
                       .arg(copy++)
                       .arg(info.suffix());
        }
        usedNames.insert(name);
        entry.displayName = name;
        entry.isMesh = model.isMesh;
        entry.isSurfaceMesh = model.isSurfaceMesh;
        entry.sourceFile = model.isMesh ? QString() : QStringLiteral("geometry/") + name;
        entry.meshFile = model.isMesh && !model.isSurfaceMesh
            ? QStringLiteral("mesh/") + name + QStringLiteral(".vtu") : QString();
        entry.surfaceFile = model.isMesh ? QStringLiteral("mesh/") + name + QStringLiteral(".vtp") : QString();
        entry.color = model.color;
        entry.opacity = model.opacity;
        entry.visible = model.visible;
        for (int i = 0; i < 7; ++i)
            entry.transform[i] = model.transform[i];
        entries.append(entry);
        if (model.isMesh && model.surface) meshSurfaces.insert(entry.surfaceFile,model.surface);
        if (!model.isMesh && !model.filePath.isEmpty())
            sourceFiles.insert(entry.sourceFile, model.filePath);
    }
}

/// 移除全部模型的渲染对象并清空模型状态。
void ViewportWidget::clearModels()
{
    for (LoadedGeometryModel& model : m_models) {
        if (model.surfaceActor)
            m_renderer->RemoveActor(model.surfaceActor);
        if (model.edgeActor)
            m_renderer->RemoveActor(model.edgeActor);
        emit modelRemoved(model.id);
    }
    m_models.clear();
    m_renderWindow->Render();
}

/// 修改模型不透明度并重绘视口。
void ViewportWidget::setModelOpacity(int modelId, double opacity)
{
    qDebug("MSTAGE: setModelOpacity id=%d opacity=%.2f peeling=%d",
           modelId, opacity, m_renderer->GetLastRenderingUsedDepthPeeling());
    opacity = qBound(0.0, opacity, 1.0);
    if (LoadedGeometryModel* model = findModel(modelId)) {
        model->opacity = opacity;
        if (model->surfaceActor)
            model->surfaceActor->GetProperty()->SetOpacity(opacity);
        m_renderWindow->Render();
    }
}

/// 查询指定模型当前的不透明度。
double ViewportWidget::modelOpacity(int modelId) const
{
    const LoadedGeometryModel* model = findModel(modelId);
    return model ? model->opacity : 1.0;
}

/// 按裁剪开关为模型的显示映射器设置裁剪平面。
void ViewportWidget::applyClippingToModel(LoadedGeometryModel& model, bool on)
{
    auto apply = [this, on](vtkActor* actor) {
        if (!actor || !actor->GetMapper())
            return;
        actor->GetMapper()->RemoveAllClippingPlanes();
        if (on && m_clipPlane)
            actor->GetMapper()->AddClippingPlane(m_clipPlane);
    };
    apply(model.surfaceActor);
    apply(model.edgeActor);
}

/// 启用或关闭显示裁剪平面。
void ViewportWidget::setClippingEnabled(bool on)
{
    m_clipping = on;
    for (LoadedGeometryModel& model : m_models)
        applyClippingToModel(model, on);
    updateClippingPlane();
}

/// 选择显示裁剪平面的轴向。
void ViewportWidget::setClippingAxis(int axis)
{
    m_clipAxis = qBound(0, axis, 2);
    updateClippingPlane();
}

/// 设置裁剪平面在场景范围内的相对位置。
void ViewportWidget::setClippingPosition(double position)
{
    m_clipPosition = qBound(0.0, position, 1.0);
    updateClippingPlane();
}

/// 反转裁剪平面保留的一侧。
void ViewportWidget::flipClipping()
{
    m_clipFlipped = !m_clipFlipped;
    updateClippingPlane();
}

/// 按场景范围、方向和相对位置更新裁剪平面。
void ViewportWidget::updateClippingPlane()
{
    if (!m_clipPlane)
        return;
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
    const bool any = sceneBounds(lo, hi);

    if (any) {
        double origin[3] = { (lo[0] + hi[0]) / 2.0,
                             (lo[1] + hi[1]) / 2.0,
                             (lo[2] + hi[2]) / 2.0 };
        origin[m_clipAxis] = lo[m_clipAxis]
            + (hi[m_clipAxis] - lo[m_clipAxis]) * m_clipPosition;
        double normal[3] = { 0, 0, 0 };
        normal[m_clipAxis] = m_clipFlipped ? 1.0 : -1.0;
        m_clipPlane->SetNormal(normal);
        m_clipPlane->SetOrigin(origin);
    }
    if (m_clipping)
        m_renderWindow->Render();
}

/// 将缩放、旋转和平移应用于显示对象。
void ViewportWidget::applyTransformToModel(LoadedGeometryModel& model)
{
    // 变换作用于 actor；原 CAD 和网格坐标保持原样，计算快照不受显示变换影响。
    vtkNew<vtkTransform> transform;
    // vtkTransform 默认 PreMultiply（后加的先作用），必须切到 PostMultiply
    // 才能得到「先缩放旋转（绕模型自身原点）、最后按世界坐标平移」的预期顺序
    const double s = model.transform[6];
    transform->PostMultiply();
    transform->Scale(s, s, s);
    transform->RotateX(model.transform[3]);
    transform->RotateY(model.transform[4]);
    transform->RotateZ(model.transform[5]);
    transform->Translate(model.transform[0], model.transform[1], model.transform[2]);
    if (model.surfaceActor)
        model.surfaceActor->SetUserTransform(transform);
    if (model.edgeActor)
        model.edgeActor->SetUserTransform(transform);
}

/// 保存指定模型的显示变换并刷新渲染对象。
void ViewportWidget::setModelTransform(int modelId, double tx, double ty, double tz,
                                       double rx, double ry, double rz, double scale)
{
    LoadedGeometryModel* model = findModel(modelId);
    if (!model)
        return;
    const double values[7] = { tx, ty, tz, rx, ry, rz, scale };
    for (int i = 0; i < 7; ++i)
        model->transform[i] = values[i];
    applyTransformToModel(*model);
    m_renderWindow->Render();
}

/// 将指定模型的显示变换恢复为单位变换。
void ViewportWidget::resetModelTransform(int modelId)
{
    setModelTransform(modelId, 0, 0, 0, 0, 0, 0, 1.0);
}

/// 读取模型当前的七个显示变换参数。
bool ViewportWidget::modelTransformValues(int modelId, double out[7]) const
{
    const LoadedGeometryModel* model = findModel(modelId);
    if (!model)
        return false;
    for (int i = 0; i < 7; ++i)
        out[i] = model->transform[i];
    return true;
}

/// 更新模型表面和边线的可见性。
void ViewportWidget::setModelVisible(int modelId, bool visible)
{
    if (LoadedGeometryModel* model = findModel(modelId)) {
        model->visible = visible;
        if (model->surfaceActor)
            model->surfaceActor->SetVisibility(visible ? 1 : 0);
        if (model->edgeActor)
            model->edgeActor->SetVisibility(visible ? 1 : 0);
        fitGroundGrid();
        m_renderWindow->Render();
    }
}

/// 移除指定模型的渲染对象、数据记录和相关显示状态。
void ViewportWidget::removeModel(int modelId)
{
    for (int i = 0; i < m_models.size(); ++i) {
        if (m_models[i].id != modelId)
            continue;
        if (m_models[i].surfaceActor)
            m_renderer->RemoveActor(m_models[i].surfaceActor);
        if (m_models[i].edgeActor)
            m_renderer->RemoveActor(m_models[i].edgeActor);
        m_models.remove(i);
        break;
    }
    fitGroundGrid();
    m_renderWindow->Render();
    emit modelRemoved(modelId);
}


/// 按当前模式设置表面、线框和边线显示属性。
void ViewportWidget::applyDisplayModeToModel(LoadedGeometryModel& model)
{
    if (!model.surfaceActor)
        return;
    auto* property = model.surfaceActor->GetProperty();
    if (m_displayMode == DisplayMode::Wireframe) {
        property->SetRepresentationToWireframe();
        property->SetEdgeVisibility(0);
        return;
    }
    property->SetRepresentationToSurface();
    // 网格模型恒定显示单元边线：外表面与源几何同形，不画线无从分辨
    if (model.isMesh) {
        property->SetEdgeVisibility(1);
        property->SetEdgeColor(0.16, 0.18, 0.21);
        return;
    }
    property->SetEdgeVisibility(m_displayMode == DisplayMode::ShadedEdges ? 1 : 0);
    if (m_displayMode == DisplayMode::ShadedEdges)
        property->SetEdgeColor(0.16, 0.19, 0.22);
}

/// 将相机切换到指定的标准观察方向。
void ViewportWidget::setStandardView(ViewDirection direction)
{
    auto* cam = m_renderer->GetActiveCamera();
    if (!cam)
        return;
    double focal[3];
    cam->GetFocalPoint(focal);
    const double d = cam->GetDistance();
    double pos[3] = { focal[0], focal[1], focal[2] };
    double up[3] = { 0.0, 0.0, 1.0 };
    constexpr double k = 0.57735027;

    switch (direction) {
    case ViewDirection::Front:  pos[1] -= d; break;
    case ViewDirection::Back:   pos[1] += d; break;
    case ViewDirection::Left:   pos[0] -= d; break;
    case ViewDirection::Right:  pos[0] += d; break;
    case ViewDirection::Top:    pos[2] += d; up[0] = 0; up[1] = 1; up[2] = 0; break;
    case ViewDirection::Bottom: pos[2] -= d; up[0] = 0; up[1] = 1; up[2] = 0; break;
    case ViewDirection::Iso:
        pos[0] += d * k; pos[1] -= d * k; pos[2] += d * k;
        break;
    }

    cam->SetPosition(pos);
    cam->SetFocalPoint(focal);
    cam->SetViewUp(up);
    m_renderer->ResetCameraClippingRange();
    m_renderWindow->Render();
}

/// 切换相机的平行投影状态。
void ViewportWidget::setParallelProjection(bool on)
{
    auto* cam = m_renderer->GetActiveCamera();
    if (!cam)
        return;
    cam->SetParallelProjection(on ? 1 : 0);
    if (on) {
        double bounds[6];
        m_renderer->ComputeVisiblePropBounds(bounds);
        const double diag = std::sqrt(
            (bounds[1] - bounds[0]) * (bounds[1] - bounds[0])
            + (bounds[3] - bounds[2]) * (bounds[3] - bounds[2])
            + (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
        cam->SetParallelScale(std::max(diag * 0.5, 1e-3));
    }
    m_renderer->ResetCameraClippingRange();
    m_renderWindow->Render();
}

/// 查询当前相机是否使用平行投影。
bool ViewportWidget::isParallelProjection() const
{
    auto* cam = m_renderer->GetActiveCamera();
    return cam && cam->GetParallelProjection() != 0;
}

/// 更新所有模型的显示模式并重绘。
void ViewportWidget::setDisplayMode(DisplayMode mode)
{
    m_displayMode = mode;
    for (LoadedGeometryModel& model : m_models)
        applyDisplayModeToModel(model);
    m_renderWindow->Render();
}

/// 渲染当前场景并返回视口截图。
QImage ViewportWidget::grabView()
{
    m_renderWindow->Render();
    return grabFramebuffer();
}

/// 恢复默认相机视角。
void ViewportWidget::resetView()
{
    applyDefaultCamera();
}

/// 仅接受包含支持格式文件的拖入操作。
void ViewportWidget::dragEnterEvent(QDragEnterEvent* event)
{
    if (!event->mimeData()->hasUrls())
        return;
    const QList<QUrl> urls = event->mimeData()->urls();
    for (const QUrl& url : urls) {
        const QString path = url.toLocalFile();
        if (isStlFile(path) || ShapeLoader::isSupported(path)) {
            event->acceptProposedAction();
            return;
        }
    }
    event->ignore();
}

/// 将拖入的本地文件依次加入加载队列。
void ViewportWidget::dropEvent(QDropEvent* event)
{
    if (!event->mimeData()->hasUrls())
        return;
    // 支持一次拖入多个文件：loadFile 内部有队列，加载完自动接着处理
    bool accepted = false;
    for (const QUrl& url : event->mimeData()->urls()) {
        const QString path = url.toLocalFile();
        if (isStlFile(path) || ShapeLoader::isSupported(path)) {
            loadFile(path);
            accepted = true;
        }
    }
    if (accepted)
        event->acceptProposedAction();
    else
        event->ignore();
}
