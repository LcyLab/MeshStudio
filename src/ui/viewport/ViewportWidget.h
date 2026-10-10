#pragma once
// 文件职责：VTK 视口管理：后台加载几何数据，GUI 线程创建渲染对象，并管理模型显示、变换和工程快照。

#include <QColor>
#include <QFutureWatcher>
#include <QHash>
#include <QImage>
#include <QVTKOpenGLNativeWidget.h>
#include <QVector>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>

#include "project/MeshProject.h"

class vtkActor;
class vtkCornerAnnotation;
class vtkPlane;
class vtkPlane;
class vtkGenericOpenGLRenderWindow;
class vtkOrientationMarkerWidget;
class vtkPolyData;
class vtkRenderer;

/// 标准视角
enum class ViewDirection { Front, Back, Left, Right, Top, Bottom, Iso };

/// 显示模式
enum class DisplayMode { Shaded, Wireframe, ShadedEdges };

/// 后台线程产出的加载结果（VTK 数据对象可跨线程构建，渲染对象须在 GUI 线程创建）
struct ViewportLoadResult
{
    bool ok = false;
    QString error;
    int solids = 0;
    int faces = 0;
    int edges = 0;
    vtkSmartPointer<vtkPolyData> surface;
    vtkSmartPointer<vtkPolyData> edgeLines;
};

struct LoadedGeometryModel
{
    int id = 0;
    QString name;
    QString filePath; ///< 来源 CAD 文件全路径（网格注入模型为空）
    bool isMesh = false; ///< true=已生成的表面或四面体网格
    bool isSurfaceMesh = false; ///< true=自研表面三角网格（无体单元）
    int solids = 0;
    int faces = 0;
    int edges = 0;
    vtkSmartPointer<vtkPolyData> surface;
    vtkSmartPointer<vtkPolyData> edgeLines;
    vtkSmartPointer<vtkActor> surfaceActor;
    vtkSmartPointer<vtkActor> edgeActor;
    QColor color;
    double opacity = 1.0;
    double transform[7] = { 0, 0, 0, 0, 0, 0, 1.0 };
    bool visible = true;
};

/// 属性面板查询结果
struct ModelSummary
{
    bool valid = false;
    bool isMesh = false;
    bool isSurfaceMesh = false;
    bool rangeValid = false;
    QString name;
    QString filePath; ///< 来源 CAD 文件全路径（不可用时为空）
    int solids = 0;
    int faces = 0;
    int edges = 0;
    bool visible = true;
    double opacity = 1.0;
    double range[6] = { 0, 0, 0, 0, 0, 0 };
    // STEP/IGES 经 OCC 转 mm；BRep/STL/独立网格未确认单位时不标注。
    QString unitHint;
};

struct EntityInfo
{
    bool valid = false;
    QString modelName;
    double range[6] = { 0, 0, 0, 0, 0, 0 };
};

// 只管理显示用模型和 VTK 场景；加载可在后台完成，actor/mapper 和窗口操作留在 GUI 线程。
// 模型 ID 是界面关联身份，与 CAD 面/边编号及网格节点索引分别管理。
class ViewportWidget : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    /// 初始化 VTK 渲染场景及后台几何加载观察器。
    explicit ViewportWidget(QWidget* parent = nullptr);

    /// 按后缀路由：STL 走 VTK 快速路径，STEP/IGES/BRep 走 OCC；均在后台线程执行
    void loadFile(const QString& filePath);

    /// 注入已生成的表面网格或四面体外表面，返回模型 id。
    int addMeshModel(vtkPolyData* meshSurface, const QString& name,
                     bool surfaceOnly = false);

    /// 收集当前全部模型为工程条目（保存工程用，按加入顺序）；
    /// sourceFiles 输出 包内相对路径(geometry/...) → 磁盘上的几何源文件路径
    void collectProjectModels(QVector<ProjectModelEntry>& entries,
                              QHash<QString, QString>& sourceFiles,
                              QHash<QString, vtkPolyData*>& meshSurfaces) const;

    /// 移除全部模型（新建/打开工程前清场）
    void clearModels();

    /// 恢复默认相机视角。
    void resetView();

    /// 更新模型表面和边线的可见性。
    void setModelVisible(int modelId, bool visible);
    /// 移除指定模型的渲染对象、数据记录和相关显示状态。
    void removeModel(int modelId);
    /// 修改模型显示颜色并重绘视口。
    void setModelColor(int modelId, const QColor& color);
    /// 查询指定模型当前的显示颜色。
    QColor modelColor(int modelId) const;
    /// 修改模型不透明度并重绘视口。
    void setModelOpacity(int modelId, double opacity);
    /// 查询指定模型当前的不透明度。
    double modelOpacity(int modelId) const;

    /// 启用或关闭显示裁剪平面。
    void setClippingEnabled(bool on);
    /// 选择显示裁剪平面的轴向。
    void setClippingAxis(int axis);
    void setClippingPosition(double position); ///< 沿所选轴包围盒的归一化位置；只裁剪显示。
    /// 反转裁剪平面保留的一侧。
    void flipClipping();

    /// 设置 actor 显示变换，旋转使用度；不改源 CAD 或传给生成器的原始网格坐标。
    void setModelTransform(int modelId, double tx, double ty, double tz,
                           double rx, double ry, double rz, double scale);
    /// 将指定模型的显示变换恢复为单位变换。
    void resetModelTransform(int modelId);
    /// 读取模型当前的七个显示变换参数。
    bool modelTransformValues(int modelId, double out[7]) const;

    /// 将相机切换到指定的标准观察方向。
    void setStandardView(ViewDirection direction);
    /// 切换相机的平行投影状态。
    void setParallelProjection(bool on);
    /// 查询当前相机是否使用平行投影。
    bool isParallelProjection() const;
    /// 更新所有模型的显示模式并重绘。
    void setDisplayMode(DisplayMode mode);
    /// 读取当前视口显示模式。
    DisplayMode displayMode() const { return m_displayMode; }
    /// 渲染当前场景并返回视口截图。
    QImage grabView();

    /// 读取指定模型的显示名称、文件和范围摘要。
    ModelSummary modelSummary(int modelId) const;
    /// 复制指定表面模型的原始网格；只在 GUI 线程调用，不共享后台可变 VTK 数据。
    vtkSmartPointer<vtkPolyData> surfaceMeshSnapshot(int modelId) const;
    /// 返回当前加载的模型数量。
    int modelCount() const;

signals:
    /// 通知界面一次后台模型加载已经开始。
    void loadStarted();
    /// 通知界面模型加载完成及其摘要信息。
    void modelLoaded(int modelId, const QString& name, int solids, int faces, int edges);
    /// 通知界面指定模型已从视口移除。
    void modelRemoved(int modelId);
    /// 通知界面模型加载失败及错误原因。
    void loadFailed(const QString& filePath, const QString& reason);

protected:
    /// 仅接受包含支持格式文件的拖入操作。
    void dragEnterEvent(QDragEnterEvent* event) override;
    /// 将拖入的本地文件依次加入加载队列。
    void dropEvent(QDropEvent* event) override;

private slots:
    /// 接收加载结果，创建显示模型并继续处理队列。
    void onLoadFinished();

private:
    /// 建立渲染器、坐标辅助和默认交互环境。
    void buildScene();
    /// 按场景范围恢复默认相机并重绘。
    void applyDefaultCamera();
    /// 按模型范围调整地面参考网格。
    void fitGroundGrid();
    /// 合并当前场景模型的三维包围范围。
    bool sceneBounds(double lo[3], double hi[3]);
    /// 按裁剪开关为模型的显示映射器设置裁剪平面。
    void applyClippingToModel(LoadedGeometryModel& model, bool on);
    /// 按场景范围、方向和相对位置更新裁剪平面。
    void updateClippingPlane();
    /// 将缩放、旋转和平移应用于显示对象。
    void applyTransformToModel(LoadedGeometryModel& model);
    /// 按当前模式设置表面、线框和边线显示属性。
    void applyDisplayModeToModel(LoadedGeometryModel& model);
    void startLoad(const QString& filePath); ///< 每次只运行一个加载任务，其余文件保存在加载队列。
    /// 按模型编号查找视口持有的模型。
    LoadedGeometryModel* findModel(int modelId);
    /// 按模型编号查找视口持有的模型。
    const LoadedGeometryModel* findModel(int modelId) const;

    vtkNew<vtkGenericOpenGLRenderWindow> m_renderWindow;
    vtkNew<vtkRenderer> m_renderer;
    vtkNew<vtkCornerAnnotation> m_annotation;
    vtkNew<vtkPlaneSource> m_gridSource;
    vtkSmartPointer<vtkActor> m_gridActor;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_axesWidget;

    QVector<LoadedGeometryModel> m_models;
    int m_nextModelId = 1;

    DisplayMode m_displayMode = DisplayMode::Shaded;
    vtkSmartPointer<vtkPlane> m_clipPlane;
    bool m_clipping = false;
    bool m_clipFlipped = false;
    int m_clipAxis = 0;
    double m_clipPosition = 0.5;

    QFutureWatcher<ViewportLoadResult> m_loadWatcher;
    QString m_pendingPath;
    QStringList m_loadQueue;
    bool m_loading = false;
};
