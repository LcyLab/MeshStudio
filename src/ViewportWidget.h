#pragma once

#include <QColor>
#include <QFutureWatcher>
#include <QHash>
#include <QImage>
#include <QVTKOpenGLNativeWidget.h>
#include <QVector>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkSmartPointer.h>

#include "core/MeshProject.h"

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
    bool isMesh = false; ///< true=四面体网格注入模型
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
    bool rangeValid = false;
    QString name;
    QString filePath; ///< 来源 CAD 文件全路径（不可用时为空）
    int solids = 0;
    int faces = 0;
    int edges = 0;
    bool visible = true;
    double opacity = 1.0;
    double range[6] = { 0, 0, 0, 0, 0, 0 };
    // 长度单位提示: STEP/IGES 经 OCC 统一转 mm; STL 无单位信息, 为空串表示不标注
    QString unitHint;
};

struct EntityInfo
{
    bool valid = false;
    QString modelName;
    double range[6] = { 0, 0, 0, 0, 0, 0 };
};

class ViewportWidget : public QVTKOpenGLNativeWidget
{
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);

    /// 按后缀路由：STL 走 VTK 快速路径，STEP/IGES/BRep 走 OCC；均在后台线程执行
    void loadFile(const QString& filePath);

    /// 注入已生成的网格外表面对象（四面体生成完成后的显示入口），返回模型 id
    int addMeshModel(vtkPolyData* meshSurface, const QString& name);

    /// 收集当前全部模型为工程条目（保存工程用，按加入顺序）；
    /// sourceFiles 输出 包内相对路径(geometry/...) → 磁盘上的几何源文件路径
    void collectProjectModels(QVector<ProjectModelEntry>& entries,
                              QHash<QString, QString>& sourceFiles) const;

    /// 移除全部模型（新建/打开工程前清场）
    void clearModels();

    void resetView();

    void setModelVisible(int modelId, bool visible);
    void removeModel(int modelId);
    void setModelColor(int modelId, const QColor& color);
    QColor modelColor(int modelId) const;
    void setModelOpacity(int modelId, double opacity);
    double modelOpacity(int modelId) const;

    void setClippingEnabled(bool on);
    void setClippingAxis(int axis);
    void setClippingPosition(double position);
    void flipClipping();

    void setModelTransform(int modelId, double tx, double ty, double tz,
                           double rx, double ry, double rz, double scale);
    void resetModelTransform(int modelId);
    bool modelTransformValues(int modelId, double out[7]) const;

    void setStandardView(ViewDirection direction);
    void setParallelProjection(bool on);
    bool isParallelProjection() const;
    void setDisplayMode(DisplayMode mode);
    DisplayMode displayMode() const { return m_displayMode; }
    QImage grabView();

    ModelSummary modelSummary(int modelId) const;
    int modelCount() const;

signals:
    void loadStarted();
    void modelLoaded(int modelId, const QString& name, int solids, int faces, int edges);
    void modelRemoved(int modelId);
    void loadFailed(const QString& filePath, const QString& reason);

protected:
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private slots:
    void onLoadFinished();

private:
    void buildScene();
    void applyDefaultCamera();
    void fitGroundGrid();
    bool sceneBounds(double lo[3], double hi[3]);
    void applyClippingToModel(LoadedGeometryModel& model, bool on);
    void updateClippingPlane();
    void applyTransformToModel(LoadedGeometryModel& model);
    void applyDisplayModeToModel(LoadedGeometryModel& model);
    void startLoad(const QString& filePath);
    LoadedGeometryModel* findModel(int modelId);
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
