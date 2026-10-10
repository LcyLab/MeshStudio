#pragma once
// 文件职责：主窗口编排：连接面板和视口，管理工程及后台网格任务；几何计算和网格生成交给应用入口。

#include <QFutureWatcher>
#include <QHash>
#include <QLabel>
#include <QMainWindow>
#include <QTemporaryDir>
#include <memory>
#include <atomic>

#include "project/MeshProject.h"
#include "app/meshing/TetMeshEngine.h"
#include "app/meshing/SurfaceMeshEngine.h"

class OutputDock;
class vtkPolyData;
class ModelTreePanel;
class PropertiesPanel;
class ViewportWidget;
class QComboBox;
class QMenu;
class QProgressBar;
class QSlider;
class QStackedWidget;
class QTabBar;
class QToolButton;
class QVBoxLayout;
class QHBoxLayout;

// UI 线程持有窗口、工程状态和任务观察器；后台任务只接收参数/网格快照。
// 完成槽负责发布结果，算法入口不直接操作面板、视口或工程状态。
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    /// 创建主界面、工程状态和后台网格任务连接。
    MainWindow();

    /// 从命令行或外部打开几何文件（按后缀路由）
    void openFile(const QString& filePath);

    /// 打开 .msp 工程包（含命令行入口）
    void openProjectFile(const QString& packagePath);

private slots:
    // 界面命令与任务完成通知：生成在后台，结果接收和渲染在 GUI 线程。
    /// 将当前视口截图保存为 PNG 文件。
    void exportView();
    /// 打开文件选择框并导入用户选择的几何文件。
    void importGeometry();
    void generateMesh();              ///< 复用已有表面，或先生成 CAD 表面，再启动体网格任务。
    void onMeshFinished();            ///< 接收体结果，更新显示、质量统计和工程修改标记。
    void generateSurfaceMesh();       ///< 固定本次参数并创建独立取消标记，启动表面任务。
    void onSurfaceMeshFinished();     ///< 取消/失败不替换已有网格，成功才发布表面模型。
    /// 恢复默认相机视角。
    void resetView();
    /// 恢复默认面板可见性和尺寸。
    void resetLayout();
    /// 显示软件说明对话框。
    void about();

    // 工程操作
    /// 处理未保存修改后清空模型和工程状态。
    void newProject();
    /// 选择工程包并启动恢复。
    void openProject();
    /// 保存到当前工程路径，未命名时转入另存为。
    void saveProject();
    /// 选择新路径并保存当前工程。
    void saveProjectAs();

private:
    // UI 构造：只组织控件和信号连接，不执行网格计算。
    /// 构造主窗口功能区并连接界面命令。
    void initRibbon();
    /// 创建模型树、属性和日志停靠面板。
    void buildDocks();
    /// 创建任务进度、模型数量和质量状态控件。
    void buildStatusBar();
    /// 恢复各停靠面板的默认尺寸。
    void applyDefaultDockSizes();
    /// 更新最近文件列表并保持数量上限。
    void addRecentFile(const QString& filePath);
    /// 根据保存的最近文件列表重建菜单。
    void rebuildRecentMenu();
    /// 从视口移除模型，并在无模型时收起属性面板。
    void removeModel(int modelId);
    /// 选择模型颜色并同步视口、属性面板和工程状态。
    void changeModelColor(int modelId);
    /// 编辑选中模型的显示变换并更新工程修改状态。
    void transformCurrentModel();
    /// 对话框编辑草稿，Accepted 后才写回工程参数并启动选择的生成任务。
    void showMeshSettings(bool surface);
    /// 按任务状态更新生成和取消按钮。
    void updateMeshActions(bool surfaceCancellable=false);

    // 工程（.msp 包）管理
    /// 在非工程恢复阶段标记工程存在未保存修改。
    void markProjectModified();
    /// 更新工程名称及未保存状态的窗口标题。
    void updateProjectTitle();
    bool maybeSaveProject();          ///< 有未保存修改时询问；取消则阻止新建、打开或关闭。
    /// 收集当前模型、参数和预览并写入工程包。
    bool saveProjectTo(const QString& packagePath);
    /// 读取工程中的 VTK 网格并恢复其显示模型。
    void restoreMeshFromProject(const ProjectModelEntry& entry);
    /// 恢复模型保存的颜色、透明度、可见性和变换。
    void applyRestoredAppearance(int modelId, const ProjectModelEntry& entry);
    void finishRestoreStep();         ///< 异步几何加载全部结束后统一恢复保存的参数状态。

    // 功能区构造辅助
    /// 创建一个功能区页面及其横向布局。
    QHBoxLayout* newRibbonPage();
    /// 在功能区页面中添加带标题的按钮分组。
    QHBoxLayout* addRibbonGroup(QHBoxLayout* pageLayout, const QString& caption,
                                QWidget** groupOut = nullptr);
    /// 创建指定图标、文字和尺寸样式的功能区按钮。
    QToolButton* addRibbonButton(QHBoxLayout* groupLayout, const QIcon& icon,
                                 const QString& text, bool large);

protected:
    /// 处理未保存修改并保存窗口布局后接受关闭。
    void closeEvent(QCloseEvent* event) override;
    /// 在窗口显示后应用平台标题栏样式。
    void showEvent(QShowEvent* event) override;

    ViewportWidget* m_viewport = nullptr;
    ModelTreePanel* m_modelTree = nullptr;
    // 参数状态属于工程；Dialog 只编辑草稿，拒绝/关闭不写入这里。
    TetMeshParams m_meshParams;
    mesher::SurfaceCriteria m_surfaceCriteria;
    bool m_lastMeshDialogSurface=true;
    OutputDock* m_output = nullptr;

    QDockWidget* m_modelTreeDock = nullptr;
    QDockWidget* m_propertiesDock = nullptr;
    QDockWidget* m_outputDock = nullptr;
    PropertiesPanel* m_properties = nullptr;
    QMenu* m_recentMenu = nullptr;

    QTabBar* m_ribbonTabs = nullptr;
    QStackedWidget* m_ribbonStack = nullptr;
    QToolButton* m_displayModeButtons[3] = {};
    QToolButton* m_surfaceGenerateButton=nullptr;
    QToolButton* m_volumeGenerateButton=nullptr;
    QToolButton* m_cancelSurfaceButton=nullptr;

    QComboBox* m_clipAxisCombo = nullptr;
    QSlider* m_clipSlider = nullptr;
    int m_currentModelId = -1;

    QLabel* m_statusMode = nullptr;
    QWidget* m_statusModel = nullptr;
    QWidget* m_statusCells = nullptr;
    QWidget* m_statusQuality = nullptr;
    QLabel* m_statusModelValue = nullptr;
    QLabel* m_statusCellsValue = nullptr;
    QLabel* m_statusQualityValue = nullptr;
    QProgressBar* m_statusProgress = nullptr;

    bool m_generating = false;

    // watcher 将后台完成事件投递回本窗口；取消标记由窗口和任务共同持有。
    QFutureWatcher<TetMeshResult> m_meshWatcher;
    QFutureWatcher<SurfaceGenerateResult> m_surfaceWatcher;
    std::shared_ptr<std::atomic_bool> m_surfaceCancellation;
    QString m_pendingMeshSource;   ///< 正在生成网格的几何文件（完成后用于日志/命名）
    int m_meshSourceModelId = -1;  ///< 正在生成网格的源几何模型（完成后自动隐藏）
    int m_meshModelId = -1;        ///< 视口中当前四面体网格模型（新生成时先移除）
    int m_surfaceModelId = -1;     ///< 当前表面三角网格模型
    vtkSmartPointer<vtkPolyData> m_lastGeneratedSurface;
    vtkSmartPointer<vtkUnstructuredGrid> m_lastMeshVolume; ///< 当前体网格（工程保存用）
    vtkSmartPointer<vtkPolyData> m_lastMeshSurface;        ///< 当前网格外表面（工程保存用）
    TetMeshQualityStats m_lastStats;
    bool m_hasLastStats = false;

    // 工程（.msp 包）
    QString m_currentProjectPath;
    bool m_projectModified = false;
    std::unique_ptr<QTemporaryDir> m_projectTempDir; ///< 打开工程后的解包目录（保活）
    QHash<QString, ProjectModelEntry> m_pendingRestore; ///< 恢复队列（key=displayName）
    TetMeshParams m_pendingRestoreParams;         ///< 恢复完成时回填的网格参数
    mesher::SurfaceCriteria m_pendingSurfaceCriteria;
    int m_restoreRemaining = 0;                   ///< 未完成的恢复条目数（归零后回填参数）
    bool m_restoringProject = false;              ///< 恢复期间跳过自动尺寸填充
};
