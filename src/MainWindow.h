#pragma once

#include <QFutureWatcher>
#include <QHash>
#include <QLabel>
#include <QMainWindow>
#include <QTemporaryDir>
#include <memory>

#include "core/MeshProject.h"
#include "core/TetMeshEngine.h"

class OutputDock;
class vtkPolyData;
class MeshSettingsPanel;
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

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow();

    /// 从命令行或外部打开几何文件（按后缀路由）
    void openFile(const QString& filePath);

    /// 打开 .msp 工程包（含命令行入口）
    void openProjectFile(const QString& packagePath);

private slots:
    void exportView();
    void importGeometry();
    void generateMesh();
    void onMeshFinished();
    void resetView();
    void resetLayout();
    void about();

    // 工程操作
    void newProject();
    void openProject();
    void saveProject();
    void saveProjectAs();

private:
    void initRibbon();
    void buildDocks();
    void buildStatusBar();
    void applyDefaultDockSizes();
    void addRecentFile(const QString& filePath);
    void rebuildRecentMenu();
    void removeModel(int modelId);
    void changeModelColor(int modelId);
    void transformCurrentModel();

    // 工程（.msp 包）管理
    void markProjectModified();
    void updateProjectTitle();
    bool maybeSaveProject();
    bool saveProjectTo(const QString& packagePath);
    void restoreMeshFromProject(const ProjectModelEntry& entry);
    void applyRestoredAppearance(int modelId, const ProjectModelEntry& entry);
    void finishRestoreStep();

    // 功能区构造辅助
    QHBoxLayout* newRibbonPage();
    QHBoxLayout* addRibbonGroup(QHBoxLayout* pageLayout, const QString& caption,
                                QWidget** groupOut = nullptr);
    QToolButton* addRibbonButton(QHBoxLayout* groupLayout, const QIcon& icon,
                                 const QString& text, bool large);

protected:
    void closeEvent(QCloseEvent* event) override;
    void showEvent(QShowEvent* event) override;

    ViewportWidget* m_viewport = nullptr;
    ModelTreePanel* m_modelTree = nullptr;
    MeshSettingsPanel* m_settings = nullptr;
    OutputDock* m_output = nullptr;

    QDockWidget* m_modelTreeDock = nullptr;
    QDockWidget* m_settingsDock = nullptr;
    QDockWidget* m_propertiesDock = nullptr;
    QDockWidget* m_outputDock = nullptr;
    PropertiesPanel* m_properties = nullptr;
    QMenu* m_recentMenu = nullptr;

    QTabBar* m_ribbonTabs = nullptr;
    QStackedWidget* m_ribbonStack = nullptr;
    QToolButton* m_displayModeButtons[3] = {};

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

    QFutureWatcher<TetMeshResult> m_meshWatcher;
    QString m_pendingMeshSource;   ///< 正在生成网格的几何文件（完成后用于日志/命名）
    int m_meshSourceModelId = -1;  ///< 正在生成网格的源几何模型（完成后自动隐藏）
    int m_meshModelId = -1;        ///< 视口中当前四面体网格模型（新生成时先移除）
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
    int m_restoreRemaining = 0;                   ///< 未完成的恢复条目数（归零后回填参数）
    bool m_restoringProject = false;              ///< 恢复期间跳过自动尺寸填充
};
