// 文件职责：主窗口编排：连接面板和视口，管理工程及后台网格任务；几何计算和网格生成交给应用入口。
#include "MainWindow.h"

#include "ui/panels/ModelTreePanel.h"
#include "ui/dialogs/MeshSettingsDialog.h"
#include "ui/panels/OutputDock.h"
#include "ui/dialogs/ModelTransformDialog.h"
#include "ui/panels/PropertiesPanel.h"
#include "ui/viewport/ViewportWidget.h"
#include "project/ProjectArchive.h"
#include "cad/import/ShapeLoader.h"
#include "cad/import/ShapeTessellator.h"
#include "ui/platform/WindowChrome.h"

#include <QCloseEvent>
#include <QColorDialog>
#include <QComboBox>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLocale>
#include <QMenu>
#include <QMessageBox>
#include <QProgressBar>
#include <QSettings>
#include <QShortcut>
#include <QSlider>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTabBar>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrentRun>
#include <array>
#include <cmath>
#include <vector>

#include <vtkGeometryFilter.h>
#include <vtkFieldData.h>
#include <vtkNew.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLUnstructuredGridReader.h>

// DWM 标题栏染色统一在 ui/platform/WindowChrome 中实现。

namespace {

constexpr char kSettingsGeometry[] = "mainwindow/geometry";
constexpr char kSettingsState[] = "mainwindow/state";

/// 包围盒对角线长度（用于自动估算默认网格尺寸）
double modelDiagonal(const double range[6])
{
    const double dx = range[3] - range[0];
    const double dy = range[4] - range[1];
    const double dz = range[5] - range[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// 创建停靠面板的标题、图标和窗口操作按钮。
QWidget* makeDockTitleBar(QDockWidget* dock, const QString& title, const QString& iconPath)
{
    auto* bar = new QWidget(dock);
    bar->setObjectName(QStringLiteral("dockTitleBar"));
    bar->setFixedHeight(34);

    auto* layout = new QHBoxLayout(bar);
    layout->setContentsMargins(10, 0, 4, 0);
    layout->setSpacing(7);

    auto* iconLabel = new QLabel(bar);
    iconLabel->setObjectName(QStringLiteral("dockTitleIcon"));
    iconLabel->setPixmap(QIcon(iconPath).pixmap(16, 16));
    iconLabel->setFixedSize(16, 16);

    auto* titleLabel = new QLabel(title, bar);
    titleLabel->setObjectName(QStringLiteral("dockTitleLabel"));

    auto* floatButton = new QToolButton(bar);
    floatButton->setObjectName(QStringLiteral("dockBtn"));
    floatButton->setIcon(QIcon(QStringLiteral(":/icons/dock-float.svg")));
    floatButton->setIconSize(QSize(13, 13));
    floatButton->setAutoRaise(true);
    floatButton->setToolTip(QDockWidget::tr("浮动窗口"));
    QObject::connect(floatButton, &QToolButton::clicked, dock,
                     [dock] { dock->setFloating(!dock->isFloating()); });

    auto* closeButton = new QToolButton(bar);
    closeButton->setObjectName(QStringLiteral("dockBtn"));
    closeButton->setIcon(QIcon(QStringLiteral(":/icons/dock-close.svg")));
    closeButton->setIconSize(QSize(13, 13));
    closeButton->setAutoRaise(true);
    closeButton->setToolTip(QDockWidget::tr("关闭面板"));
    QObject::connect(closeButton, &QToolButton::clicked, dock, &QDockWidget::close);

    layout->addWidget(iconLabel);
    layout->addWidget(titleLabel);
    layout->addStretch();
    layout->addWidget(floatButton);
    layout->addWidget(closeButton);
    return bar;
}

// 状态栏「仪器读数」徽章：细边框背板 + 小标签 + 等宽大数字
QWidget* makeBadge(const QString& caption, const QString& value, QLabel** valueOut)
{
    auto* badge = new QFrame;
    badge->setObjectName(QStringLiteral("statusBadge"));
    auto* layout = new QVBoxLayout(badge);
    layout->setContentsMargins(12, 4, 12, 4);
    layout->setSpacing(0);
    auto* captionLabel = new QLabel(caption, badge);
    captionLabel->setObjectName(QStringLiteral("statusBadgeLabel"));
    auto* valueLabel = new QLabel(value, badge);
    valueLabel->setObjectName(QStringLiteral("statusBadgeValue"));
    layout->addWidget(captionLabel);
    layout->addWidget(valueLabel);
    if (valueOut)
        *valueOut = valueLabel;
    return badge;
}

} // namespace

/// 创建主界面、工程状态和后台网格任务连接。
MainWindow::MainWindow()
{
    resize(1440, 900);
    setMinimumSize(1440, 720);

    m_viewport = new ViewportWidget(this);
    setCentralWidget(m_viewport);

    buildDocks();
    buildStatusBar();

    // initRibbon 的「最近文件」按钮会 setMenu，须先创建菜单
    m_recentMenu = new QMenu(this);
    rebuildRecentMenu();

    initRibbon();

    connect(&m_meshWatcher, &QFutureWatcher<TetMeshResult>::finished,
            this, &MainWindow::onMeshFinished);
    connect(&m_surfaceWatcher, &QFutureWatcher<SurfaceGenerateResult>::finished,
            this, &MainWindow::onSurfaceMeshFinished);

    connect(m_viewport, &ViewportWidget::loadStarted, this, [this] {
        m_statusMode->setText(tr("正在导入几何…"));
        m_statusProgress->setRange(0, 0);
        m_statusProgress->show();
    });

    connect(m_viewport, &ViewportWidget::modelLoaded, this,
            [this](int modelId, const QString& name, int solids, int faces, int edges) {
                const ModelSummary summary=m_viewport->modelSummary(modelId);
                m_statusMode->setText(tr("就绪"));
                m_statusProgress->hide();
                m_statusModelValue->setText(name);
                m_modelTree->addModel(modelId, name, summary.isMesh);

                // 打开工程：按 displayName 匹配恢复外观，全部恢复完成后回填参数
                if (m_restoringProject) {
                    const auto it = m_pendingRestore.constFind(name);
                    if (it != m_pendingRestore.constEnd()) {
                        applyRestoredAppearance(modelId, it.value());
                        m_pendingRestore.erase(it);
                    }
                    finishRestoreStep();
                    return;
                }

                if (m_currentModelId < 0)
                    m_currentModelId = modelId; // 首个加载的模型自动选中，可直接生成网格
                if (!summary.isMesh && summary.rangeValid) {
                    const double diagonal=modelDiagonal(summary.range);
                    if (m_surfaceCriteria.targetSize<=0) {
                        m_surfaceCriteria.targetSize=m_surfaceCriteria.maxSize=diagonal*.025;
                        m_surfaceCriteria.minSize=m_surfaceCriteria.maxDeviation=diagonal*.0025;
                    }
                    if (m_meshParams.targetSize<=0) m_meshParams.targetSize=diagonal*.05;
                }
                markProjectModified();
                if (name.endsWith(QStringLiteral(".stl"), Qt::CaseInsensitive))
                    m_output->appendLog(OutputDock::Level::Ok,
                                        tr("已加载 STL 三角网格 %1 —— 三角形 %2")
                                            .arg(name).arg(faces));
                else
                    m_output->appendLog(OutputDock::Level::Ok,
                                        tr("已加载 %1 —— 实体 %2 · 面 %3 · 边 %4")
                                            .arg(name)
                                            .arg(solids)
                                            .arg(faces)
                                            .arg(edges));
            });

    connect(m_viewport, &ViewportWidget::modelRemoved, this,
            [this](int modelId) {
                if (m_currentModelId == modelId)
                    m_currentModelId = -1;
                if (m_meshModelId == modelId) {
                    m_meshModelId = -1;
                    m_lastMeshVolume = nullptr;
                    m_hasLastStats = false;
                }
                if (m_surfaceModelId == modelId) {
                    m_surfaceModelId = -1;
                    m_lastGeneratedSurface = nullptr;
                }
                m_modelTree->removeModel(modelId);
                const int count = m_viewport->modelCount();
                m_statusModelValue->setText(count > 0
                                                ? tr("已导入 %1").arg(count)
                                                : tr("未导入"));
                m_output->appendLog(OutputDock::Level::Info,
                                    tr("已移除模型 #%1").arg(modelId));
                markProjectModified();
            });

    connect(m_viewport, &ViewportWidget::loadFailed, this,
            [this](const QString& path, const QString& reason) {
                m_statusMode->setText(tr("就绪"));
                m_statusProgress->hide();
                m_output->appendLog(OutputDock::Level::Err,
                                    tr("加载失败 %1: %2").arg(path, reason));
            });

    connect(m_modelTree, &ModelTreePanel::modelNodeClicked, this,
            [this](int modelId) {
                m_currentModelId = modelId;
                const ModelSummary summary = m_viewport->modelSummary(modelId);
                if (!summary.valid)
                    return;
                m_propertiesDock->show();
                m_propertiesDock->raise();
                m_properties->showModel(modelId, summary.name, summary.solids,
                                        summary.faces, summary.edges,
                                        summary.visible, m_viewport->modelColor(modelId),
                                        m_viewport->modelOpacity(modelId),
                                        summary.range, summary.rangeValid,
                                        summary.unitHint);
            });
    connect(m_modelTree, &ModelTreePanel::modelVisibilityChanged, this,
            [this](int modelId, bool visible) {
                m_viewport->setModelVisible(modelId, visible);
            });
    connect(m_modelTree, &ModelTreePanel::removeModelRequested,
            this, &MainWindow::removeModel);
    connect(m_modelTree, &ModelTreePanel::modelColorChangeRequested,
            this, &MainWindow::changeModelColor);
    connect(m_modelTree, &ModelTreePanel::placeholderClicked, this,
            [this](const QString& title) {
                m_propertiesDock->hide();
                m_properties->showPlaceholder(
                    title, tr("选中具体节点后，这里显示对应的属性信息。"));
            });
    connect(m_properties, &PropertiesPanel::visibilityToggleRequested, this,
            [this](int modelId, bool visible) {
                m_viewport->setModelVisible(modelId, visible);
                m_modelTree->setModelChecked(modelId, visible);
            });
    connect(m_properties, &PropertiesPanel::colorChangeRequested,
            this, &MainWindow::changeModelColor);
    connect(m_properties, &PropertiesPanel::modelOpacityChanged, this,
            [this](int modelId, double opacity) {
                qDebug("MSTAGE: panel opacity id=%d opacity=%.2f", modelId, opacity);
                m_viewport->setModelOpacity(modelId, opacity);
            });

    // 视图重置的快捷键（输入框内输入 r 不会触发，Qt 会先给控件 ShortcutOverride）
    auto* resetViewShortcut = new QShortcut(QKeySequence(Qt::Key_R), this);
    resetViewShortcut->setContext(Qt::WindowShortcut);
    connect(resetViewShortcut, &QShortcut::activated, this, &MainWindow::resetView);

    m_output->appendLog(OutputDock::Level::Info,
                        tr("MeshStudio v0.1 启动 —— 支持 STEP / IGES / BRep / STL 导入，四面体网格由自研引擎生成（STL 输入）。"));
    m_output->appendLog(OutputDock::Level::Info,
                        tr("导入并选中几何模型后，点击「三角面片」或「四面体网格」打开对应参数。"));

    QSettings settings;
    if (settings.contains(kSettingsGeometry)) {
        restoreGeometry(settings.value(kSettingsGeometry).toByteArray());
        restoreState(settings.value(kSettingsState).toByteArray());
    } else {
        applyDefaultDockSizes();
    }
    updateProjectTitle();

    // 属性默认隐藏；网格参数只通过 Dialog 打开。
    m_propertiesDock->hide();
    QTimer::singleShot(0, this, [this] { applyDefaultDockSizes(); });
}

/// 记录最近文件并交给视口加载。
void MainWindow::openFile(const QString& filePath)
{
    addRecentFile(filePath);
    m_viewport->loadFile(filePath);
}

// ---------- 功能区 ----------

QHBoxLayout* MainWindow::newRibbonPage()
{
    auto* page = new QWidget(this);
    page->setObjectName(QStringLiteral("ribbonPage"));
    auto* layout = new QHBoxLayout(page);
    layout->setContentsMargins(2, 4, 2, 0);
    layout->setSpacing(0);
    m_ribbonStack->addWidget(page);
    return layout;
}

/// 在功能区页面中添加带标题的按钮分组。
QHBoxLayout* MainWindow::addRibbonGroup(QHBoxLayout* pageLayout,
                                        const QString& caption, QWidget** groupOut)
{
    auto* separator = new QFrame(pageLayout->parentWidget());
    separator->setObjectName(QStringLiteral("ribbonSeparator"));
    separator->setFrameShape(QFrame::VLine);
    pageLayout->addWidget(separator);

    auto* group = new QWidget(pageLayout->parentWidget());
    auto* groupLayout = new QVBoxLayout(group);
    groupLayout->setContentsMargins(4, 0, 4, 2);
    groupLayout->setSpacing(3);

    auto* buttonRow = new QWidget(group);
    auto* rowLayout = new QHBoxLayout(buttonRow);
    rowLayout->setContentsMargins(0, 0, 0, 0);
    rowLayout->setSpacing(3);
    groupLayout->addWidget(buttonRow);

    auto* captionLabel = new QLabel(caption, group);
    captionLabel->setObjectName(QStringLiteral("ribbonCaption"));
    captionLabel->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    groupLayout->addWidget(captionLabel);

    pageLayout->addWidget(group);
    if (groupOut)
        *groupOut = group;
    return rowLayout;
}

/// 创建指定图标、文字和尺寸样式的功能区按钮。
QToolButton* MainWindow::addRibbonButton(QHBoxLayout* groupLayout,
                                         const QIcon& icon, const QString& text,
                                         bool large)
{
    auto* button = new QToolButton(groupLayout->parentWidget());
    if (large) {
        button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        button->setIconSize(QSize(26, 26));
        button->setObjectName(QStringLiteral("ribbonLargeButton"));
    } else {
        // 小按钮: 有图标时图标+文字并排, 无图标退化为纯文字
        button->setToolButtonStyle(icon.isNull() ? Qt::ToolButtonTextOnly
                                                 : Qt::ToolButtonTextBesideIcon);
        button->setIconSize(QSize(20, 20));
        button->setObjectName(QStringLiteral("ribbonSmallButton"));
    }
    button->setText(text);
    if (!icon.isNull())
        button->setIcon(icon);
    button->setCursor(Qt::PointingHandCursor);
    groupLayout->addWidget(button);
    return button;
}

/// 构造主窗口功能区并连接界面命令。
void MainWindow::initRibbon()
{
    auto* ribbonArea = new QWidget(this);
    ribbonArea->setObjectName(QStringLiteral("ribbonBar"));
    auto* ribbonLayout = new QVBoxLayout(ribbonArea);
    ribbonLayout->setContentsMargins(0, 0, 0, 0);
    ribbonLayout->setSpacing(0);

    m_ribbonTabs = new QTabBar(ribbonArea);
    m_ribbonTabs->setShape(QTabBar::RoundedNorth);
    m_ribbonTabs->setExpanding(false);
    m_ribbonTabs->setDrawBase(false);
    m_ribbonTabs->addTab(tr("文件"));
    m_ribbonTabs->addTab(tr("几何"));
    m_ribbonTabs->addTab(tr("视图"));
    m_ribbonTabs->addTab(tr("帮助"));
    ribbonLayout->addWidget(m_ribbonTabs);

    m_ribbonStack = new QStackedWidget(ribbonArea);
    ribbonLayout->addWidget(m_ribbonStack);

    connect(m_ribbonTabs, &QTabBar::currentChanged,
            m_ribbonStack, &QStackedWidget::setCurrentIndex);
    setMenuWidget(ribbonArea);

    // ========== 页面: 文件 ==========
    QHBoxLayout* fileLayout = newRibbonPage();

    QHBoxLayout* fileProject = addRibbonGroup(fileLayout, tr("工程"));
    QToolButton* newProjectButton =
        addRibbonButton(fileProject, QIcon(":/icons/project-new.svg"), tr("新建工程"), false);
    connect(newProjectButton, &QToolButton::clicked, this, &MainWindow::newProject);
    QToolButton* openProjectButton =
        addRibbonButton(fileProject, QIcon(":/icons/project-open.svg"), tr("打开工程"), false);
    connect(openProjectButton, &QToolButton::clicked, this, &MainWindow::openProject);
    QToolButton* saveProjectButton =
        addRibbonButton(fileProject, QIcon(":/icons/project-save.svg"), tr("保存工程"), false);
    connect(saveProjectButton, &QToolButton::clicked, this, &MainWindow::saveProject);
    QToolButton* saveAsButton =
        addRibbonButton(fileProject, QIcon(":/icons/project-save.svg"), tr("另存为"), false);
    connect(saveAsButton, &QToolButton::clicked, this, &MainWindow::saveProjectAs);

    QHBoxLayout* fileImport = addRibbonGroup(fileLayout, tr("几何"));
    QToolButton* importButton =
        addRibbonButton(fileImport, QIcon(":/icons/import.svg"), tr("导入几何"), true);
    connect(importButton, &QToolButton::clicked, this, &MainWindow::importGeometry);

    QHBoxLayout* fileActions = addRibbonGroup(fileLayout, tr("文件"));
    auto* recentButton = addRibbonButton(fileActions, QIcon(":/icons/recent.svg"),
                                         tr("最近文件"), true);
    recentButton->setPopupMode(QToolButton::InstantPopup);
    recentButton->setMenu(m_recentMenu);
    recentButton->setToolTip(tr("打开最近使用过的模型"));
    QToolButton* exportButton =
        addRibbonButton(fileActions, QIcon(":/icons/camera.svg"), tr("导出视图"), true);
    connect(exportButton, &QToolButton::clicked, this, &MainWindow::exportView);

    QHBoxLayout* fileSession = addRibbonGroup(fileLayout, tr("会话"));
    QToolButton* quitButton =
        addRibbonButton(fileSession, QIcon(":/icons/exit.svg"), tr("退出"), false);
    connect(quitButton, &QToolButton::clicked, this, &MainWindow::close);
    fileLayout->addStretch(1);

    // ========== 页面: 几何 ==========
    QHBoxLayout* geometryLayout = newRibbonPage();

    QHBoxLayout* geometryGenerate = addRibbonGroup(geometryLayout, tr("网格生成"));
    m_surfaceGenerateButton=addRibbonButton(geometryGenerate,QIcon(":/icons/play.svg"),tr("三角面片"),true);
    m_surfaceGenerateButton->setObjectName("generateSurfaceAction");
    m_volumeGenerateButton=addRibbonButton(geometryGenerate,QIcon(":/icons/play.svg"),tr("四面体网格"),true);
    m_volumeGenerateButton->setObjectName("generateVolumeAction");
    connect(m_surfaceGenerateButton,&QToolButton::clicked,this,[this]{showMeshSettings(true);});
    connect(m_volumeGenerateButton,&QToolButton::clicked,this,[this]{showMeshSettings(false);});
    m_cancelSurfaceButton=addRibbonButton(geometryGenerate,QIcon(":/icons/dock-close.svg"),tr("取消表面生成"),false);
    m_cancelSurfaceButton->setObjectName("cancelSurfaceAction"); m_cancelSurfaceButton->hide();
    connect(m_cancelSurfaceButton,&QToolButton::clicked,this,[this] {
        if(m_surfaceCancellation) m_surfaceCancellation->store(true);
    });

    QHBoxLayout* geometryTransform = addRibbonGroup(geometryLayout, tr("变换"));
    QToolButton* transformButton = addRibbonButton(geometryTransform,
        QIcon(":/icons/transform.svg"), tr("变换模型"), true);
    transformButton->setToolTip(tr("平移 / 旋转 / 缩放当前选中的模型"));
    connect(transformButton, &QToolButton::clicked,
            this, &MainWindow::transformCurrentModel);

    QHBoxLayout* geometrySettings = addRibbonGroup(geometryLayout, tr("设置"));
    QToolButton* settingsButton =
        addRibbonButton(geometrySettings, QIcon(":/icons/sliders.svg"), tr("网格参数设置"), true);
    connect(settingsButton, &QToolButton::clicked, this, [this] {
        showMeshSettings(m_lastMeshDialogSurface);
    });
    geometryLayout->addStretch(1);

    // ========== 页面: 视图 ==========
    QHBoxLayout* viewLayout = newRibbonPage();

    QWidget* directionGroup = nullptr;
    QHBoxLayout* directionRow = addRibbonGroup(viewLayout, tr("方向"), &directionGroup);
    static const struct { const char* name; const char* icon; ViewDirection direction; } kViews[] = {
        { "前视", ":/icons/view-front.svg", ViewDirection::Front },
        { "后视", ":/icons/view-back.svg", ViewDirection::Back },
        { "左视", ":/icons/view-left.svg", ViewDirection::Left },
        { "右视", ":/icons/view-right.svg", ViewDirection::Right },
        { "上视", ":/icons/view-top.svg", ViewDirection::Top },
        { "下视", ":/icons/view-bottom.svg", ViewDirection::Bottom },
        { "轴测", ":/icons/view-iso.svg", ViewDirection::Iso },
    };
    {
        auto* directionGrid = new QGridLayout;
        directionGrid->setContentsMargins(0, 0, 0, 0);
        directionGrid->setHorizontalSpacing(1);
        directionGrid->setVerticalSpacing(1);
        for (int i = 0; i < 7; ++i) {
            auto* button = new QToolButton(directionGroup);
            button->setObjectName(QStringLiteral("ribbonDirButton"));
            button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
            button->setIconSize(QSize(20, 20));
            button->setIcon(QIcon(QString::fromUtf8(kViews[i].icon)));
            button->setText(QString::fromUtf8(kViews[i].name));
            button->setCursor(Qt::PointingHandCursor);
            directionGrid->addWidget(button, i / 4, i % 4);
            connect(button, &QToolButton::clicked, this, [this, i] {
                m_viewport->setStandardView(kViews[i].direction);
            });
        }
        qobject_cast<QVBoxLayout*>(directionGroup->layout())
            ->insertLayout(0, directionGrid);
    }

    QHBoxLayout* projectionRow = addRibbonGroup(viewLayout, tr("投影"));
    auto* orthoButton =
        addRibbonButton(projectionRow, QIcon(":/icons/projection-ortho.svg"), tr("正交投影"), true);
    orthoButton->setCheckable(true);
    orthoButton->setToolTip(tr("透视 / 正交投影切换"));
    connect(orthoButton, &QToolButton::toggled, this, [this](bool on) {
        m_viewport->setParallelProjection(on);
    });

    QHBoxLayout* modeRow = addRibbonGroup(viewLayout, tr("显示模式"));
    static const struct { const char* name; const char* icon; DisplayMode mode; } kModes[] = {
        { "着色", ":/icons/mode-shaded.svg", DisplayMode::Shaded },
        { "线框", ":/icons/mode-wireframe.svg", DisplayMode::Wireframe },
        { "着色+棱边", ":/icons/mode-shaded-edges.svg", DisplayMode::ShadedEdges },
    };
    for (int i = 0; i < 3; ++i) {
        QToolButton* button = addRibbonButton(modeRow,
            QIcon(QString::fromUtf8(kModes[i].icon)), QString::fromUtf8(kModes[i].name), false);
        button->setCheckable(true);
        button->setChecked(i == 0);
        connect(button, &QToolButton::clicked, this, [this, i, button] {
            for (QToolButton* other : m_displayModeButtons)
                other->setChecked(other == button);
            m_viewport->setDisplayMode(kModes[i].mode);
        });
        m_displayModeButtons[i] = button;
    }

    // 剖切: 第一行 = 开关 + 方向 + 翻转, 第二行 = 位置滑块独占一行
    // (滑块与按钮同排会被 DPI 取整挤到一起, 分行后互不接触且拖动区更大)
    QWidget* clipGroup = nullptr;
    QHBoxLayout* clipRow = addRibbonGroup(viewLayout, tr("剖切"), &clipGroup);
    auto* clipCheck = addRibbonButton(clipRow, QIcon(":/icons/clip.svg"), tr("剖切"), false);
    clipCheck->setCheckable(true);
    clipCheck->setToolTip(tr("启用/关闭剖切面（拖动位置滑块也会自动开启）"));
    m_clipAxisCombo = new QComboBox;
    m_clipAxisCombo->addItems({tr("X"), tr("Y"), tr("Z")});
    m_clipAxisCombo->setToolTip(tr("剖切方向"));
    clipRow->addWidget(m_clipAxisCombo);
    auto* flipButton = addRibbonButton(clipRow, QIcon(":/icons/flip.svg"), tr("翻转"), false);
    flipButton->setToolTip(tr("剖切方向翻转"));

    auto* sliderRow = new QHBoxLayout;
    sliderRow->setContentsMargins(4, 0, 4, 0);
    sliderRow->setSpacing(0);
    m_clipSlider = new QSlider(Qt::Horizontal);
    m_clipSlider->setRange(0, 1000);
    m_clipSlider->setValue(500);
    m_clipSlider->setMinimumWidth(120);
    m_clipSlider->setToolTip(tr("剖切位置（拖动自动开启剖切）"));
    sliderRow->addWidget(m_clipSlider);
    qobject_cast<QVBoxLayout*>(clipGroup->layout())->insertLayout(1, sliderRow);

    auto ensureClippingOn = [clipCheck] {
        if (!clipCheck->isChecked())
            clipCheck->setChecked(true);
    };
    connect(clipCheck, &QToolButton::toggled, this,
            [this](bool on) { m_viewport->setClippingEnabled(on); });
    connect(m_clipAxisCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this, ensureClippingOn](int index) {
                ensureClippingOn();
                m_viewport->setClippingAxis(index);
            });
    connect(m_clipSlider, &QSlider::valueChanged, this,
            [this, ensureClippingOn](int value) {
                ensureClippingOn();
                m_viewport->setClippingPosition(value / 1000.0);
            });
    connect(flipButton, &QToolButton::clicked, this, [this, ensureClippingOn] {
        ensureClippingOn();
        m_viewport->flipClipping();
    });

    QHBoxLayout* toolRow = addRibbonGroup(viewLayout, tr("工具"));
    QToolButton* resetViewButton =
        addRibbonButton(toolRow, QIcon(":/icons/reset-view.svg"), tr("重置视图"), false);
    resetViewButton->setToolTip(tr("快捷键 R"));
    connect(resetViewButton, &QToolButton::clicked, this, &MainWindow::resetView);
    QToolButton* resetLayoutButton =
        addRibbonButton(toolRow, QIcon(":/icons/layout-reset.svg"), tr("重置布局"), false);
    resetLayoutButton->setToolTip(tr("恢复默认停靠窗口布局"));
    connect(resetLayoutButton, &QToolButton::clicked, this, &MainWindow::resetLayout);

    QHBoxLayout* windowRow = addRibbonGroup(viewLayout, tr("停靠窗口"));
    auto* treeButton = addRibbonButton(windowRow, QIcon(":/icons/tree.svg"), tr("模型树"), false);
    treeButton->setCheckable(true);
    treeButton->setChecked(true);
    treeButton->setDefaultAction(m_modelTreeDock->toggleViewAction());
    auto* propsButton = addRibbonButton(windowRow, QIcon(":/icons/props.svg"), tr("属性"), false);
    propsButton->setCheckable(true);
    propsButton->setDefaultAction(m_propertiesDock->toggleViewAction());
    auto* logButton = addRibbonButton(windowRow, QIcon(":/icons/terminal.svg"), tr("输出日志"), false);
    logButton->setCheckable(true);
    logButton->setChecked(true);
    logButton->setDefaultAction(m_outputDock->toggleViewAction());
    viewLayout->addStretch(1);

    // ========== 页面: 帮助 ==========
    QHBoxLayout* helpLayout = newRibbonPage();
    QHBoxLayout* helpRow = addRibbonGroup(helpLayout, tr("帮助"));
    QToolButton* aboutButton = addRibbonButton(helpRow, QIcon(":/icons/about.svg"),
                                               tr("关于 MeshStudio"), false);
    connect(aboutButton, &QToolButton::clicked, this, &MainWindow::about);
    helpLayout->addStretch(1);
}

/// 创建模型树、属性和日志停靠面板。
void MainWindow::buildDocks()
{
    m_modelTree = new ModelTreePanel(this);
    m_modelTreeDock = new QDockWidget(tr("模型树"), this);
    m_modelTreeDock->setObjectName("dockModelTree");
    m_modelTreeDock->setWidget(m_modelTree);
    m_modelTreeDock->setFeatures(QDockWidget::DockWidgetMovable
                                 | QDockWidget::DockWidgetClosable
                                 | QDockWidget::DockWidgetFloatable);
    m_modelTreeDock->setTitleBarWidget(
        makeDockTitleBar(m_modelTreeDock, tr("模型树"), QStringLiteral(":/icons/tree.svg")));
    addDockWidget(Qt::LeftDockWidgetArea, m_modelTreeDock);

    m_properties = new PropertiesPanel(this);
    m_propertiesDock = new QDockWidget(tr("属性"), this);
    m_propertiesDock->setObjectName("dockProperties");
    m_propertiesDock->setWidget(m_properties);
    m_propertiesDock->setFeatures(QDockWidget::DockWidgetMovable
                                  | QDockWidget::DockWidgetClosable
                                  | QDockWidget::DockWidgetFloatable);
    m_propertiesDock->setTitleBarWidget(
        makeDockTitleBar(m_propertiesDock, tr("属性"), QStringLiteral(":/icons/props.svg")));
    addDockWidget(Qt::RightDockWidgetArea, m_propertiesDock);
    m_propertiesDock->raise();

    m_output = new OutputDock(this);
    m_outputDock = new QDockWidget(tr("输出日志"), this);
    m_outputDock->setObjectName("dockOutput");
    m_outputDock->setWidget(m_output);
    m_outputDock->setFeatures(QDockWidget::DockWidgetMovable
                              | QDockWidget::DockWidgetClosable
                              | QDockWidget::DockWidgetFloatable);
    m_outputDock->setTitleBarWidget(
        makeDockTitleBar(m_outputDock, tr("输出日志"), QStringLiteral(":/icons/terminal.svg")));
    addDockWidget(Qt::BottomDockWidgetArea, m_outputDock);
}

/// 恢复各停靠面板的默认尺寸。
void MainWindow::applyDefaultDockSizes()
{
    resizeDocks({m_modelTreeDock, m_propertiesDock}, {300, 320}, Qt::Horizontal);
    resizeDocks({m_outputDock}, {190}, Qt::Vertical);
}

/// 创建任务进度、模型数量和质量状态控件。
void MainWindow::buildStatusBar()
{
    m_statusMode = new QLabel(tr("就绪"), this);
    m_statusMode->setObjectName(QStringLiteral("statusMode"));
    statusBar()->addWidget(m_statusMode);

    m_statusProgress = new QProgressBar(this);
    m_statusProgress->setFixedWidth(150);
    m_statusProgress->setFixedHeight(14);
    m_statusProgress->setTextVisible(false);
    m_statusProgress->hide();
    statusBar()->addWidget(m_statusProgress);

    m_statusModel = makeBadge(tr("模型"), tr("未导入"), &m_statusModelValue);
    m_statusCells = makeBadge(tr("单元数"), tr("—"), &m_statusCellsValue);
    m_statusQuality = makeBadge(tr("最差质量"), tr("—"), &m_statusQualityValue);
    statusBar()->addPermanentWidget(m_statusModel);
    statusBar()->addPermanentWidget(m_statusCells);
    statusBar()->addPermanentWidget(m_statusQuality);
    statusBar()->showMessage(QString());
}

/// 打开文件选择框并导入用户选择的几何文件。
void MainWindow::importGeometry()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("导入几何"), QString(),
        tr("所有支持的格式 (*.step *.stp *.iges *.igs *.brep *.stl);;"
           "STEP (*.step *.stp);;"
           "IGES (*.iges *.igs);;"
           "BRep (*.brep);;"
           "STL (*.stl)"));
    if (path.isEmpty())
        return;
    openFile(path);
}

/// 编辑参数草稿，接受后保存参数并启动对应网格任务。
void MainWindow::showMeshSettings(bool surface)
{
    if (m_generating || m_restoringProject) return;
    const ModelSummary summary=m_viewport->modelSummary(m_currentModelId);
    if (!summary.valid || (summary.isMesh && (surface || !summary.isSurfaceMesh))) {
        QMessageBox::information(this,tr("网格参数"),tr("请先选择几何模型。"));
        return;
    }
    const double diagonal=summary.rangeValid?modelDiagonal(summary.range):0;
    if (!(diagonal>0) || !std::isfinite(diagonal)) {
        QMessageBox::warning(this,tr("网格参数"),tr("模型尺寸无效，无法设置网格参数。"));
        return;
    }
    MeshSettingsDialog dialog(summary.name,diagonal,
        surface?MeshSettingsDialog::Mode::Surface:MeshSettingsDialog::Mode::Volume,
        m_surfaceCriteria,m_meshParams,this);
    // 对话框只编辑草稿；用户确认后再提交参数，取消保留工程中的原设置。
    if (dialog.exec()!=QDialog::Accepted) return;
    m_lastMeshDialogSurface=dialog.mode()==MeshSettingsDialog::Mode::Surface;
    if (m_lastMeshDialogSurface) {
        if (!ShapeLoader::isSupported(summary.filePath)) {
            QMessageBox::information(this,tr("网格参数"),tr("表面三角生成支持 STEP、IGES、BRep。STL 本身已是三角网格。"));
            return;
        }
        m_surfaceCriteria=dialog.surfaceCriteria();
    } else m_meshParams=dialog.volumeParameters();
    markProjectModified();
    if (m_lastMeshDialogSurface) generateSurfaceMesh(); else generateMesh();
}

/// 按任务状态更新生成和取消按钮。
void MainWindow::updateMeshActions(bool surfaceCancellable)
{
    m_surfaceGenerateButton->setEnabled(!m_generating);
    m_volumeGenerateButton->setEnabled(!m_generating);
    m_cancelSurfaceButton->setVisible(m_generating && surfaceCancellable);
}

/// 固定本次输入和取消标记，启动后台表面生成任务。
void MainWindow::generateSurfaceMesh()
{
    if (m_generating) return;
    if (m_currentModelId < 0) {
        QMessageBox::information(this,tr("MeshStudio"),tr("请先选择 CAD 几何模型。"));
        return;
    }
    const ModelSummary summary=m_viewport->modelSummary(m_currentModelId);
    if (!summary.valid || !ShapeLoader::isSupported(summary.filePath)) {
        QMessageBox::information(this,tr("MeshStudio"),
            tr("表面三角生成支持 STEP、IGES、BRep。STL 本身已是三角网格。"));
        return;
    }
    m_generating=true;
    updateMeshActions(true);
    m_statusMode->setText(tr("表面三角网格生成中…"));
    m_statusProgress->setRange(0,0);
    m_statusProgress->show();
    m_pendingMeshSource=summary.filePath;
    m_meshSourceModelId=m_currentModelId;
    const mesher::SurfaceCriteria criteria=m_surfaceCriteria;
    if(criteria.pointPlacement==mesher::SurfacePointPlacement::Algorithm6Experimental) {
        m_output->appendLog(OutputDock::Level::Info,tr("开始 Algorithm 6 实验生成：%1；h=%2，边界尺寸传播=%3，原平滑=%4 轮；质量需独立审计")
            .arg(QFileInfo(summary.filePath).fileName()).arg(criteria.algorithm6.targetSize)
            .arg(criteria.algorithm6.extendBoundarySize).arg(criteria.algorithm6.smoothingIterations));
    } else m_output->appendLog(OutputDock::Level::Info,
        tr("开始自研表面三角化：%1；基础目标 %2、最小尺寸 %3、偏差 %4（模型单位），法向角 %5°；最大尺寸 %6")
            .arg(QFileInfo(summary.filePath).fileName())
            .arg(criteria.targetSize).arg(criteria.minSize)
            .arg(criteria.maxDeviation).arg(criteria.maxNormalAngleDeg)
            .arg(criteria.maxSize>0?QString::number(criteria.maxSize):tr("旧目标尺寸及 5% 余量")));
    const QString path=summary.filePath;
    // 按值捕获本次输入，后台不读取可被界面继续修改的成员；取消标记跨线程共同持有。
    m_surfaceCancellation=std::make_shared<std::atomic_bool>(false);
    const auto cancellation=m_surfaceCancellation;
    m_surfaceWatcher.setFuture(QtConcurrent::run([path,criteria,cancellation]() {
        return SurfaceMeshEngine::run(path,criteria,[cancellation](int,int) {
            return !cancellation->load();
        });
    }));
}

/// 接收表面生成结果，仅在未取消且成功时发布。
void MainWindow::onSurfaceMeshFinished()
{
    m_generating=false;
    updateMeshActions();
    m_statusMode->setText(tr("就绪"));
    m_statusProgress->hide();
    const SurfaceGenerateResult result=m_surfaceWatcher.result();
    const bool cancelled=m_surfaceCancellation && m_surfaceCancellation->load();
    m_surfaceCancellation.reset();
    // 即使计算恰好已完成，只要用户已请求取消，就不发布这次结果。
    if (cancelled) {
        m_output->appendLog(OutputDock::Level::Info,tr("表面网格生成已取消。"));
        return;
    }
    if (!result.ok || !result.surface) {
        m_output->appendLog(OutputDock::Level::Err,
            tr("表面三角化失败（%1）：%2")
                .arg(QFileInfo(m_pendingMeshSource).fileName(),result.error));
        QMessageBox::warning(this,tr("MeshStudio"),
                             tr("表面三角网格生成失败：\n%1").arg(result.error));
        return;
    }
    // 前面的失败分支保留旧网格；走到这里才将成功结果交给 GUI 线程的视口。
    if (m_surfaceModelId>=0) m_viewport->removeModel(m_surfaceModelId);
    const QString name=QFileInfo(m_pendingMeshSource).completeBaseName()+tr("（表面三角网格）");
    m_surfaceModelId=m_viewport->addMeshModel(result.surface,name,true);
    if (m_surfaceModelId<0) {
        m_output->appendLog(OutputDock::Level::Err,tr("表面网格生成完成，但视口未能接收有效网格。"));
        return;
    }
    m_lastGeneratedSurface=result.surface;
    if (m_meshSourceModelId>=0) {
        m_viewport->setModelVisible(m_meshSourceModelId,false);
        m_modelTree->setModelChecked(m_meshSourceModelId,false);
    }
    const auto& detail=result.detail;
    m_statusCellsValue->setText(QLocale().toString(static_cast<qulonglong>(detail.mesh.triangles.size())));
    const bool algorithm6=result.surface->GetFieldData()->GetAbstractArray("SurfaceGenerationAlgorithm")!=nullptr;
    // 实验生成与独立审计分开，不能仅凭生成成功把质量栏显示成已验收。
    m_statusQualityValue->setText(algorithm6?tr("待审计"):QString::number(detail.measuredWorstRadiusEdgeRatio,'f',2));
    m_output->appendLog(OutputDock::Level::Ok,
        tr("表面三角网格已生成：%1 节点，%2 三角形")
            .arg(detail.mesh.vertices.size()).arg(detail.mesh.triangles.size()));
    if(!algorithm6)m_output->appendLog(OutputDock::Level::Info,
        tr("采样测得：最长边 %1、最大偏差 %2（模型单位），最大法向变化 %3°，最大半径边比 %4")
            .arg(detail.measuredMaxEdgeLength).arg(detail.measuredMaxDeviation)
            .arg(detail.measuredMaxNormalAngleDeg).arg(detail.measuredWorstRadiusEdgeRatio));
    for (const std::string& warning:detail.diagnostics)
        m_output->appendLog(OutputDock::Level::Warn,QString::fromStdString(warning));
    markProjectModified();
}

/// 准备已有或新生成的边界表面并启动体网格任务。
void MainWindow::generateMesh()
{
    if (m_generating)
        return;
    if (m_currentModelId < 0) {
        QMessageBox::information(this, tr("MeshStudio"),
                                 tr("请先在模型树中选中要划分网格的几何模型。"));
        return;
    }
    const ModelSummary summary = m_viewport->modelSummary(m_currentModelId);
    if (!summary.valid || (summary.filePath.isEmpty() && !summary.isSurfaceMesh)) {
        QMessageBox::information(this, tr("MeshStudio"),
                                 tr("当前选中节点不是可划分网格的几何模型，请先导入 STEP/IGES/BRep 文件。"));
        return;
    }

    m_generating = true;
    updateMeshActions();
    m_statusMode->setText(tr("四面体网格生成中…"));
    m_statusProgress->setRange(0, 0);
    m_statusProgress->show();

    m_pendingMeshSource = summary.isSurfaceMesh ? summary.name : summary.filePath;
    m_meshSourceModelId = m_currentModelId; // 生成成功后自动隐藏源几何
    TetMeshParams params = m_meshParams;
    params.geometryFilePath = summary.filePath;

    const QString suffix = QFileInfo(summary.filePath).suffix().toLower();
    const bool brepInput = ShapeTessellator::isBRepFormat(summary.filePath);
    const auto surfaceBoundary=summary.isSurfaceMesh ?
        TetMeshEngine::boundarySnapshot(m_viewport->surfaceMeshSnapshot(m_currentModelId)) : mesher::SurfaceMeshResult{};
    const auto surfaceCriteria=m_surfaceCriteria;
    // 已有表面保留节点/标签；CAD 需先生成表面；STL 则由体入口读取并焊接。
    m_output->appendLog(OutputDock::Level::Info,
                        tr("开始生成四面体网格：%1（输入 %2）")
                            .arg(QFileInfo(summary.filePath).fileName(),
                                 summary.isSurfaceMesh ? tr("已有自研表面，保留节点及 CAD 标签") :
                                 brepInput ? tr("B-Rep，先生成自研表面") : suffix.toUpper()));
    m_output->appendLog(
        OutputDock::Level::Info,
        tr("参数：目标尺寸 %1（模型单位）· T4 线性四面体 · 自研 Delaunay 与细化 · 优化 %2")
            .arg(params.targetSize)
            .arg(params.optimize ? tr("开") : tr("关")));

    // GUI 先复制数据；后台不访问视口/共享 VTK，CAD 查询由表面引擎负责。
    const bool surfaceInput=summary.isSurfaceMesh;
    m_meshWatcher.setFuture(QtConcurrent::run([params, brepInput,surfaceInput,surfaceBoundary,surfaceCriteria]() {
        if(surfaceInput) return TetMeshEngine::run(params,surfaceBoundary);
        if (!brepInput)
            return TetMeshEngine::run(params);
        const auto generated=SurfaceMeshEngine::run(params.geometryFilePath,surfaceCriteria);
        if(!generated.ok) {
            TetMeshResult failed;
            failed.error=tr("自研表面生成失败：%1").arg(generated.error);
            return failed;
        }
        return TetMeshEngine::run(params,generated.detail);
    }));
}

/// 接收体网格结果，更新显示、统计和工程状态。
void MainWindow::onMeshFinished()
{
    m_generating = false;
    updateMeshActions();
    m_statusMode->setText(tr("就绪"));
    m_statusProgress->hide();

    const TetMeshResult result = m_meshWatcher.result();
    const QString sourceName = QFileInfo(m_pendingMeshSource).fileName();
    if (!result.ok || !result.surface) {
        m_output->appendLog(OutputDock::Level::Err,
                            tr("四面体生成失败（%1）：%2").arg(sourceName, result.error));
        QMessageBox::warning(this, tr("MeshStudio"),
                             tr("四面体网格生成失败：\n%1").arg(result.error));
        return;
    }

    if (m_meshModelId >= 0) {
        m_viewport->removeModel(m_meshModelId);
        m_meshModelId = -1;
    }
    const QString meshName =
        QFileInfo(m_pendingMeshSource).completeBaseName() + tr("（四面体网格）");
    m_meshModelId = m_viewport->addMeshModel(result.surface, meshName);
    m_lastMeshVolume = result.volume;
    m_lastMeshSurface = result.surface;
    m_lastStats = result.stats;
    m_hasLastStats = true;

    // 网格与源几何完全贴合，隐藏源几何让网格可见（模型树勾选同步；可手动重新勾选）
    if (m_meshSourceModelId >= 0 && m_viewport->modelCount() > 0) {
        m_viewport->setModelVisible(m_meshSourceModelId, false);
        m_modelTree->setModelChecked(m_meshSourceModelId, false);
        m_output->appendLog(OutputDock::Level::Info,
                            tr("已隐藏源几何 %1 以显示网格，可在模型树重新勾选显示。")
                                .arg(QFileInfo(m_pendingMeshSource).fileName()));
    }

    const TetMeshQualityStats& s = result.stats;
    m_statusCellsValue->setText(QLocale().toString(static_cast<qulonglong>(s.tetCount)));
    m_statusQualityValue->setText(QString::number(s.minGamma, 'f', 2));

    m_output->appendLog(result.qualityAccepted?OutputDock::Level::Ok:OutputDock::Level::Warn,
                        tr("四面体生成结束（%1）：%2").arg(sourceName,
                           result.qualityAccepted?tr("质量验收通过"):tr("结果可显示，质量未达标")));
    for(const auto& warning:result.warnings) m_output->appendLog(OutputDock::Level::Warn,warning);
    m_output->appendLog(
        OutputDock::Level::Info,
        tr("输入：%1 点 / %2 三角形 → 进入体网格 %3 点（合并 %4 · 剔除退化三角形 %5）")
            .arg(result.inputPointCount)
            .arg(result.inputTriCount)
            .arg(result.weldedPointCount)
            .arg(result.weldMergedVertices)
            .arg(result.weldDroppedTriangles));
    m_output->appendLog(OutputDock::Level::Info,
                        tr("节点 %1 · 体单元 %2（%3）· 表面三角形 %4")
                            .arg(QLocale().toString(static_cast<qulonglong>(s.nodeCount)))
                            .arg(QLocale().toString(static_cast<qulonglong>(s.tetCount)))
                            .arg(s.quadratic ? tr("T10 二阶") : tr("T4 一阶"))
                            .arg(QLocale().toString(static_cast<qulonglong>(s.surfaceTriCount))));
    m_output->appendLog(OutputDock::Level::Info,
                        tr("质量：gamma 最小 %1 / 平均 %2 · scaled Jacobian %3 · 二面角 [%4°, %5°]")
                            .arg(s.minGamma, 0, 'f', 3)
                            .arg(s.avgGamma, 0, 'f', 3)
                            .arg(s.minScaledJacobian, 0, 'f', 3)
                            .arg(s.minDihedralDeg, 0, 'f', 1)
                            .arg(s.maxDihedralDeg, 0, 'f', 1));
    if (s.invertedCount > 0)
        m_output->appendLog(OutputDock::Level::Err,
                            tr("警告：检测到 %1 个翻转单元，导出前需处理！")
                                .arg(s.invertedCount));
    else if (s.minDihedralDeg < 10.0)
        m_output->appendLog(OutputDock::Level::Warn,
                            tr("最小二面角 %1° 低于 10°，存在薄片（sliver）单元，建议开启优化或减小目标尺寸。")
                                .arg(s.minDihedralDeg, 0, 'f', 1));
    else if(result.qualityAccepted)
        m_output->appendLog(OutputDock::Level::Ok,
                            tr("质量检查通过：边长、半径边比、退化/翻转和薄片检查均达标。"));
    markProjectModified();
}

// ---------- 工程（.msp 包）管理 ----------

void MainWindow::markProjectModified()
{
    if (m_restoringProject)
        return;
    m_projectModified = true;
    setWindowModified(true);
}

/// 更新工程名称及未保存状态的窗口标题。
void MainWindow::updateProjectTitle()
{
    const QString name = m_currentProjectPath.isEmpty()
        ? tr("未命名工程")
        : QFileInfo(m_currentProjectPath).fileName();
    setWindowTitle(tr("%1[*] — MeshStudio").arg(name));
    setWindowModified(m_projectModified);
}

/// 询问是否保存修改，取消或保存失败时阻止后续切换。
bool MainWindow::maybeSaveProject()
{
    if (!m_projectModified)
        return true;
    const QMessageBox::StandardButton choice = QMessageBox::question(
        this, tr("保存工程"),
        tr("当前工程有未保存的更改，是否先保存？"),
        QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
    if (choice == QMessageBox::Cancel)
        return false;
    if (choice == QMessageBox::Save) {
        saveProject();
        return !m_projectModified; // 保存被取消/失败则中止后续操作
    }
    return true; // 放弃更改
}

/// 处理未保存修改后清空模型和工程状态。
void MainWindow::newProject()
{
    if (m_generating) {
        m_output->appendLog(OutputDock::Level::Warn, tr("网格生成期间无法切换工程。"));
        return;
    }
    if (!maybeSaveProject())
        return;
    m_viewport->clearModels();
    m_currentProjectPath.clear();
    m_projectModified = false;
    m_meshModelId = -1;
    m_lastMeshVolume = nullptr;
    m_surfaceModelId = -1;
    m_lastGeneratedSurface = nullptr;
    m_hasLastStats = false;
    m_currentModelId = -1;
    m_meshParams=TetMeshParams{}; m_surfaceCriteria=mesher::SurfaceCriteria{};
    m_statusCellsValue->setText(tr("—"));
    m_statusQualityValue->setText(tr("—"));
    updateProjectTitle();
    m_output->appendLog(OutputDock::Level::Info, tr("已新建空白工程。"));
}

/// 选择工程包并启动恢复。
void MainWindow::openProject()
{
    if (!maybeSaveProject())
        return;
    const QString path = QFileDialog::getOpenFileName(
        this, tr("打开工程"), QString(),
        tr("MeshStudio 工程 (*.msp);;所有文件 (*)"));
    if (path.isEmpty())
        return;
    openProjectFile(path);
}

/// 保存到当前工程路径，未命名时转入另存为。
void MainWindow::saveProject()
{
    if (m_currentProjectPath.isEmpty())
        saveProjectAs();
    else
        saveProjectTo(m_currentProjectPath);
}

/// 选择新路径并保存当前工程。
void MainWindow::saveProjectAs()
{
    const QString suggested = m_currentProjectPath.isEmpty()
        ? QStringLiteral("Untitled.msp")
        : m_currentProjectPath;
    const QString path = QFileDialog::getSaveFileName(
        this, tr("保存工程"), suggested,
        tr("MeshStudio 工程 (*.msp)"));
    if (path.isEmpty())
        return;
    saveProjectTo(path);
}

/// 收集当前模型、参数和预览并写入工程包。
bool MainWindow::saveProjectTo(const QString& packagePath)
{
    MeshProject project;
    project.projectName = QFileInfo(packagePath).completeBaseName();
    project.meshParams = m_meshParams;
    project.surfaceCriteria = m_surfaceCriteria;
    project.hasMeshStats = m_hasLastStats;
    project.meshStats = m_lastStats;
    QHash<QString, QString> geometryFiles;
    QHash<QString, vtkPolyData*> meshSurfaces;
    m_viewport->collectProjectModels(project.models, geometryFiles,meshSurfaces);

    // 几何源文件必须存在才能自包含嵌入
    for (auto it = geometryFiles.constBegin(); it != geometryFiles.constEnd(); ++it) {
        if (!QFileInfo::exists(it.value())) {
            QMessageBox::warning(this, tr("保存工程"),
                                 tr("几何源文件缺失，无法嵌入工程包：%1").arg(it.value()));
            return false;
        }
    }

    // 体网格和表面网格分别保存；表面网格只有 .vtp。
    QHash<QString, vtkUnstructuredGrid*> meshVolumes;
    for (const ProjectModelEntry& entry : project.models) {
        if (entry.isMesh && !entry.isSurfaceMesh && m_lastMeshVolume) {
            meshVolumes.insert(entry.meshFile, m_lastMeshVolume);
        }
    }

    const QImage preview = m_viewport->grabView();
    QString error;
    if (!ProjectArchive::save(project, packagePath, preview, m_currentProjectPath,
                              geometryFiles, meshVolumes, meshSurfaces, &error)) {
        m_output->appendLog(OutputDock::Level::Err, tr("工程保存失败：%1").arg(error));
        QMessageBox::warning(this, tr("保存工程"), tr("工程保存失败：\n%1").arg(error));
        return false;
    }

    m_currentProjectPath = packagePath;
    m_projectModified = false;
    updateProjectTitle();
    m_output->appendLog(OutputDock::Level::Ok,
                        tr("工程已保存：%1（模型 %2 · 网格%3）")
                            .arg(QFileInfo(packagePath).fileName())
                            .arg(project.models.size())
                            .arg(meshSurfaces.isEmpty() ? tr("无") : tr("已嵌入")));
    return true;
}

/// 读取工程包并按保存内容恢复模型和参数。
void MainWindow::openProjectFile(const QString& packagePath)
{
    if (m_generating) {
        m_output->appendLog(OutputDock::Level::Warn, tr("网格生成期间无法切换工程。"));
        return;
    }
    MeshProject project;
    QString error;
    m_projectTempDir = std::make_unique<QTemporaryDir>(); // 重置解包目录（旧目录自动清理）
    if (!ProjectArchive::load(packagePath, project, *m_projectTempDir, &error)) {
        QMessageBox::warning(this, tr("打开工程"), tr("打开工程失败：\n%1").arg(error));
        return;
    }

    m_restoringProject = true;
    m_restoreRemaining = project.models.size();
    m_pendingRestore.clear();
    m_pendingRestoreParams = project.meshParams;
    m_pendingSurfaceCriteria = project.surfaceCriteria;
    m_meshModelId = -1;
    m_surfaceModelId = -1;
    m_lastMeshVolume = nullptr;
    m_lastGeneratedSurface = nullptr;
    m_hasLastStats = project.hasMeshStats;
    m_lastStats = project.hasMeshStats ? project.meshStats : TetMeshQualityStats{};
    m_currentModelId = -1;
    m_viewport->clearModels();

    for (const ProjectModelEntry& entry : project.models) {
        if (entry.isMesh) {
            restoreMeshFromProject(entry);
            continue;
        }
        m_pendingRestore.insert(entry.displayName, entry);
        m_viewport->loadFile(m_projectTempDir->filePath(entry.sourceFile));
    }
    if (project.models.isEmpty()) {
        m_restoreRemaining=1;
        finishRestoreStep();
    }

    m_currentProjectPath = packagePath;
    m_projectModified = false;
    updateProjectTitle();
    m_output->appendLog(OutputDock::Level::Ok,
                        tr("工程已打开：%1（模型 %2）")
                            .arg(project.projectName)
                            .arg(project.models.size()));
}

/// 读取工程中的 VTK 网格并恢复其显示模型。
void MainWindow::restoreMeshFromProject(const ProjectModelEntry& entry)
{
    vtkSmartPointer<vtkUnstructuredGrid> volume;
    if (!entry.isSurfaceMesh) {
        const QString vtuPath = m_projectTempDir->filePath(entry.meshFile);
        vtkNew<vtkXMLUnstructuredGridReader> volumeReader;
        volumeReader->SetFileName(vtuPath.toLocal8Bit().constData());
        volumeReader->Update();
        volume=volumeReader->GetOutput();
    }

    const QString vtpPath = m_projectTempDir->filePath(entry.surfaceFile);
    vtkNew<vtkXMLPolyDataReader> surfaceReader;
    surfaceReader->SetFileName(vtpPath.toLocal8Bit().constData());
    surfaceReader->Update();
    vtkPolyData* surface = surfaceReader->GetOutput();
    if ((!entry.isSurfaceMesh && (!volume || volume->GetNumberOfCells()==0)) ||
        !surface || surface->GetNumberOfCells()==0) {
        m_output->appendLog(OutputDock::Level::Warn,
                            tr("工程内网格数据无效，已跳过：%1").arg(entry.displayName));
        finishRestoreStep();
        return;
    }

    m_pendingRestore.insert(entry.displayName, entry);
    if (entry.isSurfaceMesh) {
        m_lastGeneratedSurface=surface;
        m_surfaceModelId=m_viewport->addMeshModel(surface,entry.displayName,true);
    } else {
        m_lastMeshVolume = volume;
        m_lastMeshSurface = surface;
        m_meshModelId = m_viewport->addMeshModel(surface, entry.displayName);
    }
    // addMeshModel 内部 emit modelLoaded → applyRestoredAppearance + finishRestoreStep

    if (entry.isSurfaceMesh) {
        m_statusCellsValue->setText(QLocale().toString(static_cast<qulonglong>(surface->GetNumberOfCells())));
        // 保存的表面只带节点/连接/CAD 标签；缺少质量统计时不沿用体网格数值。
        m_statusQualityValue->setText(tr("—"));
    } else if (m_hasLastStats) {
        m_statusCellsValue->setText(
            QLocale().toString(static_cast<qulonglong>(m_lastStats.tetCount)));
        m_statusQualityValue->setText(QString::number(m_lastStats.minGamma, 'f', 2));
        if(!m_lastStats.acceptanceKnown || !m_lastStats.qualityAccepted)
            m_output->appendLog(OutputDock::Level::Warn,m_lastStats.acceptanceKnown
                ? tr("恢复的体网格尚未通过质量验收。")
                : tr("旧工程未记录完整质量验收结果，请重新生成后检查。"));
    }
}

/// 恢复模型保存的颜色、透明度、可见性和变换。
void MainWindow::applyRestoredAppearance(int modelId, const ProjectModelEntry& entry)
{
    m_viewport->setModelColor(modelId, entry.color);
    m_viewport->setModelOpacity(modelId, entry.opacity);
    m_viewport->setModelVisible(modelId, entry.visible);
    const double* t = entry.transform;
    const bool identity = t[0] == 0 && t[1] == 0 && t[2] == 0 &&
                          t[3] == 0 && t[4] == 0 && t[5] == 0 && t[6] == 1.0;
    if (!identity)
        m_viewport->setModelTransform(modelId, t[0], t[1], t[2], t[3], t[4], t[5], t[6]);
}

/// 记录一次恢复结束，全部完成后发布工程参数。
void MainWindow::finishRestoreStep()
{
    if (--m_restoreRemaining <= 0 && m_restoringProject) {
        m_restoringProject = false;
        m_meshParams=m_pendingRestoreParams;
        m_surfaceCriteria=m_pendingSurfaceCriteria;
    }
}

/// 恢复默认相机视角。
void MainWindow::resetView()
{
    m_viewport->resetView();
}

/// 恢复默认面板可见性和尺寸。
void MainWindow::resetLayout()
{
    m_modelTreeDock->show();
    m_outputDock->show();
    m_propertiesDock->hide();
    applyDefaultDockSizes();
}

/// 显示软件说明对话框。
void MainWindow::about()
{
    QMessageBox box(this);
    box.setWindowTitle(tr("关于 MeshStudio"));
    box.setIconPixmap(QIcon(":/icons/logo.svg").pixmap(64, 64));
    box.setText(tr(
        "<b>MeshStudio v0.1.0</b><br>"
        "FDTD 前处理独立界面模块（薄 UI + 厚引擎）。<br><br>"
        "算法逻辑保留在 MeshEngine 库中，界面只负责:<br>"
        "收集参数 → 调用引擎 → 展示结果。<br><br>"
        "当前为 Phase 1: 多模型几何导入与显示。"));
    box.exec();
}

/// 从视口移除模型，并在无模型时收起属性面板。
void MainWindow::removeModel(int modelId)
{
    m_viewport->removeModel(modelId);
    if (m_viewport->modelCount() == 0)
        m_propertiesDock->hide();
}

/// 编辑选中模型的显示变换并更新工程修改状态。
void MainWindow::transformCurrentModel()
{
    if (m_currentModelId < 0) {
        QMessageBox::information(this, tr("MeshStudio"),
                                 tr("请先在模型树中点击一个模型节点，再进行变换。"));
        return;
    }
    const QString modelName = m_viewport->modelSummary(m_currentModelId).name;
    double values[7] = { 0, 0, 0, 0, 0, 0, 1.0 };
    m_viewport->modelTransformValues(m_currentModelId, values);

    ModelTransformDialog dialog(modelName, this);
    dialog.setValues(values[0], values[1], values[2],
                     values[3], values[4], values[5], values[6]);
    connect(&dialog, &ModelTransformDialog::transformResetRequested, this, [this] {
        m_viewport->resetModelTransform(m_currentModelId);
        m_output->appendLog(OutputDock::Level::Info,
                            tr("模型 #%1 变换已复位").arg(m_currentModelId));
    });

    if (dialog.exec() != QDialog::Accepted)
        return;
    m_viewport->setModelTransform(m_currentModelId,
                                  dialog.translateX(), dialog.translateY(),
                                  dialog.translateZ(),
                                  dialog.rotateX(), dialog.rotateY(),
                                  dialog.rotateZ(), dialog.scale());
    m_output->appendLog(OutputDock::Level::Info,
                        tr("模型 #%1 变换已更新").arg(m_currentModelId));
    markProjectModified();
}

/// 选择模型颜色并同步视口、属性面板和工程状态。
void MainWindow::changeModelColor(int modelId)
{
    const QColor current = m_viewport->modelColor(modelId);
    const QColor chosen = QColorDialog::getColor(current, this, tr("选择模型颜色"));
    if (!chosen.isValid())
        return;
    m_viewport->setModelColor(modelId, chosen);
    m_properties->updateColorSwatch(modelId, chosen);
    markProjectModified();
}

/// 将当前视口截图保存为 PNG 文件。
void MainWindow::exportView()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("导出视图"), QStringLiteral("meshstudio_view.png"),
        tr("PNG 图片 (*.png)"));
    if (path.isEmpty())
        return;
    const QImage image = m_viewport->grabView();
    if (image.save(path))
        m_output->appendLog(OutputDock::Level::Ok, tr("视图已导出: %1").arg(path));
    else
        m_output->appendLog(OutputDock::Level::Err, tr("视图导出失败: %1").arg(path));
}

/// 更新最近文件列表并保持数量上限。
void MainWindow::addRecentFile(const QString& filePath)
{
    QSettings settings;
    QStringList files = settings.value(QStringLiteral("recentFiles")).toStringList();
    files.removeAll(filePath);
    files.prepend(filePath);
    while (files.size() > 10)
        files.removeLast();
    settings.setValue(QStringLiteral("recentFiles"), files);
    rebuildRecentMenu();
}

/// 根据保存的最近文件列表重建菜单。
void MainWindow::rebuildRecentMenu()
{
    if (!m_recentMenu)
        return;
    m_recentMenu->clear();
    const QStringList files =
        QSettings().value(QStringLiteral("recentFiles")).toStringList();
    if (files.isEmpty()) {
        QAction* empty = m_recentMenu->addAction(tr("（空）"));
        empty->setEnabled(false);
        return;
    }
    for (int i = 0; i < files.size(); ++i) {
        const QString path = files.at(i);
        QAction* action = m_recentMenu->addAction(
            tr("&%1 %2").arg(i < 9 ? QString::number(i + 1) : QStringLiteral("0"), path));
        connect(action, &QAction::triggered, this, [this, path] {
            if (!QFileInfo::exists(path)) {
                QMessageBox::warning(this, tr("MeshStudio"),
                                     tr("文件不存在: %1").arg(path));
                // 失效条目移出最近列表，而不是重新加回顶部
                QSettings settings;
                QStringList files =
                    settings.value(QStringLiteral("recentFiles")).toStringList();
                files.removeAll(path);
                settings.setValue(QStringLiteral("recentFiles"), files);
                rebuildRecentMenu();
                return;
            }
            openFile(path);
        });
    }
}

/// 处理未保存修改并保存窗口布局后接受关闭。
void MainWindow::closeEvent(QCloseEvent* event)
{
    if (!maybeSaveProject()) {
        event->ignore();
        return;
    }
    QSettings settings;
    settings.setValue(kSettingsGeometry, saveGeometry());
    settings.setValue(kSettingsState, saveState());
    event->accept();
}

// 标题栏染色统一走 MeshChrome（见 ui/platform/WindowChrome）。
void MainWindow::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    MeshChrome::styleTitleBar(this);
}
