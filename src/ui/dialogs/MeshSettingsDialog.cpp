// 文件职责：参数草稿界面：编辑表面或体网格设置，确认后由主窗口读取；关闭或取消不修改工程参数。
#include "MeshSettingsDialog.h"
#include "ui/platform/WindowChrome.h"
#include "ui/forms/DialogFormFactory.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <limits>

namespace {
/// 为参数页添加可自动调整内容尺寸的滚动区域。
QScrollArea* wrapScroll(QWidget* page,QWidget* parent) {
    auto* area=new QScrollArea(parent); area->setWidgetResizable(true);
    area->setFrameShape(QFrame::NoFrame); area->setWidget(page); return area;
}
}

/// 创建表面和体网格参数草稿界面。
MeshSettingsDialog::MeshSettingsDialog(const QString& name,double diagonal,Mode initialMode,
        const mesher::SurfaceCriteria& surface,const TetMeshParams& volume,QWidget* parent)
    :QDialog(parent),modelDiagonal(diagonal),originalSurface(surface),originalVolume(volume) {
    setObjectName("meshSettingsDialog"); setWindowTitle(tr("网格参数"));
    DialogFormFactory::configure(this);
    auto* layout=new QVBoxLayout(this); layout->setContentsMargins(22,18,22,18); layout->setSpacing(12);
    layout->addWidget(DialogFormFactory::header(tr("网格参数"),tr("%1  ·  模型坐标单位").arg(name),this));
    tabs=new QTabWidget(this); tabs->setObjectName("meshTypeTabs");
    tabs->addTab(wrapScroll(buildSurfacePage(),tabs),tr("三角面片"));
    tabs->addTab(wrapScroll(buildVolumePage(),tabs),tr("四面体"));
    layout->addWidget(tabs,1);
    errorLabel=new QLabel(this); errorLabel->setObjectName("meshDialogError");
    errorLabel->setWordWrap(true); errorLabel->hide(); layout->addWidget(errorLabel);
    const auto actions=DialogFormFactory::actions(QString(),this);
    actions.reset->setObjectName("resetMeshDefaults"); actions.cancel->setObjectName("cancelMeshDialog");
    generateButton=actions.primary; layout->addWidget(actions.widget);
    connect(actions.reset,&QPushButton::clicked,this,&MeshSettingsDialog::resetCurrentPage);
    connect(actions.cancel,&QPushButton::clicked,this,&QDialog::reject);
    connect(generateButton,&QPushButton::clicked,this,&MeshSettingsDialog::accept);
    connect(tabs,&QTabWidget::currentChanged,this,[this]{updateMode();});
    fillSurface(surface);
    const auto clearRangeError=[this](double) {
        if(minimum->value()<=maximum->value()) errorLabel->hide();
    };
    connect(minimum,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,clearRangeError);
    connect(maximum,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,clearRangeError);
    volumeTarget->setValue(volume.targetSize>0?volume.targetSize:diagonal*.05);
    optimize->setChecked(volume.optimize);
    tabs->setCurrentIndex(initialMode==Mode::Surface?0:1); updateMode();
}

/// 构造表面算法选择、尺寸和高级参数页。
QWidget* MeshSettingsDialog::buildSurfacePage() {
    auto* page=new QWidget(this); auto* layout=new QVBoxLayout(page);
    layout->setContentsMargins(0,14,0,0); layout->setSpacing(14);
    const auto basic=DialogFormFactory::section(QString(),page);
    const auto route=DialogFormFactory::section(QString(),page);
    surfaceAlgorithm=new QComboBox(route.widget);surfaceAlgorithm->setObjectName("surfaceAlgorithm");
    surfaceAlgorithm->addItem(tr("现有表面生成"),int(originalSurface.pointPlacement==mesher::SurfacePointPlacement::Algorithm6Experimental?
        mesher::SurfacePointPlacement::LegacyRefinement:originalSurface.pointPlacement));
    surfaceAlgorithm->addItem(tr("Algorithm 6（实验）"),int(mesher::SurfacePointPlacement::Algorithm6Experimental));
    DialogFormFactory::row(route,tr("生成路线"),surfaceAlgorithm);layout->addWidget(route.widget);
    legacyBasic=basic.widget;
    minimum=DialogFormFactory::number(basic.widget,"surfaceMinimum",1e-9,1e12);
    maximum=DialogFormFactory::number(basic.widget,"surfaceMaximum",1e-9,1e12);
    deviation=DialogFormFactory::number(basic.widget,"surfaceDeviation",1e-9,1e12);
    angle=DialogFormFactory::number(basic.widget,"surfaceNormalAngle",.1,179.9,1,tr(" °"));
    minimum->setToolTip(tr("常规细化下限。小特征、窄间隙和几何要求可突破此值，并报告冲突。"));
    maximum->setToolTip(tr("三维三角边长上限，仅允许数值比较容差。"));
    deviation->setToolTip(tr("曲线及曲面的采样偏差要求，不是已证明的全局误差界。"));
    DialogFormFactory::row(basic,tr("最小单元尺寸"),minimum);
    DialogFormFactory::row(basic,tr("最大单元尺寸"),maximum);
    DialogFormFactory::row(basic,tr("最大几何偏差"),deviation);
    DialogFormFactory::row(basic,tr("最大法向变化角"),angle); layout->addWidget(basic.widget);
    const auto advanced=DialogFormFactory::section(tr("质量与预算"),page);
    legacyAdvanced=advanced.widget;
    quality=DialogFormFactory::number(advanced.widget,"surfaceQuality",.500001,1e6);
    growth=DialogFormFactory::number(advanced.widget,"surfaceGrowth",1,100);
    quality->setToolTip(tr("外接圆半径/最短边之比，越小质量要求越高；尖角和窄区是软目标。"));
    growth->setToolTip(tr("边界采样中相邻目标尺寸的增长比例；当前不保证全域尺寸渐变上界。"));
    vertices=DialogFormFactory::integer(advanced.widget,"surfaceVertexBudget",1,std::numeric_limits<int>::max());
    work=DialogFormFactory::integer(advanced.widget,"surfaceWorkBudget",1,std::numeric_limits<int>::max());
    DialogFormFactory::row(advanced,tr("质量目标"),quality);
    DialogFormFactory::row(advanced,tr("边界尺寸渐变率"),growth);
    DialogFormFactory::row(advanced,tr("最大节点数"),vertices); DialogFormFactory::row(advanced,tr("细化工作预算"),work);
    auto* expand=DialogFormFactory::disclosure(tr("高级设置"),advanced.widget,page);
    expand->setObjectName("meshAdvancedToggle"); layout->addWidget(expand); layout->addWidget(advanced.widget);
    const auto a6=DialogFormFactory::section(tr("Algorithm 6 实验参数"),page);algorithm6Page=a6.widget;
    algorithm6Target=DialogFormFactory::number(a6.widget,"algorithm6Target",1e-9,1e12);
    algorithm6Target->setToolTip(tr("尺寸场上限 h，模型坐标单位；不是最终三角边长或几何误差保证。"));
    algorithm6Extend=new QCheckBox(tr("从边界传播尺寸"),a6.widget);algorithm6Extend->setObjectName("algorithm6Extend");
    algorithm6Smoothing=DialogFormFactory::integer(a6.widget,"algorithm6Smoothing",0,100);
    algorithm6Nodes=DialogFormFactory::integer(a6.widget,"algorithm6Nodes",1,std::numeric_limits<int>::max());
    algorithm6Queries=DialogFormFactory::integer(a6.widget,"algorithm6Queries",1,std::numeric_limits<int>::max());
    DialogFormFactory::row(a6,tr("目标尺寸 h"),algorithm6Target);a6.form->addRow(algorithm6Extend);
    DialogFormFactory::row(a6,tr("原算法平滑轮数"),algorithm6Smoothing);
    DialogFormFactory::row(a6,tr("最大节点数"),algorithm6Nodes);DialogFormFactory::row(a6,tr("几何查询预算"),algorithm6Queries);
    a6.form->addRow(DialogFormFactory::note(tr("串行、一阶；点/曲率尺寸关闭，失败不切换算法。生成后可显示和保存；质量与 CAD 距离须独立审计。"),a6.widget));
    layout->addWidget(a6.widget);
    connect(surfaceAlgorithm,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this]{updateSurfaceAlgorithm();});
    legacyHint=DialogFormFactory::note(QString(),page); legacyHint->setObjectName("meshLegacyHint"); layout->addWidget(legacyHint);
    auto* sizeHint=DialogFormFactory::note(tr("最小尺寸为常规下限，小特征处允许例外。"),page);
    sizeHint->setObjectName("surfaceLegacySizeHint");layout->addWidget(sizeHint);
    layout->addStretch(); return page;
}

/// 构造当前支持的体网格尺寸和优化设置页。
QWidget* MeshSettingsDialog::buildVolumePage() {
    auto* page=new QWidget(this); auto* layout=new QVBoxLayout(page);
    layout->setContentsMargins(0,14,0,0); layout->setSpacing(14);
    const auto group=DialogFormFactory::section(QString(),page);
    volumeTarget=DialogFormFactory::number(group.widget,"volumeTarget",1e-9,1e12);
    optimize=new QCheckBox(tr("生成后优化单元质量"),group.widget); optimize->setObjectName("volumeOptimize");
    optimize->setToolTip(tr("当前仅支持目标尺寸和质量优化；硬最小尺寸、HXT、T10 和局部体尺寸尚未实现。"));
    DialogFormFactory::row(group,tr("目标单元尺寸"),volumeTarget); group.form->addRow(QString(),optimize);
    DialogFormFactory::row(group,tr("单元类型"),new QLabel(tr("T4 线性四面体"),group.widget));
    DialogFormFactory::row(group,tr("生成算法"),new QLabel(tr("自研 Delaunay 与细化"),group.widget)); layout->addWidget(group.widget);
    layout->addWidget(DialogFormFactory::note(tr("目标尺寸是细化目标，不是严格边长上限。"),page));
    layout->addStretch(); return page;
}

/// 将表面参数填入对应控件。
void MeshSettingsDialog::fillSurface(const mesher::SurfaceCriteria& c) {
    surfaceAlgorithm->setCurrentIndex(c.pointPlacement==mesher::SurfacePointPlacement::Algorithm6Experimental?1:0);
    algorithm6Target->setValue(c.algorithm6.targetSize>0?c.algorithm6.targetSize:modelDiagonal*.025);
    algorithm6Extend->setChecked(c.algorithm6.extendBoundarySize!=0);algorithm6Smoothing->setValue(c.algorithm6.smoothingIterations);
    algorithm6Nodes->setValue(c.algorithm6.maximumNodes);algorithm6Queries->setValue(c.algorithm6.maximumGeometryEvaluations);
    const double target=c.targetSize>0?c.targetSize:modelDiagonal*.025;
    maximum->setValue(c.maxSize>0?c.maxSize:target*(c.targetSize>0?1.05:1));
    minimum->setValue(c.minSize>0?c.minSize:target*.1);
    deviation->setValue(c.maxDeviation>0?c.maxDeviation:target*.1); angle->setValue(c.maxNormalAngleDeg);
    quality->setValue(c.maxRadiusEdgeRatio); growth->setValue(c.growthRatio);
    vertices->setValue(c.maxVertices); work->setValue(c.maxIterations);
    const bool legacy=c.maxSize==0 && c.targetSize>0;
    legacyHint->setText(legacy?tr("旧参数使用目标边长及 5% 余量；最大尺寸已按原允许值填入。点击生成后采用这里的显式上限，取消不会改动旧参数。"):QString());
    legacyHint->setVisible(legacy);
    updateSurfaceAlgorithm();
}
/// 返回当前选中的表面或体网格设置页。
MeshSettingsDialog::Mode MeshSettingsDialog::mode() const {return tabs->currentIndex()==0?Mode::Surface:Mode::Volume;}
/// 从控件读取本次表面生成参数草稿。
mesher::SurfaceCriteria MeshSettingsDialog::surfaceCriteria() const {
    if(surfaceAlgorithm->currentData().toInt()==int(mesher::SurfacePointPlacement::Algorithm6Experimental)) {
        auto c=originalSurface;c.pointPlacement=mesher::SurfacePointPlacement::Algorithm6Experimental;
        c.crossSmoothCadSplits=false;c.algorithm6.targetSize=algorithm6Target->value();
        c.algorithm6.extendBoundarySize=algorithm6Extend->isChecked()?1:0;c.algorithm6.smoothingIterations=algorithm6Smoothing->value();
        c.algorithm6.maximumNodes=algorithm6Nodes->value();c.algorithm6.maximumGeometryEvaluations=algorithm6Queries->value();return c;
    }
    auto c=originalSurface; c.targetSize=c.maxSize=maximum->value(); c.minSize=minimum->value();
    c.pointPlacement=static_cast<mesher::SurfacePointPlacement>(surfaceAlgorithm->currentData().toInt());
    c.maxDeviation=deviation->value(); c.maxNormalAngleDeg=angle->value();
    c.maxRadiusEdgeRatio=quality->value(); c.growthRatio=growth->value();
    c.maxVertices=vertices->value(); c.maxIterations=work->value(); return c;
}
/// 根据选中的算法切换可见参数组。
void MeshSettingsDialog::updateSurfaceAlgorithm() {
    const bool experimental=surfaceAlgorithm->currentData().toInt()==int(mesher::SurfacePointPlacement::Algorithm6Experimental);
    legacyBasic->setEnabled(!experimental);legacyBasic->setVisible(!experimental);
    legacyAdvanced->setEnabled(!experimental);findChild<QToolButton*>("meshAdvancedToggle")->setVisible(!experimental);
    if(experimental)legacyAdvanced->hide();
    else legacyAdvanced->setVisible(findChild<QToolButton*>("meshAdvancedToggle")->isChecked());
    algorithm6Page->setVisible(experimental);
    findChild<QLabel*>("surfaceLegacySizeHint")->setVisible(!experimental);
    if(legacyHint)legacyHint->setVisible(!experimental&&!legacyHint->text().isEmpty());
}
/// 读取当前支持的体网格生成参数。
TetMeshParams MeshSettingsDialog::volumeParameters() const {
    auto p=originalVolume; p.targetSize=volumeTarget->value(); p.optimize=optimize->isChecked();
    // 原工程的预留选项不能冒称有效；当前只生成线性四面体。
    p.elementOrder=1; p.algo3D=1; return p;
}
/// 根据当前参数页更新生成按钮和提示状态。
void MeshSettingsDialog::updateMode() {
    generateButton->setText(mode()==Mode::Surface?tr("生成三角面片"):tr("生成四面体"));
    errorLabel->hide();
    if(mode()==Mode::Volume) findChild<QToolButton*>("meshAdvancedToggle")->setChecked(false);
}
/// 将当前参数页恢复为基于模型尺度的默认设置。
void MeshSettingsDialog::resetCurrentPage() {
    if(mode()==Mode::Surface) {
        mesher::SurfaceCriteria c; c.targetSize=c.maxSize=modelDiagonal*.025;
        c.minSize=c.maxDeviation=c.maxSize*.1; fillSurface(c);
    } else {volumeTarget->setValue(modelDiagonal*.05);optimize->setChecked(true);}
    errorLabel->hide();
}
/// 提交输入并检查参数，合法后接受对话框。
void MeshSettingsDialog::accept() {
    for(auto* spin:findChildren<CompactDoubleSpinBox*>()) spin->commitInput();
    if(mode()==Mode::Surface && surfaceAlgorithm->currentIndex()==0 && minimum->value()>maximum->value()) {
        errorLabel->setText(tr("最小单元尺寸不能大于最大单元尺寸。")); errorLabel->show(); minimum->setFocus(); return;
    }
    // 预算小于 CAD 顶点数等几何相关错误由引擎返回，不能在 Dialog 猜测。
    QDialog::accept();
}
/// 在窗口显示后应用平台标题栏样式。
void MeshSettingsDialog::showEvent(QShowEvent* event) {
    QDialog::showEvent(event); QTimer::singleShot(0,this,[this]{MeshChrome::styleTitleBar(this);});
}
