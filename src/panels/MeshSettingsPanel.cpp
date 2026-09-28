#include "MeshSettingsPanel.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QTableWidget>
#include <QVBoxLayout>

MeshSettingsPanel::MeshSettingsPanel(QWidget* parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("panelSurface"));

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(10, 10, 10, 10);
    rootLayout->setSpacing(10);

    rootLayout->addWidget(buildGlobalGroup());
    rootLayout->addWidget(buildQualityGroup());
    rootLayout->addWidget(buildLocalGroup());
    rootLayout->addWidget(buildConformalGroup());

    m_btnGenerate = new QPushButton(tr("▶  生成网格"), this);
    m_btnGenerate->setObjectName("primaryButton");
    m_btnGenerate->setMinimumHeight(36);
    m_btnGenerate->setCursor(Qt::PointingHandCursor);
    connect(m_btnGenerate, &QPushButton::clicked,
            this, &MeshSettingsPanel::generateRequested);
    rootLayout->addWidget(m_btnGenerate);

    m_progress = new QProgressBar(this);
    m_progress->setTextVisible(false);
    m_progress->setFixedHeight(6);
    m_progress->hide();
    rootLayout->addWidget(m_progress);

    auto* note = new QLabel(
        tr("引擎：自研 Delaunay 细化（增量 Delaunay + 约束恢复 + Ruppert 细化）。"
           "v1 仅支持 STL 输入、T4 一阶单元。"), this);
    note->setObjectName("dimLabel");
    note->setWordWrap(true);
    rootLayout->addWidget(note);

    rootLayout->addStretch(1);
}

QWidget* MeshSettingsPanel::buildGlobalGroup()
{
    auto* group = new QGroupBox(tr("全局设置"), this);
    auto* form = new QFormLayout(group);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setSpacing(8);

    m_targetSize = new QDoubleSpinBox(group);
    m_targetSize->setRange(0.001, 10000.0);
    m_targetSize->setValue(1.00);
    m_targetSize->setSingleStep(0.1);
    m_targetSize->setDecimals(3);
    m_targetSize->setSuffix(tr(" mm"));

    m_minSize = new QDoubleSpinBox(group);
    m_minSize->setRange(0.0, 1000.0);
    m_minSize->setValue(0.10);
    m_minSize->setSingleStep(0.05);
    m_minSize->setDecimals(4);
    m_minSize->setSuffix(tr(" mm"));

    m_curvature = new QDoubleSpinBox(group);
    m_curvature->setRange(1.0, 89.0);
    m_curvature->setValue(30.0);
    m_curvature->setDecimals(0);
    m_curvature->setSuffix(tr(" °"));

    m_elementOrder = new QComboBox(group);
    m_elementOrder->addItem(tr("一阶（T4 线性）"));
    m_elementOrder->addItem(tr("二阶（T10 二次）"));
    m_elementOrder->setItemData(1, tr("应力分析推荐：二阶单元精度远高于一阶"), Qt::ToolTipRole);

    form->addRow(tr("目标网格尺寸"), m_targetSize);
    form->addRow(tr("最小网格尺寸"), m_minSize);
    form->addRow(tr("曲率细分角度"), m_curvature);
    form->addRow(tr("单元阶次"), m_elementOrder);
    return group;
}

QWidget* MeshSettingsPanel::buildQualityGroup()
{
    auto* group = new QGroupBox(tr("质量与算法"), this);
    auto* form = new QFormLayout(group);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setSpacing(8);

    m_algo3D = new QComboBox(group);
    m_algo3D->addItem(tr("Delaunay（默认）"));
    m_algo3D->addItem(tr("HXT（并行加速）"));

    m_optimize = new QCheckBox(tr("生成后优化单元质量"), group);
    m_optimize->setChecked(true);

    m_optimizeThreshold = new QDoubleSpinBox(group);
    m_optimizeThreshold->setRange(0.0, 1.0);
    m_optimizeThreshold->setSingleStep(0.05);
    m_optimizeThreshold->setDecimals(2);
    m_optimizeThreshold->setValue(0.3);

    form->addRow(tr("3D 算法"), m_algo3D);
    form->addRow(QString(), m_optimize);
    form->addRow(tr("优化阈值（gamma）"), m_optimizeThreshold);
    return group;
}

QWidget* MeshSettingsPanel::buildLocalGroup()
{
    auto* group = new QGroupBox(tr("局部加密"), this);
    auto* layout = new QVBoxLayout(group);
    layout->setSpacing(8);

    m_localTable = new QTableWidget(0, 2, group);
    m_localTable->setHorizontalHeaderLabels({tr("区域"), tr("尺寸 (mm)")});
    m_localTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    m_localTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_localTable->verticalHeader()->setVisible(false);
    m_localTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_localTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_localTable->setMinimumHeight(110);
    layout->addWidget(m_localTable);

    auto* buttons = new QHBoxLayout();
    buttons->addStretch(1);
    auto* addBtn = new QPushButton(QIcon(":/icons/plus.svg"), tr("添加"), group);
    auto* removeBtn = new QPushButton(QIcon(":/icons/minus.svg"), tr("移除"), group);
    connect(addBtn, &QPushButton::clicked, this, [this] {
        const int row = m_localTable->rowCount();
        m_localTable->insertRow(row);
        m_localTable->setItem(row, 0, new QTableWidgetItem(tr("选择区域…")));
        m_localTable->setItem(row, 1, new QTableWidgetItem(QStringLiteral("0.5")));
    });
    connect(removeBtn, &QPushButton::clicked, this, [this] {
        const auto rows = m_localTable->selectionModel()->selectedRows();
        for (int i = rows.size() - 1; i >= 0; --i)
            m_localTable->removeRow(rows.at(i).row());
    });
    buttons->addWidget(addBtn);
    buttons->addWidget(removeBtn);
    layout->addLayout(buttons);
    return group;
}

QWidget* MeshSettingsPanel::buildConformalGroup()
{
    auto* group = new QGroupBox(tr("共形设置"), this);
    auto* form = new QFormLayout(group);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setSpacing(8);

    auto* enable = new QCheckBox(tr("启用共形网格"), group);
    enable->setChecked(true);

    auto* thinWall = new QComboBox(group);
    thinWall->addItems({tr("自动处理"), tr("强制切割"), tr("暂忽略")});

    auto* tolerance = new QDoubleSpinBox(group);
    tolerance->setRange(0.0, 0.001);
    tolerance->setDecimals(7);
    tolerance->setSingleStep(0.0000001);
    tolerance->setValue(0.0000001);
    tolerance->setSuffix(tr(" m"));

    form->addRow(QString(), enable);
    form->addRow(tr("薄片处理"), thinWall);
    form->addRow(tr("判断容差"), tolerance);
    return group;
}

TetMeshParams MeshSettingsPanel::parameters() const
{
    TetMeshParams params;
    params.targetSize = m_targetSize->value();
    params.minSize = m_minSize->value();
    params.curvatureAngle = m_curvature->value();
    params.elementOrder = m_elementOrder->currentIndex() == 1 ? 2 : 1;
    params.algo3D = m_algo3D->currentIndex() == 1 ? 10 : 1;
    params.optimize = m_optimize->isChecked();
    params.optimizeThreshold = m_optimizeThreshold->value();
    return params;
}

void MeshSettingsPanel::applyParams(const TetMeshParams& params)
{
    m_targetSize->setValue(params.targetSize > 0.0 ? params.targetSize : 1.0);
    m_minSize->setValue(params.minSize > 0.0 ? params.minSize : 0.0);
    m_curvature->setValue(qBound(1.0, params.curvatureAngle, 89.0));
    m_elementOrder->setCurrentIndex(params.elementOrder == 2 ? 1 : 0);
    m_algo3D->setCurrentIndex(params.algo3D == 10 ? 1 : 0);
    m_optimize->setChecked(params.optimize);
    m_optimizeThreshold->setValue(params.optimizeThreshold);
}

void MeshSettingsPanel::setDefaultTargetSize(double diagonal)
{
    if (diagonal <= 0.0)
        return;
    const double target = diagonal * 0.025;
    m_targetSize->setValue(target);
    m_minSize->setValue(target * 0.1);
}

void MeshSettingsPanel::setGenerating(bool generating)
{
    m_btnGenerate->setEnabled(!generating);
    m_progress->setVisible(generating);
    if (generating) {
        m_progress->setRange(0, 0);
    } else {
        m_progress->setRange(0, 1);
        m_progress->setValue(1);
    }
}
