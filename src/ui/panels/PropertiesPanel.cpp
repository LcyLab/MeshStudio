// 文件职责：属性界面：展示所选模型的范围和外观，通过信号请求修改可见性、透明度和颜色。
#include "PropertiesPanel.h"

#include <QCheckBox>
#include <QColor>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

namespace {

// 与 light.qss 主题 token 保持一致（深茄紫系），避免面板与主题脱节
QString swatchStyle(const QColor& color)
{
    return QStringLiteral(
               "QPushButton { background-color: %1; color: #ffffff;"
               " border: 1px solid #D5DAE0; border-radius: 4px; padding: 3px 14px; }"
               " QPushButton:hover { border-color: #583266; }")
        .arg(color.name());
}

/// 将属性数值格式化为紧凑的可读文字。
QString formatDisplayNumber(double value)
{
    QString text = QString::number(value, 'g', 6);
    return text == QStringLiteral("-0") ? QStringLiteral("0") : text;
}

} // namespace

/// 创建模型属性、外观和质量信息面板。
PropertiesPanel::PropertiesPanel(QWidget* parent)
    : QScrollArea(parent)
{
    setWidgetResizable(true);
    setFrameShape(QFrame::NoFrame);

    m_content = new QWidget(this);
    m_content->setObjectName(QStringLiteral("panelSurface"));
    m_layout = new QVBoxLayout(m_content);
    m_layout->setContentsMargins(10, 8, 10, 10);
    m_layout->setSpacing(10);
    setWidget(m_content);

    showPlaceholder(tr("属性"), tr("在左侧模型树中选择节点后，这里显示对应信息。"));
}

/// 清空当前模型的属性显示。
void PropertiesPanel::clearContent()
{
    while (QLayoutItem* item = m_layout->takeAt(0)) {
        if (QWidget* w = item->widget())
            w->deleteLater();
        delete item;
    }
}

/// 显示选中模型的文件、范围、外观和网格统计。
void PropertiesPanel::showModel(int modelId, const QString& name, int solids,
                                int faces, int edges, bool visible,
                                const QColor& color, double opacity,
                                const double range[6], bool rangeValid,
                                const QString& unitHint)
{
    clearContent();
    m_currentModelId = modelId;
    m_swatchButton.clear();

    // ===== 基本信息 =====
    auto* base = new QGroupBox(tr("基本信息"), m_content);
    auto* baseForm = new QFormLayout(base);
    baseForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    baseForm->setSpacing(8);

    baseForm->addRow(tr("文件名"), new QLabel(name, base));

    auto* visCheck = new QCheckBox(tr("显示模型"), base);
    visCheck->setChecked(visible);
    connect(visCheck, &QCheckBox::toggled, this, [this](bool on) {
        if (m_currentModelId >= 0)
            emit visibilityToggleRequested(m_currentModelId, on);
    });
    baseForm->addRow(tr("可见"), visCheck);

    auto* swatch = new QPushButton(base);
    swatch->setCursor(Qt::PointingHandCursor);
    swatch->setStyleSheet(swatchStyle(color));
    swatch->setText(color.name().toUpper());
    swatch->setToolTip(tr("点击更改模型颜色"));
    connect(swatch, &QPushButton::clicked, this, [this] {
        if (m_currentModelId >= 0)
            emit colorChangeRequested(m_currentModelId);
    });
    baseForm->addRow(tr("颜色"), swatch);
    m_swatchButton = swatch;

    auto* opacityBox = new QWidget(base);
    auto* opacityRow = new QHBoxLayout(opacityBox);
    opacityRow->setContentsMargins(0, 0, 0, 0);
    opacityRow->setSpacing(8);
    auto* slider = new QSlider(Qt::Horizontal, opacityBox);
    slider->setRange(0, 100);
    slider->setValue(qRound(opacity * 100.0));
    slider->setToolTip(tr("拖动调整模型透明度"));
    auto* percent = new QLabel(QStringLiteral("%1%").arg(slider->value()), opacityBox);
    percent->setObjectName(QStringLiteral("propValue"));
    percent->setMinimumWidth(40);
    opacityRow->addWidget(slider);
    opacityRow->addWidget(percent);
    connect(slider, &QSlider::valueChanged, this, [this, percent](int value) {
        percent->setText(QStringLiteral("%1%").arg(value));
        if (m_currentModelId >= 0)
            emit modelOpacityChanged(m_currentModelId, value / 100.0);
    });
    baseForm->addRow(tr("透明度"), opacityBox);
    m_layout->addWidget(base);

    // ===== 几何信息 =====
    auto* geo = new QGroupBox(tr("几何信息"), m_content);
    auto* geoForm = new QFormLayout(geo);
    geoForm->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    geoForm->setSpacing(8);
    geoForm->addRow(tr("实体数"), new QLabel(QString::number(solids), geo));
    geoForm->addRow(tr("面数"), new QLabel(QString::number(faces), geo));
    geoForm->addRow(tr("边数"), new QLabel(QString::number(edges), geo));
    m_layout->addWidget(geo);

    // ===== 坐标范围（Xmin/Xmax 成对 + 按来源标注单位） =====
    auto* rangeGroup = new QGroupBox(tr("坐标范围"), m_content);
    auto* rangeGrid = new QGridLayout(rangeGroup);
    rangeGrid->setContentsMargins(9, 4, 9, 8);
    rangeGrid->setHorizontalSpacing(10);
    rangeGrid->setVerticalSpacing(6);
    rangeGrid->setColumnStretch(1, 1);
    rangeGrid->setColumnStretch(3, 1);
    const char* axis[3] = { "X", "Y", "Z" };
    const QString suffix = unitHint.isEmpty() ? QString()
                                              : QStringLiteral(" ") + unitHint;
    for (int i = 0; i < 3; ++i) {
        const double lo = rangeValid ? range[2 * i] : 0.0;
        const double hi = rangeValid ? range[2 * i + 1] : 0.0;

        auto* loLabel = new QLabel(QString::fromLatin1(axis[i]) + tr("min"), rangeGroup);
        loLabel->setObjectName(QStringLiteral("propKey"));
        auto* loValue = new QLabel(formatDisplayNumber(lo) + suffix, rangeGroup);
        loValue->setObjectName(QStringLiteral("propValue"));
        auto* hiLabel = new QLabel(QString::fromLatin1(axis[i]) + tr("max"), rangeGroup);
        hiLabel->setObjectName(QStringLiteral("propKey"));
        auto* hiValue = new QLabel(formatDisplayNumber(hi) + suffix, rangeGroup);
        hiValue->setObjectName(QStringLiteral("propValue"));

        rangeGrid->addWidget(loLabel, i, 0);
        rangeGrid->addWidget(loValue, i, 1);
        rangeGrid->addWidget(hiLabel, i, 2);
        rangeGrid->addWidget(hiValue, i, 3);
    }
    m_layout->addWidget(rangeGroup);

    m_layout->addStretch(1);
}

/// 更新当前模型的颜色预览按钮。
void PropertiesPanel::updateColorSwatch(int modelId, const QColor& color)
{
    if (modelId != m_currentModelId || m_swatchButton.isNull())
        return;
    m_swatchButton->setStyleSheet(swatchStyle(color));
    m_swatchButton->setText(color.name().toUpper());
}

/// 显示尚未关联具体模型的属性提示。
void PropertiesPanel::showPlaceholder(const QString& title, const QString& hint)
{
    clearContent();
    m_swatchButton.clear();

    auto* box = new QGroupBox(title, m_content);
    auto* layout = new QVBoxLayout(box);
    auto* hintLabel = new QLabel(hint, box);
    hintLabel->setObjectName(QStringLiteral("propHint"));
    hintLabel->setWordWrap(true);
    layout->addWidget(hintLabel);
    m_layout->addWidget(box);
    m_layout->addStretch(1);
}
