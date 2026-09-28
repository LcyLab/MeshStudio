#include "ModelTransformDialog.h"

#include "core/WindowChrome.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace {
QDoubleSpinBox* makeSpin(double minv, double maxv, double value, double step,
                         const QString& suffix, QWidget* parent)
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(minv, maxv);
    spin->setDecimals(3);
    spin->setSingleStep(step);
    spin->setValue(value);
    spin->setSuffix(suffix);
    return spin;
}
} // namespace

ModelTransformDialog::ModelTransformDialog(const QString& modelName, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("模型变换 — %1").arg(modelName));
    setMinimumWidth(360);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setSpacing(10);

    auto* translateGroup = new QFormLayout;
    translateGroup->setSpacing(8);
    const QString axes[3] = { tr("X"), tr("Y"), tr("Z") };
    for (int i = 0; i < 3; ++i) {
        m_translate[i] = makeSpin(-100000.0, 100000.0, 0.0, 1.0, tr(" mm"), this);
        translateGroup->addRow(tr("平移 %1").arg(axes[i]), m_translate[i]);
    }

    auto* rotateGroup = new QFormLayout;
    rotateGroup->setSpacing(8);
    const QString rotateNames[3] = { tr("绕 X 轴"), tr("绕 Y 轴"), tr("绕 Z 轴") };
    for (int i = 0; i < 3; ++i) {
        m_rotate[i] = makeSpin(-360.0, 360.0, 0.0, 1.0, tr(" °"), this);
        rotateGroup->addRow(rotateNames[i], m_rotate[i]);
    }

    m_scale = new QDoubleSpinBox(this);
    m_scale->setRange(0.001, 10000.0);
    m_scale->setDecimals(3);
    m_scale->setSingleStep(0.1);
    m_scale->setValue(1.0);

    auto* scaleGroup = new QFormLayout;
    scaleGroup->addRow(tr("均匀缩放"), m_scale);

    auto* translateBox = new QVBoxLayout;
    translateBox->addLayout(translateGroup);
    rootLayout->addLayout(translateBox);
    rootLayout->addLayout(rotateGroup);
    rootLayout->addLayout(scaleGroup);
    rootLayout->addStretch(1);

    auto* buttons = new QHBoxLayout;
    auto* resetButton = new QPushButton(tr("复位变换"), this);
    resetButton->setToolTip(tr("平移/旋转归零，缩放归一"));
    connect(resetButton, &QPushButton::clicked, this, [this] {
        for (int i = 0; i < 3; ++i) {
            m_translate[i]->setValue(0.0);
            m_rotate[i]->setValue(0.0);
        }
        m_scale->setValue(1.0);
        emit transformResetRequested();
    });
    buttons->addWidget(resetButton);
    buttons->addStretch(1);
    auto* okButton = new QPushButton(tr("确定"), this);
    okButton->setDefault(true);
    buttons->addWidget(okButton);
    auto* cancelButton = new QPushButton(tr("取消"), this);
    buttons->addWidget(cancelButton);
    rootLayout->addLayout(buttons);

    connect(okButton, &QPushButton::clicked, this, &QDialog::accept);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
}

void ModelTransformDialog::setValues(double tx, double ty, double tz,
                                     double rx, double ry, double rz, double scale)
{
    m_translate[0]->setValue(tx);
    m_translate[1]->setValue(ty);
    m_translate[2]->setValue(tz);
    m_rotate[0]->setValue(rx);
    m_rotate[1]->setValue(ry);
    m_rotate[2]->setValue(rz);
    m_scale->setValue(scale);
}

void ModelTransformDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    // 延迟到模态事件循环空闲再染标题栏：exec() 运行嵌套事件循环，
    // 直接在 show 期间调用 DWM/SetWindowPos(SWP_FRAMECHANGED) 会干扰主窗口 dock 重绘
    QTimer::singleShot(0, this, [this] { MeshChrome::styleTitleBar(this); });
}

double ModelTransformDialog::translateX() const { return m_translate[0]->value(); }
double ModelTransformDialog::translateY() const { return m_translate[1]->value(); }
double ModelTransformDialog::translateZ() const { return m_translate[2]->value(); }
double ModelTransformDialog::rotateX() const { return m_rotate[0]->value(); }
double ModelTransformDialog::rotateY() const { return m_rotate[1]->value(); }
double ModelTransformDialog::rotateZ() const { return m_rotate[2]->value(); }
double ModelTransformDialog::scale() const { return m_scale->value(); }
