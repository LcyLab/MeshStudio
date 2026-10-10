// 文件职责：显示变换编辑：收集平移、旋转和缩放参数，确认后交给主窗口应用到视口。
#include "ModelTransformDialog.h"

#include "ui/platform/WindowChrome.h"
#include "ui/forms/DialogFormFactory.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace {
/// 创建指定范围、步长和单位后缀的数值控件。
QDoubleSpinBox* makeSpin(double minv, double maxv, double value, double step,
                         const QString& suffix, QWidget* parent)
{
    auto* spin = DialogFormFactory::number(parent,"transformValue",minv,maxv,step,suffix,3);
    spin->setValue(value);
    return spin;
}
} // namespace

/// 构造模型显示变换参数对话框。
ModelTransformDialog::ModelTransformDialog(const QString& modelName, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("模型变换 — %1").arg(modelName));
    DialogFormFactory::configure(this,QSize(520,480));

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(22,18,22,18);
    rootLayout->setSpacing(10);
    rootLayout->addWidget(DialogFormFactory::header(tr("模型变换"),modelName,this));

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

    m_scale = DialogFormFactory::number(this,"transformScale",0.001,10000.0,.1,QString(),3);
    m_scale->setValue(1.0);

    auto* scaleGroup = new QFormLayout;
    scaleGroup->addRow(tr("均匀缩放"), m_scale);

    auto* translateBox = new QVBoxLayout;
    translateBox->addLayout(translateGroup);
    rootLayout->addLayout(translateBox);
    rootLayout->addLayout(rotateGroup);
    rootLayout->addLayout(scaleGroup);
    rootLayout->addStretch(1);

    const auto buttons=DialogFormFactory::actions(tr("确定"),this,tr("复位变换"));
    auto* resetButton=buttons.reset;
    resetButton->setToolTip(tr("平移/旋转归零，缩放归一"));
    connect(resetButton, &QPushButton::clicked, this, [this] {
        for (int i = 0; i < 3; ++i) {
            m_translate[i]->setValue(0.0);
            m_rotate[i]->setValue(0.0);
        }
        m_scale->setValue(1.0);
        emit transformResetRequested();
    });
    rootLayout->addWidget(buttons.widget);

    connect(buttons.primary, &QPushButton::clicked, this, &QDialog::accept);
    connect(buttons.cancel, &QPushButton::clicked, this, &QDialog::reject);
}

/// 将平移、旋转和缩放参数填入控件。
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

/// 在窗口显示后应用平台标题栏样式。
void ModelTransformDialog::showEvent(QShowEvent* event)
{
    QDialog::showEvent(event);
    // 延迟到模态事件循环空闲再染标题栏：exec() 运行嵌套事件循环，
    // 直接在 show 期间调用 DWM/SetWindowPos(SWP_FRAMECHANGED) 会干扰主窗口 dock 重绘
    QTimer::singleShot(0, this, [this] { MeshChrome::styleTitleBar(this); });
}

/// 读取 X 方向的平移量。
double ModelTransformDialog::translateX() const { return m_translate[0]->value(); }
/// 读取 Y 方向的平移量。
double ModelTransformDialog::translateY() const { return m_translate[1]->value(); }
/// 读取 Z 方向的平移量。
double ModelTransformDialog::translateZ() const { return m_translate[2]->value(); }
/// 读取绕 X 轴的旋转角度，单位为度。
double ModelTransformDialog::rotateX() const { return m_rotate[0]->value(); }
/// 读取绕 Y 轴的旋转角度，单位为度。
double ModelTransformDialog::rotateY() const { return m_rotate[1]->value(); }
/// 读取绕 Z 轴的旋转角度，单位为度。
double ModelTransformDialog::rotateZ() const { return m_rotate[2]->value(); }
/// 读取统一缩放比例。
double ModelTransformDialog::scale() const { return m_scale->value(); }
