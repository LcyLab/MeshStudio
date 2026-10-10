#pragma once

#include <QDialog>

class QDoubleSpinBox;

/// 模型变换对话框: 平移 / 旋转 / 缩放（绝对值，确定后一次性应用）
class ModelTransformDialog : public QDialog
{
    Q_OBJECT

public:
    /// 构造模型显示变换参数对话框。
    explicit ModelTransformDialog(const QString& modelName, QWidget* parent = nullptr);

    /// 将平移、旋转和缩放参数填入控件。
    void setValues(double tx, double ty, double tz,
                   double rx, double ry, double rz, double scale);
    /// 读取 X 方向的平移量。
    double translateX() const;
    /// 读取 Y 方向的平移量。
    double translateY() const;
    /// 读取 Z 方向的平移量。
    double translateZ() const;
    /// 读取绕 X 轴的旋转角度，单位为度。
    double rotateX() const;
    /// 读取绕 Y 轴的旋转角度，单位为度。
    double rotateY() const;
    /// 读取绕 Z 轴的旋转角度，单位为度。
    double rotateZ() const;
    /// 读取统一缩放比例。
    double scale() const;

signals:
    /// 通知主窗口恢复模型显示变换。
    void transformResetRequested();

protected:
    /// 在窗口显示后应用平台标题栏样式。
    void showEvent(QShowEvent* event) override;

private:
    QDoubleSpinBox* m_translate[3] = {};
    QDoubleSpinBox* m_rotate[3] = {};
    QDoubleSpinBox* m_scale = nullptr;
};
