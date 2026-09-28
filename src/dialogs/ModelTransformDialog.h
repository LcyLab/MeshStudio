#pragma once

#include <QDialog>

class QDoubleSpinBox;

/// 模型变换对话框: 平移 / 旋转 / 缩放（绝对值，确定后一次性应用）
class ModelTransformDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ModelTransformDialog(const QString& modelName, QWidget* parent = nullptr);

    void setValues(double tx, double ty, double tz,
                   double rx, double ry, double rz, double scale);
    double translateX() const;
    double translateY() const;
    double translateZ() const;
    double rotateX() const;
    double rotateY() const;
    double rotateZ() const;
    double scale() const;

signals:
    void transformResetRequested();

protected:
    void showEvent(QShowEvent* event) override;

private:
    QDoubleSpinBox* m_translate[3] = {};
    QDoubleSpinBox* m_rotate[3] = {};
    QDoubleSpinBox* m_scale = nullptr;
};
