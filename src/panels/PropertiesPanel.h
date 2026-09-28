#pragma once

#include <QPointer>
#include <QScrollArea>

class QPushButton;
class QSlider;
class QVBoxLayout;
class QWidget;

// 右侧属性面板：与网格参数面板一致的「分组卡片 + 表单行」布局
// （QGroupBox 白色卡片 + 紫色标题 + 名称|值 表单行）
class PropertiesPanel : public QScrollArea
{
    Q_OBJECT

public:
    explicit PropertiesPanel(QWidget* parent = nullptr);

    void showModel(int modelId, const QString& name, int solids, int faces,
                   int edges, bool visible, const QColor& color, double opacity,
                   const double range[6], bool rangeValid, const QString& unitHint);
    void updateColorSwatch(int modelId, const QColor& color);
    void showPlaceholder(const QString& title, const QString& hint);

signals:
    void visibilityToggleRequested(int modelId, bool visible);
    void modelOpacityChanged(int modelId, double opacity);
    void colorChangeRequested(int modelId);

private:
    void clearContent();

    QWidget* m_content = nullptr;
    QVBoxLayout* m_layout = nullptr;
    QPointer<QPushButton> m_swatchButton;
    int m_currentModelId = -1;
};
