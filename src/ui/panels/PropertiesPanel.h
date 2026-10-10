#pragma once
// 文件职责：属性界面：展示所选模型的范围和外观，通过信号请求修改可见性、透明度和颜色。

#include <QPointer>
#include <QScrollArea>

class QPushButton;
class QVBoxLayout;
class QWidget;

// 右侧属性面板：沿用应用的「分组卡片 + 表单行」布局
// （QGroupBox 白色卡片 + 紫色标题 + 名称|值 表单行）
class PropertiesPanel : public QScrollArea
{
    Q_OBJECT

public:
    /// 创建模型属性、外观和质量信息面板。
    explicit PropertiesPanel(QWidget* parent = nullptr);

    /// 重建所选模型属性；range 为 xmin/xmax、ymin/ymax、zmin/zmax，rangeValid=false 时不显示尺寸。
    /// unitHint 由加载来源提供，未知单位不擅自标为毫米；编辑外观只发信号，不改网格。
    void showModel(int modelId, const QString& name, int solids, int faces,
                   int edges, bool visible, const QColor& color, double opacity,
                   const double range[6], bool rangeValid, const QString& unitHint);
    /// 更新当前模型的颜色预览按钮。
    void updateColorSwatch(int modelId, const QColor& color);
    /// 显示尚未关联具体模型的属性提示。
    void showPlaceholder(const QString& title, const QString& hint);

signals:
    /// 通知主窗口切换指定模型的可见性。
    void visibilityToggleRequested(int modelId, bool visible);
    /// 通知视口更新模型透明度。
    void modelOpacityChanged(int modelId, double opacity);
    /// 通知主窗口修改指定模型的颜色。
    void colorChangeRequested(int modelId);

private:
    /// 清空当前模型的属性显示。
    void clearContent();

    QWidget* m_content = nullptr;
    QVBoxLayout* m_layout = nullptr;
    QPointer<QPushButton> m_swatchButton;
    int m_currentModelId = -1;
};
