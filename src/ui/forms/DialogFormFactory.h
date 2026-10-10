#pragma once
/** @file DialogFormFactory.h
 * @brief 可复用的 Qt 参数表单组件；显示格式与 double 数值精度分离。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include <QDoubleSpinBox>
#include <QSize>
class QDialog;
class QFormLayout;
class QLabel;
class QPushButton;
class QSpinBox;
class QToolButton;

/// 非编辑时显示最多三位小数；极小/大数使用科学计数。
/// 编辑使用 round-trip double 字符串。显示舍入不修改 value()。
class CompactDoubleSpinBox final : public QDoubleSpinBox {
    Q_OBJECT
public:
    /// 创建可紧凑显示并保留完整输入精度的数值控件。
    explicit CompactDoubleSpinBox(QWidget* parent=nullptr);
    /// 未获得焦点时保持精确值，供弹窗在接受前提交实际输入。
    void commitInput();
protected:
    /// 处理精确数值提示，其余事件交给基类。
    bool event(QEvent*) override;
    /// 根据编辑状态和数值尺度生成显示文字。
    QString textFromValue(double value) const override;
    /// 解析本地格式的数值输入，无效时保留原值。
    double valueFromText(const QString& text) const override;
    /// 校验本地格式的数值输入，支持科学计数法编辑状态。
    QValidator::State validate(QString& text,int& position) const override;
    /// 进入编辑状态并显示完整精度的数值。
    void focusInEvent(QFocusEvent*) override;
    /// 结束编辑并恢复紧凑数值显示。
    void focusOutEvent(QFocusEvent*) override;
    /// 仅在控件具有焦点时接受滚轮调节。
    void wheelEvent(QWheelEvent*) override;
    /// 先提交输入，再按指定步数调整数值。
    void stepBy(int steps) override;
private:
    /// 去除数值输入的前后缀并保留数值部分。
    QString numericPart(QString text) const;
    /// 刷新完整或紧凑数值文字，避免触发用户输入解析。
    void writeValue(bool complete);
    bool editing=false;
};

/// 工厂只创建 Qt 控件及布局；返回的控件由指定 parent 或布局父对象持有。
/// 不持有业务参数，不启动算法；Dialog 自行负责草稿及接受/取消语义。
class DialogFormFactory final {
public:
    struct Section { QWidget* widget; QFormLayout* form; };
    struct Actions { QWidget* widget; QPushButton *reset,*cancel,*primary; };
    /// 设置表单对话框的尺寸和主题属性。
    static void configure(QDialog* dialog,const QSize& size=QSize(540,460));
    /// 创建表单标题及副标题区域。
    static QWidget* header(const QString& title,const QString& subtitle,QWidget* parent);
    /// 创建可换行的表单说明文字。
    static QLabel* note(const QString& text,QWidget* parent);
    /// 创建包含可选标题和字段布局的表单分组。
    static Section section(const QString& title,QWidget* parent);
    /// 为表单分组添加标签及输入控件。
    static void row(const Section& section,const QString& label,QWidget* field);
    /// storageDecimals=-1 保留完整 double；明确传入值时保留旧控件存储精度。
    static CompactDoubleSpinBox* number(QWidget* parent,const char* name,double minimum,
        double maximum,double step=.1,const QString& suffix=QString(),int storageDecimals=-1);
    /// 创建整数输入控件并设置取值范围。
    static QSpinBox* integer(QWidget* parent,const char* name,int minimum,int maximum);
    /// 展开时在屏幕可用高度内增高 Dialog；折叠恢复原高度，长内容仍可滚动。
    static QToolButton* disclosure(const QString& text,QWidget* content,QWidget* parent);
    /// 主按钮先提交正在编辑的精确数值，再进入调用方连接的槽。
    static Actions actions(const QString& primaryText,QWidget* parent,
                           const QString& resetText=QString());
};
