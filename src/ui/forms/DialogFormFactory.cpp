// 文件职责：表单控件构造：统一字段布局、显示精度和校验规则，数值显示格式与真实 double 值分开。
#include "DialogFormFactory.h"
#include <QDialog>
#include <QDoubleValidator>
#include <QFocusEvent>
#include <QGuiApplication>
#include <QHelpEvent>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QScreen>
#include <QSpinBox>
#include <QToolButton>
#include <QToolTip>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <cmath>
#include <limits>

/// 创建可紧凑显示并保留完整输入精度的数值控件。
CompactDoubleSpinBox::CompactDoubleSpinBox(QWidget* parent):QDoubleSpinBox(parent) {
    // Qt 内部仍使用 double。323 位固定转换避免 setValue 时先按显示小数舍入。
    setDecimals(std::numeric_limits<double>::max_exponent10+std::numeric_limits<double>::digits10);
    auto numericLocale=locale();
    numericLocale.setNumberOptions(numericLocale.numberOptions()|QLocale::OmitGroupSeparator|QLocale::RejectGroupSeparator);
    setLocale(numericLocale); setKeyboardTracking(false);
    setButtonSymbols(QAbstractSpinBox::NoButtons); setAlignment(Qt::AlignRight);
    setMinimumWidth(170); setMinimumHeight(32);
}
/// 处理精确数值提示，其余事件交给基类。
bool CompactDoubleSpinBox::event(QEvent* event) {
    if(event->type()==QEvent::ToolTip) {
        const auto* help=static_cast<QHelpEvent*>(event);
        const QString complete=tr("精确数值：%1").arg(locale().toString(value(),'g',QLocale::FloatingPointShortest));
        QToolTip::showText(help->globalPos(),toolTip().isEmpty()?complete:toolTip()+"\n"+complete,this);
        return true;
    }
    return QDoubleSpinBox::event(event);
}

/// 根据编辑状态和数值尺度生成显示文字。
QString CompactDoubleSpinBox::textFromValue(double value) const {
    if(editing) return locale().toString(value,'g',QLocale::FloatingPointShortest);
    const double magnitude=std::abs(value);
    if(magnitude>0 && (magnitude<.001 || magnitude>=1e7))
        return locale().toString(value,'g',4);
    QString text=locale().toString(value,'f',3);
    while(text.contains(locale().decimalPoint()) && text.endsWith('0')) text.chop(1);
    if(text.endsWith(locale().decimalPoint())) text.chop(1);
    return text=="-0"?QStringLiteral("0"):text;
}
/// 去除数值输入的前后缀并保留数值部分。
QString CompactDoubleSpinBox::numericPart(QString text) const {
    if(!prefix().isEmpty() && text.startsWith(prefix())) text.remove(0,prefix().size());
    if(!suffix().isEmpty() && text.endsWith(suffix())) text.chop(suffix().size());
    return text.trimmed();
}
/// 解析本地格式的数值输入，无效时保留原值。
double CompactDoubleSpinBox::valueFromText(const QString& text) const {
    bool valid=false; const double parsed=locale().toDouble(numericPart(text),&valid);
    return valid && std::isfinite(parsed)?parsed:value();
}
/// 校验本地格式的数值输入，支持科学计数法编辑状态。
QValidator::State CompactDoubleSpinBox::validate(QString& text,int& position) const {
    QString number=numericPart(text); int cursor=std::min(position,number.size());
    QDoubleValidator validator(minimum(),maximum(),decimals()); validator.setLocale(locale());
    validator.setNotation(QDoubleValidator::ScientificNotation);
    return validator.validate(number,cursor);
}
/// 刷新完整或紧凑数值文字，避免触发用户输入解析。
void CompactDoubleSpinBox::writeValue(bool complete) {
    // 程序性格式刷新不能伪装为用户输入并触发 Qt 再解析简写值。
    const QSignalBlocker blocker(lineEdit());
    const QString number=complete?locale().toString(value(),'g',QLocale::FloatingPointShortest):textFromValue(value());
    lineEdit()->setText(prefix()+number+suffix());
}
/// 进入编辑状态并显示完整精度的数值。
void CompactDoubleSpinBox::focusInEvent(QFocusEvent* event) {
    editing=true; writeValue(true); QDoubleSpinBox::focusInEvent(event);
}
/// 结束编辑并恢复紧凑数值显示。
void CompactDoubleSpinBox::focusOutEvent(QFocusEvent* event) {
    QDoubleSpinBox::focusOutEvent(event);
    editing=false; writeValue(false);
}
/// 提交当前具有焦点的数值输入。
void CompactDoubleSpinBox::commitInput() {
    if(hasFocus()) interpretText();
}
/// 先提交输入，再按指定步数调整数值。
void CompactDoubleSpinBox::stepBy(int steps) {
    // 若正在输入，先提交输入；无焦点简写值不能反向覆盖原值。
    commitInput(); writeValue(true); QDoubleSpinBox::stepBy(steps); writeValue(editing);
}
/// 仅在控件具有焦点时接受滚轮调节。
void CompactDoubleSpinBox::wheelEvent(QWheelEvent* event) {
    if(!hasFocus()) {event->ignore();return;}
    QDoubleSpinBox::wheelEvent(event);
}

/// 设置表单对话框的尺寸和主题属性。
void DialogFormFactory::configure(QDialog* dialog,const QSize& size) {
    dialog->setProperty("formDialog",true); dialog->resize(size); dialog->setMinimumSize(500,400);
}
/// 创建表单标题及副标题区域。
QWidget* DialogFormFactory::header(const QString& title,const QString& subtitle,QWidget* parent) {
    auto* widget=new QWidget(parent); auto* layout=new QVBoxLayout(widget);
    layout->setContentsMargins(0,0,0,0); layout->setSpacing(4);
    auto* heading=new QLabel(title,widget); heading->setObjectName("formDialogTitle");
    auto* detail=note(subtitle,widget); detail->setObjectName("formDialogSubtitle");
    detail->setToolTip(subtitle); layout->addWidget(heading); layout->addWidget(detail);return widget;
}
/// 创建可换行的表单说明文字。
QLabel* DialogFormFactory::note(const QString& text,QWidget* parent) {
    auto* label=new QLabel(text,parent); label->setProperty("formRole","note");
    label->setWordWrap(true);return label;
}
/// 创建包含可选标题和字段布局的表单分组。
DialogFormFactory::Section DialogFormFactory::section(const QString& title,QWidget* parent) {
    auto* widget=new QWidget(parent);widget->setObjectName("formSection");
    auto* layout=new QVBoxLayout(widget);layout->setContentsMargins(0,0,0,0);layout->setSpacing(12);
    if(!title.isEmpty()) {
        auto* heading=new QLabel(title,widget);heading->setObjectName("formSectionTitle");layout->addWidget(heading);
    }
    auto* form=new QFormLayout;form->setSpacing(10);form->setContentsMargins(0,0,0,0);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setLabelAlignment(Qt::AlignLeft|Qt::AlignVCenter);layout->addLayout(form);
    return {widget,form};
}
/// 为表单分组添加标签及输入控件。
void DialogFormFactory::row(const Section& section,const QString& text,QWidget* field) {
    auto* label=new QLabel(text,section.widget);label->setObjectName("formFieldLabel");
    label->setMinimumWidth(125);section.form->addRow(label,field);
}
/// 创建浮点数输入控件并设置范围、步长和后缀。
CompactDoubleSpinBox* DialogFormFactory::number(QWidget* parent,const char* name,double low,double high,
        double step,const QString& suffix,int storageDecimals) {
    auto* spin=new CompactDoubleSpinBox(parent);spin->setObjectName(name);
    if(storageDecimals>=0) spin->setDecimals(storageDecimals);
    spin->setRange(low,high);spin->setSingleStep(step);spin->setSuffix(suffix);return spin;
}
/// 创建整数输入控件并设置取值范围。
QSpinBox* DialogFormFactory::integer(QWidget* parent,const char* name,int low,int high) {
    auto* spin=new QSpinBox(parent);spin->setObjectName(name);spin->setRange(low,high);
    spin->setKeyboardTracking(false);spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    spin->setAlignment(Qt::AlignRight);spin->setMinimumHeight(32);return spin;
}
/// 创建用于展开或收起高级内容的按钮。
QToolButton* DialogFormFactory::disclosure(const QString& text,QWidget* content,QWidget* parent) {
    auto* button=new QToolButton(parent);button->setProperty("formRole","disclosure");
    button->setText(text);button->setCheckable(true);button->setArrowType(Qt::RightArrow);
    button->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);content->hide();
    QObject::connect(button,&QToolButton::toggled,content,[button,content](bool open) {
        content->setVisible(open);button->setArrowType(open?Qt::DownArrow:Qt::RightArrow);
        auto* dialog=qobject_cast<QDialog*>(button->window());
        if(!dialog) return;
        if(open) {
            button->setProperty("collapsedHeight",dialog->height());
            auto* screen=QGuiApplication::screenAt(dialog->frameGeometry().center());
            if(!screen) screen=QGuiApplication::primaryScreen();
            const int available=screen?std::max(dialog->minimumHeight(),screen->availableGeometry().height()-80):1000;
            dialog->resize(dialog->width(),std::min(available,dialog->height()+content->sizeHint().height()+14));
        } else if(button->property("collapsedHeight").isValid()) {
            dialog->resize(dialog->width(),button->property("collapsedHeight").toInt());
        }
    });return button;
}
/// 创建表单底部的重置、取消和执行按钮。
DialogFormFactory::Actions DialogFormFactory::actions(const QString& text,QWidget* parent,const QString& resetText) {
    auto* widget=new QWidget(parent);widget->setObjectName("formActions");
    auto* layout=new QHBoxLayout(widget);layout->setContentsMargins(0,14,0,0);layout->setSpacing(8);
    auto* reset=new QPushButton(resetText.isEmpty()?QObject::tr("恢复默认"):resetText,widget);
    reset->setProperty("formRole","tertiary");layout->addWidget(reset);layout->addStretch();
    auto* cancel=new QPushButton(QObject::tr("取消"),widget);
    auto* primary=new QPushButton(text,widget);primary->setObjectName("primaryButton");primary->setDefault(true);
    for(auto* button:{reset,cancel,primary}) button->setAutoDefault(false);
    primary->setDefault(true);
    // 在调用方的接受/校验槽前提交焦点中的精确输入；不触发业务操作。
    QObject::connect(primary,&QPushButton::clicked,parent,[parent] {
        for(auto* spin:parent->findChildren<CompactDoubleSpinBox*>()) spin->commitInput();
    });
    primary->setFocus(Qt::OtherFocusReason);
    layout->addWidget(cancel);layout->addWidget(primary);return {widget,reset,cancel,primary};
}
