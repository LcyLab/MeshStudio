# Qt Dialog 表单工厂

入口：`src/ui/DialogFormFactory.h`。工厂只依赖 Qt Widgets，负责统一控件、布局和样式；业务 Dialog 自己持有草稿、校验、连接按钮。当前已有 `MeshSettingsDialog` 和 `ModelTransformDialog` 两个调用方。

## 调用示例

```cpp
#include "ui/DialogFormFactory.h"
#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>

class ExampleDialog : public QDialog {
public:
    explicit ExampleDialog(QWidget* parent=nullptr) : QDialog(parent) {
        DialogFormFactory::configure(this);
        auto* root=new QVBoxLayout(this);
        root->setContentsMargins(22,18,22,18);
        root->setSpacing(12);
        root->addWidget(DialogFormFactory::header("参数设置","当前对象",this));

        const auto section=DialogFormFactory::section("基本参数",this);
        size=DialogFormFactory::number(section.widget,"size",1e-9,1e12);
        size->setValue(38.902828952788262);
        size->setToolTip("对象的实际尺寸要求。");
        DialogFormFactory::row(section,"目标尺寸",size);
        root->addWidget(section.widget);
        root->addStretch();

        const auto actions=DialogFormFactory::actions("确定",this);
        root->addWidget(actions.widget);
        connect(actions.reset,&QPushButton::clicked,this,[this]{size->setValue(10);});
        connect(actions.cancel,&QPushButton::clicked,this,&QDialog::reject);
        connect(actions.primary,&QPushButton::clicked,this,&QDialog::accept);
    }
    double value() const {return size->value();}
private:
    CompactDoubleSpinBox* size=nullptr;
};
```

调用方在 `exec()==QDialog::Accepted` 后读取 value() 并提交业务数据；取消时丢弃草稿。工厂不会帮调用方提交工程、启动算法或发送变换信号。

## 可复用组件

| 方法 | 用途 |
|---|---|
| configure | 设置统一 Dialog 样式属性、初始/最小尺寸 |
| header / note | 标题、对象说明与简短辅助文本 |
| section / row | 无套框分区和对齐参数行 |
| number / integer | 精确实数框与整数框 |
| disclosure | 折叠区；展开在屏幕高度内增高，折叠恢复，窄屏可配滚动区 |
| actions | 恢复默认、取消、主操作栏；主操作先提交正在编辑的数值，再调用业务槽 |

Qt parent 管理控件生命期；Section/Actions 只是访问控件的句柄，不拥有独立内存。主题集中在 `resources/theme/light.qss` 的 `QDialog[formDialog="true"]` 范围内，不影响未采用工厂的窗口。新 Dialog 源文件仍需按项目习惯加入 CMake。

## 数值精度契约

- 非编辑状态：常规值最多三位小数，隐藏尾零；很小/大值使用简洁科学计数。例如 2、38.903、30°。
- 编辑状态和悬停提示：使用 Qt `FloatingPointShortest` 的完整 double 往返字符串，避免把 0.1 显示成二进制尾数。
- 简写只用于显示，读取 value()、焦点往返和主按钮确认不把参数舍入为简写值。Qt 内部的小数存储精度与显示格式分离。
- 支持科学计数和控件 locale；工厂创建时关闭分组分隔符。失焦后恢复简写；滚动未聚焦的数值框不会改变值。
- storageDecimals=-1（默认）保留完整 double；迁移原有低精度控件时可显式传入原精度。模型变换继续传 3，维持其原有范围和三位存储语义。
- 范围、单位、业务含义由调用方指定。Qt 范围/编辑校验与 Dialog 的业务校验各自保留；工厂不代替几何算法判据。

自动验证：`DialogFormFactoryTest`（只有 Qt 依赖）、`MeshSettingsDialogTest`，以及真实主题的 `CadSurfaceGuiTest`。
