// 工厂只有 Qt 依赖；覆盖数值精度、区域格式、键盘和第二个真实调用方。
#include "ui/forms/DialogFormFactory.h"
#include "ui/dialogs/ModelTransformDialog.h"
#include <QApplication>
#include <QDialog>
#include <QKeyEvent>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <cmath>
#include <iostream>
namespace {
int failures=0;
void check(bool condition,const char* name) {if(!condition){std::cerr<<"FAIL "<<name<<'\n';++failures;}}
}
int main(int argc,char** argv) {
    qputenv("QT_QPA_PLATFORM","offscreen"); QApplication app(argc,argv);
    QDialog dialog; DialogFormFactory::configure(&dialog);
    auto* layout=new QVBoxLayout(&dialog);
    auto* spin=DialogFormFactory::number(&dialog,"value",1e-9,1e12);
    auto* other=DialogFormFactory::number(&dialog,"other",-1e12,1e12);
    layout->addWidget(spin);layout->addWidget(other);
    const auto actions=DialogFormFactory::actions("Save",&dialog);layout->addWidget(actions.widget);
    const double precise=38.902828952788262;
    spin->setValue(precise);check(spin->text()=="38.903" && spin->value()==precise,"display does not round value");
    spin->setValue(2);check(spin->text()=="2","integer has no trailing zeros");
    other->setValue(-.00000001234567890123456);
    check(other->text().contains('e') && other->value()==-.00000001234567890123456,"small negative value survives");
    spin->setValue(precise);dialog.show();dialog.activateWindow();spin->setFocus();app.processEvents();
    check(spin->locale().toDouble(spin->text())==precise,"focus displays roundtrip precision");
    spin->setValue(.1);check(spin->text()=="0.1" && spin->value()==.1,"editing does not expose binary trailing digits");
    spin->setValue(precise);
    other->setFocus();app.processEvents();
    for(int i=0;i<5;++i) {spin->setFocus();app.processEvents();other->setFocus();app.processEvents();}
    check(spin->value()==precise && spin->text()=="38.903","repeated focus is lossless");
    spin->setFocus();app.processEvents();
    spin->findChild<QLineEdit*>()->setText("1.234567890123456e-8");
    double submitted=0;
    QObject::connect(actions.primary,&QPushButton::clicked,[&]{submitted=spin->value();});
    actions.primary->click();
    check(submitted==1.234567890123456e-8,"footer commits full scientific input before client slot");
    spin->setValue(precise);
    QKeyEvent up(QEvent::KeyPress,Qt::Key_Up,Qt::NoModifier);QApplication::sendEvent(spin,&up);
    check(spin->value()==precise+.1,"keyboard step uses full value");
    other->setFocus();app.processEvents();
    spin->setLocale(QLocale(QLocale::German));spin->setValue(precise);
    check(spin->text()=="38,903" && spin->value()==precise,"locale compact decimal");
    spin->setFocus();app.processEvents();spin->findChild<QLineEdit*>()->setText("2,123456789012345e-7");
    other->setFocus();app.processEvents();
    check(spin->value()==2.123456789012345e-7,"locale scientific input");
    dialog.hide();

    ModelTransformDialog transform("test CAD");transform.setValues(1.23456,-2.5,3,10,20,30,1.25);
    check(transform.translateX()==1.235 && transform.translateY()==-2.5 && transform.rotateZ()==30 && transform.scale()==1.25,
          "transform preserves existing three-decimal storage semantics");
    int resetSignals=0;
    QObject::connect(&transform,&ModelTransformDialog::transformResetRequested,[&]{++resetSignals;});
    for(auto* button:transform.findChildren<QPushButton*>()) if(button->text()==QStringLiteral("复位变换")) button->click();
    check(resetSignals==1 && transform.translateX()==0 && transform.rotateZ()==0 && transform.scale()==1,"transform reset signal and values");
    std::cout<<(failures?"FAIL":"PASS")<<" Qt-only form factory, precision, locale, keyboard, footer and transform reuse\n";
    return failures?1:0;
}
