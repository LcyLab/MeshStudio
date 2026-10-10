// 参数草稿/两页隔离及错误拒绝；不启动网格引擎，不写用户设置。
#include "ui/dialogs/MeshSettingsDialog.h"
#include <QApplication>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTabWidget>
#include <QLineEdit>
#include <QComboBox>
#include <iostream>
int main(int argc,char** argv) {
    qputenv("QT_QPA_PLATFORM","offscreen"); QApplication app(argc,argv);
    mesher::SurfaceCriteria surface; surface.targetSize=8; surface.minSize=.8; surface.maxSize=8;
    surface.maxDeviation=.12; surface.maxNormalAngleDeg=35;
    TetMeshParams volume; volume.targetSize=19; volume.optimize=false;
    MeshSettingsDialog dialog("test CAD",80,MeshSettingsDialog::Mode::Surface,surface,volume);
    auto* min=dialog.findChild<QDoubleSpinBox*>("surfaceMinimum");
    auto* max=dialog.findChild<QDoubleSpinBox*>("surfaceMaximum");
    auto* tet=dialog.findChild<QDoubleSpinBox*>("volumeTarget");
    auto* tabs=dialog.findChild<QTabWidget*>("meshTypeTabs");
    auto* generate=dialog.findChild<QPushButton*>("primaryButton");
    if(!min || !max || !tet || !tabs || !generate || dialog.mode()!=MeshSettingsDialog::Mode::Surface) return 1;
    if(min->minimum()<=0 || max->minimum()<=0) {std::cerr<<"non-positive length allowed\n";return 1;}
    if(max->text()!="8" || dialog.findChild<QDoubleSpinBox*>("surfaceNormalAngle")->text()!="35 °") {
        std::cerr<<"redundant decimals are still shown\n";return 1;
    }
    const double precise=38.902828952788262;
    max->setValue(precise);
    if(max->text()!="38.903" || max->value()!=precise) {std::cerr<<"compact display changed precision\n";return 1;}
    dialog.show(); max->setFocus(); app.processEvents();
    bool parsed=false;
    if(max->locale().toDouble(max->text(),&parsed)!=precise || !parsed) {std::cerr<<"editing hides full precision\n";return 1;}
    min->setFocus(); app.processEvents();
    if(max->value()!=precise || max->text()!="38.903") {std::cerr<<"focus roundtrip changed value\n";return 1;}
    max->setFocus(); app.processEvents();
    max->findChild<QLineEdit*>()->setText("1.234567890123456e-8");
    min->setFocus(); app.processEvents();
    if(max->value()!=1.234567890123456e-8 || max->text().contains("0.000000000")) {
        std::cerr<<"small scientific value lost precision\n";return 1;
    }
    dialog.hide();
    min->setValue(9); max->setValue(7); generate->click();
    if(dialog.result()==QDialog::Accepted) { std::cerr<<"invalid min/max accepted\n"; return 1; }
    min->setValue(.5); max->setValue(6); generate->click();
    if(dialog.result()!=QDialog::Accepted || dialog.surfaceCriteria().minSize!=.5 ||
       dialog.surfaceCriteria().maxSize!=6 || dialog.volumeParameters().targetSize!=19 ||
       dialog.volumeParameters().optimize) { std::cerr<<"surface transaction or page isolation failed\n"; return 1; }
    MeshSettingsDialog cancelled("test CAD",80,MeshSettingsDialog::Mode::Volume,surface,volume);
    cancelled.findChild<QDoubleSpinBox*>("volumeTarget")->setValue(25);
    cancelled.reject();
    if(volume.targetSize!=19 || surface.maxSize!=8 || cancelled.result()!=QDialog::Rejected) return 1;
    MeshSettingsDialog defaults("test CAD",80,MeshSettingsDialog::Mode::Surface,surface,volume);
    auto* reset=defaults.findChild<QPushButton*>("resetMeshDefaults"); reset->click();
    if(defaults.surfaceCriteria().maxSize!=2 || defaults.surfaceCriteria().minSize!=.2 ||
       defaults.volumeParameters().targetSize!=19) return 1;
    defaults.findChild<QTabWidget*>("meshTypeTabs")->setCurrentIndex(1); reset->click();
    if(defaults.volumeParameters().targetSize!=4 || defaults.surfaceCriteria().maxSize!=2) return 1;
    surface.maxSize=0;
    MeshSettingsDialog migrated("old CAD",80,MeshSettingsDialog::Mode::Surface,surface,volume);
    if(migrated.surfaceCriteria().maxSize!=8.4 || surface.maxSize!=0) { std::cerr<<"legacy maximum migration failed\n"; return 1; }
    MeshSettingsDialog experimental("CAD",80,MeshSettingsDialog::Mode::Surface,surface,volume);
    auto* route=experimental.findChild<QComboBox*>("surfaceAlgorithm");
    auto* h=experimental.findChild<QDoubleSpinBox*>("algorithm6Target");
    if(!route||!h||experimental.surfaceCriteria().pointPlacement!=mesher::SurfacePointPlacement::LegacyRefinement) {
        std::cerr<<"experimental selection missing or changes default\n";return 1;
    }
    route->setCurrentIndex(route->findData(int(mesher::SurfacePointPlacement::Algorithm6Experimental)));
    h->setValue(.3);
    const auto selected=experimental.surfaceCriteria();
    if(selected.pointPlacement!=mesher::SurfacePointPlacement::Algorithm6Experimental || selected.algorithm6.targetSize!=.3 ||
       selected.targetSize!=surface.targetSize || selected.maxSize!=surface.maxSize ||
       experimental.findChild<QDoubleSpinBox*>("surfaceDeviation")->isEnabled())return 1;
    std::cout<<"PASS dialog invalid range, independent drafts, cancellation and legacy maximum\n";
}
