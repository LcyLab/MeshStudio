// 实际 MainWindow/Qt 事件循环/VTK 渲染的表面流程回归；只写唯一测试目录。
#include "ui/window/MainWindow.h"
#include "ui/viewport/ViewportWidget.h"
#include "ui/dialogs/MeshSettingsDialog.h"
#include "ui/dialogs/ModelTransformDialog.h"
#include "app/meshing/TetMeshEngine.h"
#include "ui/panels/ModelTreePanel.h"
#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QDir>
#include <QDockWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QTabBar>
#include <QToolButton>
#include <QTabWidget>
#include <QStyleFactory>
#include <QSettings>
#include <QTemporaryDir>
#include <QTreeWidgetItemIterator>
#include <QTimer>
#include <QSurfaceFormat>
#include <QLocale>
#include <vtkCellData.h>
#include <vtkFieldData.h>
#include <vtkStringArray.h>
#include <vtkDataArray.h>
#include <vtkIdList.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <iostream>
#include <stdexcept>
#include <cmath>
namespace {
void require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
class GuiWindow : public MainWindow {
public:
    ViewportWidget* viewport() { return m_viewport; }
    mesher::SurfaceCriteria criteria() const { return m_surfaceCriteria; }
    void usePartitions(mesher::SurfacePointPlacement placement) {m_surfaceCriteria.pointPlacement=placement;m_surfaceCriteria.crossSmoothCadSplits=true;}
    TetMeshParams volumeParameters() const { return m_meshParams; }
    void showGeometryRibbon() { m_ribbonTabs->setCurrentIndex(1); }
    ModelTreePanel* tree() { return m_modelTree; }
    bool busy() const { return m_generating; }
    bool restoring() const { return m_restoringProject; }
    int surfaceId() const { return m_surfaceModelId; }
    int sourceId() const { return m_currentModelId; }
    vtkSmartPointer<vtkPolyData> surface() const { return m_lastGeneratedSurface; }
    vtkSmartPointer<vtkUnstructuredGrid> volume() const { return m_lastMeshVolume; }
    TetMeshQualityStats volumeStats() const {return m_lastStats;}
    QString cellsText() const { return m_statusCellsValue->text(); }
    bool save(const QString& path) {
        m_currentProjectPath=path;
        return QMetaObject::invokeMethod(this,"saveProject",Qt::DirectConnection) && !m_projectModified && QFile::exists(path);
    }
    void discardTestChanges() { m_projectModified=false; }
    ~GuiWindow() {
        if (m_surfaceCancellation) m_surfaceCancellation->store(true);
        if (m_surfaceWatcher.isRunning()) m_surfaceWatcher.waitForFinished();
        if (m_meshWatcher.isRunning()) m_meshWatcher.waitForFinished();
    }
};
bool waitFor(const std::function<bool()>& predicate,QString& failure,int timeout=120000) {
    QElapsedTimer clock; clock.start();
    while (!predicate()) {
        QCoreApplication::processEvents(QEventLoop::AllEvents,20);
        if (!failure.isEmpty() || clock.elapsed()>timeout) return false;
    }
    return failure.isEmpty();
}
void editDialog(QToolButton* action,QString& failure,const std::function<void(MeshSettingsDialog*)>& edit) {
    bool visited=false;
    QTimer::singleShot(0,[&] {
        auto* dialog=qobject_cast<MeshSettingsDialog*>(QApplication::activeModalWidget());
        visited=true;
        try {require(dialog,"mesh parameter modal dialog"); edit(dialog);}
        catch(const std::exception& e) {failure=e.what(); if(dialog) dialog->reject();}
    });
    action->click(); require(visited && failure.isEmpty(),"GUI parameter dialog interaction");
}
double longestEdge(vtkPolyData* mesh) {
    double maximum=0; vtkNew<vtkIdList> ids;
    for(vtkIdType i=0;i<mesh->GetNumberOfCells();++i) {
        mesh->GetCellPoints(i,ids);
        for(int k=0;k<3;++k) {
            double p[3],q[3];mesh->GetPoint(ids->GetId(k),p);mesh->GetPoint(ids->GetId((k+1)%3),q);
            maximum=std::max(maximum,std::hypot(p[0]-q[0],p[1]-q[1],p[2]-q[2]));
        }
    }
    return maximum;
}
bool same(vtkPolyData* a,vtkPolyData* b) {
    if (!a || !b || a->GetNumberOfPoints()!=b->GetNumberOfPoints() || a->GetNumberOfCells()!=b->GetNumberOfCells()) return false;
    for (vtkIdType i=0;i<a->GetNumberOfPoints();++i) {
        double p[3],q[3]; a->GetPoint(i,p); b->GetPoint(i,q);
        for (int k=0;k<3;++k) if (p[k]!=q[k]) return false;
    }
    vtkNew<vtkIdList> p,q;
    for (vtkIdType i=0;i<a->GetNumberOfCells();++i) {
        a->GetCellPoints(i,p); b->GetCellPoints(i,q);
        if (p->GetNumberOfIds()!=3 || q->GetNumberOfIds()!=3) return false;
        for (int k=0;k<3;++k) if (p->GetId(k)!=q->GetId(k)) return false;
    }
    for (const char* name:{"CadFaceId","CadEdgeId","CadCornerUV","CadBodyId"}) {
        auto* p=a->GetCellData()->GetArray(name); auto* q=b->GetCellData()->GetArray(name);
        if (!p || !q || p->GetDataType()!=q->GetDataType() || p->GetNumberOfComponents()!=q->GetNumberOfComponents() ||
            p->GetNumberOfTuples()!=q->GetNumberOfTuples() || p->GetNumberOfTuples()!=a->GetNumberOfCells()) return false;
        for (vtkIdType i=0;i<p->GetNumberOfTuples();++i) for (int k=0;k<p->GetNumberOfComponents();++k)
            if (p->GetComponent(i,k)!=q->GetComponent(i,k)&&!(std::isnan(p->GetComponent(i,k))&&std::isnan(q->GetComponent(i,k)))) return false;
    }
    for(const char* name:{"SurfacePatchId","SurfacePatchCornerUV","CadSupportFaceIds","CadSupportUV","CadSupportDistance","CadSupportProjectedToTrim"}) {
        auto* p=a->GetCellData()->GetArray(name);auto* q=b->GetCellData()->GetArray(name);
        if(bool(p)!=bool(q))return false;if(!p)continue;
        if(p->GetNumberOfComponents()!=q->GetNumberOfComponents()||p->GetNumberOfTuples()!=q->GetNumberOfTuples())return false;
        for(vtkIdType i=0;i<p->GetNumberOfTuples();++i)for(int k=0;k<p->GetNumberOfComponents();++k)
            if(p->GetComponent(i,k)!=q->GetComponent(i,k))return false;
    }
    for(const char* name:{"CadAcceptedBRep","CadInputReconciliation","CadAssemblyPolicy","CadSurfaceDiagnostics","SurfacePatchSourceFaces","CadSupportSamplingContract","SurfaceGenerationAlgorithm","Algorithm6FaceStatus"}) {
        auto* p=vtkStringArray::SafeDownCast(a->GetFieldData()->GetAbstractArray(name));
        auto* q=vtkStringArray::SafeDownCast(b->GetFieldData()->GetAbstractArray(name));
        if(bool(p)!=bool(q))return false;
        if(!p)continue;
        if(p->GetNumberOfValues()!=q->GetNumberOfValues())return false;
        for(vtkIdType i=0;i<p->GetNumberOfValues();++i)if(p->GetValue(i)!=q->GetValue(i))return false;
    }
    auto* runA=a->GetFieldData()->GetArray("Algorithm6RunConfiguration");auto* runB=b->GetFieldData()->GetArray("Algorithm6RunConfiguration");
    if(bool(runA)!=bool(runB))return false;
    if(runA) {
        if(runA->GetNumberOfComponents()!=runB->GetNumberOfComponents()||runA->GetNumberOfTuples()!=runB->GetNumberOfTuples())return false;
        for(vtkIdType i=0;i<runA->GetNumberOfTuples();++i)for(int k=0;k<runA->GetNumberOfComponents();++k)
            if(runA->GetComponent(i,k)!=runB->GetComponent(i,k))return false;
    }
    auto* interactionsA=a->GetFieldData()->GetArray("CadBodyInteractions");auto* interactionsB=b->GetFieldData()->GetArray("CadBodyInteractions");
    if(!interactionsA || !interactionsB || interactionsA->GetNumberOfComponents()!=interactionsB->GetNumberOfComponents() || interactionsA->GetNumberOfTuples()!=interactionsB->GetNumberOfTuples())return false;
    for(vtkIdType i=0;i<interactionsA->GetNumberOfTuples();++i)for(int k=0;k<interactionsA->GetNumberOfComponents();++k)
        if(interactionsA->GetComponent(i,k)!=interactionsB->GetComponent(i,k))return false;
    return true;
}
bool meshInTree(GuiWindow& window) {
    for (QTreeWidgetItemIterator it(window.tree());*it;++it) if ((*it)->text(0).contains(QStringLiteral("表面三角网格")))
        return (*it)->parent() && (*it)->parent()->text(0).contains(QStringLiteral("网格"));
    return false;
}
}
int main(int argc,char** argv) {
    if ((argc!=3&&argc!=4&&argc!=5&&argc!=7) || (argc>=5 && std::string(argv[4])!="--tet"&&std::string(argv[4])!="--frontal-partitions"&&
        std::string(argv[4])!="--reference-physical-partitions"&&std::string(argv[4])!="--algorithm6")) return 2;
    const bool algorithm6=argc>=5&&std::string(argv[4])=="--algorithm6";
    int generationTimeout=120000;
    if(argc==7) {
        if(std::string(argv[5])!="--generation-timeout-ms")return 2;
        try {size_t consumed=0;const auto value=std::stoi(argv[6],&consumed);
            if(consumed!=std::string(argv[6]).size()||value<=0||value>600000)return 2;
            generationTimeout=value;
        } catch(const std::exception&) {return 2;}
    }
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc,argv); app.setOrganizationName("CodexTests"); app.setApplicationName("MeshStudioGuiTest");
    app.setStyle(QStyleFactory::create("Fusion"));
    QFile theme(":/theme/light.qss");
    if (!theme.open(QIODevice::ReadOnly)) return 1;
    app.setStyleSheet(QString::fromUtf8(theme.readAll()));
    const auto artifactRoot=qEnvironmentVariable("MESHSTUDIO_GUI_ARTIFACT_ROOT",QDir::tempPath());
    QTemporaryDir directory(QDir(artifactRoot).filePath("MeshStudio-gui-XXXXXX"));
    if (!directory.isValid()) return 1;
    directory.setAutoRemove(false);
    QSettings::setDefaultFormat(QSettings::IniFormat); QSettings::setPath(QSettings::IniFormat,QSettings::UserScope,directory.path());
    std::cout<<"artifacts="<<directory.path().toStdString()<<std::endl;
    std::cout<<"generationTimeoutMs="<<generationTimeout<<std::endl;
    QString failure;
    QTimer dialogs;
    QObject::connect(&dialogs,&QTimer::timeout,[&] {
        for (QWidget* widget:QApplication::topLevelWidgets()) if (auto* box=qobject_cast<QMessageBox*>(widget)) {
            failure=box->text(); box->reject();
        }
    }); dialogs.start(30);
    try {
        GuiWindow window; window.resize(1440,1000); window.show();
        const QString emptyPackage=directory.filePath("empty.msp");
        require(window.save(emptyPackage),"GUI empty project save");
        window.openProjectFile(emptyPackage);
        require(!window.restoring() && window.viewport()->modelCount()==0 && window.criteria().maxSize==0,
                "GUI empty project restores parameter state and leaves restore mode");
        QObject::connect(window.viewport(),&ViewportWidget::loadFailed,[&](const QString&,const QString& reason){failure=reason;});
        window.openFile(QString::fromLocal8Bit(argv[1]));
        require(waitFor([&]{return window.viewport()->modelCount()==1;},failure),"GUI CAD import");
        if(argc>=5&&std::string(argv[4])=="--frontal-partitions")window.usePartitions(mesher::SurfacePointPlacement::FrontalDelaunay);
        if(argc>=5&&std::string(argv[4])=="--reference-physical-partitions")window.usePartitions(mesher::SurfacePointPlacement::PhysicalReferenceFrontal);
        require(window.viewport()->modelSummary(window.sourceId()).unitHint.isEmpty(),"GUI CAD has no unverified unit label");
        window.showGeometryRibbon(); QCoreApplication::processEvents();
        mesher::SurfaceCriteria requested; requested.targetSize=requested.maxSize=std::stod(argv[2]); requested.minSize=requested.maxDeviation=requested.targetSize*0.1;
        requested.maxIterations=argc>=4?std::stoi(argv[3]):200000;
        auto* generate=window.findChild<QToolButton*>("generateSurfaceAction");
        auto* volume=window.findChild<QToolButton*>("generateVolumeAction");
        auto* cancel=window.findChild<QToolButton*>("cancelSurfaceAction");
        require(generate && volume && cancel && generate->isEnabled(),"GUI separate generation buttons");
        const auto original=window.criteria(); const auto originalVolume=window.volumeParameters();
        {
            ModelTransformDialog transform(QStringLiteral("shared form template"),&window);
            transform.setValues(0,2.5,0,0,30,0,1);transform.show();QCoreApplication::processEvents();
            require(transform.grab().save(directory.filePath("transform-dialog.png")),"shared transform template image");transform.hide();
        }
        editDialog(volume,failure,[&](MeshSettingsDialog* dialog) {
            require(dialog->mode()==MeshSettingsDialog::Mode::Volume,"volume action selects volume page");
            require(dialog->grab().save(directory.filePath("volume-dialog.png")),"volume dialog image");
            dialog->findChild<QDoubleSpinBox*>("volumeTarget")->setValue(25);
            dialog->findChild<QTabWidget*>("meshTypeTabs")->setCurrentIndex(0);
            dialog->findChild<QDoubleSpinBox*>("surfaceMaximum")->setValue(27);
            dialog->reject();
        });
        require(window.criteria().maxSize==original.maxSize && window.volumeParameters().targetSize==originalVolume.targetSize && !window.busy(),"cancelled drafts do not change project or generate");
        const auto configure=[&](MeshSettingsDialog* dialog) {
            require(dialog->mode()==MeshSettingsDialog::Mode::Surface,"surface action selects surface page");
            if(algorithm6) {
                auto* route=dialog->findChild<QComboBox*>("surfaceAlgorithm");require(route,"experimental route selector");
                route->setCurrentIndex(route->findData(int(mesher::SurfacePointPlacement::Algorithm6Experimental)));
                dialog->findChild<QDoubleSpinBox*>("algorithm6Target")->setValue(requested.targetSize);
                dialog->findChild<QSpinBox*>("algorithm6Queries")->setValue(20000000);
                dialog->findChild<QCheckBox*>("algorithm6Extend")->setChecked(true);
                QCoreApplication::processEvents();
                require(dialog->grab().save(directory.filePath("algorithm6-dialog.png")),"experimental parameter view");
                dialog->findChild<QPushButton*>("primaryButton")->click();return;
            }
            auto* minimum=dialog->findChild<QDoubleSpinBox*>("surfaceMinimum");
            auto* maximum=dialog->findChild<QDoubleSpinBox*>("surfaceMaximum");
            minimum->setValue(requested.maxSize+1); maximum->setValue(requested.maxSize);
            dialog->findChild<QPushButton*>("primaryButton")->click();
            require(dialog->isVisible() && dialog->findChild<QLabel*>("meshDialogError")->isVisible(),"invalid minimum rejected inline");
            minimum->setValue(requested.minSize);
            dialog->findChild<QDoubleSpinBox*>("surfaceDeviation")->setValue(requested.maxDeviation);
            dialog->findChild<QDoubleSpinBox*>("surfaceNormalAngle")->setValue(requested.maxNormalAngleDeg);
            dialog->findChild<QSpinBox*>("surfaceWorkBudget")->setValue(requested.maxIterations);
            dialog->findChild<QSpinBox*>("surfaceVertexBudget")->setValue(requested.maxVertices);
            maximum->setFocus(); QCoreApplication::processEvents();
            require(maximum->locale().toDouble(maximum->text())==requested.maxSize,"full precision while editing maximum");
            require(dialog->grab().save(directory.filePath("surface-edit-dialog.png")),"full precision editing image");
            dialog->findChild<QPushButton*>("primaryButton")->setFocus();
            QCoreApplication::processEvents();
            require(dialog->grab().save(directory.filePath("surface-dialog.png")),"surface dialog image");
            const int compactHeight=dialog->height();
            dialog->findChild<QToolButton*>("meshAdvancedToggle")->click(); QCoreApplication::processEvents();
            require(dialog->height()>=compactHeight &&
                    !dialog->findChild<QSpinBox*>("surfaceWorkBudget")->visibleRegion().isEmpty(),"advanced budget accessible after expansion");
            require(dialog->grab().save(directory.filePath("surface-advanced-dialog.png")),"advanced dialog image");
            dialog->findChild<QPushButton*>("primaryButton")->click();
        };
        editDialog(generate,failure,configure); const auto actual=window.criteria();
        if(!algorithm6)require(actual.maxSize==requested.maxSize && actual.minSize==requested.minSize &&
                actual.maxDeviation==requested.maxDeviation && actual.maxRadiusEdgeRatio==requested.maxRadiusEdgeRatio,
                "compact UI preserves original double criteria on submission");
        if(!algorithm6)require(actual.pointPlacement==original.pointPlacement && actual.crossSmoothCadSplits==original.crossSmoothCadSplits,
                "GUI parameter submission preserves experimental placement and partition selection");
        if(algorithm6)require(actual.pointPlacement==mesher::SurfacePointPlacement::Algorithm6Experimental&&actual.algorithm6.targetSize==requested.targetSize&&
            actual.maxSize==original.maxSize&&actual.minSize==original.minSize&&actual.maxDeviation==original.maxDeviation&&!actual.crossSmoothCadSplits,
            "Algorithm 6 explicit selection and independent parameters");
        require(window.busy() && !generate->isEnabled() && !volume->isEnabled() && cancel->isVisible(),"GUI generating state");
        cancel->click(); require(waitFor([&]{return !window.busy();},failure),"GUI cancellation completes");
        require(window.viewport()->modelCount()==1 && !window.surface() && generate->isEnabled() && !cancel->isVisible(),"GUI cancellation adds no mesh");
        editDialog(generate,failure,configure); require(waitFor([&]{return !window.busy();},failure,generationTimeout),"GUI generation completes within explicit driver timeout");
        require(window.surface() && window.surface()->GetNumberOfCells()>0 && window.viewport()->modelCount()==2,"GUI mesh shown");
        require(window.viewport()->modelSummary(window.surfaceId()).visible,"GUI mesh visible");
        require(window.viewport()->modelSummary(window.surfaceId()).unitHint.isEmpty(),"GUI mesh does not invent a length unit");
        require(meshInTree(window),"GUI surface belongs to mesh tree section");
        const auto after=window.criteria();
        require(after.targetSize==actual.targetSize && after.minSize==actual.minSize && after.maxDeviation==actual.maxDeviation &&
            after.maxNormalAngleDeg==actual.maxNormalAngleDeg && after.maxSize==actual.maxSize && after.maxIterations==actual.maxIterations &&
            after.pointPlacement==actual.pointPlacement && after.crossSmoothCadSplits==actual.crossSmoothCadSplits &&
            window.volumeParameters().targetSize==originalVolume.targetSize,"GUI generation preserves independent criteria");
        vtkNew<vtkPolyData> generated; generated->DeepCopy(window.surface());
        if(!algorithm6)require(longestEdge(generated)<=actual.maxSize*(1+1e-8),"GUI explicit maximum edge length");
        if(algorithm6) {
            vtkNew<vtkXMLPolyDataWriter> writer;const auto path=directory.filePath("algorithm6-export.vtp");
            const auto encoded=QFile::encodeName(path);writer->SetFileName(encoded.constData());writer->SetInputData(generated);
            require(writer->Write()!=0,"experimental VTP export");
            vtkNew<vtkXMLPolyDataReader> reader;reader->SetFileName(encoded.constData());reader->Update();
            require(same(generated,reader->GetOutput()),"VTP export preserves physical mesh, labels, corner UV and run metadata");
        }
        require(QLocale().toLongLong(window.cellsText())==generated->GetNumberOfCells(),"GUI generated cell count");
        require(window.viewport()->grabView().save(directory.filePath("generated-view.png")),"GUI rendered screenshot");
        require(window.grab().save(directory.filePath("generated-window.png")),"GUI window screenshot");
        const QString package=directory.filePath("surface.msp"); require(window.save(package),"GUI save project");
        window.discardTestChanges(); window.close();
        GuiWindow reopened; reopened.resize(1440,1000); reopened.show(); reopened.openProjectFile(package);
        require(waitFor([&]{return !reopened.restoring() && reopened.viewport()->modelCount()==2;},failure),"GUI restore completes in fresh window");
        require(same(generated,reopened.surface()) && meshInTree(reopened),"GUI restored coordinates/connectivity/CAD tags/tree");
        const auto restored=reopened.criteria();
        if(algorithm6)require(restored.algorithm6.targetSize==actual.algorithm6.targetSize&&restored.algorithm6.extendBoundarySize==actual.algorithm6.extendBoundarySize&&
            restored.algorithm6.smoothingIterations==actual.algorithm6.smoothingIterations&&restored.algorithm6.maximumNodes==actual.algorithm6.maximumNodes&&
            restored.algorithm6.maximumGeometryEvaluations==actual.algorithm6.maximumGeometryEvaluations,"GUI restored experimental run configuration");
        require(restored.targetSize==actual.targetSize && restored.minSize==actual.minSize && restored.maxDeviation==actual.maxDeviation &&
                restored.maxNormalAngleDeg==actual.maxNormalAngleDeg && restored.maxSize==actual.maxSize &&
                restored.maxRadiusEdgeRatio==actual.maxRadiusEdgeRatio && restored.growthRatio==actual.growthRatio &&
                restored.maxVertices==actual.maxVertices && restored.maxIterations==actual.maxIterations &&
                restored.pointPlacement==actual.pointPlacement && restored.crossSmoothCadSplits==actual.crossSmoothCadSplits &&
                reopened.volumeParameters().targetSize==originalVolume.targetSize,"GUI restored independent criteria");
        require(QLocale().toLongLong(reopened.cellsText())==generated->GetNumberOfCells(),"GUI restored cell count");
        require(reopened.grab().save(directory.filePath("restored-window.png")),"GUI restored screenshot");
        if(argc>=5&&std::string(argv[4])=="--tet") {
            reopened.showGeometryRibbon();
            reopened.tree()->modelNodeClicked(reopened.surfaceId());
            require(reopened.sourceId()==reopened.surfaceId(),"GUI selects restored indexed surface");
            auto* volumeAction=reopened.findChild<QToolButton*>("generateVolumeAction");
            editDialog(volumeAction,failure,[&](MeshSettingsDialog* dialog) {
                require(dialog->mode()==MeshSettingsDialog::Mode::Volume,"surface selection enables volume dialog");
                dialog->findChild<QDoubleSpinBox*>("volumeTarget")->setValue(1e6);
                dialog->findChild<QPushButton*>("primaryButton")->click();
            });
            require(waitFor([&]{return !reopened.busy();},failure),"GUI tetra generation from restored surface completes");
            require(reopened.volume() && reopened.viewport()->modelCount()==3 &&
                reopened.volumeStats().acceptanceKnown && reopened.volumeStats().qualityAccepted,"GUI volume and quality accepted");
            const auto cells=reopened.volume()->GetNumberOfCells();
            const auto stats=reopened.volumeStats();
            const QString volumePackage=directory.filePath("volume.msp");
            require(reopened.save(volumePackage),"GUI saves surface plus tetra volume");
            GuiWindow volumeReopened;volumeReopened.show();volumeReopened.openProjectFile(volumePackage);
            require(waitFor([&]{return !volumeReopened.restoring() && volumeReopened.viewport()->modelCount()==3;},failure),"GUI restores tetra project");
            require(volumeReopened.volume() && volumeReopened.volume()->GetNumberOfCells()==cells &&
                volumeReopened.volumeStats().acceptanceKnown && volumeReopened.volumeStats().qualityAccepted &&
                volumeReopened.volumeStats().minScaledJacobian==stats.minScaledJacobian &&
                volumeReopened.volumeStats().maxEdgeLength==stats.maxEdgeLength,"GUI restores volume cells and final acceptance measurements");
            volumeReopened.discardTestChanges();volumeReopened.close();
            std::cout<<"PASS actual GUI restored surface -> tetra -> save/reopen cells="<<cells<<'\n';
        }
        reopened.discardTestChanges(); reopened.close();
        std::cout<<(algorithm6?"PASS Algorithm 6 selection, independent parameters, cancel, rendering, VTP export and save/reopen points=":
            "PASS actual themed GUI, modal drafts/pages, invalid range, generation/cancel, maximum, rendering and save/reopen points=")
            <<generated->GetNumberOfPoints()<<" triangles="<<generated->GetNumberOfCells()<<" maxEdge="<<longestEdge(generated)<<'\n'; return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL "<<e.what()<<" detail="<<failure.toStdString()<<'\n'; return 1;
    }
}
