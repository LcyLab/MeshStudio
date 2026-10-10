// 极窄斜条面：偏移 UV 格点全部落在裁剪域外，审计必须补内部样本。
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepTools.hxx>
#include <Geom_Plane.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <QCoreApplication>
#include <QDir>
#include <QProcess>
#include <QTemporaryDir>
#include <gp_Ax3.hxx>
#include <iostream>

int main(int argc,char** argv) {
    QCoreApplication app(argc,argv); QTemporaryDir temp;
    if (!temp.isValid()) return 1;
    const QString cad=temp.path()+"/narrow.brep";
    BRepBuilderAPI_MakePolygon polygon;
    for (auto p:{gp_Pnt(0,0,0),gp_Pnt(1,1,0),gp_Pnt(1,1.00001,0),gp_Pnt(0,0.00001,0)}) polygon.Add(p);
    polygon.Close();
    Handle(Geom_Plane) plane=new Geom_Plane(gp_Ax3(gp_Pnt(0,0,0),gp_Dir(0,0,1),gp_Dir(1,0,0)));
    const auto shape=BRepBuilderAPI_MakeFace(plane,polygon.Wire(),true).Face();
    if (!BRepTools::Write(shape,cad.toLocal8Bit().constData())) return 1;
    QProcess audit;
    audit.start(QDir(QCoreApplication::applicationDirPath()).filePath("CadSurfaceQualityAuditTest.exe"),
        {"--cad",cad,"--target","0.2","--min","0.02","--deviation","0.02","--timeout","30"});
    if (!audit.waitForFinished(60000)) { audit.kill(); audit.waitForFinished(1000); std::cerr<<"coverage audit did not finish\n"; return 1; }
    const QByteArray output=audit.readAllStandardOutput();
    if (audit.exitStatus()!=QProcess::NormalExit || audit.exitCode()!=0 || !output.contains("PASS independent sampled distances") ||
        !output.contains("supplementedFaces=1") || output.contains("interiorSamples=0")) {
        std::cerr<<output.constData()<<audit.readAllStandardError().constData(); return 1;
    }
    std::cout<<"PASS narrow trimmed face needs and receives independent interior samples\n";
}
