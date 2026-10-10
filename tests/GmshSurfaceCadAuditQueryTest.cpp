/** @file GmshSurfaceCadAuditQueryTest.cpp
 * @brief 原CAD直线/圆弧平面裁剪查询解析回归，覆盖孔洞、弧端点、方向及位置变换。
 * @date 2026-10-09
 */
#include "Algorithm6CadAuditQueries.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <array>
#include <iostream>
#include <string>
#include <vector>

static void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
static void checkDistance(double actual,double expected,const char* message){check(std::abs(actual-expected)<1.e-12,message);}
static Handle(Geom_Plane) plane(){return new Geom_Plane(gp_Ax3(gp_Pnt(0,0,0),gp_Dir(0,0,1),gp_Dir(1,0,0)));}
static TopoDS_Wire polygon(const std::vector<gp_Pnt>& points){
    BRepBuilderAPI_MakePolygon maker;for(const auto& p:points)maker.Add(p);maker.Close();
    check(maker.IsDone(),"fixture polygon construction");return maker.Wire();
}
static TopoDS_Face sixEdges(){
    const auto wire=polygon({{.0002,.00007,0},{.0002,0,0},{-.0012,0,0},{-.00125,-.00008660254037841099,0},
        {-.001310621778264936,-.00005160254037844890,0},{-.0012404145188432314,.00007,0}});
    return BRepBuilderAPI_MakeFace(plane(),wire,true).Face();
}
// 独立解析面：大面右侧凹口及两圆弧，位置取自实际失败样本；无BREP或参考网格输入。
static TopoDS_Face roundedNotch(){
    const std::array<gp_Pnt,14> points{{
        {.117314970344115,-.022701037427632312,0},
        {.117314970344115,-.006075340425245893,0},
        {.111511837745009,-.006075340425245887,0},
        {.1107618377450092,-.005325340425245706,0},
        {.110761837745009,.005174659574754108,0},
        {.11151183774500922,.005924659574753908,0},
        {.117314970344115,.005924659574754099,0},
        {.11731497034411499,.022821999944983587,0},
        {.11126342133830436,.03200947700598816,0},
        {-.0052365786616953985,.03200947700598822,0},
        {-.005736578661695393,.03150947700598822,0},
        {-.005736578661695419,-.031388514488636804,0},
        {-.0052365786616953985,-.03188851448863679,0},
        {.11126342133830439,-.03188851448863684,0}}};
    BRepBuilderAPI_MakeWire wire;
    for(std::size_t k=0;k<points.size();++k){
        const auto a=points[k],b=points[(k+1)%points.size()];
        if(k==7||k==13){
            const auto mid=k==7?gp_Pnt(.11566615319272673,.028322703993471678,0):gp_Pnt(.11566615319272672,-.02820174147612038,0);
            GC_MakeArcOfCircle arc(a,mid,b);check(arc.IsDone(),"rounded notch arc construction");
            wire.Add(BRepBuilderAPI_MakeEdge(arc.Value()));
        }else wire.Add(BRepBuilderAPI_MakeEdge(a,b));
    }
    check(wire.IsDone(),"rounded notch wire construction");
    const Handle(Geom_Plane) support=new Geom_Plane(gp_Ax3(gp_Pnt(.1050509119433174,.0000150651412391,0),gp_Dir(0,0,1),gp_Dir(1,0,0)));
    return BRepBuilderAPI_MakeFace(support,wire.Wire(),true).Face();
}
int main(int argc,char** argv){
    try{
        if(argc==2&&std::string(argv[1])=="rounded"){
            const auto face=roundedNotch();check(BRepCheck_Analyzer(face).IsValid(),"rounded notch CAD must be valid");
            const gp_Pnt point(.0790185861145033,.0053257659044719675,0);
            const gp_Pnt2d uv(point.X()-.1050509119433174,point.Y()-.0000150651412391);
            const algorithm6audit::CadFaceQueries query(face);
            std::cout<<"rounded notch default state "<<int(BRepClass_FaceClassifier(face,uv,1.e-10).State())
                     <<" bounded state "<<int(BRepClass_FaceClassifier(face,uv,1.e-10,true).State())
                     <<" distance "<<query.distance(point)<<'\n';
            check(query.classify(uv)==TopAbs_IN,"deep interior of rounded notch must be IN");
            check(query.usesAnalyticPlane(),"consistent rounded plane uses analytic query");
            checkDistance(query.distance(point),0,"deep rounded planar interior distance must be zero");
            checkDistance(query.distance(gp_Pnt(point.X(),point.Y(),.003)),.003,"rounded plane normal distance");
            check(query.classify(gp_Pnt2d(.114-.1050509119433174,-.0000150651412391))==TopAbs_OUT,"rounded notch cutout must remain OUT");
            checkDistance(query.distance(gp_Pnt(.114,0,0)),.114-.110761837745009,"rounded notch retains positive boundary distance");
            check(algorithm6audit::CadFaceQueries(TopoDS::Face(face.Reversed())).classify(uv)==TopAbs_IN,"rounded face direction does not change trim");
            gp_Trsf move;move.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),.7);move.SetTranslationPart(gp_Vec(7,-2,3));
            const algorithm6audit::CadFaceQueries transformed(TopoDS::Face(face.Located(TopLoc_Location(move))));
            check(transformed.usesAnalyticPlane(),"located analytic plane retains consistent curves");
            checkDistance(transformed.distance(gp_Pnt(point.X(),point.Y(),.003).Transformed(move)),.003,"rounded plane transformed normal distance");
            std::cout<<"PASS independent rounded notch planar queries\n";return 0;
        }
        const auto face=sixEdges();check(BRepCheck_Analyzer(face).IsValid(),"six-edge CAD must be valid");
        const gp_Pnt2d inside(-.0012503454323693857,.000006132486540509048);
        const algorithm6audit::CadFaceQueries query(face);
        if(argc==2&&std::string(argv[1])=="distance"){
            checkDistance(query.distance(gp_Pnt(inside.X(),inside.Y(),0)),0,"valid planar interior point distance must be zero");
            std::cout<<"PASS known planar interior distance\n";return 0;
        }
        check(query.classify(inside)==TopAbs_IN,"valid straight CAD interior point must be IN");
        checkDistance(query.distance(gp_Pnt(inside.X(),inside.Y(),0)),0,"valid planar interior distance");
        checkDistance(query.distance(gp_Pnt(inside.X(),inside.Y(),.003)),.003,"off-plane interior distance");
        check(query.usesExactLinearPlane(),"straight planar CAD must use exact query path");
        check(query.classify(gp_Pnt2d(.001,0))==TopAbs_OUT,"outer point must remain OUT");
        check(query.classify(gp_Pnt2d(.0002,.000035))==TopAbs_ON,"edge midpoint must remain ON");
        check(query.classify(gp_Pnt2d(.0002+1.e-11,.000035))==TopAbs_ON,"same configured ON tolerance");
        check(query.classify(gp_Pnt2d(.0002+1.e-8,.000035))==TopAbs_OUT,"no widened domain tolerance");
        checkDistance(query.distance(gp_Pnt(.0002+1.e-11,.000035,0)),1.e-11,"ON classification must not erase physical distance");
        bool invalidRejected=false;try{query.classify(inside,-1);}catch(const std::runtime_error&){invalidRejected=true;}
        check(invalidRejected,"negative query tolerance must fail explicitly");
        const auto reversed=TopoDS::Face(face.Reversed());
        check(algorithm6audit::CadFaceQueries(reversed).classify(inside)==TopAbs_IN,"face orientation cannot change trim membership");
        gp_Trsf moved;moved.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(0,1,0)),.7);moved.SetTranslationPart(gp_Vec(7,-2,3));
        const algorithm6audit::CadFaceQueries located(TopoDS::Face(face.Located(TopLoc_Location(moved))));
        check(located.classify(inside)==TopAbs_IN,"located face retains original UV");
        checkDistance(located.distance(gp_Pnt(inside.X(),inside.Y(),.003).Transformed(moved)),.003,"rotated and translated CAD distance");
        BRepBuilderAPI_MakeFace holed(plane(),polygon({{0,0,0},{4,0,0},{4,4,0},{0,4,0}}),true);
        holed.Add(TopoDS::Wire(polygon({{1.5,1.5,0},{2.5,1.5,0},{2.5,2.5,0},{1.5,2.5,0}}).Reversed()));
        check(BRepCheck_Analyzer(holed.Face()).IsValid(),"hole fixture CAD validity");
        const algorithm6audit::CadFaceQueries hole(holed.Face());
        check(hole.classify(gp_Pnt2d(.5,.5))==TopAbs_IN,"material region IN");
        check(hole.classify(gp_Pnt2d(2,2))==TopAbs_OUT,"hole must remain OUT");
        check(hole.classify(gp_Pnt2d(1.5,2))==TopAbs_ON,"hole edge ON");
        checkDistance(hole.distance(gp_Pnt(2,2,0)),.5,"distance inside hole is to its boundary");
        checkDistance(hole.distance(gp_Pnt(2,2,.4)),std::sqrt(.41),"hole plus normal distance");
        checkDistance(hole.distance(gp_Pnt(-1,2,3)),std::sqrt(10.),"outer plus normal distance");
        const auto circle=BRepBuilderAPI_MakeFace(plane(),BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),1))).Wire(),true).Face();
        const algorithm6audit::CadFaceQueries circular(circle);check(!circular.usesExactLinearPlane(),"curved trim keeps OCC path");
        check(circular.usesAnalyticPlane(),"consistent circular plane uses analytic classification");
        check(circular.classify(gp_Pnt2d(0,0))==TopAbs_IN,"circle center IN");checkDistance(circular.distance(gp_Pnt(0,0,.2)),.2,"curved trim existing distance");
        checkDistance(circular.distance(gp_Pnt(1+1.e-11,0,0)),1.e-11,"circular ON tolerance must not erase physical exterior distance");
        BRepBuilderAPI_MakeFace circularHole(plane(),polygon({{0,0,0},{4,0,0},{4,4,0},{0,4,0}}),true);
        circularHole.Add(TopoDS::Wire(BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(2,2,0),gp_Dir(0,0,1)),.5))).Wire().Reversed()));
        check(BRepCheck_Analyzer(circularHole.Face()).IsValid(),"circular hole CAD validity");
        const algorithm6audit::CadFaceQueries arcHole(circularHole.Face());
        check(arcHole.usesAnalyticPlane(),"circular hole keeps exact analytic boundary");
        check(arcHole.classify(gp_Pnt2d(2,2))==TopAbs_OUT,"circular hole must not fill");
        checkDistance(arcHole.distance(gp_Pnt(2,2,.4)),std::sqrt(.41),"circular hole plus normal distance");
        checkDistance(arcHole.distance(gp_Pnt(.5,.5,.4)),.4,"circular hole material projection distance");
        checkDistance(arcHole.distance(gp_Pnt(2.5-1.e-11,2,0)),1.e-11,"near circular hole boundary retains positive physical distance");
        BRepBuilderAPI_MakeWire halfWire;
        halfWire.Add(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),1),0,3.14159265358979323846));
        halfWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(-1,0,0),gp_Pnt(1,0,0)));
        const auto halfFace=BRepBuilderAPI_MakeFace(plane(),halfWire.Wire(),true).Face();
        check(BRepCheck_Analyzer(halfFace).IsValid(),"half disk CAD validity");
        const algorithm6audit::CadFaceQueries half(halfFace);
        check(half.classify(gp_Pnt2d(0,.5))==TopAbs_IN,"partial arc interior");
        check(half.classify(gp_Pnt2d(0,-.5))==TopAbs_OUT,"partial arc chord exterior");
        check(half.classify(gp_Pnt2d(0,0))==TopAbs_ON,"partial arc endpoint ray shared with chord");
        checkDistance(half.distance(gp_Pnt(0,-1.e-11,0)),1.e-11,"partial arc chord exterior distance");
        checkDistance(half.distance(gp_Pnt(0,1+1.e-11,0)),1.e-11,"partial arc tangent exterior distance");
        BRepBuilderAPI_MakeWire clockwiseWire;
        clockwiseWire.Add(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,-1),gp_Dir(1,0,0)),1),0,3.14159265358979323846));
        clockwiseWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(-1,0,0),gp_Pnt(1,0,0)));
        const auto clockwiseFace=BRepBuilderAPI_MakeFace(plane(),clockwiseWire.Wire(),true).Face();
        check(BRepCheck_Analyzer(clockwiseFace).IsValid(),"clockwise partial arc CAD validity");
        const algorithm6audit::CadFaceQueries clockwise(clockwiseFace);
        check(clockwise.usesAnalyticPlane(),"clockwise partial arc consistency");
        check(clockwise.classify(gp_Pnt2d(0,-.5))==TopAbs_IN,"clockwise partial arc interior");
        check(clockwise.classify(gp_Pnt2d(0,.5))==TopAbs_OUT,"clockwise partial arc exterior");
        for(int i=-12;i<=12;++i)for(int j=-2;j<=12;++j){
            const double x=i/10.,y=j/10.;
            if(y==0||std::abs(x*x+y*y-1)<1.e-12)continue;
            const bool expected=y>0&&x*x+y*y<1;
            check((half.classify(gp_Pnt2d(x,y))==TopAbs_IN)==expected,"half disk analytic membership grid");
        }
        BRepBuilderAPI_MakeWire quadrantWire;
        quadrantWire.Add(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),1),4.71238898038468985769,6.28318530717958647692));
        quadrantWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(1,0,0),gp_Pnt(0,0,0)));
        quadrantWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0,0,0),gp_Pnt(0,-1,0)));
        const auto quadrantFace=BRepBuilderAPI_MakeFace(plane(),quadrantWire.Wire(),true).Face();
        check(BRepCheck_Analyzer(quadrantFace).IsValid(),"wrapped quarter disk CAD validity");
        const algorithm6audit::CadFaceQueries quadrant(quadrantFace);
        check(quadrant.usesAnalyticPlane(),"wrapped partial arc accepted");
        for(int i=-2;i<=12;++i)for(int j=-12;j<=2;++j){
            const double x=i/10.,y=j/10.;
            if(x==0||y==0||std::abs(x*x+y*y-1)<1.e-12)continue;
            const bool expected=x>0&&y<0&&x*x+y*y<1;
            if((quadrant.classify(gp_Pnt2d(x,y))==TopAbs_IN)!=expected)
                std::cerr<<"quarter disk mismatch at "<<x<<','<<y<<" actual state "<<int(quadrant.classify(gp_Pnt2d(x,y)))<<" expected inside "<<expected<<'\n';
            check((quadrant.classify(gp_Pnt2d(x,y))==TopAbs_IN)==expected,"wrapped quarter disk analytic membership grid");
        }
        for(const double center:{.12345,7.1})for(const double radius:{.00123,.3})for(const double offset:{0.,.74778}){
            const double centerY=center+offset;
            BRepBuilderAPI_MakeWire shiftedWire;
            shiftedWire.Add(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(center,centerY,0),gp_Dir(0,0,1)),radius),4.71238898038468985769,6.28318530717958647692));
            shiftedWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(center+radius,centerY,0),gp_Pnt(center,centerY,0)));
            shiftedWire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(center,centerY,0),gp_Pnt(center,centerY-radius,0)));
            const auto shiftedFace=BRepBuilderAPI_MakeFace(plane(),shiftedWire.Wire(),true).Face();
            check(BRepCheck_Analyzer(shiftedFace).IsValid(),"translated quarter disk CAD validity");
            const algorithm6audit::CadFaceQueries shifted(shiftedFace);
            check(shifted.usesAnalyticPlane(),"translated quarter disk consistency gate");
            check(shifted.classify(gp_Pnt2d(center-2*radius,centerY-radius))==TopAbs_OUT,"translated tangent endpoint ray must not fill exterior");
            check(shifted.classify(gp_Pnt2d(center+.3*radius,centerY-.3*radius))==TopAbs_IN,"translated quarter disk interior");
        }
        const auto cylinder=BRepPrimAPI_MakeCylinder(1,2).Shape();bool tested=false;
        for(TopExp_Explorer it(cylinder,TopAbs_FACE);it.More();it.Next()){
            const auto curved=TopoDS::Face(it.Current());if(BRepAdaptor_Surface(curved).GetType()!=GeomAbs_Cylinder)continue;
            const algorithm6audit::CadFaceQueries cylindrical(curved);check(!cylindrical.usesExactLinearPlane(),"curved support keeps OCC path");
            check(cylindrical.classify(gp_Pnt2d(.7,.4))==TopAbs_IN,"cylinder UV IN");
            checkDistance(cylindrical.distance(gp_Pnt(std::cos(.7),std::sin(.7),.4)),0,"curved support existing distance");tested=true;
        }
        check(tested,"cylinder fallback exercised");
        // 原CAD共享顶点闭合，独立pcurve求值仍可能留下舍入量级端点间隙。
        const auto notch=BRepBuilderAPI_MakeFace(plane(),polygon({{0,0,0},{4,0,0},{4,4,0},{-1,4,0},{-1,1,0},{0,1,0}}),true).Face();
        bool changed=false;
        for(TopExp_Explorer it(notch,TopAbs_EDGE);it.More();it.Next()){
            const auto edge=TopoDS::Edge(it.Current());BRepAdaptor_Curve physical(edge);
            if(physical.Value(physical.FirstParameter()).Distance(gp_Pnt(0,0,0))>1.e-12||
               physical.Value(physical.LastParameter()).Distance(gp_Pnt(4,0,0))>1.e-12)continue;
            BRep_Builder builder;builder.UpdateEdge(edge,new Geom2d_Line(gp_Pnt2d(0,1.e-16),gp_Dir2d(1,-2.5e-17)),notch,BRep_Tool::Tolerance(edge));
            builder.SameParameter(edge,true);builder.SameRange(edge,true);changed=true;
        }
        check(changed&&BRepCheck_Analyzer(notch).IsValid(),"roundoff pcurve fixture must retain valid closed CAD");
        const algorithm6audit::CadFaceQueries roundoff(notch);
        check(roundoff.usesExactLinearPlane(),"roundoff consistent CAD remains on linear planar path");
        check(roundoff.classify(gp_Pnt2d(-.5,0))==TopAbs_OUT,"roundoff pcurve gap must not fill exterior notch");
        checkDistance(roundoff.distance(gp_Pnt(-.5,0,0)),.5,"roundoff pcurve gap must not erase exterior distance");
        check(roundoff.classify(gp_Pnt2d(2,0))==TopAbs_ON,"original physical bottom edge remains ON");
        for(TopExp_Explorer it(notch,TopAbs_EDGE);it.More();it.Next()){
            const auto edge=TopoDS::Edge(it.Current());BRepAdaptor_Curve physical(edge);
            if(physical.Value(physical.FirstParameter()).Distance(gp_Pnt(0,0,0))>1.e-12||
               physical.Value(physical.LastParameter()).Distance(gp_Pnt(4,0,0))>1.e-12)continue;
            BRep_Builder builder;builder.UpdateEdge(edge,new Geom2d_Line(gp_Pnt2d(0,5.e-11),gp_Dir2d(1,-1.25e-11)),notch,BRep_Tool::Tolerance(edge));
            builder.SameParameter(edge,true);builder.SameRange(edge,true);
        }
        check(!algorithm6audit::CadFaceQueries(notch).usesExactLinearPlane(),"geometric mismatch beyond roundoff must keep OCC path");
        for(TopExp_Explorer it(circle,TopAbs_EDGE);it.More();it.Next()){
            const auto edge=TopoDS::Edge(it.Current());BRep_Builder builder;
            builder.UpdateEdge(edge,new Geom2d_Circle(gp_Ax2d(gp_Pnt2d(0,5.e-11),gp_Dir2d(1,0)),1),circle,BRep_Tool::Tolerance(edge));
            builder.SameParameter(edge,true);builder.SameRange(edge,true);
        }
        check(!algorithm6audit::CadFaceQueries(circle).usesAnalyticPlane(),"nonroundoff circular pcurve mismatch must retain fallback");
        std::cout<<"PASS exact straight planar trim/distance; holes; orientation; location; curved fallback; shared-vertex closure\n";return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
