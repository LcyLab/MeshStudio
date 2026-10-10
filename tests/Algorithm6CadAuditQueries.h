#pragma once
/** @file Algorithm6CadAuditQueries.h
 * @brief 独立最终审计的直线/圆弧平面裁剪及点到CAD面距离；不参与生成。
 * @date 2026-10-09
 */
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Plane.hxx>
#include <ElCLib.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <limits>
#include <utility>

namespace algorithm6audit {
class CadFaceQueries {
    TopoDS_Face face;
    Handle(Geom_Plane) support;
    gp_Trsf placement;
    struct Segment {gp_Pnt2d a,b;};
    std::vector<Segment> segments;
    bool exactLinearPlane=false;
    bool analyticPlane=false;
    struct Arc {gp_Circ2d circle;double first,last;gp_Pnt2d a,b;};
    std::vector<Arc> arcs;
    std::vector<TopoDS_Edge> analyticEdges;
    mutable BRepExtrema_DistShapeShape fallback;

    bool prepareLinearPlane(){
        TopLoc_Location location;support=Handle(Geom_Plane)::DownCast(BRep_Tool::Surface(face,location));
        if(support.IsNull())return false;
        placement=location.Transformation();
        const auto forward=TopoDS::Face(face.Oriented(TopAbs_FORWARD));
        std::vector<Segment> prepared;
        for(TopExp_Explorer loop(forward,TopAbs_WIRE);loop.More();loop.Next()){
            const auto wire=TopoDS::Wire(loop.Current());
            if(wire.Orientation()==TopAbs_INTERNAL||wire.Orientation()==TopAbs_EXTERNAL)continue;
            std::size_t expected=0;
            for(TopExp_Explorer edge(wire,TopAbs_EDGE);edge.More();edge.Next())
                if(edge.Current().Orientation()==TopAbs_FORWARD||edge.Current().Orientation()==TopAbs_REVERSED)++expected;
            std::vector<Segment> chain;
            TopoDS_Vertex firstVertex,previousEnd;
            for(BRepTools_WireExplorer edge(wire,forward);edge.More();edge.Next()){
                const auto current=edge.Current();const auto orientation=current.Orientation();
                if(orientation!=TopAbs_FORWARD&&orientation!=TopAbs_REVERSED)continue;
                if(BRep_Tool::Degenerated(current)||!BRep_Tool::SameParameter(current)||!BRep_Tool::SameRange(current))return false;
                BRepAdaptor_Curve physical(current);BRepAdaptor_Curve2d pcurve(current,forward);
                if(physical.GetType()!=GeomAbs_Line||pcurve.GetType()!=GeomAbs_Line)return false;
                const double first=pcurve.FirstParameter(),last=pcurve.LastParameter();
                auto pa=pcurve.Value(first),pb=pcurve.Value(last);
                auto ca=physical.Value(first),cb=physical.Value(last);
                if(orientation==TopAbs_REVERSED){std::swap(pa,pb);std::swap(ca,cb);}
                TopoDS_Vertex begin,end;TopExp::Vertices(current,begin,end,true);
                if(begin.IsNull()||end.IsNull())return false;
                if(chain.empty())firstVertex=begin;
                else if(!previousEnd.IsSame(begin))return false;
                const auto va=BRep_Tool::Pnt(begin),vb=BRep_Tool::Pnt(end);
                const auto axes=support->Position();
                const auto project=[&](const gp_Pnt& vertex){
                    const gp_Vec offset(axes.Location(),vertex.Transformed(placement.Inverted()));
                    return gp_Pnt2d(offset.Dot(gp_Vec(axes.XDirection())),offset.Dot(gp_Vec(axes.YDirection())));
                };
                const auto a=project(va),b=project(vb);
                if(a.SquareDistance(b)==0)return false;
                // 按原CAD共享顶点身份闭合；仅容许求值舍入误差，不合并几何间隙。
                const double magnitude=std::max({1.,std::abs(va.X()),std::abs(va.Y()),std::abs(va.Z()),
                    std::abs(vb.X()),std::abs(vb.Y()),std::abs(vb.Z()),std::abs(a.X()),std::abs(a.Y()),std::abs(b.X()),std::abs(b.Y())});
                const double roundoff=std::min(1.e-10,64*std::numeric_limits<double>::epsilon()*magnitude);
                if(va.Distance(ca)>roundoff||vb.Distance(cb)>roundoff||
                   va.Distance(support->Value(pa.X(),pa.Y()).Transformed(placement))>roundoff||
                   vb.Distance(support->Value(pb.X(),pb.Y()).Transformed(placement))>roundoff||
                   va.Distance(support->Value(a.X(),a.Y()).Transformed(placement))>roundoff||
                   vb.Distance(support->Value(b.X(),b.Y()).Transformed(placement))>roundoff)return false;
                chain.push_back({a,b});
                previousEnd=end;
            }
            if(chain.size()!=expected||chain.size()<3||!previousEnd.IsSame(firstVertex))return false;
            prepared.insert(prepared.end(),chain.begin(),chain.end());
        }
        if(prepared.empty())return false;
        segments=std::move(prepared);return true;
    }
    // 原CAD必须是共享顶点闭合、二维/三维解析边一致的平面；圆弧不折线化。
    bool prepareAnalyticPlane(){
        if(support.IsNull())return false;
        const auto forward=TopoDS::Face(face.Oriented(TopAbs_FORWARD));
        std::vector<TopoDS_Edge> prepared;std::vector<Segment> lines;std::vector<Arc> circles;bool hasCircle=false;
        for(TopExp_Explorer loop(forward,TopAbs_WIRE);loop.More();loop.Next()){
            const auto wire=TopoDS::Wire(loop.Current());
            if(wire.Orientation()==TopAbs_INTERNAL||wire.Orientation()==TopAbs_EXTERNAL)continue;
            std::size_t expected=0,count=0;
            for(TopExp_Explorer edge(wire,TopAbs_EDGE);edge.More();edge.Next())
                if(edge.Current().Orientation()==TopAbs_FORWARD||edge.Current().Orientation()==TopAbs_REVERSED)++expected;
            TopoDS_Vertex firstVertex,previousEnd;
            for(BRepTools_WireExplorer edge(wire,forward);edge.More();edge.Next()){
                const auto current=edge.Current();
                if(current.Orientation()!=TopAbs_FORWARD&&current.Orientation()!=TopAbs_REVERSED)continue;
                if(BRep_Tool::Degenerated(current)||!BRep_Tool::SameParameter(current)||!BRep_Tool::SameRange(current))return false;
                BRepAdaptor_Curve physical(current);BRepAdaptor_Curve2d pcurve(current,forward);
                const auto type=physical.GetType();
                if((type!=GeomAbs_Line&&type!=GeomAbs_Circle)||pcurve.GetType()!=type)return false;
                TopoDS_Vertex begin,end;TopExp::Vertices(current,begin,end,true);
                if(begin.IsNull()||end.IsNull()||(count&&!previousEnd.IsSame(begin)))return false;
                if(!count)firstVertex=begin;
                const auto checkPair=[&](const gp_Pnt& point,const gp_Pnt2d& uv){
                    const auto mapped=support->Value(uv.X(),uv.Y()).Transformed(placement);
                    const double magnitude=std::max({1.,point.Distance(gp_Pnt(0,0,0)),mapped.Distance(gp_Pnt(0,0,0))});
                    return point.Distance(mapped)<=std::min(1.e-10,64*std::numeric_limits<double>::epsilon()*magnitude);
                };
                const double first=pcurve.FirstParameter(),last=pcurve.LastParameter();
                const double rangeTolerance=64*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(first),std::abs(last)});
                if(!std::isfinite(first)||!std::isfinite(last)||last<=first||
                   std::abs(first-physical.FirstParameter())>rangeTolerance||std::abs(last-physical.LastParameter())>rangeTolerance)return false;
                if(!checkPair(physical.Value(first),pcurve.Value(first))||!checkPair(physical.Value(last),pcurve.Value(last)))return false;
                auto a=pcurve.Value(first),b=pcurve.Value(last);
                if(current.Orientation()==TopAbs_REVERSED)std::swap(a,b);
                if(!checkPair(BRep_Tool::Pnt(begin),a)||!checkPair(BRep_Tool::Pnt(end),b))return false;
                const auto project=[&](const TopoDS_Vertex& vertex){
                    const gp_Vec offset(support->Position().Location(),BRep_Tool::Pnt(vertex).Transformed(placement.Inverted()));
                    return gp_Pnt2d(offset.Dot(gp_Vec(support->Position().XDirection())),offset.Dot(gp_Vec(support->Position().YDirection())));
                };
                a=project(begin);b=project(end);
                if(type==GeomAbs_Circle){
                    hasCircle=true;
                    // 三个完整圆上的不同相位核验中心及正余弦系数，不依靠局部弧段采样。
                    for(const double parameter:{0.,1.57079632679489661923,3.14159265358979323846})
                        if(!checkPair(ElCLib::Value(parameter,physical.Circle()),ElCLib::Value(parameter,pcurve.Circle())))return false;
                    if(current.Orientation()==TopAbs_REVERSED)std::swap(a,b);
                    circles.push_back({pcurve.Circle(),first,last,a,b});
                }else lines.push_back({a,b});
                prepared.push_back(current);previousEnd=end;++count;
            }
            if(!count||count!=expected||!previousEnd.IsSame(firstVertex))return false;
        }
        if(!hasCircle||prepared.empty())return false;
        analyticEdges=std::move(prepared);segments=std::move(lines);arcs=std::move(circles);return true;
    }
    double boundarySquaredDistance(const gp_Pnt2d& point) const {
        double best=std::numeric_limits<double>::infinity();
        for(const auto& edge:segments){
            const double dx=edge.b.X()-edge.a.X(),dy=edge.b.Y()-edge.a.Y(),length=dx*dx+dy*dy;
            const double t=std::clamp(((point.X()-edge.a.X())*dx+(point.Y()-edge.a.Y())*dy)/length,0.,1.);
            const double x=point.X()-edge.a.X()-t*dx,y=point.Y()-edge.a.Y()-t*dy;
            best=std::min(best,x*x+y*y);
        }
        constexpr double period=6.28318530717958647692;
        for(const auto& arc:arcs){
            best=std::min({best,point.SquareDistance(arc.a),point.SquareDistance(arc.b)});
            double parameter=ElCLib::Parameter(arc.circle,point);
            parameter+=period*std::ceil((arc.first-parameter)/period);
            if(parameter<=arc.last)best=std::min(best,point.SquareDistance(ElCLib::Value(parameter,arc.circle)));
        }
        return best;
    }
    bool inside(const gp_Pnt2d& point) const {
        bool result=false;
        for(const auto& edge:segments)
            if((edge.a.Y()>point.Y())!=(edge.b.Y()>point.Y())&&
               point.X()<edge.a.X()+(point.Y()-edge.a.Y())*(edge.b.X()-edge.a.X())/(edge.b.Y()-edge.a.Y()))result=!result;
        constexpr double period=6.28318530717958647692;
        for(const auto& arc:arcs){
            const auto center=arc.circle.Location();const double radius=arc.circle.Radius();
            const double angleTolerance=64*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(arc.first),std::abs(arc.last)});
            if(arc.last-arc.first>=period-angleTolerance){
                if(point.SquareDistance(center)<radius*radius)result=!result;
                continue;
            }
            bool tangentEndpoint=false;
            for(const auto endpoint:{std::make_pair(arc.first,arc.a),std::make_pair(arc.last,arc.b)}){
                if(point.Y()!=endpoint.second.Y())continue;
                gp_Pnt2d value;gp_Vec2d tangent;ElCLib::D1(endpoint.first,arc.circle,value,tangent);
                if(std::abs(tangent.Y())>64*std::numeric_limits<double>::epsilon()*radius)continue;
                // 使用共享顶点的精确Y，不从(center-radius)-center重复减法判断极值。
                tangentEndpoint=true;
                if(value.Y()<center.Y()&&endpoint.second.X()>point.X())result=!result;
            }
            if(tangentEndpoint)continue;
            const double dy=point.Y()-center.Y();
            if(std::abs(dy)>=radius){
                // 弧内切点无穿越；最低点若是弧端点，仍须与相邻直线统一计数。
                if(dy==-radius){
                    if(point.Y()==arc.a.Y()&&arc.a.X()>point.X())result=!result;
                    if(point.Y()==arc.b.Y()&&arc.b.X()>point.X())result=!result;
                }
                continue;
            }
            const double dx=std::sqrt((radius-dy)*(radius+dy));
            for(const double x:{center.X()-dx,center.X()+dx}){
                double parameter=ElCLib::Parameter(arc.circle,gp_Pnt2d(x,point.Y()));
                parameter+=period*std::floor((arc.first-parameter)/period);
                if(parameter<arc.first-angleTolerance)parameter+=period;
                if(parameter<arc.first-angleTolerance||parameter>arc.last+angleTolerance)continue;
                double crossingX=x;gp_Pnt2d value;gp_Vec2d tangent;
                // 与相邻直线共享原CAD顶点，端点使用相同的半开计数，避免射线顶点重复计数。
                if(std::abs(parameter-arc.first)<=angleTolerance){
                    ElCLib::D1(arc.first,arc.circle,value,tangent);
                    if(!((tangent.Y()>0&&point.Y()>=arc.a.Y())||(tangent.Y()<0&&point.Y()<arc.a.Y())))continue;
                    crossingX=arc.a.X();
                }else if(std::abs(parameter-arc.last)<=angleTolerance){
                    ElCLib::D1(arc.last,arc.circle,value,tangent);
                    if(!((tangent.Y()>0&&point.Y()<arc.b.Y())||(tangent.Y()<0&&point.Y()>=arc.b.Y())))continue;
                    crossingX=arc.b.X();
                }
                if(crossingX>point.X())result=!result;
            }
        }
        return result;
    }
public:
    /** 本对象持有原CAD面；仅用于串行审计。不一致边环及非解析/非平面保留原OCC查询。 */
    explicit CadFaceQueries(const TopoDS_Face& input):face(input){
        exactLinearPlane=prepareLinearPlane();
        if(!exactLinearPlane)analyticPlane=prepareAnalyticPlane();
        if(!exactLinearPlane&&!analyticPlane)fallback.LoadS2(face);
    }
    bool usesExactLinearPlane() const {return exactLinearPlane;}
    bool usesAnalyticPlane() const {return analyticPlane;}
    /** tolerance保持原审计的UV容差；解析直线/圆弧奇偶计数包括全部外环及孔洞。 */
    TopAbs_State classify(const gp_Pnt2d& point,double tolerance=1.e-10) const {
        if(!std::isfinite(point.X())||!std::isfinite(point.Y())||!std::isfinite(tolerance)||tolerance<0)
            throw std::runtime_error("invalid CAD trim query");
        if(!exactLinearPlane&&!analyticPlane)return BRepClass_FaceClassifier(face,point,tolerance).State();
        if(boundarySquaredDistance(point)<=tolerance*tolerance)return TopAbs_ON;
        return inside(point)?TopAbs_IN:TopAbs_OUT;
    }
    /** 物理模型单位的点到完整裁剪面距离；解析距离不把ON容差当作零距离。 */
    double distance(const gp_Pnt& point) const {
        if(!std::isfinite(point.X())||!std::isfinite(point.Y())||!std::isfinite(point.Z()))throw std::runtime_error("invalid CAD distance point");
        if(exactLinearPlane||analyticPlane){
            const auto local=point.Transformed(placement.Inverted());const auto axes=support->Position();
            const gp_Vec offset(axes.Location(),local);
            const gp_Pnt2d uv(offset.Dot(gp_Vec(axes.XDirection())),offset.Dot(gp_Vec(axes.YDirection())));
            const double normal=offset.Dot(gp_Vec(axes.Direction()));
            if(analyticPlane){
                // 默认ExtPF裁剪分类会丢掉已证实在圆弧凹口平面内部的正交投影。
                // ON不按容差置零，仍量到原三维边，保留实际边外/孔内距离。
                if(inside(uv))return std::abs(normal*placement.ScaleFactor());
                double best=std::numeric_limits<double>::infinity();
                for(const auto& edge:analyticEdges){
                    BRepAdaptor_Curve curve(edge);const double first=curve.FirstParameter(),last=curve.LastParameter();
                    best=std::min({best,point.Distance(curve.Value(first)),point.Distance(curve.Value(last))});
                    if(curve.GetType()==GeomAbs_Line){
                        const auto line=curve.Line();const double parameter=std::clamp(ElCLib::Parameter(line,point),first,last);
                        best=std::min(best,point.Distance(ElCLib::Value(parameter,line)));
                    }else{
                        const auto circle=curve.Circle();constexpr double period=6.28318530717958647692;
                        double parameter=ElCLib::Parameter(circle,point);
                        parameter+=period*std::ceil((first-parameter)/period);
                        if(parameter<=last)best=std::min(best,point.Distance(ElCLib::Value(parameter,circle)));
                    }
                }
                if(!std::isfinite(best))throw std::runtime_error("analytic plane has no boundary distance");
                return best;
            }
            const double result=std::hypot(normal,inside(uv)?0.:std::sqrt(boundarySquaredDistance(uv)))*std::abs(placement.ScaleFactor());
            if(!std::isfinite(result))throw std::runtime_error("nonfinite planar CAD distance");
            return result;
        }
        fallback.LoadS1(BRepBuilderAPI_MakeVertex(point).Vertex());
        if(!fallback.Perform()||!fallback.IsDone()||!std::isfinite(fallback.Value()))throw std::runtime_error("CAD sample distance failed");
        return fallback.Value();
    }
};
}
