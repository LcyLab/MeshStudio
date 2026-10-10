#pragma once
// 测试工具的独立三维质量测量；不调用生成器的判据函数。
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <ostream>
#include <set>
#include <stdexcept>

namespace cadquality {
struct Metric {bool valid=false;double minEdge=0,maxEdge=0,minAngleDeg=0,maxAngleDeg=0,radiusEdge=0,shape=0;};
inline Metric measure(mesher::Point3D a,mesher::Point3D b,mesher::Point3D c) {
    const auto sub=[](auto p,auto q){return mesher::Point3D{p.x-q.x,p.y-q.y,p.z-q.z};};
    const auto norm=[](auto p){return std::hypot(p.x,p.y,p.z);};
    const auto ab=sub(b,a),ac=sub(c,a),bc=sub(c,b);
    const double x=norm(ab),y=norm(ac),z=norm(bc),s=std::max({x,y,z});
    Metric m;m.minEdge=std::min({x,y,z});m.maxEdge=s;
    if(!(m.minEdge>0) || !std::isfinite(s))return m;
    const mesher::Point3D u{ab.x/s,ab.y/s,ab.z/s},v{ac.x/s,ac.y/s,ac.z/s};
    const double area=std::hypot(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x);
    if(!(area>0) || !std::isfinite(area))return m;
    const double xn=x/s,yn=y/s,zn=z/s;
    m.shape=2*std::sqrt(3.)*area/(xn*xn+yn*yn+zn*zn);
    m.radiusEdge=xn*yn*zn/(2*area*(m.minEdge/s));
    const std::array<double,3> angles{std::atan2(area,(xn*xn+yn*yn-zn*zn)/2),
        std::atan2(area,(xn*xn+zn*zn-yn*yn)/2),std::atan2(area,(yn*yn+zn*zn-xn*xn)/2)};
    m.minAngleDeg=*std::min_element(angles.begin(),angles.end())*180/std::acos(-1.);
    m.maxAngleDeg=*std::max_element(angles.begin(),angles.end())*180/std::acos(-1.);
    m.valid=std::isfinite(m.radiusEdge)&&std::isfinite(m.minAngleDeg);return m;
}
struct Statistics {
    size_t total=0,bad=0,invalid=0;double minAngle=180,worst=0,p95=0,p99=0;
    size_t below25=0,below30=0,nearEquilateral=0; // all three angles in [50,70]
    double p1Angle=0,p5Angle=0,p50Angle=0,p1Shape=0,p5Shape=0,p50Shape=0;
};
struct Residual {size_t triangle=0;int face=-1,body=-1,cadEdge=-1;bool boundary=false,shortEdge=false;Metric metric;};
struct Report : Statistics {double minSize=0,qualityTarget=0;std::map<int,Statistics> byFace,byBody;std::vector<Residual> residuals;};
inline Report analyze(const mesher::SurfaceMeshResult& r,double threshold,double minSize) {
    if(!(threshold>0) || !(minSize>0) || !std::isfinite(threshold) || !std::isfinite(minSize))
        throw std::invalid_argument("invalid independent quality criteria");
    const auto n=r.mesh.triangles.size();
    if(r.triangleFaceIds.size()!=n || (!r.triangleBodyIds.empty() && r.triangleBodyIds.size()!=n))
        throw std::runtime_error("quality report metadata count mismatch");
    std::map<std::pair<int,int>,int> edges;std::map<int,int> nodes;
    for(auto e:r.boundaryEdges){edges[std::minmax(e.a,e.b)]=e.cadEdge;nodes.emplace(e.a,e.cadEdge);nodes.emplace(e.b,e.cadEdge);}
    Report report;report.minSize=minSize;report.qualityTarget=threshold;
    struct Distribution {std::vector<double> ratios,angles,shapes;};
    Distribution distribution;std::map<int,Distribution> faceDistribution,bodyDistribution;
    const auto add=[&](Statistics& s,const Metric& m){++s.total;if(!m.valid){++s.invalid;return;}
        s.bad+=m.radiusEdge>threshold;s.minAngle=std::min(s.minAngle,m.minAngleDeg);s.worst=std::max(s.worst,m.radiusEdge);};
    const auto sample=[&](Statistics& s,Distribution& d,const Metric& m){add(s,m);if(m.valid){
        s.below25+=m.minAngleDeg<25;s.below30+=m.minAngleDeg<30;
        s.nearEquilateral+=m.minAngleDeg>=50&&m.maxAngleDeg<=70;
        d.ratios.push_back(m.radiusEdge);d.angles.push_back(m.minAngleDeg);d.shapes.push_back(m.shape);}};
    for(size_t t=0;t<n;++t){const auto ids=r.mesh.triangles[t];
        for(int v:ids)if(v<0 || size_t(v)>=r.mesh.vertices.size())throw std::runtime_error("quality report invalid vertex index");
        const auto m=measure(r.mesh.vertices[ids[0]],r.mesh.vertices[ids[1]],r.mesh.vertices[ids[2]]);
        const int face=r.triangleFaceIds[t],body=r.triangleBodyIds.empty()?-1:r.triangleBodyIds[t];
        sample(report,distribution,m);sample(report.byFace[face],faceDistribution[face],m);
        sample(report.byBody[body],bodyDistribution[body],m);
        if(!m.valid || m.radiusEdge>threshold){Residual issue;issue.triangle=t;issue.face=face;issue.body=body;issue.metric=m;
            issue.shortEdge=m.minEdge<minSize;
            for(int k=0;k<3;++k){const auto node=nodes.find(ids[k]);if(node!=nodes.end()){issue.boundary=true;if(issue.cadEdge<0)issue.cadEdge=node->second;}}
            for(int k=0;k<3;++k){const auto edge=edges.find(std::minmax(ids[k],ids[(k+1)%3]));if(edge!=edges.end())issue.cadEdge=edge->second;}
            report.residuals.push_back(issue);
        }
    }
    const auto finish=[](Statistics& s,Distribution& d){
        const auto percentile=[](std::vector<double>& values,double p){
            if(values.empty())return 0.;std::sort(values.begin(),values.end());return values[size_t(p*(values.size()-1))];};
        s.p95=percentile(d.ratios,.95);s.p99=percentile(d.ratios,.99);
        s.p1Angle=percentile(d.angles,.01);s.p5Angle=percentile(d.angles,.05);s.p50Angle=percentile(d.angles,.5);
        s.p1Shape=percentile(d.shapes,.01);s.p5Shape=percentile(d.shapes,.05);s.p50Shape=percentile(d.shapes,.5);};
    finish(report,distribution);for(auto& [f,s]:report.byFace)finish(s,faceDistribution[f]);
    for(auto& [b,s]:report.byBody)finish(s,bodyDistribution[b]);
    return report;
}
inline void writeCsv(std::ostream& out,const Report& report,const mesher::SurfaceMeshResult& mesh) {
    if(!mesh.triangleCornerUV.empty() && mesh.triangleCornerUV.size()!=mesh.mesh.triangles.size())throw std::runtime_error("quality CSV UV count mismatch");
    if(!mesh.trianglePartitionIds.empty()&&(mesh.trianglePartitionIds.size()!=mesh.mesh.triangles.size()||
       mesh.trianglePartitionUV.size()!=mesh.mesh.triangles.size()))throw std::runtime_error("quality CSV partition count mismatch");
    out.precision(17);out<<"triangle,body,face,related_cad_edge,touches_boundary,below_min_size,min_edge,max_edge,min_angle_deg,radius_edge,reason,x0,y0,z0,x1,y1,z1,x2,y2,z2,min_size,quality_target,u0,v0,u1,v1,u2,v2,patch_id,patch_u0,patch_v0,patch_u1,patch_v1,patch_u2,patch_v2\n";
    for(const auto& r:report.residuals){out<<r.triangle<<','<<r.body<<','<<r.face<<','<<r.cadEdge<<','<<r.boundary<<','<<r.shortEdge<<','
        <<r.metric.minEdge<<','<<r.metric.maxEdge<<','<<r.metric.minAngleDeg<<','<<r.metric.radiusEdge<<",unclassified";
        for(int v:mesh.mesh.triangles.at(r.triangle)){const auto p=mesh.mesh.vertices.at(v);out<<','<<p.x<<','<<p.y<<','<<p.z;}
        out<<','<<report.minSize<<','<<report.qualityTarget;
        if(mesh.triangleCornerUV.empty())out<<",,,,,,";
        else for(auto uv:mesh.triangleCornerUV.at(r.triangle))out<<','<<uv.x<<','<<uv.y;
        if(mesh.trianglePartitionIds.empty())out<<",,,,,,,";
        else {out<<','<<mesh.trianglePartitionIds.at(r.triangle);for(auto uv:mesh.trianglePartitionUV.at(r.triangle))out<<','<<uv.x<<','<<uv.y;}
        out<<'\n';}
    if(!out)throw std::runtime_error("cannot write quality residual CSV");
}
}
