// 文件职责：表面分区编排：识别可共同参数化的 CAD 面，生成分区网格，再补回原 CAD 支撑信息。
#include "SurfaceMeshingPartition.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <iomanip>

namespace mesher {
/// 将原 CAD 面 UV 转换到合并分区图表。
Point2D SurfaceMeshingPartition::AffineChart::toPatch(Point2D uv) const {return {a*uv.x+b*uv.y+offset.x,c*uv.x+d*uv.y+offset.y};}
/// 将分区图表 UV 转换回原 CAD 面参数。
Point2D SurfaceMeshingPartition::AffineChart::toSource(Point2D uv) const {const double det=a*d-b*c;uv.x-=offset.x;uv.y-=offset.y;return {(d*uv.x-b*uv.y)/det,(-c*uv.x+a*uv.y)/det};}
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p){return std::isfinite(p.x)&&std::isfinite(p.y);}
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
/// 计算向量点积。
double dot(Point3D a,Point3D b){return a.x*b.x+a.y*b.y+a.z*b.z;}
/// 计算向量的欧氏长度。
double norm(Point3D p){return std::hypot(p.x,p.y,p.z);}
/// 计算两点之间的欧氏距离。
double distance(Point3D a,Point3D b){return norm({a.x-b.x,a.y-b.y,a.z-b.z});}
/// 按受控的相对尺度比较两个 UV 坐标。
bool sameUV(Point2D a,Point2D b){return finite(a)&&finite(b)&&std::hypot(a.x-b.x,a.y-b.y)<=1e-9*std::max({1.,std::abs(a.x),std::abs(a.y),std::abs(b.x),std::abs(b.y)});}
/// 根据模型包围范围计算分区支撑查询的残差容差。
double geometryTolerance(const GeometrySource& source){Point3D lo,hi;return source.bounds(lo,hi)&&finite(lo)&&finite(hi)?1e-8*std::max(1.,distance(lo,hi)):1e-8;}
const std::string indexContract="Internal meshing diagnostics use virtual patch face IDs and retained-edge IDs; partition-builder CAD labels, result boundary edges, refinement CAD edges and CAD support records use original CAD IDs.";
/// 在原裁剪面集合中查找三维采样点的实际 CAD 支撑。
template<class Coordinates,class Continue>
bool exactTrimSupport(const GeometrySource& source,const std::vector<int>& faces,Point3D physical,double tolerance,
    const Coordinates& coordinates,SurfaceMeshResult::CadSupportSample& sample,const Continue& keepRunning,bool& cancelled) {
    sample={};if(!finite(physical))return false;
    for(size_t index=0;index<faces.size();++index){if(index%32==0&&!keepRunning()){cancelled=true;return false;}const int face=faces[index];
        auto native=coordinates(face);if(!finite(native))continue;const auto periods=source.facePeriods(face);SurfaceParameterBounds bounds;
        if(source.faceParameterBounds(face,bounds)){
            if(periods.x>0&&std::isfinite(periods.x))native.x+=std::round(((bounds.uMin+bounds.uMax)/2-native.x)/periods.x)*periods.x;
            if(periods.y>0&&std::isfinite(periods.y))native.y+=std::round(((bounds.vMin+bounds.vMax)/2-native.y)/periods.y)*periods.y;}
        for(int u=-1;u<=1;++u)for(int v=-1;v<=1;++v){if((u&&!(periods.x>0))||(v&&!(periods.y>0)))continue;
            const Point2D uv{native.x+u*periods.x,native.y+v*periods.y};if(!finite(uv))continue;
            const auto location=source.faceTrimLocation(face,uv);if(location!=CadTrimLocation::Inside&&location!=CadTrimLocation::Boundary)continue;
            const auto actual=source.facePoint(face,uv);if(finite(actual)&&distance(actual,physical)<=tolerance){sample={face,uv};return true;}}
    }
    return false;
}
}

/// 检查 CAD 图表兼容性，并按策略建立合并面分区。
SurfaceMeshingPartition::SurfaceMeshingPartition(const GeometrySource& source,const SurfacePartitionPolicy& policy,MeshingProgress progress) : m_source(source) {
    m_error=source.validationError();if(!m_error.empty())return;
    const int nv=source.vertexCount(),ne=source.edgeCount(),nf=source.faceCount();
    if(nv<1||ne<1||nf<1){m_error="empty CAD topology for surface partition";return;}
    const auto cancelled=[&](int done){if(progress&&!progress(done,nf)){m_error="surface meshing cancelled during partition preparation";return true;}return false;};
    if(cancelled(0))return;size_t preparationUnits=0;
    if(!policy.faceRegions.empty()&&policy.faceRegions.size()!=size_t(nf)){m_error="surface partition region count does not match CAD faces";return;}
    std::set<int> protectedEdges(policy.protectedCadEdges.begin(),policy.protectedCadEdges.end());
    for(int e:protectedEdges)if(e<0||e>=ne){m_error="invalid protected CAD edge";return;}
    std::vector<FaceTopology> topology(nf);std::vector<std::vector<BoundaryUse>> uses(ne);
    for(int f=0;f<nf;++f){if(f%64==0&&cancelled(f))return;topology[f]=source.faceTopology(f);
        for(const auto& loop:topology[f].loops)for(auto use:loop){if(preparationUnits++%64==0&&cancelled(f))return;
            if(use.edge<0||use.edge>=ne){m_error="invalid CAD edge in partition topology";return;}uses[use.edge].push_back({f,use});}}
    m_tolerance=geometryTolerance(source);const double tolerance=m_tolerance;
    const auto diagnostic=[&](const std::string& why){if(m_diagnostics.size()<16)m_diagnostics.push_back(why);};
    std::vector<bool> removable(ne,false);std::vector<int> parent(nf);std::iota(parent.begin(),parent.end(),0);
    const auto find=[&](int f){while(parent[f]!=f)f=parent[f];return f;};
    if(policy.enabled)for(int e=0;e<ne;++e){
        if(e%64==0&&cancelled(0))return;
        if(uses[e].size()!=2||uses[e][0].sourceFace==uses[e][1].sourceFace)continue;
        const int a=uses[e][0].sourceFace,b=uses[e][1].sourceFace;
        if(protectedEdges.count(e)||source.edgeDegenerate(e)||source.faceBodyId(a)<0||source.faceBodyId(a)!=source.faceBodyId(b)||
            (!policy.faceRegions.empty()&&policy.faceRegions[a]!=policy.faceRegions[b]))continue;
        if(!source.commonSurfaceChart(a,b)){diagnostic("CAD edge "+std::to_string(e)+" retained: no exact common analytic chart");continue;}
        const auto range=source.edgeRange(e);const double t=range.first+(range.second-range.first)/2;
        const auto ua=source.edgeUV(a,uses[e][0].sourceUse,t),ub=source.edgeUV(b,uses[e][1].sourceUse,t);
        const auto na=source.faceNormal(a,ua),nb=source.faceNormal(b,ub);const double lenA=norm(na),lenB=norm(nb);
        if(!finite(na)||!finite(nb)||!(lenA>0&&lenB>0)||dot(na,nb)/(lenA*lenB)<1-1e-8){diagnostic("CAD edge "+std::to_string(e)+" retained: oriented surface normals disagree");continue;}
        removable[e]=true;const int pa=find(a),pb=find(b);if(pa!=pb)parent[std::max(pa,pb)]=std::min(pa,pb);
    }
    std::map<int,std::vector<int>> groups;for(int f=0;f<nf;++f){if(f%64==0&&cancelled(f))return;groups[find(f)].push_back(f);}
    const auto single=[&](int f){Patch patch;patch.representative=f;patch.topology=topology[f];patch.charts[f]={};
        patch.bounded=source.faceParameterBounds(f,patch.bounds);for(auto& loop:patch.topology.loops)for(auto& use:loop){if(preparationUnits++%64==0&&cancelled(f))return;
            patch.uses.push_back({f,use});use.branch=int(patch.uses.size())-1;}
        m_patches.push_back(std::move(patch));m_sourceFaces.push_back({f});};
    for(const auto& group:groups){const auto& faces=group.second;if(cancelled(faces.front()))return;if(faces.size()==1){single(faces.front());if(!m_error.empty())return;continue;}
        Patch patch;patch.representative=faces.front();patch.charts[patch.representative]={};std::string refusal;
        const std::set<int> members(faces.begin(),faces.end());std::set<int> removed;
        for(int e=0;e<ne;++e){if(e%64==0&&cancelled(faces.front()))return;std::set<int> owners;for(auto use:uses[e])if(members.count(use.sourceFace))owners.insert(use.sourceFace);
            if(owners.size()>1){if(!removable[e]||uses[e].size()!=2){refusal="protected, semantic or ambiguous internal edge";break;}removed.insert(e);}}
        // 每个成员都须与代表面共享图表，仅存在 G1 连续链还不够。
        for(int f:faces){if(preparationUnits++%64==0&&cancelled(f))return;
            if(f!=patch.representative&&!source.commonSurfaceChart(patch.representative,f)){refusal="group has no common representative chart";break;}}
        while(refusal.empty()&&patch.charts.size()<faces.size()){
            bool advanced=false;
            for(int e:removed){if(preparationUnits++%64==0&&cancelled(faces.front()))return;auto left=uses[e][0],right=uses[e][1];
                if(!patch.charts.count(left.sourceFace)&&patch.charts.count(right.sourceFace))std::swap(left,right);
                if(!patch.charts.count(left.sourceFace)||patch.charts.count(right.sourceFace))continue;
                const auto range=source.edgeRange(e);const double t=range.first+(range.second-range.first)/2;
                const auto native=source.edgeUV(right.sourceFace,right.sourceUse,t);
                const auto seed=patch.charts.at(left.sourceFace).toPatch(source.edgeUV(left.sourceFace,left.sourceUse,t));
                const auto physical=source.facePoint(right.sourceFace,native);Point2D anchor;
                if(!finite(native)||!finite(seed)||!finite(physical)||!source.faceCoordinates(patch.representative,physical,seed,anchor)||!finite(anchor)||!sameUV(anchor,seed)){
                    refusal="shared-edge projection or periodic branch could not be aligned";break;}
                Point3D ru,rv,su,sv;source.faceDerivatives(patch.representative,anchor,ru,rv);source.faceDerivatives(right.sourceFace,native,su,sv);
                const double uu=dot(ru,ru),uv=dot(ru,rv),vv=dot(rv,rv),det=uu*vv-uv*uv;
                if(!finite(ru)||!finite(rv)||!finite(su)||!finite(sv)||!std::isfinite(det)||!(det>1e-14*uu*vv)){refusal="singular chart derivatives";break;}
                AffineChart chart;chart.a=(vv*dot(ru,su)-uv*dot(rv,su))/det;chart.c=(uu*dot(rv,su)-uv*dot(ru,su))/det;
                chart.b=(vv*dot(ru,sv)-uv*dot(rv,sv))/det;chart.d=(uu*dot(rv,sv)-uv*dot(ru,sv))/det;
                const double chartDet=chart.a*chart.d-chart.b*chart.c;
                if(!std::isfinite(chartDet)||std::abs(chartDet)<=1e-14*std::max(std::abs(chart.a*chart.d),std::abs(chart.b*chart.c))){refusal="non-bijective affine chart";break;}
                chart.offset={anchor.x-chart.a*native.x-chart.b*native.y,anchor.y-chart.c*native.x-chart.d*native.y};
                patch.charts[right.sourceFace]=chart;advanced=true;
            }
            if(!advanced&&refusal.empty())refusal="chart group is not connected";
        }
        // 移除边前同时核对物理支撑与 UV 分支一致性。
        for(int f:faces){if(!refusal.empty())break;for(const auto& loop:topology[f].loops)for(auto use:loop){const auto range=source.edgeRange(use.edge);
            for(double w:{0.,.25,.5,.75,1.}){if(preparationUnits++%64==0&&cancelled(f))return;const double t=range.first+(range.second-range.first)*w;const auto native=source.edgeUV(f,use,t),uv=patch.charts.at(f).toPatch(native);
                const auto p=source.facePoint(f,native),q=source.facePoint(patch.representative,uv),edge=source.edgePoint(use.edge,t);
                if(!finite(uv)||!finite(p)||!finite(q)||!finite(edge)||distance(p,q)>tolerance||distance(p,edge)>tolerance){refusal="affine boundary map failed physical residual verification";break;}}
            if(!refusal.empty())break;}}
        for(int e:removed){if(!refusal.empty())break;const auto range=source.edgeRange(e);for(double w:{0.,.25,.5,.75,1.}){if(preparationUnits++%64==0&&cancelled(faces.front()))return;const double t=range.first+(range.second-range.first)*w;
            const auto& a=uses[e][0];const auto& b=uses[e][1];
            if(!sameUV(patch.charts.at(a.sourceFace).toPatch(source.edgeUV(a.sourceFace,a.sourceUse,t)),patch.charts.at(b.sourceFace).toPatch(source.edgeUV(b.sourceFace,b.sourceUse,t)))){refusal="periodic chart cycle or shared-edge UV conflict";break;}}}
        if(refusal.empty()){
            for(int f:faces)for(const auto& loop:topology[f].loops)for(auto use:loop){if(preparationUnits++%64==0&&cancelled(f))return;
                if(!removed.count(use.edge))patch.uses.push_back({f,use});}
            const auto ends=[&](int use,bool start){const auto& origin=patch.uses[use];const auto v=source.edgeVertices(origin.sourceUse.edge);const auto r=source.edgeRange(origin.sourceUse.edge);
                const bool first=start!=origin.sourceUse.reversed;return std::pair<int,Point2D>{first?v.first:v.second,patch.charts.at(origin.sourceFace).toPatch(source.edgeUV(origin.sourceFace,origin.sourceUse,first?r.first:r.second))};};
            std::vector<bool> visited(patch.uses.size(),false);
            for(int first=0;first<int(patch.uses.size())&&refusal.empty();++first){if(visited[first])continue;std::vector<EdgeUse> loop;int current=first;
                for(size_t steps=0;steps<=patch.uses.size();++steps){if(preparationUnits++%64==0&&cancelled(faces.front()))return;if(visited[current]){refusal="trim loop revisits a boundary use";break;}visited[current]=true;
                    const auto& use=patch.uses[current].sourceUse;loop.push_back({use.edge,use.reversed,current});const auto end=ends(current,false),begin=ends(first,true);
                    if(end.first==begin.first&&sameUV(end.second,begin.second))break;
                    int next=-1;for(int candidate=0;candidate<int(patch.uses.size());++candidate){if(preparationUnits++%64==0&&cancelled(faces.front()))return;if(!visited[candidate]){const auto start=ends(candidate,true);
                        if(start.first==end.first&&sameUV(start.second,end.second)){if(next>=0){refusal="trim continuation is not single-valued";break;}next=candidate;}}
                    }
                    if(!refusal.empty())break;if(next<0){refusal="trim boundary does not close in the common chart";break;}current=next;
                }
                if(refusal.empty())patch.topology.loops.push_back(std::move(loop));
            }
            if(patch.topology.loops.empty()&&refusal.empty())refusal="merged chart has no trim boundary";
        }
        if(!refusal.empty()){diagnostic("CAD faces starting at "+std::to_string(faces.front())+" retain original splits: "+refusal);for(int f:faces){single(f);if(!m_error.empty())return;}continue;}
        patch.bounded=true;patch.bounds={1e300,-1e300,1e300,-1e300};
        for(int f:faces){if(preparationUnits++%64==0&&cancelled(f))return;SurfaceParameterBounds b;if(!source.faceParameterBounds(f,b)){patch.bounded=false;break;}
            for(Point2D uv:std::array<Point2D,4>{{{b.uMin,b.vMin},{b.uMin,b.vMax},{b.uMax,b.vMin},{b.uMax,b.vMax}}}){const auto p=patch.charts.at(f).toPatch(uv);
                if(!finite(p)){patch.bounded=false;break;}patch.bounds.uMin=std::min(patch.bounds.uMin,p.x);patch.bounds.uMax=std::max(patch.bounds.uMax,p.x);patch.bounds.vMin=std::min(patch.bounds.vMin,p.y);patch.bounds.vMax=std::max(patch.bounds.vMax,p.y);}}
        m_removedEdges.insert(m_removedEdges.end(),removed.begin(),removed.end());m_patches.push_back(std::move(patch));m_sourceFaces.push_back(faces);
    }
    std::sort(m_removedEdges.begin(),m_removedEdges.end());
    std::set<int> retained;for(const auto& patch:m_patches)for(const auto& loop:patch.topology.loops)for(auto use:loop){if(preparationUnits++%64==0&&cancelled(patch.representative))return;retained.insert(use.edge);}
    m_edges.assign(retained.begin(),retained.end());std::vector<int> edgeToCompact(ne,-1);std::set<int> vertices;
    for(int e=0;e<int(m_edges.size());++e){if(e%64==0&&cancelled(0))return;edgeToCompact[m_edges[e]]=e;const auto v=source.edgeVertices(m_edges[e]);
        if(v.first<0||v.second<0||v.first>=nv||v.second>=nv){m_error="invalid CAD edge endpoint in partition";return;}vertices.insert(v.first);vertices.insert(v.second);}
    m_vertices.assign(vertices.begin(),vertices.end());m_originalToVertex.assign(nv,-1);for(int v=0;v<int(m_vertices.size());++v){if(v%64==0&&cancelled(0))return;m_originalToVertex[m_vertices[v]]=v;}
    for(auto& patch:m_patches)for(auto& loop:patch.topology.loops)for(auto& use:loop){if(preparationUnits++%64==0&&cancelled(patch.representative))return;use.edge=edgeToCompact[use.edge];}
    if(cancelled(nf))return;
}
/// 读取分区边对应的原 CAD 边编号。
int SurfaceMeshingPartition::originalEdge(int e) const {return m_edges.at(e);}
/// 读取分区使用的代表 CAD 面编号。
int SurfaceMeshingPartition::representativeFace(int f) const {return m_patches.at(f).representative;}
/// 查找分区参数点对应的原裁剪面及原 UV。
bool SurfaceMeshingPartition::supportAt(int f,Point2D uv,SurfaceMeshResult::CadSupportSample& sample) const {
    sample={};if(f<0||f>=faceCount()||!finite(uv))return false;bool cancelled=false;
    return exactTrimSupport(m_source,m_sourceFaces[f],facePoint(f,uv),m_tolerance,
        [&](int source){return m_patches[f].charts.at(source).toSource(uv);},sample,[]{return true;},cancelled);
}
/// 校验分区结果的 CAD 支撑，并补充原面标签和支撑记录。
bool SurfaceMeshingPartition::decorateResult(SurfaceMeshResult& result,MeshingProgress progress,double maximumSupportDeviation) const {
    const auto fail=[&](const std::string& why){result.error=why;result.ok=false;return false;};
    if(!std::isfinite(maximumSupportDeviation)||maximumSupportDeviation<0)return fail("invalid maximum CAD support deviation");
    const auto keepRunning=[&](int partition){return !progress||progress(partition,faceCount());};
    if(!keepRunning(0))return fail("surface meshing cancelled during CAD support decoration");
    const size_t n=result.mesh.triangles.size();if(!result.ok||result.triangleFaceIds.size()!=n||result.triangleCornerUV.size()!=n)return fail("partition output metadata count mismatch");
    const auto ids=result.triangleFaceIds;const auto patchUV=result.triangleCornerUV;
    std::vector<std::array<SurfaceMeshResult::CadSupportSample,7>> samples(n);
    auto originalFaces=ids;auto originalUV=patchUV;auto boundaries=result.boundaryEdges;
    auto refinements=result.refinementSamples;auto interactions=result.bodyInteractions;
    size_t decorationUnits=0;
    for(auto& sample:refinements){if(decorationUnits++%64==0&&!keepRunning(0))return fail("surface meshing cancelled during CAD support decoration");const int partition=sample.face;
        if(partition<0||partition>=faceCount())return fail("invalid refinement sample partition ID");
        if(sample.cadEdge<-1||sample.cadEdge>=edgeCount())return fail("invalid refinement sample retained-edge ID");
        sample.partition=partition;sample.face=m_sourceFaces[partition].size()==1?m_sourceFaces[partition].front():-1;
        if(sample.cadEdge>=0)sample.cadEdge=originalEdge(sample.cadEdge);
    }
    for(auto& interaction:interactions){if(decorationUnits++%64==0&&!keepRunning(0))return fail("surface meshing cancelled during CAD support decoration");const int first=interaction.firstFace,second=interaction.secondFace;
        if(first<0||first>=faceCount()||second<0||second>=faceCount())return fail("invalid body interaction partition ID");
        interaction.firstPartition=first;interaction.secondPartition=second;
        interaction.firstFace=m_sourceFaces[first].size()==1?m_sourceFaces[first].front():-1;
        interaction.secondFace=m_sourceFaces[second].size()==1?m_sourceFaces[second].front():-1;
    }
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(size_t t=0;t<n;++t){const int partition=ids[t];if(partition<0||partition>=faceCount())return fail("invalid output surface partition ID");
        if(t%64==0&&!keepRunning(partition))return fail("surface meshing cancelled during CAD support decoration");const auto& uv=patchUV[t];
        std::array<Point3D,3> xyz;for(int k=0;k<3;++k){const int v=result.mesh.triangles[t][k];
            if(v<0||v>=int(result.mesh.vertices.size())||!finite(result.mesh.vertices[v]))return fail("invalid physical vertex in CAD support decoration");xyz[k]=result.mesh.vertices[v];}
        const std::array<Point2D,7> points{{uv[0],uv[1],uv[2],{(uv[0].x+uv[1].x)/2,(uv[0].y+uv[1].y)/2},
            {(uv[1].x+uv[2].x)/2,(uv[1].y+uv[2].y)/2},{(uv[2].x+uv[0].x)/2,(uv[2].y+uv[0].y)/2},
            {uv[0].x/3+uv[1].x/3+uv[2].x/3,uv[0].y/3+uv[1].y/3+uv[2].y/3}}};
        const auto midpoint=[](Point3D a,Point3D b){return Point3D{a.x/2+b.x/2,a.y/2+b.y/2,a.z/2+b.z/2};};
        const std::array<Point3D,7> physical{{xyz[0],xyz[1],xyz[2],midpoint(xyz[0],xyz[1]),midpoint(xyz[1],xyz[2]),midpoint(xyz[2],xyz[0]),
            {xyz[0].x/3+xyz[1].x/3+xyz[2].x/3,xyz[0].y/3+xyz[1].y/3+xyz[2].y/3,xyz[0].z/3+xyz[1].z/3+xyz[2].z/3}}};
        for(int k=0;k<7;++k){if(!finite(points[k])||!finite(physical[k]))return fail("non-finite CAD support sample");
            auto& support=samples[t][k];bool cancelled=false;
            const auto coordinates=[&](int source){return m_patches[partition].charts.at(source).toSource(points[k]);};
            const auto continueSample=[&]{return keepRunning(partition);};
            bool found=exactTrimSupport(m_source,m_sourceFaces[partition],facePoint(partition,points[k]),m_tolerance,coordinates,support,continueSample,cancelled);
            if(cancelled)return fail("surface meshing cancelled during CAD support decoration");
            if(found)support.distance=distance(physical[k],m_source.facePoint(support.face,support.uv));
            else{double best=std::numeric_limits<double>::infinity();
                for(size_t s=0;s<m_sourceFaces[partition].size();++s){if(s%32==0&&!continueSample())return fail("surface meshing cancelled during CAD support decoration");
                    const int source=m_sourceFaces[partition][s];Point2D native;double reported=0;
                    if(!m_source.closestTrimCoordinates(source,physical[k],coordinates(source),native,reported)||!finite(native)||!std::isfinite(reported)||reported<0)continue;
                    const auto location=m_source.faceTrimLocation(source,native);if(location!=CadTrimLocation::Inside&&location!=CadTrimLocation::Boundary)continue;
                    const auto point=m_source.facePoint(source,native);const double actual=distance(physical[k],point);
                    if(finite(point)&&std::isfinite(actual)&&actual<best){best=actual;support={source,native,actual,true};found=true;}}
            }
            const auto p=physical[k];const double slack=64*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(p.x),std::abs(p.y),std::abs(p.z)});
            if(!found||!std::isfinite(support.distance)||support.distance>maximumSupportDeviation*(1+1e-8)+slack) {
                std::ostringstream diagnostic;diagnostic<<std::setprecision(17)<<"CAD support failure patch="<<partition<<" triangle="<<t<<" sample="<<k
                    <<" found="<<found<<" face="<<support.face<<" uv="<<support.uv.x<<','<<support.uv.y<<" distance="<<support.distance
                    <<" limit="<<maximumSupportDeviation<<" projected="<<support.projectedToTrim<<" facet="<<physical[k].x<<','<<physical[k].y<<','<<physical[k].z
                    <<" patchUV="<<points[k].x<<','<<points[k].y;
                result.diagnostics.push_back(diagnostic.str());
                return fail("triangle "+std::to_string(t)+" has no verified original CAD trim support within the support deviation at sample "+std::to_string(k));
            }
        }
        if(m_sourceFaces[partition].size()==1){originalFaces[t]=m_sourceFaces[partition].front();
            for(int k=0;k<3;++k)originalUV[t][k]=m_patches[partition].charts.at(originalFaces[t]).toSource(patchUV[t][k]);}
        else{originalFaces[t]=-1;for(auto& p:originalUV[t])p={nan,nan};}
    }
    for(auto& edge:boundaries){if(decorationUnits++%64==0&&!keepRunning(0))return fail("surface meshing cancelled during CAD support decoration");
        if(edge.cadEdge<0||edge.cadEdge>=edgeCount())return fail("invalid retained CAD edge in partition output");edge.cadEdge=originalEdge(edge.cadEdge);}
    if(!keepRunning(faceCount()))return fail("surface meshing cancelled during CAD support decoration");
    result.trianglePartitionIds=ids;result.trianglePartitionUV=patchUV;result.partitionSourceFaces=m_sourceFaces;
    result.triangleCadSupportSamples=std::move(samples);result.triangleFaceIds=std::move(originalFaces);result.triangleCornerUV=std::move(originalUV);result.boundaryEdges=std::move(boundaries);
    result.refinementSamples=std::move(refinements);result.bodyInteractions=std::move(interactions);
    result.diagnostics.insert(result.diagnostics.begin(),indexContract);
    return true;
}
/// 返回 CAD 顶点数量。
int SurfaceMeshingPartition::vertexCount() const {return int(m_vertices.size());}
/// 查询指定 CAD 顶点的三维坐标。
Point3D SurfaceMeshingPartition::vertexPoint(int v) const {return m_source.vertexPoint(m_vertices.at(v));}
/// 返回 CAD 边数量。
int SurfaceMeshingPartition::edgeCount() const {return int(m_edges.size());}
/// 查询 CAD 边关联的端点编号。
std::pair<int,int> SurfaceMeshingPartition::edgeVertices(int e) const {const auto v=m_source.edgeVertices(originalEdge(e));return {m_originalToVertex.at(v.first),m_originalToVertex.at(v.second)};}
/// 查询 CAD 边的曲线参数范围。
std::pair<double,double> SurfaceMeshingPartition::edgeRange(int e) const {return m_source.edgeRange(originalEdge(e));}
/// 计算 CAD 边在指定曲线参数处的三维位置。
Point3D SurfaceMeshingPartition::edgePoint(int e,double t) const {return m_source.edgePoint(originalEdge(e),t);}
/// 计算 CAD 边在指定参数处的一阶导数。
Point3D SurfaceMeshingPartition::edgeDerivative(int e,double t) const {return m_source.edgeDerivative(originalEdge(e),t);}
/// 判断 CAD 边是否为退化边。
bool SurfaceMeshingPartition::edgeDegenerate(int e) const {return m_source.edgeDegenerate(originalEdge(e));}
/// 返回 CAD 面数量。
int SurfaceMeshingPartition::faceCount() const {return int(m_patches.size());}
/// 返回面的线环、边使用关系和参数信息。
FaceTopology SurfaceMeshingPartition::faceTopology(int f) const {return m_patches.at(f).topology;}
/// 计算边上节点在指定面分支中的 UV 坐标。
Point2D SurfaceMeshingPartition::edgeUV(int f,const EdgeUse& use,double t) const {const auto& patch=m_patches.at(f);const auto& origin=patch.uses.at(use.branch);return patch.charts.at(origin.sourceFace).toPatch(m_source.edgeUV(origin.sourceFace,origin.sourceUse,t));}
/// 计算指定面在 UV 参数处的三维位置。
Point3D SurfaceMeshingPartition::facePoint(int f,Point2D uv) const {return m_source.facePoint(representativeFace(f),uv);}
/// 计算面的参数导数，供局部度量和法向计算使用。
void SurfaceMeshingPartition::faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const {m_source.faceDerivatives(representativeFace(f),uv,du,dv);}
/// 查询底层曲面的参数范围。
bool SurfaceMeshingPartition::faceParameterBounds(int f,SurfaceParameterBounds& b) const {const auto& patch=m_patches.at(f);b=patch.bounds;return patch.bounded;}
/// 计算指定面参数处的单位法向。
Point3D SurfaceMeshingPartition::faceNormal(int f,Point2D uv) const {return m_source.faceNormal(representativeFace(f),uv);}
/// 说明法向是否已包含 CAD 面的方向约定。
bool SurfaceMeshingPartition::normalUsesOrientedDerivatives() const {return m_source.normalUsesOrientedDerivatives();}
/// 判断 CAD 面是否相对底层曲面反向。
bool SurfaceMeshingPartition::faceReversed(int f) const {return m_source.faceReversed(representativeFace(f));}
/// 返回几何输入的校验错误说明。
std::string SurfaceMeshingPartition::validationError() const {return m_error;}
/// 查询几何的三维包围范围。
bool SurfaceMeshingPartition::bounds(Point3D& lo,Point3D& hi) const {return m_source.bounds(lo,hi);}
/// 判断输入拓扑是否要求生成闭合表面。
bool SurfaceMeshingPartition::expectsClosedSurface() const {return m_source.expectsClosedSurface();}
/// 查询指定 CAD 面所属的部件编号。
int SurfaceMeshingPartition::faceBodyId(int f) const {return m_source.faceBodyId(representativeFace(f));}
/// 查询不同部件的 CAD 接触或相交关系。
CadBodyContact SurfaceMeshingPartition::bodyContact(int a,int b) const {
    bool contact=false,unknown=false;
    for(int first:m_sourceFaces.at(a))for(int second:m_sourceFaces.at(b)){
        const auto relation=m_source.bodyContact(first,second);if(relation==CadBodyContact::VolumeOverlap)return relation;
        contact|=relation==CadBodyContact::BoundaryContact;unknown|=relation==CadBodyContact::QueryFailed;
    }
    return unknown?CadBodyContact::QueryFailed:contact?CadBodyContact::BoundaryContact:CadBodyContact::None;
}
/// 构造分区适配器，生成表面并补充原 CAD 支撑信息。
SurfaceMeshResult generatePartitionedCadSurfaceMesh(const GeometrySource& source,const SurfaceCriteria& criteria,const SurfacePartitionPolicy& policy,const MeshSizeField* sizeField,MeshingProgress progress,MeshingObserver* observer,AssemblySurfacePolicy assemblyPolicy) {
    // 准备、生成和结果标注共用一个回调实例，使取消状态贯穿各阶段。
    MeshingProgress sharedProgress;if(progress)sharedProgress=[&](int done,int total){return progress(done,total);};
    SurfaceMeshingPartition partition(source,policy,sharedProgress);auto c=criteria;c.crossSmoothCadSplits=false;
    auto result=generateCadSurfaceMesh(partition,c,sizeField,sharedProgress,observer,assemblyPolicy);
    if(result.ok){double maximumSupportDeviation=criteria.maxDeviation;
        if(maximumSupportDeviation<=0){double target=criteria.targetSize;
            if(target<=0){Point3D lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};
                if(!partition.bounds(lo,hi))for(int v=0;v<partition.vertexCount();++v){
                    if(v%64==0&&sharedProgress&&!sharedProgress(0,partition.faceCount())){result.ok=false;result.error="surface meshing cancelled during CAD support decoration";break;}
                    const auto p=partition.vertexPoint(v);lo.x=std::min(lo.x,p.x);lo.y=std::min(lo.y,p.y);lo.z=std::min(lo.z,p.z);
                    hi.x=std::max(hi.x,p.x);hi.y=std::max(hi.y,p.y);hi.z=std::max(hi.z,p.z);}
                target=distance(lo,hi)*.025;}
            if(criteria.maxSize>0)target=std::min(target,criteria.maxSize);
            maximumSupportDeviation=target*.1;}
        if(result.ok&&!partition.decorateResult(result,sharedProgress,maximumSupportDeviation))result.ok=false;
    }
    result.diagnostics.insert(result.diagnostics.begin(),partition.diagnostics().begin(),partition.diagnostics().end());
    const auto existing=std::find(result.diagnostics.begin(),result.diagnostics.end(),indexContract);
    if(existing!=result.diagnostics.end())result.diagnostics.erase(existing);
    result.diagnostics.insert(result.diagnostics.begin(),indexContract);return result;
}
}
