// 文件职责：局部表面重剖分：拆分约束边、重建内部一环或恢复已验收图表，保持原节点身份和失败回滚。
#include "SurfaceLocalRemesh.h"
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/planar/Constrained2D.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>
namespace mesher {
/// 从已验收的逐片 UV/全局节点重建图表；不再剖分或改变原片，保留周期分支。
bool rebuildAcceptedSurfaceChart(const std::vector<std::array<int,3>>& triangles,
    const std::vector<std::array<Point2D,3>>& uv,Mesh2D& mesh,std::vector<int>& global,
    std::vector<std::pair<int,int>>& constraints,std::string* error,MeshObservation observation) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(triangles.size()!=uv.size())return fail("accepted chart metadata mismatch");
    Mesh2D restored;std::vector<int> ids;std::vector<std::pair<int,int>> edges;
    std::map<std::tuple<int,double,double>,int> nodes;
    std::set<std::pair<int,int>> halfedges;
    for(size_t t=0;t<triangles.size();++t){std::array<int,3> local;
        for(int k=0;k<3;++k)if(triangles[t][k]<0||!std::isfinite(uv[t][k].x)||!std::isfinite(uv[t][k].y))
            return fail("invalid accepted chart corner");
        if(orient2d(uv[t][0],uv[t][1],uv[t][2])<=0)return fail("accepted chart has a non-positive UV facet");
        for(int k=0;k<3;++k){auto p=uv[t][k];
            auto [it,added]=nodes.emplace(std::make_tuple(triangles[t][k],p.x,p.y),int(ids.size()));
            if(added){restored.addPoint(p);ids.push_back(triangles[t][k]);}local[k]=it->second;}
        for(int k=0;k<3;++k)if(!halfedges.emplace(local[k],local[(k+1)%3]).second)return fail("accepted chart repeats a directed edge");
        restored.addTri(local[0],local[1],local[2]);}
    for(const auto& t:restored.tris)for(int k=0;k<3;++k)if(t.n[k]<0)edges.push_back({t.v[k],t.v[(k+1)%3]});
    if(!restored.validate(error,observation))return false;
    mesh=std::move(restored);global=std::move(ids);constraints=std::move(edges);return true;
}
/// 局部拆分一个显式边界段，保留所有原点及其索引。
bool trySplitSurfaceBoundaryEdge(Mesh2D& mesh,int a,int b,Point2D point,
    std::vector<std::pair<int,int>>& constraints,SurfaceDomainState& domain,int* inserted,
    MeshEditDelta* published,std::string* error,MeshObservation observation,
    const std::function<bool(const Mesh2D&,int,const MeshEditDelta&)>& accept){
    if(inserted)*inserted=-1;
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(a<0||b<0||a==b||a>=int(mesh.points.size())||b>=int(mesh.points.size())||
        !std::isfinite(point.x)||!std::isfinite(point.y)||mesh.exactPoint(point)>=0)
        return fail("invalid or duplicate boundary split point");
    std::vector<std::pair<int,int>> replacement;
    if(!expandConstraintEdges(mesh,constraints,replacement,error,observation))return false;
    auto segment=std::find_if(replacement.begin(),replacement.end(),[&](auto e){
        return (e.first==a&&e.second==b)||(e.first==b&&e.second==a);});
    if(segment==replacement.end())return fail("boundary split edge is not a constraint");
    const int vertex=int(mesh.points.size());
    const auto old=*segment;*segment={old.first,vertex};replacement.push_back({vertex,old.second});
    std::vector<int> slots;std::vector<std::array<int,3>> children;
    for(auto edge:std::vector<std::pair<int,int>>{{a,b},{b,a}}){
        const int he=mesh.findHalfedge(edge.first,edge.second);if(he<0)continue;
        const auto tri=mesh.tris[he/3];const int opposite=tri.v[(he%3+2)%3];
        if(orient2d(mesh.points[edge.first],point,mesh.points[opposite])<=0||
            orient2d(point,mesh.points[edge.second],mesh.points[opposite])<=0)
            return fail("boundary split would invert an incident triangle");
        slots.push_back(he/3);children.push_back({edge.first,vertex,opposite});
        children.push_back({vertex,edge.second,opposite});
    }
    if(slots.empty())return fail("boundary split edge has no incident triangle");
    Mesh2DTransaction edit(mesh,observation);mesh.addPoint(point);
    std::sort(slots.rbegin(),slots.rend());for(int slot:slots)mesh.removeTri(slot);
    for(auto tri:children)mesh.addTri(tri[0],tri[1],tri[2]);
    // 边界两侧可能为不同域，独立重分类后再发布，不能合并继承标签。
    SurfaceDomainState updated;
    if(!updated.initialize(mesh,replacement,error,observation))return false;
    if(accept&&!accept(mesh,vertex,edit.pendingDelta()))return fail("boundary split acceptance veto");
    if(!edit.commit(error,published,replacement))return false;
    constraints.swap(replacement);domain=std::move(updated);
    if(inserted)*inserted=vertex;return true;
}
/// 移除未受约束的内部点连接，最多 16 个邻点，在原 UV 多边形内求最小最差成本剖分。
bool tryRemeshSurfaceVertex(Mesh2D& mesh,int vertex,const std::vector<std::pair<int,int>>& constraints,
    SurfaceDomainState& domain,const SurfaceTriangleCost& cost,MeshEditDelta* published,std::string* error,
    MeshObservation observation,const std::function<bool(const Mesh2D&,const MeshEditDelta&)>& accept,
    const std::vector<int>* incidentSlots){
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(vertex<0||vertex>=int(mesh.points.size())||!cost)return fail("invalid local remesh input");
    for(auto edge:constraints)if(edge.first==vertex||edge.second==vertex)return fail("protected surface vertex");
    std::vector<int> incident;std::map<int,int> next;std::set<int> incoming;
    if(incidentSlots)incident=*incidentSlots;
    else for(int t=0;t<int(mesh.tris.size());++t){const auto tri=mesh.tris[t];
        if(tri.v[0]==vertex||tri.v[1]==vertex||tri.v[2]==vertex)incident.push_back(t);}
    std::set<int> slots(incident.begin(),incident.end());
    if(slots.size()!=incident.size())return fail("duplicate local remesh incident slot");
    for(int t:incident){if(t<0||t>=int(mesh.tris.size()))return fail("stale local remesh incident slot");
        const auto tri=mesh.tris[t];bool found=false;
        for(int k=0;k<3;++k)if(tri.v[k]==vertex){
            if(t>=int(domain.values().size())||!domain.values()[t]||tri.n[k]<0||tri.n[(k+2)%3]<0)
                return fail("surface vertex does not have an interior disk");
            if(!slots.count(tri.n[k])||!slots.count(tri.n[(k+2)%3]))return fail("incomplete local remesh incident slots");
            const int a=tri.v[(k+1)%3],b=tri.v[(k+2)%3];
            if(!next.emplace(a,b).second||!incoming.insert(b).second)return fail("nonmanifold local remesh ring");
            found=true;break;}
        if(!found)return fail("stale local remesh incident slot");
    }
    if(incident.size()<3||incident.size()>16||!domain.uniform(incident,error))return fail("unsupported local remesh valence/domain");
    std::vector<int> ring;int first=next.begin()->first,v=first;
    do{if(ring.size()>=next.size()||!next.count(v))return fail("incomplete local remesh ring");
        ring.push_back(v);v=next.at(v);}while(v!=first);
    if(ring.size()!=next.size())return fail("multiple local remesh rings");
    const int n=int(ring.size());
    const auto point=[&](int i){return mesh.points[ring[i]];};
    const auto onSegment=[](Point2D a,Point2D b,Point2D p){return orient2d(a,b,p)==0&&
        p.x>=std::min(a.x,b.x)&&p.x<=std::max(a.x,b.x)&&p.y>=std::min(a.y,b.y)&&p.y<=std::max(a.y,b.y);};
    const auto inside=[&](Point2D p){int winding=0;
        for(int i=0;i<n;++i){auto a=point(i),b=point((i+1)%n);if(onSegment(a,b,p))return false;
            if(a.y<=p.y&&b.y>p.y&&orient2d(a,b,p)>0)++winding;
            if(a.y>p.y&&b.y<=p.y&&orient2d(a,b,p)<0)--winding;}return winding!=0;};
    const auto diagonal=[&](int i,int j){if(i==j||j==i+1||(i==0&&j==n-1))return true;
        auto a=point(i),b=point(j);
        if(mesh.hasEdgeAny(ring[i],ring[j]))return false; // 不借用补丁外已有连接。
        for(int k=0;k<n;++k){int l=(k+1)%n;if(k==i||k==j||l==i||l==j)continue;
            auto c=point(k),d=point(l);double o0=orient2d(a,b,c),o1=orient2d(a,b,d),o2=orient2d(c,d,a),o3=orient2d(c,d,b);
            if(onSegment(a,b,c)||onSegment(a,b,d)||onSegment(c,d,a)||onSegment(c,d,b)||
                ((o0>0)!=(o1>0)&&(o2>0)!=(o3>0)))return false;}
        return inside({a.x+(b.x-a.x)/2,a.y+(b.y-a.y)/2});};
    std::vector<std::vector<bool>> allowed(n,std::vector<bool>(n,false));
    for(int i=0;i<n;++i)for(int j=i+1;j<n;++j)allowed[i][j]=diagonal(i,j);
    struct Score{double worst=std::numeric_limits<double>::infinity(),sum=0;int split=-1;};
    std::vector<std::vector<Score>> dp(n,std::vector<Score>(n));
    for(int i=0;i<n;++i){dp[i][i].worst=0;if(i+1<n)dp[i][i+1].worst=0;}
    // 多边形动态规划，枚举全部有效剖分，按最差三维成本优先、总和次优选择。
    for(int span=2;span<n;++span)for(int i=0;i+span<n;++i){int j=i+span;if(!allowed[i][j])continue;
        for(int k=i+1;k<j;++k){if(!allowed[i][k]||!allowed[k][j]||
            !std::isfinite(dp[i][k].worst)||!std::isfinite(dp[k][j].worst)||orient2d(point(i),point(k),point(j))<=0)continue;
            const double value=cost({ring[i],ring[k],ring[j]});if(!std::isfinite(value)||!(value>0))continue;
            const double worst=std::max({value,dp[i][k].worst,dp[k][j].worst}),sum=value+dp[i][k].sum+dp[k][j].sum;
            auto& best=dp[i][j];if(worst<best.worst||(worst==best.worst&&sum<best.sum))best={worst,sum,k};
        }
    }
    if(!std::isfinite(dp[0][n-1].worst))return fail("no valid local surface retriangulation");
    std::vector<std::array<int,3>> replacement;
    std::function<void(int,int)> collect=[&](int i,int j){if(j<=i+1)return;int k=dp[i][j].split;
        replacement.push_back({ring[i],ring[k],ring[j]});collect(i,k);collect(k,j);};collect(0,n-1);
    Mesh2DTransaction edit(mesh,observation);SurfaceDomainState::Edit domainEdit(domain);
    std::sort(incident.rbegin(),incident.rend());for(int t:incident)mesh.removeTri(t);
    for(auto tri:replacement)mesh.addTri(tri[0],tri[1],tri[2]);
    if(!domainEdit.consume(edit.pendingDelta(),error)||!domainEdit.validate(mesh,edit.pendingDelta().changedSlots,error))return false;
    if(accept&&!accept(mesh,edit.pendingDelta()))return fail("local surface remesh acceptance veto");
    MeshEditDelta delta;if(!edit.commit(error,&delta,constraints))return false;
    domainEdit.commit();if(published)*published=std::move(delta);return true;
}
}
