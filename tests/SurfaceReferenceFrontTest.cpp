#include "mesh/surface/legacy/SurfaceReferenceFront.h"
#include "mesh/surface/legacy/SurfaceTangentChart.h"
#include "geometry/predicates/GeomPredicates.h"
#include "SurfaceReferenceFixtures.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace mesher;
namespace {
struct Plane : GeometrySource {
    int vertexCount() const override {return 0;}
    Point3D vertexPoint(int) const override {return {};}
    int edgeCount() const override {return 0;}
    std::pair<int,int> edgeVertices(int) const override {return {-1,-1};}
    std::pair<double,double> edgeRange(int) const override {return {0,1};}
    Point3D edgePoint(int,double) const override {return {};}
    int faceCount() const override {return 1;}
    FaceTopology faceTopology(int) const override {return {};}
    Point2D edgeUV(int,const EdgeUse&,double) const override {return {};}
    Point3D facePoint(int,Point2D p) const override {return {p.x,p.y,0};}
    Point3D faceNormal(int,Point2D) const override {return {0,0,1};}
    void faceDerivatives(int,Point2D,Point3D& du,Point3D& dv) const override {du={1,0,0};dv={0,1,0};}
};
}
int main(int argc,char** argv) {
    if(argc==3||argc==4) {
        std::ifstream input(argv[1]);std::string schema;int shape,count,edgeCount;
        if(!(input>>schema>>shape>>count>>edgeCount)||(schema!="surface-reference-v1"&&schema!="surface-reference-v2")||shape<0||shape>4||count<3||edgeCount<3)return 2;
        referencefixture::AnalyticSurface geometry(shape);referencefixture::Fixture f;
        std::vector<Point2D> uv(count);std::vector<Point3D> points(count);std::vector<SurfaceReferenceSize> size(count);
        std::vector<std::pair<int,int>> edges(edgeCount);
        for(int k=0;k<count;++k)if(!(input>>uv[k].x>>uv[k].y>>points[k].x>>points[k].y>>points[k].z>>size[k].mesh>>size[k].background))return 2;
        for(auto& e:edges)if(!(input>>e.first>>e.second)||e.first<0||e.second<0||e.first>=count||e.second>=count)return 2;
        f.initialize(uv,points,edges,size);
        if(schema=="surface-reference-v2") {
            std::string error;if(!initializeSurfaceReferenceSizes(f.mesh,f.global,f.xyz,f.domain,f.constraints,f.sizes,&error)){std::cerr<<error;return 1;}
        }
        SurfaceReferenceStatistics stats;std::string error;
        SurfaceReferenceOptions options;
        if(argc==4){const std::string variant=argv[3];options.physicalCavity=variant=="physical-cavity"||variant=="physical-chart";options.physicalPointHeight=variant=="physical-chart";}
        if(!f.run(geometry,stats,error,{}, {},options)){std::cerr<<error<<'\n';return 1;}
        int triangles=0;for(bool inside:f.domain.values())triangles+=inside;
        std::ofstream output(argv[2]);output<<std::setprecision(17)<<f.xyz.size()<<' '<<triangles<<'\n';
        for(auto p:f.xyz)output<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
        for(int k=0;k<int(f.mesh.tris.size());++k)if(f.domain.values()[k]){const auto t=f.mesh.tris[k];output<<f.global[t.v[0]]<<' '<<f.global[t.v[1]]<<' '<<f.global[t.v[2]]<<'\n';}
        std::cout<<"reference inserted="<<stats.inserted<<" rejected="<<stats.rejected<<" remaining="<<stats.remainingLargeTriangles<<" circle="<<stats.circleCorrections<<" fallback="<<stats.circleFallbacks<<'\n';
        for(const auto& reason:stats.refusals)std::cout<<reason<<'\n';
        return output?0:2;
    }
    Plane plane;Mesh2D mesh;std::vector<Point3D> xyz={{2,7,0},{0,0,0},{8,0,0}};
    for(auto p:xyz)mesh.addPoint({p.x,p.y});mesh.addTri(0,1,2);
    std::vector<int> global={0,1,2};std::vector<std::pair<int,int>> constraints={{0,1},{1,2},{2,0}};
    SurfaceDomainState domain;std::string error;
    if(!domain.initialize(mesh,constraints,&error)){std::cerr<<error;return 1;}
    SurfaceCriteria criteria;criteria.targetSize=1;criteria.minSize=.1;criteria.maxDeviation=.01;
    const std::vector<SurfaceReferenceSize> sizes={{.5,.5},{1,1},{1.5,1.5}};
    SurfaceReferenceStatistics stats;int work=0;
    const bool generated=generateSurfaceReferenceFront(mesh,global,xyz,domain,constraints,plane,0,criteria,sizes,
        [](Point2D,Point3D p){return p.x==8?1.5:p.x==2&&p.y==7?.5:1.;},
        [&](Point3D p){const int id=int(xyz.size());xyz.push_back(p);return id;},work,stats,&error);
    bool matched=false;
    if(!stats.samples.empty()) {
        const auto& s=stats.samples.front();
        matched=s.baseUV[0].x==8&&s.baseUV[0].y==0&&s.baseUV[1].x==2&&s.baseUV[1].y==7&&std::abs(s.target-1)<1e-12;
        std::cout<<"first base="<<s.baseUV[0].x<<','<<s.baseUV[0].y<<" -> "<<s.baseUV[1].x<<','<<s.baseUV[1].y<<" target="<<s.target<<'\n';
    }
    if(!generated||!matched||stats.inserted!=0||stats.rejected!=1||mesh.tris.size()!=1||xyz.size()!=3||stats.remainingLargeTriangles!=1) {
        std::cerr<<"FAIL reference requires indexed active edge, endpoint mean and single-cavity rollback: "<<error<<'\n';return 1;
    }
    bool ok=true;
    {
        referencefixture::AnalyticSurface sphere(4);
        const std::array<Point2D,4> uv{Point2D{0,-.2},Point2D{.6,.35},Point2D{.5,-.15},Point2D{-.1,.4}};
        std::array<Point3D,4> points;for(int k=0;k<4;++k)points[k]=sphere.facePoint(0,uv[k]);
        SurfaceTangentChart reference;std::string why;
        const bool created=surfaceTangentQuadChartAt(sphere,0,uv,points,reference,&why);ok=ok&&created;
        std::array<int,4> order{0,1,2,3};
        do {
            std::array<Point2D,4> shuffled;std::array<Point3D,4> positions;
            for(int k=0;k<4;++k){shuffled[k]=uv[order[k]];positions[k]=points[order[k]];}
            SurfaceTangentChart chart;const bool ready=surfaceTangentQuadChartAt(sphere,0,shuffled,positions,chart,&why);
            ok=ok&&ready;
            if(created&&ready)for(auto p:points){const auto a=reference.project(p),b=chart.project(p);ok=ok&&a.x==b.x&&a.y==b.y;}
        }while(std::next_permutation(order.begin(),order.end()));
        if(created) {
            const auto a=reference.project(points[0]),b=reference.project(points[1]),c=reference.project(points[2]),d=reference.project(points[3]);
            ok=ok&&incircle(a,b,c,d)==-incircle(c,d,a,b);
        }
        auto bad=points;bad[0].x=std::numeric_limits<double>::quiet_NaN();const auto previous=reference.project(points[1]);
        const bool invalid=surfaceTangentQuadChartAt(sphere,0,uv,bad,reference,&why);const auto after=reference.project(points[1]);
        ok=ok&&!invalid&&why.find("non-finite")!=std::string::npos&&previous.x==after.x&&previous.y==after.y;
    }
    for(int guard:{0,1,2}) {
        // 周期缝两侧共享物理身份，跨域对角线必须先展开成有效物理种子。
        Mesh2D periodic;const double pi=std::acos(-1.);
        for(auto uv:std::vector<Point2D>{{0,-pi/2},{2*pi,-pi/2},{2*pi,0},{2*pi,pi/2},{0,pi/2},{0,0}})periodic.addPoint(uv);
        periodic.addTri(0,1,2);periodic.addTri(0,2,5);periodic.addTri(5,2,3);periodic.addTri(5,3,4);
        std::vector<int> mapping={0,0,1,2,2,1};std::vector<Point3D> points={{0,0,-8},{8,0,0},{0,0,8}};
        const std::vector<std::pair<int,int>> boundary={{0,1},{1,2},{2,3},{3,4},{4,5},{5,0}};
        SurfaceDomainState state;std::string why;const bool classified=state.initialize(periodic,boundary,&why);
        SurfaceCriteria limits;if(guard==1)limits.maxVertices=3;
        int steps=0,seeds=0;const auto originalPoints=periodic.points.size();
        const bool seeded=classified&&seedSurfaceReferenceChart(periodic,mapping,points,state,boundary,referencefixture::AnalyticSurface(4),0,limits,
            [&](Point3D p){int id=int(points.size());points.push_back(p);return id;},steps,seeds,&why,
            guard==2?MeshingProgress{[](int,int){return false;}}:MeshingProgress{});
        if(!guard) {
            ok=ok&&seeded&&seeds>0&&periodic.points.size()>originalPoints&&periodic.validate(&why)&&state.verify(periodic,boundary,&why);
            for(int slot=0;slot<int(periodic.tris.size());++slot)if(state.values()[slot]) {
                const auto tri=periodic.tris[slot];
                for(int e=0;e<3;++e)if(mapping[tri.v[e]]==mapping[tri.v[(e+1)%3]])
                    ok=ok&&std::any_of(boundary.begin(),boundary.end(),[&](auto edge){return std::minmax(edge.first,edge.second)==std::minmax(tri.v[e],tri.v[(e+1)%3]);});
            }
        }else ok=ok&&!seeded&&seeds==0&&steps==0&&periodic.points.size()==originalPoints&&points.size()==3&&why.find(guard==1?"budget":"cancelled")!=std::string::npos;
        std::cout<<"periodic bootstrap guard="<<guard<<" success="<<seeded<<" seeds="<<seeds<<" error="<<why<<'\n';
    }
    {
        // 合法UV图表中两个极点角共享一个物理节点；零物理边不可污染尺寸源。
        Mesh2D pole;for(auto uv:std::vector<Point2D>{{0,0},{1,0},{1,1},{0,1}})pole.addPoint(uv);
        pole.addTri(0,1,2);pole.addTri(0,2,3);
        std::vector<int> mapping={0,1,2,2};std::vector<Point3D> physical={{0,0,0},{1,0,0},{.5,1,0}};
        const std::vector<std::pair<int,int>> boundary={{0,1},{1,2},{2,3},{3,0}};
        SurfaceDomainState state;std::vector<SurfaceReferenceSize> initialized;std::string why;
        const bool classified=state.initialize(pole,boundary,&why);
        const bool ready=classified&&initializeSurfaceReferenceSizes(pole,mapping,physical,state,boundary,initialized,&why);
        ok=ok&&ready&&initialized.size()==4;
        std::cout<<"shared physical pole seed="<<ready<<" error="<<why<<'\n';
        struct Pole:Plane {
            Point3D facePoint(int,Point2D p) const override {return {p.x*(1-p.y)+.5*p.y,p.y,0};}
            void faceDerivatives(int,Point2D p,Point3D& u,Point3D& v) const override {u={1-p.y,0,0};v={.5-p.x,1,0};}
        } geometry;
        if(ready) {
            int seeds=-1,bootstrapWork=0;
            const bool untouched=seedSurfaceReferenceChart(pole,mapping,physical,state,boundary,geometry,0,SurfaceCriteria{},
                [&](Point3D p){int id=int(physical.size());physical.push_back(p);return id;},bootstrapWork,seeds,&why);
            ok=ok&&untouched&&seeds==0&&bootstrapWork==0&&pole.points.size()==4;
            int steps=0;SurfaceReferenceStatistics result;
            const bool done=generateSurfaceReferenceFront(pole,mapping,physical,state,boundary,geometry,0,SurfaceCriteria{},initialized,
                [](Point2D,Point3D){return 1.;},[&](Point3D p){int id=int(physical.size());physical.push_back(p);return id;},steps,result,&why);
            ok=ok&&done&&result.collapsedPoleTriangles==1&&result.inserted==0&&state.verify(pole,boundary,&why)&&pole.validate(&why);
        }
        // 同坐标但不同物理身份不能凭容差推断成极点；失败须保留已有尺寸输出。
        mapping[3]=3;physical.push_back(physical[2]);const auto previous=initialized;
        const bool distinct=initializeSurfaceReferenceSizes(pole,mapping,physical,state,boundary,initialized,&why);
        ok=ok&&!distinct&&why.find("degenerate reference seed edge")!=std::string::npos&&initialized.size()==previous.size();
        for(size_t k=0;k<initialized.size();++k)ok=ok&&initialized[k].mesh==previous[k].mesh&&initialized[k].background==previous[k].background;
    }
    {
        // 均值后组合尺寸来源；不可先取每端min再平均。
        const std::array<Point2D,3> uv={Point2D{2,7},Point2D{0,0},Point2D{8,0}};
        const std::array<Point3D,3> p={Point3D{2,7,0},Point3D{0,0,0},Point3D{8,0,0}};
        const std::array<SurfaceReferenceSize,3> sources={SurfaceReferenceSize{2,.4},SurfaceReferenceSize{1,1},SurfaceReferenceSize{.4,2}};
        SurfaceReferenceCandidate candidate;std::string why;
        ok=ok&&surfaceReferenceCandidate(plane,0,uv,p,sources,2,{},candidate,&why)&&std::abs(candidate.target-1.2)<1e-12;
        auto noCorrection=SurfaceReferenceOptions{};noCorrection.correctOnSurface=false;
        ok=ok&&surfaceReferenceCandidate(plane,0,uv,p,sources,2,noCorrection,candidate,&why)&&!candidate.circleCorrected&&!candidate.circleFallback;
    }
    {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());std::string why;
        std::vector<SurfaceReferenceSize> initialized;
        ok=ok&&initializeSurfaceReferenceSizes(f.mesh,f.global,f.xyz,f.domain,f.constraints,initialized,&why,
            [](Point2D,Point3D){return .75;});
        for(size_t k=0;k<initialized.size();++k)if(f.global[k]>=0)ok=ok&&std::abs(initialized[k].mesh-.75)<1e-12&&initialized[k].background==initialized[k].mesh;
        const auto previous=initialized;
        const bool done=initializeSurfaceReferenceSizes(f.mesh,f.global,f.xyz,f.domain,f.constraints,initialized,&why,
            [](Point2D,Point3D){return std::numeric_limits<double>::quiet_NaN();});
        ok=ok&&!done&&why.find("invalid reference seed size limit")!=std::string::npos&&initialized.size()==previous.size();
        for(size_t k=0;k<initialized.size();++k)ok=ok&&initialized[k].mesh==previous[k].mesh&&initialized[k].background==previous[k].background;
    }
    {
        // 同一几何域仅把底边分成两段：质量分点不应成为更小的几何尺寸源。
        Mesh2D original;for(Point2D p:{Point2D{0,0},Point2D{2,0},Point2D{0,2}})original.addPoint(p);
        original.addTri(0,1,2);const std::vector<Point3D> positions{{0,0,0},{2,0,0},{0,2,0},{1,0,0}};
        const std::vector<std::pair<int,int>> boundary{{0,1},{1,2},{2,0}};
        SurfaceDomainState oldDomain;std::string why;std::vector<SurfaceReferenceSize> before;
        bool stable=oldDomain.initialize(original,boundary,&why)&&initializeSurfaceReferenceSizes(original,{0,1,2},positions,
            oldDomain,boundary,before,&why);
        Mesh2D refined=original;refined.addPoint({1,0});refined.removeTri(0);refined.addTri(0,3,2);refined.addTri(3,1,2);
        const std::vector<std::pair<int,int>> splitBoundary{{0,3},{3,1},{1,2},{2,0}};
        SurfaceDomainState newDomain;std::vector<SurfaceReferenceSize> raw,preserved;
        stable=newDomain.initialize(refined,splitBoundary,&why)&&stable;
        stable=initializeSurfaceReferenceSizes(refined,{0,1,2,3},positions,newDomain,splitBoundary,raw,&why)&&stable;
        stable=stable&&raw[0].mesh<before[0].mesh;
        const std::vector<std::pair<int,double>> frozen{{0,before[0].mesh},{1,before[1].mesh},{2,before[2].mesh},{3,2}};
        stable=initializeSurfaceReferenceSizes(refined,{0,1,2,3},positions,newDomain,splitBoundary,preserved,&why,{},frozen)&&stable;
        for(int k=0;k<3;++k)stable=stable&&preserved[k].mesh==before[k].mesh&&preserved[k].background==before[k].background;
        stable=stable&&preserved[3].mesh==2;
        const auto saved=preserved;
        const bool bad=initializeSurfaceReferenceSizes(refined,{0,1,2,3},positions,newDomain,splitBoundary,preserved,&why,{},
            {{3,std::numeric_limits<double>::quiet_NaN()}});
        stable=stable&&!bad&&preserved.size()==saved.size();
        for(size_t k=0;k<saved.size();++k)stable=stable&&preserved[k].mesh==saved[k].mesh&&preserved[k].background==saved[k].background;
        std::cout<<"quality boundary source stable="<<stable<<" raw="<<raw[0].mesh<<" original="<<before[0].mesh<<'\n';
        ok=ok&&stable;
    }
    for(int shape:{0,1,2})for(bool hole:{false,true}) {
        referencefixture::AnalyticSurface geometry(shape);referencefixture::Fixture f;f.rectangle(geometry,hole);
        SurfaceReferenceStatistics result;std::string why;
        SurfaceReferenceOptions options;options.physicalCavity=shape==2;options.physicalPointHeight=shape==2;
        const bool done=f.run(geometry,result,why,{}, {},options);double covered=0;int interior=0,near=0;
        for(int k=0;k<int(f.mesh.tris.size());++k)if(f.domain.values()[k]) {
            const auto t=f.mesh.tris[k];const auto a=f.xyz[f.global[t.v[0]]],b=f.xyz[f.global[t.v[1]]],c=f.xyz[f.global[t.v[2]]];
            covered+=std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2;
            const double x=(a.x+b.x+c.x)/3,y=(a.y+b.y+c.y)/3;
            if(hole&&x>6&&x<10&&y>4&&y<8)ok=false;
            if(std::min({a.x,b.x,c.x})<2||std::max({a.x,b.x,c.x})>14||std::min({a.y,b.y,c.y})<2||std::max({a.y,b.y,c.y})>10)continue;
            if(hole){const auto distance=[](Point3D p){return std::hypot(std::max({6-p.x,0.,p.x-10}),std::max({4-p.y,0.,p.y-8}));};if(std::min({distance(a),distance(b),distance(c)})<2)continue;}
            const auto length=[](Point3D p,Point3D q){return std::hypot(p.x-q.x,p.y-q.y,p.z-q.z);};
            const double sides[]={length(a,b),length(b,c),length(c,a)};bool good=true;
            for(int i=0;i<3;++i){const double s=sides[i],u=sides[(i+1)%3],v=sides[(i+2)%3];const double angle=std::acos(std::clamp((s*s+u*u-v*v)/(2*s*u),-1.,1.))*180/std::acos(-1.);good=good&&angle>=50&&angle<=70;}
            ++interior;near+=good;
        }
        const double fraction=interior?double(near)/interior:0;
        ok=ok&&done&&result.inserted>0&&f.mesh.validate(&why)&&f.domain.verify(f.mesh,f.constraints,&why)&&
            std::abs(covered-(hole?176:192))<1e-6&&interior>0&&fraction>=.7;
        std::cout<<"plane="<<shape<<" hole="<<hole<<" area="<<covered<<" interior="<<interior<<" near="<<fraction<<" inserted="<<result.inserted<<" rejected="<<result.rejected<<" remaining="<<result.remainingLargeTriangles<<" error="<<why<<'\n';
    }
    for(bool budget:{false,true}) {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());SurfaceReferenceStatistics result;std::string why;
        SurfaceCriteria limits;if(budget)limits.maxIterations=1;
        const bool done=f.run(Plane(),result,why,limits,budget?MeshingProgress{}:MeshingProgress{[&](int,int){return result.attempted==0;}});
        ok=ok&&!done&&why.find(budget?"budget":"cancelled")!=std::string::npos&&result.inserted==1&&f.mesh.validate(&why)&&f.domain.verify(f.mesh,f.constraints,&why);
        std::cout<<"termination budget="<<budget<<" inserted="<<result.inserted<<" error="<<why<<'\n';
    }
    for(bool invalid:{false,true}) {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());SurfaceReferenceStatistics result;std::string why;
        const auto before=f.xyz.size();
        const bool done=f.run(Plane(),result,why,{}, {},{},[&](Point2D,Point3D){return invalid?std::numeric_limits<double>::quiet_NaN():1.05;});
        if(invalid)ok=ok&&!done&&why=="invalid reference maximum edge limit"&&result.inserted==0&&f.xyz.size()==before;
        else {
            // 若超长片未能插入，必须保留未满足计数，不能静默宣称全域上限成立。
            int longTriangles=0;
            for(int slot=0;slot<int(f.mesh.tris.size());++slot)if(f.domain.values()[slot]) {
                const auto tri=f.mesh.tris[slot];bool longEdge=false;
                for(int e=0;e<3;++e){const auto a=f.xyz[f.global[tri.v[e]]],b=f.xyz[f.global[tri.v[(e+1)%3]]];longEdge|=std::hypot(a.x-b.x,a.y-b.y,a.z-b.z)>1.05;}
                longTriangles+=longEdge;
            }
            ok=ok&&done&&result.inserted>202&&result.remainingLargeTriangles>=longTriangles&&f.mesh.validate(&why)&&f.domain.verify(f.mesh,f.constraints,&why);
            std::cout<<"maximum edge front inserted="<<result.inserted<<" long="<<longTriangles<<" unsettled="<<result.remainingLargeTriangles<<'\n';
        }
    }
    struct BadNormal:Plane {Point3D faceNormal(int,Point2D) const override {return {0,0,std::numeric_limits<double>::quiet_NaN()};}};
    {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());SurfaceReferenceStatistics result;std::string why;
        const auto count=f.xyz.size();const bool done=f.run(BadNormal(),result,why);
        ok=ok&&!done&&why.find("non-finite")!=std::string::npos&&f.xyz.size()==count&&result.attempted==0&&f.mesh.validate(&why);
        std::cout<<"nonfinite normal success="<<done<<" inserted="<<result.inserted<<" error="<<why<<'\n';
    }
    struct BadDerivative:Plane {void faceDerivatives(int,Point2D,Point3D& u,Point3D& v) const override {u={std::numeric_limits<double>::quiet_NaN(),0,0};v={0,1,0};}};
    {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());SurfaceReferenceStatistics result;std::string why;
        const auto count=f.xyz.size();const bool done=f.run(BadDerivative(),result,why);
        ok=ok&&!done&&why.find("non-finite")!=std::string::npos&&f.xyz.size()==count&&result.attempted==0&&f.mesh.validate(&why);
    }
    {
        referencefixture::Fixture f;f.rectangle(referencefixture::AnalyticSurface());SurfaceReferenceStatistics result;std::string why;
        const auto count=f.xyz.size();int work=0;
        const bool done=generateSurfaceReferenceFront(f.mesh,f.global,f.xyz,f.domain,f.constraints,plane,0,SurfaceCriteria{},f.sizes,
            [](Point2D,Point3D){return std::numeric_limits<double>::infinity();},
            [&](Point3D p){const int v=int(f.xyz.size());f.xyz.push_back(p);return v;},work,result,&why);
        ok=ok&&!done&&why=="invalid reference candidate size"&&f.xyz.size()==count&&f.mesh.validate(&why)&&f.domain.verify(f.mesh,f.constraints,&why);
    }
    if(!ok)std::cerr<<"FAIL reference shape, coverage or termination guards\n";
    return ok?0:1;
}
