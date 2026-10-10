#include <iostream>
#include "mesh/surface/legacy/SurfaceEvaluationCache.h"
#include <limits>
#include <stdexcept>
using namespace mesher;
struct Source : GeometrySource {
    mutable int points=0,normals=0,derivatives=0,uvs=0;
    bool contract=false,reverse=false,singular=false,throws=false;
    int vertexCount() const override { return 0; }
    Point3D vertexPoint(int) const override { return {}; }
    int edgeCount() const override { return 1; }
    std::pair<int,int> edgeVertices(int) const override { return {0,0}; }
    std::pair<double,double> edgeRange(int) const override { return {0,1}; }
    Point3D edgePoint(int,double t) const override { return {t,0,0}; }
    int faceCount() const override { return 2; }
    FaceTopology faceTopology(int) const override { return {}; }
    Point2D edgeUV(int f,const EdgeUse& u,double t) const override { ++uvs; return {t+10*u.branch,double(f)}; }
    Point3D facePoint(int f,Point2D p) const override { ++points; return {p.x+100*f,p.y,0}; }
    Point3D faceNormal(int,Point2D) const override { ++normals; if (throws) throw std::runtime_error("CAD query failed"); return {0,0,7}; }
    void faceDerivatives(int,Point2D,Point3D& u,Point3D& v) const override { ++derivatives; u={1,0,0}; v=singular?Point3D{}:Point3D{0,1,0}; }
    bool faceReversed(int) const override { return reverse; }
    bool normalUsesOrientedDerivatives() const override { return contract; }
};
int main() {
    Source s; SurfaceEvaluationCache c(s,1);
    if (c.point({0,0}).x!=100 || c.point({-0.0,0}).x!=100 || s.points!=1) return 1;
    if (c.normal({0,0}).z!=7 || c.normal({0,0}).z!=7 || s.normals!=1 || s.derivatives) return 1;
    for (int branch:{0,1,0,1}) if (c.edgeUV(1,{0,false,branch},0.5).x!=0.5+10*branch) return 1;
    if (s.uvs!=2) return 1;
    SurfaceEvaluationCache other(s,0); if (other.point({0,0}).x!=0 || s.points!=2) return 1;
    bool rejected=false; try { c.point({std::numeric_limits<double>::quiet_NaN(),0}); } catch (const std::invalid_argument&) { rejected=true; }
    if (!rejected) return 1;
    s.contract=true; s.reverse=true; SurfaceEvaluationCache oriented(s,0);
    const int before=s.derivatives; if (oriented.normal({1,1}).z!=-1) return 1;
    Point3D u,v; oriented.faceDerivatives(0,{1,1},u,v); if (s.derivatives!=before+1) return 1;
    s.singular=true; SurfaceEvaluationCache pole(s,0); if (pole.normal({0,0}).z!=0) return 1;
    s.contract=false; s.throws=true; SurfaceEvaluationCache failed(s,0);
    bool caught=false; try { failed.normal({0,0}); } catch (const std::runtime_error& e) { caught=std::string(e.what())=="CAD query failed"; }
    s.throws=false; if (!caught || failed.normal({0,0}).z!=7) return 1;
    std::cout<<"PASS exact UV cache keys, face/branch separation, optional derivative contract and CAD errors\n";
}
