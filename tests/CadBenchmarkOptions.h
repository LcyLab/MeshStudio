#pragma once
// Test-tool options only; no UI or production parameter defaults are changed.
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace cadbench {
struct Options {
    std::string cad,output;
    mesher::SurfaceCriteria criteria;
    double timeout=300;
    bool observe=false;
    int snapshotFace=-1; // 手工诊断：保存已验收面的完整连接、坐标和逐角 UV。
};
inline double number(const std::string& text) {
    try {
        size_t used=0; const double value=std::stod(text,&used);
        if (used!=text.size() || !std::isfinite(value)) throw std::invalid_argument("invalid finite number");
        return value;
    } catch (const std::exception&) { throw std::invalid_argument("invalid finite number: "+text); }
}
inline int positiveInteger(const std::string& text) {
    if (text.empty() || text.find_first_not_of("0123456789")!=std::string::npos)
        throw std::invalid_argument("invalid positive integer: "+text);
    const double value=number(text);
    if (value<1 || value>std::numeric_limits<int>::max()) throw std::invalid_argument("integer out of range: "+text);
    return static_cast<int>(value);
}
inline Options parse(const std::vector<std::string>& args,bool audit=false) {
    Options o;
    if (audit && args.size()==1 && args.front().rfind("--",0)!=0) {
        o.cad=args.front(); o.criteria.targetSize=20; o.criteria.maxDeviation=2;
        o.criteria.maxIterations=40000; o.timeout=30; return o;
    }
    std::set<std::string> seen;
    for (size_t i=0;i<args.size();i+=2) {
        const auto& key=args[i];
        if (i+1>=args.size() || !seen.insert(key).second) throw std::invalid_argument("missing or duplicate option: "+key);
        const auto& value=args[i+1];
        if (key=="--cad") o.cad=value;
        else if(key=="--placement") {
            if(value=="legacy") o.criteria.pointPlacement=mesher::SurfacePointPlacement::LegacyRefinement;
            else if(value=="frontal") o.criteria.pointPlacement=mesher::SurfacePointPlacement::FrontalDelaunay;
            else if(value=="reference") o.criteria.pointPlacement=mesher::SurfacePointPlacement::ReferenceFrontal;
            else if(value=="reference-physical") o.criteria.pointPlacement=mesher::SurfacePointPlacement::PhysicalReferenceFrontal;
            else throw std::invalid_argument("--placement requires legacy, frontal, reference or reference-physical");
        }
        else if(key=="--cross-smooth") {
            if(value!="on"&&value!="off")throw std::invalid_argument("--cross-smooth requires on or off");
            o.criteria.crossSmoothCadSplits=value=="on";
        }
        else if (key=="--output" && !audit) o.output=value;
        else if (key=="--observe" && !audit) {
            if (value!="on" && value!="off") throw std::invalid_argument("--observe requires on or off");
            o.observe=value=="on";
        } else if(key=="--snapshot-face"&&!audit){
            o.snapshotFace=value=="0"?0:positiveInteger(value);
        } else if(key=="--shape-sweeps"&&!audit){
            const double n=number(value);if(n<0||n>16||n!=std::floor(n))throw std::invalid_argument("shape sweeps must be an integer in [0,16]");
            o.criteria.shapeOptimizationSweeps=int(n);
        } else if (key=="--work") o.criteria.maxIterations=positiveInteger(value);
        else if (key=="--vertices") o.criteria.maxVertices=positiveInteger(value);
        else if (key=="--timeout") {
            o.timeout=number(value); if (!(o.timeout>0)) throw std::invalid_argument("--timeout must be positive");
        } else if (key=="--target" || key=="--min" || key=="--max" || key=="--deviation" || key=="--angle") {
            const double n=number(value);
            if (n<=0 || (key=="--angle" && n>180)) throw std::invalid_argument("option out of range: "+key);
            if (key=="--target") o.criteria.targetSize=n;
            else if (key=="--min") o.criteria.minSize=n;
            else if (key=="--max") o.criteria.maxSize=n;
            else if (key=="--deviation") o.criteria.maxDeviation=n;
            else o.criteria.maxNormalAngleDeg=n;
        } else throw std::invalid_argument("unknown option: "+key);
    }
    if (o.cad.empty() || o.cad.rfind("--",0)==0) throw std::invalid_argument("--cad requires a CAD file");
    if (o.criteria.targetSize>0 && o.criteria.minSize>o.criteria.targetSize) throw std::invalid_argument("min exceeds target");
    if (o.criteria.maxSize>0 && o.criteria.minSize>o.criteria.maxSize) throw std::invalid_argument("min exceeds maximum");
    return o;
}
inline void resolve(Options& o,const mesher::GeometrySource& g) {
    auto& c=o.criteria;
    if (!c.targetSize) {
        mesher::Point3D lo,hi;
        if (!g.bounds(lo,hi)) throw std::invalid_argument("CAD bounds unavailable");
        c.targetSize=std::hypot(hi.x-lo.x,hi.y-lo.y,hi.z-lo.z)*0.025;
    }
    if (c.maxSize>0) c.targetSize=std::min(c.targetSize,c.maxSize);
    if (!c.minSize) c.minSize=c.targetSize*0.1;
    if (!c.maxDeviation) c.maxDeviation=c.targetSize*0.1;
    if (!(c.targetSize>0) || !std::isfinite(c.targetSize) || c.minSize>c.targetSize)
        throw std::invalid_argument("invalid resolved geometry size");
}
inline std::vector<std::string> arguments(int argc,char** argv) { return {argv+1,argv+argc}; }
} // namespace cadbench
