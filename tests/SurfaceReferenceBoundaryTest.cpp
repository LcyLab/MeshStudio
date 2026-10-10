#include "mesh/surface/legacy/SurfaceReferenceBoundary.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher;
int main() {
    bool ok=true;std::string error;std::vector<double> parameters;
    const auto constant=[](double){return 1.;};
    const bool nonlinear=sampleReferenceCurve(0,1,[](double t){return Point3D{50*t*t*t*t,0,0};},constant,parameters,&error);
    ok=ok&&nonlinear&&parameters.size()==11;
    double worst=0;
    for(size_t k=0;k<parameters.size();++k)worst=std::max(worst,std::abs(10*std::pow(parameters[k],5)-double(k)));
    ok=ok&&worst<2e-5;std::cout<<"nonlinear curve equal metric error="<<worst<<'\n';
    const bool variable=sampleReferenceCurve(0,1,[](double){return Point3D{10,0,0};},[](double t){return 1+10*t;},parameters,&error);
    ok=ok&&variable;
    const double total=std::log(11.),count=double(parameters.size()-1);
    worst=0;for(size_t k=0;k<parameters.size();++k)worst=std::max(worst,std::abs(std::log(1+10*parameters[k])-total*k/count));
    ok=ok&&worst<2e-5;std::cout<<"variable size equal metric error="<<worst<<'\n';
    const auto before=parameters;
    ok=ok&&!sampleReferenceCurve(0,1,[](double){return Point3D{1,0,0};},[](double){return std::numeric_limits<double>::quiet_NaN();},parameters,&error)&&parameters==before;
    ReferenceCurveSamplingOptions limited;limited.maxSegments=2;
    ok=ok&&!sampleReferenceCurve(0,1,[](double){return Point3D{10,0,0};},constant,parameters,&error,limited)&&error.find("segment budget")!=std::string::npos&&parameters==before;
    ok=ok&&!sampleReferenceCurve(0,1,[](double){return Point3D{10,0,0};},constant,parameters,&error,{},[]{return false;})&&error.find("cancelled")!=std::string::npos&&parameters==before;
    if(!ok)std::cerr<<"FAIL reference curve metric/inverse/guards "<<error<<'\n';return ok?0:1;
}
