#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

using namespace mesher;
namespace {
int failures=0;
void check(bool ok,const char* name) {
    if (!ok) { ++failures; std::cerr<<"FAIL "<<name<<'\n'; }
}
int parity(const int* order,int n) {
    int result=1;
    for (int i=0;i<n;++i) for (int j=i+1;j<n;++j)
        if (order[i]>order[j]) result=-result;
    return result;
}
// Independent Leibniz determinant oracle. Entries below are small integers, so
// every term and sum fit int64_t; unlike the production path it does not use FMA.
template<size_t N> int integerDetSign(const std::array<std::array<int64_t,N>,N>& matrix) {
    std::array<int,N> order{};
    for (size_t i=0;i<N;++i) order[i]=static_cast<int>(i);
    int64_t determinant=0;
    do {
        int64_t product=parity(order.data(),static_cast<int>(N));
        for (size_t i=0;i<N;++i) product*=matrix[i][order[i]];
        determinant+=product;
    } while (std::next_permutation(order.begin(),order.end()));
    return (determinant>0)-(determinant<0);
}
void orientation() {
    const double n=134217728;
    // (n*n)-(n-1)*(n+1)=1 exactly, but both rounded products are equal.
    const std::array<Point3D,4> points{{{0,0,0},{n,n-1,0},{n+1,n,0},{0,0,1}}};
    std::array<int,4> order{{0,1,2,3}};
    do {
        const auto a=points[order[0]],b=points[order[1]],c=points[order[2]],d=points[order[3]];
        check(orient3d(a,b,c,d)==parity(order.data(),4),"cancellation retains exact orient3d sign");
        check(orient3dSoS(a,b,c,d,order[0],order[1],order[2],order[3])==parity(order.data(),4),
              "nonzero cancellation never enters orientation SoS");
    } while (std::next_permutation(order.begin(),order.end()));
    // dyadic inputs have an independently known positive determinant 2^-50.
    check(orient3dSoS({0,0,0},{1,0,0},{0,1,0},{0,0,std::ldexp(1.0,-50)},0,1,2,3)==1,
          "positive near-coplanar determinant is not a tie");
}
void spheres() {
    const std::array<Point3D,5> points{{{1,0,0},{0,1,0},{0,0,1},{-1,0,0},{0,-1,0}}};
    const int base=insphereSoS(points[0],points[1],points[2],points[3],points[4],0,1,2,3,4);
    std::array<int,5> order{{0,1,2,3,4}};
    do {
        const auto a=points[order[0]],b=points[order[1]],c=points[order[2]],d=points[order[3]],p=points[order[4]];
        check(insphere(a,b,c,d,p)==0,"exact cosphere remains zero");
        check(insphereSoS(a,b,c,d,p,order[0],order[1],order[2],order[3],order[4])==base*parity(order.data(),5),
              "cosphere symbolic tie is alternating for all 120 permutations");
    } while (std::next_permutation(order.begin(),order.end()));
    for (int perturbation:{-1,1}) {
        Point3D p{0,-1,perturbation*std::ldexp(1.0,-30)};
        // Radius squared is 1+2^-60, so either sign of z is strictly outside.
        check(insphere(points[0],points[1],points[2],points[3],p)==-1,"near-cosphere exact outside");
        // These four points have orient3d determinant -2. The insertion SoS
        // returns -raw=orientation*normalized sign, hence (+1) for this outside p.
        check(orient3d(points[0],points[1],points[2],points[3])==-1,"near-cosphere fixture orientation");
        check(insphereSoS(points[0],points[1],points[2],points[3],p,0,1,2,3,4)==1,
              "near-cosphere nonzero does not enter SoS");
        check(insphereSoS(points[1],points[0],points[2],points[3],p,1,0,2,3,4)==-1,
              "near-cosphere insertion sign reverses with orientation");
    }
    for (int x=-2;x<=2;++x) for (int y=-2;y<=2;++y) for (int z=-2;z<=2;++z) {
        const Point3D p{double(x),double(y),double(z)};
        std::array<std::array<int64_t,5>,5> m{{{{0,0,0,0,1}},{{1,0,0,1,1}},{{0,1,0,1,1}},
            {{0,0,1,1,1}},{{x,y,z,x*x+y*y+z*z,1}}}};
        const int expected=-integerDetSign(m);
        check(insphere({0,0,0},{1,0,0},{0,1,0},{0,0,1},p)==expected,"integer insphere oracle");
        check(insphere({0,0,0},{0,1,0},{1,0,0},{0,0,1},p)==expected,"orientation-normalized insphere oracle");
    }
}
void lowRank() {
    const Point3D a{4,4,4};
    std::array<int,4> order{{0,1,2,3}};
    const int base=orient3dSoS(a,a,a,a,0,1,2,3);
    do {
        check(orient3d(a,a,a,a)==0,"coincident orientation exactly zero");
        check(orient3dSoS(a,a,a,a,order[0],order[1],order[2],order[3])==base*parity(order.data(),4),
              "fully degenerate orientation tie remains alternating");
    } while (std::next_permutation(order.begin(),order.end()));
    std::array<int,5> sphereOrder{{0,1,2,3,4}};
    const int sphereBase=insphereSoS(a,a,a,a,a,0,1,2,3,4);
    do {
        check(insphereSoS(a,a,a,a,a,sphereOrder[0],sphereOrder[1],sphereOrder[2],sphereOrder[3],sphereOrder[4])==
              sphereBase*parity(sphereOrder.data(),5),"fully degenerate sphere tie remains alternating");
    } while (std::next_permutation(sphereOrder.begin(),sphereOrder.end()));
    // All xyz orient cofactors vanish. The first coordinate term is z of ID 0;
    // its raw coefficient is det[x y lift 1] of the other rows, exactly +4.
    // The insertion convention takes -raw while lifting perturbations are negative.
    const std::array<std::array<int64_t,4>,4> minor{{{{1,0,1,1}},{{0,1,1,1}},
                                                  {{1,1,2,1}},{{2,2,8,1}}}};
    check(integerDetSign(minor)==1,"coplanar symbolic coordinate cofactor oracle");
    check(insphereSoS({0,0,0},{1,0,0},{0,1,0},{1,1,0},{2,2,0},0,1,2,3,4)==-1,
          "low-rank sphere fallback uses the same negative-raw symbolic model");
}
void scalesAndCombinedDegeneracy() {
    const double t=std::ldexp(1.0,-50);
    for (int exponent:{-100,0,100}) {
        const double scale=std::ldexp(1.0,exponent);
        const Point3D a{scale,0,0},b{0,scale,0},c{-scale,0,0},d{0,-scale,t*scale};
        // Exact circumcenter=(0,0,t*scale/2). Radius squared is scale^2*(1+t^2/4).
        // For p=(-scale,0,t*scale/2), the radial deficit is exactly scale^2*t^2/4.
        const Point3D inside{-scale,0,t*scale/2},outside{-scale,0,-t*scale};
        check(orient3d(a,b,c,d)==1,"scaled near-coplanar orientation");
        check(insphere(a,b,c,d,inside)==1,"combined near-coplanar near-cosphere exact inside");
        check(insphere(a,b,c,d,outside)==-1,"combined near-coplanar near-cosphere exact outside");
        check(insphere(b,a,c,d,inside)==1,"combined degeneracy orientation normalization");
        check(insphereSoS(a,b,c,d,inside,0,1,2,3,4)==1 &&
              insphereSoS(b,a,c,d,inside,1,0,2,3,4)==-1,"nonzero sphere SoS preserves raw convention");
    }
    // A large translation must not enter symbolic ties for a nonzero local determinant.
    const double offset=std::ldexp(1.0,40);
    check(orient3dSoS({offset,offset,offset},{offset+1,offset,offset},
        {offset,offset+1,offset},{offset,offset,offset+std::ldexp(1.0,-10)},0,1,2,3)==1,
        "large translation does not turn nonzero orientation into a tie");
}
}
int main() {
    orientation(); spheres(); lowRank(); scalesAndCombinedDegeneracy();
    std::cout<<"GeomPredicates3D: "<<failures<<" failures\n";
    return failures?1:0;
}
