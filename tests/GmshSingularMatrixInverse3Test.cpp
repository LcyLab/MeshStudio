#include "geometry/numerics/SingularMatrixInverse3.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher::gmshReference;
int main() {
    int failures = 0;
    auto check = [&](bool ok,const char* name) { if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    auto close = [](double a,double b,double tolerance=1.e-12) { return std::abs(a-b) <= tolerance; };
    Matrix3 matrix{{{{1.,0.,0.}},{{0.,1.,0.}},{{0.,0.,1.}}}};
    auto r = computeSingularMatrixPseudoInverse(matrix);
    check(r.finiteInput && r.converged && r.finiteResult && r.retainedSingularValues == 3 && r.inverse == matrix,"identity full rank");
    r = computeSingularMatrixPseudoInverse({});
    check(r.finiteResult && r.converged && r.retainedSingularValues == 0 && r.inverse == Matrix3{},"zero matrix has finite zero pseudoinverse");
    matrix[1][1] = 1.e-16; matrix[2][2] = std::nextafter(1.e-16,1.);
    r = computeSingularMatrixPseudoInverse(matrix);
    check(r.finiteResult && r.retainedSingularValues == 2 && r.inverse[1][1] == 0. && r.inverse[2][2] == 1./matrix[2][2],"strict absolute singular cutoff");
    matrix = {{{{1.,2.,3.}},{{0.,1.,4.}},{{5.,6.,0.}}}};
    r = computeSingularMatrixPseudoInverse(matrix);
    Matrix3 expected{{{{-24.,18.,5.}},{{20.,-15.,-4.}},{{-5.,4.,1.}}}};
    bool dense = r.finiteResult && r.retainedSingularValues == 3;
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) dense = dense && close(r.inverse[i][j],expected[i][j],2.e-11);
    check(dense,"dense nonsymmetric inverse");
    matrix = {{{{3.,0.,0.}},{{4.,0.,0.}},{{0.,0.,0.}}}};
    r = computeSingularMatrixPseudoInverse(matrix);
    // 原SVD的旋转舍入在理论零项留下约1e-17；原生逐值矩阵对照另行验证。
    check(r.finiteResult && r.retainedSingularValues == 1 && close(r.inverse[0][0],.12) && close(r.inverse[0][1],.16) && close(r.inverse[1][0],0.) && close(r.inverse[1][1],0.) && r.inverse[1][2] == 0. && r.inverse[2] == Vector3{},"rank one minimum norm inverse");
    matrix = {{{{1.,0.,0.}},{{1.,1.,0.}},{{0.,0.,0.}}}};
    r = computeSingularMatrixPseudoInverse(matrix);
    check(r.finiteResult && r.retainedSingularValues == 2 && close(r.inverse[0][0],1.) && close(r.inverse[0][1],0.) && close(r.inverse[1][0],-1.) && close(r.inverse[1][1],1.) && r.inverse[2] == Vector3{},"tangent matrix zero third row uses least norm inverse");
    matrix = {{{{1.e100,0.,0.}},{{0.,-2.e100,0.}},{{0.,0.,3.e100}}}};
    r = computeSingularMatrixPseudoInverse(matrix);
    check(r.finiteResult && r.retainedSingularValues == 3 && close(r.inverse[0][0]/1.e-100,1.) && close(r.inverse[1][1]/1.e-100,-.5) && close(r.inverse[2][2]/1.e-100,1./3.),"large matrix scaled without overflow");
    matrix[0][0] = std::numeric_limits<double>::quiet_NaN();
    check(!computeSingularMatrixPseudoInverse(matrix).finiteInput,"NaN input explicitly rejected");
    matrix[0][0] = std::numeric_limits<double>::infinity();
    check(!computeSingularMatrixPseudoInverse(matrix).finiteInput,"infinite input explicitly rejected");
    std::cout << "singular inverse failures=" << failures << '\n'; return failures ? 1 : 0;
}
