#include "geometry/numerics/LinearSystem3.h"
#include <cmath>
#include <iostream>
#include <limits>
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool condition, const char* name) { if(!condition) { ++failures; std::cerr << "FAIL " << name << '\n'; } };
    auto close = [](double a, double b) { return std::abs(a - b) < 1.e-12; };
    Matrix3 matrix{{{{1., 0., 0.}}, {{0., 2., 0.}}, {{0., 0., 3.}}}};
    auto result = solveColumnPivotedQr(matrix, {2., 6., 12.});
    check(result.finiteInput && result.finiteResult && result.meaningfulPivots == 3 &&
          close(result.solution[0], 2.) && close(result.solution[1], 3.) && close(result.solution[2], 4.),
          "column permutation restored to original variables");
    matrix = {{{{1., 2., 3.}}, {{4., 1., 2.}}, {{2., 3., 1.}}}};
    result = solveColumnPivotedQr(matrix, {6., 8., -1.});
    check(result.finiteResult && close(result.solution[0], 1.) && close(result.solution[1], -2.) && close(result.solution[2], 3.),
          "dense nonsymmetric solve");
    matrix = {{{{1., 0., 0.}}, {{0., 1., 0.}}, {{0., 0., 0.}}}};
    result = solveColumnPivotedQr(matrix, {2., 3., 9.});
    check(result.finiteResult && result.meaningfulPivots == 2 && result.solution == Vector3{2., 3., 0.},
          "native rank deficient solve retains approximate result without residual gate");
    matrix[2][2] = 1.e-18;
    result = solveColumnPivotedQr(matrix, {2., 3., 9.});
    check(result.finiteResult && result.meaningfulPivots == 2 && result.solution[2] == 0., "meaningful pivot threshold");
    matrix = {};
    result = solveColumnPivotedQr(matrix, {1., 2., 3.});
    check(result.finiteInput && !result.finiteResult, "zero matrix native strict pivot threshold yields nonfinite solve");
    result = solveColumnPivotedQr(matrix, {0., 0., 0.});
    check(result.finiteResult && result.solution == Vector3{}, "native triangular solve skips exact zero RHS");
    matrix[0][0] = std::numeric_limits<double>::quiet_NaN();
    check(!solveColumnPivotedQr(matrix, {1., 2., 3.}).finiteInput, "nonfinite input visible");
    std::cout << "linear system failures=" << failures << '\n';
    return failures ? 1 : 0;
}
