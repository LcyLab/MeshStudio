#pragma once
/** @file LinearSystem3.h
 * @brief 固定三阶列主元 Householder QR，供曲面求交 Newton 使用。
 * @date 2026-10-07
 */
#include <array>

namespace mesher::gmshReference {
using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;
struct LinearSolve3Result {
    Vector3 solution{};
    int meaningfulPivots = 0;
    bool finiteInput = false;
    bool finiteResult = false;
};

/** 对应 Gmsh fullMatrix::luSolve 的 HAVE_EIGEN 实际分支。
 * 只读矩阵/RHS，列置换、Householder、原生范数降阶和 meaningful pivots；
 * 秩亏时未选变量置零，不按方阵 LU 的“不可逆”直接拒绝。
 * 不验证残差或添加正则化。非有限值和运算溢出通过状态明确暴露。
 */
LinearSolve3Result solveColumnPivotedQr(const Matrix3& matrix, const Vector3& rhs);
} // namespace mesher::gmshReference
