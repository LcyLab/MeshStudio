#pragma once
/** @file SingularMatrixInverse3.h
 * @brief 原三阶双边Jacobi SVD及绝对阈值伪逆；不处理几何或Newton。
 * @date 2026-10-08
 */
#include "LinearSystem3.h"

namespace mesher::gmshReference {
struct SingularMatrixInverse3Result {
    Matrix3 inverse{};
    Matrix3 leftVectors{},rightVectors{};
    Vector3 singularValues{};
    int retainedSingularValues = 0;
    int sweeps = 0;
    bool finiteInput = false;
    bool converged = false;
    bool finiteResult = false;
};
/** 固定Gmsh HAVE_EIGEN三阶SVD实际Jacobi路径；只读输入。
 * 对应invert_singular_matrix3x3，保留abs(w)>1e-16及原乘加顺序。
 * 不以相对秩阈值、LU/QR、正则化或残差拒绝替代。
 * 非有限输入/计算失败通过显式状态返回，不输出假成功的逆矩阵。
 */
SingularMatrixInverse3Result computeSingularMatrixPseudoInverse(const Matrix3& matrix);
}
