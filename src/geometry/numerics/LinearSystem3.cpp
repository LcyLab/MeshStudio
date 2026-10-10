// 文件职责：三阶线性求解：用列主元 Householder QR 求解 Newton 步长，保留秩亏及非有限值的诊断。
#include "LinearSystem3.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher::gmshReference {
namespace {
/// 计算矩阵指定列从当前行开始的欧氏范数。
double columnNorm(const Matrix3& matrix, int column, int firstRow)
{
    double square = 0.;
    for(int row = firstRow; row < 3; ++row) square += matrix[row][column] * matrix[row][column];
    return std::sqrt(square);
}

/// 应用 Householder 反射，更新矩阵和右端项。
void applyReflector(Vector3& vector, const Matrix3& qr, int column, double tau)
{
    if(tau == 0.) return;
    double tail = 0.;
    for(int row = column + 1; row < 3; ++row) tail += qr[row][column] * vector[row];
    const double projected = tail + vector[column];
    vector[column] -= tau * projected;
    for(int row = column + 1; row < 3; ++row) vector[row] -= tau * qr[row][column] * projected;
}
} // namespace

/// 使用列主元 QR 分解求解三元线性方程组。
LinearSolve3Result solveColumnPivotedQr(const Matrix3& matrix, const Vector3& rhs)
{
    LinearSolve3Result result;
    for(int i = 0; i < 3; ++i) {
        if(!std::isfinite(rhs[i])) return result;
        for(double entry : matrix[i]) if(!std::isfinite(entry)) return result;
    }
    result.finiteInput = true;
    Matrix3 qr = matrix;
    Vector3 directNorm{}, updatedNorm{}, reflectors{};
    std::array<int, 3> permutation{0, 1, 2};
    for(int column = 0; column < 3; ++column)
        directNorm[column] = updatedNorm[column] = columnNorm(qr, column, 0);
    const double epsilon = std::numeric_limits<double>::epsilon();
    const double scaledMaximum = *std::max_element(updatedNorm.begin(), updatedNorm.end()) * epsilon;
    const double pivotThreshold = scaledMaximum * scaledMaximum / 3.;
    const double normRefreshThreshold = std::sqrt(epsilon);
    result.meaningfulPivots = 3;
    for(int column = 0; column < 3; ++column) {
        int pivot = column;
        for(int candidate = column + 1; candidate < 3; ++candidate)
            if(updatedNorm[candidate] > updatedNorm[pivot]) pivot = candidate;
        const double largestSquare = updatedNorm[pivot] * updatedNorm[pivot];
        // 严格小于保留全零矩阵的原生边界行为；不另加 LU 秩判据。
        if(result.meaningfulPivots == 3 && largestSquare < pivotThreshold * double(3 - column))
            result.meaningfulPivots = column;
        if(pivot != column) {
            for(int row = 0; row < 3; ++row) std::swap(qr[row][pivot], qr[row][column]);
            std::swap(directNorm[pivot], directNorm[column]);
            std::swap(updatedNorm[pivot], updatedNorm[column]);
            std::swap(permutation[pivot], permutation[column]);
        }
        double tailSquare = 0.;
        for(int row = column + 1; row < 3; ++row) tailSquare += qr[row][column] * qr[row][column];
        const double leading = qr[column][column];
        double diagonal = leading;
        if(tailSquare <= std::numeric_limits<double>::min()) {
            reflectors[column] = 0.;
            for(int row = column + 1; row < 3; ++row) qr[row][column] = 0.;
        }
        else {
            diagonal = std::sqrt(leading * leading + tailSquare);
            if(leading >= 0.) diagonal = -diagonal;
            for(int row = column + 1; row < 3; ++row) qr[row][column] /= leading - diagonal;
            reflectors[column] = (diagonal - leading) / diagonal;
        }
        qr[column][column] = diagonal;
        for(int next = column + 1; next < 3; ++next) {
            Vector3 values{qr[0][next], qr[1][next], qr[2][next]};
            applyReflector(values, qr, column, reflectors[column]);
            for(int row = column; row < 3; ++row) qr[row][next] = values[row];
            if(updatedNorm[next] == 0.) continue;
            const double fraction = std::abs(qr[column][next]) / updatedNorm[next];
            const double remainder = std::max(0., (1. + fraction) * (1. - fraction));
            const double ratio = updatedNorm[next] / directNorm[next];
            if(remainder * (ratio * ratio) <= normRefreshThreshold)
                directNorm[next] = updatedNorm[next] = columnNorm(qr, next, column + 1);
            else updatedNorm[next] *= std::sqrt(remainder);
        }
    }
    Vector3 transformed = rhs;
    for(int column = 0; column < result.meaningfulPivots; ++column)
        applyReflector(transformed, qr, column, reflectors[column]);
    // 上三角逐列回代；未选列为零，恢复原来的变量身份。
    for(int row = result.meaningfulPivots - 1; row >= 0; --row) {
        if(transformed[row] == 0.) continue;
        transformed[row] /= qr[row][row];
        for(int previous = 0; previous < row; ++previous)
            transformed[previous] -= transformed[row] * qr[previous][row];
    }
    result.finiteResult = true;
    for(int row = 0; row < result.meaningfulPivots; ++row) {
        result.solution[permutation[row]] = transformed[row];
        result.finiteResult &= std::isfinite(transformed[row]);
    }
    return result;
}
} // namespace mesher::gmshReference
