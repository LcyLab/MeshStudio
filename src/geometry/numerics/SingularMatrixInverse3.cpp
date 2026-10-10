// 将 Eigen 实数 Jacobi 数值计算顺序适配为三阶矩阵实现。
// Copyright (C) 2009-2010 Benoit Jacob <jacob.benoit.1@gmail.com>
// Copyright (C) 2009, 2013-2016 Gael Guennebaud <gael.guennebaud@inria.fr>
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. A copy is available at https://mozilla.org/MPL/2.0/.
#include "SingularMatrixInverse3.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher::gmshReference {
namespace {
struct Rotation { double cosine=1.,sine=0.; };
/// 构造三阶单位矩阵。
Matrix3 identity() { return {{{{1.,0.,0.}},{{0.,1.,0.}},{{0.,0.,1.}}}}; }
/// 用平面旋转更新矩阵的两行。
void rotateRows(Matrix3& matrix,int p,int q,Rotation rotation) {
    if(rotation.cosine == 1. && rotation.sine == 0.) return;
    for(int j=0;j<3;++j) {
        const double x=matrix[p][j],y=matrix[q][j];
        matrix[p][j]=rotation.cosine*x+rotation.sine*y;
        matrix[q][j]=-rotation.sine*x+rotation.cosine*y;
    }
}
/// 用平面旋转更新矩阵的两列。
void rotateColumns(Matrix3& matrix,int p,int q,Rotation rotation) {
    if(rotation.cosine == 1. && rotation.sine == 0.) return;
    const double sine=-rotation.sine;
    for(int i=0;i<3;++i) {
        const double x=matrix[i][p],y=matrix[i][q];
        matrix[i][p]=rotation.cosine*x+sine*y;
        matrix[i][q]=-sine*x+rotation.cosine*y;
    }
}
/// 对称二阶子块执行 Jacobi 对角化。
Rotation diagonalizeSymmetricBlock(double x,double y,double z) {
    Rotation result;
    const double denominator=2.*std::abs(y);
    if(denominator < std::numeric_limits<double>::min()) return result;
    const double tau=(x-z)/denominator;
    const double w=std::sqrt(tau*tau+1.);
    const double t=1./(tau > 0. ? tau+w : tau-w);
    const double sign=t > 0. ? 1. : -1.;
    const double normalization=1./std::sqrt(t*t+1.);
    result.sine=-sign*(y/std::abs(y))*std::abs(t)*normalization;
    result.cosine=normalization;
    return result;
}
/// 对二阶子块执行双边旋转，减少非对角项。
std::array<Rotation,2> diagonalizeBlock(const Matrix3& matrix,int p,int q) {
    const double trace=matrix[p][p]+matrix[q][q];
    const double skew=matrix[q][p]-matrix[p][q];
    Rotation first;
    if(std::abs(skew) >= std::numeric_limits<double>::min()) {
        const double ratio=trace/skew;
        const double denominator=std::sqrt(1.+ratio*ratio);
        first={ratio/denominator,1./denominator};
    }
    // 二阶块先左旋成对称块，再求右Jacobi；不更换稳定公式或运算次序。
    const double a=first.cosine*matrix[p][p]+first.sine*matrix[q][p];
    const double b=first.cosine*matrix[p][q]+first.sine*matrix[q][q];
    const double d=-first.sine*matrix[p][q]+first.cosine*matrix[q][q];
    const Rotation right=diagonalizeSymmetricBlock(a,b,d);
    const Rotation left{first.cosine*right.cosine+first.sine*right.sine,
                        -first.cosine*right.sine+first.sine*right.cosine};
    return {left,right};
}
/// 检查数值输入是否全部有限。
bool isFinite(const Matrix3& matrix) {
    for(const auto& row:matrix) for(double value:row) if(!std::isfinite(value)) return false;
    return true;
}
}
/// 使用 Jacobi 奇异值分解计算三阶矩阵的伪逆。
SingularMatrixInverse3Result computeSingularMatrixPseudoInverse(const Matrix3& matrix) {
    SingularMatrixInverse3Result result;
    if(!isFinite(matrix)) return result;
    result.finiteInput=true;
    double scale=0.;
    for(const auto& row:matrix) for(double value:row) scale=std::max(scale,std::abs(value));
    if(scale == 0.) scale=1.;
    Matrix3 work=matrix;
    for(auto& row:work) for(double& value:row) value/=scale;
    result.leftVectors=identity(); result.rightVectors=identity();
    const double precision=2.*std::numeric_limits<double>::epsilon();
    double maximumDiagonal=0.;
    for(int i=0;i<3;++i) maximumDiagonal=std::max(maximumDiagonal,std::abs(work[i][i]));
    // 原有限Jacobi路径以收敛判据终止；1024保护只暴露异常停滞，不接受未收敛逆。
    for(int sweep=0;sweep<1024;++sweep) {
        ++result.sweeps; bool finished=true;
        for(int p=1;p<3;++p) for(int q=0;q<p;++q) {
            const double threshold=std::max(std::numeric_limits<double>::min(),precision*maximumDiagonal);
            if(std::abs(work[p][q]) <= threshold && std::abs(work[q][p]) <= threshold) continue;
            finished=false;
            const auto rotations=diagonalizeBlock(work,p,q);
            rotateRows(work,p,q,rotations[0]);
            rotateColumns(result.leftVectors,p,q,{rotations[0].cosine,-rotations[0].sine});
            rotateColumns(work,p,q,rotations[1]);
            rotateColumns(result.rightVectors,p,q,rotations[1]);
            maximumDiagonal=std::max(maximumDiagonal,std::max(std::abs(work[p][p]),std::abs(work[q][q])));
        }
        if(!isFinite(work) || !isFinite(result.leftVectors) || !isFinite(result.rightVectors)) return result;
        if(finished) { result.converged=true; break; }
    }
    if(!result.converged) return result;
    for(int i=0;i<3;++i) {
        const double diagonal=work[i][i];
        result.singularValues[i]=std::abs(diagonal)*scale;
        if(diagonal < 0.) for(int row=0;row<3;++row) result.leftVectors[row][i]=-result.leftVectors[row][i];
    }
    // 严格大于选首个最大项；重复值保持原剩余序，零尾部不再交换。
    for(int i=0;i<3;++i) {
        int largest=i;
        for(int j=i+1;j<3;++j) if(result.singularValues[j] > result.singularValues[largest]) largest=j;
        if(result.singularValues[largest] == 0.) break;
        if(largest != i) {
            std::swap(result.singularValues[i],result.singularValues[largest]);
            for(int row=0;row<3;++row) {
                std::swap(result.leftVectors[row][i],result.leftVectors[row][largest]);
                std::swap(result.rightVectors[row][i],result.rightVectors[row][largest]);
            }
        }
    }
    Matrix3 scaledTranspose{};
    for(int i=0;i<3;++i) {
        const double singular=result.singularValues[i];
        if(!std::isfinite(singular)) return result;
        if(std::abs(singular) > 1.e-16) {
            ++result.retainedSingularValues;
            for(int j=0;j<3;++j) scaledTranspose[i][j]+=result.leftVectors[j][i]/singular;
        }
    }
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) for(int k=0;k<3;++k)
        result.inverse[i][j]+=result.rightVectors[i][k]*scaledTranspose[k][j];
    result.finiteResult=isFinite(result.inverse);
    return result;
}
}
