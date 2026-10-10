#pragma once
/** @file MeshSizeEvaluator.h
 * @brief 对应 BackgroundMeshTools.cpp 的标量尺寸组合，不修改几何或网格。
 * @date 2026-10-07
 */
#include "mesh/surface/algorithm6/data/SurfaceTypes.h"

namespace mesher::gmshReference {
struct MeshSizeQuery {
    int dimension = -1;
    int entity = -1;
    double u = 0., v = 0.;
    Point3D point;
};

// 本次运行的不可变查询源；空尺寸源对应原生 MAX_LC，无附加约束。
// 启用曲率时 maximumCurvature 必须提供实体维数对应的最大曲率：
// 顶点取邻接边/面，边取曲线和邻接面，面取自身最大主曲率。
struct GeometrySizeSource {
    std::function<double(const MeshSizeQuery&)> pointSize;
    std::function<double(const MeshSizeQuery&)> maximumCurvature;
    std::function<double(const MeshSizeQuery&)> backgroundField;
    std::function<double(const MeshSizeQuery&)> entitySize;
    std::function<double(const MeshSizeQuery&)> prescribedCurveSize;
    std::function<double(const MeshSizeQuery&)> entityFactor;
    std::function<double(const MeshSizeQuery&, double)> sizeCallback;
};

struct MeshSizeEvaluation {
    OperationStatus status;
    double size = 0.;
    bool usedInvalidSizeFallback = false;
    bool clampedCurvatureCircleCount = false;
};

/** LC_MVertex_CURV 的尺寸公式；maximumCurvature 非负且有限，单位 1/长度。
 * 0 曲率返回原生无约束尺寸；0<count<1 按原生行为取 1，输出告警状态。
 * 非法输入返回 InvalidInput，不伪造尺寸。
 */
MeshSizeEvaluation evaluateCurvatureSize(double maximumCurvature, double elementsPerCircle);

/** BGM_MeshSizeWithoutScaling/BGM_MeshSize 的原顺序：源取小、callback、
 * modelCharacteristicLength 取小、min/max、非法非正值回退、实体/全局倍率。
 * 返回物理长度；回退明确标记，供上层报告原生错误。非有限 CAD 查询失败。
 * pointSize 仅用于维数 0/1，prescribedCurveSize 仅用于维数 1。
 */
MeshSizeEvaluation evaluateMeshSize(const MeshSizeQuery& query,
    double modelCharacteristicLength, const SurfaceMeshingOptions& options,
    const GeometrySizeSource& source);
} // namespace mesher::gmshReference
