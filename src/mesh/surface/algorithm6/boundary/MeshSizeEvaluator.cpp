// 文件职责：Algorithm 6 尺寸查询：组合运行配置和几何尺寸来源；不插点，也不修改网格连接。
#include "MeshSizeEvaluator.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace mesher::gmshReference {
namespace {
constexpr double unconstrainedSize = 1.e22;

/// 调用可选尺寸源并拒绝非有限的返回值。
double evaluateSource(const std::function<double(const MeshSizeQuery&)>& source,
                      const MeshSizeQuery& query, const char* name)
{
    if(!source) return unconstrainedSize;
    const double value = source(query);
    if(!std::isfinite(value)) throw std::runtime_error(std::string(name) + " returned nonfinite value");
    return value;
}
} // namespace

/// 由曲率和采样设置计算尺寸限制。
MeshSizeEvaluation evaluateCurvatureSize(double maximumCurvature, double elementsPerCircle)
{
    MeshSizeEvaluation result;
    if(!std::isfinite(maximumCurvature) || maximumCurvature < 0. ||
       !std::isfinite(elementsPerCircle) || elementsPerCircle <= 0.) {
        result.status = {MeshingStatus::InvalidInput, "invalid maximum curvature or circle element count"};
        return result;
    }
    result.clampedCurvatureCircleCount = elementsPerCircle < 1.;
    const double count = std::max(elementsPerCircle, 1.);
    result.size = maximumCurvature > 0. ? 2. * std::acos(-1.) / maximumCurvature / count : unconstrainedSize;
    if(!std::isfinite(result.size))
        result.status = {MeshingStatus::GeometryQueryFailed, "curvature size overflow"};
    return result;
}

/// 组合实体、背景及曲率尺寸来源并处理无效尺寸。
MeshSizeEvaluation evaluateMeshSize(const MeshSizeQuery& query, double modelCharacteristicLength,
    const SurfaceMeshingOptions& options, const GeometrySizeSource& source)
{
    MeshSizeEvaluation result;
    if(query.dimension < -1 || query.dimension > 3 ||
       !std::isfinite(modelCharacteristicLength) || modelCharacteristicLength <= 0. ||
       !std::isfinite(options.sizeMin) || options.sizeMin < 0. ||
       !std::isfinite(options.sizeMax) || options.sizeMax < 0. ||
       !std::isfinite(options.sizeFactor) || options.sizeFactor <= 0. ||
       !std::isfinite(options.elementsPerCurvatureCircle) || options.elementsPerCurvatureCircle < 0. ||
       !std::isfinite(query.u) || !std::isfinite(query.v) ||
       !std::isfinite(query.point.x) || !std::isfinite(query.point.y) || !std::isfinite(query.point.z)) {
        result.status = {MeshingStatus::InvalidInput, "invalid mesh size query/options/model length"};
        return result;
    }
    try {
        const bool hasEntity = query.dimension >= 0;
        double size = unconstrainedSize;
        if(hasEntity && query.dimension < 2 && options.sizeFromPoints)
            size = std::min(size, evaluateSource(source.pointSize, query, "point size"));
        if(hasEntity && query.dimension < 3 && options.elementsPerCurvatureCircle > 0.) {
            if(!source.maximumCurvature) throw std::runtime_error("enabled maximum curvature capability missing");
            auto curvature = evaluateCurvatureSize(source.maximumCurvature(query), options.elementsPerCurvatureCircle);
            if(!curvature.status.complete()) throw std::runtime_error(curvature.status.message);
            result.clampedCurvatureCircleCount = curvature.clampedCurvatureCircleCount;
            size = std::min(size, curvature.size);
        }
        if(hasEntity) {
            size = std::min(size, evaluateSource(source.backgroundField, query, "background field"));
            size = std::min(size, evaluateSource(source.entitySize, query, "entity size"));
            if(query.dimension == 1)
                size = std::min(size, evaluateSource(source.prescribedCurveSize, query, "prescribed curve size"));
        }
        if(source.sizeCallback) {
            size = source.sizeCallback(query, size);
            if(!std::isfinite(size)) throw std::runtime_error("size callback returned nonfinite value");
        }
        size = std::min(modelCharacteristicLength, size);
        size = std::max(size, options.sizeMin);
        size = std::min(size, options.sizeMax);
        if(size <= 0.) {
            result.usedInvalidSizeFallback = true;
            size = modelCharacteristicLength;
        }
        if(hasEntity && source.entityFactor) {
            const double factor = source.entityFactor(query);
            if(!std::isfinite(factor) || factor <= 0.) throw std::runtime_error("invalid entity size factor");
            size *= factor;
        }
        result.size = size * options.sizeFactor;
        if(!std::isfinite(result.size) || result.size <= 0.)
            throw std::runtime_error("mesh size scaling overflow/underflow");
    }
    catch(const OperationFailure& failure) { result.status = failure.status; result.size = 0.; }
    catch(const std::exception& error) {
        result.status = {MeshingStatus::GeometryQueryFailed, error.what()};
        result.size = 0.;
    }
    return result;
}
} // namespace mesher::gmshReference
