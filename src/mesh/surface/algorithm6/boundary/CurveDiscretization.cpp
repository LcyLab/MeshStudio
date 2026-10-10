// 文件职责：曲线离散：按尺寸加权弧长安排采样，再保留节点参数和物理坐标供相邻面共同使用。
#include "CurveDiscretization.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>

namespace mesher::gmshReference {
namespace {
struct CurveFailure { OperationStatus status; };

/// 检查取消回调，取消时终止当前算法阶段。
void checkCancellation(const RunControl& control)
{
    if(control.keepGoing && !control.keepGoing())
        throw CurveFailure{{MeshingStatus::Cancelled, "curve discretization cancelled"}};
}

/// 用梯形公式计算相邻采样点之间的密度积分。
double integrateTrapezoid(const CurveDensitySample& first, const CurveDensitySample& last)
{
    return (last.parameter - first.parameter) * .5 * (first.density + last.density);
}

/// 执行比值计算并保留零分母对应的无穷或无效值语义。
double nativeRatio(double numerator, double denominator)
{
    if(denominator != 0.) return numerator / denominator;
    if(numerator == 0. || std::isnan(numerator)) return std::numeric_limits<double>::quiet_NaN();
    return std::copysign(std::numeric_limits<double>::infinity(), numerator);
}

/// 校验曲线密度表的参数、尺寸和累计积分。
bool isValidTable(const CurveDensityTable& table)
{
    if(table.samples.size() < 2 || table.samples.front().cumulativeDensity != 0.) return false;
    for(std::size_t i = 0; i < table.samples.size(); ++i) {
        const auto& sample = table.samples[i];
        if(!std::isfinite(sample.parameter) || !std::isfinite(sample.density) || sample.density < 0. ||
           !std::isfinite(sample.speed) || sample.speed < 0. ||
           !std::isfinite(sample.cumulativeDensity) || sample.cumulativeDensity < 0.) return false;
        if(i && (sample.parameter <= table.samples[i - 1].parameter ||
                 sample.cumulativeDensity < table.samples[i - 1].cumulativeDensity)) return false;
    }
    return true;
}
} // namespace

/// 沿曲线自适应采样并积分尺寸密度。
OperationStatus integrateCurveDensity(double first, double last,
    const std::function<Point3D(double)>& derivative, const std::function<double(double)>& sizeAt,
    CurveDensityTable& output, double absolutePrecision, const RunControl& control)
{
    if(!std::isfinite(first) || !std::isfinite(last) || first >= last ||
       !std::isfinite(absolutePrecision) || absolutePrecision <= 0. || !derivative || !sizeAt)
        return {MeshingStatus::InvalidInput, "invalid curve interval, precision or evaluator"};
    CurveDensityTable table;
    try {
        auto evaluate = [&](double parameter) {
            checkCancellation(control);
            if(table.densityEvaluations >= control.maximumGeometryEvaluations)
                throw CurveFailure{{MeshingStatus::BudgetExceeded, "curve density evaluation budget exceeded"}};
            ++table.densityEvaluations;
            const Point3D tangent = derivative(parameter);
            const double speed = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y + tangent.z * tangent.z);
            const double size = sizeAt(parameter);
            const double density = speed / size;
            if(!std::isfinite(speed) || !std::isfinite(size) || size <= 0. || !std::isfinite(density))
                throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "invalid curve derivative or mesh size"}};
            return CurveDensitySample{parameter, density, 0., speed};
        };
        auto start = evaluate(first);
        auto finish = evaluate(last);
        table.samples.push_back(start);
        auto integrateInterval = [&](auto&& recurse, CurveDensitySample left,
                                     CurveDensitySample right, int depth) -> void {
            checkCancellation(control);
            const double middleParameter = .5 * (left.parameter + right.parameter);
            if(!std::isfinite(middleParameter) || middleParameter <= left.parameter || middleParameter >= right.parameter)
                throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "curve parameter subdivision stagnated"}};
            auto middle = evaluate(middleParameter);
            const double coarse = integrateTrapezoid(left, right);
            const double lower = integrateTrapezoid(left, middle);
            const double upper = integrateTrapezoid(middle, right);
            const double error = std::abs(coarse - lower - upper);
            if(!std::isfinite(coarse) || !std::isfinite(lower) || !std::isfinite(upper))
                throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "curve density integral overflow"}};
            if((error < absolutePrecision && depth > 6) || depth > 25) {
                table.nativeIntegrationDepthLimitReached |= depth > 25;
                middle.cumulativeDensity = table.samples.back().cumulativeDensity + lower;
                table.samples.push_back(middle);
                right.cumulativeDensity = table.samples.back().cumulativeDensity + upper;
                if(!std::isfinite(right.cumulativeDensity))
                    throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "curve cumulative density overflow"}};
                table.samples.push_back(right);
            }
            else {
                recurse(recurse, left, middle, depth + 1);
                recurse(recurse, middle, right, depth + 1);
            }
        };
        integrateInterval(integrateInterval, start, finish, 1);
        output = std::move(table);
        return {};
    }
    catch(const OperationFailure& failure) { return failure.status; }
    catch(const CurveFailure& failure) { return failure.status; }
    catch(const std::exception& error) { return {MeshingStatus::GeometryQueryFailed, error.what()}; }
}

/// 限制相邻曲线采样点的尺寸变化。
OperationStatus smoothCurveSizes(CurveDensityTable& output, double growthFactor, const RunControl& control)
{
    if(!isValidTable(output) || !std::isfinite(growthFactor) || growthFactor < 1.)
        return {MeshingStatus::InvalidInput, "invalid curve density table or growth factor"};
    CurveDensityTable table = output;
    try {
        const double slope = growthFactor - 1.;
        table.smoothingIterations = 0;
        table.nativeSmoothingLimitReached = false;
        while(true) {
            checkCancellation(control);
            std::size_t modifications = 0;
            for(std::size_t i = 1; i < table.samples.size(); ++i) {
                auto& right = table.samples[i];
                auto& left = table.samples[i - 1];
                // 0/0 的 NaN 使原生比较为 false；正速度/零密度的 infinity 仍可被扫回。
                const double delta = nativeRatio(right.speed, right.density) - nativeRatio(left.speed, left.density);
                const double interval = right.parameter - left.parameter;
                if(nativeRatio(delta / interval, right.speed) > slope * 1.01) {
                    const double size = nativeRatio(left.speed, left.density) + interval * slope * right.speed;
                    right.density = nativeRatio(right.speed, size);
                    ++modifications;
                }
            }
            for(std::size_t i = table.samples.size() - 1; i > 0; --i) {
                auto& right = table.samples[i];
                auto& left = table.samples[i - 1];
                const double delta = nativeRatio(left.speed, left.density) - nativeRatio(right.speed, right.density);
                const double interval = std::abs(left.parameter - right.parameter);
                if(nativeRatio(delta / interval, left.speed) > slope * 1.01) {
                    const double size = nativeRatio(right.speed, right.density) + interval * slope * right.speed;
                    left.density = nativeRatio(left.speed, size);
                    ++modifications;
                }
            }
            ++table.smoothingIterations;
            if(table.smoothingIterations > 2000) {
                table.nativeSmoothingLimitReached = modifications != 0;
                break;
            }
            if(!modifications) break;
        }
        for(std::size_t i = 1; i < table.samples.size(); ++i)
            table.samples[i].cumulativeDensity = table.samples[i - 1].cumulativeDensity +
                integrateTrapezoid(table.samples[i - 1], table.samples[i]);
        if(!isValidTable(table)) return {MeshingStatus::GeometryQueryFailed, "curve size smoothing overflow"};
        output = std::move(table);
        return {};
    }
    catch(const CurveFailure& failure) { return failure.status; }
    catch(const std::exception& error) { return {MeshingStatus::GeometryQueryFailed, error.what()}; }
}

/// 根据累计密度和最小分段要求计算曲线节点数量。
int computeCurvePointCount(double integral, int minimumSegments)
{
    if(!std::isfinite(integral) || integral < 0. || minimumSegments < 1)
        throw std::invalid_argument("invalid curve integral or minimum segment count");
    if(integral + 1.99 >= double(std::numeric_limits<int>::max()) + 1. || minimumSegments == std::numeric_limits<int>::max())
        throw std::overflow_error("curve point count overflow");
    return std::max(minimumSegments + 1, static_cast<int>(integral + 1.99));
}

/// 按曲线类型确定最小分段数。
int computeMinimumCurveSegments(CurveKind kind, double arcSpan, bool closed, int prescribedMinimum,
    const SurfaceMeshingOptions& options)
{
    if(!std::isfinite(arcSpan) || prescribedMinimum < 1 || options.minimumLinePoints < 1 ||
       options.minimumCurvePoints < 1 || options.minimumCirclePoints < 1)
        throw std::invalid_argument("invalid curve minimum-segment metadata or options");
    int segments;
    switch(kind) {
    case CurveKind::Line: segments = options.minimumLinePoints - 1; break;
    case CurveKind::Circle:
    case CurveKind::Ellipse: {
        const double angle = std::abs(arcSpan);
        const double count = angle > 6.28 ? double(options.minimumCirclePoints) :
            .99 + double(options.minimumCirclePoints - 1) * angle / (2. * std::acos(-1.));
        if(count >= double(std::numeric_limits<int>::max()) + 1.)
            throw std::overflow_error("curve minimum-segment count overflow");
        segments = static_cast<int>(count);
        break;
    }
    case CurveKind::Other: segments = options.minimumCurvePoints - 1; break;
    default: throw std::invalid_argument("unknown curve geometry kind");
    }
    if(closed) segments = std::max(segments, 4);
    return std::max(segments, prescribedMinimum);
}

/// 计算离散曲线节点保存的边界尺寸。
StoredCurveSizeResult computeStoredCurveSize(const CurveDensitySample& left, const CurveDensitySample& right,
    double speed)
{
    StoredCurveSizeResult result;
    if(!std::isfinite(left.density) || !std::isfinite(right.density) || left.density < 0. || right.density < 0. ||
       !std::isfinite(left.cumulativeDensity) || !std::isfinite(right.cumulativeDensity) ||
       left.cumulativeDensity < 0. || right.cumulativeDensity <= left.cumulativeDensity ||
       !std::isfinite(speed) || speed < 0.) {
        result.status = {MeshingStatus::InvalidInput, "invalid curve-size interpolation bracket or speed"};
        return result;
    }
    const double densityDifference = right.density - left.density;
    const double integralDifference = right.cumulativeDensity - left.cumulativeDensity;
    const double density = left.density + densityDifference / integralDifference * (speed - left.cumulativeDensity);
    result.size = nativeRatio(speed, density);
    result.finiteResult = std::isfinite(result.size);
    return result;
}

/// 按累计密度分配曲线节点参数。
OperationStatus distributeCurveParameters(const CurveDensityTable& table, int pointCount,
    std::vector<double>& output, const RunControl& control)
{
    if(!isValidTable(table) || pointCount < 2)
        return {MeshingStatus::InvalidInput, "invalid curve density table or point count"};
    if(static_cast<std::size_t>(pointCount) > control.maximumNodes)
        return {MeshingStatus::BudgetExceeded, "curve node budget exceeded"};
    const double total = table.samples.back().cumulativeDensity;
    if(total == 0. && pointCount > 2)
        return {MeshingStatus::InvalidInput, "zero density cannot distribute interior curve nodes"};
    try {
        checkCancellation(control);
        std::vector<double> parameters;
        parameters.reserve(pointCount);
        parameters.push_back(table.samples.front().parameter);
        const double spacing = total / static_cast<double>(pointCount - 1);
        std::size_t index = 1;
        for(int node = 1; node < pointCount - 1; ++node) {
            checkCancellation(control);
            const double target = static_cast<double>(node) * spacing;
            while(index < table.samples.size() &&
                  !(table.samples[index].cumulativeDensity >= target &&
                    table.samples[index - 1].cumulativeDensity < target)) ++index;
            if(index >= table.samples.size())
                return {MeshingStatus::GeometryQueryFailed, "curve cumulative density inverse has no interval"};
            const auto& left = table.samples[index - 1];
            const auto& right = table.samples[index];
            const double parameter = left.parameter + (right.parameter - left.parameter) /
                (right.cumulativeDensity - left.cumulativeDensity) * (target - left.cumulativeDensity);
            if(!std::isfinite(parameter) || parameter <= parameters.back() || parameter >= table.samples.back().parameter)
                return {MeshingStatus::GeometryQueryFailed, "curve parameter distribution stagnated"};
            parameters.push_back(parameter);
        }
        parameters.push_back(table.samples.back().parameter);
        output = std::move(parameters);
        return {};
    }
    catch(const CurveFailure& failure) { return failure.status; }
    catch(const std::exception& error) { return {MeshingStatus::GeometryQueryFailed, error.what()}; }
}
/// 按原参数及距离规则过滤过近的曲线节点。
CurveNodeFilteringResult filterCurveNodes(std::vector<CurveNode>& nodes, Point3D beginPoint,
    double beginParameter, int minimumInteriorNodes, const std::function<double(double)>& curveSizeAt,
    const RunControl& control)
{
    if(!std::isfinite(beginParameter)) {
        CurveNodeFilteringResult result;
        result.status = {MeshingStatus::InvalidInput,"invalid curve begin parameter"};
        return result;
    }
    return filterCurveNodes(nodes,beginPoint,[=] { return beginParameter; },minimumInteriorNodes,curveSizeAt,control);
}

/// 按原参数及距离规则过滤过近的曲线节点。
CurveNodeFilteringResult filterCurveNodes(std::vector<CurveNode>& nodes, Point3D beginPoint,
    const std::function<double()>& inverseBeginParameter, int minimumInteriorNodes,
    const std::function<double(double)>& curveSizeAt, const RunControl& control)
{
    CurveNodeFilteringResult result;
    const auto finitePoint=[](const Point3D& p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
    if(minimumInteriorNodes < 0 || !finitePoint(beginPoint) ||
       (!nodes.empty() && !curveSizeAt)) {
        result.status = {MeshingStatus::InvalidInput, "invalid curve filtering input"};
        return result;
    }
    for(std::size_t i = 0; i < nodes.size(); ++i) {
        if(!finitePoint(nodes[i].point) || !std::isfinite(nodes[i].parameter) ||
           (i && nodes[i].parameter <= nodes[i - 1].parameter)) {
            result.status = {MeshingStatus::InvalidInput, "invalid curve filtering node or parameter order"};
            return result;
        }
    }
    try {
        checkCancellation(control);
        std::vector<bool> remove(nodes.size(), false);
        std::size_t removedCount = 0;
        Point3D previousPoint = beginPoint;
        double previousParameter = 0.;
        bool previousIsBegin = true;
        for(std::size_t i = 0; i < nodes.size(); ++i) {
            checkCancellation(control);
            if(result.sizeEvaluations >= control.maximumGeometryEvaluations)
                throw CurveFailure{{MeshingStatus::BudgetExceeded, "curve filtering size-query budget exceeded"}};
            const auto& node = nodes[i];
            const double dx = node.point.x - previousPoint.x, dy = node.point.y - previousPoint.y,
                         dz = node.point.z - previousPoint.z;
            const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            double parameter = node.parameter;
            if(i) {
                if(previousIsBegin) {
                    if(!inverseBeginParameter)
                        throw CurveFailure{{MeshingStatus::InvalidInput,"missing CAD begin inverse evaluator"}};
                    previousParameter = inverseBeginParameter();
                }
                parameter = .5 * (node.parameter + previousParameter);
            }
            if(!std::isfinite(distance) || !std::isfinite(parameter))
                throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "curve filtering distance or parameter overflow"}};
            ++result.sizeEvaluations;
            const double size = curveSizeAt(parameter);
            if(!std::isfinite(size) || size <= 0.)
                throw CurveFailure{{MeshingStatus::GeometryQueryFailed, "invalid curve filtering size query"}};
            if(distance < size * .3) {
                remove[i] = true;
                ++removedCount;
            }
            else {
                previousPoint = node.point;
                previousParameter = node.parameter;
                previousIsBegin = false;
            }
        }
        if(nodes.size() - removedCount < static_cast<std::size_t>(minimumInteriorNodes)) {
            result.minimumNodeCountPreventedRemoval = removedCount != 0;
            return result;
        }
        // 普通三角形基线不 forceOdd；原生排序后全删同一批候选，保留链内身份顺序。
        std::vector<CurveNode> retained;
        retained.reserve(nodes.size() - removedCount);
        for(std::size_t i = 0; i < nodes.size(); ++i) {
            if(remove[i]) result.removedNodeIds.push_back(nodes[i].id);
            else retained.push_back(nodes[i]);
        }
        nodes = std::move(retained);
    }
    catch(const OperationFailure& failure) { result.status = failure.status; }
    catch(const CurveFailure& failure) { result.status = failure.status; }
    catch(const std::exception& error) { result.status = {MeshingStatus::GeometryQueryFailed, error.what()}; }
    catch(...) { result.status = {MeshingStatus::GeometryQueryFailed, "unknown curve filtering query exception"}; }
    if(!result.status.complete()) result.removedNodeIds.clear();
    return result;
}
} // namespace mesher::gmshReference
