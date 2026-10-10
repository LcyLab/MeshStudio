#pragma once
/** @file SurfaceGenerator.h
 * @brief 独立普通/周期面算法6编排；只接收CAD回调与配置，不接受参考运行池。
 * @date 2026-10-08
 * @modified 2026-10-09
 */
#include "mesh/surface/algorithm6/chart/FaceChart.h"
#include "mesh/surface/algorithm6/front/FrontGeneration.h"
#include "mesh/surface/algorithm6/bds/InitialMeshLegalization.h"
#include "mesh/surface/algorithm6/postprocess/SurfaceSmoothing.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
namespace mesher::gmshReference {
struct Algorithm6FaceInput {
    FaceChartInput chart;
    int bodyId = -1;
    NodeReparameterizationSource reparameterize;
    InitialSurfaceMetricSource initialMetric;
    FrontGenerationSource front;
    SurfaceSmoothingSource smoothing;
    std::function<Point3D(Point2D)> orientedNormal;
    // 最终元数据：CAD边pcurve与物理点不一致时用已有支撑面投影校正UV。
    // 不覆盖生成阶段原始参数或移动边界XYZ；seed保留周期角点分支。
    std::function<Point2D(Point3D,Point2D)> finalBoundaryUV;
    // 周期CAD图表工厂使用；普通面前沿依原生入口不采样、不启用此门控。
    std::vector<TrueBoundaryCurveInput> trueBoundaryCurves;
    // 周期分支由CAD拓扑明确选择；在共享边细分后重新建立精确pcurve图表。
    // builder按control累计查询到evaluations，CAD所有者须覆盖整个运行。
    std::function<PeriodicFaceChartInput(const BoundaryMesh&,const SurfaceMeshingOptions&,
        const RunControl&,std::size_t& evaluations)> periodicChart;
};
struct Algorithm6ModelInput {
    BoundaryInput boundary;
    std::vector<Algorithm6FaceInput> faces;
    GeometrySizeSource sizes;
};
struct SurfaceGenerationStatistics {
    std::size_t geometryEvaluations = 0, inserted = 0, boundaryRetries = 0;
};
struct Algorithm6FaceSummary {
    int face = -1, bodyId = -1;
    bool attempted = false, current = false; // current只表示最新共享边版本下的完整面。
    OperationStatus status;
    std::string failedStage;
    SurfaceGenerationStatistics statistics;
};
struct Algorithm6SurfaceResult {
    OperationStatus status;
    int failedFace = -1;
    std::string failedStage;
    SurfaceGenerationStatistics statistics;
    SurfaceMeshResult surface;
    std::vector<Algorithm6FaceSummary> faces; // 全部CAD面；未运行面attempted=false，不能当成功。
};
// 观察只用于导出；boundary总是本次自研状态，mesh在boundary阶段为空。
using SurfaceStageObserver = std::function<void(const std::string&,int,const BoundaryMesh&,const FaceMesh*)>;
struct Algorithm6FaceResult {
    OperationStatus status;
    std::string failedStage;
    FaceMesh mesh;
    std::vector<UnrecoveredEdge> unrecovered;
    SurfaceGenerationStatistics statistics;
};
/** 单面只使用调用方共享边；恢复需要细分时返回unrecovered，由模型入口重建。
 * 失败不发布mesh；取消/预算明确传播。输入回调及其CAD所有者须覆盖调用生命周期。
 */
Algorithm6FaceResult generateFaceMesh(const BoundaryMesh&,const Algorithm6FaceInput&,
    double modelLength,const GeometrySizeSource&,const SurfaceMeshingOptions&,
    const RunControl& = {},const SurfaceStageObserver& = {});
/** BREP适配后的独立模型入口。普通/周期面；禁止算法切换。生成成功与质量审计分开。
 * 仅全模型成功才发布surface；共享边变化只失效关联面。局部恢复与模型PENDING
 * 重访分别采用maximumRecoveryRetries上限，与固定串行Generator调度一致。
 * 初始周期扰动流由模型运行持有，逐面及重试不重置；不污染全局CRT随机状态。
 */
Algorithm6SurfaceResult generateSurfaceMesh(const Algorithm6ModelInput&,const SurfaceMeshingOptions&,
    const RunControl& = {},const SurfaceStageObserver& = {});
}
