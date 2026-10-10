#pragma once
/**
 * @file MeshingObserver.h
 * @brief 定义可关闭的网格阶段/工作量观察接口，不控制算法和取消。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include <cstdint>

namespace mesher {
enum class MeshStage {
    Generation, BoundarySampling, InitialTriangulation, ConstraintRecovery,
    DomainClassification, Refinement, GeometryEvaluation, TopologyUpdate,
    Legalization, MeshAssembly, Validation, CriterionMeasurement, IntersectionValidation, PhysicalQuality
};
enum class MeshCounter {
    GeometryQueries, WholeMeshCopies, CopiedPoints, CopiedTriangles,
    TriangleVisits, PointVisits, CavityTriangles, WholeDomainClassifications,
    GeometryCacheHits, GeometryCacheSamples, LocalMeshTransactions,
    TransactionTrianglesLogged, TransactionHalfedgesLogged, RolledBackTransactions,
    WholeMeshValidations, LocalMeshValidations, CavityStampResets,
    RefinementQueuePushes,RefinementQueueStaleEntries,RefinementQueuePeakEntries,
    RefinementQueueCompactions,FullCriterionAudits,
    BoundaryQueueRequests,BoundaryQueueStaleRequests,BoundaryQueuePeakEntries,
    QualityCandidateTrials,QualityRejectedDomain,QualityRejectedDuplicate,
    QualityRejectedFloor,QualityRejectedShape,QualityRejectedGeometry,QualityRejectedTopology,
    QualityRejectedMetric,QualityCandidateComparisons,QualityReusedFailures,
    ShapeCandidateTrials,ShapeReusedNeighborhoods,ShapeRejectedFloor,QualityReusedFaces,NarrowBoundaryAlignmentRequests
};
struct MeshEntityRef { int dimension=-1; int id=-1; };
// 借用对象，须存活到生成返回；回调不得抛异常或改变输入几何。
class MeshingObserver {
public:
    /// 销毁观察器接口，支持通过基类释放派生对象。
    virtual ~MeshingObserver() = default;
    /// 接收网格算法阶段开始通知。
    virtual void stageBegin(MeshStage,MeshEntityRef) noexcept = 0;
    /// 接收网格算法阶段结束通知。
    virtual void stageEnd(MeshStage,MeshEntityRef) noexcept = 0;
    /// 累加指定工作量计数。
    virtual void addCount(MeshCounter,MeshEntityRef,std::uint64_t) noexcept = 0;
};
struct MeshObservation {
    MeshingObserver* observer=nullptr;
    MeshEntityRef entity{};
    /// 向观察器记录指定类型的工作量。
    void count(MeshCounter counter,std::uint64_t n) const noexcept {
        if (observer) observer->addCount(counter,entity,n);
    }
};
// 批量提交循环实际访问次数，提前返回时也记录已访问项。
struct MeshCountScope {
    MeshObservation observation;
    MeshCounter counter;
    std::uint64_t value=0;
    /// 将作用域中累计的工作量提交给观察器。
    ~MeshCountScope() { observation.count(counter,value); }
};
// 只发送事件；无观察器时不读取时钟。提前返回/异常均关闭已开始的阶段。
class MeshStageScope {
    MeshObservation observation;
    MeshStage stage;
public:
    /// 发送阶段开始通知并管理观察范围；禁止复制该对象。
    MeshStageScope(MeshObservation o,MeshStage s) noexcept : observation(o),stage(s) {
        if (observation.observer) observation.observer->stageBegin(stage,observation.entity);
    }
    /// 在作用域结束时发送阶段结束通知。
    ~MeshStageScope() { finish(); }
    /// 发送阶段开始通知并管理观察范围；禁止复制该对象。
    MeshStageScope(const MeshStageScope&)=delete;
    /// 禁止复制阶段范围，避免重复结束通知。
    MeshStageScope& operator=(const MeshStageScope&)=delete;
    /// 发送当前阶段结束通知，重复调用不再通知。
    void finish() noexcept {
        if (observation.observer) observation.observer->stageEnd(stage,observation.entity);
        observation.observer=nullptr;
    }
    /// 结束原阶段并切换观察对象、实体和阶段。
    void change(MeshObservation o,MeshStage s) noexcept {
        finish(); observation=o; stage=s;
        if (observation.observer) observation.observer->stageBegin(stage,observation.entity);
    }
};
} // namespace mesher
