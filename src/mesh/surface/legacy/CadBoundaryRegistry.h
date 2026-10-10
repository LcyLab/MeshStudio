#pragma once

#include <map>
#include <string>
#include <vector>
#include <tuple>
#include <optional>
#include "mesh/runtime/MeshingObserver.h"
#include "geometry/types/MesherTypes.h"

namespace mesher {

struct CadBoundarySample { double t; int global; };
struct CadBoundarySizingAnchor {Point3D point;double size;int cadEdge=-1;};
enum class CadBoundarySplitPurpose { Geometry,Quality };

// 每条 CAD 边共享一条物理节点链，各参数曲线分支保留独立 UV；同次运行中已分配的边参数节点身份保持不变。
class CadBoundaryRegistry {
public:
    struct Edge {
        bool ready=false;
        std::uint64_t version=1;
        std::vector<CadBoundarySample> samples;
        // 首次几何离散的固定尺寸来源；质量切分不产生更小的源。
        std::vector<CadBoundarySizingAnchor> qualityAnchors;
        std::map<double,int> nodeIds;
        std::vector<int> incidentFaces;
        double measuredDeviation=0,measuredNormalAngle=0;
        std::vector<std::string> diagnostics;
    };

    struct SplitRequest {
        std::uint64_t id=0,boundaryVersion=0;
        int edge=-1,sourceFace=-1;
        double left=0,right=0;
        std::string reason;
        double parameter=0;
        CadBoundarySplitPurpose purpose=CadBoundarySplitPurpose::Geometry;
    };
    /// 按 CAD 边数量建立共享边界登记表。
    explicit CadBoundaryRegistry(int edgeCount,MeshingObserver* observer=nullptr):m_edges(edgeCount),m_observer(observer) {}
    /// 按 CAD 边编号读取共享边界记录。
    Edge& edge(int id) { return m_edges.at(id); }
    /// 按 CAD 边编号读取共享边界记录。
    const Edge& edge(int id) const { return m_edges.at(id); }
    /// 登记共享 CAD 边关联的面，避免重复登记。
    void addUse(int edge,int face);
    // 登记可表示的新中点，仅使对应边失效；调用方负责使该边关联的面网格失效。
    /// 请求在边界参数区间内增加中点采样。
    bool requestSplit(int edge,double left,double right,double& parameter,std::string* error);
    /// 更新共享边版本并使采样状态失效。
    void invalidate(int edge);
    // 默认中点；显式参数须为未登记的有限内部参数，同区间跨面请求沿用首次选择。
    // 几何目的优先于质量目的，用于决定是否重新建立参考尺寸来源，不依赖reason文本。
    bool enqueueSplit(int edge,double left,double right,int sourceFace,const std::string& reason,
                      std::uint64_t& requestId,std::string* error,std::optional<double> parameter={},
                      CadBoundarySplitPurpose purpose=CadBoundarySplitPurpose::Geometry);
    /// 取出仍有效的边界细分请求，丢弃过期项。
    bool popSplitRequest(SplitRequest&);
    /// 应用一条与当前版本一致的边界细分请求。
    bool applySplit(const SplitRequest&,double& parameter,std::string* error);
    // 修改前先按同一个边界快照校验整批请求。
    /// 先校验同一边界快照上的全部请求，再统一登记细分。
    bool applySplitBatch(const std::vector<SplitRequest>&,std::vector<double>& parameters,std::string* error);

private:
    std::vector<Edge> m_edges;
    using RequestKey=std::tuple<int,std::uint64_t,double,double>;
    std::map<RequestKey,SplitRequest> m_requests;
    std::uint64_t m_nextRequest=1;
    size_t m_peakRequests=0;
    MeshingObserver* m_observer=nullptr;
    /// 检查边界细分请求是否仍匹配当前边版本和参数区间。
    bool current(const SplitRequest&) const;
    /// 校验新增参数并登记边界细分，使对应边的缓存失效。
    bool registerSplit(int edge,double left,double right,double selected,double& parameter,std::string* error);
};

} // namespace mesher
