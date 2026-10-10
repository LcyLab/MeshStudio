/** @file FrontInsertion.cpp
 * @brief 前沿新片验收/局部提交及新节点查询、分配槽和正常失败生命周期。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
#include "FrontInsertion.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
namespace mesher::gmshReference {
namespace {
/// 为追加条目预留容量，并检查容器容量溢出。
template<class T> void reserveAppend(std::vector<T>& values,std::size_t appended) {
    if(appended>values.max_size()-values.size())throw std::length_error("front pool capacity exceeded");
    const auto required=values.size()+appended;
    if(required<=values.capacity())return;
    const auto capacity=values.capacity();
    const auto grown=capacity>values.max_size()-capacity/2?values.max_size():capacity+capacity/2;
    values.reserve(std::max(required,grown));
}
/// 检查前沿插点操作的取消状态。
void checkControl(const RunControl& control) {
    if(control.keepGoing && !control.keepGoing()) throw OperationFailure{{MeshingStatus::Cancelled,"front insertion cancelled"}};
}
/// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
void require(OperationStatus status) { if(!status.complete()) throw OperationFailure{status}; }
/// 检查输入坐标或数值是否有限。
void finite(double value) {
    if(!std::isfinite(value)) throw OperationFailure{{MeshingStatus::NonConverged,"nonfinite front insertion arithmetic"}};
}
/// 计算两点之间的欧氏距离。
double distance(Point3D a,Point3D b) {
    const double dx = a.x-b.x,dy = a.y-b.y,dz = a.z-b.z;
    const double value = std::sqrt(dx*dx+dy*dy+dz*dz); finite(value); return value;
}
}
/// 在发布新三角形前检查空腔边界、尺寸及几何有效性。
FrontInsertionValidation validateFrontInsertion(const FrontMesh& front,const FaceMesh& mesh,const NodeSizeState& nodes,
    const FrontCavityResult& cavity,const FaceVertex& vertex,const InitializedNodeSize& node,
    const FrontInsertionOptions& options,const RunControl& control)
{
    FrontInsertionValidation result;
    try {
        checkControl(control); require(cavity.status);
        // 原生早退先于参数/物理面积和新片创建。
        if(cavity.triangles.size() == 1) { result.nativeCode = -1; return result; }
        if(cavity.shell.size() != cavity.triangles.size()+2) { result.nativeCode = -2; return result; }
        if(!vertex.id || vertex.id != node.vertex || mesh.vertices.find(vertex.id) != mesh.vertices.end())
            throw OperationFailure{{MeshingStatus::InvalidInput,"invalid new front vertex identity"}};
        if(nodes.nodes.size() > control.maximumNodes)
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"front node budget exceeded"}};
        auto triangle = [&](int index)->const FrontTriangle& {
            if(index < 0 || std::size_t(index) >= front.triangles.size()) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid insertion cavity triangle"}};
            return front.triangles[index];
        };
        auto initialized = [&](std::uint64_t id)->const InitializedNodeSize& {
            if(id == vertex.id) return node;
            const auto found = nodes.indices.find(id);
            if(found == nodes.indices.end() || found->second >= nodes.nodes.size() || nodes.nodes[found->second].vertex != id)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"insertion node index mismatch"}};
            return nodes.nodes[found->second];
        };
        auto geometry = [&](std::uint64_t id)->const FaceVertex& {
            if(id == vertex.id) return vertex;
            const auto found = mesh.vertices.find(id);
            if(found == mesh.vertices.end()) throw OperationFailure{{MeshingStatus::InvalidTopology,"insertion vertex missing from face mesh"}};
            return found->second;
        };
        auto area = [&](const std::array<std::uint64_t,3>& ids) {
            const auto a = initialized(ids[0]).parameters,b = initialized(ids[1]).parameters,c = initialized(ids[2]).parameters;
            const double value = .5*((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)); finite(value); return value;
        };
        result.relativeAreaTolerance = options.verifyStarShapeness ? 1.e-12 : 1.e12;
        for(int index : cavity.triangles) { checkControl(control); result.oldArea += std::abs(area(triangle(index).vertices)); }
        finite(result.oldArea);
        for(const auto& edge : cavity.shell) {
            checkControl(control); const auto& original = triangle(edge.triangle);
            if(edge.edge < 0 || edge.edge > 2) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid insertion shell edge"}};
            FrontInsertionTriangle created;
            created.triangle.vertices = {original.vertices[(edge.edge+2)%3],original.vertices[edge.edge],vertex.id};
            std::array<Point3D,3> points; std::array<NodeSize,3> sizes;
            for(int i = 0; i < 3; ++i) {
                points[i] = geometry(created.triangle.vertices[i]).point; sizes[i] = initialized(created.triangle.vertices[i]).sizes;
                finite(sizes[i].meshSize); finite(sizes[i].backgroundSize);
            }
            // insertVertexB使用ONE_THIRD=1./3.，不同于初始化入口的字面.3333333333。
            auto& t = created.triangle;
            t.meanSizes.meshSize = (1./3.)*(sizes[0].meshSize+sizes[1].meshSize+sizes[2].meshSize);
            t.meanSizes.backgroundSize = (1./3.)*(sizes[0].backgroundSize+sizes[1].backgroundSize+sizes[2].backgroundSize);
            finite(t.meanSizes.meshSize); finite(t.meanSizes.backgroundSize);
            const double length = std::min(t.meanSizes.meshSize,t.meanSizes.backgroundSize);
            t.targetSize = options.extendBoundarySize ? length : t.meanSizes.backgroundSize;
            const auto circle = computePhysicalCircumradius(points); require(circle.status);
            t.radius = circle.radius/t.targetSize; finite(t.radius);
            created.firstDistance = distance(points[0],points[2]); created.secondDistance = distance(points[1],points[2]); created.edgeLength = distance(points[0],points[1]);
            const double d1 = created.firstDistance,d2 = created.secondDistance,d3 = created.edgeLength;
            created.angleCosine = (d1*d1+d2*d2-d3*d3)/(2.*d1*d2);
            if(geometry(t.vertices[0]).dimension != 2 && geometry(t.vertices[1]).dimension != 2) {
                const auto a = points[0],b = points[1],c = points[2];
                const double ux = b.x-a.x,uy = b.y-a.y,uz = b.z-a.z,vx = c.x-a.x,vy = c.y-a.y,vz = c.z-a.z;
                const double x = uy*vz-uz*vy,y = uz*vx-ux*vz,z = ux*vy-uy*vx;
                created.boundaryHeight = std::sqrt(x*x+y*y+z*z)/d3;
            }
            // 保留零距离产生NaN的原比较结果，不把NaN角度变成新的拒绝原因。
            created.tooClose = (d1 < length*.5 || d2 < length*.5 || created.boundaryHeight < length*.4 || created.angleCosine < -.9999) && !options.force;
            result.tooClose = result.tooClose || created.tooClose;
            created.parameterArea = std::abs(area(t.vertices));
            if(created.parameterArea < 1.e-25) created.parameterArea = 1.e22;
            result.newArea += created.parameterArea; result.triangles.push_back(created);
        }
        finite(result.newArea); const double difference = std::abs(result.oldArea-result.newArea),threshold = result.relativeAreaTolerance*result.oldArea;
        finite(threshold);
        if(difference < threshold && !result.tooClose) result.nativeCode = 1;
        else if(difference > threshold) result.nativeCode = -3;
        else if(result.tooClose) result.nativeCode = -4;
        else result.nativeCode = -5;
        checkControl(control);
    } catch(const OperationFailure& error) { result.status = error.status; }
      catch(const std::exception& error) { result.status = {MeshingStatus::InvalidTopology,error.what()}; }
      catch(...) { result.status = {MeshingStatus::InvalidTopology,"unknown front validation failure"}; }
    return result;
}
/// 发布已校验的插点拓扑，并更新前沿邻接关系。
FrontInsertionCommit commitFrontInsertion(FrontMesh& front,FrontQueueState& queue,const FrontCavityResult& cavity,
    const FrontInsertionValidation& validation,const RunControl& control)
{
    FrontInsertionCommit result; const auto originalCount = front.triangles.size();
    std::map<int,std::array<int,3>> previousNeighbors;
    auto restore = [&] {
        auto remove = [&](FrontTriangleSet& set,const std::vector<int>& indices) {
            for(int index : indices) {
                const auto position = set.find(makeFrontTriangleKey(front.triangles[index],index));
                if(position != set.end() && position->triangle == index) set.erase(position);
            }
        };
        remove(queue.activeTriangles,result.activity.inserted); remove(queue.allTriangles,result.registration.inserted);
        for(const auto& entry : previousNeighbors) front.triangles[entry.first].neighbors = entry.second;
        front.triangles.resize(originalCount); result.createdTriangles.clear(); result.neighborhood.clear(); result.registration = {}; result.activity = {};
    };
    try {
        checkControl(control); require(cavity.status); require(validation.status);
        if(validation.nativeCode != 1 || validation.triangles.size() != cavity.shell.size())
            throw OperationFailure{{MeshingStatus::InvalidInput,"front commit requires accepted shell validation"}};
        if(validation.triangles.size() > std::size_t(std::numeric_limits<int>::max()) ||
           originalCount > std::size_t(std::numeric_limits<int>::max())-validation.triangles.size())
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"front triangle index budget exceeded"}};
        for(std::size_t i = 0; i < cavity.shell.size(); ++i) {
            checkControl(control); const auto& edge = cavity.shell[i];
            if(edge.triangle < 0 || std::size_t(edge.triangle) >= originalCount || edge.edge < 0 || edge.edge > 2)
                throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid insertion commit shell"}};
            const int index = int(originalCount+i),other = front.triangles[edge.triangle].neighbors[edge.edge];
            result.createdTriangles.push_back(index); result.neighborhood.push_back(index);
            if(other >= 0) {
                if(std::size_t(other) >= originalCount) throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid insertion shell other side"}};
                previousNeighbors.emplace(other,front.triangles[other].neighbors); result.neighborhood.push_back(other);
            }
        }
        // 提交前仍完成可能失败的分配；按几何级数增长，避免每次插点搬移整个池。
        reserveAppend(front.triangles,validation.triangles.size());
        for(const auto& created : validation.triangles) front.triangles.push_back(created.triangle);
        struct Edge { std::array<std::uint64_t,2> vertices; int triangle,edge; }; std::vector<Edge> edges;
        for(int index : result.neighborhood) {
            checkControl(control); const auto& t = front.triangles[index]; if(t.deleted) continue;
            for(int edge = 0; edge < 3; ++edge) {
                std::array<std::uint64_t,2> ids{t.vertices[(edge+2)%3],t.vertices[edge]};
                if(ids[1] < ids[0]) std::swap(ids[0],ids[1]); edges.push_back({ids,index,edge});
            }
        }
        // 包含重复外侧片的原局部序列；不清空未匹配邻接或重连全图。
        std::sort(edges.begin(),edges.end(),[](const Edge& a,const Edge& b) { return a.vertices < b.vertices; });
        for(std::size_t i = 0; i+1 < edges.size(); ++i) {
            const auto& a = edges[i]; const auto& b = edges[i+1];
            if(a.vertices == b.vertices && a.triangle != b.triangle) {
                front.triangles[a.triangle].neighbors[a.edge] = b.triangle; front.triangles[b.triangle].neighbors[b.edge] = a.triangle; ++i;
            }
        }
        result.registration = registerFrontTriangles(front,result.createdTriangles,queue,control); require(result.registration.status);
        result.activity = updateActiveTriangles(front,result.neighborhood,queue,control); require(result.activity.status);
        checkControl(control);
    } catch(const OperationFailure& error) { restore(); result.status = error.status; }
      catch(const std::exception& error) { restore(); result.status = {MeshingStatus::InvalidTopology,error.what()}; }
      catch(...) { restore(); result.status = {MeshingStatus::InvalidTopology,"unknown front commit failure"}; }
    return result;
}
/// 向空腔追加候选点连接的三角形。
FrontInsertionResult insertFrontTriangles(FrontMesh& front,FrontQueueState& queue,const FaceMesh& mesh,const NodeSizeState& nodes,
    const FrontCavityResult& cavity,const FaceVertex& vertex,const InitializedNodeSize& node,
    const FrontInsertionOptions& options,const RunControl& control)
{
    FrontInsertionResult result; result.validation = validateFrontInsertion(front,mesh,nodes,cavity,vertex,node,options,control);
    result.status = result.validation.status; if(!result.status.complete()) return result;
    result.nativeCode = result.validation.nativeCode;
    if(result.nativeCode == 1) { result.commit = commitFrontInsertion(front,queue,cavity,result.validation,control); result.status = result.commit.status; }
    else if(result.nativeCode <= -3) result.status = rollbackFrontCavity(front,cavity);
    return result;
}
/// 完成候选定位、空腔构造、校验及插点提交。
FrontPointInsertionResult insertFrontPoint(FrontMesh& front,FrontQueueState& queue,FaceMesh& mesh,NodeSizeState& nodes,
    int worst,Point2D point,SurfaceMetric metric,const FrontInsertionSource& source,
    const FrontInsertionOptions& options,const RunControl& control)
{
    FrontPointInsertionResult result; const auto nodeCount = nodes.nodes.size(); const auto maximumId = mesh.maximumVertexId;
    FrontTriangleSet savedWorst; FrontTriangleKey originalKey; bool hasOriginalKey = false;
    auto restore = [&] {
        // 空腔只记录本次成员；无需复制全部三角形删除标记。
        for(std::size_t i = 0; i < result.cavity.triangles.size(); ++i)
            front.triangles[result.cavity.triangles[i]].deleted = result.cavity.entryDeleted[i];
        nodes.nodes.resize(nodeCount); mesh.maximumVertexId = maximumId; result.nodeAppended = false;
        if(hasOriginalKey) {
            const auto current = queue.allTriangles.find(makeFrontTriangleKey(front.triangles[worst],worst));
            if(current != queue.allTriangles.end() && current->triangle == worst) queue.allTriangles.erase(current);
            front.triangles[worst].radius = originalKey.radius;
            // 预分配原键，异常恢复不再分配set节点；保留其他片的碰撞键。
            queue.allTriangles.insert(savedWorst.extract(savedWorst.begin()));
        }
    };
    try {
        checkControl(control);
        if(options.force) throw OperationFailure{{MeshingStatus::InvalidInput,"algorithm6 insertAPoint does not force insertion"}};
        if(worst < 0 || std::size_t(worst) >= front.triangles.size())
            throw OperationFailure{{MeshingStatus::InvalidTopology,"invalid front point worst triangle"}};
        originalKey = makeFrontTriangleKey(front.triangles[worst],worst);
        const auto position = queue.allTriangles.find(originalKey),active = queue.activeTriangles.find(originalKey);
        if(position == queue.allTriangles.end() || position->triangle != worst ||
           (active != queue.activeTriangles.end() && active->triangle == worst))
            throw OperationFailure{{MeshingStatus::InvalidTopology,"front point worst missing from AllTris or not popped"}};
        savedWorst.insert(originalKey); hasOriginalKey = true;
        result.cavity = buildFrontCavity(front,nodes,queue.allTriangles,worst,point,metric,false,control); require(result.cavity.status);
        if(result.cavity.containingTriangle < 0) {
            require(rollbackFrontCavity(front,result.cavity));
            result.failure = markInsertionFailure(front,queue,worst,FrontInsertionFailure::ContainingTriangleNotFound,control); require(result.failure.status);
            return result;
        }
        if(!source.surfacePoint || !source.backgroundSize)
            throw OperationFailure{{MeshingStatus::InvalidInput,"front point geometry/size source missing"}};
        if(nodes.nodes.size() >= control.maximumNodes || maximumId == std::numeric_limits<std::uint64_t>::max())
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"front point allocation budget exceeded"}};
        if((!mesh.vertices.empty() && mesh.vertices.rbegin()->first > maximumId) ||
           (!nodes.nodes.empty() && nodes.nodes.back().vertex > maximumId))
            throw OperationFailure{{MeshingStatus::InvalidTopology,"front allocation counter precedes existing vertex"}};
        auto query = [&] {
            checkControl(control);
            if(result.geometryEvaluations >= control.maximumGeometryEvaluations)
                throw OperationFailure{{MeshingStatus::BudgetExceeded,"front point query budget exceeded"}};
            ++result.geometryEvaluations;
        };
        query(); result.pointQueried = true;
        try { result.surface = source.surfacePoint(point); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
        catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown front point CAD query failure"}}; }
        finite(result.surface.point.x); finite(result.surface.point.y); finite(result.surface.point.z);
        FaceVertex vertex; vertex.id = ++mesh.maximumVertexId; vertex.dimension = 2; vertex.entity = mesh.entity;
        vertex.origin = FaceVertexOrigin::FaceCreated; vertex.parameters = point; vertex.point = result.surface.point;
        result.allocatedVertex = vertex.id;
        if(mesh.vertices.count(vertex.id) || nodes.indices.count(vertex.id))
            throw OperationFailure{{MeshingStatus::InvalidTopology,"new front identity already exists"}};
        result.node.vertex = vertex.id; result.node.parameters = point;
        require(interpolateNodeMeshSize(front.triangles[result.cavity.containingTriangle],nodes,result.cavity.coordinates.local,result.node.sizes.meshSize));
        result.sizeQuery = {2,mesh.entity,point.x,point.y,vertex.point};
        query(); result.backgroundQueried = true;
        try { result.sizeEvaluation = source.backgroundSize(result.sizeQuery); }
        catch(const OperationFailure&) { throw; }
        catch(const std::exception& error) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,error.what()}}; }
        catch(...) { throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"unknown front point BGM query failure"}}; }
        require(result.sizeEvaluation.status); result.node.sizes.backgroundSize = result.sizeEvaluation.size; finite(result.node.sizes.backgroundSize);
        result.parameterIndex = nodes.nodes.size(); nodes.nodes.push_back(result.node); result.nodeAppended = true;
        checkControl(control);
        if(result.surface.succeeded) {
            result.triangles.validation = validateFrontInsertion(front,mesh,nodes,result.cavity,vertex,result.node,options,control);
            require(result.triangles.validation.status); result.nativeCode = result.triangles.validation.nativeCode;
            result.triangles.nativeCode = result.nativeCode;
            if(result.nativeCode == 1) {
                // 准备可能分配的发布容器；成功局部提交后用node_handle发布，不再分配。
                std::map<std::uint64_t,FaceVertex> publication; publication.emplace(vertex.id,vertex);
                std::map<std::uint64_t,std::size_t> index; index.emplace(vertex.id,result.parameterIndex);
                reserveAppend(mesh.interiorVertices,1);
                result.triangles.commit = commitFrontInsertion(front,queue,result.cavity,result.triangles.validation,control); require(result.triangles.commit.status);
                mesh.vertices.insert(publication.extract(publication.begin())); nodes.indices.insert(index.extract(index.begin()));
                mesh.interiorVertices.push_back(vertex.id); result.inserted = true; return result;
            }
        }
        else result.nativeCode = -9;
        // insertVertexB先恢复几何拒绝，再由insertAPoint改worst半径；-1/-2/-9只在外层恢复。
        if(result.nativeCode >= -5 && result.nativeCode <= -3) require(rollbackFrontCavity(front,result.cavity));
        // 正常原生失败不回收参数/尺寸槽，也不退回已经消费的节点身份。
        result.failure = markInsertionFailure(front,queue,worst,FrontInsertionFailure::PointInsertionRejected,control); require(result.failure.status);
        require(rollbackFrontCavity(front,result.cavity));
    } catch(const OperationFailure& error) { restore(); result.status = error.status; }
      catch(const std::exception& error) { restore(); result.status = {MeshingStatus::InvalidTopology,error.what()}; }
      catch(...) { restore(); result.status = {MeshingStatus::InvalidTopology,"unknown front point lifecycle failure"}; }
    return result;
}
}
