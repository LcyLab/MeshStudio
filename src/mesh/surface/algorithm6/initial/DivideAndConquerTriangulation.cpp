// 文件职责：分治初始剖分：排序 UV 点、递归剖分并合并有序邻接；本模块不查询 CAD。
#include "DivideAndConquerTriangulation.h"
#include "geometry/predicates/GeomPredicates.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
namespace mesher::gmshReference {
namespace {
// 邻接向量按原生 Succ 方向排列，首槽就是可由 FixFirst 改变的环起点。
// 索引代替裸链表所有权，旋转/删除不改变原生顺序；不引入角度全排序。
class DivideAndConquerBuilder {
    const RunControl& control;
    DivideAndConquerTriangulation mesh;
    std::vector<std::vector<int>> rings;
    /// 检查运行取消状态，取消时停止当前阶段。
    void check() const {
        if(control.keepGoing && !control.keepGoing())
            throw OperationFailure{{MeshingStatus::Cancelled,"divide/conquer triangulation cancelled"}};
    }
    /// 检查操作状态或前置条件，失败时抛出本模块的明确错误。
    void require(bool valid,const char* message) const {
        if(!valid) throw OperationFailure{{MeshingStatus::InvalidTopology,message}};
    }
    /// 查询排序后输入节点的二维坐标。
    const Point2D& point(int id) const {
        require(id >= 0 && id < int(mesh.sortedPoints.size()),"unknown divide/conquer point index");
        return mesh.sortedPoints[id].coordinates;
    }
    /// 按偏移查询节点环中的相邻节点。
    int neighbor(int center,int target,int offset) const {
        check(); const auto& ring = rings.at(center);
        const auto found = std::find(ring.begin(),ring.end(),target);
        require(found != ring.end(),"missing divide/conquer neighbor");
        return ring[(std::size_t(found-ring.begin())+ring.size()+offset)%ring.size()];
    }
    /// 读取节点环中指定邻接点的前一个节点。
    int predecessor(int center,int target) const { return neighbor(center,target,-1); }
    /// 读取节点环中指定邻接点的后一个节点。
    int successor(int center,int target) const { return neighbor(center,target,1); }
    /// 读取节点环的首邻接点。
    int first(int center) const {
        require(!rings.at(center).empty(),"null divide/conquer adjacency"); return rings[center].front();
    }
    /// 旋转节点环，使指定邻接点成为首节点。
    void setFirst(int center,int target) {
        auto& ring = rings.at(center); const auto found = std::find(ring.begin(),ring.end(),target);
        require(found != ring.end(),"missing divide/conquer first neighbor");
        std::rotate(ring.begin(),found,ring.end());
    }
    /// 判断第三个点是否位于有向边左侧。
    bool leftOf(int a,int b,int c) const { return orient2d(point(a),point(b),point(c)) > 0; }
    /// 判断第三个点是否位于有向边右侧。
    bool rightOf(int a,int b,int c) const { return leftOf(b,a,c); }
    /// 按有向外接圆判定检查节点是否在圆外。
    bool outsideCircle(int a,int b,int c,int d) const {
        check();
        require(!(a == b && a == c && a == d),"identical divide/conquer point indices");
        // 原生 Qtest 仅 product<0 视为圆外；精确共线/共圆不走 outside 分支。
        // 当前符号组合已对照正常尺度；原版数值乘积上下溢分支仍待复现验证。
        return incircleOriented(point(a),point(b),point(c),point(d))*orient2d(point(a),point(b),point(c)) < 0;
    }
    /// 按几何方向将邻接点插入节点环。
    void insertNeighbor(int center,int added) {
        check(); auto& ring = rings.at(center);
        if(ring.empty()) { ring.push_back(added); return; }
        if(ring.size() == 1) { ring.push_back(added); return; }
        const int head = ring.front(); const auto origin = point(center);
        auto angle = [&](int id) { const auto p = point(id); return std::atan2(p.y-origin.y,p.x-origin.x); };
        const double firstAngle = angle(head), tau = 2.*3.14159265358979323846;
        double beta = angle(added)-firstAngle;
        if(beta <= 0.) beta += tau;
        for(std::size_t i = 0; i < ring.size(); ++i) {
            check(); const auto next = (i+1)%ring.size();
            double alpha = angle(ring[next])-firstAngle;
            if(alpha <= -1.e-15 || ring[next] == head) alpha += tau;
            else if(std::abs(alpha) <= 1.e-15 && rightOf(center,head,ring[next])) alpha += tau;
            if(alpha >= beta+1.e-15) {
                ring.insert(ring.begin()+i+1,added); return;
            }
            if(alpha >= beta-1.e-15) {
                // 近同角时保留原生严格方向决胜，而非 index 或距离决胜。
                const auto after = rightOf(center,ring[next],added) ? i : next;
                ring.insert(ring.begin()+after+1,added); return;
            }
        }
        throw OperationFailure{{MeshingStatus::InvalidTopology,"divide/conquer polar insertion found no interval"}};
    }
    /// 在两个节点的邻接环中建立双向连接。
    void connect(int a,int b) { insertNeighbor(a,b); insertNeighbor(b,a); }
    /// 从节点环中移除指定邻接点。
    void eraseNeighbor(int center,int removed) {
        check(); auto& ring = rings.at(center); const auto found = std::find(ring.begin(),ring.end(),removed);
        require(found != ring.end(),"missing divide/conquer edge to delete"); ring.erase(found);
    }
    /// 从两个节点的邻接环中移除双向连接。
    void disconnect(int a,int b) { eraseNeighbor(a,b); eraseNeighbor(b,a); }
    struct Range { int begin,end; };
    /// 查找左右子剖分凸包的下公切线。
    std::pair<int,int> lowerTangent(Range lhs,Range rhs) const {
        int x = lhs.end, y = rhs.begin, z = first(y), z2 = predecessor(x,first(x));
        for(;;) {
            check();
            if(rightOf(x,y,z)) { const int next = successor(z,y); y = z; z = next; }
            else if(rightOf(x,y,z2)) { const int next = predecessor(z2,x); x = z2; z2 = next; }
            else return {x,y};
        }
    }
    /// 查找左右子剖分凸包的上公切线。
    std::pair<int,int> upperTangent(Range lhs,Range rhs) const {
        int x = lhs.end, y = rhs.begin, z1 = first(x), z2 = predecessor(y,first(y));
        for(;;) {
            check();
            if(leftOf(x,y,z2)) { const int next = predecessor(z2,y); y = z2; z2 = next; }
            else if(leftOf(x,y,z1)) { const int next = successor(z1,x); x = z1; z1 = next; }
            else return {x,y};
        }
    }
    /// 沿公切线合并左右 Delaunay 子剖分。
    void merge(Range lhs,Range rhs) {
        const auto lower = lowerTangent(lhs,rhs), upper = upperTangent(lhs,rhs);
        int l = lower.first, r = lower.second;
        while(l != upper.first || r != upper.second) {
            check(); connect(l,r);
            int r1 = predecessor(r,l); bool rightFinished = rightOf(l,r,r1);
            if(!rightFinished) for(;;) {
                check(); const int r2 = predecessor(r,r1);
                if(r2 < rhs.begin || outsideCircle(l,r,r1,r2)) break;
                disconnect(r,r1); r1 = r2;
                if(rightOf(l,r,r1)) { rightFinished = true; break; }
            }
            int l1 = successor(l,r); bool leftFinished = leftOf(r,l,l1);
            if(!leftFinished) for(;;) {
                check(); const int l2 = successor(l,l1);
                if(l2 > lhs.end || outsideCircle(r,l,l1,l2)) break;
                disconnect(l,l1); l1 = l2;
                if(leftOf(r,l,l1)) { leftFinished = true; break; }
            }
            if(rightFinished) l = l1;
            else if(leftFinished || outsideCircle(l,r,r1,l1)) r = r1;
            else l = l1;
        }
        connect(l,r); setFirst(upper.second,upper.first); setFirst(lower.first,lower.second);
    }
    /// 递归划分输入节点并合并局部三角剖分。
    Range triangulate(int begin,int end) {
        check(); const int size = end-begin+1;
        if(size == 2) { connect(begin,end); setFirst(begin,end); setFirst(end,begin); }
        else if(size == 3) {
            connect(begin,end); connect(begin,begin+1); connect(begin+1,end);
            if(rightOf(begin,end,begin+1)) {
                setFirst(begin,begin+1); setFirst(begin+1,end); setFirst(end,begin);
            } else {
                setFirst(begin,end); setFirst(begin+1,begin); setFirst(end,begin+1);
            }
        } else if(size > 3) {
            const int middle = (begin+end)>>1;
            const auto left = triangulate(begin,middle), right = triangulate(middle+1,end);
            merge(left,right);
        }
        return {begin,end};
    }
public:
    /// 保存输入点和控制参数，建立分治剖分工作状态。
    DivideAndConquerBuilder(const std::vector<InitialPoint>& input,const RunControl& run) : control(run) { mesh.sortedPoints = input; }
    /// 构造分治剖分并转换为初始三角网格。
    DivideAndConquerTriangulation build() {
        check(); const auto size = mesh.sortedPoints.size();
        if(size > control.maximumNodes || size > std::size_t(std::numeric_limits<int>::max()/2))
            throw OperationFailure{{MeshingStatus::BudgetExceeded,"divide/conquer position budget exceeded"}};
        for(const auto& p : mesh.sortedPoints) if(!std::isfinite(p.coordinates.x) || !std::isfinite(p.coordinates.y))
            throw OperationFailure{{MeshingStatus::InvalidInput,"nonfinite divide/conquer point"}};
        if(size < 3) return std::move(mesh);
        std::sort(mesh.sortedPoints.begin(),mesh.sortedPoints.end(),[](const auto& a,const auto& b) {
            return a.coordinates.x == b.coordinates.x ? a.coordinates.y < b.coordinates.y : a.coordinates.x < b.coordinates.x;
        });
        for(std::size_t i = 1; i < size; ++i)
            if(mesh.sortedPoints[i].coordinates.x == mesh.sortedPoints[i-1].coordinates.x &&
               mesh.sortedPoints[i].coordinates.y == mesh.sortedPoints[i-1].coordinates.y)
                throw OperationFailure{{MeshingStatus::InvalidInput,"duplicate divide/conquer coordinates require distinct native perturbation"}};
        rings.resize(size); triangulate(0,int(size)-1);
        for(const auto& ring : rings) require(!ring.empty(),"null adjacency after divide/conquer");
        int p = 0, next = first(0); mesh.hullPointCount = 1;
        while(next != 0 && mesh.hullPointCount < size) {
            check(); const int after = successor(next,p); p = next; next = after; ++mesh.hullPointCount;
        }
        require(next == 0,"divide/conquer hull does not close");
        // 原生输出按左到右的节点序，以及以 Pred 方向从环首扫描的序列。
        for(int i = 0; i < int(size); ++i) {
            const auto& ring = rings[i];
            for(std::size_t j = 0; j < ring.size(); ++j) {
                check(); const int b = ring[(ring.size()-j)%ring.size()], c = ring[(ring.size()-j-1)%ring.size()];
                if(b > i && c > i && rightOf(i,b,c)) mesh.triangles.push_back({i,b,c});
            }
        }
        const auto expected = 2*(size-1)-mesh.hullPointCount;
        require(mesh.triangles.size() == expected,"divide/conquer triangle count differs from native Euler allocation");
        check(); return std::move(mesh);
    }
};
} // namespace
/// 对输入 UV 节点执行分治 Delaunay 剖分。
OperationStatus buildDivideAndConquerTriangulation(const std::vector<InitialPoint>& input,
    DivideAndConquerTriangulation& output, const RunControl& control)
{
    try { auto mesh = DivideAndConquerBuilder(input,control).build(); output = std::move(mesh); return {}; }
    catch(const OperationFailure& failure) { return failure.status; }
    catch(const std::exception& error) { return {MeshingStatus::InvalidTopology,error.what()}; }
    catch(...) { return {MeshingStatus::InvalidTopology,"unknown divide/conquer failure"}; }
}
}
