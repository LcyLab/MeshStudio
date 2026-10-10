// 文件职责：几何关系判定：先用 double 快速筛选，临近退化时使用浮点展开确定符号，供剖分和恢复复用。
#include "GeomPredicates.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace mesher {

namespace {
// 使用无误差加法和 FMA 乘积残差保存低位，避免消去误差改变谓词符号。
using Expansion = std::vector<double>;

/// 向浮点展开加入一个分量并保留加法残差。
Expansion grow(const Expansion& e,double value)
{
    Expansion result;
    result.reserve(e.size()+1);
    double q=value;
    for (double component:e) {
        const double sum=q+component;
        const double virtualComponent=sum-q;
        const double virtualQ=sum-virtualComponent;
        const double residual=(q-virtualQ)+(component-virtualComponent);
        if (residual!=0) result.push_back(residual);
        q=sum;
    }
    if (q!=0 || result.empty()) result.push_back(q);
    return result;
}

/// 将两数之差表示为保留低位残差的浮点展开。
Expansion difference(double a,double b)
{
    const double x=a-b;
    const double virtualB=a-x;
    const double virtualA=x+virtualB;
    const double tail=(a-virtualA)+(virtualB-b);
    Expansion result;
    if (tail!=0) result.push_back(tail);
    if (x!=0 || result.empty()) result.push_back(x);
    return result;
}

/// 将浮点展开按指定符号相加。
Expansion add(Expansion a,const Expansion& b,double sign=1)
{
    for (double value:b) a=grow(a,sign*value);
    return a;
}

/// 将浮点展开相乘并累计乘积残差。
Expansion multiply(const Expansion& a,const Expansion& b)
{
    Expansion result{0};
    for (double x:a) for (double y:b) {
        const double product=x*y;
        const double residual=std::fma(x,y,-product);
        if (residual!=0) result=grow(result,residual);
        result=grow(result,product);
    }
    return result;
}

/// 读取浮点展开最高有效分量的符号。
int expansionSign(const Expansion& e)
{
    for (auto i=e.rbegin();i!=e.rend();++i)
        if (*i!=0) return (*i>0)-(*i<0);
    return 0;
}

/// 计算向量叉积，确定有向关系。
Expansion cross(const Expansion& ax,const Expansion& ay,
                const Expansion& bx,const Expansion& by)
{
    return add(multiply(ax,by),multiply(ay,bx),-1);
}

/// 使用浮点展开确定二维点的方向符号。
int exactOrientation(const Point2D& a,const Point2D& b,const Point2D& c)
{
    return expansionSign(cross(difference(a.x,c.x),difference(a.y,c.y),
                               difference(b.x,c.x),difference(b.y,c.y)));
}

using ExactPoint3 = std::array<Expansion,3>;

/// 将三维相对坐标表示为浮点展开。
ExactPoint3 exactRelative(const Point3D& a,const Point3D& origin)
{
    return {difference(a.x,origin.x),difference(a.y,origin.y),difference(a.z,origin.z)};
}

/// 使用浮点展开计算三维混合积。
Expansion exactTriple(const ExactPoint3& a,const ExactPoint3& b,const ExactPoint3& c)
{
    Expansion result=multiply(a[0],cross(b[1],b[2],c[1],c[2]));
    result=add(std::move(result),multiply(a[1],cross(b[0],b[2],c[0],c[2])),-1);
    return add(std::move(result),multiply(a[2],cross(b[0],b[1],c[0],c[1])));
}

/// 使用浮点展开确定四点的三维方向符号。
int exactOrientation3(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d)
{
    return expansionSign(exactTriple(exactRelative(b,a),exactRelative(c,a),exactRelative(d,a)));
}

/// 计算三维向量的混合积。
double triple(const Point3D& a,const Point3D& b,const Point3D& c)
{
    return a.x*(b.y*c.z-b.z*c.y)-a.y*(b.x*c.z-b.z*c.x)+a.z*(b.x*c.y-b.y*c.x);
}

/// 计算混合积各乘积项绝对值之和，供误差界判断使用。
double tripleBound(const Point3D& a,const Point3D& b,const Point3D& c)
{
    return std::abs(a.x)*(std::abs(b.y*c.z)+std::abs(b.z*c.y))+
        std::abs(a.y)*(std::abs(b.x*c.z)+std::abs(b.z*c.x))+
        std::abs(a.z)*(std::abs(b.x*c.y)+std::abs(b.y*c.x));
}

/// 计算点相对指定原点的坐标。
Point3D relative(const Point3D& a,const Point3D& origin)
{
    return {a.x-origin.x,a.y-origin.y,a.z-origin.z};
}

// 原始 [x y z x*x+y*y+z*z 1] 行列式的负值：右手系四面体球内为正。
// 插点 SoS 的非零分支仍沿用此约定，不在这里归一朝向。
int sphereSign(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d,const Point3D& p)
{
    const Point3D v[4]={relative(a,p),relative(b,p),relative(c,p),relative(d,p)};
    double lift[4];
    for (int i=0;i<4;++i) lift[i]=v[i].x*v[i].x+v[i].y*v[i].y+v[i].z*v[i].z;
    const double det=lift[0]*triple(v[1],v[2],v[3])-lift[1]*triple(v[0],v[2],v[3])+
        lift[2]*triple(v[0],v[1],v[3])-lift[3]*triple(v[0],v[1],v[2]);
    const double magnitude=lift[0]*tripleBound(v[1],v[2],v[3])+lift[1]*tripleBound(v[0],v[2],v[3])+
        lift[2]*tripleBound(v[0],v[1],v[3])+lift[3]*tripleBound(v[0],v[1],v[2]);
    if (std::abs(det)>128*std::numeric_limits<double>::epsilon()*magnitude)
        return (det>0)-(det<0);
    ExactPoint3 e[4]={exactRelative(a,p),exactRelative(b,p),exactRelative(c,p),exactRelative(d,p)};
    Expansion lifts[4];
    for (int i=0;i<4;++i)
        lifts[i]=add(add(multiply(e[i][0],e[i][0]),multiply(e[i][1],e[i][1])),multiply(e[i][2],e[i][2]));
    Expansion exact=multiply(lifts[0],exactTriple(e[1],e[2],e[3]));
    exact=add(std::move(exact),multiply(lifts[1],exactTriple(e[0],e[2],e[3])),-1);
    exact=add(std::move(exact),multiply(lifts[2],exactTriple(e[0],e[1],e[3])));
    exact=add(std::move(exact),multiply(lifts[3],exactTriple(e[0],e[1],e[2])),-1);
    return expansionSign(exact);
}
}

/// 计算二维方向行列式的近似数值。
double orient2dDet(const Point2D& a, const Point2D& b, const Point2D& c)
{
    // 以 c 为原点，2x2 行列式：(a-c)×(b-c)
    const double acx = a.x - c.x, acy = a.y - c.y;
    const double bcx = b.x - c.x, bcy = b.y - c.y;
    return acx * bcy - acy * bcx;
}

/// 计算二维方向值，并在快速结果不可靠时使用浮点展开。
double orient2dValue(const Point2D& a, const Point2D& b, const Point2D& c)
{
    const double acx=a.x-c.x,acy=a.y-c.y,bcx=b.x-c.x,bcy=b.y-c.y;
    const double left=acx*bcy,right=acy*bcx;
    double determinant=left-right,magnitude;
    if (left>0) {
        if (right<=0) return determinant;
        magnitude=left+right;
    }
    else if (left<0) {
        if (right>=0) return determinant;
        magnitude=-left-right;
    }
    else return determinant;

    // Shewchuk epsilon 是单位舍入误差，即 numeric_limits::epsilon()/2。
    const double epsilon=.5*std::numeric_limits<double>::epsilon();
    const double firstBound=(3.+16.*epsilon)*epsilon*magnitude;
    if (determinant>=firstBound || -determinant>=firstBound) return determinant;

    // 复用本库无误差加法/FMA 乘积残差；先处理乘积，再处理坐标差值尾数。
    const Expansion products=cross({acx},{acy},{bcx},{bcy});
    determinant=0.;
    for (double component:products) determinant+=component;
    const double secondBound=(2.+12.*epsilon)*epsilon*magnitude;
    if (determinant>=secondBound || -determinant>=secondBound) return determinant;

    const Expansion exactAcx=difference(a.x,c.x),exactAcy=difference(a.y,c.y);
    const Expansion exactBcx=difference(b.x,c.x),exactBcy=difference(b.y,c.y);
    const auto tail=[](const Expansion& value) { return value.size()>1 ? value.front() : 0.; };
    const double acxTail=tail(exactAcx),acyTail=tail(exactAcy);
    const double bcxTail=tail(exactBcx),bcyTail=tail(exactBcy);
    if (acxTail==0. && acyTail==0. && bcxTail==0. && bcyTail==0.) return determinant;

    const double finalBound=(9.+64.*epsilon)*epsilon*epsilon*magnitude+
                           (3.+8.*epsilon)*epsilon*std::abs(determinant);
    determinant+=(acx*bcyTail+bcy*acxTail)-(acy*bcxTail+bcx*acyTail);
    if (determinant>=finalBound || -determinant>=finalBound) return determinant;
    const Expansion exact=cross(exactAcx,exactAcy,exactBcx,exactBcy);
    return exact.back();
}

/// 返回二维三点的方向符号，退化时返回零。
int orient2d(const Point2D& a, const Point2D& b, const Point2D& c)
{
    // 普通情况直接返回 double 符号；接近舍入误差界时才用展开，避免误判共线关系。
    const double d = orient2dDet(a, b, c);
    const double magnitude=std::abs((a.x-c.x)*(b.y-c.y))+
                           std::abs((a.y-c.y)*(b.x-c.x));
    if (std::abs(d)>8*std::numeric_limits<double>::epsilon()*magnitude)
        return (d > 0.0) - (d < 0.0);
    return exactOrientation(a,b,c);
}

/// 计算二维外接圆判定行列式的近似数值。
double incircleDet(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p)
{
    // 以 p 为原点；逆时针化：若 a,b,c 顺时针则交换 b,c（平移不变，用相对 p 的坐标即可）
    double ax = a.x - p.x, ay = a.y - p.y;
    double bx = b.x - p.x, by = b.y - p.y;
    double cx = c.x - p.x, cy = c.y - p.y;
    if ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax) < 0.0) {   // orient2d(a,b,c) < 0
        std::swap(bx, cx);
        std::swap(by, cy);
    }
    const double az = ax * ax + ay * ay;
    const double bz = bx * bx + by * by;
    const double cz = cx * cx + cy * cy;
    // 3x3 行列式展开
    return ax * (by * cz - bz * cy)
         - bx * (ay * cz - az * cy)
         + cx * (ay * bz - az * by);
}

/// 按三角形方向返回点相对外接圆的位置符号。
int incircleOriented(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p)
{
    const double ax=a.x-p.x,ay=a.y-p.y,bx=b.x-p.x,by=b.y-p.y;
    const double cx=c.x-p.x,cy=c.y-p.y;
    const double al=ax*ax+ay*ay,bl=bx*bx+by*by,cl=cx*cx+cy*cy;
    const double d=al*(bx*cy-by*cx)+bl*(cx*ay-cy*ax)+cl*(ax*by-ay*bx);
    const double magnitude=al*(std::abs(bx*cy)+std::abs(by*cx))+
        bl*(std::abs(cx*ay)+std::abs(cy*ax))+cl*(std::abs(ax*by)+std::abs(ay*bx));
    if (std::abs(d)>64*std::numeric_limits<double>::epsilon()*magnitude)
        return (d>0)-(d<0);
    const Expansion ex=difference(a.x,p.x),ey=difference(a.y,p.y);
    const Expansion fx=difference(b.x,p.x),fy=difference(b.y,p.y);
    const Expansion gx=difference(c.x,p.x),gy=difference(c.y,p.y);
    const Expansion ea=add(multiply(ex,ex),multiply(ey,ey));
    const Expansion eb=add(multiply(fx,fx),multiply(fy,fy));
    const Expansion ec=add(multiply(gx,gx),multiply(gy,gy));
    Expansion determinant=multiply(ea,cross(fx,fy,gx,gy));
    determinant=add(std::move(determinant),multiply(eb,cross(gx,gy,ex,ey)));
    determinant=add(std::move(determinant),multiply(ec,cross(ex,ey,fx,fy)));
    return expansionSign(determinant);
}

/// 判断点是否位于三角形外接圆内。
int incircle(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p)
{
    const int orientation=orient2d(a,b,c);
    return orientation ? orientation*incircleOriented(a,b,c,p) : 0;
}


/// 在指定线性度量下执行外接圆位置判定。
int incircleMetric(const Point2D& a,const Point2D& b,const Point2D& c,const Point2D& p,
                   double xx,double xy,double yy)
{
    const int orientation=orient2d(a,b,c);
    if (!orientation) return 0;
    const double ax=a.x-p.x,ay=a.y-p.y,bx=b.x-p.x,by=b.y-p.y,cx=c.x-p.x,cy=c.y-p.y;
    const auto lift=[&](double x,double y) { const double u=xx*x+xy*y,v=yy*y; return u*u+v*v; };
    const auto bound=[&](double x,double y) {
        const double u=std::abs(xx*x)+std::abs(xy*y),v=std::abs(yy*y); return u*u+v*v;
    };
    const double d=lift(ax,ay)*(bx*cy-by*cx)+lift(bx,by)*(cx*ay-cy*ax)+lift(cx,cy)*(ax*by-ay*bx);
    const double magnitude=bound(ax,ay)*(std::abs(bx*cy)+std::abs(by*cx))+
        bound(bx,by)*(std::abs(cx*ay)+std::abs(cy*ax))+bound(cx,cy)*(std::abs(ax*by)+std::abs(ay*bx));
    if (std::abs(d)>512*std::numeric_limits<double>::epsilon()*magnitude)
        return orientation*((d>0)-(d<0));
    const Expansion ex=difference(a.x,p.x),ey=difference(a.y,p.y);
    const Expansion fx=difference(b.x,p.x),fy=difference(b.y,p.y);
    const Expansion gx=difference(c.x,p.x),gy=difference(c.y,p.y);
    const auto exactLift=[&](const Expansion& x,const Expansion& y) {
        const Expansion u=add(multiply(x,{xx}),multiply(y,{xy})),v=multiply(y,{yy});
        return add(multiply(u,u),multiply(v,v));
    };
    Expansion determinant=multiply(exactLift(ex,ey),cross(fx,fy,gx,gy));
    determinant=add(std::move(determinant),multiply(exactLift(fx,fy),cross(gx,gy,ex,ey)));
    determinant=add(std::move(determinant),multiply(exactLift(gx,gy),cross(ex,ey,fx,fy)));
    return orientation*expansionSign(determinant);
}

/// 返回四点的三维方向符号，退化时返回零。
int orient3d(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d)
{
    // 四点接近共面时不能简单按固定 epsilon 置零，需回到输入 double 的精确符号。
    const Point3D ab=relative(b,a),ac=relative(c,a),ad=relative(d,a);
    const double det=triple(ab,ac,ad);
    if (std::abs(det)>32*std::numeric_limits<double>::epsilon()*tripleBound(ab,ac,ad))
        return (det>0)-(det<0);
    return exactOrientation3(a,b,c,d);
}

/// 计算四面体外接球判定行列式的近似数值。
double insphereDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p)
{
    // 以 p 为原点，4x4 行列式；右手系化（标准 orient3d(a,b,c,d) > 0）后 det>0 表示 p 在球内
    double ax = a.x - p.x, ay = a.y - p.y, az = a.z - p.z;
    double bx = b.x - p.x, by = b.y - p.y, bz = b.z - p.z;
    double cx = c.x - p.x, cy = c.y - p.y, cz = c.z - p.z;
    double dx = d.x - p.x, dy = d.y - p.y, dz = d.z - p.z;
    // 标准 orient3d(a,b,c,d)（平移不变，用相对 p 的坐标计算）：det[b-a, c-a, d-a]
    const double abx = bx - ax, aby = by - ay, abz = bz - az;
    const double acx = cx - ax, acy = cy - ay, acz = cz - az;
    const double adx = dx - ax, ady = dy - ay, adz = dz - az;
    if (abx * (acy * adz - acz * ady) - aby * (acx * adz - acz * adx)
        + abz * (acx * ady - acy * adx) < 0.0) {
        std::swap(bx, cx); std::swap(by, cy); std::swap(bz, cz);
    }
    const double aw = ax * ax + ay * ay + az * az;
    const double bw = bx * bx + by * by + bz * bz;
    const double cw = cx * cx + cy * cy + cz * cz;
    const double dw = dx * dx + dy * dy + dz * dz;
    // 4x4 行列式按第一行展开；取负使约定为"p 在球内 > 0"（a,b,c,d 右手系）。
    // 原始行列式对右手系四面体在 p 球内时为负（与 Shewchuk 的逆序参数约定一致）。
    return -(ax * (by * (cz * dw - cw * dz) - bz * (cy * dw - cw * dy) + bw * (cy * dz - cz * dy))
         - bx * (ay * (cz * dw - cw * dz) - az * (cy * dw - cw * dy) + aw * (cy * dz - cz * dy))
         + cx * (ay * (bz * dw - bw * dz) - az * (by * dw - bw * dy) + aw * (by * dz - bz * dy))
         - dx * (ay * (bz * cw - bw * cz) - az * (by * cw - bw * cy) + aw * (by * cz - bz * cy)));
}

/// 判断点相对四面体外接球的位置。
int insphere(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p)
{
    const int orientation=orient3d(a,b,c,d);
    return orientation==0 ? 0 : orientation*sphereSign(a,b,c,d,p);
}

// ============================================================
// 符号扰动谓词（SoS 实用版）
// ============================================================

double orient3dDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d)
{
    // 标准约定：det[b-a, c-a, d-a]，>0 表示 a,b,c 逆时针看向 d 时 d 在正侧（右手系）
    const double abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const double acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    const double adx = d.x - a.x, ady = d.y - a.y, adz = d.z - a.z;
    return abx * (acy * adz - acz * ady)
         - aby * (acx * adz - acz * adx)
         + abz * (acx * ady - acy * adx);
}

/// 按给定误差尺度执行三维方向判定。
double orient3dEps(double scale)
{
    // det 量级 ~ scale^3；double 相对误差 ~ 2^-52 ≈ 2.2e-16，取安全系数 64。
    // 仅兼容旧调用点的尺度容差；精确谓词和 SoS 不使用它判断退化。
    const double s = (scale > 0.0) ? scale : 1.0;
    return 64.0 * 2.220446049250313e-16 * s * s * s;
}

/// 按给定误差尺度执行外接球位置判定。
double insphereEps(double scale)
{
    const double s = (scale > 0.0) ? scale : 1.0;
    return 64.0 * 2.220446049250313e-16 * s * s * s * s;
}

namespace {

// 独立的 2 的幂避免指数碰撞，取最低指数的非零系数决定符号。
// lifting 的负扰动优先于坐标的正扰动，统一以 -raw 保持插点符号约定。
// 全局 ID 必须互异，压缩排名保持单项式顺序。
template<size_t N>
int symbolicDeterminant(const std::array<Point3D,N>& points,const std::array<int,N>& ids,bool lifted)
{
    std::array<int,N> ranks{};
    for (size_t i=0;i<N;++i) for (size_t j=0;j<N;++j)
        if (ids[j]<ids[i]) ++ranks[i];
    std::array<std::array<Expansion,N>,N> matrix;
    std::array<std::array<uint64_t,N>,N> powers{};
    for (size_t i=0;i<N;++i) {
        matrix[i][0]={points[i].x}; matrix[i][1]={points[i].y}; matrix[i][2]={points[i].z};
        matrix[i][N-1]={1};
        for (size_t col=0;col<3;++col)
            powers[i][col]=uint64_t{1}<<((lifted ? N : 0)+3*ranks[i]+col);
        if (lifted) {
            matrix[i][3]=add(add(multiply(matrix[i][0],matrix[i][0]),multiply(matrix[i][1],matrix[i][1])),
                             multiply(matrix[i][2],matrix[i][2]));
            powers[i][3]=uint64_t{1}<<ranks[i];
        }
    }
    std::array<int,N> order{};
    for (size_t i=0;i<N;++i) order[i]=static_cast<int>(i);
    std::map<uint64_t,Expansion> coefficients;
    do {
        int sign=1;
        for (size_t i=0;i<N;++i) for (size_t j=i+1;j<N;++j)
            if (order[i]>order[j]) sign=-sign;
        for (unsigned mask=1;mask<(1u<<N);++mask) {
            bool valid=true;
            uint64_t exponent=0;
            Expansion coefficient{double(sign)};
            for (size_t row=0;row<N;++row) {
                const int col=order[row];
                if (mask&(1u<<row)) {
                    if (col==N-1) { valid=false; break; }
                    exponent+=powers[row][col];
                    if (lifted && col==3)
                        for (double& component:coefficient) component=-component;
                } else coefficient=multiply(coefficient,matrix[row][col]);
            }
            if (!valid) continue;
            auto inserted=coefficients.emplace(exponent,Expansion{0});
            inserted.first->second=add(std::move(inserted.first->second),coefficient);
        }
    } while (std::next_permutation(order.begin(),order.end()));
    for (const auto& term:coefficients) {
        const int sign=expansionSign(term.second);
        if (sign) return sign;
    }
    return 0; // 重复全局 ID 不满足 SoS 的输入约定。
}

/// 在方向恰好退化时按顶点身份作符号扰动判定。
int sosTieOrient(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d,
                 int ia,int ib,int ic,int id)
{
    // det[x y z 1] = -det[b-a,c-a,d-a].
    return -symbolicDeterminant<4>({a,b,c,d},{ia,ib,ic,id},false);
}

/// 在外接球判定恰好退化时按顶点身份作符号扰动判定。
int sosTieInsphere(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d,const Point3D& p,
                   int ia,int ib,int ic,int id,int ip)
{
    const std::array<Point3D,5> points{{a,b,c,d,p}};
    const std::array<int,5> ids{{ia,ib,ic,id,ip}};
    std::array<int,5> order{{0,1,2,3,4}};
    std::sort(order.begin(),order.end(),[&](int left,int right) { return ids[left]<ids[right]; });
    for (const int row:order) {
        std::array<Point3D,4> minor;
        int next=0;
        for (int i=0;i<5;++i) if (i!=row) minor[next++]=points[i];
        // 原始 lifting 列余子式 (-1)^(row+3)*det[x y z 1]
        // = (-1)^row*orient3d；不要再次乘排列奇偶符号。
        const int cofactor=orient3d(minor[0],minor[1],minor[2],minor[3]);
        if (cofactor) return (row%2==0 ? 1 : -1)*cofactor;
    }
    return -symbolicDeterminant<5>(points,ids,true);
}

} // namespace

/// 执行三维方向判定，并用符号扰动消除精确退化。
int orient3dSoS(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d,
               int ia,int ib,int ic,int id)
{
    const int sign=orient3d(a,b,c,d);
    return sign ? sign : sosTieOrient(a,b,c,d,ia,ib,ic,id);
}

/// 执行外接球判定，并用符号扰动消除精确退化。
int insphereSoS(const Point3D& a,const Point3D& b,const Point3D& c,const Point3D& d,
               const Point3D& p,int ia,int ib,int ic,int id,int ip)
{
    // 非零分支保留原始行列式负号，不归一朝向；精确零保留 raw lifting tie。
    const int sign=sphereSign(a,b,c,d,p);
    return sign ? sign : sosTieInsphere(a,b,c,d,p,ia,ib,ic,id,ip);
}

} // namespace mesher
