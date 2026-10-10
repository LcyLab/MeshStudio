#pragma once
// ============================================================
// 几何谓词（M0 核心模块）
// 职责：判断点的位置关系；符号接口与需要量级的数值接口分开。
// 2D 原型使用 orient2d / incircle；orient3d / insphere 供 3D 阶段直接复用。
//
// 2D/3D 符号判定使用 double 误差筛选，接近退化时用浮点展开回退；
// 以输入 double 的精确值为准，Det 接口仍返回 double 估计值。
// 适用条件：有限 IEEE-754 double、round-to-nearest，禁用 fast-math/FTZ；
// 差值、和、展开乘积及 FMA 残差不得上下溢，残差必须可表示。
// 仅“输入有限”不足以保证任意指数跨度准确；本模块不做任意精度回退。
// 行列式量级：orient2d:^2，orient3d:^3，incircle:^4，insphere:^5。
// ============================================================

#include "geometry/types/MesherTypes.h"

namespace mesher {

/// a,b,c 三点关系：>0 逆时针（c 在 a→b 左侧），<0 顺时针，==0 共线
int orient2d(const Point2D& a, const Point2D& b, const Point2D& c);

/// 自适应有向面积数值，与 orient2d 同号；供初始三角化 Walk 比较面积大小。
/// 对应 Gmsh robustPredicates::orient2d 的过滤/乘积残差/差值尾数次序。
/// 保留上述指数与舍入前提；原符号接口和非鲁棒 orient2dDet 语义不变。
double orient2dValue(const Point2D& a, const Point2D& b, const Point2D& c);

/// p 相对三角形 a,b,c 外接圆：>0 在圆内，<0 在圆外，==0 共圆。
/// 内部先把 a,b,c 归一为逆时针，故朝向不影响结果。
int incircle(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p);

/// 未归一朝向的圆内行列式符号，对应 Gmsh 初始翻边实际调用。
/// a,b,c 顺时针时圆内为负；三点共线仍返回行列式符号，不将其强制置零。
int incircleOriented(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p);

/// L=[xx xy;0 yy] 的度量圆判定，xx、yy>0；不先舍入映射后的点坐标。
/// 朝向仍按原始 UV；输入须有限，差值及所有展开中间乘积不允许上下溢。
int incircleMetric(const Point2D& a,const Point2D& b,const Point2D& c,const Point2D& p,
                   double xx,double xy,double yy);

/// a,b,c,d 四点关系：>0 表示 d 位于有向面 (a,b,c) 的正侧（法向 (b-a)×(c-a) 指向侧）
int orient3d(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d);

/// p 相对四面体 a,b,c,d 外接球：>0 在球内，<0 在球外，==0 共球。
/// 内部先把 a,b,c,d 归一为右手系，故朝向不影响结果。
/// 四面体精确共面时返回 0（没有唯一外接球）。
int insphere(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p);

// ============================================================
// 符号扰动谓词（SoS，M0.5）
// ------------------------------------------------------------
// 目的：共球/共面退化时，朴素 double 谓词返回 0（或受舍入污染的不定符号），
// 导致 Bowyer-Watson 剖分出现缺凸包面/内部点悬空等非法结构。
//
// 做法：先用 double 筛选 + 精确展开确认符号；仅精确零进入符号扰动。
//   orient3dSoS 用按全局索引排序的 x/y/z 独立无穷小；
//   insphereSoS 先扰动 lifting 列，用精确 orient3d 余子式定序；
//   若所有余子式为零，再取坐标与 lifting 扰动的首个非零多项式系数。
//   其统一模型为负 lifting 扰动、正坐标扰动后的 -raw 行列式；
//   普通共球的首项因此仍是原始 lifting 余子式符号。
// 幂次使用不同的 2 的幂，按系数的精确符号判定，不实际移动坐标。
// 各点全局索引必须互异；在上述数值适用条件内精确零 tie 恒非零且反对称。
//
// 与旧实现的区别（关键）：旧版在 Delaunay3D 里对**坐标**加 ±1e-8 随机扰动，
// 会把精确的凸包顶点打散（立方体棱被判为"非网格边"），且不可复现。
// 本实现**不改变任何坐标**，扰动只存在于谓词的符号决策中。
// ============================================================

/// 插点用 insphere：非零时保留 -raw 行列式符号（不归一朝向），
/// 精确零时保留 raw lifting 余子式 tie 约定；低秩使用同一扰动模型。
/// 互异全局索引下不返回 0。
int insphereSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                const Point3D& p, int ia, int ib, int ic, int id, int ip);

/// 带符号扰动的 orient3d：仅精确零按索引打破平局；索引必须互异。
int orient3dSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                int ia, int ib, int ic, int id);

/// 兼容旧调用点的尺度容差；不得据此认定精确共面/共球或触发 SoS。
/// orient3dEps ~ scale^3；insphereEps 保留旧 scale^4 估计，不是严格误差界。
double orient3dEps(double scale);
/// 按给定误差尺度执行外接球位置判定。
double insphereEps(double scale);

// ---- 行列式 double 估计值：测试/容差比较用；近退化时不保证精确符号 ----
double orient2dDet(const Point2D& a, const Point2D& b, const Point2D& c);
/// 计算二维外接圆判定行列式的近似数值。
double incircleDet(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p);
/// 计算三维方向行列式的近似数值。
double orient3dDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d);
/// 计算四面体外接球判定行列式的近似数值。
double insphereDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p);

} // namespace mesher
