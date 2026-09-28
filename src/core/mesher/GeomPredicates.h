#pragma once
// ============================================================
// 几何符号谓词（M0 核心模块）
// 职责：判断点的位置关系（在平面/圆/球的内外），只返回符号。
// 2D 原型使用 orient2d / incircle；orient3d / insphere 供 3D 阶段直接复用。
//
// 当前实现为 double 快速版。若退化输入下鲁棒性不足，将按
// Shewchuk(1997) 自适应精确谓词的思路升级实现，接口不变，调用方无感。
// 约定：所有谓词先做坐标平移（以最后一点为原点）再求行列式，
//       行列式量级 = 坐标^维度（2D:^2，3D:^3，圆/球:^4）。
// ============================================================

#include "MesherTypes.h"

namespace mesher {

/// a,b,c 三点关系：>0 逆时针（c 在 a→b 左侧），<0 顺时针，==0 共线
int orient2d(const Point2D& a, const Point2D& b, const Point2D& c);

/// p 相对三角形 a,b,c 外接圆：>0 在圆内，<0 在圆外，==0 共圆。
/// 内部先把 a,b,c 归一为逆时针，故朝向不影响结果。
int incircle(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p);

/// a,b,c,d 四点关系：>0 表示 d 位于有向面 (a,b,c) 的正侧（法向 (b-a)×(c-a) 指向侧）
int orient3d(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d);

/// p 相对四面体 a,b,c,d 外接球：>0 在球内，<0 在球外，==0 共球。
/// 内部先把 a,b,c,d 归一为右手系，故朝向不影响结果。
int insphere(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p);

// ============================================================
// 符号扰动谓词（SoS，M0.5）
// ------------------------------------------------------------
// 目的：共球/共面退化时，朴素 double 谓词返回 0（或受舍入污染的不定符号），
// 导致 Bowyer-Watson 剖分出现缺凸包面/内部点悬空等非法结构。
//
// 做法：
//   1) 先用带自适应容差的 double 谓词判定；符号确定（|det| > 误差界）时直接返回，
//      保证绝大多数情形零开销且与朴素谓词一致。
//   2) 仅在**判定为退化**（|det| <= 误差界）时，用**只依赖顶点索引**的确定性
//      扰动打破平局：
//        * insphereSoS 用「按索引升序依次沿 (1,1,1) 方向微移 t = scale·2^-40，
//          重算原始行列式，取首个非零符号」，最后乘置换符号；
//        * orient3dSoS 用「字典序 ε 展开」——枚举 4 元位移的编码，取最大合法
//          编码下首个非零符号。
//      两者都严格满足置换反对称性（实测 24/24、120/120 反号），且绝不返回 0。
//      索引作为唯一依据 → 同一组点无论以何顺序/朝向调用，结果一致 →
//      共享面两侧判定互相兼容，剖分保持流形。
//
// 与旧实现的区别（关键）：旧版在 Delaunay3D 里对**坐标**加 ±1e-8 随机扰动，
// 会把精确的凸包顶点打散（立方体棱被判为"非网格边"），且不可复现。
// 本实现**不改变任何坐标**，扰动只存在于谓词的符号决策中。
// ============================================================

/// 带符号扰动的 insphere：退化时按索引扰动打破平局，绝不返回 0。
int insphereSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                const Point3D& p, int ia, int ib, int ic, int id, int ip);

/// 带符号扰动的 orient3d：退化时按索引扰动打破平局，绝不返回 0。
int orient3dSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                int ia, int ib, int ic, int id);

/// 谓词误差界：用于判断 double 行列式是否可信（退化）。
/// scale 取参与点坐标的绝对量级上界，维度 3 → det 量级 ~ scale^3。
double orient3dEps(double scale);
double insphereEps(double scale);

// ---- 行列式值版本：测试与数值容差比较用，符号与上面的谓词一致 ----
double orient2dDet(const Point2D& a, const Point2D& b, const Point2D& c);
double incircleDet(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p);
double orient3dDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d);
double insphereDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p);

} // namespace mesher
