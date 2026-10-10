#pragma once
// 文件职责：独立表面检查：查找三角形自交及几何接触；检查结果不参与网格生成或优化。

#include "mesh/surface/legacy/SurfaceMesh3D.h"
#include <functional>

namespace mesher {
struct SurfaceIntersection { int firstTriangle=-1,secondTriangle=-1; };

/// 检查三角面片穿透或共面重叠，允许沿共享节点/共享边接触。
/// tolerance=0 时按模型尺度与 double 分辨率确定几何容差；这是容差检查。
/// 失败时 error 包含相交三角形编号，供 CAD 面标签与未来体网格边界复用。
/// acceptedCadContact 可仅接受有 CAD 证据的跨部件接触；默认空时严格检查。
/// 接受接触不建立共享节点，也不保证多区域体网格边界符合性。
/// active 非空时只检查至少一片活动的相交对；调用方必须已验证未变片之间的全部对。
/// 掩码必须与三角形数一致；有限坐标、连接和退化检查仍覆盖全体。
bool validateSurfaceIntersections(const SurfaceMesh3D& surface,std::string* error,
                                  double tolerance=0,SurfaceIntersection* intersection=nullptr,
                                  const std::function<bool(int,int)>& acceptedCadContact={},
                                  const std::vector<bool>* active=nullptr);

}
