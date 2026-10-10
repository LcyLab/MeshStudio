// 文件职责：反求数据适配：向纯数值模块提供 OCC 支撑面的范围、位置和导数；迭代由算法模块执行。
#include "OccParameterInverse.h"
using namespace mesher;
using namespace mesher::gmshReference;
/// 为曲面参数反求构造 OCC 求值回调。
SurfaceParameterInverseSource makeOccSurfaceParameterInverseSource(const OccCadGeometry& geometry,int face) {
    SurfaceParameterInverseSource source;
    if(!geometry.faceTrimParameterBounds(face,source.bounds))
        throw OperationFailure{{MeshingStatus::GeometryQueryFailed,"OCC inverse source has no finite original trim bounds"}};
    source.point=[&geometry,face](Point2D uv) { return geometry.faceSurfacePoint(face,uv); };
    source.derivatives=[&geometry,face](Point2D uv) {
        Point3D du,dv;geometry.faceSurfaceDerivatives(face,uv,du,dv);return std::array<Point3D,2>{{du,dv}};
    };
    return source;
}
