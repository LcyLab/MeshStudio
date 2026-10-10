// 文件职责：成对三角函数求值：Windows x64 使用独立 x87 实现，以保持球面前沿的数值兼容性。
#include "NativeSinCos.h"
#include <cmath>
#if defined(MESHSTUDIO_NATIVE_SINCOS_X64)
extern "C" /* 调用汇编实现的正弦和余弦联合计算接口。 */ void meshstudioAlgorithm6SinCos(double angle,double* sine,double* cosine);
#endif
namespace mesher::gmshReference {
/// 计算同一角度的正弦和余弦，供解析曲面求值使用。
SinCos evaluateNativeSinCos(double angle) {
    SinCos result;
#if defined(MESHSTUDIO_NATIVE_SINCOS_X64)
    if(std::isfinite(angle)) {
        meshstudioAlgorithm6SinCos(angle,&result.sine,&result.cosine);
        return result;
    }
#endif
    result.sine=std::sin(angle);result.cosine=std::cos(angle);return result;
}
}
