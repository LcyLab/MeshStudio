/** @file NativeSinCos.h
 * @brief 固定Windows64基线的成对三角函数求值；不读取网格或几何参考数据。
 * @date 2026-10-09
 */
#pragma once
namespace mesher::gmshReference {
struct SinCos {double sine=0,cosine=1;};
/// Windows MSVC x64使用与固定SDK相同的x87成对求值及大参数约减。
/// 其他平台使用标准库；不声明跨平台逐值一致，不改变浮点控制字。
SinCos evaluateNativeSinCos(double angle);
}
