#pragma once
/** @file BdsTypes.h
 * @brief BDS注册分类与临时域对象身份；同tag/degree仍可能是不同原生指针。
 * @date 2026-10-08
 * @modified 2026-10-10
 */
namespace mesher::gmshReference {
// m.geom去重注册对象，与当前面的CLASS_F/CLASS_EXTERIOR/周期CLASS_E栈对象分开。
enum class BdsClassificationOrigin { Registered, FaceInterior, FaceExterior, FaceBoundary };
struct BdsClassification {
    int tag = 0, dimension = -1; // dimension=-1 对应原生空分类指针。
    BdsClassificationOrigin origin = BdsClassificationOrigin::Registered;
    /// 按维度、实体标签和来源比较分类，供有序容器使用。
    bool operator<(const BdsClassification& other) const {
        if(dimension != other.dimension) return dimension < other.dimension;
        return tag != other.tag ? tag < other.tag : origin < other.origin;
    }
};
}
