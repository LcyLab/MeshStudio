#pragma once
/** @file SurfaceDomainState.h
 * @brief 面内 UV 域标签随局部编辑继承；初始化及最终验收与完整分类核对。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "mesh/data/Mesh2DTransaction.h"
#include <unordered_set>
namespace mesher {
class SurfaceDomainState {
    std::vector<bool> labels;
    std::unordered_set<std::uint64_t> boundary;
    bool editing=false;
public:
    /// 返回按三角形槽位排列的域标签。
    const std::vector<bool>& values() const { return labels; }
    /// 完整分类约束域，建立与三角形槽位对应的标签。
    bool initialize(const Mesh2D&,const std::vector<std::pair<int,int>>&,
                    std::string* error,MeshObservation observation={});
    /// 检查局部修改涉及的三角形是否具有一致域标签。
    bool uniform(const std::vector<int>& slots,std::string* error) const;
    /// 应用网格变化并校验局部域标签，失败时回滚。
    bool apply(const Mesh2D&,const MeshEditDelta&,std::string* error);
    /// 将当前域标签与完整重新分类的结果对照。
    bool verify(const Mesh2D&,const std::vector<std::pair<int,int>>&,
                std::string* error,MeshObservation observation={}) const;
    // 与拓扑事务配对；Lawson 每次翻边后消费新增报告，供下次翻边检查当前标签。
    // 不复制整片标签；异常或失败只恢复实际写过的槽位。
    class Edit {
        SurfaceDomainState& state;
        size_t originalSize,cursor=0;
        std::unordered_map<int,bool> undo;
        int inherited=-1,expectedMoveFrom=-1,expectedMoveTo=-1;
        bool createdGroup=false,finished=false;
        /// 在修改前保存原槽位的域标签。
        void touch(int slot);
    public:
        /// 建立域标签编辑范围；禁止复制以保持唯一回滚责任。
        explicit Edit(SurfaceDomainState&);
        /// 未提交时恢复原域标签和编辑状态。
        ~Edit();
        /// 建立域标签编辑范围；禁止复制以保持唯一回滚责任。
        Edit(const Edit&)=delete;
        /// 禁止复制域编辑对象，避免重复提交或回滚。
        Edit& operator=(const Edit&)=delete;
        /// 按局部网格变化顺序迁移三角形域标签。
        bool consume(const MeshEditDelta&,std::string* error);
        /// 检查受影响槽位的标签数量和局部域一致性。
        bool validate(const Mesh2D&,const std::vector<int>& changed,std::string* error) const;
        /// 提交域标签编辑并结束编辑状态。
        void commit();
    };
};
} // namespace mesher
