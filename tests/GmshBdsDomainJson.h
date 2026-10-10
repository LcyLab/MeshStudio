#pragma once
/** @file GmshBdsDomainJson.h
 * @brief 测试用域标记/池清理快照；只序列化模块结果。
 * @date 2026-10-08
 */
#include "GmshBdsPoolJson.h"
#include "mesh/surface/algorithm6/bds/BdsDomainClassification.h"
#include "mesh/surface/algorithm6/bds/BdsInteriorPreparation.h"
inline void writeBdsDomainJson(std::ostream& out,const mesher::gmshReference::BdsMesh& mesh,
    const mesher::gmshReference::BdsDomainClassificationResult& result)
{
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true); out << ",\"fills\":[";
    for(std::size_t i = 0; i < result.fills.size(); ++i) {
        if(i) out << ','; const auto& f = result.fills[i];
        out << "{\"seed\":" << f.seed << ",\"classification\":[" << f.classification.tag << ',' << f.classification.dimension << ',' << int(f.classification.origin) << "],\"visits\":[";
        for(std::size_t j = 0; j < f.visits.size(); ++j) { if(j) out << ','; out << '[' << f.visits[j].first << ',' << (f.visits[j].second ? "true" : "false") << ']'; } out << "]}";
    }
    out << "],\"cleared_exterior\":[";
    for(std::size_t i = 0; i < result.clearedExteriorTriangles.size(); ++i) { if(i) out << ','; out << result.clearedExteriorTriangles[i]; }
    out << "],\"interior_triangles\":" << result.interiorTriangles << '}';
}
inline void writeBdsInteriorJson(std::ostream& out,const mesher::gmshReference::BdsMesh& mesh,
    const mesher::gmshReference::BdsInteriorPreparationResult& result)
{
    using namespace mesher::gmshReference;
    out << "{\"bds\":"; writeBdsPoolJson(out,mesh,true);
    auto list = [&](const std::vector<int>& values) { out << '['; for(std::size_t i = 0; i < values.size(); ++i) { if(i) out << ','; out << values[i]; } out << ']'; };
    out << ",\"unclassified_triangles\":"; list(result.unclassifiedTriangles);
    out << ",\"orphan_edges\":"; list(result.orphanEdges);
    out << ",\"removed_outer_points\":"; list(result.removedOuterPoints);
    auto mapping = [&](const char* name,const BdsPoolRemapping& m) { out << ",\"" << name << "\":{\"triangles\":"; list(m.triangles); out << ",\"edges\":"; list(m.edges); out << '}'; };
    mapping("first_cleanup",result.firstCleanup); mapping("second_cleanup",result.secondCleanup); out << '}';
}
