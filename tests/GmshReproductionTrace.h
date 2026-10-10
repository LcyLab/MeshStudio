#pragma once
/** @file GmshReproductionTrace.h
 * @brief 固定 Gmsh 参考副本的只读 JSONL 观察器，仅由外部参考构建包含。
 *
 * 输入已计算的原生状态，不查询 CAD、不修改或排序算法容器。
 * 一次进程只运行一个串行案例；MESHSTUDIO_GMSH_TRACE 指定新输出文件。
 * 序列和实体 ID 用于定位首次分歧；尺寸为物理长度，UV 保留原分支。
 */
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <locale>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <utility>

#if defined(_WIN32)
#define MESHSTUDIO_REFERENCE_EXPORT __declspec(dllexport)
#else
#define MESHSTUDIO_REFERENCE_EXPORT __attribute__((visibility("default")))
#endif

namespace meshstudioGmshTrace {
inline std::ofstream& output()
{
    static std::ofstream stream;
    static const bool initialized = [&] {
        const char* path = std::getenv("MESHSTUDIO_GMSH_TRACE");
        if(path && *path) {
            stream.open(path, std::ios::out | std::ios::trunc);
            if(!stream) throw std::runtime_error("cannot open Gmsh reproduction trace");
            stream.imbue(std::locale::classic());
            stream << std::setprecision(17);
        }
        return true;
    }();
    (void)initialized;
    return stream;
}

inline std::ostream* begin(const char* stage, int entity)
{
    auto& stream = output();
    if(!stream.is_open()) return nullptr;
    if(!stream) throw std::runtime_error("Gmsh reproduction trace write failed");
    static std::size_t sequence = 0;
    stream << "{\"schema\":1,\"sequence\":" << sequence++
           << ",\"stage\":\"" << stage << "\",\"entity\":" << entity;
    return &stream;
}

inline void number(std::ostream& stream, double value)
{
    if(std::isfinite(value)) stream << value;
    else stream << (std::isnan(value) ? "\"nan\"" : (value > 0 ? "\"inf\"" : "\"-inf\""));
}

inline void end(std::ostream& stream)
{
    stream << "}\n";
    if(!stream) throw std::runtime_error("Gmsh reproduction trace write failed");
}

inline void recordValues(const char* stage, int entity,
                         std::initializer_list<std::pair<const char*, double>> values)
{
    auto* stream = begin(stage, entity);
    if(!stream) return;
    for(const auto& value : values) {
        *stream << ",\"" << value.first << "\":";
        number(*stream, value.second);
    }
    end(*stream);
}

template<class Samples>
void recordCurveSamples(const char* stage, int entity, const Samples& samples)
{
    auto* stream = begin(stage, entity);
    if(!stream) return;
    *stream << ",\"samples\":[";
    bool separator = false;
    for(const auto& sample : samples) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[';
        number(*stream, sample.t); *stream << ',';
        number(*stream, sample.lc); *stream << ',';
        number(*stream, sample.p); *stream << ',';
        number(*stream, sample.xp); *stream << ']';
    }
    *stream << ']';
    end(*stream);
}

template<class Face>
void recordStageMesh(const char* stage, const Face* face)
{
    auto* stream = begin(stage, face->tag());
    if(!stream) return;
    // 只排序独立观察副本；原生邻接、活动集合和节点容器保持原顺序。
    using Vertex = decltype(face->triangles.front()->getVertex(0));
    std::map<std::size_t, Vertex> vertices;
    *stream << ",\"triangles\":[";
    bool separator = false;
    for(auto* triangle : face->triangles) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[';
        for(int i = 0; i < 3; ++i) {
            auto* vertex = triangle->getVertex(i);
            vertices.emplace(vertex->getNum(), vertex);
            if(i) *stream << ',';
            *stream << vertex->getNum();
        }
        *stream << ']';
    }
    *stream << "],\"nodes\":[";
    separator = false;
    for(const auto& item : vertices) {
        if(separator) *stream << ',';
        separator = true;
        const auto* vertex = item.second;
        *stream << '[' << item.first << ',';
        number(*stream, vertex->x()); *stream << ',';
        number(*stream, vertex->y()); *stream << ',';
        number(*stream, vertex->z());
        *stream << ',' << (vertex->onWhat() ? vertex->onWhat()->dim() : -1)
                << ',' << (vertex->onWhat() ? vertex->onWhat()->tag() : -1) << ", [";
        double parameter;
        bool hasFirst = vertex->getParameter(0, parameter);
        if(hasFirst) number(*stream, parameter);
        if(vertex->getParameter(1, parameter)) {
            if(hasFirst) *stream << ',';
            number(*stream, parameter);
        }
        *stream << "]]";
    }
    *stream << ']';
    end(*stream);
}

template<class Triangle, class Data>
void recordFrontDecision(const char* stage, int face, Triangle* triangle,
                         int edge, Data& data, const double* uv, const double* metric)
{
    auto* stream = begin(stage, face);
    if(!stream) return;
    *stream << ",\"active_edge\":" << edge << ",\"normalized_radius\":";
    number(*stream, triangle->getRadius());
    *stream << ",\"uv\":["; number(*stream, uv[0]); *stream << ',';
    number(*stream, uv[1]); *stream << "],\"metric\":[";
    for(int i = 0; i < 3; ++i) {
        if(i) *stream << ',';
        number(*stream, metric[i]);
    }
    *stream << "],\"base_nodes\":[";
    for(int i = 0; i < 3; ++i) {
        if(i) *stream << ',';
        auto* vertex = triangle->tri()->getVertex(i);
        int index = data.getIndex(vertex);
        *stream << '[' << vertex->getNum() << ',';
        number(*stream, data.Us[index]); *stream << ',';
        number(*stream, data.Vs[index]); *stream << ',';
        number(*stream, data.vSizes[index]); *stream << ',';
        number(*stream, data.vSizesBGM[index]); *stream << ']';
    }
    *stream << ']';
    end(*stream);
}

template<class Cavity, class Shell, class Triangle>
void recordFrontCavity(int face, const Cavity& cavity, const Shell& shell, Triangle* containing)
{
    auto* stream = begin("front_cavity", face);
    if(!stream) return;
    *stream << ",\"shell_edges\":[";
    bool separator = false;
    for(const auto& edge : shell) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[' << edge._v(0)->getNum() << ',' << edge._v(1)->getNum()
                << ',' << edge.ori << ']';
    }
    *stream << "],\"triangles\":[";
    separator = false;
    for(const auto* triangle : cavity) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[';
        for(int i = 0; i < 3; ++i) {
            if(i) *stream << ',';
            *stream << triangle->tri()->getVertex(i)->getNum();
        }
        *stream << ']';
    }
    *stream << "],\"containing_triangle\":[";
    if(containing) for(int i = 0; i < 3; ++i) {
        if(i) *stream << ',';
        *stream << containing->tri()->getVertex(i)->getNum();
    }
    *stream << ']';
    end(*stream);
}

template<class Mesh>
void recordBdsMesh(const char* stage, int face, const Mesh& mesh)
{
    auto* stream = begin(stage, face);
    if(!stream) return;
    *stream << ",\"nodes\":[";
    bool separator = false;
    for(const auto* point : mesh.points) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[' << point->iD << ',';
        number(*stream, point->u); *stream << ',';
        number(*stream, point->v); *stream << ',';
        number(*stream, point->X); *stream << ',';
        number(*stream, point->Y); *stream << ',';
        number(*stream, point->Z); *stream << ']';
    }
    *stream << "],\"triangles\":[";
    separator = false;
    using Point = typename std::decay<decltype(*mesh.points.begin())>::type;
    for(const auto* triangle : mesh.triangles) {
        if(triangle->deleted) continue;
        Point nodes[4] = {};
        // 原生 getNodes/commonvertex 在非法连接上会写 Msg::Error；观察必须纯读。
        const auto common = [](const auto* first, const auto* second) -> Point {
            if(first->p1 == second->p1 || first->p1 == second->p2) return first->p1;
            if(first->p2 == second->p1 || first->p2 == second->p2) return first->p2;
            return nullptr;
        };
        nodes[0] = common(triangle->e1, triangle->e4 ? triangle->e4 : triangle->e3);
        nodes[1] = common(triangle->e1, triangle->e2);
        nodes[2] = common(triangle->e2, triangle->e3);
        if(triangle->e4) nodes[3] = common(triangle->e3, triangle->e4);
        if(separator) *stream << ',';
        separator = true;
        *stream << '[';
        for(int i = 0; i < (triangle->e4 ? 4 : 3); ++i) {
            if(i) *stream << ',';
            if(nodes[i]) *stream << nodes[i]->iD;
            else *stream << "null";
        }
        *stream << ']';
    }
    *stream << ']';
    end(*stream);
}

template<class Mesh>
void recordInitialMesh(const char* stage, int face, const Mesh& mesh)
{
    auto* stream = begin(stage, face);
    if(!stream) return;
    *stream << ",\"triangles\":[";
    bool separator = false;
    for(const auto* triangle : mesh.faces) {
        if(separator) *stream << ',';
        separator = true;
        *stream << '[' << triangle->data;
        auto* edge = triangle->he;
        for(int i = 0; i < 3; ++i, edge = edge->next) {
            *stream << ",[" << edge->v->data << ',';
            number(*stream, edge->v->position.x()); *stream << ',';
            number(*stream, edge->v->position.y()); *stream << ']';
        }
        *stream << ']';
    }
    *stream << ']';
    end(*stream);
}
} // namespace meshstudioGmshTrace
