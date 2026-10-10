/** @file GmshSurfaceParameterInverseCadReplay.cpp
 * @brief 同一BREP独立通用反求；参数和每次Value/D1请求逐值输出。
 * @date 2026-10-08
 */
#include "cad/occ/OccParameterInverse.h"
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    try {
        if(argc!=4) throw std::runtime_error("expected BREP, input and output paths");
        BRep_Builder builder;TopoDS_Shape shape;
        if(!BRepTools::Read(shape,argv[1],builder) || shape.IsNull()) throw std::runtime_error("cannot read frozen BREP");
        OccCadGeometry geometry(shape);
        if(!geometry.validationError().empty()) throw std::runtime_error(geometry.validationError());
        std::ifstream input(argv[2]);std::ofstream output(argv[3]);
        input.imbue(std::locale::classic());output.imbue(std::locale::classic());output << std::setprecision(17);
        if(!input || !output) throw std::runtime_error("cannot open inverse replay files");
        int count;if(!(input>>count) || count<0 || count>10000) throw std::runtime_error("invalid inverse replay count");
        output << "{\"cases\":[";
        for(int i=0;i<count;++i) {
            int face,onSurface,conv,newton;Point3D target;SurfaceParameterInverseOptions options;
            if(!(input>>face>>target.x>>target.y>>target.z>>options.modelLength>>options.relaxation>>onSurface>>conv>>newton) || face<1 || face>geometry.faceCount())
                throw std::runtime_error("invalid inverse replay row");
            options.onSurface=onSurface!=0;options.convergenceTestXYZ=conv!=0;options.newtonConvergenceTestXYZ=newton!=0;
            std::vector<SurfaceParameterInverseQuery> queries;
            RunControl control;control.maximumGeometryEvaluations=1000000;
            const auto result=invertSurfaceParameters(target,makeOccSurfaceParameterInverseSource(geometry,face-1),options,control,
                [&](const SurfaceParameterInverseQuery& query) { queries.push_back(query); });
            if(!result.status.complete()) throw std::runtime_error(result.status.message);
            if(i) output << ',';
            output << "{\"uv\":[" << result.parameters.x << ',' << result.parameters.y << "],\"stop\":" << int(result.stop)
                << ",\"passes\":" << result.relaxationPasses << ",\"seeds\":" << result.seeds << ",\"steps\":" << result.steps << ",\"queries\":[";
            for(std::size_t j=0;j<queries.size();++j) {
                if(j) output << ',';const auto& q=queries[j];
                output << '[' << q.kind << ',' << q.parameters.x << ',' << q.parameters.y;
                const int values=q.kind==0 ? 1 : 2;
                for(int k=0;k<values;++k) output << ',' << q.values[k].x << ',' << q.values[k].y << ',' << q.values[k].z;
                output << ']';
            }
            output << "]}";
        }
        std::string extra;if(input>>extra) throw std::runtime_error("unexpected inverse replay tail");
        output << "]}\n";if(!output) throw std::runtime_error("inverse replay write failed");return 0;
    }
    catch(const OperationFailure& error) { std::cerr << error.status.message << '\n';return 1; }
    catch(const Standard_Failure& error) { std::cerr << error.GetMessageString() << '\n';return 1; }
    catch(const std::exception& error) { std::cerr << error.what() << '\n';return 1; }
}
