/** @file GmshSingularMatrixInverse3Replay.cpp
 * @brief 独立数值重放；输出伪逆及SVD各因子供逐阶段诊断。
 * @date 2026-10-08
 */
#include "geometry/numerics/SingularMatrixInverse3.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
using namespace mesher::gmshReference;
int main(int argc,char** argv) {
    try {
        if(argc != 3) throw std::runtime_error("expected input and output paths");
        std::ifstream input(argv[1]); std::ofstream output(argv[2]);
        input.imbue(std::locale::classic()); output.imbue(std::locale::classic());
        if(!input || !output) throw std::runtime_error("cannot open numerical replay file");
        output << std::setprecision(17) << "{\"cases\":[";
        std::string line; int count=0;
        while(std::getline(input,line)) {
            if(line.empty()) continue;
            std::istringstream row(line); row.imbue(std::locale::classic()); Matrix3 matrix{};
            for(auto& values:matrix) for(double& value:values) if(!(row >> value)) throw std::runtime_error("invalid nine coefficient input");
            std::string extra; if(row >> extra) throw std::runtime_error("unexpected extra coefficient");
            const auto result=computeSingularMatrixPseudoInverse(matrix);
            if(!result.finiteInput || !result.converged || !result.finiteResult) throw std::runtime_error("singular inverse returned incomplete numerical result");
            if(count++) output << ',';
            output << "{\"sweeps\":" << result.sweeps << ",\"values\":["; int written=0;
            for(const auto* values:{&result.inverse,&result.leftVectors,&result.rightVectors})
                for(const auto& valuesRow:*values) for(double value:valuesRow) { if(written++) output << ','; output << value; }
            for(double value:result.singularValues) output << ',' << value;
            output << ',' << result.retainedSingularValues << "]}";
        }
        output << "]}\n"; return 0;
    }
    catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
