// CLI contract: malformed values must fail before CAD import or output creation.
#include <iostream>
#include <stdexcept>
#include "CadBenchmarkOptions.h"
int main() {
    const auto parse=[](std::vector<std::string> args,bool legacy=false) { return cadbench::parse(args,legacy); };
    const auto old=parse({"ball.brep"},true);
    if (old.criteria.targetSize!=20 || old.criteria.maxDeviation!=2 || old.criteria.maxIterations!=40000 || old.timeout!=30) return 1;
    const auto automatic=parse({"--cad","move.stp"});
    if (automatic.criteria.targetSize!=0 || automatic.timeout!=300 ||
        automatic.criteria.pointPlacement!=mesher::SurfacePointPlacement::LegacyRefinement ||
        old.criteria.pointPlacement!=mesher::SurfacePointPlacement::LegacyRefinement) return 1;
    for(bool audit:{false,true}) {
        try {
            if(!parse({"--cad","a","--cross-smooth","on"},audit).criteria.crossSmoothCadSplits ||
               parse({"--cad","a","--cross-smooth","off"},audit).criteria.crossSmoothCadSplits)return 1;
        } catch(const std::invalid_argument& error) {
            std::cerr<<"FAIL valid partition option rejected: "<<error.what()<<'\n';return 1;
        }
        for(const auto& args:std::vector<std::vector<std::string>>{
            {"--cad","a","--cross-smooth","yes"},{"--cad","a","--cross-smooth"},
            {"--cad","a","--cross-smooth","on","--cross-smooth","off"}}) {
            bool rejected=false;try{parse(args,audit);}catch(const std::invalid_argument&){rejected=true;}
            if(!rejected)return 1;
        }
        cadbench::Options legacy,frontal;
        try {
            legacy=parse({"--cad","a","--placement","legacy"},audit);
            frontal=parse({"--cad","a","--placement","frontal"},audit);
            const auto reference=parse({"--cad","a","--placement","reference"},audit);
            const auto physical=parse({"--cad","a","--placement","reference-physical"},audit);
            if(reference.criteria.pointPlacement==mesher::SurfacePointPlacement::LegacyRefinement||
                physical.criteria.pointPlacement==reference.criteria.pointPlacement)return 1;
        } catch(const std::invalid_argument& error) {
            std::cerr<<"FAIL valid benchmark/audit placement rejected: "<<error.what()<<'\n';return 1;
        }
        if(legacy.criteria.pointPlacement!=mesher::SurfacePointPlacement::LegacyRefinement ||
            frontal.criteria.pointPlacement!=mesher::SurfacePointPlacement::FrontalDelaunay) {
            std::cerr<<"FAIL benchmark/audit placement was not applied\n";return 1;
        }
        for(const auto& args:std::vector<std::vector<std::string>>{
            {"--cad","a","--placement","other"},{"--cad","a","--placement",""},
            {"--cad","a","--placement"},
            {"--cad","a","--placement","legacy","--placement","frontal"},
            {"--cad","a","--placement","frontal","--placement","frontal"}}) {
            bool rejected=false;try {parse(args,audit);}catch(const std::invalid_argument&){rejected=true;}
            if(!rejected){std::cerr<<"FAIL invalid/duplicate placement was accepted\n";return 1;}
        }
    }
    const auto specified=parse({"--cad","move.stp","--target","38.902828952788262","--min","3.8902828952788262",
        "--deviation","3.8902828952788262","--angle","30","--work","200000","--vertices","200000","--observe","on"});
    if (!specified.observe || specified.criteria.targetSize!=38.902828952788262) return 1;
    const auto maximum=parse({"--cad","a","--target","2","--max","1","--min","0.1"});
    if (maximum.criteria.maxSize!=1) return 1;
    if(parse({"--cad","a","--shape-sweeps","0"}).criteria.shapeOptimizationSweeps!=0 ||
        parse({"--cad","a","--shape-sweeps","4"}).criteria.shapeOptimizationSweeps!=4)return 1;
    for (const auto& args:std::vector<std::vector<std::string>>{
        {},{"--cad"},{"--cad","a","--unknown","1"},{"--cad","a","--shape-sweeps","-1"},
        {"--cad","a","--shape-sweeps","17"},{"--cad","a","--shape-sweeps","1.5"},{"--cad","a","--target","nan"},
        {"--cad","a","--target","inf"},{"--cad","a","--target","1junk"},
        {"--cad","a","--work","1.5"},{"--cad","a","--work","0"},
        {"--cad","a","--timeout","-1"},{"--cad","a","--target","1","--min","2"},
        {"--cad","a","--angle","181"},{"--cad","a","--observe","yes"},
        {"--cad","a","--max","nan"},{"--cad","a","--max","0"},{"--cad","a","--max","1","--min","2"},
        {"--cad","a","--cad","b"},{"--cad","a","--deviation","0"}}) {
        bool rejected=false; try { parse(args); } catch (const std::invalid_argument&) { rejected=true; }
        if (!rejected) { std::cerr<<"FAIL malformed benchmark arguments were accepted\n"; return 1; }
    }
    std::cout<<"PASS benchmark CLI validation and legacy audit defaults\n";
}
