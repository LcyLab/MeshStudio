"""实际 Gmsh OCCFace.closestPoint 与同 BREP 的生产适配器支撑面投影对照。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    function = gmsh.lib.meshstudioReferenceFaceProjection
    pointer = ctypes.POINTER(ctypes.c_double)
    function.argtypes = [ctypes.c_int,pointer,pointer]; function.restype = ctypes.c_int
    report = {"scope": "successful direct OCC support-surface projection, including bounded UV initialization and cache reuse",
              "query_points_from_reference_surface": True, "projection_values_from_reference": False,
              "generic_inverse_fallback_validated": False, "complete": False, "cases": [],
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE",None)
    try:
        for fixture in ("rectangle","hole","narrow_band","cylinder","sphere","torus","embedded"):
            gmsh.initialize([],readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong fixed reference version")
                gmsh.option.setNumber("General.Terminal",0.)
                gmsh.option.setNumber("General.NumThreads",1.)
                if gmsh.option.getNumber("Geometry.OCCUseGenericClosestPoint") != 0.:
                    raise ValueError("projection test requires fixed direct OCC mode")
                gmsh.model.add(fixture)
                cad = Path(args.frozen_dir)/(fixture+".brep")
                gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                queries,expected = [],{"queries":[]}
                for _,face in gmsh.model.getEntities(2):
                    low,high = gmsh.model.getParametrizationBounds(2,face)
                    for fu,fv in ((.13,.27),(.5,.5),(.87,.73)):
                        u = float(low[0]+(high[0]-low[0])*fu); v = float(low[1]+(high[1]-low[1])*fv)
                        on = list(map(float,gmsh.model.getValue(2,face,[u,v])))
                        for offset in (0.,.07):
                            point = [on[0]+.03*offset,on[1]-.02*offset,on[2]+offset]
                            for seed in ((0.,0.),(u,v)):
                                query = point+list(seed); output = (ctypes.c_double*8)()
                                if function(face,(ctypes.c_double*5)(*query),output) != 1:
                                    raise ValueError("actual native closestPoint failed")
                                queries.append([face,*query]); expected["queries"].append([face,*output])
                if not queries: raise ValueError("empty projection fixture")
                frozen = directory/(fixture+"-queries.tsv")
                frozen.write_text(str(len(queries))+"\n"+"".join(" ".join(format(x,".17g") for x in row)+"\n" for row in queries),encoding="utf-8")
                (directory/(fixture+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                own = directory/(fixture+"-own.json")
                subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True)
                actual = json.loads(own.read_text(encoding="utf-8"))
                for row in actual["queries"]: row[1:] = map(float,row[1:])
                difference = first_trace_difference([{"stage":"occ_projection",**expected}],[{"stage":"occ_projection",**actual}],0.)
                report["cases"].append({"fixture":fixture,"queries":len(queries),
                                        "cad_sha256":hashlib.sha256(cad.read_bytes()).hexdigest(),
                                        "query_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest(),"first_difference":difference})
                if difference: raise ValueError("native projection divergence: "+json.dumps(difference))
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 7 or sum(c["queries"] for c in report["cases"]) != 108:
            raise ValueError("fixed projection matrix must contain seven BREP and 108 queries")
        report["complete"] = True
    finally:
        (directory/"occ-projection-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"fixtures":len(report["cases"]),"queries":sum(c["queries"] for c in report["cases"]),"direct_occ_projection_equivalent":report["complete"]}))


if __name__ == "__main__":
    main()
