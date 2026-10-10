"""实际Algorithm6 optimalPointFrontal/B候选叶对照；同BREP，冻结UV/XYZ/双尺寸，不验证空腔或插点反馈。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def parse_native_queries(raw):
    # MSVC ostream emits -nan(ind), which is a source diagnostic, not valid JSON.
    text = re.sub(r'(?<![\w"])(-?(?:nan(?:\([^)]*\))?|inf))(?![\w"])',r'"\1"',raw)
    return json.loads(text)


def reference_front_point(gmsh,face,edge,extension,rows):
    native = gmsh.lib.meshstudioReferenceFrontPoint; pointer = ctypes.POINTER(ctypes.c_double)
    native.argtypes = [ctypes.c_int,ctypes.c_int,ctypes.c_int,pointer,pointer,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; native.restype = ctypes.c_int
    flat = [x for row in rows for x in row]; values = (ctypes.c_double*38)(); buffer = ctypes.create_string_buffer(1024*1024)
    code = native(face,edge,extension,(ctypes.c_double*21)(*flat),values,buffer,len(buffer))
    if code != 38: raise ValueError("native front leaf failed:"+str(code))
    values = list(values); finite = all(math.isfinite(x) for x in values)
    return {"status":0 if finite else 6,"values":values,"queries":parse_native_queries(buffer.value.decode())}


def normalize_front_points(points):
    for item in points:
        result = item["candidate"]
        result["values"] = list(map(float,result["values"])); result["queries"] = [list(map(float,row)) for row in result["queries"]]
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    gmsh = load_reference_api(args.api, args.library)
    report = {"scope": __doc__, "cases": [], "complete": False, "triangle_state_frozen": True,
              "metric_and_surface_values_frozen": False, "full_prefix_integration_validated": False,
              "dynamic_insertion_feedback_validated": False,
              "nonfinite_candidate_trace_equivalent": False,
              "nonfinite_scope": "own NonConverged diagnostic only; native NaN arithmetic/401 surface query sequence retained as evidence but not reproduced",
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "torus", "embedded"):
            gmsh.initialize([], readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong fixed version")
                gmsh.option.setNumber("General.Terminal", 0.); gmsh.option.setNumber("General.NumThreads", 1.)
                gmsh.model.add(fixture); cad = Path(args.frozen_dir)/(fixture+".brep")
                gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize()
                inputs, expected, descriptors = [], [], []
                def collect(face,edge,extension,rows,descriptor):
                    expected.append(reference_front_point(gmsh,face,edge,extension,rows))
                    inputs.append([face,edge,extension,*[x for row in rows for x in row]]); descriptors.append(descriptor)
                for _, face in gmsh.model.getEntities(2):
                    low, high = gmsh.model.getParametrizationBounds(2,face)
                    # Unequal UV lengths and both orientations exercise all native edge numbers.
                    for location in ((.2,.25),(.6,.6)):
                        uv = [(float(low[0]+(high[0]-low[0])*u),float(low[1]+(high[1]-low[1])*v))
                              for u,v in (location,(location[0]+.17,location[1]),(location[0]+.04,location[1]+.13))]
                        points = [list(map(float,gmsh.model.getValue(2,face,p))) for p in uv]
                        for reverse in (False,True):
                            order = (0,2,1) if reverse else (0,1,2)
                            rows = [[*uv[k],*points[k],.18+.07*k,.11+.29*k] for k in order]
                            for edge in range(3):
                                for extension in (0,1):
                                    collect(face,edge,extension,rows,[face,*location,reverse,edge,extension])
                if fixture == "rectangle":
                    face = gmsh.model.getEntities(2)[0][1]
                    for name,normal in (("normal-below",math.nextafter(1.e-12,0.)),("normal-exact",1.e-12),("normal-above",math.nextafter(1.e-12,1.)),("normal-zero",0.)):
                        rows = [[0.,0.,0.,0.,0.,.5,2.],[2.,0.,2.,0.,0.,.5,2.],[1.,2.,1.,normal*.5,0.,.5,2.]]
                        for extension in (0,1): collect(face,1,extension,rows,[name,extension])
                    for name,height,size in (("zero-height",2.,0.),("negative-height",2.,-.5),("zero-direction",1.,.5)):
                        rows = [[0.,0.,0.,0.,0.,size,size],[2.,0.,2.,0.,0.,size,size],[1.,height,1.,height,0.,size,size]]
                        for extension in (0,1): collect(face,1,extension,rows,[name,extension])
                frozen = directory/(fixture+"-input.tsv")
                frozen.write_text(str(len(inputs))+"\n"+"".join(" ".join(format(x,".17g") for x in row)+"\n" for row in inputs),encoding="utf-8")
                own = directory/(fixture+"-own.json"); subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True)
                actual = json.loads(own.read_text(encoding="utf-8"))["cases"]
                if len(actual) != len(expected): raise ValueError("case count differs")
                summary = {"fixture":fixture,"queries":len(inputs),"finite_candidates":0,"nonfinite_candidates":0,"corrected":0,"retained":0,"surface_queries":0,"first_difference":None,
                           "cad_sha256":hashlib.sha256(cad.read_bytes()).hexdigest(),"input_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest()}
                for index,(reference,result) in enumerate(zip(expected,actual)):
                    result["values"] = list(map(float,result["values"])); result["queries"] = [list(map(float,row)) for row in result["queries"]]
                    if reference["status"] == 6:
                        summary["nonfinite_candidates"] += 1
                        difference = None if result["status"] == 6 else {"native_nonfinite":True,"own_status":result["status"]}
                    else:
                        summary["finite_candidates"] += 1; summary["corrected"] += int(reference["values"][33]); summary["retained"] += int(not reference["values"][33]); summary["surface_queries"] += len(reference["queries"])
                        difference = first_trace_difference([{"stage":"front_point",**reference}],[{"stage":"front_point",**result}],0.)
                    if difference:
                        summary["first_difference"] = {"index":index,"case":descriptors[index],"difference":difference}; break
                # JSON allow_nan false: retain source nonfinite values as diagnostic strings, never repaired.
                safe = lambda x: x if not isinstance(x,float) or math.isfinite(x) else str(x)
                (directory/(fixture+"-reference.json")).write_text(json.dumps([{**r,"values":list(map(safe,r["values"]))} for r in expected],allow_nan=False),encoding="utf-8")
                report["cases"].append(summary)
                if summary["first_difference"]: raise ValueError("native candidate divergence:"+json.dumps(summary["first_difference"]))
            finally: gmsh.finalize()
        if len(report["cases"]) != 7: raise ValueError("fixed matrix incomplete")
        report["complete"] = True
    finally:
        (directory/"front-point-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"fixtures":len(report["cases"]),"candidates":sum(c["queries"] for c in report["cases"]),
                      "finite_candidates_equivalent":report["complete"],"nonfinite_trace_equivalent":False}))


if __name__ == "__main__": main()
