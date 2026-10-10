"""实际 DocRecord::MakeMeshWithPoints 对照；严格比较点排序和有向三角形输出序。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","out-dir"):
        parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api,args.library); ptr = ctypes.POINTER(ctypes.c_double)
    native = gmsh.lib.meshstudioReferenceDivideAndConquer
    native.argtypes = [ctypes.c_int,ptr,ptr,ptr,ptr,ctypes.c_int]; native.restype = ctypes.c_int
    report = {"scope":"actual native DocRecord sort/merge/triangle output; no CAD/bounding box/constraint recovery",
              "cases":[],"complete":False,"library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    cases = [("less-two",[[20,1.,0.],[21,0.,0.]]),("triangle",[[7,2.,0.],[8,0.,0.],[9,1.,1.]]),
             ("exact-square",[[0,0.,0.],[1,1.,0.],[2,1.,1.],[3,0.,1.]])]
    for seed in (1,17,20261007):
        for count in (5,17,64,257):
            rng = random.Random(seed); rows = [[i,rng.uniform(-1.,1.),rng.uniform(-1.,1.)] for i in range(count)]
            for scale in (1.e-8,1.,1.e8):
                cases.append((f"random-{seed}-{count}-{scale}",[[i,x*scale,y*scale] for i,x,y in rows]))
        rng = random.Random(seed)
        cases.append((f"near-line-{seed}",[[i,float(i),(rng.random()-.5)*1.e-10] for i in range(35)]))
        cases.append((f"grid-jitter-{seed}",[[7,float(i)+rng.random()*1.e-9,float(j)+rng.random()*1.e-9] for i in range(6) for j in range(7)]))
        cases.append((f"circle-jitter-{seed}",[[i,math.cos(i*.17)+rng.random()*1.e-9,math.sin(i*.17)+rng.random()*1.e-9] for i in range(36)]))
    try:
        gmsh.initialize([],readConfigFiles=False)
        try:
            if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong fixed version")
            gmsh.option.setNumber("General.Terminal",0.); gmsh.option.setNumber("General.NumThreads",1.)
            for name,rows in cases:
                capacity = max(16,4*len(rows)); count = len(rows)
                inputs = (ctypes.c_double*(count*3))(*(x for row in rows for x in row))
                sorted_rows = (ctypes.c_double*(capacity*3))(); triangles = (ctypes.c_double*(capacity*3))(); state = (ctypes.c_double*3)()
                n = native(count,inputs,sorted_rows,triangles,state,capacity)
                if n < 0: raise ValueError("native triangulation failed: "+name+" "+str(n))
                expected = {"hull_count":int(state[2]),"sorted_points":[[int(sorted_rows[3*i]),sorted_rows[3*i+1],sorted_rows[3*i+2]] for i in range(count)],
                            "triangles":[list(map(int,triangles[3*i:3*(i+1)])) for i in range(n)]}
                if any(min(row)<0 or max(row)>=count for row in expected["triangles"]): raise ValueError("native triangle indices outside input")
                source = directory/(name+"-input.tsv"); source.write_text(str(count)+"\n"+"".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in rows),encoding="utf-8")
                (directory/(name+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                target = directory/(name+"-own.json"); subprocess.run([args.own_executable,str(source),str(target)],check=True,timeout=30)
                actual = json.loads(target.read_text(encoding="utf-8"))
                for row in actual["sorted_points"]: row[1:] = map(float,row[1:])
                difference = first_trace_difference([{"stage":"divide_conquer",**expected}],[{"stage":"divide_conquer",**actual}],0.)
                report["cases"].append({"name":name,"points":count,"triangles":n,"input_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"first_difference":difference})
                if difference: raise ValueError("divide/conquer differs: "+json.dumps(difference))
        finally: gmsh.finalize()
        if len(report["cases"]) != 48: raise ValueError("fixed divide/conquer matrix must contain 48 cases")
        report["complete"] = True
    finally:
        (directory/"divide-conquer-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for h in handles: h.close()
    print(json.dumps({"cases":len(report["cases"]),"native_divide_conquer_equivalent":report["complete"]}))


if __name__ == "__main__": main()
