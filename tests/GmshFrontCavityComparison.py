"""冻结前沿运行池，调用实际native圆/DFS/UV搜索/恢复；不执行实际插点提交。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def normalize_cavity(document):
    document["coordinates"][:2] = map(float,document["coordinates"][:2])
    for key in ("circles","maps"):
        for row in document[key]: row[1:] = map(float,row[1:])
    if document["mesh_size"] is not None: document["mesh_size"] = float(document["mesh_size"])
    return document


def reference_front_cavity(gmsh,face,nodes,triangles,all_ids,internal,worst,point,metric,force=False):
    native = gmsh.lib.meshstudioReferenceFrontCavityLeaf; p = ctypes.POINTER(ctypes.c_double)
    native.argtypes = [ctypes.c_int,p,ctypes.c_int,p,ctypes.c_int,ctypes.POINTER(ctypes.c_int),ctypes.c_int,p,ctypes.c_int,ctypes.c_int,p,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; native.restype = ctypes.c_int
    def array(rows):
        flat = [x for row in rows for x in row]; return (ctypes.c_double*max(1,len(flat)))(*flat)
    ids = (ctypes.c_int*max(1,len(all_ids)))(*all_ids); query = (ctypes.c_double*5)(*point,*metric); out = ctypes.create_string_buffer(16*1024*1024)
    code = native(face,array(nodes),len(nodes),array(triangles),len(triangles),ids,len(all_ids),array(internal),len(internal),worst,query,int(force),out,len(out))
    if code < 0: raise ValueError("native front cavity leaf failed:"+str(code))
    return normalize_cavity(json.loads(out.value.decode()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","frozen-dir","out-dir"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]; gmsh = load_reference_api(args.api,args.library)
    report = {"scope":__doc__,"complete":False,"cases":[],"runtime_state_frozen":True,"actual_insertion_feedback_validated":False,
              "library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),"own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    nodes = [[i+1,*uv,i+1,99.] for i,uv in enumerate(((0.,0.),(2.,0.),(0.,2.),(2.,2.)))]; triangles = [[1,2,3,2.,0,-1,-1,1],[2,4,3,2.,0,0,-1,-1]]
    cases = []
    for metric in ((1.,0.,1.),(4.,.3,1.),(0.,0.,0.)):
        for point in ((.5,.5),(1.5,1.5),(2.,2.),(3.,3.),(1.,1.),(math.nextafter(2.,0.),2.)):
            for worst in (0,1):
                for internal in ([],[[2,3]]): cases.append(("case-"+str(len(cases)),nodes,triangles,[0,1],internal,worst,point,metric,False))
    dead = [list(t) for t in triangles]; dead[1][4] = 1
    cases.append(("predeleted-neighbor",nodes,dead,[0,1],[],0,(.5,.5),(1.,0.,1.),False))
    cases.append(("search-enters-predeleted-seed",nodes,dead,[0,1],[],0,(2.,2.),(1.,0.,1.),False))
    unlinked = [list(t) for t in triangles]; unlinked[0][7] = -1
    for force in (False,True): cases.append(("unlinked-force"+str(force),nodes,unlinked,[0,1],[],0,(1.5,1.5),(1.,0.,1.),force))
    cycle = [list(t) for t in triangles]; cycle[1][6] = 0
    for force in (False,True): cases.append(("search-cycle-force"+str(force),nodes,cycle,[0,1],[],0,(3.,3.),(1.,0.,1.),force))
    degenerate = [[row[0],0.,0.,row[3],row[4]] for row in nodes]
    cases.append(("singular-map",degenerate,triangles,[0,1],[],0,(1.,1.),(1.,0.,1.),False))
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal",0.); gmsh.model.add("cavity-leaf"); gmsh.model.occ.importShapes(str(Path(args.frozen_dir)/"rectangle.brep"),highestDimOnly=False); gmsh.model.occ.synchronize(); face = gmsh.model.getEntities(2)[0][1]
        for name,nodes,triangles,all_ids,internal,worst,point,metric,force in cases:
            expected = reference_front_cavity(gmsh,face,nodes,triangles,all_ids,internal,worst,point,metric,force)
            frozen = directory/(name+"-input.tsv"); rows = [[len(nodes),len(triangles),len(all_ids),len(internal),worst,int(force),*point,*metric],*nodes,*triangles,*[[i] for i in all_ids],*internal]
            frozen.write_text("".join(" ".join(format(x,".17g") for x in row)+"\n" for row in rows),encoding="utf-8")
            (directory/(name+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
            own = directory/(name+"-own.json"); subprocess.run([args.own_executable,str(frozen),str(own)],check=True)
            actual = normalize_cavity(json.loads(own.read_text(encoding="utf-8")))
            difference = first_trace_difference([{"stage":"front_cavity",**expected}],[{"stage":"front_cavity",**actual}],0.)
            report["cases"].append({"name":name,"cavity":len(expected["cavity"]),"shell":len(expected["shell"]),"circle_tests":len(expected["circles"]),"containing":expected["containing"],"first_difference":difference})
            if difference: raise ValueError("native cavity divergence:"+name+":"+json.dumps(difference))
        if len(report["cases"]) != 79: raise ValueError("fixed front cavity matrix count differs")
        report["complete"] = True
    finally:
        gmsh.finalize(); (directory/"front-cavity-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases":len(report["cases"]),"native_front_cavity_equivalent":report["complete"]}))


if __name__ == "__main__": main()
