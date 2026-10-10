"""冻结MTri3叶状态，实际native isActive/compareTri3Ptr/forceRadius及set对照；未运行候选/空腔或实际插点失败。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def normalize_queue(document):
    document["limit"] = float(document["limit"])
    for operation in document["operations"]: operation["radii"] = list(map(float,operation["radii"]))
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","out-dir"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api,args.library); pointer = ctypes.POINTER(ctypes.c_double)
    function = gmsh.lib.meshstudioReferenceFrontQueueLeaf
    function.argtypes = [pointer,ctypes.c_int,ctypes.POINTER(ctypes.c_int),ctypes.c_int,ctypes.c_double,pointer,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; function.restype = ctypes.c_int
    report = {"scope":__doc__,"cases":[],"complete":False,"mtri3_radius_and_neighbors_frozen":True,
              "initial_pop_control_mirrored_from_native_source":True,"actual_insertion_failure_branch_validated":False,
              "library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),"own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    limit = .5*math.sqrt(2.)*1.; cases = []
    def triangle(ids,radius,deleted=0,neighbors=(-1,-1,-1)): return [*ids,radius,deleted,*neighbors]
    for radius in (-1.,0.,math.nextafter(0.,1.),math.nextafter(limit,0.),limit,math.nextafter(limit,1.),2.):
        for deleted_target in (0,1):
            for deleted_neighbor in (0,1):
                for seed in (0,77):
                    points = [triangle((1,2,3),2.,deleted_target,(1,1,1)),triangle((2,4,3),radius,deleted_neighbor)]
                    cases.append(("activity-"+str(len(cases)),points,[0,1],[[0,0,seed]]))
    points = [triangle((1,2,3),2.),triangle((4,5,6),1.),triangle((7,8,9),.3,neighbors=(0,0,0)),triangle((10,11,12),.1),triangle((13,14,15),2.)]
    cases.append(("scan-break-and-failure-local-update",points,[0,1,2,3],[[1,0,0],[2,0,0],[1,0,0],[2,1,1],[4,2,0],[4,3,0],[5,4,0],[4,4,0],[4,4,0],[1,0,0],[2,4,1]]))
    points = [triangle((4,5,6),limit),triangle((1,2,3),limit),triangle((7,8,9),.2),triangle((10,11,12),0.),triangle((13,14,15),2.)]
    cases.append(("stale-deleted-and-duplicate-registration",points,[1,0,2,3],[[3,1,1],[1,0,0],[1,0,0],[5,4,0],[5,4,0],[4,4,0],[4,4,0],[0,1,77]]))
    cases.append(("failure-reinsert-equal-key-rejection",[triangle((1,2,3),2.),triangle((3,2,1),0.)],[0,1],[[1,0,0],[2,0,0]]))
    cases.append(("stale-active-after-neighbor-replacement",[triangle((1,2,3),2.),triangle((4,5,6),2.)],[0,1],[[6,0,1],[1,0,0]]))
    cases.append(("empty",[],[],[[1,0,0]]))
    def array(rows):
        flat = [x for row in rows for x in row]; return (ctypes.c_double*max(1,len(flat)))(*flat)
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal",0.)
        for name,points,initial,operations in cases:
            # AllTris is already normalized by native construction in production; leaf input preserves that contract.
            initial = sorted(initial,key=lambda i:(-points[i][3],sorted(points[i][:3])))
            buffer = ctypes.create_string_buffer(16*1024*1024); initial_array = (ctypes.c_int*max(1,len(initial)))(*initial)
            result = function(array(points),len(points),initial_array,len(initial),limit,array(operations),len(operations),buffer,len(buffer))
            if result < 0: raise ValueError("native queue leaf failed: "+name+":"+str(result))
            expected = normalize_queue(json.loads(buffer.value.decode("utf-8")))
            rows = [[limit,len(points),len(initial),len(operations)],*points,*[[i] for i in initial],*operations]
            frozen = directory/(name+"-input.tsv"); frozen.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in rows),encoding="utf-8")
            own = directory/(name+"-own.json"); subprocess.run([args.own_executable,str(frozen),str(own)],check=True)
            actual = normalize_queue(json.loads(own.read_text(encoding="utf-8")))
            difference = first_trace_difference([{"stage":"front_queue",**expected}],[{"stage":"front_queue",**actual}],0.)
            (directory/(name+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
            report["cases"].append({"name":name,"initial_active":len(expected["initial_active"]),"operations":len(operations),"pops":len(expected["pops"]),"first_difference":difference,
                                    "input_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest()})
            if difference: raise ValueError("front queue divergence: "+name+":"+json.dumps(difference))
        if len(report["cases"]) != 61: raise ValueError("fixed queue leaf matrix count differs")
        report["complete"] = True
    finally:
        gmsh.finalize(); (directory/"front-queue-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases":len(report["cases"]),"native_front_queue_equivalent":report["complete"]}))


if __name__ == "__main__": main()
