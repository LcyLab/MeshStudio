"""实际原生BDS2GMSH三角叶对照；输入BDS/原MVertex元数据冻结，未验证CAD生成驱动。"""
import argparse
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","out-dir"): parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api,args.library); pointer = ctypes.POINTER(ctypes.c_double)
    function = gmsh.lib.meshstudioReferenceBdsToFaceMesh
    function.argtypes = [ctypes.c_int,pointer,ctypes.c_int,pointer,ctypes.c_int,pointer,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    function.restype = ctypes.c_int
    report = {"scope":__doc__,"cases":[],"complete":False,
              "vertex_identity_comparison":"original physical IDs retained; new native pointers canonicalized in creation order, raw process-global MVertex numbers not compared",
              "library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal",0.); gmsh.model.add("bds-face-conversion")
        face = gmsh.model.occ.addRectangle(0.,0.,0.,4.,3.); gmsh.model.occ.synchronize()
        vertex = gmsh.model.getEntities(0)[0][1]; edge = gmsh.model.getEntities(1)[0][1]
        def original(id,dimension): return [id,dimension,vertex if dimension == 0 else edge if dimension == 1 else face,9.,8.,7.,0. if dimension == 0 else .37, .52 if dimension == 2 else 0.,.19 if dimension == 1 else -1.]
        base = [[1,0,1.,2.,3.,.1,.2],[2,0,4.,5.,6.,.7,.8],[3,0,7.,8.,9.,1.2,1.3]]
        cases = []
        for order in itertools.permutations((1,2,3)):
            suffix = "".join(map(str,order))
            cases.append(("create-"+suffix,base,[],[[*order,0]]))
            alias = [row[:] for row in base]; alias[0][1] = alias[1][1] = 10; alias[2][1] = 11
            cases.append(("identity-collapse-"+suffix,alias,[original(10,0),original(11,1)],[[*order,0]]))
            distinct = [row[:] for row in base]
            for i,row in enumerate(distinct): row[1] = 10+i
            cases.append(("same-xyz-distinct-"+suffix,distinct,[original(10,0),original(11,1),original(12,2)],[[*order,0]]))
        extended = [*base,[4,0,10.,11.,12.,1.7,1.8]]
        cases.append(("reuse-new-vertices",extended,[],[[1,2,3,0],[2,4,3,0]]))
        cases.append(("deleted-first-pool-record",extended,[],[[1,2,3,1],[1,3,4,0]]))
        create_collapsed = [row[:] for row in base]; create_collapsed[0][1] = create_collapsed[1][1] = 10
        cases.append(("create-before-collapsed-filter",create_collapsed,[original(10,0)],[[1,2,3,0]]))
        one_original = [row[:] for row in base]; one_original[0][1] = 10
        cases.append(("unused-original-does-not-shift-id",one_original,[original(10,0),original(1000,2)],[[1,2,3,0]]))
        negative = [row[:] for row in base]; negative[0][0] = -1
        cases.append(("unmapped-negative-local-id",negative,[],[[-1,2,3,0]]))
        cases.append(("empty-triangle-pool",base,[],[]))
        def array(rows):
            flat = [value for row in rows for value in row]
            return (ctypes.c_double*max(1,len(flat)))(*flat)
        for name,points,originals,triangles in cases:
            buffer = ctypes.create_string_buffer(16*1024*1024)
            count = function(face,array(points),len(points),array(originals),len(originals),array(triangles),len(triangles),buffer,len(buffer))
            if count < 0: raise ValueError("native face conversion leaf failed: "+name+": "+str(count))
            expected = json.loads(buffer.value.decode("utf-8")); frozen = directory/(name+"-input.tsv")
            rows = [[face,len(points),len(originals),len(triangles)],*points,*originals,*triangles]
            frozen.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in rows),encoding="utf-8")
            own = directory/(name+"-own.json"); subprocess.run([args.own_executable,str(frozen),str(own)],check=True)
            actual = json.loads(own.read_text(encoding="utf-8"))
            for document in (expected,actual):
                for row in document["vertices"]:
                    for k in range(5,11): row[k] = float(row[k])
            difference = first_trace_difference([{"stage":"bds_to_face_mesh",**expected}],[{"stage":"bds_to_face_mesh",**actual}],0.)
            (directory/(name+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
            report["cases"].append({"name":name,"triangles":count,"created":len(expected["conversion"]["created_local_nodes"]),
                                    "collapsed":len(expected["conversion"]["collapsed"]),"skipped_deleted":len(expected["conversion"]["skipped_deleted"]),
                                    "input_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest(),"first_difference":difference})
            if difference: raise ValueError("face conversion leaf divergence: "+name+": "+json.dumps(difference))
        if len(report["cases"]) != 24: raise ValueError("fixed face conversion leaf matrix must contain 24 cases")
        report["complete"] = True
    finally:
        gmsh.finalize(); (directory/"face-mesh-conversion-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases":len(report["cases"]),"native_bds_to_face_mesh_equivalent":report["complete"]}))


if __name__ == "__main__": main()
