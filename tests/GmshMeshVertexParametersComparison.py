"""Actual MVertex.cpp candidate/single/edge routines vs independent BREP topology, pcurves and OCC projections."""
import argparse
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference
from GmshBdsMeshConversionComparison import create_metric_fixture


def reference_parameters(gmsh,face,nodes,pairs,direction,fail,on_surface):
    native = gmsh.lib.meshstudioReferenceMeshVertexParameters
    native.argtypes = [ctypes.c_int,ctypes.POINTER(ctypes.c_double),ctypes.c_int,ctypes.POINTER(ctypes.c_int),ctypes.c_int,
                       ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
    native.restype = ctypes.c_int
    rows = [x for row in nodes for x in row]; endpoints = [x for row in pairs for x in row]
    buffer = ctypes.create_string_buffer(16*1024*1024)
    code = native(face,(ctypes.c_double*len(rows))(*rows),len(nodes),(ctypes.c_int*len(endpoints))(*endpoints),len(pairs),direction,fail,on_surface,buffer,len(buffer))
    if code != 0:
        raise ValueError("actual mesh vertex parameters failed: "+str(code))
    return json.loads(buffer.value.decode())


def make_nodes(gmsh,face):
    nodes,descriptors = [],[]
    def add(dimension,entity,point,uv,available,label):
        nodes.append([len(nodes)+1,dimension,entity,*map(float,point),*map(float,uv),*available])
        descriptors.append(label)
    for _,entity in gmsh.model.getEntities(0):
        add(0,entity,gmsh.model.getValue(0,entity,[]),[0.,0.],[0,0],["cad-vertex",entity])
    excluded = []
    for _,entity in gmsh.model.getEntities(1):
        low,high = gmsh.model.getParametrizationBounds(1,entity); first,last = float(low[0]),float(high[0])
        derivative = gmsh.model.getDerivative(1,entity,[first+(last-first)*.4])
        if first == last or not any(float(v) != 0. for v in derivative):
            excluded.append(entity); continue
        for location in (.23,.71):
            t = first+(last-first)*location
            add(1,entity,gmsh.model.getValue(1,entity,[t]),[t,0.],[1,0],["edge-stored",entity,location])
        if gmsh.model.getType(1,entity) == "Unknown":
            # Sphere pole edges have pcurves but no 3D support: native exact-zero inverse raises
            # "OpenCASCADE curve is null in projection". Preserve the nonzero stored paths.
            excluded.append(entity)
        else:
            t = first+(last-first)*.43
            add(1,entity,gmsh.model.getValue(1,entity,[t]),[0.,0.],[1,0],["edge-exact-zero-inverse",entity])
    low,high = gmsh.model.getParametrizationBounds(2,face)
    for location in ((.2,.35),(.7,.65)):
        uv = [float(low[k]+(high[k]-low[k])*location[k]) for k in range(2)]
        point = gmsh.model.getValue(2,face,uv)
        add(2,face,point,uv,[1,1],["same-face-stored",*location])
        add(2,face,point,[0.,0.],[0,0],["same-face-without-parameters",*location])
        other = next((entity for _,entity in gmsh.model.getEntities(2) if entity != face),None)
        if other:
            add(2,other,point,[.4,.6],[1,1],["other-face-classification",other,*location])
    return nodes,descriptors,excluded


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","frozen-dir","out-dir"):
        parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(p) for p in args.runtime_dir]
    os.environ["PATH"] = os.pathsep.join(args.runtime_dir+[os.environ["PATH"]]); os.environ.pop("MESHSTUDIO_GMSH_TRACE",None)
    gmsh = load_reference_api(args.api,args.library)
    report = {"complete":False,"scope":__doc__,"cases":[],"topology_and_parameter_results_frozen":False,
              "node_classification_XYZ_stored_parameters_frozen":True,"generic_inverse_fallback_validated":False,
              "degenerate_edge_zero_inverse_validated":False,"full_prefix_smoothing_validated":False,
              "excluded_scope":"Unknown-support curves retain nonzero stored nodes; exact-zero inverse fails in native OCC. Zero-range/zero-derivative curves have no sampled 1D nodes.",
              "library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        warped = directory/"warped-bspline.brep"; create_metric_fixture(gmsh,"warped-bspline",warped)
        for fixture in ("rectangle","hole","cylinder","sphere","torus","warped-bspline"):
            cad = warped if fixture == "warped-bspline" else Path(args.frozen_dir)/(fixture+".brep")
            gmsh.initialize([],readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong reference version")
                gmsh.option.setNumber("General.Terminal",0.); gmsh.option.setNumber("General.NumThreads",1.)
                if gmsh.option.getNumber("Geometry.ReparamOnFaceRobust") != 0. or gmsh.option.getNumber("Geometry.OCCUseGenericClosestPoint") != 0.:
                    raise ValueError("reference must use fixed default direct OCC options")
                gmsh.model.add(fixture); gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                inputs,expected,descriptions = [],[],[]
                summary = {"fixture":fixture,"count":0,"node_calls":0,"pair_calls":0,"multi_candidate_nodes":0,"seam_single_rejections":0,"empty_candidate_nodes":0,
                           "excluded_degenerate_edges":[],"first_difference":None,"cad_sha256":hashlib.sha256(cad.read_bytes()).hexdigest()}
                report["cases"].append(summary)
                for _,face in gmsh.model.getEntities(2):
                    nodes,labels,excluded = make_nodes(gmsh,face); summary["excluded_degenerate_edges"] += [[face,e] for e in excluded]
                    pairs = list(itertools.product(range(len(nodes)),repeat=2))
                    for direction,fail,on_surface in itertools.product((1,-1),(0,1),(0,1)):
                        result = reference_parameters(gmsh,face,nodes,pairs,direction,fail,on_surface)
                        expected.append(result); descriptions.append([face,direction,fail,on_surface,labels])
                        inputs.append([face,len(nodes),len(pairs),direction,fail,on_surface,*[x for row in nodes for x in row],*[x for pair in pairs for x in pair]])
                        summary["count"] += 1; summary["node_calls"] += len(nodes); summary["pair_calls"] += len(pairs)
                        summary["multi_candidate_nodes"] += sum(len(r["all"]) > 1 for r in result["nodes"])
                        summary["empty_candidate_nodes"] += sum(not r["all"] for r in result["nodes"])
                        summary["seam_single_rejections"] += sum(not r["single"][0] for r in result["nodes"])
                frozen = directory/(fixture+"-input.tsv")
                frozen.write_text(str(len(inputs))+"\n"+"".join(" ".join(format(x,".17g") for x in row)+"\n" for row in inputs),encoding="utf-8")
                (directory/(fixture+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                own = directory/(fixture+"-own.json")
                subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True,timeout=60)
                actual = json.loads(own.read_text(encoding="utf-8"))["cases"]
                if len(actual) != len(expected):
                    raise ValueError("parameter case count differs")
                for index,(reference,result) in enumerate(zip(expected,actual)):
                    difference = first_trace_difference([reference],[result],0.)
                    if difference:
                        summary["first_difference"] = {"index":index,"case":descriptions[index],"difference":difference}
                        raise ValueError("raw mesh parameter divergence: "+json.dumps(summary["first_difference"]))
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 6 or not sum(c["multi_candidate_nodes"] for c in report["cases"]):
            raise ValueError("raw mesh parameter coverage incomplete")
        report["complete"] = True
    finally:
        (directory/"mesh-vertex-parameters-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles:
            handle.close()
    print(json.dumps({"cases":sum(c["count"] for c in report["cases"]),"node_calls":sum(c["node_calls"] for c in report["cases"]),
                      "pair_calls":sum(c["pair_calls"] for c in report["cases"]),"actual_raw_parameters_equal":report["complete"]}))


if __name__ == "__main__":
    main()
