"""Actual laplaceSmoothing on frozen unsmoothed Algorithm6 meshes; independent 0D/1D/seam topology/pcurves/projections, D1 and Value."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshSurfaceSmoothingComparison import reference_smoothing, validate_result
from GmshReproductionComparison import load_reference_api, first_trace_difference
from GmshBdsMeshConversionComparison import create_metric_fixture


def unsmoothed_face_mesh(gmsh,face):
    types,_,connectivity = gmsh.model.mesh.getElements(2,face)
    if list(types) != [2]:
        raise ValueError("first-order triangle-only smoothing oracle required")
    flat = list(map(int,connectivity[0])); triangles = [flat[i:i+3] for i in range(0,len(flat),3)]
    nodes = []
    for id in sorted(set(flat)):
        coordinates,parameters,dimension,entity = gmsh.model.mesh.getNode(id)
        available = [int(dimension >= 1 and len(parameters) >= 1),int(dimension == 2 and len(parameters) >= 2)]
        uv = [float(parameters[0]) if available[0] else 0.,float(parameters[1]) if available[1] else 0.]
        nodes.append([id,dimension,entity,*map(float,coordinates),*uv,*available,0])
    return nodes,triangles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","frozen-dir","out-dir"):
        parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(p) for p in args.runtime_dir]
    os.environ["PATH"] = os.pathsep.join(args.runtime_dir+[os.environ["PATH"]]); os.environ.pop("MESHSTUDIO_GMSH_TRACE",None)
    gmsh = load_reference_api(args.api,args.library)
    report = {"complete":False,"scope":__doc__,"cases":[],"input_triangles_frozen":True,"input_node_classification_XYZ_parameters_frozen":True,
              "raw_reparameterization_results_or_topology_frozen":False,"continuous_own_front_to_smoothing_validated":False,
              "full_cad_shape_acceptance":False,"generic_inverse_fallback_validated":False,
              "library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        warped = directory/"warped-bspline.brep"; create_metric_fixture(gmsh,"warped-bspline",warped)
        for fixture in ("rectangle","hole","narrow_band","cylinder","sphere","torus","embedded","warped-bspline"):
            cad = warped if fixture == "warped-bspline" else Path(args.frozen_dir)/(fixture+".brep")
            gmsh.initialize([],readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong fixed reference version")
                gmsh.option.setNumber("General.Terminal",0.)
                gmsh.model.add(fixture); gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                if fixture == "embedded":
                    points = [tag for dim,tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(dim,tag,[])) == [2.,1.5,0.]]
                    if len(points) != 1:
                        raise ValueError("frozen embedded point identity missing")
                    gmsh.model.mesh.embed(0,points,2,gmsh.model.getEntities(2)[0][1])
                for key,value in {"General.NumThreads":1.,"Mesh.Algorithm":6.,"Mesh.ElementOrder":1.,"Mesh.Smoothing":0.,
                                  "Mesh.MeshSizeMin":0.,"Mesh.MeshSizeMax":.55,"Mesh.MeshSizeFromCurvature":0.,
                                  "Mesh.MeshSizeFromPoints":0.,"Mesh.AlgorithmSwitchOnFailure":0.}.items():
                    gmsh.option.setNumber(key,value)
                gmsh.model.mesh.generate(2)
                inputs,expected,descriptions = [],[],[]
                summary = {"fixture":fixture,"count":0,"triangle_inputs":0,"raw_dimensions":[0,0,0],"visits":0,"eligible":0,"writes":0,"queries":0,
                           "failed_publications":0,"first_difference":None,"cad_sha256":hashlib.sha256(cad.read_bytes()).hexdigest()}
                report["cases"].append(summary)
                for _,face in gmsh.model.getEntities(2):
                    nodes,triangles = unsmoothed_face_mesh(gmsh,face)
                    for reverse in (False,True):
                        ordered = [list(reversed(t)) for t in reversed(triangles)] if reverse else triangles
                        for iterations,failed in ((0,0),(-1,0),(1,0),(2,0),(1,1),(1,2)):
                            result = reference_smoothing(gmsh,face,nodes,ordered,iterations,failed,classified=True)
                            validate_result(result,iterations); expected.append(result); descriptions.append([face,reverse,iterations,failed])
                            inputs.append([face,len(nodes),len(ordered),iterations,failed,*[x for row in nodes for x in row],*[x for row in ordered for x in row]])
                            summary["count"] += 1; summary["triangle_inputs"] += len(ordered)
                            for row in nodes:
                                summary["raw_dimensions"][row[1]] += 1
                frozen = directory/(fixture+"-input.tsv")
                frozen.write_text(str(len(inputs))+"\n"+"".join(" ".join(format(x,".17g") for x in row)+"\n" for row in inputs),encoding="utf-8")
                (directory/(fixture+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                own = directory/(fixture+"-own.json")
                subprocess.run([args.own_executable,str(cad),str(frozen),str(own),"--classified"],check=True,timeout=120)
                actual = json.loads(own.read_text(encoding="utf-8"))["cases"]
                if len(actual) != len(expected):
                    raise ValueError("classified smoothing case count differs")
                for index,(reference,result) in enumerate(zip(expected,actual)):
                    validate_result(result,descriptions[index][2])
                    difference = first_trace_difference([reference],[result],0.)
                    if difference:
                        summary["first_difference"] = {"index":index,"case":descriptions[index],"difference":difference}
                        raise ValueError("classified smoothing divergence: "+json.dumps(summary["first_difference"]))
                    for key in ("visits","eligible","writes","queries"):
                        summary[key] += reference[key]
                    summary["failed_publications"] += sum(int(t[2] and not t[10]) for e in reference["events"] for t in e["trials"])
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 8 or not sum(c["eligible"] for c in report["cases"]):
            raise ValueError("classified smoothing coverage incomplete")
        report["complete"] = True
    finally:
        (directory/"classified-smoothing-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles:
            handle.close()
    print(json.dumps({"cases":sum(c["count"] for c in report["cases"]),"visits":sum(c["visits"] for c in report["cases"]),
                      "queries":sum(c["queries"] for c in report["cases"]),"actual_classified_smoothing_equal":report["complete"]}))


if __name__ == "__main__":
    main()
