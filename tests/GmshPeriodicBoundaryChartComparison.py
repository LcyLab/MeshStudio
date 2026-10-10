"""原生周期边界 BDS 初始化对照；重复环是跨环复用压力输入，不是完整面生成。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def read_loop(function, face, loop):
    buffer = (ctypes.c_double * (65536*7))()
    count = function(face, loop, buffer, 65536)
    if count < 0: raise ValueError("native exact loop input failed")
    curves = []
    for i in range(count):
        use, edge, seam, physical, u, v, branch = buffer[7*i:7*(i+1)]
        use = int(use); physical = int(physical); branch = int(branch)
        if use == len(curves): curves.append({"entity": int(edge), "seam": int(seam), "vertices": []})
        if use != len(curves)-1: raise ValueError("native curve use order changed")
        if branch == 1: curves[use]["vertices"].append([physical, u, v])
        elif branch == -1:
            row = curves[use]["vertices"][-1]
            if not curves[use]["seam"] or len(row) != 3 or row[0] != physical: raise ValueError("invalid native seam copy")
            row.extend([u, v])
        else: raise ValueError("invalid native pcurve branch")
    return curves


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    parser.add_argument("--cad-executable", help="also query UV/surface XYZ from the same BREP using production OccCadGeometry")
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    pointer = ctypes.POINTER(ctypes.c_double)
    loop_count = gmsh.lib.meshstudioReferencePeriodicLoopCount
    loop_count.argtypes = [ctypes.c_int]; loop_count.restype = ctypes.c_int
    loop_input = gmsh.lib.meshstudioReferencePeriodicLoopInput
    loop_input.argtypes = [ctypes.c_int, ctypes.c_int, pointer, ctypes.c_int]; loop_input.restype = ctypes.c_int
    chart_result = gmsh.lib.meshstudioReferencePeriodicBoundaryChart
    chart_result.argtypes = [ctypes.c_int, ctypes.c_int, pointer, ctypes.c_int, pointer, ctypes.c_int, pointer]
    chart_result.restype = ctypes.c_int
    if args.cad_executable:
        curve_metadata = gmsh.lib.meshstudioReferenceTrueBoundaryCurves
        curve_metadata.argtypes = [ctypes.c_int, pointer, ctypes.c_int]; curve_metadata.restype = ctypes.c_int
    report = {"scope": "actual native periodic boundary BDS including prior-loop reuse and counterparts",
              "own_cad_adapter_validated": False, "embedded_or_initial_triangulation_validated": False,
              "cases": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    if args.cad_executable:
        report.update(cad_cases=[], cad_geometry_values_from_reference=False,
                      cad_scope="independent pcurve UV, trimmed bounds and surface XYZ; frozen 1D mesh and CAD loop sequence",
                      cad_boundary_mesh_from_reference=True, cad_loop_order_from_reference=True,
                      production_exact_pcurve_capabilities_validated=False, full_geometry_adapter_validated=False,
                      cad_executable_sha256=hashlib.sha256(Path(args.cad_executable).read_bytes()).hexdigest())
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "torus", "embedded"):
            for graded in (0, 1):
                gmsh.initialize([], readConfigFiles=False)
                try:
                    if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                        raise ValueError("wrong fixed reference version")
                    if args.cad_executable and gmsh.option.getNumber("Geometry.ReparamOnFaceRobust") != 0.:
                        raise ValueError("direct pcurve test requires fixed default robust mode zero")
                    gmsh.model.add(fixture)
                    cad = Path(args.frozen_dir) / (fixture + ".brep")
                    gmsh.model.occ.importShapes(str(cad), highestDimOnly=False)
                    gmsh.model.occ.synchronize()
                    for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6.,
                                        "Mesh.MeshSizeMax": .3, "Mesh.MeshSizeFromPoints": 0.,
                                        "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1.25}.items():
                        gmsh.option.setNumber(name, value)
                    if graded: gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: min(lc, .12+.015*abs(x)+.005*abs(y)))
                    gmsh.model.mesh.generate(1)
                    nodes = []
                    for dim, entity in gmsh.model.getEntities():
                        ids, xyz, params = gmsh.model.mesh.getNodes(dim, entity, False, True)
                        for i, node in enumerate(ids):
                            if dim > 1: raise ValueError("unexpected surface node before chart")
                            nodes.append([int(node), *map(float, xyz[3*i:3*i+3]), dim, entity, float(params[i]) if dim else 0.])
                    for _, face in gmsh.model.getEntities(2):
                        number = loop_count(face)
                        if number <= 0: raise ValueError("empty native fixture edge loops")
                        edge_loops = [read_loop(loop_input, face, loop) for loop in range(number)]
                        if args.cad_executable:
                            metadata = (ctypes.c_double * (4096*4))()
                            metadata_count = curve_metadata(face, metadata, 4096)
                            if metadata_count <= 0: raise ValueError("native CAD topology metadata unavailable")
                            metadata_rows = [list(map(int, metadata[4*i:4*(i+1)])) for i in range(metadata_count)]
                            metadata_by_edge = {row[0]: row[1:] for row in metadata_rows}
                        lower, upper = gmsh.model.getParametrizationBounds(2, face)
                        for repeats in (1, 2, 3):
                            points = (ctypes.c_double * (65536*12))()
                            loops = (ctypes.c_double * (65536*5))()
                            state = (ctypes.c_double * 8)()
                            count = chart_result(face, repeats, points, 65536, loops, 65536, state)
                            if count <= 0: raise ValueError("native chart oracle failed: " + str(count))
                            vertices = []
                            for i in range(count):
                                row = list(points[12*i:12*(i+1)])
                                for k in (0, 1, 7, 8, 11): row[k] = int(row[k])
                                vertices.append(row)
                            expected = {"face": face, "position_count": int(state[0]), "maximum_id": int(state[1]),
                                        "bounds": list(state[2:6]), "vertices": vertices,
                                        "loops": [[] for _ in range(number*repeats)], "selections": [None]*(number*repeats)}
                            if int(state[7]) != number*repeats: raise ValueError("wrong native loop count")
                            for i in range(int(state[0])):
                                loop, position, local, tolerance, seam_first = loops[5*i:5*(i+1)]
                                loop = int(loop)
                                if int(position) != len(expected["loops"][loop]): raise ValueError("wrong native loop position")
                                expected["loops"][loop].append(int(local)); expected["selections"][loop] = [tolerance, int(seam_first)]
                            lines = [[face, len(nodes), number*repeats, count, *map(float, lower), *map(float, upper), state[6], .3, 1.25, graded]]
                            lines.extend(nodes)
                            for curves in edge_loops * repeats:
                                lines.append([len(curves)])
                                for curve in curves:
                                    lines.append([curve["entity"], curve["seam"], len(curve["vertices"])])
                                    lines.extend(curve["vertices"])
                            for row in vertices: lines.append(row[2:7])
                            prefix = f"{fixture}-face{face}-graded{graded}-repeat{repeats}"
                            frozen = directory / (prefix + "-input.tsv")
                            frozen.write_text("".join(" ".join(format(x, ".17g") if isinstance(x, float) else str(x)
                                                              for x in line) + "\n" for line in lines), encoding="utf-8")
                            (directory / (prefix + "-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
                            own = directory / (prefix + "-own.json")
                            subprocess.run([args.own_executable, str(frozen), str(own)], check=True)
                            actual = json.loads(own.read_text(encoding="utf-8"))
                            actual["bounds"] = list(map(float, actual["bounds"]))
                            for row in actual["vertices"]:
                                for k in (2, 3, 4, 5, 6, 9, 10): row[k] = float(row[k])
                            for row in actual["selections"]: row[0] = float(row[0])
                            difference = first_trace_difference(
                                [{"stage": "periodic_boundary_chart", "entity": face, **expected}],
                                [{"stage": "periodic_boundary_chart", "entity": face, **actual}], 0.)
                            report["cases"].append({"fixture": fixture, "face": face, "graded": graded,
                                                    "loop_repetitions": repeats, "native_face_loop_input": repeats == 1,
                                                    "position_count": int(state[0]), "unique_nodes": count,
                                                    "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                                    "input_sha256": hashlib.sha256(frozen.read_bytes()).hexdigest(),
                                                    "first_difference": difference})
                            if difference: raise ValueError("periodic BDS divergence: " + json.dumps(difference))
                            if args.cad_executable:
                                # No reference UV, surface XYZ, parameter ranges or UV bounds in this input.
                                cad_lines = [[face, len(nodes), number*repeats, state[6], .3, 1.25, graded], *nodes]
                                for curves in edge_loops * repeats:
                                    cad_lines.append([len(curves)])
                                    for curve in curves:
                                        orientation, seam, face_orientation = metadata_by_edge[curve["entity"]]
                                        if seam != curve["seam"]: raise ValueError("inconsistent native seam metadata")
                                        cad_lines.append([curve["entity"], seam, orientation, face_orientation, len(curve["vertices"])])
                                        cad_lines.extend([[row[0]] for row in curve["vertices"]])
                                cad_input = directory / (prefix + "-cad-input.tsv")
                                cad_input.write_text("".join(" ".join(format(x, ".17g") if isinstance(x, float) else str(x)
                                                                   for x in line) + "\n" for line in cad_lines), encoding="utf-8")
                                cad_output = directory / (prefix + "-cad-own.json")
                                subprocess.run([args.cad_executable, str(cad), str(cad_input), str(cad_output)], check=True)
                                actual_cad = json.loads(cad_output.read_text(encoding="utf-8"))
                                actual_cad["bounds"] = list(map(float, actual_cad["bounds"]))
                                for row in actual_cad["vertices"]:
                                    for k in (2, 3, 4, 5, 6, 9, 10): row[k] = float(row[k])
                                for row in actual_cad["selections"]: row[0] = float(row[0])
                                cad_difference = first_trace_difference(
                                    [{"stage": "periodic_boundary_chart_cad", "entity": face, **expected}],
                                    [{"stage": "periodic_boundary_chart_cad", "entity": face, **actual_cad}], 0.)
                                report["cad_cases"].append({"fixture": fixture, "face": face, "graded": graded,
                                                            "loop_repetitions": repeats, "native_face_loop_input": repeats == 1,
                                                            "input_sha256": hashlib.sha256(cad_input.read_bytes()).hexdigest(),
                                                            "first_difference": cad_difference})
                                if cad_difference: raise ValueError("own CAD periodic BDS divergence: " + json.dumps(cad_difference))
                finally:
                    gmsh.finalize()
        if len(report["cases"]) != 54: raise ValueError("fixed chart matrix must contain 54 cases")
        if args.cad_executable:
            if len(report["cad_cases"]) != 54: raise ValueError("fixed production CAD chart matrix must contain 54 cases")
            report["production_exact_pcurve_capabilities_validated"] = True
        report["complete"] = True
    finally:
        (directory / "periodic-boundary-chart-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        for handle in handles: handle.close()
    summary = {"cases": len(report["cases"]), "frozen_periodic_bds_equivalent": report["complete"]}
    if args.cad_executable:
        summary.update(cad_cases=len(report["cad_cases"]), production_cad_periodic_bds_equivalent=report["complete"])
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
