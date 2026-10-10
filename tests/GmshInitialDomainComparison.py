"""实际 GFaceInitialMesh 的 Color 与后置 UV 扫描对照；不冻结标签/连接/访问/翻边输出。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def create_fixture(gmsh, name, path):
    gmsh.initialize([], readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0.); gmsh.model.add(name)
        if name == "embedded-line-pressure":
            gmsh.model.occ.addRectangle(0., 0., 0., 4., 3.)
            a = gmsh.model.occ.addPoint(1., 1.5, 0.); b = gmsh.model.occ.addPoint(3., 1.5, 0.)
            gmsh.model.occ.addLine(a, b)
            gmsh.model.occ.addPoint(2., 1.47, 0.); gmsh.model.occ.addPoint(2., 1.53, 0.)
        else:
            coordinates = ([[(0., 0.), (4., 0.), (4., 3.), (0., 3.)], [(1., 1.), (3., 1.), (3., 1.04), (1., 1.04)]]
                           if name == "slit-embedded-point" else
                           [[(0., 0.), (6., 0.), (6., 4.), (0., 4.)], [(1., 1.), (2., 1.), (2., 2.), (1., 2.)], [(4., 1.), (5., 1.), (5., 2.), (4., 2.)]])
            loops = []
            for polygon in coordinates:
                points = [gmsh.model.occ.addPoint(x, y, 0.) for x, y in polygon]
                edges = [gmsh.model.occ.addLine(points[i], points[(i+1)%4]) for i in range(4)]
                loops.append(gmsh.model.occ.addCurveLoop(edges))
            gmsh.model.occ.addPlaneSurface(loops)
            if name == "slit-embedded-point": gmsh.model.occ.addPoint(2., .97, 0.)
        gmsh.model.occ.synchronize(); gmsh.write(str(path))
    finally: gmsh.finalize()


def restore_embedding(gmsh, fixture):
    coordinates = {"embedded": [[2., 1.5, 0.]], "slit-embedded-point": [[2., .97, 0.]],
                   "embedded-line-pressure": [[2., 1.47, 0.], [2., 1.53, 0.]]}.get(fixture, [])
    if not coordinates: return
    faces = gmsh.model.getEntities(2)
    if len(faces) != 1: raise ValueError("ambiguous embedding face")
    for xyz in coordinates:
        vertices = [tag for dim, tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(dim, tag, [])) == xyz]
        if len(vertices) != 1: raise ValueError("ambiguous embedding point")
        gmsh.model.mesh.embed(0, vertices, 2, faces[0][1])
    if fixture == "embedded-line-pressure":
        lines = []
        for dim, tag in gmsh.model.getEntities(1):
            lo, hi = gmsh.model.getParametrizationBounds(dim, tag)
            mid = list(gmsh.model.getValue(dim, tag, [float((lo[0]+hi[0])*.5)]))
            if mid == [2., 1.5, 0.]: lines.append(tag)
        if len(lines) != 1: raise ValueError("ambiguous embedded line")
        gmsh.model.mesh.embed(1, lines, 2, faces[0][1])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library); ptr = ctypes.POINTER(ctypes.c_double)
    pipeline = gmsh.lib.meshstudioReferenceOrdinaryInitialPipeline
    pipeline.argtypes = [ctypes.c_int, ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ptr, ctypes.c_int, ctypes.c_int]; pipeline.restype = ctypes.c_int
    recovery = gmsh.lib.meshstudioReferenceOrdinaryConstraintRecovery
    recovery.argtypes = [ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ptr, ctypes.c_int, ctypes.c_int]; recovery.restype = ctypes.c_int
    copies = gmsh.lib.meshstudioReferenceInitialChart
    copies.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; copies.restype = ctypes.c_int
    uses = gmsh.lib.meshstudioReferenceFaceUses
    uses.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; uses.restype = ctypes.c_int
    report = {"scope": "actual ordinary GFaceInitialMesh Color and UV passes, including LIFO repeated visits, seeds, tags and pool rewrites",
              "own_cad_adapter_validated": False, "bds_conversion_validated": False, "bds_metric_legalization_validated": False,
              "frozen_inputs": ["1D mesh", "UV", "getNodeCopies order", "physical CAD constraint chains"],
              "cases": [], "deferred_periodic_faces": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    extra = {name: directory/(name+".brep") for name in ("slit-embedded-point", "embedded-line-pressure", "two-holes")}
    try:
        for name, path in extra.items(): create_fixture(gmsh, name, path)
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded", *extra):
            for maximum in (.3, .8):
                for graded in (False, True):
                    gmsh.initialize([], readConfigFiles=False)
                    try:
                        if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong fixed version")
                        gmsh.model.add(fixture); cad = extra.get(fixture, Path(args.frozen_dir)/(fixture+".brep"))
                        gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize(); restore_embedding(gmsh, fixture)
                        for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6., "Mesh.MeshSizeMax": maximum,
                                            "Mesh.MeshSizeFromPoints": 0., "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1.}.items(): gmsh.option.setNumber(name, value)
                        if fixture == "slit-embedded-point":
                            top = []
                            for dim, tag in gmsh.model.getEntities(1):
                                lo, hi = gmsh.model.getParametrizationBounds(dim, tag)
                                xyz = gmsh.model.getValue(dim, tag, [float((lo[0]+hi[0])*.5)])
                                if abs(xyz[0]-2.) < 1.e-12 and abs(xyz[1]-1.04) < 1.e-12: top.append(tag)
                            if len(top) != 1: raise ValueError("ambiguous upper slit curve")
                            gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: min(maximum*.18, .08) if dim == 1 and tag == top[0]
                                else maximum*(.25+.6/(1.+x*x+y*y+z*z)) if graded else maximum)
                        elif graded: gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: maximum*(.25+.6/(1.+x*x+y*y+z*z)))
                        gmsh.model.mesh.generate(1)
                        for _, face in gmsh.model.getEntities(2):
                            buffer = (ctypes.c_double*(65536*3))(); n = uses(face, buffer, 65536)
                            if n < 0: raise ValueError("native face uses failed")
                            if any(buffer[3*i] < 2 and buffer[3*i+2] for i in range(n)):
                                report["deferred_periodic_faces"].append({"fixture": fixture, "face": face, "maximum": maximum, "graded": graded}); continue
                            uv = (ctypes.c_double*(65536*9))(); count = copies(face, uv, 65536)
                            if count < 4: raise ValueError("unexpected native early route")
                            rows = [[int(uv[9*i]), uv[9*i+1], uv[9*i+2]] for i in range(count)]
                            capacity = 6+6*count; trace_capacity = capacity*128
                            vertices = (ctypes.c_double*(capacity*4))(); edges = (ctypes.c_double*(capacity*6))(); faces = (ctypes.c_double*(capacity*2))()
                            decisions = (ctypes.c_double*(capacity*7))(); crosses = (ctypes.c_double*trace_capacity)(); recovery_state = (ctypes.c_double*5)()
                            if recovery(face, vertices, edges, faces, decisions, crosses, recovery_state, capacity, trace_capacity) < 0: raise ValueError("native physical chains unavailable")
                            chains = [[int(vertices[4*int(decisions[7*i])]), int(vertices[4*int(decisions[7*i+1])]), int(decisions[7*i+2])]
                                      for i in range(int(recovery_state[3]))]
                            for stage in (2, 3):
                                visits = (ctypes.c_double*(trace_capacity*2))(); flips = (ctypes.c_double*(trace_capacity*2))(); state = (ctypes.c_double*9)()
                                if pipeline(face, stage, vertices, edges, faces, visits, flips, state, capacity, trace_capacity) < 0: raise ValueError("native Color/UV prefix failed")
                                nv, ne, nf = map(int, state[:3])
                                if (nv, ne, nf) != (count+4, 6*count+6, 2*count+2): raise ValueError("native pool counts changed")
                                expected = {"vertices": [[int(vertices[4*i]), vertices[4*i+1], vertices[4*i+2], int(vertices[4*i+3])] for i in range(nv)],
                                            "half_edges": [list(map(int, edges[6*i:6*(i+1)])) for i in range(ne)],
                                            "faces": [list(map(int, faces[2*i:2*(i+1)])) for i in range(nf)],
                                            "domain": {"seeds": list(map(int, state[4:6])), "visits": [[], []]},
                                            "uv": {"passes": [[] for _ in range(int(state[6]))], "limit": bool(state[8])}}
                                for i in range(int(state[3])): expected["domain"]["visits"][int(visits[2*i])].append(int(visits[2*i+1]))
                                for i in range(int(state[7])): expected["uv"]["passes"][int(flips[2*i])].append(int(flips[2*i+1]))
                                name = f"{fixture}-{maximum}-{'graded' if graded else 'constant'}-face{face}-{'domain' if stage == 2 else 'uv'}"
                                lines = [["domain" if stage == 2 else "uv", count, face]]+rows+[[len(chains)]]+chains
                                source = directory/(name+"-input.tsv")
                                source.write_text("".join(" ".join(str(x) if isinstance(x, (str, int)) else format(x, ".17g") for x in row)+"\n" for row in lines), encoding="utf-8")
                                (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
                                target = directory/(name+"-own.json"); subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
                                actual = json.loads(target.read_text(encoding="utf-8"))
                                for row in actual["vertices"]: row[1:3] = map(float, row[1:3])
                                difference = first_trace_difference([{"stage": "ordinary_domain_uv", **expected}], [{"stage": "ordinary_domain_uv", **actual}], 0.)
                                report["cases"].append({"name": name, "stage": stage, "points": count, "visits": int(state[3]), "uv_passes": int(state[6]),
                                    "uv_flips": int(state[7]), "native_pass_limit": bool(state[8]), "unknown_faces": sum(row[1] == -1 for row in expected["faces"]),
                                    "duplicate_visits": sum(len(v)-len(set(v)) for v in expected["domain"]["visits"]), "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                    "input_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "first_difference": difference})
                                if difference: raise ValueError(name+" domain/UV differs: "+json.dumps(difference))
                    finally: gmsh.finalize()
        if len(report["cases"]) != 72: raise ValueError("fixed ordinary Color/UV matrix count mismatch")
        report["actual_uv_flips"] = sum(c["uv_flips"] for c in report["cases"] if c["stage"] == 3)
        if report["actual_uv_flips"] == 0: raise ValueError("actual CAD UV matrix must exercise flips")
        report["complete"] = True
    finally:
        (directory/"initial-domain-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        for h in handles: h.close()
    print(json.dumps({"actual_domain_uv_cases": len(report["cases"]), "uv_flips": report.get("actual_uv_flips", 0), "domain_uv_equivalent": report["complete"]}))


if __name__ == "__main__": main()
