"""实际普通 PolyMesh 恢复前缀对照；保存原生穿越顺序/诊断/标签和池状态。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library); ptr = ctypes.POINTER(ctypes.c_double)
    native = gmsh.lib.meshstudioReferenceConstraintRecovery
    native.argtypes = [ctypes.c_int, ptr, ptr, ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ptr, ptr, ctypes.c_int, ctypes.c_int]; native.restype = ctypes.c_int
    ordinary = gmsh.lib.meshstudioReferenceOrdinaryConstraintRecovery
    ordinary.argtypes = [ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ptr, ctypes.c_int, ctypes.c_int]; ordinary.restype = ctypes.c_int
    intersection = gmsh.lib.meshstudioReferenceRecoveryIntersection
    intersection.argtypes = [ptr]; intersection.restype = ctypes.c_int
    copies = gmsh.lib.meshstudioReferenceInitialChart
    copies.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; copies.restype = ctypes.c_int
    uses = gmsh.lib.meshstudioReferenceFaceUses
    uses.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; uses.restype = ctypes.c_int
    report = {"scope": "actual recover_edge/getEdge and GFaceInitialMesh before Color; original crossing list, native codes, tags and all pools",
              "own_cad_adapter_validated": False, "physical_constraint_chains_frozen": True, "pool_endpoint_mapping_frozen": False, "domain_classification_validated": False,
              "intersection_cases": 0, "walk_cases": [], "ordinary_cases": [], "deferred_periodic_faces": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}

    def replay(name, lines, expected):
        source = directory/(name+"-input.tsv")
        source.write_text("".join(" ".join(str(x) if isinstance(x, (str, int)) else format(x, ".17g") for x in row)+"\n" for row in lines), encoding="utf-8")
        (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
        target = directory/(name+"-own.json")
        subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
        actual = json.loads(target.read_text(encoding="utf-8"))
        if "vertices" in actual:
            for row in actual["vertices"]: row[1:3] = map(float, row[1:3])
        difference = first_trace_difference([{"stage": "initial_constraint_recovery", **expected}], [{"stage": "initial_constraint_recovery", **actual}], 0.)
        if difference: raise ValueError(name+" recovery divergence: "+json.dumps(difference))
        return hashlib.sha256(source.read_bytes()).hexdigest()

    def compare(name, mode, rows, constraints=None, bounds=None, face=None):
        count = len(rows); capacity = max(6+6*count, len(constraints or [])); crossing_capacity = capacity*16
        vertices = (ctypes.c_double*(capacity*4))(); edges = (ctypes.c_double*(capacity*6))(); faces = (ctypes.c_double*(capacity*2))()
        decisions = (ctypes.c_double*(capacity*7))(); crossings = (ctypes.c_double*crossing_capacity)(); state = (ctypes.c_double*5)()
        if mode == "walk":
            p = (ctypes.c_double*(count*3))(*(x for row in rows for x in row)); b = (ctypes.c_double*4)(*bounds)
            c = (ctypes.c_double*(len(constraints)*3))(*(x for row in constraints for x in row))
            result = native(count, p, b, len(constraints), c, vertices, edges, faces, decisions, crossings, state, capacity, crossing_capacity)
        else:
            result = ordinary(face, vertices, edges, faces, decisions, crossings, state, capacity, crossing_capacity)
        if result < 0: raise ValueError("native recovery oracle failed: "+name+" "+str(result))
        nv, ne, nf, nd, nc = map(int, state)
        if (nv, ne, nf) != (count+4, 6*count+6, 2*count+2): raise ValueError("native pool count changed")
        expected = {"vertices": [[int(vertices[4*i]), vertices[4*i+1], vertices[4*i+2], int(vertices[4*i+3])] for i in range(nv)],
                    "half_edges": [list(map(int, edges[6*i:6*(i+1)])) for i in range(ne)],
                    "faces": [list(map(int, faces[2*i:2*(i+1)])) for i in range(nf)], "decisions": []}
        for i in range(nd):
            row = list(map(int, decisions[7*i:7*(i+1)])); start, length = row[5:7]
            if start < 0 or length < 0 or start+length > nc: raise ValueError("invalid native crossing trace")
            expected["decisions"].append(row[:5]+[list(map(int, crossings[start:start+length]))])
        native_constraints = [d[:3] for d in expected["decisions"]]
        if mode == "walk" and native_constraints != constraints: raise ValueError("native constraints reordered")
        input_constraints = native_constraints if mode == "walk" else [[int(vertices[4*a]), int(vertices[4*b]), tag] for a, b, tag in native_constraints]
        lines = [[mode, count]+(bounds if mode == "walk" else [])]+rows+[[nd]]+input_constraints
        input_sha = replay(name, lines, expected)
        report["walk_cases" if mode == "walk" else "ordinary_cases"].append({"name": name, "points": count, "constraints": nd,
            "actual_flipping_recoveries": sum(d[3] > 0 for d in expected["decisions"]), "crossings": nc,
            "native_failures": [d[:5] for d in expected["decisions"] if d[3] < 0 or d[4] < 0], "input_sha256": input_sha, "first_difference": None})

    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        gmsh.initialize([], readConfigFiles=False)
        try:
            if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong fixed version")
            gmsh.option.setNumber("General.Terminal", 0.)
            segments = [[-1., 0., 1., 0., 0., -1., 0., 1.], [-1., 0., 1., 0., 0., 0., 0., 1.],
                        [-1., 0., 1., 0., -.5, 0., .5, 0.], [-1., 0., 1., 0., -1., 1., 1., 1.],
                        [-1., -1., 1., 1., -1., 1., 1., -1.], [0., 0., 0., 0., -1., 1., 1., -1.]]
            for seed in (1, 17, 20261007):
                rng = random.Random(seed); segments.extend([[rng.uniform(-1., 1.) for _ in range(8)] for _ in range(50)])
            scaled = [[x*scale for x in row] for scale in (1.e-100, 1.e-8, 1., 1.e8, 1.e100) for row in segments]
            values = [intersection((ctypes.c_double*8)(*row)) for row in scaled]
            if any(x not in (0, 1) for x in values): raise ValueError("native strict intersection failed")
            replay("strict-intersection", [["intersection", len(scaled)]]+scaled, {"intersections": values}); report["intersection_cases"] = len(scaled)
            square = [[1, -.5, -.5], [2, .5, -.5], [3, .5, .5], [4, -.5, .5]]
            cases = [("existing-square", square, [[4, 5, 10], [5, 6, 11], [6, 7, 12], [7, 4, 13]]),
                     ("crossed-marked-diagonals", square, [[4, 6, 21], [5, 7, 22], [4, 6, 23]]),
                     ("native-self-failure-continues", square, [[4, 4, 30], [4, 5, 31]])]
            for seed in (1, 17, 20261007):
                for count in (9, 33, 97):
                    rng = random.Random(seed+count); rows = [[i+1, rng.uniform(-1., 1.), rng.uniform(-1., 1.)] for i in range(count)]
                    constraints = [[a+4, b+4, i+100] for i in range(9) for a, b in [rng.sample(range(count), 2)]]
                    constraints.append([constraints[-1][0], constraints[-1][1], 200])
                    for scale in (1.e-8, 1., 1.e8):
                        compare(f"random-{seed}-{count}-{scale}", "walk", [[i, x*scale, y*scale] for i, x, y in rows], constraints, [-2.*scale, -3.*scale, 2.*scale, 3.*scale])
                rows.reverse()
                cases.append((f"reversed-insertion-{seed}", rows, constraints))
            for name, rows, constraints in cases: compare(name, "walk", rows, constraints, [-2., -3., 2., 3.])
        finally: gmsh.finalize()
        # Nontrivial real CAD: a coarse slit edge is encroached on both sides.
        slit_cad = directory/"slit-embedded-point.brep"
        gmsh.initialize([], readConfigFiles=False)
        try:
            gmsh.option.setNumber("General.Terminal", 0.)
            loops = []
            for coordinates in (((0., 0.), (4., 0.), (4., 3.), (0., 3.)), ((1., 1.), (3., 1.), (3., 1.04), (1., 1.04))):
                points = [gmsh.model.occ.addPoint(x, y, 0.) for x, y in coordinates]
                edges = [gmsh.model.occ.addLine(points[i], points[(i+1)%4]) for i in range(4)]
                loops.append(gmsh.model.occ.addCurveLoop(edges))
            gmsh.model.occ.addPlaneSurface(loops); gmsh.model.occ.addPoint(2., .97, 0.)
            gmsh.model.occ.synchronize(); gmsh.write(str(slit_cad))
        finally: gmsh.finalize()
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded", "slit-embedded-point"):
            for maximum in (.3, .8):
                for graded in (False, True):
                    gmsh.initialize([], readConfigFiles=False)
                    try:
                        gmsh.model.add(fixture); cad = slit_cad if fixture == "slit-embedded-point" else Path(args.frozen_dir)/(fixture+".brep")
                        gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize()
                        if fixture in ("embedded", "slit-embedded-point"):
                            coordinate = [2., 1.5, 0.] if fixture == "embedded" else [2., .97, 0.]
                            embedding = [tag for dim, tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(dim, tag, [])) == coordinate]
                            face_entities = gmsh.model.getEntities(2)
                            if len(embedding) != 1 or len(face_entities) != 1: raise ValueError("ambiguous embedded fixture")
                            gmsh.model.mesh.embed(0, embedding, 2, face_entities[0][1])
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
                            if count < 4: raise ValueError("unexpected ordinary recovery early route")
                            rows = [[int(uv[9*i]), uv[9*i+1], uv[9*i+2]] for i in range(count)]
                            compare(f"{fixture}-{maximum}-{'graded' if graded else 'constant'}-face{face}", "chart", rows, face=face)
                            report["ordinary_cases"][-1]["cad_sha256"] = hashlib.sha256(cad.read_bytes()).hexdigest()
                    finally: gmsh.finalize()
        if report["intersection_cases"] != 780 or len(report["walk_cases"]) != 33 or len(report["ordinary_cases"]) != 28: raise ValueError("fixed recovery matrix count mismatch")
        report["actual_cad_flipping_recoveries"] = sum(c["actual_flipping_recoveries"] for c in report["ordinary_cases"])
        if report["actual_cad_flipping_recoveries"] == 0: raise ValueError("actual CAD matrix must exercise flipping recovery")
        report["complete"] = True
    finally:
        (directory/"constraint-recovery-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        for h in handles: h.close()
    print(json.dumps({"intersection_cases": report["intersection_cases"], "walk_cases": len(report["walk_cases"]),
                      "actual_ordinary_cases": len(report["ordinary_cases"]), "recovery_equivalent": report["complete"]}))


if __name__ == "__main__": main()
