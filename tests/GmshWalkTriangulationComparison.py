"""实际 Walk/split 池及 GFaceInitialMesh(recover=0) 对照；CAD UV/1D 输入仍冻结。"""
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
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library); ptr = ctypes.POINTER(ctypes.c_double)
    native_walk = gmsh.lib.meshstudioReferenceWalkTriangulation
    native_walk.argtypes = [ctypes.c_int, ptr, ptr, ptr, ptr, ptr, ptr, ctypes.c_int]; native_walk.restype = ctypes.c_int
    native_initial = gmsh.lib.meshstudioReferenceOrdinaryInitialTriangulation
    native_initial.argtypes = [ctypes.c_int, ptr, ptr, ptr, ptr, ctypes.c_int]; native_initial.restype = ctypes.c_int
    copies = gmsh.lib.meshstudioReferenceInitialChart
    copies.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; copies.restype = ctypes.c_int
    uses = gmsh.lib.meshstudioReferenceFaceUses
    uses.argtypes = [ctypes.c_int, ptr, ctypes.c_int]; uses.restype = ctypes.c_int
    report = {"scope": "actual native pool identities, UV, face/vertex heads, half-edge cycles/opposites before recovery",
              "own_cad_adapter_validated": False, "constraint_recovery_validated": False,
              "walk_cases": [], "initial_cases": [], "deferred_periodic_faces": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}

    def compare(name, mode, rows, bounds=None, face=None):
        count = len(rows); capacity = 6+6*count
        vertices = (ctypes.c_double*(capacity*4))(); edges = (ctypes.c_double*(capacity*6))()
        faces = (ctypes.c_double*(capacity*2))(); state = (ctypes.c_double*3)()
        if mode == "walk":
            source_rows = (ctypes.c_double*(count*3))(*(x for row in rows for x in row))
            source_bounds = (ctypes.c_double*4)(*bounds)
            result = native_walk(count, source_rows, source_bounds, vertices, edges, faces, state, capacity)
        else:
            result = native_initial(face, vertices, edges, faces, state, capacity)
        if result < 0: raise ValueError("native initial pool failed: "+name+" "+str(result))
        nv, ne, nf = map(int, state)
        if (nv, ne, nf) != (count+4, 6*count+6, 2*count+2): raise ValueError("native initial pool count mismatch")
        expected = {"vertices": [[int(vertices[4*i]), vertices[4*i+1], vertices[4*i+2], int(vertices[4*i+3])] for i in range(nv)],
                    "half_edges": [list(map(int, edges[6*i:6*(i+1)])) for i in range(ne)],
                    "faces": [list(map(int, faces[2*i:2*(i+1)])) for i in range(nf)]}
        header = [mode, count]+(bounds if mode == "walk" else [])
        source = directory/(name+"-input.tsv")
        source.write_text(" ".join(str(x) if isinstance(x, (str, int)) else format(x, ".17g") for x in header)+"\n"+
                          "".join(" ".join(str(x) if isinstance(x, int) else format(x, ".17g") for x in row)+"\n" for row in rows), encoding="utf-8")
        (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
        target = directory/(name+"-own.json")
        subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
        actual = json.loads(target.read_text(encoding="utf-8"))
        for row in actual["vertices"]: row[1:3] = map(float, row[1:3])
        difference = first_trace_difference([{"stage": "ordinary_initial", **expected}], [{"stage": "ordinary_initial", **actual}], 0.)
        entry = {"name": name, "points": count, "faces": nf, "input_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "first_difference": difference}
        report["walk_cases" if mode == "walk" else "initial_cases"].append(entry)
        if difference: raise ValueError("ordinary initial divergence: "+json.dumps(difference))

    cases = [("empty", []), ("exact-diagonal", [[1, 0., 0.]]), ("two-symmetric", [[1, -.5, -.5], [2, .5, .5]]),
             ("exact-square", [[1, -.5, -.5], [2, .5, -.5], [3, .5, .5], [4, -.5, .5]]),
             ("exact-grid", [[1+i*7+j, i*.4-.8, j*.4-1.2] for i in range(5) for j in range(7)]),
             ("exact-line", [[i+1, i*.08-.8, 0.] for i in range(21)]),
             ("walk-negative-area-ties", [[i+1, x, y] for i, (x, y) in enumerate([(1., 2.), (-1., -2.), (1., -2.), (-1., 2.), (0., 0.), (.1, -.7), (-.7, .1)])])]
    for seed in (1, 17, 20261007):
        rng = random.Random(seed)
        for count in (5, 17, 64, 257):
            rows = [[i+1, rng.uniform(-1., 1.), rng.uniform(-1., 1.)] for i in range(count)]
            for scale in (1.e-8, 1., 1.e8):
                cases.append((f"random-{seed}-{count}-{scale}", [[i, x*scale, y*scale] for i, x, y in rows], [-2.*scale, -3.*scale, 2.*scale, 3.*scale]))
        cases.append((f"circle-{seed}", [[i+1, math.cos(i*.17), math.sin(i*.17)] for i in range(36)]))
        cases.append((f"near-line-{seed}", [[i+1, i*.04-.7, (rng.random()-.5)*1.e-10] for i in range(35)]))
        rows = [[i+1, i*.13-.7, math.sin(i*.31)] for i in range(12)]; rng.shuffle(rows)
        cases.append((f"insertion-order-{seed}", rows))
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        gmsh.initialize([], readConfigFiles=False)
        try:
            if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong fixed version")
            gmsh.option.setNumber("General.Terminal", 0.)
            for case in cases: compare(case[0], "walk", case[1], case[2] if len(case) == 3 else [-2., -3., 2., 3.])
        finally: gmsh.finalize()
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded"):
            for maximum in (.3, .8):
                for graded in (False, True):
                    gmsh.initialize([], readConfigFiles=False)
                    try:
                        gmsh.model.add(fixture)
                        cad = Path(args.frozen_dir)/(fixture+".brep")
                        gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize()
                        if fixture == "embedded":
                            embedding = [tag for dim, tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(dim, tag, [])) == [2., 1.5, 0.]]
                            face_entities = gmsh.model.getEntities(2)
                            if len(embedding) != 1 or len(face_entities) != 1: raise ValueError("ambiguous embedded fixture")
                            gmsh.model.mesh.embed(0, embedding, 2, face_entities[0][1])
                        for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6.,
                                            "Mesh.MeshSizeMax": maximum, "Mesh.MeshSizeFromPoints": 0.,
                                            "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1.}.items(): gmsh.option.setNumber(name, value)
                        if graded: gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: maximum*(.25+.6/(1.+x*x+y*y+z*z)))
                        gmsh.model.mesh.generate(1)
                        for _, face in gmsh.model.getEntities(2):
                            buffer = (ctypes.c_double*(65536*3))(); n = uses(face, buffer, 65536)
                            if n < 0: raise ValueError("native face uses failed")
                            if any(buffer[3*i] < 2 and buffer[3*i+2] for i in range(n)):
                                report["deferred_periodic_faces"].append({"fixture": fixture, "face": face, "maximum": maximum, "graded": graded}); continue
                            uv = (ctypes.c_double*(65536*9))(); count = copies(face, uv, 65536)
                            if count < 4: raise ValueError("unexpected ordinary initial early route")
                            rows = [[int(uv[9*i]), uv[9*i+1], uv[9*i+2]] for i in range(count)]
                            prefix = f"{fixture}-{maximum}-{'graded' if graded else 'constant'}-face{face}"
                            compare(prefix, "initial", rows, face=face)
                            report["initial_cases"][-1]["cad_sha256"] = hashlib.sha256(cad.read_bytes()).hexdigest()
                    finally: gmsh.finalize()
        if len(report["walk_cases"]) != 52 or len(report["initial_cases"]) != 24: raise ValueError("fixed ordinary matrix count mismatch")
        report["complete"] = True
    finally:
        (directory/"walk-triangulation-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        for h in handles: h.close()
    print(json.dumps({"walk_cases": len(report["walk_cases"]), "actual_initial_cases": len(report["initial_cases"]), "ordinary_initial_equivalent": report["complete"]}))


if __name__ == "__main__": main()
