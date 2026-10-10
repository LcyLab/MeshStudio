"""冻结实际 getNodeCopies 的 UV 后对照普通 FaceChart；不代替自研 CAD 求值验收。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--own-executable", required=True)
    parser.add_argument("--frozen-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    array_pointer = ctypes.POINTER(ctypes.c_double)
    functions = {}
    for name in ("InitialChart", "FaceUses", "BoundaryRecovery"):
        function = getattr(gmsh.lib, "meshstudioReference" + name)
        function.argtypes = ([ctypes.c_int, ctypes.c_int, array_pointer, ctypes.c_int] if name == "BoundaryRecovery"
                             else [ctypes.c_int, array_pointer, ctypes.c_int])
        function.restype = ctypes.c_int; functions[name] = function
    properties = gmsh.lib.meshstudioReferenceBoundaryProperties
    properties.argtypes = [ctypes.c_int, array_pointer]; properties.restype = ctypes.c_int
    report = {"scope": "actual native getNodeCopies frozen UV replay and ordinary BDS input contract",
              "own_cad_adapter_validated": False, "cases": [], "deferred_periodic_faces": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded"):
            gmsh.initialize([], readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong fixed reference version")
                gmsh.model.add(fixture)
                cad = Path(args.frozen_dir) / (fixture + ".brep")
                gmsh.model.occ.importShapes(str(cad), highestDimOnly=False)
                gmsh.model.occ.synchronize()
                if fixture == "embedded":
                    faces = gmsh.model.getEntities(2)
                    vertices = [tag for dim, tag in gmsh.model.getEntities(0)
                                if list(gmsh.model.getValue(dim, tag, [])) == [2., 1.5, 0.]]
                    if len(faces) != 1 or len(vertices) != 1: raise ValueError("ambiguous embedded fixture")
                    gmsh.model.mesh.embed(0, vertices, 2, faces[0][1])
                for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6.,
                                     "Mesh.MeshSizeMax": .3, "Mesh.MeshSizeFromPoints": 0.,
                                     "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1.}.items():
                    gmsh.option.setNumber(name, value)
                gmsh.model.mesh.generate(1)
                nodes = {}
                for dim, entity in gmsh.model.getEntities():
                    ids, xyz, params = gmsh.model.mesh.getNodes(dim, entity, False, True)
                    for i, node in enumerate(ids):
                        if dim > 1: raise ValueError("unexpected surface node before chart replay")
                        nodes[int(node)] = [int(node), *map(float, xyz[3*i:3*i+3]), dim, entity,
                                            float(params[i]) if dim else 0., -1.]
                curves = []
                for _, entity in gmsh.model.getEntities(1):
                    state = (ctypes.c_double * 6)()
                    if not properties(entity, state): raise ValueError("native curve properties unavailable")
                    buffer = (ctypes.c_double * (4096*8))()
                    count = functions["BoundaryRecovery"](entity, -1, buffer, 4096)
                    if count < 0: raise ValueError("native boundary snapshot failed")
                    samples = []
                    for i in range(count):
                        row = list(buffer[8*i:8*(i+1)]); node = int(row[0]); samples.append([node, row[1]])
                        nodes[node][7] = row[7]
                    curves.append({"entity": entity, "begin": int(state[0]), "end": int(state[1]),
                                   "mesh_degenerate": int(state[2]), "geometry_degenerate": int(state[3]), "samples": samples})
                for _, face in gmsh.model.getEntities(2):
                    buffer = (ctypes.c_double * (4096*3))()
                    count = functions["FaceUses"](face, buffer, 4096)
                    if count < 0: raise ValueError("native face uses unavailable")
                    uses = [list(map(int, buffer[3*i:3*(i+1)])) for i in range(count)]
                    if any(kind < 2 and seam for kind, entity, seam in uses):
                        report["deferred_periodic_faces"].append({"fixture": fixture, "face": face})
                        continue
                    copies = (ctypes.c_double * (4096*9))()
                    count = functions["InitialChart"](face, copies, 4096)
                    if count < 0: raise ValueError("actual getNodeCopies failed")
                    rows = [list(copies[9*i:9*(i+1)]) for i in range(count)]
                    available = {int(row[0]): row for row in rows}
                    if len(available) != count: raise ValueError("unexpected multiple UV copies in ordinary fixture")
                    boundary_uses = [[entity, seam] for kind, entity, seam in uses if kind == 0]
                    embedded_uses = [[entity, seam] for kind, entity, seam in uses if kind == 1]
                    embedded_vertices = [entity for kind, entity, seam in uses if kind == 2]
                    local = {node: index for index, node in enumerate(sorted(available))}
                    expected = {"face": face, "vertices": [], "initial_vertices": [], "constraints": []}
                    for index, node in enumerate(sorted(available)):
                        row = available[node]
                        expected["vertices"].append([index, node, *row[1:6], int(row[6]), int(row[7])])
                    for row in rows: expected["initial_vertices"].append([int(row[0]), row[1], row[2]])
                    by_entity = {curve["entity"]: curve for curve in curves}
                    for embedded, selected in ((0, boundary_uses), (1, embedded_uses)):
                        for entity, seam in selected:
                            curve = by_entity[entity]
                            if curve["mesh_degenerate"]: continue
                            for a, b in zip(curve["samples"], curve["samples"][1:]):
                                expected["constraints"].append([entity, local[a[0]], local[b[0]], embedded])
                    lines = [[face, len(nodes), len(curves), len(boundary_uses), len(embedded_uses), len(embedded_vertices)]]
                    for node in sorted(nodes):
                        row = nodes[node]; frozen = available.get(node)
                        lines.append(row + [int(frozen is not None)] + (frozen[1:6] if frozen else [0.]*5))
                    for curve in curves:
                        lines.append([curve["entity"], curve["begin"], curve["end"], curve["mesh_degenerate"],
                                      curve["geometry_degenerate"], len(curve["samples"])])
                        lines.extend(curve["samples"])
                    lines.extend(boundary_uses); lines.extend(embedded_uses); lines.extend([[node] for node in embedded_vertices])
                    prefix = fixture + "-face" + str(face)
                    frozen_path = directory / (prefix + "-input.tsv")
                    frozen_path.write_text("".join(" ".join(format(value, ".17g") if isinstance(value, float) else str(value)
                                                              for value in line) + "\n" for line in lines), encoding="utf-8")
                    (directory / (prefix + "-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
                    own_path = directory / (prefix + "-own.json")
                    subprocess.run([args.own_executable, str(frozen_path), str(own_path)], check=True)
                    actual = json.loads(own_path.read_text(encoding="utf-8"))
                    for row in actual["vertices"]:
                        for k in range(2, 7): row[k] = float(row[k])
                    for row in actual["initial_vertices"]: row[1:] = map(float, row[1:])
                    found = first_trace_difference([{"stage": "ordinary_face_chart", "entity": face, **expected}],
                                                   [{"stage": "ordinary_face_chart", "entity": face, **actual}], 0.)
                    report["cases"].append({"fixture": fixture, "face": face, "nodes": len(available),
                                            "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                            "input_sha256": hashlib.sha256(frozen_path.read_bytes()).hexdigest(), "first_difference": found})
                    if found: raise ValueError("ordinary chart divergence: " + json.dumps(found))
            finally:
                gmsh.finalize()
        report["complete"] = True
    finally:
        (directory / "face-chart-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"ordinary_faces": len(report["cases"]), "deferred_periodic_faces": len(report["deferred_periodic_faces"]),
                      "frozen_uv_replay_equivalent": report["complete"]}))


if __name__ == "__main__":
    main()
