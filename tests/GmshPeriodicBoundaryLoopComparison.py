"""对照实际 buildConsecutiveListOfVertices 的环选择；CAD adapter/跨环 BDS 复用不在本项。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    pointer = ctypes.POINTER(ctypes.c_double)
    loop_count = gmsh.lib.meshstudioReferencePeriodicLoopCount
    loop_count.argtypes = [ctypes.c_int]; loop_count.restype = ctypes.c_int
    loop_input = gmsh.lib.meshstudioReferencePeriodicLoopInput
    loop_input.argtypes = [ctypes.c_int, ctypes.c_int, pointer, ctypes.c_int]; loop_input.restype = ctypes.c_int
    loop_result = gmsh.lib.meshstudioReferencePeriodicLoop
    loop_result.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_int, pointer, ctypes.c_int]
    loop_result.restype = ctypes.c_int
    report = {"scope": "native consecutive-loop selection and closure from frozen exact pcurves",
              "own_cad_adapter_validated": False, "prior_loop_bds_reuse_validated": False,
              "cases": [], "complete": False,
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
                for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6.,
                                    "Mesh.MeshSizeMax": .3, "Mesh.MeshSizeFromPoints": 0.,
                                    "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1.}.items():
                    gmsh.option.setNumber(name, value)
                gmsh.model.mesh.generate(1)
                for _, face in gmsh.model.getEntities(2):
                    lower, upper = gmsh.model.getParametrizationBounds(2, face)
                    du = float(upper[0]-lower[0]); dv = float(upper[1]-lower[1])
                    scale = math.sqrt(du*du+dv*dv)
                    if not math.isfinite(scale) or scale <= 0.: raise ValueError("invalid native face bounds")
                    number = loop_count(face)
                    if number <= 0: raise ValueError("native fixture edge loops unavailable or empty")
                    for loop in range(number):
                        buffer = (ctypes.c_double * (65536*7))()
                        count = loop_input(face, loop, buffer, 65536)
                        if count < 0: raise ValueError("native exact loop input failed: " + str(count))
                        curves = []
                        for i in range(count):
                            use, edge, seam, physical, u, v, branch = buffer[7*i:7*(i+1)]
                            use = int(use); physical = int(physical); branch = int(branch)
                            if use == len(curves):
                                curves.append({"entity": int(edge), "seam": int(seam), "vertices": []})
                            if use != len(curves)-1 or (branch == -1 and not curves[use]["seam"]):
                                raise ValueError("invalid native curve order")
                            if branch == 1: curves[use]["vertices"].append([physical, u, v])
                            elif branch == -1:
                                row = curves[use]["vertices"][-1]
                                if row[0] != physical or len(row) != 3: raise ValueError("invalid seam physical copy")
                                row.extend([u, v])
                            else: raise ValueError("invalid pcurve branch")
                        for factor in (1.e-12, 1.e-9, 1.e-6, 1.e-3, 1.e-2, 1.e-1):
                            tolerance = factor * scale
                            for seam_first in (0, 1):
                                output = (ctypes.c_double * (65536*4))()
                                result = loop_result(face, loop, tolerance, seam_first, output, 65536)
                                if result < 0 and result != -4: raise ValueError("native loop oracle failed: " + str(result))
                                expected = {"closed": result >= 0, "vertices": []}
                                for i in range(max(result, 0)):
                                    local, physical, u, v = output[4*i:4*(i+1)]
                                    if int(local) != i: raise ValueError("unexpected local ID in fresh native context")
                                    expected["vertices"].append([int(physical), u, v])
                                lines = [[tolerance, seam_first, len(curves)]]
                                for curve in curves:
                                    lines.append([curve["entity"], curve["seam"], len(curve["vertices"])])
                                    lines.extend(curve["vertices"])
                                prefix = f"{fixture}-face{face}-loop{loop}-factor{factor:g}-seam{seam_first}"
                                frozen = directory / (prefix + "-input.tsv")
                                frozen.write_text("".join(" ".join(format(x, ".17g") if isinstance(x, float) else str(x)
                                                                  for x in line) + "\n" for line in lines), encoding="utf-8")
                                (directory / (prefix + "-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
                                own = directory / (prefix + "-own.json")
                                subprocess.run([args.own_executable, str(frozen), str(own)], check=True)
                                actual = json.loads(own.read_text(encoding="utf-8"))
                                for row in actual["vertices"]: row[1:] = map(float, row[1:])
                                difference = first_trace_difference(
                                    [{"stage": "periodic_boundary_loop", "entity": face, **expected}],
                                    [{"stage": "periodic_boundary_loop", "entity": face, **actual}], 0.)
                                report["cases"].append({"fixture": fixture, "face": face, "loop": loop,
                                                        "tolerance": tolerance, "seam_first": seam_first,
                                                        "closed": expected["closed"], "vertices": len(expected["vertices"]),
                                                        "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                                        "input_sha256": hashlib.sha256(frozen.read_bytes()).hexdigest(),
                                                        "first_difference": difference})
                                if difference: raise ValueError("loop divergence: " + json.dumps(difference))
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 108:
            raise ValueError("fixed fixture loop matrix must contain 108 cases")
        report["complete"] = True
    finally:
        (directory / "periodic-loop-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases": len(report["cases"]), "native_closed": sum(c["closed"] for c in report["cases"]),
                      "frozen_pcurve_loop_equivalent": report["complete"]}))


if __name__ == "__main__":
    main()
