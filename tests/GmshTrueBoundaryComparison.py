"""实际 trueBoundary 对照；自研测试端读取相同 BREP 求范围/UV，参考只提供拓扑顺序。"""
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
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    pointer = ctypes.POINTER(ctypes.c_double)
    curves = gmsh.lib.meshstudioReferenceTrueBoundaryCurves
    curves.argtypes = [ctypes.c_int, pointer, ctypes.c_int]; curves.restype = ctypes.c_int
    sample = gmsh.lib.meshstudioReferenceTrueBoundary
    sample.argtypes = [ctypes.c_int, pointer, ctypes.c_int]; sample.restype = ctypes.c_int
    report = {"scope": "actual trueBoundary versus independent OCC pcurve range/value queries",
              "topology_order_and_orientation_from_reference": True, "geometry_values_from_reference": False,
              "generic_pcurve_fallback_validated": False, "production_exact_pcurve_capabilities_validated": False,
              "full_geometry_adapter_validated": False, "topology_metadata_checked_against_own_brep": True,
              "cases": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "torus", "embedded"):
            gmsh.initialize([], readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong fixed reference version")
                gmsh.option.setNumber("General.Terminal", 0.)
                gmsh.option.setNumber("General.NumThreads", 1.)
                if gmsh.option.getNumber("Geometry.ReparamOnFaceRobust") != 0.:
                    raise ValueError("direct pcurve test requires fixed default robust mode zero")
                gmsh.model.add(fixture)
                cad = Path(args.frozen_dir) / (fixture + ".brep")
                gmsh.model.occ.importShapes(str(cad), highestDimOnly=False)
                gmsh.model.occ.synchronize()
                for _, face in gmsh.model.getEntities(2):
                    metadata = (ctypes.c_double * (4096*4))()
                    count = curves(face, metadata, 4096)
                    if count <= 0: raise ValueError("native unique CAD boundary order unavailable")
                    rows = [list(map(int, metadata[4*i:4*(i+1)])) for i in range(count)]
                    points = (ctypes.c_double * (65536*2))()
                    number = sample(face, points, 65536)
                    if number <= 0 or number % 2: raise ValueError("native trueBoundary failed")
                    expected = {"face": face, "segments": []}
                    position = 0
                    for edge, orientation, seam, face_orientation in rows:
                        for branch in range(2 if seam else 1):
                            for k in range(299):
                                expected["segments"].append([edge, branch, *points[2*position:2*position+4]])
                                position += 2
                    if position != number: raise ValueError("native pair count differs from trueBoundary contract")
                    prefix = fixture + "-face" + str(face)
                    topology = directory / (prefix + "-topology.tsv")
                    topology.write_text(str(face) + " " + str(count) + "\n" +
                                        "".join(" ".join(map(str, row)) + "\n" for row in rows), encoding="utf-8")
                    (directory / (prefix + "-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
                    own = directory / (prefix + "-own.json")
                    subprocess.run([args.own_executable, str(cad), str(topology), str(own)], check=True)
                    actual = json.loads(own.read_text(encoding="utf-8"))
                    for row in actual["segments"]: row[2:] = map(float, row[2:])
                    difference = first_trace_difference(
                        [{"stage": "true_boundary", "entity": face, **expected}],
                        [{"stage": "true_boundary", "entity": face, **actual}], 0.)
                    report["cases"].append({"fixture": fixture, "face": face, "curves": count, "segments": number//2,
                                            "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                            "topology_sha256": hashlib.sha256(topology.read_bytes()).hexdigest(), "first_difference": difference})
                    if difference: raise ValueError("trueBoundary divergence: " + json.dumps(difference))
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 9: raise ValueError("fixed trueBoundary matrix must contain nine faces")
        report["production_exact_pcurve_capabilities_validated"] = True
        report["complete"] = True
    finally:
        (directory / "true-boundary-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"faces": len(report["cases"]), "own_occ_pcurve_equivalent": report["complete"]}))


if __name__ == "__main__":
    main()
