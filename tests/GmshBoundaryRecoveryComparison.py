"""调用固定 Gmsh 的实际 remeshUnrecoveredEdges，逐次比较边参数和 getLc 元数据。"""
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
    parser.add_argument("--api", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--own-executable", required=True)
    parser.add_argument("--frozen-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    recovery = gmsh.lib.meshstudioReferenceBoundaryRecovery
    recovery.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    recovery.restype = ctypes.c_int
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(args.runtime_dir + [environment.get("PATH", "")])
    report = {"scope": "actual ordinary-map native remeshUnrecoveredEdges, explicit serial requests",
              "coordinate_parameter_lc_absolute_tolerance": 1e-12,
              "reference_library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest(),
              "cases": [], "complete": False}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        for name in ("line", "bezier", "rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded"):
            gmsh.initialize([], readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong reference version")
                gmsh.model.add(name)
                cad = Path(args.frozen_dir) / (name + ".brep")
                gmsh.model.occ.importShapes(str(cad), highestDimOnly=False)
                gmsh.model.occ.synchronize()
                for option, value in {"General.NumThreads": 1., "General.Terminal": 0., "Mesh.Algorithm": 6.,
                                      "Mesh.MeshSizeMin": 0., "Mesh.MeshSizeMax": .3, "Mesh.MeshSizeFactor": 1.,
                                      "Mesh.MeshSizeFromPoints": 0., "Mesh.MeshSizeFromCurvature": 0.,
                                      "Mesh.RecombineAll": 0.}.items():
                    gmsh.option.setNumber(option, value)
                bounds = gmsh.model.getBoundingBox(-1, -1)
                model_length = math.sqrt(sum((bounds[k + 3] - bounds[k]) ** 2 for k in range(3))) or 1.
                gmsh.model.mesh.generate(1)
                entities = [tag for _, tag in gmsh.model.getEntities(1)]

                def snapshot():
                    result = []
                    for entity in entities:
                        buffer = (ctypes.c_double * (4096 * 8))()
                        count = recovery(entity, -1, buffer, 4096)
                        if count < 0: raise ValueError(f"native chain query failed {entity}: {count}")
                        rows = []
                        for i in range(count):
                            row = list(buffer[8*i:8*(i+1)])
                            for k in (0, 5, 6): row[k] = int(row[k])
                            for k in (1, 2, 3, 4, 7):
                                if not math.isfinite(row[k]):
                                    row[k] = "nan" if math.isnan(row[k]) else "-inf" if row[k] < 0 else "inf"
                            rows.append(row)
                        result.append({"stage": "boundary_recovery", "entity": entity, "rows": rows})
                    return result

                stages = [snapshot()]
                requests = []
                for chain in list(stages[0]):
                    count = len(chain["rows"])
                    if count < 2: continue
                    entity = chain["entity"]
                    # 首段、末段、内段；索引按已经更新的当前链确定。
                    for segment in (0, count-1, 1):
                        buffer = (ctypes.c_double * (4096 * 8))()
                        count_after = recovery(entity, segment, buffer, 4096)
                        if count_after < 0: raise ValueError(f"native recovery failed {entity}:{segment}")
                        requests.append(f"{entity}:{segment}")
                        stages.append(snapshot())
                reference_path = out / (name + "-reference.json")
                reference_path.write_text(json.dumps({"requests": requests, "recovery_stages": stages}, allow_nan=False), encoding="utf-8")
                own_path = out / (name + "-own.json")
                subprocess.run([args.own_executable, str(cad), ".3", str(model_length), str(own_path), *requests],
                               env=environment, check=True)
                own = json.loads(own_path.read_text(encoding="utf-8"))
                expected = [event for stage in stages for event in stage]
                actual = [event for stage in own["recovery_stages"] for event in stage]
                # 整数拼写只用于物理身份/分类；浮点列统一 float 以保持严格身份比较。
                for event in actual:
                    for row in event["rows"]:
                        for k in (1, 2, 3, 4, 7):
                            if isinstance(row[k], (int, float)): row[k] = float(row[k])
                found = first_trace_difference(expected, actual, 1e-12)
                report["cases"].append({"fixture": name, "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                        "requests": requests, "stages": len(stages), "first_difference": found})
                if found: raise ValueError("native recovery divergence: " + name + " " + json.dumps(found))
            finally:
                gmsh.finalize()
        report["complete"] = True
    finally:
        (out / "boundary-recovery-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases": len(report["cases"]), "equivalent": report["complete"]}))


if __name__ == "__main__":
    main()
