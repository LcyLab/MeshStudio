"""原生BDS单次恢复/求交对照：冻结输入BDS，比较完整扫描、失败诊断和池改写。"""
import argparse
import ctypes
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import random
import subprocess
from GmshBdsMeshConversionComparison import normalize, write_input
from GmshReproductionComparison import load_reference_api, first_trace_difference


def numeric(value):
    if math.isnan(value): return "nan"
    if math.isinf(value): return "-inf" if value < 0 else "inf"
    if value == 0. and math.copysign(1., value) < 0.: return "-0"
    return float(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "out-dir", "periodic-bds-dir"): parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library); pointer = ctypes.POINTER(ctypes.c_double); int_pointer = ctypes.POINTER(ctypes.c_int)
    scalar = gmsh.lib.meshstudioReferenceBdsRecoveryIntersection; scalar.argtypes = [pointer, pointer]; scalar.restype = ctypes.c_int
    recover = gmsh.lib.meshstudioReferenceBdsConstraintRecovery
    recover.argtypes = [ctypes.c_int, pointer, ctypes.c_int, int_pointer, ctypes.c_int, int_pointer, ctypes.c_int, int_pointer,
                       ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]; recover.restype = ctypes.c_int
    report = {"scope": "native BDS recover_edge leaf with frozen point/triangle pools, numeric intersection, scan/attempt order and failure records",
              "actual_cad_recovery_driver_validated": False, "boundary_pass_classification_validated": False,
              "frozen_inputs": ["BDS points/triangles/classification/modified flags", "constraint registry and requested edge"],
              "scalar_cases": 0, "cases": [], "complete": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    gmsh.initialize([], readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0.); gmsh.model.add("bds-leaf-registry")
        a = gmsh.model.occ.addPoint(0., 0., 0.); b = gmsh.model.occ.addPoint(1., 0., 0.); c = gmsh.model.occ.addPoint(0., 1., 0.)
        gmsh.model.occ.addLine(a, b, tag=11); gmsh.model.occ.addLine(a, c, tag=12); gmsh.model.occ.synchronize()
        base = [([0., 0.], [1., 1.], [0., 1.], [1., 0.]), ([0., 0.], [1., 0.], [1., 0.], [1., 1.]),
                ([0., 0.], [1., 0.], [0., 1.], [1., 1.]), ([0., 0.], [0., 0.], [0., 1.], [1., 0.]),
                ([0., 0.], [1., 0.], [.5, -1.], [.5, 1.]), ([0., 0.], [1., 0.], [2., -1.], [2., 1.]),
                ([0., 0.], [1., 1.e-17], [0., 1.e-17], [1., 0.]), ([0., 0.], [1., 5.e-16], [0., 5.e-16], [1., 0.]),
                ([0., 0.], [1., 1.], [1., 1.], [2., 0.]), ([0., 0.], [1., 0.], [.25, 0.], [.75, 0.])]
        rows = []
        for case in base:
            for scale in (1.e-160, 1.e-150, 1.e-80, 1.e-8, 1., 1.e8, 1.e80, 1.e150, 1.e160):
                scaled = [[x*scale for x in p] for p in case]
                for reverse_first, reverse_second, swap in itertools.product((False, True), repeat=3):
                    first = scaled[:2][::(-1 if reverse_first else 1)]; second = scaled[2:][::(-1 if reverse_second else 1)]
                    ordered = second+first if swap else first+second; rows.append([x for p in ordered for x in p])
        expected = []
        for row in rows:
            input_buffer = (ctypes.c_double*8)(*row); output = (ctypes.c_double*4)()
            if scalar(input_buffer, output) != 4: raise ValueError("native BDS intersection failed")
            expected.append([int(output[0]), int(output[1]), numeric(output[2]), numeric(output[3])])
        source = directory/"intersection-input.tsv"; write_input(source, [["intersection", len(rows)], *rows])
        target = directory/"intersection-own.json"; subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
        actual = json.loads(target.read_text(encoding="utf-8"))
        for row in actual:
            for i in (2, 3):
                if not isinstance(row[i], str): row[i] = float(row[i])
        difference = first_trace_difference([{"stage": "bds_intersection", "values": expected}], [{"stage": "bds_intersection", "values": actual}], 0.)
        report["scalar_cases"] = len(rows); report["scalar_first_difference"] = difference
        (directory/"intersection-reference.json").write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
        if difference: raise ValueError("BDS intersection diverged: "+json.dumps(difference))

        def run_case(name, points, triangles, first, second, labels=(), registry=(), origin="synthetic"):
            p = (ctypes.c_double*max(1, 13*len(points)))(*(x for r in points for x in r))
            t = (ctypes.c_int*max(1, 3*len(triangles)))(*(x for r in triangles for x in r))
            l = (ctypes.c_int*max(1, 5*len(labels)))(*(x for r in labels for x in r))
            r = (ctypes.c_int*max(1, 3*len(registry)))(*(x for row in registry for x in row))
            native = ctypes.create_string_buffer(32*1024*1024)
            status = recover(len(points), p, len(triangles), t, len(labels), l, len(registry), r, first, second, native, len(native))
            if status < 0: raise ValueError(name+" native recovery error: "+str(status))
            expected = json.loads(native.value.decode("utf-8")); normalize(expected["mesh"])
            source = directory/(name+"-input.tsv")
            write_input(source, [["recover", len(points)], *points, [len(triangles)], *triangles, [len(labels)], *labels, [len(registry)], *registry, [first, second]])
            target = directory/(name+"-own.json"); subprocess.run([args.own_executable, str(source), str(target)], check=True, timeout=30)
            actual = json.loads(target.read_text(encoding="utf-8")); normalize(actual["mesh"])
            difference = first_trace_difference([{"stage": "bds_recovery", **expected}], [{"stage": "bds_recovery", **actual}], 0.)
            (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
            report["cases"].append({"name": name, "origin": origin, "first": first, "second": second,
                "points": len(points), "triangles": len(triangles), "native_recovered": status == 1, "native_fatal": expected["fatal"],
                "self_intersection_requests": len(expected["not_recovered"]), "scans": len(expected["scans"]),
                "successful_swaps": sum(int(ok) for scan in expected["scans"] for _, ok in scan["attempts"]),
                "input_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "first_difference": difference})
            if difference: raise ValueError(name+" BDS recovery divergence: "+json.dumps(difference))

        coordinates = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]]
        for ordering in itertools.permutations(range(3)):
            for kind in ("normal", "protected", "unchanged", "registry", "counterpart"):
                points = [[i, 0, *uv, *uv, 0., -1, 0, 1.e22, 1.e22,
                           (3 if i == 1 else 1 if i == 3 else -1) if kind == "counterpart" else -1, int(kind != "unchanged")] for i, uv in enumerate(coordinates)]
                triangles = [[(0, 1, 2)[i] for i in ordering], [0, 2, 3]]
                labels = [[1, 0, 0, 901, 2], [1, 1, 0, 902, 2], [0, 0, 2, 903, 1 if kind == "protected" else 2]]
                registry = [[0, 2, 11], [1, 3, 12]] if kind == "registry" else []
                run_case("square-"+"".join(map(str, ordering))+"-"+kind, points, triangles, 1, 3, labels, registry)
        generator = random.Random(4152)
        for i in range(18):
            count = 8+i; coordinates = [[math.cos(k*2*math.pi/count), math.sin(k*2*math.pi/count)] for k in range(count)]
            points = [[k, 0, *uv, *uv, .1*k, -1, 0, .7, .9, -1, 1] for k, uv in enumerate(coordinates)]
            triangles = [[0, k, k+1] for k in range(1, count-1)]; generator.shuffle(triangles); generator.shuffle(points)
            run_case("convex-fan-"+str(i), points, triangles, 2, count-2)
        run_case("existing", [[i, 0, *uv, *uv, 0., -1, 0, 1.e22, 1.e22, -1, 1] for i, uv in enumerate([[0., 0.], [1., 0.], [1., 1.], [0., 1.]])], [[0, 1, 2], [0, 2, 3]], 0, 2)
        coordinates = [[2., 0.], [1., 1.], [0., 2.], [-1., 1.], [-2., 0.], [-1., -1.], [0., -2.], [1., -1.]]
        points = [[i, 0, *uv, *uv, 0., -1, 0, 1.e22, 1.e22, -1, 1] for i, uv in enumerate(coordinates)]
        run_case("failure-after-swaps", points, [[0, 1, 2], [0, 4, 5], [0, 3, 4], [0, 2, 3], [0, 6, 7], [0, 5, 6]], 2, 6, [[0, 0, 5, 903, 1]])

        # CAD-derived frozen BDS is a leaf input, not proof of the CAD recovery driver.
        files = sorted(Path(args.periodic_bds_dir).glob("*-initial-own.json"))
        if len(files) != 32: raise ValueError("fixed periodic BDS source matrix must contain 32 files")
        for index, path in enumerate(files):
            document = json.loads(path.read_text(encoding="utf-8")); mesh = document["bds"]
            points = [[r[0], 0, *r[2:7], r[8], r[7], r[9], r[10], r[13] if r[13] is not None else -1, r[11]] for r in mesh["points"]]
            triangles = mesh["triangle_nodes"]; constraints = document["constraints"][:3]
            existing_pairs = {tuple(sorted(r[:2])) for r in mesh["edges"]}
            positive = [r[0] for r in points if r[0] >= 0]
            pressure = [(a, b) for a, b in zip(positive[1:4], reversed(positive[-4:-1])) if a != b and tuple(sorted((a, b))) not in existing_pairs]
            for j, (a, b) in enumerate([*constraints, *pressure]):
                run_case(f"cad-derived-{index}-{j}", points, triangles, a, b, origin="frozen actual periodic BDS / "+path.name)
        if not any(c["successful_swaps"] > 1 for c in report["cases"]): raise ValueError("matrix must exercise multiple swaps")
        if not any(c["self_intersection_requests"] == 2 for c in report["cases"]): raise ValueError("matrix must exercise CAD self intersection")
        if not any(c["native_fatal"] and c["successful_swaps"] > 0 for c in report["cases"]): raise ValueError("matrix must exercise failure after topology mutation")
        report["complete"] = True
    finally:
        gmsh.finalize()
        (directory/"bds-recovery-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"scalar_cases": report["scalar_cases"], "recovery_cases": len(report["cases"]), "equivalent": report["complete"]}))


if __name__ == "__main__": main()
