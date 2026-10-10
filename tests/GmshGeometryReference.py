"""固定源版本的数值 oracle；自研程序与参考 DLL 分离，输入在运行前冻结。"""
import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
from GmshReproductionComparison import load_reference_api, first_trace_difference


def cases():
    result = []
    for scale in (1e-6, 1., 1e6):
        result.extend([
            ("metric_circle", [0., 0., scale, 0., 0., scale, 4., 2., 2.]),
            ("metric_circle", [5. * scale, 5. * scale, 6. * scale, 5. * scale,
                               7. * scale, 5. * scale, 1., 0., 1.]),
            ("physical_circle", [0., 0., 0., scale, 0., 0., .5 * scale, math.sqrt(3.) * .5 * scale, 0.]),
            ("physical_circle", [0., 0., 0., scale, 0., 0., 2. * scale, 0., 0.]),
            ("orient2d", [0., 0., scale, 0., 0., scale]),
            ("orient2d", [0., 0., scale, scale, 2. * scale, math.nextafter(2. * scale, math.inf)]),
            ("orient2d", [0., 0., scale, scale, 2. * scale, 2. * scale]),
            ("incircle", [0., 0., scale, 0., 0., scale, scale, scale]),
            ("incircle", [0., 0., 0., scale, scale, 0., .25 * scale, .25 * scale]),
            ("incircle", [0., 0., scale, 0., 0., scale, scale, math.nextafter(scale, math.inf)])])
    rng = random.Random(20261007)
    result.extend([
        ("linear_system", [1., 0., 0., 0., 2., 0., 0., 0., 3., 2., 6., 12.]),
        ("linear_system", [1., 0., 0., 0., 1., 0., 0., 0., 0., 2., 3., 9.]),
        ("linear_system", [1., 0., 0., 0., 1., 0., 0., 0., 1e-18, 2., 3., 9.]),
        ("linear_system", [0.] * 12),
        ("linear_system", [1., 2., 3., 4., 1., 2., 2., 3., 1., 6., 8., -1.])])
    for _ in range(80):
        uv = [rng.uniform(-4, 4) for _ in range(6)]
        shear = rng.uniform(-2, 2)
        metric = [4., 2. * shear, shear * shear + 1.]
        result.append(("metric_circle", uv + metric))
        result.append(("physical_circle", [rng.uniform(-4, 4) for _ in range(9)]))
        result.append(("orient2d", uv))
        result.append(("incircle", uv + [rng.uniform(-4, 4), rng.uniform(-4, 4)]))
        result.append(("linear_system", [rng.uniform(-4, 4) for _ in range(12)]))
    result.extend([
        ("curve_smoothing", [1.5, 2., 0., .2, 0., 2., 1., 4., 2.1, 4.]),
        ("curve_smoothing", [1.5, 3., 0., 4., 0., 4., 1., .2, 2.1, 2., 2., .1, 2.25, 2.]),
        ("curve_smoothing", [1.5, 3., 0., 0., 0., 0., 1., 1., .5, 1., 2., 1., 1.5, 1.]),
        ("curve_smoothing", [1.5, 2., 0., 0., 0., 1., 1., 1., .5, 1.]),
        ("curve_smoothing", [1., 2., 0., .2, 0., 2., 1., 4., 2.1, 4.])])
    circle = [0., 0., 0., 1., 0., 0., 0., 1., 0., 1.]
    result.extend([
        ("intersection", [0., 1., 0., .5, .1, .5] + circle),
        ("intersection", [0., 1., 0., 1., 0., 1.] + circle),
        ("intersection", [0., 1., 0., 1e6 + 1., 0., 1.] + circle),
        ("intersection", [0., -1., 0., .5, .1, .5] + circle),
        ("intersection", [1., 0., 0., 1., 0., 0.] + circle),
        ("intersection", [1., 1., 0., 1., 0., 0.] + circle),
        ("intersection", [1., 0., 0., 1e6 + 1., 0., 1.] + circle)])
    for _ in range(60):
        mode = float(rng.randrange(2))
        result.append(("intersection", [mode, rng.uniform(-1, 1), rng.uniform(-.3, .3),
                                        rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(.2, 2)] + circle))
    epsilon = sys.float_info.epsilon
    result.extend([("orient2d_value", [1. + epsilon, 1., 1., 1. - epsilon, 0., 0.]),
                   ("orient2d_value", [1., 1. - epsilon, 1. + epsilon, 1., 0., 0.]),
                   ("orient2d_value", [1. + epsilon, 1., 1., 1. - epsilon, 1e16, 1e16]),
                   ("orient2d_value", [1., 1. - epsilon, 1. + epsilon, 1., 1e16, 1e16])])
    for scale in (1e-6, 1., 1e6):
        for _ in range(100):
            origin = [rng.uniform(-4, 4) * scale for _ in range(2)]
            delta = [rng.uniform(-4, 4) * scale for _ in range(2)]
            result.append(("orient2d_value", origin + [origin[i] + delta[i] for i in range(2)] +
                           [origin[0] + 2. * delta[0], math.nextafter(origin[1] + 2. * delta[1], math.inf)]))
    result.extend([("incircle_oriented", [0.,0.,1.,0.,0.,1.,.1,.1]),
                   ("incircle_oriented", [0.,0.,0.,1.,1.,0.,.1,.1]),
                   ("incircle_oriented", [0.,0.,1.,0.,2.,0.,1.,1.]),
                   ("incircle_oriented", [0.,0.,1.,0.,0.,1.,1.,1.])])
    for scale in (1e-6, 1., 1e6):
        for _ in range(80):
            result.append(("incircle_oriented", [rng.uniform(-4,4)*scale for _ in range(8)]))
    clustered = [t for parameter in (.05,.1,1.,1.05,2.) for t in (parameter,parameter,0.,0.)]
    result.extend([("curve_filter", [1.,0.,5.] + clustered),
                   ("curve_filter", [1.,3.,5.] + clustered),
                   ("curve_filter", [1.,0.,1.,.3,.3,0.,0.]),
                   ("curve_filter", [1.,0.,2.,.1,0.,0.,0.,.5,.5,0.,0.]),
                   ("curve_filter", [1.,0.,0.])])
    for _ in range(80):
        count = rng.randrange(2,20)
        parameters = sorted(rng.uniform(.01,9.99) for _ in range(count))
        rows = [value for t in parameters for value in (t,t,rng.uniform(-.02,.02),0.)]
        result.append(("curve_filter", [rng.uniform(.1,2.),float(rng.randrange(count+1)),float(count)] + rows))
    result.extend([("curve_filter_field", [.5,2.,0.,2.,.2,.2,0.,0.,.5,.5,0.,0.]),
                   ("curve_filter_field", [.5,2.,2.,2.,.2,.2,0.,0.,.5,.5,0.,0.]),
                   ("curve_filter_field", [.5,2.,0.,3.,.2,.2,0.,0.,.21,.21,0.,0.,.5,.5,0.,0.])])
    for _ in range(40):
        count = rng.randrange(2,20)
        parameters = sorted(rng.uniform(.01,9.99) for _ in range(count))
        rows = [value for t in parameters for value in (t,t,rng.uniform(-.02,.02),0.)]
        result.append(("curve_filter_field", [rng.uniform(.1,2.),rng.uniform(0.,.3),
                                             float(rng.randrange(count+1)),float(count)] + rows))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--own-executable", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    directory = Path(args.out_dir)
    directory.mkdir(parents=True, exist_ok=True)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    gmsh = load_reference_api(args.api, args.library)
    gmsh.initialize([], readConfigFiles=False)
    try:
        if not gmsh.option.getString("General.Version").startswith("4.15.2"):
            raise ValueError("wrong reference version")
        functions = {}
        for operation, name in [("metric_circle", "MetricCircle"), ("physical_circle", "PhysicalCircle"),
                                ("orient2d", "Orient2d"), ("incircle", "Incircle"), ("linear_system", "Solve3")]:
            function = getattr(gmsh.lib, "meshstudioReference" + name)
            width = 3 if operation in ("metric_circle", "linear_system") else 2 if operation == "physical_circle" else 1
            function.argtypes = [ctypes.POINTER(ctypes.c_double)] * width
            function.restype = ctypes.c_double if operation in ("orient2d", "incircle") else None
            functions[operation] = function
        smoothing = gmsh.lib.meshstudioReferenceSmoothCurve
        smoothing.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_double, ctypes.POINTER(ctypes.c_double)]
        smoothing.restype = None
        intersection = gmsh.lib.meshstudioReferenceIntersection
        intersection.argtypes = [ctypes.POINTER(ctypes.c_double)] * 2
        intersection.restype = None
        filtering = gmsh.lib.meshstudioReferenceFilterCurve
        filtering.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_int,
                              ctypes.POINTER(ctypes.c_double)]
        filtering.restype = ctypes.c_int
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("curve_filter_oracle")
        filter_curve = gmsh.model.occ.addLine(gmsh.model.occ.addPoint(0.,0.,0.), gmsh.model.occ.addPoint(10.,0.,0.))
        gmsh.model.occ.synchronize()
        for name, value in {"Mesh.MeshSizeMin": 0., "Mesh.MeshSizeFactor": 1., "Mesh.MeshSizeFromPoints": 0.,
                            "Mesh.MeshSizeFromCurvature": 0., "Mesh.RecombineAll": 0.}.items():
            gmsh.option.setNumber(name, value)
        # generate(0) does not create GVertex mesh nodes in this fixed version.
        gmsh.model.mesh.generate(1)
        frozen = cases()
        input_path = directory / "geometry-inputs.tsv"
        input_path.write_text("".join(operation + " " + " ".join(format(value, ".17g") for value in values) + "\n"
                                      for operation, values in frozen), encoding="utf-8")
        expected = []
        predicate_values = []
        native_nonfinite = {}
        for entity, (operation, values) in enumerate(frozen):
            native_input = (ctypes.c_double * len(values))(*values)
            if operation in ("metric_circle", "linear_system"):
                native_output = (ctypes.c_double * 3)()
                parameterOffset = 6 if operation == "metric_circle" else 9
                functions[operation](native_input, ctypes.cast(ctypes.byref(native_input, parameterOffset * ctypes.sizeof(ctypes.c_double)),
                                                              ctypes.POINTER(ctypes.c_double)), native_output)
                output = list(native_output)
            elif operation == "curve_smoothing":
                count = int(values[1])
                native_output = (ctypes.c_double * (4 * count))()
                smoothing(ctypes.cast(ctypes.byref(native_input, 2 * ctypes.sizeof(ctypes.c_double)),
                                     ctypes.POINTER(ctypes.c_double)), count, values[0], native_output)
                output = list(native_output)
            elif operation == "intersection":
                native_output = (ctypes.c_double * 6)()
                intersection(native_input, native_output)
                output = list(native_output)[:5]
                if native_output[5]:
                    native_nonfinite[entity] = int(native_output[5])
            elif operation in ("curve_filter", "curve_filter_field"):
                field = operation == "curve_filter_field"
                offset = 4 if field else 3
                count, minimum = int(values[offset - 1]), int(values[offset - 2])
                gmsh.option.setNumber("Mesh.MeshSizeMax", 1e22 if field else values[0])
                if field:
                    gmsh.model.mesh.setSizeCallback(lambda dim,tag,x,y,z,lc,base=values[0],gradient=values[1]:
                                                    base * (1. + gradient * x))
                native_output = (ctypes.c_double * (count + 1))()
                if not filtering(filter_curve, ctypes.cast(ctypes.byref(native_input, offset * ctypes.sizeof(ctypes.c_double)),
                                                          ctypes.POINTER(ctypes.c_double)), count, minimum, native_output):
                    raise ValueError("native curve filter fixture is invalid")
                if field:
                    gmsh.model.mesh.removeSizeCallback()
                output = list(native_output)[:1 + int(native_output[0])]
            elif operation == "physical_circle":
                native_output = (ctypes.c_double * 4)()
                functions[operation](native_input, native_output)
                output = list(native_output)
            elif operation == "orient2d_value":
                raw = functions["orient2d"](native_input)
                output = [raw, (raw > 0) - (raw < 0)]
            elif operation == "incircle_oriented":
                raw = functions["incircle"](native_input)
                output = [(raw > 0) - (raw < 0)]
            else:
                raw = functions[operation](native_input)
                sign = (raw > 0) - (raw < 0)
                if operation == "incircle":
                    orientation = functions["orient2d"](native_input)
                    sign *= (orientation > 0) - (orientation < 0)
                output = [sign]
                predicate_values.append({"entity": entity, "operation": operation, "raw_value": raw})
            expected.append({"stage": operation, "entity": entity, "values": output})
        reference_path = directory / "geometry-native.jsonl"
        reference_path.write_text("".join(json.dumps(event, allow_nan=False) + "\n" for event in expected), encoding="utf-8")
        own_path = directory / "geometry-own.jsonl"
        subprocess.run([args.own_executable, str(input_path), str(own_path)], check=True)
        actual = [json.loads(line) for line in own_path.read_text(encoding="utf-8").splitlines()]
        if len(actual) != len(expected):
            raise ValueError("geometry replay event count differs")
        first = None
        maximum_error = 0.
        nonfinite_failures = []
        for wanted, received in zip(expected, actual):
            # Diagnostic metadata is not a numerical output of the native function.
            comparable = {key: received[key] for key in ("stage", "entity", "values")}
            if wanted["entity"] in native_nonfinite:
                # Preserve the failure flag and original parameters as strict comparison;
                # classify the earlier invalid-arithmetic exit instead of claiming equal iteration counts.
                stop = received.get("intersection_stop")
                if wanted["values"][0] or received["values"][0] or stop not in ("nonfiniteStep", "invalidQuery"):
                    raise ValueError("unexplained nonfinite native intersection outcome")
                nonfinite_failures.append({"entity": wanted["entity"],
                                           "native_first_nonfinite_evaluation": native_nonfinite[wanted["entity"]],
                                           "native_evaluations": wanted["values"][4],
                                           "own_evaluations": received["values"][4], "own_stop": stop})
                wanted = dict(wanted, values=wanted["values"][:4])
                comparable = dict(comparable, values=comparable["values"][:4])
            tolerance = (16. * sys.float_info.epsilon * abs(wanted["values"][0]) if wanted["stage"] == "orient2d_value"
                         else 0. if wanted["stage"] in ("orient2d", "incircle", "incircle_oriented", "curve_filter", "curve_filter_field") else (
                             16. * sys.float_info.epsilon * max(1., *(abs(value) for value in wanted["values"]))))
            if wanted["stage"] == "orient2d_value" and wanted["values"][1] != comparable["values"][1]:
                raise ValueError("adaptive orientation numerical sign differs")
            maximum_error = max(maximum_error, *(abs(a - b) for a, b in zip(wanted["values"], comparable["values"])))
            first = first_trace_difference([wanted], [comparable], tolerance)
            if first:
                break
        report = {"cases": len(expected), "predicate_comparison": "exact normalized sign",
                  "construction_tolerance": "16 * epsilon * max(1, absolute native outputs)",
                  "orientation_value_tolerance": "16 * epsilon * abs(native area), plus exact sign",
                  "maximum_absolute_error": maximum_error, "first_difference": first,
                  "classified_nonfinite_failures": nonfinite_failures,
                  "native_predicate_values": predicate_values}
        (directory / "geometry-report.json").write_text(json.dumps(report, allow_nan=False), encoding="utf-8")
        print(json.dumps({key: value for key, value in report.items() if key != "native_predicate_values"}))
        raise SystemExit(1 if first else 0)
    finally:
        gmsh.finalize()
        for handle in handles:
            handle.close()


if __name__ == "__main__":
    main()
