"""固定 4.15.2 参考的重放与首次分歧报告；不会调用产品生成器。

每次参考运行使用独立进程和显式 DLL/API 路径。指纹保留节点身份、
坐标、单元朝向及实际算法，只忽略输出枚举顺序和计时/观察元数据。
"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import types


def mesh_fingerprint(document):
    def exact(value):
        if isinstance(value, float):
            if not math.isfinite(value):
                raise ValueError("mesh contains nonfinite coordinates")
            return value.hex()
        if isinstance(value, list):
            return [exact(item) for item in value]
        return value

    payload = {key: exact(sorted(document[key], key=lambda row: row[0]))
               for key in ("nodes", "elements", "actual_algorithms")}
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def first_trace_difference(reference, actual, absolute_tolerance=0.0):
    if not math.isfinite(absolute_tolerance) or absolute_tolerance < 0:
        raise ValueError("absolute tolerance must be nonnegative and finite")

    def difference(expected, received, path):
        if isinstance(expected, dict) and isinstance(received, dict):
            for key in sorted(expected.keys() | received.keys()):
                child = f"{path}.{key}" if path else key
                if key not in expected or key not in received:
                    return child, expected.get(key), received.get(key)
                found = difference(expected[key], received[key], child)
                if found:
                    return found
            return None
        if isinstance(expected, list) and isinstance(received, list):
            for index in range(max(len(expected), len(received))):
                child = f"{path}[{index}]"
                if index >= len(expected) or index >= len(received):
                    return child, (expected[index] if index < len(expected) else None), (
                        received[index] if index < len(received) else None)
                found = difference(expected[index], received[index], child)
                if found:
                    return found
            return None
        identity_fields = {"entity", "sequence", "physical_id", "branch", "local_id", "point_count",
                           "minimum_segments", "index", "active_edge", "result", "radius_marker",
                           "dimension", "from", "to"}
        numeric_pair = type(expected) in (int, float) and type(received) in (int, float)
        if numeric_pair and (type(expected) is float or type(received) is float) and path.split(".")[-1] not in identity_fields:
            if math.isfinite(expected) and math.isfinite(received):
                if abs(expected - received) <= absolute_tolerance:
                    return None
        elif type(expected) is type(received) and expected == received:
            return None
        return path, expected, received

    for index in range(max(len(reference), len(actual))):
        expected = reference[index] if index < len(reference) else None
        received = actual[index] if index < len(actual) else None
        found = ("$event", expected, received) if expected is None or received is None else (
            difference(expected, received, ""))
        if found:
            event = expected or received
            return {"event": index, "stage": event.get("stage"), "entity": event.get("entity"),
                    "path": found[0], "reference": found[1], "actual": found[2]}
    return None


def load_reference_api(api_path, library_path):
    """执行固定 API 的函数定义；替换加载路径，不导入机器默认 Gmsh。"""
    source = Path(api_path).read_text(encoding="utf-8")
    begin = source.index("moduledir = ")
    end = source.index("lib = CDLL(libpath)", begin) + len("lib = CDLL(libpath)")
    source = source[:begin] + "libpath = _meshstudioLibrary\nlib = CDLL(libpath)" + source[end:]
    source = source.replace("try_numpy = True", "try_numpy = False", 1)
    module = types.ModuleType("meshstudio_fixed_gmsh")
    module.__file__ = str(Path(api_path).resolve())
    module._meshstudioLibrary = str(Path(library_path).resolve())
    exec(compile(source, module.__file__, "exec"), module.__dict__)
    if module.__version__ != "4.15.2":
        raise ValueError(f"reference API version is {module.__version__}, expected 4.15.2")
    return module


def build_fixture(gmsh, name):
    occ = gmsh.model.occ
    if name in ("rectangle", "narrow_band", "embedded"):
        face = occ.addRectangle(0, 0, 0, 4, .2 if name == "narrow_band" else 3)
        occ.synchronize()
        if name == "embedded":
            point = occ.addPoint(2, 1.5, 0)
            occ.synchronize()
            gmsh.model.mesh.embed(0, [point], 2, face)
    elif name == "hole":
        face = occ.addRectangle(0, 0, 0, 4, 3)
        hole = occ.addDisk(2, 1.5, 0, .5, .5)
        occ.cut([(2, face)], [(2, hole)])
    elif name == "cylinder":
        occ.addCylinder(0, 0, 0, 0, 0, 3, 1)
    elif name == "sphere":
        occ.addSphere(0, 0, 0, 1)
    elif name == "line":
        occ.addLine(occ.addPoint(0, 0, 0), occ.addPoint(10, 0, 0))
    elif name == "bezier":
        # x(t)=t+9t^5，导数处处为正；保留原生非线性参数化。
        points = [occ.addPoint(x, 0, 0) for x in (0, .2, .4, .6, .8, 10)]
        occ.addBezier(points)
    else:
        raise ValueError(f"unknown fixture: {name}")
    occ.synchronize()


def run_reference(args):
    dll_handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    if args.trace:
        if Path(args.trace).exists():
            raise ValueError("trace output already exists; use a new path")
        os.environ["MESHSTUDIO_GMSH_TRACE"] = str(Path(args.trace).resolve())
    else:
        os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    gmsh = load_reference_api(args.api, args.library)
    gmsh.initialize([], readConfigFiles=False)
    try:
        version = gmsh.option.getString("General.Version")
        if not version.startswith("4.15.2"):
            raise ValueError(f"reference library version is {version}")
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("reproduction")
        if args.cad:
            gmsh.model.occ.importShapes(str(Path(args.cad).resolve()), highestDimOnly=False)
            gmsh.model.occ.synchronize()
        else:
            build_fixture(gmsh, args.fixture)
        if args.restore_fixture_embedding:
            if not args.cad or args.fixture != "embedded":
                raise ValueError("embedding restoration requires the frozen embedded BREP fixture")
            faces = gmsh.model.getEntities(2)
            points = [tag for dim, tag in gmsh.model.getEntities(0)
                      if list(gmsh.model.getValue(dim, tag, [])) == [2., 1.5, 0.]]
            if len(faces) != 1 or len(points) != 1:
                raise ValueError("frozen embedding identities are missing or ambiguous")
            # BREP preserves geometry, not this meshing relation; restore fixture metadata explicitly.
            gmsh.model.mesh.embed(0, points, 2, faces[0][1])
        if args.export_cad:
            gmsh.write(str(Path(args.export_cad).resolve()))
        options = {"Mesh.Algorithm": 6., "Mesh.ElementOrder": 1.,
                   "Mesh.MeshSizeMin": 0., "Mesh.MeshSizeMax": args.size,
                   "Mesh.MeshSizeFactor": 1., "Mesh.MeshSizeFromPoints": 0.,
                   "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeExtendFromBoundary": 1.,
                   "Mesh.Smoothing": args.smoothing, "Mesh.AlgorithmSwitchOnFailure": 1.}
        for name, value in options.items():
            gmsh.option.setNumber(name, value)
        if args.size_field:
            def field_size(dim, tag, x, y, z, lc):
                if args.size_field == "step": return .1 if x < 5. else 10.
                if args.size_field == "graded": return .1 + .8*abs(x)
                return .05 if dim == 0 else .3
            gmsh.model.mesh.setSizeCallback(field_size)
        for name in ("Mesh.LcIntegrationPrecision", "Mesh.SmoothRatio", "Mesh.RandomFactor",
                     "Mesh.MinimumCurvePoints", "Mesh.MinimumCirclePoints", "Mesh.OldInitialDelaunay2D"):
            options[name] = gmsh.option.getNumber(name)
        gmsh.logger.start()
        bounds = gmsh.model.getBoundingBox(-1, -1)
        model_length = math.sqrt(sum((bounds[i + 3] - bounds[i]) ** 2 for i in range(3))) or 1.
        gmsh.model.mesh.generate(args.dimension)
        messages = gmsh.logger.get()
        gmsh.logger.stop()
        if any(message.startswith("Error:") for message in messages):
            raise RuntimeError("\n".join(messages))
        nodes, elements, algorithms = [], [], {}
        for dim, tag in gmsh.model.getEntities():
            tags, xyz, uv = gmsh.model.mesh.getNodes(dim, tag, False, True)
            for index, node in enumerate(tags):
                parameters = list(uv[index * dim:(index + 1) * dim]) if dim else []
                nodes.append([int(node), *map(float, xyz[3 * index:3 * index + 3]), dim, tag, parameters])
            types_, tags_, connectivity = gmsh.model.mesh.getElements(dim, tag)
            for kind, ids, connected in zip(types_, tags_, connectivity):
                width = gmsh.model.mesh.getElementProperties(kind)[3]
                for index, element in enumerate(ids):
                    elements.append([int(element), dim, tag, int(kind),
                                     list(map(int, connected[width * index:width * (index + 1)]))])
        for message in messages:
            found = re.search(r"Meshing surface (\d+) \([^,]+, ([^)]+)\)", message)
            if found:
                algorithms[int(found[1])] = found[2]
        result = {"schema": 1, "nodes": nodes, "elements": elements,
                  "actual_algorithms": sorted(algorithms.items()),
                  "metadata": {"version": version, "api": str(Path(args.api).resolve()),
                               "library": str(Path(args.library).resolve()),
                               "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
                               "build_options": gmsh.option.getString("General.BuildOptions"),
                               "options": options, "fixture": args.fixture, "cad": args.cad,
                               "model_characteristic_length": model_length,
                               "size_field": args.size_field,
                               "import_highest_dimension_only": False if args.cad else None,
                               "fixture_embedding_restored": args.restore_fixture_embedding,
                               "trace_enabled": bool(args.trace), "messages": messages}}
        result["fingerprint"] = mesh_fingerprint(result)
        Path(args.out).write_text(json.dumps(result, ensure_ascii=False, allow_nan=False), encoding="utf-8")
        print(json.dumps({"fingerprint": result["fingerprint"], "nodes": len(nodes),
                          "elements": len(elements), "actual_algorithms": result["actual_algorithms"]}))
    finally:
        gmsh.finalize()
        for handle in dll_handles:
            handle.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--api", required=True)
    run.add_argument("--library", required=True)
    run.add_argument("--out", required=True)
    run.add_argument("--runtime-dir", action="append", default=[])
    run.add_argument("--trace")
    run.add_argument("--cad")
    run.add_argument("--restore-fixture-embedding", action="store_true")
    run.add_argument("--export-cad")
    run.add_argument("--fixture", default="rectangle")
    run.add_argument("--size", type=float, default=.3)
    run.add_argument("--size-field", choices=("step", "graded", "endpoint"))
    run.add_argument("--smoothing", type=int, default=1)
    run.add_argument("--dimension", type=int, choices=(1, 2), default=2)
    compare = commands.add_parser("compare")
    compare.add_argument("reference")
    compare.add_argument("actual")
    traces = commands.add_parser("compare-trace")
    traces.add_argument("reference")
    traces.add_argument("actual")
    traces.add_argument("--prefix", default="")
    traces.add_argument("--tolerance", type=float, default=0.)
    args = parser.parse_args()
    if args.command == "run":
        run_reference(args)
    elif args.command == "compare":
        reference, actual = [json.loads(Path(path).read_text(encoding="utf-8"))
                             for path in (args.reference, args.actual)]
        same = mesh_fingerprint(reference) == mesh_fingerprint(actual)
        print(json.dumps({"identical_mesh_and_algorithms": same}))
        raise SystemExit(0 if same else 1)
    else:
        streams = []
        for path in (args.reference, args.actual):
            events = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line]
            streams.append([{key: value for key, value in event.items() if key not in ("sequence", "schema")}
                            for event in events if event["stage"].startswith(args.prefix)])
        # 参考端按生成顺序；独立模块可按 CAD 枚举顺序重放，实体内顺序保持。
        streams = [sorted(events, key=lambda event: event["entity"]) for events in streams]
        found = first_trace_difference(*streams, args.tolerance)
        print(json.dumps({"events": [len(events) for events in streams], "first_difference": found}))
        raise SystemExit(1 if found else 0)


if __name__ == "__main__":
    main()
