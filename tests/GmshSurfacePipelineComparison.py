"""Independent BREP-only Algorithm 6 generation and identical final-mesh audit.

The reference process never supplies topology, UV, sizes, node order or mesh to
the own generator. BREP fixtures preserve geometry; both sides read the same file.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from GmshReferenceParameters import collect_reference_face_parameters


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evaluate_shape_gates(own, reference):
    gates = {}
    for region in ("all", "boundary", "interior"):
        a, b = own["shape"][region], reference["shape"][region]
        if not a["triangles"] or not b["triangles"]:
            gates[region] = dict(evaluated=False, reason="no triangles in this region")
        else:
            gates[region] = dict(evaluated=True, near=a["near_fraction"] >= b["near_fraction"]-.05,
                                 p5=a["angle_p5"] >= b["angle_p5"]-2,
                                 below25=a["below25_fraction"] <= b["below25_fraction"]+.01)
    passed = all(all(v for k, v in gate.items() if k in ("near", "p5", "below25")) for gate in gates.values())
    return gates, passed


def write_reference_vtp(gmsh, output, parameter_diagnostics=None):
    tags, coordinates, _ = gmsh.model.mesh.getNodes()
    points = [tuple(float(x) for x in coordinates[k:k+3]) for k in range(0, len(coordinates), 3)]
    index = {int(tag): k for k, tag in enumerate(tags)}
    triangles, corner_uv, face_ids, lines, edge_ids = [], [], [], [], []
    surfaces = gmsh.model.getEntities(2)
    body_ids = {}
    for _, body in gmsh.model.getEntities(3):
        for dim, entity in gmsh.model.getBoundary([(3,body)], oriented=False, recursive=False):
            if dim == 2:
                body_ids[entity] = body-1
    bodies = []
    for dim, entity in surfaces:
        surface_type = gmsh.model.getType(2, entity)
        diagnostics = {}
        types, _, nodes = gmsh.model.mesh.getElements(dim, entity)
        required_coordinates = {int(tag): points[index[int(tag)]] for group in nodes for tag in group}
        parameters = collect_reference_face_parameters(gmsh, entity, diagnostics, required_coordinates)
        if parameter_diagnostics is not None:
            parameter_diagnostics.append(dict(face=entity, **diagnostics))
        for kind, ids in zip(types, nodes):
            if kind != 2:
                raise ValueError("reference did not return linear triangles")
            for k in range(0, len(ids), 3):
                tri = [index[int(v)] for v in ids[k:k+3]]
                triangles.append(tri)
                xyz = [x for v in tri for x in points[v]]
                if any(int(node) not in parameters for node in ids[k:k+3]):
                    raise ValueError(f"reference face {entity}: triangle node has no face parameter")
                uv = [value for node in ids[k:k+3] for value in parameters[int(node)]]
                # 同一物理seam节点的直接反求只给一侧；选择靠近片内点的周期副本。
                if surface_type in ("Cylinder", "Sphere", "Cone", "Torus"):
                    center_xyz = [sum(xyz[axis+3*k] for k in range(3))/3 for axis in range(3)]
                    center = gmsh.model.getParametrization(2, entity, center_xyz)
                    for k in range(3):
                        for axis in ((0,1) if surface_type == "Torus" else (0,)):
                            value = uv[2*k+axis]
                            period = 2*math.pi
                            uv[2*k+axis] = min((value-period,value,value+period), key=lambda v:abs(v-center[axis]))
                        if surface_type == "Sphere" and abs(abs(uv[2*k+1])-math.pi/2)<1.e-10:
                            uv[2*k] = float(center[0])
                corner_uv.append(uv)
                face_ids.append(entity-1)
                bodies.append(body_ids.get(entity, -1))
    # Gmsh OCC BREP indexed edge tags are recorded; mapping is checked by CAD audit.
    for _, entity in gmsh.model.getEntities(1):
        types, _, nodes = gmsh.model.mesh.getElements(1, entity)
        for kind, ids in zip(types, nodes):
            if kind != 1:
                raise ValueError("nonlinear reference boundary")
            for k in range(0, len(ids), 2):
                lines.append([index[int(v)] for v in ids[k:k+2]])
                edge_ids.append(entity-1)
    root = ET.Element("VTKFile", type="PolyData", version="0.1", byte_order="LittleEndian")
    piece = ET.SubElement(ET.SubElement(root, "PolyData"), "Piece", NumberOfPoints=str(len(points)), NumberOfVerts="0", NumberOfLines=str(len(lines)), NumberOfStrips="0", NumberOfPolys=str(len(triangles)))
    def array(parent, name, values, kind="Float64", components=1):
        attrs = dict(type=kind, format="ascii", NumberOfComponents=str(components))
        if name:
            attrs["Name"] = name
        node = ET.SubElement(parent, "DataArray", **attrs)
        node.text = " ".join(str(v) for v in values)
    array(ET.SubElement(piece, "Points"), None, [x for p in points for x in p], components=3)
    for label, cells in (("Lines", lines), ("Polys", triangles)):
        parent = ET.SubElement(piece, label)
        array(parent, "connectivity", [v for c in cells for v in c], "Int64")
        array(parent, "offsets", [(k+1)*len(cells[0]) for k in range(len(cells))] if cells else [], "Int64")
    data = ET.SubElement(piece, "CellData")
    array(data, "CadFaceId", [-1]*len(lines)+face_ids, "Int32")
    array(data, "CadBodyId", [-1]*len(lines)+bodies, "Int32")
    array(data, "CadEdgeId", edge_ids+[-1]*len(triangles), "Int32")
    array(data, "CadCornerUV", [0.]*(6*len(lines))+[v for uv in corner_uv for v in uv], components=6)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return len(triangles)


def reference(args):
    from GmshReproductionComparison import load_reference_api
    gmsh = load_reference_api(args.api, args.library)
    gmsh.initialize([], readConfigFiles=False)
    settings = {}
    try:
        if gmsh.option.getString("General.Version") != "4.15.2":
            raise ValueError("reference must be Gmsh 4.15.2")
        settings = {"General.NumThreads": 1, "Mesh.MaxNumThreads1D": 1, "Mesh.MaxNumThreads2D": 1,
                    "Mesh.Algorithm": 6, "Mesh.ElementOrder": 1, "Mesh.MeshSizeMin": 0,
                    "Mesh.MeshSizeMax": args.size, "Mesh.MeshSizeFactor": 1,
                    "Mesh.MeshSizeFromPoints": int(args.point_size > 0), "Mesh.MeshSizeFromCurvature": args.curvature,
                    "Mesh.MeshSizeExtendFromBoundary": args.extension, "Mesh.Smoothing": 1,
                    "Mesh.AlgorithmSwitchOnFailure": 0, "Mesh.RandomSeed": 1,
                    "Geometry.OCCAutoEmbed": 1}
        for key, value in settings.items():
            gmsh.option.setNumber(key, value)
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.logger.start()
        start = time.perf_counter()
        gmsh.open(str(args.cad))
        if args.point_size > 0:
            gmsh.model.mesh.setSize(gmsh.model.getEntities(0), args.point_size)
        loaded = time.perf_counter()
        gmsh.model.mesh.generate(2)
        generated = time.perf_counter()
        for key in ("Mesh.RandomFactor", "Mesh.LcIntegrationPrecision", "Mesh.SmoothRatio", "Mesh.MinimumCurvePoints", "Mesh.MinimumCirclePoints", "Mesh.OldInitialDelaunay2D"):
            settings[key] = gmsh.option.getNumber(key)
        logs = list(gmsh.logger.get())
        if any("meshadapt" in line.lower() or "switching" in line.lower() or "error" in line.lower() for line in logs):
            raise ValueError("reference failed or switched algorithm: " + "\n".join(logs))
        failures = list(gmsh.model.mesh.getLastEntityError())
        face_status = []
        for dim, entity in gmsh.model.getEntities(2):
            _, element_ids, _ = gmsh.model.mesh.getElements(dim, entity)
            count = sum(len(ids) for ids in element_ids)
            face_status.append(dict(face=entity, triangles=count, complete=count > 0 and (dim, entity) not in failures))
        (args.output / "reference-faces.json").write_text(json.dumps(face_status, indent=2), encoding="utf-8")
        if not all(face["complete"] for face in face_status):
            raise ValueError("reference has failed or empty CAD faces")
        export_start = time.perf_counter()
        parameter_diagnostics = []
        count = write_reference_vtp(gmsh, args.output / "reference.vtp", parameter_diagnostics)
        export_end = time.perf_counter()
        report = dict(complete=True, version="4.15.2", settings=settings, triangles=count,
                      cad_sha256=digest(args.cad), library_sha256=digest(args.library),
                      import_seconds=loaded-start, generation_seconds=generated-loaded,
                      export_seconds=export_end-export_start,
                      actual_algorithms=[6], algorithm_evidence="switch disabled; inspected generation log",
                      uniform_point_size=args.point_size,
                      corner_uv_source="SDK native face parameters/direct inverse selected by forward XYZ residual; periodic branches per corner",
                      reference_parameter_script_sha256=digest(Path(__file__).with_name("GmshReferenceParameters.py")),
                      parameter_diagnostics=parameter_diagnostics,
                      log=logs)
        (args.output / "reference-generation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    except Exception as error:
        report = dict(complete=False, error=str(error), settings=settings, cad_sha256=digest(args.cad),
                      version="4.15.2", library_sha256=digest(args.library), actual_algorithms=[6],
                      log=list(gmsh.logger.get()))
        (args.output / "reference-generation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        raise
    finally:
        gmsh.finalize()


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    own_env = dict(os.environ)
    # Deliberately remove the frozen/reference data location from the own process.
    own_env["PATH"] = os.pathsep.join(args.runtime + [os.environ.get("SystemRoot", "C:/Windows")+"/System32"])
    own_env["PYTHONPATH"] = ""
    fixtures = [(name, args.fixtures / (name + ".brep")) for name in ("rectangle", "hole", "narrow_band")] if not args.case else []
    for specification in args.case:
        name, cad = specification.split("=", 1)
        if not name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in name):
            raise ValueError("case name must be a safe filename")
        fixtures.append((name, Path(cad)))
    if len({name for name, _ in fixtures}) != len(fixtures):
        raise ValueError("duplicate case names")
    for _, cad in fixtures:
        if not cad.is_file():
            raise ValueError("missing CAD fixture: " + str(cad))
    sizes = {name: float(size) for name, size in (s.split("=", 1) for s in args.case_size)}
    if set(sizes) - {name for name, _ in fixtures}:
        raise ValueError("case size names are not in the case list")
    cases = []
    for name, cad in fixtures:
        size = sizes.get(name, args.size)
        if not math.isfinite(size) or size <= 0:
            raise ValueError("case size must be finite and positive")
        for extension in args.extensions:
            folder = args.output / f"{name}-extend{extension}"
            folder.mkdir()
            # Independent input directory contains exactly BREP; no trace/UV/node metadata.
            independent = folder / "input"
            independent.mkdir()
            isolated_cad = independent / "model.brep"
            isolated_cad.write_bytes(cad.read_bytes())
            own = folder / "own"
            own_command = [str(args.executable), "generate", str(isolated_cad), str(size), str(extension), str(own), "--point-size", str(args.point_size), "--curvature", str(args.curvature)]
            if args.no_stages:
                own_command.append("--no-stages")
            command = [sys.executable, __file__, "--reference", "--api", str(args.api), "--library", str(args.library), "--cad", str(isolated_cad), "--size", str(size), "--extension", str(extension), "--point-size", str(args.point_size), "--curvature", str(args.curvature), "--output", str(folder)]
            own_run = subprocess.run(own_command, env=own_env, capture_output=True, text=True)
            (folder / "own-run.log").write_text(own_run.stdout + own_run.stderr, encoding="utf-8")
            reference_run = subprocess.run(command, capture_output=True, text=True)
            (folder / "reference-run.log").write_text(reference_run.stdout + reference_run.stderr, encoding="utf-8")
            audit_run = subprocess.run([str(args.executable), "audit", str(isolated_cad), str(folder / "reference.vtp"), str(folder / "reference-audit.json")], env=own_env, capture_output=True, text=True) if reference_run.returncode == 0 else None
            if audit_run:
                (folder / "reference-audit.log").write_text(audit_run.stdout + audit_run.stderr, encoding="utf-8")
            if own_run.returncode or reference_run.returncode or audit_run is None or audit_run.returncode:
                row = dict(case=name, extension=extension, size=size, complete=False, shape_passed=False,
                           own_exit=own_run.returncode, reference_exit=reference_run.returncode,
                           reference_audit_exit=None if audit_run is None else audit_run.returncode,
                           cad_sha256=digest(isolated_cad), error="see run/audit logs and per-face reports")
                cases.append(row)
                (args.output / "comparison.json").write_text(json.dumps(dict(complete=False, cases=cases), indent=2), encoding="utf-8")
                print(name, extension, "generation/audit failed; evidence retained", flush=True)
                continue
            ours = json.loads((own / "audit.json").read_text())
            theirs = json.loads((folder / "reference-audit.json").read_text())
            gates, passed = evaluate_shape_gates(ours, theirs)
            row = dict(case=name, extension=extension, size=size, complete=True, shape_passed=passed, gates=gates,
                       own=ours, reference=theirs, cad_sha256=digest(isolated_cad),
                       own_vtp_sha256=digest(own / "own.vtp"), reference_vtp_sha256=digest(folder / "reference.vtp"))
            cases.append(row)
            print(name, extension, "shape_passed", passed, "near", ours["shape"]["all"]["near_fraction"], theirs["shape"]["all"]["near_fraction"], flush=True)
            (args.output / "comparison.json").write_text(json.dumps(dict(complete=False, cases=cases), indent=2), encoding="utf-8")
    report = dict(complete=all(c["complete"] for c in cases), shape_passed=all(c["shape_passed"] for c in cases), cases=cases,
                  uniform_point_size=args.point_size, curvature_circle_elements=args.curvature,
                  geometry_acceptance="finite sample distances reported; no user tolerance supplied",
                  own_executable_sha256=digest(args.executable), script_sha256=digest(__file__),
                  reference_parameter_script_sha256=digest(Path(__file__).with_name("GmshReferenceParameters.py")),
                  independent_input_contract="own executable receives isolated BREP plus size/extension; no reference artifacts",
                  performance_acceptance="single generation measurements; stage D three-run gate not evaluated")
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if report["shape_passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", action="store_true")
    parser.add_argument("--api", type=Path, required=True)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--size", type=float, default=.3)
    parser.add_argument("--extension", type=int, default=1)
    parser.add_argument("--cad", type=Path)
    parser.add_argument("--fixtures", type=Path)
    parser.add_argument("--executable", type=Path)
    parser.add_argument("--runtime", action="append", default=[])
    parser.add_argument("--case", action="append", default=[], help="case-name=BREP path; default is the stage A fixture set")
    parser.add_argument("--case-size", action="append", default=[], help="case-name=maximum size")
    parser.add_argument("--extensions", nargs="+", type=int, choices=(0, 1), default=[0, 1])
    parser.add_argument("--point-size", type=float, default=0, help="explicit uniform CAD vertex size; zero disables point sizing")
    parser.add_argument("--curvature", type=float, default=0, help="elements per curvature circle; zero disables curvature sizing")
    parser.add_argument("--no-stages", action="store_true")
    args = parser.parse_args()
    if args.reference:
        reference(args)
    else:
        sys.exit(main(args))
