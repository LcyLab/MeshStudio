"""实际普通meshGenerator初始至面转换前缀及BDS拓扑对照，不冻结结果连接。"""
import argparse
import ctypes
import hashlib
import itertools
import json
import os
from pathlib import Path
import random
import subprocess
from GmshInitialDomainComparison import create_fixture, restore_embedding
from GmshReproductionComparison import load_reference_api, first_trace_difference
from GmshFrontQueueComparison import normalize_queue
from GmshFrontPointPlacementComparison import reference_front_point, normalize_front_points
from GmshFrontCavityComparison import reference_front_cavity, normalize_cavity
from GmshFrontPointInsertionComparison import reference_initial_point_insertion, normalize_point_insertion
from GmshFrontGenerationComparison import reference_initial_generation, normalize_generation
from GmshFrontMeshTransferComparison import reference_transfer, generation_transfer_input
from GmshSurfaceSmoothingComparison import reference_smoothing, generation_smoothing_input, validate_result as validate_smoothing


def normalize(document):
    for row in document["points"]:
        for k in (2, 3, 4, 5, 6, 9, 10): row[k] = float(row[k])
    return document


def normalize_face_mesh(document):
    for row in document["vertices"]:
        for k in range(5, 11): row[k] = float(row[k])
    for row in document["parametric_coordinates"]: row[1:] = map(float, row[1:])
    return document


def normalize_front_nodes(document):
    for row in document["nodes"]: row[1:] = map(float, row[1:])
    for row in document["reparameterizations"]: row[1:] = map(float, row[1:])
    for row in document["embedded_queries"]: row[3:] = map(float, row[3:])
    return document


def write_input(path, lines):
    path.write_text("".join(" ".join(str(x) if isinstance(x, (str, int)) else format(x, ".17g") for x in row)+"\n" for row in lines), encoding="utf-8")


def compare(args, directory, name, lines, expected, cad=None):
    source = directory/(name+"-input.tsv"); write_input(source, lines)
    target = directory/(name+"-own.json")
    (directory/(name+"-reference.json")).write_text(json.dumps(expected, allow_nan=False), encoding="utf-8")
    # 原30秒限制针对初始化叶；完整动态观察还包括逐轮集合/诊断输出，单独保留较长运行保护。
    subprocess.run([args.own_executable, *([str(cad)] if args.metric_cad and cad is not None else []), str(source), str(target)], check=True,
                   timeout=300 if args.check_front_generation else 30)
    actual = json.loads(target.read_text(encoding="utf-8"))
    if "node_sizes" in expected:
        normalize_front_nodes(actual["node_sizes"])
        for row in actual["front_triangles"]["triangles"]: row[3:7] = map(float,row[3:7])
        if "front_queue" in actual: normalize_queue(actual["front_queue"])
        if "front_generation" in actual: normalize_generation(actual["front_generation"])
        if "front_points" in actual: normalize_front_points(actual["front_points"])
        if args.check_front_cavity and "front_points" in actual:
            for item in actual["front_points"]: normalize_cavity(item["cavity"])
        if args.check_front_insertion and "front_points" in actual:
            for item in actual["front_points"]:
                if "insertion" in item: normalize_point_insertion(item["insertion"])
    elif "nodes" in expected: normalize_front_nodes(actual)
    elif "bds_to_vertex" in expected: normalize_face_mesh(actual)
    elif "bds" in expected: normalize(actual["bds"])
    else: normalize(actual)
    difference = first_trace_difference([{"stage": "bds_conversion", **expected}], [{"stage": "bds_conversion", **actual}], 0.)
    pool = expected.get("node_sizes",expected.get("bds",expected))
    result = {"name": name, "points": len(pool.get("points", pool.get("vertices", pool.get("nodes", [])))), "edges": len(pool.get("edges", [])), "triangles": len(pool.get("triangles", [])),
              "input_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "first_difference": difference}
    if "node_sizes" in expected:
        triangles = expected["front_triangles"]
        result.update({"front_triangles":len(triangles["triangles"]),"ordered":len(triangles["ordered"]),
                       "rejected_duplicates":len(triangles["rejected_duplicates"]),
                       "connected_edges":sum(n >= 0 for t in triangles["triangles"] for n in t[7:10])//2})
        nodes = expected["node_sizes"]
        result.update({"front_nodes":len(nodes["nodes"]),"embedded_queries":len(nodes["embedded_queries"]),
                       "different_dual_sizes":sum(row[3] != row[4] for row in nodes["nodes"])})
        if "front_queue" in expected:
            queue = expected["front_queue"]
            result.update(initial_active=len(queue["initial_active"]),queue_pops=len(queue["pops"]),
                          insertion_eligible=sum(p[5] for p in queue["pops"]),inactive_pops=sum(p[2] and not p[3] for p in queue["pops"]))
        if "front_generation" in expected:
            generation = expected["front_generation"]
            result.update(generation_stats=generation["stats"],new_triangle_pops=sum(e["selection"][0] >= len(triangles["triangles"]) for e in generation["events"]))
        if "smoothing" in expected:
            validate_smoothing(actual["smoothing"],1)
            result["smoothing_counts"] = {key:expected["smoothing"][key] for key in ("visits","eligible","writes","queries")}
        if "front_points" in expected:
            candidates = [p["candidate"] for p in expected["front_points"]]
            result.update(front_points=len(candidates),point_surface_queries=sum(len(c["queries"]) for c in candidates),
                          corrected_points=sum(c["values"][33] for c in candidates))
            if args.check_front_cavity:
                cavities = [p["cavity"] for p in expected["front_points"]]
                result.update(front_cavities=len(cavities),cavity_triangles=sum(len(c["cavity"]) for c in cavities),shell_edges=sum(len(c["shell"]) for c in cavities),
                              containing_missing=sum(c["containing"] < 0 for c in cavities),searches=sum(bool(c["search"]["visits"]) for c in cavities))
            if args.check_front_insertion:
                insertions = [p["insertion"] for p in expected["front_points"] if "insertion" in p]
                result.update(first_insertions=len(insertions),first_inserted=sum(i["inserted"] for i in insertions),first_insertion_codes=[i["code"] for i in insertions])
    if "nodes" in expected:
        result.update({"front_nodes": len(expected["nodes"]), "reparameterizations": len(expected["reparameterizations"]),
                       "embedded_queries": len(expected["embedded_queries"]), "internal_edges": len(expected["internal_edges"]),
                       "different_dual_sizes": sum(row[3] != row[4] for row in expected["nodes"])})
    if "bds_to_vertex" in expected:
        result.update({"created_local_nodes": len(expected["conversion"]["created_local_nodes"]),
                       "skipped_deleted": len(expected["conversion"]["skipped_deleted"]),
                       "collapsed": len(expected["conversion"]["collapsed"]),
                       "copies": len(expected["equivalence"]), "parametric_coordinates": len(expected["parametric_coordinates"])})
    if "registered" in expected:
        result.update({"registered": len(expected["registered"]), "not_recovered": len(expected["not_recovered"]), "native_fatal": expected.get("fatal",False),
                       "had_fatal_recovery": expected.get("had_fatal_recovery",False),
                       "curve_attempts": len(expected["attempts"]), "successful_swaps": sum(int(ok) for a in expected["attempts"] for s in a["scans"] for _,ok in s["attempts"])})
    if "fills" in expected:
        result.update({"fills": len(expected["fills"]), "interior_triangles": expected["interior_triangles"],
                       "visits": sum(len(f["visits"]) for f in expected["fills"]),
                       "redundant_visits": sum(int(not expanded) for f in expected["fills"] for _,expanded in f["visits"]),
                       "cleared_exterior_triangles": len(expected["cleared_exterior"]),
                       "unclassified_live_triangles": sum(int(not r[3] and r[5] == -1) for r in pool["triangles"])})
    if "queries" in expected:
        result.update({"size_queries": len(expected["queries"]), "unconstrained_points": len(expected["unconstrained"])})
    if "metric_queries" in expected:
        result.update({"metric_queries": expected["metric_queries"], "metric_swaps": expected["swaps"],
                       "metric_scans": len(expected["scans"]), "configurations": len(expected["configurations"]),
                       "metric_attempts": sum(len(s["attempts"]) for s in expected["scans"]),
                       "rejected_swaps": sum(a["decision"] == 4 for s in expected["scans"] for a in s["attempts"])})
    if "first_cleanup" in expected:
        result.update({"unclassified_triangles": len(expected["unclassified_triangles"]), "orphan_edges": len(expected["orphan_edges"]),
                       "removed_outer_points": len(expected["removed_outer_points"]),
                       "triangles_reordered": [i for i in expected["first_cleanup"]["triangles"] if i >= 0] != sorted(i for i in expected["first_cleanup"]["triangles"] if i >= 0),
                       "edges_reordered": [i for i in expected["second_cleanup"]["edges"] if i >= 0] != sorted(i for i in expected["second_cleanup"]["edges"] if i >= 0)})
    if difference: raise ValueError(name+" BDS differs: "+json.dumps(difference))
    return result


def create_metric_fixture(gmsh, name, path):
    """非周期Bezier支撑面的各向异性参数化/变度量输入；不冻结结果连接。"""
    gmsh.initialize([], readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal", 0.)
        gmsh.model.add(name)
        points = []
        for j in range(4):
            for i in range(4):
                u, v = i/3., j/3.
                points.append(gmsh.model.occ.addPoint(4.*u+1.5*v, 3.*v,
                    1.2*u*v+0.4*u*u if name == "warped-bspline" else 0.))
        gmsh.model.occ.addBSplineSurface(points, 4, degreeU=3, degreeV=3)
        gmsh.model.occ.synchronize(); gmsh.write(str(path))
    finally: gmsh.finalize()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"): parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    parser.add_argument("--check-boundary",action="store_true",help="continue own initial/conversion prefix through actual ordinary boundary registration/recovery pass")
    parser.add_argument("--check-domain",action="store_true",help="continue through ordinary BDS domain tags before embedded recovery; compare classification origins and actual LIFO events")
    parser.add_argument("--check-embedded",action="store_true",help="continue through actual post-domain embedded pass2, before node sizes or cleanup")
    parser.add_argument("--check-node-sizes",action="store_true",help="continue through actual original MVertex BGM size initialization before cleanup; own size evaluation is not frozen")
    parser.add_argument("--check-interior",action="store_true",help="continue through ordinary two cleanups/classification propagation/outer point removal, before metric legalization")
    parser.add_argument("--check-metric",action="store_true",help="continue through actual ordinary delaunayizeBDS; compare dynamic scans, configurations, metric circles, qualification and complete pools")
    parser.add_argument("--check-face-mesh",action="store_true",help="continue actual ordinary meshGenerator through BDS2GMSH before front sizing/generation; own complete prefix computes output connections")
    parser.add_argument("--check-front-node-sizes",action="store_true",help="continue ordinary full prefix into actual front node stage; reparameterization values and native pointer iteration order remain frozen")
    parser.add_argument("--metric-cad",action="store_true",help="CAD replay independently evaluates derivatives from the same BREP; frozen derivative values are ignored")
    parser.add_argument("--check-front-triangles",action="store_true",help="continue actual native MTri3 construction/AllTris ordering/connectTriangles after own ordinary full prefix")
    parser.add_argument("--front-boundary-extension",type=int,choices=(0,1),default=1,help="fixed native Mesh.MeshSizeExtendFromBoundary setting for triangle baseline")
    parser.add_argument("--check-front-queue",action="store_true",help="compare initial native activity set/first edges and drain with no point insertion after own ordinary complete prefix")
    parser.add_argument("--check-front-points",action="store_true",help="continue eligible initial queue triangles through actual metric candidate and OCC circle correction; requires CAD replay, no insertion feedback")
    parser.add_argument("--check-front-cavity",action="store_true",help="continue each eligible initial candidate through native circle/DFS/search and rollback, no actual insertion feedback")
    parser.add_argument("--check-front-insertion",action="store_true",help="actual insertAPoint for first eligible initial candidate after own full prefix; later initial drain isolated from that commit")
    parser.add_argument("--check-front-generation",action="store_true",help="own complete ordinary prefix enters actual dynamic generation; no frozen initial front pool or new candidate/feedback")
    parser.add_argument("--check-smoothing",action="store_true",help="continue own generation/transfer through classified smoothing with independent raw CAD parameter source")
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    if args.check_smoothing: args.check_front_generation = True
    if args.check_front_generation:
        if args.check_front_points or args.check_front_cavity or args.check_front_insertion: parser.error("dynamic generation and isolated initial drain are separate modes")
        if not args.metric_cad: parser.error("dynamic generation requires --metric-cad")
        args.check_front_queue = True
    if args.check_front_cavity and args.check_front_insertion: parser.error("cavity drain and isolated first insertion are separate comparison modes")
    if args.check_front_cavity: args.check_front_points = True
    if args.check_front_insertion: args.check_front_points = True
    if args.check_front_points:
        if not args.metric_cad: parser.error("--check-front-points requires --metric-cad and actual CAD replay")
        args.check_front_queue = True
    if args.check_front_queue: args.check_front_triangles = True
    if args.check_front_triangles: args.check_front_node_sizes = True
    if args.check_front_node_sizes: args.check_face_mesh = True
    if args.check_face_mesh: args.check_metric = True
    if args.metric_cad and not args.check_metric: parser.error("--metric-cad requires --check-metric or --check-face-mesh")
    if args.check_metric: args.check_interior = True
    if args.check_interior: args.check_node_sizes = True
    if args.check_node_sizes: args.check_embedded = True
    if args.check_embedded: args.check_domain = True
    if args.check_domain: args.check_boundary = True
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    os.environ['PATH'] = os.pathsep.join([*args.runtime_dir,os.environ.get('PATH','')])
    gmsh = load_reference_api(args.api, args.library); pointer = ctypes.POINTER(ctypes.c_double)
    topology = gmsh.lib.meshstudioReferenceBdsTopology
    topology.argtypes = [ctypes.c_int, pointer, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_char), ctypes.c_int]; topology.restype = ctypes.c_int
    native = gmsh.lib.meshstudioReferenceOrdinaryFrontQueue if args.check_front_queue else gmsh.lib.meshstudioReferenceOrdinaryFrontTriangles if args.check_front_triangles else gmsh.lib.meshstudioReferenceOrdinaryFrontNodeSizes if args.check_front_node_sizes else gmsh.lib.meshstudioReferenceOrdinaryFaceMesh if args.check_face_mesh else gmsh.lib.meshstudioReferenceOrdinaryBdsMetric if args.check_metric else gmsh.lib.meshstudioReferenceOrdinaryBdsInterior if args.check_interior else gmsh.lib.meshstudioReferenceOrdinaryBdsNodeSizes if args.check_node_sizes else gmsh.lib.meshstudioReferenceOrdinaryBdsEmbedded if args.check_embedded else gmsh.lib.meshstudioReferenceOrdinaryBdsDomain if args.check_domain else gmsh.lib.meshstudioReferenceOrdinaryBdsBoundary if args.check_boundary else gmsh.lib.meshstudioReferenceOrdinaryBdsConversion
    native.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]; native.restype = ctypes.c_int
    copies = gmsh.lib.meshstudioReferenceInitialChart
    copies.argtypes = [ctypes.c_int, pointer, ctypes.c_int]; copies.restype = ctypes.c_int
    recovery = gmsh.lib.meshstudioReferenceOrdinaryConstraintRecovery
    recovery.argtypes = [ctypes.c_int, pointer, pointer, pointer, pointer, pointer, pointer, ctypes.c_int, ctypes.c_int]; recovery.restype = ctypes.c_int
    uses = gmsh.lib.meshstudioReferenceFaceUses; uses.argtypes = [ctypes.c_int, pointer, ctypes.c_int]; uses.restype = ctypes.c_int
    report = {"scope": "actual ordinary BDS conversion after own Walk/recovery/Color/UV prefix, plus pure native BDS append",
              "frozen_inputs": ["1D", "PolyMesh UV and copy order", "BDS chart ID/physical map/reparam UV/XYZ/classification", "physical CAD constraint chains", "outer CAD evaluations"],
              "result_connectivity_frozen": False, "own_cad_adapter_validated": False, "bds_recovery_validated": False,
              "bds_metric_legalization_validated": False, "topology_cases": [], "ordinary_cases": [], "deferred_periodic_faces": [], "complete": False,
              "bds_boundary_driver_validated": False, "bds_embedded_recovery_validated": False,
              "bds_ordinary_domain_validated": False,
              "bds_node_sizes_validated": False,
              "bds_interior_preparation_validated": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    if args.check_boundary:
        report["scope"] = "actual ordinary meshGenerator boundary registry/pass2 after own Walk/recovery/Color/UV/BDS conversion prefix"
        report["frozen_inputs"].extend(["CAD curve enumeration/physical segment chains", "curve degeneracy and geometric endpoint physical IDs", "refineAll flag"])
    if args.check_domain: report["scope"] = "actual ordinary meshGenerator BDS domain classification after own full initial/conversion/boundary prefix, including marker identity and LIFO visits"
    if args.check_embedded: report["scope"] = "actual ordinary meshGenerator post-domain embedded curve recovery after own full initial/conversion/boundary/domain prefix, before node size initialization or cleanup"
    if args.check_node_sizes:
        report["scope"] = "actual ordinary meshGenerator BDS node BGM initialization after own full initial/conversion/boundary/domain/embedded prefix, before cleanup"
        report["frozen_inputs"].extend(["original recovered MVertex CAD entity/physical XYZ/curve parameter", "model characteristic length"])
        report["node_size_values_frozen"] = False
    if args.check_interior: report["scope"] = "actual ordinary meshGenerator two BDS cleanups, original edge-order classification propagation and outer point removal after own full initial/conversion/recovery/domain/size prefix, before metric legalization"
    if args.check_metric:
        report["scope"] = "actual ordinary meshGenerator metric legalization after own complete ordinary initial/recovery/domain/size/interior prefix; full dynamic scans and BDS pool state"
        report["metric_derivatives_frozen"] = not args.metric_cad
        report["metric_cad_derivatives_validated"] = False
        if not args.metric_cad: report["frozen_inputs"].append("CAD du/dv at actual edge midpoint queries (query order and UV checked)")
    if args.check_face_mesh:
        report["scope"] = "actual ordinary meshGenerator BDS2GMSH after own full initial/recovery/domain/size/interior/metric prefix; original MVertex metadata frozen"
        report["frozen_inputs"].append("original MVertex CAD entity/physical XYZ/parameters/stored edge size")
        report["bds_to_face_mesh_validated"] = False
        report["complete_front_generation_validated"] = False
        report["vertex_identity_comparison"] = "original physical IDs retained; new native pointers canonicalized in actual creation order, raw process-global MVertex numbers not compared"
    if args.check_front_node_sizes:
        report["scope"] = "actual ordinary buildMeshGenerationDataStructures node stage after own full prefix, before MTri3 construction; raw CAD parameters/native pointer order frozen"
        report["frozen_inputs"].extend(["raw front reparameterization results", "native vSizesMap pointer iteration order", "embedded first MVertex/prescribed point size"])
        report["front_node_sizes_validated"] = False
        report["front_node_size_values_frozen"] = False
        report["front_reparameterization_adapter_validated"] = False
    if args.check_front_triangles:
        report["scope"] = "actual ordinary buildMeshGenerationDataStructures MTri3 construction/AllTris insertion/connectTriangles after own full prefix; raw CAD parameters/native pointer order and resolved Extend1dMeshIn2dSurfaces frozen; no frozen triangle radii/order/neighbors"
        report["front_triangle_initialization_validated"] = False
        report["front_triangle_values_frozen"] = False
        report["front_boundary_extension"] = args.front_boundary_extension
        report["duplicate_set_rejection_actual_validated"] = False
    if args.check_front_queue:
        report["scope"] = "actual native buildMeshGenerationDataStructures states plus native isActive/compareTri3Ptr/sets; observer initialization/pop control statements mirror bowyerWatsonFrontal; drain without candidate/insertion after own ordinary complete prefix"
        report["front_queue_validated"] = False
        report["front_queue_values_frozen"] = False
        report["front_point_insertion_feedback_validated"] = False
    if args.check_front_points:
        report["scope"] = "actual native initial prefix/queue plus actual optimalPointFrontal/B on each eligible triangle; own complete ordinary prefix/queue/placement and actual OCC metric/Value; no cavity or insertion feedback"
        report["front_point_placement_validated"] = False
        report["front_point_triangle_state_frozen"] = False
        report["front_point_values_frozen"] = False
    if args.check_front_cavity:
        report["scope"] = "own complete ordinary prefix/queue/point/cavity; reference actual circle/DFS/invMap/search on corresponding native-prefix state; initial insertAPoint branch control mirrored, no insertion commit/feedback"
        report["front_cavity_validated"] = False
        report["front_cavity_values_frozen"] = False
        report["parametric_domain_gate_applied"] = False
        report["actual_insertion_feedback_validated"] = False
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    try:
        gmsh.initialize([], readConfigFiles=False)
        try:
            # Edge creation order depends on triangle corner order; test all six.
            ids = [-4, 0, 7, 19, 28]
            for ordering in itertools.permutations(range(3)):
                for reverse_points in (False, True):
                    points = [[id, 0, i*.7, i*.2, i*.9, i*.4, i*.1, 2 if i == 0 else 0, 11-i, .2+i*.3, .4+i*.7, -1] for i, id in enumerate(ids)]
                    if reverse_points: points.reverse()
                    triangles = [[ids[i] for i in ordering], [ids[1], ids[3], ids[2]], [ids[2], ids[3], ids[4]]]
                    triangles.append(triangles[0]) # native add_triangle does not deduplicate.
                    point_buffer = (ctypes.c_double*(12*len(points)))(*(x for row in points for x in row))
                    triangle_buffer = (ctypes.c_int*(3*len(triangles)))(*(x for row in triangles for x in row))
                    output = ctypes.create_string_buffer(1024*1024)
                    if topology(len(points), point_buffer, len(triangles), triangle_buffer, output, len(output)) != len(triangles): raise ValueError("native BDS append failed")
                    expected = normalize(json.loads(output.value.decode("utf-8")))
                    name = "append-"+"".join(map(str, ordering))+"-reverse"+str(int(reverse_points))
                    report["topology_cases"].append(compare(args, directory, name, [["topology", len(points)], *points, [len(triangles)], *triangles], expected))
            # Arbitrary ID gaps, negative outer IDs, face order and repeated faces.
            generator = random.Random(4152)
            for index in range(12):
                ids = [-4, -3, -2, -1, *generator.sample(range(0, 1000), 16)]
                points = [[id, 0, i*.031, i*.067, i*.017, i*.021, i*.009, 2, 5, .17+i*.04, .11+i*.02, -1] for i, id in enumerate(ids)]
                triangles = [[ids[0], ids[i], ids[i+1]] for i in range(1, len(ids)-1)]
                generator.shuffle(points); generator.shuffle(triangles)
                point_buffer = (ctypes.c_double*(12*len(points)))(*(x for row in points for x in row))
                triangle_buffer = (ctypes.c_int*(3*len(triangles)))(*(x for row in triangles for x in row))
                output = ctypes.create_string_buffer(1024*1024)
                if topology(len(points), point_buffer, len(triangles), triangle_buffer, output, len(output)) != len(triangles): raise ValueError("native BDS fan failed")
                expected = normalize(json.loads(output.value.decode("utf-8")))
                report["topology_cases"].append(compare(args, directory, "fan-"+str(index), [["topology", len(points)], *points, [len(triangles)], *triangles], expected))
        finally: gmsh.finalize()
        extra = {name: directory/(name+".brep") for name in ("slit-embedded-point", "embedded-line-pressure", "two-holes")}
        for name, path in extra.items(): create_fixture(gmsh, name, path)
        if args.check_metric:
            for name in ("parameter-shear", "warped-bspline"):
                extra[name] = directory/(name+".brep"); create_metric_fixture(gmsh, name, extra[name])
        for fixture in ("rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded", *extra):
            for maximum in (.3, .8):
                for graded in (False, True):
                    gmsh.initialize([], readConfigFiles=False)
                    try:
                        gmsh.model.add(fixture); cad = extra.get(fixture, Path(args.frozen_dir)/(fixture+".brep"))
                        gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize(); restore_embedding(gmsh, fixture)
                        for name, value in {"General.Terminal": 0., "General.NumThreads": 1., "Mesh.Algorithm": 6., "Mesh.MeshSizeMax": maximum,
                                            "Mesh.MeshSizeFromPoints": 0., "Mesh.MeshSizeFromCurvature": 0., "Mesh.MeshSizeFactor": 1., "Mesh.OldInitialDelaunay2D": 0.,
                                            "Mesh.MeshSizeExtendFromBoundary":args.front_boundary_extension}.items(): gmsh.option.setNumber(name, value)
                        if not gmsh.option.getString("General.Version").startswith("4.15.2"): raise ValueError("wrong version")
                        if fixture == "slit-embedded-point":
                            top = []
                            for dim, tag in gmsh.model.getEntities(1):
                                lo, hi = gmsh.model.getParametrizationBounds(dim, tag); xyz = gmsh.model.getValue(dim, tag, [float((lo[0]+hi[0])*.5)])
                                if abs(xyz[0]-2.) < 1.e-12 and abs(xyz[1]-1.04) < 1.e-12: top.append(tag)
                            if len(top) != 1: raise ValueError("ambiguous slit edge")
                            gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: min(maximum*.18, .08) if dim == 1 and tag == top[0]
                                else maximum*(.25+.6/(1.+x*x+y*y+z*z)) if graded else maximum)
                        elif graded: gmsh.model.mesh.setSizeCallback(lambda dim, tag, x, y, z, lc: maximum*(.25+.6/(1.+x*x+y*y+z*z)))
                        gmsh.model.mesh.generate(1)
                        for _, face in gmsh.model.getEntities(2):
                            buffer = (ctypes.c_double*(65536*3))(); n = uses(face, buffer, 65536)
                            if n < 0: raise ValueError("native topology unavailable")
                            if any(buffer[3*i] < 2 and buffer[3*i+2] for i in range(n)):
                                report["deferred_periodic_faces"].append({"fixture": fixture, "face": face}); continue
                            uv = (ctypes.c_double*(65536*9))(); count = copies(face, uv, 65536)
                            if count < 4: raise ValueError("unexpected early route")
                            rows = [[int(uv[9*i]), uv[9*i+1], uv[9*i+2]] for i in range(count)]
                            capacity = 6+6*count; trace_capacity = capacity*128
                            vertices = (ctypes.c_double*(capacity*4))(); edges = (ctypes.c_double*(capacity*6))(); faces = (ctypes.c_double*(capacity*2))()
                            decisions = (ctypes.c_double*(capacity*7))(); crosses = (ctypes.c_double*trace_capacity)(); state = (ctypes.c_double*5)()
                            if recovery(face, vertices, edges, faces, decisions, crosses, state, capacity, trace_capacity) < 0: raise ValueError("native chain collection failed")
                            chains = [[int(vertices[4*int(decisions[7*i])]), int(vertices[4*int(decisions[7*i+1])]), int(decisions[7*i+2])] for i in range(int(state[3]))]
                            output = ctypes.create_string_buffer(16*1024*1024)
                            native_count = native(face, output, len(output))
                            if native_count <= 0: raise ValueError("actual native ordinary BDS failed: "+str(native_count))
                            document = json.loads(output.value.decode("utf-8"))
                            if args.check_boundary:
                                expected = dict(document["result"])
                                metadata = document["boundary"] if args.check_domain else expected
                                boundary_curves = metadata.pop("boundary_curves"); embedded_curves = metadata.pop("embedded_curves"); refine_all = metadata.pop("refine_all")
                                if args.check_embedded and not args.check_node_sizes:
                                    for key in ("boundary_curves","embedded_curves","refine_all","fatal","needs_refinement","failed_curve"): expected.pop(key)
                                    expected["had_fatal_recovery"] = any(a["fatal"] for a in expected["attempts"])
                                if args.check_front_triangles:
                                    normalize_front_nodes(expected["node_sizes"])
                                    for row in expected["front_triangles"]["triangles"]: row[3:7] = map(float,row[3:7])
                                    if args.check_front_queue: normalize_queue(expected["front_queue"])
                                    if args.check_front_generation:
                                        expected["front_generation"] = reference_initial_generation(gmsh,face,expected["node_sizes"],document["face_mesh"],expected["front_triangles"],expected["front_queue"])
                                        expected["front_transfer"] = reference_transfer(gmsh,face,*generation_transfer_input(expected["front_generation"],[]),False,False)
                                        if args.check_smoothing:
                                            expected["smoothing"] = reference_smoothing(gmsh,face,*generation_smoothing_input(face,expected["front_generation"],expected["front_transfer"],document["face_mesh"]),1,0,classified=True)
                                            validate_smoothing(expected["smoothing"],1)
                                    if args.check_front_points:
                                        node_rows = {r[0]:r for r in expected["node_sizes"]["nodes"]}
                                        physical = {r[0]:r[5:8] for r in document["face_mesh"]["vertices"]}
                                        expected["front_points"] = []
                                        consumed = set(); insertion_written = False
                                        for selected in expected["front_queue"]["pops"]:
                                            consumed.add(selected[0])
                                            if not selected[5]: continue
                                            index, edge = selected[0], selected[4]
                                            ids = expected["front_triangles"]["triangles"][index][:3]
                                            inputs = [[*node_rows[id][1:3],*physical[id],*node_rows[id][3:5]] for id in ids]
                                            candidate = reference_front_point(gmsh,face,edge,int(document["front_extend_boundary"]),inputs)
                                            if candidate["status"] != 0: raise ValueError("native initial candidate nonfinite: "+str((fixture,face,index,edge)))
                                            item = {"triangle":index,"edge":edge,"candidate":candidate}
                                            if args.check_front_cavity:
                                                triangle_rows = [[*r[:3],r[6],int(r[10]),*r[7:10]] for r in expected["front_triangles"]["triangles"]]
                                                item["cavity"] = reference_front_cavity(gmsh,face,expected["node_sizes"]["nodes"],triangle_rows,expected["front_queue"]["initial_all"],expected["node_sizes"]["internal_edges"],index,candidate["values"][34:36],candidate["values"][10:13])
                                            if args.check_front_insertion and not insertion_written:
                                                item["insertion"] = reference_initial_point_insertion(gmsh,face,expected["node_sizes"],document["face_mesh"],expected["front_triangles"],expected["front_queue"],index,candidate,consumed)
                                                insertion_written = True
                                            expected["front_points"].append(item)
                                elif args.check_front_node_sizes: normalize_front_nodes(expected)
                                elif args.check_face_mesh: normalize_face_mesh(expected)
                                else: normalize(expected["bds"])
                            else: expected = normalize(document["bds"])
                            chart = [[r[0], r[1], *r[2:7], r[8], r[7]] for r in document["chart"]["points"]]
                            # Compute the outer queries from input UV, not the native output points.
                            lo = [min(row[k] for row in rows) for k in (1, 2)]; hi = [max(row[k] for row in rows) for k in (1, 2)]
                            center = [(a+b)*.5 for a, b in zip(lo, hi)]
                            lo = [(a-c)*1.1+c for a, c in zip(lo, center)]; hi = [(a-c)*1.1+c for a, c in zip(hi, center)]
                            outer = [[u, v, *map(float, gmsh.model.getValue(2, face, [u, v]))] for u, v in ((lo[0], lo[1]), (lo[0], hi[1]), (hi[0], hi[1]), (hi[0], lo[1]))]
                            name = f"{fixture}-{maximum}-{'graded' if graded else 'constant'}-face{face}"
                            lines = [["ordinary-face-mesh" if args.check_face_mesh else "ordinary-metric" if args.check_metric else "ordinary-interior" if args.check_interior else "ordinary-node-sizes" if args.check_node_sizes else "ordinary-embedded" if args.check_embedded else "ordinary-domain" if args.check_domain else "ordinary-boundary" if args.check_boundary else "ordinary", count, face], *rows, [len(chart)], *chart, [len(chains)], *chains, *outer]
                            if args.check_boundary:
                                lines.append([int(refine_all)])
                                for curves in (boundary_curves,embedded_curves):
                                    lines.append([len(curves)])
                                    for curve in curves:
                                        endpoints = curve["endpoints"]
                                        lines.append([curve["entity"],int(curve["degenerated"]),int(endpoints is not None),*(endpoints or [0,0]),len(curve["segments"])])
                                        lines.extend(curve["segments"])
                            if args.check_node_sizes:
                                size_input = document["node_size_input"]
                                callback_mode = (3 if graded else 2) if fixture == "slit-embedded-point" else (1 if graded else 0)
                                lines.append([len(size_input["vertices"]),size_input["model_length"],maximum,callback_mode,top[0] if fixture == "slit-embedded-point" else -1])
                                lines.extend(size_input["vertices"])
                            if args.check_metric:
                                lines.append([len(document["metric_input"])]); lines.extend(document["metric_input"])
                            if args.check_face_mesh:
                                lines.append([len(document["original_vertices"])]); lines.extend(document["original_vertices"])
                            if args.check_front_node_sizes:
                                front = document["front_input"]
                                lines.append([len(front["vertex_order"])]); lines.extend([[id] for id in front["vertex_order"]])
                                lines.append([len(front["embedded_vertices"])]); lines.extend(front["embedded_vertices"])
                                lines.append([len(front["parameters"])]); lines.extend(front["parameters"])
                                lines[0][0] = "ordinary-front-node-sizes"
                                if args.check_front_triangles:
                                    lines[0][0] = "ordinary-front-triangles"; lines.append([int(document["front_extend_boundary"])])
                                    if args.check_front_queue: lines[0][0] = "ordinary-front-queue"
                                    if args.check_front_points: lines[0][0] = "ordinary-front-points"
                                    if args.check_front_cavity: lines[0][0] = "ordinary-front-cavity"
                                    if args.check_front_insertion: lines[0][0] = "ordinary-front-insertion"
                                    if args.check_front_generation: lines[0][0] = "ordinary-front-generation"
                                    if args.check_smoothing: lines[0][0] = "ordinary-smoothing"
                            result = compare(args, directory, name, lines, expected, cad)
                            result["cad_sha256"] = hashlib.sha256(cad.read_bytes()).hexdigest(); report["ordinary_cases"].append(result)
                    finally: gmsh.finalize()
        if len(report["topology_cases"]) != 24 or len(report["ordinary_cases"]) != (44 if args.check_metric else 36): raise ValueError("BDS fixed matrix count mismatch")
        report["bds_boundary_driver_validated"] = args.check_boundary and not args.check_domain
        report["bds_ordinary_domain_validated"] = args.check_domain and not args.check_embedded
        report["bds_embedded_recovery_validated"] = args.check_embedded and not args.check_node_sizes
        report["bds_node_sizes_validated"] = args.check_node_sizes and not args.check_interior
        report["bds_interior_preparation_validated"] = args.check_interior and not args.check_metric
        report["bds_metric_legalization_validated"] = args.check_metric and not args.check_face_mesh
        if args.check_face_mesh and not args.check_front_node_sizes:
            if any(c["copies"] or c["parametric_coordinates"] for c in report["ordinary_cases"]): raise ValueError("ordinary conversion incorrectly used periodic seam maps")
            report["bds_to_face_mesh_validated"] = True
        if args.check_front_node_sizes and not args.check_front_triangles:
            if not any(c["embedded_queries"] and c["different_dual_sizes"] for c in report["ordinary_cases"]): raise ValueError("front matrix did not cover embedded background size exception")
            report["front_node_sizes_validated"] = True
            report["bds_to_face_mesh_validated"] = False
        if args.check_front_triangles:
            if not any(c["embedded_queries"] and c["different_dual_sizes"] for c in report["ordinary_cases"]): raise ValueError("triangle matrix did not cover embedded dual sizes")
            report["front_triangle_initialization_validated"] = True
            report["front_node_sizes_validated"] = True
        if args.check_front_queue: report["front_queue_validated"] = True
        if args.check_front_generation:
            if not any(c["generation_stats"][3] and c["new_triangle_pops"] for c in report["ordinary_cases"]): raise ValueError("ordinary generation matrix does not cover successful new-triangle feedback")
            report["front_generation_validated"] = True
            report["complete_front_generation_validated"] = True
            report["initial_front_pool_frozen"] = False
            report["front_mesh_transfer_validated"] = True
            report["generation_scope"] = "Own ordinary initial/Walk/recovery/domain/UV/BDS/metric/face/node/front prefix enters generateFrontMesh and transferFrontMesh in one run; CAD raw reparameterization, 1D and topology order remain frozen. No outer trueBoundary gate in the ordinary route; smoothing/assembly incomplete."
        if args.check_front_points: report["front_point_placement_validated"] = True
        if args.check_front_cavity: report["front_cavity_validated"] = True
        if args.check_front_insertion:
            report["first_front_point_insertion_validated"] = True
            report["insertion_scope"] = "Only first eligible candidate commits in an isolated test copy; full front-generation feedback and outer trueBoundary gate are not validated."
        if args.check_metric: report["metric_cad_derivatives_validated"] = args.metric_cad
        if args.check_smoothing:
            report["surface_smoothing_validated"] = True
            report["smoothing_scope"] = "Own initial/BDS/front-generation/transfer/smoothing runs continuously. Smoothing parameters and topology are independently computed from BREP; raw earlier chart/front-sizing inputs and 1D/topology order remain frozen. No final assembly/full inverse failure fallback/quality acceptance."
        report["complete"] = True
    finally:
        (directory/"bds-conversion-report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"topology_cases": len(report["topology_cases"]), "ordinary_cases": len(report["ordinary_cases"]), "bds_conversion_equivalent": report["complete"],
                      "bds_boundary_driver_validated": report["bds_boundary_driver_validated"], "bds_ordinary_domain_validated": report["bds_ordinary_domain_validated"],
                      "bds_embedded_recovery_validated": report["bds_embedded_recovery_validated"], "bds_node_sizes_validated": report["bds_node_sizes_validated"],
                      "bds_interior_preparation_validated": report["bds_interior_preparation_validated"],
                      "bds_metric_legalization_validated": report["bds_metric_legalization_validated"],
                      "bds_to_face_mesh_validated": report.get("bds_to_face_mesh_validated", False),
                      "front_node_sizes_validated": report.get("front_node_sizes_validated", False),
                      "metric_cad_derivatives_validated": report.get("metric_cad_derivatives_validated", False),
                      "front_triangle_initialization_validated":report.get("front_triangle_initialization_validated",False),
                      "front_queue_validated":report.get("front_queue_validated",False),
                      "front_generation_validated":report.get("front_generation_validated",False),
                      "front_mesh_transfer_validated":report.get("front_mesh_transfer_validated",False),
                      "surface_smoothing_validated":report.get("surface_smoothing_validated",False),
                      "front_point_placement_validated":report.get("front_point_placement_validated",False),
                      "front_cavity_validated":report.get("front_cavity_validated",False)}))


if __name__ == "__main__": main()
