"""实际周期图表至前沿节点/MTri3初始化对照；前沿反求UV/指针序显式冻结，尺寸与结果连接不冻结。"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference
from GmshPeriodicBoundaryChartComparison import read_loop
from GmshFrontQueueComparison import normalize_queue
from GmshFrontPointPlacementComparison import reference_front_point, normalize_front_points
from GmshFrontCavityComparison import reference_front_cavity, normalize_cavity
from GmshFrontPointInsertionComparison import reference_initial_point_insertion, normalize_point_insertion
from GmshFrontGenerationComparison import reference_initial_generation, normalize_generation
from GmshFrontMeshTransferComparison import reference_transfer, generation_transfer_input
from GmshSurfaceSmoothingComparison import reference_smoothing, generation_smoothing_input, validate_result as validate_smoothing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api","library","own-executable","frozen-dir","out-dir"):
        parser.add_argument("--"+name,required=True)
    parser.add_argument("--runtime-dir",action="append",default=[])
    parser.add_argument("--check-full-chart",action="store_true",help="also compose trueBoundary/boundary/embedded/all DocRecord points with independent CAD queries")
    parser.add_argument("--check-initial-triangulation",action="store_true",help="also compare actual native outer points and divide/conquer triangle sequence")
    parser.add_argument("--check-bds-conversion",action="store_true",help="compare actual complete BDS pools and ordered adjacency after own full chart/initial chain")
    parser.add_argument("--check-bds-recovery",action="store_true",help="continue own full CAD/chart/initial/BDS prefix through actual periodic embedded-first and loop recovery stage")
    parser.add_argument("--check-bds-domain",action="store_true",help="compare native periodic LIFO domain classification after own complete recovery prefix")
    parser.add_argument("--check-bds-interior",action="store_true",help="compare native periodic double cleanup, classification propagation and outer point removal")
    parser.add_argument("--check-bds-metric",action="store_true",help="compare native periodic metric legalization after independent CAD derivatives and full chart/initial/recovery/domain/cleanup prefix")
    parser.add_argument("--check-face-mesh",action="store_true",help="continue full prefix through original periodic seam MVertex copies and BDS2GMSH triangle conversion")
    parser.add_argument("--check-front-node-sizes",action="store_true",help="continue periodic full prefix into actual front node initialization; raw reparameterization/native pointer order frozen")
    parser.add_argument("--check-front-triangles",action="store_true",help="continue actual native MTri3 construction/AllTris ordering/connectTriangles after own periodic full prefix")
    parser.add_argument("--front-boundary-extension",type=int,choices=(0,1),default=1,help="fixed native Mesh.MeshSizeExtendFromBoundary setting for triangle baseline")
    parser.add_argument("--check-front-queue",action="store_true",help="compare initial native activity set/first edges and drain without point insertion after own periodic complete prefix")
    parser.add_argument("--check-front-points",action="store_true",help="continue eligible periodic initial queue triangles through actual candidate and OCC circle correction, no insertion feedback")
    parser.add_argument("--check-front-cavity",action="store_true",help="continue each eligible initial candidate through native circle/DFS/search/rollback, no actual insertion feedback")
    parser.add_argument("--check-front-insertion",action="store_true",help="actual insertAPoint for first eligible initial candidate after own full periodic prefix, isolated from later initial drain")
    parser.add_argument("--check-front-generation",action="store_true",help="own complete periodic prefix enters dynamic generation with sampled trueBoundary; initial front pool and new decisions not frozen")
    parser.add_argument("--check-smoothing",action="store_true",help="continue own periodic generation/transfer through classified smoothing and original CAD parameter queries")
    args = parser.parse_args()
    if args.check_smoothing: args.check_front_generation = True
    if args.check_front_generation:
        if args.check_front_points or args.check_front_cavity or args.check_front_insertion: parser.error("dynamic generation and isolated initial drain are separate modes")
        args.check_front_queue = True
    if args.check_front_cavity and args.check_front_insertion: parser.error("cavity drain and isolated first insertion are separate comparison modes")
    if args.check_front_cavity: args.check_front_points = True
    if args.check_front_insertion: args.check_front_points = True
    if args.check_front_points: args.check_front_queue = True
    if args.check_front_queue: args.check_front_triangles = True
    if args.check_front_triangles: args.check_front_node_sizes = True
    if args.check_front_node_sizes: args.check_face_mesh = True
    if args.check_face_mesh: args.check_bds_metric = True
    if args.check_bds_metric: args.check_bds_interior = True
    if args.check_bds_interior: args.check_bds_domain = True
    if args.check_bds_domain: args.check_bds_recovery = True
    if args.check_bds_recovery: args.check_bds_conversion = True
    if args.check_bds_conversion: args.check_initial_triangulation = True
    if args.check_initial_triangulation: args.check_full_chart = True
    directory = Path(args.out_dir); directory.mkdir(parents=True,exist_ok=False)
    handles = [os.add_dll_directory(str(Path(path).resolve())) for path in args.runtime_dir]
    os.environ['PATH'] = os.pathsep.join([*args.runtime_dir,os.environ.get('PATH','')])
    gmsh = load_reference_api(args.api,args.library); pointer = ctypes.POINTER(ctypes.c_double)
    stage = gmsh.lib.meshstudioReferencePeriodicEmbeddedChart
    stage.argtypes = [ctypes.c_int,ctypes.c_uint,pointer,pointer,pointer,pointer,pointer,ctypes.c_int]; stage.restype = ctypes.c_int
    metadata = gmsh.lib.meshstudioReferencePeriodicEmbeddedInput
    metadata.argtypes = [ctypes.c_int,pointer,ctypes.c_int]; metadata.restype = ctypes.c_int
    native_curves = gmsh.lib.meshstudioReferenceTrueBoundaryCurves
    native_curves.argtypes = [ctypes.c_int,pointer,ctypes.c_int]; native_curves.restype = ctypes.c_int
    if args.check_initial_triangulation:
        native_initial = gmsh.lib.meshstudioReferencePeriodicInitialTriangulation
        native_initial.argtypes = [ctypes.c_int,ctypes.c_uint,pointer,pointer,pointer,pointer,pointer,pointer,ctypes.c_int]
        native_initial.restype = ctypes.c_int
    if args.check_bds_conversion:
        native_bds = gmsh.lib.meshstudioReferencePeriodicBdsConversion
        native_bds.argtypes = [ctypes.c_int,ctypes.c_uint,ctypes.POINTER(ctypes.c_char),ctypes.c_int]
        native_bds.restype = ctypes.c_int
    if args.check_bds_recovery:
        native_recovery = gmsh.lib.meshstudioReferencePeriodicBdsRecovery
        native_recovery.argtypes = [ctypes.c_int,ctypes.c_uint,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; native_recovery.restype = ctypes.c_int
    native_domain_stages = []
    for enabled,name,export in ((args.check_bds_domain,"domain","meshstudioReferencePeriodicBdsDomain"),
                                (args.check_bds_interior,"interior","meshstudioReferencePeriodicBdsInterior"),
                                (args.check_bds_metric,"metric","meshstudioReferencePeriodicBdsMetric"),
                                (args.check_face_mesh,"face_mesh","meshstudioReferencePeriodicFaceMesh"),
                                (args.check_front_node_sizes,"front_node_sizes","meshstudioReferencePeriodicFrontNodeSizes"),
                                (args.check_front_triangles,"front_triangles","meshstudioReferencePeriodicFrontTriangles"),
                                (args.check_front_queue,"front_queue","meshstudioReferencePeriodicFrontQueue")):
        if enabled:
            function = getattr(gmsh.lib,export)
            function.argtypes = [ctypes.c_int,ctypes.c_uint,ctypes.POINTER(ctypes.c_char),ctypes.c_int]; function.restype = ctypes.c_int
            native_domain_stages.append((name,function))
    if args.check_full_chart:
        loop_count = gmsh.lib.meshstudioReferencePeriodicLoopCount
        loop_count.argtypes = [ctypes.c_int]; loop_count.restype = ctypes.c_int
        loop_input = gmsh.lib.meshstudioReferencePeriodicLoopInput
        loop_input.argtypes = [ctypes.c_int,ctypes.c_int,pointer,ctypes.c_int]; loop_input.restype = ctypes.c_int
        sample = gmsh.lib.meshstudioReferenceTrueBoundary
        sample.argtypes = [ctypes.c_int,pointer,ctypes.c_int]; sample.restype = ctypes.c_int
    report = {"scope":"actual periodic embedded BDS/constraint/DocRecord prefix; no initial triangulation",
              "boundary_chart_and_physical_mesh_frozen":True,"embedded_cad_values_from_reference":False,
              "boundary_pointer_order_from_reference":True,"generic_inverse_fallback_validated":False,
              "cases":[],"complete":False,"library_sha256":hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256":hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    if args.check_full_chart:
        report.update(full_chart_cases=[], full_chart_geometry_values_from_reference=False,
                      full_chart_scope="periodic chart composition before outer four points/triangulation; frozen 1D mesh/topology/copy pointer order",
                      full_chart_composition_validated=False)
    if args.check_initial_triangulation:
        report.update(initial_triangulation_cases=[],initial_triangulation_validated=False,
                      initial_triangulation_scope="default random factor 1e-9: actual periodic outer points and DocRecord connections before recovery; frozen 1D/topology/copy pointer order",
                      zero_perturbation_initial_stage_validated=False)
    if args.check_bds_conversion:
        report.update(bds_conversion_cases=[],bds_conversion_validated=False,
                      bds_conversion_scope="actual periodic BDS before recovery, own CAD/full chart/initial chain; frozen 1D/topology/copy order",
                      bds_recovery_validated=False,bds_metric_legalization_validated=False)
    os.environ.pop("MESHSTUDIO_GMSH_TRACE",None)
    if args.check_bds_recovery:
        report.update(bds_recovery_cases=[],bds_recovery_validated=False,
                      bds_recovery_scope="actual periodic embedded-first/boundary-loop recovery after independent CAD/full-chart/initial/BDS prefix; frozen 1D/topology/copy order and boundary curve physical chains",
                      periodic_retry_execution_validated=False,periodic_domain_validated=False)
    for name,function in native_domain_stages:
        report["bds_"+name+"_cases"] = []
        report["bds_"+name+"_validated"] = False
        report["bds_"+name+"_scope"] = "actual native periodic "+name+" after independent CAD/full-chart/initial/BDS/recovery prefix; frozen 1D/topology/copy order and physical curve chains"
    if args.check_bds_metric:
        report["bds_metric_scope"] = "actual native periodic delaunayizeBDS after own CAD/full-chart/initial/BDS/recovery/domain/cleanup; own direct supporting-surface D1 derivatives, no frozen metric values or output connections"
        report["periodic_metric_actual_swaps_validated"] = False
        report["seam_reconstruction_validated"] = False
    if args.check_face_mesh:
        report["bds_face_mesh_scope"] = "actual periodic seam copies and BDS2GMSH after own complete CAD/chart/initial/recovery/domain/cleanup/metric prefix; original 1D MVertex metadata frozen"
        report["vertex_identity_comparison"] = "original physical IDs retained; temporary/new native pointers canonicalized in actual creation order, raw process-global MVertex numbers not compared"
        report["complete_front_generation_validated"] = False
    if args.check_front_node_sizes:
        report["bds_front_node_sizes_scope"] = "actual periodic buildMeshGenerationDataStructures node stage after own complete CAD/chart/initial/recovery/domain/cleanup/metric/seam/conversion prefix; raw reparameterization/native pointer order frozen"
        report["front_node_size_values_from_reference"] = False
        report["front_raw_parameters_from_reference"] = True
        report["front_pointer_order_from_reference"] = True
        report["front_reparameterization_adapter_validated"] = False
    if args.check_front_triangles:
        report["bds_front_triangles_scope"] = "actual buildMeshGenerationDataStructures MTri3 construction/AllTris insertion/connectTriangles after own complete periodic CAD/chart/initial/BDS/recovery/domain/cleanup/metric/seam/conversion/node prefix; resolved Extend1dMeshIn2dSurfaces frozen as CAD option metadata; no frozen triangle radii/order/neighbors"
        report["front_triangle_values_from_reference"] = False
        report["front_boundary_extension"] = args.front_boundary_extension
        report["duplicate_set_rejection_actual_validated"] = False
    if args.check_front_queue:
        report["bds_front_queue_scope"] = "actual native buildMeshGenerationDataStructures states plus native isActive/compareTri3Ptr/sets; observer initialization/pop control statements mirror bowyerWatsonFrontal; drain without candidate/insertion after own complete periodic prefix"
        report["front_queue_values_from_reference"] = False
        report["front_point_insertion_feedback_validated"] = False
    if args.check_front_points:
        report["bds_front_points_scope"] = "own complete periodic prefix/queue/placement; actual optimalPointFrontal/B leaf on corresponding native prefix triangles, independent CAD metric/Value; no cavity/insertion feedback"
        report["front_point_triangle_state_from_reference"] = False
        report["front_point_values_from_reference"] = False
        report["bds_front_points_cases"] = []
        report["bds_front_points_validated"] = False
    if args.check_front_cavity:
        report["bds_front_cavity_scope"] = "own complete periodic prefix/queue/point/cavity; actual native circle/DFS/invMap/search on corresponding native-prefix state, mirrored initial insertAPoint branch control, no insertion commit/feedback"
        report["bds_front_cavity_cases"] = []
        report["bds_front_cavity_validated"] = False
        report["front_cavity_values_from_reference"] = False
        report["parametric_domain_gate_applied"] = False
        report["actual_insertion_feedback_validated"] = False
    # Freeze a valid plane with a distinct interior CAD line; BREP does not preserve embedding relations.
    gmsh.initialize([],readConfigFiles=False)
    try:
        gmsh.option.setNumber("General.Terminal",0.); gmsh.model.add("embedded-line")
        occ = gmsh.model.occ; occ.addRectangle(0,0,0,4,3)
        occ.addLine(occ.addPoint(1,1,0),occ.addPoint(3,2,0)); occ.synchronize()
        line_cad = directory/"embedded-line.brep"; gmsh.write(str(line_cad))
    finally:
        gmsh.finalize()
    stress_inputs = {}
    if args.check_bds_metric:
        for kind in ("cylinder","sphere"):
            gmsh.initialize([],readConfigFiles=False)
            try:
                gmsh.option.setNumber("General.Terminal",0.); gmsh.model.add(kind+"-embedded-stress")
                occ = gmsh.model.occ
                if kind == "cylinder": occ.addCylinder(0.,0.,0.,0.,0.,3.,2.)
                else: occ.addSphere(0.,0.,0.,2.)
                xyz = []
                for j in range(3):
                    for i in range(5):
                        u = .35+1.1*i+.09*j
                        v = .45+.93*j+.06*((i+2*j)%3) if kind == "cylinder" else -1.05+.88*j+.035*((i+j)%3)
                        point = [2.*math.cos(u),2.*math.sin(u),v] if kind == "cylinder" else [2.*math.cos(u)*math.cos(v),2.*math.sin(u)*math.cos(v),2.*math.sin(v)]
                        xyz.append(point); occ.addPoint(*point)
                occ.synchronize(); cad = directory/(kind+"-embedded-stress.brep"); gmsh.write(str(cad))
                stress_inputs[kind+"-embedded-stress"] = (cad,xyz)
            finally:
                gmsh.finalize()
    try:
        fixtures = ("embedded-point","embedded-line","embedded-point-and-line","cylinder-seam-pressure","sphere-seam-pressure",
                    "cylinder-periodic","sphere-periodic","torus-periodic",*stress_inputs)
        for fixture in fixtures:
            cad = stress_inputs[fixture][0] if fixture in stress_inputs else line_cad if "line" in fixture else Path(args.frozen_dir)/(("embedded" if fixture == "embedded-point" else fixture.split("-")[0])+".brep")
            for graded in (0,1):
                for random_factor in (0.,1.e-9):
                    for seed in (1,20261007):
                        gmsh.initialize([],readConfigFiles=False)
                        try:
                            if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                                raise ValueError("wrong fixed reference version")
                            gmsh.option.setNumber("General.Terminal",0.); gmsh.option.setNumber("General.NumThreads",1.)
                            gmsh.model.add(fixture); gmsh.model.occ.importShapes(str(cad),highestDimOnly=False); gmsh.model.occ.synchronize()
                            if gmsh.option.getNumber("Geometry.ReparamOnFaceRobust") != 0. or gmsh.option.getNumber("Geometry.OCCUseGenericClosestPoint") != 0.:
                                raise ValueError("direct pcurve/projection test requires fixed default inverse modes zero")
                            faces = gmsh.model.getEntities(2)
                            if "seam-pressure" in fixture or fixture.endswith("-periodic") or fixture in stress_inputs:
                                candidates = []
                                for _,face in faces:
                                    rows = (ctypes.c_double*(4096*4))(); n = native_curves(face,rows,4096)
                                    if n <= 0: raise ValueError("native boundary metadata unavailable")
                                    for i in range(n):
                                        edge,orientation,seam,face_orientation = map(int,rows[4*i:4*(i+1)])
                                        if seam: candidates.append((face,edge))
                                face_ids = {face for face,edge in candidates}
                                if len(face_ids) != 1: raise ValueError("expected one native periodic face")
                                face = face_ids.pop()
                                if fixture in stress_inputs:
                                    ids = []
                                    for xyz in stress_inputs[fixture][1]:
                                        matching = [tag for _,tag in gmsh.model.getEntities(0) if max(abs(a-b) for a,b in zip(gmsh.model.getValue(0,tag,[]),xyz)) < 1.e-12]
                                        if len(matching) != 1: raise ValueError("stress embedded CAD point identity missing/ambiguous")
                                        ids.append(matching[0])
                                    gmsh.model.mesh.embed(0,ids,2,face)
                                if "seam-pressure" in fixture:
                                    if len(candidates) != 1: raise ValueError("pressure fixture requires one seam")
                                    edge = candidates[0][1]; gmsh.model.mesh.embed(1,[edge],2,face)
                                    # Endpoint-only pressure input: interior seam copies would exercise an
                                    # invalid native DocRecord allocation after failed endpoint selection.
                                    gmsh.model.mesh.setTransfiniteCurve(edge,2)
                            else:
                                if len(faces) != 1: raise ValueError("expected one embedded plane")
                                face = faces[0][1]
                                xyz = [2.,1.5,0.] if fixture == "embedded-point" else [1.,1.,0.]
                                point_ids = [tag for _,tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(0,tag,[])) == xyz]
                                if len(point_ids) != 1: raise ValueError("embedded point identity missing/ambiguous")
                                if fixture != "embedded-line": gmsh.model.mesh.embed(0,point_ids,2,face)
                                if "line" in fixture:
                                    end_ids = [tag for _,tag in gmsh.model.getEntities(0) if list(gmsh.model.getValue(0,tag,[])) == [3.,2.,0.]]
                                    wanted = set(point_ids+end_ids)
                                    edges = [tag for _,tag in gmsh.model.getEntities(1) if set(v for d,v in gmsh.model.getBoundary([(1,tag)],oriented=False)) == wanted]
                                    if len(edges) != 1: raise ValueError("embedded line identity missing/ambiguous")
                                    gmsh.model.mesh.embed(1,edges,2,face)
                            for name,value in {"Mesh.Algorithm":6.,"Mesh.MeshSizeMax":.3,"Mesh.MeshSizeFromPoints":0.,
                                               "Mesh.MeshSizeFromCurvature":0.,"Mesh.MeshSizeFactor":1.25,"Mesh.RandomFactor":random_factor,
                                               "Mesh.MeshSizeExtendFromBoundary":args.front_boundary_extension}.items():
                                gmsh.option.setNumber(name,value)
                            if graded: gmsh.model.mesh.setSizeCallback(lambda dim,tag,x,y,z,lc:min(lc,.12+.015*abs(x)+.005*abs(y)))
                            gmsh.model.mesh.generate(1)
                            nodes = []
                            for dim,entity in gmsh.model.getEntities():
                                ids,xyz,parameters = gmsh.model.mesh.getNodes(dim,entity,False,True)
                                for i,node in enumerate(ids):
                                    if dim > 1: raise ValueError("unexpected surface nodes")
                                    nodes.append([int(node),*map(float,xyz[3*i:3*i+3]),dim,entity,float(parameters[i]) if dim else 0.])
                            meta = (ctypes.c_double*(65536*4))(); meta_count = metadata(face,meta,65536)
                            if meta_count < 0 or (not fixture.endswith("-periodic") and not meta_count):
                                raise ValueError("embedding metadata missing")
                            if fixture.endswith("-periodic") and meta_count: raise ValueError("ordinary periodic case must have no embedding")
                            meta_rows = [list(map(int,meta[4*i:4*(i+1)])) for i in range(meta_count)]
                            if "seam-pressure" in fixture and sum(row[0] == 1 for row in meta_rows) != 1:
                                raise ValueError("seam pressure requires one endpoint-only line before entering native stage")
                            points = (ctypes.c_double*(65536*12))(); initial = (ctypes.c_double*(65536*3))()
                            constraints = (ctypes.c_double*(65536*2))(); copies = (ctypes.c_double*65536)(); state = (ctypes.c_double*8)()
                            count = stage(face,seed,points,initial,constraints,copies,state,65536)
                            if count <= 0: raise ValueError("actual native periodic embedded stage failed: "+str(count))
                            rows = []
                            for i in range(count):
                                row = list(points[12*i:12*(i+1)])
                                for k in (0,1,7,8,11): row[k] = int(row[k])
                                rows.append(row)
                            boundary_rows = [row for row in rows if row[8] >= 0]; embedded_rows = [row for row in rows if row[8] < 0]
                            embedded_count = int(state[7]); boundary_positions = int(state[1])-embedded_count
                            if embedded_count != len(embedded_rows) or len(boundary_rows) != int(state[6]): raise ValueError("native stage counts differ")
                            expected = {"maximum_id":int(state[4]),"expected_count":int(state[2])-boundary_positions,
                                        "mismatch":int(state[2]) != int(state[1]),"vertices":embedded_rows,
                                        "initial_points":[[int(initial[3*i]),initial[3*i+1],initial[3*i+2]] for i in range(embedded_count)],
                                        "constraints":[list(map(int,constraints[2*i:2*(i+1)])) for i in range(int(state[3]))],"seam_selection_failures":[]}
                            max_boundary = max(row[0] for row in boundary_rows)
                            lines = [[face,len(nodes),len(boundary_rows),int(state[6]),len(meta_rows),max_boundary,state[5],.3,1.25,graded,random_factor,seed],*nodes,*boundary_rows]
                            lines.extend([[int(copies[i])] for i in range(int(state[6]))]); lines.extend(meta_rows)
                            prefix = f"{fixture}-graded{graded}-random{random_factor}-seed{seed}"
                            frozen = directory/(prefix+"-input.tsv")
                            frozen.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in lines),encoding="utf-8")
                            (directory/(prefix+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                            own = directory/(prefix+"-own.json"); subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True)
                            actual = json.loads(own.read_text(encoding="utf-8"))
                            for row in actual["vertices"]:
                                for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                            for row in actual["initial_points"]: row[1:] = map(float,row[1:])
                            difference = first_trace_difference([{"stage":"periodic_embedded_chart",**expected}],[{"stage":"periodic_embedded_chart",**actual}],0.)
                            report["cases"].append({"fixture":fixture,"graded":graded,"random_factor":random_factor,"seed":seed,
                                                    "boundary_seam_pressure_input":"seam-pressure" in fixture,"embedded_nodes":embedded_count,
                                                    "ordinary_face_forced_periodic_path":fixture.startswith("embedded"),
                                                    "native_periodic_face_input":fixture.endswith("-periodic") or fixture in stress_inputs,
                                                    "multi_embedded_metric_stress_input":fixture in stress_inputs,
                                                    "constraints":int(state[3]),"cad_sha256":hashlib.sha256(cad.read_bytes()).hexdigest(),
                                                    "input_sha256":hashlib.sha256(frozen.read_bytes()).hexdigest(),"first_difference":difference})
                            if difference: raise ValueError("periodic embedded divergence: "+json.dumps(difference))
                            if args.check_full_chart:
                                topology = (ctypes.c_double*(4096*4))(); topology_count = native_curves(face,topology,4096)
                                if topology_count <= 0: raise ValueError("full chart topology unavailable")
                                topology_rows = [list(map(int,topology[4*i:4*(i+1)])) for i in range(topology_count)]
                                by_edge = {row[0]:row[1:] for row in topology_rows}
                                number = loop_count(face)
                                if number <= 0: raise ValueError("full chart has no native boundary loops")
                                edge_loops = [read_loop(loop_input,face,i) for i in range(number)]
                                truth = (ctypes.c_double*(65536*2))(); truth_count = sample(face,truth,65536)
                                if truth_count <= 0 or truth_count % 2: raise ValueError("full chart trueBoundary unavailable")
                                true_segments = []; position = 0
                                for edge,orientation,seam,face_orientation in topology_rows:
                                    for branch in range(2 if seam else 1):
                                        for k in range(299):
                                            true_segments.append([edge,branch,*truth[2*position:2*position+4]]); position += 2
                                if position != truth_count: raise ValueError("full chart trueBoundary count mismatch")
                                full_expected = {**expected,"expected_count":int(state[2]),"vertices":rows,
                                                 "initial_points":[[int(initial[3*i]),initial[3*i+1],initial[3*i+2]] for i in range(int(state[1]))],
                                                 "true_boundary":true_segments}
                                # Topology only: no frozen boundary UV, XYZ, bounds, sizes or BDS IDs.
                                full_lines = [[face,len(nodes),0,int(state[6]),len(meta_rows),0,state[5],.3,1.25,graded,random_factor,seed],*nodes]
                                full_lines.extend([[int(copies[i])] for i in range(int(state[6]))]); full_lines.extend(meta_rows)
                                full_lines.append([topology_count]); full_lines.extend(topology_rows); full_lines.append([number])
                                for curves in edge_loops:
                                    full_lines.append([len(curves)])
                                    for curve in curves:
                                        orientation,seam,face_orientation = by_edge[curve["entity"]]
                                        full_lines.append([curve["entity"],seam,orientation,face_orientation,len(curve["vertices"])])
                                        full_lines.extend([[row[0]] for row in curve["vertices"]])
                                full_input = directory/(prefix+"-full-input.tsv")
                                full_input.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in full_lines),encoding="utf-8")
                                (directory/(prefix+"-full-reference.json")).write_text(json.dumps(full_expected,allow_nan=False),encoding="utf-8")
                                full_output = directory/(prefix+"-full-own.json")
                                subprocess.run([args.own_executable,str(cad),str(full_input),str(full_output),"--full-chart"],check=True)
                                full_actual = json.loads(full_output.read_text(encoding="utf-8"))
                                for row in full_actual["vertices"]:
                                    for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                                for row in full_actual["initial_points"]: row[1:] = map(float,row[1:])
                                for row in full_actual["true_boundary"]: row[2:] = map(float,row[2:])
                                full_difference = first_trace_difference([{"stage":"periodic_face_chart",**full_expected}],[{"stage":"periodic_face_chart",**full_actual}],0.)
                                report["full_chart_cases"].append({"fixture":fixture,"graded":graded,"random_factor":random_factor,"seed":seed,
                                                                  "input_sha256":hashlib.sha256(full_input.read_bytes()).hexdigest(),"first_difference":full_difference})
                                if full_difference: raise ValueError("full periodic chart divergence: "+json.dumps(full_difference))
                                if args.check_initial_triangulation and random_factor != 0.:
                                    init_points = (ctypes.c_double*(65536*12))(); init_sorted = (ctypes.c_double*(65536*3))()
                                    init_triangles = (ctypes.c_double*(65536*3))(); init_constraints = (ctypes.c_double*(65536*2))()
                                    init_copies = (ctypes.c_double*65536)(); init_state = (ctypes.c_double*14)()
                                    init_count = native_initial(face,seed,init_points,init_sorted,init_triangles,init_constraints,init_copies,init_state,65536)
                                    if init_count <= 0 and init_count != -6: raise ValueError("actual native periodic initial stage failed: "+str(init_count))
                                    if init_count > 0 and int(init_state[8]) != int(init_state[9]): raise ValueError("DocRecord and actual BDS triangle counts differ")
                                    init_rows = []
                                    for i in range(init_count):
                                        row = list(init_points[12*i:12*(i+1)])
                                        for k in (0,1,7,8,11): row[k] = int(row[k])
                                        init_rows.append(row)
                                    init_expected = {"vertices":init_rows,"bounds":list(init_state[10:14]),
                                                     "sorted_points":[[int(init_sorted[3*i]),init_sorted[3*i+1],init_sorted[3*i+2]] for i in range(int(init_state[1]))],
                                                     "triangles":[list(map(int,init_triangles[3*i:3*(i+1)])) for i in range(int(init_state[8]))]}
                                    if init_count == -6: init_expected = {"status":5} # InvalidTopology; native tail is uninitialized, do not read it.
                                    init_lines = list(full_lines)
                                    for i in range(int(init_state[6])): init_lines[1+len(nodes)+i] = [int(init_copies[i])]
                                    init_input = directory/(prefix+"-initial-input.tsv")
                                    init_input.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in init_lines),encoding="utf-8")
                                    (directory/(prefix+"-initial-reference.json")).write_text(json.dumps(init_expected,allow_nan=False),encoding="utf-8")
                                    init_output = directory/(prefix+"-initial-own.json")
                                    subprocess.run([args.own_executable,str(cad),str(init_input),str(init_output),"--bds-conversion" if args.check_bds_conversion else "--initial-triangulation"],check=True,timeout=30)
                                    init_document = json.loads(init_output.read_text(encoding="utf-8")); init_actual = init_document["periodic_initial"]
                                    for row in init_actual.get("vertices",[]):
                                        for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                                    for row in init_actual.get("sorted_points",[]): row[1:] = map(float,row[1:])
                                    if "bounds" in init_actual: init_actual["bounds"] = list(map(float,init_actual["bounds"]))
                                    init_difference = first_trace_difference([{"stage":"periodic_initial",**init_expected}],[{"stage":"periodic_initial",**init_actual}],0.)
                                    report["initial_triangulation_cases"].append({"fixture":fixture,"graded":graded,"random_factor":random_factor,"seed":seed,
                                                                               "triangles":int(init_state[8]),"native_predicted_triangles":int(init_state[9]),
                                                                               "native_uninitialized_triangle_array":init_count == -6,
                                                                               "input_sha256":hashlib.sha256(init_input.read_bytes()).hexdigest(),"first_difference":init_difference})
                                    if init_difference: raise ValueError("periodic initial stage divergence: "+json.dumps(init_difference))
                                    if args.check_bds_conversion:
                                        native_json = ctypes.create_string_buffer(16*1024*1024)
                                        result = native_bds(face,seed,native_json,len(native_json))
                                        if result < 0: raise ValueError("actual native periodic BDS failed: "+str(result))
                                        bds_expected = json.loads(native_json.value.decode("utf-8")); bds_actual = init_document["bds"]
                                        for document in (bds_expected,bds_actual):
                                            for row in document["points"]:
                                                for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                                        bds_difference = first_trace_difference([{"stage":"periodic_bds",**bds_expected}],[{"stage":"periodic_bds",**bds_actual}],0.)
                                        report["bds_conversion_cases"].append({"fixture":fixture,"graded":graded,"seed":seed,
                                            "points":len(bds_expected["points"]),"edges":len(bds_expected["edges"]),"triangles":result,
                                            "input_sha256":hashlib.sha256(init_input.read_bytes()).hexdigest(),"first_difference":bds_difference})
                                        (directory/(prefix+"-bds-reference.json")).write_text(json.dumps(bds_expected,allow_nan=False),encoding="utf-8")
                                        if bds_difference: raise ValueError("periodic BDS divergence: "+json.dumps(bds_difference))
                                        if args.check_bds_recovery:
                                            recovered_json = ctypes.create_string_buffer(32*1024*1024)
                                            native_result = native_recovery(face,seed,recovered_json,len(recovered_json))
                                            if native_result < 0: raise ValueError("actual native periodic recovery failed: "+str(native_result))
                                            recovery_document = json.loads(recovered_json.value.decode("utf-8"))
                                            recovery_lines = list(init_lines)
                                            recovery_lines.append([int(recovery_document["refine_all"]),len(recovery_document["boundary_curves"])])
                                            for curve in recovery_document["boundary_curves"]:
                                                recovery_lines.append([curve["entity"],len(curve["segments"])]); recovery_lines.extend(curve["segments"])
                                            recovery_input = directory/(prefix+"-recovery-input.tsv")
                                            recovery_input.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in recovery_lines),encoding="utf-8")
                                            recovery_output = directory/(prefix+"-recovery-own.json")
                                            subprocess.run([args.own_executable,str(cad),str(recovery_input),str(recovery_output),"--bds-recovery"],check=True,timeout=30)
                                            actual = json.loads(recovery_output.read_text(encoding="utf-8"))["periodic_recovery"]
                                            expected = recovery_document["result"]
                                            for document in (expected,actual):
                                                for row in document["bds"]["points"]:
                                                    for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                                            difference = first_trace_difference([{"stage":"periodic_recovery",**expected}],[{"stage":"periodic_recovery",**actual}],0.)
                                            attempts = expected["attempts"]
                                            report["bds_recovery_cases"].append({"fixture":fixture,"graded":graded,"seed":seed,
                                                "embedded_failed":expected["embedded_failed"],"needs_retry":expected["needs_retry"],
                                                "embedded_attempts":sum(a["embedded"] for a in attempts),"boundary_attempts":sum(not a["embedded"] for a in attempts),
                                                "missing_boundary_attempts":sum(not a["embedded"] and not a["already_present"] for a in attempts),
                                                "attempted_boundary_curves":len(expected["attempted_boundary_curves"]),
                                                "successful_swaps":sum(ok for a in attempts if a["recovery"] for s in a["recovery"]["scans"] for _,ok in s["attempts"]),
                                                "input_sha256":hashlib.sha256(recovery_input.read_bytes()).hexdigest(),"first_difference":difference})
                                            (directory/(prefix+"-recovery-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                                            if difference: raise ValueError("periodic recovery divergence: "+json.dumps(difference))
                                            for stage_name,native_function in native_domain_stages:
                                                stage_buffer = ctypes.create_string_buffer(32*1024*1024)
                                                stage_result = native_function(face,seed,stage_buffer,len(stage_buffer))
                                                if stage_result < 0: raise ValueError("native periodic "+stage_name+" unavailable: "+str(stage_result))
                                                stage_document = json.loads(stage_buffer.value.decode("utf-8"))
                                                for key in ("refine_all","boundary_curves"):
                                                    if stage_document["recovery"][key] != recovery_document[key]:
                                                        raise ValueError("periodic "+stage_name+" recovery input metadata changed: "+key)
                                                stage_output = directory/(prefix+"-"+stage_name+"-own.json")
                                                stage_input = recovery_input
                                                if stage_name in ("face_mesh","front_node_sizes","front_triangles","front_queue"):
                                                    face_lines = list(recovery_lines)
                                                    face_lines.append([len(stage_document["original_vertices"])]); face_lines.extend(stage_document["original_vertices"])
                                                    if stage_name in ("front_node_sizes","front_triangles","front_queue"):
                                                        front = stage_document["front_input"]
                                                        face_lines.append([len(front["vertex_order"])]); face_lines.extend([[id] for id in front["vertex_order"]])
                                                        face_lines.append([len(front["embedded_vertices"])]); face_lines.extend(front["embedded_vertices"])
                                                        face_lines.append([len(front["parameters"])]); face_lines.extend(front["parameters"])
                                                        for kind in ("boundary","embedded"):
                                                            curves = stage_document["front_curves"][kind]; face_lines.append([len(curves)])
                                                            for curve in curves:
                                                                face_lines.append([int(curve["degenerated"]),len(curve["segments"])]); face_lines.extend(curve["segments"])
                                                    if stage_name in ("front_triangles","front_queue"): face_lines.append([int(stage_document["front_extend_boundary"])])
                                                    stage_input = directory/(prefix+"-"+stage_name+"-input.tsv")
                                                    stage_input.write_text("".join(" ".join(format(x,".17g") if isinstance(x,float) else str(x) for x in row)+"\n" for row in face_lines),encoding="utf-8")
                                                mode = "--front-queue" if stage_name == "front_queue" else "--front-triangles" if stage_name == "front_triangles" else "--front-node-sizes" if stage_name == "front_node_sizes" else "--face-mesh" if stage_name == "face_mesh" else "--bds-"+stage_name
                                                if stage_name == "front_queue" and args.check_front_points: mode = "--front-points"
                                                if stage_name == "front_queue" and args.check_front_cavity: mode = "--front-cavity"
                                                if stage_name == "front_queue" and args.check_front_insertion: mode = "--front-insertion"
                                                if stage_name == "front_queue" and args.check_front_generation: mode = "--front-generation"
                                                if stage_name == "front_queue" and args.check_smoothing: mode = "--smoothing"
                                                subprocess.run([args.own_executable,str(cad),str(stage_input),str(stage_output),mode],check=True,timeout=30)
                                                replay_document = json.loads(stage_output.read_text(encoding="utf-8"))
                                                stage_actual = replay_document["periodic_front_node_sizes" if stage_name in ("front_triangles","front_queue") else "periodic_"+stage_name]
                                                stage_expected = stage_document["result"]
                                                for document in (stage_expected,stage_actual):
                                                    if stage_name in ("front_node_sizes","front_triangles","front_queue"):
                                                        for key,begin in (("nodes",1),("reparameterizations",1),("embedded_queries",3)):
                                                            for row in document[key]: row[begin:] = map(float,row[begin:])
                                                    elif stage_name == "face_mesh":
                                                        for row in document["vertices"]:
                                                            for k in range(5,11): row[k] = float(row[k])
                                                        for row in document["parametric_coordinates"]: row[1:] = map(float,row[1:])
                                                    else:
                                                        for row in document["bds"]["points"]:
                                                            for k in (2,3,4,5,6,9,10): row[k] = float(row[k])
                                                stage_difference = first_trace_difference([{"stage":"periodic_"+stage_name,**stage_expected}],
                                                                                         [{"stage":"periodic_"+stage_name,**stage_actual}],0.)
                                                if stage_name in ("front_triangles","front_queue"):
                                                    reference_triangles = stage_document["front_triangles"]; actual_triangles = replay_document["periodic_front_triangles"]
                                                    for document in (reference_triangles,actual_triangles):
                                                        for row in document["triangles"]: row[3:7] = map(float,row[3:7])
                                                    stage_difference = stage_difference or first_trace_difference([{"stage":"front_triangles",**reference_triangles}],[{"stage":"front_triangles",**actual_triangles}],0.)
                                                    stage_expected = {"node_sizes":stage_expected,"front_triangles":reference_triangles}
                                                    if stage_name == "front_queue":
                                                        reference_queue = normalize_queue(stage_document["front_queue"]); actual_queue = normalize_queue(replay_document["periodic_front_queue"])
                                                        stage_difference = stage_difference or first_trace_difference([{"stage":"front_queue",**reference_queue}],[{"stage":"front_queue",**actual_queue}],0.)
                                                        stage_expected["front_queue"] = reference_queue
                                                        if args.check_front_generation:
                                                            generation_expected = reference_initial_generation(gmsh,face,stage_expected["node_sizes"],stage_document["face_mesh"],reference_triangles,reference_queue,[r[2:] for r in true_segments])
                                                            generation_actual = normalize_generation(replay_document["periodic_front_generation"])
                                                            stage_difference = stage_difference or first_trace_difference([{"stage":"dynamic_front",**generation_expected}],[{"stage":"dynamic_front",**generation_actual}],0.)
                                                            stage_expected["front_generation"] = generation_expected
                                                            native_face = stage_document["face_mesh"]
                                                            transfer_expected = reference_transfer(gmsh,face,*generation_transfer_input(generation_expected,native_face["equivalence"],native_face["vertices"]),True,False)
                                                            stage_difference = stage_difference or first_trace_difference([{"stage":"front_transfer",**transfer_expected}],[{"stage":"front_transfer",**replay_document["periodic_front_transfer"]}],0.)
                                                            stage_expected["front_transfer"] = transfer_expected
                                                            if args.check_smoothing:
                                                                smoothing_expected = reference_smoothing(gmsh,face,*generation_smoothing_input(face,generation_expected,transfer_expected,native_face),1,0,classified=True)
                                                                validate_smoothing(smoothing_expected,1)
                                                                validate_smoothing(replay_document["periodic_smoothing"],1)
                                                                stage_difference = stage_difference or first_trace_difference([smoothing_expected],[replay_document["periodic_smoothing"]],0.)
                                                                stage_expected["smoothing"] = smoothing_expected
                                                                report.setdefault("smoothing_cases",[]).append({"fixture":fixture,"graded":graded,"seed":seed,
                                                                    **{key:smoothing_expected[key] for key in ("visits","eligible","writes","queries")},"first_difference":stage_difference})
                                                            report.setdefault("front_generation_cases",[]).append({"fixture":fixture,"graded":graded,"seed":seed,"stats":generation_expected["stats"],
                                                                "new_triangle_pops":sum(e["selection"][0] >= len(reference_triangles["triangles"]) for e in generation_expected["events"]),
                                                                "input_sha256":hashlib.sha256(stage_input.read_bytes()).hexdigest(),"first_difference":stage_difference})
                                                        if args.check_front_points:
                                                            node_rows = {r[0]:r for r in stage_expected["node_sizes"]["nodes"]}
                                                            physical = {r[0]:r[5:8] for r in stage_document["face_mesh"]["vertices"]}
                                                            point_expected = []
                                                            consumed = set(); insertion_written = False
                                                            for selected in reference_queue["pops"]:
                                                                consumed.add(selected[0])
                                                                if not selected[5]: continue
                                                                index, edge = selected[0], selected[4]; ids = reference_triangles["triangles"][index][:3]
                                                                inputs = [[*node_rows[id][1:3],*physical[id],*node_rows[id][3:5]] for id in ids]
                                                                candidate = reference_front_point(gmsh,face,edge,int(stage_document["front_extend_boundary"]),inputs)
                                                                if candidate["status"] != 0: raise ValueError("native periodic candidate nonfinite: "+str((fixture,face,index,edge)))
                                                                item = {"triangle":index,"edge":edge,"candidate":candidate}
                                                                if args.check_front_cavity:
                                                                    triangle_rows = [[*r[:3],r[6],int(r[10]),*r[7:10]] for r in reference_triangles["triangles"]]
                                                                    item["cavity"] = reference_front_cavity(gmsh,face,stage_expected["node_sizes"]["nodes"],triangle_rows,reference_queue["initial_all"],stage_expected["node_sizes"]["internal_edges"],index,candidate["values"][34:36],candidate["values"][10:13])
                                                                if args.check_front_insertion and not insertion_written:
                                                                    item["insertion"] = reference_initial_point_insertion(gmsh,face,stage_expected["node_sizes"],stage_document["face_mesh"],reference_triangles,reference_queue,index,candidate,consumed)
                                                                    insertion_written = True
                                                                point_expected.append(item)
                                                            point_actual = normalize_front_points(replay_document["periodic_front_points"])
                                                            if args.check_front_cavity:
                                                                for item in point_actual: normalize_cavity(item["cavity"])
                                                            if args.check_front_insertion:
                                                                for item in point_actual:
                                                                    if "insertion" in item: normalize_point_insertion(item["insertion"])
                                                            stage_difference = stage_difference or first_trace_difference([{"stage":"front_points","points":point_expected}],[{"stage":"front_points","points":point_actual}],0.)
                                                            stage_expected["front_points"] = point_expected
                                                            report["bds_front_points_cases"].append({"fixture":fixture,"graded":graded,"seed":seed,"points":len(point_expected),
                                                                "surface_queries":sum(len(p["candidate"]["queries"]) for p in point_expected),"corrected_points":sum(p["candidate"]["values"][33] for p in point_expected),
                                                                "input_sha256":hashlib.sha256(stage_input.read_bytes()).hexdigest(),"first_difference":stage_difference})
                                                            if args.check_front_insertion:
                                                                insertions = [p["insertion"] for p in point_expected if "insertion" in p]
                                                                report["bds_front_points_cases"][-1].update(first_insertions=len(insertions),first_inserted=sum(i["inserted"] for i in insertions),first_insertion_codes=[i["code"] for i in insertions])
                                                            if args.check_front_cavity:
                                                                cavities = [p["cavity"] for p in point_expected]
                                                                report["bds_front_cavity_cases"].append({"fixture":fixture,"graded":graded,"seed":seed,"cavities":len(cavities),"cavity_triangles":sum(len(c["cavity"]) for c in cavities),
                                                                    "shell_edges":sum(len(c["shell"]) for c in cavities),"containing_missing":sum(c["containing"] < 0 for c in cavities),"searches":sum(bool(c["search"]["visits"]) for c in cavities),"first_difference":stage_difference})
                                                case = {"fixture":fixture,"graded":graded,"seed":seed,"triangles":len(stage_document["face_mesh"]["triangles"] if stage_name in ("front_node_sizes","front_triangles","front_queue") else stage_expected["triangles"] if stage_name == "face_mesh" else stage_expected["bds"]["triangles"]),
                                                        "input_sha256":hashlib.sha256(stage_input.read_bytes()).hexdigest(),"first_difference":stage_difference}
                                                if stage_name == "domain":
                                                    case.update(fills=len(stage_expected["fills"]),interior_triangles=stage_expected["interior_triangles"],
                                                                cleared_exterior=len(stage_expected["cleared_exterior"]),
                                                                redundant_visits=sum(not expanded for fill in stage_expected["fills"] for _,expanded in fill["visits"]))
                                                elif stage_name == "interior":
                                                    case.update(unclassified_triangles=len(stage_expected["unclassified_triangles"]),
                                                                orphan_edges=len(stage_expected["orphan_edges"]),removed_outer_points=stage_expected["removed_outer_points"])
                                                elif stage_name == "metric":
                                                    case.update(swaps=stage_expected["swaps"],metric_queries=stage_expected["metric_queries"],
                                                                scans=len(stage_expected["scans"]),configurations=len(stage_expected["configurations"]),
                                                                qualification_rejections=sum(a["decision"] == 4 for s in stage_expected["scans"] for a in s["attempts"]))
                                                elif stage_name == "front_queue":
                                                    case.update(initial_active=len(reference_queue["initial_active"]),queue_pops=len(reference_queue["pops"]),
                                                                insertion_eligible=sum(p[5] for p in reference_queue["pops"]),inactive_pops=sum(p[2] and not p[3] for p in reference_queue["pops"]),
                                                                extend_boundary=stage_document["front_extend_boundary"])
                                                elif stage_name == "front_triangles":
                                                    case.update(ordered=len(reference_triangles["ordered"]),rejected_duplicates=len(reference_triangles["rejected_duplicates"]),
                                                                connected_edges=sum(n >= 0 for t in reference_triangles["triangles"] for n in t[7:10])//2,
                                                                extend_boundary=stage_document["front_extend_boundary"])
                                                elif stage_name == "front_node_sizes":
                                                    face_vertices = {r[0]:r for r in stage_document["face_mesh"]["vertices"]}
                                                    raw = {r[0]:r[1:] for r in stage_expected["reparameterizations"]}
                                                    case.update(front_nodes=len(stage_expected["nodes"]),reparameterizations=len(stage_expected["reparameterizations"]),
                                                                embedded_queries=len(stage_expected["embedded_queries"]),internal_edges=len(stage_expected["internal_edges"]),
                                                                different_dual_sizes=sum(r[3] != r[4] for r in stage_expected["nodes"]),
                                                                copies=sum(face_vertices[r[0]][2] == 1 for r in stage_expected["nodes"]),
                                                                parameter_overrides=sum(raw[r[0]] != r[1:3] for r in stage_expected["nodes"]))
                                                else:
                                                    case.update(copies=len(stage_expected["equivalence"]),created_local_nodes=len(stage_expected["conversion"]["created_local_nodes"]),
                                                                skipped_deleted=len(stage_expected["conversion"]["skipped_deleted"]),collapsed=len(stage_expected["conversion"]["collapsed"]),
                                                                unreconstructed_seams=len(stage_expected["unreconstructed_seams"]))
                                                report["bds_"+stage_name+"_cases"].append(case)
                                                (directory/(prefix+"-"+stage_name+"-reference.json")).write_text(json.dumps(stage_expected,allow_nan=False),encoding="utf-8")
                                                if stage_difference: raise ValueError("periodic "+stage_name+" divergence: "+json.dumps(stage_difference))
                        finally:
                            gmsh.finalize()
        expected_cases = 8*len(fixtures); expected_initial = expected_cases//2
        if len(report["cases"]) != expected_cases: raise ValueError("fixed embedded matrix count differs")
        if args.check_full_chart:
            if len(report["full_chart_cases"]) != expected_cases: raise ValueError("fixed full chart matrix count differs")
            report["full_chart_composition_validated"] = True
        if args.check_initial_triangulation:
            if len(report["initial_triangulation_cases"]) != expected_initial: raise ValueError("fixed default-random initial stage matrix count differs")
            report["initial_triangulation_validated"] = True
        if args.check_bds_conversion:
            if len(report["bds_conversion_cases"]) != expected_initial: raise ValueError("fixed periodic BDS conversion matrix count differs")
            report["bds_conversion_validated"] = True
        if args.check_bds_recovery:
            if len(report["bds_recovery_cases"]) != expected_initial: raise ValueError("fixed periodic recovery matrix count differs")
            report["bds_recovery_validated"] = True
        for name,function in native_domain_stages:
            if len(report["bds_"+name+"_cases"]) != expected_initial: raise ValueError("fixed periodic "+name+" matrix count differs")
            report["bds_"+name+"_validated"] = True
        if args.check_bds_domain: report["periodic_domain_validated"] = True
        if args.check_front_points:
            if len(report["bds_front_points_cases"]) != expected_initial: raise ValueError("fixed periodic candidate matrix count differs")
            report["bds_front_points_validated"] = True
        if args.check_front_cavity:
            if len(report["bds_front_cavity_cases"]) != expected_initial: raise ValueError("fixed periodic cavity matrix count differs")
            report["bds_front_cavity_validated"] = True
        if args.check_bds_metric:
            for fixture in stress_inputs:
                if not any(case["fixture"] == fixture and case["swaps"] > 0 for case in report["bds_metric_cases"]):
                    raise ValueError("periodic metric matrix does not cover actual successful flips for "+fixture)
            report["bds_metric_legalization_validated"] = True
            report["periodic_metric_actual_swaps_validated"] = True
            report["metric_cad_derivatives_validated"] = True
            report["metric_cad_derivatives_from_reference"] = False
        if args.check_face_mesh:
            report["seam_reconstruction_validated"] = True
            report["original_mvertex_metadata_from_reference"] = True
            report["bds_to_face_mesh_validated"] = True
        if args.check_front_node_sizes:
            cases = report["bds_front_node_sizes_cases"]
            if not any(c["copies"] and c["parameter_overrides"] for c in cases): raise ValueError("periodic front matrix did not cover temporary copies and periodic UV override")
            if not any(c["embedded_queries"] and c["different_dual_sizes"] for c in cases): raise ValueError("periodic front matrix did not cover embedded dual size exception")
        if args.check_front_generation:
            cases = report["front_generation_cases"]
            if len(cases) != expected_initial or not any(c["stats"][3] and c["new_triangle_pops"] for c in cases): raise ValueError("periodic generation feedback coverage incomplete")
            report["complete_front_generation_validated"] = True
            report["initial_front_pool_frozen"] = False
            report["front_mesh_transfer_validated"] = True
            report["generation_scope"] = "Own full periodic chart/initial/BDS/recovery/domain/cleanup/metric/seam/face/node/front prefix enters generateFrontMesh and transferFrontMesh in one run with independent CAD trueBoundary; 1D/topology order/raw reparameterization remain frozen; smoothing/assembly incomplete."
        if args.check_smoothing:
            report["surface_smoothing_validated"] = True
            report["smoothing_scope"] = "Own periodic chart/initial/BDS/front-generation/transfer/smoothing runs continuously; original seam nodes are mapped before smoothing, temporary objects retained through smoothing. Smoothing parameters/topology independently computed from BREP; earlier 1D/topology/raw chart/front-sizing inputs still frozen. No final seam-object cleanup/assembly/full generic inverse/quality acceptance."
        report["complete"] = True
        if args.check_front_insertion:
            report["first_front_point_insertion_validated"] = True
            report["insertion_scope"] = "Only first eligible candidate commits in an isolated test copy; full front-generation feedback and outer trueBoundary gate are not validated."
    finally:
        (directory/"periodic-embedded-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles: handle.close()
    print(json.dumps({"cases":len(report["cases"]),"native_periodic_embedded_equivalent":report["complete"],
                      "full_chart_cases":len(report.get("full_chart_cases",[])),
                      "full_chart_composition_validated":report.get("full_chart_composition_validated",False),
                      "initial_triangulation_cases":len(report.get("initial_triangulation_cases",[])),
                      "initial_triangulation_validated":report.get("initial_triangulation_validated",False),
                      "bds_conversion_cases":len(report.get("bds_conversion_cases",[])),"bds_conversion_validated":report.get("bds_conversion_validated",False),
                      "bds_recovery_cases":len(report.get("bds_recovery_cases",[])),"bds_recovery_validated":report.get("bds_recovery_validated",False),
                      "bds_domain_cases":len(report.get("bds_domain_cases",[])),"bds_domain_validated":report.get("bds_domain_validated",False),
                      "bds_interior_cases":len(report.get("bds_interior_cases",[])),"bds_interior_validated":report.get("bds_interior_validated",False),
                      "bds_metric_cases":len(report.get("bds_metric_cases",[])),"bds_metric_validated":report.get("bds_metric_validated",False),
                      "face_mesh_cases":len(report.get("bds_face_mesh_cases",[])),"face_mesh_validated":report.get("bds_face_mesh_validated",False),
                      "front_node_size_cases":len(report.get("bds_front_node_sizes_cases",[])),"front_node_sizes_validated":report.get("bds_front_node_sizes_validated",False),
                      "front_triangle_cases":len(report.get("bds_front_triangles_cases",[])),"front_triangles_validated":report.get("bds_front_triangles_validated",False),
                      "front_queue_cases":len(report.get("bds_front_queue_cases",[])),"front_queue_validated":report.get("bds_front_queue_validated",False),
                      "front_generation_cases":len(report.get("front_generation_cases",[])),"front_generation_validated":report.get("complete_front_generation_validated",False),
                      "front_mesh_transfer_validated":report.get("front_mesh_transfer_validated",False),
                      "surface_smoothing_validated":report.get("surface_smoothing_validated",False),
                      "front_point_cases":len(report.get("bds_front_points_cases",[])),"front_points_validated":report.get("bds_front_points_validated",False),
                      "front_cavity_cases":len(report.get("bds_front_cavity_cases",[])),"front_cavity_validated":report.get("bds_front_cavity_validated",False)}))


if __name__ == "__main__":
    main()
