"""固定Gmsh实际laplaceSmoothing与自研核逐值对照；同面MFaceVertex，冻结连接/分类/入口UV+XYZ。"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
from GmshReproductionComparison import load_reference_api, first_trace_difference


def reference_smoothing(gmsh, face, nodes, triangles, iterations, failed, classified=False):
    pointer = ctypes.POINTER(ctypes.c_double)
    native = gmsh.lib.meshstudioReferenceSurfaceSmoothingClassified if classified else gmsh.lib.meshstudioReferenceSurfaceSmoothing
    native.argtypes = [ctypes.c_int, pointer, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int,
                       ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char), ctypes.c_int]
    native.restype = ctypes.c_int
    flat = [x for row in nodes for x in row]
    flat_triangles = [x for row in triangles for x in row]
    buffer = ctypes.create_string_buffer(16 * 1024 * 1024)
    code = native(face, (ctypes.c_double * len(flat))(*flat), len(nodes),
                  (ctypes.c_int * len(flat_triangles))(*flat_triangles), len(triangles),
                  iterations, failed, buffer, len(buffer))
    if code != 0:
        raise ValueError("actual native smoothing failed: " + str(code))
    return json.loads(buffer.value.decode())


def generation_smoothing_input(face, generation, transfer, face_mesh):
    """Native prefix metadata + native generated live vertices; never replace stored UV with seam slots."""
    vertices = {r[0]:[r[0],r[3],r[4],*r[5:10],int(r[3] >= 1),int(r[3] == 2),0] for r in face_mesh['vertices']}
    final = generation['final']
    slots = {r[0]:final['slots'][r[1]][1:3] for r in final['live_indices']}
    for row in generation.get('vertices',final.get('vertices',[])):
        id = row[0]
        if id in vertices:
            vertices[id][3:6] = row[1:4]
        else:
            if row[4] != 2 or id not in slots:
                raise ValueError('generated smoothing vertex lacks face classification/stored UV')
            vertices[id] = [id,2,face,*row[1:4],*slots[id],1,1,0]
    return [vertices[id] for id in sorted(vertices)], transfer['triangles']


def validate_result(result, iterations):
    """Check observed decisions and query counts without recomputing smoothing geometry."""
    events = result["events"]
    if len(events) != result["visits"]:
        raise ValueError("smoothing event count differs")
    eligible = writes = queries = 0
    per_iteration = {}
    for event in events:
        per_iteration.setdefault(event["iteration"], []).append(event["vertex"])
        if event["skip"]:
            if event["trials"] or event["neighbors"]:
                raise ValueError("protected vertex has smoothing work")
            continue
        eligible += 1
        if len(event["trials"]) != 5:
            raise ValueError("native five trials missing")
        neighbors = event["neighbors"]
        if [n[0] for n in neighbors] != sorted(n[0] for n in neighbors):
            raise ValueError("neighbors not in node identity order")
        # Each incident triangle touches the central vertex with exactly two edges.
        incident = sum(event["vertex"] in t for t in result["triangles"])
        queries += 2 * incident + len(neighbors)
        actual = event["before"]
        for trial in event["trials"]:
            accepted, succeeded, published, publication_succeeded = trial[2], trial[3], trial[9], trial[10]
            if published and not (accepted and succeeded and publication_succeeded):
                raise ValueError("unaccepted/failed surface point was published")
            if published:
                actual = trial[1]
                writes += 1
            if trial[12] != actual:
                raise ValueError("actual position does not follow completed publications")
            expected_queries = 1 + (6 * incident if succeeded else 0)
            if trial[13] != expected_queries:
                raise ValueError("acceptance raw reparameterization query count differs")
            queries += expected_queries + int(accepted)
    if any(ids != sorted(ids) for ids in per_iteration.values()):
        raise ValueError("vertex visits not in identity order")
    if len(per_iteration) != max(iterations, 0):
        raise ValueError("smoothing iteration count differs")
    if (eligible, writes, queries) != (result["eligible"], result["writes"], result["queries"]):
        raise ValueError("smoothing decision/count summary differs")


def patches(gmsh, face):
    low, high = gmsh.model.getParametrizationBounds(2, face)
    def nodes(points, protected):
        # Scrambled node identities exercise both adjacency and neighbor identity ordering.
        ids = [11 + ((i * 7) % len(points)) * 3 for i in range(len(points))]
        if len(set(ids)) != len(ids):
            ids = [11 + (len(points) - i) * 3 for i in range(len(points))]
        rows = []
        for i, (u, v) in enumerate(points):
            uv = [float(low[0] + (high[0] - low[0]) * u), float(low[1] + (high[1] - low[1]) * v)]
            rows.append([ids[i], *uv, *map(float, gmsh.model.getValue(2, face, uv)), int(i in protected)])
        return rows, ids
    for center in ((.32, .41), (.63, .28)):
        points = [(.15,.15),(.85,.15),(.85,.85),(.15,.85),center]
        rows, ids = nodes(points, {0,1,2,3})
        yield "fan-"+str(center), rows, [[ids[i], ids[(i+1)%4], ids[4]] for i in range(4)]
    points, protected = [], set()
    for j in range(4):
        for i in range(4):
            u, v = .15 + .7*i/3., .15 + .7*j/3.
            if 0 < i < 3 and 0 < j < 3:
                u += .024 * (1 if j == 1 else -1); v += .038 * (1 if i == 1 else -1)
            else:
                protected.add(len(points))
            points.append((u,v))
    rows, ids = nodes(points, protected)
    triangles = []
    for j in range(3):
        for i in range(3):
            a = j*4+i; b=a+1; c=a+4; d=c+1
            triangles += [[ids[a],ids[b],ids[d]], [ids[a],ids[d],ids[c]]]
    yield "four-interior-sequential", rows, triangles


def main():
    from GmshBdsMeshConversionComparison import create_metric_fixture
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("api", "library", "own-executable", "frozen-dir", "out-dir"):
        parser.add_argument("--"+name, required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args(); directory = Path(args.out_dir); directory.mkdir(parents=True, exist_ok=False)
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in args.runtime_dir]
    os.environ["PATH"] = os.pathsep.join(args.runtime_dir + [os.environ["PATH"]])
    os.environ.pop("MESHSTUDIO_GMSH_TRACE", None)
    gmsh = load_reference_api(args.api, args.library)
    report = {"scope": __doc__, "complete": False, "cases": [], "topology_and_classification_frozen": True,
              "same_face_stored_parameters_only": True, "raw_0D_1D_seam_adapters_validated": False,
              "continuous_front_to_smoothing_validated": False, "full_cad_quality_acceptance": False,
              "library_sha256": hashlib.sha256(Path(args.library).read_bytes()).hexdigest(),
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        warped = directory / "warped-bspline.brep"; create_metric_fixture(gmsh, "warped-bspline", warped)
        for fixture in ("rectangle", "cylinder", "sphere", "torus", "warped-bspline"):
            cad = warped if fixture == "warped-bspline" else Path(args.frozen_dir)/(fixture+".brep")
            gmsh.initialize([], readConfigFiles=False)
            try:
                if not gmsh.option.getString("General.Version").startswith("4.15.2"):
                    raise ValueError("wrong fixed Gmsh version")
                gmsh.option.setNumber("General.Terminal", 0.); gmsh.option.setNumber("General.NumThreads", 1.)
                gmsh.model.add(fixture); gmsh.model.occ.importShapes(str(cad), highestDimOnly=False); gmsh.model.occ.synchronize()
                inputs, expected, descriptors = [], [], []
                for _, face in gmsh.model.getEntities(2):
                    if fixture == "cylinder" and gmsh.model.getType(2,face) != "Cylinder":
                        continue
                    for name, rows, triangles in patches(gmsh, face):
                        for reverse in (False, True):
                            ordered = [list(reversed(t)) for t in reversed(triangles)] if reverse else triangles
                            for iterations, failed in ((0,0),(-1,0),(1,0),(2,0),(1,1),(1,2)):
                                result = reference_smoothing(gmsh, face, rows, ordered, iterations, failed)
                                validate_result(result, iterations); expected.append(result)
                                descriptors.append([face,name,reverse,iterations,failed])
                                inputs.append([face,len(rows),len(ordered),iterations,failed,*[v for row in rows for v in row],*[v for t in ordered for v in t]])
                frozen = directory/(fixture+"-input.tsv")
                frozen.write_text(str(len(inputs))+"\n"+"".join(" ".join(format(v,".17g") for v in row)+"\n" for row in inputs), encoding="utf-8")
                own = directory/(fixture+"-own.json")
                subprocess.run([args.own_executable,str(cad),str(frozen),str(own)],check=True,timeout=60)
                actual = json.loads(own.read_text(encoding="utf-8"))["cases"]
                (directory/(fixture+"-reference.json")).write_text(json.dumps(expected,allow_nan=False),encoding="utf-8")
                summary = {"fixture": fixture, "count": len(inputs), "visits": 0, "eligible": 0, "writes": 0, "queries": 0,
                           "accepted": 0, "rejected": 0, "failed_points": 0, "failed_publications": 0,
                           "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(), "first_difference": None}
                report["cases"].append(summary)
                if len(actual) != len(expected):
                    raise ValueError("smoothing case count differs")
                for index, (reference, result) in enumerate(zip(expected, actual)):
                    validate_result(result, descriptors[index][3])
                    difference = first_trace_difference([reference],[result],0.)
                    if difference:
                        summary["first_difference"] = {"index":index,"case":descriptors[index],"difference":difference}
                        raise ValueError("smoothing divergence: "+json.dumps(summary["first_difference"]))
                    for key in ("visits","eligible","writes","queries"):
                        summary[key] += reference[key]
                    for event in reference["events"]:
                        for trial in event["trials"]:
                            summary["accepted"] += int(trial[2]); summary["rejected"] += int(not trial[2])
                            summary["failed_points"] += int(not trial[3])
                            summary["failed_publications"] += int(trial[2] and not trial[10])
            finally:
                gmsh.finalize()
        if len(report["cases"]) != 5 or not sum(c["failed_publications"] for c in report["cases"]):
            raise ValueError("smoothing matrix or second point failure coverage incomplete")
        report["complete"] = True
    finally:
        (directory/"smoothing-report.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
        for handle in handles:
            handle.close()
    print(json.dumps({"cases":sum(c["count"] for c in report["cases"]),"visits":sum(c["visits"] for c in report["cases"]),
                      "queries":sum(c["queries"] for c in report["cases"]),"actual_smoothing_equal":report["complete"]}))


if __name__ == "__main__":
    main()
