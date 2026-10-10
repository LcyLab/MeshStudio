"""Fixed-version, identical-boundary comparison of the independent generator.

Uses an explicit SDK path; never imported by the product or installed globally.
The nonlinear/sheared cases keep the physical plane but change our UV chart;
Gmsh keeps its native CAD chart, so these rows test physical-chart robustness.
"""
import argparse
import hashlib
import json
import math
import pathlib
import subprocess
import sys
import time

import numpy as np


def distribution(xyz, tri, boundary_xyz, boundary_edges, target=1.):
    corners = xyz[tri]
    a, b, c = (corners[:, k] for k in range(3))
    sides = np.stack([np.linalg.norm(a-b, axis=1), np.linalg.norm(b-c, axis=1), np.linalg.norm(c-a, axis=1)], axis=1)
    twice_area = np.linalg.norm(np.cross(b-a, c-a), axis=1)
    if not np.all(np.isfinite(xyz)) or np.any(twice_area <= 0):
        raise RuntimeError("invalid physical mesh")
    angles = np.stack([np.degrees(np.arccos(np.clip((sides[:, k]**2+sides[:, (k+1)%3]**2-sides[:, (k+2)%3]**2)/(2*sides[:, k]*sides[:, (k+1)%3]), -1., 1.))) for k in range(3)], axis=1)
    shape = 2*math.sqrt(3)*twice_area/np.sum(sides**2, axis=1)
    near = np.all((angles >= 50) & (angles <= 70), axis=1)
    distances = np.full(len(xyz), np.inf)
    for first, last in boundary_edges:
        p, q = boundary_xyz[first], boundary_xyz[last]
        direction = q-p
        t = np.clip((xyz-p)@direction/np.dot(direction, direction), 0, 1)
        distances = np.minimum(distances, np.linalg.norm(xyz-p-t[:, None]*direction, axis=1))
    inside = np.all(distances[tri] >= 2*target, axis=1)
    result = {"vertices": len(xyz), "triangles": len(tri), "area": float(twice_area.sum()/2),
              "near_all": float(near.mean()), "angle_p5": float(np.quantile(angles.min(axis=1), .05)), "q_p5": float(np.quantile(shape, .05)),
              "interior_triangles": int(inside.sum()), "near_interior": float(near[inside].mean()) if np.any(inside) else None}
    edge_count = {}
    for t in tri:
        for i in range(3):
            e = tuple(sorted((int(t[i]), int(t[(i+1)%3]))))
            edge_count[e] = edge_count.get(e, 0)+1
    if any(value > 2 for value in edge_count.values()):
        raise RuntimeError("non-manifold mesh edge")
    result["open_edges"] = sum(value == 1 for value in edge_count.values())
    return result


def mesh_data(gmsh, face):
    tags, coordinates, _ = gmsh.model.mesh.getNodes()
    coordinates = np.asarray(coordinates).reshape(-1, 3)
    lookup = {int(tag): k for k, tag in enumerate(tags)}
    types, _, nodes = gmsh.model.mesh.getElements(2, face)
    if list(types) != [2]:
        raise RuntimeError(f"expected linear triangles, got {types}")
    tri = np.asarray([lookup[int(tag)] for tag in nodes[0]], dtype=np.int64).reshape(-1, 3)
    active = np.unique(tri)
    return coordinates[active], np.searchsorted(active, tri)


def run_own(args, case, source, boundary, edges, variant):
    destination = args.output / f"{case}.{variant}.own.txt"
    command = [str(args.executable), str(source), str(destination)]
    if variant != "reference":
        command.append(variant)
    start = time.perf_counter()
    process = subprocess.run(command, capture_output=True, text=True, check=False)
    seconds = time.perf_counter()-start
    (args.output / f"{case}.{variant}.own.log").write_text(process.stdout+process.stderr, encoding="utf-8")
    if process.returncode:
        raise RuntimeError(f"own {variant} failed: {process.stdout} {process.stderr}")
    data = destination.read_text().splitlines()
    node_count, tri_count = map(int, data[0].split())
    xyz = np.asarray([[float(x) for x in line.split()] for line in data[1:1+node_count]])
    tri = np.asarray([[int(x) for x in line.split()] for line in data[1+node_count:]], dtype=np.int64)
    if len(tri) != tri_count or not np.array_equal(xyz[:len(boundary)], boundary):
        raise RuntimeError("boundary changed or output count mismatch")
    edge_count = {}
    for t in tri:
        for k in range(3):
            edge = tuple(sorted((int(t[k]), int(t[(k+1)%3]))))
            edge_count[edge] = edge_count.get(edge, 0)+1
    actual_boundary = {edge for edge, count in edge_count.items() if count == 1}
    if actual_boundary != {tuple(sorted(map(int, e))) for e in edges}:
        raise RuntimeError("own protected boundary segments changed")
    metrics = distribution(xyz, tri, boundary, edges)
    return {"metrics": metrics, "seconds_including_process": seconds, "diagnostics": process.stdout.splitlines()}


def create_model(gmsh, case):
    gmsh.model.add(case)
    if case in ("plane", "sheared-plane", "nonlinear-plane", "holed-plane"):
        outer = gmsh.model.occ.addRectangle(0, 0, 0, 16, 12)
        if case == "holed-plane":
            hole = gmsh.model.occ.addRectangle(6, 4, 0, 4, 4)
            faces, _ = gmsh.model.occ.cut([(2, outer)], [(2, hole)])
            face = faces[0][1]
        else:
            face = outer
        gmsh.model.occ.synchronize()
        return face
    if case == "cylinder":
        gmsh.model.occ.addCylinder(0, 0, 0, 0, 0, 8, 2, angle=math.pi)
        kind = "Cylinder"
    else:
        gmsh.model.occ.addSphere(0, 0, 0, 8, angle1=-.6, angle2=.6, angle3=math.pi)
        kind = "Sphere"
    gmsh.model.occ.synchronize()
    faces = [tag for _, tag in gmsh.model.getEntities(2) if gmsh.model.getType(2, tag) == kind]
    if len(faces) != 1:
        raise RuntimeError(f"ambiguous {kind} face: {faces}")
    return faces[0]


def boundary_data(gmsh, face):
    edges_by_tag = []
    for dim, edge in gmsh.model.getBoundary([(2, face)], combined=False):
        if dim != 1:
            raise RuntimeError("unexpected face boundary")
        types, _, vertices = gmsh.model.mesh.getElements(1, abs(edge))
        if list(types) != [1]:
            raise RuntimeError(f"unsupported edge elements {types}")
        edges_by_tag.extend(np.asarray(vertices[0], dtype=np.int64).reshape(-1, 2).tolist())
    tags = sorted({tag for e in edges_by_tag for tag in e})
    index = {tag: k for k, tag in enumerate(tags)}
    xyz = np.asarray([gmsh.model.mesh.getNode(tag)[0] for tag in tags])
    edges = np.asarray([(index[a], index[b]) for a, b in edges_by_tag])
    degree = np.bincount(edges.ravel(), minlength=len(tags))
    if np.any(degree != 2):
        raise RuntimeError("boundary is not a collection of closed loops")
    # Initial source estimate; the C++ harness reconstructs shortest seed-CDT
    # edges and the half-boundary-edge floor, matching the reference source rule.
    return xyz, edges


def own_uv(case, xyz):
    if case == "sheared-plane":
        return np.stack([(xyz[:, 0]-2*xyz[:, 1])/10, xyz[:, 1]/3], axis=1)
    if case == "nonlinear-plane":
        low, high = np.zeros(len(xyz)), np.full(len(xyz), 12.)
        for _ in range(64):
            mid = (low+high)/2
            physical = mid+.9*np.sin(2*math.pi*mid)/(2*math.pi)
            low, high = np.where(physical < xyz[:, 1], mid, low), np.where(physical < xyz[:, 1], high, mid)
        return np.stack([xyz[:, 0], (low+high)/2], axis=1)
    if case in ("cylinder", "sphere"):
        return np.stack([np.arctan2(xyz[:, 1], xyz[:, 0]), xyz[:, 2] if case == "cylinder" else np.arcsin(xyz[:, 2]/8)], axis=1)
    return xyz[:, :2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-lib", type=pathlib.Path, required=True)
    parser.add_argument("--executable", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--cases", nargs="+", default=["plane", "holed-plane", "sheared-plane", "nonlinear-plane", "cylinder", "sphere"])
    parser.add_argument("--variants", nargs="+", choices=["reference", "physical-cavity", "physical-chart"], default=["reference", "physical-cavity", "physical-chart"])
    args = parser.parse_args()
    sys.path.insert(0, str(args.sdk_lib.resolve()))
    import gmsh
    if gmsh.__version__ != "4.15.2":
        raise RuntimeError(f"version must match fixed source 4.15.2, got {gmsh.__version__}")
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"gmsh_version": gmsh.__version__, "sdk_module": gmsh.__file__,
              "own_executable_sha256": hashlib.sha256(args.executable.read_bytes()).hexdigest(), "cases": []}
    shapes = {"plane": 0, "holed-plane": 0, "sheared-plane": 1, "nonlinear-plane": 2, "cylinder": 3, "sphere": 4}
    for case in args.cases:
        gmsh.initialize()
        try:
            gmsh.option.setNumber("General.Terminal", 0)
            gmsh.logger.start()
            settings = {"General.NumThreads": 1, "Mesh.MaxNumThreads1D": 1, "Mesh.MaxNumThreads2D": 1,
                        "Mesh.Algorithm": 6, "Mesh.MeshSizeFromPoints": 0, "Mesh.MeshSizeFromCurvature": 0,
                        "Mesh.MeshSizeExtendFromBoundary": 1, "Mesh.MeshSizeMin": 0, "Mesh.MeshSizeMax": 1,
                        "Mesh.Smoothing": 0, "Mesh.Optimize": 0, "Mesh.OptimizeNetgen": 0}
            for option, value in settings.items():
                gmsh.option.setNumber(option, value)
            face = create_model(gmsh, case)
            gmsh.model.mesh.generate(1)
            boundary, edges = boundary_data(gmsh, face)
            uv = own_uv(case, boundary)
            source = args.output / f"{case}.seed.txt"
            with source.open("w", encoding="utf-8") as stream:
                stream.write(f"surface-reference-v2 {shapes[case]} {len(boundary)} {len(edges)}\n")
                for p, q in zip(uv, boundary):
                    stream.write(" ".join(format(float(x), ".17g") for x in [*p, *q, 1., 1.])+"\n")
                for e in edges:
                    stream.write(f"{e[0]} {e[1]}\n")
            variants = {variant: run_own(args, case, source, boundary, edges, variant) for variant in args.variants}
            start = time.perf_counter()
            gmsh.model.mesh.generate(2)
            gmsh_seconds = time.perf_counter()-start
            gmsh_xyz, gmsh_tri = mesh_data(gmsh, face)
            gmsh_raw = distribution(gmsh_xyz, gmsh_tri, boundary, edges)
            gmsh.model.mesh.optimize("Laplace2D", force=False, niter=1, dimTags=[(2, face)])
            smooth_xyz, smooth_tri = mesh_data(gmsh, face)
            gmsh_smooth = distribution(smooth_xyz, smooth_tri, boundary, edges)
            logs = gmsh.logger.get()
            (args.output / f"{case}.gmsh.log").write_text("\n".join(logs), encoding="utf-8")
            result = {"name": case, "settings": settings, "boundary_nodes": len(boundary), "variants": variants, "gmsh_raw": gmsh_raw,
                      "gmsh_one_laplace": gmsh_smooth,
                      "gmsh_seconds_all_faces": gmsh_seconds, "fallback_reported": any("MeshAdapt" in line for line in logs)}
            for variant in variants.values():
                if variant["metrics"]["near_interior"] is not None and gmsh_raw["near_interior"] is not None:
                    variant["near_interior_gap"] = variant["metrics"]["near_interior"]-gmsh_raw["near_interior"]
            report["cases"].append(result)
            (args.output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
            print(case, {name: (value["metrics"]["near_interior"], value["metrics"]["triangles"]) for name, value in variants.items()}, "gmsh", (gmsh_raw["near_interior"], gmsh_raw["triangles"]), flush=True)
        finally:
            gmsh.finalize()


if __name__ == "__main__":
    main()
