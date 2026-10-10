"""Optional external comparison; requires an already installed gmsh and numpy.

Never imported by the product. Each run uses a fresh Gmsh model and unique output.
The triangle exchange file records face area/centroid for explicit OCC mapping.
"""
import argparse
import hashlib
import json
import pathlib
import sys
import time

import numpy as np


def distribution(values):
    return dict(zip(("min", "p50", "p95", "max"),
                    map(float, np.quantile(values, [0, .5, .95, 1]))))


def run(args, curvature, algorithm, threads, repeat):
    folder = args.output / f"curvature-{curvature}-algorithm-{algorithm}-threads-{threads}-run-{repeat}"
    folder.mkdir(exist_ok=False)
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.logger.start()
        settings = {"General.NumThreads": threads, "Mesh.MaxNumThreads1D": threads,
                    "Mesh.MaxNumThreads2D": threads, "Mesh.Algorithm": algorithm,
                    "Mesh.MeshSizeFromPoints": 0, "Mesh.MeshSizeExtendFromBoundary": 1,
                    "Mesh.MeshSizeFromCurvature": curvature, "Mesh.MeshSizeMin": 0,
                    "Mesh.MeshSizeMax": args.target, "Mesh.ElementOrder": 1}
        for key, value in settings.items():
            gmsh.option.setNumber(key, value)
        settings = {key: gmsh.option.getNumber(key) for key in settings}
        for key in ("Mesh.Smoothing", "Mesh.Optimize", "Mesh.OptimizeNetgen"):
            settings[key] = gmsh.option.getNumber(key)
        start = time.perf_counter()
        gmsh.model.occ.importShapes(str(args.cad))
        gmsh.model.occ.synchronize()
        imported = time.perf_counter()
        topology = {str(d): len(gmsh.model.getEntities(d)) for d in range(4)}
        gmsh.model.mesh.generate(1)
        edges_done = time.perf_counter()
        gmsh.model.mesh.generate(2)
        faces_done = time.perf_counter()
        tags, xyz, _ = gmsh.model.mesh.getNodes()
        xyz = np.asarray(xyz).reshape(-1, 3)
        index = {int(tag): i for i, tag in enumerate(tags)}
        chunks, face_ids, signatures, max_normal = [], [], [], 0.
        for _, face in gmsh.model.getEntities(2):
            signatures.append([face, gmsh.model.occ.getMass(2, face),
                               *gmsh.model.occ.getCenterOfMass(2, face)])
            types, _, nodes = gmsh.model.mesh.getElements(2, face)
            if list(types) != [2]:
                raise RuntimeError(f"face {face}: expected linear triangles, got {types}")
            tri = np.asarray([index[int(tag)] for tag in nodes[0]], dtype=np.int64).reshape(-1, 3)
            chunks.append(tri)
            face_ids.extend([face] * len(tri))
            unique, inverse = np.unique(tri, return_inverse=True)
            uv = gmsh.model.getParametrization(2, face, xyz[unique].ravel())
            normal = np.asarray(gmsh.model.getNormal(face, uv)).reshape(-1, 3)[inverse].reshape(-1, 3, 3)
            magnitude = np.linalg.norm(normal, axis=2)
            if np.any(magnitude <= 0) or not np.all(np.isfinite(magnitude)):
                raise RuntimeError(f"face {face}: singular/non-finite nodal normal")
            normal /= magnitude[:, :, None]
            for a, b in ((0, 1), (1, 2), (2, 0)):
                max_normal = max(max_normal, float(np.degrees(np.arccos(np.clip(
                    np.sum(normal[:, a] * normal[:, b], axis=1), -1., 1.))).max()))
        tri = np.concatenate(chunks)
        a, b, c = (xyz[tri[:, i]] for i in range(3))
        lengths = np.stack([np.linalg.norm(a-b, axis=1), np.linalg.norm(b-c, axis=1),
                            np.linalg.norm(c-a, axis=1)], axis=1)
        twice_area = np.linalg.norm(np.cross(b-a, c-a), axis=1)
        if not np.all(np.isfinite(xyz)) or np.any(twice_area <= 0):
            raise RuntimeError("non-finite or degenerate triangles")
        rho = np.prod(lengths, axis=1) / (2 * twice_area * lengths.min(axis=1))
        x, y, z = lengths.T
        angles = np.degrees(np.arctan2(twice_area[:, None], np.stack([
            (x*x+z*z-y*y)/2, (x*x+y*y-z*z)/2, (y*y+z*z-x*x)/2], axis=1)))
        minimum_angles = angles.min(axis=1)
        shape_quality = 2*np.sqrt(3)*twice_area / np.sum(lengths*lengths, axis=1)
        near_equilateral = np.all((angles >= 50) & (angles <= 70), axis=1)
        mesh_edges = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
        _, counts = np.unique(np.sort(mesh_edges, axis=1), axis=0, return_counts=True)
        checked = time.perf_counter()
        report = {"ok": True, "version": gmsh.__version__,
                  "sdkModule": gmsh.__file__,
                  "build": gmsh.option.getString("General.BuildInfo"), "settings": settings,
                  "cad": str(args.cad), "cadSha256": hashlib.sha256(args.cad.read_bytes()).hexdigest(),
                  "scriptSha256": hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                  "topology": topology, "points": len(xyz), "triangles": len(tri),
                  "seconds": {"import": imported-start, "edges": edges_done-imported,
                              "faces": faces_done-edges_done, "quality": checked-faces_done},
                  "edgeLength": distribution(lengths.ravel()), "radiusEdgeRatio": distribution(rho),
                  "minAngle": float(minimum_angles.min()),
                  "p5Angle": float(np.quantile(minimum_angles, .05)),
                  "p50Angle": float(np.quantile(minimum_angles, .5)),
                  "p5Shape": float(np.quantile(shape_quality, .05)),
                  "p50Shape": float(np.quantile(shape_quality, .5)),
                  "nearEquilateral": int(near_equilateral.sum()),
                  "nearEquilateralFraction": float(near_equilateral.mean()),
                  "vertexNormalAngleMaxDegrees": max_normal,
                  "openEdges": int(np.count_nonzero(counts == 1)),
                  "nonManifoldEdges": int(np.count_nonzero(counts > 2)),
                  "qualityNote": "Nodal normal angles; not the product's full sampling criterion. Independent OCC audit is separate."}
        # One representative mesh per setting; timings exclude serialization.
        if repeat == 1:
            with (folder / "surface.txt").open("w", encoding="ascii") as stream:
                stream.write(f"MESHCAD1 {len(signatures)} {len(xyz)} {len(tri)}\n")
                for row in signatures:
                    stream.write(" ".join(format(x, ".17g") for x in row) + "\n")
                for row in xyz:
                    stream.write(" ".join(format(x, ".17g") for x in row) + "\n")
                for face, row in zip(face_ids, tri):
                    stream.write(f"{face} {row[0]} {row[1]} {row[2]}\n")
        (folder / "run.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        (folder / "gmsh.log").write_text("\n".join(gmsh.logger.get()), encoding="utf-8")
        print(folder.name, report["seconds"], "triangles", len(tri), flush=True)
        return report
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad", type=pathlib.Path, required=True)
    parser.add_argument("--target", type=float, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--curvatures", type=int, nargs="+", default=[0, 12])
    parser.add_argument("--sdk-lib", type=pathlib.Path)
    parser.add_argument("--expected-version")
    parser.add_argument("--algorithms", type=int, nargs="+", choices=(5, 6), default=[5, 6])
    parser.add_argument("--threads", type=int, nargs="+", default=[1, 4])
    parser.add_argument("--repeats", type=int, default=3)
    args = parser.parse_args()
    if not np.isfinite(args.target) or args.target <= 0 or not args.cad.is_file():
        parser.error("valid CAD and positive finite target required")
    if args.sdk_lib:
        if not args.sdk_lib.is_dir():
            parser.error("SDK library directory does not exist")
        sys.path.insert(0, str(args.sdk_lib.resolve()))
    import gmsh
    if args.expected_version and gmsh.__version__ != args.expected_version:
        parser.error(f"expected Gmsh {args.expected_version}, got {gmsh.__version__}")
    if not 1 <= args.repeats <= 20 or any(value < 1 for value in args.threads):
        parser.error("repeats must be in [1,20] and threads must be positive")
    args.output.mkdir(exist_ok=True)
    results = []
    if any(value < 0 for value in args.curvatures):
        parser.error("curvature element counts must be nonnegative")
    for curvature in args.curvatures:
        for algorithm in args.algorithms:
            for threads in args.threads:
                for repeat in range(1, args.repeats+1):
                    results.append(run(args, curvature, algorithm, threads, repeat))
    summary = args.output / ("summary-" + "-".join(map(str, args.curvatures)) + ".json")
    with summary.open("x", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2)
