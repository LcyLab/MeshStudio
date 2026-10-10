"""Test-only Gmsh 2D fixtures. Never generate Gmsh tetrahedra or alter CAD files."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import gmsh


def generate(case, size, output):
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("General.NumThreads", 1)
        gmsh.option.setNumber("Mesh.MaxNumThreads2D", 1)
        gmsh.option.setNumber("Mesh.Algorithm", 6)
        gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
        gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
        gmsh.option.setNumber("Mesh.MeshSizeMin", size)
        gmsh.option.setNumber("Mesh.MeshSizeMax", size)
        gmsh.option.setNumber("Mesh.ElementOrder", 1)
        gmsh.model.add(case)
        occ = gmsh.model.occ
        analytic = None
        if case == "cube":
            occ.addBox(0, 0, 0, 1, 1, 1)
            analytic = 1
        elif case == "concave":
            a = occ.addBox(0, 0, 0, 2, 2, 1)
            b = occ.addBox(1, 1, 0, 1, 1, 1)
            occ.cut([(3, a)], [(3, b)])
            analytic = 3
        elif case == "cavity":
            a = occ.addBox(0, 0, 0, 3, 3, 3)
            b = occ.addBox(1, 1, 1, 1, 1, 1)
            occ.cut([(3, a)], [(3, b)])
            analytic = 26
        elif case == "disconnected":
            occ.addBox(0, 0, 0, 1, 1, 1)
            occ.addBox(2, 0, 0, 1, 1, 1)
            analytic = 2
        elif case == "sphere":
            occ.addSphere(0, 0, 0, 1)
            analytic = 4 * math.pi / 3
        else:
            raise ValueError(case)
        occ.synchronize()
        for _, volume in gmsh.model.getEntities(3):
            gmsh.model.mesh.setOutwardOrientation(volume)
        gmsh.model.mesh.generate(2)
        tags, xyz, _ = gmsh.model.mesh.getNodes()
        points = {int(tag): tuple(map(float, xyz[3*i:3*i+3])) for i, tag in enumerate(tags)}
        triangles = []
        labels = []
        uv = []
        for _, face in gmsh.model.getEntities(2):
            types, _, node_lists = gmsh.model.mesh.getElements(2, face)
            for kind, nodes in zip(types, node_lists):
                if kind != 2:
                    raise RuntimeError(f"non-T3 surface element {kind}")
                for i in range(0, len(nodes), 3):
                    tri = tuple(map(int, nodes[i:i+3]))
                    triangles.append(tri)
                    labels.append(face)
                    values = gmsh.model.getParametrization(2, face, [c for tag in tri for c in points[tag]])
                    if len(values) != 6 or not all(math.isfinite(v) for v in values):
                        raise RuntimeError("invalid Gmsh corner UV")
                    uv.append(tuple(map(float, values)))
        edges = collections.defaultdict(list)
        for t, tri in enumerate(triangles):
            for k in range(3):
                a, b = tri[k], tri[(k+1) % 3]
                edges[min(a, b), max(a, b)].append((t, 1 if a < b else -1))
        if any(len(uses) != 2 or sum(sign for _, sign in uses) != 0 for uses in edges.values()):
            raise RuntimeError("raw Gmsh surface is not closed and consistently oriented")
        # Preserve only used Gmsh node identities; do not geometrically weld.
        used = sorted({v for tri in triangles for v in tri})
        index = {tag: i for i, tag in enumerate(used)}
        curve_edges = {}
        for _, curve in gmsh.model.getEntities(1):
            types, _, node_lists = gmsh.model.mesh.getElements(1, curve)
            for kind, nodes in zip(types, node_lists):
                if kind != 1:
                    continue
                for i in range(0, len(nodes), 2):
                    a, b = map(int, nodes[i:i+2])
                    if a in index and b in index and (min(a, b), max(a, b)) in edges:
                        curve_edges[min(a, b), max(a, b)] = curve
        # Independent surface integral of raw Gmsh boundary, relative to one origin.
        origin = points[used[0]]
        def relative(tag):
            return tuple(points[tag][i] - origin[i] for i in range(3))
        contributions = []
        for tri in triangles:
            a, b, c = map(relative, tri)
            contributions.append((a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6)
        surface_volume = abs(math.fsum(contributions))
        if case != "sphere" and abs(surface_volume - analytic) > analytic * 1e-9:
            raise RuntimeError(f"raw Gmsh planar surface volume {surface_volume} != {analytic}")
        output.mkdir(parents=True, exist_ok=False)
        mesh = output / "surface.gmstet"
        with mesh.open("x", encoding="ascii") as stream:
            stream.write(f"GMSTET1 {len(used)} {len(triangles)} {len(curve_edges)} {surface_volume:.17g}\n")
            for tag in used:
                stream.write(" ".join(format(v, ".17g") for v in points[tag]) + "\n")
            for face, tri, corners in zip(labels, triangles, uv):
                stream.write(" ".join(map(str, (face, *(index[v] for v in tri)))) + " " + " ".join(format(v, ".17g") for v in corners) + "\n")
            for (a, b), curve in sorted(curve_edges.items()):
                stream.write(f"{index[a]} {index[b]} {curve}\n")
        gmsh.write(str(output / "surface.msh"))
        report = {"generator": "Gmsh surface only, generate(2)", "version": gmsh.__version__,
                  "case": case, "surfaceSize": size, "points": len(used), "triangles": len(triangles),
                  "curveSegments": len(curve_edges), "rawClosedOriented": True,
                  "orientation": "Gmsh setOutwardOrientation before generate(2)", "exporterReversedTriangles": 0,
                  "surfaceVolume": surface_volume,
                  "exactCadVolume": analytic, "surfaceSha256": hashlib.sha256(mesh.read_bytes()).hexdigest()}
        (output / "source.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report), flush=True)
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=["cube", "concave", "cavity", "disconnected", "sphere"], required=True)
    parser.add_argument("--size", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.size) or args.size <= 0:
        parser.error("positive finite surface size required")
    generate(args.case, args.size, args.output)
