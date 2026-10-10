"""Plot final meshes with matching views and optional recorded audit positions."""
import argparse
import json
import hashlib
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.tri as tri
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.colors import Normalize


def read_mesh(path):
    root = ET.parse(path).getroot()
    points = list(map(float, root.find(".//Points/DataArray").text.split()))
    nodes = [points[k:k+3] for k in range(0, len(points), 3)]
    ids = list(map(int, root.find(".//Polys/DataArray[@Name='connectivity']").text.split()))
    triangles = [ids[k:k+3] for k in range(0, len(ids), 3)]
    return nodes, triangles


def audit_markers(path, nodes, triangles):
    if path is None:
        return [], None
    report = json.loads(path.read_text(encoding="utf-8"))
    outside, wrong_uv, intersections = [], [], []
    for message in report["violations"]:
        if message.startswith("triangle centroid outside trim/hole:"):
            coordinates = re.search(r"\bxyz=([^ ]+)", message)
            if coordinates:
                point = [float(v) for v in coordinates.group(1).split(",")]
                if len(point) != 3:
                    raise ValueError("invalid recorded trim centroid")
                outside.append(point)
        elif message.startswith("corner UV disagrees with XYZ/CAD face:"):
            node = re.search(r"\bnode=(\d+)", message)
            if node:
                wrong_uv.append(nodes[int(node.group(1))])
        else:
            pair = re.search(r"surface triangles intersect: (\d+) and (\d+)", message)
            if pair:
                for triangle in map(int, pair.groups()):
                    intersections.append(np.asarray(nodes)[triangles[triangle]].mean(axis=0).tolist())
    markers = [(outside, "x", "#ff2448", "outside trim centroid on CAD support"),
               (wrong_uv, "o", "#ffab00", "node with inconsistent corner UV"),
               (intersections, "+", "#f200ff", "first reported intersection: triangle centers")]
    metadata = dict(audit=str(path.resolve()), audit_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    complete=report["complete"], topology=report["topology"],
                    positions=dict(outside=outside, wrong_uv=wrong_uv, first_intersection=intersections),
                    details_limit=report["violation_details_limit"])
    return markers, metadata


def clip_local_polygon(polygon, lower, upper):
    # 仅裁剪展示多边形，防止跨度大的原片穿出局部图；指标仍来自原三角形。
    for axis in range(3):
        for bound, direction in ((lower[axis], 1), (upper[axis], -1)):
            clipped = []
            if len(polygon) == 0:
                return clipped
            previous = polygon[-1]
            previous_inside = direction*(previous[axis]-bound) >= 0
            for current in polygon:
                current_inside = direction*(current[axis]-bound) >= 0
                if previous_inside != current_inside:
                    ratio = (bound-previous[axis])/(current[axis]-previous[axis])
                    point = previous + ratio*(current-previous)
                    point[axis] = bound
                    clipped.append(point)
                if current_inside:
                    clipped.append(current)
                previous, previous_inside = current, current_inside
            polygon = clipped
    return polygon


def plot_curved_pair(own, reference, output, title, focus=None, own_audit=None, reference_audit=None):
    meshes = [read_mesh(own), read_mesh(reference)]
    diagnostics = [audit_markers(path, *mesh) for path, mesh in zip((own_audit, reference_audit), meshes)]
    positions = [np.asarray(nodes) for nodes, _ in meshes]
    lower = np.minimum(positions[0].min(axis=0), positions[1].min(axis=0))
    upper = np.maximum(positions[0].max(axis=0), positions[1].max(axis=0))
    center = (lower + upper) / 2
    span = float(np.max(upper - lower))
    if focus is None:
        target = center + (upper - lower) * np.array([.4, 0, .35])
        focus = positions[0][np.argmin(np.sum((positions[0] - target)**2, axis=1))]
    figure = plt.figure(figsize=(12, 10), layout="constrained")
    normalization = Normalize(20, 60)
    for col, ((nodes, triangles), side) in enumerate(zip(meshes, ("Own Algorithm 6", "Gmsh 4.15.2"))):
        xyz = np.asarray(nodes)[np.asarray(triangles)]
        lengths = np.linalg.norm(xyz - np.roll(xyz, -1, axis=1), axis=2)
        angles = []
        for k in range(3):
            a, b, c = lengths[:, (k+1)%3], lengths[:, k], lengths[:, (k+2)%3]
            cosine = np.divide(b*b+c*c-a*a, 2*b*c, out=np.ones_like(a), where=b*c > 0)
            angles.append(np.degrees(np.arccos(np.clip(cosine, -1, 1))))
        minimum = np.min(angles, axis=0)
        for row in (0, 1):
            ax = figure.add_subplot(2, 2, row*2+col+1, projection="3d", computed_zorder=False)
            view_center = center if row == 0 else np.asarray(focus)
            half = span * (.55 if row == 0 else .13)
            if row == 0:
                polygons, values = xyz, minimum
            else:
                mask = np.all(xyz.min(axis=1) <= view_center+half, axis=1) & np.all(xyz.max(axis=1) >= view_center-half, axis=1)
                polygons, values = [], []
                for polygon, value in zip(xyz[mask], minimum[mask]):
                    clipped = clip_local_polygon(polygon, view_center-half, view_center+half)
                    if len(clipped) >= 3:
                        polygons.append(clipped)
                        values.append(value)
            collection = Poly3DCollection(polygons, linewidths=.3, edgecolors="#314052")
            collection.set_facecolor(plt.colormaps["viridis"](normalization(values)))
            ax.add_collection3d(collection)
            marked = False
            for recorded, symbol, color, label in diagnostics[col][0]:
                if recorded:
                    points = np.asarray(recorded)
                    points = points[np.all(np.abs(points-view_center) <= half, axis=1)]
                    if len(points):
                        ax.scatter(points[:,0], points[:,1], points[:,2], marker=symbol, color=color,
                                   s=32, linewidths=1.2, depthshade=False, zorder=10, label=label)
                        marked = True
            if marked:
                ax.legend(loc="upper left", fontsize=6)
            for axis, set_limits in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
                set_limits(view_center[axis]-half, view_center[axis]+half)
            ax.set_box_aspect((1, 1, 1));ax.set_proj_type("ortho");ax.view_init(elev=24, azim=35)
            ax.set_xlabel("x");ax.set_ylabel("y");ax.set_zlabel("z")
            ax.set_title(f"{side} | {len(triangles)} triangles | {'whole' if row == 0 else 'same local region'}", fontsize=10)
    audit_status = ""
    if own_audit is not None and reference_audit is not None:
        audit_status = " | audit: " + " / ".join("passed" if metadata["complete"] else "failed" for _, metadata in diagnostics)
    figure.suptitle(title + audit_status + "\nCAD coordinates in model units; color = minimum angle", fontsize=12)
    figure.colorbar(plt.cm.ScalarMappable(norm=normalization, cmap="viridis"), ax=figure.axes, shrink=.5, label="Minimum angle (degrees)")
    figure.savefig(output, dpi=180)
    plt.close(figure)
    if own_audit is not None or reference_audit is not None:
        output.with_suffix(".markers.json").write_text(json.dumps(dict(focus=np.asarray(focus).tolist(), local_display="polygons clipped to matching cube; colors measure original triangles", sides=[m for _, m in diagnostics]), indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--3d", dest="curved", action="store_true")
    parser.add_argument("--own", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--focus", nargs=3, type=float)
    parser.add_argument("--own-audit", type=Path)
    parser.add_argument("--reference-audit", type=Path)
    args = parser.parse_args()
    if (args.own_audit or args.reference_audit) and not (args.own and args.reference):
        parser.error("audit position overlays require the explicit --own / --reference pair")
    if args.own or args.reference:
        if not args.own or not args.reference:
            parser.error("both --own and --reference are required")
        plot_curved_pair(args.own, args.reference, args.directory / "whole-and-local-3d.png", "Independent final meshes", args.focus, args.own_audit, args.reference_audit)
        return
    report = json.loads((args.directory / "comparison.json").read_text())
    if args.curved:
        for case in report["cases"]:
            folder = args.directory / f"{case['case']}-extend{case['extension']}"
            own, reference = folder / "own/own.vtp", folder / "reference.vtp"
            if own.exists() and reference.exists():
                title = f"{case['case']} | h={case['size']} | extension={case['extension']} | shape {'passed' if case['shape_passed'] else 'not passed'}"
                plot_curved_pair(own, reference, folder / "whole-and-local-3d.png", title)
        return
    labels = dict(rectangle="Rectangle", hole="Plane with hole", narrow_band="Narrow band")
    for extension in (0, 1):
        for local in (False, True):
            figure, axes = plt.subplots(3, 2, figsize=(12, 9), layout="constrained")
            for row, name in enumerate(("rectangle", "hole", "narrow_band")):
                case = next(c for c in report["cases"] if c["case"] == name and c["extension"] == extension)
                for col, (side, filename) in enumerate((("own", "own/own.vtp"), ("reference", "reference.vtp"))):
                    nodes, triangles = read_mesh(args.directory / f"{name}-extend{extension}" / filename)
                    x, y = [p[0] for p in nodes], [p[1] for p in nodes]
                    ax = axes[row, col]
                    ax.triplot(tri.Triangulation(x, y, triangles), color="#354459", linewidth=.55)
                    ax.set_aspect("equal")
                    if local:
                        if name == "hole":
                            ax.set_xlim(1.3, 2.8); ax.set_ylim(.7, 2.3)
                        elif name == "narrow_band":
                            ax.set_xlim(.8, 1.8); ax.set_ylim(-.02, .22)
                        else:
                            ax.set_xlim(.4, 2); ax.set_ylim(.5, 2.1)
                    else:
                        ax.set_xlim(-.05, 4.05); ax.set_ylim((-.03 if name == "narrow_band" else -.05), (.23 if name == "narrow_band" else 3.05))
                    metric = case[side]["shape"]["all"]
                    ax.set_title(f"{labels[name]} — {'Own Algorithm 6' if side == 'own' else 'Gmsh 4.15.2'}\n{metric['triangles']} triangles | near-equilateral {metric['near_fraction']:.1%}", fontsize=10)
                    ax.set_xlabel("CAD x");ax.set_ylabel("CAD y")
                    ax.spines[["top", "right"]].set_visible(False)
            figure.suptitle(f"BREP-only Algorithm 6 | h=0.3 | boundary extension={extension} | {'matching local views' if local else 'matching full views'}", fontsize=13)
            path = args.directory / f"{'local' if local else 'full'}-comparison-extend{extension}"
            figure.savefig(path.with_suffix(".png"), dpi=180)
            figure.savefig(path.with_suffix(".pdf"))
            plt.close(figure)


if __name__ == "__main__":
    main()
