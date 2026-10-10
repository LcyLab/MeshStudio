"""Measure final VTP edge lengths and minimum-angle distributions independently.

CAD boundary edges are the exported 1D segments, not a distance-based selection.
Each physical mesh edge counts once; all other edges are reported as interior.
Normalized lengths use configured sizeMax, not a claimed local target size.
This report supplements the CAD audit and does not introduce acceptance gates.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def percentile(values, q):
    return sorted(values)[int(q*(len(values)-1))] if values else None


def summarize(values, limits):
    bins = [0]*(len(limits)+1)
    for value in values:
        index = next((i for i, limit in enumerate(limits) if value < limit), len(limits))
        bins[index] += 1
    return dict(count=len(values), p1=percentile(values, .01), p5=percentile(values, .05),
                p50=percentile(values, .5), p95=percentile(values, .95),
                maximum=max(values, default=None), histogram_upper_exclusive=limits,
                histogram_counts=bins)


def measure(nodes, triangles, cad_segments, size_max):
    if not math.isfinite(size_max) or size_max <= 0:
        raise ValueError('sizeMax must be finite and positive')
    if any(len(p) != 3 or not all(math.isfinite(v) for v in p) for p in nodes):
        raise ValueError('invalid physical coordinate')
    cad_edges = {tuple(sorted(edge)) for edge in cad_segments}
    edges, minimum_angles = set(), []
    for triangle in triangles:
        if len(triangle) != 3 or len(set(triangle)) != 3 or any(i < 0 or i >= len(nodes) for i in triangle):
            raise ValueError('invalid triangle connectivity')
        p = [nodes[i] for i in triangle]
        lengths = [math.dist(p[i], p[(i+1)%3]) for i in range(3)]
        if min(lengths) <= 0:
            raise ValueError('triangle has coincident physical vertices')
        angles = []
        for i in range(3):
            a = [p[(i+1)%3][k]-p[i][k] for k in range(3)]
            b = [p[(i+2)%3][k]-p[i][k] for k in range(3)]
            cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
            magnitude = math.sqrt(sum(v*v for v in cross))
            if magnitude == 0:
                raise ValueError('triangle has zero physical area')
            angles.append(math.degrees(math.atan2(magnitude, sum(a[k]*b[k] for k in range(3)))))
        minimum_angles.append(min(angles))
        edges.update(tuple(sorted((triangle[i], triangle[(i+1)%3]))) for i in range(3))
    boundary = edges & cad_edges
    if cad_edges - edges:
        raise ValueError('CAD segment is absent from final surface connectivity')
    regions = dict(all=edges, boundary=boundary, interior=edges-boundary)
    distributions = {}
    for region, identifiers in regions.items():
        lengths = [math.dist(nodes[a], nodes[b]) for a, b in identifiers]
        distributions[region] = dict(physical=summarize(lengths, [size_max*v for v in (.25, .5, .75, 1., 1.25, 1.5, 2.)]),
                                     relative_to_size_max=summarize([v/size_max for v in lengths], [.25, .5, .75, 1., 1.25, 1.5, 2.]))
    return dict(used_nodes=len({i for triangle in triangles for i in triangle}), triangles=len(triangles),
                unique_edge_lengths=distributions,
                minimum_angle_degrees=summarize(minimum_angles, [10., 20., 25., 30., 40., 50., 60.]))


def read_vtp(path):
    root = ET.parse(path).getroot()
    coordinates = list(map(float, root.find('.//Points/DataArray').text.split()))
    nodes = [coordinates[i:i+3] for i in range(0, len(coordinates), 3)]
    def cells(kind, width):
        array = root.find(f".//{kind}/DataArray[@Name='connectivity']")
        ids = list(map(int, (array.text or '').split())) if array is not None else []
        if len(ids) % width:
            raise ValueError('incomplete VTP cell')
        offsets = root.find(f".//{kind}/DataArray[@Name='offsets']")
        if offsets is not None and list(map(int, (offsets.text or '').split())) != list(range(width, len(ids)+1, width)):
            raise ValueError('nonlinear or malformed VTP cell offsets')
        return [ids[i:i+width] for i in range(0, len(ids), width)]
    return nodes, cells('Polys', 3), cells('Lines', 2)


def measure_file(path, size_max):
    result = measure(*read_vtp(path), size_max)
    result.update(vtp=str(path.resolve()), vtp_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--plots', action='store_true', help='also plot edge length and minimum-angle histograms')
    args = parser.parse_args()
    report = json.loads((args.directory/'comparison.json').read_text(encoding='utf-8'))
    cases = []
    for case in report['cases']:
        folder = args.directory/f"{case['case']}-extend{case['extension']}"
        paths = dict(own=folder/'own/own.vtp', reference=folder/'reference.vtp')
        if not all(path.exists() for path in paths.values()):
            cases.append(dict(case=case['case'], extension=case['extension'], measured=False, reason='missing final VTP'))
            continue
        cases.append(dict(case=case['case'], extension=case['extension'], size_max=case['size'], measured=True,
                          **{side: measure_file(path, case['size']) for side, path in paths.items()}))
    output = dict(contract='unique physical edges; boundary=CAD 1D segments; interior=all remaining edges; minimum angle per triangle; quantile index=floor(q*(n-1))',
                  interpretation='length/sizeMax is a scale comparison; point/curvature/boundary propagation may prescribe smaller local sizes; no new acceptance gates',
                  comparison_sha256=hashlib.sha256((args.directory/'comparison.json').read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), cases=cases)
    (args.directory/'mesh-distributions.json').write_text(json.dumps(output, indent=2), encoding='utf-8')
    if args.plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        for case in cases:
            if not case['measured']:
                continue
            figure, axes = plt.subplots(2, 2, figsize=(11, 7), layout='constrained')
            specifications = [(region, f'Unique edge lengths: {region}') for region in ('all', 'boundary', 'interior')]
            specifications.append(('angles', 'Minimum angle per triangle'))
            for ax, (region, label) in zip(axes.flat, specifications):
                for offset, side, color in ((-.18, 'own', '#3568b0'), (.18, 'reference', '#db873d')):
                    data = case[side]['minimum_angle_degrees'] if region == 'angles' else case[side]['unique_edge_lengths'][region]['relative_to_size_max']
                    counts = np.asarray(data['histogram_counts'])
                    fractions = counts/data['count'] if data['count'] else counts
                    ax.bar(np.arange(len(counts))+offset, fractions, width=.35, label=side, color=color)
                limits = data['histogram_upper_exclusive']
                labels = [f'<{limits[0]:g}']+[f'{a:g}–<{b:g}' for a, b in zip(limits, limits[1:])]+[f'≥{limits[-1]:g}']
                ax.set_xticks(np.arange(len(labels)), labels, rotation=35, fontsize=8)
                ax.set_title(label)
                ax.set_xlabel('Degrees' if region == 'angles' else 'Length / configured sizeMax')
                ax.set_ylabel('Fraction'); ax.legend(); ax.set_ylim(bottom=0)
            figure.suptitle(f"{case['case']} | extension={case['extension']} | h={case['size_max']}\nIndependent final mesh distributions; no additional gates")
            figure.savefig(args.directory/f"{case['case']}-extend{case['extension']}"/'mesh-distributions.png', dpi=150)
            plt.close(figure)
    print(f"Measured {sum(case['measured'] for case in cases)}/{len(cases)} final mesh pairs")


if __name__ == '__main__':
    main()
