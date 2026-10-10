"""冻结 BREP 上的 0D/1D 最终节点身份、参数、坐标和有向线段对照。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from GmshReproductionComparison import first_trace_difference


def boundary_events(document):
    nodes = sorted(document["nodes"], key=lambda row: row[0])
    # JSON 的 0/0.0 拼写不同不构成坐标差异，节点和 CAD 身份仍为整数。
    nodes = [[row[0], *map(float, row[1:4]), row[4], row[5], list(map(float, row[6]))] for row in nodes]
    return [{"stage": "boundary_nodes", "entity": -1, "nodes": nodes},
            {"stage": "boundary_elements", "entity": -1,
             "elements": sorted(document["elements"], key=lambda row: row[0])}]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", required=True)
    parser.add_argument("--library", required=True)
    parser.add_argument("--own-executable", required=True)
    parser.add_argument("--frozen-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    out = Path(args.out_dir); out.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).with_name("GmshReproductionComparison.py")
    own_environment = dict(os.environ)
    own_environment["PATH"] = os.pathsep.join(args.runtime_dir + [own_environment.get("PATH", "")])
    report = {"scope": "OCC serial first-order algorithm-6 boundary 0D/1D, constant size .3",
              "coordinate_parameter_absolute_tolerance": 1e-12, "cases": [], "complete": False,
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        for name in ("line", "bezier", "rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded"):
            cad = Path(args.frozen_dir) / (name + ".brep")
            expected = out / (name + "-reference.json")
            command = [sys.executable, "-B", str(script), "run", "--api", args.api,
                       "--library", args.library, "--cad", str(cad), "--fixture", name,
                       "--dimension", "1", "--out", str(expected)]
            for runtime in args.runtime_dir: command.extend(["--runtime-dir", runtime])
            subprocess.run(command, check=True)
            reference = json.loads(expected.read_text(encoding="utf-8"))
            actual = out / (name + "-own.json")
            subprocess.run([args.own_executable, str(cad), ".3",
                            str(reference["metadata"]["model_characteristic_length"]), str(actual)],
                           env=own_environment, check=True)
            own = json.loads(actual.read_text(encoding="utf-8"))
            found = first_trace_difference(boundary_events(reference), boundary_events(own), 1e-12)
            report["cases"].append({"fixture": name, "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                    "reference_library_sha256": reference["metadata"]["library_sha256"],
                                    "nodes": [len(reference["nodes"]), len(own["nodes"])],
                                    "elements": [len(reference["elements"]), len(own["elements"])],
                                    "first_difference": found})
            if found: raise ValueError("boundary divergence: " + name + " " + json.dumps(found))
        report["complete"] = True
    finally:
        (out / "boundary-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": len(report["cases"]), "equivalent": report["complete"]}))


if __name__ == "__main__":
    main()
