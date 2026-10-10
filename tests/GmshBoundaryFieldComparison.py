"""变化尺寸下比较生成期原生边链；最终 Renumber 是独立组装阶段，不混同身份。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from GmshReproductionComparison import first_trace_difference
from GmshBoundaryComparison import boundary_events


def native_full_save_numbering(document):
    """测试端 GModel::renumberMeshVertices 的 full-save、无显式 mapping 分支。

    顺序为实体维数/tag 和实体内节点顺序；不改变生成期身份。
    仅解释当前阶段的输出编号差异，生产组装模块尚未实现此步骤。
    """
    nodes = sorted(document["nodes"], key=lambda row: (row[4], row[5], row[0]))
    mapping = {row[0]: index + 1 for index, row in enumerate(nodes)}
    return {"nodes": [[mapping[row[0]], *row[1:]] for row in nodes],
            "elements": [[row[0], *row[1:4], [mapping[node] for node in row[4]]]
                         for row in document["elements"]]}


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
    environment = dict(os.environ)
    environment["PATH"] = os.pathsep.join(args.runtime_dir + [environment.get("PATH", "")])
    script = Path(__file__).with_name("GmshReproductionComparison.py")
    report = {"scope": "generation-time physical boundary chains, varying callback sizes, Mesh.Renumber default",
              "absolute_tolerance": 1e-12, "cases": [], "complete": False,
              "own_executable_sha256": hashlib.sha256(Path(args.own_executable).read_bytes()).hexdigest()}
    try:
        fixtures = ("line", "bezier", "rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded")
        for field, names in (("endpoint", fixtures), ("graded", fixtures), ("step", ("line", "bezier"))):
            for name in names:
                label = name + "-" + field
                cad = Path(args.frozen_dir) / (name + ".brep")
                reference_path = out / (label + "-reference.json")
                trace_path = out / (label + "-reference.jsonl")
                command = [sys.executable, "-B", str(script), "run", "--api", args.api, "--library", args.library,
                           "--cad", str(cad), "--fixture", name, "--dimension", "1", "--size", "10",
                           "--size-field", field, "--out", str(reference_path), "--trace", str(trace_path)]
                for runtime in args.runtime_dir: command.extend(["--runtime-dir", runtime])
                subprocess.run(command, check=True)
                reference = json.loads(reference_path.read_text(encoding="utf-8"))
                own_path = out / (label + "-own.json")
                subprocess.run([args.own_executable, str(cad), "10",
                                str(reference["metadata"]["model_characteristic_length"]), str(own_path), "--field=" + field],
                               env=environment, check=True)
                own = json.loads(own_path.read_text(encoding="utf-8"))
                events = [json.loads(line) for line in trace_path.read_text(encoding="utf-8").splitlines() if line]
                expected = [{key: value for key, value in event.items() if key not in ("schema", "sequence")}
                            for event in events if event["stage"] == "boundary_chain"]
                actual = own["boundary_chains"]
                for event in actual:
                    for row in event["rows"]:
                        for k in (1, 2, 3, 4, 7):
                            if isinstance(row[k], (int, float)): row[k] = float(row[k])
                found = first_trace_difference(sorted(expected, key=lambda e: e["entity"]),
                                               sorted(actual, key=lambda e: e["entity"]), 1e-12)
                final_difference = first_trace_difference(boundary_events(reference), boundary_events(own), 1e-12)
                renumbered_difference = first_trace_difference(boundary_events(reference),
                                                               boundary_events(native_full_save_numbering(own)), 1e-12)
                allocated = sum(event["stage"] == "curve_node_creation" for event in events) + sum(row[4] == 0 for row in reference["nodes"])
                report["cases"].append({"fixture": name, "field": field,
                                        "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
                                        "reference_library_sha256": reference["metadata"]["library_sha256"],
                                        "allocated_nodes": [allocated, own["allocated_node_count"]],
                                        "erased_nodes": own["erased_node_count"], "first_generation_difference": found,
                                        "final_output_difference": final_difference,
                                        "after_test_only_native_renumber_difference": renumbered_difference})
                if found or renumbered_difference or allocated != own["allocated_node_count"]:
                    raise ValueError("generation-time boundary divergence: " + label + " " + json.dumps(found))
        report["complete"] = True
    finally:
        (out / "boundary-field-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": len(report["cases"]), "generation_equivalent": report["complete"],
                      "cases_with_erased_nodes": sum(case["erased_nodes"] > 0 for case in report["cases"]),
                      "final_output_differences": sum(case["final_output_difference"] is not None for case in report["cases"])}))


if __name__ == "__main__":
    main()
