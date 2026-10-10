"""Three serial trials of currently qualified Algorithm 6 cases; no stage observer.

Generation, CAD import, independent audit and VTP export are recorded separately.
Every trial is audited again; a faster but unqualified mesh cannot pass this gate.
"""
import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
from GmshSurfacePipelineComparison import digest, evaluate_shape_gates


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def run(command, folder, name, env=None):
    result = subprocess.run(command, env=env, capture_output=True, text=True)
    (folder / (name + ".log")).write_text(result.stdout + result.stderr, encoding="utf-8")
    return result.returncode


def main(args):
    baseline = read(args.comparison / "comparison.json")
    if baseline["own_executable_sha256"] != digest(args.executable):
        raise ValueError("qualification used another executable; rerun the quality comparison")
    reference_script = Path(__file__).with_name("GmshSurfacePipelineComparison.py")
    parameter_script = Path(__file__).with_name("GmshReferenceParameters.py")
    if baseline["script_sha256"] != digest(reference_script) or baseline.get("reference_parameter_script_sha256") != digest(parameter_script):
        raise ValueError("qualification used another reference exporter; rerun the quality comparison")
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ)
    env["PATH"] = os.pathsep.join(args.runtime + [os.environ.get("SystemRoot", "C:/Windows") + "/System32"])
    env["PYTHONPATH"] = ""
    report = dict(complete=False, trials=3, serial=True, stages=False, cases=[],
                  executable_sha256=digest(args.executable), library_sha256=digest(args.library),
                  qualification_sha256=digest(args.comparison / "comparison.json"), script_sha256=digest(__file__),
                  reference_script_sha256=digest(reference_script), reference_parameter_script_sha256=digest(parameter_script),
                  skipped=[dict(case=c["case"], extension=c["extension"], reason="generation/audit/shape not qualified")
                           for c in baseline["cases"] if not c["complete"] or not c["shape_passed"]])
    for case in baseline["cases"]:
        if not case["complete"] or not case["shape_passed"]:
            continue
        original = args.comparison / f"{case['case']}-extend{case['extension']}"
        cad = original / "input/model.brep"
        if digest(cad) != case["cad_sha256"] or read(original / "reference-generation.json")["library_sha256"] != digest(args.library):
            raise ValueError("qualified CAD or reference library changed")
        row = dict(case=case["case"], extension=case["extension"], size=case["size"], cad_sha256=digest(cad), rounds=[])
        point_size, curvature = baseline.get("uniform_point_size", 0), baseline.get("curvature_circle_elements", 0)
        for trial in range(3):
            folder = args.output / f"{case['case']}-extend{case['extension']}-trial{trial+1}"
            folder.mkdir()
            own = folder / "own"
            command = [str(args.executable), "generate", str(cad), str(case["size"]), str(case["extension"]), str(own),
                       "--no-stages", "--point-size", str(point_size), "--curvature", str(curvature)]
            ours = run(command, folder, "own", env)
            command = [sys.executable, str(Path(__file__).with_name("GmshSurfacePipelineComparison.py")), "--reference",
                       "--api", str(args.api), "--library", str(args.library), "--cad", str(cad), "--size", str(case["size"]),
                       "--extension", str(case["extension"]), "--point-size", str(point_size), "--curvature", str(curvature), "--output", str(folder)]
            theirs = run(command, folder, "reference")
            audited = run([str(args.executable), "audit", str(cad), str(folder / "reference.vtp"), str(folder / "reference-audit.json")], folder, "reference-audit", env) if theirs == 0 else None
            round_report = dict(trial=trial+1, own_exit=ours, reference_exit=theirs, reference_audit_exit=audited, quality_passed=False)
            if ours == 0 and theirs == 0 and audited == 0:
                gates, passed = evaluate_shape_gates(read(own / "audit.json"), read(folder / "reference-audit.json"))
                reference = read(folder / "reference-generation.json")
                round_report.update(quality_passed=passed, gates=gates, own=read(own / "timing.json"),
                                    reference={k: reference[k] for k in ("import_seconds", "generation_seconds", "export_seconds")})
                round_report["reference"]["audit_seconds"] = read(folder / "reference-audit-timing.json")["audit_seconds"]
            row["rounds"].append(round_report)
        row["quality_passed"] = all(r["quality_passed"] for r in row["rounds"])
        if row["quality_passed"]:
            row["medians"] = {side: {phase: statistics.median(r[side][phase] for r in row["rounds"])
                                       for phase in ("import_seconds", "generation_seconds", "audit_seconds", "export_seconds")}
                              for side in ("own", "reference")}
            row["generation_ratio"] = row["medians"]["own"]["generation_seconds"] / row["medians"]["reference"]["generation_seconds"]
            row["speed_passed"] = row["generation_ratio"] <= 2
        else:
            row["speed_passed"] = False
        report["cases"].append(row)
        (args.output / "performance.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(case["case"], case["extension"], "quality", row["quality_passed"], "generation ratio", row.get("generation_ratio"), flush=True)
    report["complete"] = True
    report["speed_passed"] = bool(report["cases"]) and all(c["speed_passed"] for c in report["cases"])
    (args.output / "performance.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if report["speed_passed"] else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ("comparison", "executable", "api", "library", "output"):
        parser.add_argument("--" + flag, type=Path, required=True)
    parser.add_argument("--runtime", action="append", default=[])
    sys.exit(main(parser.parse_args()))
