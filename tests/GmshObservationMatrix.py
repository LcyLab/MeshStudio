"""复核参考只读钩子：同一冻结 BREP，原版/观察关闭/观察开启逐个独立进程比较。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from GmshReproductionComparison import mesh_fingerprint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--api", required=True)
    parser.add_argument("--unobserved-library", required=True)
    parser.add_argument("--observed-library", required=True)
    parser.add_argument("--frozen-dir", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--runtime-dir", action="append", default=[])
    args = parser.parse_args()
    frozen = Path(args.frozen_dir)
    names = ("line", "bezier", "rectangle", "hole", "narrow_band", "cylinder", "sphere", "embedded")
    inputs = {name: frozen / (name + ".brep") for name in names}
    hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in inputs.items()}
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=False)
    script = Path(__file__).with_name("GmshReproductionComparison.py")
    report = {"fixture_script_sha256": hashlib.sha256(script.read_bytes()).hexdigest(),
              "cad_sha256": hashes, "cases": [], "complete": False}
    try:
        for name in names:
            documents = []
            for label, library, trace in (("unobserved", args.unobserved_library, False),
                                           ("observer-off", args.observed_library, False),
                                           ("observer-on", args.observed_library, True)):
                output = out / (name + "-" + label + ".json")
                command = [sys.executable, "-B", str(script), "run", "--api", args.api,
                           "--library", library, "--fixture", name, "--cad", str(inputs[name]),
                           "--dimension", "1" if name in ("line", "bezier") else "2", "--out", str(output)]
                for runtime in args.runtime_dir:
                    command.extend(["--runtime-dir", runtime])
                if name == "embedded":
                    command.append("--restore-fixture-embedding")
                if trace:
                    command.extend(["--trace", str(out / (name + ".jsonl"))])
                subprocess.run(command, check=True)
                documents.append(json.loads(output.read_text(encoding="utf-8")))
            fingerprints = [mesh_fingerprint(document) for document in documents]
            same = len(set(fingerprints)) == 1
            report["cases"].append({"fixture": name, "identical": same, "fingerprints": fingerprints})
            if not same:
                raise ValueError("reference observer changed mesh: " + name)
        report["complete"] = True
    finally:
        (out / "observation-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"cases": len(report["cases"]), "identical": report["complete"]}))


if __name__ == "__main__":
    main()
