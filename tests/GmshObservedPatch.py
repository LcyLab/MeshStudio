"""生成/真实重放固定 Gmsh 参考补丁；只操作明确的观察副本和新重放目录。"""
import argparse
import difflib
import hashlib
import json
import re
from pathlib import Path
import subprocess
import tarfile

ARCHIVE_SHA256 = "be3f66f225d27ba9fa014f07e83169285da8a051b0e8ab7103d88066b39bdd3e"
FILES = tuple("src/mesh/" + name for name in (
    "meshGEdge.cpp", "BackgroundMeshTools.cpp", "meshTriangulation.cpp", "meshGFaceBDS.cpp",
    "meshGFaceDelaunayInsertion.cpp", "meshGFaceOptimize.cpp", "meshGFace.cpp", "DivideAndConquer.cpp", "BDS.cpp")) + ("src/geo/MVertex.cpp", "src/geo/GFace.cpp", "src/mesh/MeshStudioTrace.h")


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    generate = commands.add_parser("generate")
    generate.add_argument("--archive", required=True)
    generate.add_argument("--pristine", required=True)
    generate.add_argument("--observed", required=True)
    generate.add_argument("--patch", required=True)
    generate.add_argument("--manifest", required=True)
    verify = commands.add_parser("verify")
    verify.add_argument("--pristine", required=True)
    verify.add_argument("--patch", required=True)
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--replay-dir", required=True)
    args = parser.parse_args()
    pristine = Path(args.pristine).resolve()
    patch_path = Path(args.patch).resolve()
    manifest_path = Path(args.manifest).resolve()
    if args.command == "generate":
        archive_path = Path(args.archive)
        if sha(archive_path.read_bytes()) != ARCHIVE_SHA256:
            raise ValueError("fixed source archive SHA256 mismatch")
        observed = Path(args.observed).resolve()
        if observed == pristine: raise ValueError("observed must be a separate source copy")
        files, chunks, oracle_entries = {}, [], set()
        with tarfile.open(archive_path, "r:gz") as archive:
            members = archive.getmembers()
            for name in FILES:
                before = b""
                if name != FILES[-1]:
                    before = (pristine / name).read_bytes()
                    matches = [member for member in members if member.name.endswith("/" + name)]
                    if len(matches) != 1 or archive.extractfile(matches[0]).read() != before:
                        raise ValueError("pristine source differs from archive: " + name)
                after = (observed / name).read_bytes()
                oracle_entries.update(re.findall(rb"\b(meshstudioReference\w+)\s*\(", after))
                if b"\r\n" in after: raise ValueError("observed source must use LF: " + name)
                if name == FILES[-1] and not after.startswith(b"\xef\xbb\xbf"):
                    raise ValueError("external observer header requires UTF-8 BOM")
                files[name] = {"observed_sha256": sha(after)}
                if before: files[name]["pristine_sha256"] = sha(before)
                chunks.extend(difflib.unified_diff(before.decode("utf-8").replace("\r\n", "\n").splitlines(True),
                                                   after.decode("utf-8").splitlines(True),
                                                   fromfile="a/" + name if before else "/dev/null",
                                                   tofile="b/" + name))
        patch = "".join(chunks).encode("utf-8")
        manifest = {"version": "4.15.2", "archive_sha256": ARCHIVE_SHA256, "patch_sha256": sha(patch),
                    "files": files, "header_encoding": "UTF-8 with BOM in external reference copy",
                    "patch_replay_EOL": "Normalize all patched replay files to LF after Git apply; preserve the header BOM.",
                    "oracle_entry_points": sorted(entry.decode("ascii") for entry in oracle_entries)}
        patch_path.write_bytes(patch)
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"patch_bytes": len(patch), "audited_pristine_files": len(FILES)-1, "observed_files": len(FILES)}))
    else:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if set(manifest["files"]) != set(FILES) or sha(patch_path.read_bytes()) != manifest["patch_sha256"]:
            raise ValueError("patch or manifest mismatch")
        replay = Path(args.replay_dir).resolve()
        if replay == pristine or replay.exists(): raise ValueError("replay requires a new distinct directory")
        replay.mkdir(parents=True)
        for name in FILES[:-1]:
            original = (pristine / name).read_bytes()
            if sha(original) != manifest["files"][name]["pristine_sha256"]:
                raise ValueError("pristine SHA256 mismatch: " + name)
            target = replay / name; target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(original.replace(b"\r\n", b"\n"))
        subprocess.run(["git", "apply", str(patch_path)], cwd=replay, check=True)
        for name in FILES:
            target = replay / name
            normalized = target.read_bytes().replace(b"\r\n", b"\n")
            target.write_bytes(normalized)
            if sha(normalized) != manifest["files"][name]["observed_sha256"]:
                raise ValueError("replayed SHA256 mismatch: " + name)
        print(json.dumps({"actually_applied": True, "identical_files": len(FILES)}))


if __name__ == "__main__":
    main()
