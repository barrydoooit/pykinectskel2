"""
Stages a self-contained Kinect v2 3D Body Tracking runtime folder ready to copy to Linux.

Usage on Windows (where Kinect for Windows SDK v2.0_1409 is installed):
    uv run python scripts/stage_linux_bundle.py --out ./dist/kinectv2_linux_bundle
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path


KINECT_SYS_DIR = Path(r"C:\Windows\System32\Kinect")
SYS32_DIR = Path(r"C:\Windows\System32")

REQUIRED_KINECT_FILES = [
    "vvtechs.dll",
    "exemplardb.xmplr",
    "FernsModel.bin",
    "HeadPosition.mshdf",
    "LSVMGripReleaseRight.gbd",
]

OPTIONAL_CRT_FILES = [
    "msvcr110.dll",
    "msvcp110.dll",
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def stage_bundle(repo_root: Path, out_dir: Path) -> dict:
    bin_dir = repo_root / "bin"
    worker_exe = bin_dir / "vvtechs_worker.exe"
    k4wcll_dll = bin_dir / "k4wcll.dll"
    if not worker_exe.is_file() or not k4wcll_dll.is_file():
        raise FileNotFoundError("Compile bin/vvtechs_worker.exe and bin/k4wcll.dll first.")

    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = {"files": {}}

    for src in (worker_exe, k4wcll_dll):
        dst = out_dir / src.name
        shutil.copy2(src, dst)
        manifest["files"][dst.name] = {"size": dst.stat().st_size, "sha256": sha256_file(dst)}

    for name in REQUIRED_KINECT_FILES:
        src = KINECT_SYS_DIR / name
        if not src.is_file():
            raise FileNotFoundError(f"Missing required Kinect v2 runtime file: {src}")
        dst = out_dir / name
        shutil.copy2(src, dst)
        manifest["files"][dst.name] = {"size": dst.stat().st_size, "sha256": sha256_file(dst)}

    for name in OPTIONAL_CRT_FILES:
        src = SYS32_DIR / name
        if src.is_file():
            dst = out_dir / name
            shutil.copy2(src, dst)
            manifest["files"][dst.name] = {"size": dst.stat().st_size, "sha256": sha256_file(dst)}

    manifest_path = out_dir / "bundle_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage Kinect v2 Linux Wine/DXVK bundle")
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "dist" / "kinectv2_linux_bundle",
    )
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    manifest = stage_bundle(repo_root, args.out)
    print(f"Staged {len(manifest['files'])} files into {args.out.resolve()}")
    for name, info in manifest["files"].items():
        print(f"  {name:28s} {info['size']:10d} bytes  sha256={info['sha256'][:16]}...")


if __name__ == "__main__":
    main()
