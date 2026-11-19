"""
End-to-end verification script for Windows x64 (and Linux x86_64) `pykinectskel2` runtime:
  1. Verifies C ABI DLL symbol exports and ctypes struct layouts (`kinectv2_sdk.dll` / `libkinectv2_sdk.so`).
  2. Verifies offline 25-joint 3D skeleton numerical parity (`golden_fixture_v1.npz` < 2.0 mm).
  3. Opens live USB Kinect v2 via `pykinectskel2.KinectV2Sensor` (without Windows Kinect SDK service),
     streams multi-modal RGB + Depth + IR + Registered RGB-D + BodyIndex + 25-joint 3D skeletons,
     checks camera-to-color / camera-to-depth projection, closes cleanly, and confirms zero orphan
     `vvtechs_worker.exe` child processes remain.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

import pykinectskel2 as pks2
from kinectv2_sdk.sensor import _CKv2Body, _CKv2Config, _CKv2FrameView, _CKv2Joint, _load_sdk_lib, _resolve_lib_path


def default_runtime_dir() -> Optional[Path]:
    if sys.platform == "win32":
        la = os.environ.get("LOCALAPPDATA")
        if la:
            cand = Path(la) / "kinectv2-runtime"
            if cand.exists():
                return cand
    else:
        cand = Path.home() / ".local" / "share" / "kinectv2-runtime"
        if cand.exists():
            return cand
    return None


def resolve_bundle_dir(runtime_dir: Optional[Path]) -> Path:
    candidates = []
    if runtime_dir is not None:
        candidates.append(runtime_dir / "dist" / "kinectv2_windows_bundle")
        candidates.append(runtime_dir / "dist" / "kinectv2_linux_bundle")
    candidates.append(REPO_ROOT / "dist" / "kinectv2_windows_bundle")
    candidates.append(REPO_ROOT / "dist" / "kinectv2_linux_bundle")
    for c in candidates:
        if (c / "vvtechs_worker.exe").is_file():
            return c.resolve()
    return candidates[0]


def count_running_workers() -> int:
    if sys.platform == "win32":
        proc = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq vvtechs_worker.exe", "/NH"],
            capture_output=True,
            text=True,
            check=False,
        )
        return sum(
            1 for line in proc.stdout.splitlines() if "vvtechs_worker.exe" in line.lower()
        )
    proc = subprocess.run(
        ["pgrep", "-f", "vvtechs_worker.exe"],
        capture_output=True,
        text=True,
        check=False,
    )
    return len([ln for ln in proc.stdout.splitlines() if ln.strip()])


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify pykinectskel2 DLL/SO + offline parity + live USB sensor")
    parser.add_argument("--runtime-dir", type=Path, default=default_runtime_dir())
    parser.add_argument("--lib-path", type=Path, default=None)
    parser.add_argument("--bundle", type=Path, default=None)
    parser.add_argument("--duration", type=float, default=5.0, help="Live USB streaming test duration in seconds")
    parser.add_argument("--offline-only", action="store_true", help="Skip live USB Kinect v2 sensor streaming test")
    args = parser.parse_args()

    rt_dir = args.runtime_dir.resolve() if args.runtime_dir else None
    lib_path = args.lib_path
    if lib_path is None and rt_dir is not None:
        win_dll = rt_dir / "bin" / "kinectv2_sdk.dll"
        lin_so = rt_dir / "lib" / "libkinectv2_sdk.so"
        if sys.platform == "win32" and win_dll.is_file():
            lib_path = win_dll
        elif sys.platform != "win32" and lin_so.is_file():
            lib_path = lin_so

    bundle_dir = args.bundle.resolve() if args.bundle else resolve_bundle_dir(rt_dir)
    worker_exe = bundle_dir / "vvtechs_worker.exe"

    resolved_lib = _resolve_lib_path(lib_path)
    print(f"[1/3] Checking C ABI shared library: {resolved_lib}")
    assert ctypes.sizeof(_CKv2Joint) == 48
    assert ctypes.sizeof(_CKv2Body) == 1216
    assert ctypes.sizeof(_CKv2FrameView) == 7424
    lib = _load_sdk_lib(lib_path, dll_dir=bundle_dir)
    cfg = _CKv2Config()
    lib.kv2_default_config(ctypes.byref(cfg))
    assert cfg.stream_flags == pks2.KV2_STREAM_DEFAULT
    lib.kv2_joint_name.argtypes = [ctypes.c_int]
    lib.kv2_joint_name.restype = ctypes.c_char_p
    for idx, expected in enumerate(pks2.JOINT_NAMES):
        actual = lib.kv2_joint_name(idx).decode("utf-8")
        assert actual == expected
    print("      C ABI & struct layout check PASSED.")

    print(f"[2/3] Running offline 25-joint parity test via {worker_exe}...")
    parity_cmd = [
        sys.executable,
        str(REPO_ROOT / "tests" / "verify_linux_parity.py"),
        "--bundle",
        str(bundle_dir),
    ]
    if sys.platform != "win32":
        wp = Path.home() / ".wine-kinectv2"
        if wp.is_dir():
            parity_cmd.extend(["--wineprefix", str(wp)])
    rc = subprocess.call(parity_cmd)
    if rc != 0:
        print("FAIL: Offline parity check failed.")
        return rc

    if args.offline_only:
        print("[3/3] Skipped live USB sensor check (--offline-only).")
        return 0

    baseline_workers = count_running_workers()
    print(f"[3/3] Opening live USB Kinect v2 via pykinectskel2.KinectV2Sensor ({args.duration:.1f}s)...")
    wine_bin = None if sys.platform == "win32" else None
    wine_pfx = None if sys.platform == "win32" else str(Path.home() / ".wine-kinectv2")

    n_frames = 0
    max_tracked_bodies = 0
    infer_ms_list = []
    last_sensor_fps = 0.0
    last_skel_fps = 0.0

    with pks2.KinectV2Sensor(
        serial=None,
        pipeline=pks2.KV2_PIPELINE_AUTO,
        stream_flags=pks2.KV2_STREAM_ALL,
        mirror_skeleton=False,
        worker_exe=worker_exe,
        dll_dir=bundle_dir,
        wine_binary=wine_bin,
        wineprefix=wine_pfx,
        lib_path=lib_path,
    ) as sensor:
        ir_intr, col_intr = sensor.get_intrinsics()
        print(
            f"      Opened serial={sensor.serial} pipeline={sensor.pipeline_name} "
            f"IR=({ir_intr.fx:.2f},{ir_intr.fy:.2f}) Color=({col_intr.fx:.2f},{col_intr.fy:.2f})"
        )
        u_d, v_d = sensor.project_camera_to_depth((0.0, 0.0, 2.0))
        u_c, v_c = sensor.project_camera_to_color((0.0, 0.0, 2.0))
        xyz = sensor.deproject_depth_to_camera(256, 212, 2000.0)
        assert 100.0 < u_d < 400.0 and 100.0 < v_d < 350.0
        assert 400.0 < u_c < 1500.0 and 200.0 < v_c < 900.0
        assert xyz is not None and abs(xyz[2] - 2.0) < 1e-4

        t_end = time.monotonic() + args.duration
        while time.monotonic() < t_end:
            frame = sensor.wait_frame(timeout_ms=1500)
            if frame is None:
                continue
            with frame:
                n_frames += 1
                rgb = frame.color_bgrx
                depth = frame.depth_mm
                ir = frame.ir
                reg = frame.registered_bgrx
                bidx = frame.body_index
                assert rgb is not None and rgb.shape == (1080, 1920, 4)
                assert depth is not None and depth.shape == (424, 512) and int(np.count_nonzero(depth)) > 1000
                assert ir is not None and ir.shape == (424, 512)
                assert reg is not None and reg.shape == (424, 512, 4)
                assert bidx is not None and bidx.shape == (424, 512)
                last_sensor_fps = frame.sensor_fps
                last_skel_fps = frame.skeleton_fps
                infer_ms_list.append(frame.skeleton_infer_ms)
                if len(frame.bodies) > max_tracked_bodies:
                    max_tracked_bodies = len(frame.bodies)
        host_mb, worker_mb = sensor.get_memory_rss_mb()

    time.sleep(0.3)
    remaining_workers = count_running_workers()
    mean_infer = float(np.mean(infer_ms_list)) if infer_ms_list else 0.0
    print(
        f"      Captured {n_frames} frames | Sensor FPS={last_sensor_fps:.1f} | "
        f"Skeleton FPS={last_skel_fps:.1f} | Mean infer={mean_infer:.2f} ms | "
        f"Max bodies={max_tracked_bodies} | RSS host={host_mb:.1f} MB worker={worker_mb:.1f} MB | "
        f"Workers remaining={remaining_workers} (baseline={baseline_workers})"
    )

    if n_frames < max(10, int(args.duration * 20)):
        print(f"FAIL: Expected >= {max(10, int(args.duration * 20))} frames in {args.duration:.1f}s, got {n_frames}")
        return 2
    if remaining_workers > baseline_workers:
        print(f"FAIL: Orphan vvtechs_worker.exe detected ({remaining_workers} > {baseline_workers})")
        return 3

    print("RESULT: LIVE USB KINECT V2 VERIFICATION PASSED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
