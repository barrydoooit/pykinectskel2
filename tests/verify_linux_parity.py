"""
Cross-platform offline parity verifier against Windows Kinect v2 SDK ground truth (`golden_fixture_v1.npz`).

Run on Linux before waking/connecting the physical Kinect v2 sensor:
    python3 tests/verify_linux_parity.py \
        --bundle ./dist/kinectv2_linux_bundle \
        --wineprefix ~/.wine-kinectv2
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from kinectv2_sdk import JOINT_NAMES, KinectV2BodyTracker


def quat_max_angle_deg(q_est: np.ndarray, q_ref: np.ndarray) -> float:
    """Max angular error in degrees across non-zero quaternions (handling q and -q equivalence in float64)."""
    q_e = q_est.astype(np.float64)
    q_r = q_ref.astype(np.float64)
    norms_ref = np.linalg.norm(q_r, axis=1)
    valid = norms_ref > 0.5
    if not np.any(valid):
        return 0.0
    a = q_e[valid] / np.maximum(np.linalg.norm(q_e[valid], axis=1, keepdims=True), 1e-12)
    b = q_r[valid] / norms_ref[valid, None]
    signs = np.where(np.sum(a * b, axis=1, keepdims=True) < 0.0, -1.0, 1.0)
    b = b * signs
    angles_rad = 2.0 * np.arctan2(np.linalg.norm(a - b, axis=1), np.linalg.norm(a + b, axis=1))
    return float(np.degrees(np.max(angles_rad)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify offline Kinect v2 3D skeleton parity on Linux/Windows")
    parser.add_argument(
        "--bundle",
        type=Path,
        default=REPO_ROOT / "dist" / "kinectv2_linux_bundle",
        help="Path to staged kinectv2_linux_bundle directory",
    )
    parser.add_argument(
        "--fixture",
        type=Path,
        default=REPO_ROOT / "tests" / "golden_fixture_v1.npz",
        help="Path to golden_fixture_v1.npz generated on Windows",
    )
    parser.add_argument("--wineprefix", type=str, default=None, help="Optional WINEPREFIX on Linux")
    parser.add_argument("--wine-binary", type=str, default="wine64", help="Wine binary on Linux (default: wine64)")
    parser.add_argument("--transport", choices=("stdio", "tcp"), default="stdio", help="IPC transport mode")
    parser.add_argument("--warp", action="store_true", help="Request D3D11 WARP software rasterizer")
    parser.add_argument(
        "--pos-tol-mm",
        type=float,
        default=2.0,
        help="Max allowed 3D joint Euclidean discrepancy in millimeters (default: 2.0 mm)",
    )
    args = parser.parse_args()

    data = np.load(args.fixture)
    depth_seq = data["depth_sequence_mm"]
    ir_frame = data["ir_frame"]
    ref_counts = data["ref_default_body_counts"]
    ref_pos = data["ref_default_joint_positions"]
    ref_ori = data["ref_default_joint_orientations"]
    ref_st = data["ref_default_joint_states"]
    ref_floor = data["ref_default_floor_clip_plane"]
    ref_bidx = data["ref_default_body_index"]

    intr = data["custom_intrinsics"]
    ref_c_pos = data["ref_custom_joint_positions"]
    ref_c_ori = data["ref_custom_joint_orientations"]
    ref_c_st = data["ref_custom_joint_states"]
    ref_c_floor = data["ref_custom_floor_clip_plane"]
    ref_c_bidx = data["ref_custom_body_index"]

    worker_exe = args.bundle / "vvtechs_worker.exe"
    print(f"[1/3] Launching worker: {worker_exe} (platform={sys.platform}, transport={args.transport})")

    latencies_ms = []
    with KinectV2BodyTracker(
        worker_exe=worker_exe,
        dll_dir=args.bundle,
        wine_binary=args.wine_binary,
        wineprefix=args.wineprefix,
        use_warp=args.warp,
        transport=args.transport,
    ) as tracker:
        print("[2/3] Scenario A: Default Kinect v2 intrinsics (32 frames, 15-frame warmup check)...")
        got_counts = []
        last_res = None
        for idx in range(depth_seq.shape[0]):
            t0 = time.perf_counter()
            last_res = tracker.process_frame(
                depth_seq[idx],
                want_body_index=(idx == depth_seq.shape[0] - 1),
            )
            dt = (time.perf_counter() - t0) * 1000.0
            if idx >= 5:
                latencies_ms.append(dt)
            got_counts.append(len(last_res.bodies))

        if last_res is None or len(last_res.bodies) != 1:
            print(f"FAIL: Expected 1 tracked body on frame 32, got {0 if last_res is None else len(last_res.bodies)}")
            return 1

        b0 = last_res.bodies[0]
        pos_err_mm = float(np.max(np.linalg.norm(b0.joint_positions - ref_pos, axis=1)) * 1000.0)
        pos_linf_m = float(np.max(np.abs(b0.joint_positions - ref_pos)))
        ori_linf = float(np.max(np.abs(b0.joint_orientations - ref_ori)))
        ori_deg = quat_max_angle_deg(b0.joint_orientations, ref_ori)
        floor_linf = float(np.max(np.abs(last_res.floor_clip_plane - ref_floor)))
        states_equal = bool(np.array_equal(b0.joint_tracking_states, ref_st))
        warmup_zero = all(c == 0 for c in got_counts[:15])
        counts_equal = bool(np.array_equal(np.array(got_counts, dtype=np.int32), ref_counts))

        assert last_res.body_index_map is not None
        fg_got = last_res.body_index_map < 6
        fg_ref = ref_bidx < 6
        fg_iou = float(np.sum(fg_got & fg_ref)) / float(max(np.sum(fg_got | fg_ref), 1))
        bidx_exact_ratio = float(np.mean(last_res.body_index_map == ref_bidx))

        mean_ms = float(np.mean(latencies_ms))
        print(f"  Mean frame latency (IPC + GPU):          {mean_ms:.2f} ms ({1000.0 / max(mean_ms, 1e-6):.0f} FPS)")
        print(f"  Warmup gate (frames 1..15 == 0 bodies):  {warmup_zero} (all 32 counts match={counts_equal})")
        print(f"  Max 3D joint Euclidean error:            {pos_err_mm:.6f} mm (L-inf={pos_linf_m:.8f} m)")
        print(f"  Max joint quaternion angular error:      {ori_deg:.6f} deg (L-inf={ori_linf:.8f})")
        print(f"  Max FloorClipPlane L-inf diff:           {floor_linf:.8f}")
        print(f"  Joint TrackingStates match (25/25):      {states_equal} (tracked={int(np.sum(b0.joint_tracking_states == 2))}/25)")
        print(f"  BodyIndex foreground IoU / exact match:  IoU={fg_iou:.6f}, exact={bidx_exact_ratio * 100.0:.4f}%")

        print("[3/3] Scenario B: Custom libfreenect2 IrCameraParams + 512x424 IR frame...")
        tracker.reset()
        tracker.set_intrinsics(*[float(x) for x in intr])
        last_c = None
        for idx in range(depth_seq.shape[0]):
            last_c = tracker.process_frame(
                depth_seq[idx],
                ir_frame=ir_frame,
                want_body_index=(idx == depth_seq.shape[0] - 1),
            )
        if last_c is None or len(last_c.bodies) != 1:
            print(f"FAIL: Scenario B expected 1 tracked body, got {0 if last_c is None else len(last_c.bodies)}")
            return 1

        bc = last_c.bodies[0]
        c_pos_err_mm = float(np.max(np.linalg.norm(bc.joint_positions - ref_c_pos, axis=1)) * 1000.0)
        c_ori_deg = quat_max_angle_deg(bc.joint_orientations, ref_c_ori)
        c_states_equal = bool(np.array_equal(bc.joint_tracking_states, ref_c_st))
        assert last_c.body_index_map is not None
        c_fg_iou = float(np.sum((last_c.body_index_map < 6) & (ref_c_bidx < 6))) / float(
            max(np.sum((last_c.body_index_map < 6) | (ref_c_bidx < 6)), 1)
        )
        print(f"  Scenario B max 3D joint error:           {c_pos_err_mm:.6f} mm")
        print(f"  Scenario B max quaternion error:         {c_ori_deg:.6f} deg")
        print(f"  Scenario B TrackingStates match:         {c_states_equal}")
        print(f"  Scenario B BodyIndex foreground IoU:     {c_fg_iou:.6f}")

    ok = (
        warmup_zero
        and pos_err_mm <= args.pos_tol_mm
        and ori_deg <= 0.5
        and states_equal
        and fg_iou >= 0.995
        and c_pos_err_mm <= args.pos_tol_mm
        and c_ori_deg <= 0.5
        and c_states_equal
        and c_fg_iou >= 0.995
    )
    if not ok:
        print("RESULT: PARITY CHECK FAILED (exceeded tolerance).")
        return 1

    print("RESULT: PARITY CHECK PASSED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
