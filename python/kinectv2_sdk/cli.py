"""Command-line quickstart entry point (`kinectv2-demo`)."""

from __future__ import annotations

import argparse

from .sensor import JointType, KV2_STREAM_ALL, KinectV2Sensor, TrackingState


def run_demo() -> None:
    parser = argparse.ArgumentParser(description="Stream 30 FPS multi-modal frames and 3D skeletons from Kinect v2")
    parser.add_argument("--frames", type=int, default=90, help="Number of frames to read (0 = until Ctrl+C)")
    args = parser.parse_args()

    with KinectV2Sensor(stream_flags=KV2_STREAM_ALL) as sensor:
        ir_intr, col_intr = sensor.get_intrinsics()
        print(
            f"Connected: serial={sensor.serial}, pipeline={sensor.pipeline_name}, "
            f"IR=({ir_intr.fx:.1f},{ir_intr.fy:.1f}), Color=({col_intr.fx:.1f},{col_intr.fy:.1f})"
        )
        count = 0
        try:
            while args.frames <= 0 or count < args.frames:
                frame = sensor.wait_frame(timeout_ms=1000)
                if frame is None:
                    continue
                with frame:
                    count += 1
                    if frame.bodies:
                        b0 = frame.bodies[0]
                        head = b0.joint_positions[JointType.HEAD]
                        head_px = b0.joint_color_uv[JointType.HEAD]
                        tracked = int((b0.joint_tracking_states == TrackingState.TRACKED).sum())
                        print(
                            f"[#{frame.frame_seq:05d}] {frame.sensor_fps:4.1f} FPS | infer={frame.skeleton_infer_ms:4.2f} ms | "
                            f"Bodies={len(frame.bodies)} ({tracked}/25 tracked) | "
                            f"Head=({head[0]:+.2f},{head[1]:+.2f},{head[2]:.2f}m) -> RGB=({head_px[0]:.0f},{head_px[1]:.0f})"
                        )
                    elif count == 1 or count % 30 == 0:
                        host_mb, worker_mb = sensor.get_memory_rss_mb()
                        print(
                            f"[#{frame.frame_seq:05d}] {frame.sensor_fps:4.1f} FPS | infer={frame.skeleton_infer_ms:4.2f} ms | "
                            f"RAM={host_mb + worker_mb:.0f} MB | RGB={frame.color_bgrx.shape} Depth={frame.depth_mm.shape}"
                        )
        except KeyboardInterrupt:
            pass
