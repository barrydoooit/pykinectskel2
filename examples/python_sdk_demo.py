#!/usr/bin/env python3
"""Zero-copy Python quickstart example for `kinectv2_sdk` at locked 30.0 FPS."""

from __future__ import annotations

from kinectv2_sdk import JointType, KV2_STREAM_ALL, KinectV2Sensor, TrackingState


def main() -> None:
    with KinectV2Sensor(stream_flags=KV2_STREAM_ALL) as sensor:
        print(f"Connected to Kinect v2 (serial={sensor.serial}, pipeline={sensor.pipeline_name})")
        for i in range(90):
            frame = sensor.wait_frame(timeout_ms=1000)
            if frame is None:
                continue
            with frame:
                if frame.bodies:
                    b0 = frame.bodies[0]
                    head = b0.joint_positions[JointType.HEAD]
                    head_rgb_uv = b0.joint_color_uv[JointType.HEAD]
                    tracked = int((b0.joint_tracking_states == TrackingState.TRACKED).sum())
                    print(
                        f"[#{frame.frame_seq:05d}] {frame.sensor_fps:4.1f} FPS | infer={frame.skeleton_infer_ms:4.2f} ms | "
                        f"Bodies={len(frame.bodies)} ({tracked}/25 tracked) | "
                        f"Head 3D=({head[0]:+.2f},{head[1]:+.2f},{head[2]:.2f}m) -> RGB px=({head_rgb_uv[0]:.0f},{head_rgb_uv[1]:.0f})"
                    )
                elif i % 30 == 0:
                    print(
                        f"[#{frame.frame_seq:05d}] {frame.sensor_fps:4.1f} FPS | infer={frame.skeleton_infer_ms:4.2f} ms | "
                        f"RGB={frame.color_bgrx.shape} Depth={frame.depth_mm.shape} RegRGBD={frame.registered_bgrx.shape}"
                    )


if __name__ == "__main__":
    main()
