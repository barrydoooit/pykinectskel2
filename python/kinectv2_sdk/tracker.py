"""
Cross-platform Python SDK for Microsoft Kinect v2 3D Body Tracking (vvtechs_worker).

Runs identically on:
  - Linux x86_64: spawns `wine64 vvtechs_worker.exe --stdio --dll-dir <bundle_dir>`
    (requires Wine 8+ and DXVK installed in the Wine prefix for Vulkan Compute)
  - Windows x64: spawns `vvtechs_worker.exe --stdio --dll-dir <bundle_dir>` directly
"""

from __future__ import annotations

import os
import shutil
import socket
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

KV2_REQ_MAGIC = 0x4B563242
KV2_RESP_MAGIC = 0x4B563252

KV2_CMD_RESET = 1
KV2_CMD_SET_INTRINSICS = 2
KV2_CMD_PROCESS_FRAME = 3
KV2_CMD_QUIT = 9

KV2_FLAG_WANT_BODY_INDEX = 0x1
KV2_FLAG_HAS_IR_FRAME = 0x2

DEPTH_WIDTH = 512
DEPTH_HEIGHT = 424
DEPTH_PIXELS = DEPTH_WIDTH * DEPTH_HEIGHT
BODY_COUNT = 6
JOINT_COUNT = 25

JOINT_NAMES: Tuple[str, ...] = (
    "SpineBase",
    "SpineMid",
    "Neck",
    "Head",
    "ShoulderLeft",
    "ElbowLeft",
    "WristLeft",
    "HandLeft",
    "ShoulderRight",
    "ElbowRight",
    "WristRight",
    "HandRight",
    "HipLeft",
    "KneeLeft",
    "AnkleLeft",
    "FootLeft",
    "HipRight",
    "KneeRight",
    "AnkleRight",
    "FootRight",
    "SpineShoulder",
    "HandTipLeft",
    "ThumbLeft",
    "HandTipRight",
    "ThumbRight",
)


@dataclass
class TrackedBody:
    slot_index: int
    tracking_id: int
    joint_positions: np.ndarray        # shape (25, 3), float32, CameraSpace meters (X, Y, Z)
    joint_orientations: np.ndarray     # shape (25, 4), float32, Quaternion (X, Y, Z, W)
    joint_tracking_states: np.ndarray  # shape (25,), uint32 (0=NotTracked, 1=Inferred, 2=Tracked)
    clipped_edges: int


@dataclass
class BodyFrameResult:
    frame_index: int
    status: int
    floor_clip_plane: np.ndarray       # shape (4,), float32 (x, y, z, w)
    up_vector: np.ndarray              # shape (4,), float32
    bodies: List[TrackedBody]
    body_index_map: Optional[np.ndarray] = None  # shape (424, 512), uint8 (0..5 or 255)


class KinectV2BodyTracker:
    """Client for the standalone `vvtechs_worker.exe` 3D skeleton host."""

    def __init__(
        self,
        worker_exe: os.PathLike | str,
        dll_dir: Optional[os.PathLike | str] = None,
        wine_binary: str = "wine64",
        wineprefix: Optional[str] = None,
        use_warp: bool = False,
        use_dxvk: bool = True,
        transport: str = "stdio",
    ) -> None:
        self.worker_exe = Path(worker_exe).resolve()
        self.dll_dir = Path(dll_dir).resolve() if dll_dir is not None else None
        self._frame_counter = 0
        self._sock: Optional[socket.socket] = None
        self._xvfb_proc: Optional[subprocess.Popen] = None
        self._transport = transport

        cmd: List[str] = []
        env = os.environ.copy()
        if sys.platform != "win32":
            resolved_wine = shutil.which(wine_binary)
            if resolved_wine is None and wine_binary == "wine64":
                local_wine = Path.home() / ".local" / "bin" / "wine64"
                if local_wine.exists():
                    resolved_wine = str(local_wine)
            if use_dxvk and not use_warp and not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
                env["DISPLAY"] = self._ensure_headless_x11()
            cmd.append(resolved_wine or wine_binary)
            env.setdefault("WINEDEBUG", "-all")
            env.setdefault("DXVK_LOG_LEVEL", "none")
            if use_dxvk and not use_warp:
                env.setdefault("WINEDLLOVERRIDES", "wineusb.sys,winebus.sys=d;d3d11,dxgi,msvcr110,msvcp110,k4wcll=n,b")
            if wineprefix is not None:
                env["WINEPREFIX"] = str(wineprefix)

        tcp_port = 0
        if transport == "stdio":
            cmd.extend([str(self.worker_exe), "--stdio"])
        elif transport == "tcp":
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                probe.bind(("127.0.0.1", 0))
                tcp_port = probe.getsockname()[1]
            cmd.extend([str(self.worker_exe), "--port", str(tcp_port)])
        else:
            raise ValueError(f"Unsupported transport {transport!r}; expected 'stdio' or 'tcp'")

        if self.dll_dir is not None and (sys.platform == "win32" or self.dll_dir != self.worker_exe.parent):
            cmd.extend(["--dll-dir", str(self.dll_dir)])
        if use_warp:
            cmd.append("--warp")

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE if transport == "stdio" else subprocess.DEVNULL,
            stdout=subprocess.PIPE if transport == "stdio" else subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            env=env,
        )

        if transport == "tcp":
            deadline = time.monotonic() + 30.0
            last_err: Optional[Exception] = None
            while time.monotonic() < deadline:
                if self._proc.poll() is not None:
                    stderr_out = self._proc.stderr.read().decode("utf-8", "replace") if self._proc.stderr else ""
                    raise RuntimeError(f"vvtechs_worker exited before accepting TCP connection:\n{stderr_out}")
                try:
                    s = socket.create_connection(("127.0.0.1", tcp_port), timeout=0.5)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    s.settimeout(None)
                    self._sock = s
                    break
                except OSError as exc:
                    last_err = exc
                    time.sleep(0.05)
            if self._sock is None:
                self._proc.kill()
                raise TimeoutError(f"Timed out connecting to vvtechs_worker TCP port {tcp_port}: {last_err}")

    def _send_and_recv(self, code: int, payload: bytes = b"", flags: int = 0) -> Tuple[int, int, bytes]:
        hdr = struct.pack("<IIII", KV2_REQ_MAGIC, code, len(payload), flags)
        if self._transport == "tcp":
            if self._sock is None:
                raise RuntimeError("Worker TCP socket is closed")
            self._sock.sendall(hdr + payload if len(payload) <= 65536 else hdr)
            if len(payload) > 65536:
                self._sock.sendall(payload)
        else:
            if self._proc.stdin is None or self._proc.stdout is None:
                raise RuntimeError("Worker stdio pipe is closed")
            self._proc.stdin.write(hdr)
            if payload:
                self._proc.stdin.write(payload)
            self._proc.stdin.flush()

        resp_hdr = self._read_exact(16)
        magic, status, resp_len, meta = struct.unpack("<IIII", resp_hdr)
        if magic != KV2_RESP_MAGIC:
            stderr_tail = b""
            if self._proc.stderr is not None and self._proc.poll() is not None:
                stderr_tail = self._proc.stderr.read()
            raise RuntimeError(f"Unexpected worker response magic 0x{magic:08x}; stderr={stderr_tail!r}")
        body = self._read_exact(resp_len) if resp_len > 0 else b""
        return status, meta, body

    def _read_exact(self, nbytes: int) -> bytes:
        chunks: List[bytes] = []
        rem = nbytes
        if self._transport == "tcp":
            assert self._sock is not None
            while rem > 0:
                chunk = self._sock.recv(rem)
                if not chunk:
                    stderr_out = self._proc.stderr.read().decode("utf-8", "replace") if self._proc.stderr else ""
                    raise EOFError(f"vvtechs_worker closed TCP connection unexpectedly. stderr:\n{stderr_out}")
                chunks.append(chunk)
                rem -= len(chunk)
            return b"".join(chunks)

        assert self._proc.stdout is not None
        while rem > 0:
            chunk = self._proc.stdout.read(rem)
            if not chunk:
                stderr_out = self._proc.stderr.read().decode("utf-8", "replace") if self._proc.stderr else ""
                raise EOFError(f"vvtechs_worker terminated unexpectedly. stderr:\n{stderr_out}")
            chunks.append(chunk)
            rem -= len(chunk)
        return b"".join(chunks)

    def reset(self) -> None:
        """Reset temporal skeleton tracking state and background model."""
        self._send_and_recv(KV2_CMD_RESET)

    def set_intrinsics(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        k1: float = 0.0,
        k2: float = 0.0,
        k3: float = 0.0,
    ) -> None:
        """Configure sensor-specific IR/Depth camera intrinsics (e.g. from libfreenect2 IrCameraParams)."""
        payload = struct.pack("<7f", fx, fy, cx, cy, k1, k2, k3)
        self._send_and_recv(KV2_CMD_SET_INTRINSICS, payload)

    def process_frame(
        self,
        depth_mm: np.ndarray,
        ir_frame: Optional[np.ndarray] = None,
        gravity: Tuple[float, float, float, float] = (0.0, -1.0, 0.0, 0.0),
        want_body_index: bool = False,
        mirror_horizontal: bool = False,
        frame_index: Optional[int] = None,
    ) -> BodyFrameResult:
        """
        Run Microsoft Kinect v2 3D skeleton tracking on a single 512x424 depth frame.

        Parameters:
          depth_mm: (424, 512) array of depth values in millimeters (uint16 or float32).
          ir_frame: optional (424, 512) uint16/float32 infrared array.
          gravity: 4-tuple accelerometer/gravity vector in camera space (default (0, -1, 0, 0)).
          want_body_index: when True, also returns the 424x512 uint8 per-pixel player mask.
          mirror_horizontal: set True if the upstream driver flipped depth columns horizontally
            relative to Kinect v2 selfie depth layout (col 0 = sensor right / user left).
        """
        if depth_mm.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
            raise ValueError(f"Expected depth_mm shape (424, 512), got {depth_mm.shape}")

        if mirror_horizontal:
            depth_mm = depth_mm[:, ::-1]
            if ir_frame is not None:
                ir_frame = ir_frame[:, ::-1]

        if depth_mm.dtype != np.uint16:
            clean_depth = np.nan_to_num(depth_mm, copy=False, nan=0.0, posinf=0.0, neginf=0.0)
            depth_u16 = np.ascontiguousarray(np.clip(np.rint(clean_depth), 0, 65535).astype("<u2"))
        else:
            depth_u16 = np.ascontiguousarray(depth_mm, dtype="<u2")

        self._frame_counter += 1
        fidx = self._frame_counter if frame_index is None else int(frame_index)

        flags = KV2_FLAG_WANT_BODY_INDEX if want_body_index else 0
        parts = [
            struct.pack("<I4f", fidx, gravity[0], gravity[1], gravity[2], gravity[3]),
            depth_u16.tobytes(),
        ]
        if ir_frame is not None:
            if ir_frame.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
                raise ValueError(f"Expected ir_frame shape (424, 512), got {ir_frame.shape}")
            ir_u16 = np.ascontiguousarray(np.clip(np.rint(ir_frame), 0, 65535).astype("<u2"))
            parts.append(ir_u16.tobytes())
            flags |= KV2_FLAG_HAS_IR_FRAME

        status, resp_fidx, raw = self._send_and_recv(KV2_CMD_PROCESS_FRAME, b"".join(parts), flags=flags)
        return self._decode_response(status, resp_fidx, raw, want_body_index)

    @staticmethod
    def _decode_response(status: int, frame_index: int, raw: bytes, want_body_index: bool) -> BodyFrameResult:
        floor_clip = np.frombuffer(raw, dtype="<f4", count=4, offset=0).copy()
        up_vec = np.frombuffer(raw, dtype="<f4", count=4, offset=16).copy()

        bodies: List[TrackedBody] = []
        for slot in range(BODY_COUNT):
            base = 0x20 + slot * 0x410
            is_tracked = struct.unpack_from("<I", raw, base + 0x3F8)[0]
            if is_tracked == 0:
                continue
            tracking_id = struct.unpack_from("<Q", raw, base + 0x3F0)[0]
            clipped_edges = struct.unpack_from("<I", raw, base + 0x40C)[0]
            pos4 = np.frombuffer(raw, dtype="<f4", count=100, offset=base + 0x000).reshape(25, 4)
            ori4 = np.frombuffer(raw, dtype="<f4", count=100, offset=base + 0x190).reshape(25, 4)
            states = np.frombuffer(raw, dtype="<u4", count=25, offset=base + 0x320).copy()
            bodies.append(
                TrackedBody(
                    slot_index=slot,
                    tracking_id=tracking_id,
                    joint_positions=pos4[:, :3].copy(),
                    joint_orientations=ori4.copy(),
                    joint_tracking_states=states,
                    clipped_edges=clipped_edges,
                )
            )

        bidx_map: Optional[np.ndarray] = None
        if want_body_index:
            bidx_map = np.frombuffer(raw, dtype=np.uint8, count=DEPTH_PIXELS, offset=0x1890).reshape(
                DEPTH_HEIGHT, DEPTH_WIDTH
            ).copy()

        return BodyFrameResult(
            frame_index=frame_index,
            status=status,
            floor_clip_plane=floor_clip,
            up_vector=up_vec,
            bodies=bodies,
            body_index_map=bidx_map,
        )

    def _ensure_headless_x11(self) -> str:
        """Locate an existing user-owned X11 socket or start a private Xvfb for Wine Vulkan WSI."""
        x11_dir = Path("/tmp/.X11-unix")
        uid = os.getuid()
        if x11_dir.is_dir():
            for sock in sorted(x11_dir.glob("X*")):
                suffix = sock.name[1:]
                if suffix.isdigit() and sock.stat().st_uid == uid:
                    return f":{suffix}"
        xvfb_bin = shutil.which("Xvfb")
        if xvfb_bin is None:
            return ":0"
        for disp in range(95, 125):
            if not (x11_dir / f"X{disp}").exists():
                self._xvfb_proc = subprocess.Popen(
                    [xvfb_bin, f":{disp}", "-screen", "0", "640x480x24", "-nolisten", "tcp"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    if (x11_dir / f"X{disp}").exists():
                        break
                    time.sleep(0.05)
                return f":{disp}"
        return ":0"

    def close(self) -> None:
        if self._proc.poll() is None:
            try:
                self._send_and_recv(KV2_CMD_QUIT)
                self._proc.wait(timeout=2.0)
            except Exception:
                self._proc.kill()
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._xvfb_proc is not None and self._xvfb_proc.poll() is None:
            try:
                self._xvfb_proc.terminate()
                self._xvfb_proc.wait(timeout=2.0)
            except Exception:
                self._xvfb_proc.kill()
            self._xvfb_proc = None

    def __enter__(self) -> "KinectV2BodyTracker":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
