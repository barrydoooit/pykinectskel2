"""
Zero-copy Python binding for `libkinectv2_sdk.so` (Native Linux C/C++ Kinect v2 SDK).

Provides locked 30.0 FPS multi-modal streaming (1080p RGB, 512x424 Depth, IR, Registered RGB-D,
BodyIndex mask, and 25-joint 3D Skeletons with pre-projected 2D Depth & Color pixel coordinates)
without Python frame-copy bottlenecks.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
from ctypes import (
    POINTER,
    Structure,
    byref,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_int32,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
)
from dataclasses import dataclass
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

KV2_COLOR_WIDTH = 1920
KV2_COLOR_HEIGHT = 1080
KV2_DEPTH_WIDTH = 512
KV2_DEPTH_HEIGHT = 424
KV2_MAX_BODIES = 6
KV2_JOINT_COUNT = 25


class PipelineType(IntEnum):
    AUTO = 0
    CUDA = 1
    OPENCL = 2
    CPU = 3


class StreamFlags(IntFlag):
    DEPTH = 0x01
    IR = 0x02
    COLOR = 0x04
    BODY = 0x08
    BODY_INDEX = 0x10
    REGISTERED_COLOR = 0x20
    DEFAULT = 0x1F
    ALL = 0x3F


class TrackingState(IntEnum):
    NOT_TRACKED = 0
    INFERRED = 1
    TRACKED = 2


class JointType(IntEnum):
    SPINE_BASE = 0
    SPINE_MID = 1
    NECK = 2
    HEAD = 3
    SHOULDER_LEFT = 4
    ELBOW_LEFT = 5
    WRIST_LEFT = 6
    HAND_LEFT = 7
    SHOULDER_RIGHT = 8
    ELBOW_RIGHT = 9
    WRIST_RIGHT = 10
    HAND_RIGHT = 11
    HIP_LEFT = 12
    KNEE_LEFT = 13
    ANKLE_LEFT = 14
    FOOT_LEFT = 15
    HIP_RIGHT = 16
    KNEE_RIGHT = 17
    ANKLE_RIGHT = 18
    FOOT_RIGHT = 19
    SPINE_SHOULDER = 20
    HAND_TIP_LEFT = 21
    THUMB_LEFT = 22
    HAND_TIP_RIGHT = 23
    THUMB_RIGHT = 24


KV2_PIPELINE_AUTO = int(PipelineType.AUTO)
KV2_PIPELINE_CUDA = int(PipelineType.CUDA)
KV2_PIPELINE_OPENCL = int(PipelineType.OPENCL)
KV2_PIPELINE_CPU = int(PipelineType.CPU)

KV2_STREAM_DEPTH = int(StreamFlags.DEPTH)
KV2_STREAM_IR = int(StreamFlags.IR)
KV2_STREAM_COLOR = int(StreamFlags.COLOR)
KV2_STREAM_BODY = int(StreamFlags.BODY)
KV2_STREAM_BODY_INDEX = int(StreamFlags.BODY_INDEX)
KV2_STREAM_REGISTERED_COLOR = int(StreamFlags.REGISTERED_COLOR)
KV2_STREAM_DEFAULT = int(StreamFlags.DEFAULT)
KV2_STREAM_ALL = int(StreamFlags.ALL)

JOINT_NAMES: Tuple[str, ...] = (
    "SpineBase", "SpineMid", "Neck", "Head",
    "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft",
    "ShoulderRight", "ElbowRight", "WristRight", "HandRight",
    "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
    "HipRight", "KneeRight", "AnkleRight", "FootRight",
    "SpineShoulder", "HandTipLeft", "ThumbLeft", "HandTipRight", "ThumbRight",
)

BONE_CONNECTIONS: Tuple[Tuple[int, int], ...] = (
    (0, 1), (1, 20), (20, 2), (2, 3),
    (20, 4), (4, 5), (5, 6), (6, 7), (7, 21), (6, 22),
    (20, 8), (8, 9), (9, 10), (10, 11), (11, 23), (10, 24),
    (0, 12), (12, 13), (13, 14), (14, 15),
    (0, 16), (16, 17), (17, 18), (18, 19),
)


class _CKv2IrIntrinsics(Structure):
    _fields_ = [
        ("fx", c_float), ("fy", c_float),
        ("cx", c_float), ("cy", c_float),
        ("k1", c_float), ("k2", c_float), ("k3", c_float),
        ("p1", c_float), ("p2", c_float),
    ]


class _CKv2ColorIntrinsics(Structure):
    _fields_ = [
        ("fx", c_float), ("fy", c_float),
        ("cx", c_float), ("cy", c_float),
    ]


class _CKv2Joint(Structure):
    _fields_ = [
        ("pos", c_float * 3),
        ("quat", c_float * 4),
        ("depth_uv", c_float * 2),
        ("color_uv", c_float * 2),
        ("tracking_state", c_uint32),
    ]


class _CKv2Body(Structure):
    _fields_ = [
        ("tracking_id", c_uint64),
        ("is_tracked", c_uint8),
        ("slot_index", c_uint8),
        ("reserved", c_uint16),
        ("clipped_edges", c_uint32),
        ("joints", _CKv2Joint * KV2_JOINT_COUNT),
    ]


class _CKv2FrameView(Structure):
    _fields_ = [
        ("frame_seq", c_uint64),
        ("timestamp_sec", c_double),
        ("sensor_fps", c_float),
        ("skeleton_fps", c_float),
        ("skeleton_infer_ms", c_float),
        ("stream_flags", c_uint32),
        ("color_bgrx", POINTER(c_uint8)),
        ("depth_mm", POINTER(c_uint16)),
        ("ir", POINTER(c_uint16)),
        ("registered_bgrx", POINTER(c_uint8)),
        ("undistorted_depth_mm", POINTER(c_float)),
        ("body_index", POINTER(c_uint8)),
        ("floor_clip_plane", c_float * 4),
        ("up_vector", c_float * 4),
        ("body_count", c_uint32),
        ("bodies", _CKv2Body * KV2_MAX_BODIES),
        ("_slot_id", c_int32),
        ("_reserved", c_int32),
    ]


class _CKv2Config(Structure):
    _fields_ = [
        ("serial", c_char_p),
        ("pipeline", c_int32),
        ("stream_flags", c_uint32),
        ("worker_exe", c_char_p),
        ("dll_dir", c_char_p),
        ("wine_binary", c_char_p),
        ("wineprefix", c_char_p),
        ("mirror_skeleton", c_int32),
    ]


def _resolve_lib_path(lib_path: Optional[os.PathLike | str]) -> str:
    if lib_path is not None:
        return str(Path(lib_path).expanduser().resolve())
    env_lib = os.environ.get("KINECTV2_SDK_LIB")
    if env_lib:
        return str(Path(env_lib).expanduser().resolve())
    repo_root = Path(__file__).resolve().parents[2]
    local_candidate = repo_root / "lib" / "libkinectv2_sdk.so"
    if local_candidate.is_file():
        return str(local_candidate)
    sys_lib = ctypes.util.find_library("kinectv2_sdk")
    if sys_lib:
        return sys_lib
    return str(local_candidate)


def _load_sdk_lib(lib_path: Optional[os.PathLike | str] = None) -> ctypes.CDLL:
    resolved = _resolve_lib_path(lib_path)
    lib = ctypes.CDLL(resolved)

    lib.kv2_default_config.argtypes = [POINTER(_CKv2Config)]
    lib.kv2_default_config.restype = None

    lib.kv2_open.argtypes = [POINTER(_CKv2Config), ctypes.c_char_p, c_size_t]
    lib.kv2_open.restype = c_void_p

    lib.kv2_close.argtypes = [c_void_p]
    lib.kv2_close.restype = None

    lib.kv2_start.argtypes = [c_void_p, ctypes.c_char_p, c_size_t]
    lib.kv2_start.restype = c_int

    lib.kv2_stop.argtypes = [c_void_p]
    lib.kv2_stop.restype = None

    lib.kv2_wait_frame.argtypes = [c_void_p, c_uint64, c_int32, POINTER(_CKv2FrameView)]
    lib.kv2_wait_frame.restype = c_int

    lib.kv2_get_latest_frame.argtypes = [c_void_p, POINTER(_CKv2FrameView)]
    lib.kv2_get_latest_frame.restype = c_int

    lib.kv2_release_frame.argtypes = [c_void_p, POINTER(_CKv2FrameView)]
    lib.kv2_release_frame.restype = None

    lib.kv2_reset_skeleton.argtypes = [c_void_p]
    lib.kv2_reset_skeleton.restype = c_int

    lib.kv2_set_registered_color_enabled.argtypes = [c_void_p, c_int]
    lib.kv2_set_registered_color_enabled.restype = None

    lib.kv2_get_memory_rss_mb.argtypes = [c_void_p, POINTER(c_float), POINTER(c_float)]
    lib.kv2_get_memory_rss_mb.restype = None

    lib.kv2_get_serial.argtypes = [c_void_p]
    lib.kv2_get_serial.restype = c_char_p

    lib.kv2_get_pipeline_name.argtypes = [c_void_p]
    lib.kv2_get_pipeline_name.restype = c_char_p

    lib.kv2_get_intrinsics.argtypes = [c_void_p, POINTER(_CKv2IrIntrinsics), POINTER(_CKv2ColorIntrinsics)]
    lib.kv2_get_intrinsics.restype = None

    lib.kv2_project_camera_to_depth.argtypes = [c_void_p, POINTER(c_float), POINTER(c_float)]
    lib.kv2_project_camera_to_depth.restype = None

    lib.kv2_project_camera_to_color.argtypes = [c_void_p, POINTER(c_float), POINTER(c_float)]
    lib.kv2_project_camera_to_color.restype = None

    lib.kv2_map_depth_to_color.argtypes = [c_void_p, c_int, c_int, c_float, POINTER(c_float)]
    lib.kv2_map_depth_to_color.restype = c_int

    lib.kv2_deproject_depth_to_camera.argtypes = [c_void_p, c_int, c_int, c_float, POINTER(c_float)]
    lib.kv2_deproject_depth_to_camera.restype = c_int

    return lib


@dataclass(frozen=True)
class IrIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float
    k2: float
    k3: float
    p1: float
    p2: float


@dataclass(frozen=True)
class ColorIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class BodyData:
    tracking_id: int
    slot_index: int
    clipped_edges: int
    joint_positions: np.ndarray        # (25, 3) float32 CameraSpace meters (X, Y, Z)
    joint_orientations: np.ndarray     # (25, 4) float32 quaternion (x, y, z, w)
    joint_depth_uv: np.ndarray         # (25, 2) float32 pixel (u, v) on 512x424 Depth/IR
    joint_color_uv: np.ndarray         # (25, 2) float32 pixel (u, v) on 1920x1080 Color
    joint_tracking_states: np.ndarray  # (25,) uint32 (0=NotTracked, 1=Inferred, 2=Tracked)


class FrameGuard:
    """RAII zero-copy view into a C++ pinned FrameSlot; release via `with` block or `.release()`."""

    def __init__(self, lib: ctypes.CDLL, dev_ptr: c_void_p, view: _CKv2FrameView) -> None:
        self._lib = lib
        self._dev = dev_ptr
        self._view = view
        self._released = False

        self.frame_seq: int = int(view.frame_seq)
        self.timestamp_sec: float = float(view.timestamp_sec)
        self.sensor_fps: float = float(view.sensor_fps)
        self.skeleton_fps: float = float(view.skeleton_fps)
        self.skeleton_infer_ms: float = float(view.skeleton_infer_ms)
        self.floor_clip_plane: np.ndarray = np.ctypeslib.as_array(view.floor_clip_plane).copy()
        self.up_vector: np.ndarray = np.ctypeslib.as_array(view.up_vector).copy()

        self.bodies: List[BodyData] = []
        for i in range(int(view.body_count)):
            cb = view.bodies[i]
            pos = np.empty((25, 3), dtype=np.float32)
            quat = np.empty((25, 4), dtype=np.float32)
            duv = np.empty((25, 2), dtype=np.float32)
            cuv = np.empty((25, 2), dtype=np.float32)
            st = np.empty((25,), dtype=np.uint32)
            for j in range(25):
                cj = cb.joints[j]
                pos[j] = (cj.pos[0], cj.pos[1], cj.pos[2])
                quat[j] = (cj.quat[0], cj.quat[1], cj.quat[2], cj.quat[3])
                duv[j] = (cj.depth_uv[0], cj.depth_uv[1])
                cuv[j] = (cj.color_uv[0], cj.color_uv[1])
                st[j] = cj.tracking_state
            self.bodies.append(
                BodyData(
                    tracking_id=int(cb.tracking_id),
                    slot_index=int(cb.slot_index),
                    clipped_edges=int(cb.clipped_edges),
                    joint_positions=pos,
                    joint_orientations=quat,
                    joint_depth_uv=duv,
                    joint_color_uv=cuv,
                    joint_tracking_states=st,
                )
            )

    def _require_open(self) -> None:
        if self._released:
            raise RuntimeError("FrameGuard has already been released")

    @property
    def color_bgrx(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.color_bgrx):
            return None
        arr = np.ctypeslib.as_array(self._view.color_bgrx, shape=(KV2_COLOR_HEIGHT, KV2_COLOR_WIDTH, 4))
        arr.flags.writeable = False
        return arr

    @property
    def depth_mm(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.depth_mm):
            return None
        arr = np.ctypeslib.as_array(self._view.depth_mm, shape=(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH))
        arr.flags.writeable = False
        return arr

    @property
    def ir(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.ir):
            return None
        arr = np.ctypeslib.as_array(self._view.ir, shape=(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH))
        arr.flags.writeable = False
        return arr

    @property
    def registered_bgrx(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.registered_bgrx):
            return None
        arr = np.ctypeslib.as_array(self._view.registered_bgrx, shape=(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH, 4))
        arr.flags.writeable = False
        return arr

    @property
    def undistorted_depth_mm(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.undistorted_depth_mm):
            return None
        arr = np.ctypeslib.as_array(self._view.undistorted_depth_mm, shape=(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH))
        arr.flags.writeable = False
        return arr

    @property
    def body_index(self) -> Optional[np.ndarray]:
        self._require_open()
        if not bool(self._view.body_index):
            return None
        arr = np.ctypeslib.as_array(self._view.body_index, shape=(KV2_DEPTH_HEIGHT, KV2_DEPTH_WIDTH))
        arr.flags.writeable = False
        return arr

    def release(self) -> None:
        if not self._released and self._dev:
            self._lib.kv2_release_frame(self._dev, byref(self._view))
            self._released = True

    def __enter__(self) -> "FrameGuard":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.release()

    def __del__(self) -> None:
        self.release()


class KinectV2Sensor:
    """High-level zero-copy Python interface to the native C++ `libkinectv2_sdk.so` runtime."""

    def __init__(
        self,
        serial: Optional[str] = None,
        pipeline: int = KV2_PIPELINE_AUTO,
        stream_flags: int = KV2_STREAM_DEFAULT,
        mirror_skeleton: bool = False,
        worker_exe: Optional[os.PathLike | str] = None,
        dll_dir: Optional[os.PathLike | str] = None,
        wine_binary: Optional[str] = None,
        wineprefix: Optional[os.PathLike | str] = None,
        lib_path: Optional[os.PathLike | str] = None,
    ) -> None:
        self._lib = _load_sdk_lib(lib_path)
        cfg = _CKv2Config()
        self._lib.kv2_default_config(byref(cfg))
        self._serial_b = serial.encode("utf-8") if serial else None
        self._worker_b = str(worker_exe).encode("utf-8") if worker_exe else None
        self._dlldir_b = str(dll_dir).encode("utf-8") if dll_dir else None
        self._wine_b = wine_binary.encode("utf-8") if wine_binary else None
        self._prefix_b = str(wineprefix).encode("utf-8") if wineprefix else None

        cfg.serial = self._serial_b
        cfg.pipeline = int(pipeline)
        cfg.stream_flags = int(stream_flags)
        cfg.worker_exe = self._worker_b
        cfg.dll_dir = self._dlldir_b
        cfg.wine_binary = self._wine_b
        cfg.wineprefix = self._prefix_b
        cfg.mirror_skeleton = 1 if mirror_skeleton else 0

        err = ctypes.create_string_buffer(512)
        self._dev = self._lib.kv2_open(byref(cfg), err, len(err))
        if not self._dev:
            raise RuntimeError(f"kv2_open failed: {err.value.decode('utf-8', 'replace')}")
        self._last_seq: int = 0

    @property
    def serial(self) -> str:
        return self._lib.kv2_get_serial(self._dev).decode("utf-8", "replace")

    @property
    def pipeline_name(self) -> str:
        return self._lib.kv2_get_pipeline_name(self._dev).decode("utf-8", "replace")

    def start(self) -> None:
        err = ctypes.create_string_buffer(512)
        if self._lib.kv2_start(self._dev, err, len(err)) != 0:
            raise RuntimeError(f"kv2_start failed: {err.value.decode('utf-8', 'replace')}")

    def stop(self) -> None:
        if self._dev:
            self._lib.kv2_stop(self._dev)

    def close(self) -> None:
        if self._dev:
            self._lib.kv2_close(self._dev)
            self._dev = None

    def reset_skeleton(self) -> None:
        self._lib.kv2_reset_skeleton(self._dev)

    def set_registered_color_enabled(self, enabled: bool) -> None:
        self._lib.kv2_set_registered_color_enabled(self._dev, 1 if enabled else 0)

    def get_memory_rss_mb(self) -> Tuple[float, float]:
        host_mb = c_float(0.0)
        worker_mb = c_float(0.0)
        self._lib.kv2_get_memory_rss_mb(self._dev, byref(host_mb), byref(worker_mb))
        return float(host_mb.value), float(worker_mb.value)

    def get_intrinsics(self) -> Tuple[IrIntrinsics, ColorIntrinsics]:
        ir_c = _CKv2IrIntrinsics()
        col_c = _CKv2ColorIntrinsics()
        self._lib.kv2_get_intrinsics(self._dev, byref(ir_c), byref(col_c))
        return (
            IrIntrinsics(
                fx=float(ir_c.fx), fy=float(ir_c.fy),
                cx=float(ir_c.cx), cy=float(ir_c.cy),
                k1=float(ir_c.k1), k2=float(ir_c.k2), k3=float(ir_c.k3),
                p1=float(ir_c.p1), p2=float(ir_c.p2),
            ),
            ColorIntrinsics(
                fx=float(col_c.fx), fy=float(col_c.fy),
                cx=float(col_c.cx), cy=float(col_c.cy),
            ),
        )

    def wait_frame(self, timeout_ms: int = 1000) -> Optional[FrameGuard]:
        v = _CKv2FrameView()
        rc = self._lib.kv2_wait_frame(self._dev, c_uint64(self._last_seq), c_int32(timeout_ms), byref(v))
        if rc == 1:
            self._last_seq = int(v.frame_seq)
            return FrameGuard(self._lib, self._dev, v)
        if rc == 0:
            return None
        raise RuntimeError("kv2_wait_frame failed (stream stopped)")

    def get_latest_frame(self) -> Optional[FrameGuard]:
        v = _CKv2FrameView()
        rc = self._lib.kv2_get_latest_frame(self._dev, byref(v))
        if rc == 1:
            self._last_seq = int(v.frame_seq)
            return FrameGuard(self._lib, self._dev, v)
        return None

    def project_camera_to_depth(self, xyz_m: Tuple[float, float, float]) -> Tuple[float, float]:
        inp = (c_float * 3)(float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2]))
        out = (c_float * 2)()
        self._lib.kv2_project_camera_to_depth(self._dev, inp, out)
        return float(out[0]), float(out[1])

    def project_camera_to_color(self, xyz_m: Tuple[float, float, float]) -> Tuple[float, float]:
        inp = (c_float * 3)(float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2]))
        out = (c_float * 2)()
        self._lib.kv2_project_camera_to_color(self._dev, inp, out)
        return float(out[0]), float(out[1])

    def map_depth_to_color(self, dx: int, dy: int, depth_mm: float) -> Optional[Tuple[float, float]]:
        out = (c_float * 2)()
        if self._lib.kv2_map_depth_to_color(self._dev, int(dx), int(dy), c_float(depth_mm), out) != 0:
            return None
        return float(out[0]), float(out[1])

    def deproject_depth_to_camera(self, dx: int, dy: int, depth_mm: float) -> Optional[Tuple[float, float, float]]:
        out = (c_float * 3)()
        if self._lib.kv2_deproject_depth_to_camera(self._dev, int(dx), int(dy), c_float(depth_mm), out) != 0:
            return None
        return float(out[0]), float(out[1]), float(out[2])

    def __enter__(self) -> "KinectV2Sensor":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
