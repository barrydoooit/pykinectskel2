"""Unit tests for `kinectv2_sdk` public Python API, enums, constants, and C ABI binding."""

from __future__ import annotations

import ctypes
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import kinectv2_sdk as kv2
import pykinectskel2 as pks2
from kinectv2_sdk.sensor import _CKv2Body, _CKv2FrameView, _CKv2Joint, _load_sdk_lib


def test_public_exports_and_abi() -> None:
    assert pks2.KinectV2Sensor is kv2.KinectV2Sensor
    assert len(kv2.JOINT_NAMES) == 25
    assert len(kv2.BONE_CONNECTIONS) == 24
    assert kv2.JointType.HEAD == 3
    assert kv2.JointType.HAND_LEFT == 7
    assert kv2.JointType.HAND_RIGHT == 11
    assert kv2.TrackingState.TRACKED == 2
    assert kv2.StreamFlags.ALL == 0x3F

    assert ctypes.sizeof(_CKv2Joint) == 48
    assert ctypes.sizeof(_CKv2Body) == 16 + 25 * 48
    assert ctypes.sizeof(_CKv2FrameView) > ctypes.sizeof(_CKv2Body) * 6

    lib = _load_sdk_lib()
    lib.kv2_joint_name.argtypes = [ctypes.c_int]
    lib.kv2_joint_name.restype = ctypes.c_char_p
    for idx, expected in enumerate(kv2.JOINT_NAMES):
        actual = lib.kv2_joint_name(idx).decode("utf-8")
        assert actual == expected, f"Joint {idx}: {actual} != {expected}"


if __name__ == "__main__":
    test_public_exports_and_abi()
    print("RESULT: PYTHON API & C ABI CHECK PASSED.")
