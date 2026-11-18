"""
kinectv2_sdk: Native Linux C/C++ and zero-copy Python SDK for Microsoft Kinect v2.

Provides locked 30.0 FPS streaming of 1080p RGB, 512x424 millimeter Depth, Infrared,
Registered RGB-D, BodyIndex segmentation masks, and 25-joint 3D skeletons matching the
official Microsoft Kinect for Windows v2 SDK.
"""

from .sensor import (
    BONE_CONNECTIONS,
    JOINT_NAMES,
    KV2_COLOR_HEIGHT,
    KV2_COLOR_WIDTH,
    KV2_DEPTH_HEIGHT,
    KV2_DEPTH_WIDTH,
    KV2_JOINT_COUNT,
    KV2_MAX_BODIES,
    KV2_PIPELINE_AUTO,
    KV2_PIPELINE_CPU,
    KV2_PIPELINE_CUDA,
    KV2_PIPELINE_OPENCL,
    KV2_STREAM_ALL,
    KV2_STREAM_BODY,
    KV2_STREAM_BODY_INDEX,
    KV2_STREAM_COLOR,
    KV2_STREAM_DEFAULT,
    KV2_STREAM_DEPTH,
    KV2_STREAM_IR,
    KV2_STREAM_REGISTERED_COLOR,
    BodyData,
    ColorIntrinsics,
    FrameGuard,
    IrIntrinsics,
    JointType,
    KinectV2Sensor,
    PipelineType,
    StreamFlags,
    TrackingState,
)
from .tracker import (
    BodyFrameResult,
    KinectV2BodyTracker,
    TrackedBody,
)

__version__ = "1.0.0"

__all__ = [
    "__version__",
    "KinectV2Sensor",
    "FrameGuard",
    "BodyData",
    "IrIntrinsics",
    "ColorIntrinsics",
    "JointType",
    "TrackingState",
    "PipelineType",
    "StreamFlags",
    "JOINT_NAMES",
    "BONE_CONNECTIONS",
    "KV2_COLOR_WIDTH",
    "KV2_COLOR_HEIGHT",
    "KV2_DEPTH_WIDTH",
    "KV2_DEPTH_HEIGHT",
    "KV2_MAX_BODIES",
    "KV2_JOINT_COUNT",
    "KV2_PIPELINE_AUTO",
    "KV2_PIPELINE_CUDA",
    "KV2_PIPELINE_OPENCL",
    "KV2_PIPELINE_CPU",
    "KV2_STREAM_DEPTH",
    "KV2_STREAM_IR",
    "KV2_STREAM_COLOR",
    "KV2_STREAM_BODY",
    "KV2_STREAM_BODY_INDEX",
    "KV2_STREAM_REGISTERED_COLOR",
    "KV2_STREAM_DEFAULT",
    "KV2_STREAM_ALL",
    "KinectV2BodyTracker",
    "TrackedBody",
    "BodyFrameResult",
]
