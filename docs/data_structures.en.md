# Data Structures & Coordinate Systems Reference (EN)

[English](data_structures.en.md) | [繁體中文](data_structures.zh-TW.md) | [简体中文](data_structures.zh-CN.md)

This document describes the coordinate conventions, C/C++ structs (`include/kinectv2_sdk.h`), and zero-copy Python classes (`kinectv2_sdk`) exposed by `kinectv2-sdk`.

---

## 1. Coordinate Systems & Units

### 1.1 3D Camera Space (`pos` / `joint_positions`)
All 3D joint positions and deprojected points are expressed in **meters (`float32`)** with origin `(0, 0, 0)` at the optical center of the Kinect v2 IR/Depth sensor:
- **`+X` (meters)**: Horizontal axis pointing right in the default `libfreenect2` selfie view (`mirror_skeleton = 0`), matching the subject's anatomical right on positive `X` when `mirror_skeleton = 1`, or aligned directly with column index `u` (`0..511`) when `mirror_skeleton = 0`.
- **`+Y` (meters)**: Vertical axis pointing **up** (against gravity).
- **`+Z` (meters)**: Optical depth axis pointing **forward** out of the sensor (`> 0` in front of the lens; native Kinect v2 skeleton tracking operates over `1.45 m .. 3.8 m`, and frames `1..15` after startup or `reset_skeleton()` initialize the internal background model).

### 1.2 Joint Orientations (`quat` / `joint_orientations`)
- Unit quaternion **`(x, y, z, w)`** (`float32`) representing bone orientation along the kinematic hierarchy in Camera Space, identical to Microsoft `JointOrientation.Orientation`. Root or leaf joints without defined twist return `(0, 0, 0, 0)`.

### 1.3 2D Image Spaces
| Space | Resolution | Origin `(0, 0)` | Joint Projection Field |
| :--- | :--- | :--- | :--- |
| Depth / IR / BodyIndex | `512 x 424` | Top-left pixel (`u` right `0..511`, `v` down `0..423`) | `Kv2Joint.depth_uv` / `BodyData.joint_depth_uv` |
| Color (RGB) | `1920 x 1080` | Top-left pixel (`u` right `0..1919`, `v` down `0..1079`) | `Kv2Joint.color_uv` / `BodyData.joint_color_uv` |

Unprojected or invalid joints (`tracking_state == 0`) have UV coordinates set to `(-1e6, -1e6)`.

---

## 2. Enumerations & Bitmasks

### 2.1 `Kv2PipelineType` / `kinectv2_sdk.PipelineType`
| Value | C Enum | Python Enum | Description |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_PIPELINE_AUTO` | `PipelineType.AUTO` | Selects `CUDA` -> `OpenCL` -> `CPU` automatically |
| `1` | `KV2_PIPELINE_CUDA` | `PipelineType.CUDA` | NVIDIA CUDA depth packet processor |
| `2` | `KV2_PIPELINE_OPENCL` | `PipelineType.OPENCL` | OpenCL 1.2+ depth packet processor |
| `3` | `KV2_PIPELINE_CPU` | `PipelineType.CPU` | Pure CPU fallback processor |

### 2.2 `Kv2StreamFlags` / `kinectv2_sdk.StreamFlags`
| Hex | C Flag | Python Flag | Enabled Output Buffer |
| :---: | :--- | :--- | :--- |
| `0x01` | `KV2_STREAM_DEPTH` | `StreamFlags.DEPTH` | `depth_mm` (`512x424 uint16` mm) |
| `0x02` | `KV2_STREAM_IR` | `StreamFlags.IR` | `ir` (`512x424 uint16` amplitude) |
| `0x04` | `KV2_STREAM_COLOR` | `StreamFlags.COLOR` | `color_bgrx` (`1920x1080x4 uint8` BGRX) |
| `0x08` | `KV2_STREAM_BODY` | `StreamFlags.BODY` | `bodies` (25-joint 3D skeletons) |
| `0x10` | `KV2_STREAM_BODY_INDEX` | `StreamFlags.BODY_INDEX` | `body_index` (`512x424 uint8` player mask) |
| `0x20` | `KV2_STREAM_REGISTERED_COLOR` | `StreamFlags.REGISTERED_COLOR` | `registered_bgrx` & `undistorted_depth_mm` |
| `0x1F` | `KV2_STREAM_DEFAULT` | `StreamFlags.DEFAULT` | `DEPTH \| IR \| COLOR \| BODY \| BODY_INDEX` |
| `0x3F` | `KV2_STREAM_ALL` | `StreamFlags.ALL` | All six streams enabled |

### 2.3 `Kv2TrackingState` / `kinectv2_sdk.TrackingState`
| Value | C Enum | Python Enum | Meaning |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_TRACKING_NOT_TRACKED` | `TrackingState.NOT_TRACKED` | Joint is not tracked |
| `1` | `KV2_TRACKING_INFERRED` | `TrackingState.INFERRED` | Joint position is kinematically inferred (occluded) |
| `2` | `KV2_TRACKING_TRACKED` | `TrackingState.TRACKED` | Joint is directly observed with high confidence |

---

## 3. Core Data Structures (C/C++ vs. Python)

### 3.1 Frame Container: `Kv2FrameView` (C/C++) / `FrameGuard` (Python)
Returned by `kv2_wait_frame()` / `sensor.wait_frame()`. Image pointers/arrays are **zero-copy views** into a pre-allocated 4-slot C++ ring pool and remain valid until `kv2_release_frame()` / exiting the `with frame:` block (copy with `.copy()` in Python if keeping beyond the `with` block).

| Field | C Type | Python Type / Shape (`dtype`) | Description |
| :--- | :--- | :--- | :--- |
| `frame_seq` | `uint64_t` | `int` | Monotonically increasing frame sequence number (`>= 1`) |
| `timestamp_sec` | `double` | `float` | Steady monotonic clock timestamp in seconds |
| `sensor_fps` | `float` | `float` | Rolling 30-frame sensor capture rate (Hz) |
| `skeleton_fps` | `float` | `float` | Rolling 30-frame skeleton inference rate (Hz) |
| `skeleton_infer_ms` | `float` | `float` | GPU + IPC inference latency for this frame (ms) |
| `color_bgrx` | `const uint8_t*` | `ndarray (1080, 1920, 4)` `uint8` or `None` | 1080p color image in `B, G, R, X` channel order |
| `depth_mm` | `const uint16_t*` | `ndarray (424, 512)` `uint16` or `None` | Metric depth in millimeters (`0` = invalid pixel) |
| `ir` | `const uint16_t*` | `ndarray (424, 512)` `uint16` or `None` | Active infrared amplitude (`0..65535`) |
| `registered_bgrx` | `const uint8_t*` | `ndarray (424, 512, 4)` `uint8` or `None` | Color pixels warped onto the `512x424` depth grid |
| `undistorted_depth_mm` | `const float*` | `ndarray (424, 512)` `float32` or `None` | Lens-undistorted depth in millimeters |
| `body_index` | `const uint8_t*` | `ndarray (424, 512)` `uint8` or `None` | Per-pixel body slot (`0..5` = person, `255` = background) |
| `floor_clip_plane` | `float[4]` | `ndarray (4,)` `float32` | Floor plane `(a, b, c, d)` where `aX + bY + cZ + d = 0` (`d` = camera height in meters) |
| `up_vector` | `float[4]` | `ndarray (4,)` `float32` | Gravity/up vector reported by the tracker |
| `body_count` / `bodies` | `uint32_t` + `Kv2Body[6]` | `List[BodyData]` (`len <= 6`) | Active tracked human bodies in this frame |

### 3.2 Tracked Human Body: `Kv2Body` & `Kv2Joint` (C/C++) / `BodyData` (Python)

| Concept | C Struct Field | Python `BodyData` Attribute | Shape & Type | Description |
| :--- | :--- | :--- | :--- | :--- |
| Tracking ID | `Kv2Body.tracking_id` | `tracking_id` | `uint64` / `int` | Persistent ID assigned to a tracked subject |
| Body Slot | `Kv2Body.slot_index` | `slot_index` | `uint8` / `int` (`0..5`) | Matches pixel values `0..5` in `body_index` |
| Clipped Edges | `Kv2Body.clipped_edges` | `clipped_edges` | `uint32` / `int` | Bitmask (`1=Right, 2=Left, 4=Top, 8=Bottom`) |
| 3D Positions | `joints[j].pos[3]` | `joint_positions` | `(25, 3)` `float32` | `(X, Y, Z)` Camera Space coordinates in meters |
| Orientations | `joints[j].quat[4]` | `joint_orientations` | `(25, 4)` `float32` | `(x, y, z, w)` unit quaternion per joint |
| Depth 2D UV | `joints[j].depth_uv[2]` | `joint_depth_uv` | `(25, 2)` `float32` | Pixel `(u, v)` projected onto `512x424` Depth/IR |
| Color 2D UV | `joints[j].color_uv[2]` | `joint_color_uv` | `(25, 2)` `float32` | Pixel `(u, v)` projected onto `1920x1080` RGB |
| Tracking State | `joints[j].tracking_state` | `joint_tracking_states` | `(25,)` `uint32` | `0=NotTracked`, `1=Inferred`, `2=Tracked` |

### 3.3 Camera Intrinsics: `Kv2IrIntrinsics` / `IrIntrinsics` & `Kv2ColorIntrinsics` / `ColorIntrinsics`
- **`IrIntrinsics`**: `fx, fy, cx, cy` (pixels on `512x424`), radial distortion `k1, k2, k3`, tangential distortion `p1, p2` (read from Kinect v2 factory EEPROM).
- **`ColorIntrinsics`**: `fx, fy, cx, cy` (pixels on `1920x1080`).

### 3.4 Offline / Array Body Tracker: `KinectV2BodyTracker` (Python)
For offline `.npz` / ROS bag processing without opening a physical USB Kinect v2:
- `tracker.process_frame(depth_mm, ir_frame=None, want_body_index=False) -> BodyFrameResult`
- `BodyFrameResult`: `frame_index`, `status`, `floor_clip_plane (4,)`, `up_vector (4,)`, `bodies: List[TrackedBody]`, `body_index_map: Optional[ndarray (424, 512) uint8]`.

---

## 4. 25-Joint Index Table (`JointType` / `JOINT_NAMES`)

| Index | C Enum | Python `JointType` | Name | Parent Index |
| :---: | :--- | :--- | :--- | :---: |
| `0` | `KV2_JOINT_SPINE_BASE` | `SPINE_BASE` | `SpineBase` | Root |
| `1` | `KV2_JOINT_SPINE_MID` | `SPINE_MID` | `SpineMid` | `0` |
| `2` | `KV2_JOINT_NECK` | `NECK` | `Neck` | `20` |
| `3` | `KV2_JOINT_HEAD` | `HEAD` | `Head` | `2` |
| `4` | `KV2_JOINT_SHOULDER_LEFT` | `SHOULDER_LEFT` | `ShoulderLeft` | `20` |
| `5` | `KV2_JOINT_ELBOW_LEFT` | `ELBOW_LEFT` | `ElbowLeft` | `4` |
| `6` | `KV2_JOINT_WRIST_LEFT` | `WRIST_LEFT` | `WristLeft` | `5` |
| `7` | `KV2_JOINT_HAND_LEFT` | `HAND_LEFT` | `HandLeft` | `6` |
| `8` | `KV2_JOINT_SHOULDER_RIGHT` | `SHOULDER_RIGHT` | `ShoulderRight` | `20` |
| `9` | `KV2_JOINT_ELBOW_RIGHT` | `ELBOW_RIGHT` | `ElbowRight` | `8` |
| `10` | `KV2_JOINT_WRIST_RIGHT` | `WRIST_RIGHT` | `WristRight` | `9` |
| `11` | `KV2_JOINT_HAND_RIGHT` | `HAND_RIGHT` | `HandRight` | `10` |
| `12` | `KV2_JOINT_HIP_LEFT` | `HIP_LEFT` | `HipLeft` | `0` |
| `13` | `KV2_JOINT_KNEE_LEFT` | `KNEE_LEFT` | `KneeLeft` | `12` |
| `14` | `KV2_JOINT_ANKLE_LEFT` | `ANKLE_LEFT` | `AnkleLeft` | `13` |
| `15` | `KV2_JOINT_FOOT_LEFT` | `FOOT_LEFT` | `FootLeft` | `14` |
| `16` | `KV2_JOINT_HIP_RIGHT` | `HIP_RIGHT` | `HipRight` | `0` |
| `17` | `KV2_JOINT_KNEE_RIGHT` | `KNEE_RIGHT` | `KneeRight` | `16` |
| `18` | `KV2_JOINT_ANKLE_RIGHT` | `ANKLE_RIGHT` | `AnkleRight` | `17` |
| `19` | `KV2_JOINT_FOOT_RIGHT` | `FOOT_RIGHT` | `FootRight` | `18` |
| `20` | `KV2_JOINT_SPINE_SHOULDER` | `SPINE_SHOULDER` | `SpineShoulder` | `1` |
| `21` | `KV2_JOINT_HAND_TIP_LEFT` | `HAND_TIP_LEFT` | `HandTipLeft` | `7` |
| `22` | `KV2_JOINT_THUMB_LEFT` | `THUMB_LEFT` | `ThumbLeft` | `6` |
| `23` | `KV2_JOINT_HAND_TIP_RIGHT` | `HAND_TIP_RIGHT` | `HandTipRight` | `11` |
| `24` | `KV2_JOINT_THUMB_RIGHT` | `THUMB_RIGHT` | `ThumbRight` | `10` |

Note: Index `19` is `FootRight` (parent `18`, `AnkleRight`). All 24 parent-child bone pairs are exported in Python as `kinectv2_sdk.BONE_CONNECTIONS`.
