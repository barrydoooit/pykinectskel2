# 数据结构与坐标系说明文档 (ZH-CN)

[English](data_structures.en.md) | [繁體中文](data_structures.zh-TW.md) | [简体中文](data_structures.zh-CN.md)

本文档说明 `kinectv2-sdk` 的三维与二维坐标约定、C/C++ 头文件结构体（`include/kinectv2_sdk.h`）以及 Python 零拷贝数据类型（`kinectv2_sdk`）。

---

## 1. 坐标系统与物理单位

### 1.1 三维相机坐标系 Camera Space（`pos` / `joint_positions`）
所有 3D 关节位置与反投影点均以 **米（`float32`）** 为单位，原点 `(0, 0, 0)` 位于 Kinect v2 红外／深度传感器光学中心：
- **`+X`（米）**：水平轴，在默认自拍镜像模式（`mirror_skeleton = 0`）下指向画面右侧（与像素列坐标 `u = 0..511` 同向）。
- **`+Y`（米）**：垂直轴，指向**正上方**（与重力方向相反）。
- **`+Z`（米）**：光轴深度方向，指向镜头**正前方**（原生骨架追踪有效距离为 `1.45 m .. 3.8 m`；启动或调用 `reset_skeleton()` 后的前 `1..15` 帧为内部背景模型初始化期）。

### 1.2 关节旋转四元数（`quat` / `joint_orientations`）
- 单位四元数 **`(x, y, z, w)`**（`float32`），表示骨骼沿运动学父子链在相机坐标系下的朝向，与微软官方 SDK `JointOrientation.Orientation` 完全一致；无扭转定义的末端关节返回 `(0, 0, 0, 0)`。

### 1.3 二维图像像素坐标系
| 图像空间 | 分辨率 | 原点 `(0, 0)` | 预计算的关节投影字段 |
| :--- | :--- | :--- | :--- |
| 深度图／红外图／人体掩码 | `512 x 424` | 左上角像素（`u` 向右 `0..511`，`v` 向下 `0..423`） | `Kv2Joint.depth_uv` / `BodyData.joint_depth_uv` |
| 彩色图像（RGB） | `1920 x 1080` | 左上角像素（`u` 向右 `0..1919`，`v` 向下 `0..1079`） | `Kv2Joint.color_uv` / `BodyData.joint_color_uv` |

未追踪的关节（`tracking_state == 0`）其二维投影坐标固定填入 `(-1e6, -1e6)`。

---

## 2. 枚举常量与数据流掩码

### 2.1 深度解包管线 `Kv2PipelineType` / `kinectv2_sdk.PipelineType`
| 数值 | C 枚举名称 | Python 枚举 | 说明 |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_PIPELINE_AUTO` | `PipelineType.AUTO` | 自动依次尝试 `CUDA` -> `OpenCL` -> `CPU` |
| `1` | `KV2_PIPELINE_CUDA` | `PipelineType.CUDA` | NVIDIA CUDA 硬件深度解算管线 |
| `2` | `KV2_PIPELINE_OPENCL` | `PipelineType.OPENCL` | OpenCL 1.2+ 深度解算管线 |
| `3` | `KV2_PIPELINE_CPU` | `PipelineType.CPU` | 纯 CPU 解算管线 |

### 2.2 数据流掩码 `Kv2StreamFlags` / `kinectv2_sdk.StreamFlags`
| 十六进制 | C 常量 | Python 掩码 | 启用的数据缓冲区 |
| :---: | :--- | :--- | :--- |
| `0x01` | `KV2_STREAM_DEPTH` | `StreamFlags.DEPTH` | `depth_mm`（`512x424 uint16` 毫米深度） |
| `0x02` | `KV2_STREAM_IR` | `StreamFlags.IR` | `ir`（`512x424 uint16` 红外振幅） |
| `0x04` | `KV2_STREAM_COLOR` | `StreamFlags.COLOR` | `color_bgrx`（`1920x1080x4 uint8` BGRX 彩色图） |
| `0x08` | `KV2_STREAM_BODY` | `StreamFlags.BODY` | `bodies`（25 关节 3D 人体骨架） |
| `0x10` | `KV2_STREAM_BODY_INDEX` | `StreamFlags.BODY_INDEX` | `body_index`（`512x424 uint8` 人体像素掩码） |
| `0x20` | `KV2_STREAM_REGISTERED_COLOR` | `StreamFlags.REGISTERED_COLOR` | `registered_bgrx` 与 `undistorted_depth_mm` |
| `0x1F` | `KV2_STREAM_DEFAULT` | `StreamFlags.DEFAULT` | 默认开启除 `REGISTERED_COLOR` 外的五路数据流 |
| `0x3F` | `KV2_STREAM_ALL` | `StreamFlags.ALL` | 开启全部六路数据流 |

### 2.3 关节追踪状态 `Kv2TrackingState` / `kinectv2_sdk.TrackingState`
| 数值 | C 枚举名称 | Python 枚举 | 含义 |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_TRACKING_NOT_TRACKED` | `TrackingState.NOT_TRACKED` | 关节未被追踪 |
| `1` | `KV2_TRACKING_INFERRED` | `TrackingState.INFERRED` | 关节被遮挡，位置由运动学推算 |
| `2` | `KV2_TRACKING_TRACKED` | `TrackingState.TRACKED` | 关节直接可见且高置信度追踪 |

---

## 3. 核心数据结构对照（C/C++ 与 Python）

### 3.1 单帧视图：`Kv2FrameView`（C/C++）／`FrameGuard`（Python）
由 `kv2_wait_frame()` 或 `sensor.wait_frame()` 获取。所有图像指针与 `numpy` 数组均为指向 C++ 4 槽固定缓冲池的**零拷贝只读视图**，生命周期在调用 `kv2_release_frame()` 或退出 Python `with frame:` 代码块时结束（如需跨帧保存请在代码块内调用 `.copy()`）。

| 字段名称 | C 类型 | Python 类型与形状（`dtype`） | 说明 |
| :--- | :--- | :--- | :--- |
| `frame_seq` | `uint64_t` | `int` | 单调递增的帧序号（从 `1` 开始） |
| `timestamp_sec` | `double` | `float` | 单调时钟时间戳（秒） |
| `sensor_fps` | `float` | `float` | 最近 30 帧传感器采集帧率（Hz） |
| `skeleton_fps` | `float` | `float` | 最近 30 帧骨架推理帧率（Hz） |
| `skeleton_infer_ms` | `float` | `float` | 当前帧骨架 GPU + IPC 推理耗时（毫秒） |
| `color_bgrx` | `const uint8_t*` | `ndarray (1080, 1920, 4)` `uint8` 或 `None` | 1080p 彩色图像（通道顺序 `B, G, R, X`） |
| `depth_mm` | `const uint16_t*` | `ndarray (424, 512)` `uint16` 或 `None` | 毫米级深度图（`0` 表示无效深度） |
| `ir` | `const uint16_t*` | `ndarray (424, 512)` `uint16` 或 `None` | 红外振幅图（`0..65535`） |
| `registered_bgrx` | `const uint8_t*` | `ndarray (424, 512, 4)` `uint8` 或 `None` | 配准到 `512x424` 深度网格的彩色图像 |
| `undistorted_depth_mm` | `const float*` | `ndarray (424, 512)` `float32` 或 `None` | 去镜头畸变后的毫米深度图 |
| `body_index` | `const uint8_t*` | `ndarray (424, 512)` `uint8` 或 `None` | 人体分割掩码（`0..5` 为人体编号，`255` 为背景） |
| `floor_clip_plane` | `float[4]` | `ndarray (4,)` `float32` | 地面平面方程 `(a, b, c, d)`，满足 `aX + bY + cZ + d = 0`（`d` 为镜头离地高度米数） |
| `up_vector` | `float[4]` | `ndarray (4,)` `float32` | 重力／向上单位向量 |
| `body_count` / `bodies` | `uint32_t` + `Kv2Body[6]` | `List[BodyData]`（长度 `<= 6`） | 当前帧追踪到的人体列表 |

### 3.2 人体与关节结构：`Kv2Body`、`Kv2Joint`（C/C++）／`BodyData`（Python）

| 项目 | C 结构体字段 | Python `BodyData` 属性 | 形状与类型 | 说明 |
| :--- | :--- | :--- | :--- | :--- |
| 追踪 ID | `Kv2Body.tracking_id` | `tracking_id` | `uint64` / `int` | 持续追踪同一受试者的唯一标识符 |
| 人体槽位 | `Kv2Body.slot_index` | `slot_index` | `uint8` / `int` (`0..5`) | 对应 `body_index` 掩码中的像素值 `0..5` |
| 边界裁切掩码 | `Kv2Body.clipped_edges` | `clipped_edges` | `uint32` / `int` | 画面边缘裁切标志（`1=右, 2=左, 4=上, 8=下`） |
| 3D 关节坐标 | `joints[j].pos[3]` | `joint_positions` | `(25, 3)` `float32` | 相机空间 `(X, Y, Z)` 坐标（米） |
| 关节四元数 | `joints[j].quat[4]` | `joint_orientations` | `(25, 4)` `float32` | 关节朝向单位四元数 `(x, y, z, w)` |
| 深度图 2D 坐标 | `joints[j].depth_uv[2]` | `joint_depth_uv` | `(25, 2)` `float32` | 投影到 `512x424` 深度／红外图的 `(u, v)` |
| 彩色图 2D 坐标 | `joints[j].color_uv[2]` | `joint_color_uv` | `(25, 2)` `float32` | 投影到 `1920x1080` 彩色图的 `(u, v)` |
| 关节追踪状态 | `joints[j].tracking_state` | `joint_tracking_states` | `(25,)` `uint32` | `0=未追踪`、`1=推算`、`2=已追踪` |

### 3.3 相机内参：`Kv2IrIntrinsics` / `IrIntrinsics` 与 `Kv2ColorIntrinsics` / `ColorIntrinsics`
- **`IrIntrinsics`**：从传感器出厂固件读取的 `fx, fy, cx, cy`（对应 `512x424`）及径向畸变 `k1, k2, k3`、切向畸变 `p1, p2`。
- **`ColorIntrinsics`**：彩色镜头内参 `fx, fy, cx, cy`（对应 `1920x1080`）。

### 3.4 离线深度数组追踪器：`KinectV2BodyTracker`（Python）
无需连接物理 USB 传感器，可直接输入预录的 `512x424` 毫米深度数组进行骨架推理：
- `tracker.process_frame(depth_mm, ir_frame=None, want_body_index=False) -> BodyFrameResult`
- 返回 `BodyFrameResult`：包含 `frame_index`、`status`、`floor_clip_plane (4,)`、`up_vector (4,)`、`bodies: List[TrackedBody]` 与可选 `body_index_map (424, 512) uint8`。

---

## 4. 25 关节索引对照表（`JointType` / `JOINT_NAMES`）

| 索引 | C 枚举名称 | Python `JointType` | 英文名称 | 中文部位 | 父关节索引 |
| :---: | :--- | :--- | :--- | :--- | :---: |
| `0` | `KV2_JOINT_SPINE_BASE` | `SPINE_BASE` | `SpineBase` | 脊柱底部（骨盆中心） | 根节点 |
| `1` | `KV2_JOINT_SPINE_MID` | `SPINE_MID` | `SpineMid` | 脊柱中段 | `0` |
| `2` | `KV2_JOINT_NECK` | `NECK` | `Neck` | 颈部 | `20` |
| `3` | `KV2_JOINT_HEAD` | `HEAD` | `Head` | 头部 | `2` |
| `4` | `KV2_JOINT_SHOULDER_LEFT` | `SHOULDER_LEFT` | `ShoulderLeft` | 左肩 | `20` |
| `5` | `KV2_JOINT_ELBOW_LEFT` | `ELBOW_LEFT` | `ElbowLeft` | 左肘 | `4` |
| `6` | `KV2_JOINT_WRIST_LEFT` | `WRIST_LEFT` | `WristLeft` | 左腕 | `5` |
| `7` | `KV2_JOINT_HAND_LEFT` | `HAND_LEFT` | `HandLeft` | 左手掌 | `6` |
| `8` | `KV2_JOINT_SHOULDER_RIGHT` | `SHOULDER_RIGHT` | `ShoulderRight` | 右肩 | `20` |
| `9` | `KV2_JOINT_ELBOW_RIGHT` | `ELBOW_RIGHT` | `ElbowRight` | 右肘 | `8` |
| `10` | `KV2_JOINT_WRIST_RIGHT` | `WRIST_RIGHT` | `WristRight` | 右腕 | `9` |
| `11` | `KV2_JOINT_HAND_RIGHT` | `HAND_RIGHT` | `HandRight` | 右手掌 | `10` |
| `12` | `KV2_JOINT_HIP_LEFT` | `HIP_LEFT` | `HipLeft` | 左髋 | `0` |
| `13` | `KV2_JOINT_KNEE_LEFT` | `KNEE_LEFT` | `KneeLeft` | 左膝 | `12` |
| `14` | `KV2_JOINT_ANKLE_LEFT` | `ANKLE_LEFT` | `AnkleLeft` | 左踝 | `13` |
| `15` | `KV2_JOINT_FOOT_LEFT` | `FOOT_LEFT` | `FootLeft` | 左脚掌 | `14` |
| `16` | `KV2_JOINT_HIP_RIGHT` | `HIP_RIGHT` | `HipRight` | 右髋 | `0` |
| `17` | `KV2_JOINT_KNEE_RIGHT` | `KNEE_RIGHT` | `KneeRight` | 右膝 | `16` |
| `18` | `KV2_JOINT_ANKLE_RIGHT` | `ANKLE_RIGHT` | `AnkleRight` | 右踝 | `17` |
| `19` | `KV2_JOINT_FOOT_RIGHT` | `FOOT_RIGHT` | `FootRight` | 右脚掌 | `18` |
| `20` | `KV2_JOINT_SPINE_SHOULDER` | `SPINE_SHOULDER` | `SpineShoulder` | 锁骨／肩部脊柱中心 | `1` |
| `21` | `KV2_JOINT_HAND_TIP_LEFT` | `HAND_TIP_LEFT` | `HandTipLeft` | 左手指尖 | `7` |
| `22` | `KV2_JOINT_THUMB_LEFT` | `THUMB_LEFT` | `ThumbLeft` | 左拇指 | `6` |
| `23` | `KV2_JOINT_HAND_TIP_RIGHT` | `HAND_TIP_RIGHT` | `HandTipRight` | 右手指尖 | `11` |
| `24` | `KV2_JOINT_THUMB_RIGHT` | `THUMB_RIGHT` | `ThumbRight` | 右拇指 | `10` |

Python 端可直接导入 `kinectv2_sdk.BONE_CONNECTIONS` 获取全部 24 组骨架连线索引对。
