# 資料結構與座標系說明文件 (ZH-TW)

[English](data_structures.en.md) | [繁體中文](data_structures.zh-TW.md) | [简体中文](data_structures.zh-CN.md)

本文件說明 `kinectv2-sdk` 的三維與二維座標慣例、C/C++ 標頭結構（`include/kinectv2_sdk.h`）以及 Python 零拷貝資料型別（`kinectv2_sdk`）。

---

## 1. 座標系統與物理單位

### 1.1 三維相機座標系 Camera Space（`pos` / `joint_positions`）
所有 3D 關節位置與反投影點皆以 **公尺（`float32`）** 為單位，原點 `(0, 0, 0)` 位於 Kinect v2 紅外線／深度感測器光學中心：
- **`+X`（公尺）**：水平軸，在預設自拍鏡像模式（`mirror_skeleton = 0`）下指向畫面右側（與像素行座標 `u = 0..511` 同向）。
- **`+Y`（公尺）**：垂直軸，指向**正上方**（與重力方向相反）。
- **`+Z`（公尺）**：光軸深度方向，指向鏡頭**正前方**（有效原生骨架追蹤距離為 `1.45 m .. 3.8 m`；啟動或呼叫 `reset_skeleton()` 後的前 `1..15` 幀為內部背景模型初始化期）。

### 1.2 關節旋轉四元數（`quat` / `joint_orientations`）
- 單位四元數 **`(x, y, z, w)`**（`float32`），表示骨骼沿運動學父子鏈在相機座標系下的朝向，與微軟官方 SDK `JointOrientation.Orientation` 完全一致；無扭轉定義之末端關節回傳 `(0, 0, 0, 0)`。

### 1.3 二維影像像素座標系
| 影像空間 | 解析度 | 原點 `(0, 0)` | 預先計算之關節投影欄位 |
| :--- | :--- | :--- | :--- |
| 深度圖／紅外線／人體遮罩 | `512 x 424` | 左上角像素（`u` 向右 `0..511`，`v` 向下 `0..423`） | `Kv2Joint.depth_uv` / `BodyData.joint_depth_uv` |
| 彩色影像（RGB） | `1920 x 1080` | 左上角像素（`u` 向右 `0..1919`，`v` 向下 `0..1079`） | `Kv2Joint.color_uv` / `BodyData.joint_color_uv` |

未追蹤之關節（`tracking_state == 0`）其二維投影座標固定填入 `(-1e6, -1e6)`。

---

## 2. 列舉常數與串流旗標

### 2.1 深度解包管線 `Kv2PipelineType` / `kinectv2_sdk.PipelineType`
| 數值 | C 列舉名稱 | Python 列舉 | 說明 |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_PIPELINE_AUTO` | `PipelineType.AUTO` | 自動依序嘗試 `CUDA` -> `OpenCL` -> `CPU` |
| `1` | `KV2_PIPELINE_CUDA` | `PipelineType.CUDA` | NVIDIA CUDA 硬體深度解算管線 |
| `2` | `KV2_PIPELINE_OPENCL` | `PipelineType.OPENCL` | OpenCL 1.2+ 深度解算管線 |
| `3` | `KV2_PIPELINE_CPU` | `PipelineType.CPU` | 純 CPU 解算管線 |

### 2.2 串流旗標遮罩 `Kv2StreamFlags` / `kinectv2_sdk.StreamFlags`
| 十六進位 | C 常數 | Python 旗標 | 啟用之資料緩衝區 |
| :---: | :--- | :--- | :--- |
| `0x01` | `KV2_STREAM_DEPTH` | `StreamFlags.DEPTH` | `depth_mm`（`512x424 uint16` 毫米深度） |
| `0x02` | `KV2_STREAM_IR` | `StreamFlags.IR` | `ir`（`512x424 uint16` 紅外線振幅） |
| `0x04` | `KV2_STREAM_COLOR` | `StreamFlags.COLOR` | `color_bgrx`（`1920x1080x4 uint8` BGRX 彩色圖） |
| `0x08` | `KV2_STREAM_BODY` | `StreamFlags.BODY` | `bodies`（25 關節 3D 人體骨架） |
| `0x10` | `KV2_STREAM_BODY_INDEX` | `StreamFlags.BODY_INDEX` | `body_index`（`512x424 uint8` 人體像素遮罩） |
| `0x20` | `KV2_STREAM_REGISTERED_COLOR` | `StreamFlags.REGISTERED_COLOR` | `registered_bgrx` 與 `undistorted_depth_mm` |
| `0x1F` | `KV2_STREAM_DEFAULT` | `StreamFlags.DEFAULT` | 預設開啟除 `REGISTERED_COLOR` 外的五項串流 |
| `0x3F` | `KV2_STREAM_ALL` | `StreamFlags.ALL` | 開啟全部六項串流 |

### 2.3 關節追蹤狀態 `Kv2TrackingState` / `kinectv2_sdk.TrackingState`
| 數值 | C 列舉名稱 | Python 列舉 | 意義 |
| :---: | :--- | :--- | :--- |
| `0` | `KV2_TRACKING_NOT_TRACKED` | `TrackingState.NOT_TRACKED` | 關節未被追蹤 |
| `1` | `KV2_TRACKING_INFERRED` | `TrackingState.INFERRED` | 關節受遮擋，位置由運動學推算 |
| `2` | `KV2_TRACKING_TRACKED` | `TrackingState.TRACKED` | 關節直接可見且高信心度追蹤 |

---

## 3. 核心資料結構對照（C/C++ 與 Python）

### 3.1 單幀視圖：`Kv2FrameView`（C/C++）／`FrameGuard`（Python）
由 `kv2_wait_frame()` 或 `sensor.wait_frame()` 取得。所有影像指標與 `numpy` 陣列皆為指向 C++ 4 槽固定緩衝池的**零拷貝唯讀視圖**，有效期間至呼叫 `kv2_release_frame()` 或離開 Python `with frame:` 區塊為止（若需跨幀保留請在區塊內呼叫 `.copy()`）。

| 欄位名稱 | C 型別 | Python 型別與形狀（`dtype`） | 說明 |
| :--- | :--- | :--- | :--- |
| `frame_seq` | `uint64_t` | `int` | 單調遞增之幀序號（自 `1` 起算） |
| `timestamp_sec` | `double` | `float` | 單調時鐘時間戳記（秒） |
| `sensor_fps` | `float` | `float` | 近 30 幀感測器擷取幀率（Hz） |
| `skeleton_fps` | `float` | `float` | 近 30 幀骨架推論幀率（Hz） |
| `skeleton_infer_ms` | `float` | `float` | 該幀骨架 GPU + IPC 推論耗時（毫秒） |
| `color_bgrx` | `const uint8_t*` | `ndarray (1080, 1920, 4)` `uint8` 或 `None` | 1080p 彩色影像（通道順序 `B, G, R, X`） |
| `depth_mm` | `const uint16_t*` | `ndarray (424, 512)` `uint16` 或 `None` | 毫米級深度圖（`0` 代表無效深度） |
| `ir` | `const uint16_t*` | `ndarray (424, 512)` `uint16` 或 `None` | 紅外線振幅圖（`0..65535`） |
| `registered_bgrx` | `const uint8_t*` | `ndarray (424, 512, 4)` `uint8` 或 `None` | 對齊至 `512x424` 深度網格的彩色影像 |
| `undistorted_depth_mm` | `const float*` | `ndarray (424, 512)` `float32` 或 `None` | 去鏡頭畸變後的毫米深度圖 |
| `body_index` | `const uint8_t*` | `ndarray (424, 512)` `uint8` 或 `None` | 人體分割遮罩（`0..5` 為人體編號，`255` 為背景） |
| `floor_clip_plane` | `float[4]` | `ndarray (4,)` `float32` | 地板平面方程 `(a, b, c, d)`，滿足 `aX + bY + cZ + d = 0`（`d` 為鏡頭離地高度公尺數） |
| `up_vector` | `float[4]` | `ndarray (4,)` `float32` | 重力／向上單位向量 |
| `body_count` / `bodies` | `uint32_t` + `Kv2Body[6]` | `List[BodyData]`（長度 `<= 6`） | 當前幀追蹤到的人體清單 |

### 3.2 人體與關節結構：`Kv2Body`、`Kv2Joint`（C/C++）／`BodyData`（Python）

| 項目 | C 結構欄位 | Python `BodyData` 屬性 | 形狀與型別 | 說明 |
| :--- | :--- | :--- | :--- | :--- |
| 追蹤識別碼 | `Kv2Body.tracking_id` | `tracking_id` | `uint64` / `int` | 持續追蹤同一位受測者的識別碼 |
| 人體槽位 | `Kv2Body.slot_index` | `slot_index` | `uint8` / `int` (`0..5`) | 對應 `body_index` 遮罩中的像素值 `0..5` |
| 邊界裁切遮罩 | `Kv2Body.clipped_edges` | `clipped_edges` | `uint32` / `int` | 畫面邊緣裁切旗標（`1=右, 2=左, 4=上, 8=下`） |
| 3D 關節座標 | `joints[j].pos[3]` | `joint_positions` | `(25, 3)` `float32` | 相機空間 `(X, Y, Z)` 座標（公尺） |
| 關節四元數 | `joints[j].quat[4]` | `joint_orientations` | `(25, 4)` `float32` | 關節朝向單位四元數 `(x, y, z, w)` |
| 深度圖 2D 座標 | `joints[j].depth_uv[2]` | `joint_depth_uv` | `(25, 2)` `float32` | 投影至 `512x424` 深度／紅外線圖之 `(u, v)` |
| 彩色圖 2D 座標 | `joints[j].color_uv[2]` | `joint_color_uv` | `(25, 2)` `float32` | 投影至 `1920x1080` 彩色圖之 `(u, v)` |
| 關節追蹤狀態 | `joints[j].tracking_state` | `joint_tracking_states` | `(25,)` `uint32` | `0=未追蹤`、`1=推算`、`2=已追蹤` |

### 3.3 相機內參：`Kv2IrIntrinsics` / `IrIntrinsics` 與 `Kv2ColorIntrinsics` / `ColorIntrinsics`
- **`IrIntrinsics`**：讀取自感測器出廠韌體之 `fx, fy, cx, cy`（對應 `512x424`）及徑向畸變 `k1, k2, k3`、切向畸變 `p1, p2`。
- **`ColorIntrinsics`**：彩色鏡頭內參 `fx, fy, cx, cy`（對應 `1920x1080`）。

### 3.4 離線深度陣列追蹤器：`KinectV2BodyTracker`（Python）
在無實體感測器連線時，可直接輸入預錄的 `512x424` 毫米深度陣列進行骨架推論：
- `tracker.process_frame(depth_mm, ir_frame=None, want_body_index=False) -> BodyFrameResult`
- 回傳 `BodyFrameResult`：包含 `frame_index`、`status`、`floor_clip_plane (4,)`、`up_vector (4,)`、`bodies: List[TrackedBody]` 與選用之 `body_index_map (424, 512) uint8`。

---

## 4. 25 關節索引對照表（`JointType` / `JOINT_NAMES`）

| 索引 | C 列舉名稱 | Python `JointType` | 英文名稱 | 中文部位 | 父關節索引 |
| :---: | :--- | :--- | :--- | :--- | :---: |
| `0` | `KV2_JOINT_SPINE_BASE` | `SPINE_BASE` | `SpineBase` | 脊椎底部（骨盆中心） | 根節點 |
| `1` | `KV2_JOINT_SPINE_MID` | `SPINE_MID` | `SpineMid` | 脊椎中段 | `0` |
| `2` | `KV2_JOINT_NECK` | `NECK` | `Neck` | 頸部 | `20` |
| `3` | `KV2_JOINT_HEAD` | `HEAD` | `Head` | 頭部 | `2` |
| `4` | `KV2_JOINT_SHOULDER_LEFT` | `SHOULDER_LEFT` | `ShoulderLeft` | 左肩 | `20` |
| `5` | `KV2_JOINT_ELBOW_LEFT` | `ELBOW_LEFT` | `ElbowLeft` | 左肘 | `4` |
| `6` | `KV2_JOINT_WRIST_LEFT` | `WRIST_LEFT` | `WristLeft` | 左腕 | `5` |
| `7` | `KV2_JOINT_HAND_LEFT` | `HAND_LEFT` | `HandLeft` | 左手掌 | `6` |
| `8` | `KV2_JOINT_SHOULDER_RIGHT` | `SHOULDER_RIGHT` | `ShoulderRight` | 右肩 | `20` |
| `9` | `KV2_JOINT_ELBOW_RIGHT` | `ELBOW_RIGHT` | `ElbowRight` | 右肘 | `8` |
| `10` | `KV2_JOINT_WRIST_RIGHT` | `WRIST_RIGHT` | `WristRight` | 右腕 | `9` |
| `11` | `KV2_JOINT_HAND_RIGHT` | `HAND_RIGHT` | `HandRight` | 右手掌 | `10` |
| `12` | `KV2_JOINT_HIP_LEFT` | `HIP_LEFT` | `HipLeft` | 左髖 | `0` |
| `13` | `KV2_JOINT_KNEE_LEFT` | `KNEE_LEFT` | `KneeLeft` | 左膝 | `12` |
| `14` | `KV2_JOINT_ANKLE_LEFT` | `ANKLE_LEFT` | `AnkleLeft` | 左踝 | `13` |
| `15` | `KV2_JOINT_FOOT_LEFT` | `FOOT_LEFT` | `FootLeft` | 左腳掌 | `14` |
| `16` | `KV2_JOINT_HIP_RIGHT` | `HIP_RIGHT` | `HipRight` | 右髖 | `0` |
| `17` | `KV2_JOINT_KNEE_RIGHT` | `KNEE_RIGHT` | `KneeRight` | 右膝 | `16` |
| `18` | `KV2_JOINT_ANKLE_RIGHT` | `ANKLE_RIGHT` | `AnkleRight` | 右踝 | `17` |
| `19` | `KV2_JOINT_FOOT_RIGHT` | `FOOT_RIGHT` | `FootRight` | 右腳掌 | `18` |
| `20` | `KV2_JOINT_SPINE_SHOULDER` | `SPINE_SHOULDER` | `SpineShoulder` | 鎖骨／肩部脊椎中心 | `1` |
| `21` | `KV2_JOINT_HAND_TIP_LEFT` | `HAND_TIP_LEFT` | `HandTipLeft` | 左手指尖 | `7` |
| `22` | `KV2_JOINT_THUMB_LEFT` | `THUMB_LEFT` | `ThumbLeft` | 左拇指 | `6` |
| `23` | `KV2_JOINT_HAND_TIP_RIGHT` | `HAND_TIP_RIGHT` | `HandTipRight` | 右手指尖 | `11` |
| `24` | `KV2_JOINT_THUMB_RIGHT` | `THUMB_RIGHT` | `ThumbRight` | 右拇指 | `10` |

Python 端可直接匯入 `kinectv2_sdk.BONE_CONNECTIONS` 取得全部 24 組骨架連線索引對。
