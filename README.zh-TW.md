# pykinectskel2 — Kinect v2 Linux C/C++ 與 Python 3D 骨架開發套件

[English](README.md) | [繁體中文](README.zh-TW.md) | [简体中文](README.zh-CN.md)

面向 **Microsoft Kinect v2** 的低消耗 Linux 原生 C/C++17 與零拷貝 Python 開發套件，提供穩定 **30.0 FPS** 的 `1920x1080` RGB、`512x424` 毫米級深度圖、紅外線圖、對齊彩色深度圖（Registered RGB-D）、`BodyIndex` 人體遮罩，以及**與 Windows 官方 SDK v2.0_1409 完全一致的 25 關節 3D 骨架（誤差 `< 1 mm`，單幀 GPU 推論約 `1.6 ms`）**。

- **資料結構與座標系說明文件**：[English](docs/data_structures.en.md) | [繁體中文](docs/data_structures.zh-TW.md) | [简体中文](docs/data_structures.zh-CN.md)

---

## 專案目錄結構

```text
├── CMakeLists.txt                   # 頂層 CMake 建置腳本（編譯動態庫、視覺化工具與 C++ 範例）
├── pyproject.toml                   # Python 套件定義（支援 `pip install -e .`）
├── LICENSE / NOTICE                 # Apache-2.0 開源授權條款與第三方元件聲明
├── include/kinectv2_sdk.h           # C 語言介面（`kv2_*`）與 C++17 封裝（`kv2::Device`）
├── src/kinectv2_sdk.cpp             # C++17 零拷貝多模態執行核心
├── src/tools/kinectv2_visualizer.cpp# 30 FPS SDL2 + OpenCV 四分割桌面即時視覺化工具
├── native/                          # `vvtechs_worker.c` 推論宿主與 `k4wcll_stub.c` 空殼原始碼
├── patches/libfreenect2_linux_sdk.patch # OpenCL 3.0 修正與 `close()` 關閉白燈修補檔
├── python/kinectv2_sdk/             # Python 零拷貝套件（`KinectV2Sensor`、`KinectV2BodyTracker`）
├── scripts/
│   ├── build_libfreenect2.sh        # 自動套用修補檔並編譯安裝 libfreenect2
│   ├── stage_linux_bundle.py        # 於 Windows 從官方 SDK v2.0_1409 打包執行時期檔案
│   └── launch_visualizer.sh         # 啟動 30 FPS 本機視覺化視窗
├── examples/                        # `python_sdk_demo.py` 與 `cpp_sdk_demo.cpp` 最小範例
└── tests/                           # 離線黃金基準一致性驗證腳本與基準測試集
```

---

## 完整編譯與安裝教學（Linux x86_64）

### 1. 安裝系統基礎依賴與 USB 權限規則

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config git \
    libusb-1.0-0-dev libturbojpeg0-dev libsdl2-dev libopencv-dev vulkan-tools

# 設定 Kinect v2 USB 權限並停用核心自動休眠
sudo tee /etc/udev/rules.d/90-kinect2.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666", ATTR{power/control}="on"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. 編譯含白燈修復修補檔的 `libfreenect2`
執行內附腳本自動套用 `patches/libfreenect2_linux_sdk.patch`（修正 OpenCL 3.0 標頭巨集衝突，並修正 `close()` 退出時白色指示燈常亮不滅的問題）：

```bash
./scripts/build_libfreenect2.sh "$HOME/.local/share/kinectv2-runtime/freenect2"
```

### 3. 準備骨架執行包與 Wine64 + DXVK（僅需一次）
1. 在安裝有 **Kinect for Windows SDK v2.0_1409** 的 Windows 主機編譯 `native/` 下的 `vvtechs_worker.c` 與 `k4wcll_stub.c`（亦可在 Linux 以 `x86_64-w64-mingw32-gcc` 交叉編譯），執行 `python scripts/stage_linux_bundle.py --out dist/kinectv2_linux_bundle` 後，將產生的 `dist/kinectv2_linux_bundle/` 放至本專案根目錄。
2. 建立專用 Wine64 環境、安裝 DXVK（`d3d11.dll` 與 `dxgi.dll`），並停用 Wine 內建的 USB 服務以免與原生 `libfreenect2` 搶佔連接埠：

```bash
export WINEPREFIX="$HOME/.wine-kinectv2"
wineboot --init
# 將 DXVK x64 的 d3d11.dll 與 dxgi.dll 放入 $WINEPREFIX/drive_c/windows/system32/（或執行 winetricks -q dxvk）
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\wineusb" /v Start /t REG_DWORD /d 4 /f
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\winebus" /v Start /t REG_DWORD /d 4 /f
```

### 4. 編譯 C/C++ 動態庫與桌面視覺化工具

```bash
cmake -S . -B build \
    -DFREENECT2_ROOT="$HOME/.local/share/kinectv2-runtime/freenect2" \
    -DKV2_BUILD_VISUALIZER=ON \
    -DKV2_BUILD_EXAMPLES=ON
cmake --build build -j"$(nproc)"
```

編譯產物：
- 動態庫：`lib/libkinectv2_sdk.so`（公開標頭檔：`include/kinectv2_sdk.h`）
- 30 FPS 視覺化視窗：`bin/kinectv2_visualizer`
- C++ 範例程式：`bin/cpp_sdk_demo`

### 5. 安裝 Python 套件

```bash
python3 -m venv .venv
.venv/bin/pip install -e .
```

### 6. 執行離線精度對齊驗證

```bash
.venv/bin/python tests/verify_linux_parity.py \
    --bundle ./dist/kinectv2_linux_bundle \
    --wineprefix "$HOME/.wine-kinectv2"
```

---

## Python 庫接口快速上手

`pykinectskel2` 與 `kinectv2_sdk` 兩個套件名稱皆可直接匯入相同介面：

```python
from pykinectskel2 import JointType, KV2_STREAM_ALL, KinectV2Sensor, TrackingState

with KinectV2Sensor(stream_flags=KV2_STREAM_ALL) as sensor:
    ir_intr, color_intr = sensor.get_intrinsics()
    while True:
        frame = sensor.wait_frame(timeout_ms=1000)
        if frame is None:
            continue
        with frame:  # 區塊內所有影像陣列皆為零拷貝唯讀視圖
            rgb = frame.color_bgrx          # (1080, 1920, 4) uint8 BGRX
            depth = frame.depth_mm          # (424, 512) uint16 毫米深度
            reg = frame.registered_bgrx     # (424, 512, 4) uint8 對齊後 RGB-D
            mask = frame.body_index         # (424, 512) uint8 (0..5 人體編號, 255 背景)

            for body in frame.bodies:
                head_xyz = body.joint_positions[JointType.HEAD]   # (X, Y, Z) 公尺
                head_uv = body.joint_color_uv[JointType.HEAD]     # 1920x1080 像素座標
                if body.joint_tracking_states[JointType.HEAD] == TrackingState.TRACKED:
                    print(f"{frame.sensor_fps:.1f} FPS | Head={head_xyz} -> px={head_uv}")
```

### 相機座標轉換工具方法

```python
u_d, v_d = sensor.project_camera_to_depth((0.0, 0.2, 2.0))          # 3D 公尺 -> 512x424 像素
u_c, v_c = sensor.project_camera_to_color((0.0, 0.2, 2.0))          # 3D 公尺 -> 1920x1080 像素
rgb_uv   = sensor.map_depth_to_color(dx=256, dy=212, depth_mm=2000) # 深度像素 -> 1080p 彩色像素
xyz_m    = sensor.deproject_depth_to_camera(dx=256, dy=212, depth_mm=2000) # 深度像素 -> 3D 公尺
```

---

## 啟動 30 FPS 即時視覺化視窗

```bash
./scripts/launch_visualizer.sh
```

視窗快捷鍵：`[R]` 重置骨架背景學習 · `[G]` 切換對齊後 Registered RGB-D · `[S]` 儲存畫面截圖 · `[Q / ESC]` 結束並自動熄滅感測器白色指示燈。

---

## 授權條款與第三方元件聲明

- **專案原始碼**：採用 [Apache License, Version 2.0](LICENSE) 開源授權（`Copyright 2026 barrydoooit`）。
- **`libfreenect2` 修補檔**：`patches/libfreenect2_linux_sdk.patch` 對應上游 [OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2)（採用 Apache-2.0 與 GPL-2.0 雙授權）。
- **微軟官方 SDK 執行時期檔案**：本倉庫**不包含亦不散佈**任何微軟專屬二進位檔與模型檔（如 `vvtechs.dll`、`*.xmplr`、`*.bin`、`*.mshdf`、`*.gbd`）。相關執行時期檔案須由使用者自行從合法安裝的官方 Microsoft Kinect for Windows SDK v2.0 提取，並受微軟官方授權條款約束；詳見 [NOTICE](NOTICE)。
