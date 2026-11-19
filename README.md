# pykinectskel2 — Kinect v2 Linux C/C++ & Python 3D Skeleton SDK

[English](README.md) | [繁體中文](README.zh-TW.md) | [简体中文](README.zh-CN.md)

Low-overhead native Linux C/C++17 and zero-copy Python SDK for the **Microsoft Kinect v2** sensor, providing locked **30.0 FPS** streaming of `1920x1080` RGB, `512x424` millimeter Depth, Infrared, Registered RGB-D, `BodyIndex` masks, and **official Microsoft 25-joint 3D skeletons (`< 1 mm` parity with Windows SDK v2.0_1409, ~1.6 ms GPU inference)**.

- **Data Structures & Coordinate Reference**: [English](docs/data_structures.en.md) | [繁體中文](docs/data_structures.zh-TW.md) | [简体中文](docs/data_structures.zh-CN.md)

---

## Repository Layout

```text
├── CMakeLists.txt                   # CMake build for libkinectv2_sdk.so, visualizer, and C++ demo
├── pyproject.toml                   # PEP 621 Python package (`pip install -e .`)
├── LICENSE / NOTICE                 # Apache-2.0 license and third-party attribution notices
├── include/kinectv2_sdk.h           # Public C ABI (`kv2_*`) & C++17 RAII wrapper (`kv2::Device`)
├── src/kinectv2_sdk.cpp             # Zero-copy multi-modal C++17 runtime
├── src/tools/kinectv2_visualizer.cpp# 30 FPS SDL2 + OpenCV 4-pane desktop visualizer
├── native/                          # Clean-room PE host (`vvtechs_worker.c`, `k4wcll_stub.c`)
├── patches/libfreenect2_linux_sdk.patch # OpenCL 3.0 fix + proper LED shutoff on device close
├── python/kinectv2_sdk/             # Zero-copy Python SDK (`KinectV2Sensor`, `KinectV2BodyTracker`)
├── scripts/
│   ├── build_libfreenect2.sh        # Builds patched libfreenect2 from source
│   ├── stage_linux_bundle.py        # Stages runtime bundle on Windows from Kinect SDK v2.0_1409
│   └── launch_visualizer.sh         # Launches the 30 FPS desktop visualizer
├── examples/                        # `python_sdk_demo.py` and `cpp_sdk_demo.cpp`
└── tests/                           # Deterministic offline parity & Python API tests
```

---

## Build & Installation Tutorial (Linux x86_64)

### 1. System Prerequisites
Install C++ build tools, `libusb-1.0`, `TurboJPEG`, `OpenCV`, `SDL2`, `Wine64` (8.0+), and `DXVK` (for Vulkan Compute acceleration):

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config git \
    libusb-1.0-0-dev libturbojpeg0-dev libsdl2-dev libopencv-dev vulkan-tools

# USB permissions & disable autosuspend for Kinect v2 (045e:02c4 / 02d8 / 02d9)
sudo tee /etc/udev/rules.d/90-kinect2.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666", ATTR{power/control}="on"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. Build Patched `libfreenect2`
Build `libfreenect2` with `patches/libfreenect2_linux_sdk.patch` (fixes OpenCL 3.0 header macro collision and ensures the Kinect v2 white status LED turns off cleanly on `close()`):

```bash
./scripts/build_libfreenect2.sh "$HOME/.local/share/kinectv2-runtime/freenect2"
```

### 3. Stage Runtime Bundle & Configure Wine64 + DXVK (One-Time)
1. On a Windows machine with **Kinect for Windows SDK v2.0_1409** installed, run `scripts\build_worker_msvc.bat` (or cross-compile `native/src/vvtechs_worker.c` & `native/src/k4wcll_stub.c` on Linux with `x86_64-w64-mingw32-gcc`) followed by `python scripts/stage_linux_bundle.py --out dist/kinectv2_linux_bundle`, then place `dist/kinectv2_linux_bundle/` into this repository root.
2. Initialize a dedicated Wine64 prefix with DXVK (`d3d11.dll` and `dxgi.dll`) and disable Wine's background USB drivers (`wineusb.sys` / `winebus.sys`) so Wine never contends with `libfreenect2`:

```bash
export WINEPREFIX="$HOME/.wine-kinectv2"
wineboot --init
# Copy DXVK x64 d3d11.dll & dxgi.dll into $WINEPREFIX/drive_c/windows/system32/ (or run: winetricks -q dxvk)
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\wineusb" /v Start /t REG_DWORD /d 4 /f
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\winebus" /v Start /t REG_DWORD /d 4 /f
```

### 4. Build C/C++ Library & Desktop Visualizer

```bash
cmake -S . -B build \
    -DFREENECT2_ROOT="$HOME/.local/share/kinectv2-runtime/freenect2" \
    -DKV2_BUILD_VISUALIZER=ON \
    -DKV2_BUILD_EXAMPLES=ON
cmake --build build -j"$(nproc)"
```

Build outputs:
- Shared library: `lib/libkinectv2_sdk.so` (header: `include/kinectv2_sdk.h`)
- 30 FPS Desktop Visualizer: `bin/kinectv2_visualizer`
- C++ Demo: `bin/cpp_sdk_demo`

Optional system/prefix install: `cmake --install build --prefix /usr/local`.

### 5. Install Python Package

```bash
python3 -m venv .venv
.venv/bin/pip install -e .
```

### 6. Verify Offline Numerical Parity (Before Connecting Sensor)

```bash
.venv/bin/python tests/verify_linux_parity.py \
    --bundle ./dist/kinectv2_linux_bundle \
    --wineprefix "$HOME/.wine-kinectv2"
```

---

## Native Windows x64 USB Build (Without Kinect SDK Service)

`pykinectskel2` also builds natively on Windows x64 (`kinectv2_sdk.dll` + `freenect2.dll` + native Win32 `vvtechs_worker.exe` pipes), bypassing `KinectService.exe`:

1. Install **UsbDk** (x64) or bind **libusbK** via Zadig to `Xbox NUI Sensor (Composite Parent)` (`VID 045E`, `PID 02C4` / `02D8`) so Windows `libfreenect2` can claim the USB 3.0 sensor directly.
2. Build and stage `%LOCALAPPDATA%\kinectv2-runtime` (`bin\kinectv2_sdk.dll`, `freenect2\bin\freenect2.dll`, and `dist\kinectv2_windows_bundle\vvtechs_worker.exe`):

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_windows_sdk.ps1
python .\tests\verify_windows_live.py
```

---

## Python API Quickstart

Both `pykinectskel2` and `kinectv2_sdk` expose the identical Python API:

```python
from pykinectskel2 import JointType, KV2_STREAM_ALL, KinectV2Sensor, TrackingState

with KinectV2Sensor(stream_flags=KV2_STREAM_ALL) as sensor:
    ir_intr, color_intr = sensor.get_intrinsics()
    while True:
        frame = sensor.wait_frame(timeout_ms=1000)
        if frame is None:
            continue
        with frame:  # Zero-copy views valid inside this block
            rgb = frame.color_bgrx          # (1080, 1920, 4) uint8 BGRX
            depth = frame.depth_mm          # (424, 512) uint16 millimeters
            reg = frame.registered_bgrx     # (424, 512, 4) uint8 aligned RGB-D
            mask = frame.body_index         # (424, 512) uint8 (0..5 or 255)

            for body in frame.bodies:
                head_xyz = body.joint_positions[JointType.HEAD]   # (X, Y, Z) meters
                head_uv = body.joint_color_uv[JointType.HEAD]     # (u, v) on 1920x1080
                if body.joint_tracking_states[JointType.HEAD] == TrackingState.TRACKED:
                    print(f"{frame.sensor_fps:.1f} FPS | Head={head_xyz} -> px={head_uv}")
```

### Coordinate Projection Utilities

```python
u_d, v_d = sensor.project_camera_to_depth((0.0, 0.2, 2.0))
u_c, v_c = sensor.project_camera_to_color((0.0, 0.2, 2.0))
rgb_uv   = sensor.map_depth_to_color(dx=256, dy=212, depth_mm=2000.0)
xyz_m    = sensor.deproject_depth_to_camera(dx=256, dy=212, depth_mm=2000.0)
```

---

## C++17 Quickstart

```cpp
#include "kinectv2_sdk.h"
#include <cstdio>

int main() {
    Kv2Config cfg{};
    kv2_default_config(&cfg);
    kv2::Device dev(&cfg);
    dev.start();

    uint64_t seq = 0;
    while (auto frame = dev.wait_frame(seq, 1000)) {
        seq = frame->frame_seq;
        for (uint32_t i = 0; i < frame->body_count; ++i) {
            const auto& head = frame->bodies[i].joints[KV2_JOINT_HEAD];
            std::printf("Head: (%.2f, %.2f, %.2f m)\n", head.pos[0], head.pos[1], head.pos[2]);
        }
    }
}
```

---

## 30 FPS Desktop Online Visualizer

```bash
./scripts/launch_visualizer.sh
```

Hotkeys: `[R]` Reset skeleton background learner · `[G]` Toggle Registered RGB-D view · `[S]` Save PNG snapshot · `[Q / ESC]` Quit cleanly (automatically turns off the sensor status LED).

---

## License & Third-Party Notice

- **Source Code**: Licensed under the [Apache License, Version 2.0](LICENSE) (`Copyright 2026 barrydoooit`).
- **`libfreenect2` Patch**: `patches/libfreenect2_linux_sdk.patch` applies to [OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2), which is dual-licensed under Apache-2.0 and GPL-2.0.
- **Microsoft Kinect SDK Runtime Assets**: This repository does **not** distribute any proprietary Microsoft DLLs or model files (`vvtechs.dll`, `*.xmplr`, `*.bin`, `*.mshdf`, `*.gbd`). Users stage those files from their own legitimately installed copy of the official Microsoft Kinect for Windows SDK v2.0 under Microsoft's applicable license terms. See [NOTICE](NOTICE) for details.
