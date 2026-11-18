# pykinectskel2 — Kinect v2 Linux C/C++ 与 Python 3D 骨架开发库

[English](README.md) | [繁體中文](README.zh-TW.md) | [简体中文](README.zh-CN.md)

面向 **Microsoft Kinect v2** 的低开销 Linux 原生 C/C++17 与零拷贝 Python 开发库，提供稳定 **30.0 FPS** 的 `1920x1080` RGB、`512x424` 毫米级深度图、红外图、配准彩色深度图（Registered RGB-D）、`BodyIndex` 人体掩码，以及**与 Windows 官方 SDK v2.0_1409 完全一致的 25 关节 3D 骨架（误差 `< 1 mm`，单帧 GPU 推理约 `1.6 ms`）**。

- **数据结构与坐标系说明文档**：[English](docs/data_structures.en.md) | [繁體中文](docs/data_structures.zh-TW.md) | [简体中文](docs/data_structures.zh-CN.md)

---

## 项目目录结构

```text
├── CMakeLists.txt                   # 顶层 CMake 构建脚本（编译动态库、可视化工具与 C++ 示例）
├── pyproject.toml                   # Python 包配置（支持 `pip install -e .`）
├── LICENSE / NOTICE                 # Apache-2.0 开源许可协议与第三方组件声明
├── include/kinectv2_sdk.h           # C 语言接口（`kv2_*`）与 C++17 封装（`kv2::Device`）
├── src/kinectv2_sdk.cpp             # C++17 零拷贝多模态运行核心
├── src/tools/kinectv2_visualizer.cpp# 30 FPS SDL2 + OpenCV 四分屏桌面实时可视化工具
├── native/                          # `vvtechs_worker.c` 推理宿主与 `k4wcll_stub.c` 桩源码
├── patches/libfreenect2_linux_sdk.patch # OpenCL 3.0 修复与 `close()` 关闭白灯补丁
├── python/kinectv2_sdk/             # Python 零拷贝包（`KinectV2Sensor`、`KinectV2BodyTracker`）
├── scripts/
│   ├── build_libfreenect2.sh        # 自动打补丁并编译安装 libfreenect2
│   ├── stage_linux_bundle.py        # 在 Windows 从官方 SDK v2.0_1409 打包运行时文件
│   └── launch_visualizer.sh         # 启动 30 FPS 本地可视化窗口
├── examples/                        # `python_sdk_demo.py` 与 `cpp_sdk_demo.cpp` 最小示例
└── tests/                           # 离线黄金基准一致性验证脚本与测试集
```

---

## 完整编译与安装教程（Linux x86_64）

### 1. 安装系统依赖与配置 USB 权限规则

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config git \
    libusb-1.0-0-dev libturbojpeg0-dev libsdl2-dev libopencv-dev vulkan-tools

# 配置 Kinect v2 USB 权限并禁用内核自动休眠
sudo tee /etc/udev/rules.d/90-kinect2.rules >/dev/null <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02c4", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d8", MODE="0666", ATTR{power/control}="on"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="02d9", MODE="0666", ATTR{power/control}="on"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. 编译带白灯修复补丁的 `libfreenect2`
运行脚本自动应用 `patches/libfreenect2_linux_sdk.patch`（修复 OpenCL 3.0 头文件宏冲突，并修复 `close()` 退出后白色指示灯常亮不灭的问题）：

```bash
./scripts/build_libfreenect2.sh "$HOME/.local/share/kinectv2-runtime/freenect2"
```

### 3. 准备骨架运行包与 Wine64 + DXVK（仅需一次）
1. 在安装有 **Kinect for Windows SDK v2.0_1409** 的 Windows 机器上编译 `native/` 下的 `vvtechs_worker.c` 与 `k4wcll_stub.c`（也可在 Linux 用 `x86_64-w64-mingw32-gcc` 交叉编译），运行 `python scripts/stage_linux_bundle.py --out dist/kinectv2_linux_bundle`，再将生成的 `dist/kinectv2_linux_bundle/` 放到本项目根目录。
2. 初始化专用 Wine64 前缀、安装 DXVK（`d3d11.dll` 与 `dxgi.dll`），并禁用 Wine 自带的后台 USB 服务以免与原生 `libfreenect2` 争用端口：

```bash
export WINEPREFIX="$HOME/.wine-kinectv2"
wineboot --init
# 将 DXVK x64 的 d3d11.dll 与 dxgi.dll 复制到 $WINEPREFIX/drive_c/windows/system32/（或执行 winetricks -q dxvk）
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\wineusb" /v Start /t REG_DWORD /d 4 /f
wine reg add "HKLM\\System\\CurrentControlSet\\Services\\winebus" /v Start /t REG_DWORD /d 4 /f
```

### 4. 编译 C/C++ 动态库与桌面可视化工具

```bash
cmake -S . -B build \
    -DFREENECT2_ROOT="$HOME/.local/share/kinectv2-runtime/freenect2" \
    -DKV2_BUILD_VISUALIZER=ON \
    -DKV2_BUILD_EXAMPLES=ON
cmake --build build -j"$(nproc)"
```

编译产物：
- 动态库：`lib/libkinectv2_sdk.so`（公共头文件：`include/kinectv2_sdk.h`）
- 30 FPS 可视化窗口：`bin/kinectv2_visualizer`
- C++ 示例程序：`bin/cpp_sdk_demo`

### 5. 安装 Python 包

```bash
python3 -m venv .venv
.venv/bin/pip install -e .
```

### 6. 运行离线数值一致性验证

```bash
.venv/bin/python tests/verify_linux_parity.py \
    --bundle ./dist/kinectv2_linux_bundle \
    --wineprefix "$HOME/.wine-kinectv2"
```

---

## Python 库接口快速上手

`pykinectskel2` 与 `kinectv2_sdk` 两个包名均可直接导入相同接口：

```python
from pykinectskel2 import JointType, KV2_STREAM_ALL, KinectV2Sensor, TrackingState

with KinectV2Sensor(stream_flags=KV2_STREAM_ALL) as sensor:
    ir_intr, color_intr = sensor.get_intrinsics()
    while True:
        frame = sensor.wait_frame(timeout_ms=1000)
        if frame is None:
            continue
        with frame:  # 代码块内所有图像数组均为零拷贝只读视图
            rgb = frame.color_bgrx          # (1080, 1920, 4) uint8 BGRX
            depth = frame.depth_mm          # (424, 512) uint16 毫米深度
            reg = frame.registered_bgrx     # (424, 512, 4) uint8 配准后 RGB-D
            mask = frame.body_index         # (424, 512) uint8 (0..5 人体编号, 255 背景)

            for body in frame.bodies:
                head_xyz = body.joint_positions[JointType.HEAD]   # (X, Y, Z) 米
                head_uv = body.joint_color_uv[JointType.HEAD]     # 1920x1080 像素坐标
                if body.joint_tracking_states[JointType.HEAD] == TrackingState.TRACKED:
                    print(f"{frame.sensor_fps:.1f} FPS | Head={head_xyz} -> px={head_uv}")
```

### 相机坐标转换工具方法

```python
u_d, v_d = sensor.project_camera_to_depth((0.0, 0.2, 2.0))          # 3D 米 -> 512x424 像素
u_c, v_c = sensor.project_camera_to_color((0.0, 0.2, 2.0))          # 3D 米 -> 1920x1080 像素
rgb_uv   = sensor.map_depth_to_color(dx=256, dy=212, depth_mm=2000) # 深度像素 -> 1080p 彩色像素
xyz_m    = sensor.deproject_depth_to_camera(dx=256, dy=212, depth_mm=2000) # 深度像素 -> 3D 米
```

---

## 启动 30 FPS 实时可视化窗口

```bash
./scripts/launch_visualizer.sh
```

窗口快捷键：`[R]` 重置骨架背景学习 · `[G]` 切换配准后 Registered RGB-D · `[S]` 保存画面截图 · `[Q / ESC]` 退出并自动熄灭传感器白色指示灯。

---

## 开源许可与第三方组件声明

- **项目源代码**：采用 [Apache License, Version 2.0](LICENSE) 开源许可协议（`Copyright 2026 barrydoooit`）。
- **`libfreenect2` 补丁**：`patches/libfreenect2_linux_sdk.patch` 对应上游 [OpenKinect/libfreenect2](https://github.com/OpenKinect/libfreenect2)（采用 Apache-2.0 与 GPL-2.0 双许可）。
- **微软官方 SDK 运行时文件**：本仓库**不包含也不分发**任何微软专有二进制文件与模型文件（如 `vvtechs.dll`、`*.xmplr`、`*.bin`、`*.mshdf`、`*.gbd`）。相关运行时文件须由用户自行从合法安装的官方 Microsoft Kinect for Windows SDK v2.0 提取，并受微软官方许可条款约束；详见 [NOTICE](NOTICE)。
