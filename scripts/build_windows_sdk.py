"""
Zero-MSVC cross/native Windows x64 builder for `pykinectskel2` using `zig cc` / `zig c++`
(`-target x86_64-windows-gnu`).

Works identically on:
  - Windows x64 (even without Visual Studio, MSVC, system CMake, or 7-Zip):
      uv run --no-project --with ziglang python scripts/build_windows_sdk.py
  - Linux x86_64 (cross-compiling full Windows x64 PE64 DLLs + worker EXE):
      uv run --no-project --with ziglang python scripts/build_windows_sdk.py --runtime-root ./build_win64_rt

Builds from clean upstream sources and stages:
  1. libusb-1.0.dll      (v1.0.27, with UsbDk + WinUSB/libusbK backends)
  2. turbojpeg.dll       (libjpeg-turbo 3.0.4)
  3. freenect2.dll       (OpenKinect libfreenect2 + LED-off patch + OpenCL 1.2 GPU pipeline)
  4. k4wcll.dll & vvtechs_worker.exe (standalone D3D11 25-joint skeleton worker)
  5. kinectv2_sdk.dll    (pykinectskel2 C/C++17 zero-copy multi-modal runtime)
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
TARGET = "x86_64-windows-gnu"

OPENCL_12_EXPORTS = [
    "clGetPlatformIDs",
    "clGetPlatformInfo",
    "clGetDeviceIDs",
    "clGetDeviceInfo",
    "clCreateSubDevices",
    "clRetainDevice",
    "clReleaseDevice",
    "clCreateContext",
    "clCreateContextFromType",
    "clRetainContext",
    "clReleaseContext",
    "clGetContextInfo",
    "clCreateCommandQueue",
    "clRetainCommandQueue",
    "clReleaseCommandQueue",
    "clGetCommandQueueInfo",
    "clCreateBuffer",
    "clCreateSubBuffer",
    "clCreateImage",
    "clCreateImage2D",
    "clCreateImage3D",
    "clRetainMemObject",
    "clReleaseMemObject",
    "clGetSupportedImageFormats",
    "clGetMemObjectInfo",
    "clGetImageInfo",
    "clSetMemObjectDestructorCallback",
    "clCreateSampler",
    "clRetainSampler",
    "clReleaseSampler",
    "clGetSamplerInfo",
    "clCreateProgramWithSource",
    "clCreateProgramWithBinary",
    "clCreateProgramWithBuiltInKernels",
    "clRetainProgram",
    "clReleaseProgram",
    "clBuildProgram",
    "clCompileProgram",
    "clLinkProgram",
    "clUnloadPlatformCompiler",
    "clUnloadCompiler",
    "clGetProgramInfo",
    "clGetProgramBuildInfo",
    "clCreateKernel",
    "clCreateKernelsInProgram",
    "clRetainKernel",
    "clReleaseKernel",
    "clSetKernelArg",
    "clGetKernelInfo",
    "clGetKernelArgInfo",
    "clGetKernelWorkGroupInfo",
    "clWaitForEvents",
    "clGetEventInfo",
    "clCreateUserEvent",
    "clRetainEvent",
    "clReleaseEvent",
    "clSetUserEventStatus",
    "clSetEventCallback",
    "clGetEventProfilingInfo",
    "clFlush",
    "clFinish",
    "clEnqueueReadBuffer",
    "clEnqueueReadBufferRect",
    "clEnqueueWriteBuffer",
    "clEnqueueWriteBufferRect",
    "clEnqueueFillBuffer",
    "clEnqueueCopyBuffer",
    "clEnqueueCopyBufferRect",
    "clEnqueueReadImage",
    "clEnqueueWriteImage",
    "clEnqueueFillImage",
    "clEnqueueCopyImage",
    "clEnqueueCopyImageToBuffer",
    "clEnqueueCopyBufferToImage",
    "clEnqueueMapBuffer",
    "clEnqueueMapImage",
    "clEnqueueUnmapMemObject",
    "clEnqueueMigrateMemObjects",
    "clEnqueueNDRangeKernel",
    "clEnqueueTask",
    "clEnqueueNativeKernel",
    "clEnqueueMarkerWithWaitList",
    "clEnqueueBarrierWithWaitList",
    "clEnqueueMarker",
    "clEnqueueWaitForEvents",
    "clEnqueueBarrier",
    "clGetExtensionFunctionAddressForPlatform",
    "clGetExtensionFunctionAddress",
]


def default_runtime_root() -> Path:
    if os.name == "nt":
        la = os.environ.get("LOCALAPPDATA")
        if la:
            return Path(la) / "kinectv2-runtime"
    return REPO_ROOT / "build_win64_runtime"


def find_zig(explicit: Optional[str] = None) -> Path:
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if p.is_file():
            return p
    env_zig = os.environ.get("ZIG_EXE")
    if env_zig and Path(env_zig).is_file():
        return Path(env_zig).resolve()
    try:
        import ziglang  # type: ignore

        cand = Path(ziglang.__file__).resolve().parent / ("zig.exe" if os.name == "nt" else "zig")
        if cand.is_file():
            return cand
    except ImportError:
        pass
    which_zig = shutil.which("zig")
    if which_zig:
        return Path(which_zig).resolve()
    if os.name == "nt":
        la = os.environ.get("LOCALAPPDATA")
        if la:
            uv_cache = Path(la) / "uv" / "cache"
            if uv_cache.is_dir():
                for found in uv_cache.rglob("zig.exe"):
                    return found.resolve()
    raise FileNotFoundError(
        "Zig toolchain not found. Run via: uv run --no-project --with ziglang python scripts/build_windows_sdk.py"
    )


def run_cmd(cmd: List[str], cwd: Optional[Path] = None) -> None:
    proc = subprocess.run(cmd, cwd=str(cwd) if cwd else None, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(
            f"Command failed (exit {proc.returncode}): {' '.join(cmd)}\n"
            f"STDOUT:\n{proc.stdout}\nSTDERR:\n{proc.stderr}"
        )


def compile_objects_parallel(tasks: List[List[str]]) -> None:
    max_workers = min(os.cpu_count() or 4, 12)
    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as pool:
        futs = [pool.submit(run_cmd, cmd) for cmd in tasks]
        for f in concurrent.futures.as_completed(futs):
            f.result()


def git_clone_once(url: str, dst: Path, branch: Optional[str] = None) -> None:
    if (dst / ".git").is_dir():
        return
    if dst.exists():
        shutil.rmtree(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["git", "clone", "--depth", "1"]
    if branch:
        cmd.extend(["--branch", branch])
    cmd.extend([url, str(dst)])
    run_cmd(cmd)


def build_libusb(zig: Path, src_root: Path, deps_dir: Path, obj_root: Path) -> Path:
    out_dll = deps_dir / "libusb" / "bin" / "libusb-1.0.dll"
    out_lib = deps_dir / "libusb" / "lib" / "libusb-1.0.dll.a"
    inc_dir = deps_dir / "libusb" / "include"
    if out_dll.is_file() and out_lib.is_file():
        print(f"[1/5] Reusing cached libusb-1.0.dll at {out_dll}")
        return out_dll

    print("[1/5] Building libusb v1.0.27 (x86_64-windows-gnu)...")
    repo = src_root / "libusb"
    git_clone_once("https://github.com/libusb/libusb.git", repo, branch="v1.0.27")

    cfg_dir = obj_root / "libusb_cfg"
    obj_dir = obj_root / "libusb_obj"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    obj_dir.mkdir(parents=True, exist_ok=True)

    (cfg_dir / "config.h").write_text(
        "#pragma once\n"
        "#define DEFAULT_VISIBILITY /**/\n"
        "#define ENABLE_LOGGING 1\n"
        "#define PLATFORM_WINDOWS 1\n"
        "#define PRINTF_FORMAT(a, b) __attribute__((__format__(__printf__, a, b)))\n"
        "#define HAVE_STRING_H 1\n",
        encoding="utf-8",
    )

    c_files = [
        "libusb/core.c",
        "libusb/descriptor.c",
        "libusb/hotplug.c",
        "libusb/io.c",
        "libusb/strerror.c",
        "libusb/sync.c",
        "libusb/os/events_windows.c",
        "libusb/os/threads_windows.c",
        "libusb/os/windows_common.c",
        "libusb/os/windows_usbdk.c",
        "libusb/os/windows_winusb.c",
    ]
    objs: List[Path] = []
    tasks: List[List[str]] = []
    for rel in c_files:
        src = repo / rel
        obj = obj_dir / (Path(rel).stem + ".o")
        objs.append(obj)
        tasks.append(
            [
                str(zig), "cc", "-target", TARGET, "-O3", "-std=gnu11",
                f"-I{cfg_dir}", f"-I{repo / 'libusb'}", f"-I{repo / 'libusb' / 'os'}",
                "-Wno-unused-parameter",
                "-c", str(src), "-o", str(obj),
            ]
        )
    compile_objects_parallel(tasks)

    out_dll.parent.mkdir(parents=True, exist_ok=True)
    out_lib.parent.mkdir(parents=True, exist_ok=True)
    (inc_dir / "libusb-1.0").mkdir(parents=True, exist_ok=True)
    shutil.copy2(repo / "libusb" / "libusb.h", inc_dir / "libusb.h")
    shutil.copy2(repo / "libusb" / "libusb.h", inc_dir / "libusb-1.0" / "libusb.h")

    raw_def = (repo / "libusb" / "libusb-1.0.def").read_text(encoding="utf-8").splitlines()
    x64_def = cfg_dir / "libusb-1.0-x64.def"
    x64_def.write_text("\n".join(ln for ln in raw_def if "@" not in ln) + "\n", encoding="utf-8")

    run_cmd(
        [
            str(zig), "cc", "-target", TARGET, "-shared", "-O3",
            *[str(o) for o in objs],
            str(x64_def),
            f"-Wl,--out-implib,{out_lib}",
            "-ladvapi32", "-lkernel32", "-luser32",
            "-o", str(out_dll),
        ]
    )
    return out_dll


def build_turbojpeg(zig: Path, src_root: Path, deps_dir: Path, obj_root: Path) -> Path:
    out_dll = deps_dir / "turbojpeg" / "bin" / "turbojpeg.dll"
    out_lib = deps_dir / "turbojpeg" / "lib" / "libturbojpeg.dll.a"
    inc_dir = deps_dir / "turbojpeg" / "include"
    if out_dll.is_file() and out_lib.is_file():
        print(f"[2/5] Reusing cached turbojpeg.dll at {out_dll}")
        return out_dll

    print("[2/5] Building libjpeg-turbo 3.0.4 (x86_64-windows-gnu)...")
    repo = src_root / "libjpeg-turbo"
    git_clone_once("https://github.com/libjpeg-turbo/libjpeg-turbo.git", repo, branch="3.0.4")

    cfg_dir = obj_root / "jpeg_cfg"
    obj_dir = obj_root / "jpeg_obj"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    obj_dir.mkdir(parents=True, exist_ok=True)

    (cfg_dir / "jconfig.h").write_text(
        "#define JPEG_LIB_VERSION 62\n"
        "#define LIBJPEG_TURBO_VERSION 3.0.4\n"
        "#define LIBJPEG_TURBO_VERSION_NUMBER 3000004\n"
        "#define C_ARITH_CODING_SUPPORTED 1\n"
        "#define D_ARITH_CODING_SUPPORTED 1\n"
        "#define MEM_SRCDST_SUPPORTED 1\n"
        "#ifndef BITS_IN_JSAMPLE\n"
        "#define BITS_IN_JSAMPLE 8\n"
        "#endif\n"
        "#ifdef _WIN32\n"
        "#ifndef __RPCNDR_H__\n"
        "typedef unsigned char boolean;\n"
        "#endif\n"
        "#define HAVE_BOOLEAN\n"
        "#if !(defined(_BASETSD_H_) || defined(_BASETSD_H))\n"
        "typedef short INT16;\n"
        "typedef signed int INT32;\n"
        "#endif\n"
        "#define XMD_H\n"
        "#endif\n",
        encoding="utf-8",
    )
    (cfg_dir / "jconfigint.h").write_text(
        '#define BUILD "20261119"\n'
        "#define HIDDEN\n"
        "#undef inline\n"
        "#define INLINE __inline__\n"
        "#define THREAD_LOCAL __thread\n"
        '#define PACKAGE_NAME "libjpeg-turbo"\n'
        '#define VERSION "3.0.4"\n'
        "#define SIZEOF_SIZE_T 8\n"
        "#define HAVE_INTRIN_H 1\n"
        "#define FALLTHROUGH\n"
        "#ifndef BITS_IN_JSAMPLE\n"
        "#define BITS_IN_JSAMPLE 8\n"
        "#endif\n"
        "#undef C_ARITH_CODING_SUPPORTED\n"
        "#undef D_ARITH_CODING_SUPPORTED\n"
        "#undef WITH_SIMD\n"
        "#if BITS_IN_JSAMPLE == 8\n"
        "#define C_ARITH_CODING_SUPPORTED 1\n"
        "#define D_ARITH_CODING_SUPPORTED 1\n"
        "#endif\n",
        encoding="utf-8",
    )
    (cfg_dir / "jversion.h").write_text(
        '#define JVERSION "6b  27-Mar-1998"\n'
        '#define JCOPYRIGHT1 "Copyright (C) 2009-2024 D. R. Commander\\n"\n'
        '#define JCOPYRIGHT2 "Copyright (C) 1991-2020 Thomas G. Lane, Guido Vollbeding\\n"\n'
        '#define JCOPYRIGHT_SHORT "Copyright (C) 2024 The libjpeg-turbo Project"\n',
        encoding="utf-8",
    )

    jpeg16 = [
        "jcapistd.c", "jccolor.c", "jcdiffct.c", "jclossls.c", "jcmainct.c",
        "jcprepct.c", "jcsample.c", "jdapistd.c", "jdcolor.c", "jddiffct.c",
        "jdlossls.c", "jdmainct.c", "jdpostct.c", "jdsample.c", "jutils.c",
    ]
    jpeg12 = jpeg16 + [
        "jccoefct.c", "jcdctmgr.c", "jdcoefct.c", "jddctmgr.c", "jdmerge.c",
        "jfdctfst.c", "jfdctint.c", "jidctflt.c", "jidctfst.c", "jidctint.c",
        "jidctred.c", "jquant1.c", "jquant2.c",
    ]
    jpeg8 = jpeg12 + [
        "jcapimin.c", "jchuff.c", "jcicc.c", "jcinit.c", "jclhuff.c",
        "jcmarker.c", "jcmaster.c", "jcomapi.c", "jcparam.c", "jcphuff.c",
        "jctrans.c", "jdapimin.c", "jdatadst.c", "jdatasrc.c", "jdhuff.c",
        "jdicc.c", "jdinput.c", "jdlhuff.c", "jdmarker.c", "jdmaster.c",
        "jdphuff.c", "jdtrans.c", "jerror.c", "jfdctflt.c", "jmemmgr.c",
        "jmemnobs.c", "jpeg_nbits.c", "jaricom.c", "jcarith.c", "jdarith.c",
        "turbojpeg.c", "transupp.c", "jdatadst-tj.c", "jdatasrc-tj.c",
        "rdbmp.c", "rdppm.c", "wrbmp.c", "wrppm.c",
    ]

    objs: List[Path] = []
    tasks: List[List[str]] = []

    def add_batch(files: Iterable[str], prefix: str, extra_defs: List[str]) -> None:
        for f in files:
            src = repo / f
            obj = obj_dir / f"{prefix}_{Path(f).stem}.o"
            objs.append(obj)
            tasks.append(
                [
                    str(zig), "cc", "-target", TARGET, "-O3",
                    f"-I{cfg_dir}", f"-I{repo}",
                    "-DDLLDEFINE", "-DBMP_SUPPORTED", "-DPPM_SUPPORTED",
                    *extra_defs,
                    "-Wno-unused-parameter",
                    "-c", str(src), "-o", str(obj),
                ]
            )

    add_batch(jpeg8, "b8", ["-DBITS_IN_JSAMPLE=8"])
    add_batch(jpeg12 + ["rdppm.c", "wrppm.c"], "b12", ["-DBITS_IN_JSAMPLE=12"])
    add_batch(jpeg16 + ["rdppm.c", "wrppm.c"], "b16", ["-DBITS_IN_JSAMPLE=16"])
    compile_objects_parallel(tasks)

    out_dll.parent.mkdir(parents=True, exist_ok=True)
    out_lib.parent.mkdir(parents=True, exist_ok=True)
    inc_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(repo / "turbojpeg.h", inc_dir / "turbojpeg.h")
    shutil.copy2(repo / "jpeglib.h", inc_dir / "jpeglib.h")
    shutil.copy2(repo / "jmorecfg.h", inc_dir / "jmorecfg.h")
    shutil.copy2(repo / "jerror.h", inc_dir / "jerror.h")
    shutil.copy2(cfg_dir / "jconfig.h", inc_dir / "jconfig.h")

    run_cmd(
        [
            str(zig), "cc", "-target", TARGET, "-shared", "-O3",
            *[str(o) for o in objs],
            f"-Wl,--out-implib,{out_lib}",
            "-o", str(out_dll),
        ]
    )
    return out_dll


def build_opencl_import(zig: Path, src_root: Path, deps_dir: Path, obj_root: Path) -> Tuple[Path, Path]:
    ocl_inc = src_root / "OpenCL-Headers"
    ocl_lib = deps_dir / "opencl" / "lib" / "libOpenCL.dll.a"
    if not (ocl_inc / "CL" / "cl.h").is_file():
        git_clone_once("https://github.com/KhronosGroup/OpenCL-Headers.git", ocl_inc)
    if not ocl_lib.is_file():
        ocl_lib.parent.mkdir(parents=True, exist_ok=True)
        def_path = obj_root / "OpenCL.def"
        obj_root.mkdir(parents=True, exist_ok=True)
        lines = ["LIBRARY OpenCL.dll", "EXPORTS"] + [f"  {sym}" for sym in OPENCL_12_EXPORTS]
        def_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        run_cmd([str(zig), "dlltool", "-m", "i386:x86-64", "-d", str(def_path), "-l", str(ocl_lib)])
    return ocl_inc, ocl_lib


def generate_freenect2_resources(fn2_repo: Path, rel_files: List[str], out_header: Path) -> None:
    lines: List[str] = []
    for idx, rel in enumerate(rel_files):
        data = (fn2_repo / rel).read_bytes()
        lines.append(f"static unsigned char resource{idx}[] = {{")
        for off in range(0, len(data), 16):
            chunk = data[off : off + 16]
            hex_items = ", ".join(f"0x{b:02x}" for b in chunk)
            lines.append(f"  {hex_items},")
        lines.append("};")
    lines.append("static ResourceDescriptor resource_descriptors[] = {")
    for idx, rel in enumerate(rel_files):
        name = Path(rel).name
        lines.append(f'  {{ "{name}", resource{idx}, sizeof(resource{idx}) }},')
    lines.append("  {NULL, NULL, 0},")
    lines.append("};")
    lines.append(f"static int resource_descriptors_length = {len(rel_files)};\n")
    out_header.parent.mkdir(parents=True, exist_ok=True)
    out_header.write_text("\n".join(lines), encoding="utf-8")


def build_freenect2(
    zig: Path,
    src_root: Path,
    deps_dir: Path,
    fn2_install: Path,
    obj_root: Path,
    enable_opencl: bool = True,
) -> Path:
    out_dll = fn2_install / "bin" / "freenect2.dll"
    out_lib = fn2_install / "lib" / "libfreenect2.dll.a"
    if out_dll.is_file() and out_lib.is_file():
        print(f"[3/5] Reusing cached freenect2.dll at {out_dll}")
        return out_dll

    use_ocl = enable_opencl and (os.name != "nt" or Path(r"C:\Windows\System32\OpenCL.dll").is_file())
    ocl_label = "OpenCL + TurboJPEG" if use_ocl else "TurboJPEG + CPU"
    print(f"[3/5] Building patched libfreenect2 ({ocl_label} + clean LED shutoff)...")
    fn2_repo = src_root / "libfreenect2"
    git_clone_once("https://github.com/OpenKinect/libfreenect2.git", fn2_repo)

    patch_file = REPO_ROOT / "patches" / "libfreenect2_linux_sdk.patch"
    chk = subprocess.run(
        ["git", "-C", str(fn2_repo), "apply", "--check", str(patch_file)],
        capture_output=True,
        text=True,
    )
    if chk.returncode == 0:
        run_cmd(["git", "-C", str(fn2_repo), "apply", str(patch_file)])
        print("      Applied patches/libfreenect2_linux_sdk.patch")

    ocl_inc: Optional[Path] = None
    ocl_lib: Optional[Path] = None
    if use_ocl:
        ocl_inc, ocl_lib = build_opencl_import(zig, src_root, deps_dir, obj_root)

    inc_out = fn2_install / "include" / "libfreenect2"
    inc_out.mkdir(parents=True, exist_ok=True)
    for hdr in (fn2_repo / "include" / "libfreenect2").iterdir():
        if hdr.is_file() and not hdr.name.endswith(".in"):
            shutil.copy2(hdr, inc_out / hdr.name)

    (inc_out / "export.h").write_text(
        "#ifndef LIBFREENECT2_EXPORT_H\n"
        "#define LIBFREENECT2_EXPORT_H\n"
        "#ifdef freenect2_EXPORTS\n"
        "#  define LIBFREENECT2_EXPORT __declspec(dllexport)\n"
        "#else\n"
        "#  define LIBFREENECT2_EXPORT __declspec(dllimport)\n"
        "#endif\n"
        "#define LIBFREENECT2_NO_EXPORT\n"
        "#define LIBFREENECT2_DEPRECATED\n"
        "#define LIBFREENECT2_DEPRECATED_EXPORT LIBFREENECT2_EXPORT\n"
        "#define LIBFREENECT2_DEPRECATED_NO_EXPORT\n"
        "#endif\n",
        encoding="utf-8",
    )
    ocl_cfg_line = "#define LIBFREENECT2_WITH_OPENCL_SUPPORT 1\n" if use_ocl else "/* #undef LIBFREENECT2_WITH_OPENCL_SUPPORT */\n"
    (inc_out / "config.h").write_text(
        "#ifndef LIBFREENECT2_CONFIG_H\n"
        "#define LIBFREENECT2_CONFIG_H\n"
        '#define LIBFREENECT2_VERSION "0.2.0"\n'
        "#define LIBFREENECT2_API_VERSION ((0 << 16) | 2)\n"
        "#define LIBFREENECT2_PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))\n"
        "#include <libfreenect2/export.h>\n"
        "#define LIBFREENECT2_API LIBFREENECT2_EXPORT\n"
        + ocl_cfg_line +
        "#define LIBFREENECT2_WITH_TURBOJPEG_SUPPORT 1\n"
        '#define LIBFREENECT2_TEGRAJPEG_LIBRARY ""\n'
        "#define LIBFREENECT2_THREADING_STDLIB 1\n"
        "#define LIBFREENECT2_WITH_CXX11_SUPPORT 1\n"
        "#endif\n",
        encoding="utf-8",
    )

    gen_dir = obj_root / "fn2_gen"
    obj_dir = obj_root / "fn2_obj"
    obj_dir.mkdir(parents=True, exist_ok=True)
    res_inputs = ["src/opencl_depth_packet_processor.cl", "src/opencl_kde_depth_packet_processor.cl"] if use_ocl else []
    generate_freenect2_resources(fn2_repo, res_inputs, gen_dir / "resources.inc.h")

    cpp_files = [
        "src/transfer_pool.cpp",
        "src/event_loop.cpp",
        "src/usb_control.cpp",
        "src/allocator.cpp",
        "src/frame_listener_impl.cpp",
        "src/packet_pipeline.cpp",
        "src/rgb_packet_stream_parser.cpp",
        "src/rgb_packet_processor.cpp",
        "src/turbo_jpeg_rgb_packet_processor.cpp",
        "src/depth_packet_stream_parser.cpp",
        "src/depth_packet_processor.cpp",
        "src/cpu_depth_packet_processor.cpp",
        "src/resource.cpp",
        "src/command_transaction.cpp",
        "src/registration.cpp",
        "src/logging.cpp",
        "src/libfreenect2.cpp",
    ]
    if use_ocl:
        cpp_files.extend(
            [
                "src/opencl_depth_packet_processor.cpp",
                "src/opencl_kde_depth_packet_processor.cpp",
            ]
        )

    objs: List[Path] = []
    tasks: List[List[str]] = []
    for rel in cpp_files:
        src = fn2_repo / rel
        obj = obj_dir / (Path(rel).stem + ".o")
        objs.append(obj)
        cmd = [
            str(zig), "c++", "-target", TARGET, "-std=gnu++14", "-O3",
            "-Dfreenect2_EXPORTS", "-DRESOURCES_INC",
            "-D_CRT_SECURE_NO_WARNINGS", "-D_USE_MATH_DEFINES",
            "-DCL_TARGET_OPENCL_VERSION=120",
            "-DCL_USE_DEPRECATED_OPENCL_1_1_APIS",
            "-DCL_USE_DEPRECATED_OPENCL_1_2_APIS",
            f"-I{fn2_install / 'include'}",
            f"-I{fn2_repo / 'include'}",
            f"-I{fn2_repo / 'include' / 'internal'}",
            f"-I{gen_dir}",
            f"-I{deps_dir / 'libusb' / 'include' / 'libusb-1.0'}",
            f"-I{deps_dir / 'turbojpeg' / 'include'}",
        ]
        if ocl_inc is not None:
            cmd.append(f"-I{ocl_inc}")
        cmd.extend(["-Wno-unused-parameter", "-Wno-deprecated-declarations", "-c", str(src), "-o", str(obj)])
        tasks.append(cmd)
    compile_objects_parallel(tasks)

    out_dll.parent.mkdir(parents=True, exist_ok=True)
    out_lib.parent.mkdir(parents=True, exist_ok=True)
    link_cmd = [
        str(zig), "c++", "-target", TARGET, "-shared", "-O3",
        *[str(o) for o in objs],
        str(deps_dir / "libusb" / "lib" / "libusb-1.0.dll.a"),
        str(deps_dir / "turbojpeg" / "lib" / "libturbojpeg.dll.a"),
    ]
    if ocl_lib is not None:
        link_cmd.append(str(ocl_lib))
    link_cmd.extend(
        [
            f"-Wl,--out-implib,{out_lib}",
            "-lws2_32", "-lsetupapi", "-lole32", "-ladvapi32",
            "-o", str(out_dll),
        ]
    )
    run_cmd(link_cmd)
    return out_dll


def build_worker_and_stage_bundles(zig: Path, runtime_root: Path, extra_bundle_src: Optional[Path]) -> None:
    print("[4/5] Building vvtechs_worker.exe + k4wcll.dll and staging bundles...")
    out_bin = (REPO_ROOT / "bin") if os.name == "nt" else (runtime_root / "bin")
    out_bin.mkdir(parents=True, exist_ok=True)

    k4wcll_out = out_bin / "k4wcll.dll"
    worker_out = out_bin / "vvtechs_worker.exe"

    run_cmd(
        [
            str(zig), "cc", "-target", TARGET, "-shared", "-O2",
            str(REPO_ROOT / "native" / "src" / "k4wcll_stub.c"),
            "-o", str(k4wcll_out),
        ]
    )
    run_cmd(
        [
            str(zig), "cc", "-target", TARGET, "-O2",
            f"-I{REPO_ROOT / 'native' / 'include'}",
            str(REPO_ROOT / "native" / "src" / "vvtechs_worker.c"),
            "-lws2_32", "-ld3d11", "-luser32",
            "-o", str(worker_out),
        ]
    )

    search_dirs = []
    if extra_bundle_src:
        search_dirs.append(extra_bundle_src)
    if os.name == "nt":
        search_dirs.append(Path(r"C:\Windows\System32\Kinect"))
    search_dirs.extend(
        [
            runtime_root / "dist" / "kinectv2_windows_bundle",
            runtime_root / "dist" / "kinectv2_linux_bundle",
            REPO_ROOT / "dist" / "kinectv2_windows_bundle",
            REPO_ROOT / "dist" / "kinectv2_linux_bundle",
            Path.home() / ".local" / "share" / "kinectv2-runtime" / "dist" / "kinectv2_linux_bundle",
        ]
    )

    model_src: Optional[Path] = None
    for cand in search_dirs:
        if (cand / "vvtechs.dll").is_file():
            model_src = cand.resolve()
            break

    if model_src is None:
        print("      Note: vvtechs.dll not found locally; compiled worker + stub into bin/ only.")
        return

    req_assets = [
        "vvtechs.dll",
        "exemplardb.xmplr",
        "FernsModel.bin",
        "HeadPosition.mshdf",
        "LSVMGripReleaseRight.gbd",
    ]
    opt_crt = ["msvcr110.dll", "msvcp110.dll"]
    target_bundles = [
        runtime_root / "dist" / "kinectv2_windows_bundle",
        runtime_root / "dist" / "kinectv2_linux_bundle",
    ]
    if os.name == "nt":
        target_bundles.extend(
            [
                REPO_ROOT / "dist" / "kinectv2_windows_bundle",
                REPO_ROOT / "dist" / "kinectv2_linux_bundle",
            ]
        )
    for bdir in target_bundles:
        bdir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(worker_out, bdir / "vvtechs_worker.exe")
        shutil.copy2(k4wcll_out, bdir / "k4wcll.dll")
        for asset in req_assets:
            s = model_src / asset
            d = bdir / asset
            if s.is_file() and s.resolve() != d.resolve():
                shutil.copy2(s, d)
        for crt in opt_crt:
            sys_crt = Path(r"C:\Windows\System32") / crt if os.name == "nt" else None
            alt_crt = model_src / crt
            d = bdir / crt
            if sys_crt and sys_crt.is_file():
                shutil.copy2(sys_crt, d)
            elif alt_crt.is_file() and alt_crt.resolve() != d.resolve():
                shutil.copy2(alt_crt, d)


def build_kinectv2_sdk_dll(zig: Path, runtime_root: Path, deps_dir: Path, fn2_install: Path) -> Path:
    print("[5/5] Building kinectv2_sdk.dll and staging Windows x64 runtime...")
    rt_bin = runtime_root / "bin"
    rt_lib = runtime_root / "lib"
    fn2_bin = fn2_install / "bin"
    rt_bin.mkdir(parents=True, exist_ok=True)
    rt_lib.mkdir(parents=True, exist_ok=True)
    fn2_bin.mkdir(parents=True, exist_ok=True)

    sdk_dll = rt_bin / "kinectv2_sdk.dll"
    sdk_implib = rt_lib / "libkinectv2_sdk.dll.a"

    run_cmd(
        [
            str(zig), "c++", "-target", TARGET, "-std=c++17", "-shared", "-O3",
            "-DKV2_SDK_EXPORTS", "-D_CRT_SECURE_NO_WARNINGS", "-DNOMINMAX", "-DWIN32_LEAN_AND_MEAN",
            f"-I{REPO_ROOT / 'include'}",
            f"-I{fn2_install / 'include'}",
            str(REPO_ROOT / "src" / "kinectv2_sdk.cpp"),
            str(fn2_install / "lib" / "libfreenect2.dll.a"),
            f"-Wl,--out-implib,{sdk_implib}",
            "-lpsapi", "-lkernel32", "-luser32",
            "-o", str(sdk_dll),
        ]
    )

    demo_exe = rt_bin / "cpp_sdk_demo.exe"
    run_cmd(
        [
            str(zig), "c++", "-target", TARGET, "-std=c++17", "-O3",
            f"-I{REPO_ROOT / 'include'}",
            str(REPO_ROOT / "examples" / "cpp_sdk_demo.cpp"),
            str(sdk_implib),
            "-o", str(demo_exe),
        ]
    )

    dst_dirs = [fn2_bin, rt_bin]
    if os.name == "nt":
        repo_bin = REPO_ROOT / "bin"
        repo_lib = REPO_ROOT / "lib"
        repo_bin.mkdir(parents=True, exist_ok=True)
        repo_lib.mkdir(parents=True, exist_ok=True)
        shutil.copy2(sdk_dll, repo_bin / "kinectv2_sdk.dll")
        shutil.copy2(demo_exe, repo_bin / "cpp_sdk_demo.exe")
        shutil.copy2(sdk_implib, repo_lib / "libkinectv2_sdk.dll.a")
        dst_dirs.append(repo_bin)

    companions = [
        fn2_bin / "freenect2.dll",
        deps_dir / "libusb" / "bin" / "libusb-1.0.dll",
        deps_dir / "turbojpeg" / "bin" / "turbojpeg.dll",
    ]
    for dll in companions:
        for dst_dir in dst_dirs:
            dst = dst_dir / dll.name
            if dll.resolve() != dst.resolve():
                shutil.copy2(dll, dst)

    return sdk_dll


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Windows x64 pykinectskel2 runtime with zig cc/c++")
    parser.add_argument("--runtime-root", type=Path, default=default_runtime_root())
    parser.add_argument("--zig", type=str, default=None, help="Optional explicit path to zig binary")
    parser.add_argument("--bundle-src", type=Path, default=None, help="Optional existing bundle directory")
    parser.add_argument("--no-opencl", action="store_true", help="Build freenect2.dll without OpenCL support")
    args = parser.parse_args()

    zig = find_zig(args.zig)
    rt_root = args.runtime_root.expanduser().resolve()
    src_root = rt_root / "src"
    deps_dir = rt_root / "deps"
    fn2_install = rt_root / "freenect2"
    obj_root = rt_root / "obj"

    print(f"Using Zig toolchain : {zig}")
    print(f"Target runtime root : {rt_root}")

    build_libusb(zig, src_root, deps_dir, obj_root)
    build_turbojpeg(zig, src_root, deps_dir, obj_root)
    build_freenect2(zig, src_root, deps_dir, fn2_install, obj_root, enable_opencl=not args.no_opencl)
    build_worker_and_stage_bundles(zig, rt_root, args.bundle_src)
    staged_dll = build_kinectv2_sdk_dll(zig, rt_root, deps_dir, fn2_install)

    print("")
    print("SUCCESS: Windows x64 pykinectskel2 runtime built and staged.")
    print(f"  Runtime root : {rt_root}")
    print(f"  SDK DLL      : {staged_dll}")
    print(f"  freenect2 DLL: {fn2_install / 'bin' / 'freenect2.dll'}")
    print(f"  Worker bundle: {rt_root / 'dist' / 'kinectv2_windows_bundle' / 'vvtechs_worker.exe'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
