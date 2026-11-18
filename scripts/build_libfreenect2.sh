#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_PREFIX="${1:-$HOME/.local/share/kinectv2-runtime/freenect2}"
SRC_DIR="${FREENECT2_SRC_DIR:-$HOME/.local/share/kinectv2-runtime/src/libfreenect2}"

mkdir -p "$(dirname "$SRC_DIR")"
if [ ! -d "$SRC_DIR/.git" ]; then
    git clone https://github.com/OpenKinect/libfreenect2.git "$SRC_DIR"
fi

if git -C "$SRC_DIR" apply --check "$REPO_ROOT/patches/libfreenect2_linux_sdk.patch" 2>/dev/null; then
    git -C "$SRC_DIR" apply "$REPO_ROOT/patches/libfreenect2_linux_sdk.patch"
    echo "[build_libfreenect2] Applied patches/libfreenect2_linux_sdk.patch"
else
    echo "[build_libfreenect2] Patch already applied or tree modified; proceeding to build"
fi

cmake -S "$SRC_DIR" -B "$SRC_DIR/build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_OPENNI2_DRIVER=OFF \
    -DENABLE_CXX11=ON

cmake --build "$SRC_DIR/build" --target install -j"$(nproc)"
echo "[build_libfreenect2] Installed patched libfreenect2 to $INSTALL_PREFIX"
