#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="$HOME/.local/bin:$PATH"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export DXVK_FILTER_DEVICE_NAME="${DXVK_FILTER_DEVICE_NAME:-NVIDIA}"
export LIBFREENECT2_IR_TRANSFERS="${LIBFREENECT2_IR_TRANSFERS:-24}"

exec "$REPO_ROOT/bin/kinectv2_visualizer" "$@"
