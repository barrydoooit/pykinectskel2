param(
    [string]$RuntimeRoot = "$env:LOCALAPPDATA\kinectv2-runtime"
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$BuildPy  = Join-Path $PSScriptRoot "build_windows_sdk.py"

if (Get-Command uv -ErrorAction SilentlyContinue) {
    & uv run --no-project --with ziglang python $BuildPy --runtime-root $RuntimeRoot
    if ($LASTEXITCODE -ne 0) {
        throw "build_windows_sdk.py failed with exit code $LASTEXITCODE"
    }
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python $BuildPy --runtime-root $RuntimeRoot
    if ($LASTEXITCODE -ne 0) {
        throw "build_windows_sdk.py failed with exit code $LASTEXITCODE"
    }
} else {
    throw "Neither uv nor python was found on PATH."
}
