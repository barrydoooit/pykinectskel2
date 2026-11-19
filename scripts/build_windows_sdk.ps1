param(
    [string]$RuntimeRoot = "$env:LOCALAPPDATA\kinectv2-runtime"
)

$ErrorActionPreference = "Stop"
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$SrcRoot  = Join-Path $RuntimeRoot "src"
$DepsDir  = Join-Path $RuntimeRoot "deps"

New-Item -ItemType Directory -Force -Path $RuntimeRoot, $SrcRoot, $DepsDir | Out-Null

Write-Host "[1/6] Locating Visual Studio x64 MSVC toolchain..."
$VsWhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $VsWhere)) {
    throw "vswhere.exe not found under Program Files (x86). Install Visual Studio 2019/2022 Build Tools with C++ workload."
}
$VsPath = & $VsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VsPath) {
    throw "No Visual Studio installation with VC.Tools.x86.x64 found."
}
$VcVars64 = Join-Path $VsPath "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path $VcVars64)) {
    throw "vcvars64.bat not found at $VcVars64"
}

$EnvLines = cmd.exe /c "`"$VcVars64`" >nul && set"
foreach ($line in $EnvLines) {
    if ($line -match "^([^=]+)=(.*)$") {
        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
    }
}

$VsCMakeBin = Join-Path $VsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin"
$VsNinjaBin = Join-Path $VsPath "Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja"
if ((Test-Path $VsCMakeBin) -and (-not (Get-Command cmake -ErrorAction SilentlyContinue))) {
    $env:PATH = "$VsCMakeBin;$env:PATH"
}
if ((Test-Path $VsNinjaBin) -and (-not (Get-Command ninja -ErrorAction SilentlyContinue))) {
    $env:PATH = "$VsNinjaBin;$env:PATH"
}

$CMakeGen = if (Get-Command ninja -ErrorAction SilentlyContinue) { "Ninja" } else { "NMake Makefiles" }
Write-Host "      Using MSVC from: $VsPath (CMake generator: $CMakeGen)"

# -------------------------------------------------------------------------
# [2/6] Build libusb-1.0 (v1.0.27, includes UsbDk + WinUSB/libusbK backends)
# -------------------------------------------------------------------------
$LibusbDstDll = Join-Path $DepsDir "libusb\MS64\dll\libusb-1.0.dll"
if (-not (Test-Path $LibusbDstDll)) {
    Write-Host "[2/6] Building libusb v1.0.27 (x64 Release)..."
    $LibusbSrc = Join-Path $SrcRoot "libusb"
    if (-not (Test-Path (Join-Path $LibusbSrc ".git"))) {
        if (Test-Path $LibusbSrc) { Remove-Item -Recurse -Force $LibusbSrc }
        git clone --branch v1.0.27 --depth 1 https://github.com/libusb/libusb.git $LibusbSrc
    }
    $ToolsetArgs = @()
    if ($env:VisualStudioVersion -eq "17.0") { $ToolsetArgs += "/p:PlatformToolset=v143" }
    elseif ($env:VisualStudioVersion -eq "16.0") { $ToolsetArgs += "/p:PlatformToolset=v142" }

    & msbuild (Join-Path $LibusbSrc "msvc\libusb_dll.vcxproj") `
        /p:Configuration=Release /p:Platform=x64 /p:TreatWarningAsError=false @ToolsetArgs /m
    if ($LASTEXITCODE -ne 0) { throw "libusb_dll.vcxproj MSBuild failed." }

    $BuiltDll = Get-ChildItem -Path (Join-Path $LibusbSrc "build") -Recurse -Filter "libusb-1.0.dll" |
        Where-Object { $_.FullName -match "x64" -and $_.FullName -match "Release" } |
        Select-Object -First 1
    $BuiltLib = Get-ChildItem -Path (Join-Path $LibusbSrc "build") -Recurse -Filter "libusb-1.0.lib" |
        Where-Object { $_.FullName -match "x64" -and $_.FullName -match "Release" -and $_.DirectoryName -eq $BuiltDll.DirectoryName } |
        Select-Object -First 1
    if (-not $BuiltDll -or -not $BuiltLib) {
        throw "Could not locate built libusb-1.0.dll / libusb-1.0.lib under $LibusbSrc\build"
    }

    $IncDir1 = Join-Path $DepsDir "libusb\include\libusb-1.0"
    $IncDir2 = Join-Path $DepsDir "libusb\include"
    $DllDir  = Join-Path $DepsDir "libusb\MS64\dll"
    New-Item -ItemType Directory -Force -Path $IncDir1, $IncDir2, $DllDir | Out-Null
    Copy-Item -Force (Join-Path $LibusbSrc "libusb\libusb.h") (Join-Path $IncDir1 "libusb.h")
    Copy-Item -Force (Join-Path $LibusbSrc "libusb\libusb.h") (Join-Path $IncDir2 "libusb.h")
    Copy-Item -Force $BuiltDll.FullName (Join-Path $DllDir "libusb-1.0.dll")
    Copy-Item -Force $BuiltLib.FullName (Join-Path $DllDir "libusb-1.0.lib")
} else {
    Write-Host "[2/6] Reusing cached libusb-1.0 at $LibusbDstDll"
}

# -------------------------------------------------------------------------
# [3/6] Build libjpeg-turbo (3.0.4, provides turbojpeg.dll + turbojpeg.lib)
# -------------------------------------------------------------------------
$TurboJpegDir = Join-Path $DepsDir "libjpeg-turbo64"
$TurboJpegDll = Join-Path $TurboJpegDir "bin\turbojpeg.dll"
if (-not (Test-Path $TurboJpegDll)) {
    Write-Host "[3/6] Building libjpeg-turbo 3.0.4 (x64 Release)..."
    $JpegSrc = Join-Path $SrcRoot "libjpeg-turbo"
    if (-not (Test-Path (Join-Path $JpegSrc ".git"))) {
        if (Test-Path $JpegSrc) { Remove-Item -Recurse -Force $JpegSrc }
        git clone --branch 3.0.4 --depth 1 https://github.com/libjpeg-turbo/libjpeg-turbo.git $JpegSrc
    }
    $WithSimd = if (Get-Command nasm -ErrorAction SilentlyContinue) { "ON" } else { "OFF" }
    $JpegBuild = Join-Path $JpegSrc "build_x64"
    & cmake -S $JpegSrc -B $JpegBuild -G $CMakeGen `
        -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_INSTALL_PREFIX="$TurboJpegDir" `
        -DENABLE_SHARED=ON `
        -DENABLE_STATIC=OFF `
        -DWITH_SIMD=$WithSimd `
        -DWITH_TURBOJPEG=ON `
        -DWITH_JAVA=OFF
    if ($LASTEXITCODE -ne 0) { throw "libjpeg-turbo CMake configure failed." }
    & cmake --build $JpegBuild --target install
    if ($LASTEXITCODE -ne 0) { throw "libjpeg-turbo build/install failed." }
} else {
    Write-Host "[3/6] Reusing cached turbojpeg.dll at $TurboJpegDll"
}

# -------------------------------------------------------------------------
# [4/6] Build patched libfreenect2 (freenect2.dll with clean LED shutoff)
# -------------------------------------------------------------------------
Write-Host "[4/6] Building patched libfreenect2..."
$Fn2Src = Join-Path $SrcRoot "libfreenect2"
$Fn2Install = Join-Path $RuntimeRoot "freenect2"
if (-not (Test-Path (Join-Path $Fn2Src ".git"))) {
    if (Test-Path $Fn2Src) { Remove-Item -Recurse -Force $Fn2Src }
    git clone --depth 1 https://github.com/OpenKinect/libfreenect2.git $Fn2Src
}
$PatchFile = Join-Path $RepoRoot "patches\libfreenect2_linux_sdk.patch"
& git -C $Fn2Src apply --check $PatchFile 2>$null
if ($LASTEXITCODE -eq 0) {
    & git -C $Fn2Src apply $PatchFile
    Write-Host "      Applied patches/libfreenect2_linux_sdk.patch"
} else {
    Write-Host "      Patch already applied; proceeding"
}

$Fn2Build = Join-Path $Fn2Src "build_win64"
& cmake -S $Fn2Src -B $Fn2Build -G $CMakeGen `
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_INSTALL_PREFIX="$Fn2Install" `
    -DDEPENDS_DIR="$DepsDir" `
    -DBUILD_EXAMPLES=OFF `
    -DBUILD_OPENNI2_DRIVER=OFF `
    -DENABLE_OPENGL=OFF `
    -DENABLE_CUDA=OFF `
    -DENABLE_OPENCL=ON `
    -DENABLE_CXX11=ON
if ($LASTEXITCODE -ne 0) { throw "libfreenect2 CMake configure failed." }
& cmake --build $Fn2Build --target install
if ($LASTEXITCODE -ne 0) { throw "libfreenect2 build/install failed." }

# -------------------------------------------------------------------------
# [5/6] Build vvtechs_worker.exe + k4wcll.dll and stage body-tracking bundle
# -------------------------------------------------------------------------
Write-Host "[5/6] Building vvtechs_worker.exe + k4wcll.dll and staging runtime bundles..."
$RepoBin = Join-Path $RepoRoot "bin"
New-Item -ItemType Directory -Force -Path $RepoBin | Out-Null

& cl /nologo /O2 /W3 /LD (Join-Path $RepoRoot "native\src\k4wcll_stub.c") `
    /Fe:(Join-Path $RepoBin "k4wcll.dll") `
    /Fo:(Join-Path $RepoBin "k4wcll_stub.obj") `
    /link /IMPLIB:(Join-Path $RepoBin "k4wcll_stub.lib") /NOEXP
if ($LASTEXITCODE -ne 0) { throw "k4wcll_stub.c compilation failed." }

& cl /nologo /O2 /W3 /I (Join-Path $RepoRoot "native\include") `
    (Join-Path $RepoRoot "native\src\vvtechs_worker.c") `
    /Fe:(Join-Path $RepoBin "vvtechs_worker.exe") `
    /Fo:(Join-Path $RepoBin "vvtechs_worker.obj") `
    /link ws2_32.lib d3d11.lib user32.lib
if ($LASTEXITCODE -ne 0) { throw "vvtechs_worker.c compilation failed." }
Remove-Item -Force -ErrorAction SilentlyContinue `
    (Join-Path $RepoBin "k4wcll_stub.obj"), `
    (Join-Path $RepoBin "k4wcll_stub.lib"), `
    (Join-Path $RepoBin "vvtechs_worker.obj")

$KinectSysDir = "C:\Windows\System32\Kinect"
$ExistingBundle = Join-Path $RepoRoot "dist\kinectv2_linux_bundle"
$ExistingWinBundle = Join-Path $RepoRoot "dist\kinectv2_windows_bundle"
$ModelSourceDir = $null
if (Test-Path (Join-Path $KinectSysDir "vvtechs.dll")) {
    $ModelSourceDir = $KinectSysDir
} elseif (Test-Path (Join-Path $ExistingWinBundle "vvtechs.dll")) {
    $ModelSourceDir = $ExistingWinBundle
} elseif (Test-Path (Join-Path $ExistingBundle "vvtechs.dll")) {
    $ModelSourceDir = $ExistingBundle
} else {
    throw "vvtechs.dll not found in C:\Windows\System32\Kinect or dist\kinectv2_*_bundle."
}

$RequiredAssets = @(
    "vvtechs.dll",
    "exemplardb.xmplr",
    "FernsModel.bin",
    "HeadPosition.mshdf",
    "LSVMGripReleaseRight.gbd"
)
$OptionalCrt = @("msvcr110.dll", "msvcp110.dll")

$TargetBundles = @(
    (Join-Path $RepoRoot "dist\kinectv2_windows_bundle"),
    (Join-Path $RepoRoot "dist\kinectv2_linux_bundle"),
    (Join-Path $RuntimeRoot "dist\kinectv2_windows_bundle"),
    (Join-Path $RuntimeRoot "dist\kinectv2_linux_bundle")
)

foreach ($BDir in $TargetBundles) {
    New-Item -ItemType Directory -Force -Path $BDir | Out-Null
    Copy-Item -Force (Join-Path $RepoBin "vvtechs_worker.exe") (Join-Path $BDir "vvtechs_worker.exe")
    Copy-Item -Force (Join-Path $RepoBin "k4wcll.dll") (Join-Path $BDir "k4wcll.dll")
    foreach ($asset in $RequiredAssets) {
        $srcFile = Join-Path $ModelSourceDir $asset
        $dstFile = Join-Path $BDir $asset
        if ($srcFile -ne $dstFile) {
            Copy-Item -Force $srcFile $dstFile
        }
    }
    foreach ($crt in $OptionalCrt) {
        $crtSys = Join-Path "C:\Windows\System32" $crt
        $crtAlt = Join-Path $ModelSourceDir $crt
        $dstCrt = Join-Path $BDir $crt
        if (Test-Path $crtSys) {
            Copy-Item -Force $crtSys $dstCrt
        } elseif ((Test-Path $crtAlt) -and ($crtAlt -ne $dstCrt)) {
            Copy-Item -Force $crtAlt $dstCrt
        }
    }
}

# -------------------------------------------------------------------------
# [6/6] Build kinectv2_sdk.dll and stage into %LOCALAPPDATA%\kinectv2-runtime
# -------------------------------------------------------------------------
Write-Host "[6/6] Building kinectv2_sdk.dll and staging runtime DLLs..."
$SdkBuild = Join-Path $RepoRoot "build_win64"
& cmake -S $RepoRoot -B $SdkBuild -G $CMakeGen `
    -DCMAKE_BUILD_TYPE=Release `
    -DFREENECT2_ROOT="$Fn2Install" `
    -DKV2_BUILD_VISUALIZER=OFF `
    -DKV2_BUILD_EXAMPLES=ON
if ($LASTEXITCODE -ne 0) { throw "kinectv2_sdk CMake configure failed." }
& cmake --build $SdkBuild
if ($LASTEXITCODE -ne 0) { throw "kinectv2_sdk build failed." }

$RuntimeBin = Join-Path $RuntimeRoot "bin"
$Fn2Bin     = Join-Path $Fn2Install "bin"
New-Item -ItemType Directory -Force -Path $RuntimeBin, $Fn2Bin | Out-Null

$SdkDll = Join-Path $RepoBin "kinectv2_sdk.dll"
if (-not (Test-Path $SdkDll)) {
    throw "Expected $SdkDll was not produced."
}

Copy-Item -Force $SdkDll (Join-Path $RuntimeBin "kinectv2_sdk.dll")

$CompanionDlls = @(
    (Join-Path $Fn2Bin "freenect2.dll"),
    $LibusbDstDll,
    $TurboJpegDll
)
foreach ($dll in $CompanionDlls) {
    $leaf = Split-Path $dll -Leaf
    $dstFn2 = Join-Path $Fn2Bin $leaf
    if ($dll -ne $dstFn2) { Copy-Item -Force $dll $dstFn2 }
    Copy-Item -Force $dll (Join-Path $RuntimeBin $leaf)
    Copy-Item -Force $dll (Join-Path $RepoBin $leaf)
}

Write-Host ""
Write-Host "SUCCESS: Windows x64 pykinectskel2 runtime built and staged."
Write-Host "  Runtime root : $RuntimeRoot"
Write-Host "  SDK DLL      : $(Join-Path $RuntimeBin 'kinectv2_sdk.dll')"
Write-Host "  freenect2 DLL: $(Join-Path $Fn2Bin 'freenect2.dll')"
Write-Host "  Worker bundle: $(Join-Path $RuntimeRoot 'dist\kinectv2_windows_bundle\vvtechs_worker.exe')"
