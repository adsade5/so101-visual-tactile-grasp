param(
    [string]$Command,
    [string]$CommandFile,

    [string]$PixiWorkspace = "C:\pixi_ws"
)

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$pixiCandidates = @(
    (Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"),
    (Join-Path $env:LOCALAPPDATA "pixi\bin\pixi.exe")
)
$pixiExe = $pixiCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $pixiExe) {
    $pixiCmd = Get-Command pixi -ErrorAction SilentlyContinue
    if ($pixiCmd) {
        $pixiExe = $pixiCmd.Source
    }
}
if (-not $pixiExe) {
    throw "pixi.exe was not found. Expected user-level Pixi under $env:USERPROFILE\.pixi\bin."
}

if (-not (Test-Path -LiteralPath $PixiWorkspace)) {
    throw "Pixi workspace does not exist: $PixiWorkspace"
}
$setupBat = Join-Path $PixiWorkspace "ros2-windows\local_setup.bat"
if (-not (Test-Path -LiteralPath $setupBat)) {
    throw "ROS2 local_setup.bat was not found: $setupBat"
}
if ($env:ROS_DISTRO -and ($env:ROS_DISTRO.ToLowerInvariant() -ne "lyrical")) {
    throw "Refusing to mix ROS_DISTRO=$env:ROS_DISTRO with ROS2 Lyrical."
}
if (-not $Command -and -not $CommandFile) {
    throw "Provide either -Command or -CommandFile."
}
if ($CommandFile) {
    if (-not (Test-Path -LiteralPath $CommandFile)) {
        throw "Command file does not exist: $CommandFile"
    }
    $Command = Get-Content -LiteralPath $CommandFile -Raw
}

$tempCmd = Join-Path ([System.IO.Path]::GetTempPath()) ("run_ros2_lyrical_{0}.cmd" -f ([System.Guid]::NewGuid().ToString("N")))
$vsCandidates = @(
    "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat",
    "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat",
    "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
)
$vsSetup = $vsCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
$batch = @"
@echo off
setlocal
cd /d "$PixiWorkspace"
if exist "$vsSetup" call "$vsSetup" -arch=x64 -host_arch=x64
call "$setupBat"
set "CMAKE_GENERATOR=Ninja"
if /I not "%ROS_DISTRO%"=="lyrical" (
  echo ERROR: expected ROS_DISTRO=lyrical but got ROS_DISTRO=%ROS_DISTRO%
  exit /b 120
)
echo ROS_DISTRO=%ROS_DISTRO%
echo ROS_PYTHON=%CONDA_PREFIX%\python.exe
where cl 2>NUL
where nmake 2>NUL
$Command
exit /b %ERRORLEVEL%
"@
Set-Content -LiteralPath $tempCmd -Value $batch -Encoding ASCII

try {
    Push-Location -LiteralPath $PixiWorkspace
    & $pixiExe run cmd /d /s /c "call `"$tempCmd`""
    $exitCode = $LASTEXITCODE
}
finally {
    Pop-Location
    Remove-Item -LiteralPath $tempCmd -Force -ErrorAction SilentlyContinue
}

exit $exitCode
