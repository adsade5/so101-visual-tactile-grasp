param(
    [switch]$EnableHardwareMotion,
    [switch]$TestCommandFile,
    [string]$LogDirectory,
    [string]$CommandFilePath,
    [string]$Ros2WrapperPath,
    [string]$Ros2WorkspacePath
)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
if (-not $Ros2WorkspacePath) {
    $Ros2WorkspacePath = Join-Path $ProjectRoot "ros2_ws"
}
if (-not $Ros2WrapperPath) {
    $Ros2WrapperPath = Join-Path $ProjectRoot "audit\run_in_ros2_lyrical.ps1"
}
$RuntimeLogDir = if ($LogDirectory) { $LogDirectory } else { Join-Path $ProjectRoot "logs\runtime\bridge_runtime" }
New-Item -ItemType Directory -Force -Path $RuntimeLogDir | Out-Null
$BridgeHomeDir = Join-Path $RuntimeLogDir "bridge_ros_home"
$BridgeRosLogDir = Join-Path $RuntimeLogDir "bridge_ros2"
New-Item -ItemType Directory -Force -Path $BridgeHomeDir | Out-Null
New-Item -ItemType Directory -Force -Path $BridgeRosLogDir | Out-Null
if (-not $CommandFilePath) {
    $CommandFilePath = Join-Path $RuntimeLogDir "bridge_command.cmd"
}
if ($TestCommandFile) {
    $FakeWorkspace = Join-Path $RuntimeLogDir "fake_ros2_ws"
    New-Item -ItemType Directory -Force -Path $FakeWorkspace | Out-Null
}

$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
$env:ROS_HOME = $BridgeHomeDir
$env:ROS_LOG_DIR = $BridgeRosLogDir

Write-Output "BRIDGE_RUNNER_STARTED"
Write-Output "BRIDGE_RUNNER_PROJECT_ROOT $ProjectRoot"
Write-Output "BRIDGE_RUNNER_WORKSPACE $Ros2WorkspacePath"
Write-Output "BRIDGE_RUNNER_RMW $env:RMW_IMPLEMENTATION"
Write-Output "BRIDGE_RUNNER_ROS_HOME $env:ROS_HOME"
Write-Output "BRIDGE_RUNNER_ROS_LOG_DIR $env:ROS_LOG_DIR"
Write-Output "BRIDGE_RUNNER_WRAPPER $Ros2WrapperPath"
Write-Output "BRIDGE_RUNNER_LOG_DIRECTORY $RuntimeLogDir"

$motionValue = if ($EnableHardwareMotion) { "true" } else { "false" }
if ($TestCommandFile) {
    $commandLines = @(
        "@echo off"
        "setlocal EnableExtensions"
        "echo BRIDGE_COMMAND_FILE_STARTED"
        "cd /d `"$FakeWorkspace`""
        "if errorlevel 1 exit /b 111"
        "echo BRIDGE_OVERLAY_SETUP_OK"
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"'
        'if /I not "%RMW_IMPLEMENTATION%"=="rmw_zenoh_cpp" ('
        "    echo BRIDGE_RMW_MISMATCH actual=%RMW_IMPLEMENTATION%"
        "    exit /b 121"
        ")"
        "echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%"
        "echo BRIDGE_LAUNCH_STARTING"
        "echo FAKE_BRIDGE_LAUNCH_OK"
        "exit /b 0"
    )
}
else {
    $commandLines = @(
        "@echo off"
        "setlocal EnableExtensions"
        "echo BRIDGE_COMMAND_FILE_STARTED"
        "cd /d `"$Ros2WorkspacePath`""
        "if errorlevel 1 ("
        "    echo BRIDGE_WORKSPACE_CD_FAILED code=%ERRORLEVEL%"
        "    exit /b 111"
        ")"
        'call "install\local_setup.bat"'
        "if errorlevel 1 ("
        "    echo BRIDGE_OVERLAY_SETUP_FAILED code=%ERRORLEVEL%"
        "    exit /b 112"
        ")"
        'set "RMW_IMPLEMENTATION=rmw_zenoh_cpp"'
        'if /I not "%RMW_IMPLEMENTATION%"=="rmw_zenoh_cpp" ('
        "    echo BRIDGE_RMW_MISMATCH actual=%RMW_IMPLEMENTATION%"
        "    exit /b 121"
        ")"
        "echo BRIDGE_RMW_IMPLEMENTATION %RMW_IMPLEMENTATION%"
        "echo BRIDGE_LAUNCH_STARTING"
        "ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=$motionValue"
        'set "BRIDGE_RC=%ERRORLEVEL%"'
        "echo BRIDGE_LAUNCH_EXIT code=%BRIDGE_RC%"
        "exit /b %BRIDGE_RC%"
    )
}

Set-Content -LiteralPath $CommandFilePath -Value $commandLines -Encoding ASCII

try {
    Write-Output "BRIDGE_RUNNER_COMMAND_FILE $CommandFilePath"
    Write-Output "BRIDGE_COMMAND_FILE_CONTENT_BEGIN"
    foreach ($line in Get-Content -LiteralPath $CommandFilePath) {
        Write-Output $line
    }
    Write-Output "BRIDGE_COMMAND_FILE_CONTENT_END"
    & $Ros2WrapperPath -CommandFile $CommandFilePath
    $wrapperExit = $LASTEXITCODE
    Write-Output "BRIDGE_RUNNER_WRAPPER_EXIT code=$wrapperExit"
    exit $wrapperExit
}
catch {
    Write-Error "BRIDGE_RUNNER_EXCEPTION type=$($_.Exception.GetType().FullName) message=$($_.Exception.Message)"
    if ($_.ScriptStackTrace) {
        Write-Error $_.ScriptStackTrace
    }
    exit 1
}
