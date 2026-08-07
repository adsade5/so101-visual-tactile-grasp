# MVP-4E: Simple multi-terminal opener for final manual acceptance.
#
# This is NOT a supervisor. It does not:
#   - wait for readiness
#   - parse logs
#   - judge TCP state
#   - manage PID trees
#   - auto-close processes
#   - auto-restart
#   - auto-reconnect
#   - generate .cmd files
#   - scan for ERROR keywords
#   - auto-execute motion
#
# It opens 4 independent PowerShell windows with the fixed commands below.
# The user inspects each terminal by eye before running plan-only / execute
# manually in the workspace PowerShell.

$ErrorActionPreference = "Stop"

$ProjectRoot = "E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp"
$Ros2Wrapper = Join-Path $ProjectRoot "audit\run_in_ros2_lyrical.ps1"

# --- pre-flight checks -------------------------------------------------------
if (-not (Test-Path $ProjectRoot)) {
    Write-Error "Project root not found: $ProjectRoot"
    exit 1
}
if (-not (Test-Path $Ros2Wrapper)) {
    Write-Error "ROS2 wrapper not found: $Ros2Wrapper"
    exit 1
}

$CondaExe = "E:\Anaconda\Scripts\conda.exe"
if (-not (Test-Path $CondaExe)) {
    Write-Error "conda.exe not found: $CondaExe"
    exit 1
}

Write-Host "=== MVP-4E opening 4 terminals ==="
Write-Host ""

# --- Terminal 0: Zenoh -------------------------------------------------------
Write-Host "[0] Zenoh ..."
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "`$Host.UI.RawUI.WindowTitle='MVP4E - 0 Zenoh'; " +
    "Write-Host 'Starting Zenoh router...'; " +
    "cd '$ProjectRoot'; " +
    "& '$Ros2Wrapper' -Command 'ros2 run rmw_zenoh_cpp rmw_zenohd'"
)
Start-Sleep -Seconds 1

# --- Terminal 1: LeRobot Server ----------------------------------------------
Write-Host "[1] Server COM4 COM8 ..."
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "`$Host.UI.RawUI.WindowTitle='MVP4E - 1 Server COM4 COM8'; " +
    "Write-Host 'Starting LeRobot server (COM4 + COM8 tactile)...'; " +
    "cd '$ProjectRoot'; " +
    "& '$CondaExe' run --no-capture-output -p 'E:\Anaconda\envs_dirs\lerobot' " +
    "python -u scripts\mvp_so101_server.py " +
    "--config config\mvp_hardware.json " +
    "--enable-hardware-motion"
)
Start-Sleep -Seconds 1

# --- Terminal 2: ROS2 Hardware Bridge ----------------------------------------
Write-Host "[2] ROS2 Bridge ..."
$BridgeCommand = "cd /d $ProjectRoot\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "`$Host.UI.RawUI.WindowTitle='MVP4E - 2 ROS2 Bridge'; " +
    "Write-Host 'Starting ROS2 hardware bridge...'; " +
    "cd '$ProjectRoot'; " +
    "& '$Ros2Wrapper' -Command '$BridgeCommand'"
)
Start-Sleep -Seconds 1

# --- Terminal 3: Vision Nodes ------------------------------------------------
Write-Host "[3] Vision ..."
$VisionCommand = "cd /d $ProjectRoot\ros2_ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py"
Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-Command",
    "`$Host.UI.RawUI.WindowTitle='MVP4E - 3 Vision'; " +
    "Write-Host 'Starting visual perception nodes...'; " +
    "cd '$ProjectRoot'; " +
    "& '$Ros2Wrapper' -Command '$VisionCommand'"
)
Start-Sleep -Seconds 1

# --- done --------------------------------------------------------------------
Write-Host ""
Write-Host "=== All 4 terminals opened ==="
Write-Host ""
Write-Host "Manual checklist (read each window by eye):"
Write-Host "  [0] Zenoh running normally"
Write-Host "  [1] TACTILE_SERIAL_OPENED port=COM8"
Write-Host "      TACTILE_BASELINE_COMPLETED"
Write-Host "      TACTILE_READY true"
Write-Host "      ROBOT_CONNECTED port=COM4"
Write-Host "      TCP_SERVER_LISTENING"
Write-Host "  [2] Bridge node running, BRIDGE_TCP_CONNECTED, BRIDGE_TCP_READY true"
Write-Host "  [3] Object pose publishing, vision nodes healthy"
Write-Host ""
Write-Host "Only when ALL four are healthy, run in THIS terminal:"
Write-Host ""
Write-Host "  plan-only:"
Write-Host "    cd $ProjectRoot"
Write-Host "    & '$Ros2Wrapper' -Command 'cd /d $ProjectRoot\ros2_ws && call install\local_setup.bat && cd /d $ProjectRoot && python scripts\mvp_visual_grasp.py --plan-only'"
Write-Host ""
Write-Host "  execute (only after plan-only PASS):"
Write-Host "    cd $ProjectRoot"
Write-Host "    & '$Ros2Wrapper' -Command 'cd /d $ProjectRoot\ros2_ws && call install\local_setup.bat && cd /d $ProjectRoot && python scripts\mvp_visual_grasp.py --execute --confirm VISUAL_GRASP'"
Write-Host ""
Write-Host "  tactile test (optional):"
Write-Host "    cd $ProjectRoot"
Write-Host "    & '$Ros2Wrapper' -Command 'cd /d $ProjectRoot\ros2_ws && call install\local_setup.bat && cd /d $ProjectRoot && python scripts\mvp_visual_grasp.py --tactile-test'"
Write-Host ""
Write-Host "Ctrl+C shutdown order: [3] Vision -> [2] Bridge -> [1] Server -> [0] Zenoh -> Follower power off"
