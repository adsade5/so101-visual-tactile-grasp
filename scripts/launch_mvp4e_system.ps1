param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("TactileTest", "PlanOnly", "FinalAcceptance")]
    [string]$Mode
)

$ErrorActionPreference = "Stop"

$ProjectRoot = "E:\PycharmProjects\Embodied_AI\LeRobot_Project\so101_visual_tactile_grasp"
$CondaExe = "E:\Anaconda\Scripts\conda.exe"
$LeRobotEnv = "E:\Anaconda\envs_dirs\lerobot"
$Ros2Wrapper = Join-Path $ProjectRoot "audit\run_in_ros2_lyrical.ps1"
$Ros2Ws = Join-Path $ProjectRoot "ros2_ws"
$TimeStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$LogDir = Join-Path $ProjectRoot "logs\runtime\$TimeStamp"
$LauncherLog = Join-Path $LogDir "launcher.log"
$ActionLog = Join-Path $LogDir "action.log"

$script:Managed = [ordered]@{}
$script:ActionProcess = $null

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null

function Write-LauncherLog {
    param([string]$Message)
    $line = "{0} {1}" -f (Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"), $Message
    Add-Content -LiteralPath $LauncherLog -Value $line -Encoding UTF8
}

function Write-Step {
    param([string]$Message)
    Write-Host $Message
    Write-LauncherLog $Message
}

function Assert-PathExists {
    param([string]$Path, [string]$Label)
    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Label not found: $Path"
    }
}

function Assert-NoOldLauncher {
    try {
        $matches = Get-CimInstance Win32_Process |
            Where-Object {
                $_.ProcessId -ne $PID -and
                $_.CommandLine -and
                $_.CommandLine.Contains("launch_mvp4e_system.ps1") -and
                $_.CommandLine.Contains($ProjectRoot)
            }
        if ($matches) {
            $ids = ($matches | ForEach-Object { $_.ProcessId }) -join ","
            throw "Existing project launcher instance detected: $ids"
        }
        Write-LauncherLog "old_launcher_check=pass"
    }
    catch {
        Write-LauncherLog "old_launcher_check=warning message=$($_.Exception.Message)"
    }
}

function New-ProcessScript {
    param(
        [string]$Name,
        [string]$Command,
        [string]$LogFile
    )
    $scriptPath = Join-Path $LogDir "$Name.ps1"
    $body = @"
`$ErrorActionPreference = "Continue"
Set-Location -LiteralPath "$ProjectRoot"
& {
$Command
} *> "$LogFile"
exit `$LASTEXITCODE
"@
    Set-Content -LiteralPath $scriptPath -Value $body -Encoding ASCII
    return $scriptPath
}

function Start-ManagedCommand {
    param(
        [string]$Name,
        [string]$Command,
        [string]$LogName
    )
    $logFile = Join-Path $LogDir $LogName
    $scriptPath = New-ProcessScript -Name $Name -Command $Command -LogFile $logFile
    $process = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $scriptPath) `
        -WorkingDirectory $ProjectRoot `
        -PassThru `
        -WindowStyle Hidden
    $script:Managed[$Name] = [pscustomobject]@{
        Name = $Name
        Process = $process
        Log = $logFile
        Script = $scriptPath
    }
    Write-LauncherLog "started name=$Name pid=$($process.Id) log=$logFile"
    return $process
}

function Test-ManagedAlive {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return $false
    }
    $proc = $script:Managed[$Name].Process
    try {
        $proc.Refresh()
        return -not $proc.HasExited
    }
    catch {
        return $false
    }
}

function Wait-ManagedAlive {
    param([string]$Name, [double]$TimeoutSec)
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        if (Test-ManagedAlive $Name) {
            return
        }
        Start-Sleep -Milliseconds 200
    }
    throw "process_not_alive:$Name"
}

function Wait-LogPattern {
    param(
        [string]$Name,
        [string]$Pattern,
        [double]$TimeoutSec,
        [string]$FailedStage
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $logFile = $script:Managed[$Name].Log
    while ((Get-Date) -lt $deadline) {
        if (-not (Test-ManagedAlive $Name)) {
            throw "$FailedStage process_exited name=$Name log=$logFile"
        }
        if ((Test-Path -LiteralPath $logFile) -and (Select-String -LiteralPath $logFile -Pattern $Pattern -Quiet)) {
            return
        }
        Start-Sleep -Milliseconds 200
    }
    throw "$FailedStage timeout waiting_for=$Pattern log=$logFile"
}

function Invoke-Ros2TopicProbe {
    param(
        [string]$Topic,
        [string]$ExpectedPattern,
        [double]$TimeoutSec
    )
    $probeName = "probe_" + ($Topic -replace "[^A-Za-z0-9]", "_") + "_" + ([System.Guid]::NewGuid().ToString("N").Substring(0, 8))
    $probeLog = Join-Path $LogDir "$probeName.log"
    $command = "& `"$Ros2Wrapper`" -Command `"cd /d $Ros2Ws && call install\local_setup.bat && ros2 topic echo --once $Topic`""
    $scriptPath = New-ProcessScript -Name $probeName -Command $command -LogFile $probeLog
    $proc = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $scriptPath) `
        -WorkingDirectory $ProjectRoot `
        -PassThru `
        -WindowStyle Hidden
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        $proc.Refresh()
        if ($proc.HasExited) {
            break
        }
        Start-Sleep -Milliseconds 200
    }
    $proc.Refresh()
    if (-not $proc.HasExited) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    $text = ""
    if (Test-Path -LiteralPath $probeLog) {
        $text = Get-Content -LiteralPath $probeLog -Raw
    }
    if ($text -notmatch $ExpectedPattern) {
        throw "ros2_topic_probe_failed topic=$Topic expected=$ExpectedPattern log=$probeLog"
    }
}

function Wait-Ros2TopicPattern {
    param(
        [string]$Topic,
        [string]$ExpectedPattern,
        [double]$TimeoutSec,
        [string]$FailedStage
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        try {
            Invoke-Ros2TopicProbe -Topic $Topic -ExpectedPattern $ExpectedPattern -TimeoutSec 2.0
            return
        }
        catch {
            Write-LauncherLog "topic_probe_retry topic=$Topic reason=$($_.Exception.Message)"
        }
        Start-Sleep -Milliseconds 200
    }
    throw "$FailedStage timeout topic=$Topic expected=$ExpectedPattern"
}

function Get-JsonObjectsFromText {
    param([string]$Text)
    $lines = $Text -split "`r?`n"
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i].TrimStart().StartsWith("{")) {
            $candidate = ($lines[$i..($lines.Count - 1)] -join "`n")
            try {
                return $candidate | ConvertFrom-Json -ErrorAction Stop
            }
            catch {
            }
        }
    }
    return $null
}

function Invoke-ActionCommand {
    param([string]$Arguments)
    $command = "cd /d $ProjectRoot && python scripts\mvp_visual_grasp.py $Arguments"
    $full = "& `"$Ros2Wrapper`" -Command `"$command`""
    Write-LauncherLog "action_start arguments=$Arguments"
    $output = & powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $full 2>&1
    $exitCode = $LASTEXITCODE
    $outputText = ($output | Out-String)
    Add-Content -LiteralPath $ActionLog -Value $outputText -Encoding UTF8
    Write-LauncherLog "action_exit arguments=$Arguments exit_code=$exitCode"
    return [pscustomobject]@{
        ExitCode = $exitCode
        Text = $outputText
        Json = Get-JsonObjectsFromText -Text $outputText
    }
}

function Stop-OwnedProcessTree {
    param([int]$RootPid)
    $ids = New-Object System.Collections.Generic.List[int]
    $ids.Add($RootPid)
    try {
        $all = Get-CimInstance Win32_Process
        $changed = $true
        while ($changed) {
            $changed = $false
            foreach ($proc in $all) {
                if ($ids.Contains([int]$proc.ParentProcessId) -and -not $ids.Contains([int]$proc.ProcessId)) {
                    $ids.Add([int]$proc.ProcessId)
                    $changed = $true
                }
            }
        }
    }
    catch {
        Write-LauncherLog "child_tree_lookup_warning pid=$RootPid message=$($_.Exception.Message)"
    }
    foreach ($id in ($ids | Sort-Object -Descending)) {
        try {
            $proc = Get-Process -Id $id -ErrorAction SilentlyContinue
            if ($proc) {
                Stop-Process -Id $id -Force -ErrorAction SilentlyContinue
                Write-LauncherLog "stopped pid=$id"
            }
        }
        catch {
            Write-LauncherLog "stop_warning pid=$id message=$($_.Exception.Message)"
        }
    }
}

function Cleanup {
    Write-LauncherLog "cleanup_start order=action,visual_nodes,ros2_bridge,lerobot_server,zenoh"
    foreach ($name in @("vision", "bridge", "server", "zenoh")) {
        if ($script:Managed.Contains($name)) {
            $proc = $script:Managed[$name].Process
            try {
                $proc.Refresh()
                if (-not $proc.HasExited) {
                    Stop-OwnedProcessTree -RootPid $proc.Id
                }
                else {
                    Write-LauncherLog "already_exited name=$name pid=$($proc.Id) exit_code=$($proc.ExitCode)"
                }
            }
            catch {
                Write-LauncherLog "cleanup_warning name=$name message=$($_.Exception.Message)"
            }
        }
    }
    Write-LauncherLog "cleanup_complete"
}

function Start-Zenoh {
    Write-Step "[1/5] Starting Zenoh..."
    Start-ManagedCommand -Name "zenoh" -LogName "zenoh.log" -Command "& `"$Ros2Wrapper`" -Command `"ros2 run rmw_zenoh_cpp rmw_zenohd`"" | Out-Null
    Wait-ManagedAlive -Name "zenoh" -TimeoutSec 10.0
    Write-Step "[1/5] Zenoh ready"
}

function Start-Server {
    Write-Step "[2/5] Starting LeRobot server..."
    $cmd = "& `"$CondaExe`" run -p `"$LeRobotEnv`" python scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion"
    Start-ManagedCommand -Name "server" -LogName "server.log" -Command $cmd | Out-Null
    Wait-LogPattern -Name "server" -Pattern "TCP_SERVER_LISTENING" -TimeoutSec 30.0 -FailedStage "server_start"
    Write-Step "[2/5] COM4 connected"
    Wait-LogPattern -Name "server" -Pattern "TACTILE_SERIAL_OPENED port=COM8" -TimeoutSec 20.0 -FailedStage "tactile_open"
    Write-Step "[2/5] COM8 connected"
    Wait-LogPattern -Name "server" -Pattern "TACTILE_BASELINE_COMPLETED" -TimeoutSec 20.0 -FailedStage "tactile_baseline"
    Wait-LogPattern -Name "server" -Pattern "TACTILE_READY true" -TimeoutSec 5.0 -FailedStage "tactile_ready"
    Write-Step "[2/5] Tactile ready"
}

function Start-Bridge {
    Write-Step "[3/5] Starting ROS2 bridge..."
    $bridgeCommand = "cd /d $Ros2Ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
    Start-ManagedCommand -Name "bridge" -LogName "bridge.log" -Command "& `"$Ros2Wrapper`" -Command `"$bridgeCommand`"" | Out-Null
    Wait-LogPattern -Name "bridge" -Pattern "BRIDGE_TCP_CONNECTED" -TimeoutSec 15.0 -FailedStage "bridge_tcp"
    Wait-LogPattern -Name "bridge" -Pattern "BRIDGE_TCP_READY true" -TimeoutSec 5.0 -FailedStage "bridge_ready"
    Wait-Ros2TopicPattern -Topic "/mvp/tcp_connected" -ExpectedPattern "data:\s*true" -TimeoutSec 8.0 -FailedStage "tcp_connected_topic"
    Wait-Ros2TopicPattern -Topic "/mvp/tactile_ready" -ExpectedPattern "data:\s*true" -TimeoutSec 8.0 -FailedStage "tactile_ready_topic"
    Write-Step "[3/5] TCP connected"
}

function Start-Vision {
    Write-Step "[4/5] Starting visual nodes..."
    $visionCommand = "cd /d $Ros2Ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_pregrasp_preview.launch.py"
    Start-ManagedCommand -Name "vision" -LogName "vision.log" -Command "& `"$Ros2Wrapper`" -Command `"$visionCommand`"" | Out-Null
    Wait-Ros2TopicPattern -Topic "/object_pose_base" -ExpectedPattern "pose:" -TimeoutSec 15.0 -FailedStage "object_pose"
    Write-Step "[4/5] Object pose ready"
}

function Print-PlanSummary {
    param([object]$Plan)
    Write-Host "PLAN_ONLY PASS"
    Write-Host ("Object position: {0}" -f (($Plan.object_pose_base.position_m | ConvertTo-Json -Compress) 2>$null))
    Write-Host ("Pregrasp: {0}" -f (($Plan.pregrasp_pose_base.position_m | ConvertTo-Json -Compress) 2>$null))
    Write-Host ("Descent waypoints: {0}" -f $Plan.waypoint_count)
    Write-Host ("Lift waypoints: {0}" -f $Plan.lift_waypoint_count)
    Write-Host ("TCP connected: {0}" -f $Plan.tcp_connected)
    Write-Host ("Tactile ready: {0}" -f $Plan.tactile_ready_before_motion)
    Write-Host ("Tactile contact now: {0}" -f $Plan.tactile_contact_before_motion)
}

try {
    Assert-PathExists -Path $CondaExe -Label "Conda executable"
    Assert-PathExists -Path $LeRobotEnv -Label "LeRobot Conda env"
    Assert-PathExists -Path $Ros2Wrapper -Label "ROS2 wrapper"
    Assert-PathExists -Path $Ros2Ws -Label "ROS2 workspace"
    Assert-NoOldLauncher
    Write-LauncherLog "mode=$Mode log_dir=$LogDir"

    Start-Zenoh
    Start-Server
    Start-Bridge

    if ($Mode -eq "TactileTest") {
        Write-Step "[5/5] Running tactile test..."
        $result = Invoke-ActionCommand -Arguments "--tactile-test"
        if ($result.ExitCode -ne 0 -or -not $result.Json.success) {
            throw "tactile_test_failed log=$ActionLog"
        }
        Write-Step "TACTILE_TEST PASS"
        Write-Host ($result.Json | ConvertTo-Json -Depth 20)
        return
    }

    Start-Vision
    Write-Step "[5/5] Running plan-only..."
    $plan = Invoke-ActionCommand -Arguments "--plan-only"
    if ($plan.ExitCode -ne 0 -or -not $plan.Json.success) {
        Write-Step "PLAN_ONLY FAIL"
        Write-Host ($plan.Json | ConvertTo-Json -Depth 20)
        throw "plan_only_failed log=$ActionLog"
    }
    Print-PlanSummary -Plan $plan.Json

    if ($Mode -eq "PlanOnly") {
        Write-Host ($plan.Json | ConvertTo-Json -Depth 20)
        return
    }

    $confirm = Read-Host "Type VISUAL_GRASP to execute"
    if ($confirm -cne "VISUAL_GRASP") {
        Write-Step "Execution cancelled: confirmation mismatch"
        Write-Host (@{
            success = $false
            reason = "confirmation_mismatch"
            hardware_command_sent = $false
            log_dir = $LogDir
        } | ConvertTo-Json -Depth 10)
        return
    }

    Write-Step "[5/5] Executing final visual grasp..."
    $execute = Invoke-ActionCommand -Arguments "--execute --confirm VISUAL_GRASP"
    Write-Host ($execute.Json | ConvertTo-Json -Depth 30)
    if ($execute.ExitCode -ne 0 -or -not $execute.Json.success) {
        throw "final_acceptance_execute_failed log=$ActionLog"
    }
}
catch {
    Write-Step "FAILED_STAGE"
    Write-Step ("FAILED_REASON {0}" -f $_.Exception.Message)
    Write-Step ("LOG_DIR {0}" -f $LogDir)
    throw
}
finally {
    Cleanup
    Write-Host "Logs: $LogDir"
}
