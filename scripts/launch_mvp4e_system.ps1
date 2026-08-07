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
$HardwareConfig = Join-Path $ProjectRoot "config\mvp_hardware.json"
$ActiveManifest = Join-Path $ProjectRoot "logs\runtime\active_launcher.json"
$TimeStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$LogDir = Join-Path $ProjectRoot "logs\runtime\$TimeStamp"
$LauncherLog = Join-Path $LogDir "launcher.log"
$ActionLog = Join-Path $LogDir "action.log"

$ZenohStartTimeoutS = 10.0
$ZenohStabilityWindowS = 1.0
$ServerProcessStartTimeoutS = 10.0
$TactileSerialOpenTimeoutS = 15.0
$TactileBaselineTimeoutS = 30.0
$RobotConnectTimeoutS = 30.0
$TcpListenTimeoutS = 15.0

$ZenohReadyMarkers = @("zenohd", "Started", "router", "listening", "scouting")
$FatalPatterns = @("Traceback", "Fatal", "FATAL", "ERROR", "Error", "Address already in use", "bind failed", "panic")

$script:Managed = [ordered]@{}

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $ActiveManifest) | Out-Null

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

function Read-TextFileLive {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        return ""
    }
    try {
        return [System.IO.File]::ReadAllText($Path, [System.Text.Encoding]::UTF8)
    }
    catch {
        Write-LauncherLog "log_read_retry path=$Path message=$($_.Exception.Message)"
        return ""
    }
}

function Get-LogTail {
    param([string]$Path, [int]$Lines = 100)
    if (-not (Test-Path -LiteralPath $Path)) {
        return @()
    }
    try {
        return Get-Content -LiteralPath $Path -Tail $Lines -ErrorAction Stop
    }
    catch {
        return @("LOG_READ_FAILED $($_.Exception.Message)")
    }
}

function Get-ProcessCommandLine {
    param([int]$ProcessId)
    try {
        $proc = Get-CimInstance Win32_Process -Filter "ProcessId=$ProcessId" -ErrorAction Stop
        return [string]$proc.CommandLine
    }
    catch {
        return ""
    }
}

function Get-DescendantProcesses {
    param([int]$RootPid)
    $result = @()
    try {
        $all = @(Get-CimInstance Win32_Process -ErrorAction Stop)
        $ids = New-Object System.Collections.Generic.HashSet[int]
        [void]$ids.Add($RootPid)
        $changed = $true
        while ($changed) {
            $changed = $false
            foreach ($proc in $all) {
                if ($ids.Contains([int]$proc.ParentProcessId) -and -not $ids.Contains([int]$proc.ProcessId)) {
                    [void]$ids.Add([int]$proc.ProcessId)
                    $result += $proc
                    $changed = $true
                }
            }
        }
    }
    catch {
        Write-LauncherLog "descendant_lookup_warning root_pid=$RootPid message=$($_.Exception.Message)"
    }
    return @($result)
}

function Update-ComponentProcessTree {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return
    }
    $entry = $script:Managed[$Name]
    $desc = @(Get-DescendantProcesses -RootPid $entry.RootPid)
    $entry.DescendantPids = @($desc | ForEach-Object { [int]$_.ProcessId })
    $entry.CommandLines = @(
        "root=$($entry.CommandLine)"
        $desc | ForEach-Object { "pid=$($_.ProcessId) $($_.CommandLine)" }
    )
    Write-LauncherLog "COMPONENT_PROCESS_STARTED name=$Name root_pid=$($entry.RootPid) descendants=[$(($entry.DescendantPids) -join ',')]"
}

function Save-Manifest {
    $components = @{}
    foreach ($name in $script:Managed.Keys) {
        $entry = $script:Managed[$name]
        Update-ComponentProcessTree -Name $name
        $components[$name] = @{
            root_pid = $entry.RootPid
            descendant_pids = $entry.DescendantPids
            log = $entry.Log
            command_line = $entry.CommandLine
        }
    }
    $manifest = @{
        launcher_pid = $PID
        start_time = (Get-Date).ToString("o")
        project_root = $ProjectRoot
        log_dir = $LogDir
        zenoh_root_pid = if ($components.ContainsKey("zenoh")) { $components["zenoh"].root_pid } else { $null }
        server_root_pid = if ($components.ContainsKey("server")) { $components["server"].root_pid } else { $null }
        bridge_root_pid = if ($components.ContainsKey("bridge")) { $components["bridge"].root_pid } else { $null }
        vision_root_pid = if ($components.ContainsKey("vision")) { $components["vision"].root_pid } else { $null }
        action_root_pid = $null
        components = $components
    }
    $manifest | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ActiveManifest -Encoding UTF8
}

function Test-CommandLineOwned {
    param([int]$Pid, [string]$CommandLine)
    if ([string]::IsNullOrWhiteSpace($CommandLine)) {
        return $false
    }
    return $CommandLine.Contains($ProjectRoot) -or
        $CommandLine.Contains("rmw_zenohd") -or
        $CommandLine.Contains("mvp_so101_server.py") -or
        $CommandLine.Contains("mvp_hardware_bridge_motion_enabled.launch.py") -or
        $CommandLine.Contains("mvp_pregrasp_preview.launch.py") -or
        $CommandLine.Contains("mvp_visual_grasp.py")
}

function Stop-RecordedOwnedPid {
    param([int]$Pid, [string]$Name)
    $cmd = Get-ProcessCommandLine -ProcessId $Pid
    if (-not (Test-CommandLineOwned -Pid $Pid -CommandLine $cmd)) {
        Write-LauncherLog "stale_child_skip_unowned name=$Name pid=$Pid command_line=$cmd"
        return
    }
    $proc = Get-Process -Id $Pid -ErrorAction SilentlyContinue
    if ($proc) {
        Write-LauncherLog "COMPONENT_PROCESS_STOPPING name=$Name pid=$Pid"
        Stop-Process -Id $Pid -Force -ErrorAction SilentlyContinue
        Start-Sleep -Milliseconds 100
        $again = Get-Process -Id $Pid -ErrorAction SilentlyContinue
        $exitCode = if ($again) { "still_running" } else { "stopped" }
        Write-LauncherLog "COMPONENT_PROCESS_STOPPED name=$Name pid=$Pid exit_code=$exitCode"
    }
}

function Assert-NoOldLauncher {
    if (-not (Test-Path -LiteralPath $ActiveManifest)) {
        Write-LauncherLog "old_launcher_check=pass"
        return
    }
    $raw = Read-TextFileLive -Path $ActiveManifest
    if (-not $raw) {
        Remove-Item -LiteralPath $ActiveManifest -Force -ErrorAction SilentlyContinue
        Write-LauncherLog "old_launcher_check=removed_empty_manifest"
        return
    }
    $manifest = $raw | ConvertFrom-Json
    $oldLauncherPid = [int]$manifest.launcher_pid
    $oldLauncher = Get-Process -Id $oldLauncherPid -ErrorAction SilentlyContinue
    if ($oldLauncher) {
        throw "Existing project launcher instance detected: $oldLauncherPid"
    }
    Write-LauncherLog "stale_launcher_manifest_detected path=$ActiveManifest"
    foreach ($component in $manifest.components.PSObject.Properties) {
        $name = $component.Name
        $value = $component.Value
        if ($value.root_pid) {
            Stop-RecordedOwnedPid -Pid ([int]$value.root_pid) -Name $name
        }
        foreach ($pidValue in @($value.descendant_pids)) {
            if ($pidValue) {
                Stop-RecordedOwnedPid -Pid ([int]$pidValue) -Name $name
            }
        }
    }
    Remove-Item -LiteralPath $ActiveManifest -Force -ErrorAction SilentlyContinue
    Write-LauncherLog "old_launcher_check=pass cleaned_stale_manifest=true"
}

function New-ProcessScript {
    param([string]$Name, [string]$Command, [string]$LogFile)
    $scriptPath = Join-Path $LogDir "$Name.ps1"
    $body = @"
`$ErrorActionPreference = "Continue"
Set-Location -LiteralPath "$ProjectRoot"
`$env:PYTHONUNBUFFERED = "1"
& {
$Command
} *> "$LogFile"
exit `$LASTEXITCODE
"@
    Set-Content -LiteralPath $scriptPath -Value $body -Encoding ASCII
    return $scriptPath
}

function Start-ManagedCommand {
    param([string]$Name, [string]$Command, [string]$LogName)
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
        RootPid = [int]$process.Id
        DescendantPids = @()
        Log = $logFile
        Script = $scriptPath
        CommandLine = Get-ProcessCommandLine -ProcessId ([int]$process.Id)
        Command = $Command
    }
    Update-ComponentProcessTree -Name $Name
    Save-Manifest
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

function Get-ManagedExitCode {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return $null
    }
    $proc = $script:Managed[$Name].Process
    try {
        $proc.Refresh()
        if ($proc.HasExited) {
            return $proc.ExitCode
        }
    }
    catch {
    }
    return $null
}

function Test-LogHasFatal {
    param([string]$Path)
    $text = Read-TextFileLive -Path $Path
    foreach ($pattern in $FatalPatterns) {
        if ($text -match [regex]::Escape($pattern)) {
            return $pattern
        }
    }
    return $null
}

function Write-ComponentFailure {
    param([string]$Stage, [string]$Reason, [string]$Name)
    $entry = if ($script:Managed.Contains($Name)) { $script:Managed[$Name] } else { $null }
    $rootPid = if ($entry) { $entry.RootPid } else { $null }
    $desc = if ($entry) { ($entry.DescendantPids -join ",") } else { "" }
    $exitCode = if ($entry) { Get-ManagedExitCode -Name $Name } else { $null }
    $logFile = if ($entry) { $entry.Log } else { "" }
    $empty = $true
    if ($logFile -and (Test-Path -LiteralPath $logFile)) {
        $empty = ((Get-Item -LiteralPath $logFile).Length -eq 0)
    }
    Write-Step "FAILED_STAGE $Stage"
    Write-Step "FAILED_REASON $Reason"
    Write-Step "COMPONENT $Name"
    Write-Step "ROOT_PID $rootPid"
    Write-Step "DESCENDANT_PIDS [$desc]"
    Write-Step "EXIT_CODE $exitCode"
    Write-Step "LOG_FILE $logFile"
    Write-Step ("{0}_LOG_EMPTY {1}" -f $Name.ToUpperInvariant(), $empty.ToString().ToLowerInvariant())
    Write-Host "LOG_TAIL_BEGIN"
    foreach ($line in Get-LogTail -Path $logFile -Lines 100) {
        Write-Host $line
    }
    Write-Host "LOG_TAIL_END"
    Write-Step "LOG_DIR $LogDir"
}

function Wait-ComponentLogPattern {
    param(
        [string]$Name,
        [string]$Pattern,
        [double]$TimeoutSec,
        [string]$FailedStage
    )
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $logFile = $script:Managed[$Name].Log
    while ((Get-Date) -lt $deadline) {
        Update-ComponentProcessTree -Name $Name
        if (-not (Test-ManagedAlive $Name)) {
            Write-ComponentFailure -Stage $FailedStage -Reason "${Name}_process_exited" -Name $Name
            throw "$FailedStage ${Name}_process_exited"
        }
        $fatal = Test-LogHasFatal -Path $logFile
        if ($fatal) {
            Write-ComponentFailure -Stage $FailedStage -Reason "${Name}_log_error:$fatal" -Name $Name
            throw "$FailedStage ${Name}_log_error:$fatal"
        }
        $text = Read-TextFileLive -Path $logFile
        if ($text -match [regex]::Escape($Pattern)) {
            return
        }
        Start-Sleep -Milliseconds 200
    }
    Write-ComponentFailure -Stage $FailedStage -Reason "timeout waiting_for=$Pattern" -Name $Name
    throw "$FailedStage timeout waiting_for=$Pattern"
}

function Wait-ZenohReady {
    $deadline = (Get-Date).AddSeconds($ZenohStartTimeoutS)
    $logFile = $script:Managed["zenoh"].Log
    $marker = $null
    while ((Get-Date) -lt $deadline) {
        Update-ComponentProcessTree -Name "zenoh"
        if (-not (Test-ManagedAlive "zenoh")) {
            Write-ComponentFailure -Stage "zenoh_start" -Reason "zenoh_process_exited" -Name "zenoh"
            throw "zenoh_process_exited"
        }
        $fatal = Test-LogHasFatal -Path $logFile
        if ($fatal) {
            Write-ComponentFailure -Stage "zenoh_start" -Reason "zenoh_log_error:$fatal" -Name "zenoh"
            throw "zenoh_log_error:$fatal"
        }
        $text = Read-TextFileLive -Path $logFile
        foreach ($candidate in $ZenohReadyMarkers) {
            if ($text -match [regex]::Escape($candidate)) {
                $marker = $candidate
                break
            }
        }
        if ($marker) {
            break
        }
        Start-Sleep -Milliseconds 200
    }
    if (-not $marker) {
        Write-ComponentFailure -Stage "zenoh_start" -Reason "zenoh_ready_marker_missing" -Name "zenoh"
        throw "zenoh_ready_marker_missing"
    }
    Write-LauncherLog "zenoh_ready_marker=$marker"
    Start-Sleep -Milliseconds ([int]($ZenohStabilityWindowS * 1000))
    Update-ComponentProcessTree -Name "zenoh"
    if (-not (Test-ManagedAlive "zenoh")) {
        Write-ComponentFailure -Stage "zenoh_start" -Reason "zenoh_process_exited_after_ready_marker" -Name "zenoh"
        throw "zenoh_process_exited_after_ready_marker"
    }
}

function Invoke-Ros2TopicProbe {
    param([string]$Topic, [string]$ExpectedPattern, [double]$TimeoutSec)
    $probeName = "probe_" + ($Topic -replace "[^A-Za-z0-9]", "_") + "_" + ([System.Guid]::NewGuid().ToString("N").Substring(0, 8))
    $probeLog = Join-Path $LogDir "$probeName.log"
    $command = "& `"$Ros2Wrapper`" -Command `"cd /d $Ros2Ws && call install\local_setup.bat && ros2 topic echo --once $Topic`""
    $scriptPath = New-ProcessScript -Name $probeName -Command $command -LogFile $probeLog
    $proc = Start-Process -FilePath "powershell.exe" -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $scriptPath) -WorkingDirectory $ProjectRoot -PassThru -WindowStyle Hidden
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
    $text = Read-TextFileLive -Path $probeLog
    if ($text -notmatch $ExpectedPattern) {
        throw "ros2_topic_probe_failed topic=$Topic expected=$ExpectedPattern log=$probeLog"
    }
}

function Wait-Ros2TopicPattern {
    param([string]$Topic, [string]$ExpectedPattern, [double]$TimeoutSec, [string]$FailedStage)
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        foreach ($name in $script:Managed.Keys) {
            if (-not (Test-ManagedAlive $name)) {
                Write-ComponentFailure -Stage $FailedStage -Reason "${name}_process_exited" -Name $name
                throw "$FailedStage ${name}_process_exited"
            }
        }
        try {
            Invoke-Ros2TopicProbe -Topic $Topic -ExpectedPattern $ExpectedPattern -TimeoutSec 2.0
            return
        }
        catch {
            Write-LauncherLog "topic_probe_retry topic=$Topic reason=$($_.Exception.Message)"
        }
        Start-Sleep -Milliseconds 200
    }
    Write-Step "FAILED_STAGE $FailedStage"
    Write-Step "FAILED_REASON timeout topic=$Topic expected=$ExpectedPattern"
    Write-Step "LOG_DIR $LogDir"
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
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return
    }
    Update-ComponentProcessTree -Name $Name
    $entry = $script:Managed[$Name]
    $pids = @($entry.DescendantPids + @($entry.RootPid)) | Sort-Object -Descending -Unique
    foreach ($pidValue in $pids) {
        $proc = Get-Process -Id $pidValue -ErrorAction SilentlyContinue
        if ($proc) {
            Write-LauncherLog "COMPONENT_PROCESS_STOPPING name=$Name pid=$pidValue"
            Stop-Process -Id $pidValue -Force -ErrorAction SilentlyContinue
            Start-Sleep -Milliseconds 100
            $again = Get-Process -Id $pidValue -ErrorAction SilentlyContinue
            $exitCode = if ($again) { "still_running" } else { "stopped" }
            Write-LauncherLog "COMPONENT_PROCESS_STOPPED name=$Name pid=$pidValue exit_code=$exitCode"
        }
    }
}

function Cleanup {
    Write-LauncherLog "cleanup_start order=action,visual_nodes,ros2_bridge,lerobot_server,zenoh"
    foreach ($name in @("vision", "bridge", "server", "zenoh")) {
        if ($script:Managed.Contains($name)) {
            try {
                if (Test-ManagedAlive $name) {
                    Stop-OwnedProcessTree -Name $name
                }
                else {
                    $exit = Get-ManagedExitCode -Name $name
                    Write-LauncherLog "already_exited name=$name pid=$($script:Managed[$name].RootPid) exit_code=$exit"
                }
            }
            catch {
                Write-LauncherLog "cleanup_warning name=$name message=$($_.Exception.Message)"
            }
        }
    }
    if (Test-Path -LiteralPath $ActiveManifest) {
        try {
            $raw = Read-TextFileLive -Path $ActiveManifest
            if ($raw) {
                $manifest = $raw | ConvertFrom-Json
                if ([int]$manifest.launcher_pid -eq $PID) {
                    Remove-Item -LiteralPath $ActiveManifest -Force -ErrorAction SilentlyContinue
                }
            }
        }
        catch {
            Write-LauncherLog "manifest_cleanup_warning message=$($_.Exception.Message)"
        }
    }
    Write-LauncherLog "cleanup_complete"
}

function Start-Zenoh {
    Write-Step "[1/5] Starting Zenoh..."
    Start-ManagedCommand -Name "zenoh" -LogName "zenoh.log" -Command "& `"$Ros2Wrapper`" -Command `"ros2 run rmw_zenoh_cpp rmw_zenohd`"" | Out-Null
    Wait-ZenohReady
    Write-Step "[1/5] Zenoh ready"
}

function Start-Server {
    Write-Step "[2/5] Starting LeRobot server..."
    $cmd = "& `"$CondaExe`" run --no-capture-output -p `"$LeRobotEnv`" python -u scripts\mvp_so101_server.py --config config\mvp_hardware.json --enable-hardware-motion"
    Start-ManagedCommand -Name "server" -LogName "server.log" -Command $cmd | Out-Null
    Wait-ComponentLogPattern -Name "server" -Pattern "SERVER_PROCESS_STARTED" -TimeoutSec $ServerProcessStartTimeoutS -FailedStage "server_process_start"
    Write-Step "[2/5] Waiting for FlexiTac COM8..."
    Wait-ComponentLogPattern -Name "server" -Pattern "TACTILE_SERIAL_OPENED port=COM8" -TimeoutSec $TactileSerialOpenTimeoutS -FailedStage "tactile_open"
    Write-Step "[2/5] COM8 connected"
    Write-Step "[2/5] Waiting for tactile baseline..."
    Wait-ComponentLogPattern -Name "server" -Pattern "TACTILE_BASELINE_COMPLETED" -TimeoutSec $TactileBaselineTimeoutS -FailedStage "tactile_baseline"
    Wait-ComponentLogPattern -Name "server" -Pattern "TACTILE_READY true" -TimeoutSec 5.0 -FailedStage "tactile_ready"
    Write-Step "[2/5] Tactile ready"
    Write-Step "[2/5] Waiting for robot COM4..."
    Wait-ComponentLogPattern -Name "server" -Pattern "ROBOT_CONNECTED port=COM4" -TimeoutSec $RobotConnectTimeoutS -FailedStage "robot_connect"
    Write-Step "[2/5] COM4 connected"
    Write-Step "[2/5] Waiting for TCP listener..."
    Wait-ComponentLogPattern -Name "server" -Pattern "TCP_SERVER_LISTENING" -TimeoutSec $TcpListenTimeoutS -FailedStage "tcp_listen"
    Write-Step "[2/5] TCP server listening"
}

function Start-Bridge {
    Write-Step "[3/5] Starting ROS2 bridge..."
    $bridgeCommand = "cd /d $Ros2Ws && call install\local_setup.bat && ros2 launch so101_mvp_bringup mvp_hardware_bridge_motion_enabled.launch.py enable_hardware_motion:=true"
    Start-ManagedCommand -Name "bridge" -LogName "bridge.log" -Command "& `"$Ros2Wrapper`" -Command `"$bridgeCommand`"" | Out-Null
    Wait-ComponentLogPattern -Name "bridge" -Pattern "BRIDGE_TCP_CONNECTED" -TimeoutSec 15.0 -FailedStage "bridge_tcp"
    Wait-ComponentLogPattern -Name "bridge" -Pattern "BRIDGE_TCP_READY true" -TimeoutSec 5.0 -FailedStage "bridge_ready"
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
    Assert-PathExists -Path $HardwareConfig -Label "Hardware config"
    Assert-NoOldLauncher
    Save-Manifest
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
    if ($_.Exception.Message -notmatch "process_exited|ready_marker|timeout|log_error") {
        Write-Step "FAILED_STAGE launcher"
        Write-Step ("FAILED_REASON {0}" -f $_.Exception.Message)
        Write-Step ("LOG_DIR {0}" -f $LogDir)
    }
    throw
}
finally {
    Cleanup
    Write-Host "Logs: $LogDir"
}
