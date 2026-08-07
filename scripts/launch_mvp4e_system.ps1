# =============================================================================
# DEPRECATED_FOR_FINAL_ACCEPTANCE
#
# This one-launch orchestrator introduced unnecessary Windows/PowerShell/ROS2
# process-management complexity. Final MVP-4E acceptance uses the previously
# validated manual multi-terminal startup via scripts/open_mvp4e_terminals.ps1.
#
# This file is retained only for historical debugging.
# =============================================================================

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
$BridgeStdoutLog = Join-Path $LogDir "bridge.stdout.log"
$BridgeStderrLog = Join-Path $LogDir "bridge.stderr.log"
$BridgeCombinedLog = Join-Path $LogDir "bridge.log"
$BridgeCommandFile = Join-Path $LogDir "bridge_command.cmd"
$BridgeRunnerScript = Join-Path $ProjectRoot "scripts\run_mvp4e_bridge.ps1"
$RosLogDir = Join-Path $LogDir "ros2"

$ZenohStartTimeoutS = 10.0
$ZenohStabilityWindowS = 1.0
$ServerProcessStartTimeoutS = 10.0
$TactileSerialOpenTimeoutS = 15.0
$TactileBaselineTimeoutS = 30.0
$RobotConnectTimeoutS = 30.0
$TcpListenTimeoutS = 15.0

$ZenohReadyMarkers = @("zenohd", "Started", "router", "listening", "scouting")
$ProjectFatalMarkers = @(
    "BRIDGE_TCP_CONNECT_FAILED",
    "BRIDGE_TCP_FATAL",
    "BRIDGE_PROCESS_FAILED",
    "TCP_PROTOCOL_FATAL",
    "CONFIG_LOAD_FAILED",
    "NODE_START_FAILED",
    "BRIDGE_RMW_MISMATCH"
)

$script:Managed = [ordered]@{}

New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
New-Item -ItemType Directory -Force -Path $RosLogDir | Out-Null
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
    $stream = $null
    $reader = $null
    try {
        $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
        $reader = New-Object System.IO.StreamReader($stream, [System.Text.Encoding]::UTF8, $true)
        return $reader.ReadToEnd()
    }
    catch {
        Write-LauncherLog "log_read_retry path=$Path message=$($_.Exception.Message)"
        return ""
    }
    finally {
        if ($reader) {
            $reader.Dispose()
        }
        elseif ($stream) {
            $stream.Dispose()
        }
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

function Merge-BridgeLogs {
    param([string]$StdoutPath, [string]$StderrPath, [string]$CombinedPath)
    $stdout = Read-TextFileLive -Path $StdoutPath
    $stderr = Read-TextFileLive -Path $StderrPath
    $merged = @()
    if ($stdout) {
        $merged += $stdout.TrimEnd()
    }
    if ($stderr) {
        $merged += $stderr.TrimEnd()
    }
    $text = ($merged -join "`r`n")
    Set-Content -LiteralPath $CombinedPath -Value $text -Encoding UTF8
}

function Get-ComponentLogText {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return ""
    }
    $entry = $script:Managed[$Name]
    if ($entry.PSObject.Properties.Name -contains "StdoutLog" -and $entry.StdoutLog -and $entry.StderrLog -and $entry.CombinedLog) {
        Merge-BridgeLogs -StdoutPath $entry.StdoutLog -StderrPath $entry.StderrLog -CombinedPath $entry.CombinedLog
        return (Read-TextFileLive -Path $entry.CombinedLog)
    }
    if ($entry.Log) {
        return Read-TextFileLive -Path $entry.Log
    }
    return ""
}

function Get-BridgeReportedExitCode {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return $null
    }
    $text = Get-ComponentLogText -Name $Name
    if ($text -match 'BRIDGE_RUNNER_WRAPPER_EXIT code=(\d+)') {
        return [int]$matches[1]
    }
    return $null
}

function Get-CommandFileContent {
    param([string]$Path)
    if (-not $Path -or -not (Test-Path -LiteralPath $Path)) {
        return @()
    }
    try {
        return @(Get-Content -LiteralPath $Path)
    }
    catch {
        return @()
    }
}

function Find-CommandFileSyntaxErrorLine {
    param([string]$Path)
    $lines = Get-CommandFileContent -Path $Path
    foreach ($line in $lines) {
        if ($line -match '(?i)^\s*if\s*\(\s*$') {
            return $line
        }
        if ($line -match '(?i)^\s*.*(?:&&|\|\|)\s*$') {
            return $line
        }
        if ($line -match '(?i)^\s*call\s+"[^"]*"\s*$') {
            continue
        }
    }
    if ($lines.Count -gt 0) {
        return $lines[0]
    }
    return ""
}

function Find-FatalLogEntryFromText {
    param([string]$Text)
    foreach ($line in ($Text -split "`r?`n")) {
        $severity = Get-LogSeverity -Line $line
        if ($severity.Severity -eq "FATAL") {
            return $severity
        }
    }
    return $null
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
    $currentDescendantPids = @($desc | ForEach-Object { [int]$_.ProcessId } | Sort-Object -Unique)
    $lastKnownDescendantPids = @()
    if ($entry.PSObject.Properties.Name -contains "LastKnownDescendantPids" -and $entry.LastKnownDescendantPids) {
        $lastKnownDescendantPids = @($entry.LastKnownDescendantPids)
    }
    $addedDescendants = @($currentDescendantPids | Where-Object { $_ -notin $lastKnownDescendantPids })
    $removedDescendants = @($lastKnownDescendantPids | Where-Object { $_ -notin $currentDescendantPids })
    $entry.DescendantPids = $currentDescendantPids
    $entry.LastKnownDescendantPids = $currentDescendantPids
    $entry.AllSeenDescendantPids = @(@($entry.AllSeenDescendantPids) + $currentDescendantPids) | Sort-Object -Unique
    $entry.CommandLines = @(
        "root=$($entry.CommandLine)"
        $desc | ForEach-Object { "pid=$($_.ProcessId) $($_.CommandLine)" }
    )
    if ($entry.PSObject.Properties.Name -contains "StdoutLog" -and $entry.StdoutLog -and $entry.StderrLog -and $entry.CombinedLog) {
        try {
            Merge-BridgeLogs -StdoutPath $entry.StdoutLog -StderrPath $entry.StderrLog -CombinedPath $entry.CombinedLog
        }
        catch {
            Write-LauncherLog "bridge_log_merge_warning message=$($_.Exception.Message)"
        }
    }
    if (-not ($entry.PSObject.Properties.Name -contains "ProcessTreeStartedLogged") -or -not $entry.ProcessTreeStartedLogged) {
        $entry.ProcessTreeStartedLogged = $true
        Write-LauncherLog "COMPONENT_PROCESS_STARTED name=$Name root_pid=$($entry.RootPid) descendants=[$(($entry.DescendantPids) -join ',')]"
    }
    elseif (($addedDescendants.Count -gt 0) -or ($removedDescendants.Count -gt 0)) {
        Write-LauncherLog "COMPONENT_PROCESS_TREE_UPDATED name=$Name added=[$(($addedDescendants) -join ',')] removed=[$(($removedDescendants) -join ',')] current=[$(($entry.DescendantPids) -join ',')]"
    }
}

function Save-Manifest {
    $components = @{}
    foreach ($name in $script:Managed.Keys) {
        $entry = $script:Managed[$name]
        Update-ComponentProcessTree -Name $name
        $components[$name] = @{
            root_pid = $entry.RootPid
            descendant_pids = $entry.DescendantPids
            last_known_descendant_pids = if ($entry.PSObject.Properties.Name -contains "LastKnownDescendantPids") { $entry.LastKnownDescendantPids } else { $null }
            all_seen_descendant_pids = if ($entry.PSObject.Properties.Name -contains "AllSeenDescendantPids") { $entry.AllSeenDescendantPids } else { $null }
            log = $entry.Log
            stdout_log = if ($entry.PSObject.Properties.Name -contains "StdoutLog") { $entry.StdoutLog } else { $null }
            stderr_log = if ($entry.PSObject.Properties.Name -contains "StderrLog") { $entry.StderrLog } else { $null }
            combined_log = if ($entry.PSObject.Properties.Name -contains "CombinedLog") { $entry.CombinedLog } else { $null }
            command_file = if ($entry.PSObject.Properties.Name -contains "CommandFile") { $entry.CommandFile } else { $null }
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
`$env:RMW_IMPLEMENTATION = "rmw_zenoh_cpp"
`$env:ROS_LOG_DIR = "$RosLogDir"
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
        LastKnownDescendantPids = @()
        AllSeenDescendantPids = @()
        CommandLines = @()
        ProcessTreeStartedLogged = $false
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

function Start-ManagedBridgeRunner {
    Assert-PathExists -Path $BridgeRunnerScript -Label "Bridge runner script"
    $arguments = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $BridgeRunnerScript, "-EnableHardwareMotion", "-LogDirectory", $LogDir, "-CommandFilePath", $BridgeCommandFile, "-Ros2WrapperPath", $Ros2Wrapper, "-Ros2WorkspacePath", $Ros2Ws)
    Write-LauncherLog "BRIDGE_PROCESS_COMMAND executable=powershell.exe argument_list=$($arguments -join ' ')"
    Write-LauncherLog "BRIDGE_PROCESS_WORKING_DIRECTORY $ProjectRoot"
    Write-LauncherLog "BRIDGE_STDOUT_LOG $BridgeStdoutLog"
    Write-LauncherLog "BRIDGE_STDERR_LOG $BridgeStderrLog"
    Write-LauncherLog "BRIDGE_COMBINED_LOG $BridgeCombinedLog"
    Write-LauncherLog "BRIDGE_COMMAND_FILE $BridgeCommandFile"
    Write-LauncherLog "BRIDGE_RMW_IMPLEMENTATION rmw_zenoh_cpp"
    $process = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList $arguments `
        -WorkingDirectory $ProjectRoot `
        -RedirectStandardOutput $BridgeStdoutLog `
        -RedirectStandardError $BridgeStderrLog `
        -PassThru `
        -WindowStyle Hidden
    $script:Managed["bridge"] = [pscustomobject]@{
        Name = "bridge"
        Process = $process
        RootPid = [int]$process.Id
        DescendantPids = @()
        LastKnownDescendantPids = @()
        AllSeenDescendantPids = @()
        CommandLines = @()
        ProcessTreeStartedLogged = $false
        Log = $BridgeCombinedLog
        StdoutLog = $BridgeStdoutLog
        StderrLog = $BridgeStderrLog
        CombinedLog = $BridgeCombinedLog
        CommandFile = $BridgeCommandFile
        Script = $BridgeRunnerScript
        CommandLine = Get-ProcessCommandLine -ProcessId ([int]$process.Id)
        Command = "powershell.exe $($arguments -join ' ')"
    }
    Update-ComponentProcessTree -Name "bridge"
    Save-Manifest
    Write-LauncherLog "started name=bridge pid=$($process.Id) stdout=$BridgeStdoutLog stderr=$BridgeStderrLog"
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

function Test-ManagedProcessTreeAlive {
    param([string]$Name, [bool]$RequireDescendant = $false)
    if (-not (Test-ManagedAlive $Name)) {
        return $false
    }
    if (-not $RequireDescendant) {
        return $true
    }
    Update-ComponentProcessTree -Name $Name
    $entry = $script:Managed[$Name]
    if (@($entry.DescendantPids).Count -eq 0) {
        return $true
    }
    foreach ($pidValue in @($entry.DescendantPids)) {
        if (Get-Process -Id $pidValue -ErrorAction SilentlyContinue) {
            return $true
        }
    }
    return $false
}

function Get-ManagedExitCode {
    param([string]$Name)
    if (-not $script:Managed.Contains($Name)) {
        return $null
    }
    $proc = $script:Managed[$Name].Process
    $deadline = (Get-Date).AddMilliseconds(1000)
    try {
        while ((Get-Date) -lt $deadline) {
            $proc.Refresh()
            if ($proc.HasExited) {
                return $proc.ExitCode
            }
            Start-Sleep -Milliseconds 50
        }
    }
    catch {
    }
    if ($Name -eq "bridge") {
        $reported = Get-BridgeReportedExitCode -Name $Name
        if ($null -ne $reported) {
            return $reported
        }
    }
    return $null
}

function Get-LogSeverity {
    param([string]$Line)
    if ($Line -match '(?i)^\s*(?:\d{4}-\d{2}-\d{2}[^\[]*\s+)?(?:\[[^\]]+\]\s*)?\[(ERROR|FATAL)\]') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "ros_error_level"; Line = $Line }
    }
    if ($Line -match '(?i)^\s*Traceback \(most recent call last\):') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "python_traceback"; Line = $Line }
    }
    if ($Line -match '(?i)^\s*(ModuleNotFoundError|ImportError|SyntaxError):') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "python_import_or_syntax_error"; Line = $Line }
    }
    if ($Line -match '(?i)^\s*Unhandled exception\b') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "python_unhandled_exception"; Line = $Line }
    }
    foreach ($marker in $ProjectFatalMarkers) {
        if ($Line -match ("^\s*" + [regex]::Escape($marker) + "\b")) {
            return [pscustomobject]@{ Severity = "FATAL"; Pattern = "project_fatal_marker"; Line = $Line }
        }
    }
    if ($Line -match '(?i)\b(required process has died|process has died|process exited with code)\b') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "process_died"; Line = $Line }
    }
    if ($Line -match '(?i)\b(Address already in use|bind failed|panic)\b') {
        return [pscustomobject]@{ Severity = "FATAL"; Pattern = "runtime_fatal_text"; Line = $Line }
    }
    if ($Line -match '(?i)(UserWarning:|\[warning\]|FutureWarning|DeprecationWarning|ResourceWarning|RuntimeWarning|WinError 1314|Cannot create a symlink to latest log directory|RTI Connext DDS will not be available at runtime)') {
        return [pscustomobject]@{ Severity = "WARNING"; Pattern = "known_nonfatal_warning"; Line = $Line }
    }
    if ($Line -match '(?i)^\s*(?:\[[^\]]+\]\s*)?\[INFO\]') {
        return [pscustomobject]@{ Severity = "INFO"; Pattern = "ros_info_level"; Line = $Line }
    }
    return [pscustomobject]@{ Severity = "INFO"; Pattern = "default_info"; Line = $Line }
}

function Find-FatalLogEntry {
    param([string]$Path)
    $text = Read-TextFileLive -Path $Path
    foreach ($line in ($text -split "`r?`n")) {
        $severity = Get-LogSeverity -Line $line
        if ($severity.Severity -eq "FATAL") {
            return $severity
        }
    }
    return $null
}

function Test-LogHasFatal {
    param([string]$Path)
    return Find-FatalLogEntry -Path $Path
}

function Get-BridgeFailureClassification {
    param([string]$Text, [bool]$TimedOut = $false)
    if ($Text -notmatch "BRIDGE_RUNNER_STARTED") {
        return [pscustomobject]@{ Stage = "bridge_spawn"; Reason = "bridge_runner_failed_before_start" }
    }
    if ($Text -match '(?i)(The syntax of the command is incorrect\.|The filename, directory name, or volume label syntax is incorrect\.|was unexpected at this time\.|is not recognized as an internal or external command\.)') {
        return [pscustomobject]@{ Stage = "bridge_command_file"; Reason = "cmd_syntax_error" }
    }
    if ($Text -match "(?i)(ParameterBindingException|A positional parameter cannot be found|Cannot bind parameter|BRIDGE_RUNNER_EXCEPTION|The term '.+' is not recognized)") {
        return [pscustomobject]@{ Stage = "bridge_wrapper"; Reason = "bridge_wrapper_exited" }
    }
    if ($Text -match "(?i)(InvalidLaunchFileError|launch\.invalid_launch_file_error|PermissionError|Launch file may have a syntax error)") {
        return [pscustomobject]@{ Stage = "bridge_launch"; Reason = "ros2_launch_failed" }
    }
    if (($Text -match "mvp_hardware_bridge_node.*process started with pid") -and ($Text -match "(?i)(process has died|process exited with code|BRIDGE_RUNNER_WRAPPER_EXIT code=[1-9])")) {
        return [pscustomobject]@{ Stage = "bridge_node"; Reason = "bridge_process_exited" }
    }
    if (($Text -match "BRIDGE_RUNNER_WRAPPER_EXIT code=[1-9]") -or ($Text -match "Traceback \(most recent call last\):")) {
        return [pscustomobject]@{ Stage = "bridge_wrapper"; Reason = "bridge_wrapper_exited" }
    }
    if ($TimedOut) {
        return [pscustomobject]@{ Stage = "bridge_tcp"; Reason = "bridge_tcp_timeout" }
    }
    return [pscustomobject]@{ Stage = "bridge_wrapper"; Reason = "bridge_wrapper_exited" }
}

function Wait-BridgeRunnerStarted {
    $deadline = (Get-Date).AddSeconds(5.0)
    while ((Get-Date) -lt $deadline) {
        Update-ComponentProcessTree -Name "bridge"
        $text = Get-ComponentLogText -Name "bridge"
        if ($text -match "BRIDGE_RUNNER_STARTED") {
            return
        }
        if (-not (Test-ManagedAlive "bridge")) {
            $classification = Get-BridgeFailureClassification -Text $text
            Write-ComponentFailure -Stage $classification.Stage -Reason $classification.Reason -Name "bridge"
            throw "$($classification.Stage) $($classification.Reason)"
        }
        Start-Sleep -Milliseconds 200
    }
    $text = Get-ComponentLogText -Name "bridge"
    $classification = Get-BridgeFailureClassification -Text $text
    Write-ComponentFailure -Stage $classification.Stage -Reason $classification.Reason -Name "bridge"
    throw "$($classification.Stage) $($classification.Reason)"
}

function Write-ComponentFailure {
    param([string]$Stage, [string]$Reason, [string]$Name, [object]$FatalEntry = $null)
    $entry = if ($script:Managed.Contains($Name)) { $script:Managed[$Name] } else { $null }
    $rootPid = if ($entry) { $entry.RootPid } else { $null }
    $desc = if ($entry) { ($entry.DescendantPids -join ",") } else { "" }
    $exitCode = if ($entry) { Get-ManagedExitCode -Name $Name } else { $null }
    $runnerExitCode = if ($Name -eq "bridge") { Get-BridgeReportedExitCode -Name $Name } else { $null }
    if ($null -eq $exitCode -and $null -ne $runnerExitCode) {
        $exitCode = $runnerExitCode
    }
    elseif (($null -ne $exitCode) -and ($null -ne $runnerExitCode) -and ($exitCode -ne $runnerExitCode) -and $Name -eq "bridge") {
        Write-Step "BRIDGE_EXIT_CODE_MISMATCH process_exit_code=$exitCode runner_reported_code=$runnerExitCode"
    }
    elseif (($null -eq $exitCode) -and ($null -eq $runnerExitCode) -and $Name -eq "bridge") {
        Write-Step "BRIDGE_EXIT_CODE_RECOVERY_FAILED process_exit_code=null runner_reported_code=null"
    }
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
    if ($Name -eq "bridge") {
        if ($null -ne $runnerExitCode) {
            Write-Step "BRIDGE_RUNNER_REPORTED_EXIT_CODE $runnerExitCode"
        }
        $commandFile = if ($entry -and ($entry.PSObject.Properties.Name -contains "CommandFile")) { $entry.CommandFile } else { "" }
        if ($commandFile) {
            Write-Step "COMMAND_FILE $commandFile"
            Write-Step "COMMAND_FILE_CONTENT_BEGIN"
            foreach ($line in Get-CommandFileContent -Path $commandFile) {
                Write-LauncherLog $line
                Write-Host $line
            }
            Write-Step "COMMAND_FILE_CONTENT_END"
            if ($Stage -eq "bridge_command_file" -or $Reason -eq "cmd_syntax_error") {
                $cmdErrorLine = Find-CommandFileSyntaxErrorLine -Path $commandFile
                if ($cmdErrorLine) {
                    Write-Step "CMD_ERROR_LINE $cmdErrorLine"
                }
            }
        }
    }
    if ($entry -and ($entry.PSObject.Properties.Name -contains "StdoutLog")) {
        $stdoutLog = $entry.StdoutLog
        $stderrLog = $entry.StderrLog
        $combinedLog = $entry.CombinedLog
        $stdoutEmpty = -not (Test-Path -LiteralPath $stdoutLog) -or ((Get-Item -LiteralPath $stdoutLog).Length -eq 0)
        $stderrEmpty = -not (Test-Path -LiteralPath $stderrLog) -or ((Get-Item -LiteralPath $stderrLog).Length -eq 0)
        Write-Step "BRIDGE_STDOUT_LOG $stdoutLog"
        Write-Step "BRIDGE_STDERR_LOG $stderrLog"
        Write-Step "BRIDGE_COMBINED_LOG $combinedLog"
        Write-Step ("BRIDGE_STDOUT_EMPTY {0}" -f $stdoutEmpty.ToString().ToLowerInvariant())
        Write-Step ("BRIDGE_STDERR_EMPTY {0}" -f $stderrEmpty.ToString().ToLowerInvariant())
        Write-Host "BRIDGE_STDOUT_TAIL_BEGIN"
        foreach ($line in Get-LogTail -Path $stdoutLog -Lines 100) {
            Write-Host $line
        }
        Write-Host "BRIDGE_STDOUT_TAIL_END"
        Write-Host "BRIDGE_STDERR_TAIL_BEGIN"
        foreach ($line in Get-LogTail -Path $stderrLog -Lines 100) {
            Write-Host $line
        }
        Write-Host "BRIDGE_STDERR_TAIL_END"
    }
    if ($FatalEntry) {
        Write-Step "FATAL_PATTERN $($FatalEntry.Pattern)"
        Write-Step "FATAL_LINE $($FatalEntry.Line)"
    }
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
        if (-not (Test-ManagedProcessTreeAlive -Name $Name -RequireDescendant ($Name -eq "bridge"))) {
            if ($Name -eq "bridge") {
                $text = Get-ComponentLogText -Name $Name
                $classification = Get-BridgeFailureClassification -Text $text
                Write-ComponentFailure -Stage $classification.Stage -Reason $classification.Reason -Name $Name
                throw "$($classification.Stage) $($classification.Reason)"
            }
            Write-ComponentFailure -Stage $FailedStage -Reason "${Name}_process_exited" -Name $Name
            throw "$FailedStage ${Name}_process_exited"
        }
        $text = Get-ComponentLogText -Name $Name
        $fatal = Find-FatalLogEntryFromText -Text $text
        if ($fatal) {
            Write-ComponentFailure -Stage $FailedStage -Reason "${Name}_fatal_log" -Name $Name -FatalEntry $fatal
            throw "$FailedStage ${Name}_fatal_log:$($fatal.Pattern)"
        }
        if ($text -match [regex]::Escape($Pattern)) {
            return
        }
        Start-Sleep -Milliseconds 200
    }
    if ($Name -eq "bridge") {
        $text = Get-ComponentLogText -Name $Name
        $classification = Get-BridgeFailureClassification -Text $text -TimedOut $true
        Write-ComponentFailure -Stage $classification.Stage -Reason $classification.Reason -Name $Name
        throw "$($classification.Stage) $($classification.Reason)"
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
        $fatal = Find-FatalLogEntry -Path $logFile
        if ($fatal) {
            Write-ComponentFailure -Stage "zenoh_start" -Reason "zenoh_fatal_log" -Name "zenoh" -FatalEntry $fatal
            throw "zenoh_fatal_log:$($fatal.Pattern)"
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
            if (-not (Test-ManagedProcessTreeAlive -Name $name -RequireDescendant ($name -eq "bridge"))) {
                if ($name -eq "bridge") {
                    $text = Get-ComponentLogText -Name $name
                    $classification = Get-BridgeFailureClassification -Text $text
                    Write-ComponentFailure -Stage $classification.Stage -Reason $classification.Reason -Name $name
                    throw "$($classification.Stage) $($classification.Reason)"
                }
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
    $pids = @($entry.DescendantPids + @($entry.AllSeenDescendantPids) + @($entry.RootPid)) | Sort-Object -Descending -Unique
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
    try {
        Start-ManagedBridgeRunner | Out-Null
    }
    catch {
        Write-Step "FAILED_STAGE bridge_spawn"
        Write-Step "FAILED_REASON bridge_runner_spawn_failed"
        Write-Step "BRIDGE_STDOUT_LOG $BridgeStdoutLog"
        Write-Step "BRIDGE_STDERR_LOG $BridgeStderrLog"
        Write-Step "LOG_DIR $LogDir"
        throw "bridge_spawn bridge_runner_spawn_failed"
    }
    Wait-BridgeRunnerStarted
    Write-Step "[3/5] Bridge process started"
    Write-Step "[3/5] Waiting for TCP connection..."
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
