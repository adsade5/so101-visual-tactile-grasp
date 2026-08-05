param(
    [switch]$SkipOfficialDemo,
    [switch]$SkipBuild,
    [string]$PixiWorkspace = "C:\pixi_ws"
)

$ErrorActionPreference = "Continue"
$projectRoot = Split-Path -Parent $PSScriptRoot
$resultRoot = Join-Path $projectRoot "audit_results\stage_minus1b"
$null = New-Item -ItemType Directory -Force -Path $resultRoot
$runner = Join-Path $PSScriptRoot "run_in_ros2_lyrical.ps1"
$summaryPath = Join-Path $resultRoot "verification_summary.txt"
$results = New-Object System.Collections.Generic.List[object]

function Add-Result {
    param([string]$Name, [string]$Status, [string]$Evidence, [bool]$Blocking = $false)
    $item = [pscustomobject]@{ Name = $Name; Status = $Status; Evidence = $Evidence; Blocking = $Blocking }
    $script:results.Add($item) | Out-Null
    $line = "{0}: {1} | {2}" -f $Name, $Status, $Evidence
    Write-Host $line
    Add-Content -LiteralPath $summaryPath -Value $line
}

function Run-Ros2Command {
    param([string]$Name, [string]$Command, [int]$TimeoutSeconds = 60)
    $safeName = $Name -replace '[^A-Za-z0-9_.-]', '_'
    $log = Join-Path $resultRoot "$safeName.log"
    $err = Join-Path $resultRoot "$safeName.err.log"
    $commandFile = Join-Path $resultRoot "$safeName.command.cmd"
    Set-Content -LiteralPath $commandFile -Value $Command -Encoding ASCII
    $inner = '"powershell.exe" -NoProfile -ExecutionPolicy Bypass -File "' + $runner + '" -PixiWorkspace "' + $PixiWorkspace + '" -CommandFile "' + $commandFile + '" 1> "' + $log + '" 2> "' + $err + '"'
    $psi = [System.Diagnostics.ProcessStartInfo]::new()
    $psi.FileName = "cmd.exe"
    $psi.Arguments = '/d /s /c "' + $inner + '"'
    $psi.UseShellExecute = $false
    $p = [System.Diagnostics.Process]::new()
    $p.StartInfo = $psi
    $null = $p.Start()
    if (-not $p.WaitForExit($TimeoutSeconds * 1000)) {
        try { $p.Kill() } catch {}
        Add-Content -LiteralPath $err -Value "TIMEOUT after ${TimeoutSeconds}s"
        Add-Result $Name "失败" "timeout after ${TimeoutSeconds}s; log=$log" $true
        return 124
    }
    $p.WaitForExit()
    $exitCode = $p.ExitCode
    if ($exitCode -eq 0) {
        Add-Result $Name "通过" "exit=0; log=$log" $false
    } else {
        Add-Result $Name "失败" "exit=$exitCode; log=$log; err=$err" $true
    }
    return $exitCode
}

Set-Content -LiteralPath $summaryPath -Value "Stage -1B verification started: $(Get-Date -Format s)"

$pixiExe = Join-Path $env:USERPROFILE ".pixi\bin\pixi.exe"
if (Test-Path -LiteralPath $pixiExe) {
    $pixiVersion = & $pixiExe --version
    Add-Result "Pixi" "通过" "$pixiVersion at $pixiExe" $false
} else {
    Add-Result "Pixi" "失败" "pixi.exe not found at $pixiExe" $true
}

$setupBat = Join-Path $PixiWorkspace "ros2-windows\local_setup.bat"
if (Test-Path -LiteralPath $setupBat) {
    Add-Result "ROS2 setup" "通过" $setupBat $false
} else {
    Add-Result "ROS2 setup" "失败" "missing $setupBat" $true
}

Run-Ros2Command "base_environment" "echo ROS_DISTRO=%ROS_DISTRO% && echo RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION% && where ros2 && ros2 --help > NUL && ros2 node list && where python && python --version && python -c ""import sys; print(sys.executable); print(sys.version)"" && python -c ""import rclpy; print(rclpy.__file__)"" && python -c ""from std_msgs.msg import String; print(String)"" && where colcon && colcon --help > NUL && where cmake && cmake --version && where ninja && ninja --version" 90 | Out-Null
Run-Ros2Command "ros2_doctor_report" "ros2 doctor --report" 120 | Out-Null

if (-not $SkipOfficialDemo) {
    $runtimeChecks = Join-Path $PSScriptRoot "ros2_stage_minus1b_runtime_checks.py"
    Run-Ros2Command "official_demo_run" "python ""$runtimeChecks"" --result-root ""$resultRoot"" --official-demo" 45 | Out-Null
    $talkerLog = Join-Path $resultRoot "official_talker.log"
    $listenerLog = Join-Path $resultRoot "official_listener.log"
    $talkerCount = if (Test-Path -LiteralPath $talkerLog) { (Select-String -LiteralPath $talkerLog -Pattern "Publishing|Hello World").Count } else { 0 }
    $listenerCount = if (Test-Path -LiteralPath $listenerLog) { (Select-String -LiteralPath $listenerLog -Pattern "I heard|Hello World").Count } else { 0 }
    if ($talkerCount -ge 3 -and $listenerCount -ge 3) {
        Add-Result "official_demo_messages" "通过" "talker=$talkerCount listener=$listenerCount" $false
    } else {
        Add-Result "official_demo_messages" "失败" "talker=$talkerCount listener=$listenerCount" $true
    }
} else {
    Add-Result "official_demo" "未执行" "SkipOfficialDemo was set" $true
}

if (-not $SkipBuild) {
    $runtimeChecks = Join-Path $PSScriptRoot "ros2_stage_minus1b_runtime_checks.py"
    $minimalWs = Join-Path $projectRoot "audit_results\minimal_ros2_ws"
    foreach ($name in @("build", "install", "log")) {
        $target = Join-Path $minimalWs $name
        if (Test-Path -LiteralPath $target) {
            $resolvedMinimal = [System.IO.Path]::GetFullPath($minimalWs)
            $resolvedTarget = [System.IO.Path]::GetFullPath($target)
            if ($resolvedTarget.StartsWith($resolvedMinimal, [System.StringComparison]::OrdinalIgnoreCase)) {
                Remove-Item -LiteralPath $target -Recurse -Force
            } else {
                Add-Result "minimal cleanup" "失败" "refused to clean outside ${minimalWs}: $target" $true
            }
        }
    }
    $minimalCmd = "cd /d ""$minimalWs"" && colcon build --merge-install --event-handlers console_direct+ --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo && call install\local_setup.bat && python ""$runtimeChecks"" --result-root ""$resultRoot"" --minimal-probes"
    Run-Ros2Command "minimal_probe_build_and_run" $minimalCmd 240 | Out-Null

    $tactileWs = Join-Path $projectRoot "audit_results\tactile_bridge_ws"
    $tactileSrc = Join-Path $tactileWs "src"
    $oldPkg = Join-Path (Split-Path -Parent $projectRoot) "so101_ros2_tactile_guard\ros2\so101_flexitac_bridge"
    $null = New-Item -ItemType Directory -Force -Path $tactileSrc
    $targetPkg = Join-Path $tactileSrc "so101_flexitac_bridge"
    if (-not (Test-Path -LiteralPath $targetPkg)) {
        Copy-Item -LiteralPath $oldPkg -Destination $targetPkg -Recurse
        Add-Result "tactile package copy" "通过" "copied to $targetPkg" $false
    } else {
        Add-Result "tactile package copy" "通过" "reused existing $targetPkg" $false
    }
    $copyCmd = "cd /d ""$tactileWs"" && colcon build --merge-install --event-handlers console_direct+ && call install\local_setup.bat && python ""$runtimeChecks"" --result-root ""$resultRoot"" --tactile-check"
    Run-Ros2Command "tactile_bridge_build" $copyCmd 240 | Out-Null
} else {
    Add-Result "builds" "未执行" "SkipBuild was set" $true
}

$lerobotPython = "E:\Anaconda\envs_dirs\lerobot\python.exe"
if (Test-Path -LiteralPath $lerobotPython) {
    $log = Join-Path $resultRoot "lerobot_regression.log"
    $err = Join-Path $resultRoot "lerobot_regression.err.log"
    $args = "-c ""import sys, lerobot, cv2, serial; print(sys.executable); print(sys.version); print(lerobot.__file__); print(cv2.__version__); print(serial.VERSION)"""
    $p = Start-Process -FilePath $lerobotPython -ArgumentList $args -NoNewWindow -Wait -PassThru -RedirectStandardOutput $log -RedirectStandardError $err
    if ($p.ExitCode -eq 0) {
        Add-Result "LeRobot regression" "通过" "exit=0; log=$log" $false
    } else {
        Add-Result "LeRobot regression" "失败" "exit=$($p.ExitCode); log=$log; err=$err" $true
    }
} else {
    Add-Result "LeRobot regression" "失败" "missing $lerobotPython" $true
}

$failures = @($results | Where-Object { $_.Blocking -and $_.Status -ne "通过" })
$results | Format-Table -AutoSize | Out-String | Add-Content -LiteralPath $summaryPath
if ($failures.Count -gt 0) {
    exit 1
}
exit 0
