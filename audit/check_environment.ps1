[CmdletBinding()]
param(
    [string]$ResultsDir = "",
    [string]$PythonExecutable = "python"
)

$ErrorActionPreference = "Continue"

if ([string]::IsNullOrWhiteSpace($ResultsDir)) {
    $ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
    $ProjectRoot = Split-Path -Parent $ScriptDir
    $ResultsDir = Join-Path $ProjectRoot "audit_results"
}

New-Item -ItemType Directory -Force -Path $ResultsDir | Out-Null

function Invoke-Probe {
    param(
        [string]$Label,
        [scriptblock]$Block
    )
    Write-Output ""
    Write-Output "===== $Label ====="
    try {
        & $Block
        Write-Output "EXIT_CODE=$LASTEXITCODE"
    }
    catch {
        Write-Output "ERROR=$($_.Exception.Message)"
    }
}

function Invoke-IfAvailable {
    param(
        [string]$CommandName,
        [string[]]$Arguments = @()
    )
    Write-Output "COMMAND=$CommandName $($Arguments -join ' ')"
    $command = Get-Command $CommandName -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        Write-Output "NOT_FOUND=$CommandName"
        return
    }
    try {
        & $CommandName @Arguments
        Write-Output "EXIT_CODE=$LASTEXITCODE"
    }
    catch {
        Write-Output "ERROR=$($_.Exception.Message)"
    }
}

Write-Output "Stage -1 Windows Environment Audit"
Write-Output "Timestamp: $(Get-Date -Format o)"

Invoke-Probe "Operating system and terminal" {
    Get-ComputerInfo | Select-Object WindowsProductName, WindowsVersion, OsHardwareAbstractionLayer, OsArchitecture, CsSystemType
    Write-Output "PowerShellVersion=$($PSVersionTable.PSVersion)"
    Write-Output "UserName=$env:USERNAME"
    Write-Output "CurrentDirectory=$(Get-Location)"
}

Invoke-Probe "PATH entries relevant to ROS2/Python/Conda/VS/CMake/Git" {
    $env:Path -split ';' |
        Where-Object { $_ -match 'ros|python|conda|visual studio|vs|cmake|git|colcon' } |
        Sort-Object -Unique
}

Invoke-Probe "Git" {
    Invoke-IfAvailable "git" @("--version")
    Invoke-IfAvailable "git" @("config", "--global", "core.autocrlf")
}

Invoke-Probe "Python and Conda" {
    where.exe python
    Invoke-IfAvailable "python" @("--version")
    Invoke-IfAvailable "python" @("-c", "import sys; print(sys.executable); print(sys.version)")
    where.exe conda
    Invoke-IfAvailable "conda" @("--version")
    Invoke-IfAvailable "conda" @("env", "list")
}

Invoke-Probe "Candidate Python interpreter versions" {
    $paths = @()
    try { $paths += (where.exe python) } catch {}
    try {
        $condaInfo = conda env list 2>$null
        foreach ($line in $condaInfo) {
            if ($line -match '^\S+\s+[\*\s]\s+(.+)$') {
                $envPath = $Matches[1].Trim()
                $candidate = Join-Path $envPath "python.exe"
                if (Test-Path $candidate) { $paths += $candidate }
            }
        }
    } catch {}
    $paths | Sort-Object -Unique | ForEach-Object {
        Write-Output "PYTHON=$($_)"
        try { & $_ --version } catch { Write-Output "ERROR=$($_.Exception.Message)" }
    }
}

Invoke-Probe "ROS2 CLI and environment" {
    where.exe ros2
    Invoke-IfAvailable "ros2" @("--help")
    Invoke-IfAvailable "ros2" @("doctor", "--report")
    [System.Environment]::GetEnvironmentVariables().GetEnumerator() |
        Where-Object { $_.Key -match 'ROS|AMENT|COLCON|RMW' } |
        Sort-Object Key |
        ForEach-Object { Write-Output "$($_.Key)=$($_.Value)" }
    foreach ($path in @("C:\dev\ros2_humble", "C:\opt\ros\humble", "C:\ros2_humble")) {
        Write-Output "ROS_COMMON_PATH_EXISTS $path = $(Test-Path $path)"
    }
}

Invoke-Probe "Build tools" {
    where.exe cmake
    Invoke-IfAvailable "cmake" @("--version")
    where.exe colcon
    Invoke-IfAvailable "colcon" @("--version")
    where.exe cl
    where.exe ninja
    Invoke-IfAvailable "ninja" @("--version")
    Get-Command cl -ErrorAction SilentlyContinue
    Get-Command cmake -ErrorAction SilentlyContinue
    $vswherePaths = @(
        "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe",
        "${env:ProgramFiles}\Microsoft Visual Studio\Installer\vswhere.exe"
    )
    foreach ($vswhere in $vswherePaths) {
        if (Test-Path $vswhere) {
            & $vswhere -all -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        }
    }
}

Invoke-Probe "OpenCV import/build" {
    & $PythonExecutable -c "import cv2, sys; print(sys.executable); print(cv2.__version__); print('aruco=', hasattr(cv2, 'aruco')); info=cv2.getBuildInformation(); print('MSMF=', ('Media Foundation' in info or 'MSMF' in info)); print('DirectShow=', ('DirectShow' in info)); print('GUI marker=', ('GUI:' in info or 'Win32 UI' in info or 'QT:' in info or 'GTK' in info))"
}

Invoke-Probe "LeRobot and pyserial package checks" {
    & $PythonExecutable -m pip show lerobot
    & $PythonExecutable -m pip show pyserial
    & $PythonExecutable -c "import sys, importlib.util; print(sys.executable); print('lerobot_spec=', importlib.util.find_spec('lerobot')); print('serial_spec=', importlib.util.find_spec('serial'))"
}

Invoke-Probe "Read-only serial port enumeration" {
    & $PythonExecutable -c "from serial.tools import list_ports; [print(f'{p.device}|{p.description}|{p.hwid}') for p in list_ports.comports()]"
}
