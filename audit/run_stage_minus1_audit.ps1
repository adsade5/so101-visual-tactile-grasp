[CmdletBinding()]
param(
    [switch]$TestCamera,
    [string]$PythonExecutable = "python"
)

$ErrorActionPreference = "Continue"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$WorkspaceRoot = Split-Path -Parent $ProjectRoot
$LegacyRoot = Join-Path $WorkspaceRoot "so101_ros2_tactile_guard"
$ResultsDir = Join-Path $ProjectRoot "audit_results"
$MinimalWs = Join-Path $ResultsDir "minimal_ros2_ws"
$Commands = New-Object System.Collections.Generic.List[string]
$BlockingFailures = New-Object System.Collections.Generic.List[string]

New-Item -ItemType Directory -Force -Path $ResultsDir | Out-Null

function Invoke-Logged {
    param(
        [string]$Name,
        [string]$Command,
        [string]$WorkingDirectory = $WorkspaceRoot
    )
    $Commands.Add("[$WorkingDirectory] $Command") | Out-Null
    $log = Join-Path $ResultsDir "$Name.log"
    "COMMAND: $Command`nWORKDIR: $WorkingDirectory`nTIMESTAMP: $(Get-Date -Format o)`n" | Set-Content -Encoding UTF8 -Path $log
    Push-Location $WorkingDirectory
    try {
        powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $Command *>> $log
        $code = $LASTEXITCODE
    }
    catch {
        "EXCEPTION: $($_.Exception.Message)" | Add-Content -Encoding UTF8 -Path $log
        $code = 99
    }
    finally {
        Pop-Location
    }
    "EXIT_CODE=$code" | Add-Content -Encoding UTF8 -Path $log
    return $code
}

function Test-CommandAvailable {
    param([string]$CommandName)
    return $null -ne (Get-Command $CommandName -ErrorAction SilentlyContinue)
}

Write-Output "Stage -1 audit started: $(Get-Date -Format o)"
Write-Output "WorkspaceRoot=$WorkspaceRoot"
Write-Output "ProjectRoot=$ProjectRoot"
Write-Output "LegacyRoot=$LegacyRoot"
Write-Output "ResultsDir=$ResultsDir"
Write-Output "PythonExecutable=$PythonExecutable"
Write-Output "TestCamera=$TestCamera"

$code = Invoke-Logged -Name "check_environment" -Command "& '$ScriptDir\check_environment.ps1' -ResultsDir '$ResultsDir' -PythonExecutable '$PythonExecutable'"
if ($code -ne 0) { $BlockingFailures.Add("check_environment exited $code") | Out-Null }

$code = Invoke-Logged -Name "check_python_runtime" -Command "& '$PythonExecutable' '$ScriptDir\check_python_runtime.py' --results-dir '$ResultsDir'"
if ($code -ne 0) { $BlockingFailures.Add("check_python_runtime exited $code") | Out-Null }

$cameraArg = ""
if ($TestCamera) { $cameraArg = " --test-camera" }
$code = Invoke-Logged -Name "check_opencv_camera" -Command "& '$PythonExecutable' '$ScriptDir\check_opencv_camera.py' --results-dir '$ResultsDir'$cameraArg"
if ($code -ne 0) { $BlockingFailures.Add("check_opencv_camera exited $code") | Out-Null }

$code = Invoke-Logged -Name "check_ros2_import" -Command "& '$PythonExecutable' '$ScriptDir\check_ros2_import.py' --results-dir '$ResultsDir'"
if ($code -ne 0) { $BlockingFailures.Add("check_ros2_import exited $code") | Out-Null }

$OldExtensionSrc = Join-Path $LegacyRoot "lerobot_extension\src"
$code = Invoke-Logged -Name "check_lerobot_import" -Command "& '$PythonExecutable' '$ScriptDir\check_lerobot_import.py' --results-dir '$ResultsDir' --old-extension-src '$OldExtensionSrc'"
if ($code -ne 0) { $BlockingFailures.Add("check_lerobot_import exited $code") | Out-Null }

$code = Invoke-Logged -Name "check_combined_import" -Command "& '$PythonExecutable' '$ScriptDir\check_combined_import.py' --results-dir '$ResultsDir'"
if ($code -ne 0) { $BlockingFailures.Add("check_combined_import exited $code") | Out-Null }

$code = Invoke-Logged -Name "scan_linux_dependencies" -Command "& '$PythonExecutable' '$ScriptDir\scan_linux_dependencies.py' '$LegacyRoot' --results-dir '$ResultsDir'"
if ($code -ne 0) { $BlockingFailures.Add("scan_linux_dependencies exited $code") | Out-Null }

New-Item -ItemType Directory -Force -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\stage_minus1_python_probe") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $MinimalWs "src\stage_minus1_cpp_probe\src") | Out-Null

@"
from setuptools import setup

package_name = 'stage_minus1_python_probe'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stage_minus1',
    maintainer_email='stage_minus1@example.invalid',
    description='Safe ROS2 Python stage -1 probe',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'python_probe_node = stage_minus1_python_probe.python_probe_node:main',
        ],
    },
)
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\setup.py")
New-Item -ItemType Directory -Force -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\resource") | Out-Null
"stage_minus1_python_probe" | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\resource\stage_minus1_python_probe")
"" | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\stage_minus1_python_probe\__init__.py")
@"
<?xml version="1.0"?>
<package format="3">
  <name>stage_minus1_python_probe</name>
  <version>0.0.0</version>
  <description>Safe ROS2 Python stage -1 probe</description>
  <maintainer email="stage_minus1@example.invalid">stage_minus1</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\package.xml")
@"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PythonProbeNode(Node):
    def __init__(self):
        super().__init__('python_probe_node')
        self.publisher = self.create_publisher(String, 'stage_minus1_python_probe', 10)
        self.timer = self.create_timer(1.0, self.publish_probe)

    def publish_probe(self):
        msg = String()
        msg.data = 'stage_minus1_python_probe_ok'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PythonProbeNode()
    try:
        rclpy.spin(node)
    finally:
            node.destroy_node()
        rclpy.shutdown()
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_python_probe\stage_minus1_python_probe\python_probe_node.py")

@"
cmake_minimum_required(VERSION 3.8)
project(stage_minus1_cpp_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(cpp_probe_node src/cpp_probe_node.cpp)
ament_target_dependencies(cpp_probe_node rclcpp std_msgs)

install(TARGETS cpp_probe_node DESTINATION lib/`${PROJECT_NAME})

ament_package()
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_cpp_probe\CMakeLists.txt")
@"
<?xml version="1.0"?>
<package format="3">
  <name>stage_minus1_cpp_probe</name>
  <version>0.0.0</version>
  <description>Safe ROS2 C++ stage -1 probe</description>
  <maintainer email="stage_minus1@example.invalid">stage_minus1</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_cpp_probe\package.xml")
@"
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CppProbeNode : public rclcpp::Node {
 public:
  CppProbeNode() : Node("cpp_probe_node") {
    publisher_ = create_publisher<std_msgs::msg::String>("stage_minus1_cpp_probe", 10);
    timer_ = create_wall_timer(1s, [this]() {
      auto message = std_msgs::msg::String();
      message.data = "stage_minus1_cpp_probe_ok";
      publisher_->publish(message);
    });
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppProbeNode>());
  rclcpp::shutdown();
  return 0;
}
"@ | Set-Content -Encoding UTF8 -Path (Join-Path $MinimalWs "src\stage_minus1_cpp_probe\src\cpp_probe_node.cpp")

$ros2Available = Test-CommandAvailable "ros2"
$colconAvailable = Test-CommandAvailable "colcon"
$cmakeAvailable = Test-CommandAvailable "cmake"
$clAvailable = Test-CommandAvailable "cl"

if ($ros2Available -and $colconAvailable) {
    $code = Invoke-Logged -Name "minimal_ros2_colcon_build" -Command "colcon build --event-handlers console_direct+" -WorkingDirectory $MinimalWs
    if ($code -ne 0) { $BlockingFailures.Add("minimal_ros2_colcon_build exited $code") | Out-Null }

    $setup = Join-Path $MinimalWs "install\local_setup.ps1"
    if (Test-Path $setup) {
        $pyRun = Join-Path $ResultsDir "minimal_ros2_python_run.log"
        $cppRun = Join-Path $ResultsDir "minimal_ros2_cpp_run.log"

        $Commands.Add("[$MinimalWs] source install/local_setup.ps1; ros2 run stage_minus1_python_probe python_probe_node; ros2 topic echo --once /stage_minus1_python_probe") | Out-Null
        $pyProc = Start-Process -FilePath "powershell.exe" -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ". '$setup'; ros2 run stage_minus1_python_probe python_probe_node") -RedirectStandardOutput $pyRun -RedirectStandardError $pyRun -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds 3
        $pyEcho = Invoke-Logged -Name "minimal_ros2_python_topic_echo" -Command ". '$setup'; ros2 topic echo --once /stage_minus1_python_probe --timeout 5" -WorkingDirectory $MinimalWs
        Stop-Process -Id $pyProc.Id -Force -ErrorAction SilentlyContinue
        if ($pyEcho -ne 0) { $BlockingFailures.Add("minimal_ros2_python_topic_echo exited $pyEcho") | Out-Null }

        $Commands.Add("[$MinimalWs] source install/local_setup.ps1; ros2 run stage_minus1_cpp_probe cpp_probe_node; ros2 topic echo --once /stage_minus1_cpp_probe") | Out-Null
        $cppProc = Start-Process -FilePath "powershell.exe" -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ". '$setup'; ros2 run stage_minus1_cpp_probe cpp_probe_node") -RedirectStandardOutput $cppRun -RedirectStandardError $cppRun -PassThru -WindowStyle Hidden
        Start-Sleep -Seconds 3
        $cppEcho = Invoke-Logged -Name "minimal_ros2_cpp_topic_echo" -Command ". '$setup'; ros2 topic echo --once /stage_minus1_cpp_probe --timeout 5" -WorkingDirectory $MinimalWs
        Stop-Process -Id $cppProc.Id -Force -ErrorAction SilentlyContinue
        if ($cppEcho -ne 0) { $BlockingFailures.Add("minimal_ros2_cpp_topic_echo exited $cppEcho") | Out-Null }
    }
    else {
        "install/local_setup.ps1 not found after build." | Set-Content -Encoding UTF8 -Path (Join-Path $ResultsDir "minimal_ros2_run_skipped.log")
        $BlockingFailures.Add("minimal_ros2_run skipped because install/local_setup.ps1 was not generated") | Out-Null
    }
}
else {
    "ros2Available=$ros2Available`ncolconAvailable=$colconAvailable`ncmakeAvailable=$cmakeAvailable`nclAvailable=$clAvailable" |
        Set-Content -Encoding UTF8 -Path (Join-Path $ResultsDir "minimal_ros2_skipped.log")
    $BlockingFailures.Add("minimal ROS2 tests skipped because ros2 or colcon is unavailable") | Out-Null
}

$Commands | Set-Content -Encoding UTF8 -Path (Join-Path $ResultsDir "commands_executed.txt")
$BlockingFailures | Set-Content -Encoding UTF8 -Path (Join-Path $ResultsDir "blocking_failures.txt")

Write-Output ""
Write-Output "Stage -1 audit summary"
Write-Output "ResultsDir=$ResultsDir"
Write-Output "BlockingFailures=$($BlockingFailures.Count)"
foreach ($failure in $BlockingFailures) {
    Write-Output "- $failure"
}

if ($BlockingFailures.Count -gt 0) {
    exit 2
}
exit 0
