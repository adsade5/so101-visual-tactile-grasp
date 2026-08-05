@echo off
setlocal
set "PIXI_WS=C:\pixi_ws"
set "PIXI_EXE=%USERPROFILE%\.pixi\bin\pixi.exe"
set "ROS2_SETUP=%PIXI_WS%\ros2-windows\local_setup.bat"
set "VS_SETUP=C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat"

if not exist "%PIXI_EXE%" (
  echo ERROR: pixi.exe not found at "%PIXI_EXE%"
  exit /b 1
)
if not exist "%ROS2_SETUP%" (
  echo ERROR: ROS2 setup not found at "%ROS2_SETUP%"
  exit /b 1
)
if defined ROS_DISTRO if /I not "%ROS_DISTRO%"=="lyrical" (
  echo ERROR: Refusing to mix existing ROS_DISTRO=%ROS_DISTRO% with Lyrical.
  exit /b 2
)

cd /d "%PIXI_WS%"
"%PIXI_EXE%" run cmd /d /k "if exist ""%VS_SETUP%"" call ""%VS_SETUP%"" -arch=x64 -host_arch=x64 && call ""%ROS2_SETUP%"" && if /I not ""%ROS_DISTRO%""==""lyrical"" (echo ERROR: ROS_DISTRO=%ROS_DISTRO% & exit /b 3) && echo ROS_DISTRO=%ROS_DISTRO% && where python && where colcon && where cmake && where cl && echo RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%"
