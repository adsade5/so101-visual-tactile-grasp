Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

Write-Host ''
Write-Host 'MVP-3C-LIVE MANUAL ACCEPTANCE'
Write-Host 'This script does not start ROS2, hardware, TCP server, or COM4.'
Write-Host ''
Write-Host '[Terminal 0] Start Zenoh Router manually'
Write-Host '[Terminal 1] Start read-only SO-101 TCP server manually'
Write-Host '[Terminal 2] Start ROS2 hardware bridge manually'
Write-Host '[Terminal 3] Run topic checks manually'
Write-Host ''
Write-Host 'See: docs\MVP3C_LIVE_MANUAL_ACCEPTANCE.md'
Write-Host ''
Write-Host 'MANUAL_ACCEPTANCE_INSTRUCTIONS_READY'
Write-Host 'NO_PROCESS_STARTED'
Write-Host 'NO_COM_PORT_OPENED'
Write-Host 'NO_HARDWARE_COMMAND_SENT'
exit 0
