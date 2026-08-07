param([string]$CommandFile)
Write-Output 'FAKE_WRAPPER_STARTED'
Start-Sleep -Seconds 3
Write-Output 'FAKE_LAUNCH_DONE'
exit 0
