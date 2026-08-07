param([string]$CommandFile)
Write-Output 'FAKE_STDOUT'
[Console]::Error.WriteLine('FAKE_STDERR')
exit 0
