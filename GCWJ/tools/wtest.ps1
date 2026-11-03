$ErrorActionPreference = 'Continue'
New-Item -ItemType Directory -Force -Path 'D:\GCWJ\__wtest' | Out-Null
Set-Content -Path 'D:\GCWJ\__wtest\a.txt' -Value 'x'
if (-not (Test-Path 'D:\GCWJ\__wtest\a.txt')) { Write-Output 'CREATE FAILED'; exit 1 }
Set-Content -Path 'D:\GCWJ\__wtest\a.txt' -Value 'y'
$c = Get-Content 'D:\GCWJ\__wtest\a.txt'
Remove-Item 'D:\GCWJ\__wtest' -Recurse -Force
Write-Output "write+overwrite+delete OK, content=$c"
