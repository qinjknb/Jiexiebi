# ============================================================================
# build_fw.ps1 - command line build for the DOFBOT Pro arm firmware
#                (standalone; does NOT touch the Keil GUI build in .\Objects)
#
# Usage (run inside the GCWJ folder):
#   powershell -ExecutionPolicy Bypass -File .\build_fw.ps1
#   powershell -ExecutionPolicy Bypass -File .\build_fw.ps1 -Flash
#
# Output: .\build_check\GCWJ.hex   (+ .\Listings\GCWJ.map)
# Note: this script is pure ASCII on purpose, so Windows PowerShell 5.1
#       parses it correctly no matter which code page is active.
# ============================================================================
param(
    [switch]$Flash
)

$ErrorActionPreference = 'Continue'

$root    = $PSScriptRoot
$keil    = 'D:\keil5\ARM\ARMCC\Bin'
$armcc   = Join-Path $keil 'armcc.exe'
$armasm  = Join-Path $keil 'armasm.exe'
$armlink = Join-Path $keil 'armlink.exe'
$fromelf = Join-Path $keil 'fromelf.exe'

if (-not (Test-Path $armcc)) { throw "armcc not found: $armcc  (edit `$keil in build_fw.ps1)" }

$out = Join-Path $root 'build_check'
$lst = Join-Path $root 'Listings'
New-Item -ItemType Directory -Force -Path $out | Out-Null
New-Item -ItemType Directory -Force -Path $lst | Out-Null

$cFlags = @(
    '-c',
    '--cpu', 'Cortex-M3',
    '-g', '-O1', '--apcs=interwork',
    '-DSTM32F10X_MD', '-DUSE_STDPERIPH_DRIVER',
    '--c99',
    "-I$root\Startup",
    "-I$root\User",
    "-I$root\Library"
)

$skipNames = @('main_original_demo.c', 'main_servohost_backup.c')

$sources  = @()
$sources += Get-ChildItem (Join-Path $root 'User')    -Filter *.c -File
$sources += Get-ChildItem (Join-Path $root 'Library') -Filter *.c -File
$sources += Get-ChildItem (Join-Path $root 'Startup') -Filter *.c -File

$objs = @()
$fail = 0

foreach ($src in $sources) {
    if ($skipNames -contains $src.Name) { continue }
    $obj = Join-Path $out ($src.BaseName + '.o')
    Write-Host ("  CC  {0}" -f $src.Name)
    $res = & $armcc @cFlags -o $obj $src.FullName 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-Host ($res | Out-String) -ForegroundColor Red
        $fail++
    }
    $objs += $obj
}

$startS = Join-Path $root 'Startup\startup_stm32f10x_md.s'
if (Test-Path $startS) {
    $startO = Join-Path $out 'startup_stm32f10x_md.o'
    Write-Host '  AS  startup_stm32f10x_md.s'
    $res = & $armasm --cpu Cortex-M3 -g --apcs=interwork --pd "STM32F10X_MD SETA 1" -o $startO $startS 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Host ($res | Out-String) -ForegroundColor Red; $fail++ }
    $objs += $startO
}

if ($fail -ne 0) { throw "compile failed in $fail file(s)" }

$scatter = Join-Path $root 'Objects\GCWJ.sct'
if (-not (Test-Path $scatter)) { throw "scatter file not found: $scatter" }
$elf = Join-Path $out 'GCWJ.1'
$map = Join-Path $lst 'GCWJ.map'

Write-Host '  LD  GCWJ.1'
$linkArgs = @('--cpu', 'Cortex-M3') + $objs + @(
    '--strict',
    '--scatter', $scatter,
    '--list', $map,
    '--info', 'sizes',
    '--info', 'totals',
    '-o', $elf
)
$res = & $armlink @linkArgs 2>&1
$res | Out-String | Write-Host
if ($LASTEXITCODE -ne 0) { throw 'link failed' }
$res | Where-Object { $_ -match 'Total (RO|RW|ROM)' } | ForEach-Object { Write-Host $_ -ForegroundColor Green }

$hex = Join-Path $out 'GCWJ.hex'
& $fromelf --i32combined --output $hex $elf 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'fromelf failed' }
Write-Host ("  OK  {0}  ({1:N0} bytes)" -f $hex, (Get-Item $hex).Length) -ForegroundColor Green

if ($Flash) {
    $ocd     = Join-Path $root 'tools\bin\openocd.exe'
    $scripts = Join-Path $root 'tools\share\openocd\scripts'
    if (-not (Test-Path $ocd)) { throw "openocd not found: $ocd" }
    Write-Host '  FLASH via ST-Link ...' -ForegroundColor Cyan
    # OpenOCD 的命令解析器会把 Windows 路径里的 \x 当转义，必须换成正斜杠；
    # 路径用 {} 括起来，避免引号被 PowerShell 拆掉
    $hexOcd = $hex -replace '\\', '/'
    $scOcd  = $scripts -replace '\\', '/'
    $progCmd = 'program {' + $hexOcd + '} verify reset exit'
    & $ocd -s $scOcd -f interface/stlink.cfg -f target/stm32f1x.cfg -c $progCmd 2>&1 |
        ForEach-Object { Write-Host $_ }
    if ($LASTEXITCODE -ne 0) { throw 'flash failed: check ST-Link / SWD wiring' }
    Write-Host '  FLASH OK' -ForegroundColor Green
}
