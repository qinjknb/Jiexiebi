# 直接调用 ARMCC V5 工具链构建 GCWJ 工程（等价于 µVision 的 Build）
# 用法: pwsh -File build-fw.ps1    （在 D:\GCWJ 目录下运行）
$ErrorActionPreference = 'Continue'
$armccDir = 'D:\Keil_v5\ARM\ARMCC\bin'
$root     = 'D:\GCWJ'
$log      = Join-Path $root 'tools\build-direct.log'

$env:TMP  = Join-Path $root 'tools\tmp'
$env:TEMP = Join-Path $root 'tools\tmp'
New-Item -ItemType Directory -Force -Path $env:TMP, (Join-Path $root 'build'), (Join-Path $root 'build\obj') | Out-Null

$cSources = @(
    'User\main.c', 'User\LED.c', 'User\Servo.c', 'User\stm32f10x_it.c',
    'Library\misc.c',
    'Library\stm32f10x_adc.c','Library\stm32f10x_bkp.c','Library\stm32f10x_can.c',
    'Library\stm32f10x_cec.c','Library\stm32f10x_crc.c','Library\stm32f10x_dac.c',
    'Library\stm32f10x_dbgmcu.c','Library\stm32f10x_dma.c','Library\stm32f10x_exti.c',
    'Library\stm32f10x_flash.c','Library\stm32f10x_fsmc.c','Library\stm32f10x_gpio.c',
    'Library\stm32f10x_i2c.c','Library\stm32f10x_iwdg.c','Library\stm32f10x_pwr.c',
    'Library\stm32f10x_rcc.c','Library\stm32f10x_rtc.c','Library\stm32f10x_sdio.c',
    'Library\stm32f10x_spi.c','Library\stm32f10x_tim.c','Library\stm32f10x_usart.c',
    'Library\stm32f10x_wwdg.c',
    'Startup\core_cm3.c','Startup\system_stm32f10x.c'
)

$asmSource = 'Startup\startup_stm32f10x_md.s'

$cFlags = @('--c99','-c','--cpu','Cortex-M3','-g','-O1',
            '-DUSE_STDPERIPH_DRIVER','-DSTM32F10X_MD',
            '-I.\User','-I.\Library','-I.\Startup')

$objects = @()
$i = 0

function Run([string]$exe, [string[]]$argsList) {
    $line = "$exe " + ($argsList -join ' ')
    Add-Content -Path $script:log -Value $line
    & $exe @argsList *>&1 | Tee-Object -FilePath $script:log -Append | ForEach-Object { $script:lastLine = $_ }
    if ($LASTEXITCODE -ne 0) {
        Add-Content -Path $script:log -Value ("*** FAILED exit={0}: {1}" -f $LASTEXITCODE, $line)
        throw ("Tool failed (exit {0}): {1}" -f $LASTEXITCODE, $line)
    }
}

Set-Location $root
Remove-Item $log -ErrorAction SilentlyContinue

# 1) 编译所有 C 文件
foreach ($src in $cSources) {
    $i++
    $obj = 'build\obj\' + [IO.Path]::GetFileNameWithoutExtension($src) + '.o'
    $objects += $obj
    Write-Output "[$i/$($cSources.Count)] $src"
    Run (Join-Path $armccDir 'armcc.exe') ($cFlags + @('-o', $obj, $src))
}

# 2) 汇编启动文件
Write-Output "asm: $asmSource"
$startupObj = 'build\obj\startup_stm32f10x_md.o'
$objects += $startupObj
Run (Join-Path $armccDir 'armasm.exe') @('--cpu','Cortex-M3','-g','-o',$startupObj,$asmSource)

# 3) 链接
Write-Output "link..."
$linkArgs = @('--cpu','Cortex-M3','--strict','--scatter','.\Objects\GCWJ.sct',
              '--list','.\build\GCWJ.map','--info','sizes','--info','totals',
              '--info','unused','--info','veneers','-o','.\build\GCWJ.axf') + $objects
Run (Join-Path $armccDir 'armlink.exe') $linkArgs

# 4) 生成 HEX
Write-Output "fromelf -> hex"
Run (Join-Path $armccDir 'fromelf.exe') @('--i32','.\build\GCWJ.axf','--output','.\build\GCWJ.hex')

Write-Output "BUILD OK"
Get-Item '.\build\GCWJ.axf','.\build\GCWJ.hex' | Select-Object Name,Length,LastWriteTime | Format-Table -AutoSize
