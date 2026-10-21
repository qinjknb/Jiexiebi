@echo off
cd /d D:\GCWJ
"D:\Keil_v5\UV4\UV4.exe" -j0 -b "D:\GCWJ\GCWJ.1.uvprojx" -t "Target 1" -o "D:\GCWJ\tools\uv-build.log"
echo BUILD_EXIT=%ERRORLEVEL% > "D:\GCWJ\tools\uv-run.log"
"D:\Keil_v5\UV4\UV4.exe" -j0 -f "D:\GCWJ\GCWJ.1.uvprojx" -t "Target 1" -o "D:\GCWJ\tools\uv-flash.log"
echo FLASH_EXIT=%ERRORLEVEL% >> "D:\GCWJ\tools\uv-run.log"
