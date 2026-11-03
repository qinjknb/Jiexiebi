@echo off
cd /d "%~dp0"

rem 相机访问需要管理员权限，否则会报 0x80070005 拒绝访问
net session >nul 2>&1
if errorlevel 1 (
    echo.
    echo   [需要管理员权限]
    echo   请右键这个 运行.bat，选择"以管理员身份运行"
    echo.
    pause
    exit /b 1
)

set PY=%~dp0.venv\python.exe
if not exist "%PY%" set PY=%~dp0.venv\Scripts\python.exe
if not exist "%PY%" set PY=python

echo 使用解释器: %PY%
echo.
"%PY%" strawberry_demo.py %*

pause
