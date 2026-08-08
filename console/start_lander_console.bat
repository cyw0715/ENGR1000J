@echo off
setlocal EnableExtensions

rem One-click launcher for the Mars Lander PC console.
rem Optional first argument: ESP32-P4 IP address, e.g. 192.168.137.58
set "SCRIPT_DIR=%~dp0"
set "PYTHONW=%SCRIPT_DIR%.venv_win\Scripts\pythonw.exe"
set "CONSOLE=%SCRIPT_DIR%lander_console.py"

if not exist "%PYTHONW%" (
    echo ERROR: Windows virtual-environment Python was not found:
    echo %PYTHONW%
    pause
    exit /b 1
)

if not exist "%CONSOLE%" (
    echo ERROR: Console program was not found:
    echo %CONSOLE%
    pause
    exit /b 1
)

if "%~1"=="" (
    start "Mars Lander Console" "%PYTHONW%" "%CONSOLE%"
) else (
    start "Mars Lander Console" "%PYTHONW%" "%CONSOLE%" "%~1"
)

if errorlevel 1 (
    echo ERROR: The console process could not be started.
    pause
    exit /b 1
)

endlocal
exit /b 0
