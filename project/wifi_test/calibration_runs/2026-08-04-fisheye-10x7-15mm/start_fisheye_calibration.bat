@echo off
setlocal
cd /d "%~dp0"
set "PY=E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\pc_viewer\.venv_win\Scripts\python.exe"
"%PY%" capture_and_calibrate_fisheye.py
if errorlevel 1 (
  echo.
  echo Calibration did not complete. Check P4 power, hotspot, TCP5000 and view count.
  pause
)
