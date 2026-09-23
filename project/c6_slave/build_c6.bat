@echo off
cd /d "E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\c6_slave"
rmdir /s /q build 2>nul
del /f sdkconfig 2>nul
idf.py set-target esp32c6
powershell -Command "(Get-Content sdkconfig) -replace 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=.*', 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' | Set-Content sdkconfig"
findstr /C:"CONFIG_ESPTOOLPY_CHIP_REV_MIN=0" sdkconfig >nul 2>&1
if errorlevel 1 echo CONFIG_ESPTOOLPY_CHIP_REV_MIN=0 >> sdkconfig
idf.py build
