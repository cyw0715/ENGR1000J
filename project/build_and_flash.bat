@echo off
cd /d "E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project"
rmdir /s /q build 2>nul
del /f sdkconfig 2>nul
idf.py set-target esp32p4
powershell -Command "(Get-Content sdkconfig) -replace 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=.*', 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' | Set-Content sdkconfig"
powershell -Command "if (-not (Select-String -Path sdkconfig -Pattern 'CONFIG_ESPTOOLPY_CHIP_REV_MIN')) { Add-Content -Path sdkconfig -Value 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' }"
idf.py build
powershell -Command "(Get-Content sdkconfig) -replace 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=.*', 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' | Set-Content sdkconfig"
powershell -Command "if (-not (Select-String -Path sdkconfig -Pattern 'CONFIG_ESPTOOLPY_CHIP_REV_MIN')) { Add-Content -Path sdkconfig -Value 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' }"
idf.py -p COM5 flash monitor
