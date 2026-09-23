@echo off
:: ESP-IDF v5.4.1 environment
set IDF_PATH=C:\Users\Kerbal Chen\esp\esp-idf
set IDF_TOOLS_PATH=C:\Espressif\tools
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.4_py3.11_env
set PATH=C:\Espressif\python_env\idf5.4_py3.11_env\Scripts;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\idf-exe\1.0.3;C:\Espressif\tools\ccache\4.10.2;C:\Users\Kerbal Chen\esp\esp-idf\tools;%PATH%
cd /d "E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test"
del /f sdkconfig 2>nul
rmdir /s /q build 2>nul
echo === set-target ===
python "C:\Users\Kerbal Chen\esp\esp-idf\tools\idf.py" set-target esp32p4
echo === SPIRAM CHECK AFTER SET-TARGET ===
findstr "CONFIG_SPIRAM_SPEED" sdkconfig
findstr "IDF_EXPERIMENTAL_FEATURES" sdkconfig
echo === patch chip rev ===
powershell -Command "(Get-Content sdkconfig) -replace 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=.*', 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' | Set-Content sdkconfig"
echo === build ===
python "C:\Users\Kerbal Chen\esp\esp-idf\tools\idf.py" build
echo === BUILD DONE ===
echo === patch chip rev again ===
powershell -Command "(Get-Content sdkconfig) -replace 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=.*', 'CONFIG_ESPTOOLPY_CHIP_REV_MIN=0' | Set-Content sdkconfig"
echo === FINAL SPIRAM CHECK ===
findstr "CONFIG_SPIRAM_SPEED" sdkconfig
findstr "IDF_EXPERIMENTAL_FEATURES" sdkconfig
echo === ALL DONE ===
