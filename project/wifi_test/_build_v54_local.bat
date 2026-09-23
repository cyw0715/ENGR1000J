@echo off
set IDF_PATH=C:\Users\Kerbal Chen\esp\esp-idf
set IDF_TOOLS_PATH=C:\Users\Kerbal Chen\.espressif
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.4_py3.11_env
set PATH=C:\Espressif\python_env\idf5.4_py3.11_env\Scripts;C:\Users\Kerbal Chen\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;C:\Users\Kerbal Chen\.espressif\tools\cmake\3.30.2\bin;C:\Users\Kerbal Chen\.espressif\tools\ninja\1.12.1;C:\Users\Kerbal Chen\.espressif\tools\idf-exe\1.0.3;C:\Users\Kerbal Chen\.espressif\tools\ccache\4.10.2;C:\Users\Kerbal Chen\esp\esp-idf\tools;%PATH%
cd /d "E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test"
echo === set-target ===
"C:\Espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe" "C:\Users\Kerbal Chen\esp\esp-idf\tools\idf.py" set-target esp32p4
echo === build ===
"C:\Espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe" "C:\Users\Kerbal Chen\esp\esp-idf\tools\idf.py" build
echo === DONE ===
