$env:IDF_PATH = 'D:\.espressif\v5.5.4\esp-idf'
$env:IDF_TOOLS_PATH = 'C:\Espressif\tools'
$env:IDF_PYTHON_ENV_PATH = 'C:\Espressif\tools\python\v5.5.4\venv'
$env:PATH = "C:\Espressif\tools\python\v5.5.4\venv\Scripts;C:\Espressif\tools\xtensa-esp-elf\esp-14.2.0_20241119\xtensa-esp-elf\bin;C:\Espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;C:\Espressif\tools\cmake\3.30.2\bin;C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\idf-exe\1.0.3;C:\Espressif\tools\ccache\4.10.2;${env:IDF_PATH}\tools;$env:PATH"

Set-Location 'E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test'

Write-Host "=== set-target esp32p4 ==="
python "${env:IDF_PATH}\tools\idf.py" set-target esp32p4
if ($LASTEXITCODE -ne 0) { Write-Host "set-target FAILED"; exit 1 }

Write-Host "=== build ==="
python "${env:IDF_PATH}\tools\idf.py" build
if ($LASTEXITCODE -ne 0) { Write-Host "build FAILED"; exit 1 }

Write-Host "=== BUILD DONE ==="
