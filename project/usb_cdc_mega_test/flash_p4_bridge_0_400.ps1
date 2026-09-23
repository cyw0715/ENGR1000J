$ErrorActionPreference = 'Stop'
$py = 'C:\Espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe'
$base = 'E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\usb_cdc_mega_test\build'
& $py -m esptool --chip esp32p4 -p COM5 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m 0x2000 "$base\bootloader\bootloader.bin" 0x8000 "$base\partition_table\partition-table.bin" 0x10000 "$base\usb_cdc_mega_test.bin"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
