import sys
from pathlib import Path

ROOT = Path(r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\usb_cdc_mega_test")
sys.path.insert(0, str(ROOT))
import p4_mega_upload_sr04 as uploader

image, highest = uploader.parse_intel_hex(uploader.DEFAULT_HEX)
assert len(image) == 5376, len(image)
assert highest == 5140, highest
assert len(image) // uploader.PAGE_SIZE == 21
page = bytes(range(256))
command = bytes((uploader.CMD_PROGRAM_FLASH_ISP, 1, 0, 0xC1, 10,
                 0x40, 0x4C, 0x20, 0xFF, 0xFF)) + page
assert len(command) == 266
assert command[10:] == page
print(f"UPLOADER_STATIC_OK pages={len(image)//uploader.PAGE_SIZE} highest={highest} program_command_bytes={len(command)}")
