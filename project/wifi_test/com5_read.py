"""Read-only COM5 log capture for ESP32-P4 firmware verification.
Does not write to or reset the target.
Usage: python com5_read.py [seconds]
"""

import sys
import time

import serial

PORT = "COM5"
BAUD = 115200
DURATION_SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

print(f"[COM] Read-only capture: {PORT} @ {BAUD}, {DURATION_SECONDS:.0f}s")
print("[COM] No serial bytes will be written; target reset is not requested.")

with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
    deadline = time.monotonic() + DURATION_SECONDS
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            print(chunk.decode("utf-8", errors="replace"), end="", flush=True)

print("\n[COM] Capture complete.")
