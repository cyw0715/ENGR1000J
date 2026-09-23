"""
USB Serial/JTAG 连通性测试
"""
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
print(f"Testing {port}...")
ser = serial.Serial(port, 115200, timeout=1)
time.sleep(2)
ser.reset_input_buffer()
print("Reading 10 samples...")

for i in range(10):
    data = ser.read(256)
    if data:
        hex_str = data[:32].hex(' ')
        print(f"[{i}] {len(data)} bytes: {hex_str}")
    else:
        print(f"[{i}] empty")

ser.close()
print("Done.")
