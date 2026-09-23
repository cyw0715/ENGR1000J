"""
调试: 查看串口收到的原始数据
"""
import sys
import serial

port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
ser = serial.Serial(port, 115200, timeout=2)
ser.reset_input_buffer()

print(f"Reading from {port}... Press Ctrl+C to stop")
count = 0
while True:
    data = ser.read(256)
    if data:
        count += 1
        # 显示十六进制
        hex_str = ' '.join(f'{b:02X}' for b in data[:64])
        print(f"[{count}] {len(data)} bytes: {hex_str}...")
        # 检查是否包含帧头标记
        if b'\xAA\x55\xAA\x55' in data:
            print("  >>> FOUND FRAME MARKER! <<<")
