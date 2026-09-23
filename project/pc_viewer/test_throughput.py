"""
串口吞吐量测试
"""
import sys
import time
import serial

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    print(f"Testing throughput on {port}...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2)
    ser.reset_input_buffer()

    total = 0
    t0 = time.time()
    last_report = t0

    try:
        while True:
            data = ser.read(65536)
            if data:
                total += len(data)

            now = time.time()
            if now - last_report >= 2:
                elapsed = now - t0
                rate = total / elapsed / 1024  # KB/s
                print(f"[{elapsed:.0f}s] {total/1024:.0f} KB received, rate: {rate:.1f} KB/s")
                last_report = now

    except KeyboardInterrupt:
        elapsed = time.time() - t0
        print(f"\nTotal: {total/1024:.0f} KB in {elapsed:.1f}s = {total/elapsed/1024:.1f} KB/s")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
