"""
Mars Lander - PC端 800x800 灰度显示
协议: [0xAA55AA55][size:4][灰度数据]
"""
import sys
import struct
import time
import serial
import numpy as np
import cv2

SYNC = b'\x55\xAA\x55\xAA'

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 115200, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()
    print("Ready. Press 'q' to quit.")

    cv2.namedWindow('Mars Lander Camera', cv2.WINDOW_NORMAL)
    frame_count = 0
    last_time = time.time()
    buf = b''
    debug_timer = time.time()
    total_bytes = 0
    markers = 0
    W, H = 800, 800
    EXPECTED = W * H

    try:
        while True:
            n = ser.in_waiting
            if n > 0:
                data = ser.read(n)
                buf += data
                total_bytes += len(data)
                if len(buf) > 2_000_000:
                    buf = buf[-1_000_000:]

            now = time.time()

            if now - debug_timer > 3:
                debug_timer = now
                fps = frame_count / max(now - last_time, 1)
                print(f"[DBG] {total_bytes/1024:.0f}KB buf={len(buf)/1024:.0f}KB mk={markers} fr={frame_count} {fps:.2f}FPS")

            while len(buf) >= 8:
                idx = buf.find(SYNC)
                if idx < 0:
                    buf = buf[-3:]
                    break
                if len(buf) < idx + 8:
                    break

                size = struct.unpack('<I', buf[idx+4:idx+8])[0]
                markers += 1

                if size != EXPECTED:
                    buf = buf[idx+1:]
                    continue

                end = idx + 8 + size
                if len(buf) < end:
                    break

                frame_data = buf[idx+8:end]
                buf = buf[end:]

                img = np.frombuffer(frame_data, dtype=np.uint8).reshape((H, W))
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                img = cv2.resize(img, (800, 800), interpolation=cv2.INTER_NEAREST)

                frame_count += 1
                fps = frame_count / max(now - last_time, 1)
                cv2.putText(img, f"#{frame_count} {W}x{H} {fps:.2f}FPS",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow('Mars Lander Camera', img)
                print(f"[FRAME #{frame_count}] {fps:.2f}FPS")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print(f"\nDone: {frame_count} frames")
    finally:
        ser.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
