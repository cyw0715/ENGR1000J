"""
Mars Lander - PC端 WiFi JPEG接收 (实时FPS)
"""
import socket
import struct
import time
import numpy as np
import cv2
import concurrent.futures

PC_PORT = 5000

def check_host(ip):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex((ip, PC_PORT))
        sock.close()
        if result == 0:
            return ip
    except:
        pass
    return None

def find_esp32():
    print("Scanning 192.168.137.x for ESP32...")
    base = "192.168.137."
    ips = [base + str(i) for i in [241, 133, 100, 101, 102, 1, 2, 10, 20, 50, 150, 200, 254]]
    with concurrent.futures.ThreadPoolExecutor(max_workers=20) as executor:
        futures = {executor.submit(check_host, ip): ip for ip in ips}
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result:
                print(f"  Found: {result}:{PC_PORT}")
                return result

    print("  Scanning full range...")
    all_ips = [base + str(i) for i in range(2, 255)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        futures = {executor.submit(check_host, ip): ip for ip in all_ips}
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result:
                print(f"  Found: {result}:{PC_PORT}")
                return result
    return None

def main():
    esp_ip = find_esp32()
    if not esp_ip:
        print("ESP32 not found!")
        return

    print(f"Connecting to {esp_ip}:{PC_PORT}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    try:
        sock.connect((esp_ip, PC_PORT))
        print("Connected!")
    except Exception as e:
        print(f"Failed: {e}")
        return

    cv2.namedWindow('Mars Lander Camera', cv2.WINDOW_NORMAL)
    frame_count = 0
    last_time = time.time()
    last_frame_time = time.time()
    fps = 0.0
    buf = b''

    try:
        while True:
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                print("Disconnected")
                break
            buf += data

            while len(buf) >= 4:
                jpeg_size = struct.unpack('<I', buf[:4])[0]
                if jpeg_size < 1000 or jpeg_size > 100000:
                    buf = buf[1:]
                    continue
                if len(buf) < 4 + jpeg_size:
                    break

                jpeg_data = buf[4:4+jpeg_size]
                buf = buf[4+jpeg_size:]

                nparr = np.frombuffer(jpeg_data, dtype=np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue

                h, w = img.shape[:2]

                # 实时FPS（基于最近一帧的时间间隔）
                now = time.time()
                dt = now - last_frame_time
                last_frame_time = now
                if dt > 0:
                    instant_fps = 1.0 / dt
                    # 平滑：80%旧值 + 20%新值
                    fps = fps * 0.8 + instant_fps * 0.2

                frame_count += 1

                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                img = cv2.resize(img, (640, 640), interpolation=cv2.INTER_LINEAR)

                # 显示信息
                cv2.putText(img, f"#{frame_count} {w}x{h} {jpeg_size}B",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(img, f"{fps:.1f} FPS",
                            (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

                cv2.imshow('Mars Lander Camera', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
