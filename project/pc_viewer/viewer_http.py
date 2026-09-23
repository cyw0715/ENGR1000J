"""
Mars Lander - PC端HTTP帧接收与显示
通过HTTP流式接收ESP32-P4的摄像头帧数据并实时显示

依赖: pip install opencv-python numpy requests
用法: python viewer_http.py
"""

import time
import numpy as np
import cv2
import requests
import struct

# ESP32-P4地址 (USB网络或WiFi)
ESP32_IP = "192.168.4.1"  # AP模式默认IP
# ESP32_IP = "192.168.137.100"  # WiFi STA模式

def rgb565_to_bgr(data, width, height):
    """RGB565转BGR (OpenCV格式)"""
    arr = np.frombuffer(data, dtype=np.uint16).reshape((height, width))
    r = ((arr >> 11) & 0x1F) << 3
    g = ((arr >> 5) & 0x3F) << 2
    b = (arr & 0x1F) << 3
    return np.stack([b, g, r], axis=-1).astype(np.uint8)

def main():
    url = f"http://{ESP32_IP}/stream"
    print(f"Connecting to {url}...")

    cv2.namedWindow('Mars Lander Camera', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Mars Lander Camera', 800, 800)

    frame_count = 0
    last_time = time.time()

    try:
        response = requests.get(url, stream=True, timeout=10)
        print("Connected! Receiving frames...")

        boundary = b'--frame'
        buffer = b''

        for chunk in response.iter_content(chunk_size=4096):
            buffer += chunk

            # 解析multipart boundary
            while True:
                idx = buffer.find(boundary)
                if idx < 0:
                    break

                # 找到下一个boundary
                next_idx = buffer.find(boundary, idx + len(boundary))
                if next_idx < 0:
                    break

                # 提取一个frame
                frame_part = buffer[idx + len(boundary):next_idx]
                buffer = buffer[next_idx:]

                # 解析Content-Type和数据
                header_end = frame_part.find(b'\r\n\r\n')
                if header_end < 0:
                    continue

                header = frame_part[:header_end].decode('utf-8', errors='ignore')
                frame_data = frame_part[header_end + 4:]

                # 去掉尾部的\r\n
                if frame_data.endswith(b'\r\n'):
                    frame_data = frame_data[:-2]

                if len(frame_data) == 0:
                    continue

                frame_count += 1
                now = time.time()
                fps = 1.0 / max(now - last_time, 0.001)
                last_time = now

                # 尝试解码为图像
                width = 800
                height = 800
                expected_size = width * height * 2  # RGB565

                if len(frame_data) == expected_size:
                    img = rgb565_to_bgr(frame_data, width, height)
                    cv2.putText(img, f"Frame #{frame_count} FPS:{fps:.1f}",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow('Mars Lander Camera', img)
                else:
                    # 可能是其他格式，尝试直接解码
                    nparr = np.frombuffer(frame_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if img is not None:
                        cv2.putText(img, f"Frame #{frame_count} FPS:{fps:.1f}",
                                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.imshow('Mars Lander Camera', img)
                    else:
                        print(f"Frame #{frame_count}: {len(frame_data)} bytes (unknown format)")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except requests.exceptions.ConnectionError:
        print(f"Cannot connect to {ESP32_IP}")
        print("Make sure ESP32-P4 is running and accessible")
    except requests.exceptions.Timeout:
        print("Connection timeout")
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
