"""Quick TCP test - receive one JPEG frame from ESP32"""
import socket, struct, sys

ESP32_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.74"

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5)
print(f"Connecting to {ESP32_IP}:5000...")
sock.connect((ESP32_IP, 5000))
print("Connected! Waiting for frame...")

# Read 4-byte size header
hdr = b''
while len(hdr) < 4:
    chunk = sock.recv(4 - len(hdr))
    if not chunk:
        print("Connection closed before header"); sock.close(); exit(1)
    hdr += chunk

size = struct.unpack('<I', hdr)[0]
print(f"Frame size: {size} bytes")

# Read JPEG data
buf = b''
while len(buf) < size:
    chunk = sock.recv(min(65536, size - len(buf)))
    if not chunk:
        break
    buf += chunk

print(f"Received: {len(buf)} bytes")

# Save and show
with open("test_frame.jpg", "wb") as f:
    f.write(buf)
print("Saved to test_frame.jpg")

import cv2, numpy as np
img = cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_GRAYSCALE)
if img is not None:
    print(f"Image: {img.shape}")
    cv2.imshow("ESP32 Frame", cv2.resize(img, (640, 640)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Failed to decode JPEG")

sock.close()
