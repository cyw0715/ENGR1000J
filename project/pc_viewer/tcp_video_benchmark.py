"""Windows-native raw ESP32 TCP video throughput benchmark (no AI inference)."""
import socket
import struct
import time
import sys
import numpy as np
import cv2

from esp_discovery import discover_esp32

IP = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
PORT = 5000
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


def main():
    sock = socket.create_connection((IP, PORT), timeout=10)
    sock.settimeout(5)
    print(f"TCP_CONNECTED {IP}:{PORT}", flush=True)
    buffer = b""
    frames = 0
    valid_jpeg = 0
    total_bytes = 0
    resync = 0
    start = time.monotonic()
    deadline = start + DURATION_S

    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            print("SOCKET_TIMEOUT", flush=True)
            break
        if not chunk:
            print("TCP_CLOSED", flush=True)
            break
        buffer += chunk
        while len(buffer) >= 4:
            size = struct.unpack("<I", buffer[:4])[0]
            if not 1000 <= size <= 200000:
                buffer = buffer[1:]
                resync += 1
                continue
            if len(buffer) < 4 + size:
                break
            jpeg = buffer[4:4 + size]
            buffer = buffer[4 + size:]
            frames += 1
            total_bytes += size
            if jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9":
                # Decode periodically to validate the actual wire payload.
                if frames % 10 == 0:
                    valid_jpeg += int(cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE) is not None)
                else:
                    valid_jpeg += 1

    elapsed = time.monotonic() - start
    sock.close()
    print("RESULT", flush=True)
    print(f"ELAPSED_S={elapsed:.3f}", flush=True)
    print(f"FRAMES={frames}", flush=True)
    print(f"FPS={frames / elapsed:.2f}", flush=True)
    print(f"MEAN_JPEG_BYTES={total_bytes / frames:.0f}" if frames else "MEAN_JPEG_BYTES=NA", flush=True)
    print(f"PAYLOAD_KBPS={total_bytes / elapsed / 1024:.1f}", flush=True)
    print(f"VALID_JPEG={valid_jpeg}/{frames}", flush=True)
    print(f"RESYNC_BYTES={resync}", flush=True)


if __name__ == "__main__":
    main()
