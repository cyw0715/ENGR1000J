"""Bounded Windows-native validation: ESP32 TCP JPEG -> Depth Anything V2 CUDA."""
import socket
import struct
import time
import numpy as np
import cv2
import torch
from PIL import Image
from transformers import pipeline
import sys
from esp_discovery import discover_esp32

IP = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
PORT = 5000
FRAME_COUNT = 12


def main():
    print("CUDA", torch.cuda.is_available(), flush=True)
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for this validation")
    print("GPU", torch.cuda.get_device_name(0), flush=True)

    # Load and warm the model before opening TCP.  Otherwise model startup
    # causes artificial receive-side backpressure and contaminates the video
    # throughput measurement.
    print("MODEL_LOADING", flush=True)
    model = pipeline(
        "depth-estimation",
        model="depth-anything/Depth-Anything-V2-Base-hf",
        device="cuda",
        dtype=torch.float16,
    )
    model(Image.fromarray(np.zeros((512, 512, 3), dtype=np.uint8)))
    print("MODEL_WARMED", flush=True)

    print(f"CONNECT {IP}:{PORT}", flush=True)
    sock = socket.create_connection((IP, PORT), timeout=10)
    sock.settimeout(12)
    print("TCP_CONNECTED", flush=True)

    buffer = b""
    records = []
    first_receive = None
    while len(records) < FRAME_COUNT:
        chunk = sock.recv(65536)
        if not chunk:
            raise RuntimeError("ESP32 closed TCP connection")
        buffer += chunk

        while len(buffer) >= 4 and len(records) < FRAME_COUNT:
            size = struct.unpack("<I", buffer[:4])[0]
            if not 1000 <= size <= 200000:
                buffer = buffer[1:]
                continue
            if len(buffer) < 4 + size:
                break

            jpeg = buffer[4:4 + size]
            buffer = buffer[4 + size:]
            now = time.monotonic()
            if first_receive is None:
                first_receive = now

            decode_start = time.monotonic()
            gray = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            decode_end = time.monotonic()
            if gray is None:
                raise RuntimeError("JPEG decode failed")

            inference_start = time.monotonic()
            result = model(Image.fromarray(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)))
            depth = result["predicted_depth"].cpu().float().numpy()
            inference_end = time.monotonic()

            records.append({
                "jpeg_bytes": len(jpeg),
                "decode_ms": (decode_end - decode_start) * 1000,
                "infer_ms": (inference_end - inference_start) * 1000,
                "image_shape": gray.shape,
                "depth_shape": depth.shape,
                "depth_min": float(depth.min()),
                "depth_max": float(depth.max()),
                "depth_std": float(depth.std()),
            })
            print(f"FRAME {len(records):02d}: jpeg={len(jpeg)}B decode={records[-1]['decode_ms']:.1f}ms infer={records[-1]['infer_ms']:.1f}ms depth_std={depth.std():.5f}", flush=True)

    elapsed = time.monotonic() - first_receive
    sock.close()

    jpeg = np.array([x["jpeg_bytes"] for x in records])
    decode = np.array([x["decode_ms"] for x in records])
    infer = np.array([x["infer_ms"] for x in records])
    finite = all(
        np.isfinite([x["depth_min"], x["depth_max"], x["depth_std"]]).all()
        and x["depth_std"] > 0 for x in records
    )
    print("RESULT", flush=True)
    print(f"VALID_FRAMES={len(records)}/{FRAME_COUNT}", flush=True)
    print(f"TCP_RECEIVE_FPS={len(records) / elapsed:.2f}", flush=True)
    print(f"JPEG_BYTES=min:{jpeg.min()} mean:{jpeg.mean():.0f} max:{jpeg.max()}", flush=True)
    print(f"JPEG_DECODE_MS=mean:{decode.mean():.1f} max:{decode.max():.1f}", flush=True)
    print(f"DEPTH_INFER_MS=min:{infer.min():.1f} mean:{infer.mean():.1f} max:{infer.max():.1f}", flush=True)
    print(f"DEPTH_INFER_FPS={1000 / infer.mean():.1f}", flush=True)
    print(f"IMAGE_SHAPE={records[0]['image_shape']} DEPTH_SHAPE={records[0]['depth_shape']}", flush=True)
    print(f"ALL_DEPTH_FINITE_AND_NONCONSTANT={finite}", flush=True)


if __name__ == "__main__":
    main()
