"""Independent real-frame benchmark for official DA-V2 Metric Hypersim Base.

Loads the official metric checkpoint, receives one complete JPEG from the
existing ESP32 TCP protocol, and saves only diagnostic artifacts under results/.
The model is loaded before TCP connect to avoid artificial sender backpressure.
"""
import os
import socket
import struct
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import torch

ROOT = os.path.dirname(os.path.abspath(__file__))
PC_VIEWER_ROOT = os.path.dirname(ROOT)
if PC_VIEWER_ROOT not in sys.path:
    sys.path.insert(0, PC_VIEWER_ROOT)
from esp_discovery import discover_esp32
SOURCE_ROOT = os.path.join(ROOT, "Depth-Anything-V2", "metric_depth")
CHECKPOINT = os.path.join(ROOT, "depth_anything_v2_metric_hypersim_vitb.pth")
RESULTS = os.path.join(ROOT, "results")
ESP_IP = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
PORT = 5000
INPUT_SIZE = 518

sys.path.insert(0, SOURCE_ROOT)
from depth_anything_v2.dpt import DepthAnythingV2  # noqa: E402

MODEL_CONFIG = {
    "encoder": "vitb",
    "features": 128,
    "out_channels": [96, 192, 384, 768],
    "max_depth": 20,
}


def receive_complete_jpeg(sock):
    buffer = b""
    while True:
        packet = sock.recv(65536)
        if not packet:
            raise ConnectionError("ESP32 closed video TCP connection")
        buffer += packet
        while len(buffer) >= 4:
            size = struct.unpack("<I", buffer[:4])[0]
            if not 1000 <= size <= 200000:
                buffer = buffer[1:]
                continue
            if len(buffer) < 4 + size:
                break
            jpeg = buffer[4:4 + size]
            return jpeg


def make_metric_visualization(depth_m):
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not np.any(valid):
        raise RuntimeError("Metric model returned no finite positive depth")
    near_m = float(np.percentile(depth_m[valid], 2))
    far_m = float(np.percentile(depth_m[valid], 98))
    # Invert display mapping: near is warm/yellow, farther is dark/purple.
    normalized = np.clip((depth_m - near_m) / max(far_m - near_m, 1e-6), 0, 1)
    image = cv2.applyColorMap(((1.0 - normalized) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    return image, near_m, far_m


def main():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for this benchmark")
    if not os.path.isfile(CHECKPOINT):
        raise FileNotFoundError(CHECKPOINT)

    print("GPU", torch.cuda.get_device_name(0), flush=True)
    print("MODEL", "Depth-Anything-V2-Metric-Hypersim-Base", flush=True)
    print("CHECKPOINT", CHECKPOINT, flush=True)
    print("MODEL_LOADING", flush=True)
    model = DepthAnythingV2(**MODEL_CONFIG)
    checkpoint = torch.load(CHECKPOINT, map_location="cpu", weights_only=True)
    model.load_state_dict(checkpoint, strict=True)
    model = model.to("cuda").eval()

    # Warmup uses an 800x800 BGR image, matching the actual stream geometry.
    dummy = np.zeros((800, 800, 3), dtype=np.uint8)
    with torch.inference_mode():
        _ = model.infer_image(dummy, INPUT_SIZE)
    torch.cuda.synchronize()
    print("MODEL_WARMED", flush=True)

    print(f"CONNECT {ESP_IP}:{PORT}", flush=True)
    with socket.create_connection((ESP_IP, PORT), timeout=10) as sock:
        sock.settimeout(15)
        jpeg = receive_complete_jpeg(sock)
    print("JPEG_RECEIVED", len(jpeg), flush=True)

    gray = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise RuntimeError("Received JPEG failed OpenCV decode")
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    torch.cuda.reset_peak_memory_stats()
    torch.cuda.synchronize()
    start = time.perf_counter()
    with torch.inference_mode():
        depth_m = model.infer_image(bgr, INPUT_SIZE)
    torch.cuda.synchronize()
    inference_ms = (time.perf_counter() - start) * 1000.0
    depth_m = depth_m.astype(np.float32)

    visualization, near_m, far_m = make_metric_visualization(depth_m)
    os.makedirs(RESULTS, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(RESULTS, f"{stamp}_ov5647_metric_hypersim_base")
    cv2.imwrite(prefix + "_source.png", bgr)
    cv2.imwrite(prefix + "_metric_depth_vis.png", visualization)
    np.save(prefix + "_metric_depth_m.npy", depth_m)

    h, w = depth_m.shape
    cy, cx = h // 2, w // 2
    valid = np.isfinite(depth_m) & (depth_m > 0)
    print("RESULT", flush=True)
    print("IMAGE_SHAPE", gray.shape, "DEPTH_SHAPE", depth_m.shape, flush=True)
    print("INFERENCE_MS", f"{inference_ms:.1f}", "FPS", f"{1000.0 / inference_ms:.2f}", flush=True)
    print("VRAM_MB", f"{torch.cuda.max_memory_allocated() / 1024 / 1024:.1f}", flush=True)
    print("DEPTH_FINITE_POSITIVE", bool(np.all(valid)), flush=True)
    print("DEPTH_M_MIN_MEDIAN_MAX", f"{float(depth_m[valid].min()):.3f}", f"{float(np.median(depth_m[valid])):.3f}", f"{float(depth_m[valid].max()):.3f}", flush=True)
    print("DEPTH_M_P02_P98", f"{near_m:.3f}", f"{far_m:.3f}", flush=True)
    print("CENTER_DEPTH_M", f"{float(depth_m[cy, cx]):.3f}", flush=True)
    print("SAVED_PREFIX", prefix, flush=True)


if __name__ == "__main__":
    main()
