"""Real-frame benchmark for official MoGe-2 ViT-B Normal.

Uses the existing TCP video protocol and DHCP ESP32-P4 discovery.  The optional
horizontal FOV derives from the project's current *provisional* OV5647 fx;
replace it with chessboard-calibrated values before treating geometry as truth.
"""
import math
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
from moge.model.v2 import MoGeModel

MODEL_ID = "Ruicheng/moge-2-vitb-normal"
PORT = 5000
RESULTS = os.path.join(ROOT, "moge_results")
# Provisional lens value from project notes. Replace after chessboard calibration.
PROVISIONAL_FX_PX = 1333.0
IMAGE_WIDTH_PX = 800.0
FOV_X_DEG = math.degrees(2.0 * math.atan(IMAGE_WIDTH_PX / (2.0 * PROVISIONAL_FX_PX)))


def receive_jpeg(sock):
    buffer = b""
    while True:
        packet = sock.recv(65536)
        if not packet:
            raise ConnectionError("ESP32 closed TCP stream")
        buffer += packet
        while len(buffer) >= 4:
            size = struct.unpack("<I", buffer[:4])[0]
            if not 1000 <= size <= 200000:
                buffer = buffer[1:]
                continue
            if len(buffer) < 4 + size:
                break
            return buffer[4:4 + size]


def depth_visualization(depth_m, valid_mask):
    valid = valid_mask & np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        raise RuntimeError("MoGe produced no valid positive metric depth")
    near_m, far_m = np.percentile(depth_m[valid], [2, 98]).astype(float)
    normalized = np.clip((depth_m - near_m) / max(far_m - near_m, 1e-6), 0.0, 1.0)
    vis = cv2.applyColorMap(((1.0 - normalized) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    vis[~valid] = (0, 0, 0)
    return vis, near_m, far_m


def normal_visualization(normal, valid_mask):
    # OpenCV camera-coordinate normal [-1,1] -> display RGB-ish BGR encoding.
    encoded = np.clip((normal + 1.0) * 127.5, 0, 255).astype(np.uint8)
    encoded = cv2.cvtColor(encoded, cv2.COLOR_RGB2BGR)
    encoded[~valid_mask] = (0, 0, 0)
    return encoded


def main():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required")
    ip = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
    print("GPU", torch.cuda.get_device_name(0), flush=True)
    print("MODEL", MODEL_ID, flush=True)
    print("FOV_X_DEG_PROVISIONAL", f"{FOV_X_DEG:.3f}", flush=True)
    print("MODEL_LOADING", flush=True)
    model = MoGeModel.from_pretrained(MODEL_ID).to("cuda").eval().half()

    dummy = torch.zeros((3, 800, 800), dtype=torch.float32, device="cuda")
    _ = model.infer(dummy, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
    torch.cuda.synchronize()
    print("MODEL_WARMED", flush=True)

    print(f"CONNECT {ip}:{PORT}", flush=True)
    with socket.create_connection((ip, PORT), timeout=10) as sock:
        sock.settimeout(15)
        jpeg = receive_jpeg(sock)
    gray = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise RuntimeError("JPEG decode failed")
    rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    image = torch.as_tensor(rgb, dtype=torch.float32, device="cuda").permute(2, 0, 1) / 255.0

    torch.cuda.reset_peak_memory_stats()
    torch.cuda.synchronize()
    start = time.perf_counter()
    output = model.infer(image, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
    torch.cuda.synchronize()
    elapsed_ms = (time.perf_counter() - start) * 1000.0

    depth_m = output["depth"].float().cpu().numpy()
    mask = output["mask"].bool().cpu().numpy()
    normal = output["normal"].float().cpu().numpy()
    intrinsics_normalized = output["intrinsics"].float().cpu().numpy()
    points = output["points"].float().cpu().numpy()
    depth_vis, near_m, far_m = depth_visualization(depth_m, mask)
    normal_vis = normal_visualization(normal, mask)

    h, w = depth_m.shape
    center_valid = bool(mask[h // 2, w // 2] and np.isfinite(depth_m[h // 2, w // 2]))
    valid_depth = depth_m[mask & np.isfinite(depth_m) & (depth_m > 0)]
    os.makedirs(RESULTS, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(RESULTS, f"{stamp}_ov5647_moge2_vitb_normal")
    cv2.imwrite(prefix + "_source.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(prefix + "_metric_depth_vis.png", depth_vis)
    cv2.imwrite(prefix + "_normal_vis.png", normal_vis)
    cv2.imwrite(prefix + "_mask.png", (mask.astype(np.uint8) * 255))
    np.save(prefix + "_depth_m.npy", depth_m)
    np.save(prefix + "_normal.npy", normal)
    np.save(prefix + "_mask.npy", mask)
    np.save(prefix + "_intrinsics_normalized.npy", intrinsics_normalized)
    np.save(prefix + "_points_m.npy", points)

    print("RESULT", flush=True)
    print("JPEG_BYTES", len(jpeg), flush=True)
    print("IMAGE_SHAPE", gray.shape, "DEPTH_SHAPE", depth_m.shape, flush=True)
    print("INFERENCE_MS", f"{elapsed_ms:.1f}", "FPS", f"{1000.0 / elapsed_ms:.2f}", flush=True)
    print("VRAM_MB", f"{torch.cuda.max_memory_allocated() / 1024 / 1024:.1f}", flush=True)
    print("MASK_COVERAGE", f"{mask.mean() * 100.0:.2f}%", flush=True)
    print("DEPTH_M_P02_MEDIAN_P98", f"{near_m:.3f}", f"{np.median(valid_depth):.3f}", f"{far_m:.3f}", flush=True)
    print("CENTER_VALID", center_valid, "CENTER_DEPTH_M", f"{depth_m[h // 2, w // 2]:.3f}", flush=True)
    print("INTRINSICS_NORMALIZED", np.array2string(intrinsics_normalized, precision=5), flush=True)
    print("SAVED_PREFIX", prefix, flush=True)


if __name__ == "__main__":
    main()
