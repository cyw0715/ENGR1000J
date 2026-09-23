"""
Mars Lander - Real-time Depth Anything V2 Base visualization.

Receives grayscale JPEG frames from ESP32 TCP 5000 and displays three panels:
  left   : camera image
  middle : relative-depth heat map (yellow/white = nearer, dark = farther)
  right  : heat map blended over the camera image

Run on Windows with the project-local AI environment:
  .\.venv_win\Scripts\python.exe .\depth_viewer.py  # DHCP auto-discovery
  .\.venv_win\Scripts\python.exe .\depth_viewer.py <ESP_IP>  # explicit override

Keys:
  q / ESC : quit
  s       : save source, depth heat map, overlay, and raw float32 depth (.npy)
  1       : camera-only view
  2       : depth-only view
  3       : overlay-only view
  v       : restore three-panel view
"""
from esp_discovery import discover_esp32
import os
import socket
import struct
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import pipeline

PC_PORT = 5000
WINDOW_NAME = "Mars Lander - Depth Anything V2"
DISPLAY_HEIGHT = 560
PERCENTILE_LOW = 2.0
PERCENTILE_HIGH = 98.0
RANGE_SMOOTHING = 0.85

# ── ESP32 discovery ────────────────────────────────────────────────────
def find_esp32():
    """Discover DHCP-assigned ESP32-P4 via its HTTP /board identity endpoint."""
    try:
        return discover_esp32()
    except ConnectionError as exc:
        print(f"Discovery failed: {exc}")
        return None


def recv_frame(buffer: bytes, sock: socket.socket):
    """Read until one valid length-prefixed JPEG frame is available."""
    data = sock.recv(65536)
    if not data:
        raise ConnectionError("ESP32 closed the TCP video connection")
    buffer += data

    while len(buffer) >= 4:
        jpeg_size = struct.unpack("<I", buffer[:4])[0]
        if not 1000 <= jpeg_size <= 200000:
            buffer = buffer[1:]
            continue
        if len(buffer) < 4 + jpeg_size:
            break
        jpeg = buffer[4:4 + jpeg_size]
        return buffer[4 + jpeg_size:], jpeg
    return buffer, None


def load_model():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable. This viewer is intended for the RTX GPU.")

    print(f"GPU: {torch.cuda.get_device_name(0)}", flush=True)
    print("Loading Depth Anything V2 Base...", flush=True)
    model = pipeline(
        "depth-estimation",
        model="depth-anything/Depth-Anything-V2-Base-hf",
        device="cuda",
        dtype=torch.float16,
    )
    # Warm before connecting TCP, preventing model loading from causing ESP TCP backpressure.
    model(Image.fromarray(np.zeros((512, 512, 3), dtype=np.uint8)))
    print("Depth model warmed.", flush=True)
    return model


def infer_depth(model, gray: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    result = model(Image.fromarray(rgb))
    return result["predicted_depth"].cpu().float().numpy()


def stable_depth_range(depth: np.ndarray, previous_range):
    current_low = float(np.percentile(depth, PERCENTILE_LOW))
    current_high = float(np.percentile(depth, PERCENTILE_HIGH))
    if current_high <= current_low:
        current_high = current_low + 1e-6

    if previous_range is None:
        return current_low, current_high

    old_low, old_high = previous_range
    low = RANGE_SMOOTHING * old_low + (1.0 - RANGE_SMOOTHING) * current_low
    high = RANGE_SMOOTHING * old_high + (1.0 - RANGE_SMOOTHING) * current_high
    return low, max(high, low + 1e-6)


def colorize_depth(depth: np.ndarray, depth_range, output_size):
    low, high = depth_range
    normalized = np.clip((depth - low) / (high - low), 0.0, 1.0)
    gray8 = (normalized * 255.0).astype(np.uint8)
    heatmap = cv2.applyColorMap(gray8, cv2.COLORMAP_INFERNO)
    return cv2.resize(heatmap, output_size, interpolation=cv2.INTER_LINEAR), normalized


def add_panel_title(image, text: str, color=(255, 255, 255)):
    cv2.rectangle(image, (0, 0), (image.shape[1], 31), (18, 18, 18), -1)
    cv2.putText(image, text, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)


def draw_crosshair(image):
    h, w = image.shape[:2]
    x, y = w // 2, h // 2
    cv2.drawMarker(image, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 22, 1, cv2.LINE_AA)
    cv2.circle(image, (x, y), 4, (0, 0, 0), 1, cv2.LINE_AA)


def draw_colorbar(panel, low: float, high: float):
    h, w = panel.shape[:2]
    bar_width = 17
    x0 = w - 26
    y0, y1 = 46, h - 48
    ramp = np.linspace(255, 0, y1 - y0, dtype=np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    panel[y0:y1, x0:x0 + bar_width] = bar
    cv2.rectangle(panel, (x0, y0), (x0 + bar_width, y1), (255, 255, 255), 1)
    cv2.putText(panel, "near", (x0 - 38, y0 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, "far", (x0 - 30, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{high:.2f}", (8, y0 + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{low:.2f}", (8, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (230, 230, 230), 1, cv2.LINE_AA)


def compose_display(gray, depth, depth_range, frame_number, capture_fps, infer_ms, jpeg_size, mode):
    h, w = gray.shape
    display_w = int(w * DISPLAY_HEIGHT / h)
    source = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (display_w, DISPLAY_HEIGHT), interpolation=cv2.INTER_LINEAR)
    heatmap, normalized = colorize_depth(depth, depth_range, (display_w, DISPLAY_HEIGHT))
    overlay = cv2.addWeighted(source, 0.52, heatmap, 0.48, 0.0)

    center_depth = float(depth[depth.shape[0] // 2, depth.shape[1] // 2])
    for panel, title in ((source, "Camera"), (heatmap, "Relative depth"), (overlay, "Depth overlay")):
        add_panel_title(panel, title)
        draw_crosshair(panel)

    draw_colorbar(heatmap, depth_range[0], depth_range[1])
    draw_colorbar(overlay, depth_range[0], depth_range[1])

    if mode == 1:
        canvas = source
    elif mode == 2:
        canvas = heatmap
    elif mode == 3:
        canvas = overlay
    else:
        separator = np.full((DISPLAY_HEIGHT, 4, 3), 40, dtype=np.uint8)
        canvas = np.hstack((source, separator, heatmap, separator, overlay))

    footer_height = 58
    footer = np.full((footer_height, canvas.shape[1], 3), 18, dtype=np.uint8)
    cv2.putText(
        footer,
        f"Frame {frame_number} | TCP {capture_fps:.2f} FPS | Depth {1000.0 / max(infer_ms, 0.001):.1f} FPS ({infer_ms:.0f} ms) | JPEG {jpeg_size / 1024:.1f} KiB",
        (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 245, 245), 1, cv2.LINE_AA,
    )
    cv2.putText(
        footer,
        f"Center relative depth: {center_depth:.3f} | yellow/white = relatively nearer, dark = relatively farther | 1/2/3/v view  s save  q quit",
        (12, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA,
    )
    return np.vstack((canvas, footer)), source, heatmap, overlay


def save_snapshot(save_dir, source, heatmap, overlay, depth, frame_number):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(save_dir, f"{timestamp}_frame{frame_number:06d}")
    cv2.imwrite(prefix + "_source.png", source)
    cv2.imwrite(prefix + "_relative_depth.png", heatmap)
    cv2.imwrite(prefix + "_overlay.png", overlay)
    np.save(prefix + "_relative_depth.npy", depth)
    print(f"Saved {prefix}_{{source,relative_depth,overlay}}.png and .npy", flush=True)


def main():
    # Model first: avoid artificial network backpressure during startup.
    model = load_model()

    esp_ip = sys.argv[1] if len(sys.argv) > 1 else find_esp32()
    if not esp_ip:
        raise RuntimeError("ESP32 TCP video stream was not found")

    print(f"Connecting to {esp_ip}:{PC_PORT}...", flush=True)
    sock = socket.create_connection((esp_ip, PC_PORT), timeout=10)
    sock.settimeout(10)
    print("TCP connected. Starting visualizer.", flush=True)

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1500, 620)

    save_dir = os.path.join(os.path.dirname(__file__), "depth_saves")
    os.makedirs(save_dir, exist_ok=True)

    buffer = b""
    frame_number = 0
    mode = 0  # 0=three panels, 1=camera, 2=depth, 3=overlay
    previous_range = None
    capture_fps = 0.0
    last_frame_time = time.monotonic()
    last_snapshot = None

    print("Keys: 1=camera 2=depth 3=overlay v=three-panel s=save q/ESC=quit", flush=True)
    try:
        while True:
            buffer, jpeg = recv_frame(buffer, sock)
            if jpeg is None:
                continue

            gray = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print("Discarded malformed JPEG", flush=True)
                continue

            now = time.monotonic()
            dt = now - last_frame_time
            last_frame_time = now
            if dt > 0:
                instantaneous_fps = 1.0 / dt
                capture_fps = instantaneous_fps if frame_number == 0 else 0.8 * capture_fps + 0.2 * instantaneous_fps

            infer_start = time.monotonic()
            depth = infer_depth(model, gray)
            infer_ms = (time.monotonic() - infer_start) * 1000.0
            previous_range = stable_depth_range(depth, previous_range)
            frame_number += 1

            canvas, source, heatmap, overlay = compose_display(
                gray, depth, previous_range, frame_number, capture_fps, infer_ms, len(jpeg), mode
            )
            last_snapshot = (source, heatmap, overlay, depth.copy())
            cv2.imshow(WINDOW_NAME, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("1"):
                mode = 1
            elif key == ord("2"):
                mode = 2
            elif key == ord("3"):
                mode = 3
            elif key == ord("v"):
                mode = 0
            elif key == ord("s") and last_snapshot is not None:
                save_snapshot(save_dir, *last_snapshot, frame_number)
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print(f"Viewer stopped after {frame_number} depth frames.", flush=True)


if __name__ == "__main__":
    main()
