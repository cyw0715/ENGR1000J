"""Real-time three-panel viewer for official DA-V2 Metric Hypersim Base.

Panels:
  left   camera image from ESP32 TCP 5000
  middle metric depth in meters (near = yellow/white, far = purple/black)
  right  metric-depth overlay

Run with the existing Windows CUDA environment:
  ..\\.venv_win\\Scripts\\python.exe metric_depth_viewer.py  # DHCP auto-discovery
  ..\\.venv_win\\Scripts\\python.exe metric_depth_viewer.py <ESP_IP>  # explicit override

Keys: q/ESC quit, s save, 1 camera, 2 metric depth, 3 overlay, v three panels.
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
METRIC_SOURCE = os.path.join(ROOT, "Depth-Anything-V2", "metric_depth")
CHECKPOINT = os.path.join(ROOT, "depth_anything_v2_metric_hypersim_vitb.pth")
SAVE_DIR = os.path.join(ROOT, "viewer_saves")
PORT = 5000
WINDOW = "Mars Lander - Metric Depth (m)"
INPUT_SIZE = 518
DISPLAY_HEIGHT = 560

sys.path.insert(0, METRIC_SOURCE)
from depth_anything_v2.dpt import DepthAnythingV2  # noqa: E402

MODEL_CONFIG = {
    "encoder": "vitb",
    "features": 128,
    "out_channels": [96, 192, 384, 768],
    "max_depth": 20,
}


def load_model():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    if not os.path.isfile(CHECKPOINT):
        raise FileNotFoundError(CHECKPOINT)

    print("GPU:", torch.cuda.get_device_name(0), flush=True)
    print("Loading official DA-V2 Metric Hypersim Base...", flush=True)
    model = DepthAnythingV2(**MODEL_CONFIG)
    state = torch.load(CHECKPOINT, map_location="cpu", weights_only=True)
    model.load_state_dict(state, strict=True)
    model = model.to("cuda").eval()
    with torch.inference_mode():
        _ = model.infer_image(np.zeros((800, 800, 3), dtype=np.uint8), INPUT_SIZE)
    torch.cuda.synchronize()
    print("Metric model warmed.", flush=True)
    return model


def connect_stream(ip):
    """Connect with a short receive timeout so GUI events remain responsive."""
    print(f"Connecting to {ip}:{PORT}", flush=True)
    sock = socket.create_connection((ip, PORT), timeout=10)
    sock.settimeout(1.0)
    print("TCP connected.", flush=True)
    return sock


def recv_jpeg(buffer, sock):
    try:
        packet = sock.recv(65536)
    except socket.timeout:
        # A Windows-hotspot/C6 stall is transient. Keep the GPU model and GUI
        # alive; the caller can poll keys and reconnect only on a real close.
        return buffer, None
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
        return buffer[4 + size:], buffer[4:4 + size]
    return buffer, None


def add_title(image, text):
    cv2.rectangle(image, (0, 0), (image.shape[1], 32), (18, 18, 18), -1)
    cv2.putText(image, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)


def draw_crosshair(image):
    h, w = image.shape[:2]
    cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 22, 1, cv2.LINE_AA)


def metric_heatmap(depth_m, output_size):
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not np.any(valid):
        raise RuntimeError("Model returned no finite positive metric depth")
    near_m = float(np.percentile(depth_m[valid], 2))
    far_m = float(np.percentile(depth_m[valid], 98))
    scaled = np.clip((depth_m - near_m) / max(far_m - near_m, 1e-6), 0.0, 1.0)
    # Inverted: warm colors indicate closer metric range.
    heat = cv2.applyColorMap(((1.0 - scaled) * 255.0).astype(np.uint8), cv2.COLORMAP_INFERNO)
    heat = cv2.resize(heat, output_size, interpolation=cv2.INTER_LINEAR)
    return heat, near_m, far_m


def draw_meter_colorbar(panel, near_m, far_m):
    h, w = panel.shape[:2]
    x0, width = w - 28, 17
    y0, y1 = 45, h - 47
    # Top is near/warm; bottom is far/dark, matching metric_heatmap.
    ramp = np.linspace(255, 0, y1 - y0, dtype=np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    panel[y0:y1, x0:x0 + width] = bar
    cv2.rectangle(panel, (x0, y0), (x0 + width, y1), (255, 255, 255), 1)
    cv2.putText(panel, f"{near_m:.2f}m", (7, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{far_m:.2f}m", (7, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, "near", (x0 - 39, y0 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, "far", (x0 - 29, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 255), 1, cv2.LINE_AA)


def make_canvas(gray, depth_m, frame_id, tcp_fps, infer_ms, jpeg_bytes, mode):
    h, w = gray.shape
    panel_w = int(w * DISPLAY_HEIGHT / h)
    source = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (panel_w, DISPLAY_HEIGHT), interpolation=cv2.INTER_LINEAR)
    heat, near_m, far_m = metric_heatmap(depth_m, (panel_w, DISPLAY_HEIGHT))
    overlay = cv2.addWeighted(source, 0.52, heat, 0.48, 0.0)

    for image, title in ((source, "Camera"), (heat, "Metric depth (m)"), (overlay, "Metric depth overlay")):
        add_title(image, title)
        draw_crosshair(image)
    draw_meter_colorbar(heat, near_m, far_m)
    draw_meter_colorbar(overlay, near_m, far_m)

    if mode == 1:
        view = source
    elif mode == 2:
        view = heat
    elif mode == 3:
        view = overlay
    else:
        separator = np.full((DISPLAY_HEIGHT, 4, 3), 40, dtype=np.uint8)
        view = np.hstack((source, separator, heat, separator, overlay))

    cy, cx = depth_m.shape[0] // 2, depth_m.shape[1] // 2
    center_m = float(depth_m[cy, cx])
    footer = np.full((59, view.shape[1], 3), 18, dtype=np.uint8)
    cv2.putText(footer, f"Frame {frame_id} | TCP {tcp_fps:.2f} FPS | Metric inference {1000.0 / max(infer_ms, 0.001):.1f} FPS ({infer_ms:.0f} ms) | JPEG {jpeg_bytes / 1024:.1f} KiB", (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(footer, f"Center candidate depth: {center_m:.3f} m | model estimate only: not control truth | 1/2/3/v view  s save  q quit", (12, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 220, 220), 1, cv2.LINE_AA)
    return np.vstack((view, footer)), source, heat, overlay, near_m, far_m


def save_snapshot(source, heat, overlay, depth_m, frame_id):
    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(SAVE_DIR, f"{stamp}_frame{frame_id:06d}")
    cv2.imwrite(prefix + "_source.png", source)
    cv2.imwrite(prefix + "_metric_depth.png", heat)
    cv2.imwrite(prefix + "_overlay.png", overlay)
    np.save(prefix + "_metric_depth_m.npy", depth_m)
    print("Saved", prefix, flush=True)


def main():
    model = load_model()
    ip = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
    sock = connect_stream(ip)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1500, 620)
    print("Viewer started. Keys: 1/2/3/v, s, q/ESC", flush=True)

    buffer = b""
    frame_id = 0
    mode = 0
    smoothed_tcp_fps = 0.0
    previous_time = time.monotonic()
    last_reconnect_attempt = 0.0
    try:
        while True:
            try:
                buffer, jpeg = recv_jpeg(buffer, sock)
            except ConnectionError as exc:
                try:
                    sock.close()
                except OSError:
                    pass
                now = time.monotonic()
                if now - last_reconnect_attempt >= 2.0:
                    last_reconnect_attempt = now
                    print(f"TCP closed ({exc}); retrying without reloading model...", flush=True)
                    try:
                        sock = connect_stream(ip)
                        buffer = b""
                    except OSError as reconnect_error:
                        print(f"Reconnect pending: {reconnect_error}", flush=True)
                cv2.waitKey(1)
                continue
            if jpeg is None:
                # Timeout means no payload this second, not a failed model.
                # Keep the last display responsive and permit q/Esc exit.
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue
            gray = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue

            now = time.monotonic()
            interval = now - previous_time
            previous_time = now
            instant_fps = 1.0 / max(interval, 1e-6)
            smoothed_tcp_fps = instant_fps if frame_id == 0 else 0.8 * smoothed_tcp_fps + 0.2 * instant_fps

            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            torch.cuda.synchronize()
            start = time.perf_counter()
            with torch.inference_mode():
                depth_m = model.infer_image(bgr, INPUT_SIZE).astype(np.float32)
            torch.cuda.synchronize()
            infer_ms = (time.perf_counter() - start) * 1000.0
            frame_id += 1

            canvas, source, heat, overlay, near_m, far_m = make_canvas(gray, depth_m, frame_id, smoothed_tcp_fps, infer_ms, len(jpeg), mode)
            cv2.imshow(WINDOW, canvas)
            if frame_id % 5 == 0:
                print(f"FRAME {frame_id}: infer={infer_ms:.1f}ms center={depth_m[depth_m.shape[0]//2, depth_m.shape[1]//2]:.3f}m range={near_m:.3f}-{far_m:.3f}m", flush=True)

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
            elif key == ord("s"):
                save_snapshot(source, heat, overlay, depth_m, frame_id)
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print(f"Viewer stopped after {frame_id} metric frames.", flush=True)


if __name__ == "__main__":
    main()
