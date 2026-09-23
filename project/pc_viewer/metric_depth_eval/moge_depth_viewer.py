"""Real-time three-panel MoGe-2 viewer for the Mars Lander.

Default panels:
  left   OV5647 camera image
  middle MoGe-2 metric depth in meters
  right  MoGe-2 surface-normal map

Modes: 1=camera, 2=depth, 3=normal, 4=depth overlay, v=three panels.
The ESP32-P4 is found by DHCP-safe HTTP /board discovery when no IP is given.
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
WINDOW = "Mars Lander - MoGe-2 Metric Geometry"
SAVE_DIR = os.path.join(ROOT, "moge_viewer_saves")
DISPLAY_HEIGHT = 560
# Current provisional camera calibration. Replace after a chessboard calibration.
PROVISIONAL_FX_PX = 1333.0
IMAGE_WIDTH_PX = 800.0
FOV_X_DEG = math.degrees(2.0 * math.atan(IMAGE_WIDTH_PX / (2.0 * PROVISIONAL_FX_PX)))


def load_model():
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    print("GPU:", torch.cuda.get_device_name(0), flush=True)
    print("Loading MoGe-2 ViT-B Normal...", flush=True)
    model = MoGeModel.from_pretrained(MODEL_ID).to("cuda").eval().half()
    dummy = torch.zeros((3, 800, 800), dtype=torch.float32, device="cuda")
    _ = model.infer(dummy, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
    torch.cuda.synchronize()
    print(f"MoGe warmed; provisional FOVx={FOV_X_DEG:.3f} deg", flush=True)
    return model


def connect_stream(ip):
    print(f"Connecting to {ip}:{PORT}", flush=True)
    sock = socket.create_connection((ip, PORT), timeout=10)
    sock.settimeout(1.0)
    print("TCP connected.", flush=True)
    return sock


def recv_jpeg(buffer, sock):
    try:
        packet = sock.recv(65536)
    except socket.timeout:
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


def make_depth_panel(depth_m, mask, output_size):
    valid = mask & np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        raise RuntimeError("MoGe returned no finite positive metric depths")
    near_m, far_m = np.percentile(depth_m[valid], [2, 98]).astype(float)
    normalized = np.clip((depth_m - near_m) / max(far_m - near_m, 1e-6), 0.0, 1.0)
    panel = cv2.applyColorMap(((1.0 - normalized) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    panel[~valid] = (0, 0, 0)
    panel = cv2.resize(panel, output_size, interpolation=cv2.INTER_LINEAR)
    return panel, near_m, far_m


def make_normal_panel(normal, mask, output_size):
    # Camera-coordinate normal [-1, 1] encoded as RGB then converted to BGR.
    panel = np.clip((normal + 1.0) * 127.5, 0, 255).astype(np.uint8)
    panel = cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)
    panel[~mask] = (0, 0, 0)
    return cv2.resize(panel, output_size, interpolation=cv2.INTER_LINEAR)


def draw_depth_colorbar(panel, near_m, far_m):
    h, w = panel.shape[:2]
    x0, width = w - 28, 17
    y0, y1 = 45, h - 48
    ramp = np.linspace(255, 0, y1 - y0, dtype=np.uint8).reshape(-1, 1)
    panel[y0:y1, x0:x0 + width] = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    cv2.rectangle(panel, (x0, y0), (x0 + width, y1), (255, 255, 255), 1)
    cv2.putText(panel, f"{near_m:.2f}m", (7, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{far_m:.2f}m", (7, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, "near", (x0 - 39, y0 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, "far", (x0 - 29, y1 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 255, 255), 1, cv2.LINE_AA)


def make_canvas(gray, depth_m, normal, mask, frame_id, tcp_fps, infer_ms, jpeg_bytes, mode):
    height, width = gray.shape
    panel_w = int(width * DISPLAY_HEIGHT / height)
    source = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (panel_w, DISPLAY_HEIGHT), interpolation=cv2.INTER_LINEAR)
    depth, near_m, far_m = make_depth_panel(depth_m, mask, (panel_w, DISPLAY_HEIGHT))
    normal_panel = make_normal_panel(normal, mask, (panel_w, DISPLAY_HEIGHT))
    overlay = cv2.addWeighted(source, 0.52, depth, 0.48, 0.0)

    panels = ((source, "Camera"), (depth, "MoGe metric depth (m)"), (normal_panel, "MoGe surface normal"))
    for panel, title in panels:
        add_title(panel, title)
        draw_crosshair(panel)
    add_title(overlay, "MoGe metric-depth overlay")
    draw_crosshair(overlay)
    draw_depth_colorbar(depth, near_m, far_m)
    draw_depth_colorbar(overlay, near_m, far_m)

    if mode == 1:
        view = source
    elif mode == 2:
        view = depth
    elif mode == 3:
        view = normal_panel
    elif mode == 4:
        view = overlay
    else:
        separator = np.full((DISPLAY_HEIGHT, 4, 3), 40, dtype=np.uint8)
        view = np.hstack((source, separator, depth, separator, normal_panel))

    cy, cx = depth_m.shape[0] // 2, depth_m.shape[1] // 2
    center_m = float(depth_m[cy, cx])
    normal_center = normal[cy, cx]
    footer = np.full((60, view.shape[1], 3), 18, dtype=np.uint8)
    cv2.putText(footer, f"Frame {frame_id} | TCP {tcp_fps:.2f} FPS | MoGe {1000.0 / max(infer_ms, 0.001):.1f} FPS ({infer_ms:.0f} ms) | JPEG {jpeg_bytes / 1024:.1f} KiB", (12, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(footer, f"Center candidate: {center_m:.3f}m | normal [{normal_center[0]:+.2f}, {normal_center[1]:+.2f}, {normal_center[2]:+.2f}] | candidate only, not control truth", (12, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)
    return np.vstack((view, footer)), source, depth, normal_panel, overlay, near_m, far_m


def save_snapshot(source, depth, normal_panel, overlay, depth_m, normal, mask, frame_id):
    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(SAVE_DIR, f"{stamp}_frame{frame_id:06d}")
    cv2.imwrite(prefix + "_source.png", source)
    cv2.imwrite(prefix + "_metric_depth.png", depth)
    cv2.imwrite(prefix + "_normal.png", normal_panel)
    cv2.imwrite(prefix + "_overlay.png", overlay)
    np.save(prefix + "_depth_m.npy", depth_m)
    np.save(prefix + "_normal.npy", normal)
    np.save(prefix + "_mask.npy", mask)
    print("Saved", prefix, flush=True)


def main():
    model = load_model()
    ip = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
    sock = connect_stream(ip)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1500, 620)
    print("Viewer started. Keys: 1=camera 2=depth 3=normal 4=overlay v=three-panel s=save q/ESC=quit", flush=True)

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
            instantaneous_fps = 1.0 / max(interval, 1e-6)
            smoothed_tcp_fps = instantaneous_fps if frame_id == 0 else 0.8 * smoothed_tcp_fps + 0.2 * instantaneous_fps

            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            image = torch.as_tensor(rgb, dtype=torch.float32, device="cuda").permute(2, 0, 1) / 255.0
            torch.cuda.synchronize()
            start = time.perf_counter()
            output = model.infer(image, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
            torch.cuda.synchronize()
            infer_ms = (time.perf_counter() - start) * 1000.0
            depth_m = output["depth"].float().cpu().numpy()
            normal = output["normal"].float().cpu().numpy()
            mask = output["mask"].bool().cpu().numpy()
            frame_id += 1

            canvas, source, depth, normal_panel, overlay, near_m, far_m = make_canvas(gray, depth_m, normal, mask, frame_id, smoothed_tcp_fps, infer_ms, len(jpeg), mode)
            cv2.imshow(WINDOW, canvas)
            if frame_id % 5 == 0:
                center = depth_m[depth_m.shape[0] // 2, depth_m.shape[1] // 2]
                print(f"FRAME {frame_id}: infer={infer_ms:.1f}ms center={center:.3f}m range={near_m:.3f}-{far_m:.3f}m mask={mask.mean() * 100:.1f}%", flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("1"):
                mode = 1
            elif key == ord("2"):
                mode = 2
            elif key == ord("3"):
                mode = 3
            elif key == ord("4"):
                mode = 4
            elif key == ord("v"):
                mode = 0
            elif key == ord("s"):
                save_snapshot(source, depth, normal_panel, overlay, depth_m, normal, mask, frame_id)
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print(f"Viewer stopped after {frame_id} MoGe frames.", flush=True)


if __name__ == "__main__":
    main()
