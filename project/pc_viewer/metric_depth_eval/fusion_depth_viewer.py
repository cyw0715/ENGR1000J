"""Four-panel, reliability-weighted fusion viewer: DA-V2 Metric + MoGe-2.

Panels (2 x 2):
  top-left     camera image
  top-right    DA-V2 Metric Hypersim Base depth overlay + meter legend
  bottom-left  MoGe-2 ViT-B Normal depth overlay + meter legend
  bottom-right reliability-weighted fusion overlay + meter legend

Important: neither model exposes an absolute calibrated per-pixel confidence for
this OV5647 grayscale domain. "Reliability" below is an explicit heuristic:
validity mask × local image texture × temporal stability × model agreement.
It is for rejecting/attenuating unreliable pixels, NOT an uncertainty guarantee.

The two metric outputs may have different scale bias. Before averaging, DA depth
is robustly scale-aligned to the MoGe median over jointly valid pixels. This
makes a coherent fused visualization, but does NOT turn the result into a
calibrated absolute sensor. Raw DA/MoGe maps remain unmodified in saved output.
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

MOGE_SOURCE = os.path.join(ROOT, "MoGe")
DA_SOURCE = os.path.join(ROOT, "Depth-Anything-V2", "metric_depth")
if MOGE_SOURCE not in sys.path:
    sys.path.insert(0, MOGE_SOURCE)
if DA_SOURCE not in sys.path:
    sys.path.insert(0, DA_SOURCE)
from landing_projection import project_lander_plan

PORT = 5000
WINDOW = "Mars Lander - DA + MoGe Reliability Fusion"
SAVE_DIR = os.path.join(ROOT, "fusion_viewer_saves")
PANEL_W = 600
PANEL_H = 430
FOOTER_H = 70

MOGE_MODEL_ID = "Ruicheng/moge-2-vitb-normal"
DA_CHECKPOINT = os.path.join(ROOT, "depth_anything_v2_metric_hypersim_vitb.pth")
DA_CONFIG = {
    "encoder": "vitb",
    "features": 128,
    "out_channels": [96, 192, 384, 768],
    "max_depth": 20,
}
DA_INPUT_SIZE = 518
# Provisional OV5647 value; must be replaced after chessboard calibration.
PROVISIONAL_FX_PX = 1333.0
IMAGE_WIDTH_PX = 800.0
FOV_X_DEG = math.degrees(2.0 * math.atan(IMAGE_WIDTH_PX / (2.0 * PROVISIONAL_FX_PX)))


def load_models():
    # Keep heavyweight optional model imports here so geometry/overlay tests can
    # run without loading the MoGe dependency tree.
    from moge.model.v2 import MoGeModel
    from depth_anything_v2.dpt import DepthAnythingV2

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    if not os.path.isfile(DA_CHECKPOINT):
        raise FileNotFoundError(DA_CHECKPOINT)
    print("GPU:", torch.cuda.get_device_name(0), flush=True)
    print("Loading MoGe-2 ViT-B Normal...", flush=True)
    moge = MoGeModel.from_pretrained(MOGE_MODEL_ID).to("cuda").eval().half()
    print("Loading DA-V2 Metric Hypersim Base...", flush=True)
    da = DepthAnythingV2(**DA_CONFIG)
    da.load_state_dict(torch.load(DA_CHECKPOINT, map_location="cpu", weights_only=True), strict=True)
    da = da.to("cuda").eval()

    dummy_bgr = np.zeros((800, 800, 3), dtype=np.uint8)
    dummy_moge = torch.zeros((3, 800, 800), dtype=torch.float32, device="cuda")
    _ = moge.infer(dummy_moge, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
    with torch.inference_mode():
        _ = da.infer_image(dummy_bgr, DA_INPUT_SIZE)
    torch.cuda.synchronize()
    print(f"Models warmed; provisional FOVx={FOV_X_DEG:.3f} deg", flush=True)
    return moge, da


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


def add_title(image, text, color=(255, 255, 255)):
    cv2.rectangle(image, (0, 0), (image.shape[1], 31), (18, 18, 18), -1)
    cv2.putText(image, text, (9, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 2, cv2.LINE_AA)


def draw_crosshair(image):
    h, w = image.shape[:2]
    cv2.drawMarker(image, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)


def draw_lander_overlay(panel, center_depth_m):
    """Draw a display-only platform/foot projection on the fusion panel.

    The user-selected provisional model is a camera centered on the horizontal
    platform, looking vertically down. It intentionally does not feed control.
    """
    plan = project_lander_plan(
        center_ground_depth_m=center_depth_m,
        image_shape=panel.shape[:2],
        fx_px_at_800=PROVISIONAL_FX_PX,
    )
    if plan is None:
        cv2.putText(panel, "Lander overlay unavailable: invalid center depth", (10, panel.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (60, 60, 255), 1, cv2.LINE_AA)
        return None

    platform = np.rint(plan["platform_corners_px"]).astype(np.int32)
    feet = np.rint(plan["foot_points_px"]).astype(np.int32)
    labels = ("FL", "FR", "RR", "RL")

    # Amber square = 300 mm platform; cyan radii = the four nominal actuator axes.
    cv2.polylines(panel, [platform], True, (0, 215, 255), 2, cv2.LINE_AA)
    for corner, foot, label in zip(platform, feet, labels):
        cv2.line(panel, tuple(corner), tuple(foot), (255, 255, 0), 2, cv2.LINE_AA)
        cv2.circle(panel, tuple(corner), 4, (0, 215, 255), -1, cv2.LINE_AA)
        cv2.drawMarker(panel, tuple(foot), (255, 255, 0), cv2.MARKER_TILTED_CROSS, 12, 2, cv2.LINE_AA)
        text_origin = (int(foot[0] + 6), int(foot[1] - 6))
        cv2.putText(panel, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA)

    cv2.putText(panel, "PROVISIONAL: centered downward camera | amber=platform cyan=feet", (10, panel.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)
    return plan


def robust_scale_align(source_m, reference_m, valid):
    """Return source scaled to reference median; no shift is introduced."""
    if np.count_nonzero(valid) < 100:
        return source_m.copy(), 1.0
    src_med = float(np.median(source_m[valid]))
    ref_med = float(np.median(reference_m[valid]))
    if not np.isfinite(src_med) or src_med <= 1e-6 or not np.isfinite(ref_med):
        return source_m.copy(), 1.0
    scale = float(np.clip(ref_med / src_med, 0.25, 4.0))
    return source_m * scale, scale


def texture_reliability(gray):
    """Texture support [0,1], low for featureless pixels where monocular depth is weak."""
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    grad = cv2.magnitude(gx, gy)
    local = cv2.GaussianBlur(grad, (0, 0), 5.0)
    p95 = float(np.percentile(local, 95))
    if p95 <= 1e-6:
        return np.zeros_like(local, dtype=np.float32)
    # Keep some nonzero support in textured-but-low-contrast OV5647 frames.
    return np.clip(local / p95, 0.0, 1.0).astype(np.float32)


def temporal_reliability(current, previous, valid, relative_tolerance=0.10):
    if previous is None or previous.shape != current.shape:
        return np.ones_like(current, dtype=np.float32)
    denom = np.maximum(np.maximum(np.abs(current), np.abs(previous)), 0.10)
    relative_change = np.abs(current - previous) / denom
    score = np.exp(-relative_change / relative_tolerance).astype(np.float32)
    score[~valid] = 0.0
    return score


def fusion_reliability(gray, da_aligned, moge_depth, da_valid, moge_valid, prev_da, prev_moge):
    joint = da_valid & moge_valid
    texture = texture_reliability(gray)
    # Agreement only measures consistency *after* median scale alignment.
    denom = np.maximum(np.maximum(np.abs(da_aligned), np.abs(moge_depth)), 0.10)
    disagreement = np.abs(da_aligned - moge_depth) / denom
    agreement = np.exp(-disagreement / 0.15).astype(np.float32)
    da_temporal = temporal_reliability(da_aligned, prev_da, da_valid)
    moge_temporal = temporal_reliability(moge_depth, prev_moge, moge_valid)

    # The same texture/agreement gates both models; remaining difference comes
    # from each map's temporal stability and MoGe's actual valid-pixel mask.
    shared = 0.20 + 0.80 * texture
    da_weight = shared * agreement * da_temporal * da_valid.astype(np.float32)
    moge_weight = shared * agreement * moge_temporal * moge_valid.astype(np.float32)
    da_weight[~joint] = 0.0
    moge_weight[~joint] = 0.0
    return da_weight, moge_weight, texture, agreement, da_temporal, moge_temporal


def metric_panel(depth_m, valid, output_size, title, source=None):
    valid = valid & np.isfinite(depth_m) & (depth_m > 0)
    if not valid.any():
        raise RuntimeError(f"No valid metric depths for {title}")
    near_m, far_m = np.percentile(depth_m[valid], [2, 98]).astype(float)
    normalized = np.clip((depth_m - near_m) / max(far_m - near_m, 1e-6), 0.0, 1.0)
    heat = cv2.applyColorMap(((1.0 - normalized) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
    heat[~valid] = (0, 0, 0)
    heat = cv2.resize(heat, output_size, interpolation=cv2.INTER_LINEAR)
    if source is not None:
        panel = cv2.addWeighted(source, 0.50, heat, 0.50, 0.0)
    else:
        panel = heat
    add_title(panel, title)
    draw_crosshair(panel)
    draw_colorbar(panel, near_m, far_m)
    return panel, near_m, far_m


def draw_colorbar(panel, near_m, far_m):
    h, w = panel.shape[:2]
    x0, width = w - 27, 16
    y0, y1 = 43, h - 44
    ramp = np.linspace(255, 0, y1 - y0, dtype=np.uint8).reshape(-1, 1)
    panel[y0:y1, x0:x0 + width] = cv2.applyColorMap(ramp, cv2.COLORMAP_INFERNO)
    cv2.rectangle(panel, (x0, y0), (x0 + width, y1), (255, 255, 255), 1)
    cv2.putText(panel, f"{near_m:.2f}m", (7, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{far_m:.2f}m", (7, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (245, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(panel, "near", (x0 - 37, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, "far", (x0 - 27, y1 + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)


def reliability_thumbnail(texture, agreement, da_weight, moge_weight):
    # Debug diagnostic, shown only when the user selects mode 'r'.
    combined = np.stack((moge_weight, agreement, texture), axis=-1)
    combined = np.clip(combined * 255.0, 0, 255).astype(np.uint8)
    return combined


def make_canvas(gray, da_raw, moge_raw, moge_mask, frame_id, tcp_fps, da_ms, moge_ms, jpeg_bytes, previous):
    source = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (PANEL_W, PANEL_H), interpolation=cv2.INTER_LINEAR)
    da_valid = np.isfinite(da_raw) & (da_raw > 0)
    moge_valid = moge_mask & np.isfinite(moge_raw) & (moge_raw > 0)
    joint = da_valid & moge_valid
    da_aligned, scale = robust_scale_align(da_raw, moge_raw, joint)

    prev_da, prev_moge = previous
    da_w, moge_w, texture, agreement, da_temporal, moge_temporal = fusion_reliability(
        gray, da_aligned, moge_raw, da_valid, moge_valid, prev_da, prev_moge
    )
    total_w = da_w + moge_w
    fused_valid = total_w > 1e-4
    fused = np.full_like(moge_raw, np.nan, dtype=np.float32)
    fused[fused_valid] = (da_w[fused_valid] * da_aligned[fused_valid] + moge_w[fused_valid] * moge_raw[fused_valid]) / total_w[fused_valid]

    camera = source.copy()
    add_title(camera, "Camera (OV5647)")
    draw_crosshair(camera)
    da_panel, da_near, da_far = metric_panel(da_aligned, joint, (PANEL_W, PANEL_H), "DA Metric (scale-aligned) + legend", source)
    moge_panel, moge_near, moge_far = metric_panel(moge_raw, moge_valid, (PANEL_W, PANEL_H), "MoGe Metric + legend", source)
    fusion_panel, fusion_near, fusion_far = metric_panel(fused, fused_valid, (PANEL_W, PANEL_H), "Reliability-weighted fusion + lander overlay", source)
    cy, cx = fused.shape[0] // 2, fused.shape[1] // 2
    center_fused = float(fused[cy, cx]) if fused_valid[cy, cx] else float("nan")
    lander_plan = draw_lander_overlay(fusion_panel, center_fused)

    separator = np.full((PANEL_H, 4, 3), 42, dtype=np.uint8)
    row1 = np.hstack((camera, separator, da_panel))
    row2 = np.hstack((moge_panel, separator, fusion_panel))
    divider = np.full((4, row1.shape[1], 3), 42, dtype=np.uint8)
    grid = np.vstack((row1, divider, row2))

    da_share = float(np.mean(da_w[fused_valid] / total_w[fused_valid])) if fused_valid.any() else 0.0
    footer = np.full((FOOTER_H, grid.shape[1], 3), 18, dtype=np.uint8)
    cv2.putText(footer, f"Frame {frame_id} | TCP {tcp_fps:.2f} FPS | DA {da_ms:.0f} ms + MoGe {moge_ms:.0f} ms = {da_ms + moge_ms:.0f} ms | JPEG {jpeg_bytes / 1024:.1f} KiB", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (80, 245, 245), 1, cv2.LINE_AA)
    cv2.putText(footer, f"Fusion center: {center_fused:.3f}m | DA scale-align x{scale:.3f} | average DA weight: {da_share * 100:.0f}% | heuristic reliability only; not control truth", (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)
    return np.vstack((grid, footer)), {
        "da_aligned": da_aligned,
        "moge": moge_raw,
        "fused": fused,
        "da_weight": da_w,
        "moge_weight": moge_w,
        "valid": fused_valid,
        "texture": texture,
        "agreement": agreement,
        "diagnostic": reliability_thumbnail(texture, agreement, da_w, moge_w),
        "scale": scale,
        "source": source,
        "da_panel": da_panel,
        "moge_panel": moge_panel,
        "fusion_panel": fusion_panel,
        "lander_plan": lander_plan,
    }


def save_snapshot(outputs, frame_id):
    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = os.path.join(SAVE_DIR, f"{stamp}_frame{frame_id:06d}")
    for name in ("source", "da_panel", "moge_panel", "fusion_panel", "diagnostic"):
        cv2.imwrite(prefix + f"_{name}.png", outputs[name])
    for name in ("da_aligned", "moge", "fused", "da_weight", "moge_weight", "valid", "texture", "agreement"):
        np.save(prefix + f"_{name}.npy", outputs[name])
    with open(prefix + "_metadata.txt", "w", encoding="utf-8") as file:
        file.write(f"da_scale_to_moge={outputs['scale']:.8f}\n")
        file.write("reliability=validity*texture*temporal_stability*model_agreement\n")
        file.write("warning=heuristic fusion, not calibrated measurement\n")
    print("Saved fusion snapshot", prefix, flush=True)


def main():
    moge, da = load_models()
    ip = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
    sock = connect_stream(ip)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1215, 960)
    print("Four-panel viewer started. Keys: s=save q/ESC=quit r=reliability diagnostic", flush=True)

    buffer = b""
    frame_id = 0
    previous_time = time.monotonic()
    smoothed_tcp_fps = 0.0
    previous = (None, None)
    last_outputs = None
    diagnostic_mode = False
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
                    print(f"TCP closed ({exc}); retrying without reloading models...", flush=True)
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
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
            moge_input = torch.as_tensor(rgb, dtype=torch.float32, device="cuda").permute(2, 0, 1) / 255.0

            torch.cuda.synchronize()
            start = time.perf_counter()
            with torch.inference_mode():
                da_depth = da.infer_image(bgr, DA_INPUT_SIZE).astype(np.float32)
            torch.cuda.synchronize()
            da_ms = (time.perf_counter() - start) * 1000.0

            torch.cuda.synchronize()
            start = time.perf_counter()
            moge_output = moge.infer(moge_input, fov_x=FOV_X_DEG, resolution_level=5, use_fp16=True)
            torch.cuda.synchronize()
            moge_ms = (time.perf_counter() - start) * 1000.0
            moge_depth = moge_output["depth"].float().cpu().numpy()
            moge_mask = moge_output["mask"].bool().cpu().numpy()
            frame_id += 1

            canvas, outputs = make_canvas(gray, da_depth, moge_depth, moge_mask, frame_id, smoothed_tcp_fps, da_ms, moge_ms, len(jpeg), previous)
            previous = (outputs["da_aligned"].copy(), outputs["moge"].copy())
            last_outputs = outputs
            if diagnostic_mode:
                diagnostic = cv2.resize(outputs["diagnostic"], (1215, 860), interpolation=cv2.INTER_NEAREST)
                cv2.putText(diagnostic, "Reliability diagnostic: B=MoGe weight, G=model agreement, R=texture support", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow(WINDOW, diagnostic)
            else:
                cv2.imshow(WINDOW, canvas)
            if frame_id % 5 == 0:
                center = outputs["fused"][outputs["fused"].shape[0] // 2, outputs["fused"].shape[1] // 2]
                print(f"FRAME {frame_id}: DA={da_ms:.1f}ms MoGe={moge_ms:.1f}ms fused_center={center:.3f}m scale={outputs['scale']:.3f}", flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and last_outputs is not None:
                save_snapshot(last_outputs, frame_id)
            elif key == ord("r"):
                diagnostic_mode = not diagnostic_mode
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print(f"Fusion viewer stopped after {frame_id} frames.", flush=True)


if __name__ == "__main__":
    main()
