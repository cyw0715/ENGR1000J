from __future__ import annotations
import json, socket, struct, time, urllib.request
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import pipeline

from absolute_terrain import (
    anchor_depth_with_ultrasonic, fuse_metric_and_relative,
    load_provisional_calibration, select_best_depth_frames,
    source_observability, stable_cloud_average,
)
from landing_control import DepthTerrainHeightField, cv_plan_to_relative_deltas, solve_best_visible_platform
from lander_console import (
    METRIC_DEPTH_CHECKPOINT, METRIC_DEPTH_INPUT_SIZE, METRIC_DEPTH_SOURCE,
    RELATIVE_DEPTH_MODEL_ID, stabilize_ultrasonic_samples,
)

HOST = "192.168.137.106"
OUT = Path(r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test\calibration_runs\2026-08-04-fisheye-10x7-15mm\sr04_depth_correction_live.json")
N = 30


def get_health():
    with urllib.request.urlopen(f"http://{HOST}/health", timeout=3) as r:
        return json.load(r)


def exact(sock, n):
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(n - len(out))
        if not chunk: raise ConnectionError("video closed")
        out.extend(chunk)
    return bytes(out)


def get_frame(sock):
    n = struct.unpack("<I", exact(sock, 4))[0]
    data = exact(sock, n)
    gray = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    if gray is None: raise ValueError("JPEG decode failed")
    return Image.fromarray(gray).convert("RGB"), n


def terrain_plan(depth, cal):
    terrain = DepthTerrainHeightField(
        depth, fx_px=cal["fx_px"], fy_px=cal["fy_px"],
        cx_px=cal["cx_px"], cy_px=cal["cy_px"],
        projection_model=cal["projection_model"],
        distortion_coefficients=cal.get("D"),
    )
    plan = solve_best_visible_platform(terrain.get_height)
    delta = None
    if plan.get("status") == "safe" and plan.get("geometric_safe"):
        delta = cv_plan_to_relative_deltas(plan)
    return terrain, plan, delta


def compact_plan(plan):
    return {
        "status": plan.get("status"), "geometric_safe": plan.get("geometric_safe"),
        "visible_leg_count": plan.get("visible_leg_count"),
        "lengths_mm": plan.get("lengths_mm"),
        "unsafe_reasons": plan.get("unsafe_reasons"),
    }


def main():
    import sys
    sys.path.insert(0, str(METRIC_DEPTH_SOURCE))
    from depth_anything_v2.dpt import DepthAnythingV2

    cal = load_provisional_calibration()
    relative = pipeline("depth-estimation", model=RELATIVE_DEPTH_MODEL_ID, device="cuda", dtype=torch.float16)
    metric = DepthAnythingV2(encoder="vitb", features=128, out_channels=[96,192,384,768], max_depth=20)
    metric.load_state_dict(torch.load(METRIC_DEPTH_CHECKPOINT, map_location="cpu", weights_only=True), strict=True)
    metric = metric.to("cuda").eval()
    relative(Image.new("RGB", (512,512), "black"))
    with torch.inference_mode(): metric.infer_image(np.zeros((800,800,3),np.uint8), METRIC_DEPTH_INPUT_SIZE)

    samples = deque(maxlen=7); seen = set(); raw_maps=[]; corrected_maps=[]; metas=[]; jpeg_sizes=[]
    sock = socket.create_connection((HOST, 5000), 5); sock.settimeout(8)
    try:
        deadline = time.monotonic() + 180
        while len(corrected_maps) < N:
            if time.monotonic() > deadline: raise TimeoutError(f"only {len(corrected_maps)}/{N} valid frames")
            health = get_health(); u = health["ultrasonic"]
            if u.get("valid") and u.get("valid_count") not in seen:
                seen.add(u["valid_count"]); samples.append(dict(u))
            image, size = get_frame(sock)
            quality = source_observability(np.asarray(image))
            if not quality["observable"]: continue
            try: stable_u = stabilize_ultrasonic_samples(list(samples))
            except ValueError: continue
            rgb=np.asarray(image); bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            with torch.inference_mode(): metric_map=metric.infer_image(bgr,METRIC_DEPTH_INPUT_SIZE).astype(np.float32)
            rel=relative(image)["predicted_depth"].detach().float().cpu().numpy().astype(np.float32)
            if rel.shape != metric_map.shape: rel=cv2.resize(rel,(metric_map.shape[1],metric_map.shape[0]))
            raw,_,_=fuse_metric_and_relative(metric_map,rel)
            corrected,meta=anchor_depth_with_ultrasonic(raw,stable_u,cx_px=cal["cx_px"],cy_px=cal["cy_px"])
            raw_maps.append(raw); corrected_maps.append(corrected); metas.append(meta); jpeg_sizes.append(size)
            print(f"FRAME {len(corrected_maps):02d}/{N} US={meta['ultrasonic_mm']:.0f}mm scale={meta['scale']:.4f}", flush=True)
    finally: sock.close()

    _, indices, scores = select_best_depth_frames(corrected_maps, 10)
    raw_cloud=stable_cloud_average([raw_maps[i] for i in indices])
    corrected_cloud=stable_cloud_average([corrected_maps[i] for i in indices])
    if raw_cloud is None or corrected_cloud is None: raise RuntimeError("stable cloud failed")
    raw_terrain,raw_plan,raw_delta=terrain_plan(raw_cloud,cal)
    cor_terrain,cor_plan,cor_delta=terrain_plan(corrected_cloud,cal)
    scales=np.array([m["scale"] for m in metas]); distances=np.array([m["ultrasonic_mm"] for m in metas])
    report={
      "schema":"sr04-depth-correction-live-v1","host":HOST,"frames":N,
      "selected_indices":indices,"selection_scores":scores,
      "jpeg_bytes":{"min":min(jpeg_sizes),"max":max(jpeg_sizes),"mean":float(np.mean(jpeg_sizes))},
      "sr04_mm":{"min":float(distances.min()),"median":float(np.median(distances)),"max":float(distances.max())},
      "scale":{"min":float(scales.min()),"median":float(np.median(scales)),"max":float(scales.max())},
      "raw":{"camera_ground_depth_mm":raw_terrain.camera_ground_depth_mm,"plan":compact_plan(raw_plan),"relative_delta_mm":raw_delta},
      "corrected":{"camera_ground_depth_mm":cor_terrain.camera_ground_depth_mm,"plan":compact_plan(cor_plan),"relative_delta_mm":cor_delta},
      "control_semantics":"SR04 scales the depth map; CV command remains zero-common-mode relative terrain shape.",
    }
    OUT.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    print("RESULT",json.dumps(report,ensure_ascii=False),flush=True)
    print("SAVED",OUT,flush=True)

if __name__=="__main__": main()
