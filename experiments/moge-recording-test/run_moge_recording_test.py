"""Offline MoGe-2 test on cropped left-camera panels from a Mars Lander console recording.

This script is an evaluation artifact only. It has no hardware, network, actuator,
or console-control calls. It writes maps and a JSONL report for later review.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

RUNTIME_ROOT = Path(r"C:\Users\Kerbal Chen\AppData\Local\Temp\moge_recording_runtime")
MOGE_ROOT = RUNTIME_ROOT / "MoGe"
if str(MOGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOGE_ROOT))

from moge.model.v2 import MoGeModel

INPUT_DIR = RUNTIME_ROOT / "left_camera_frames"
OUTPUT_DIR = RUNTIME_ROOT / "results"
MODEL_PATH = Path(r"C:\Users\Kerbal Chen\.cache\huggingface\hub\models--Ruicheng--moge-2-vitb-normal\snapshots\54ad3a693e61907ea4633d13dec6ee682fa09419\model.pt")

# The extraction crop is only a display-scaled copy of the console's left panel.
# Do not treat its pixel geometry or the following provisional FOV as calibrated truth.
PROVISIONAL_FOV_X_DEG = math.degrees(2.0 * math.atan(800.0 / (2.0 * 2182.891044827313)))
RESOLUTION_LEVEL = 5


def colorize_depth(depth_m: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.0)
    if not valid.any():
        raise RuntimeError("MoGe produced no finite positive depth")
    p02, p50, p98 = np.percentile(depth_m[valid], [2.0, 50.0, 98.0]).astype(float)
    normalized = np.clip((depth_m - p02) / max(p98 - p02, 1e-6), 0.0, 1.0)
    image = cv2.applyColorMap(((1.0 - normalized) * 255.0).astype(np.uint8), cv2.COLORMAP_INFERNO)
    image[~valid] = 0
    return image, {"p02_m": p02, "median_m": p50, "p98_m": p98}


def main() -> None:
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable")
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(MODEL_PATH)
    frames = sorted(INPUT_DIR.glob("*.png"))
    if not frames:
        raise FileNotFoundError(f"No extracted camera panels under {INPUT_DIR}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    report_path = OUTPUT_DIR / "report.jsonl"

    print(f"GPU={torch.cuda.get_device_name(0)}", flush=True)
    print(f"MODEL={MODEL_PATH}", flush=True)
    print(f"INPUT_COUNT={len(frames)} FOV_X_DEG_PROVISIONAL={PROVISIONAL_FOV_X_DEG:.6f}", flush=True)
    model = MoGeModel.from_pretrained(str(MODEL_PATH)).to("cuda").eval().half()

    warm = torch.zeros((3, 430, 438), device="cuda", dtype=torch.float32)
    _ = model.infer(warm, fov_x=PROVISIONAL_FOV_X_DEG, resolution_level=RESOLUTION_LEVEL, use_fp16=True)
    torch.cuda.synchronize()
    print("MODEL_WARMED=1", flush=True)

    with report_path.open("w", encoding="utf-8") as report:
        for source_path in frames:
            source = cv2.imread(str(source_path), cv2.IMREAD_COLOR)
            if source is None:
                raise RuntimeError(f"Could not decode {source_path}")
            rgb = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
            tensor = torch.as_tensor(rgb, device="cuda", dtype=torch.float32).permute(2, 0, 1) / 255.0
            torch.cuda.reset_peak_memory_stats()
            torch.cuda.synchronize()
            started = time.perf_counter()
            output = model.infer(tensor, fov_x=PROVISIONAL_FOV_X_DEG, resolution_level=RESOLUTION_LEVEL, use_fp16=True)
            torch.cuda.synchronize()
            elapsed_ms = (time.perf_counter() - started) * 1000.0

            depth = output["depth"].float().cpu().numpy()
            normal = output["normal"].float().cpu().numpy()
            mask = output["mask"].bool().cpu().numpy()
            intrinsics = output["intrinsics"].float().cpu().numpy()
            points = output["points"].float().cpu().numpy()
            depth_vis, quantiles = colorize_depth(depth, mask)
            normal_vis = cv2.cvtColor(np.clip((normal + 1.0) * 127.5, 0.0, 255.0).astype(np.uint8), cv2.COLOR_RGB2BGR)
            normal_vis[~mask] = 0

            prefix = OUTPUT_DIR / source_path.stem
            cv2.imwrite(str(prefix) + "_source.png", source)
            cv2.imwrite(str(prefix) + "_depth_vis.png", depth_vis)
            cv2.imwrite(str(prefix) + "_normal_vis.png", normal_vis)
            cv2.imwrite(str(prefix) + "_mask.png", (mask.astype(np.uint8) * 255))
            np.save(str(prefix) + "_depth_m.npy", depth)
            np.save(str(prefix) + "_mask.npy", mask)
            np.save(str(prefix) + "_normal.npy", normal)
            np.save(str(prefix) + "_intrinsics_normalized.npy", intrinsics)
            np.save(str(prefix) + "_points_m.npy", points)

            h, w = depth.shape
            valid = mask & np.isfinite(depth) & (depth > 0.0)
            item = {
                "source": source_path.name,
                "source_shape_hwc": list(source.shape),
                "depth_shape_hw": list(depth.shape),
                "resolution_level": RESOLUTION_LEVEL,
                "fov_x_deg_provisional": PROVISIONAL_FOV_X_DEG,
                "inference_ms": elapsed_ms,
                "vram_peak_mb": torch.cuda.max_memory_allocated() / 1024.0 / 1024.0,
                "mask_coverage_pct": float(mask.mean() * 100.0),
                "center_depth_m_candidate": float(depth[h // 2, w // 2]) if valid[h // 2, w // 2] else None,
                "intrinsics_normalized": intrinsics.tolist(),
                **quantiles,
                "metric_warning": "Candidate only: cropped display image plus provisional calibration; not a calibrated measurement or control input.",
            }
            report.write(json.dumps(item, ensure_ascii=False) + "\n")
            print(json.dumps(item, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
