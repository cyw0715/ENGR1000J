"""Display-only absolute-height terrain fusion and lander projection helpers.

All returned values are visualization/planning estimates. This module contains
no actuator, P4 command, Mega, PWM, or network-control calls.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

CAPTURE_FRAME_COUNT = 30
SELECTED_FRAME_COUNT = 10
# Stable-cloud aggregation still consumes the selected best ten frames.
STABLE_FRAME_COUNT = SELECTED_FRAME_COUNT
STABLE_RELATIVE_SPREAD_LIMIT = 0.20
SOURCE_MIN_P5_P95_SPAN = 60.0
SOURCE_MIN_STRUCTURE_STD = 12.0
SOURCE_MIN_STRUCTURE_COHERENCE = 0.55
SOURCE_MAX_SATURATION_FRACTION = 0.25
ULTRASONIC_STALE_US = 1_000_000
ULTRASONIC_MIN_MM = 20.0
ULTRASONIC_MAX_MM = 4000.0
ULTRASONIC_CENTER_RADIUS_PX = 12
ULTRASONIC_MIN_VALID_FRACTION = 0.70
ULTRASONIC_MAX_RELATIVE_MAD = 0.10
ULTRASONIC_SCALE_MIN = 0.25
ULTRASONIC_SCALE_MAX = 4.0
ULTRASONIC_FILTER_WINDOW = 7
ULTRASONIC_FILTER_MIN_SAMPLES = 5
ULTRASONIC_FILTER_MAX_RELATIVE_MAD = 0.05
ULTRASONIC_FILTER_MAX_RELATIVE_SPAN = 0.15
ULTRASONIC_FILTER_MAX_ABSOLUTE_SPAN_MM = 30.0
BODY_SIZE_M = 0.300
NOMINAL_LEG_LENGTH_M = 0.350
LEG_ANGLE_FROM_VERTICAL_RAD = math.radians(30.0)


class BestOfThirtyDepthCapture:
    """Collect exactly thirty future valid maps for later best-ten selection."""

    def __init__(self) -> None:
        self.frames: list[np.ndarray] = []
        self.active = False

    @property
    def count(self) -> int:
        return len(self.frames)

    def start(self) -> None:
        self.frames = []
        self.active = True

    def accept(self, depth_m: np.ndarray) -> bool:
        if not self.active:
            return False
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or not _positive_finite(depth).any():
            return False
        self.frames.append(depth.copy())
        if len(self.frames) >= CAPTURE_FRAME_COUNT:
            self.active = False
        return True


CALIBRATION_RELATIVE_PATH = (
    Path("wifi_test") / "calibration_runs" /
    "2026-07-24-ov5647-intrinsics-8x5-20mm" / "views" /
    "provisional_user_accepted_camera_calibration.json"
)


def calibration_path() -> Path:
    """Find the calibration beside the project, including a staged Windows viewer."""
    module_dir = Path(__file__).resolve().parent
    candidates = (
        module_dir.parent / CALIBRATION_RELATIVE_PATH,
        module_dir.parent.parent / CALIBRATION_RELATIVE_PATH,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "未找到暂定相机标定文件；已检查：" + ", ".join(str(path) for path in candidates)
    )


FISHEYE_IMAGE_WIDTH_PX = 800
FISHEYE_IMAGE_HEIGHT_PX = 800
FISHEYE_DIAGONAL_FOV_DEG = 160.0
FISHEYE_PHYSICAL_FOCAL_LENGTH_MM = 3.15
FISHEYE_CX_PX = (FISHEYE_IMAGE_WIDTH_PX - 1) / 2.0
FISHEYE_CY_PX = (FISHEYE_IMAGE_HEIGHT_PX - 1) / 2.0
FISHEYE_CORNER_RADIUS_PX = math.hypot(FISHEYE_CX_PX, FISHEYE_CY_PX)
FISHEYE_F_PX = FISHEYE_CORNER_RADIUS_PX / math.radians(FISHEYE_DIAGONAL_FOV_DEG / 2.0)


FISHEYE_CALIBRATION_RELATIVE_PATH = (
    Path("wifi_test") / "calibration_runs" /
    "2026-08-04-fisheye-10x7-15mm" / "fisheye_calibration.json"
)
CAMERA_EXTRINSICS_RELATIVE_PATH = (
    Path("wifi_test") / "calibration_runs" /
    "2026-08-04-fisheye-10x7-15mm" / "camera_to_board_pose.json"
)


def accepted_camera_extrinsics_path() -> Path:
    module_dir = Path(__file__).resolve().parent
    for candidate in (
        module_dir.parent / CAMERA_EXTRINSICS_RELATIVE_PATH,
        module_dir.parent.parent / CAMERA_EXTRINSICS_RELATIVE_PATH,
    ):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("accepted camera extrinsics not found")


def load_camera_extrinsics() -> dict:
    path = accepted_camera_extrinsics_path()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("axis_mapping_validated", False):
        raise ValueError("camera platform-axis mapping is not validated")
    R = np.asarray(data["R_board_to_camera"], dtype=np.float64)
    t = np.asarray(data["t_board_to_camera_mm"], dtype=np.float64).reshape(3)
    if R.shape != (3, 3) or not np.isfinite(R).all() or not np.isfinite(t).all():
        raise ValueError("invalid camera extrinsics")
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-4) or np.linalg.det(R) < 0.99:
        raise ValueError("camera rotation is not proper orthonormal")
    return {**data, "R": R, "t_mm": t, "extrinsics_path": str(path)}


def accepted_fisheye_calibration_path() -> Path:
    module_dir = Path(__file__).resolve().parent
    candidates = (
        module_dir.parent / FISHEYE_CALIBRATION_RELATIVE_PATH,
        module_dir.parent.parent / FISHEYE_CALIBRATION_RELATIVE_PATH,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("accepted fisheye calibration not found")


def load_provisional_calibration() -> dict:
    """Load accepted OpenCV fisheye intrinsics, with explicit provisional fallback."""
    try:
        path = accepted_fisheye_calibration_path()
        data = json.loads(path.read_text(encoding="utf-8"))
        if not data.get("accepted_for_geometry", False):
            raise ValueError("fisheye calibration failed quality gates")
        K = np.asarray(data["K"], dtype=np.float64)
        D = np.asarray(data["D"], dtype=np.float64).reshape(4)
        width, height = map(int, data["image_size"])
        if K.shape != (3, 3) or not np.isfinite(K).all() or not np.isfinite(D).all():
            raise ValueError("invalid calibrated K/D")
        return {
            "fx_px": float(K[0, 0]), "fy_px": float(K[1, 1]),
            "cx_px": float(K[0, 2]), "cy_px": float(K[1, 2]),
            "K": K.tolist(), "D": D.tolist(),
            "image_width_px": width, "image_height_px": height,
            "projection_model": "opencv_fisheye",
            "calibrated": True, "provisional": False,
            "calibration_path": str(path),
            "warning": "Accepted OpenCV fisheye intrinsics; depth scale and camera-to-platform extrinsics still require validation.",
        }
    except (FileNotFoundError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return {
            "fx_px": FISHEYE_F_PX, "fy_px": FISHEYE_F_PX,
            "cx_px": FISHEYE_CX_PX, "cy_px": FISHEYE_CY_PX,
            "image_width_px": FISHEYE_IMAGE_WIDTH_PX,
            "image_height_px": FISHEYE_IMAGE_HEIGHT_PX,
            "projection_model": "equidistant_fisheye",
            "diagonal_fov_deg": FISHEYE_DIAGONAL_FOV_DEG,
            "physical_focal_length_mm": FISHEYE_PHYSICAL_FOCAL_LENGTH_MM,
            "calibrated": False, "provisional": True,
            "warning": (
                "Planning-only provisional equidistant fisheye model: 800x800, "
                "160 deg diagonal FOV, 3.15 mm stated focal length; no lens calibration."
            ),
        }


def camera_fov_x_deg(fx_px: float, image_width_px: int) -> float:
    """Horizontal FOV derived from an image-calibration focal length."""
    if not np.isfinite(fx_px) or fx_px <= 0 or image_width_px <= 0:
        raise ValueError("invalid calibrated focal length or image width")
    return math.degrees(2.0 * math.atan(float(image_width_px) / (2.0 * float(fx_px))))


def _positive_finite(depth: np.ndarray) -> np.ndarray:
    return np.isfinite(depth) & (depth > 0.0)


def source_observability(image: np.ndarray) -> dict:
    """Reject frames whose stable model output would be unsupported by scene structure."""
    gray = np.asarray(image)
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray.astype(np.uint8), cv2.COLOR_RGB2GRAY)
    if gray.ndim != 2 or gray.size == 0:
        return {"observable": False, "reason": "invalid source image"}
    gray = gray.astype(np.float32)
    p5, p95 = np.percentile(gray, [5, 95])
    span = float(p95 - p5)
    raw_std = float(gray.std())
    # Downsample by area to suppress fixed high-frequency sensor/DCT patterns;
    # real scene-scale edges and illumination structure remain.
    structure = cv2.resize(gray, (50, 50), interpolation=cv2.INTER_AREA)
    structure_std = float(structure.std())
    coherence = structure_std / max(raw_std, 1e-6)
    saturation = float(((gray < 5.0) | (gray > 250.0)).mean())
    gates = {
        "p5_p95_span": span >= SOURCE_MIN_P5_P95_SPAN,
        "structure_std": structure_std >= SOURCE_MIN_STRUCTURE_STD,
        "structure_coherence": coherence >= SOURCE_MIN_STRUCTURE_COHERENCE,
        "saturation_fraction": saturation <= SOURCE_MAX_SATURATION_FRACTION,
    }
    failed = [name for name, passed in gates.items() if not passed]
    return {
        "observable": all(gates.values()), "reason": "ok" if not failed else ",".join(failed),
        "p5_p95_span": span, "raw_std": raw_std, "structure_std": structure_std,
        "structure_coherence": float(coherence), "saturation_fraction": saturation,
        "gates": gates,
    }


def anchor_depth_with_ultrasonic(depth_m: np.ndarray, ultrasonic: dict,
                                 *, cx_px: float, cy_px: float,
                                 radius_px: int = ULTRASONIC_CENTER_RADIUS_PX) -> tuple[np.ndarray, dict]:
    """Scale one model-depth map using a fresh camera-center ultrasonic anchor."""
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth map must be 2D")
    if not isinstance(ultrasonic, dict) or not ultrasonic.get("valid", False):
        raise ValueError("ultrasonic anchor invalid")
    distance_mm = float(ultrasonic.get("distance_mm", -1))
    sample_age_us = float(ultrasonic.get("sample_age_us", -1))
    if not ULTRASONIC_MIN_MM <= distance_mm <= ULTRASONIC_MAX_MM:
        raise ValueError("ultrasonic distance out of range")
    if sample_age_us < 0 or sample_age_us > ULTRASONIC_STALE_US:
        raise ValueError("ultrasonic anchor stale")
    cx, cy = int(round(cx_px)), int(round(cy_px))
    y0, y1 = max(0, cy - radius_px), min(depth.shape[0], cy + radius_px + 1)
    x0, x1 = max(0, cx - radius_px), min(depth.shape[1], cx + radius_px + 1)
    roi = depth[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi > 0)
    if roi.size == 0 or float(valid.mean()) < ULTRASONIC_MIN_VALID_FRACTION:
        raise ValueError("model center ROI insufficient")
    values = roi[valid].astype(np.float64)
    center_m = float(np.median(values))
    relative_mad = float(np.median(np.abs(values - center_m)) / max(center_m, 1e-6))
    if relative_mad > ULTRASONIC_MAX_RELATIVE_MAD:
        raise ValueError("model center ROI inconsistent")
    scale = (distance_mm / 1000.0) / center_m
    if not ULTRASONIC_SCALE_MIN <= scale <= ULTRASONIC_SCALE_MAX:
        raise ValueError("ultrasonic scale outside safe range")
    anchored = depth.copy()
    finite = np.isfinite(anchored) & (anchored > 0)
    anchored[finite] *= np.float32(scale)
    anchored[~finite] = np.nan
    return anchored, {
        "scale": float(scale), "ultrasonic_mm": distance_mm,
        "model_center_m": center_m, "center_relative_mad": relative_mad,
        "sample_age_us": int(sample_age_us), "roi_valid_fraction": float(valid.mean()),
        "center_px": [float(cx_px), float(cy_px)], "source": ultrasonic.get("source", "unknown"),
        "raw_latest_mm": ultrasonic.get("raw_latest_mm", distance_mm),
        "filter_samples": int(ultrasonic.get("filter_samples", 1)),
        "filter_mad_mm": float(ultrasonic.get("filter_mad_mm", 0.0)),
        "filter_span_mm": float(ultrasonic.get("filter_span_mm", 0.0)),
    }


def fuse_metric_and_relative(metric_m: np.ndarray, relative_m: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Median-align a second depth model then take an equal display-only mean.

    The metric model supplies the nominal scale. Relative output is aligned only
    over jointly valid pixels. This is an estimate, never a calibrated sensor.
    """
    metric = np.asarray(metric_m, dtype=np.float32)
    relative = np.asarray(relative_m, dtype=np.float32)
    if metric.shape != relative.shape or metric.ndim != 2:
        raise ValueError("metric and relative depth maps must be equal 2-D shapes")
    valid = _positive_finite(metric) & _positive_finite(relative)
    fused = np.full(metric.shape, np.nan, dtype=np.float32)
    if int(valid.sum()) < 1:
        return fused, valid, float("nan")
    metric_median = float(np.median(metric[valid]))
    relative_median = float(np.median(relative[valid]))
    if relative_median <= 1e-6:
        return fused, valid, float("nan")
    scale = float(np.clip(metric_median / relative_median, 0.10, 10.0))
    aligned_relative = relative * scale
    fused[valid] = 0.5 * (metric[valid] + aligned_relative[valid])
    return fused, valid, scale


def metric_contour_map(depth_m: np.ndarray, level_count: int = 10):
    """Render a dark metric contour map from a finite positive depth cloud.

    Contours are linearly spaced between the robust P10/P90 depth bounds so a
    single outlier does not flatten all visible terrain detail. Invalid pixels
    remain black. Returned levels are in metres.
    """
    from PIL import Image

    depth = np.asarray(depth_m, dtype=np.float32)
    valid = _positive_finite(depth)
    if depth.ndim != 2 or not valid.any():
        raise ValueError("depth map has no finite positive metric values")
    if level_count < 2:
        raise ValueError("level_count must be at least 2")

    low_m, high_m = np.percentile(depth[valid], [10.0, 90.0]).astype(float)
    if high_m <= low_m + 1e-6:
        center = float(np.median(depth[valid]))
        low_m, high_m = center - 0.005, center + 0.005
    levels = np.linspace(low_m, high_m, level_count, dtype=np.float32)

    height, width = depth.shape
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    # A faint blue field preserves the cloud footprint without resembling the
    # old binary terrain-threshold panel.
    normalized = np.clip(np.nan_to_num((depth - low_m) / (high_m - low_m), nan=0.0), 0.0, 1.0)
    rgb[..., 2] = (20 + 35 * normalized).astype(np.uint8)
    rgb[..., 1] = (8 + 18 * normalized).astype(np.uint8)
    rgb[~valid] = 0

    palette = np.array([
        [90, 220, 255], [80, 255, 170], [210, 255, 90],
        [255, 205, 70], [255, 115, 85], [240, 100, 220],
    ], dtype=np.uint8)
    for index, level in enumerate(levels):
        above = valid & (depth >= level)
        edge = np.zeros_like(valid)
        edge[:, 1:] |= above[:, 1:] != above[:, :-1]
        edge[1:, :] |= above[1:, :] != above[:-1, :]
        edge &= valid
        rgb[edge] = palette[index % len(palette)]

    return Image.fromarray(rgb, "RGB"), [float(level) for level in levels]


def metric_depth_heatmap(depth_m: np.ndarray):
    """Colorize metric meters without hiding invalid pixels; warm colors are nearer."""
    from PIL import Image

    depth = np.asarray(depth_m, dtype=np.float32)
    valid = _positive_finite(depth)
    if depth.ndim != 2 or not valid.any():
        raise ValueError("depth map has no finite positive metric values")
    near_m, far_m = np.percentile(depth[valid], [2.0, 98.0]).astype(float)
    far_m = max(far_m, near_m + 1e-6)
    normalized = np.clip(np.nan_to_num((depth - near_m) / (far_m - near_m), nan=1.0), 0.0, 1.0)
    red = np.clip(255.0 * (1.7 * (1.0 - normalized)), 0, 255).astype(np.uint8)
    green = np.clip(255.0 * (1.2 * (1.0 - normalized) - 0.15), 0, 255).astype(np.uint8)
    blue = np.clip(255.0 * (0.55 - (1.0 - normalized)), 0, 255).astype(np.uint8)
    rgb = np.dstack((red, green, blue))
    rgb[~valid] = 0
    return Image.fromarray(rgb, "RGB"), near_m, far_m


def select_best_depth_frames(depth_maps_m: Iterable[np.ndarray],
                             selected_count: int = SELECTED_FRAME_COUNT
                             ) -> tuple[list[np.ndarray], list[int], list[float]]:
    """Select maps nearest a robust 30-frame median reference.

    Each score is the median absolute error to the pixelwise median reference,
    normalized by the reference depth. Invalid overlap is penalized. Lower is
    better; ties retain acquisition order.
    """
    maps = [np.asarray(depth, dtype=np.float32) for depth in depth_maps_m]
    if len(maps) < selected_count or selected_count < 1:
        raise ValueError("not enough depth maps for best-frame selection")
    shape = maps[0].shape
    if any(depth.ndim != 2 or depth.shape != shape for depth in maps):
        raise ValueError("all depth maps must share one 2-D shape")

    stack = np.stack(maps, axis=0)
    valid_stack = np.isfinite(stack) & (stack > 0.0)
    reference_stack = np.where(valid_stack, stack, np.nan)
    with np.errstate(all="ignore"):
        reference = np.nanmedian(reference_stack, axis=0)
    reference_valid = np.isfinite(reference) & (reference > 0.0)
    if not reference_valid.any():
        raise ValueError("no valid median depth reference")

    scored: list[tuple[float, int]] = []
    reference_scale = np.maximum(reference, 0.10)
    reference_pixels = int(reference_valid.sum())
    for index, depth in enumerate(maps):
        overlap = reference_valid & np.isfinite(depth) & (depth > 0.0)
        overlap_count = int(overlap.sum())
        if overlap_count == 0:
            score = float("inf")
        else:
            relative_error = np.abs(depth[overlap] - reference[overlap]) / reference_scale[overlap]
            missing_fraction = 1.0 - overlap_count / reference_pixels
            score = float(np.median(relative_error) + missing_fraction)
        scored.append((score, index))

    winners = sorted(scored, key=lambda item: (item[0], item[1]))[:selected_count]
    if not np.isfinite(winners[-1][0]):
        raise ValueError("fewer than requested valid depth maps")
    indices = [index for _score, index in winners]
    scores = [float(score) for score, _index in winners]
    return [maps[index].copy() for index in indices], indices, scores


def stable_cloud_average(depth_maps_m: Iterable[np.ndarray]) -> np.ndarray | None:
    """Return a pixelwise mean only after exactly ten equally shaped valid maps."""
    maps = [np.asarray(depth, dtype=np.float32) for depth in depth_maps_m]
    if len(maps) != STABLE_FRAME_COUNT:
        return None
    shape = maps[0].shape
    if len(shape) != 2 or any(depth.shape != shape for depth in maps):
        return None
    stack = np.stack(maps, axis=0)
    height, width = shape
    valid_count = np.isfinite(stack).sum(axis=0)
    average = np.full(shape, np.nan, dtype=np.float32)
    fully_valid = valid_count == STABLE_FRAME_COUNT
    if not fully_valid.any():
        return None
    mean = np.mean(stack[:, fully_valid], axis=0)
    spread = np.max(stack[:, fully_valid], axis=0) - np.min(stack[:, fully_valid], axis=0)
    stable = spread <= STABLE_RELATIVE_SPREAD_LIMIT * np.maximum(mean, 0.10)
    if not stable.any():
        return None
    average[fully_valid] = mean
    average[np.flatnonzero(fully_valid)[~stable] // width, np.flatnonzero(fully_valid)[~stable] % width] = np.nan
    return average


def ground_height_summary(stable_depth_m: np.ndarray) -> dict | None:
    """Robust center-region ground height summary from a ten-frame stable cloud."""
    depth = np.asarray(stable_depth_m, dtype=np.float32)
    if depth.ndim != 2:
        return None
    height, width = depth.shape
    y0, y1 = height // 4, max(height // 4 + 1, height * 3 // 4)
    x0, x1 = width // 4, max(width // 4 + 1, width * 3 // 4)
    region = depth[y0:y1, x0:x1]
    valid = _positive_finite(region)
    if not valid.any():
        return None
    values = region[valid]
    return {
        "height_m": float(np.median(values)),
        "p10_m": float(np.percentile(values, 10.0)),
        "p90_m": float(np.percentile(values, 90.0)),
        "valid_pixels": int(valid.sum()),
    }


def project_lander_plan(
    center_height_m: float,
    image_shape: tuple[int, int],
    *,
    fx_px: float,
    fy_px: float,
    cx_px: float,
    cy_px: float,
    body_size_m: float = BODY_SIZE_M,
    nominal_leg_length_m: float = NOMINAL_LEG_LENGTH_M,
) -> dict | None:
    """Project a nominal horizontal 300 mm lander and its four foot axes.

    Assumptions: camera centered on platform, optical axis vertically downward,
    platform horizontal. The result is explicitly display-only.
    """
    if not np.isfinite(center_height_m) or center_height_m <= 0.0:
        return None
    if min(fx_px, fy_px) <= 0.0 or len(image_shape) != 2:
        return None
    height, width = int(image_shape[0]), int(image_shape[1])
    if min(height, width) <= 0:
        return None
    scale_x, scale_y = width / 800.0, height / 800.0
    fx, fy = fx_px * scale_x, fy_px * scale_y
    center = np.array([cx_px * scale_x, cy_px * scale_y], dtype=np.float32)
    signs = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=np.float32)
    corner_radius = body_size_m / math.sqrt(2.0)
    foot_radius = corner_radius + nominal_leg_length_m * math.sin(LEG_ANGLE_FROM_VERTICAL_RAD)

    def project(radius_m: float) -> np.ndarray:
        xy = signs * (radius_m / math.sqrt(2.0))
        return np.column_stack((center[0] + fx * xy[:, 0] / center_height_m,
                                center[1] + fy * xy[:, 1] / center_height_m)).astype(np.float32)

    return {
        "mode": "display_only",
        "assumption": "camera_centered_downward_horizontal_platform",
        "platform_px": project(corner_radius),
        "feet_px": project(foot_radius),
        "center_height_m": float(center_height_m),
    }
