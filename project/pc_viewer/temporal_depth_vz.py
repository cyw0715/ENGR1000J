"""Temporal monocular depth for a vertically descending camera.

Assumptions (must hold for metric output):
  - camera optical axis points approximately downward;
  - horizontal velocity vx = vy = 0;
  - downward speed vz is known and constant (default 50 mm/s);
  - rotation between selected frames is negligible or pre-compensated;
  - camera intrinsics are calibrated. Defaults are only provisional.

For a static scene under forward/downward optical-axis translation, a feature at
pixel radius r from the principal point expands radially by dr.  With a small
known descent baseline B, its range is approximately Z = B * r / dr.

This is a sparse CV estimator, not an AI depth model and not a replacement for
camera calibration. It reports a robust median ground range and marks tracked
features that are significantly closer/farther than that reference.

Input protocol is unchanged from the ESP firmware:
  TCP 5000: [4-byte little-endian JPEG length][JPEG bytes]

Usage:
  python temporal_depth_vz.py <ESP_IP>
  python temporal_depth_vz.py <ESP_IP> --fx 1333 --fy 1778 --cx 400 --cy 400
  python temporal_depth_vz.py --self-test
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

TCP_PORT = 5000
JPEG_MIN_BYTES = 1000
JPEG_MAX_BYTES = 500_000


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class DepthResult:
    baseline_mm: float
    elapsed_s: float
    tracks_total: int
    tracks_valid: int
    ground_depth_mm: Optional[float]
    point_xy: np.ndarray
    point_depth_mm: np.ndarray
    point_residual_mm: np.ndarray
    # Median cosine between measured optical flow and the expected radial
    # direction. Values close to 1 indicate the vx=vy=0 assumption holds.
    radial_alignment: float = float("nan")


def receive_jpeg(sock: socket.socket) -> Optional[np.ndarray]:
    """Read one complete JPEG frame from the unchanged TCP 5000 protocol."""
    header = bytearray()
    while len(header) < 4:
        part = sock.recv(4 - len(header))
        if not part:
            return None
        header.extend(part)

    size = struct.unpack("<I", header)[0]
    if not JPEG_MIN_BYTES <= size <= JPEG_MAX_BYTES:
        raise ValueError(f"invalid JPEG length: {size}")

    payload = bytearray()
    while len(payload) < size:
        part = sock.recv(min(65_536, size - len(payload)))
        if not part:
            return None
        payload.extend(part)

    if payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
        raise ValueError("payload is not a complete JPEG")

    frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if frame is None:
        raise ValueError("OpenCV could not decode JPEG")
    return frame


def estimate_depth_from_tracks(
    previous_gray: np.ndarray,
    current_gray: np.ndarray,
    camera: CameraModel,
    baseline_mm: float,
    max_lateral_px: float = 6.0,
    max_rotation_deg: float = 0.8,
) -> DepthResult:
    """Estimate sparse per-feature range using radial LK optical flow.

    Positive radial flow means the feature expands away from the principal point
    as a downward-facing camera approaches a static surface. Only such features
    are accepted. The exact finite-baseline formula is used:

        Z1 = B * r2 / (r2 - r1)

    where r1/r2 are feature radii in pixels before/after descent B.
    """
    empty = DepthResult(
        baseline_mm=baseline_mm,
        elapsed_s=0.0,
        tracks_total=0,
        tracks_valid=0,
        ground_depth_mm=None,
        point_xy=np.empty((0, 2), dtype=np.float32),
        point_depth_mm=np.empty(0, dtype=np.float32),
        point_residual_mm=np.empty(0, dtype=np.float32),
    )
    if baseline_mm <= 0:
        return empty

    previous = cv2.goodFeaturesToTrack(
        previous_gray,
        maxCorners=700,
        qualityLevel=0.01,
        minDistance=8,
        blockSize=7,
    )
    if previous is None or len(previous) < 20:
        return empty

    current, status, error = cv2.calcOpticalFlowPyrLK(
        previous_gray,
        current_gray,
        previous,
        None,
        winSize=(21, 21),
        maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if current is None or status is None or error is None:
        return empty

    p1 = previous.reshape(-1, 2)
    p2 = current.reshape(-1, 2)
    tracked = status.reshape(-1).astype(bool)
    lk_error = error.reshape(-1)
    center = np.array([camera.cx, camera.cy], dtype=np.float32)

    # Fit the dominant image motion with RANSAC. For the allowed small lateral
    # drift model, p2 ≈ scale*p1 + translation; translation is removed before
    # radial depth recovery, while substantial rotation or drift is rejected.
    fit_quality = tracked & (lk_error < 20.0)
    if np.count_nonzero(fit_quality) < 20:
        return empty
    affine, affine_inliers = cv2.estimateAffinePartial2D(
        p1[fit_quality], p2[fit_quality], method=cv2.RANSAC,
        ransacReprojThreshold=1.5, maxIters=2000, confidence=0.99,
    )
    if affine is None:
        return empty
    rotation_deg = abs(math.degrees(math.atan2(affine[1, 0], affine[0, 0])))
    scale = float(math.hypot(affine[0, 0], affine[1, 0]))
    affine_translation = affine[:, 2].astype(np.float32)
    # Scale about the principal point itself produces affine translation
    # center*(1-scale); remove that expected term before judging lateral drift.
    lateral_translation = affine_translation - center * (1.0 - scale)
    lateral_translation_px = float(np.linalg.norm(lateral_translation))
    if rotation_deg > max_rotation_deg or lateral_translation_px > max_lateral_px:
        return empty

    # Compensate the fitted small lateral image shift. Keep scale untouched:
    # the residual radial expansion contains the desired depth information.
    p2_comp = p2 - lateral_translation
    v1 = p1 - center
    v2 = p2_comp - center
    r1 = np.linalg.norm(v1, axis=1)
    r2 = np.linalg.norm(v2, axis=1)
    radial_delta = r2 - r1
    flow = p2_comp - p1
    flow_norm = np.linalg.norm(flow, axis=1)
    radial_unit = np.divide(v1, r1[:, None], out=np.zeros_like(v1), where=r1[:, None] > 1e-6)
    flow_unit = np.divide(flow, flow_norm[:, None], out=np.zeros_like(flow), where=flow_norm[:, None] > 1e-6)
    alignment = np.sum(radial_unit * flow_unit, axis=1)

    # Reject image-centre features (depth is ill-conditioned), bad LK tracks,
    # inward/noisy motion, huge jumps, and physically implausible depth.
    valid = (
        tracked
        & (lk_error < 20.0)
        & (r1 > 25.0)
        & (radial_delta > 0.15)
        & (radial_delta < 100.0)
        & (alignment > 0.85)
    )
    if not np.any(valid):
        return empty

    z1 = baseline_mm * r2[valid] / radial_delta[valid]
    finite = np.isfinite(z1) & (z1 >= 100.0) & (z1 <= 10_000.0)
    points = p2[valid][finite]
    depths = z1[finite].astype(np.float32)
    if len(depths) < 12:
        return empty

    # Robustly reject non-ground points for an initial scene-range estimate.
    median = float(np.median(depths))
    mad = float(np.median(np.abs(depths - median)))
    robust_sigma = max(1.4826 * mad, 20.0)
    inlier = np.abs(depths - median) <= 3.0 * robust_sigma
    if np.count_nonzero(inlier) < 12:
        return empty

    ground = float(np.median(depths[inlier]))
    residual = depths - ground
    return DepthResult(
        baseline_mm=baseline_mm,
        elapsed_s=0.0,
        tracks_total=len(p1),
        tracks_valid=len(depths),
        ground_depth_mm=ground,
        point_xy=points,
        point_depth_mm=depths,
        point_residual_mm=residual,
        radial_alignment=float(np.median(alignment[valid][finite])),
    )


def draw_overlay(gray: np.ndarray, result: DepthResult) -> np.ndarray:
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if result.ground_depth_mm is None:
        cv2.putText(image, "Depth: waiting for valid radial tracks", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 180, 255), 2)
        return image

    for (x, y), residual in zip(result.point_xy, result.point_residual_mm):
        # Nearer than ground = potential bump/obstacle (red); farther = pit (blue).
        if residual < -50.0:
            color = (0, 0, 255)
        elif residual > 50.0:
            color = (255, 150, 0)
        else:
            color = (0, 220, 0)
        cv2.circle(image, (int(x), int(y)), 2, color, -1)

    text = (
        f"Zground={result.ground_depth_mm:.0f}mm "
        f"B={result.baseline_mm:.1f}mm tracks={result.tracks_valid}/{result.tracks_total}"
    )
    cv2.putText(image, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 0), 2)
    cv2.putText(image, "red: nearer/bump  blue: farther/pit  green: ground", (12, 54),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
    return image


def _synthetic_textured_plane(
    seed: int,
    size: int = 800,
    count: int = 900,
    texture_sigma: float = 14.0,
) -> np.ndarray:
    """Generate repeatable texture with corners suitable for LK optical flow."""
    rng = np.random.default_rng(seed)
    image = rng.normal(90.0, texture_sigma, (size, size)).clip(0, 255).astype(np.uint8)
    for _ in range(count):
        x, y = rng.integers(8, size - 8, size=2)
        radius = int(rng.integers(1, 5))
        color = int(rng.integers(130, 255))
        cv2.circle(image, (int(x), int(y)), radius, color, -1)
    return image


def _forward_warp(image: np.ndarray, scale: float, center: tuple[float, float],
                  tx_px: float = 0.0, ty_px: float = 0.0, blur_sigma: float = 0.0,
                  jpeg_quality: Optional[int] = None, occlude: bool = False) -> np.ndarray:
    """Create a later frame under forward descent plus optional adverse effects."""
    cx, cy = center
    matrix = np.array([[scale, 0.0, cx * (1.0 - scale) + tx_px],
                       [0.0, scale, cy * (1.0 - scale) + ty_px]], dtype=np.float32)
    warped = cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]),
                            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    if blur_sigma > 0:
        warped = cv2.GaussianBlur(warped, (0, 0), blur_sigma)
    if jpeg_quality is not None:
        ok, encoded = cv2.imencode('.jpg', warped, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            raise RuntimeError('synthetic JPEG encode failed')
        warped = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    if occlude:
        cv2.rectangle(warped, (60, 80), (350, 410), 0, -1)
        cv2.rectangle(warped, (500, 460), (760, 730), 210, -1)
    return warped


def _run_image_case(name: str, previous: np.ndarray, current: np.ndarray,
                    camera: CameraModel, baseline_mm: float, true_z_mm: float,
                    must_detect: bool, max_error_mm: float = 150.0) -> bool:
    result = estimate_depth_from_tracks(previous, current, camera, baseline_mm)
    if not must_detect:
        passed = result.ground_depth_mm is None
        print(f"STRESS {name}: {'PASS' if passed else 'FAIL'} "
              f"expected_reject=true detected={result.ground_depth_mm is not None}")
        return passed

    if result.ground_depth_mm is None:
        print(f"STRESS {name}: FAIL no valid depth")
        return False
    error_mm = abs(result.ground_depth_mm - true_z_mm)
    passed = error_mm <= max_error_mm
    print(f"STRESS {name}: {'PASS' if passed else 'FAIL'} "
          f"Z={result.ground_depth_mm:.1f}mm true={true_z_mm:.1f}mm "
          f"error={error_mm:.1f}mm tracks={result.tracks_valid}/{result.tracks_total} "
          f"alignment={result.radial_alignment:.3f}")
    return passed


def stress_test() -> None:
    """Run deliberately harsh, repeatable OpenCV end-to-end test cases.

    These use actual corner detection and LK flow, unlike the formula-only
    self-test. A pass means either an accurate robust estimate or a safe reject
    when the motion model is deliberately violated.
    """
    camera = CameraModel(fx=1333.0, fy=1778.0, cx=400.0, cy=400.0)
    true_z_mm = 1500.0
    baseline_mm = 50.0  # exactly one second at the required 50 mm/s descent
    scale = true_z_mm / (true_z_mm - baseline_mm)
    base = _synthetic_textured_plane(seed=41)

    results = []
    results.append(_run_image_case(
        'ideal_forward', base, _forward_warp(base, scale, (camera.cx, camera.cy)),
        camera, baseline_mm, true_z_mm, must_detect=True, max_error_mm=80.0))
    results.append(_run_image_case(
        'jpeg_blur_noise', base, _forward_warp(base, scale, (camera.cx, camera.cy),
                                                blur_sigma=1.2, jpeg_quality=35),
        camera, baseline_mm, true_z_mm, must_detect=True, max_error_mm=180.0))
    results.append(_run_image_case(
        'partial_occlusion', base, _forward_warp(base, scale, (camera.cx, camera.cy),
                                                   blur_sigma=0.8, jpeg_quality=45,
                                                   occlude=True),
        camera, baseline_mm, true_z_mm, must_detect=True, max_error_mm=220.0))

    # A blank scene has no corners: it must safely decline to estimate depth.
    blank = np.full((800, 800), 128, dtype=np.uint8)
    results.append(_run_image_case('no_texture_safe_reject', blank, blank, camera,
                                   baseline_mm, true_z_mm, must_detect=False))

    results.append(_run_image_case(
        'small_lateral_drift_compensated',
        base, _forward_warp(base, scale, (camera.cx, camera.cy), tx_px=4.0, ty_px=-3.0),
        camera, baseline_mm, true_z_mm, must_detect=True, max_error_mm=180.0))

    # Deliberate 35px lateral shift is beyond the permitted small-drift bound.
    # It must be rejected instead of reporting a plausible-looking depth.
    lateral = _forward_warp(base, scale, (camera.cx, camera.cy), tx_px=35.0)
    results.append(_run_image_case('large_lateral_motion_safe_reject', base, lateral, camera,
                                   baseline_mm, true_z_mm, must_detect=False))

    passed = sum(results)
    print(f"STRESS_SUMMARY {passed}/{len(results)} cases passed")
    if passed != len(results):
        raise SystemExit('stress test failed')


def self_test() -> None:
    """Synthetic geometry test: validates the finite-baseline depth formula."""
    rng = np.random.default_rng(7)
    true_z_mm = 1500.0
    baseline_mm = 50.0  # 1 second at 50 mm/s
    center = np.array([400.0, 400.0])
    p1 = center + rng.uniform(-300, 300, size=(200, 2))
    # For forward motion: p2 = c + (p1-c) * Z1/(Z1-B).
    p2 = center + (p1 - center) * true_z_mm / (true_z_mm - baseline_mm)
    p2 += rng.normal(0.0, 0.08, size=p2.shape)

    r1 = np.linalg.norm(p1 - center, axis=1)
    r2 = np.linalg.norm(p2 - center, axis=1)
    estimate = baseline_mm * r2 / (r2 - r1)
    median = float(np.median(estimate))
    error_mm = abs(median - true_z_mm)
    print(f"SELF_TEST true_z_mm={true_z_mm:.1f} estimated_z_mm={median:.2f} error_mm={error_mm:.2f}")
    if error_mm > 10.0:
        raise SystemExit("self-test failed")


def run_stream(esp_ip: str, camera: CameraModel, descent_speed_mm_s: float) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(8)
    sock.connect((esp_ip, TCP_PORT))
    print(f"Connected to {esp_ip}:{TCP_PORT}")
    print("Assumption: vx=vy=0, downward speed=" f"{descent_speed_mm_s:.1f} mm/s")
    print("Press q in the OpenCV window to stop.")

    previous_gray: Optional[np.ndarray] = None
    previous_t: Optional[float] = None
    frame_index = 0
    try:
        while True:
            gray = receive_jpeg(sock)
            if gray is None:
                print("Video connection closed")
                break
            now = time.monotonic()
            frame_index += 1

            if previous_gray is None:
                previous_gray, previous_t = gray, now
                continue

            elapsed_s = now - previous_t
            baseline_mm = descent_speed_mm_s * elapsed_s
            result = estimate_depth_from_tracks(previous_gray, gray, camera, baseline_mm)
            result.elapsed_s = elapsed_s
            overlay = draw_overlay(gray, result)
            if result.ground_depth_mm is not None:
                print(
                    f"[DEPTH] frame={frame_index} dt={elapsed_s:.3f}s "
                    f"B={baseline_mm:.1f}mm Zground={result.ground_depth_mm:.0f}mm "
                    f"tracks={result.tracks_valid}/{result.tracks_total}"
                )

            cv2.imshow("Temporal depth: vertical descent", overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            previous_gray, previous_t = gray, now
    finally:
        sock.close()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("esp_ip", nargs="?", help="ESP32-P4 DHCP IP")
    parser.add_argument("--speed-mm-s", type=float, default=50.0,
                        help="known downward speed; default 50 mm/s")
    # These are provisional values from previous lens estimates. Replace them
    # with the output of calibrate.py before using metric results for control.
    parser.add_argument("--fx", type=float, default=1333.0)
    parser.add_argument("--fy", type=float, default=1778.0)
    parser.add_argument("--cx", type=float, default=400.0)
    parser.add_argument("--cy", type=float, default=400.0)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--stress-test", action="store_true",
                        help="run adverse synthetic OpenCV/LK test suite")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.self_test:
        self_test()
    elif args.stress_test:
        stress_test()
    elif not args.esp_ip:
        raise SystemExit("ESP_IP is required unless --self-test is used")
    elif args.speed_mm_s <= 0:
        raise SystemExit("--speed-mm-s must be positive")
    else:
        run_stream(args.esp_ip, CameraModel(args.fx, args.fy, args.cx, args.cy), args.speed_mm_s)
