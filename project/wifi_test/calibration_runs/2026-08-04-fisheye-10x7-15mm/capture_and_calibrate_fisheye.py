"""Interactive TCP5000 fisheye chessboard capture and audited calibration.

Board: calib.io 200x150 mm page, 11x8 squares, 10x7 inner corners,
15.0 mm square. Source protocol: [u32_le jpeg_len][jpeg].
"""
from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import socket
import struct
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

PATTERN = (10, 7)
SQUARE_MM = 15.0
MIN_VIEWS = 20
AUTO_TARGET_VIEWS = 36
AUTO_STABLE_FRAMES = 2
AUTO_MIN_SAVE_INTERVAL_S = 0.5
AUTO_MIN_CENTER_SHIFT_FRAC = 0.035
AUTO_MIN_AREA_CHANGE = 0.015
AUTO_MIN_ANGLE_CHANGE_DEG = 4.0
AUTO_MIN_SHAPE_CHANGE_FRAC = 0.020
AUTO_MAX_STABLE_CORNER_MOTION_PX = 3.0
AUTO_MIN_BOARD_LAPLACIAN_VAR = 20.0
VIDEO_PORT = 5000
ROOT = Path(__file__).resolve().parent
VIEWS = ROOT / "views"
RESULT = ROOT / "fisheye_calibration.json"


def discover_p4() -> str:
    def probe(host: str):
        try:
            with urllib.request.urlopen(f"http://{host}/board", timeout=.45) as response:
                board = json.load(response)
            if board.get("board") == "ESP32-P4" and board.get("ip") == host:
                return host
        except Exception:
            return None
    hosts = [f"192.168.137.{last}" for last in range(1, 255)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as pool:
        matches = [host for host in pool.map(probe, hosts) if host]
    if not matches:
        raise RuntimeError("no ESP32-P4 found on 192.168.137.1-254")
    print("DISCOVERED", matches[0])
    return matches[0]


def recv_exact(sock: socket.socket, count: int) -> bytes:
    out = bytearray()
    while len(out) < count:
        chunk = sock.recv(count - len(out))
        if not chunk:
            raise ConnectionError("video socket closed")
        out.extend(chunk)
    return bytes(out)


def receive_frame(sock: socket.socket) -> np.ndarray:
    size = struct.unpack("<I", recv_exact(sock, 4))[0]
    if not 1000 <= size <= 300000:
        raise ValueError(f"invalid JPEG size {size}")
    payload = recv_exact(sock, size)
    if payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
        raise ValueError("invalid JPEG markers")
    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("JPEG decode failed")
    return image


def detect(image: np.ndarray):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(image, PATTERN, flags)
    if not found:
        found, corners = cv2.findChessboardCornersSB(image, PATTERN, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    return cv2.cornerSubPix(
        image, corners.astype(np.float32), (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4),
    ).reshape(-1, 1, 2)


def view_metrics(corners: np.ndarray, shape: tuple[int, int]):
    pts = corners.reshape(-1, 2)
    height, width = shape
    hull = cv2.convexHull(pts.astype(np.float32))
    area_fraction = float(cv2.contourArea(hull) / (width * height))
    center = pts.mean(axis=0)
    span = pts.max(axis=0) - pts.min(axis=0)
    return {
        "center_x": float(center[0]), "center_y": float(center[1]),
        "area_fraction": area_fraction,
        "span_x_fraction": float(span[0] / width),
        "span_y_fraction": float(span[1] / height),
    }


def view_signature(corners: np.ndarray, shape: tuple[int, int]) -> dict:
    pts = corners.reshape(PATTERN[1], PATTERN[0], 2)
    height, width = shape
    flat = pts.reshape(-1, 2)
    center = flat.mean(axis=0)
    hull = cv2.convexHull(flat.astype(np.float32))
    area = float(cv2.contourArea(hull) / (width * height))
    horizontal = pts[:, -1, :].mean(axis=0) - pts[:, 0, :].mean(axis=0)
    angle = math.degrees(math.atan2(float(horizontal[1]), float(horizontal[0])))
    corners4 = np.array([pts[0, 0], pts[0, -1], pts[-1, -1], pts[-1, 0]], np.float64)
    corners4[:, 0] /= width
    corners4[:, 1] /= height
    return {
        "center": np.array([center[0] / width, center[1] / height]),
        "area": area, "angle": angle, "corners": corners4,
    }


def angle_difference_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def is_novel_view(signature: dict, saved_signatures: list[dict]) -> bool:
    if not saved_signatures:
        return True
    # Reject only when the candidate is too similar to an existing view in all
    # dimensions. A meaningful difference in any one dimension is useful.
    for old in saved_signatures:
        center_shift = float(np.linalg.norm(signature["center"] - old["center"]))
        area_change = abs(signature["area"] - old["area"])
        angle_change = angle_difference_deg(signature["angle"], old["angle"])
        shape_change = float(np.sqrt(np.mean((signature["corners"] - old["corners"]) ** 2)))
        if (center_shift < AUTO_MIN_CENTER_SHIFT_FRAC and
                area_change < AUTO_MIN_AREA_CHANGE and
                angle_change < AUTO_MIN_ANGLE_CHANGE_DEG and
                shape_change < AUTO_MIN_SHAPE_CHANGE_FRAC):
            return False
    return True


def board_sharpness(image: np.ndarray, corners: np.ndarray) -> float:
    pts = corners.reshape(-1, 2)
    x0, y0 = np.floor(pts.min(axis=0)).astype(int)
    x1, y1 = np.ceil(pts.max(axis=0)).astype(int)
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(image.shape[1], x1 + 1), min(image.shape[0], y1 + 1)
    roi = image[y0:y1, x0:x1]
    return 0.0 if roi.size == 0 else float(cv2.Laplacian(roi, cv2.CV_32F).var())


def object_points() -> np.ndarray:
    obj = np.zeros((1, PATTERN[0] * PATTERN[1], 3), np.float64)
    obj[0, :, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM
    return obj


def reprojection_errors(obj, image_points, rvecs, tvecs, K, D):
    errors = []
    for img, rv, tv in zip(image_points, rvecs, tvecs):
        projected, _ = cv2.fisheye.projectPoints(obj, rv, tv, K, D)
        errors.append(float(np.sqrt(np.mean(np.sum((img.reshape(-1, 2) - projected.reshape(-1, 2)) ** 2, axis=1)))))
    return errors


def _calibrate(objects, images, image_size):
    K = np.array([[400.0, 0.0, image_size[0] / 2], [0.0, 400.0, image_size[1] / 2], [0.0, 0.0, 1.0]], np.float64)
    D = np.zeros((4, 1), np.float64)
    flags = cv2.CALIB_RECOMPUTE_EXTRINSIC | cv2.CALIB_CHECK_COND | cv2.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8)
    return cv2.fisheye.calibrate(objects, images, image_size, K, D, None, None, flags, criteria)


def jackknife_stability(objects, images, image_size, full_K, full_D):
    """Leave one view out; unstable intrinsics reject an apparently low RMS set."""
    samples = []
    for omitted in range(len(objects)):
        subset_objects = objects[:omitted] + objects[omitted + 1:]
        subset_images = images[:omitted] + images[omitted + 1:]
        try:
            rms, K, D, _rvecs, _tvecs = _calibrate(subset_objects, subset_images, image_size)
        except cv2.error:
            return {"completed": False, "failed_omission": omitted}
        samples.append({
            "omitted": omitted, "rms": float(rms),
            "fx_relative_change": float(abs(K[0, 0] - full_K[0, 0]) / full_K[0, 0]),
            "fy_relative_change": float(abs(K[1, 1] - full_K[1, 1]) / full_K[1, 1]),
            "principal_point_change_px": float(math.hypot(K[0, 2] - full_K[0, 2], K[1, 2] - full_K[1, 2])),
            "distortion_l2_change": float(np.linalg.norm(D.reshape(-1) - full_D.reshape(-1))),
        })
    return {
        "completed": True, "samples": samples,
        "max_focal_relative_change": max(max(s["fx_relative_change"], s["fy_relative_change"]) for s in samples),
        "max_principal_point_change_px": max(s["principal_point_change_px"] for s in samples),
        "max_distortion_l2_change": max(s["distortion_l2_change"] for s in samples),
    }


def solve(files: list[Path]):
    obj = object_points()
    objects, images, records = [], [], []
    image_size = None
    for path in files:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        corners = detect(image)
        if corners is None:
            continue
        image_size = (image.shape[1], image.shape[0])
        objects.append(obj.copy())
        images.append(corners.astype(np.float64).reshape(1, -1, 2))
        records.append({"file": path.name, **view_metrics(corners, image.shape)})
    if len(objects) < MIN_VIEWS:
        raise RuntimeError(f"need at least {MIN_VIEWS} detected views; got {len(objects)}")
    rms, K, D, rvecs, tvecs = _calibrate(objects, images, image_size)
    stability = jackknife_stability(objects, images, image_size, K, D)
    errors = reprojection_errors(obj, images, rvecs, tvecs, K, D)
    centers_x = [r["center_x"] for r in records]
    centers_y = [r["center_y"] for r in records]
    areas = [r["area_fraction"] for r in records]
    gates = {
        "views_at_least_20": len(records) >= 20,
        "mean_reprojection_below_1px": float(np.mean(errors)) < 1.0,
        "max_reprojection_below_2px": max(errors) < 2.0,
        "center_x_coverage_at_least_45pct": (max(centers_x) - min(centers_x)) / image_size[0] >= 0.45,
        "center_y_coverage_at_least_45pct": (max(centers_y) - min(centers_y)) / image_size[1] >= 0.45,
        "large_board_view_present": max(areas) >= 0.18,
        "principal_point_inside_image": bool(
            0 <= K[0, 2] < image_size[0] and 0 <= K[1, 2] < image_size[1]
        ),
        "jackknife_completed": bool(stability.get("completed")),
        "jackknife_focal_change_below_3pct": bool(
            stability.get("completed") and stability["max_focal_relative_change"] < 0.03
        ),
        "jackknife_principal_point_change_below_10px": bool(
            stability.get("completed") and stability["max_principal_point_change_px"] < 10.0
        ),
    }
    report = {
        "schema": "mars-lander-fisheye-calibration-v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "board": {"squares": [11, 8], "inner_corners": list(PATTERN), "square_mm": SQUARE_MM,
                  "page_mm": [200, 150], "source_pdf": "calib.io_checker_200x150_8x11_15.pdf"},
        "projection_model": "opencv_fisheye",
        "image_size": list(image_size), "valid_views": len(records),
        "rms": float(rms), "mean_reprojection_error_px": float(np.mean(errors)),
        "median_reprojection_error_px": float(np.median(errors)), "max_reprojection_error_px": max(errors),
        "K": K.tolist(), "D": D.reshape(-1).tolist(), "per_view_errors_px": errors,
        "jackknife_stability": stability,
        "view_records": records, "gates": gates, "accepted_for_geometry": all(gates.values()),
    }
    RESULT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("valid_views", "rms", "mean_reprojection_error_px", "max_reprojection_error_px", "gates", "accepted_for_geometry")}, indent=2))
    print("RESULT", RESULT)
    return report


def capture(ip: str):
    VIEWS.mkdir(parents=True, exist_ok=True)
    existing = sorted(VIEWS.glob("view_*.jpg"))
    saved = len(existing)
    saved_signatures = []
    for path in existing:
        old_image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        old_corners = None if old_image is None else detect(old_image)
        if old_corners is not None:
            saved_signatures.append(view_signature(old_corners, old_image.shape))
    auto = True
    stable_history: list[np.ndarray] = []
    last_save = 0.0
    sock = socket.create_connection((ip, VIDEO_PORT), timeout=5)
    sock.settimeout(8)
    print(f"CONNECTED tcp://{ip}:{VIDEO_PORT}; saved={saved}; AUTO=ON; A=toggle SPACE=save Q/ESC=finish", flush=True)

    def save_view(image, corners, reason):
        nonlocal saved, last_save
        saved += 1
        path = VIEWS / f"view_{saved:03d}.jpg"
        cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 100])
        signature = view_signature(corners, image.shape)
        saved_signatures.append(signature)
        last_save = time.monotonic()
        print("SAVED", path.name, reason, view_metrics(corners, image.shape), flush=True)

    try:
        while True:
            image = receive_frame(sock)
            corners = detect(image)
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            status = ""
            color = (0, 0, 255)
            if corners is not None:
                cv2.drawChessboardCorners(display, PATTERN, corners, True)
                signature = view_signature(corners, image.shape)
                sharpness = board_sharpness(image, corners)
                novel = is_novel_view(signature, saved_signatures)
                stable_history.append(corners.reshape(-1, 2).copy())
                stable_history = stable_history[-AUTO_STABLE_FRAMES:]
                stable = False
                motion = float("inf")
                if len(stable_history) == AUTO_STABLE_FRAMES:
                    motion = max(float(np.mean(np.linalg.norm(stable_history[i] - stable_history[i - 1], axis=1)))
                                 for i in range(1, len(stable_history)))
                    stable = motion <= AUTO_MAX_STABLE_CORNER_MOTION_PX
                ready = (novel and stable and sharpness >= AUTO_MIN_BOARD_LAPLACIAN_VAR and
                         time.monotonic() - last_save >= AUTO_MIN_SAVE_INTERVAL_S)
                status = (f"DETECTED AUTO={'ON' if auto else 'OFF'} saved={saved}/{AUTO_TARGET_VIEWS} "
                          f"sharp={sharpness:.0f} motion={motion:.1f} "
                          f"{'READY' if ready else ('MOVE TO NEW VIEW' if not novel else 'HOLD STILL')}")
                color = (0, 255, 0) if ready else (0, 210, 255)
                if auto and ready:
                    save_view(image, corners, "AUTO")
                    stable_history.clear()
                    if saved >= AUTO_TARGET_VIEWS:
                        print("AUTO_TARGET_REACHED", saved, flush=True)
                        break
            else:
                stable_history.clear()
                status = f"NO 10x7 CORNERS AUTO={'ON' if auto else 'OFF'} saved={saved}/{AUTO_TARGET_VIEWS}"
            cv2.putText(display, status, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .48, color, 1, cv2.LINE_AA)
            cv2.putText(display, "Move board across center/corners; vary distance and tilt", (10, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow("Mars Lander Fisheye Calibration", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("a"):
                auto = not auto
                print("AUTO", "ON" if auto else "OFF", flush=True)
            if key == ord(" ") and corners is not None:
                save_view(image, corners, "MANUAL")
                stable_history.clear()
    finally:
        sock.close(); cv2.destroyAllWindows()
    return solve(sorted(VIEWS.glob("view_*.jpg")))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", nargs="?", help="P4 IP; omit with --solve-only")
    parser.add_argument("--solve-only", action="store_true")
    args = parser.parse_args()
    if args.solve_only:
        solve(sorted(VIEWS.glob("view_*.jpg")))
    elif args.ip:
        capture(args.ip)
    else:
        capture(discover_p4())


if __name__ == "__main__":
    main()
