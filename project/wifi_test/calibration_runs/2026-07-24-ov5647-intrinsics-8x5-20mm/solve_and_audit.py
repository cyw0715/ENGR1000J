"""Solve and audit an isolated OV5647 chessboard calibration run.

Writes candidate_camera_calibration.json only when quality gates pass; it never
replaces the rejected historical camera_config.py.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

PATTERN = (8, 5)
SQUARE_MM = 20.0
MIN_VIEWS = 20


def reproj_error(obj, observed, rvec, tvec, K, dist) -> float:
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    return float(cv2.norm(observed.reshape(-1, 2), projected.reshape(-1, 2).astype(np.float32), cv2.NORM_L2) / len(observed))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    files = sorted(args.run_dir.glob("view_*.png"))
    obj = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM
    objects, image_points, records = [], [], []
    image_size = None
    for path in files:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        found, corners = cv2.findChessboardCorners(image, PATTERN, None)
        if not found:
            print(f"REJECT {path.name}: no_corners")
            continue
        refined = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001))
        points = refined.reshape(-1, 2)
        h, w = image.shape
        x, y, bw, bh = cv2.boundingRect(points.astype(np.float32))
        objects.append(obj)
        image_points.append(refined)
        image_size = (w, h)
        records.append({"file": path.name, "center_x": float(points[:, 0].mean()), "center_y": float(points[:, 1].mean()),
                        "area_fraction": float((bw * bh) / (w * h))})
    if len(objects) < MIN_VIEWS:
        raise RuntimeError(f"Need {MIN_VIEWS} valid views; got {len(objects)}")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(objects, image_points, image_size, None, None)
    errors, tilts = [], []
    for rec, objp, observed, rvec, tvec in zip(records, objects, image_points, rvecs, tvecs):
        err = reproj_error(objp, observed, rvec, tvec, K, dist)
        rotation, _ = cv2.Rodrigues(rvec)
        normal = rotation[:, 2]
        tilt = float(np.degrees(np.arccos(np.clip(abs(normal[2]), -1, 1))))
        rec.update(error_px=err, tilt_deg=tilt)
        errors.append(err); tilts.append(tilt)

    centers_x, centers_y = [r["center_x"] for r in records], [r["center_y"] for r in records]
    areas = [r["area_fraction"] for r in records]
    gates = {
        "views_ge_20": len(records) >= 20,
        "mean_reprojection_error_lt_0_5px": float(np.mean(errors)) < 0.5,
        "max_reprojection_error_lt_1_0px": max(errors) < 1.0,
        "board_area_max_ge_0_30": max(areas) >= 0.30,
        "board_area_min_le_0_20": min(areas) <= 0.20,
        "center_x_span_ge_0_40_width": (max(centers_x) - min(centers_x)) >= 0.40 * image_size[0],
        "center_y_span_ge_0_40_height": (max(centers_y) - min(centers_y)) >= 0.40 * image_size[1],
        "tilt_span_ge_25deg": (max(tilts) - min(tilts)) >= 25.0,
        "at_least_4_near_frontal_le_15deg": sum(t <= 15 for t in tilts) >= 4,
    }
    accepted = all(gates.values())
    report = {
        "accepted_for_candidate_use": accepted,
        "gates": gates,
        "board": {"squares": [9, 6], "inner_corners": list(PATTERN), "square_mm": SQUARE_MM},
        "image_size": image_size,
        "rms": float(rms),
        "mean_reprojection_error_px": float(np.mean(errors)),
        "max_reprojection_error_px": max(errors),
        "K": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "views": records,
    }
    report_path = args.run_dir / "calibration_audit.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"AUDIT_FILE={report_path}")
    print(f"ACCEPTED_FOR_CANDIDATE_USE={accepted}")
    print(f"RMS={rms:.4f} MEAN_ERROR_PX={np.mean(errors):.4f} MAX_ERROR_PX={max(errors):.4f}")
    print("GATES=" + json.dumps(gates, separators=(",", ":")))
    if accepted:
        candidate = args.run_dir / "candidate_camera_calibration.json"
        candidate.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"CANDIDATE_FILE={candidate}")
    else:
        print("NO_CANDIDATE_WRITTEN: collect additional diverse views")


if __name__ == "__main__":
    main()
