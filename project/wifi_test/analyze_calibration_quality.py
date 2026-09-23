"""Quality audit for saved OV5647 chessboard calibration views.

Reports board coverage, center spread, rotation/tilt diversity, and per-view
reprojection error. It does not overwrite camera_config.py.
"""
import os
import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.join(ROOT, "calib_frames")
PATTERN = (8, 5)
SQUARE_MM = 20.0

objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM
obj_points, image_points, names = [], [], []
image_size = None
records = []

for name in sorted(f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(".jpg")):
    image = cv2.imread(os.path.join(IMAGE_DIR, name), cv2.IMREAD_GRAYSCALE)
    found, corners = cv2.findChessboardCorners(image, PATTERN, None)
    if not found:
        print("REJECT", name, "no_corners")
        continue
    refined = cv2.cornerSubPix(
        image, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    pts = refined.reshape(-1, 2)
    h, w = image.shape
    x, y, bw, bh = cv2.boundingRect(pts.astype(np.float32))
    image_size = (w, h)
    obj_points.append(objp)
    image_points.append(refined)
    names.append(name)
    records.append({
        "name": name,
        "center_x": float(pts[:, 0].mean()),
        "center_y": float(pts[:, 1].mean()),
        "cover_w": float(bw / w),
        "cover_h": float(bh / h),
        "area": float((bw * bh) / (w * h)),
    })

if len(obj_points) < 10:
    raise RuntimeError(f"Need >=10 valid views; got {len(obj_points)}")

_, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, image_points, image_size, None, None)
errors = []
for i, (obj, observed, rvec, tvec) in enumerate(zip(obj_points, image_points, rvecs, tvecs)):
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = cv2.norm(observed.reshape(-1, 2), projected.reshape(-1, 2).astype(np.float32), cv2.NORM_L2) / len(observed)
    rotation, _ = cv2.Rodrigues(rvec)
    # board normal in camera frame; angle from optical axis shows tilt diversity.
    normal = rotation[:, 2]
    tilt_deg = float(np.degrees(np.arccos(np.clip(abs(normal[2]), -1.0, 1.0))))
    records[i]["error_px"] = float(err)
    records[i]["tilt_deg"] = tilt_deg
    errors.append(float(err))

print("SUMMARY")
print("VALID_VIEWS", len(records))
print("IMAGE_SIZE", image_size)
print("CENTER_X_RANGE", f"{min(r['center_x'] for r in records):.1f}", f"{max(r['center_x'] for r in records):.1f}")
print("CENTER_Y_RANGE", f"{min(r['center_y'] for r in records):.1f}", f"{max(r['center_y'] for r in records):.1f}")
print("BOARD_AREA_FRAC_MIN_MEDIAN_MAX", f"{min(r['area'] for r in records):.3f}", f"{np.median([r['area'] for r in records]):.3f}", f"{max(r['area'] for r in records):.3f}")
print("TILT_DEG_MIN_MEDIAN_MAX", f"{min(r['tilt_deg'] for r in records):.2f}", f"{np.median([r['tilt_deg'] for r in records]):.2f}", f"{max(r['tilt_deg'] for r in records):.2f}")
print("ERROR_PX_MIN_MEDIAN_MEAN_MAX", f"{min(errors):.4f}", f"{np.median(errors):.4f}", f"{np.mean(errors):.4f}", f"{max(errors):.4f}")
print("K", K.tolist())
print("DIST", dist.ravel().tolist())
print("VIEWS")
for row in records:
    print("{name} center=({center_x:.1f},{center_y:.1f}) area={area:.3f} tilt={tilt_deg:.2f}deg err={error_px:.4f}px".format(**row))
