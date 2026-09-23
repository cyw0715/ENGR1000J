"""Compare calibration stability with/without high-error views; read-only audit."""
import os
import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
DIR = os.path.join(ROOT, "calib_frames")
PATTERN = (8, 5)
SQUARE_MM = 20.0
obj = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
obj[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM

rows = []
for name in sorted(x for x in os.listdir(DIR) if x.endswith(".jpg")):
    image = cv2.imread(os.path.join(DIR, name), cv2.IMREAD_GRAYSCALE)
    found, corners = cv2.findChessboardCorners(image, PATTERN, None)
    if not found:
        continue
    refined = cv2.cornerSubPix(image, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.001))
    rows.append((name, obj, refined, image.shape[::-1]))


def solve(selected):
    objects = [x[1] for x in selected]
    points = [x[2] for x in selected]
    _, k, d, rvecs, tvecs = cv2.calibrateCamera(objects, points, selected[0][3], None, None)
    errors = []
    for o, p, r, t in zip(objects, points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, k, d)
        errors.append(cv2.norm(p.reshape(-1,2), proj.reshape(-1,2).astype(np.float32), cv2.NORM_L2) / len(p))
    return k, d.ravel(), np.array(errors)

for label, excluded in (("all", set()), ("drop_006", {"calib_006.jpg"}), ("drop_top2", {"calib_006.jpg", "calib_001.jpg"})):
    selected = [r for r in rows if r[0] not in excluded]
    K, dist, errors = solve(selected)
    print(label, "views", len(selected))
    print("  fx_fy_cx_cy", *[f"{v:.3f}" for v in (K[0,0],K[1,1],K[0,2],K[1,2])])
    print("  dist", " ".join(f"{v:.5g}" for v in dist))
    print("  err_min_median_mean_max", *[f"{v:.4f}" for v in (errors.min(),np.median(errors),errors.mean(),errors.max())])
