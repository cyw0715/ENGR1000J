import numpy as np

corners = np.zeros((40, 2), dtype=np.float32)
corners[:, 0] = np.arange(40)
corners[:, 1] = np.arange(40) * 2
points = corners.reshape(-1, 2)
cx, cy = np.mean(points[:, 0]), np.mean(points[:, 1])
assert (cx, cy) == (19.5, 39.0)
print("CORNER_SHAPE_COMPAT_OK", corners.shape, cx, cy)
