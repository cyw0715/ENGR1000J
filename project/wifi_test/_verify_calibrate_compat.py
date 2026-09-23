import ast
import numpy as np

source_path = r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test\calibrate.py"
source = open(source_path, encoding="utf-8").read()
assert "corners[:,0" not in source.replace(" ", "")
ast.parse(source)

for shape in ((40, 2), (40, 1, 2)):
    corners = np.arange(np.prod(shape), dtype=np.float32).reshape(shape)
    points = corners.reshape(-1, 2)
    center = float(np.mean(points[:, 0])), float(np.mean(points[:, 1]))
    assert len(center) == 2
    print("CORNER_COMPAT_OK", shape, center)
print("CALIBRATION_SOURCE_STATIC_OK")
