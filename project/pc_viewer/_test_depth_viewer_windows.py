import sys
import numpy as np

sys.path.insert(0, r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\pc_viewer")
import depth_viewer as d

gray = np.tile(np.arange(800, dtype=np.uint8), (800, 1))
depth = np.linspace(0.1, 1.0, 800 * 800, dtype=np.float32).reshape(800, 800)
depth_range = d.stable_depth_range(depth, None)
canvas, source, heatmap, overlay = d.compose_display(
    gray, depth, depth_range, 1, 8.3, 55.0, 17018, 0
)
assert canvas.shape[0] > source.shape[0]
assert heatmap.shape == source.shape == overlay.shape
assert canvas.shape[1] == source.shape[1] * 3 + 8
print("VISUAL_COMPOSE_OK", canvas.shape, depth_range)
