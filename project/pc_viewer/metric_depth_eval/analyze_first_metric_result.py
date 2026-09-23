import numpy as np

path = r"E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\pc_viewer\metric_depth_eval\results\20260723_191012_ov5647_metric_hypersim_base_metric_depth_m.npy"
depth = np.load(path)
h, w = depth.shape
regions = {
    "center": depth[h // 4:3 * h // 4, w // 4:3 * w // 4],
    "left": depth[h // 4:3 * h // 4, :w // 4],
    "right": depth[h // 4:3 * h // 4, 3 * w // 4:],
    "top": depth[:h // 4, w // 4:3 * w // 4],
    "bottom": depth[3 * h // 4:, w // 4:3 * w // 4],
}
print("GLOBAL_STD_M", "%.5f" % depth.std())
for name, values in regions.items():
    print(name, "median=%.5f" % np.median(values), "std=%.5f" % values.std())
