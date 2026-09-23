import time
import numpy as np
import torch
from PIL import Image
from transformers import pipeline

assert torch.cuda.is_available(), "CUDA unavailable"
print("GPU", torch.cuda.get_device_name(0), flush=True)
model = pipeline(
    "depth-estimation",
    model="depth-anything/Depth-Anything-V2-Base-hf",
    device="cuda",
    dtype=torch.float16,
)
image = Image.fromarray(np.zeros((800, 800, 3), dtype=np.uint8))
model(image)  # warmup
start = time.perf_counter()
output = model(image)
elapsed_ms = (time.perf_counter() - start) * 1000.0
depth = output["predicted_depth"]
print(
    "BASE_OK",
    "depth=", tuple(depth.shape),
    "time_ms=%.1f" % elapsed_ms,
    "fps=%.1f" % (1000.0 / elapsed_ms),
    "vram_mb=%.0f" % (torch.cuda.max_memory_allocated() / 1024 / 1024),
    flush=True,
)
