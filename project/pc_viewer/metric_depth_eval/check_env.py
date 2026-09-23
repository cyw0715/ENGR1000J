import importlib.util
for module in ("torch", "torchvision", "cv2", "matplotlib", "numpy"):
    print(module, bool(importlib.util.find_spec(module)))
