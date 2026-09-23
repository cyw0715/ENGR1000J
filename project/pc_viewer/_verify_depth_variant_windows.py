import torch
from transformers import AutoConfig, pipeline

MODEL_IDS = [
    "depth-anything/Depth-Anything-V2-Small-hf",
    "depth-anything/Depth-Anything-V2-Base-hf",
]

for model_id in MODEL_IDS:
    cfg = AutoConfig.from_pretrained(model_id)
    backbone = cfg.backbone_config
    print(
        "CONFIG",
        model_id,
        "hidden_size=", backbone.hidden_size,
        "layers=", backbone.num_hidden_layers,
        "heads=", backbone.num_attention_heads,
        flush=True,
    )

assert torch.cuda.is_available(), "CUDA unavailable"
base_id = MODEL_IDS[1]
loaded = pipeline(
    "depth-estimation",
    model=base_id,
    device="cuda",
    dtype=torch.float16,
)
net = loaded.model
params = sum(p.numel() for p in net.parameters())
print("LOADED_NAME_OR_PATH", net.config._name_or_path, flush=True)
print("LOADED_BACKBONE_HIDDEN", net.config.backbone_config.hidden_size, flush=True)
print("LOADED_BACKBONE_LAYERS", net.config.backbone_config.num_hidden_layers, flush=True)
print("LOADED_PARAMETERS_M", "%.2f" % (params / 1_000_000), flush=True)
