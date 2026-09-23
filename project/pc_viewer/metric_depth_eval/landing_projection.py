"""Top-down provisional projection of the 300 mm square lander into the camera image.

Coordinate contract:
- The user confirmed a camera fixed at platform center, optical axis vertically down.
- This module therefore assumes camera x/right = platform +x/right and camera y/down
  = platform -y/front. It is a display-only approximation until the true camera
  extrinsics and accepted calibration are available.
- Lander feet are the four corner actuator axes at a nominal common length.
"""

import math
from typing import Optional, Tuple

import numpy as np

BODY_SIZE_M = 0.300
HALF_BODY_M = BODY_SIZE_M / 2.0
LEG_ANGLE_FROM_VERTICAL_RAD = math.radians(30.0)
DEFAULT_NOMINAL_LEG_LENGTH_M = 0.350
OVERLAY_ASSUMPTION = "camera_centered_downward_provisional"

# Clockwise visual polygon order in image coordinates: FL, FR, RR, RL.
CORNER_SIGNS = np.array([
    [-1.0, -1.0],
    [+1.0, -1.0],
    [+1.0, +1.0],
    [-1.0, +1.0],
], dtype=np.float32)


def project_lander_plan(
    center_ground_depth_m: float,
    image_shape: Tuple[int, int],
    fx_px_at_800: float,
    body_size_m: float = BODY_SIZE_M,
    nominal_leg_length_m: float = DEFAULT_NOMINAL_LEG_LENGTH_M,
) -> Optional[dict]:
    """Project platform corners and nominal corner feet into a downward-looking image.

    `center_ground_depth_m` is the fused depth at the optical axis. The platform
    lies at the camera center and remains horizontal for this overlay. The result
    is intentionally not used for actuator commands or metric terrain decisions.
    """
    if not np.isfinite(center_ground_depth_m) or center_ground_depth_m <= 0.0:
        return None
    if not np.isfinite(fx_px_at_800) or fx_px_at_800 <= 0.0:
        return None
    if len(image_shape) < 2 or image_shape[0] <= 0 or image_shape[1] <= 0:
        return None

    image_h, image_w = int(image_shape[0]), int(image_shape[1])
    # The fusion script uses square 800x800 sensor input and scales to a panel.
    # Scale focal length together with displayed width so overlay aligns in panel px.
    fx_px = fx_px_at_800 * image_w / 800.0
    fy_px = fx_px_at_800 * image_h / 800.0
    center_px = np.array([image_w / 2.0, image_h / 2.0], dtype=np.float32)

    corner_radius_m = (body_size_m / 2.0) * math.sqrt(2.0)
    # Each actuator extends radially away from its platform corner by L*sin(30).
    foot_radius_m = corner_radius_m + nominal_leg_length_m * math.sin(LEG_ANGLE_FROM_VERTICAL_RAD)

    def project_xy(radius_m: float) -> np.ndarray:
        xy_m = CORNER_SIGNS * (radius_m / math.sqrt(2.0))
        pixels = np.empty((4, 2), dtype=np.float32)
        pixels[:, 0] = center_px[0] + fx_px * xy_m[:, 0] / center_ground_depth_m
        pixels[:, 1] = center_px[1] + fy_px * xy_m[:, 1] / center_ground_depth_m
        return pixels

    return {
        "assumption": OVERLAY_ASSUMPTION,
        "image_center_px": center_px,
        "platform_corners_px": project_xy(corner_radius_m),
        "foot_points_px": project_xy(foot_radius_m),
        "center_ground_depth_m": float(center_ground_depth_m),
        "nominal_leg_length_m": float(nominal_leg_length_m),
    }
