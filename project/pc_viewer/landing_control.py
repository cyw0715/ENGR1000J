"""
Mars Lander - 着陆控制算法

根据地形高度图解算四条推杆的轴向长度，使 200 mm × 200 mm 平台
以水平姿态接触地面。推杆相对水平面向下 60°，即相对竖直方向
向外 30°；长度范围为 250–400 mm。

安全模型：
- 四条推杆安装在平台四角；每条推杆相对其安装角沿平面对角线向外下方展开。
- 每条 20 mm 直径推杆的两端均为球铰/万向铰；推杆本体只承受轴向力。
- 每个脚点必须落在该推杆的设计轴线上。若地形只允许侧向顶住推杆，
  或轴线上的触地点不在行程内，解算器会拒绝运动，而不会用错误长度硬撑。
- 沿推杆本体（下端球铰附近保留 30 mm 机械连接区）采样三条直径方向
  的母线，发现地形进入 20 mm 推杆包络就拒绝该方案。

几何模型:
  主体: 200mm × 200mm 正方形
  4条腿安装在四个角,与竖直方向呈30°向外展开
  腿长可调: 250mm ~ 400mm

传感器输入:
  1. BNO085 IMU  → HTTP GET http://<ESP32_IP>/imu
  2. HC-SR04 超声波 → HTTP GET http://<ESP32_IP>/distance
  3. 视频流 (预留)  → http://<ESP32_IP>/stream

输出: 四角推杆长度 (mm), [front_left, front_right, rear_left, rear_right]

依赖: pip install requests numpy
用法: python landing_control.py
"""

import argparse
import math
import time
from typing import Callable, Dict, Optional, Tuple

import numpy as np

try:  # Optional: geometry/console integration must not require HTTP tooling.
    import requests
except ImportError:  # pragma: no cover - exercised only by legacy live diagnostics.
    requests = None

ESP32_IP = "192.168.1.100"  # 根据实际网络修改

# ============================================================
# 着陆器几何常量（单位：mm）
# ============================================================
BODY_SIZE = 200.0
HALF_BODY = BODY_SIZE / 2.0
# Camera optical center is 50 mm below the four upper-joint mount plane.
CAMERA_TO_MOUNT_PLANE_MM = 50.0
# 用户确认：推杆相对水平面向下 60°，即相对竖直向外 30°。
LEG_ANGLE_DEG = 30.0
LEG_ANGLE_RAD = math.radians(LEG_ANGLE_DEG)
SIN_LEG = math.sin(LEG_ANGLE_RAD)
COS_LEG = math.cos(LEG_ANGLE_RAD)
LEG_MIN = 250.0
LEG_MAX = 400.0
ACTUATOR_DIAMETER_MM = 20.0
ACTUATOR_RADIUS_MM = ACTUATOR_DIAMETER_MM / 2.0
FOOT_JOINT_EXCLUSION_MM = 30.0
SHAFT_CLEARANCE_MARGIN_MM = 5.0

LEG_NAMES = ("front_left", "front_right", "rear_left", "rear_right")

# 坐标系：x 向右、y 向前、z 向上。平台中心为 (0, 0, platform_height)。
# 四个安装点必须位于平台四角，而不是边中点。
LEG_MOUNT_POINTS = {
    "front_left": np.array([-HALF_BODY, +HALF_BODY, 0.0]),
    "front_right": np.array([+HALF_BODY, +HALF_BODY, 0.0]),
    "rear_left": np.array([-HALF_BODY, -HALF_BODY, 0.0]),
    "rear_right": np.array([+HALF_BODY, -HALF_BODY, 0.0]),
}

# 每条推杆沿相应角点的平面对角线向外展开，保持四腿完全对称。
LEG_OUTWARD_DIR = {
    name: point / np.linalg.norm(point)
    for name, point in LEG_MOUNT_POINTS.items()
}


class DepthTerrainHeightField:
    """Query a downward-camera depth map as terrain height relative to image center.

    Coordinate frame matches the actuator solver: x right, y front, z up. Camera
    image v increases toward the rear, hence the minus sign in the y projection.
    The optical center is 50 mm below the upper-joint plane.
    """

    def __init__(self, depth_m: np.ndarray, *, fx_px: float, fy_px: float,
                 cx_px: float, cy_px: float,
                 projection_model: str = "pinhole",
                 distortion_coefficients: Optional[np.ndarray] = None) -> None:
        depth = np.asarray(depth_m, dtype=np.float32)
        valid = np.isfinite(depth) & (depth > 0.0)
        if depth.ndim != 2 or not valid.any() or min(fx_px, fy_px) <= 0.0:
            raise ValueError("invalid metric depth terrain input")
        self.depth_mm = depth * 1000.0
        self.height, self.width = depth.shape
        self.fx_px, self.fy_px = float(fx_px), float(fy_px)
        self.cx_px, self.cy_px = float(cx_px), float(cy_px)
        if projection_model not in ("pinhole", "equidistant_fisheye", "opencv_fisheye"):
            raise ValueError(f"unsupported projection model: {projection_model}")
        self.projection_model = projection_model
        if projection_model == "opencv_fisheye":
            coefficients = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
            if coefficients.size != 4 or not np.isfinite(coefficients).all():
                raise ValueError("opencv_fisheye requires four finite distortion coefficients")
            self.distortion_coefficients = coefficients
        else:
            self.distortion_coefficients = np.zeros(4, dtype=np.float64)
        y0, y1 = max(0, int(round(cy_px)) - 5), min(self.height, int(round(cy_px)) + 6)
        x0, x1 = max(0, int(round(cx_px)) - 5), min(self.width, int(round(cx_px)) + 6)
        center = self.depth_mm[y0:y1, x0:x1]
        center_valid = np.isfinite(center) & (center > 0.0)
        if not center_valid.any():
            raise ValueError("depth map has no valid optical-center ground reference")
        self.camera_ground_depth_mm = float(np.median(center[center_valid]))
        self.current_mount_height_mm = self.camera_ground_depth_mm + CAMERA_TO_MOUNT_PLANE_MM
        relative = self.camera_ground_depth_mm - self.depth_mm[valid]
        self.min_height_mm = float(np.percentile(relative, 2.0))
        self.max_height_mm = float(np.percentile(relative, 98.0))

    def _bilinear_depth_mm(self, u: float, v: float) -> float:
        if not (0.0 <= u <= self.width - 1 and 0.0 <= v <= self.height - 1):
            return float("nan")
        x0, y0 = int(math.floor(u)), int(math.floor(v))
        x1, y1 = min(x0 + 1, self.width - 1), min(y0 + 1, self.height - 1)
        values = np.array([
            self.depth_mm[y0, x0], self.depth_mm[y0, x1],
            self.depth_mm[y1, x0], self.depth_mm[y1, x1],
        ], dtype=np.float64)
        if not np.isfinite(values).all() or np.any(values <= 0.0):
            return float("nan")
        tx, ty = u - x0, v - y0
        return float(
            values[0] * (1 - tx) * (1 - ty) + values[1] * tx * (1 - ty) +
            values[2] * (1 - tx) * ty + values[3] * tx * ty
        )

    def project_ground_xy_to_pixel(self, x_mm: float, y_mm: float,
                                   z_depth_mm: float) -> tuple[float, float]:
        """Project a ground offset using the configured downward camera model."""
        z = max(float(z_depth_mm), 1.0)
        x, y = float(x_mm), float(y_mm)
        if self.projection_model == "pinhole":
            return (
                self.cx_px + self.fx_px * x / z,
                self.cy_px - self.fy_px * y / z,
            )
        radial_xy = math.hypot(x, y)
        if radial_xy <= 1e-12:
            return self.cx_px, self.cy_px
        theta = math.atan2(radial_xy, z)
        if self.projection_model == "opencv_fisheye":
            k1, k2, k3, k4 = self.distortion_coefficients
            theta2 = theta * theta
            theta_d = theta * (
                1.0 + k1 * theta2 + k2 * theta2 ** 2 +
                k3 * theta2 ** 3 + k4 * theta2 ** 4
            )
        else:
            # Ideal equidistant fisheye: image radius r=f*theta.
            theta_d = theta
        return (
            self.cx_px + self.fx_px * theta_d * x / radial_xy,
            self.cy_px - self.fy_px * theta_d * y / radial_xy,
        )

    def get_height(self, x: float, y: float) -> float:
        depth_mm = self.camera_ground_depth_mm
        for _ in range(5):
            u, v = self.project_ground_xy_to_pixel(x, y, depth_mm)
            sampled = self._bilinear_depth_mm(u, v)
            if not np.isfinite(sampled):
                return float("nan")
            depth_mm = sampled
        return self.camera_ground_depth_mm - depth_mm


class TerrainModel:
    """用于离线验证的连续高度场。真实控制可传入视觉/测距得到的高度函数。"""

    def __init__(self, terrain_type: str = "flat", seed: Optional[int] = None):
        self.terrain_type = terrain_type
        self.rng = np.random.RandomState(seed)
        self.features = []
        if terrain_type == "flat":
            return
        if terrain_type == "bumps":
            self._generate_features("bump", 3, (20, 60), (30, 80))
        elif terrain_type == "pits":
            self._generate_features("pit", 3, (20, 60), (30, 80))
        elif terrain_type == "mixed":
            self._generate_features("bump", 2, (20, 50), (30, 60))
            self._generate_features("pit", 2, (20, 50), (30, 60))
        else:
            raise ValueError(f"未知地形类型: {terrain_type}")

    def _generate_features(self, feature_type, num, height_range, radius_range):
        for _ in range(num):
            self.features.append({
                "type": feature_type,
                "center": np.array([self.rng.uniform(-350, 350), self.rng.uniform(-350, 350)]),
                "height": self.rng.uniform(*height_range),
                "radius": self.rng.uniform(*radius_range),
            })

    def get_height(self, x: float, y: float) -> float:
        total = 0.0
        point = np.array([x, y])
        for feature in self.features:
            distance = np.linalg.norm(point - feature["center"])
            if distance < feature["radius"]:
                scale = feature["radius"] / 3.0
                amplitude = feature["height"] * math.exp(-(distance ** 2) / (2.0 * scale ** 2))
                total += amplitude if feature["type"] == "bump" else -amplitude
        return total

    def add_bump(self, x: float, y: float, height: float, radius: float) -> None:
        self.features.append({"type": "bump", "center": np.array([x, y]), "height": height, "radius": radius})

    def add_pit(self, x: float, y: float, depth: float, radius: float) -> None:
        self.features.append({"type": "pit", "center": np.array([x, y]), "height": depth, "radius": radius})


def _mount_world_point(name: str, platform_height_mm: float) -> np.ndarray:
    point = LEG_MOUNT_POINTS[name].copy()
    point[2] = platform_height_mm
    return point


def leg_axis_direction(name: str) -> np.ndarray:
    """单位向量：从平台安装球铰指向脚端球铰。"""
    outward = LEG_OUTWARD_DIR[name]
    return np.array([
        outward[0] * SIN_LEG,
        outward[1] * SIN_LEG,
        -COS_LEG,
    ])


def foot_point(name: str, length_mm: float, platform_height_mm: float) -> np.ndarray:
    return _mount_world_point(name, platform_height_mm) + leg_axis_direction(name) * length_mm


def _bisect_first_crossing(
    signed_height: Callable[[float], float],
    lower: float,
    upper: float,
    iterations: int = 48,
) -> float:
    """Return the first line/terrain crossing between a positive and non-positive sample."""
    lo, hi = lower, upper
    for _ in range(iterations):
        mid = (lo + hi) / 2.0
        if signed_height(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def find_axial_contact_length(
    name: str,
    platform_height_mm: float,
    terrain_height: Callable[[float, float], float],
    sample_step_mm: float = 2.0,
) -> Optional[float]:
    """Find the first terrain intersection on a prescribed actuator axis.

    A returned length is an axial contact solution. `None` means no contact in the
    allowed actuator travel range, so using any other length would require the
    ground to push sideways on the actuator.
    """
    mount = _mount_world_point(name, platform_height_mm)
    direction = leg_axis_direction(name)

    def clearance(length_mm: float) -> float:
        point = mount + direction * length_mm
        return point[2] - terrain_height(point[0], point[1])

    previous_length = LEG_MIN
    previous_clearance = clearance(previous_length)
    if previous_clearance <= 0.0:
        # The terrain reaches the prescribed axis before the actuator can
        # retract to its minimum length. Any command at LEG_MIN would preload
        # the shaft against terrain, so report the actual shorter intersection.
        if LEG_MIN > 0.0:
            return _bisect_first_crossing(clearance, 0.0, LEG_MIN)
        return 0.0

    steps = max(1, int(math.ceil((LEG_MAX - LEG_MIN) / sample_step_mm)))
    for index in range(1, steps + 1):
        current_length = LEG_MIN + (LEG_MAX - LEG_MIN) * index / steps
        current_clearance = clearance(current_length)
        if current_clearance <= 0.0:
            return _bisect_first_crossing(clearance, previous_length, current_length)
        previous_length = current_length
        previous_clearance = current_clearance
    return None


def _shaft_clearance_report(
    name: str,
    length_mm: float,
    platform_height_mm: float,
    terrain_height: Callable[[float, float], float],
    sample_step_mm: float = 5.0,
) -> Tuple[bool, float, Optional[np.ndarray]]:
    """Check a conservative cylindrical shaft envelope against terrain.

    At each shaft station, sample the centerline plus both local horizontal
    perpendicular directions. This is conservative for a height-field terrain
    and detects terrain rubbing on the 20 mm diameter actuator body.
    """
    mount = _mount_world_point(name, platform_height_mm)
    direction = leg_axis_direction(name)
    horizontal_outward = LEG_OUTWARD_DIR[name]
    horizontal_tangent = np.array([-horizontal_outward[1], horizontal_outward[0], 0.0])
    radius = ACTUATOR_RADIUS_MM + SHAFT_CLEARANCE_MARGIN_MM
    start = 0.0
    end = max(start, length_mm - FOOT_JOINT_EXCLUSION_MM)
    sample_count = max(1, int(math.ceil((end - start) / sample_step_mm)))
    min_clearance = float("inf")
    worst = None

    for index in range(sample_count + 1):
        length = start + (end - start) * index / sample_count
        center = mount + direction * length
        for lateral in (np.zeros(3), horizontal_outward * radius, -horizontal_outward * radius,
                        horizontal_tangent * radius, -horizontal_tangent * radius):
            point = center + lateral
            clearance = point[2] - terrain_height(point[0], point[1])
            if clearance < min_clearance:
                min_clearance = clearance
                worst = point
    return min_clearance >= 0.0, min_clearance, worst


def solve_level_platform(
    terrain_height: Callable[[float, float], float],
    requested_platform_height_mm: float,
) -> dict:
    """Solve all four corner actuator lengths for a horizontal platform.

    This is a pre-contact landing pose solver. It never clamps an impossible
    length: a travel-limit or non-axial-contact failure yields an unsafe result
    and `motion_permitted=False`.
    """
    legs = {}
    unsafe_reasons = []
    lengths = {}

    for name in LEG_NAMES:
        contact_length = find_axial_contact_length(name, requested_platform_height_mm, terrain_height)
        leg = {
            "mount_point_mm": _mount_world_point(name, requested_platform_height_mm),
            "axis_unit_vector": leg_axis_direction(name),
            "contact_solution_found": contact_length is not None,
            "travel_safe": contact_length is not None and LEG_MIN <= contact_length <= LEG_MAX,
        }
        if contact_length is None:
            leg.update({
                "length_mm": None,
                "foot_point_mm": None,
                "contact_height_error_mm": None,
                "side_load_safe": False,
                "side_load_reason": "no_terrain_contact_on_prescribed_axis",
                "shaft_clearance_safe": False,
                "shaft_min_clearance_mm": None,
            })
            unsafe_reasons.append("no_contact_solution")
            legs[name] = leg
            continue

        foot = foot_point(name, contact_length, requested_platform_height_mm)
        contact_error = foot[2] - terrain_height(foot[0], foot[1])
        shaft_safe, shaft_clearance, worst_point = _shaft_clearance_report(
            name, contact_length, requested_platform_height_mm, terrain_height
        )
        leg.update({
            "length_mm": contact_length,
            "foot_point_mm": foot,
            "contact_height_error_mm": contact_error,
            "side_load_safe": True,
            "side_load_reason": "axial_only_with_spherical_joints",
            "shaft_clearance_safe": shaft_safe,
            "shaft_min_clearance_mm": shaft_clearance,
            "shaft_worst_point_mm": worst_point,
        })
        lengths[name] = contact_length
        if not leg["travel_safe"]:
            unsafe_reasons.append("travel_limit")
        if not shaft_safe:
            unsafe_reasons.append("shaft_collision")
        legs[name] = leg

    if "no_contact_solution" in unsafe_reasons:
        status = "unsafe_no_contact_solution"
    elif "travel_limit" in unsafe_reasons:
        status = "unsafe_travel_limit"
    elif "shaft_collision" in unsafe_reasons:
        status = "unsafe_shaft_collision"
    else:
        status = "safe"

    return {
        "status": status,
        "motion_permitted": status == "safe",
        "platform_height_mm": requested_platform_height_mm,
        "lengths_mm": lengths,
        "legs": legs,
        # Pose is prescribed horizontal. Contact errors prove whether each foot
        # satisfies that horizontal-platform geometry instead of estimating tilt
        # from an unrelated plane through differently located foot points.
        "residual_tilt_deg": {"roll": 0.0, "pitch": 0.0} if status == "safe" else None,
        "unsafe_reasons": sorted(set(unsafe_reasons)),
    }


def solve_best_visible_platform(
    terrain_height: Callable[[float, float], float],
    *,
    search_step_mm: float = 2.0,
) -> dict:
    """Choose the most informative preview when only some leg regions are visible.

    No terrain extrapolation is performed. The search maximizes solved visible
    legs, then balances their travel reserve and length spread. Missing legs are
    explicitly marked unavailable, and motion is always prohibited.
    """
    nominal_length = (LEG_MIN + LEG_MAX) / 2.0
    nominal_height = nominal_length * COS_LEG
    low_height = LEG_MIN * COS_LEG - 60.0
    high_height = LEG_MAX * COS_LEG + 60.0
    best = None
    best_score = None
    height = low_height
    while height <= high_height + 1e-9:
        result = solve_level_platform(terrain_height, height)
        usable = {}
        legs = {}
        for name in LEG_NAMES:
            leg = dict(result["legs"].get(name, {}))
            length = leg.get("length_mm")
            safe = bool(
                length is not None and leg.get("travel_safe") and
                leg.get("side_load_safe") and leg.get("shaft_clearance_safe")
            )
            if safe:
                usable[name] = float(length)
                leg["preview_status"] = "visible_safe_preview"
            else:
                leg["preview_status"] = "not_visible_or_no_solution"
            legs[name] = leg
        if usable:
            values = list(usable.values())
            score = (
                -len(usable),
                max(abs(value - nominal_length) for value in values),
                max(values) - min(values),
                abs(height - nominal_height),
            )
            if best_score is None or score < best_score:
                best_score = score
                best = {
                    "status": "safe" if len(usable) == 4 else "partial_preview",
                    "mode": "preview_only", "motion_permitted": False,
                    "geometric_safe": len(usable) == 4,
                    "platform_height_mm": height,
                    "lengths_mm": usable,
                    "legs": legs,
                    "visible_leg_count": len(usable),
                    "unsafe_reasons": [] if len(usable) == 4 else ["partial_terrain_visibility"],
                }
        height += search_step_mm
    if best is not None:
        return best
    return {
        "status": "unobservable", "mode": "preview_only",
        "motion_permitted": False, "geometric_safe": False,
        "lengths_mm": {}, "legs": {
            name: {"preview_status": "terrain_not_observed"} for name in LEG_NAMES
        }, "visible_leg_count": 0, "unsafe_reasons": ["terrain_not_observed_at_leg_regions"],
    }


def solve_best_level_platform(
    terrain_height: Callable[[float, float], float],
    *,
    search_step_mm: float = 2.0,
) -> dict:
    """Choose a safe horizontal pose with balanced travel reserve.

    The objective minimizes the worst distance from the midpoint of the real
    250–400 mm travel, then the four-leg length spread. The returned result is
    permanently preview-only even when the geometry is safe.
    """
    if search_step_mm <= 0.0:
        raise ValueError("search_step_mm must be positive")
    nominal_length = (LEG_MIN + LEG_MAX) / 2.0
    # Cover all realistic terrain offsets around the solver's z=0 reference.
    terrain_samples = []
    nominal_radius = HALF_BODY + nominal_length * SIN_LEG / math.sqrt(2.0)
    for sx in (-1.0, 0.0, 1.0):
        for sy in (-1.0, 0.0, 1.0):
            value = float(terrain_height(sx * nominal_radius, sy * nominal_radius))
            if math.isfinite(value):
                terrain_samples.append(value)
    if not terrain_samples:
        return {
            "status": "unsafe_invalid_terrain", "motion_permitted": False,
            "mode": "preview_only", "lengths_mm": {}, "legs": {},
            "unsafe_reasons": ["invalid_terrain"],
        }
    low_height = min(terrain_samples) + LEG_MIN * COS_LEG - 40.0
    high_height = max(terrain_samples) + LEG_MAX * COS_LEG + 40.0

    best = None
    best_score = None
    height = low_height
    while height <= high_height + 1e-9:
        candidate = solve_level_platform(terrain_height, height)
        if candidate["status"] == "safe" and len(candidate["lengths_mm"]) == 4:
            values = list(candidate["lengths_mm"].values())
            score = (
                max(abs(value - nominal_length) for value in values),
                max(values) - min(values),
                abs(sum(values) / len(values) - nominal_length),
            )
            if best_score is None or score < best_score:
                best_score = score
                best = candidate
        height += search_step_mm

    if best is None:
        return {
            "status": "unsafe_no_level_pose", "motion_permitted": False,
            "mode": "preview_only", "lengths_mm": {}, "legs": {},
            "unsafe_reasons": ["no_safe_level_pose"],
        }
    best = dict(best)
    best["geometric_safe"] = True
    best["motion_permitted"] = False
    best["mode"] = "preview_only"
    best["objective"] = "balanced_travel_reserve_then_minimum_length_spread"
    best["travel_margin_mm"] = min(
        min(value - LEG_MIN, LEG_MAX - value) for value in best["lengths_mm"].values()
    )
    return best


def compute_leg_lengths(roll_deg: float, pitch_deg: float, center_height_mm: float) -> list:
    """Compatibility helper for a planar-terrain estimate from IMU attitude.

    It does not command hardware. The newer `solve_level_platform()` should be
    used with terrain observations before any landing command is emitted.
    """
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)

    def terrain_height(x: float, y: float) -> float:
        # Ground plane through z=0 under the platform center. Positive pitch
        # raises terrain toward +x; positive roll raises it toward +y.
        return math.tan(pitch_rad) * x + math.tan(roll_rad) * y

    result = solve_level_platform(terrain_height, center_height_mm)
    return [int(round(result["lengths_mm"].get(name, LEG_MIN))) for name in LEG_NAMES]


class IMUReader:
    """Read BNO085 attitude diagnostics from the ESP32-P4 HTTP endpoint."""

    def __init__(self, esp32_ip=ESP32_IP, timeout=2.0):
        self.url = f"http://{esp32_ip}/imu"
        self.timeout = timeout
        self.roll = self.pitch = self.yaw = 0.0

    def read(self) -> bool:
        try:
            response = requests.get(self.url, timeout=self.timeout)
            response.raise_for_status()
            attitude = response.json().get("attitude", response.json())
            self.roll = float(attitude.get("roll", 0.0))
            self.pitch = float(attitude.get("pitch", 0.0))
            self.yaw = float(attitude.get("yaw", 0.0))
            return True
        except (requests.RequestException, ValueError, KeyError) as error:
            print(f"[IMU] 读取失败: {error}")
            return False


class DistanceReader:
    """Read center-height diagnostic data if the endpoint exists."""

    def __init__(self, esp32_ip=ESP32_IP, timeout=2.0):
        self.url = f"http://{esp32_ip}/distance"
        self.timeout = timeout
        self.distance_mm = None

    def read(self) -> bool:
        try:
            response = requests.get(self.url, timeout=self.timeout)
            response.raise_for_status()
            self.distance_mm = float(response.json().get("distance_mm", 0.0))
            return self.distance_mm > 0.0
        except (requests.RequestException, ValueError, KeyError) as error:
            print(f"[距离] 读取失败: {error}")
            return False


def format_result(result: dict) -> str:
    lines = [
        "=" * 72,
        "Mars Lander corner-actuator landing solution",
        f"status={result['status']} motion_permitted={result['motion_permitted']}",
        f"horizontal platform height={result['platform_height_mm']:.1f} mm",
        "-" * 72,
    ]
    for name in LEG_NAMES:
        leg = result["legs"][name]
        if leg["length_mm"] is None:
            lines.append(f"{name:12s}: REJECTED ({leg['side_load_reason']})")
            continue
        foot = leg["foot_point_mm"]
        lines.append(
            f"{name:12s}: L={leg['length_mm']:.2f} mm, "
            f"foot=({foot[0]:+.1f}, {foot[1]:+.1f}, {foot[2]:+.1f}), "
            f"shaft_clearance={leg['shaft_min_clearance_mm']:.2f} mm, "
            f"axial={leg['side_load_safe']}"
        )
    if result["unsafe_reasons"]:
        lines.append("unsafe_reasons=" + ", ".join(result["unsafe_reasons"]))
    lines.append("=" * 72)
    return "\n".join(lines)


# ============================================================
# Phase 2: IMU attitude PID leg controller
# ============================================================
IMU_DEAD_ZONE_DEG = 5.0
IMU_TARGET_TOLERANCE_DEG = 1.0

# PID gains — outside dead zone (proportional only, fast approach)
IMU_KP_OUTER = 3.0  # mm per degree
# PID gains — inside dead zone (full PID, precise convergence)
IMU_KP_INNER = 1.0  # reduced from 2.0 to suppress PID-zone twitching
IMU_KI_INNER = 0.5
IMU_KD_INNER = 1.0
IMU_INTEGRAL_LIMIT_MM = 30.0
IMU_MAX_ADJUSTMENT_MM = 50.0
IMU_D_FILTER_ALPHA = 0.3  # EMA smoothing for derivative

# IMU → delta command safety limits
IMU_DELTA_MAX_MM = 350         # protocol span; Mega enforces each ARM baseline + delta in 50..400 mm
IMU_DELTA_SLOPE_MAX_MM_S = 5   # max rate of change of delta target
IMU_SEND_MIN_INTERVAL_S = 0.1  # min time between delta sends (10 Hz cap)
CV_RELATIVE_DELTA_MAX_MM = 100.0
CV_RELATIVE_DEADZONE_MM = 5.0
CV_CAPTURE_MAX_AGE_S = 5.0


def cv_plan_to_relative_deltas(plan: dict, *, max_abs_mm: float = CV_RELATIVE_DELTA_MAX_MM) -> dict[str, float]:
    """Convert a complete stable mechanical plan to zero-common-mode VL53 deltas.

    CV controls only differential leg shape. It must not invent the unknown
    absolute actuator-length/VL53 offset or command common heave.
    """
    if plan.get("status") != "safe" or not plan.get("geometric_safe", False):
        raise ValueError("CV plan is not a complete geometrically safe four-leg solution")
    lengths = plan.get("lengths_mm", {})
    key_map = {
        "fl": "front_left", "fr": "front_right",
        "rl": "rear_left", "rr": "rear_right",
    }
    if set(lengths) != set(key_map.values()):
        raise ValueError("CV plan must contain exactly four visible leg lengths")
    values = {short: float(lengths[long_name]) for short, long_name in key_map.items()}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("CV leg lengths must be finite")
    common = sum(values.values()) / 4.0
    raw = {name: value - common for name, value in values.items()}
    peak = max(abs(value) for value in raw.values())
    scale = 1.0 if peak <= max_abs_mm or peak <= 1e-9 else max_abs_mm / peak
    result = {name: round(value * scale, 3) for name, value in raw.items()}
    # Remove floating roundoff so CV never commands common heave.
    residual = sum(result.values()) / 4.0
    return {name: round(value - residual, 3) for name, value in result.items()}

IMU_STALE_THRESHOLD_S = 2.0    # IMU sample older than this → emergency STOP
IMU_HEALTH_MIN_UPTIME_S = 5.0  # P4/Mega must be up for at least this long
IMU_P4_HEALTH_MIN_AGE_S = 5.0  # Mega status must be fresh


IMU_ORTHOGONAL_TRANSFORMS: dict[str, tuple[tuple[int, int], tuple[int, int]]] = {
    "identity": ((1, 0), (0, 1)),
    "rot90": ((0, 1), (-1, 0)),
    "rot180": ((-1, 0), (0, -1)),
    "rot270": ((0, -1), (1, 0)),
    "swap": ((0, 1), (1, 0)),
    "swap_neg": ((0, -1), (-1, 0)),
    "flip_roll": ((-1, 0), (0, 1)),
    "flip_pitch": ((1, 0), (0, -1)),
}


def wrap_angle_deg(angle_deg: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def transform_imu_to_platform(
    roll_deg: float,
    pitch_deg: float,
    transform: str,
    *,
    roll_zero_deg: float = 0.0,
    pitch_zero_deg: float = 0.0,
) -> tuple[float, float]:
    """Apply installation-zero removal then an axis-aligned orthogonal transform."""
    if transform not in IMU_ORTHOGONAL_TRANSFORMS:
        raise ValueError("IMU/platform orthogonal transform is not identified")
    roll = float(roll_deg)
    pitch = float(pitch_deg)
    if not math.isfinite(roll) or not math.isfinite(pitch):
        raise ValueError("IMU attitude must be finite")
    roll = wrap_angle_deg(roll - float(roll_zero_deg))
    pitch = wrap_angle_deg(pitch - float(pitch_zero_deg))
    matrix = IMU_ORTHOGONAL_TRANSFORMS[transform]
    return (
        matrix[0][0] * roll + matrix[0][1] * pitch,
        matrix[1][0] * roll + matrix[1][1] * pitch,
    )


class ImuLegController:
    """PID controller that adjusts leg lengths to level the platform.

    Given roll/pitch errors from IMU, computes per-leg length corrections
    that tend toward zero tilt. Uses proportional control outside a 5° dead
    zone for speed, and full PID inside for precision. The target is stable
    orientation within 2° of level.

    Only four symmetric corner-mounted legs are supported. The controller
    never commands hardware; it only returns preview length adjustments.
    """

    # Platform geometry: 200mm square, legs at corners
    CORNER_X_MM = 100.0
    CORNER_Y_MM = 100.0
    LEG_AXIS_VERTICAL_COMPONENT = COS_LEG  # cos(30°) ≈ 0.866

    # Signed position factors: how each leg responds to roll (y) and pitch (x)
    # Positive roll = right side down → right legs longer
    # Positive pitch = front down → front legs longer
    LEG_FACTORS = {
        "front_left":  (-1.0, +1.0),   # (roll_sign, pitch_sign)
        "front_right": (+1.0, +1.0),
        "rear_left":   (-1.0, -1.0),
        "rear_right":  (+1.0, -1.0),
    }

    def __init__(self) -> None:
        self._integral_roll = 0.0
        self._integral_pitch = 0.0
        self._prev_roll = 0.0
        self._prev_pitch = 0.0
        self._prev_time: float | None = None
        self._roll_deriv = 0.0
        self._pitch_deriv = 0.0
        self.mode = "idle"  # "idle" | "outer" | "pid"

    def reset(self) -> None:
        self._integral_roll = 0.0
        self._integral_pitch = 0.0
        self._prev_roll = 0.0
        self._prev_pitch = 0.0
        self._prev_time = None
        self._roll_deriv = 0.0
        self._pitch_deriv = 0.0
        self.mode = "idle"

    def compute(
        self,
        roll_deg: float,
        pitch_deg: float,
        now: float | None = None,
    ) -> dict:
        """Compute leg length adjustments for current IMU attitude.

        Returns dict with keys:
          roll_error, pitch_error: signed errors in degrees
          mode: "outer" or "pid"
          within_target: True if both errors < 2°
          adjustments_mm: {"front_left": ΔL, ...} in mm (positive = longer)
        """
        if now is None:
            now = time.monotonic()

        # Compute dt
        if self._prev_time is None:
            dt = 0.0
        else:
            dt = max(now - self._prev_time, 0.001)
        self._prev_time = now

        roll_err = float(roll_deg)
        pitch_err = float(pitch_deg)
        if not math.isfinite(roll_err) or not math.isfinite(pitch_err):
            raise ValueError("IMU attitude must be finite")
        abs_roll = abs(roll_err)
        abs_pitch = abs(pitch_err)

        # Determine control mode
        if abs_roll <= IMU_DEAD_ZONE_DEG and abs_pitch <= IMU_DEAD_ZONE_DEG:
            entering_pid = self.mode != "pid"
            self.mode = "pid"
            if entering_pid:
                # Bumpless outer→PID transfer: the previous outer-loop error must
                # not become a large derivative impulse at the 5° boundary.
                self._roll_deriv = 0.0
                self._pitch_deriv = 0.0
                self._prev_roll = roll_err
                self._prev_pitch = pitch_err
                dt = 0.0
            # PID for roll
            self._integral_roll += roll_err * dt
            self._integral_roll = max(-IMU_INTEGRAL_LIMIT_MM,
                                      min(IMU_INTEGRAL_LIMIT_MM, self._integral_roll))
            if dt > 0:
                raw_d_roll = (roll_err - self._prev_roll) / dt
                self._roll_deriv = (IMU_D_FILTER_ALPHA * raw_d_roll +
                                    (1 - IMU_D_FILTER_ALPHA) * self._roll_deriv)
            delta_roll = (IMU_KP_INNER * roll_err +
                          IMU_KI_INNER * self._integral_roll +
                          IMU_KD_INNER * self._roll_deriv)
            # PID for pitch
            self._integral_pitch += pitch_err * dt
            self._integral_pitch = max(-IMU_INTEGRAL_LIMIT_MM,
                                       min(IMU_INTEGRAL_LIMIT_MM, self._integral_pitch))
            if dt > 0:
                raw_d_pitch = (pitch_err - self._prev_pitch) / dt
                self._pitch_deriv = (IMU_D_FILTER_ALPHA * raw_d_pitch +
                                     (1 - IMU_D_FILTER_ALPHA) * self._pitch_deriv)
            delta_pitch = (IMU_KP_INNER * pitch_err +
                           IMU_KI_INNER * self._integral_pitch +
                           IMU_KD_INNER * self._pitch_deriv)
        else:
            self.mode = "outer"
            # Proportional only for fast approach
            delta_roll = IMU_KP_OUTER * roll_err
            delta_pitch = IMU_KP_OUTER * pitch_err
            # Reset integrals when leaving dead zone
            self._integral_roll *= 0.5
            self._integral_pitch *= 0.5

        self._prev_roll = roll_err
        self._prev_pitch = pitch_err

        # Clamp per-axis adjustments
        delta_roll = max(-IMU_MAX_ADJUSTMENT_MM,
                         min(IMU_MAX_ADJUSTMENT_MM, delta_roll))
        delta_pitch = max(-IMU_MAX_ADJUSTMENT_MM,
                          min(IMU_MAX_ADJUSTMENT_MM, delta_pitch))

        # Convert to per-leg axial length adjustments
        # Each leg's adjustment = (roll_sign * delta_roll + pitch_sign * delta_pitch) / cos(30°)
        adjustments = {}
        for name, (roll_sign, pitch_sign) in self.LEG_FACTORS.items():
            raw = roll_sign * delta_roll + pitch_sign * delta_pitch
            adjustments[name] = raw / self.LEG_AXIS_VERTICAL_COMPONENT

        within = abs_roll < IMU_TARGET_TOLERANCE_DEG and abs_pitch < IMU_TARGET_TOLERANCE_DEG

        return {
            "roll_error": roll_err,
            "pitch_error": pitch_err,
            "mode": self.mode,
            "within_target": within,
            "adjustments_mm": adjustments,
        }


class ImuDeltaCommandGenerator:
    """Convert ImuLegController output to rate-limited relative delta commands.

    Takes the per-leg adjustment_mm from ImuLegController, clamps to
    DELTA_MAX_MM, applies slope limiting, and tracks send timing.
    Produces FL/FR/RL/RR deltas for SET_DELTA commands.
    """

    def __init__(self) -> None:
        self._last_target_delta = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
        self._last_send_time: float | None = None
        self._send_seq: int = 0
        self._last_imu_time: float | None = None
        self._imu_stale = False

    def reset(self) -> None:
        self._last_target_delta = {"fl": 0.0, "fr": 0.0, "rl": 0.0, "rr": 0.0}
        self._last_send_time = None
        self._send_seq = 0
        self._last_imu_time = None
        self._imu_stale = False

    def update_imu_time(self, now: float) -> None:
        """Track latest IMU sample time for stale detection."""
        self._last_imu_time = now
        self._imu_stale = False

    def is_imu_stale(self, now: float) -> bool:
        """Check if IMU data is stale."""
        if self._last_imu_time is None:
            return True
        return (now - self._last_imu_time) > IMU_STALE_THRESHOLD_S

    def compute_deltas(
        self,
        adjustments_mm: dict[str, float],
        now: float,
        *,
        hold: bool = False,
    ) -> dict[str, float] | None:
        """Advance ARM-relative targets slowly in the demanded direction.

        ``adjustments_mm`` is a correction demand from the attitude controller,
        not an absolute target.  Repeated non-zero demand accumulates against the
        single baseline captured by Mega at ARM.  ``hold=True`` preserves the
        current target after attitude enters the ±2° stable region.
        """
        if self.is_imu_stale(now):
            return None
        if self._last_send_time is not None and now - self._last_send_time < IMU_SEND_MIN_INTERVAL_S:
            return None
        if hold:
            return None

        name_map = {
            "front_left": "fl", "front_right": "fr",
            "rear_left": "rl", "rear_right": "rr",
        }
        dt = (now - self._last_send_time) if self._last_send_time is not None else IMU_SEND_MIN_INTERVAL_S
        dt = max(IMU_SEND_MIN_INTERVAL_S, min(dt, 0.5))
        max_step = IMU_DELTA_SLOPE_MAX_MM_S * dt
        new_delta: dict[str, float] = {}
        for solver_name, short_name in name_map.items():
            demand = float(adjustments_mm.get(solver_name, 0.0))
            if not math.isfinite(demand):
                raise ValueError("IMU leg adjustment must be finite")
            previous = self._last_target_delta[short_name]
            if abs(demand) < 0.1:
                candidate = previous
            else:
                step = max(-max_step, min(max_step, demand))
                candidate = previous + step
            candidate = max(-IMU_DELTA_MAX_MM, min(IMU_DELTA_MAX_MM, candidate))
            new_delta[short_name] = round(candidate, 2)

        if all(abs(new_delta[k] - self._last_target_delta[k]) < 0.01 for k in new_delta):
            return None
        self._last_target_delta = new_delta
        self._last_send_time = now
        self._send_seq += 1
        return dict(new_delta)

    @property
    def current_target(self) -> dict[str, float]:
        return dict(self._last_target_delta)

    @property
    def send_seq(self) -> int:
        return self._send_seq


def run_simulated() -> None:
    nominal_height = 350.0 * COS_LEG
    scenarios = {
        "flat": lambda _x, _y: 0.0,
        "front_right_slope": lambda x, y: 0.05 * x + 0.10 * y,
        "axis_obstacle": lambda _x, y: 500.0 if y > 250.0 else 0.0,
    }
    for name, terrain in scenarios.items():
        print(f"\n--- {name} ---")
        print(format_result(solve_level_platform(terrain, nominal_height)))


def run_live() -> None:
    print("Live mode currently reads IMU/height only; it never sends actuator commands.")
    imu = IMUReader()
    dist = DistanceReader()
    while True:
        imu_ok = imu.read()
        height_ok = dist.read()
        print(f"IMU={imu_ok} roll={imu.roll:.2f} pitch={imu.pitch:.2f}; height={dist.distance_mm if height_ok else 'unavailable'}")
        time.sleep(0.5)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mars Lander corner actuator solver")
    parser.add_argument("--sim", action="store_true", help="run offline geometry scenarios")
    parser.add_argument("--live", action="store_true", help="read-only live sensor diagnostics")
    args = parser.parse_args()
    if args.live:
        run_live()
    else:
        run_simulated()
