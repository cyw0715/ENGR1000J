"""Mars Lander Windows desktop control console.

Run on Windows with the project-local CUDA environment:
  .\\.venv_win\\Scripts\\python.exe lander_console.py [P4_IP]

This console monitors the ESP32-P4 video/telemetry endpoints and controls the
four Mega/VL53 legs only through explicit user actions. Motion can start only
after an explicit IMU ARM confirmation or the confirmed one-click 150 mm
restore workflow; STOP/ESTOP and Mega's local watchdog remain authoritative.

Current first-stage control contract:
  CV mode: Remote absolute VL53 targets, FL/FR/RL/RR each 50..400 mm.
  IMU mode: existing ARM-relative attitude loop through the same Mega controller.
  Automatic CV terrain output is preview-only and cannot ARM or send a target.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import ctypes
import io
import json
import math
import queue
import socket
import struct
import sys
import threading
import time
from collections import deque
import tkinter as tk
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any

from absolute_terrain import (
    CAPTURE_FRAME_COUNT,
    SELECTED_FRAME_COUNT,
    STABLE_FRAME_COUNT,
    BestOfThirtyDepthCapture,
    camera_fov_x_deg,
    ULTRASONIC_FILTER_WINDOW,
    anchor_depth_with_ultrasonic,
    fuse_metric_and_relative,
    ground_height_summary,
    load_provisional_calibration,
    metric_contour_map,
    metric_depth_heatmap,
    project_lander_plan,
    select_best_depth_frames,
    source_observability,
    stable_cloud_average,
)
from landing_control import (
    cv_plan_to_relative_deltas,
    CV_CAPTURE_MAX_AGE_S,
    DepthTerrainHeightField,
    ImuLegController,
    ImuDeltaCommandGenerator,
    IMU_STALE_THRESHOLD_S,
    IMU_HEALTH_MIN_UPTIME_S,
    IMU_ORTHOGONAL_TRANSFORMS,
    transform_imu_to_platform,
    solve_best_visible_platform,
)
from PIL import Image, ImageDraw, ImageFont, ImageTk

APP_TITLE = "Mars Lander · Desktop Control Console"
VIDEO_PORT = 5000
IMU_TCP_PORT = 5001
HTTP_PORT = 80
VIDEO_VIEW_SIZE = (1040, 960)
CV_VIEW_NAMES = ("原始相机", "绝对高度 / 地面云图", "等高线图")
RESTORE_LEG_TARGET_MM = 150
RESTORE_LEG_TOLERANCE_MM = 1
RESTORE_SEGMENT_MAX_MM = 350  # full protocol span; Mega enforces 50..400 absolute target
LEG_TO_MEGA_SENSOR = {"fl": "a4", "fr": "a3", "rl": "a1", "rr": "a2"}
P4_SCAN_PREFIX = "192.168.137."
P4_SCAN_WORKERS = 48
P4_SCAN_TIMEOUT_S = 0.35
CV_CONTROL_VALIDATED = False  # Requires accepted current-camera plane/step measurements.


def enable_windows_high_dpi() -> None:
    """Request native per-monitor pixels before Tk creates its first HWND."""
    if sys.platform != "win32":
        return
    try:
        ctypes.windll.user32.SetProcessDpiAwarenessContext(ctypes.c_void_p(-4))
        return
    except (AttributeError, OSError):
        pass
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass


def configure_tk_scaling(root: tk.Misc) -> float:
    """Align Tk point sizes to the actual native monitor DPI."""
    try:
        dpi = float(root.winfo_fpixels("1i"))
        scaling = max(1.0, min(3.0, dpi / 72.0))
        root.tk.call("tk", "scaling", scaling)
        return scaling
    except (tk.TclError, ValueError, TypeError):
        return 1.0
RELATIVE_DEPTH_MODEL_ID = "depth-anything/Depth-Anything-V2-Base-hf"
METRIC_DEPTH_INPUT_SIZE = 518
METRIC_DEPTH_CHECKPOINT = (
    Path(__file__).resolve().parent / "metric_depth_eval" / "depth_anything_v2_metric_hypersim_vitb.pth"
)
METRIC_DEPTH_SOURCE = Path(__file__).resolve().parent / "metric_depth_eval" / "Depth-Anything-V2" / "metric_depth"
DEPTH_RANGE_SMOOTHING = 0.85
SOLVER_MIN_MM = 250
SOLVER_MAX_MM = 390
SENSOR_OFFSET_MM = 190
SENSOR_MIN_MM = 60
SENSOR_MAX_MM = 200

DARK = "#10151e"
PANEL = "#19212d"
PANEL_ALT = "#202b38"
TEXT = "#e6edf7"
MUTED = "#91a1b7"
GREEN = "#52d88c"
AMBER = "#f4c95d"
RED = "#ff6b6b"
CYAN = "#63d8ff"


CONTROL_MODE_CV = "cv"
CONTROL_MODE_LOCKED = "locked"
CONTROL_MODE_IMU = "imu"
CONTROL_MODES = (CONTROL_MODE_CV, CONTROL_MODE_LOCKED, CONTROL_MODE_IMU)


def control_mode_description(mode: str) -> str:
    """Return the local-only planning meaning of a mode; never a motion grant."""
    descriptions = {
        CONTROL_MODE_CV: "CV控制（第一阶段Remote）：未ARM保持安全锁定；忠实启用111(1).ino的PS2手动/16预设；右下栏也可输入四路绝对VL53目标；自动CV仅预览。",
        CONTROL_MODE_LOCKED: "锁定控制：保持安全锁定，拒绝ARM和全部目标。",
        CONTROL_MODE_IMU: "IMU控制：TCP5001姿态闭环；未显式ARM时保持安全锁定。",
    }
    if mode not in descriptions:
        raise ValueError(f"未知控制模式：{mode}")
    return descriptions[mode]


def fit_panel_image(image: Image.Image, available_width: int, available_height: int,
                    resample: Image.Resampling = Image.Resampling.LANCZOS) -> Image.Image:
    """Fit a display copy to the live panel while preserving aspect ratio."""
    width = max(1, int(available_width))
    height = max(1, int(available_height))
    fitted = image.convert("RGB").copy()
    fitted.thumbnail((width, height), resample)
    return fitted


def five_second_average_fps(timestamps: deque[float], now: float,
                            window_seconds: float = 5.0) -> float:
    """Return completed frame intervals per second over the trailing window."""
    cutoff = now - window_seconds
    while timestamps and timestamps[0] < cutoff:
        timestamps.popleft()
    if len(timestamps) < 2:
        return 0.0
    span = max(now - timestamps[0], 1e-6)
    return (len(timestamps) - 1) / span


def overlay_fps(image: Image.Image, fps: float) -> Image.Image:
    """Overlay trailing-5-second FPS on a copy of the raw camera image."""
    canvas = image.convert("RGB").copy()
    draw = ImageDraw.Draw(canvas)
    text = f"5s AVG FPS: {fps:.2f}"
    font = ImageFont.load_default(size=22)
    box = draw.textbbox((0, 0), text, font=font, stroke_width=1)
    width, height = box[2] - box[0], box[3] - box[1]
    x, y, pad = 12, 12, 8
    draw.rounded_rectangle((x, y, x + width + 2 * pad, y + height + 2 * pad),
                           radius=8, fill=(5, 9, 14, 205))
    draw.text((x + pad, y + pad), text, font=font, fill=(255, 255, 255),
              stroke_width=1, stroke_fill=(0, 0, 0))
    return canvas


def build_cv_triptych(image: Image.Image) -> tuple[Image.Image, Image.Image, Image.Image]:
    """Build the raw-camera view plus blank capture-result placeholders."""
    original = image.convert("RGB")
    unavailable = Image.new("RGB", original.size, (24, 28, 36))
    return original, unavailable.copy(), unavailable.copy()


def colorize_relative_depth(depth: Any, previous_range: tuple[float, float] | None) -> tuple[Image.Image, tuple[float, float]]:
    """Convert Depth Anything tensor/array to a stable inferno-like relative depth heat map."""
    import numpy as np

    array = depth.detach().float().cpu().numpy() if hasattr(depth, "detach") else np.asarray(depth, dtype=np.float32)
    if array.ndim != 2 or not np.isfinite(array).all():
        raise ValueError("Depth model returned an invalid depth map")
    low, high = float(np.percentile(array, 2.0)), float(np.percentile(array, 98.0))
    if high <= low:
        high = low + 1e-6
    if previous_range is not None:
        old_low, old_high = previous_range
        low = DEPTH_RANGE_SMOOTHING * old_low + (1.0 - DEPTH_RANGE_SMOOTHING) * low
        high = max(DEPTH_RANGE_SMOOTHING * old_high + (1.0 - DEPTH_RANGE_SMOOTHING) * high, low + 1e-6)
    normalized = np.clip((array - low) / (high - low), 0.0, 1.0)
    # Nearer values are bright yellow; farther values are dark purple/black.
    red = np.clip(255.0 * (1.7 * normalized), 0, 255).astype(np.uint8)
    green = np.clip(255.0 * (1.2 * normalized - 0.15), 0, 255).astype(np.uint8)
    blue = np.clip(255.0 * (0.55 - normalized), 0, 255).astype(np.uint8)
    return Image.fromarray(np.dstack((red, green, blue)), "RGB"), (low, high)


def probe_p4_identity(host: str, timeout: float = P4_SCAN_TIMEOUT_S) -> dict[str, Any] | None:
    """Return verified ESP32-P4 identity, never a mere open-port result."""
    started = time.monotonic()
    try:
        board = fetch_json(host, "/board", timeout=timeout)
    except Exception:
        return None
    if board.get("board") != "ESP32-P4":
        return None
    reported_ip = str(board.get("ip", host))
    if reported_ip != host:
        return None
    return {"host": host, "latency_ms": (time.monotonic() - started) * 1000.0, "board": board}


def scan_p4_subnet(prefix: str = P4_SCAN_PREFIX, workers: int = P4_SCAN_WORKERS,
                   timeout: float = P4_SCAN_TIMEOUT_S) -> list[dict[str, Any]]:
    """Concurrently scan one /24 and return identity-verified boards by latency."""
    hosts = [f"{prefix}{last}" for last in range(1, 255)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        results = list(pool.map(lambda host: probe_p4_identity(host, timeout), hosts))
    return sorted((result for result in results if result is not None),
                  key=lambda item: (item["latency_ms"], item["host"]))


def validate_remote_absolute_targets(values: dict[str, Any]) -> dict[str, int]:
    """Validate first-stage Remote targets in the authoritative VL53 coordinate."""
    names = ("fl", "fr", "rl", "rr")
    if set(values) != set(names):
        raise ValueError("Remote目标必须完整包含FL/FR/RL/RR")
    result: dict[str, int] = {}
    for name in names:
        value = values[name]
        text = str(value).strip()
        if not text or not text.lstrip("-").isdigit():
            raise ValueError(f"{name.upper()}目标必须为整数毫米值")
        parsed = int(text)
        if not 0 <= parsed <= 400:
            raise ValueError(f"{name.upper()}Console绝对VL53目标必须在0–400 mm")
        result[name] = parsed
    return result


def build_remote_absolute_delta(
    absolute_targets: dict[str, Any], health: dict[str, Any]
) -> dict[str, int]:
    """Convert absolute VL53 targets to one fixed ARM-baseline SET_DELTA vector."""
    targets = validate_remote_absolute_targets(absolute_targets)
    lander = health.get("mega_lander", {})
    mega = health.get("mega_imu", {})
    if mega.get("mode") not in ("ARMED", "ACTIVE"):
        raise ValueError("Mega尚未确认ARM Remote")
    result: dict[str, int] = {}
    for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
        sensor = lander.get(sensor_name, {})
        if not sensor.get("valid", False):
            raise ValueError(f"{sensor_name.upper()} VL53无效")
        current = int(sensor["mm"])
        applied = int(mega.get(f"applied_{leg}", 0))
        baseline = current - applied
        if not 50 <= baseline <= 400:
            raise ValueError(f"{leg.upper()} ARM基准超出50–400 mm")
        delta = targets[leg] - baseline
        if not -350 <= delta <= 350:
            raise ValueError(f"{leg.upper()}相对ARM目标超出±350 mm协议范围")
        result[leg] = delta
    return result


def clamp_deltas_to_live_leg_range(delta: dict[str, float], health: dict[str, Any]) -> dict[str, float]:
    """Clamp each ARM-relative target to its own live 50..400 mm absolute range."""
    lander = health.get("mega_lander", {})
    mega = health.get("mega_imu", {})
    clamped: dict[str, float] = {}
    for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
        sensor = lander.get(sensor_name, {})
        if not sensor.get("valid", False):
            raise ValueError(f"{sensor_name.upper()} VL53无效")
        current = float(sensor["mm"])
        applied = float(mega.get(f"applied_{leg}", 0))
        baseline = current - applied
        minimum = 50.0
        maximum = 400.0
        clamped[leg] = max(minimum - baseline, min(maximum - baseline, float(delta[leg])))
    return clamped


def scale_relative_deltas_to_live_leg_range(
    delta: dict[str, float], health: dict[str, Any]
) -> dict[str, int]:
    """Uniformly scale a zero-common-mode CV vector into all live leg ranges.

    Independent per-leg clipping would distort terrain-relative shape and inject
    common heave. This keeps one shared scale factor for all four legs, then uses
    balanced integer rounding so the command sum remains exactly zero.
    """
    names = ("fl", "fr", "rl", "rr")
    if set(delta) != set(names):
        raise ValueError("CV relative target must contain exactly FL/FR/RL/RR")
    values = {name: float(delta[name]) for name in names}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("CV relative target must be finite")
    common = sum(values.values()) / 4.0
    if abs(common) > 0.01:
        raise ValueError("CV relative target contains common heave")
    # Remove only floating-point residue; meaningful common motion was rejected.
    values = {name: value - common for name, value in values.items()}

    lander = health.get("mega_lander", {})
    mega = health.get("mega_imu", {})
    scale = 1.0
    bounds: dict[str, tuple[float, float]] = {}
    for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
        sensor = lander.get(sensor_name, {})
        if not sensor.get("valid", False):
            raise ValueError(f"{sensor_name.upper()} VL53无效")
        baseline = float(sensor["mm"]) - float(mega.get(f"applied_{leg}", 0))
        if not 50.0 <= baseline <= 400.0:
            raise ValueError(f"{leg.upper()} ARM基准超出50–400 mm")
        lower, upper = 50.0 - baseline, 400.0 - baseline
        bounds[leg] = (lower, upper)
        value = values[leg]
        if value > 0.0:
            scale = min(scale, upper / value)
        elif value < 0.0:
            scale = min(scale, lower / value)
    scale = max(0.0, min(1.0, scale))
    exact = {name: values[name] * scale for name in names}

    # Largest-remainder integerization with target sum 0.
    rounded = {name: math.floor(exact[name]) for name in names}
    units = -sum(rounded.values())
    order = sorted(names, key=lambda name: exact[name] - rounded[name], reverse=True)
    for name in order[:units]:
        rounded[name] += 1
    if sum(rounded.values()) != 0:
        raise RuntimeError("balanced CV integerization failed")
    for name in names:
        lower, upper = bounds[name]
        if not lower <= rounded[name] <= upper:
            raise RuntimeError(f"balanced CV target exceeds {name.upper()} live range")
    return rounded


def restore_segment_deltas(health: dict[str, Any], target_mm: int = RESTORE_LEG_TARGET_MM) -> dict[str, int]:
    """Map absolute VL53 readings to one bounded FL/FR/RL/RR restore segment."""
    lander = health.get("mega_lander", {})
    result: dict[str, int] = {}
    for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
        sensor = lander.get(sensor_name, {})
        if not sensor.get("valid", False):
            raise ValueError(f"{sensor_name.upper()} VL53无效")
        current = int(sensor["mm"])
        error = target_mm - current
        result[leg] = max(-RESTORE_SEGMENT_MAX_MM, min(RESTORE_SEGMENT_MAX_MM, error))
    return result


def restore_complete(health: dict[str, Any], target_mm: int = RESTORE_LEG_TARGET_MM) -> bool:
    return all(abs(delta) <= RESTORE_LEG_TOLERANCE_MM
               for delta in restore_segment_deltas(health, target_mm).values())


@dataclass(frozen=True)
class TargetSet:
    fl: int
    fr: int
    rl: int
    rr: int

    def as_dict(self) -> dict[str, int]:
        return {"fl": self.fl, "fr": self.fr, "rl": self.rl, "rr": self.rr}

    def sensor_targets(self) -> dict[str, int]:
        return {name: value - SENSOR_OFFSET_MM for name, value in self.as_dict().items()}


def validate_solver_target(value: Any) -> int:
    """Parse and strictly validate one solver axial length."""
    try:
        parsed = int(str(value).strip())
    except (TypeError, ValueError) as exc:
        raise ValueError("目标必须为整数毫米值") from exc
    if not SOLVER_MIN_MM <= parsed <= SOLVER_MAX_MM:
        raise ValueError(f"目标必须在 {SOLVER_MIN_MM}–{SOLVER_MAX_MM} mm")
    return parsed


def build_target_set(values: dict[str, Any]) -> TargetSet:
    return TargetSet(**{name: validate_solver_target(values[name]) for name in ("fl", "fr", "rl", "rr")})


def recv_exact(sock: socket.socket, count: int) -> bytes:
    chunks: list[bytes] = []
    remaining = count
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("P4 视频连接已关闭")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def fetch_json(host: str, path: str, timeout: float = 1.5) -> dict[str, Any]:
    with urllib.request.urlopen(f"http://{host}:{HTTP_PORT}{path}", timeout=timeout) as response:
        result = json.loads(response.read().decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError(f"{path} did not return a JSON object")
    return result
def post_json(host: str, payload: dict[str, Any], timeout: float = 0.6) -> dict[str, Any]:
    request = urllib.request.Request(
        f"http://{host}:{HTTP_PORT}/cmd",
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        result = json.loads(response.read().decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("/cmd did not return a JSON object")
    return result


class ImuCommandWorker(threading.Thread):
    """Own command sequencing, heartbeat timing and latest-delta transport."""

    def __init__(self, events: queue.Queue[tuple[str, Any]]) -> None:
        super().__init__(daemon=True)
        self.events = events
        self.commands: queue.Queue[tuple[str, dict[str, Any]]] = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self._lock = threading.Lock()
        self._session_host = ""
        self._session_kind = ""
        self._session_active = False
        self._seq = 0
        self._latest_delta: dict[str, int] | None = None
        self._sent_delta: dict[str, int] | None = None
        self._next_heartbeat = 0.0
        self._consecutive_errors = 0

    @property
    def current_seq(self) -> int:
        with self._lock:
            return self._seq

    def start_session(self, host: str, kind: str) -> None:
        if kind not in ("remote", "imu"):
            raise ValueError(f"unknown session kind: {kind}")
        with self._lock:
            self._session_host = host
            self._session_kind = kind
            self._session_active = True
            self._seq = 0
            self._latest_delta = None
            self._sent_delta = None
            self._next_heartbeat = 0.0
            self._consecutive_errors = 0

    def stop_session(self) -> None:
        with self._lock:
            self._session_active = False
            self._session_kind = ""
            self._latest_delta = None
            self._sent_delta = None

    def set_latest_delta(self, host: str, delta: dict[str, float]) -> bool:
        if not host:
            return False
        rounded = {name: int(round(delta[name])) for name in ("fl", "fr", "rl", "rr")}
        with self._lock:
            if not self._session_active or self._session_host != host:
                return False
            self._latest_delta = rounded
        return True

    def submit(self, host: str, payload: dict[str, Any], *, urgent: bool = False) -> bool:
        if not host:
            return False
        if urgent:
            self.stop_session()
            while True:
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
        try:
            self.commands.put_nowait((host, payload))
            return True
        except queue.Full:
            return False

    def _send_session_command(self, host: str, payload: dict[str, Any]) -> bool:
        try:
            result = post_json(host, payload)
            self.events.put(("imu_command_result", (payload, result, None)))
            self._consecutive_errors = 0
            return True
        except Exception as exc:
            self._consecutive_errors += 1
            self.events.put(("imu_command_result", (payload, None, str(exc))))
            return False

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                host, payload = self.commands.get_nowait()
            except queue.Empty:
                host = ""
                payload = None
            if payload is not None:
                try:
                    result = post_json(host, payload)
                    self.events.put(("imu_command_result", (payload, result, None)))
                    if result.get("ok", False):
                        if payload.get("remote_arm"):
                            self.start_session(host, "remote")
                        elif payload.get("imu_arm"):
                            self.start_session(host, "imu")
                except Exception as exc:
                    self.events.put(("imu_command_result", (payload, None, str(exc))))
                continue

            now = time.monotonic()
            with self._lock:
                active = self._session_active
                session_host = self._session_host
                session_kind = self._session_kind
                next_heartbeat = self._next_heartbeat
                pending = (
                    None if self._latest_delta == self._sent_delta
                    else dict(self._latest_delta or {})
                )
            if not active:
                self.stop_event.wait(0.02)
                continue

            if now >= next_heartbeat:
                with self._lock:
                    self._seq += 1
                    seq = self._seq
                payload = {"imu_heartbeat": True, "seq": seq}
                ok = self._send_session_command(session_host, payload)
                with self._lock:
                    self._next_heartbeat = time.monotonic() + (0.15 if ok else 0.05)
                if self._consecutive_errors >= 3:
                    self.stop_session()
                continue

            if pending:
                with self._lock:
                    self._seq += 1
                    seq = self._seq
                payload = {
                    ("remote_set_absolute" if session_kind == "remote" else "imu_set_delta"): True,
                    "seq": seq,
                    **pending,
                }
                if self._send_session_command(session_host, payload):
                    with self._lock:
                        self._sent_delta = pending
                elif self._consecutive_errors >= 3:
                    self.stop_session()
                continue
            self.stop_event.wait(0.02)


class RestoreLegsWorker(threading.Thread):
    """Segmented continuous restore of all four absolute VL53 readings."""

    def __init__(self, host: str, command_worker: ImuCommandWorker,
                 events: queue.Queue[tuple[str, Any]], stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.command_worker = command_worker
        self.events = events
        self.stop_event = stop_event

    def _health(self) -> dict[str, Any]:
        last: Exception | None = None
        for _ in range(3):
            try:
                return fetch_json(self.host, "/health", timeout=2.0)
            except Exception as exc:
                last = exc
                self.stop_event.wait(0.15)
        raise RuntimeError(f"读取Mega状态失败：{last}")

    def _wait_mode(self, modes: tuple[str, ...], timeout_s: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self.stop_event.is_set():
            health = self._health()
            mega = health.get("mega_imu", {})
            if mega.get("mode") in modes:
                return health
            if mega.get("mode") in ("FAULT", "ESTOP"):
                raise RuntimeError(f"Mega进入{mega.get('mode')}：{mega.get('fault_reason')}")
            self.stop_event.wait(0.1)
        raise RuntimeError(f"等待Mega状态{modes}超时")

    def _stop_and_confirm(self) -> None:
        self.command_worker.submit(self.host, {"imu_stop": True}, urgent=True)
        self._wait_mode(("LOCKED", "DISARMED"), 4.0)

    def run(self) -> None:
        segment = 0
        try:
            while not self.stop_event.is_set():
                health = self._health()
                mega = health.get("mega_imu", {})
                if mega.get("mode") == "FAULT" and mega.get("fault_reason") == "heartbeat_timeout":
                    self._stop_and_confirm()
                    health = self._health()
                    mega = health.get("mega_imu", {})
                if mega.get("mode") not in ("LOCKED", "DISARMED") or mega.get("fault_reason") not in (None, "", "none"):
                    raise RuntimeError(f"恢复前Mega状态不安全：{mega}")
                if restore_complete(health):
                    self.events.put(("restore_done", health))
                    return

                segment += 1
                if segment > 40:
                    raise RuntimeError("恢复超过40段仍未到达150 mm")
                deltas = restore_segment_deltas(health)
                self.events.put(("restore_progress", (segment, deltas, health.get("mega_lander", {}))))
                if not self.command_worker.submit(self.host, {"imu_arm": True}):
                    raise RuntimeError("ARM队列已满")
                self._wait_mode(("ARMED", "ACTIVE"), 4.0)

                deadline = time.monotonic() + 2.0
                while not self.command_worker.set_latest_delta(self.host, deltas):
                    if time.monotonic() >= deadline:
                        raise RuntimeError("恢复命令泵未进入ARM会话")
                    self.stop_event.wait(0.05)

                target_confirmed = False
                deadline = time.monotonic() + 8.0
                while time.monotonic() < deadline and not self.stop_event.is_set():
                    health = self._health()
                    mega = health.get("mega_imu", {})
                    if mega.get("mode") in ("FAULT", "ESTOP"):
                        raise RuntimeError(f"恢复中Mega故障：{mega.get('fault_reason')}")
                    mirrored = {name: int(mega.get(f"target_{name}", 999))
                                for name in ("fl", "fr", "rl", "rr")}
                    target_confirmed = target_confirmed or mirrored == deltas
                    reached = target_confirmed and all(
                        abs(int(mega.get(f"applied_{name}", 999)) - deltas[name]) <= RESTORE_LEG_TOLERANCE_MM
                        for name in ("fl", "fr", "rl", "rr")
                    )
                    if reached and mega.get("mode") == "ARMED":
                        break
                    self.stop_event.wait(0.08)
                else:
                    if not self.stop_event.is_set():
                        raise RuntimeError(f"第{segment}段未在8秒内到位")
                self._stop_and_confirm()

            self._stop_and_confirm()
            self.events.put(("restore_cancelled", None))
        except Exception as exc:
            try:
                self._stop_and_confirm()
            except Exception:
                pass
            self.events.put(("restore_error", str(exc)))


class ImuStreamReader(threading.Thread):
    """Receive P4's independent TCP 5001 BNO085 JSONL stream."""

    def __init__(self, host: str, stop_event: threading.Event, generation: int,
                 events: queue.Queue[tuple[str, Any]],
                 sample_sink: Callable[[int, dict[str, Any]], None] | None = None) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.stop_event = stop_event
        self.generation = generation
        self.events = events
        self.sample_sink = sample_sink
        self.sock: socket.socket | None = None

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.sock = socket.create_connection((self.host, IMU_TCP_PORT), timeout=4)
                self.sock.settimeout(None)
                self.events.put(("imu_stream_state", (self.generation, "TCP 5001 独立 IMU 流已连接")))
                with self.sock.makefile("rb") as stream:
                    while not self.stop_event.is_set():
                        raw = stream.readline()
                        if not raw:
                            raise ConnectionError("P4 独立 IMU 流已关闭")
                        record = json.loads(raw.decode("utf-8"))
                        if "timestamp_us" not in record or "valid_packet_count" not in record:
                            raise ValueError("independent IMU JSONL 缺少时间戳或健康计数")
                        if self.sample_sink is not None:
                            self.sample_sink(self.generation, record)
                        else:
                            self.events.put(("imu_stream", (self.generation, record)))
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.events.put(("imu_stream_state", (self.generation, f"TCP 5001 重连中：{exc}")))
                    self.stop_event.wait(1.0)
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                    self.sock = None


def stabilize_ultrasonic_samples(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Return a robust independent-sample SR04 anchor or reject an unstable window."""
    import numpy as np
    from absolute_terrain import (
        ULTRASONIC_FILTER_MAX_ABSOLUTE_SPAN_MM,
        ULTRASONIC_FILTER_MAX_RELATIVE_MAD,
        ULTRASONIC_FILTER_MAX_RELATIVE_SPAN,
        ULTRASONIC_FILTER_MIN_SAMPLES,
    )
    valid = [sample for sample in samples if sample.get("valid", False)]
    if len(valid) < ULTRASONIC_FILTER_MIN_SAMPLES:
        raise ValueError("ultrasonic anchor warming up")
    values = [float(sample["distance_mm"]) for sample in valid]
    median = float(np.median(values))
    mad = float(np.median(np.abs(np.asarray(values) - median)))
    span = float(max(values) - min(values))
    relative_mad = mad / max(median, 1.0)
    relative_span = span / max(median, 1.0)
    if relative_mad > ULTRASONIC_FILTER_MAX_RELATIVE_MAD:
        raise ValueError("ultrasonic window MAD unstable")
    if span > ULTRASONIC_FILTER_MAX_ABSOLUTE_SPAN_MM and relative_span > ULTRASONIC_FILTER_MAX_RELATIVE_SPAN:
        raise ValueError("ultrasonic window span unstable")
    newest = dict(valid[-1])
    newest["distance_mm"] = median
    newest["raw_latest_mm"] = float(valid[-1]["distance_mm"])
    newest["filter_samples"] = len(valid)
    newest["filter_mad_mm"] = mad
    newest["filter_span_mm"] = span
    return newest


class DepthEstimator(threading.Thread):
    """Two-model metric visualization worker; no hardware-control interface exists."""

    def __init__(self, stop_event: threading.Event, generation: int,
                 events: queue.Queue[tuple[str, Any]]) -> None:
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.generation = generation
        self.events = events
        self.frames: queue.Queue[Image.Image] = queue.Queue(maxsize=1)
        self.capture = BestOfThirtyDepthCapture()
        self.capture_anchor_meta: list[dict[str, Any]] = []
        self.capture_requested = threading.Event()
        self._ultrasonic_lock = threading.Lock()
        self._ultrasonic: dict[str, Any] | None = None
        self._ultrasonic_samples: deque[dict[str, Any]] = deque(maxlen=ULTRASONIC_FILTER_WINDOW)
        self._ultrasonic_last_count: int | None = None

    def set_ultrasonic(self, reading: dict[str, Any] | None) -> None:
        with self._ultrasonic_lock:
            self._ultrasonic = dict(reading) if isinstance(reading, dict) else None
            if not isinstance(reading, dict) or not reading.get("valid", False):
                self._ultrasonic_samples.clear()
                self._ultrasonic_last_count = None
                return
            count = int(reading.get("valid_count", -1))
            if count < 0 or count == self._ultrasonic_last_count:
                return
            self._ultrasonic_last_count = count
            self._ultrasonic_samples.append(dict(reading))

    def _ultrasonic_snapshot(self) -> dict[str, Any] | None:
        with self._ultrasonic_lock:
            latest = dict(self._ultrasonic) if self._ultrasonic is not None else None
            samples = [dict(item) for item in self._ultrasonic_samples]
        if latest is None or not latest.get("valid", False):
            return latest
        return stabilize_ultrasonic_samples(samples)

    def start_capture(self) -> None:
        """Start a fresh bounded capture; safe to call from the Tk thread."""
        self.capture_requested.set()

    def submit(self, image: Image.Image) -> None:
        try:
            self.frames.put_nowait(image.copy())
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            try:
                self.frames.put_nowait(image.copy())
            except queue.Full:
                pass

    def run(self) -> None:
        try:
            import cv2
            import numpy as np
            import torch
            from transformers import pipeline

            if not torch.cuda.is_available():
                raise RuntimeError("CUDA unavailable")
            if not METRIC_DEPTH_CHECKPOINT.is_file():
                raise FileNotFoundError(METRIC_DEPTH_CHECKPOINT)
            if str(METRIC_DEPTH_SOURCE) not in sys.path:
                sys.path.insert(0, str(METRIC_DEPTH_SOURCE))
            from depth_anything_v2.dpt import DepthAnythingV2

            calibration = load_provisional_calibration()
            self.events.put(("depth_state", (self.generation, "正在加载两路深度模型与暂定标定…")))
            if self.stop_event.is_set():
                return
            relative = pipeline("depth-estimation", model=RELATIVE_DEPTH_MODEL_ID, device="cuda", dtype=torch.float16)
            if self.stop_event.is_set():
                return
            metric = DepthAnythingV2(encoder="vitb", features=128, out_channels=[96, 192, 384, 768], max_depth=20)
            metric.load_state_dict(torch.load(METRIC_DEPTH_CHECKPOINT, map_location="cpu", weights_only=True), strict=True)
            if self.stop_event.is_set():
                del relative, metric
                torch.cuda.empty_cache()
                return
            metric = metric.to("cuda").eval()
            relative(Image.new("RGB", (512, 512), "black"))
            with torch.inference_mode():
                metric.infer_image(np.zeros((800, 800, 3), dtype=np.uint8), METRIC_DEPTH_INPUT_SIZE)
            torch.cuda.synchronize()
            self.events.put(("depth_state", (
                self.generation,
                "两路深度已就绪：中栏实时云图；右栏采集30帧并选择最好的10帧生成等高线",
            )))
            while not self.stop_event.is_set():
                if self.capture_requested.is_set():
                    self.capture_requested.clear()
                    self.capture.start()
                    self.capture_anchor_meta.clear()
                    # Discard a queued pre-click frame so the capture begins with
                    # imagery received after the user's explicit request.
                    try:
                        self.frames.get_nowait()
                    except queue.Empty:
                        pass
                    self.events.put(("capture_state", (self.generation, 0, CAPTURE_FRAME_COUNT, False)))
                try:
                    image = self.frames.get(timeout=0.25)
                except queue.Empty:
                    continue
                started = time.monotonic()
                rgb = np.asarray(image.convert("RGB"))
                source_quality = source_observability(rgb)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                with torch.inference_mode():
                    metric_map = metric.infer_image(bgr, METRIC_DEPTH_INPUT_SIZE).astype(np.float32)
                relative_map = relative(image)["predicted_depth"]
                relative_map = relative_map.detach().float().cpu().numpy().astype(np.float32)
                if relative_map.shape != metric_map.shape:
                    relative_map = cv2.resize(relative_map, (metric_map.shape[1], metric_map.shape[0]), interpolation=cv2.INTER_LINEAR)
                fused_map, _valid, model_scale = fuse_metric_and_relative(metric_map, relative_map)
                anchor_meta = None
                anchor_error = None
                try:
                    fused_map, anchor_meta = anchor_depth_with_ultrasonic(
                        fused_map, self._ultrasonic_snapshot(),
                        cx_px=calibration["cx_px"], cy_px=calibration["cy_px"],
                    )
                except ValueError as exc:
                    anchor_error = str(exc)
                live_heatmap, live_near_m, live_far_m = metric_depth_heatmap(fused_map)
                live_heatmap = live_heatmap.resize(image.size, Image.Resampling.BILINEAR)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                live_summary = ground_height_summary(fused_map)
                self.events.put(("live_depth", (self.generation, live_heatmap, elapsed_ms, {
                    "summary": live_summary, "model_scale": model_scale,
                    "ultrasonic_anchor": anchor_meta, "anchor_error": anchor_error,
                    "source_observability": source_quality,
                    "near_m": live_near_m, "far_m": live_far_m,
                })))

                if not source_quality["observable"]:
                    if self.capture.active:
                        self.events.put(("capture_source_wait", (self.generation, source_quality)))
                    continue
                if anchor_meta is None:
                    capture_frame = fused_map
                    capture_anchor = {
                        "anchored": False, "scale": 1.0,
                        "ultrasonic_mm": None, "anchor_error": anchor_error,
                    }
                else:
                    capture_frame = fused_map
                    capture_anchor = {"anchored": True, **anchor_meta}
                accepted = self.capture.accept(capture_frame)
                if not accepted:
                    continue
                self.capture_anchor_meta.append(dict(capture_anchor))
                capture_count = self.capture.count
                complete = capture_count == CAPTURE_FRAME_COUNT
                self.events.put(("capture_state", (self.generation, capture_count, CAPTURE_FRAME_COUNT, complete)))
                if not complete:
                    continue

                best_frames, selected_indices, selection_scores = select_best_depth_frames(
                    self.capture.frames, SELECTED_FRAME_COUNT
                )
                stable_cloud = stable_cloud_average(best_frames)
                if stable_cloud is None:
                    self.events.put(("capture_failed", (
                        self.generation,
                        "已采集30帧并选出最佳10帧，但其像素波动仍超过20%稳定阈值；请重试",
                    )))
                    continue
                terrain = DepthTerrainHeightField(
                    stable_cloud,
                    fx_px=calibration["fx_px"], fy_px=calibration["fy_px"],
                    cx_px=calibration["cx_px"], cy_px=calibration["cy_px"],
                    projection_model=calibration["projection_model"],
                    distortion_coefficients=calibration.get("D"),
                )
                leg_plan = solve_best_visible_platform(terrain.get_height)
                contour, contour_levels = metric_contour_map(stable_cloud)
                contour = contour.resize(image.size, Image.Resampling.NEAREST)
                summary = ground_height_summary(stable_cloud)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                anchored_meta = [item for item in self.capture_anchor_meta if item.get("anchored")]
                anchor_scales = [item["scale"] for item in anchored_meta]
                anchor_distances = [item["ultrasonic_mm"] for item in anchored_meta]
                anchor_summary = {
                    "frames": len(self.capture_anchor_meta),
                    "anchored_frames": len(anchored_meta),
                    "metric_source": "ultrasonic_anchored" if len(anchored_meta) == len(self.capture_anchor_meta) else "model_only",
                    "scale_median": float(np.median(anchor_scales)) if anchor_scales else None,
                    "scale_min": float(min(anchor_scales)) if anchor_scales else None,
                    "scale_max": float(max(anchor_scales)) if anchor_scales else None,
                    "ultrasonic_median_mm": float(np.median(anchor_distances)) if anchor_distances else None,
                    "ultrasonic_min_mm": float(min(anchor_distances)) if anchor_distances else None,
                    "ultrasonic_max_mm": float(max(anchor_distances)) if anchor_distances else None,
                }
                self.events.put(("capture_result", (self.generation, contour, elapsed_ms, {
                    "captured_count": capture_count, "selected_count": len(best_frames),
                    "selected_indices": selected_indices, "selection_scores": selection_scores,
                    "stable": True, "summary": summary, "model_scale": model_scale,
                    "ultrasonic_anchor": anchor_summary,
                    "leg_plan": leg_plan,
                    "camera_ground_depth_mm": terrain.camera_ground_depth_mm,
                    "current_mount_height_mm": terrain.current_mount_height_mm,
                    "contour_levels_m": contour_levels,
                    "projection_model": calibration["projection_model"],
                    "calibrated_intrinsics": calibration.get("calibrated", False),
                    "diagonal_fov_deg": calibration.get("diagonal_fov_deg"),
                    "calibration_warning": calibration["warning"],
                })))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.events.put(("depth_state", (self.generation, f"绝对高度估计不可用：{exc}")))


class LanderConsole(tk.Tk):
    def __init__(self, initial_host: str = "") -> None:
        enable_windows_high_dpi()
        super().__init__()
        self.ui_scaling = configure_tk_scaling(self)
        self.title(APP_TITLE)
        screen_w = self.winfo_screenwidth()
        screen_h = self.winfo_screenheight()
        default_w = max(1200, min(1800, int(screen_w * 0.82)))
        default_h = max(760, min(1200, int(screen_h * 0.82)))
        self.geometry(f"{default_w}x{default_h}")
        self.minsize(min(1200, default_w), min(760, default_h))
        try:
            self.state("zoomed")  # Full desktop on Windows, while preserving a 3200×1800 design grid.
        except tk.TclError:
            pass
        self.configure(bg=DARK)

        self.host_var = tk.StringVar(value=initial_host)
        self.connection_var = tk.StringVar(value="未连接")
        self.control_enabled = tk.BooleanVar(value=False)
        self.control_mode_var = tk.StringVar(value=CONTROL_MODE_LOCKED)
        self.status_var = tk.StringVar(value="准备就绪：锁定控制，不会自动发送推杆命令。")
        self.camera_var = tk.StringVar(value="相机：未连接")
        self.imu_stream_var = tk.StringVar(value="独立 IMU / TCP 5001：未连接")
        self.ultrasonic_var = tk.StringVar(value="超声波：本次 P4 独立测试已禁用")
        self.mega_var = tk.StringVar(value="Mega：未连接")
        self.safety_var = tk.StringVar(value="安全：P4 本地 PWM 保持锁定")
        self.fps_var = tk.StringVar(value="视频：0.0 FPS")
        self.target_vars = {name: tk.StringVar(value="150") for name in ("fl", "fr", "rl", "rr")}
        self.target_entries: dict[str, ttk.Entry] = {}
        self._remote_targets_initialized = False
        self.sensor_vars = {name: tk.StringVar(value="—") for name in ("fl", "fr", "rl", "rr")}
        self.leg_status_vars = {name: tk.StringVar(value="等待解算") for name in ("fl", "fr", "rl", "rr")}
        self.imu_armed = tk.BooleanVar(value=False)
        self.mega_imu_mode = tk.StringVar(value="UNKNOWN")
        self.mega_heartbeat_seq = tk.IntVar(value=0)
        self.mega_fault_count = tk.IntVar(value=0)
        self.cv_candidate_delta: dict[str, float] | None = None
        self.cv_candidate_time: float | None = None
        self.cv_armed = False
        self._arm_owner: str | None = None

        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.imu_command_worker = ImuCommandWorker(self.events)
        self.imu_command_worker.start()
        self._imu_command_seq = 0
        self._last_heartbeat_send = 0.0
        self._last_imu_receive_monotonic: float | None = None
        self._mega_arm_confirmed = False
        self._arm_request_pending = False
        self._arm_request_time: float | None = None
        self._last_mega_ack_seq = 0
        self._last_mega_ack_time = time.monotonic()
        self._highest_sent_seq = 0
        self.imu_platform_transform: str | None = "swap_neg"
        self.imu_roll_zero_deg = -180.0
        self.imu_pitch_zero_deg = 0.0
        # IMU is ~100 Hz; keep only the newest sample instead of flooding Tk's event queue.
        self._latest_imu_lock = threading.Lock()
        self._pending_imu_sample: tuple[int, dict[str, Any]] | None = None
        self._imu_rx_count = 0
        self._imu_ui_count = 0
        self._imu_ui_times: deque[float] = deque()
        self.stop_event = threading.Event()
        self.depth_stop_event = threading.Event()
        self.video_stop_event = threading.Event()
        self.video_sock: socket.socket | None = None
        self.video_thread: threading.Thread | None = None
        self.poll_thread: threading.Thread | None = None
        self.imu_stream_thread: ImuStreamReader | None = None
        self.depth_thread: DepthEstimator | None = None
        self.depth_status_var = tk.StringVar(value="绝对高度模型：等待启动")
        self.ground_var = tk.StringVar(value="实时云图：等待双深度模型就绪")
        self.capture_var = tk.StringVar(value="等待采集")
        self.panel_title_vars = [tk.StringVar(value=name) for name in CV_VIEW_NAMES]
        self.restore_status_var = tk.StringVar(value="恢复：待命")
        self.restore_stop_event = threading.Event()
        self.restore_thread: RestoreLegsWorker | None = None
        self.scan_thread: threading.Thread | None = None
        self.latest_health: dict[str, Any] = {}
        self.latest_imu_stream: dict[str, Any] = {}
        self.imu_stream_records: deque[dict[str, Any]] = deque(maxlen=120)
        self._video_generation = 0

        self._build_style()
        self._build_layout()
        self.update_idletasks()
        # Prevent manual resizing below the real high-DPI content bounds.
        required_w = max(1100, self.winfo_reqwidth())
        required_h = max(700, self.winfo_reqheight())
        self.minsize(min(required_w, max(1100, screen_w - 80)),
                     min(required_h, max(700, screen_h - 120)))
        self.protocol("WM_DELETE_WINDOW", self._close)
        self.after(75, self._drain_events)
        self.after(20, self._refresh_imu_display)
        self._update_target_preview()
        if not initial_host.strip():
            self.after(150, self.scan_p4)

    def _build_style(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=DARK)
        style.configure("Panel.TFrame", background=PANEL, borderwidth=1, relief="solid")
        style.configure("TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 13))
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 11))
        style.configure("Header.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI Semibold", 19))
        style.configure("Title.TLabel", background=DARK, foreground=TEXT, font=("Segoe UI Semibold", 27))
        style.configure("TButton", font=("Segoe UI Semibold", 11), padding=(12, 7))
        style.configure("Compact.TButton", font=("Segoe UI Semibold", 10), padding=(9, 6))
        style.configure("Primary.TButton", foreground="#081016", background=CYAN)
        style.map("Primary.TButton", background=[("active", "#a3e7ff")])
        style.configure("CompactPrimary.TButton", foreground="#081016", background=CYAN,
                        font=("Segoe UI Semibold", 10), padding=(9, 6))
        style.map("CompactPrimary.TButton", background=[("active", "#a3e7ff")])
        style.configure("Danger.TButton", foreground="#220b0b", background="#ff9494")
        style.map("Danger.TButton", background=[("active", "#ffc0c0")])
        style.configure("CompactDanger.TButton", foreground="#220b0b", background="#ff9494",
                        font=("Segoe UI Semibold", 10), padding=(9, 6))
        style.map("CompactDanger.TButton", background=[("active", "#ffc0c0")])
        style.configure("TEntry", fieldbackground="#0c1119", foreground=TEXT, insertcolor=TEXT)
        style.configure("Compact.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 11))
        style.configure("CompactMuted.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 10))
        style.configure("CompactHeader.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI Semibold", 16))
        style.configure("Compact.TRadiobutton", background=PANEL, foreground=TEXT, font=("Segoe UI", 10))
        style.map("Compact.TRadiobutton", background=[("active", PANEL)])
        style.configure("Compact.TEntry", fieldbackground="#0c1119", foreground=TEXT, insertcolor=TEXT,
                        font=("Segoe UI", 10), padding=2)
        style.configure("TCheckbutton", background=PANEL, foreground=TEXT, font=("Segoe UI", 10))
        style.map("TCheckbutton", background=[("active", PANEL)])

    @staticmethod
    def _panel(parent: tk.Misc, padding: int = 14) -> ttk.Frame:
        return ttk.Frame(parent, style="Panel.TFrame", padding=padding)

    def _build_layout(self) -> None:
        """Use the upper two thirds for three equal CV views; telemetry stays below."""
        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=4, minsize=420)
        root.rowconfigure(2, weight=0, minsize=210)

        top = ttk.Frame(root)
        top.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        top.columnconfigure(0, weight=0)
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="MARS LANDER", style="Title.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(top, text="CV · IMU · 四腿连续控制", foreground=MUTED, background=DARK,
                  font=("Segoe UI", 13)).grid(row=0, column=1, sticky="w", padx=14)
        connect = self._panel(top, 6)
        connect.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        connect.columnconfigure(1, weight=1)
        ttk.Label(connect, text="P4 IP", style="Muted.TLabel").grid(row=0, column=0, padx=(0, 6))
        ip = ttk.Entry(connect, textvariable=self.host_var, width=18)
        ip.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ip.bind("<Return>", lambda _event: self.connect())
        self.connect_button = ttk.Button(
            connect, text="连接 / 重连", style="Primary.TButton", command=self.connect
        )
        self.connect_button.grid(row=0, column=2, padx=(0, 6))
        self.scan_button = ttk.Button(
            connect, text="自动扫描P4", command=self.scan_p4
        )
        self.scan_button.grid(row=0, column=3)

        cv_area = ttk.Frame(root)
        cv_area.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        cv_area.rowconfigure(0, weight=1)
        self.cv_labels: list[tk.Label] = []
        self.cv_photos: list[ImageTk.PhotoImage | None] = [None, None, None]
        for column, name in enumerate(CV_VIEW_NAMES):
            cv_area.columnconfigure(column, weight=1, uniform="cv")
            panel = self._panel(cv_area, 10)
            panel.grid(row=0, column=column, sticky="nsew", padx=(0 if column == 0 else 6, 0 if column == 2 else 6))
            panel.columnconfigure(0, weight=1)
            panel.rowconfigure(1, weight=1)
            header = ttk.Frame(panel)
            header.grid(row=0, column=0, sticky="ew", pady=(0, 6))
            header.columnconfigure(0, weight=1)
            ttk.Label(header, textvariable=self.panel_title_vars[column], style="Header.TLabel").grid(row=0, column=0, sticky="w")
            if column == 2:
                self.capture_controls = ttk.Frame(header)
                self.capture_controls.grid(row=0, column=1, sticky="e")
                self.capture_button = ttk.Button(
                    self.capture_controls, text="开始采集", style="Primary.TButton", command=self._start_depth_capture
                )
                self.capture_button.grid(row=0, column=0, sticky="e")
                ttk.Label(self.capture_controls, textvariable=self.capture_var, style="Muted.TLabel").grid(
                    row=0, column=1, sticky="e", padx=(12, 0)
                )
            label = tk.Label(panel, bg="#080d14", fg=MUTED, text=f"等待 {name}…",
                             font=("Segoe UI", 15), anchor="center", justify="center",
                             padx=14, pady=14, bd=0, highlightthickness=1,
                             highlightbackground="#2b394a")
            label.grid(row=1, column=0, sticky="nsew")
            label.bind("<Configure>", lambda event, target=label: target.configure(
                wraplength=max(240, target.winfo_width() - 40)
            ))
            self.cv_labels.append(label)
        ttk.Label(cv_area, textvariable=self.fps_var, foreground=MUTED, background=DARK,
                  font=("Segoe UI", 12)).grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(cv_area, textvariable=self.ground_var, foreground=AMBER, background=DARK,
                  font=("Segoe UI", 12)).grid(row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Label(cv_area, textvariable=self.depth_status_var, foreground=CYAN, background=DARK,
                  font=("Segoe UI", 12)).grid(row=1, column=2, sticky="e", pady=(6, 0))

        lower = ttk.Frame(root)
        lower.grid(row=2, column=0, sticky="nsew")
        lower.columnconfigure(0, weight=11, minsize=250)
        lower.columnconfigure(1, weight=10, minsize=300)
        lower.columnconfigure(2, weight=9, minsize=260)
        lower.rowconfigure(0, weight=1)

        telemetry = self._panel(lower, 12)
        telemetry.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        ttk.Label(telemetry, text="遥测状态", style="CompactHeader.TLabel").grid(row=0, column=0, columnspan=2, sticky="w")
        self._status_row(telemetry, 1, "连接", self.connection_var)
        self._status_row(telemetry, 2, "相机", self.camera_var)
        self._status_row(telemetry, 3, "独立 IMU", self.imu_stream_var)
        self._status_row(telemetry, 4, "安全", self.safety_var)

        mode_panel = self._panel(lower, 12)
        mode_panel.grid(row=0, column=1, sticky="nsew", padx=6)
        ttk.Label(mode_panel, text="控制模式", style="CompactHeader.TLabel").grid(row=0, column=0, sticky="w")
        for row, (value, label) in enumerate(((CONTROL_MODE_CV, "CV 控制"),
                                              (CONTROL_MODE_LOCKED, "锁定控制"),
                                              (CONTROL_MODE_IMU, "IMU 控制")), start=1):
            ttk.Radiobutton(mode_panel, text=label, value=value, variable=self.control_mode_var,
                            style="Compact.TRadiobutton", command=self._on_control_mode_changed).grid(
                                row=row, column=0, sticky="w", pady=1)
        ttk.Label(mode_panel, text="模式切换数据源；动作仅由明确ARM/恢复触发。",
                  style="CompactMuted.TLabel", wraplength=300).grid(row=4, column=0, sticky="w", pady=(4, 0))

        imu_control_frame = ttk.Frame(mode_panel)
        imu_control_frame.grid(row=5, column=0, sticky="ew", pady=(7, 0))
        imu_control_frame.columnconfigure(0, weight=1, uniform="imu_actions")
        imu_control_frame.columnconfigure(1, weight=1, uniform="imu_actions")
        self.arm_button = ttk.Button(
            imu_control_frame, text="ARM IMU",
            style="CompactPrimary.TButton", command=self._arm_current_mode
        )
        self.arm_button.grid(row=0, column=0, sticky="ew", padx=(0, 3), pady=(0, 3))
        self.stop_button = ttk.Button(
            imu_control_frame, text="STOP",
            style="CompactDanger.TButton", command=self._stop_imu
        )
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=(3, 0), pady=(0, 3))
        self.estop_button = ttk.Button(
            imu_control_frame, text="ESTOP",
            style="CompactDanger.TButton", command=self._estop_imu
        )
        self.estop_button.grid(row=1, column=0, sticky="ew", padx=(0, 3), pady=(3, 0))
        self.restore_button = ttk.Button(
            imu_control_frame, text="一键恢复 150mm",
            style="CompactPrimary.TButton", command=self._restore_legs_to_150
        )
        self.restore_button.grid(row=1, column=1, sticky="ew", padx=(3, 0), pady=(3, 0))
        self.imu_mode_label = ttk.Label(mode_panel, textvariable=self.mega_imu_mode,
                                        style="CompactMuted.TLabel")
        self.imu_mode_label.grid(row=6, column=0, sticky="w", pady=(5, 0))
        ttk.Label(mode_panel, textvariable=self.restore_status_var,
                  style="CompactMuted.TLabel", wraplength=300).grid(row=7, column=0, sticky="w", pady=(2, 0))

        target_panel = self._panel(lower, 12)
        target_panel.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        self.target_panel_title = ttk.Label(
            target_panel, text="Remote四腿绝对位置", style="CompactHeader.TLabel"
        )
        self.target_panel_title.grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(target_panel, text="腿", style="CompactMuted.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Label(target_panel, text="目标VL53 mm", style="CompactMuted.TLabel").grid(row=1, column=1, sticky="w")
        ttk.Label(target_panel, text="当前VL53", style="CompactMuted.TLabel").grid(row=1, column=2, sticky="w")
        ttk.Label(target_panel, text="映射/状态", style="CompactMuted.TLabel").grid(row=1, column=3, sticky="w")
        names = (("fl", "前左"), ("fr", "前右"), ("rl", "后左"), ("rr", "后右"))
        for row, (name, title) in enumerate(names, start=2):
            ttk.Label(target_panel, text=title, style="Compact.TLabel").grid(row=row, column=0, sticky="w", pady=1)
            entry = ttk.Entry(target_panel, width=7, textvariable=self.target_vars[name],
                              style="Compact.TEntry", state="readonly")
            entry.grid(row=row, column=1, sticky="w", padx=(8, 10), pady=1)
            self.target_entries[name] = entry
            ttk.Label(target_panel, textvariable=self.sensor_vars[name], foreground=CYAN,
                      background=PANEL, font=("Segoe UI Semibold", 9)).grid(row=row, column=2, sticky="w", pady=1)
            ttk.Label(target_panel, textvariable=self.leg_status_vars[name], style="CompactMuted.TLabel").grid(
                row=row, column=3, sticky="w", padx=(8, 0), pady=1)
        self.remote_apply_button = ttk.Button(
            target_panel, text="应用四腿目标", style="CompactPrimary.TButton",
            command=self._apply_remote_targets,
        )
        self.remote_apply_button.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(5, 0), padx=(0, 4))
        self.target_note = ttk.Label(
            target_panel,
            text="第一阶段Remote：输入四路VL53绝对目标0–400 mm；先ARM Remote，再手动应用。自动CV仅预览。",
            style="CompactMuted.TLabel", wraplength=360,
        )
        self.target_note.grid(
            row=7, column=0, columnspan=4, sticky="w", pady=(3, 0))

        footer = tk.Label(root, textvariable=self.status_var, bg="#0b1017", fg=TEXT,
                          anchor="w", padx=14, pady=6, font=("Segoe UI", 10))
        footer.grid(row=3, column=0, sticky="ew", pady=(10, 0))

    @staticmethod
    def _status_row(parent: ttk.Frame, row: int, label: str, variable: tk.StringVar) -> None:
        ttk.Label(parent, text=label, style="CompactMuted.TLabel").grid(
            row=row, column=0, sticky="nw", padx=(0, 10), pady=1)
        ttk.Label(parent, textvariable=variable, style="Compact.TLabel", wraplength=430).grid(
            row=row, column=1, sticky="w", pady=1)

    @staticmethod
    def _show_panel_message(label: tk.Label, message: str) -> None:
        """Show a centered, readable multi-line panel status/error message."""
        label.configure(
            image="", text=message, justify="center", anchor="center",
            wraplength=max(240, label.winfo_width() - 40),
        )

    def _start_depth_capture(self) -> None:
        """Request exactly thirty future preview frames; never affects Remote control."""
        self.cv_candidate_delta = None
        self.cv_candidate_time = None
        if self.depth_thread is None or not self.depth_thread.is_alive():
            self.capture_var.set("模型未就绪")
            self.status_var.set("无法采集：深度模型线程尚未运行，请先连接 P4并等待模型就绪。")
            return
        self.depth_thread.start_capture()
        self.capture_button.state(["disabled"])
        self.capture_var.set(f"采集中 0/{CAPTURE_FRAME_COUNT}")
        self.cv_photos[2] = None
        self._show_panel_message(self.cv_labels[2], "正在采集30帧；完成后选择最佳10帧生成等高线…")
        self.status_var.set("已开始右栏采集：中栏实时云图继续更新；30帧完成后选最佳10帧生成等高线。")

    def _arm_current_mode(self) -> None:
        mode = self.control_mode_var.get()
        if mode == CONTROL_MODE_IMU:
            self._arm_imu()
        elif mode == CONTROL_MODE_CV:
            self._arm_remote()
        else:
            self.status_var.set("锁定模式拒绝ARM。")

    def _arm_remote(self) -> None:
        """ARM first-stage Remote mode; never send a target automatically."""
        host = self.host_var.get().strip()
        if self.control_mode_var.get() != CONTROL_MODE_CV:
            self.status_var.set("只有CV控制（第一阶段Remote）模式可以ARM Remote。")
            return
        if not host or not self._check_remote_health(require_owner=False):
            self.status_var.set("ARM Remote已阻止：P4/Mega/四路VL53健康门槛未满足或Mega未LOCKED。")
            return
        if self._arm_owner is not None:
            self.status_var.set("ARM Remote已阻止：已有会话；请先STOP。")
            return
        if not messagebox.askyesno(
                "确认 ARM Remote",
                "启用111(1).ino的原PS2遥控逻辑？\n"
                "START切换MANUAL/PRESET，16组预设及按键映射保持原源码。\n"
                "ARM不会自动移动；右下栏也可显式提交四腿绝对VL53目标。"):
            return
        self.imu_armed.set(False)
        self.cv_armed = False
        self._arm_owner = "remote"
        self._mega_arm_confirmed = False
        self._arm_request_pending = True
        self._arm_request_time = time.monotonic()
        if self.imu_command_worker.submit(host, {"remote_arm": True}):
            self.status_var.set("Remote ARM请求已排队；等待Mega启用原PS2手动/16预设逻辑。")
        else:
            self._arm_owner = None
            self._arm_request_pending = False
            self._arm_request_time = None
            self.status_var.set("Remote ARM队列已满，未发送。")

    def _apply_remote_targets(self) -> None:
        """Submit four absolute VL53 targets only in an already confirmed Remote session."""
        if self.control_mode_var.get() != CONTROL_MODE_CV or self._arm_owner != "remote":
            self.status_var.set("应用已阻止：请先在CV控制模式ARM Remote。")
            return
        if not self._mega_arm_confirmed or not self._check_remote_health(require_owner=True):
            self.status_var.set("应用已阻止：Mega尚未确认REMOTE owner或四路VL53健康状态失效。")
            return
        try:
            absolute = validate_remote_absolute_targets(
                {name: variable.get() for name, variable in self.target_vars.items()}
            )
        except (ValueError, KeyError, TypeError) as exc:
            self.status_var.set(f"Remote目标无效：{exc}")
            return
        if not messagebox.askyesno(
                "确认应用四腿目标",
                f"四路VL53绝对目标(mm)：{absolute}\n"
                "目标将进入与原16组预设相同的Mega绝对位置闭环；期间PS2输入暂停。\n"
                "确认机构周围无夹点风险？"):
            return
        if self._queue_imu_delta(absolute):
            self.cv_armed = True
            self.status_var.set(f"Remote四腿绝对目标已提交：{absolute}")
        else:
            self.status_var.set("Remote目标排队失败；正在STOP。")
            self._queue_imu_stop()

    def _arm_cv(self) -> None:
        """Automatic CV actuation is structurally disabled for the whole first stage."""
        self.status_var.set(
            "自动CV执行器控制在第一阶段已禁用；请使用ARM Remote和右下栏四腿绝对VL53目标。"
        )

    def _arm_imu(self) -> None:
        """Queue ARM; local armed state is set only after Mega reports ARMED."""
        host = self.host_var.get().strip()
        if self.restore_thread is not None and self.restore_thread.is_alive():
            self.status_var.set("恢复正在运行；请先STOP后再ARM IMU。")
            return
        if self.control_mode_var.get() != CONTROL_MODE_IMU:
            self.status_var.set("只有IMU控制模式可以ARM。")
            return
        if self.imu_platform_transform not in IMU_ORTHOGONAL_TRANSFORMS:
            self.status_var.set("ARM已阻止：尚未完成IMU与平台前后左右的正交辨识。")
            return
        if not host or not self._check_mega_health(require_armed=False):
            self.status_var.set("ARM已阻止：P4/Mega/四传感器健康门槛未满足。")
            return
        if self._arm_owner is not None or self.latest_health.get("mega_imu", {}).get("mode") not in ("LOCKED", "DISARMED"):
            self.status_var.set("ARM IMU已阻止：已有会话或Mega未处于LOCKED；请先STOP。")
            return
        if not messagebox.askyesno("确认 ARM IMU", "确认允许IMU闭环控制四条腿？\nSTOP与心跳超时仍由Mega本地执行。"):
            return
        self.imu_armed.set(False)
        self.cv_armed = False
        self._arm_owner = "imu"
        self._mega_arm_confirmed = False
        self._arm_request_pending = True
        self._arm_request_time = time.monotonic()
        self._last_mega_ack_seq = int(self.mega_heartbeat_seq.get())
        self._last_mega_ack_time = time.monotonic()
        self._highest_sent_seq = self._imu_command_seq
        if self.imu_command_worker.submit(host, {"imu_arm": True}):
            self.status_var.set("ARM请求已排队；等待Mega状态确认。")
        else:
            self._arm_owner = None
            self._arm_request_pending = False
            self._arm_request_time = None
            self.status_var.set("ARM队列已满，未发送。")

    def _cancel_restore(self) -> None:
        if self.restore_thread is not None and self.restore_thread.is_alive():
            self.restore_stop_event.set()
            self.restore_status_var.set("恢复：正在停止…")

    def _restore_legs_to_150(self) -> None:
        host = self.host_var.get().strip()
        if self.control_mode_var.get() != CONTROL_MODE_IMU:
            self.status_var.set("一键恢复只在IMU模式可用。")
            return
        if self.imu_armed.get() or (self.restore_thread is not None and self.restore_thread.is_alive()):
            self.status_var.set("当前已有IMU/恢复会话，请先STOP。")
            return
        if not host or not self._check_mega_health(require_armed=False):
            self.status_var.set("恢复已阻止：P4/Mega/四路VL53健康门槛未满足。")
            return
        if not messagebox.askyesno(
                "确认一键恢复",
                "将四路VL53绝对位置连续恢复到150±1 mm。\n"
                "系统会自动分段ARM/STOP；确认机构周围无夹点风险？"):
            return
        self.restore_stop_event = threading.Event()
        self.restore_thread = RestoreLegsWorker(
            host, self.imu_command_worker, self.events, self.restore_stop_event
        )
        self.restore_button.state(["disabled"])
        self.arm_button.state(["disabled"])
        self.restore_status_var.set("恢复：准备分段控制…")
        self.restore_thread.start()

    def _queue_imu_stop(self, *, estop: bool = False) -> None:
        self._cancel_restore()
        host = self.host_var.get().strip()
        self.imu_armed.set(False)
        self.cv_armed = False
        self._arm_owner = None
        self._mega_arm_confirmed = False
        self._arm_request_pending = False
        self._arm_request_time = None
        payload = {"imu_estop": True} if estop else {"imu_stop": True}
        self.imu_command_worker.submit(host, payload, urgent=True)

    def _stop_imu(self) -> None:
        self._queue_imu_stop()
        self.status_var.set("STOP已优先排队；本地ARM已关闭。")

    def _estop_imu(self) -> None:
        if messagebox.askyesno("确认 ESTOP", "ESTOP将锁存，必须物理复位Mega才能解除。"):
            self._queue_imu_stop(estop=True)
            self.status_var.set("ESTOP已优先排队；本地ARM已关闭。")

    def _try_send_stop(self) -> None:
        """Non-blocking best-effort STOP; Mega heartbeat timeout is the final backstop."""
        self._queue_imu_stop()

    def _next_imu_seq(self) -> int:
        self._imu_command_seq += 1
        self._highest_sent_seq = self._imu_command_seq
        return self._imu_command_seq

    def _queue_heartbeat_if_due(self, now: float) -> None:
        """Heartbeat is owned by ImuCommandWorker, independent of Tk timing."""
        self._last_heartbeat_send = now

    def _queue_imu_delta(self, delta: dict[str, float]) -> bool:
        return self.imu_command_worker.set_latest_delta(
            self.host_var.get().strip(), delta
        )

    def _apply_leg_plan_preview(self, plan: dict) -> None:
        """Render geometry output only; never construct or send a command."""
        key_map = {
            "fl": "front_left", "fr": "front_right",
            "rl": "rear_left", "rr": "rear_right",
        }
        lengths = plan.get("lengths_mm", {})
        plan_status = plan.get("status")
        safe = plan_status == "safe" and len(lengths) == 4
        partial = plan_status == "partial_preview" and bool(lengths)
        unobservable = plan_status == "unobservable"
        for short_name, solver_name in key_map.items():
            length = lengths.get(solver_name)
            leg = plan.get("legs", {}).get(solver_name, {})
            preview_status = leg.get("preview_status", "")
            if length is None:
                self.target_vars[short_name].set("—")
                self.sensor_vars[short_name].set("—")
                if preview_status == "terrain_not_observed":
                    self.leg_status_vars[short_name].set("未观测到地形")
                elif preview_status == "not_visible_or_no_solution":
                    self.leg_status_vars[short_name].set("已观测/无安全轴向解")
                else:
                    self.leg_status_vars[short_name].set("—")
                continue
            rounded = int(round(float(length)))
            self.target_vars[short_name].set(str(rounded))
            # The existing Mega/VL53 contract is validated only through 390 mm.
            if rounded <= SOLVER_MAX_MM:
                self.sensor_vars[short_name].set(str(rounded - SENSOR_OFFSET_MM))
                self.leg_status_vars[short_name].set("预览")
            else:
                self.sensor_vars[short_name].set("未标定")
                self.leg_status_vars[short_name].set("390–400禁发")
        if unobservable:
            self.status_var.set(
                "腿长不可计算：四个落脚区均未被当前深度图覆盖，无高差数据可解算。"
            )
        elif partial:
            self.status_var.set(
                f"腿长部分预览：当前只覆盖 {len(lengths)}/4 个脚区；视野外不外推，缺失腿保持未观测；CV候选无效。"
            )
        elif not safe:
            self.status_var.set(
                "腿长解算未找到可见轴向安全解：" + ", ".join(plan.get("unsafe_reasons", [plan.get("status", "unknown")]))
            )
        else:
            values = [float(value) for value in lengths.values()]
            self.status_var.set(
                f"腿长规划预览：平台200×200 mm，四角腿250–400 mm；跨度 "
                f"{min(values):.1f}–{max(values):.1f} mm；仅生成去公共量CV候选，需验证门和显式ARM。"
            )

    def _on_control_mode_changed(self) -> None:
        """Switch planning/control owner; active owner is STOPped before handoff."""
        mode = self.control_mode_var.get()
        try:
            self.status_var.set(control_mode_description(mode))
        except ValueError:
            self.status_var.set("未知控制模式；保持安全锁定。")
        self._refresh_send_state()
        if mode != CONTROL_MODE_IMU:
            self._cancel_restore()
        active_mode = CONTROL_MODE_CV if self._arm_owner == "remote" else self._arm_owner
        if self._arm_owner is not None and active_mode != mode:
            self._queue_imu_stop()
        if mode == CONTROL_MODE_IMU:
            self.arm_button.configure(text="ARM IMU")
            self.arm_button.state(["!disabled"])
            self.restore_button.state(["!disabled"])
            self.remote_apply_button.state(["disabled"])
            self.target_panel_title.configure(text="四腿实时位置（IMU）")
            for entry in self.target_entries.values():
                entry.configure(state="readonly")
            self.panel_title_vars[1].set("当前姿态")
            self.panel_title_vars[2].set("四腿长度")
            self.capture_controls.grid_remove()
            # Enter IMU control mode: pause depth, show IMU dashboard
            if not hasattr(self, "_imu_leg_ctrl"):
                self._imu_leg_ctrl = ImuLegController()
            if not hasattr(self, "_imu_delta_gen"):
                self._imu_delta_gen = ImuDeltaCommandGenerator()
            self._imu_leg_ctrl.reset()
            self._imu_delta_gen.reset()
            self.imu_armed.set(False)
            self._stop_video_stream()
            if self.depth_thread is not None:
                self.depth_stop_event.set()
                self.depth_thread = None
            self.capture_button.state(["disabled"])
            self.capture_var.set("IMU模式")
            self.fps_var.set("IMU模式：视频流已停止")
            self.ground_var.set("IMU控制模式：等待姿态数据…")
            for index in (1, 2):
                self.cv_photos[index] = None
                self._show_panel_message(self.cv_labels[index], "IMU控制模式：深度推理已暂停")
            # Keep the user's Remote absolute targets across mode switches.
            for name in ("fl", "fr", "rl", "rr"):
                self.sensor_vars[name].set("—")
                self.leg_status_vars[name].set("IMU实时状态见上方")
        elif mode == CONTROL_MODE_CV:
            self.arm_button.configure(text="ARM Remote")
            self.arm_button.state(["!disabled"])
            self.restore_button.state(["disabled"])
            self.remote_apply_button.state(["!disabled"])
            self.target_panel_title.configure(text="Remote四腿绝对位置（VL53 mm）")
            self.target_note.configure(
                text="第一阶段Remote：ARM后启用原PS2手动/16预设；此处也可输入0–400 mm绝对目标。自动CV仅预览。"
            )
            for entry in self.target_entries.values():
                entry.configure(state="normal")
            self.cv_candidate_delta = None
            self.cv_candidate_time = None
            self.panel_title_vars[1].set(CV_VIEW_NAMES[1])
            self.panel_title_vars[2].set(CV_VIEW_NAMES[2])
            self.capture_controls.grid()
            # Return to CV mode: start a fresh depth worker if connected.
            if self.imu_stream_thread is not None and self.imu_stream_thread.is_alive():
                self._start_video_stream()
                if self.depth_thread is None or not self.depth_thread.is_alive():
                    self.depth_stop_event = threading.Event()
                    self.depth_thread = DepthEstimator(
                        self.depth_stop_event, self._video_generation, self.events
                    )
                    self.depth_thread.start()
                self.capture_button.state(["!disabled"])
                self.capture_var.set("正在加载深度模型")
                self.ground_var.set("实时云图：等待双深度模型就绪")
            else:
                self.capture_button.state(["disabled"])
                self.capture_var.set("CV模式（未连接）")
                self.ground_var.set("实时云图：请先连接P4")
            for index in (1, 2):
                self.cv_photos[index] = None
                self._show_panel_message(self.cv_labels[index], "恢复CV模式，等待深度数据…")
        else:
            # Locked mode
            self.arm_button.configure(text="ARM")
            self.arm_button.state(["disabled"])
            self.restore_button.state(["disabled"])
            self.remote_apply_button.state(["disabled"])
            self.target_panel_title.configure(text="四腿目标（锁定）")
            for entry in self.target_entries.values():
                entry.configure(state="readonly")
            self.panel_title_vars[1].set(CV_VIEW_NAMES[1])
            self.panel_title_vars[2].set(CV_VIEW_NAMES[2])
            self.capture_controls.grid()
            self.capture_button.state(["disabled"])
            self.capture_var.set("锁定模式")

    def _set_all(self, value: int) -> None:
        for var in self.target_vars.values():
            var.set(str(value))
        self._update_target_preview()

    def _update_target_preview(self) -> None:
        try:
            validate_remote_absolute_targets(
                {name: var.get() for name, var in self.target_vars.items()}
            )
        except ValueError as exc:
            self.status_var.set(f"Remote目标输入无效：{exc}")
            self._refresh_send_state(valid_targets=False)
            return
        self._refresh_send_state(valid_targets=True)

    def _healthy_for_command(self) -> bool:
        # SR04 is an optional depth-scale anchor. This is display-only; the ARM
        # methods enforce the exact LOCKED/owner state separately.
        if self.control_mode_var.get() == CONTROL_MODE_CV:
            return (self._check_remote_health(require_owner=False) or
                    self._check_remote_health(require_owner=True))
        return (self._check_mega_health(require_armed=False) or
                self._check_mega_health(require_armed=True))

    def _check_remote_health(self, *, require_owner: bool) -> bool:
        mega = self.latest_health.get("mega_imu", {})
        lander = self.latest_health.get("mega_lander", {})
        network = self.latest_health.get("network", {})
        if not network.get("connected") or not mega.get("connected"):
            return False
        if not mega.get("control_enabled") or mega.get("estop") or int(mega.get("fault_count", 0)) != 0:
            return False
        modes = ("REMOTE_MANUAL", "REMOTE_PRESET") if require_owner else ("LOCKED", "DISARMED")
        if mega.get("mode") not in modes:
            return False
        return all(
            lander.get(sensor, {}).get("valid") and
            0 <= int(lander.get(sensor, {}).get("mm", -1)) <= 400
            for sensor in LEG_TO_MEGA_SENSOR.values()
        )

    def _check_mega_health(self, *, require_armed: bool = True) -> bool:
        """Require fresh P4/Mega status, compile gate, all sensors and no fault."""
        mega = self.latest_health.get("mega_imu", {})
        lander = self.latest_health.get("mega_lander", {})
        network = self.latest_health.get("network", {})
        if not network.get("connected") or not mega.get("connected"):
            return False
        if not mega.get("control_enabled") or mega.get("estop") or int(mega.get("fault_count", 0)) != 0:
            return False
        if require_armed and mega.get("mode") not in ("ARMED", "ACTIVE"):
            return False
        if not require_armed and mega.get("mode") not in ("LOCKED", "DISARMED"):
            return False
        return all(lander.get(name, {}).get("valid", False) for name in ("a1", "a2", "a3", "a4"))

    def _refresh_send_state(self, valid_targets: bool | None = None) -> None:
        if valid_targets is None:
            try:
                build_target_set({name: var.get() for name, var in self.target_vars.items()})
                valid_targets = True
            except ValueError:
                valid_targets = False
        # Targets remain local unless one mode owns an explicitly armed session.
        # CV additionally requires a fresh stable capture and validation gate.

    def scan_p4(self) -> None:
        """Scan 192.168.137.0/24 off Tk and connect to a verified P4."""
        if self.scan_thread is not None and self.scan_thread.is_alive():
            self.status_var.set("P4扫描正在进行中…")
            return
        self.scan_button.state(["disabled"])
        self.connection_var.set(f"正在扫描 {P4_SCAN_PREFIX}1–254…")
        self.status_var.set("正在并发扫描Windows热点子网，并验证/board设备身份。")

        def worker() -> None:
            started = time.monotonic()
            try:
                matches = scan_p4_subnet()
                self.events.put(("scan_result", (matches, time.monotonic() - started)))
            except Exception as exc:
                self.events.put(("scan_error", str(exc)))

        self.scan_thread = threading.Thread(target=worker, daemon=True)
        self.scan_thread.start()

    def connect(self) -> None:
        host = self.host_var.get().strip()
        self._cancel_restore()
        if self.imu_armed.get() or (self.restore_thread is not None and self.restore_thread.is_alive()):
            self._queue_imu_stop()
        if not host:
            self.status_var.set("请输入 P4 IP 地址，例如 192.168.137.58。")
            return
        self.stop_event.set()
        self.depth_stop_event.set()
        self.video_stop_event.set()
        self._close_video_socket()
        self.stop_event = threading.Event()
        self.depth_stop_event = threading.Event()
        self.video_stop_event = threading.Event()
        self._video_generation += 1
        generation = self._video_generation
        self.connection_var.set(f"连接中：{host}")
        self.status_var.set("正在连接 P4 视频与遥测服务…")
        self.poll_thread = threading.Thread(target=self._poll_worker, args=(host, self.stop_event), daemon=True)
        self.video_thread = None
        self.imu_stream_thread = ImuStreamReader(
            host, self.stop_event, generation, self.events, self._accept_latest_imu_sample
        )
        if self.control_mode_var.get() == CONTROL_MODE_IMU:
            self.depth_thread = None
            self.capture_var.set("IMU模式")
            self.capture_button.state(["disabled"])
        else:
            self.depth_thread = DepthEstimator(self.depth_stop_event, generation, self.events)
            self.capture_var.set("等待模型就绪")
            self.capture_button.state(["!disabled"])
            self.depth_thread.start()
        self.poll_thread.start()
        if self.control_mode_var.get() != CONTROL_MODE_IMU:
            self._start_video_stream()
        self.imu_stream_thread.start()

    def _poll_worker(self, host: str, stop_event: threading.Event) -> None:
        while not stop_event.is_set():
            try:
                board = fetch_json(host, "/board")
                health = fetch_json(host, "/health")
                imu = fetch_json(host, "/imu")
                self.events.put(("telemetry", (host, board, health, imu)))
            except Exception as exc:  # Network state is displayed, never propagated into Tk thread.
                self.events.put(("network_error", str(exc)))
            stop_event.wait(0.5)

    def _close_video_socket(self) -> None:
        sock = self.video_sock
        self.video_sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _stop_video_stream(self) -> None:
        thread = self.video_thread
        self.video_stop_event.set()
        self._close_video_socket()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.2)
        self.video_thread = None
        self.camera_var.set("IMU模式：TCP 5000视频流已停止")
        self.fps_var.set("IMU模式：视频流已停止")
        self.cv_photos[0] = None
        self._show_panel_message(self.cv_labels[0], "IMU模式：视频流已停止")

    def _start_video_stream(self) -> None:
        if self.video_thread is not None and self.video_thread.is_alive():
            return
        host = self.host_var.get().strip()
        if not host or self.stop_event.is_set():
            return
        self.video_stop_event = threading.Event()
        self.video_thread = threading.Thread(
            target=self._video_worker,
            args=(host, self.stop_event, self.video_stop_event, self._video_generation),
            daemon=True,
        )
        self.video_thread.start()

    def _video_worker(self, host: str, stop_event: threading.Event,
                      video_stop_event: threading.Event, generation: int) -> None:
        frame_count = 0
        frame_times: deque[float] = deque()
        while not stop_event.is_set() and not video_stop_event.is_set():
            sock: socket.socket | None = None
            try:
                with socket.create_connection((host, VIDEO_PORT), timeout=4) as sock:
                    self.video_sock = sock
                    sock.settimeout(1.0)
                    self.events.put(("video_state", "视频已连接"))
                    while not stop_event.is_set() and not video_stop_event.is_set():
                        size = struct.unpack("<I", recv_exact(sock, 4))[0]
                        if not 1000 <= size <= 300000:
                            raise ValueError(f"异常 JPEG 长度 {size}")
                        jpeg = recv_exact(sock, size)
                        image = Image.open(io.BytesIO(jpeg)).convert("RGB")
                        # Keep native pixels here; resize exactly once to the live panel.
                        frame_count += 1
                        now = time.monotonic()
                        frame_times.append(now)
                        fps_5s = five_second_average_fps(frame_times, now, 5.0)
                        self.events.put(("frame", (generation, image.copy(), frame_count, fps_5s, size)))
            except Exception as exc:
                if not stop_event.is_set() and not video_stop_event.is_set():
                    self.events.put(("video_state", f"视频重连中：{exc}"))
                    video_stop_event.wait(1.5)
            finally:
                if self.video_sock is sock:
                    self.video_sock = None

    def _accept_latest_imu_sample(self, generation: int, record: dict[str, Any]) -> None:
        """Network-thread sink: overwrite pending IMU sample, never touch Tk here."""
        with self._latest_imu_lock:
            self._pending_imu_sample = (generation, record)
            self._imu_rx_count += 1
            self._last_imu_receive_monotonic = time.monotonic()

    def _take_latest_imu_sample(self) -> tuple[int, dict[str, Any]] | None:
        with self._latest_imu_lock:
            pending = self._pending_imu_sample
            self._pending_imu_sample = None
            return pending

    def _refresh_imu_display(self) -> None:
        """Independent 20 ms latest-sample display/PID path (up to 50 Hz)."""
        now = time.monotonic()
        if (self.control_mode_var.get() == CONTROL_MODE_IMU and self.imu_armed.get() and
                (self._last_imu_receive_monotonic is None or
                 now - self._last_imu_receive_monotonic > IMU_STALE_THRESHOLD_S)):
            self._queue_imu_stop()
            self.status_var.set("IMU数据超时：STOP已排队，本地ARM关闭。")
        pending_imu = self._take_latest_imu_sample()
        if pending_imu is not None:
            generation, record = pending_imu
            if generation == self._video_generation:
                self._imu_ui_count += 1
                now = time.monotonic()
                self._imu_ui_times.append(now)
                while self._imu_ui_times and self._imu_ui_times[0] < now - 5.0:
                    self._imu_ui_times.popleft()
                self._apply_imu_stream(record)
        self.after(20, self._refresh_imu_display)

    def _drain_events(self) -> None:
        try:
            # Bound work per callback so the independent 20 ms IMU refresh cannot starve.
            for _ in range(30):
                kind, payload = self.events.get_nowait()
                if kind == "scan_result":
                    matches, elapsed = payload
                    self.scan_thread = None
                    self.scan_button.state(["!disabled"])
                    if not matches:
                        self.connection_var.set("未发现ESP32-P4")
                        self.status_var.set(
                            f"扫描完成（{elapsed:.2f}s）：{P4_SCAN_PREFIX}1–254中未找到/board身份为ESP32-P4的设备。"
                        )
                    else:
                        selected = matches[0]
                        host = selected["host"]
                        self.host_var.set(host)
                        self.connection_var.set(f"已发现 {host}，正在连接…")
                        if len(matches) == 1:
                            self.status_var.set(
                                f"扫描完成（{elapsed:.2f}s）：发现ESP32-P4 {host}，正在自动连接。"
                            )
                        else:
                            hosts = ", ".join(item["host"] for item in matches)
                            self.status_var.set(
                                f"扫描发现{len(matches)}台P4：{hosts}；选择响应最快的{host}。"
                            )
                        self.connect()
                elif kind == "scan_error":
                    self.scan_thread = None
                    self.scan_button.state(["!disabled"])
                    self.connection_var.set("P4扫描失败")
                    self.status_var.set(f"自动扫描失败：{payload}")
                elif kind == "telemetry":
                    self._apply_telemetry(*payload)
                elif kind == "network_error":
                    self.connection_var.set("HTTP 未连接")
                    self.status_var.set(f"遥测读取失败：{payload}")
                    if self._arm_owner is not None:
                        self._try_send_stop()
                        self.imu_armed.set(False)
                        self.cv_armed = False
                    self._refresh_send_state()
                elif kind == "video_state":
                    if self.control_mode_var.get() != CONTROL_MODE_IMU:
                        self.camera_var.set(str(payload))
                    # First-stage Remote control is independent of video/CV.
                    # P4 HTTP/Mega loss still triggers STOP via network_error.
                elif kind == "imu_stream_state":
                    generation, status = payload
                    if generation == self._video_generation:
                        self.imu_stream_var.set(str(status))
                        if (self.control_mode_var.get() == CONTROL_MODE_IMU and
                                "重连中" in str(status) and self.imu_armed.get()):
                            self._try_send_stop()
                            self.imu_armed.set(False)
                elif kind == "imu_stream":
                    generation, record = payload
                    if generation == self._video_generation:
                        self._apply_imu_stream(record)
                elif kind == "depth_state":
                    generation, status = payload
                    if generation == self._video_generation:
                        self.depth_status_var.set(str(status))
                        if "已就绪" in str(status):
                            self.capture_var.set("就绪，等待开始采集")
                        if "不可用" in str(status):
                            # Automatic CV is preview-only; depth loss cannot own or stop Remote.
                            self.capture_button.state(["!disabled"])
                            self.capture_var.set("模型不可用")
                            self.ground_var.set("实时云图：深度模型未运行")
                            self._show_panel_message(self.cv_labels[1], str(status))
                            self._show_panel_message(self.cv_labels[2], str(status))
                elif kind == "live_depth":
                    generation, heatmap, elapsed_ms, detail = payload
                    if generation == self._video_generation:
                        heatmap = fit_panel_image(
                            heatmap, self.cv_labels[1].winfo_width() - 12,
                            self.cv_labels[1].winfo_height() - 12,
                        )
                        self.cv_photos[1] = ImageTk.PhotoImage(heatmap)
                        self.cv_labels[1].configure(image=self.cv_photos[1], text="")
                        summary = detail["summary"]
                        anchor = detail.get("ultrasonic_anchor")
                        anchor_text = (
                            f"US中位={anchor['ultrasonic_mm']:.0f}mm 原始={anchor['raw_latest_mm']:.0f}mm "
                            f"n={anchor['filter_samples']} span={anchor['filter_span_mm']:.0f}mm "
                            f"×{anchor['scale']:.3f} age={anchor['sample_age_us']/1000:.0f}ms"
                            if anchor else f"US未锚定：{detail.get('anchor_error', 'unknown')}"
                        )
                        observable = detail.get("source_observability", {})
                        source_text = "场景可观测" if observable.get("observable") else f"场景不可观测:{observable.get('reason','?')}"
                        if summary is None:
                            self.ground_var.set(
                                f"实时云图：{detail['near_m']:.2f}–{detail['far_m']:.2f} m | {source_text} | {anchor_text} | {elapsed_ms:.0f} ms"
                            )
                        else:
                            self.ground_var.set(
                                f"实时候选高度：{summary['height_m']:.3f} m | "
                                f"范围 {detail['near_m']:.2f}–{detail['far_m']:.2f} m | {source_text} | {anchor_text} | {elapsed_ms:.0f} ms"
                            )
                elif kind == "capture_source_wait":
                    generation, quality = payload
                    if generation == self._video_generation:
                        self.capture_var.set(f"等待可观测场景：{quality['reason']}")
                elif kind == "capture_state":
                    generation, count, total, complete = payload
                    if generation == self._video_generation:
                        self.capture_var.set(
                            f"采集中 {count}/{total}" if not complete else "正在选择最佳10帧并生成等高线…"
                        )
                elif kind == "capture_failed":
                    generation, message = payload
                    if generation == self._video_generation:
                        self.capture_button.state(["!disabled"])
                        self.capture_var.set("采集不稳定，可重试")
                        self.status_var.set(str(message))
                        self.cv_photos[2] = None
                        self._show_panel_message(self.cv_labels[2], str(message))
                elif kind == "capture_result":
                    generation, contour, elapsed_ms, detail = payload
                    if generation == self._video_generation:
                        contour = fit_panel_image(
                            contour, self.cv_labels[2].winfo_width() - 12,
                            self.cv_labels[2].winfo_height() - 12,
                            Image.Resampling.NEAREST,
                        )
                        self.cv_photos[2] = ImageTk.PhotoImage(contour)
                        self.cv_labels[2].configure(image=self.cv_photos[2], text="")
                        self.capture_button.state(["!disabled"])
                        self.capture_var.set("30/30 完成；已选最佳10帧")
                        # Automatic CV remains display-only in first-stage Remote mode.
                        # Never overwrite the four user-entered absolute VL53 targets.
                        try:
                            self.cv_candidate_delta = cv_plan_to_relative_deltas(detail["leg_plan"])
                            self.cv_candidate_time = time.monotonic()
                            if CV_CONTROL_VALIDATED:
                                self.status_var.set(
                                    f"稳定CV候选已就绪：差动目标 {self.cv_candidate_delta}；可显式ARM CV。"
                                )
                            else:
                                self.status_var.set(
                                    f"稳定CV候选={self.cv_candidate_delta}；当前仅预览，等待当前相机平面/台阶毫米误差验收。"
                                )
                        except ValueError:
                            self.cv_candidate_delta = None
                            self.cv_candidate_time = None
                        levels = detail["contour_levels_m"]
                        worst_selected = max(detail["selection_scores"])
                        calibration_text = (
                            "已验收OpenCV鱼眼内参" if detail.get("calibrated_intrinsics")
                            else f"暂定等距鱼眼 {detail.get('diagonal_fov_deg', 160):.0f}°对角"
                        )
                        anchor = detail["ultrasonic_anchor"]
                        if anchor["metric_source"] == "ultrasonic_anchored":
                            metric_text = (
                                f"US {anchor['ultrasonic_min_mm']:.0f}–{anchor['ultrasonic_max_mm']:.0f}mm "
                                f"scale {anchor['scale_min']:.3f}–{anchor['scale_max']:.3f}"
                            )
                        else:
                            metric_text = "模型尺度（未使用SR04）"
                        self.depth_status_var.set(
                            f"30帧→最佳10帧（最差入选误差 {worst_selected * 100:.2f}%） | "
                            f"{calibration_text} | {metric_text} | "
                            f"等高线 {len(levels)}级：{levels[0]:.2f}–{levels[-1]:.2f} m"
                        )
                elif kind == "imu_command_result":
                    payload, result, error = payload
                    if error or not result or not result.get("ok", False):
                        if payload.get("imu_arm") or payload.get("remote_arm"):
                            self.imu_armed.set(False)
                            self._mega_arm_confirmed = False
                            self._arm_request_pending = False
                            self._arm_request_time = None
                            self._arm_owner = None
                        # Session transport tolerates up to three consecutive
                        # heartbeat/delta errors; Mega's watchdog remains final.
                        self.status_var.set(f"IMU命令传输警告：{error or (result or {}).get('error', 'unknown')}")
                    elif payload.get("remote_arm"):
                        self.status_var.set("P4已转发ARM REMOTE；等待Mega启用原PS2逻辑。")
                    elif payload.get("imu_arm"):
                        # Transport accepted only; wait for Mega status ARMED/ACTIVE.
                        self.status_var.set("P4已转发ARM IMU；等待Mega确认。")
                    elif payload.get("imu_stop"):
                        self.status_var.set("Mega STOP已接受。")
                    elif payload.get("imu_estop"):
                        self.status_var.set("Mega ESTOP已接受；等待状态确认。")
                elif kind == "restore_progress":
                    segment, deltas, lander = payload
                    readings = {leg: lander.get(sensor, {}).get("mm", "—")
                                for leg, sensor in LEG_TO_MEGA_SENSOR.items()}
                    self.restore_status_var.set(
                        f"恢复第{segment}段：FL/FR/RL/RR={readings['fl']}/{readings['fr']}/"
                        f"{readings['rl']}/{readings['rr']} mm → Δ {deltas}"
                    )
                    self.status_var.set("正在连续恢复四腿至VL53=150±1 mm；STOP可随时取消。")
                elif kind == "restore_done":
                    self.restore_thread = None
                    self.restore_button.state(["!disabled"])
                    self.arm_button.state(["!disabled"])
                    self.restore_status_var.set("恢复完成：四腿均为150±1 mm")
                    self.status_var.set("一键恢复完成；Mega已DISARMED。")
                elif kind == "restore_cancelled":
                    self.restore_thread = None
                    self.restore_button.state(["!disabled"])
                    self.arm_button.state(["!disabled"])
                    self.restore_status_var.set("恢复：已取消并STOP")
                elif kind == "restore_error":
                    self.restore_thread = None
                    self.restore_button.state(["!disabled"])
                    self.arm_button.state(["!disabled"])
                    self.restore_status_var.set(f"恢复失败：{payload}")
                    self.status_var.set(f"一键恢复失败，已尝试STOP：{payload}")
                elif kind == "frame":
                    generation, image, frame_number, fps_5s, size = payload
                    if (generation == self._video_generation and
                            self.control_mode_var.get() != CONTROL_MODE_IMU):
                        original, _cloud_placeholder, _contour_placeholder = build_cv_triptych(image)
                        original = overlay_fps(original, fps_5s)
                        original = fit_panel_image(
                            original, self.cv_labels[0].winfo_width() - 12,
                            self.cv_labels[0].winfo_height() - 12,
                        )
                        self.cv_photos[0] = ImageTk.PhotoImage(original)
                        self.cv_labels[0].configure(image=self.cv_photos[0], text="")
                        if self.depth_thread is not None and self.control_mode_var.get() != CONTROL_MODE_IMU:
                            self.depth_thread.submit(image)
                        if self.control_mode_var.get() == CONTROL_MODE_IMU:
                            self.fps_var.set(
                                f"视频：过去5秒平均 {fps_5s:.2f} FPS  |  帧 {frame_number}  |  JPEG {size / 1024:.1f} KiB  |  "
                                "IMU控制模式：深度推理已暂停"
                            )
                        else:
                            self.fps_var.set(
                                f"视频：过去5秒平均 {fps_5s:.2f} FPS  |  帧 {frame_number}  |  JPEG {size / 1024:.1f} KiB  |  "
                                "左侧原图 / 中栏实时云图 / 右栏30帧选最佳10帧等高线"
                            )
        except queue.Empty:
            pass
        self.after(75, self._drain_events)

    def _apply_imu_stream(self, record: dict[str, Any]) -> None:
        """Render one independent BNO085 sample; in IMU mode also run PID + delta."""
        self.latest_imu_stream = record
        self.imu_stream_records.append(record)
        valid_packets = int(record.get("valid_packet_count", 0))
        timeout_count = int(record.get("uart_timeout_count", 0))
        age_ms = int(record.get("timestamp_us", 0))
        yaw = float(record.get("yaw", 0.0))
        pitch = float(record.get("pitch", 0.0))
        roll = float(record.get("roll", 0.0))
        sample_valid = bool(record.get("valid", True)) and all(math.isfinite(v) for v in (yaw, pitch, roll))
        if not sample_valid:
            if self.imu_armed.get():
                self._queue_imu_stop()
            self.status_var.set("IMU样本无效或非有限值：STOP已排队。")
            return
        if len(self._imu_ui_times) >= 2:
            imu_ui_hz = (len(self._imu_ui_times) - 1) / max(
                self._imu_ui_times[-1] - self._imu_ui_times[0], 1e-6
            )
        else:
            imu_ui_hz = 0.0
        self.imu_stream_var.set(
            f"TCP5001 RX={self._imu_rx_count} UI={self._imu_ui_count} ({imu_ui_hz:.1f}Hz) | "
            f"index={record.get('index', '?')} valid_packets={valid_packets} "
            f"uart_timeout={timeout_count} | "
            f"Y/P/R={yaw:+.2f}/{pitch:+.2f}/{roll:+.2f}° "
            f"ts={age_ms} us"
        )
        if self.control_mode_var.get() == CONTROL_MODE_IMU:
            now = time.monotonic()
            if not hasattr(self, "_imu_leg_ctrl"):
                self._imu_leg_ctrl = ImuLegController()
            if not hasattr(self, "_imu_delta_gen"):
                self._imu_delta_gen = ImuDeltaCommandGenerator()

            if self.imu_platform_transform not in IMU_ORTHOGONAL_TRANSFORMS:
                self.ground_var.set(
                    f"原始IMU: roll={roll:+.2f} pitch={pitch:+.2f} | 等待正交辨识"
                )
                return
            platform_roll, platform_pitch = transform_imu_to_platform(
                roll, pitch, self.imu_platform_transform,
                roll_zero_deg=self.imu_roll_zero_deg,
                pitch_zero_deg=self.imu_pitch_zero_deg,
            )
            self._imu_delta_gen.update_imu_time(now)
            result = self._imu_leg_ctrl.compute(platform_roll, platform_pitch, now=now)

            # Health gates and a heartbeat independent of target changes.
            mega_healthy = self._check_mega_health(require_armed=True)
            self._queue_heartbeat_if_due(now)

            delta = None
            if self.imu_armed.get() and self._mega_arm_confirmed and mega_healthy:
                delta = self._imu_delta_gen.compute_deltas(
                    result["adjustments_mm"], now, hold=result["within_target"]
                )
                if delta is not None:
                    try:
                        delta = clamp_deltas_to_live_leg_range(delta, self.latest_health)
                    except (ValueError, KeyError, TypeError):
                        delta = None
                        self.status_var.set("四腿绝对边界状态无效：本帧不发送目标。")

            # Update middle panel with live IMU readings
            delta_str = ""
            if delta is not None:
                delta_str = (
                    f"\n---\n"
                    f"delta FL={delta['fl']:+.1f} FR={delta['fr']:+.1f}\n"
                    f"      RL={delta['rl']:+.1f} RR={delta['rr']:+.1f}\n"
                    f"seq={self._imu_delta_gen.send_seq}"
                )
            imu_text = (
                f"Yaw:   {yaw:+.2f}\n"
                f"Pitch: {pitch:+.2f}\n"
                f"Roll:  {roll:+.2f}\n"
                f"---\n"
                f"mode: {'PID' if result['mode'] == 'pid' else 'proportional'}\n"
                f"{'stable (target +-1)' if result['within_target'] else 'correcting...'}"
                f"{delta_str}"
            )
            self.cv_labels[1].configure(
                image="", text=imu_text,
                justify="left", anchor="nw",
                wraplength=max(240, self.cv_labels[1].winfo_width() - 40),
                font=("Consolas", 16),
            )

            # Queue delta command; HTTP runs only in the worker thread.
            if delta is not None:
                queued = self._queue_imu_delta(delta)
                for name in ("fl", "fr", "rl", "rr"):
                    self.leg_status_vars[name].set(
                        f"Δ={delta[name]:+.0f}mm {'排队' if queued else '队列满'}"
                    )
            else:
                for name in ("fl", "fr", "rl", "rr"):
                    solver_name = {"fl": "front_left", "fr": "front_right",
                                   "rl": "rear_left", "rr": "rear_right"}[name]
                    adj = result["adjustments_mm"].get(solver_name, 0.0)
                    sign = "+" if adj >= 0 else ""
                    self.leg_status_vars[name].set(f"dL={sign}{adj:.1f}mm")

            # Status bar
            gate_status = ""
            if not self.imu_armed.get():
                gate_status = "未ARM"
            elif not mega_healthy:
                gate_status = "Mega未确认/不健康"
            else:
                gate_status = f"delta seq={self._imu_delta_gen.send_seq}"

            self.ground_var.set(
                f"IMU: raw R/P={roll:+.2f}/{pitch:+.2f} | "
                f"platform R/P={platform_roll:+.2f}/{platform_pitch:+.2f} | "
                f"{'PID' if result['mode'] == 'pid' else 'P'} | "
                f"{'stable' if result['within_target'] else 'correcting'} | "
                f"{gate_status}"
            )
            self.status_var.set(
                f"IMU: {'target reached' if result['within_target'] else 'converging'}; "
                f"ARM={'ON' if self.imu_armed.get() else 'OFF'}; "
                f"Mega={self.mega_imu_mode.get()}"
            )

    def _render_remote_leg_positions(self, health: dict[str, Any]) -> None:
        lander = health.get("mega_lander", {})
        all_valid = True
        for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
            sensor = lander.get(sensor_name, {})
            valid = bool(sensor.get("valid", False))
            mm = sensor.get("mm", "—")
            all_valid = all_valid and valid
            self.sensor_vars[leg].set(f"{mm} mm" if valid else "INVALID")
            self.leg_status_vars[leg].set(f"{sensor_name.upper()} / {'有效' if valid else '无效'}")
        if all_valid and not self._remote_targets_initialized:
            for leg, sensor_name in LEG_TO_MEGA_SENSOR.items():
                self.target_vars[leg].set(str(int(lander[sensor_name]["mm"])))
            self._remote_targets_initialized = True

    def _render_imu_leg_lengths(self, health: dict[str, Any]) -> None:
        lander = health.get("mega_lander", {})
        mega = health.get("mega_imu", {})
        rows = ["真实VL53绝对位置", ""]
        for leg, title in (("fl", "FL 前左"), ("fr", "FR 前右"),
                           ("rl", "RL 后左"), ("rr", "RR 后右")):
            sensor_name = LEG_TO_MEGA_SENSOR[leg]
            sensor = lander.get(sensor_name, {})
            valid = bool(sensor.get("valid", False))
            mm = sensor.get("mm", "—")
            target = int(mega.get(f"target_{leg}", 0))
            applied = int(mega.get(f"applied_{leg}", 0))
            rows.append(
                f"{title} / {sensor_name.upper()}\n"
                f"  长度 {mm} mm  {'有效' if valid else '无效'}\n"
                f"  会话目标 {target:+d} mm  反馈 {applied:+d} mm"
            )
        rows.extend(("", f"Mega: {mega.get('mode', 'UNKNOWN')}  fault={mega.get('fault_reason', 'none')}"))
        self.cv_labels[2].configure(
            image="", text="\n".join(rows), justify="left", anchor="nw",
            wraplength=max(240, self.cv_labels[2].winfo_width() - 40),
            font=("Consolas", 15), fg=CYAN,
        )

    def _apply_telemetry(self, host: str, board: dict[str, Any], health: dict[str, Any], imu: dict[str, Any]) -> None:
        self.latest_health = health
        network = health.get("network", {})
        camera = health.get("camera", {})
        ultra = health.get("ultrasonic", {})
        if self.depth_thread is not None and self.depth_thread.is_alive():
            self.depth_thread.set_ultrasonic(ultra)
        actuator = health.get("actuator", {})
        self.connection_var.set(f"P4 {board.get('board', '?')}  |  {host}  |  uptime {board.get('uptime', '?')} s")
        if self.control_mode_var.get() == CONTROL_MODE_IMU:
            self.camera_var.set("IMU模式：TCP 5000视频流已停止")
        else:
            self.camera_var.set(f"ready={camera.get('ready')} started={camera.get('started')} restarts={camera.get('restart_count')}")
        self.ultrasonic_var.set(
            f"{ultra.get('distance_mm', -1)} mm  valid={ultra.get('valid')}  "
            f"status={ultra.get('status', '?')}  age={ultra.get('sample_age_us', -1) / 1000:.0f} ms"
        )
        self.mega_var.set(f"USB connected={ultra.get('mega_connected')}  requests={ultra.get('request_count', 0)}")
        self.safety_var.set(
            f"P4 local PWM enabled={actuator.get('motion_enabled')} | "
            f"Mega command gate={'READY' if self._healthy_for_command() else 'BLOCKED'}"
        )
        # Update Mega IMU state display
        mega_imu = health.get("mega_imu", {})
        if mega_imu:
            mode = mega_imu.get("mode", "UNKNOWN")
            ack_seq = int(mega_imu.get("heartbeat_seq", 0))
            if ack_seq > self._last_mega_ack_seq:
                self._last_mega_ack_seq = ack_seq
                self._last_mega_ack_time = time.monotonic()
            self.mega_imu_mode.set(f"Mega: {mode}")
            self.mega_heartbeat_seq.set(ack_seq)
            self.mega_fault_count.set(int(mega_imu.get("fault_count", 0)))
            expected_modes = (
                ("REMOTE_MANUAL", "REMOTE_PRESET")
                if self._arm_owner == "remote" else ("ARMED", "ACTIVE")
            )
            owner_healthy = (
                self._check_remote_health(require_owner=True)
                if self._arm_owner == "remote" else self._check_mega_health(require_armed=True)
            )
            confirmed = (
                (self._arm_request_pending or self._mega_arm_confirmed) and
                mode in expected_modes and owner_healthy
            )
            self._mega_arm_confirmed = confirmed
            if confirmed:
                self.imu_armed.set(True)
                self._arm_request_pending = False
                self._arm_request_time = None
                if self._arm_owner == "remote" and not self.cv_armed:
                    self.status_var.set("Remote已ARM：原PS2 MANUAL/16预设已启用；右下栏也可提交绝对目标。")
            elif (self._arm_request_pending and self._arm_request_time is not None and
                  time.monotonic() - self._arm_request_time > 3.0):
                self.status_var.set("ARM确认超时：STOP已排队，未发送任何目标。")
                self._queue_imu_stop()
            elif mode in ("LOCKED", "DISARMED", "FAULT", "ESTOP", "UNKNOWN"):
                self.imu_armed.set(False)
                self.cv_armed = False
                if mode in ("FAULT", "ESTOP") or (mode in ("LOCKED", "DISARMED") and not self._arm_request_pending):
                    self._arm_owner = None
                if mode in ("FAULT", "ESTOP"):
                    self._arm_request_pending = False
        # In IMU mode, the IMU stream handler owns status_var/ground_var.
        if self.control_mode_var.get() == CONTROL_MODE_IMU:
            self._render_imu_leg_lengths(health)
            return
        if self.control_mode_var.get() == CONTROL_MODE_CV:
            self._render_remote_leg_positions(health)
        if self._arm_owner == "remote" and self._mega_arm_confirmed:
            mega_remote = health.get("mega_imu", {})
            if self.cv_armed and not mega_remote.get("remote_active", False):
                self.cv_armed = False
            if mega_remote.get("remote_active", False):
                source = mega_remote.get("remote_source", "?")
                preset = int(mega_remote.get("remote_preset", -1))
                suffix = f"，预设{preset + 1}" if source == "ps2" and preset >= 0 else ""
                self.status_var.set(f"Remote绝对目标运行中：source={source}{suffix}；PS2在Console目标期间暂停。")
            elif health.get("mega_imu", {}).get("mode") == "REMOTE_PRESET":
                self.status_var.set("Remote PRESET：按原111(1).ino映射选择16组预设；START返回MANUAL。")
            else:
                self.status_var.set("Remote MANUAL：原8个PS2手动按键已启用；START进入16组PRESET。")
        elif not network.get("connected", False):
            self.status_var.set("P4 已连接到控制台，但 Wi-Fi 尚未显示为已连接。")
        elif self._healthy_for_command():
            self.status_var.set("遥测正常。目标发送仍需要勾选机械安全确认并在确认框中再次确认。")
        else:
            self.status_var.set("监测正常；Mega/TCA/超声波健康条件未满足，目标发送已禁用。")
        self._refresh_send_state()

    def _close(self) -> None:
        self._try_send_stop()
        self.stop_event.set()
        self.depth_stop_event.set()
        self.video_stop_event.set()
        self._close_video_socket()
        # Give the urgent STOP a short bounded chance to leave before stopping worker.
        self.after(150, self._finish_close)

    def _finish_close(self) -> None:
        self.imu_command_worker.stop_event.set()
        self.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="Mars Lander Windows desktop control console")
    parser.add_argument("ip", nargs="?", default="", help="ESP32-P4 IP address, e.g. 192.168.137.58")
    args = parser.parse_args()
    app = LanderConsole(args.ip)
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
