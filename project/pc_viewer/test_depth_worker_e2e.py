"""Bounded end-to-end smoke test for two-model display-only depth worker."""
from __future__ import annotations

import queue
import threading
import time

from pathlib import Path
from PIL import Image

from lander_console import DepthEstimator

SCENE = Image.open(
    Path(__file__).parent / "metric_depth_eval" / "Depth-Anything-V2" /
    "assets" / "examples" / "demo01.jpg"
).convert("RGB").resize((800, 800), Image.Resampling.LANCZOS)


def main() -> int:
    events: queue.Queue = queue.Queue()
    stop = threading.Event()
    worker = DepthEstimator(stop, generation=99, events=events)
    worker.start()
    deadline = time.monotonic() + 180.0
    capture_started = False
    pre_capture_submitted = False
    live_before_capture = False
    submitted = 0
    states: list[str] = []
    try:
        while time.monotonic() < deadline:
            try:
                kind, payload = events.get(timeout=2.0)
            except queue.Empty:
                continue
            if kind == "depth_state":
                states.append(str(payload[1]))
                print("STATE", payload[1], flush=True)
                if "已就绪" in str(payload[1]) and not pre_capture_submitted:
                    worker.submit(SCENE.copy())
                    pre_capture_submitted = True
            elif kind == "capture_state":
                _, count, total, complete = payload
                print("CAPTURE", count, "/", total, flush=True)
                if capture_started and not complete and submitted < total and count >= submitted:
                    worker.submit(SCENE.copy())
                    submitted += 1
            elif kind == "live_depth":
                _, heatmap, elapsed_ms, detail = payload
                if not capture_started:
                    live_before_capture = True
                    print("LIVE_BEFORE_CAPTURE_OK", flush=True)
                    worker.start_capture()
                    capture_started = True
                    submitted = 0
                print(
                    "LIVE_DEPTH_OK",
                    "heatmap=", heatmap.size,
                    "elapsed_ms=%.1f" % elapsed_ms,
                    "range=%.3f..%.3f" % (detail["near_m"], detail["far_m"]),
                    flush=True,
                )
            elif kind == "capture_result":
                assert live_before_capture, "middle live cloud did not run before capture"
                _, contour, elapsed_ms, detail = payload
                assert detail["ultrasonic_anchor"]["frames"] == 30
                assert detail["ultrasonic_anchor"]["anchored_frames"] == 0
                assert detail["ultrasonic_anchor"]["metric_source"] == "model_only"
                print(
                    "DEPTH_WORKER_E2E_OK",
                    "contour=", contour.size,
                    "elapsed_ms=%.1f" % elapsed_ms,
                    "captured_count=", detail["captured_count"],
                    "selected_count=", detail["selected_count"],
                    "leg_status=", detail["leg_plan"]["status"],
                    "leg_lengths=", {k: round(v, 1) for k, v in detail["leg_plan"]["lengths_mm"].items()},
                    "contour_levels=", len(detail["contour_levels_m"]),
                    flush=True,
                )
                return 0
        raise TimeoutError(f"no depth frame; states={states}")
    finally:
        stop.set()


if __name__ == "__main__":
    raise SystemExit(main())
