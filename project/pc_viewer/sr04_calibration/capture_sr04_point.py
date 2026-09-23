"""Capture one HC-SR04 calibration point from P4 frame-metadata TCP 5002.

The target distance must be measured manually from the FRONT ACOUSTIC FACE of
HC-SR04 to a rigid, flat target normal to the sensor beam. This script stores
every metadata sample (including invalid samples) and stops after N valid ones.
It never sends commands to P4, Mega, or any actuator.
"""
from __future__ import annotations

import argparse
import csv
import json
import socket
import struct
import threading
import time
from datetime import datetime
from pathlib import Path

META_PORT = 5002
VIDEO_PORT = 5000
DEFAULT_IP = "192.168.137.19"


def _recv_exact(sock: socket.socket, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        part = sock.recv(count - len(data))
        if not part:
            raise ConnectionError("P4 video socket closed")
        data.extend(part)
    return bytes(data)


def _video_trigger(host: str, stop: threading.Event, errors: list[Exception]) -> None:
    """Keep TCP 5000 flowing so P4 emits one 5002 record per JPEG."""
    try:
        with socket.create_connection((host, VIDEO_PORT), timeout=10) as sock:
            sock.settimeout(5)
            while not stop.is_set():
                size = struct.unpack("<I", _recv_exact(sock, 4))[0]
                if not 1000 <= size <= 200_000:
                    raise RuntimeError(f"invalid JPEG size {size}")
                _recv_exact(sock, size)  # Intentionally discard image bytes.
    except Exception as exc:
        if not stop.is_set():
            errors.append(exc)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("true_mm", type=float, help="Manually measured sensor-face-to-target distance in mm")
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--valid-samples", type=int, default=100)
    parser.add_argument("--max-seconds", type=float, default=45.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent / "captures")
    args = parser.parse_args()
    if not 20 <= args.true_mm <= 4000:
        raise ValueError("true_mm must be inside HC-SR04 20..4000 mm range")
    if args.valid_samples < 10:
        raise ValueError("--valid-samples must be at least 10")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    label = "".join(c if c.isalnum() or c in "-_" else "_" for c in args.label).strip("_")
    suffix = f"_{label}" if label else ""
    path = args.out_dir / f"sr04_true_{args.true_mm:g}mm_{stamp}{suffix}.csv"

    deadline = time.monotonic() + args.max_seconds
    rows: list[dict[str, object]] = []
    valid_count = 0
    start = time.monotonic()
    stop_video = threading.Event()
    video_errors: list[Exception] = []
    trigger = threading.Thread(target=_video_trigger, args=(args.ip, stop_video, video_errors), daemon=True)
    trigger.start()
    try:
        with socket.create_connection((args.ip, META_PORT), timeout=10) as sock:
            sock.settimeout(5)
            print(f"VIDEO_TRIGGER_AND_META_CONNECTED {args.ip}:5000/{META_PORT}")
            with sock.makefile("rb") as stream:
                while valid_count < args.valid_samples and time.monotonic() < deadline:
                    if video_errors:
                        raise RuntimeError(f"video trigger failed: {video_errors[0]}")
                    raw = stream.readline()
                    if not raw:
                        raise ConnectionError("P4 metadata socket closed")
                    record = json.loads(raw.decode("utf-8"))
                    if record.get("protocol") != "sr04-frame-meta-v1":
                        continue
                    sr04 = record["sr04"]
                    valid = bool(sr04["valid"]) and sr04["status"] == "ok" and bool(sr04["mega_connected"])
                    row = {
                        "capture_utc": datetime.utcnow().isoformat(timespec="milliseconds") + "Z",
                        "true_mm": args.true_mm,
                        "label": args.label,
                        "frame_id": int(record["frame_id"]),
                        "jpeg_timestamp_us": int(record["jpeg_timestamp_us"]),
                        "sr04_distance_mm": int(sr04["distance_mm"]),
                        "pulse_us": int(sr04["pulse_us"]),
                        "status": str(sr04["status"]),
                        "valid": int(valid),
                        "sample_age_us": int(sr04["sample_age_us"]),
                        "mega_connected": int(bool(sr04["mega_connected"])),
                    }
                    rows.append(row)
                    if valid:
                        valid_count += 1
                    if len(rows) <= 3 or len(rows) % 10 == 0 or valid_count == args.valid_samples:
                        print(f"SAMPLE total={len(rows)} valid={valid_count}/{args.valid_samples} "
                              f"frame={row['frame_id']} measured={row['sr04_distance_mm']}mm "
                              f"status={row['status']} age={row['sample_age_us']}us")
    finally:
        stop_video.set()
        trigger.join(timeout=2)

    fields = list(rows[0].keys()) if rows else []
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    elapsed = time.monotonic() - start
    print(f"CAPTURE_FILE={path}")
    print(f"TOTAL_SAMPLES={len(rows)} VALID_SAMPLES={valid_count} ELAPSED_S={elapsed:.1f}")
    if valid_count != args.valid_samples:
        raise SystemExit("CAPTURE_INCOMPLETE")
    print("CAPTURE_OK")


if __name__ == "__main__":
    main()
