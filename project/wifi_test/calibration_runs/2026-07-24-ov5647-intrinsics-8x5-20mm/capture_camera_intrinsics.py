"""Interactive OV5647 intrinsic-calibration image capture.

Board: 9 columns x 6 rows of squares, 20.0 mm per square.
OpenCV pattern: 8 x 5 INNER corners.

Uses unchanged P4 video TCP 5000. Captures are intentionally stored in a new
run directory and never touch the rejected historical calib_frames dataset.
Controls: s=save a qualifying view, a=toggle auto-save, q=quit.
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

IP_DEFAULT = "192.168.137.19"
VIDEO_PORT = 5000
PATTERN = (8, 5)
SQUARE_MM = 20.0
MIN_AREA = 0.12
MAX_AREA = 0.65
MIN_CENTER_DELTA = 70.0
AUTO_INTERVAL_S = 2.0


def recv_exact(sock: socket.socket, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("P4 video socket closed")
        data.extend(chunk)
    return bytes(data)


def receive_frame(sock: socket.socket) -> np.ndarray:
    size = struct.unpack("<I", recv_exact(sock, 4))[0]
    if not 1000 <= size <= 200_000:
        raise RuntimeError(f"invalid JPEG size {size}")
    image = cv2.imdecode(np.frombuffer(recv_exact(sock, size), np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError("JPEG decode failed")
    return image


def corner_metrics(corners: np.ndarray, image: np.ndarray) -> dict[str, float]:
    points = corners.reshape(-1, 2)
    x, y, bw, bh = cv2.boundingRect(points.astype(np.float32))
    h, w = image.shape
    return {
        "center_x": float(points[:, 0].mean()),
        "center_y": float(points[:, 1].mean()),
        "area_fraction": float((bw * bh) / (w * h)),
        "cover_w": float(bw / w),
        "cover_h": float(bh / h),
    }


def sufficiently_new(metrics: dict[str, float], accepted: list[dict[str, float]]) -> bool:
    return all(np.hypot(metrics["center_x"] - prior["center_x"], metrics["center_y"] - prior["center_y"]) >= MIN_CENTER_DELTA
               or abs(metrics["area_fraction"] - prior["area_fraction"]) >= 0.08
               for prior in accepted)


def connect_video(host: str) -> socket.socket:
    """Wait for P4's intentionally single-client TCP 5000 service to become free."""
    while True:
        try:
            sock = socket.create_connection((host, VIDEO_PORT), timeout=10)
            sock.settimeout(8)
            print(f"VIDEO_CONNECTED {host}:{VIDEO_PORT}")
            return sock
        except (TimeoutError, ConnectionError, OSError) as exc:
            print(f"VIDEO_CONNECT_WAIT: {exc}; retrying in 2 s")
            time.sleep(2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=IP_DEFAULT)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--target", type=int, default=25)
    parser.add_argument("--auto", action="store_true", help="Start automatic qualifying-view capture immediately")
    parser.add_argument("--append", action="store_true", help="Safely append to an existing capture run and preserve its manifest")
    args = parser.parse_args()
    if args.target < 20:
        raise ValueError("target must be at least 20 views")

    if args.run_dir.exists() and not args.append:
        raise FileExistsError(f"run directory already exists: {args.run_dir}; use --append to preserve and extend it")
    args.run_dir.mkdir(parents=True, exist_ok=True)
    metadata_path = args.run_dir / "capture_manifest.json"
    if args.append and metadata_path.exists():
        manifest = json.loads(metadata_path.read_text(encoding="utf-8"))
        accepted = [
            {key: float(entry[key]) for key in ("center_x", "center_y", "area_fraction", "cover_w", "cover_h")}
            for entry in manifest.get("accepted", [])
        ]
        print(f"APPEND_MODE existing_views={len(accepted)}")
    else:
        manifest = {
            "created_utc": datetime.utcnow().isoformat(timespec="seconds") + "Z",
            "board_squares": [9, 6],
            "inner_corners": list(PATTERN),
            "square_mm": SQUARE_MM,
            "source": f"tcp://{args.ip}:{VIDEO_PORT}",
            "accepted": [],
        }
        accepted = []
    auto = args.auto
    last_save = 0.0

    sock = connect_video(args.ip)
    try:
        cv2.namedWindow("OV5647 Camera Calibration", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("OV5647 Camera Calibration", 900, 900)
        print("CAPTURE_READY: s=save qualifying view, a=toggle auto, q=quit")
        while True:
            try:
                image = receive_frame(sock)
            except (TimeoutError, ConnectionError, OSError) as exc:
                print(f"VIDEO_INTERRUPTED: {exc}; reconnecting in 1 s")
                sock.close()
                time.sleep(1)
                sock = connect_video(args.ip)
                continue
            found, corners = cv2.findChessboardCorners(image, PATTERN, None)
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            metrics = None
            qualifying = False
            if found:
                corners = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001))
                metrics = corner_metrics(corners, image)
                qualifying = MIN_AREA <= metrics["area_fraction"] <= MAX_AREA and sufficiently_new(metrics, accepted)
                cv2.drawChessboardCorners(display, PATTERN, corners, True)
                color = (0, 220, 0) if qualifying else (0, 180, 255)
                text = (f"DETECTED area={metrics['area_fraction']:.1%} center="
                        f"({metrics['center_x']:.0f},{metrics['center_y']:.0f}) "
                        f"{'QUALIFIES' if qualifying else 'MOVE/RESIZE'}")
                cv2.putText(display, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.53, color, 2)
            else:
                cv2.putText(display, "NO 8x5 INNER-CORNER CHESSBOARD", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            cv2.putText(display, f"saved={len(accepted)}/{args.target}  auto={'ON' if auto else 'OFF'}  [s] save [a] auto [q] quit",
                        (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (255, 230, 0), 2)
            cv2.imshow("OV5647 Camera Calibration", display)
            key = cv2.waitKey(1) & 0xFF
            now = time.monotonic()
            save = key == ord("s") or (auto and qualifying and now - last_save >= AUTO_INTERVAL_S)
            if save:
                if not found or not qualifying or metrics is None:
                    print("REJECTED: require detected board, 12%..65% coverage, and a new position/scale")
                else:
                    index = len(accepted) + 1
                    filename = f"view_{index:03d}.png"
                    cv2.imwrite(str(args.run_dir / filename), image)
                    entry = {"file": filename, **metrics}
                    accepted.append(metrics)
                    manifest["accepted"].append(entry)
                    metadata_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
                    last_save = now
                    print(f"SAVED {filename} area={metrics['area_fraction']:.3f} center=({metrics['center_x']:.1f},{metrics['center_y']:.1f})")
                    if len(accepted) >= args.target:
                        print("CAPTURE_TARGET_REACHED")
                        break
            if key == ord("a"):
                auto = not auto
            if key == ord("q"):
                break
    finally:
        try:
            sock.close()
        except OSError:
            pass
        cv2.destroyAllWindows()
    print(f"RUN_DIR={args.run_dir}")
    print(f"SAVED_VIEWS={len(accepted)}")


if __name__ == "__main__":
    main()
