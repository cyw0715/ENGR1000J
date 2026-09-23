"""End-to-end video + independent BNO085 stream verifier.

Connections:
  TCP 5000: [uint32_le JPEG length][JPEG]
  TCP 5001: independent BNO085 JSON Lines samples

No frame/IMU association is attempted. This script sends no control command and
never contacts Arduino/Mega.
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from collections import deque
from typing import Any

VIDEO_PORT = 5000
IMU_PORT = 5001
MAX_JPEG = 300_000


def recv_exact(sock: socket.socket, count: int) -> bytes:
    result = bytearray()
    while len(result) < count:
        chunk = sock.recv(count - len(result))
        if not chunk:
            raise ConnectionError("video socket closed")
        result.extend(chunk)
    return bytes(result)


class ImuReader(threading.Thread):
    def __init__(self, host: str, records: deque[dict[str, Any]], ready: threading.Event,
                 error: list[Exception], stop: threading.Event) -> None:
        super().__init__(daemon=True)
        self.host, self.records, self.ready, self.error, self.stop = host, records, ready, error, stop
        self.sock: socket.socket | None = None

    def run(self) -> None:
        try:
            self.sock = socket.create_connection((self.host, IMU_PORT), timeout=10)
            self.sock.settimeout(2)
            self.ready.set()
            with self.sock.makefile("rb") as stream:
                while not self.stop.is_set():
                    raw = stream.readline()
                    if not raw:
                        raise ConnectionError("IMU stream closed")
                    record = json.loads(raw.decode("utf-8"))
                    if "timestamp_us" not in record or "valid_packet_count" not in record:
                        raise RuntimeError("invalid independent IMU JSONL record")
                    self.records.append(record)
        except Exception as exc:
            if not self.stop.is_set():
                self.error.append(exc)
            self.ready.set()

    def close(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", help="P4 Wi-Fi IP")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--min-imu-samples", type=int, default=20)
    args = parser.parse_args()
    if args.frames < 1 or args.min_imu_samples < 1:
        raise ValueError("--frames and --min-imu-samples must be >= 1")

    records: deque[dict[str, Any]] = deque(maxlen=4096)
    ready, stop = threading.Event(), threading.Event()
    errors: list[Exception] = []
    reader = ImuReader(args.ip, records, ready, errors, stop)
    reader.start()
    if not ready.wait(12):
        raise TimeoutError("TCP 5001 connection timeout")
    if errors:
        raise RuntimeError(f"IMU connection failed: {errors[0]}")
    print(f"IMU_CONNECTED {args.ip}:{IMU_PORT}", flush=True)

    video = socket.create_connection((args.ip, VIDEO_PORT), timeout=10)
    video.settimeout(8)
    print(f"VIDEO_CONNECTED {args.ip}:{VIDEO_PORT}", flush=True)
    try:
        for number in range(1, args.frames + 1):
            jpeg_size = struct.unpack("<I", recv_exact(video, 4))[0]
            if not 1000 <= jpeg_size <= MAX_JPEG:
                raise RuntimeError(f"invalid JPEG size: {jpeg_size}")
            jpeg = recv_exact(video, jpeg_size)
            if jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
                raise RuntimeError("invalid JPEG SOI/EOI")
            if number % 5 == 0 or number == 1:
                print(f"VIDEO n={number}/{args.frames} jpeg={jpeg_size}B imu_received={len(records)}", flush=True)
        deadline = time.monotonic() + 5
        while len(records) < args.min_imu_samples and time.monotonic() < deadline and not errors:
            time.sleep(0.01)
    finally:
        video.close()
        stop.set()
        reader.close()

    if errors:
        raise RuntimeError(f"IMU reader failed: {errors[0]}")
    samples = list(records)
    timestamps = [int(sample["timestamp_us"]) for sample in samples]
    increasing = all(b > a for a, b in zip(timestamps, timestamps[1:]))
    valid_packets = int(samples[-1].get("valid_packet_count", 0)) if samples else 0
    print("RESULT")
    print(f"VIDEO_FRAMES={args.frames}/{args.frames}")
    print(f"INDEPENDENT_IMU_SAMPLES={len(samples)}")
    print(f"IMU_TIMESTAMPS_STRICTLY_INCREASING={increasing}")
    print(f"IMU_VALID_PACKET_COUNT={valid_packets}")
    if len(samples) < args.min_imu_samples or not increasing or valid_packets < 1:
        raise SystemExit("P4_INDEPENDENT_VIDEO_IMU_TEST_FAIL")
    print("P4_INDEPENDENT_VIDEO_IMU_TEST_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
