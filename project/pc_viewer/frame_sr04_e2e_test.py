"""End-to-end validation of frame-synchronous SR04 metadata.

Video TCP 5000 remains the historical protocol:
    [uint32_le jpeg_len][jpeg]

P4 TCP 5002 is JSON Lines. Every successful 5000 JPEG send triggers exactly one
5002 record with the same frame_id and an SR04 snapshot taken in the same P4
send-success path. This client buffers both streams and only emits a record
when IDs match; it never approximates a frame/SR04 association using polling.
"""
from __future__ import annotations

import argparse
import json
import socket
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from esp_discovery import discover_esp32

VIDEO_PORT = 5000
META_PORT = 5002
MAX_JPEG = 200_000


def recv_exact(sock: socket.socket, count: int) -> bytes:
    data = bytearray()
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("socket closed")
        data.extend(chunk)
    return bytes(data)


@dataclass
class VideoFrame:
    received_at: float
    jpeg: bytes


class MetaReader(threading.Thread):
    def __init__(self, host: str, queue: deque, condition: threading.Condition, error: list):
        super().__init__(daemon=True)
        self.host, self.queue, self.condition, self.error = host, queue, condition, error
        self.sock: Optional[socket.socket] = None

    def run(self):
        try:
            self.sock = socket.create_connection((self.host, META_PORT), timeout=10)
            self.sock.settimeout(5)
            print(f"META_CONNECTED {self.host}:{META_PORT}", flush=True)
            file = self.sock.makefile("rb")
            for raw in file:
                record = json.loads(raw.decode("utf-8"))
                if record.get("protocol") != "sr04-frame-meta-v1":
                    raise RuntimeError(f"unexpected metadata protocol: {record.get('protocol')!r}")
                frame_id = int(record["frame_id"])
                with self.condition:
                    self.queue.append((frame_id, record))
                    self.condition.notify_all()
        except Exception as exc:  # returned to main test instead of silently dying
            self.error.append(exc)
            with self.condition:
                self.condition.notify_all()

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ip", nargs="?", help="explicit ESP IP; absent = DHCP discovery")
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    if args.frames < 1:
        raise ValueError("--frames must be >= 1")
    host = args.ip or discover_esp32()

    meta_queue: deque = deque()
    condition = threading.Condition()
    meta_error: list = []
    reader = MetaReader(host, meta_queue, condition, meta_error)
    reader.start()

    video = socket.create_connection((host, VIDEO_PORT), timeout=10)
    video.settimeout(8)
    print(f"VIDEO_CONNECTED {host}:{VIDEO_PORT}", flush=True)
    records = []
    unmatched_meta = 0
    first_frame_id = None
    try:
        for frame_index in range(args.frames):
            size = struct.unpack("<I", recv_exact(video, 4))[0]
            if not 1000 <= size <= MAX_JPEG:
                raise RuntimeError(f"invalid JPEG size {size}")
            jpeg = recv_exact(video, size)
            if jpeg[:2] != b"\xff\xd8" or jpeg[-2:] != b"\xff\xd9":
                raise RuntimeError("invalid JPEG SOI/EOI")
            if (frame_index + 1) % 10 == 0 and cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE) is None:
                raise RuntimeError("JPEG decode failed")

            deadline = time.monotonic() + 3
            matched = None
            with condition:
                while time.monotonic() < deadline:
                    if meta_error:
                        raise RuntimeError(f"metadata reader failed: {meta_error[0]}")
                    if first_frame_id is None and meta_queue:
                        # Both TCP streams begin at their current live positions. The
                        # first metadata record establishes the actual P4 frame_id;
                        # subsequent JPEGs must be strictly consecutive.
                        first_frame_id = meta_queue[0][0]
                    expected_id = first_frame_id + frame_index if first_frame_id is not None else None
                    if expected_id is not None:
                        while meta_queue and meta_queue[0][0] < expected_id:
                            meta_queue.popleft()
                            unmatched_meta += 1
                        if meta_queue and meta_queue[0][0] == expected_id:
                            _, matched = meta_queue.popleft()
                            break
                    condition.wait(max(0.01, deadline - time.monotonic()))
            if matched is None:
                raise TimeoutError(f"no matching 5002 metadata for video frame index={frame_index}")

            sr04 = matched["sr04"]
            frame_id = int(matched["frame_id"])
            if first_frame_id is None or frame_id != first_frame_id + frame_index:
                raise RuntimeError(f"non-contiguous metadata frame_id={frame_id}")
            record = {
                "frame_id": frame_id,
                "jpeg_bytes": size,
                "jpeg_timestamp_us": int(matched["jpeg_timestamp_us"]),
                "sr04_distance_mm": int(sr04["distance_mm"]),
                "sr04_pulse_us": int(sr04["pulse_us"]),
                "sr04_valid": bool(sr04["valid"]),
                "sr04_status": str(sr04["status"]),
                "sr04_age_us": int(sr04["sample_age_us"]),
                "mega_connected": bool(sr04["mega_connected"]),
            }
            records.append(record)
            print("PAIR"
                  f" frame_id={record['frame_id']} jpeg={record['jpeg_bytes']}B"
                  f" sr04_mm={record['sr04_distance_mm']}"
                  f" valid={record['sr04_valid']} status={record['sr04_status']}"
                  f" age_us={record['sr04_age_us']}", flush=True)
    finally:
        video.close()
        reader.close()

    valid = [r for r in records if r["sr04_valid"] and r["sr04_status"] == "ok" and r["mega_connected"]]
    ages = [r["sr04_age_us"] for r in records]
    ids = [r["frame_id"] for r in records]
    print("RESULT", flush=True)
    print(f"EXACT_FRAME_META_PAIRS={len(records)}/{args.frames}", flush=True)
    expected_ids = list(range(first_frame_id, first_frame_id + args.frames)) if first_frame_id is not None else []
    print(f"FRAME_IDS_CONTIGUOUS={ids == expected_ids}", flush=True)
    statuses = {}
    for record in records:
        status = record["sr04_status"]
        statuses[status] = statuses.get(status, 0) + 1
    print(f"SR04_VALID_OK={len(valid)}/{len(records)}", flush=True)
    print(f"SR04_STATUS_COUNTS={statuses}", flush=True)
    print(f"SR04_AGE_US=min:{min(ages)} mean:{sum(ages)/len(ages):.0f} max:{max(ages)}", flush=True)
    print(f"STALE_OR_UNMATCHED_META={unmatched_meta}", flush=True)
    # Transport succeeds when every JPEG has one exact metadata record. SR04
    # validity is intentionally reported separately: no echo must remain an
    # explicit invalid sample, not be silently replaced with a prior distance.
    if (len(records) != args.frames or ids != expected_ids or unmatched_meta != 0
            or any(age < 0 or age > 1_000_000 for age in ages)
            or any(not record["mega_connected"] for record in records)):
        raise SystemExit("FRAME_SR04_E2E_FAIL")
    print("FRAME_SR04_E2E_OK", flush=True)


if __name__ == "__main__":
    main()
