"""
Mars Lander - Timestamped Video + IMU Receiver

Receives JPEG frames via TCP port 5000 (unchanged protocol: 4B LE size + JPEG)
and IMU data via TCP port 5001 (newline-delimited JSON, latest-sample stream).
After each complete JPEG frame, fetches /video_meta for ESP-side timestamp.

Usage: python timestamped_receiver.py [ESP_IP]
  - With ESP_IP: connect directly (no scanning)
  - Without ESP_IP: auto-scan 192.168.137.2-254 for ESP32-P4
Output: timestamp_capture/frames/*.jpg, timestamp_capture/video.jsonl, timestamp_capture/imu.jsonl
"""

import socket
import struct
import time
import json
import os
import sys
import threading
import signal
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed

TCP_PORT = 5000
IMU_TCP_PORT = 5001
HTTP_PORT = 80
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "timestamp_capture")

_running = True


def signal_handler(sig, frame):
    global _running
    _running = False
    print("\nStopping...")


signal.signal(signal.SIGINT, signal_handler)


def fetch_video_meta(esp_ip, frame_id=None):
    if frame_id is not None:
        url = f"http://{esp_ip}:{HTTP_PORT}/video_meta?frame_id={frame_id}"
    else:
        url = f"http://{esp_ip}:{HTTP_PORT}/video_meta"
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=1) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except Exception:
        return None


def fetch_video_meta_by_id(esp_ip, frame_id, max_retries=5, retry_delay=0.01):
    for _ in range(max_retries):
        meta = fetch_video_meta(esp_ip, frame_id=frame_id)
        if meta and meta.get("valid"):
            return meta
        time.sleep(retry_delay)
    return None


def tcp_video_receiver(esp_ip, frame_dir, video_log, lock):
    global _running

    baseline_meta = fetch_video_meta(esp_ip)
    baseline_frame_id = baseline_meta["frame_id"] if baseline_meta else None
    if baseline_frame_id is not None:
        print(f"[VIDEO] Baseline frame_id={baseline_frame_id}")
    else:
        print("[VIDEO] WARNING: could not fetch baseline frame_id")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    try:
        sock.connect((esp_ip, TCP_PORT))
    except Exception as e:
        print(f"[VIDEO] TCP connect failed: {e}")
        return
    print(f"[VIDEO] Connected to {esp_ip}:{TCP_PORT}")

    # Keep a continuous TCP buffer.  TCP has no message boundaries, and this
    # follows the existing local viewer_wifi.py re-synchronization strategy:
    # on an invalid length, discard ONE byte (never the whole 4-byte candidate).
    rx_buf = bytearray()
    frame_idx = 0
    resync_count = 0
    frame_id_mapping_trusted = baseline_frame_id is not None
    fps_window_start = time.monotonic()
    fps_frame_count = 0

    while _running:
        try:
            sock.settimeout(1)
            chunk = sock.recv(65536)
            if not chunk:
                print("[VIDEO] TCP connection closed")
                break
            rx_buf.extend(chunk)

            while len(rx_buf) >= 4 and _running:
                size = struct.unpack_from('<I', rx_buf, 0)[0]
                if size < 1000 or size > 500000:
                    # Critical: do NOT discard four bytes.  The candidate may
                    # start inside JPEG data; shift one byte and keep scanning.
                    del rx_buf[0]
                    resync_count += 1
                    frame_id_mapping_trusted = False
                    if resync_count <= 5 or resync_count % 10000 == 0:
                        print(f"[VIDEO] resync: invalid size {size} (dropped {resync_count} byte(s))")
                    continue

                # A real frame header is immediately followed by JPEG SOI.
                # Check this as soon as byte 5/6 are available; without it,
                # random JPEG entropy bytes can look like a plausible length
                # and make the receiver wait on the wrong boundary.
                if len(rx_buf) >= 6 and rx_buf[4:6] != b'\xff\xd8':
                    del rx_buf[0]
                    resync_count += 1
                    frame_id_mapping_trusted = False
                    if resync_count <= 5 or resync_count % 10000 == 0:
                        print("[VIDEO] resync: plausible size but missing JPEG SOI")
                    continue

                if len(rx_buf) < 4 + size:
                    break

                jpeg = bytes(rx_buf[4:4 + size])
                # Require the matching JPEG EOI as well before saving.
                if jpeg[-2:] != b'\xff\xd9':
                    del rx_buf[0]
                    resync_count += 1
                    frame_id_mapping_trusted = False
                    if resync_count <= 5 or resync_count % 10000 == 0:
                        print("[VIDEO] resync: candidate JPEG missing EOI")
                    continue

                del rx_buf[:4 + size]
                pc_time_ns = time.time_ns()
                frame_idx += 1

                # A byte-level resynchronization means an unknown number of
                # transport frames may have been skipped.  Do not fabricate an
                # ESP frame_id/timestamp for subsequent frames on this socket.
                expected_fid = (
                    baseline_frame_id + frame_idx
                    if frame_id_mapping_trusted else None
                )

                fname = f"frame_{frame_idx:06d}.jpg"
                fpath = os.path.join(frame_dir, fname)
                with open(fpath, 'wb') as f:
                    f.write(jpeg)

                meta = (
                    fetch_video_meta_by_id(esp_ip, expected_fid)
                    if expected_fid is not None else None
                )

                entry = {
                    "frame_idx": frame_idx,
                    "expected_esp_frame_id": expected_fid,
                    "esp_frame_id": meta["frame_id"] if meta else None,
                    "esp_timestamp_us": meta["timestamp_us"] if meta and meta.get("valid") else None,
                    "meta_valid": meta.get("valid", False) if meta else False,
                    "pc_receive_time_ns": pc_time_ns,
                    "jpeg_size": len(jpeg),
                    "file": fname,
                    "transport_resync_bytes": resync_count,
                }
                with lock:
                    video_log.write(json.dumps(entry) + "\n")
                    video_log.flush()

                # Periodic concise status for live verification and later
                # operator logs: PC frame number, exact ESP frame id/timestamp,
                # payload size, and whether byte-stream re-sync was needed.
                fps_frame_count += 1
                if frame_idx % 5 == 0:
                    now = time.monotonic()
                    elapsed = now - fps_window_start
                    fps = fps_frame_count / elapsed if elapsed > 0 else 0.0
                    fps_window_start = now
                    fps_frame_count = 0
                    print(
                        f"[VIDEO] frame={frame_idx} "
                        f"fps={fps:.1f} "
                        f"esp_ts_us={entry['esp_timestamp_us']} "
                        f"size={len(jpeg)}B resync={resync_count} "
                        f"meta_valid={entry['meta_valid']}"
                    )

        except socket.timeout:
            continue
        except Exception as e:
            if _running:
                print(f"[VIDEO] Error: {e}")
            break

    sock.close()
    print("[VIDEO] Disconnected")


def imu_tcp_receiver(esp_ip, imu_log, lock):
    global _running
    print(f"[IMU] Connecting to {esp_ip}:{IMU_TCP_PORT} (JSONL latest sample stream)")

    while _running:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((esp_ip, IMU_TCP_PORT))
            sock.settimeout(2)
            print(f"[IMU] Connected to {esp_ip}:{IMU_TCP_PORT}")

            rx_buf = bytearray()
            recv_count = 0
            window_count = 0
            window_start = time.monotonic()
            latest_data = None

            while _running:
                try:
                    chunk = sock.recv(65536)
                    if not chunk:
                        print("[IMU] TCP connection closed")
                        break
                    rx_buf.extend(chunk)

                    while True:
                        nl = rx_buf.find(b'\n')
                        if nl < 0:
                            break
                        line = rx_buf[:nl].decode('utf-8').strip()
                        del rx_buf[:nl + 1]

                        if not line:
                            continue

                        try:
                            data = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        pc_time_ns = time.time_ns()
                        recv_count += 1
                        window_count += 1
                        latest_data = data

                        entry = {
                            "pc_receive_time_ns": pc_time_ns,
                            "timestamp_us": data.get("timestamp_us", 0),
                            "index": data.get("index", 0),
                            "yaw": data.get("yaw", 0.0),
                            "pitch": data.get("pitch", 0.0),
                            "roll": data.get("roll", 0.0),
                            "raw_accel_mg": data.get("raw_accel_mg", {}),
                            "valid_packet_count": data.get("valid_packet_count", 0),
                            "checksum_error_count": data.get("checksum_error_count", 0),
                            "uart_timeout_count": data.get("uart_timeout_count", 0),
                        }
                        with lock:
                            imu_log.write(json.dumps(entry) + "\n")
                            imu_log.flush()

                    now = time.monotonic()
                    if now - window_start >= 1.0:
                        if window_count > 0 and latest_data:
                            print(
                                f"[IMU] imu_samples={recv_count} "
                                f"imu_rate_hz={window_count} "
                                f"timestamp_us={latest_data.get('timestamp_us', 0)} "
                                f"index={latest_data.get('index', 0)} "
                                f"valid_packet_count={latest_data.get('valid_packet_count', 0)} "
                                f"checksum_error_count={latest_data.get('checksum_error_count', 0)} "
                                f"uart_timeout_count={latest_data.get('uart_timeout_count', 0)}"
                            )
                        else:
                            print(f"[IMU] imu_samples={recv_count} imu_rate_hz=0 NO NEW IMU")
                        window_count = 0
                        window_start = now

                except socket.timeout:
                    now = time.monotonic()
                    if now - window_start >= 1.0:
                        if window_count > 0 and latest_data:
                            print(
                                f"[IMU] imu_samples={recv_count} "
                                f"imu_rate_hz={window_count} "
                                f"timestamp_us={latest_data.get('timestamp_us', 0)} "
                                f"index={latest_data.get('index', 0)} "
                                f"valid_packet_count={latest_data.get('valid_packet_count', 0)} "
                                f"checksum_error_count={latest_data.get('checksum_error_count', 0)} "
                                f"uart_timeout_count={latest_data.get('uart_timeout_count', 0)}"
                            )
                        else:
                            print(f"[IMU] imu_samples={recv_count} imu_rate_hz=0 NO NEW IMU")
                        window_count = 0
                        window_start = now
                    continue

            sock.close()
            if _running:
                print("[IMU] Reconnecting in 2s...")
                time.sleep(2)

        except Exception as e:
            if _running:
                print(f"[IMU] Connection error: {e}, retrying in 2s...")
                time.sleep(2)

    print(f"[IMU] Stopped (total received={recv_count})")


def _probe_esp(ip, timeout=0.4):
    """Probe a single IP for ESP32-P4 via /board or /imu."""
    for endpoint in ("/board", "/imu"):
        try:
            url = f"http://{ip}{endpoint}"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode("utf-8"))
                    if data.get("board") == "ESP32-P4":
                        return True
        except Exception:
            pass
    return False


def discover_esp(subnet="192.168.137", start=2, end=254, max_workers=32):
    """Scan subnet for ESP32-P4. Returns first found IP or None."""
    print(f"[DISCOVERY] Scanning {subnet}.{start}-{end} for ESP32-P4...")
    candidates = [f"{subnet}.{i}" for i in range(start, end + 1)]
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(_probe_esp, ip): ip for ip in candidates}
        for future in as_completed(futures):
            ip = futures[future]
            try:
                if future.result():
                    pool.shutdown(wait=False, cancel_futures=True)
                    print(f"[DISCOVERY] ESP found: {ip}")
                    return ip
            except Exception:
                pass
    return None


def main():
    global _running

    if len(sys.argv) >= 2:
        esp_ip = sys.argv[1]
    else:
        esp_ip = discover_esp()
        if esp_ip is None:
            print("[DISCOVERY] No ESP32-P4 found on 192.168.137.0/24.")
            print("            Ensure ESP is powered on and connected to hotspot.")
            print(f"            Or specify IP manually: python {sys.argv[0]} <ESP_IP>")
            sys.exit(1)
    frame_dir = os.path.join(OUT_DIR, "frames")
    os.makedirs(frame_dir, exist_ok=True)

    video_log_path = os.path.join(OUT_DIR, "video.jsonl")
    imu_log_path = os.path.join(OUT_DIR, "imu.jsonl")

    video_log = open(video_log_path, 'w')
    imu_log = open(imu_log_path, 'w')
    lock = threading.Lock()

    print(f"ESP IP:        {esp_ip}")
    print(f"Video TCP:     port {TCP_PORT} (4B LE len + JPEG, continuous stream)")
    print(f"Video meta:    GET /video_meta (frame_id, timestamp_us, jpeg_size, valid)")
    print(f"IMU  TCP:      port {IMU_TCP_PORT} (JSONL latest sample stream)")
    print(f"  IMU note:    latest-sample logging, not lossless 100Hz stream")
    print(f"Output:        {OUT_DIR}")
    print(f"Press Ctrl+C to stop\n")

    video_thread = threading.Thread(target=tcp_video_receiver,
                                    args=(esp_ip, frame_dir, video_log, lock),
                                    daemon=True)
    imu_thread = threading.Thread(target=imu_tcp_receiver,
                                  args=(esp_ip, imu_log, lock),
                                  daemon=True)

    video_thread.start()
    imu_thread.start()

    while _running:
        time.sleep(0.5)

    video_thread.join(timeout=5)
    imu_thread.join(timeout=5)

    video_log.close()
    imu_log.close()
    print(f"\nSaved: {video_log_path}, {imu_log_path}")


if __name__ == '__main__':
    main()
