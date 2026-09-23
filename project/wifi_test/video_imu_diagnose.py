"""Controlled video+IMU diagnostic: records /imu health once per second while
running timestamped_receiver.py against an explicit ESP IP. Read-only on ESP.
Usage: python video_imu_diagnose.py <ESP_IP> [seconds]
"""

import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

if len(sys.argv) < 2:
    raise SystemExit(f"Usage: python {Path(sys.argv[0]).name} <ESP_IP> [seconds]")

ip = sys.argv[1]
duration_s = int(sys.argv[2]) if len(sys.argv) > 2 else 12
root = Path(__file__).resolve().parent
stamp = time.strftime("%Y%m%d_%H%M%S")
health_log = root / f"video_imu_health_{stamp}.jsonl"
receiver_log = root / f"video_imu_health_{stamp}.receiver.log"

with receiver_log.open("w", encoding="utf-8") as output:
    receiver = subprocess.Popen(
        [sys.executable, "-u", "timestamped_receiver.py", ip],
        cwd=root,
        stdout=output,
        stderr=subprocess.STDOUT,
    )
    time.sleep(1.0)
    end = time.monotonic() + duration_s
    with health_log.open("w", encoding="utf-8") as records:
        while time.monotonic() < end:
            row = {"pc_monotonic_ns": time.monotonic_ns()}
            try:
                with urllib.request.urlopen(f"http://{ip}/imu", timeout=0.8) as response:
                    row["health"] = json.loads(response.read().decode("utf-8"))
            except Exception as exc:
                row["error"] = str(exc)
            records.write(json.dumps(row) + "\n")
            records.flush()
            time.sleep(1.0)
    receiver.terminate()
    try:
        receiver.wait(timeout=3)
    except subprocess.TimeoutExpired:
        receiver.kill()
        receiver.wait()

print(f"HEALTH_LOG={health_log}")
print(f"RECEIVER_LOG={receiver_log}")
