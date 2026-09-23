"""Collect fixed-scene HC-SR04 readings through the P4 framed bridge.

Read-only with respect to hardware: sends GET_DISTANCE only. Does not reset
Mega, write flash, drive actuators, or modify P4 firmware.
"""
import re
import statistics
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from p4_mega_bridge_client import Bridge, CMD_MEGA_TX, RSP_ERROR, RSP_MEGA_RX, RSP_STATUS

N = 30
PATTERN = re.compile(r"distance_mm=(-?\d+)\s+pulse_us=(\d+)\s+status=(\S+)")


def main():
    bridge = None
    readings = []
    reconnects = 0
    index = 1
    try:
        while index <= N:
            if bridge is None:
                bridge = Bridge("COM5")
                bridge.hello()
            try:
                bridge.send(CMD_MEGA_TX, b"GET_DISTANCE\n")
                deadline = time.monotonic() + 2.0
                response = None
                while time.monotonic() < deadline:
                    kind, payload = bridge.receive(max(0.05, deadline - time.monotonic()))
                    if kind == RSP_ERROR:
                        raise RuntimeError(f"P4 error: {payload.hex()}")
                    if kind == RSP_STATUS:
                        if not payload[0]:
                            raise RuntimeError("Mega disconnected")
                        continue
                    if kind == RSP_MEGA_RX:
                        response = payload.decode("ascii", "replace").strip()
                        break
                if response is None:
                    raise TimeoutError(f"sample {index}: no SR04 reply")
            except TimeoutError as exc:
                reconnects += 1
                print(f"BRIDGE_RECONNECT {reconnects}/5 after sample {index}: {exc}")
                bridge.close(); bridge = None
                if reconnects > 5:
                    raise
                time.sleep(0.25)
                continue
            match = PATTERN.search(response)
            if not match:
                raise RuntimeError(f"sample {index}: unexpected reply {response!r}")
            distance, pulse, status = int(match.group(1)), int(match.group(2)), match.group(3)
            readings.append((distance, pulse, status))
            print(f"SAMPLE {index:02d}/{N}: distance_mm={distance} pulse_us={pulse} status={status}")
            index += 1
            time.sleep(0.12)
    finally:
        if bridge is not None:
            bridge.close()

    valid = [distance for distance, _, status in readings if status == "ok" and distance >= 0]
    statuses = {}
    for _, _, status in readings:
        statuses[status] = statuses.get(status, 0) + 1
    print(f"SUMMARY total={len(readings)} valid={len(valid)} statuses={statuses}")
    if valid:
        print(f"DISTANCE_MM mean={statistics.mean(valid):.2f} median={statistics.median(valid):.2f} "
              f"min={min(valid)} max={max(valid)} stddev={statistics.pstdev(valid):.2f}")
    if len(valid) != N:
        raise SystemExit("SR04_REPEATABILITY_PARTIAL")
    print("SR04_REPEATABILITY_OK")


if __name__ == "__main__":
    main()
