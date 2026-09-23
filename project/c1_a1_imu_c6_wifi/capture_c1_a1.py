"""Capture C1-A1 diagnostic serial statistics from P4 COM5."""
from __future__ import annotations

import time
import serial

DURATION_S = 60


def main() -> int:
    stats: list[str] = []
    timeouts: list[str] = []
    boot: list[str] = []
    print(f"C1_A1_CAPTURE_SECONDS={DURATION_S}", flush=True)
    with serial.Serial("COM5", 115200, timeout=0.25) as port:
        deadline = time.monotonic() + DURATION_S
        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            if "C1_A1_STATS" in line:
                stats.append(line)
                print(line, flush=True)
            if "C1_A1_NO_UART" in line:
                timeouts.append(line)
                print(line, flush=True)
            if "C1-A1" in line or "C1_A1 Wi-Fi IP" in line:
                boot.append(line)
                print(line, flush=True)
    print(f"C1_A1_STATS_LINES={len(stats)}")
    print(f"C1_A1_UART_TIMEOUT_LINES={len(timeouts)}")
    print(f"C1_A1_LAST_STATS={stats[-1] if stats else 'NONE'}")
    print(f"C1_A1_RESULT={'PASS' if stats and not timeouts else 'FAIL'}")
    return 0 if stats and not timeouts else 2


if __name__ == "__main__":
    raise SystemExit(main())
