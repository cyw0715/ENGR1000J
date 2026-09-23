"""Capture and summarize a bounded UART-RVC A/B run from COM5."""
from __future__ import annotations

import argparse
import re
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("--seconds", type=float, default=35.0)
    args = parser.parse_args()
    sample_lines: list[str] = []
    timeout_lines: list[str] = []
    indices: list[int] = []
    print(f"AB_{args.label}_CAPTURE_SECONDS={args.seconds}", flush=True)
    with serial.Serial("COM5", 115200, timeout=0.25) as port:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            if "YPR=[" in line:
                sample_lines.append(line)
                match = re.search(r"\[\s*(\d+)\] YPR=", line)
                if match:
                    indices.append(int(match.group(1)))
            if "no UART data" in line:
                timeout_lines.append(line)
    print(f"{args.label}_SAMPLE_LINES={len(sample_lines)}")
    print(f"{args.label}_TIMEOUT_LINES={len(timeout_lines)}")
    print(f"{args.label}_FIRST={sample_lines[0] if sample_lines else 'NONE'}")
    print(f"{args.label}_LAST={sample_lines[-1] if sample_lines else 'NONE'}")
    print(f"{args.label}_TIMEOUTS={' | '.join(timeout_lines) if timeout_lines else 'NONE'}")
    if sample_lines and not timeout_lines:
        print(f"AB_{args.label}_UART_CONTINUOUS_OK")
        return 0
    print(f"AB_{args.label}_UART_CONTINUOUS_FAIL")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
