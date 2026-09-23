"""Legacy retired: frame-synchronous IMU verification was removed.

The production firmware intentionally publishes BNO085 data only on independent
TCP 5001 JSONL. This command exists solely to make an old invocation fail with
an explicit migration message instead of silently validating a nonexistent 5002
protocol.
"""
from __future__ import annotations

import argparse


def main() -> int:
    parser = argparse.ArgumentParser(description="retired P4 frame-IMU verifier")
    parser.add_argument("ip", nargs="?", help="unused; independent IMU has no frame association")
    parser.parse_args()
    raise SystemExit(
        "RETIRED: TCP 5002 frame-IMU metadata was removed. "
        "Use TCP 5000 for JPEG and TCP 5001 for independent BNO085 JSONL."
    )


if __name__ == "__main__":
    main()
