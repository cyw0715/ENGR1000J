"""Capture Mega boot output after a bridge-controlled reset. Never sends SET_TARGET."""
from __future__ import annotations

import time
from p4_mega_bridge_client import Bridge, CMD_RESET_MEGA, RSP_ERROR, RSP_MEGA_RX, RSP_STATUS, show


def main() -> int:
    bridge = Bridge("COM5")
    try:
        for frame in bridge.hello():
            show(*frame)
        bridge.send(CMD_RESET_MEGA)
        print("MEGA_RESET_SENT_BOOT_CAPTURE_SECONDS=20")
        deadline = time.monotonic() + 20
        rx = bytearray()
        while time.monotonic() < deadline:
            try:
                kind, payload = bridge.receive(min(1.0, deadline - time.monotonic()))
            except TimeoutError:
                continue
            if kind == RSP_MEGA_RX:
                rx.extend(payload)
                print("MEGA_BOOT_RX", payload.decode("utf-8", "replace"), repr(payload))
            elif kind in (RSP_STATUS, RSP_ERROR):
                show(kind, payload)
        print("MEGA_BOOT_CAPTURE_BYTES", len(rx))
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
