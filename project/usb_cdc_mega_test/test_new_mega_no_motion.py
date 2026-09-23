"""Read-only/no-motion Mega application handshake via the P4 bridge."""
from __future__ import annotations

import time
from p4_mega_bridge_client import Bridge, CMD_MEGA_TX, RSP_MEGA_RX, show


def request_line(bridge: Bridge, text: str, timeout_s: float = 15.0) -> bool:
    bridge.send(CMD_MEGA_TX, (text + "\n").encode("ascii"))
    print(f"SENT={text}")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            kind, payload = bridge.receive(min(1.0, deadline - time.monotonic()))
        except TimeoutError:
            continue
        show(kind, payload)
        if kind == RSP_MEGA_RX:
            return True
    return False


def main() -> int:
    bridge = Bridge("COM5")
    try:
        for frame in bridge.hello():
            show(*frame)
        # PING does not command any physical motion.
        if not request_line(bridge, "PING"):
            raise RuntimeError("No Mega application response to no-motion PING within 15 s")
        print("NO_MOTION_PING_OK")
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
