"""Discover the Mars Lander ESP32-P4 on a Windows Mobile Hotspot subnet.

The ESP receives DHCP, so its address may change after reboot/reconnect.
Discovery identifies the board by HTTP GET /board instead of trusting a stale IP.
"""
from __future__ import annotations

import concurrent.futures
import ipaddress
import json
import socket
import sys
import urllib.request

DEFAULT_SUBNET = "192.168.137.0/24"
HTTP_PORT = 80
VIDEO_PORT = 5000


def _probe_host(ip: str, timeout_s: float = 0.45) -> str | None:
    """Return IP only when /board confirms this is the ESP32-P4."""
    url = f"http://{ip}:{HTTP_PORT}/board"
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
        if payload.get("board") == "ESP32-P4":
            return ip
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return None


def discover_esp32(subnet: str = DEFAULT_SUBNET, timeout_s: float = 0.45, workers: int = 48) -> str:
    """Find the board on the local hotspot subnet or raise ConnectionError."""
    network = ipaddress.ip_network(subnet, strict=False)
    print(f"DISCOVERY_SCAN subnet={network} via HTTP /board", flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(_probe_host, str(ip), timeout_s) for ip in network.hosts()]
        for future in concurrent.futures.as_completed(futures):
            found = future.result()
            if found:
                for pending in futures:
                    pending.cancel()
                print(f"DISCOVERY_FOUND ip={found} video_tcp={VIDEO_PORT}", flush=True)
                return found
    raise ConnectionError(f"No ESP32-P4 responding to /board in {network}")


def resolve_esp_ip(argv: list[str] | None = None) -> str:
    """Use optional explicit IP; otherwise discover dynamically."""
    args = sys.argv[1:] if argv is None else argv
    return args[0] if args else discover_esp32()


if __name__ == "__main__":
    try:
        ip = discover_esp32(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SUBNET)
        print(f"ESP32_P4_IP={ip}")
    except ConnectionError as exc:
        print(f"DISCOVERY_FAILED: {exc}", file=sys.stderr)
        raise SystemExit(2)
