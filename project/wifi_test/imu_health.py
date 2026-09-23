"""
IMU Health Monitor - polls ESP32 /imu endpoint every 0.5s
Displays: timestamp_us, index, sample_age_us, packet counts, errors

Usage: python imu_health.py <ESP_IP>
"""

import sys
import time
import json
import urllib.request

POLL_INTERVAL = 0.5


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <ESP_IP>")
        sys.exit(1)

    esp_ip = sys.argv[1]
    url = f"http://{esp_ip}/imu"

    print(f"IMU Health Monitor - polling {url} every {POLL_INTERVAL}s")
    print(f"{'timestamp_us':>16} {'idx':>4} {'age_ms':>8} "
          f"{'vpc':>8} {'crc_err':>8} {'uart_to':>8} "
          f"{'rx_ovf':>8} {'short':>8} {'bad_hdr':>8} {'sync':>8}")
    print("-" * 92)

    prev_vpc = -1
    while True:
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=2) as resp:
                data = json.loads(resp.read().decode("utf-8"))

            ts = data.get("timestamp_us", 0)
            idx = data.get("index", 0)
            age = data.get("sample_age_us", -1)
            vpc = data.get("valid_packet_count", 0)
            crc = data.get("checksum_error_count", 0)
            uart_to = data.get("uart_timeout_count", 0)
            rx_ovf = data.get("rx_overflow_count", 0)
            short_pkt = data.get("short_packet_count", 0)
            bad_hdr = data.get("bad_header_count", 0)
            sync = data.get("sync_fail_count", 0)

            age_ms = f"{age / 1000:.1f}" if age >= 0 else "N/A"

            marker = ""
            if prev_vpc >= 0 and vpc == prev_vpc:
                marker = "  <-- NO NEW PACKETS"
            elif prev_vpc >= 0:
                marker = f"  (+{vpc - prev_vpc})"

            print(f"{ts:>16} {idx:>4} {age_ms:>8} "
                  f"{vpc:>8} {crc:>8} {uart_to:>8} "
                  f"{rx_ovf:>8} {short_pkt:>8} {bad_hdr:>8} {sync:>8}{marker}")

            prev_vpc = vpc

        except Exception as e:
            print(f"ERROR: {e}")

        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    main()
