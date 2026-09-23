"""Read-only STK500v2 probe via the P4<->Mega framed bridge.

This tool deliberately never sends chip erase, load-address, or flash-program
commands. It only asks P4 to reset the Mega, then performs STK500v2 SIGN_ON
and reads the three ATmega2560 signature bytes.

Expected ATmega2560 signature: 1E 98 01.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from p4_mega_bridge_client import (  # noqa: E402
    Bridge,
    CMD_MEGA_TX,
    CMD_RESET_MEGA,
    CMD_SET_LINE,
    RSP_ERROR,
    RSP_MEGA_RX,
    RSP_STATUS,
)

MESSAGE_START = 0x1B
TOKEN = 0x0E
CMD_SIGN_ON = 0x01
CMD_READ_SIGNATURE_ISP = 0x1B
STATUS_CMD_OK = 0x00
EXPECTED_SIGNATURE = bytes((0x1E, 0x98, 0x01))


def stk_frame(sequence: int, payload: bytes) -> bytes:
    """Build standard STK500v2 frame: start/seq/BE length/token/payload/XOR."""
    if not payload or len(payload) > 275:
        raise ValueError("STK payload must be 1..275 bytes")
    body = bytes((MESSAGE_START, sequence & 0xFF)) + struct.pack(">H", len(payload)) + bytes((TOKEN,)) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return body + bytes((checksum,))


def parse_stk_frame(buffer: bytearray):
    """Return (sequence, payload) and consume one complete valid STK frame, else None."""
    while len(buffer) >= 1 and buffer[0] != MESSAGE_START:
        del buffer[0]
    if len(buffer) < 5:
        return None
    length = (buffer[2] << 8) | buffer[3]
    if buffer[4] != TOKEN or length > 275:
        del buffer[0]
        return None
    total = 5 + length + 1
    if len(buffer) < total:
        return None
    packet = bytes(buffer[:total])
    del buffer[:total]
    checksum = 0
    for byte in packet:
        checksum ^= byte
    if checksum != 0:
        return None
    return packet[1], packet[5:-1]


def status_or_error(bridge: Bridge, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        kind, payload = bridge.receive(max(0.05, deadline - time.monotonic()))
        if kind == RSP_ERROR:
            command = payload[0] if payload else 0
            error = struct.unpack("<i", payload[1:5])[0] if len(payload) == 5 else -1
            raise RuntimeError(f"P4 bridge error command=0x{command:02X} esp_err=0x{error & 0xFFFFFFFF:08X}")
        if kind == RSP_STATUS:
            if len(payload) >= 1 and not payload[0]:
                raise RuntimeError("P4 reports Mega is not connected")
            return
    raise TimeoutError("No P4 status response")


def send_bridge_command(bridge: Bridge, kind: int, payload: bytes = b""):
    bridge.send(kind, payload)
    status_or_error(bridge)


def exchange_stk(bridge: Bridge, request: bytes, wanted_sequence: int, timeout: float = 2.0) -> bytes:
    """Send one raw STK500v2 packet through P4 and collect fragmented MEGA_RX frames."""
    bridge.send(CMD_MEGA_TX, request)
    rx = bytearray()
    deadline = time.monotonic() + timeout
    got_tx_status = False
    while time.monotonic() < deadline:
        kind, payload = bridge.receive(max(0.05, deadline - time.monotonic()))
        if kind == RSP_ERROR:
            command = payload[0] if payload else 0
            error = struct.unpack("<i", payload[1:5])[0] if len(payload) == 5 else -1
            raise RuntimeError(f"P4 bridge error command=0x{command:02X} esp_err=0x{error & 0xFFFFFFFF:08X}")
        if kind == RSP_STATUS:
            got_tx_status = True
            if len(payload) >= 1 and not payload[0]:
                raise RuntimeError("Mega disconnected while sending STK frame")
            continue
        if kind != RSP_MEGA_RX:
            continue
        rx.extend(payload)
        parsed = parse_stk_frame(rx)
        if parsed is None:
            continue
        sequence, response = parsed
        if sequence != wanted_sequence:
            continue
        if not got_tx_status:
            # Response may race P4 status; it still proves the Mega bootloader answered.
            pass
        return response
    raise TimeoutError(f"No valid STK500v2 response for sequence {wanted_sequence}")


def expect_ok(response: bytes, command: int):
    if len(response) < 2 or response[0] != command or response[1] != STATUS_CMD_OK:
        raise RuntimeError(f"Unexpected STK response to 0x{command:02X}: {response.hex(' ')}")


def probe(port: str):
    bridge = Bridge(port)
    try:
        # Standard Mega 2560 bootloader line coding. P4 configures CH340; PC
        # never toggles COM5 DTR/RTS.
        send_bridge_command(bridge, CMD_SET_LINE, struct.pack("<IBBB", 115200, 0, 0, 8))
        send_bridge_command(bridge, CMD_RESET_MEGA)
        time.sleep(0.35)

        # Bootloader can be briefly unavailable after reset. Retry SIGN_ON only;
        # it is read-only and has no flash side effects.
        signon = None
        for sequence in range(1, 6):
            try:
                signon = exchange_stk(bridge, stk_frame(sequence, bytes((CMD_SIGN_ON,))), sequence, timeout=0.7)
                expect_ok(signon, CMD_SIGN_ON)
                break
            except (TimeoutError, RuntimeError):
                time.sleep(0.12)
        if signon is None:
            raise RuntimeError("STK500v2 SIGN_ON failed after reset; no flash command was sent")
        name_len = signon[2] if len(signon) >= 3 else 0
        name = signon[3:3 + name_len].decode("ascii", "replace")
        print(f"STK_SIGN_ON_OK name={name!r} response={signon.hex(' ')}")

        signature = bytearray()
        sequence = 16
        for index in range(3):
            # Mega bootloader uses byte 4 of the command payload as signature index.
            cmd = bytes((CMD_READ_SIGNATURE_ISP, 0, 0x30, 0x00, index, 0x00))
            response = exchange_stk(bridge, stk_frame(sequence, cmd), sequence, timeout=1.0)
            expect_ok(response, CMD_READ_SIGNATURE_ISP)
            if len(response) < 4 or response[3] != STATUS_CMD_OK:
                raise RuntimeError(f"Bad signature response index={index}: {response.hex(' ')}")
            signature.append(response[2])
            sequence += 1
        print("ATMEGA_SIGNATURE=" + signature.hex().upper())
        if bytes(signature) != EXPECTED_SIGNATURE:
            raise RuntimeError(f"Expected {EXPECTED_SIGNATURE.hex().upper()}, got {signature.hex().upper()}")
        print("READ_ONLY_BOOTLOADER_PROBE_OK")
    finally:
        bridge.close()


def selftest():
    request = stk_frame(1, bytes((CMD_SIGN_ON,)))
    assert request.hex() == "1b0100010e0114"
    parsed = parse_stk_frame(bytearray(request))
    assert parsed == (1, bytes((CMD_SIGN_ON,)))
    response = stk_frame(1, bytes((CMD_SIGN_ON, 0, 8)) + b"AVRISP_2")
    parsed = parse_stk_frame(bytearray(response))
    assert parsed[0] == 1 and parsed[1][0:3] == bytes((CMD_SIGN_ON, 0, 8))
    print("STK500V2_FRAME_SELFTEST_OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
    else:
        probe(args.port)


if __name__ == "__main__":
    main()
