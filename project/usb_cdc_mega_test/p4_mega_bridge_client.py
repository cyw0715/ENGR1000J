"""Windows PC client for the P4-NANO <-> Mega framed bridge.

Never point Arduino IDE or avrdude directly at COM5: their DTR/RTS toggling can
reset the P4. This client sends framed commands to P4; P4 controls CH340/Mega.

Requires: pyserial in the Python environment used to run this tool.
Examples:
  python p4_mega_bridge_client.py --port COM5 status
  python p4_mega_bridge_client.py --port COM5 echo HELLO
  python p4_mega_bridge_client.py --port COM5 reset
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing pyserial. Install it into the Python environment that runs this client: python -m pip install pyserial") from exc

SOF = b"\xA5\x5A"
MAX_PAYLOAD = 512
CMD_HELLO, RSP_HELLO = 0x01, 0x02
CMD_MEGA_TX, RSP_MEGA_RX = 0x10, 0x11
CMD_SET_LINE, CMD_SET_CONTROL, CMD_RESET_MEGA, CMD_GET_STATUS, RSP_STATUS = 0x12, 0x13, 0x14, 0x15, 0x16
RSP_ERROR = 0x7F


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(kind: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 256 bytes")
    body = bytes((kind,)) + struct.pack("<H", len(payload)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt(body))


class Bridge:
    def __init__(self, port: str):
        # Configure control lines before opening: opening a CH343 UART with
        # default DTR asserted can reset the P4. This client never asks the
        # operating system to toggle P4 reset/control lines.
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.10
        self.ser.write_timeout = 1
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        time.sleep(0.15)
        self.ser.reset_input_buffer()  # discard P4 ROM boot text, if any

    def close(self):
        self.ser.close()

    def send(self, kind: int, payload: bytes = b""):
        self.ser.write(frame(kind, payload))
        self.ser.flush()

    def receive(self, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        state = 0
        header = bytearray()
        payload = bytearray()
        expected = 0
        crc = bytearray()
        while time.monotonic() < deadline:
            ch = self.ser.read(1)
            if not ch:
                continue
            value = ch[0]
            if state == 0:
                state = 1 if value == SOF[0] else 0
            elif state == 1:
                state = 2 if value == SOF[1] else (1 if value == SOF[0] else 0)
                if state == 2:
                    header.clear()
            elif state == 2:
                header.append(value)
                if len(header) == 3:
                    expected = header[1] | (header[2] << 8)
                    if expected > MAX_PAYLOAD:
                        state = 0
                    else:
                        payload.clear()
                        state = 3 if expected else 4
            elif state == 3:
                payload.append(value)
                if len(payload) == expected:
                    crc.clear()
                    state = 4
            else:
                crc.append(value)
                if len(crc) == 2:
                    body = bytes(header) + bytes(payload)
                    received = crc[0] | (crc[1] << 8)
                    state = 0
                    if crc16_ccitt(body) == received:
                        return header[0], bytes(payload)
        raise TimeoutError("no valid bridge frame before timeout")

    def hello(self):
        self.send(CMD_HELLO)
        responses = []
        until = time.monotonic() + 2
        while time.monotonic() < until:
            kind, payload = self.receive(until - time.monotonic())
            responses.append((kind, payload))
            if kind == RSP_STATUS:
                return responses
        return responses


def show(kind, payload):
    if kind == RSP_HELLO:
        print("HELLO_ACK", payload.decode("ascii", "replace"))
    elif kind == RSP_STATUS:
        if len(payload) == 5:
            connected = bool(payload[0])
            error = struct.unpack("<i", payload[1:])[0]
            print(f"STATUS mega_connected={connected} last_esp_error=0x{error & 0xFFFFFFFF:08X}")
        elif len(payload) == 15:
            connected, seen, address = payload[0], payload[1], payload[2]
            vid = payload[3] | (payload[4] << 8)
            pid = payload[5] | (payload[6] << 8)
            error = struct.unpack("<i", payload[7:11])[0]
            open_error = struct.unpack("<i", payload[11:15])[0]
            print("STATUS "
                  f"mega_connected={bool(connected)} usb_seen={bool(seen)} "
                  f"addr={address} vidpid={vid:04X}:{pid:04X} "
                  f"last_esp_error=0x{error & 0xFFFFFFFF:08X} "
                  f"ch34x_open_error=0x{open_error & 0xFFFFFFFF:08X}")
        else:
            print(f"STATUS malformed length={len(payload)} payload={payload.hex()}")
    elif kind == RSP_MEGA_RX:
        print("MEGA_RX", payload.decode("utf-8", "replace"), repr(payload))
    elif kind == RSP_ERROR and len(payload) == 5:
        command = payload[0]
        error = struct.unpack("<i", payload[1:])[0]
        print(f"P4_ERROR command=0x{command:02X} esp_err=0x{error & 0xFFFFFFFF:08X}")
    else:
        print(f"FRAME type=0x{kind:02X} payload={payload.hex()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    echo = sub.add_parser("echo")
    echo.add_argument("text")
    sub.add_parser("reset")
    args = ap.parse_args()

    bridge = Bridge(args.port)
    try:
        for response in bridge.hello():
            show(*response)
        if args.command == "status":
            bridge.send(CMD_GET_STATUS)
        elif args.command == "echo":
            bridge.send(CMD_MEGA_TX, (args.text + "\n").encode("utf-8"))
        elif args.command == "reset":
            bridge.send(CMD_RESET_MEGA)
        while True:
            try:
                kind, payload = bridge.receive(2.0)
            except TimeoutError:
                if args.command == "echo":
                    print("TIMEOUT_WAITING_FOR_MEGA_RX (P4 link may be healthy; Mega application did not reply)")
                    break
                raise
            show(kind, payload)
            if args.command in ("status", "reset") and kind in (RSP_STATUS, RSP_ERROR):
                break
            if args.command == "echo" and kind in (RSP_MEGA_RX, RSP_ERROR):
                break
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
