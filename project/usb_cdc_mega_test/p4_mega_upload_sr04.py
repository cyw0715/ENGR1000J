"""Upload the fixed Mega SR04 diagnostic HEX through PC -> P4 -> CH340.

Safety boundaries:
- Targets only an ATmega2560 that passes the read-only STK500v2 probe.
- Only writes the Arduino application area [0, 0x3E000), never bootloader,
  fuses, lock bits, or EEPROM.
- Reads and saves the old application image first.
- Programs 256-byte pages, reads each page back, and compares before continuing.
- Accepts only the project SR04 diagnostic .hex unless --hex is explicitly used.

Protocol facts grounded in Arduino's stk500boot_v2 source and avrdude config:
- 115200 bps Wiring/STK500v2
- ATmega2560 signature 1E9801
- application area ends at 0x3E000 (8 KiB bootloader reserved)
- flash pages are 256 bytes
"""
from __future__ import annotations

import argparse
import hashlib
import struct
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from p4_mega_bridge_client import (  # noqa: E402
    Bridge, CMD_MEGA_TX, CMD_RESET_MEGA, CMD_SET_LINE,
    RSP_ERROR, RSP_MEGA_RX, RSP_STATUS,
)
from p4_mega_stk500v2_probe import (  # noqa: E402
    MESSAGE_START, TOKEN, CMD_SIGN_ON, CMD_READ_SIGNATURE_ISP, STATUS_CMD_OK,
    EXPECTED_SIGNATURE, stk_frame, parse_stk_frame,
)

CMD_LOAD_ADDRESS = 0x06
CMD_PROGRAM_FLASH_ISP = 0x13
CMD_READ_FLASH_ISP = 0x14
CMD_LEAVE_PROGMODE_ISP = 0x11
PAGE_SIZE = 256
APP_END = 0x3E000  # 253,952 B: Arduino Mega 2560 application area
DEFAULT_HEX = HERE / "mega_sr04_bridge_diagnostic" / "build_atmega2560" / "mega_sr04_bridge_diagnostic.ino.hex"
DEFAULT_HEX_SHA256 = "13AA62A8240CCEE6746D5563C70FEDC35F956B12C70C41FA12F7C61DD021D7AB"


def require_status_ok(kind: int, payload: bytes, context: str):
    if kind == RSP_ERROR:
        command = payload[0] if payload else 0
        error = struct.unpack("<i", payload[1:5])[0] if len(payload) == 5 else -1
        raise RuntimeError(f"{context}: P4 error cmd=0x{command:02X}, esp_err=0x{error & 0xFFFFFFFF:08X}")
    if kind != RSP_STATUS:
        return False
    if not payload or not payload[0]:
        raise RuntimeError(f"{context}: Mega disconnected")
    return True


class StkBridge:
    def __init__(self, port: str):
        self.bridge = Bridge(port)
        self.sequence = 1
        self.rx = bytearray()

    def close(self):
        self.bridge.close()

    def _next_sequence(self):
        seq = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return seq

    def wait_status(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            kind, payload = self.bridge.receive(max(0.05, deadline - time.monotonic()))
            if require_status_ok(kind, payload, "P4 bridge"):
                return
        raise TimeoutError("Timed out waiting for P4 status")

    def bridge_command(self, kind: int, payload: bytes = b""):
        self.bridge.send(kind, payload)
        self.wait_status()

    def reset_and_enter_bootloader(self):
        self.bridge_command(CMD_SET_LINE, struct.pack("<IBBB", 115200, 0, 0, 8))
        self.bridge_command(CMD_RESET_MEGA)
        time.sleep(0.30)
        last_error = None
        for _ in range(6):
            try:
                response = self.command(bytes((CMD_SIGN_ON,)), timeout=0.7)
                if len(response) >= 3 and response[0] == CMD_SIGN_ON and response[1] == STATUS_CMD_OK:
                    name = response[3:3 + response[2]].decode("ascii", "replace")
                    print(f"BOOTLOADER_SIGN_ON={name}")
                    return
                last_error = RuntimeError(f"bad sign-on response {response.hex(' ')}")
            except (TimeoutError, RuntimeError) as exc:
                last_error = exc
                time.sleep(0.10)
        raise RuntimeError(f"Cannot enter Mega bootloader: {last_error}")

    def command(self, payload: bytes, timeout: float = 2.0) -> bytes:
        seq = self._next_sequence()
        self.bridge.send(CMD_MEGA_TX, stk_frame(seq, payload))
        deadline = time.monotonic() + timeout
        pending_status = False
        while time.monotonic() < deadline:
            kind, data = self.bridge.receive(max(0.05, deadline - time.monotonic()))
            if kind == RSP_ERROR:
                require_status_ok(kind, data, f"STK cmd 0x{payload[0]:02X}")
            if kind == RSP_STATUS:
                require_status_ok(kind, data, f"STK cmd 0x{payload[0]:02X}")
                pending_status = True
                continue
            if kind != RSP_MEGA_RX:
                continue
            self.rx.extend(data)
            while True:
                parsed = parse_stk_frame(self.rx)
                if parsed is None:
                    break
                response_seq, response = parsed
                if response_seq == seq:
                    return response
        raise TimeoutError(f"No STK response cmd=0x{payload[0]:02X}, P4_TX_status={pending_status}")

    def command_ok(self, payload: bytes, timeout: float = 2.0):
        response = self.command(payload, timeout)
        if len(response) < 2 or response[0] != payload[0] or response[1] != STATUS_CMD_OK:
            raise RuntimeError(f"STK command 0x{payload[0]:02X} failed: {response.hex(' ')}")
        return response

    def verify_signature(self):
        signature = bytearray()
        for index in range(3):
            response = self.command_ok(bytes((CMD_READ_SIGNATURE_ISP, 0, 0x30, 0, index, 0)), 1.0)
            if len(response) < 4 or response[3] != STATUS_CMD_OK:
                raise RuntimeError(f"Bad signature response {response.hex(' ')}")
            signature.append(response[2])
        if bytes(signature) != EXPECTED_SIGNATURE:
            raise RuntimeError(f"Target signature {signature.hex().upper()} != expected {EXPECTED_SIGNATURE.hex().upper()}")
        print("TARGET_SIGNATURE=" + signature.hex().upper())

    def load_address(self, byte_address: int):
        if byte_address < 0 or byte_address >= APP_END or byte_address & 1:
            raise ValueError(f"Invalid application byte address 0x{byte_address:X}")
        word_address = byte_address >> 1
        self.command_ok(bytes((CMD_LOAD_ADDRESS, (word_address >> 24) & 0xFF,
                               (word_address >> 16) & 0xFF, (word_address >> 8) & 0xFF,
                               word_address & 0xFF)))

    def read_page(self, byte_address: int) -> bytes:
        self.load_address(byte_address)
        response = self.command_ok(bytes((CMD_READ_FLASH_ISP, PAGE_SIZE >> 8, PAGE_SIZE & 0xFF)), 2.0)
        if len(response) != PAGE_SIZE + 3 or response[-1] != STATUS_CMD_OK:
            raise RuntimeError(f"Bad READ_FLASH response at 0x{byte_address:05X}: len={len(response)}")
        return response[2:-1]

    def program_page(self, byte_address: int, page: bytes):
        if len(page) != PAGE_SIZE:
            raise ValueError("page must be exactly 256 bytes")
        self.load_address(byte_address)
        # Mega stk500boot_v2 uses bytes 1..2 for size and data starts at byte 10.
        # The other fields mirror Arduino avrdude's paged STK500v2 command:
        # mode 0xC1 (paged/write/last), delay 10, LOADPAGE 0x40,
        # WRITEPAGE 0x4C, READ 0x20, readback FF/FF.
        command = bytes((CMD_PROGRAM_FLASH_ISP, PAGE_SIZE >> 8, PAGE_SIZE & 0xFF,
                         0xC1, 10, 0x40, 0x4C, 0x20, 0xFF, 0xFF)) + page
        response = self.command_ok(command, 3.0)
        if len(response) != 2:
            raise RuntimeError(f"Bad PROGRAM_FLASH response at 0x{byte_address:05X}: {response.hex(' ')}")

    def leave_programming(self):
        self.command_ok(bytes((CMD_LEAVE_PROGMODE_ISP,)), 2.0)


def parse_intel_hex(path: Path) -> tuple[bytearray, int]:
    memory = bytearray(b"\xFF" * APP_END)
    highest = 0
    base = 0
    for number, text in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not text:
            continue
        if not text.startswith(":"):
            raise ValueError(f"HEX line {number}: missing ':'")
        raw = bytes.fromhex(text[1:])
        if len(raw) < 5 or len(raw) != raw[0] + 5 or sum(raw) & 0xFF:
            raise ValueError(f"HEX line {number}: invalid length/checksum")
        count, address, record_type = raw[0], (raw[1] << 8) | raw[2], raw[3]
        data = raw[4:4 + count]
        if record_type == 0x00:
            absolute = base + address
            if absolute + count > APP_END:
                raise ValueError(f"HEX tries to write outside Mega application area: 0x{absolute:X}")
            memory[absolute:absolute + count] = data
            highest = max(highest, absolute + count)
        elif record_type == 0x01:
            break
        elif record_type == 0x04:
            if count != 2:
                raise ValueError(f"HEX line {number}: malformed extended linear address")
            base = ((data[0] << 8) | data[1]) << 16
        else:
            raise ValueError(f"HEX line {number}: unsupported record type {record_type}")
    if highest == 0:
        raise ValueError("HEX has no application data")
    programmed_bytes = (highest + PAGE_SIZE - 1) // PAGE_SIZE * PAGE_SIZE
    return memory[:programmed_bytes], highest


def save_backup(stk: StkBridge, out_path: Path):
    """Read application flash only; this has no target write side effects."""
    backup = bytearray()
    print(f"READING_EXISTING_APP bytes={APP_END}")
    for address in range(0, APP_END, PAGE_SIZE):
        backup.extend(stk.read_page(address))
        if address % (PAGE_SIZE * 128) == 0:
            print(f"BACKUP_PROGRESS {address // PAGE_SIZE}/{APP_END // PAGE_SIZE} pages")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(backup)
    digest = hashlib.sha256(backup).hexdigest().upper()
    out_path.with_suffix(out_path.suffix + ".sha256.txt").write_text(digest + "  " + out_path.name + "\n", encoding="ascii")
    print(f"BACKUP_SAVED={out_path}\nBACKUP_SHA256={digest}")


def upload(port: str, hex_path: Path, backup_path: Path, skip_backup: bool):
    digest = hashlib.sha256(hex_path.read_bytes()).hexdigest().upper()
    if hex_path.resolve() == DEFAULT_HEX.resolve() and digest != DEFAULT_HEX_SHA256:
        raise RuntimeError(f"Fixed SR04 HEX checksum mismatch: {digest}")
    image, highest = parse_intel_hex(hex_path)
    pages = len(image) // PAGE_SIZE
    print(f"HEX={hex_path}\nHEX_SHA256={digest}\nAPP_DATA_BYTES={highest}\nPAGES_TO_WRITE={pages}")

    stk = StkBridge(port)
    try:
        stk.reset_and_enter_bootloader()
        stk.verify_signature()
        if not skip_backup:
            save_backup(stk, backup_path)
            # Re-enter, because a full backup takes longer than the Mega bootloader timeout.
            stk.reset_and_enter_bootloader()
            stk.verify_signature()
        for page_index, address in enumerate(range(0, len(image), PAGE_SIZE), 1):
            page = image[address:address + PAGE_SIZE]
            stk.program_page(address, page)
            actual = stk.read_page(address)
            if actual != page:
                raise RuntimeError(f"VERIFY_FAIL page={page_index} address=0x{address:05X}")
            print(f"PAGE_OK {page_index}/{pages} address=0x{address:05X}")
        stk.leave_programming()
        print("UPLOAD_AND_READBACK_VERIFY_OK")
    finally:
        stk.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--hex", type=Path, default=DEFAULT_HEX)
    parser.add_argument("--backup", type=Path,
                        default=HERE / "mega_backups" / "mega_pre_sr04_application_2026-07-24.bin")
    parser.add_argument("--skip-backup", action="store_true", help="not recommended; does not read existing app first")
    args = parser.parse_args()
    upload(args.port, args.hex.resolve(), args.backup.resolve(), args.skip_backup)


if __name__ == "__main__":
    main()
