"""Faithfulness and safety contracts for the 111(1).ino Remote hybrid."""
from __future__ import annotations

import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent
ORIGINAL = ROOT / "reference" / "111_1_original.ino"
HYBRID = ROOT / "faithful_remote_hybrid" / "faithful_remote_hybrid.ino"
ORIGINAL_SHA256 = "1973c42c856dd61e18a53a6e8ca50d0e7503ce2a498bed8b9e75497d92f651f8"


def function_body(source: str, name: str) -> str:
    start = source.index(f"{name}(")
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError(f"unterminated {name}")


def preset_matrix(source: str) -> list[list[int]]:
    block = re.search(
        r"PRESET_LENGTH_MM\s*\[PRESET_COUNT\]\[4\]\s*=\s*\{(.*?)\};",
        source,
        re.S,
    )
    if not block:
        raise AssertionError("preset matrix missing")
    rows = []
    for row in re.findall(r"\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*\}", block.group(1)):
        rows.append([int(value) for value in row])
    return rows


class FaithfulRemoteContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.original_bytes = ORIGINAL.read_bytes()
        cls.original = cls.original_bytes.decode("utf-8")
        cls.hybrid = HYBRID.read_text(encoding="utf-8")

    def test_original_reference_hash_is_immutable(self) -> None:
        import hashlib
        self.assertEqual(hashlib.sha256(self.original_bytes).hexdigest(), ORIGINAL_SHA256)

    def test_all_sixteen_presets_are_byte_value_equivalent(self) -> None:
        original = preset_matrix(self.original)
        hybrid = preset_matrix(self.hybrid)
        self.assertEqual(len(original), 16)
        self.assertEqual(hybrid, original)

    def test_ps2_pin_contract_is_identical(self) -> None:
        for name in ("PS2_DAT", "PS2_CMD", "PS2_CLK", "PS2_CS"):
            pattern = rf"const\s+(?:int|uint8_t)\s+{name}\s*=\s*(\d+)"
            self.assertEqual(re.search(pattern, self.hybrid).group(1), re.search(pattern, self.original).group(1))

    def test_remote_timing_and_confirmation_constants_are_identical(self) -> None:
        names = (
            "RECONNECT_INTERVAL_MS", "PS2_MAX_READ_FAILURES",
            "SENSOR_STALE_TIMEOUT_MS", "PRESET_MODE_RUMBLE_DURATION_MS",
            "PRESET_MODE_RUMBLE_STRENGTH", "PRESET_TIMEOUT_MS",
        )
        aliases = {"REMOTE_REQUIRED_CONFIRMATIONS": "REQUIRED_CONFIRMATIONS",
                   "REMOTE_TARGET_TOLERANCE_MM": "TARGET_TOLERANCE_MM"}
        for hybrid_name in names:
            original_name = hybrid_name
            hp = re.search(rf"{hybrid_name}\s*=\s*(\d+)", self.hybrid)
            op = re.search(rf"{original_name}\s*=\s*(\d+)", self.original)
            self.assertIsNotNone(hp, hybrid_name)
            self.assertIsNotNone(op, original_name)
            self.assertEqual(hp.group(1), op.group(1), hybrid_name)
        for hybrid_name, original_name in aliases.items():
            hp = re.search(rf"{hybrid_name}\s*=\s*(\d+)", self.hybrid)
            op = re.search(rf"{original_name}\s*=\s*(\d+)", self.original)
            self.assertEqual(hp.group(1), op.group(1), hybrid_name)

    def test_sixteen_selection_priority_is_identical(self) -> None:
        original = function_body(self.original, "detectPresetSelection")
        hybrid = function_body(self.hybrid, "detectPresetSelection")
        expected = [
            "PSB_L2", "PSB_L1", "PSB_PAD_UP", "PSB_PAD_LEFT",
            "PSB_PAD_DOWN", "PSB_PAD_RIGHT", "leftStickLeft",
            "leftStickRight", "rightStickLeft", "rightStickRight",
            "PSB_SQUARE", "PSB_CROSS", "PSB_CIRCLE", "PSB_TRIANGLE",
            "PSB_R1", "PSB_R2",
        ]
        for body in (original, hybrid):
            positions = [body.rfind(token) for token in expected]
            self.assertTrue(all(position >= 0 for position in positions))
            self.assertEqual(positions, sorted(positions))

    def test_manual_mapping_matches_original_serial_contract(self) -> None:
        # A1..A4 retract / extend buttons printed by the canonical source.
        expected = {
            0: ("PSB_PAD_DOWN", "PSB_CROSS"),
            1: ("PSB_PAD_RIGHT", "PSB_CIRCLE"),
            2: ("PSB_PAD_UP", "PSB_TRIANGLE"),
            3: ("PSB_PAD_LEFT", "PSB_SQUARE"),
        }
        body = function_body(self.hybrid, "updateRemotePS2")
        extend = re.search(r"bool extend\[4\]\s*=\s*\{([^}]+)\}", body, re.S).group(1)
        retract = re.search(r"bool retract\[4\]\s*=\s*\{([^}]+)\}", body, re.S).group(1)
        extend_tokens = re.findall(r"PSB_[A-Z_]+", extend)
        retract_tokens = re.findall(r"PSB_[A-Z_]+", retract)
        self.assertEqual([(retract_tokens[i], extend_tokens[i]) for i in range(4)], list(expected.values()))

    def test_start_toggle_and_disconnect_stop_are_preserved(self) -> None:
        body = function_body(self.hybrid, "updateRemotePS2")
        self.assertIn("PSB_START", body)
        self.assertIn("PS2_MAX_READ_FAILURES", body)
        self.assertIn("stopRemoteMovement()", body)
        self.assertIn("enterPresetSelectionMode()", body)
        self.assertIn("enterManualMode()", body)

    def test_owner_boundary_prevents_unarmed_ps2_motion(self) -> None:
        body = function_body(self.hybrid, "updateRemotePS2")
        self.assertIn("state != STATE_REMOTE", body)
        commands = function_body(self.hybrid, "processCommand")
        self.assertIn('strcmp(command, "ARM REMOTE")', commands)
        self.assertIn('enterLocked(); Serial.println("OK STOPPED LOCKED ALL_LOW")', commands)
        self.assertIn('strcmp(command, "ARM IMU")', commands)
        self.assertIn('strcmp(kind, "SET_REMOTE")', commands)
        self.assertIn('strcmp(kind, "SET_DELTA")', commands)

    def test_remote_keeps_live_bounds_and_sensor_gate(self) -> None:
        drive = function_body(self.hybrid, "driveRemoteLeg")
        self.assertIn("SENSOR_RANGE_MIN_MM", drive)
        self.assertIn("SENSOR_RANGE_MAX_MM", drive)
        self.assertIn("SENSOR_STALE_TIMEOUT_MS", drive)
        movement = function_body(self.hybrid, "updateRemoteTargetMovement")
        self.assertIn("allSensorsFreshAndInRange()", movement)
        self.assertIn("PRESET_TIMEOUT_MS", movement)
        self.assertIn("REMOTE_REQUIRED_CONFIRMATIONS", movement)


if __name__ == "__main__":
    unittest.main()
