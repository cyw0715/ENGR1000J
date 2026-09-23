"""V2 safety-contract tests for the Mega IMU controller. No hardware access."""
from pathlib import Path
import re
import unittest

SKETCH = Path(__file__).with_name("mega_lander_imu_controller.ino")


class MegaImuControllerContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SKETCH.read_text(encoding="utf-8")

    def test_boot_and_fault_states_are_safe(self):
        for token in ("STATE_DISARMED", "STATE_FAULT", "STATE_ESTOP", "hardStopAll()"):
            self.assertIn(token, self.source)
        self.assertIn("BOOT DISARMED ALL_LOW", self.source)
        # Communication/session timeouts are STOP-recoverable for lab iteration;
        # actuator/sensor/direction faults remain latched until reset.
        self.assertIn('strcmp(faultReason, "heartbeat_timeout") == 0', self.source)
        self.assertNotIn('strcmp(faultReason, "session_timeout")', self.source)
        self.assertIn("state == STATE_FAULT && !recoverable", self.source)
        self.assertIn("ERR FAULT_LATCHED_RESET_REQUIRED", self.source)

    def test_arm_captures_one_physical_baseline(self):
        self.assertIn('strcmp(command, "ARM IMU")', self.source)
        self.assertIn("allSensorsFreshAndInRange()", self.source)
        self.assertIn("armBaselineA[a] = (int16_t)currentMM[a]", self.source)
        # SET_DELTA may read the fixed baseline for absolute range validation,
        # but must never recapture or assign it.
        set_delta = self.source[self.source.index('strcmp(kind, "SET_DELTA")'):]
        self.assertNotRegex(set_delta, r"armBaselineA\s*\[[^]]+\]\s*=")

    def test_verified_mapping_and_feedback_delta(self):
        self.assertIn("{3, 2, 0, 1}", self.source)
        self.assertIn("(int16_t)currentMM[a] - armBaselineA[a]", self.source)
        self.assertIn("A1=RL, A2=RR, A3=FR, A4=FL", self.source)

    def test_continuous_feedback_drive(self):
        self.assertNotIn("PULSE_MAX_MS", self.source)
        self.assertNotIn("finishPulse", self.source)
        self.assertNotIn("pulseStart", self.source)
        self.assertRegex(self.source, r"REVERSAL_DEAD_TIME_MS\s*=\s*120")
        self.assertIn("if (outputDirA[a] == desired)", self.source)
        self.assertIn("continuous drive; do not pulse", self.source)
        self.assertIn("abs(error) <= TARGET_TOLERANCE_MM", self.source)
        self.assertIn("stopLeg(a)", self.source)
        self.assertNotIn("CONTROL_SESSION_MAX_MS", self.source)
        self.assertNotIn("delay(30000", self.source)

    def test_local_watchdogs_and_ranges(self):
        self.assertRegex(self.source, r"HEARTBEAT_TIMEOUT_MS\s*=\s*2000")
        self.assertNotIn("CONTROL_SESSION_MAX_MS", self.source)
        self.assertIn("SENSOR_STALE_TIMEOUT_MS", self.source)
        self.assertIn("sensor_invalid_or_range", self.source)
        self.assertRegex(self.source, r"SENSOR_RANGE_MIN_MM\s*=\s*50")
        self.assertRegex(self.source, r"SENSOR_RANGE_MAX_MM\s*=\s*400")
        self.assertRegex(self.source, r"DELTA_PROTOCOL_MAX_MM\s*=\s*350")
        self.assertIn("armBaselineA[a] + candidate[solver]", self.source)
        self.assertIn("uint8_t a = SOLVER_TO_MEGA[solver]", self.source)
        self.assertIn("ERR SET_DELTA_ABSOLUTE_RANGE", self.source)
        range_check = self.source.index("ERR SET_DELTA_ABSOLUTE_RANGE")
        target_commit = self.source.index("targetDelta[i] = candidate[i]")
        self.assertLess(range_check, target_commit)
        self.assertRegex(self.source, r"TARGET_TOLERANCE_MM\s*=\s*1")

    def test_monotonic_sequence_and_exact_protocol(self):
        for cmd in ("PING", "STATUS", "ARM IMU", "HEARTBEAT", "SET_DELTA", "STOP", "ESTOP"):
            self.assertIn(cmd, self.source)
        self.assertIn("seq <= lastCommandSeq", self.source)
        self.assertIn("ERR SET_DELTA_EXTRA", self.source)

    def test_status_is_compact_and_machine_parseable(self):
        self.assertIn('"IMU_STATUS mode="', self.source)
        self.assertIn('" target="', self.source)
        self.assertIn('" applied="', self.source)
        self.assertIn("directionVerified", self.source)

    def test_nonblocking_sr04_camera_center_anchor(self):
        self.assertRegex(self.source, r"SR04_TRIG_PIN\s*=\s*53")
        self.assertRegex(self.source, r"SR04_ECHO_PIN\s*=\s*52")
        self.assertIn("ISR(PCINT0_vect)", self.source)
        self.assertIn("PCMSK0 |= _BV(PCINT1)", self.source)
        self.assertGreater(
            self.source.index("PCMSK0 |= _BV(PCINT1)"),
            self.source.index('latchFault("sensor_init")'),
        )
        self.assertIn('strcmp(command, "GET_DISTANCE")', self.source)
        self.assertIn('Serial.print("SR04 seq=")', self.source)
        self.assertIn("updateSr04();", self.source)
        self.assertNotIn("pulseIn(", self.source)
        self.assertIn("delayMicroseconds(2)", self.source)
        self.assertIn("delayMicroseconds(10)", self.source)

    def test_reference_pins_and_no_ps2(self):
        for pins in ("{22, 24, 26, 28}", "{23, 25, 27, 29}", "{30, 31, 32, 33}"):
            self.assertIn(pins, self.source)
        self.assertIn("TCA_ADDRESS = 0x70", self.source)
        self.assertNotIn("PS2X", self.source)

    def test_no_long_blocking_delays(self):
        for delay in re.findall(r"delay\((\d+)\)", self.source):
            self.assertLessEqual(int(delay), 1500)


if __name__ == "__main__":
    unittest.main(verbosity=2)
