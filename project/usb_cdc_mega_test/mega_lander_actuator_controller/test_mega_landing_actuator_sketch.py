"""Source-level safety contract tests for the Mega landing-actuator sketch."""

from pathlib import Path
import unittest


SKETCH = Path(__file__).with_name("mega_lander_actuator_controller.ino")


class MegaLandingActuatorSafetyContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SKETCH.read_text(encoding="utf-8")

    def test_boot_is_disarmed_and_drivers_start_disabled(self):
        self.assertIn("ControllerState controllerState = DISARMED;", self.source)
        self.assertIn("digitalWrite(MOTOR_EN[i], LOW);", self.source)

    def test_software_estop_and_watchdog_stop_every_motor(self):
        self.assertIn('strcmp(command, "ESTOP") == 0', self.source)
        self.assertIn("WATCHDOG_TIMEOUT_MS", self.source)
        self.assertIn("enterFault(\"heartbeat_timeout\")", self.source)
        self.assertIn("stopAll();", self.source)

    def test_invalid_or_uncalibrated_position_feedback_cannot_enable_motion(self):
        self.assertIn("POSITION_CALIBRATION_VALID", self.source)
        self.assertIn("position_calibration_invalid", self.source)
        self.assertIn("MAX_CONSECUTIVE_SENSOR_FAILURES", self.source)

    def test_target_protocol_requires_all_four_lengths_in_safe_range(self):
        self.assertIn("SET_TARGET", self.source)
        self.assertIn("LEG_MIN_MM", self.source)
        self.assertIn("LEG_MAX_MM", self.source)
        self.assertIn("targets_out_of_range", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
