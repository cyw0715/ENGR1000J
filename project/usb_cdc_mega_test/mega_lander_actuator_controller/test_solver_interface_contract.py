"""Static contract tests for the solver-to-Mega interface."""
from pathlib import Path
import unittest

SKETCH = Path(__file__).with_name("mega_lander_actuator_controller.ino")


class SolverInterfaceContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SKETCH.read_text(encoding="utf-8")

    def test_uses_user_confirmed_coordinate_contract(self):
        self.assertIn("SOLVER_MIN_MM = 250", self.source)
        self.assertIn("SOLVER_MAX_MM = 390", self.source)
        self.assertIn("SENSOR_MIN_MM = 60", self.source)
        self.assertIn("SENSOR_MAX_MM = 200", self.source)
        self.assertIn("SOLVER_TO_SENSOR_OFFSET_MM = 190", self.source)
        self.assertIn("solverMM - SOLVER_TO_SENSOR_OFFSET_MM", self.source)

    def test_requires_exactly_four_solver_lengths(self):
        self.assertIn("SET_TARGET <FL> <FR> <RL> <RR>", self.source)
        self.assertIn("ERR set_target_requires_FL_FR_RL_RR", self.source)
        self.assertIn("ERR set_target_requires_exactly_four_values", self.source)

    def test_no_boot_motion_and_preserves_existing_motion_safeguards(self):
        self.assertIn("Waiting for SET_TARGET FL FR RL RR; no automatic movement.", self.source)
        self.assertNotIn("EXTEND_TARGET_MM", self.source)
        self.assertNotIn("RETRACT_TARGET_MM", self.source)
        self.assertIn("REQUIRED_CONFIRMATIONS = 3", self.source)
        self.assertIn("MAX_MOVE_TIME_MS = 30000UL", self.source)
        self.assertIn("MAX_CONSECUTIVE_FAILURES = 10", self.source)
        self.assertIn("void emergencyStop", self.source)

    def test_uses_existing_verified_move_function(self):
        self.assertIn("moveActuatorToTarget(", self.source)
        self.assertIn("currentMM < sensorTargetMM", self.source)
        self.assertIn("emergencyStop(\"Solver target sequence failed.\")", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
