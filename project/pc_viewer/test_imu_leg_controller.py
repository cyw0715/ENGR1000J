"""Unit tests for the IMU attitude PID leg-length controller."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from landing_control import (
    IMU_DEAD_ZONE_DEG,
    IMU_TARGET_TOLERANCE_DEG,
    ImuLegController,
    LEG_MAX,
    LEG_MIN,
    LEG_NAMES,
)


class ImuLegControllerTests(unittest.TestCase):
    def test_constants_match_user_contract(self):
        self.assertAlmostEqual(IMU_DEAD_ZONE_DEG, 5.0)
        self.assertAlmostEqual(IMU_TARGET_TOLERANCE_DEG, 1.0)

    def test_initial_state_is_idle(self):
        ctrl = ImuLegController()
        self.assertEqual(ctrl.mode, "idle")
        result = ctrl.compute(0.0, 0.0, now=1.0)
        self.assertEqual(result["mode"], "pid")
        self.assertTrue(result["within_target"])
        for name in LEG_NAMES:
            self.assertAlmostEqual(result["adjustments_mm"][name], 0.0, places=3)

    def test_positive_roll_extends_right_legs_and_retracts_left(self):
        ctrl = ImuLegController()
        # Outside dead zone → proportional
        result = ctrl.compute(roll_deg=10.0, pitch_deg=0.0, now=1.0)
        self.assertEqual(result["mode"], "outer")
        self.assertFalse(result["within_target"])
        adj = result["adjustments_mm"]
        # Right legs should get positive (longer), left legs negative (shorter)
        self.assertGreater(adj["front_right"], 0.0)
        self.assertGreater(adj["rear_right"], 0.0)
        self.assertLess(adj["front_left"], 0.0)
        self.assertLess(adj["rear_left"], 0.0)

    def test_positive_pitch_extends_front_legs_and_retracts_rear(self):
        ctrl = ImuLegController()
        result = ctrl.compute(roll_deg=0.0, pitch_deg=10.0, now=1.0)
        adj = result["adjustments_mm"]
        self.assertGreater(adj["front_left"], 0.0)
        self.assertGreater(adj["front_right"], 0.0)
        self.assertLess(adj["rear_left"], 0.0)
        self.assertLess(adj["rear_right"], 0.0)

    def test_inside_dead_zone_uses_pid_mode(self):
        ctrl = ImuLegController()
        # First call sets up prev
        ctrl.compute(0.0, 0.0, now=1.0)
        # Small angle inside dead zone
        result = ctrl.compute(3.0, 2.0, now=1.016)
        self.assertEqual(result["mode"], "pid")

    def test_outside_dead_zone_uses_outer_proportional_mode(self):
        ctrl = ImuLegController()
        result = ctrl.compute(8.0, 6.0, now=1.0)
        self.assertEqual(result["mode"], "outer")

    def test_symmetry_roll_and_pitch_opposite_signs_cancel(self):
        ctrl = ImuLegController()
        # Equal magnitude roll and pitch: FL gets (-roll+pitch)=0, RR gets (+roll-pitch)=0
        result = ctrl.compute(roll_deg=10.0, pitch_deg=10.0, now=1.0)
        adj = result["adjustments_mm"]
        self.assertAlmostEqual(adj["front_left"], 0.0, places=3)
        self.assertAlmostEqual(adj["rear_right"], 0.0, places=3)
        # FR gets double, RL gets negative double
        self.assertGreater(adj["front_right"], 0.0)
        self.assertLess(adj["rear_left"], 0.0)

    def test_within_target_flag(self):
        ctrl = ImuLegController()
        result = ctrl.compute(0.9, 0.9, now=1.0)
        self.assertTrue(result["within_target"])
        result = ctrl.compute(1.1, 0.5, now=1.016)
        self.assertFalse(result["within_target"])

    def test_reset_clears_integrals(self):
        ctrl = ImuLegController()
        # Build up some integral
        for i in range(10):
            ctrl.compute(4.0, 4.0, now=1.0 + i * 0.01)
        self.assertNotEqual(ctrl._integral_roll, 0.0)
        ctrl.reset()
        self.assertEqual(ctrl._integral_roll, 0.0)
        self.assertEqual(ctrl._integral_pitch, 0.0)
        self.assertIsNone(ctrl._prev_time)

    def test_integral_winds_up_in_pid_mode_then_clamps(self):
        ctrl = ImuLegController()
        ctrl.compute(0.0, 0.0, now=1.0)
        # Sustained error in dead zone → integral grows
        for i in range(200):
            ctrl.compute(4.0, 0.0, now=1.016 + i * 0.016)
        result = ctrl.compute(4.0, 0.0, now=5.0)
        # The integral should have been clamped
        self.assertLessEqual(abs(ctrl._integral_roll), 30.0 + 0.1)
        # But adjustments should still be non-zero
        self.assertGreater(abs(result["adjustments_mm"]["front_right"]), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
