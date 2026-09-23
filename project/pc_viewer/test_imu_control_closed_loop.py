"""Tests for IMU-only closed-loop control: ImuLegController, ImuDeltaCommandGenerator, console gates."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from landing_control import (
    IMU_DEAD_ZONE_DEG,
    IMU_TARGET_TOLERANCE_DEG,
    IMU_DELTA_MAX_MM,
    IMU_DELTA_SLOPE_MAX_MM_S,
    IMU_SEND_MIN_INTERVAL_S,
    IMU_STALE_THRESHOLD_S,
    transform_imu_to_platform,
    ImuLegController,
    ImuDeltaCommandGenerator,
    LEG_NAMES,
)


class ImuOrthogonalTransformTests(unittest.TestCase):
    def test_all_eight_axis_aligned_orthogonal_transforms(self):
        self.assertEqual(transform_imu_to_platform(3, 4, "identity"), (3.0, 4.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "rot90"), (4.0, -3.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "rot180"), (-3.0, -4.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "rot270"), (-4.0, 3.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "swap"), (4.0, 3.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "swap_neg"), (-4.0, -3.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "flip_roll"), (-3.0, 4.0))
        self.assertEqual(transform_imu_to_platform(3, 4, "flip_pitch"), (3.0, -4.0))

    def test_commissioned_swap_neg_with_upside_down_zero(self):
        # Physical +roll probe changed raw IMU pitch -19.115 -> -18.710.
        before = transform_imu_to_platform(
            -172.510, -19.115, "swap_neg", roll_zero_deg=-180.0, pitch_zero_deg=0.0
        )
        after_roll = transform_imu_to_platform(
            -172.555, -18.710, "swap_neg", roll_zero_deg=-180.0, pitch_zero_deg=0.0
        )
        self.assertLess(abs(after_roll[0]), abs(before[0]))

        # Physical negative-pitch correction must move raw roll toward -180°.
        corrected_pitch = transform_imu_to_platform(
            -173.0, -19.115, "swap_neg", roll_zero_deg=-180.0, pitch_zero_deg=0.0
        )
        self.assertLess(abs(corrected_pitch[1]), abs(before[1]))

    def test_unknown_transform_is_rejected(self):
        with self.assertRaises(ValueError):
            transform_imu_to_platform(1, 2, "unknown")


class ImuLegControllerTests(unittest.TestCase):
    def test_constants_match_contract(self):
        self.assertAlmostEqual(IMU_DEAD_ZONE_DEG, 5.0)
        self.assertAlmostEqual(IMU_TARGET_TOLERANCE_DEG, 1.0)

    def test_initial_state_is_idle(self):
        ctrl = ImuLegController()
        self.assertEqual(ctrl.mode, "idle")
        result = ctrl.compute(0.0, 0.0, now=1.0)
        self.assertEqual(result["mode"], "pid")
        self.assertTrue(result["within_target"])

    def test_positive_roll_extends_right_retracts_left(self):
        ctrl = ImuLegController()
        result = ctrl.compute(roll_deg=10.0, pitch_deg=0.0, now=1.0)
        self.assertEqual(result["mode"], "outer")
        adj = result["adjustments_mm"]
        self.assertGreater(adj["front_right"], 0.0)
        self.assertGreater(adj["rear_right"], 0.0)
        self.assertLess(adj["front_left"], 0.0)
        self.assertLess(adj["rear_left"], 0.0)

    def test_positive_pitch_extends_front_retracts_rear(self):
        ctrl = ImuLegController()
        result = ctrl.compute(roll_deg=0.0, pitch_deg=10.0, now=1.0)
        adj = result["adjustments_mm"]
        self.assertGreater(adj["front_left"], 0.0)
        self.assertGreater(adj["front_right"], 0.0)
        self.assertLess(adj["rear_left"], 0.0)
        self.assertLess(adj["rear_right"], 0.0)

    def test_5_degree_boundary_switches_mode(self):
        ctrl = ImuLegController()
        result_out = ctrl.compute(roll_deg=6.0, pitch_deg=0.0, now=1.0)
        self.assertEqual(result_out["mode"], "outer")
        ctrl2 = ImuLegController()
        result_in = ctrl2.compute(roll_deg=4.0, pitch_deg=0.0, now=1.0)
        self.assertEqual(result_in["mode"], "pid")

    def test_within_target_at_plus_minus_1_deg(self):
        ctrl = ImuLegController()
        result = ctrl.compute(0.9, 0.9, now=1.0)
        self.assertTrue(result["within_target"])
        ctrl2 = ImuLegController()
        result2 = ctrl2.compute(1.1, 0.5, now=1.0)
        self.assertFalse(result2["within_target"])

    def test_delta_sign_and_assignment(self):
        ctrl = ImuLegController()
        result = ctrl.compute(roll_deg=8.0, pitch_deg=0.0, now=1.0)
        adj = result["adjustments_mm"]
        self.assertGreater(adj["front_right"], 0)
        self.assertGreater(adj["rear_right"], 0)
        self.assertLess(adj["front_left"], 0)
        self.assertLess(adj["rear_left"], 0)

    def test_symmetry_roll_pitch_cancel(self):
        ctrl = ImuLegController()
        result = ctrl.compute(10.0, 10.0, now=1.0)
        adj = result["adjustments_mm"]
        self.assertAlmostEqual(adj["front_left"], 0.0, places=3)
        self.assertAlmostEqual(adj["rear_right"], 0.0, places=3)

    def test_outer_to_pid_transition_is_bumpless(self):
        ctrl = ImuLegController()
        outer = ctrl.compute(roll_deg=10.0, pitch_deg=0.0, now=1.0)
        inner = ctrl.compute(roll_deg=4.0, pitch_deg=0.0, now=1.02)
        self.assertEqual(outer["mode"], "outer")
        self.assertEqual(inner["mode"], "pid")
        # Entering PID must not reverse the corrective sign through a derivative kick.
        self.assertGreater(inner["adjustments_mm"]["front_right"], 0.0)
        self.assertLess(inner["adjustments_mm"]["front_left"], 0.0)
        self.assertLessEqual(max(abs(v) for v in inner["adjustments_mm"].values()), 15.0)

    def test_non_finite_attitude_is_rejected(self):
        ctrl = ImuLegController()
        for roll, pitch in ((float("nan"), 0.0), (0.0, float("inf")),
                            (float("-inf"), 0.0)):
            with self.assertRaises(ValueError):
                ctrl.compute(roll, pitch, now=1.0)

    def test_reset_clears_state(self):
        ctrl = ImuLegController()
        ctrl.compute(4.0, 4.0, now=1.0)
        ctrl.reset()
        self.assertEqual(ctrl.mode, "idle")
        self.assertIsNone(ctrl._prev_time)


class ImuDeltaCommandGeneratorTests(unittest.TestCase):
    def test_delta_constants(self):
        self.assertEqual(IMU_DELTA_MAX_MM, 350)
        self.assertEqual(IMU_DELTA_SLOPE_MAX_MM_S, 5)
        self.assertAlmostEqual(IMU_SEND_MIN_INTERVAL_S, 0.1)
        self.assertAlmostEqual(IMU_STALE_THRESHOLD_S, 2.0)

    def test_initial_state(self):
        gen = ImuDeltaCommandGenerator()
        self.assertEqual(gen.send_seq, 0)
        self.assertTrue(gen.is_imu_stale(10.0))

    def test_compute_returns_none_when_stale(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        result = gen.compute_deltas({"front_left": 5.0, "front_right": 0.0,
                                     "rear_left": 0.0, "rear_right": 0.0}, 5.0)
        self.assertIsNone(result)

    def test_compute_returns_none_when_rate_limited(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        result1 = gen.compute_deltas({"front_left": 5.0, "front_right": 0.0,
                                       "rear_left": 0.0, "rear_right": 0.0}, 1.0)
        self.assertIsNotNone(result1)
        result2 = gen.compute_deltas({"front_left": 6.0, "front_right": 0.0,
                                       "rear_left": 0.0, "rear_right": 0.0}, 1.05)
        self.assertIsNone(result2)

    def test_slope_limiting(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        result = gen.compute_deltas({"front_left": 20.0, "front_right": 0.0,
                                      "rear_left": 0.0, "rear_right": 0.0}, 1.0)
        self.assertIsNotNone(result)
        # The first command advances only one 100 ms control step.
        self.assertAlmostEqual(result["fl"], 0.5, places=2)
        gen.update_imu_time(1.1)
        result2 = gen.compute_deltas({"front_left": 20.0, "front_right": 0.0,
                                       "rear_left": 0.0, "rear_right": 0.0}, 1.1)
        self.assertIsNotNone(result2)
        self.assertAlmostEqual(result2["fl"], 1.0, delta=0.1)

    def test_name_mapping(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        result = gen.compute_deltas({"front_left": 3.0, "front_right": -2.0,
                                      "rear_left": 1.0, "rear_right": -4.0}, 1.0)
        self.assertIsNotNone(result)
        self.assertIn("fl", result)
        self.assertIn("fr", result)
        self.assertIn("rl", result)
        self.assertIn("rr", result)
        self.assertAlmostEqual(result["fl"], 0.5, delta=0.01)
        self.assertAlmostEqual(result["fr"], -0.5, delta=0.01)
        self.assertAlmostEqual(result["rl"], 0.5, delta=0.01)
        self.assertAlmostEqual(result["rr"], -0.5, delta=0.01)

    def test_zero_demand_holds_current_target(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        r1 = gen.compute_deltas({"front_left": 3.0, "front_right": 0.0,
                                  "rear_left": 0.0, "rear_right": 0.0}, 1.0)
        self.assertIsNotNone(r1)
        gen.update_imu_time(1.2)
        r2 = gen.compute_deltas({"front_left": 0.0, "front_right": 0.0,
                                  "rear_left": 0.0, "rear_right": 0.0}, 1.2)
        self.assertIsNone(r2)
        self.assertEqual(gen.current_target, r1)

    def test_seq_increments(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        gen.compute_deltas({"front_left": 3.0, "front_right": 0.0,
                             "rear_left": 0.0, "rear_right": 0.0}, 1.0)
        self.assertEqual(gen.send_seq, 1)
        gen.update_imu_time(1.2)
        gen.compute_deltas({"front_left": 5.0, "front_right": 0.0,
                             "rear_left": 0.0, "rear_right": 0.0}, 1.2)
        self.assertEqual(gen.send_seq, 2)

    def test_accumulates_relative_to_arm_baseline_and_holds_when_stable(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        demand = {"front_left": -20.0, "front_right": 20.0,
                  "rear_left": -20.0, "rear_right": 20.0}
        first = gen.compute_deltas(demand, 1.0)
        self.assertIsNotNone(first)
        gen.update_imu_time(1.2)
        second = gen.compute_deltas(demand, 1.2)
        self.assertIsNotNone(second)
        self.assertGreater(second["fr"], first["fr"])
        held = dict(second)
        gen.update_imu_time(1.4)
        self.assertIsNone(gen.compute_deltas(
            {"front_left": 0.0, "front_right": 0.0,
             "rear_left": 0.0, "rear_right": 0.0}, 1.4,
            hold=True,
        ))
        self.assertEqual(gen.current_target, held)

    def test_reset_clears(self):
        gen = ImuDeltaCommandGenerator()
        gen.update_imu_time(1.0)
        gen.compute_deltas({"front_left": 3.0, "front_right": 0.0,
                             "rear_left": 0.0, "rear_right": 0.0}, 1.0)
        gen.reset()
        self.assertEqual(gen.send_seq, 0)
        self.assertTrue(gen.is_imu_stale(10.0))


class ConsoleImuControlContractTests(unittest.TestCase):
    """Static tests: verify lander_console.py has required IMU control features."""

    @classmethod
    def setUpClass(cls):
        cls.source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")

    def test_arm_button_exists(self):
        self.assertIn("ARM IMU", self.source)
        self.assertIn("_arm_imu", self.source)
        self.assertIn("arm_button", self.source)

    def test_stop_button_exists(self):
        self.assertIn("STOP", self.source)
        self.assertIn("_stop_imu", self.source)
        self.assertIn("stop_button", self.source)

    def test_estop_button_exists(self):
        self.assertIn("ESTOP", self.source)
        self.assertIn("_estop_imu", self.source)
        self.assertIn("estop_button", self.source)

    def test_async_delta_queue_exists(self):
        self.assertIn("ImuCommandWorker", self.source)
        self.assertIn("_queue_imu_delta", self.source)
        self.assertIn("imu_set_delta", self.source)
        # The 20 ms Tk callback must never perform blocking HTTP itself.
        self.assertNotIn("def _send_imu_delta", self.source)

    def test_try_send_stop_method(self):
        self.assertIn("_try_send_stop", self.source)

    def test_stop_on_disconnect(self):
        self.assertIn("_try_send_stop()", self.source)

    def test_stop_on_window_close(self):
        self.assertIn("def _close(self)", self.source)
        self.assertIn("self._try_send_stop()", self.source)

    def test_imu_delta_generator_used(self):
        self.assertIn("ImuDeltaCommandGenerator", self.source)
        self.assertIn("_imu_delta_gen", self.source)

    def test_health_gating(self):
        self.assertIn("_check_mega_health", self.source)
        self.assertIn("mega_imu", self.source)
        self.assertIn("estop", self.source)

    def test_default_not_armed(self):
        self.assertIn("self.imu_armed = tk.BooleanVar(value=False)", self.source)

    def test_no_cv_in_imu_mode(self):
        self.assertIn("CONTROL_MODE_IMU", self.source)
        self.assertIn("control_mode_var.get() != CONTROL_MODE_IMU", self.source)

    def test_window_owned_arm_gate_exists(self):
        self.assertIn("_arm_request_pending", self.source)
        self.assertIn("or self._mega_arm_confirmed", self.source)

    def test_invalid_imu_triggers_stop(self):
        self.assertIn("math.isfinite", self.source)
        self.assertIn("IMU样本无效或非有限值", self.source)
        self.assertIn("_queue_imu_stop()", self.source)

    def test_command_pump_owns_heartbeat_and_latest_delta(self):
        self.assertIn("class ImuCommandWorker", self.source)
        self.assertIn("_next_heartbeat", self.source)
        self.assertIn("0.15 if ok else 0.05", self.source)
        self.assertIn("_latest_delta", self.source)
        self.assertIn("_sent_delta", self.source)
        self.assertIn("_consecutive_errors >= 3", self.source)
        self.assertIn("self.stop_session()", self.source)

    def test_mid_panel_shows_imu_data(self):
        self.assertIn("Yaw:", self.source)
        self.assertIn("Pitch:", self.source)
        self.assertIn("Roll:", self.source)

    def test_imu_mode_stops_video(self):
        self.assertIn("IMU模式：视频流已停止", self.source)

    def test_askyesno_arm_confirm(self):
        self.assertIn("askyesno", self.source)
        self.assertIn("确认 ARM IMU", self.source)

    def test_askyesno_estop_confirm(self):
        self.assertIn("确认 ESTOP", self.source)


class LegOrderMappingTests(unittest.TestCase):
    """Verify solver→Mega physical mapping consistency."""

    def test_solver_order_is_fl_fr_rl_rr(self):
        self.assertEqual(LEG_NAMES, ("front_left", "front_right",
                                      "rear_left", "rear_right"))

    def test_imu_leg_factors_signs(self):
        ctrl = ImuLegController()
        self.assertEqual(ctrl.LEG_FACTORS["front_left"], (-1.0, +1.0))
        self.assertEqual(ctrl.LEG_FACTORS["front_right"], (+1.0, +1.0))
        self.assertEqual(ctrl.LEG_FACTORS["rear_left"], (-1.0, -1.0))
        self.assertEqual(ctrl.LEG_FACTORS["rear_right"], (+1.0, -1.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
