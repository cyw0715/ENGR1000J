"""Unit tests for safety-critical non-GUI logic in lander_console.py."""
from __future__ import annotations

import sys
import unittest
from unittest import mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from lander_console import (  # noqa: E402
    clamp_deltas_to_live_leg_range,
    scale_relative_deltas_to_live_leg_range,
    stabilize_ultrasonic_samples,
    CV_CONTROL_VALIDATED,
    P4_SCAN_PREFIX,
    probe_p4_identity,
    scan_p4_subnet,
    RESTORE_LEG_TARGET_MM,
    restore_complete,
    restore_segment_deltas,
    RELATIVE_DEPTH_MODEL_ID,
    CV_VIEW_NAMES,
    CONTROL_MODE_CV,
    CONTROL_MODE_IMU,
    CONTROL_MODE_LOCKED,
    CONTROL_MODES,
    IMU_TCP_PORT,
    SENSOR_MAX_MM,
    SENSOR_MIN_MM,
    SOLVER_MAX_MM,
    SOLVER_MIN_MM,
    TargetSet,
    build_cv_triptych,
    build_target_set,
    build_remote_absolute_delta,
    validate_remote_absolute_targets,
    five_second_average_fps,
    overlay_fps,
    colorize_relative_depth,
    control_mode_description,
    validate_solver_target,
)


class LanderConsoleTargetTests(unittest.TestCase):
    def test_leg_solver_preview_is_integrated_without_command_path(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn("DepthTerrainHeightField", source)
        self.assertIn("ImuLegController", source)
        self.assertIn("solve_best_visible_platform", source)
        self.assertIn("_apply_leg_plan_preview", source)
        self.assertIn("平台200×200 mm", source)
        self.assertNotIn("def send_targets", source)
        self.assertNotIn('"/cmd"', source)

    def test_imu_mode_stops_video_and_uses_independent_20ms_refresh(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn("self.video_stop_event.set()", source)
        self.assertIn("def _start_video_stream", source)
        self.assertIn("self.after(20, self._refresh_imu_display)", source)
        self.assertIn("IMU模式：视频流已停止", source)

    def test_imu_mode_pauses_depth_and_runs_pid(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        # Gate depth submission by mode
        self.assertIn("CONTROL_MODE_IMU", source)
        self.assertIn("self.control_mode_var.get() != CONTROL_MODE_IMU", source)
        # IMU mode disables capture button
        self.assertIn("IMU模式", source)
        # PID controller integrated into IMU stream handler
        self.assertIn("ImuLegController", source)

    def test_middle_live_cloud_and_right_capture_paths_are_independent(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn('("live_depth",', source)
        self.assertIn('elif kind == "live_depth":', source)
        self.assertIn('elif kind == "capture_result":', source)
        self.assertNotIn('for index in (1, 2):\n                            self.cv_photos[index] = None', source)
        self.assertIn('self._show_panel_message(self.cv_labels[2]', source)

    def test_panel_image_fills_available_square_without_upscaling_past_box(self) -> None:
        from PIL import Image
        from lander_console import fit_panel_image

        source = Image.new("RGB", (800, 800), "black")
        fitted = fit_panel_image(source, 590, 520)
        self.assertEqual(fitted.size, (520, 520))

    def test_layout_uses_dynamic_panel_images_and_wrapped_messages(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertNotIn("PANEL_IMAGE_SIZE", source)
        self.assertIn("cv_area.rowconfigure(0, weight=1)", source)
        self.assertIn("wraplength=max(240, label.winfo_width() - 40)", source)
        self.assertIn("5s AVG FPS", source)

    def test_five_second_fps_uses_timestamp_span_and_excludes_old_frames(self) -> None:
        from collections import deque

        timestamps = deque([0.0, 1.0, 2.0, 5.0, 6.0])
        fps = five_second_average_fps(timestamps, now=6.0, window_seconds=5.0)
        self.assertAlmostEqual(fps, 3.0 / 5.0)
        self.assertEqual(list(timestamps), [1.0, 2.0, 5.0, 6.0])

    def test_fps_overlay_changes_only_a_copy(self) -> None:
        from PIL import Image

        source = Image.new("RGB", (320, 240), (80, 160, 240))
        overlaid = overlay_fps(source, 12.34)
        self.assertEqual(source.getpixel((4, 4)), (80, 160, 240))
        self.assertNotEqual(source.tobytes(), overlaid.tobytes())

    def test_capture_button_is_in_contour_header_before_image_row(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn('self.capture_controls.grid(row=0, column=1', source)
        self.assertNotIn('self.capture_controls.grid(row=2, column=0', source)

    def test_endpoint_mapping(self) -> None:
        low = TargetSet(250, 250, 250, 250)
        high = TargetSet(390, 390, 390, 390)
        self.assertEqual(low.sensor_targets(), {"fl": 60, "fr": 60, "rl": 60, "rr": 60})
        self.assertEqual(high.sensor_targets(), {"fl": 200, "fr": 200, "rl": 200, "rr": 200})
        self.assertEqual((SOLVER_MIN_MM, SOLVER_MAX_MM, SENSOR_MIN_MM, SENSOR_MAX_MM), (250, 390, 60, 200))

    def test_rejects_out_of_range_and_invalid_values(self) -> None:
        for bad in (249, 391, "", "three hundred", None):
            with self.assertRaises(ValueError):
                validate_solver_target(bad)

    def test_console_remote_uses_faithful_absolute_protocol(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        arm = source[source.index("def _arm_remote"):source.index("def _apply_remote_targets")]
        apply = source[source.index("def _apply_remote_targets"):source.index("def _arm_cv")]
        worker = source[source.index("class ImuCommandWorker"):source.index("class RestoreLegsWorker")]
        self.assertIn('{"remote_arm": True}', arm)
        self.assertNotIn('{"imu_arm": True}', arm)
        self.assertIn("validate_remote_absolute_targets", apply)
        self.assertNotIn("build_remote_absolute_delta", apply)
        self.assertIn("self._queue_imu_delta(absolute)", apply)
        self.assertIn('"remote_set_absolute" if session_kind == "remote"', worker)
        self.assertIn('elif payload.get("imu_arm")', worker)
        self.assertIn('else "imu_set_delta"', worker)

    def test_remote_arm_timeout_and_disarmed_only_contract(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn('modes = ("REMOTE_MANUAL", "REMOTE_PRESET") if require_owner else ("LOCKED", "DISARMED")', source)
        self.assertIn("time.monotonic() - self._arm_request_time > 3.0", source)
        self.assertIn("ARM确认超时：STOP已排队", source)
        self.assertIn("First-stage Remote control is independent of video/CV", source)
        self.assertIn("Automatic CV is preview-only; depth loss cannot own or stop Remote", source)

    def test_remote_health_gate_does_not_depend_on_sr04(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        health_block = source[
            source.index("def _healthy_for_command"):
            source.index("def _refresh_send_state")
        ]
        self.assertIn("_check_remote_health(require_owner=False)", health_block)
        self.assertIn("_check_remote_health(require_owner=True)", health_block)
        self.assertIn("_check_mega_health(require_armed=False)", health_block)
        self.assertNotIn("ultrasonic", health_block)

    def test_remote_absolute_targets_validate_vl53_coordinate(self) -> None:
        self.assertEqual(
            validate_remote_absolute_targets({"fl": "0", "fr": "125", "rl": "275", "rr": "400"}),
            {"fl": 0, "fr": 125, "rl": 275, "rr": 400},
        )
        for bad in (-1, 401, "", "10.5", None):
            with self.assertRaises(ValueError):
                validate_remote_absolute_targets({"fl": bad, "fr": 100, "rl": 100, "rr": 100})

    def test_remote_absolute_targets_convert_from_fixed_arm_baseline(self) -> None:
        health = {
            "mega_imu": {"mode": "ARMED", "applied_fl": 10, "applied_fr": -5,
                         "applied_rl": 20, "applied_rr": 0},
            "mega_lander": {
                "a4": {"mm": 130, "valid": True},  # FL baseline 120
                "a3": {"mm": 175, "valid": True},  # FR baseline 180
                "a1": {"mm": 220, "valid": True},  # RL baseline 200
                "a2": {"mm": 150, "valid": True},  # RR baseline 150
            },
        }
        result = build_remote_absolute_delta(
            {"fl": 140, "fr": 160, "rl": 250, "rr": 100}, health
        )
        self.assertEqual(result, {"fl": 20, "fr": -20, "rl": 50, "rr": -50})

    def test_remote_mode_replaces_cv_motion_path_but_keeps_preview_locked(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertFalse(CV_CONTROL_VALIDATED)
        self.assertIn('CONTROL_MODE_CV: "CV控制（第一阶段Remote）', source)
        self.assertIn('text="应用四腿目标"', source)
        self.assertIn("def _arm_remote", source)
        self.assertIn("def _apply_remote_targets", source)
        self.assertIn('self._arm_owner = "remote"', source)
        self.assertIn('self.arm_button.configure(text="ARM Remote")', source)
        arm_dispatch = source[source.index("def _arm_current_mode"):source.index("def _arm_remote")]
        self.assertIn("self._arm_remote()", arm_dispatch)
        self.assertNotIn("self._arm_cv()", arm_dispatch)

    def test_requires_all_four_valid_targets(self) -> None:
        result = build_target_set({"fl": "250", "fr": "280", "rl": "350", "rr": "390"})
        self.assertEqual(result.as_dict(), {"fl": 250, "fr": 280, "rl": 350, "rr": 390})
        with self.assertRaises(KeyError):
            build_target_set({"fl": 250, "fr": 250, "rl": 250})
    def test_control_modes_are_local_planning_states_only(self) -> None:
        self.assertEqual(CONTROL_MODES, (CONTROL_MODE_CV, CONTROL_MODE_LOCKED, CONTROL_MODE_IMU))
        self.assertEqual(IMU_TCP_PORT, 5001)
        for mode in CONTROL_MODES:
            description = control_mode_description(mode)
            self.assertIn("安全锁定", description)
        with self.assertRaises(ValueError):
            control_mode_description("unsafe-motion")
    def test_live_absolute_range_clamps_each_leg_independently(self) -> None:
        health = {
            "mega_lander": {
                "a1": {"mm": 184, "valid": True},  # baseline 179 + applied 5
                "a2": {"mm": 198, "valid": True},  # baseline 202 + applied -4
                "a3": {"mm": 235, "valid": True},  # baseline 232 + applied 3
                "a4": {"mm": 138, "valid": True},  # baseline 140 + applied -2
            },
            "mega_imu": {
                "applied_fl": -2, "applied_fr": 3,
                "applied_rl": 5, "applied_rr": -4,
            },
        }
        self.assertEqual(
            clamp_deltas_to_live_leg_range(
                {"fl": -350, "fr": 350, "rl": 350, "rr": -350}, health
            ),
            {"fl": -90.0, "fr": 168.0, "rl": 221.0, "rr": -152.0},
        )

    def test_ultrasonic_filter_uses_independent_stable_median(self) -> None:
        samples = [
            {"valid": True, "distance_mm": mm, "sample_age_us": 20_000,
             "valid_count": index, "source": "mega_usb"}
            for index, mm in enumerate((198, 200, 199, 201, 200, 199, 200), 1)
        ]
        result = stabilize_ultrasonic_samples(samples)
        self.assertEqual(result["distance_mm"], 200.0)
        self.assertEqual(result["filter_samples"], 7)
        self.assertEqual(result["filter_span_mm"], 3.0)

    def test_ultrasonic_filter_rejects_warmup_and_jump_window(self) -> None:
        with self.assertRaisesRegex(ValueError, "warming up"):
            stabilize_ultrasonic_samples([
                {"valid": True, "distance_mm": 200, "valid_count": i}
                for i in range(4)
            ])
        with self.assertRaisesRegex(ValueError, "unstable"):
            stabilize_ultrasonic_samples([
                {"valid": True, "distance_mm": mm, "valid_count": i}
                for i, mm in enumerate((30, 45, 200, 32, 198), 1)
            ])

    def test_cv_live_range_uses_uniform_scale_and_preserves_zero_heave(self) -> None:
        health = {
            "mega_lander": {
                "a4": {"mm": 390, "valid": True},  # FL baseline 390: +10 available
                "a3": {"mm": 200, "valid": True},
                "a1": {"mm": 200, "valid": True},
                "a2": {"mm": 200, "valid": True},
            },
            "mega_imu": {
                "applied_fl": 0, "applied_fr": 0,
                "applied_rl": 0, "applied_rr": 0,
            },
        }
        original = {"fl": 100.0, "fr": 50.0, "rl": -50.0, "rr": -100.0}
        scaled = scale_relative_deltas_to_live_leg_range(original, health)
        self.assertEqual(scaled, {"fl": 10, "fr": 5, "rl": -5, "rr": -10})
        self.assertEqual(sum(scaled.values()), 0)
        self.assertAlmostEqual(scaled["fr"] / scaled["fl"], 0.5)

    def test_cv_live_range_rejects_common_heave(self) -> None:
        health = {
            "mega_lander": {
                "a4": {"mm": 200, "valid": True}, "a3": {"mm": 200, "valid": True},
                "a1": {"mm": 200, "valid": True}, "a2": {"mm": 200, "valid": True},
            },
            "mega_imu": {},
        }
        with self.assertRaisesRegex(ValueError, "common heave"):
            scale_relative_deltas_to_live_leg_range(
                {"fl": 20.0, "fr": 10.0, "rl": 0.0, "rr": -10.0}, health
            )

    def test_restore_mapping_and_segment_clamp(self) -> None:
        health = {"mega_lander": {
            "a1": {"mm": 192, "valid": True},
            "a2": {"mm": 140, "valid": True},
            "a3": {"mm": 151, "valid": True},
            "a4": {"mm": 130, "valid": True},
        }}
        self.assertEqual(RESTORE_LEG_TARGET_MM, 150)
        self.assertEqual(
            restore_segment_deltas(health),
            {"fl": 20, "fr": -1, "rl": -42, "rr": 10},
        )
        self.assertFalse(restore_complete(health))

    def test_restore_complete_requires_all_four_within_one_mm(self) -> None:
        health = {"mega_lander": {
            "a1": {"mm": 149, "valid": True},
            "a2": {"mm": 150, "valid": True},
            "a3": {"mm": 151, "valid": True},
            "a4": {"mm": 150, "valid": True},
        }}
        self.assertTrue(restore_complete(health))
        health["mega_lander"]["a4"]["mm"] = 152
        self.assertFalse(restore_complete(health))

    def test_restore_rejects_invalid_sensor(self) -> None:
        health = {"mega_lander": {
            name: {"mm": 150, "valid": name != "a3"}
            for name in ("a1", "a2", "a3", "a4")
        }}
        with self.assertRaises(ValueError):
            restore_segment_deltas(health)

    def test_p4_probe_requires_exact_board_identity_and_matching_ip(self) -> None:
        self.assertEqual(P4_SCAN_PREFIX, "192.168.137.")
        with mock.patch("lander_console.fetch_json", return_value={
                "board": "ESP32-P4", "ip": "192.168.137.42"}):
            result = probe_p4_identity("192.168.137.42", timeout=0.01)
        self.assertEqual(result["host"], "192.168.137.42")
        with mock.patch("lander_console.fetch_json", return_value={
                "board": "not-p4", "ip": "192.168.137.42"}):
            self.assertIsNone(probe_p4_identity("192.168.137.42", timeout=0.01))
        with mock.patch("lander_console.fetch_json", return_value={
                "board": "ESP32-P4", "ip": "192.168.137.99"}):
            self.assertIsNone(probe_p4_identity("192.168.137.42", timeout=0.01))

    def test_scan_uses_full_host_range_and_sorts_by_latency(self) -> None:
        seen: list[str] = []
        def fake_probe(host: str, timeout: float):
            seen.append(host)
            if host.endswith(".20"):
                return {"host": host, "latency_ms": 20.0, "board": {}}
            if host.endswith(".10"):
                return {"host": host, "latency_ms": 10.0, "board": {}}
            return None
        with mock.patch("lander_console.probe_p4_identity", side_effect=fake_probe):
            results = scan_p4_subnet(prefix="192.168.137.", workers=4, timeout=0.01)
        self.assertEqual(len(seen), 254)
        self.assertIn("192.168.137.1", seen)
        self.assertIn("192.168.137.254", seen)
        self.assertEqual([item["host"] for item in results],
                         ["192.168.137.10", "192.168.137.20"])

    def test_imu_layout_and_restore_controls_exist(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn('self.panel_title_vars[1].set("当前姿态")', source)
        self.assertIn('self.panel_title_vars[2].set("四腿长度")', source)
        self.assertIn('text="一键恢复 150mm"', source)
        self.assertIn("class RestoreLegsWorker", source)
        self.assertIn("LEG_TO_MEGA_SENSOR", source)
        self.assertIn("self._render_imu_leg_lengths(health)", source)
        self.assertIn('text="自动扫描P4"', source)
        self.assertIn("self.after(150, self.scan_p4)", source)
        self.assertIn('board.get("board") != "ESP32-P4"', source)

    def test_automatic_cv_stays_preview_only_in_first_stage_remote_mode(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertFalse(CV_CONTROL_VALIDATED)
        self.assertIn("def _arm_cv", source)
        arm_cv = source[source.index("def _arm_cv"):source.index("def _arm_imu")]
        self.assertIn("Automatic CV actuation is structurally disabled", arm_cv)
        self.assertNotIn("imu_command_worker.submit", arm_cv)
        self.assertNotIn("_queue_imu_delta", arm_cv)
        self.assertIn('self.arm_button.configure(text="ARM Remote")', source)
        self.assertIn("self.cv_candidate_delta = cv_plan_to_relative_deltas", source)
        self.assertIn("Automatic CV remains display-only", source)
        self.assertNotIn('self._arm_owner == "cv"', source)
        self.assertNotIn("CV候选在ARM确认前已过期", source)
        self.assertIn("self.cv_candidate_delta = None", source)

    def test_high_dpi_and_responsive_layout_contract(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn("enable_windows_high_dpi()", source)
        self.assertIn("SetProcessDpiAwarenessContext", source)
        self.assertIn("configure_tk_scaling(self)", source)
        self.assertNotIn('uniform="data"', source)
        self.assertIn('uniform="imu_actions"', source)
        self.assertIn('style="CompactPrimary.TButton"', source)
        self.assertIn('style="CompactDanger.TButton"', source)
        self.assertIn("Keep native pixels here; resize exactly once", source)

    def test_cv_triptych_has_three_display_views(self) -> None:
        from PIL import Image

        self.assertEqual(CV_VIEW_NAMES, ("原始相机", "绝对高度 / 地面云图", "等高线图"))
        self.assertEqual(RELATIVE_DEPTH_MODEL_ID, "depth-anything/Depth-Anything-V2-Base-hf")
        source = Image.new("RGB", (4, 4), (80, 160, 240))
        original, cloud_placeholder, contour_placeholder = build_cv_triptych(source)
        self.assertEqual((original.size, cloud_placeholder.size, contour_placeholder.size), ((4, 4), (4, 4), (4, 4)))
        self.assertEqual(original.getpixel((1, 1)), (80, 160, 240))
        self.assertEqual(cloud_placeholder.getpixel((1, 1)), (24, 28, 36))
        self.assertEqual(contour_placeholder.getpixel((1, 1)), (24, 28, 36))
    def test_depth_colorization_has_stable_rgb_output(self) -> None:
        import numpy as np

        heatmap, value_range = colorize_relative_depth(
            np.array([[0.1, 0.4], [0.7, 1.0]], dtype=np.float32), None
        )
        self.assertEqual(heatmap.mode, "RGB")
        self.assertEqual(heatmap.size, (2, 2))
        self.assertLess(value_range[0], value_range[1])
        self.assertNotEqual(heatmap.getpixel((0, 0)), heatmap.getpixel((1, 1)))
    def test_metric_cloud_panel_keeps_pure_depth_colors_without_lander_labels(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertNotIn("draw_lander_projection(heatmap", source)
        self.assertNotIn('zip(("FL", "FR", "RR", "RL")', source)

    def test_staged_viewer_calibration_layout_is_supported(self) -> None:
        from absolute_terrain import calibration_path

        # The resolver's second candidate supports a viewer staged at <root>/mars_lander_console
        # with calibration copied to <root>/wifi_test/..., as used by the Windows launcher.
        import tempfile
        from unittest.mock import patch

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            viewer = root / "mars_lander_console"
            calibration = root / "wifi_test" / "calibration_runs" / "2026-07-24-ov5647-intrinsics-8x5-20mm" / "views"
            calibration.mkdir(parents=True)
            expected = calibration / "provisional_user_accepted_camera_calibration.json"
            expected.write_text("{}", encoding="utf-8")
            with patch("absolute_terrain.Path.resolve", return_value=viewer / "absolute_terrain.py"):
                self.assertEqual(calibration_path(), expected)

    def test_console_has_no_legacy_hardware_command_path(self) -> None:
        source = (Path(__file__).parent / "lander_console.py").read_text(encoding="utf-8")
        self.assertNotIn("def send_targets", source)
        self.assertNotIn("def confirm_and_send", source)
        self.assertNotIn('"/cmd"', source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
