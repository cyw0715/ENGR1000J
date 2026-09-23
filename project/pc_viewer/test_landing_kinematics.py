"""Behavior tests for the constrained four-actuator landing-platform solver."""

import math
import unittest

from landing_control import (
    cv_plan_to_relative_deltas,
    CV_RELATIVE_DELTA_MAX_MM,
    LEG_MAX,
    LEG_MIN,
    LEG_NAMES,
    LEG_ANGLE_RAD,
    LEG_MOUNT_POINTS,
    CAMERA_TO_MOUNT_PLANE_MM,
    DepthTerrainHeightField,
    solve_best_level_platform,
    solve_best_visible_platform,
    solve_level_platform,
)


class LandingKinematicsTests(unittest.TestCase):
    def test_current_mechanical_contract(self):
        self.assertEqual(CAMERA_TO_MOUNT_PLANE_MM, 50.0)
        self.assertEqual((LEG_MIN, LEG_MAX), (250.0, 400.0))

    def test_depth_height_field_uses_camera_below_mount_plane(self):
        import numpy as np

        depth_m = np.full((9, 9), 0.300, dtype=np.float32)
        terrain = DepthTerrainHeightField(
            depth_m, fx_px=900.0, fy_px=900.0, cx_px=4.0, cy_px=4.0
        )
        self.assertAlmostEqual(terrain.camera_ground_depth_mm, 300.0)
        self.assertAlmostEqual(terrain.current_mount_height_mm, 350.0)
        self.assertAlmostEqual(terrain.get_height(0.0, 0.0), 0.0, places=5)

    def test_equidistant_fisheye_projects_known_angle(self):
        import numpy as np

        depth_m = np.full((801, 801), 1.0, dtype=np.float32)
        terrain = DepthTerrainHeightField(
            depth_m, fx_px=400.0, fy_px=400.0, cx_px=400.0, cy_px=400.0,
            projection_model="equidistant_fisheye",
        )
        # x/z=tan(45°)=1 maps to r=f*theta=400*pi/4≈314.16 px.
        u, v = terrain.project_ground_xy_to_pixel(1000.0, 0.0, 1000.0)
        self.assertAlmostEqual(u, 400.0 + 400.0 * math.pi / 4.0, places=3)
        self.assertAlmostEqual(v, 400.0, places=3)

    def test_calibrated_opencv_fisheye_projection_matches_opencv(self):
        import cv2
        import numpy as np
        from absolute_terrain import load_provisional_calibration

        calibration = load_provisional_calibration()
        depth_m = np.full((800, 800), 1.0, dtype=np.float32)
        terrain = DepthTerrainHeightField(
            depth_m, fx_px=calibration["fx_px"], fy_px=calibration["fy_px"],
            cx_px=calibration["cx_px"], cy_px=calibration["cy_px"],
            projection_model="opencv_fisheye",
            distortion_coefficients=np.array(calibration["D"]),
        )
        K = np.array(calibration["K"], dtype=np.float64)
        D = np.array(calibration["D"], dtype=np.float64)
        for x, y, z in ((100.0, 50.0, 300.0), (-150.0, 80.0, 500.0), (0.0, -200.0, 400.0)):
            ours = terrain.project_ground_xy_to_pixel(x, y, z)
            # Terrain +y maps to camera -y by the project coordinate contract.
            point = np.array([[[x, -y, z]]], dtype=np.float64)
            expected, _ = cv2.fisheye.projectPoints(
                point, np.zeros((3, 1)), np.zeros((3, 1)), K, D
            )
            self.assertTrue(np.allclose(ours, expected.reshape(2), atol=1e-6))

    def test_solver_is_invariant_to_uniform_terrain_elevation_shift(self):
        base = lambda x, y: 0.04 * x - 0.07 * y
        shifted = lambda x, y: base(x, y) + 1234.5
        first = solve_best_level_platform(base)
        second = solve_best_level_platform(shifted)
        self.assertEqual(first["status"], "safe")
        self.assertEqual(second["status"], "safe")
        for name in LEG_NAMES:
            self.assertAlmostEqual(first["lengths_mm"][name], second["lengths_mm"][name], places=5)
        self.assertAlmostEqual(
            second["platform_height_mm"] - first["platform_height_mm"], 1234.5, places=5
        )

    def test_cv_command_is_invariant_to_absolute_terrain_height(self):
        relative = lambda x, y: 0.04 * x - 0.07 * y
        absolute_shifted = lambda x, y: relative(x, y) + 5000.0
        first = cv_plan_to_relative_deltas(solve_best_level_platform(relative))
        second = cv_plan_to_relative_deltas(solve_best_level_platform(absolute_shifted))
        self.assertEqual(first, second)
        self.assertAlmostEqual(sum(first.values()), 0.0)

    def test_missing_depth_is_unobservable_not_mechanical_no_solution(self):
        result = solve_best_visible_platform(lambda _x, _y: float("nan"))
        self.assertEqual(result["status"], "unobservable")
        self.assertEqual(result["visible_leg_count"], 0)
        self.assertFalse(result["motion_permitted"])
        self.assertEqual(result["unsafe_reasons"], ["terrain_not_observed_at_leg_regions"])

    def test_partial_visibility_returns_only_observed_leg_solutions(self):
        def partially_visible_terrain(x, y):
            return 0.0 if x < 0.0 else float("nan")

        result = solve_best_visible_platform(partially_visible_terrain)
        self.assertEqual(result["status"], "partial_preview")
        self.assertFalse(result["motion_permitted"])
        self.assertGreater(result["visible_leg_count"], 0)
        self.assertLess(result["visible_leg_count"], 4)
        self.assertEqual(set(result["lengths_mm"]), {"front_left", "rear_left"})
        self.assertEqual(result["legs"]["front_right"]["preview_status"], "not_visible_or_no_solution")

    def test_auto_height_solver_selects_safe_centered_nominal_solution(self):
        result = solve_best_level_platform(lambda _x, _y: 0.0)
        self.assertEqual(result["status"], "safe")
        self.assertFalse(result["motion_permitted"])
        self.assertEqual(result["mode"], "preview_only")
        self.assertTrue(all(LEG_MIN <= value <= LEG_MAX for value in result["lengths_mm"].values()))
        self.assertLessEqual(max(result["lengths_mm"].values()) - min(result["lengths_mm"].values()), 1e-6)

    def test_legs_are_mounted_at_platform_corners(self):
        self.assertEqual(set(LEG_NAMES), {"front_left", "front_right", "rear_left", "rear_right"})
        expected_corners = {
            (-100.0, -100.0),
            (-100.0, 100.0),
            (100.0, -100.0),
            (100.0, 100.0),
        }
        actual_corners = {(mount[0], mount[1]) for mount in LEG_MOUNT_POINTS.values()}
        self.assertEqual(actual_corners, expected_corners)

    def test_cv_plan_converts_only_differential_shape(self):
        plan = {
            "status": "safe", "geometric_safe": True,
            "lengths_mm": {
                "front_left": 300.0, "front_right": 320.0,
                "rear_left": 340.0, "rear_right": 360.0,
            },
        }
        delta = cv_plan_to_relative_deltas(plan)
        self.assertEqual(delta, {"fl": -30.0, "fr": -10.0, "rl": 10.0, "rr": 30.0})
        self.assertAlmostEqual(sum(delta.values()), 0.0)
        shifted = dict(plan)
        shifted["lengths_mm"] = {key: value + 1000.0 for key, value in plan["lengths_mm"].items()}
        self.assertEqual(cv_plan_to_relative_deltas(shifted), delta)

    def test_cv_plan_scales_vector_without_destroying_axis_mix(self):
        plan = {
            "status": "safe", "geometric_safe": True,
            "lengths_mm": {
                "front_left": 100.0, "front_right": 200.0,
                "rear_left": 300.0, "rear_right": 400.0,
            },
        }
        delta = cv_plan_to_relative_deltas(plan)
        self.assertEqual(CV_RELATIVE_DELTA_MAX_MM, 100.0)
        self.assertAlmostEqual(max(abs(v) for v in delta.values()), 100.0)
        self.assertAlmostEqual(delta["fr"] / delta["fl"], 1.0 / 3.0, places=3)

    def test_cv_plan_rejects_partial_or_nonfinite_geometry(self):
        with self.assertRaises(ValueError):
            cv_plan_to_relative_deltas({
                "status": "partial_preview", "geometric_safe": False,
                "lengths_mm": {"front_left": 300.0},
            })
        with self.assertRaises(ValueError):
            cv_plan_to_relative_deltas({
                "status": "safe", "geometric_safe": True,
                "lengths_mm": {
                    "front_left": float("nan"), "front_right": 300.0,
                    "rear_left": 300.0, "rear_right": 300.0,
                },
            })

    def test_flat_ground_at_nominal_height_uses_equal_nominal_lengths(self):
        nominal_length = 350.0
        platform_height = nominal_length * math.cos(LEG_ANGLE_RAD)

        result = solve_level_platform(
            terrain_height=lambda _x, _y: 0.0,
            requested_platform_height_mm=platform_height,
        )

        self.assertEqual(result["status"], "safe")
        self.assertEqual(result["lengths_mm"], {name: nominal_length for name in LEG_NAMES})
        self.assertEqual(result["residual_tilt_deg"], {"roll": 0.0, "pitch": 0.0})
        self.assertTrue(all(leg["side_load_safe"] for leg in result["legs"].values()))

    def test_sloped_terrain_keeps_platform_level_with_axial_leg_lines(self):
        # Ground rises 0.10 mm per mm toward +y (front) and 0.05 toward +x (right).
        def terrain_height(x, y):
            return 0.05 * x + 0.10 * y

        result = solve_level_platform(
            terrain_height=terrain_height,
            requested_platform_height_mm=350.0 * math.cos(LEG_ANGLE_RAD),
        )

        self.assertEqual(result["status"], "safe")
        self.assertEqual(result["residual_tilt_deg"], {"roll": 0.0, "pitch": 0.0})
        self.assertGreater(result["lengths_mm"]["rear_left"], result["lengths_mm"]["front_left"])
        self.assertGreater(result["lengths_mm"]["rear_right"], result["lengths_mm"]["front_right"])
        self.assertGreater(result["lengths_mm"]["front_left"], result["lengths_mm"]["front_right"])
        for leg in result["legs"].values():
            self.assertLessEqual(abs(leg["contact_height_error_mm"]), 1e-6)
            self.assertTrue(leg["side_load_safe"])
            self.assertEqual(leg["side_load_reason"], "axial_only_with_spherical_joints")

    def test_unsafe_when_required_contact_is_shorter_than_minimum_travel(self):
        # The geometric contact is 20 mm shorter than the allowed 250 mm
        # retracted length. The solver must reject it, not clamp to 250 mm.
        result = solve_level_platform(
            terrain_height=lambda _x, _y: 0.0,
            requested_platform_height_mm=(LEG_MIN - 20.0) * math.cos(LEG_ANGLE_RAD),
        )

        self.assertEqual(result["status"], "unsafe_travel_limit")
        self.assertTrue(any(not leg["travel_safe"] for leg in result["legs"].values()))
        self.assertFalse(result["motion_permitted"])

    def test_unsafe_when_terrain_has_no_axial_contact_solution_in_travel_range(self):
        # Flat terrain has no axial intersection in the requested travel window
        # when the platform is higher than the maximum vertical leg reach.
        result = solve_level_platform(
            terrain_height=lambda _x, _y: 0.0,
            requested_platform_height_mm=(LEG_MAX + 10.0) * math.cos(LEG_ANGLE_RAD),
        )

        self.assertEqual(result["status"], "unsafe_no_contact_solution")
        self.assertFalse(result["motion_permitted"])
        self.assertFalse(result["legs"]["front_left"]["contact_solution_found"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
