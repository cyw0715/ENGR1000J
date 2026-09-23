"""Behavior tests for display-only absolute-height terrain fusion helpers."""
from __future__ import annotations

import sys
import unittest
from unittest import mock
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT))
from absolute_terrain import (  # noqa: E402
    CAPTURE_FRAME_COUNT,
    SELECTED_FRAME_COUNT,
    STABLE_FRAME_COUNT,
    STABLE_RELATIVE_SPREAD_LIMIT,
    BestOfThirtyDepthCapture,
    anchor_depth_with_ultrasonic,
    camera_fov_x_deg,
    fuse_metric_and_relative,
    ground_height_summary,
    load_provisional_calibration,
    metric_contour_map,
    metric_depth_heatmap,
    project_lander_plan,
    select_best_depth_frames,
    source_observability,
    stable_cloud_average,
)


class AbsoluteTerrainTests(unittest.TestCase):
    def test_stability_spread_limit_is_twenty_percent(self) -> None:
        self.assertAlmostEqual(STABLE_RELATIVE_SPREAD_LIMIT, 0.20)
        maps = [np.full((4, 4), 1.0 + index * 0.02, dtype=np.float32) for index in range(STABLE_FRAME_COUNT)]
        self.assertIsNotNone(stable_cloud_average(maps))

    def test_capture_collects_thirty_frames_then_stops(self) -> None:
        self.assertEqual((CAPTURE_FRAME_COUNT, SELECTED_FRAME_COUNT, STABLE_FRAME_COUNT), (30, 10, 10))
        capture = BestOfThirtyDepthCapture()
        self.assertFalse(capture.active)
        capture.start()
        for index in range(CAPTURE_FRAME_COUNT + 3):
            capture.accept(np.full((2, 2), 1.0 + index * 0.001, dtype=np.float32))
        self.assertFalse(capture.active)
        self.assertEqual(capture.count, CAPTURE_FRAME_COUNT)
        self.assertEqual(len(capture.frames), CAPTURE_FRAME_COUNT)
        capture.start()
        self.assertTrue(capture.active)
        self.assertEqual(capture.count, 0)

    def test_selects_ten_frames_closest_to_thirty_frame_median_reference(self) -> None:
        good = [np.full((4, 4), 2.0 + index * 0.001, dtype=np.float32) for index in range(25)]
        outliers = [np.full((4, 4), value, dtype=np.float32) for value in (0.2, 0.3, 6.0, 7.0, 8.0)]
        selected, indices, scores = select_best_depth_frames(good + outliers, SELECTED_FRAME_COUNT)
        self.assertEqual(len(selected), SELECTED_FRAME_COUNT)
        self.assertEqual(len(indices), SELECTED_FRAME_COUNT)
        self.assertTrue(all(index < 25 for index in indices))
        self.assertEqual(scores, sorted(scores))
        self.assertLess(max(scores), 0.01)

    def test_fusion_scales_relative_map_to_metric_median(self) -> None:
        metric = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
        relative = metric * 2.0
        fused, valid, scale = fuse_metric_and_relative(metric, relative)
        self.assertTrue(valid.all())
        self.assertAlmostEqual(scale, 0.5, places=5)
        self.assertTrue(np.allclose(fused, metric, atol=1e-5))

    def test_stable_cloud_requires_exactly_ten_valid_frames(self) -> None:
        maps = [np.full((2, 2), 1.6, dtype=np.float32) for _ in range(STABLE_FRAME_COUNT - 1)]
        self.assertIsNone(stable_cloud_average(maps))
        maps.append(np.full((2, 2), 1.6, dtype=np.float32))
        average = stable_cloud_average(maps)
        self.assertTrue(np.allclose(average, 1.6))

    def test_projection_is_symmetric_and_never_commands_hardware(self) -> None:
        plan = project_lander_plan(2.0, (800, 800), fx_px=2000.0, fy_px=2000.0, cx_px=400.0, cy_px=400.0)
        self.assertEqual(plan["mode"], "display_only")
        self.assertEqual(plan["platform_px"].shape, (4, 2))
        self.assertEqual(plan["feet_px"].shape, (4, 2))
        center = np.array([400.0, 400.0])
        self.assertTrue(np.all(np.linalg.norm(plan["feet_px"] - center, axis=1) >
                               np.linalg.norm(plan["platform_px"] - center, axis=1)))

    def test_provisional_fallback_preserves_stated_160_degree_spec(self) -> None:
        with mock.patch("absolute_terrain.accepted_fisheye_calibration_path", side_effect=FileNotFoundError):
            calibration = load_provisional_calibration()
        self.assertTrue(calibration["provisional"])
        f_px = calibration["fx_px"]
        edge_half_angle_deg = np.degrees((calibration["image_width_px"] / 2.0) / f_px)
        self.assertAlmostEqual(2.0 * edge_half_angle_deg, 113.28, places=2)
        self.assertAlmostEqual(calibration["diagonal_fov_deg"], 160.0)
    def test_metric_heatmap_uses_meter_range_and_marks_invalid_pixels(self) -> None:
        depth = np.array([[1.0, 2.0], [np.nan, 4.0]], dtype=np.float32)
        image, near_m, far_m = metric_depth_heatmap(depth)
        self.assertEqual(image.mode, "RGB")
        self.assertEqual(image.size, (2, 2))
        self.assertGreater(far_m, near_m)
        self.assertEqual(image.getpixel((0, 1)), (0, 0, 0))
    def test_metric_contour_map_draws_levels_and_marks_invalid_pixels(self) -> None:
        y, x = np.mgrid[0:32, 0:32]
        depth = (1.0 + x * 0.01 + y * 0.02).astype(np.float32)
        depth[0, 0] = np.nan
        image, levels = metric_contour_map(depth, level_count=8)
        self.assertEqual(image.mode, "RGB")
        self.assertEqual(image.size, (32, 32))
        self.assertGreaterEqual(len(levels), 2)
        self.assertEqual(image.getpixel((0, 0)), (0, 0, 0))
        self.assertGreater(len(set(image.getdata())), 4)

    def test_ground_summary_requires_valid_stable_cloud(self) -> None:
        depth = np.full((10, 10), 2.0, dtype=np.float32)
        summary = ground_height_summary(depth)
        self.assertAlmostEqual(summary["height_m"], 2.0)
        self.assertEqual(summary["valid_pixels"], 25)
        self.assertIsNone(ground_height_summary(np.full((10, 10), np.nan, dtype=np.float32)))
    def test_source_observability_rejects_uniform_and_historical_pattern_noise(self) -> None:
        uniform = np.full((800, 800), 128, dtype=np.uint8)
        self.assertFalse(source_observability(uniform)["observable"])
        bad_path = ROOT / "metric_depth_eval" / "results" / "20260723_191012_ov5647_metric_hypersim_base_source.png"
        bad = np.asarray(Image.open(bad_path).convert("L"))
        result = source_observability(bad)
        self.assertFalse(result["observable"], result)

    def test_source_observability_accepts_real_scene_structure(self) -> None:
        scene_path = ROOT / "metric_depth_eval" / "Depth-Anything-V2" / "assets" / "examples" / "demo01.jpg"
        scene = np.asarray(Image.open(scene_path).convert("RGB"))
        result = source_observability(scene)
        self.assertTrue(result["observable"], result)

    def test_ultrasonic_anchor_scales_at_calibrated_principal_point(self) -> None:
        depth = np.full((800, 800), 2.0, dtype=np.float32)
        depth[400, 400] = 99.0  # geometric image center must not be used
        anchored, meta = anchor_depth_with_ultrasonic(
            depth, {"valid": True, "distance_mm": 1000, "sample_age_us": 10_000, "source": "mega_usb"},
            cx_px=613.459, cy_px=452.471,
        )
        self.assertAlmostEqual(meta["model_center_m"], 2.0)
        self.assertAlmostEqual(meta["scale"], 0.5)
        self.assertAlmostEqual(float(anchored[452, 613]), 1.0)
        self.assertAlmostEqual(float(anchored[0, 0]), 1.0)

    def test_ultrasonic_anchor_rejects_invalid_stale_noisy_or_extreme(self) -> None:
        depth = np.ones((100, 100), dtype=np.float32)
        valid = {"valid": True, "distance_mm": 1000, "sample_age_us": 10_000}
        for reading in (
            {**valid, "valid": False},
            {**valid, "sample_age_us": 1_000_001},
            {**valid, "distance_mm": 10},
            {**valid, "distance_mm": 4001},
        ):
            with self.assertRaises(ValueError):
                anchor_depth_with_ultrasonic(depth, reading, cx_px=50, cy_px=50)
        noisy = depth.copy()
        noisy[38:63, 38:63] = np.linspace(.2, 2.0, 25 * 25).reshape(25, 25)
        with self.assertRaises(ValueError):
            anchor_depth_with_ultrasonic(noisy, valid, cx_px=50, cy_px=50)
        shallow = np.full((100, 100), 0.5, dtype=np.float32)
        with self.assertRaises(ValueError):
            anchor_depth_with_ultrasonic(shallow, {**valid, "distance_mm": 3000}, cx_px=50, cy_px=50)

    def test_loads_accepted_fisheye_calibration(self) -> None:
        calibration = load_provisional_calibration()
        self.assertFalse(calibration["provisional"])
        self.assertTrue(calibration["calibrated"])
        self.assertEqual((calibration["image_width_px"], calibration["image_height_px"]), (800, 800))
        self.assertEqual(calibration["projection_model"], "opencv_fisheye")
        self.assertEqual(len(calibration["D"]), 4)
        self.assertAlmostEqual(calibration["fx_px"], 638.0752908, places=3)
        self.assertAlmostEqual(calibration["fy_px"], 636.3654525, places=3)
        self.assertAlmostEqual(calibration["cx_px"], 613.4594473, places=3)
        self.assertAlmostEqual(calibration["cy_px"], 452.4711643, places=3)
    def test_stable_cloud_rejects_moving_depth_sequence(self) -> None:
        maps = [np.full((4, 4), 1.0 + index * 0.15, dtype=np.float32) for index in range(STABLE_FRAME_COUNT)]
        self.assertIsNone(stable_cloud_average(maps))


if __name__ == "__main__":
    unittest.main(verbosity=2)
