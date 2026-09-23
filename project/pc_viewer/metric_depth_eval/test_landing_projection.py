"""Geometry tests for the provisional top-down lander overlay."""

import unittest

import numpy as np

from landing_projection import project_lander_plan


class LandingProjectionTests(unittest.TestCase):
    def test_centered_downward_camera_projects_symmetric_corner_platform(self):
        plan = project_lander_plan(
            center_ground_depth_m=2.0,
            image_shape=(800, 800),
            fx_px_at_800=800.0,
            body_size_m=0.30,
            nominal_leg_length_m=0.35,
        )

        corners = plan["platform_corners_px"]
        self.assertEqual(plan["assumption"], "camera_centered_downward_provisional")
        self.assertTrue(np.allclose(corners[0], [340.0, 340.0]))  # front-left
        self.assertTrue(np.allclose(corners[1], [460.0, 340.0]))  # front-right
        self.assertTrue(np.allclose(corners[2], [460.0, 460.0]))  # rear-right
        self.assertTrue(np.allclose(corners[3], [340.0, 460.0]))  # rear-left
        self.assertTrue(np.allclose(plan["image_center_px"], [400.0, 400.0]))

    def test_feet_are_farther_from_image_center_than_their_mount_corners(self):
        plan = project_lander_plan(
            center_ground_depth_m=2.0,
            image_shape=(800, 800),
            fx_px_at_800=800.0,
            body_size_m=0.30,
            nominal_leg_length_m=0.35,
        )
        center = plan["image_center_px"]
        for mount, foot in zip(plan["platform_corners_px"], plan["foot_points_px"]):
            self.assertGreater(np.linalg.norm(foot - center), np.linalg.norm(mount - center))

    def test_invalid_depth_returns_no_plan_instead_of_fake_projection(self):
        self.assertIsNone(project_lander_plan(0.0, (800, 800), 800.0))
        self.assertIsNone(project_lander_plan(float("nan"), (800, 800), 800.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
