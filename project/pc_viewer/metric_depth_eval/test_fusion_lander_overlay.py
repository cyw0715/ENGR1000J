"""Integration tests for the lander overlay in the fusion panel."""

import unittest

import numpy as np

import fusion_depth_viewer as viewer


class FusionLanderOverlayTests(unittest.TestCase):
    def test_make_canvas_draws_provisional_lander_plan_on_fusion_panel(self):
        height, width = 80, 80
        gray = np.full((height, width), 128, dtype=np.uint8)
        da = np.full((height, width), 2.0, dtype=np.float32)
        moge = np.full((height, width), 2.0, dtype=np.float32)
        mask = np.ones((height, width), dtype=bool)

        canvas, outputs = viewer.make_canvas(
            gray, da, moge, mask,
            frame_id=1, tcp_fps=3.0, da_ms=10.0, moge_ms=12.0,
            jpeg_bytes=12000, previous=(None, None),
        )

        self.assertEqual(canvas.shape, (viewer.PANEL_H * 2 + 4 + viewer.FOOTER_H, viewer.PANEL_W * 2 + 4, 3))
        self.assertIsNotNone(outputs["lander_plan"])
        self.assertEqual(outputs["lander_plan"]["assumption"], "camera_centered_downward_provisional")
        # The visible overlay adds saturated amber/cyan pixels beyond the heatmap.
        fusion = outputs["fusion_panel"]
        self.assertGreater(np.count_nonzero((fusion[:, :, 0] > 240) & (fusion[:, :, 1] > 180)), 30)


if __name__ == "__main__":
    unittest.main(verbosity=2)
