"""Contract tests for P4-independent video and IMU transport.

Video remains TCP 5000. BNO085 samples are independently published on TCP 5001
and must never be emitted from the successful-JPEG path.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT = Path(__file__).resolve().parent
MAIN_C = PROJECT / "main" / "main.c"
PC_VIEWER = PROJECT.parent / "pc_viewer"
sys.path.insert(0, str(PC_VIEWER))
from lander_console import IMU_TCP_PORT, VIDEO_VIEW_SIZE  # noqa: E402


class P4IndependentImuTransportContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = MAIN_C.read_text(encoding="utf-8")

    def test_isolated_from_mega(self) -> None:
        self.assertIn("#define MEGA_SR04_ENABLED 0", self.source)
        self.assertIn("Mega USB HC-SR04 disabled", self.source)

    def test_video_and_imu_use_separate_services(self) -> None:
        self.assertIn("#define PC_PORT        5000", self.source)
        self.assertIn("#define IMU_TCP_PORT   5001", self.source)
        self.assertIn("independent JSONL sample stream", self.source)
        self.assertIn("static void imu_tcp_server_task", self.source)
        self.assertNotIn("FRAME_META_TCP_PORT", self.source)
        self.assertNotIn("send_frame_imu_metadata", self.source)
        self.assertNotIn("frame_to_imu_age_us", self.source)

    def test_imu_stream_reads_driver_not_video_state(self) -> None:
        start = self.source.index("static void imu_tcp_server_task")
        end = self.source.index("static void capture_task")
        snippet = self.source[start:end]
        self.assertIn("esp_err_t sample_ret = imu_read(&sample);", snippet)
        self.assertIn("sample.timestamp_us == last_sent_timestamp_us", snippet)
        self.assertNotIn("s_jpeg_timestamp_us", snippet)
        self.assertNotIn("s_video_frame_id", snippet)

    def test_console_uses_independent_imu_reader_at_large_design_resolution(self) -> None:
        self.assertEqual(IMU_TCP_PORT, 5001)
        self.assertEqual(VIDEO_VIEW_SIZE, (1040, 960))
        viewer = (PC_VIEWER / "lander_console.py").read_text(encoding="utf-8")
        self.assertIn("CV_VIEW_NAMES", viewer)
        self.assertIn("rowconfigure(1, weight=2", viewer)
        self.assertIn("rowconfigure(2, weight=1", viewer)
        self.assertIn('self.geometry("1600x1000")', viewer)
        self.assertIn('self.minsize(1100, 700)', viewer)
        self.assertIn('cv_area.columnconfigure(column, weight=1, uniform="cv")', viewer)
        self.assertIn("class ImuStreamReader", viewer)
        self.assertIn('"imu_stream"', viewer)
        self.assertNotIn("FrameImuReader", viewer)
        self.assertNotIn("FRAME_IMU_META_PORT", viewer)


if __name__ == "__main__":
    unittest.main(verbosity=2)
