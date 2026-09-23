"""Regression checks: integrated IMU parser must retain the proven bytewise RVC loop."""
from __future__ import annotations

import unittest
from pathlib import Path

SOURCE = (Path(__file__).parent / "main" / "imu.c").read_text(encoding="utf-8")


class IntegratedImuParserRegressionTests(unittest.TestCase):
    def test_uses_proven_header_then_payload_sequence(self) -> None:
        self.assertIn("UART-RVC parser shared with the locally hardware-proven imu_test project", SOURCE)
        self.assertIn("uart_read_bytes(RVC_UART, &byte, 1, pdMS_TO_TICKS(1000))", SOURCE)
        self.assertIn("ESP_INTR_FLAG_IRAM", SOURCE)
        self.assertIn('IRAM ISR + parser CPU1 prio=24, event queue disabled', SOURCE)
        self.assertIn('24, &uart_task_h, 1', SOURCE)
        self.assertIn("packet[0] = 0xAA;", SOURCE)
        self.assertIn("packet[1] = 0xAA;", SOURCE)
        self.assertIn("RVC_PACKET_LEN - 2", SOURCE)
        self.assertIn("rvc_checksum_valid(packet)", SOURCE)

    def test_publishes_new_valid_samples_and_health_timestamp(self) -> None:
        self.assertIn("s_latest = sample;", SOURCE)
        self.assertIn("s_health.last_valid_timestamp_us = ts;", SOURCE)
        self.assertIn("s_health.valid_packet_count++;", SOURCE)
        self.assertIn("xQueueOverwrite(s_sample_queue, &sample)", SOURCE)

    def test_removed_unproven_bulk_replay_parser(self) -> None:
        self.assertNotIn("READ_CHUNK_SIZE", SOURCE)
        self.assertNotIn("replay_len", SOURCE)
        self.assertNotIn("FSM_COLLECT", SOURCE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
