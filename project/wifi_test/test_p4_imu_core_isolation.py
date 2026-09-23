"""Static regression checks for the P4 IMU cache-safe UART configuration."""
from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).parent
IMU = (ROOT / "main" / "imu.c").read_text(encoding="utf-8")
HOSTED_OS = (ROOT / "managed_components" / "espressif__esp_hosted" / "host" / "port" / "src" / "os_wrapper.c").read_text(encoding="utf-8")


class CoreIsolationRegressionTests(unittest.TestCase):
    def test_uart_driver_and_parser_have_cpu1_isolation(self) -> None:
        self.assertIn("ESP_INTR_FLAG_IRAM", IMU)
        self.assertIn("IRAM ISR + parser CPU1 prio=24, event queue disabled", IMU)
        self.assertIn("24, &uart_task_h, 1", IMU)
        self.assertIn("uart_driver_install(RVC_UART, RVC_RX_BUFFER_SIZE, 0, 0", IMU)

    def test_hosted_keeps_native_unpinned_scheduling(self) -> None:
        # Do not pin all ESP-Hosted tasks: the component owns its transport
        # topology and some SDIO/RPC work can require the scheduler's choice.
        self.assertIn("task_created = xTaskCreate((void (*)(void *))start_routine", HOSTED_OS)
        self.assertNotIn("xTaskCreatePinnedToCore((void (*)(void *))start_routine", HOSTED_OS)


if __name__ == "__main__":
    unittest.main(verbosity=2)
