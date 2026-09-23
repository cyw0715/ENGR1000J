"""V2 P4 transport safety-contract tests. No hardware access."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent.parent
SR04_H = ROOT / "wifi_test" / "main" / "mega_sr04.h"
SR04_C = ROOT / "wifi_test" / "main" / "mega_sr04.c"
MAIN_C = ROOT / "wifi_test" / "main" / "main.c"


class P4MegaImuTransportContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = SR04_H.read_text(encoding="utf-8")
        cls.source = SR04_C.read_text(encoding="utf-8")
        cls.main = MAIN_C.read_text(encoding="utf-8")

    def test_motion_gate_enabled_for_user_authorized_commissioning(self):
        self.assertIn("#define MEGA_IMU_CONTROL_ENABLED 1", self.header)
        self.assertIn("MEGA_IMU_CONTROL_ENABLED != 0", self.source)

    def test_stop_and_estop_bypass_motion_gate(self):
        self.assertIn('imu_send_command("STOP\\n", true)', self.source)
        self.assertIn('imu_send_command("ESTOP\\n", true)', self.source)
        self.assertIn('imu_send_command("ARM IMU\\n", false)', self.source)

    def test_one_serialized_tx_path(self):
        self.assertIn("s_tx_mutex", self.source)
        self.assertIn("mega_write_line", self.source)
        for command in ("ARM IMU", "STOP", "ESTOP", "HEARTBEAT", "SET_DELTA"):
            self.assertIn(command, self.source)

    def test_status_poll_also_requests_nonblocking_ultrasonic_sample(self):
        self.assertIn('"STATUS\\nGET_DISTANCE\\n"', self.source)
        self.assertIn('strncmp(s_rx_line, "SR04 ", 5)', self.source)
        self.assertIn("parse_sr04_line", self.source)

    def test_leg_status_cannot_refresh_ultrasonic_freshness(self):
        lander_block = self.source[
            self.source.index("static void parse_lander_status_line"):
            self.source.index("static void parse_imu_status_line")
        ]
        imu_block = self.source[
            self.source.index("static void parse_imu_status_line"):
            self.source.index("static bool mega_rx_callback")
        ]
        for block in (lander_block, imu_block):
            self.assertNotIn("s_reading.timestamp_us =", block)
            self.assertNotIn("s_reading.valid =", block)
            self.assertNotIn("s_reading.distance_mm =", block)
        sr04_block = self.source[
            self.source.index("static void parse_sr04_line"):
            self.source.index("static void parse_lander_status_line")
        ]
        self.assertIn("s_reading.timestamp_us =", sr04_block)
        self.assertIn("s_reading.valid =", sr04_block)

    def test_production_open_resets_mega_before_polling(self):
        open_block = self.source[
            self.source.index("ch34x_vcp_open"):
            self.source.index("if (s_solver_target_active)")
        ]
        self.assertIn("set_control_line_state(s_mega, false, true)", open_block)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(100))", open_block)
        self.assertIn("set_control_line_state(s_mega, true, true)", open_block)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(6500))", open_block)
        self.assertLess(
            open_block.index("set_control_line_state(s_mega, false, true)"),
            open_block.index("vTaskDelay(pdMS_TO_TICKS(6500))"),
        )

    def test_health_exposes_rx_diagnostics(self):
        self.assertIn("rx_byte_count", self.header)
        self.assertIn("rx_line_count", self.header)
        self.assertIn("s_reading.rx_byte_count", self.source)
        self.assertIn("s_reading.rx_line_count", self.source)
        self.assertIn('\\"rx_byte_count\\"', self.main)
        self.assertIn('\\"rx_line_count\\"', self.main)

    def test_faithful_remote_protocol_uses_one_serialized_tx_path(self):
        self.assertIn('return imu_send_command("ARM REMOTE\\n", false);', self.source)
        self.assertIn('"SET_REMOTE %lu %u %u %u %u\\n"', self.source)
        self.assertIn("values[i] > 400", self.source)
        self.assertNotIn("values[i] < 50 || values[i] > 400", self.source)
        self.assertIn("xSemaphoreTake(s_tx_mutex", self.source)
        self.assertIn('strstr(buf, "\\\"remote_arm\\\"")', self.main)
        self.assertIn('strstr(buf, "\\\"remote_set_absolute\\\"")', self.main)

    def test_remote_status_fields_are_exposed(self):
        for token in ("remote_active", "remote_source", "remote_preset",
                      "remote_target_fl", "ps2_connected", "remote_submode"):
            self.assertIn(token, self.main)
        self.assertIn('strstr(line, " remote_active=")', self.source)

    def test_complete_v2_status_parse(self):
        self.assertIn("MEGA_SR04_RX_LINE_MAX   512", self.source)
        self.assertIn('strncmp(s_rx_line, "IMU_STATUS ", 11)', self.source)
        self.assertIn("if (fields != 33)", self.source)
        self.assertIn("imu_parse_error", self.source)

    def test_delta_range_is_consistent(self):
        self.assertIn("fl < -350 || fl > 350", self.source)
        self.assertIn("[-350, +350]", self.header)

    def test_health_exposes_state_and_local_pwm_stays_separate(self):
        self.assertIn('\\"mega_imu\\"', self.main)
        self.assertIn("mega_imu_get_state", self.main)
        self.assertIn("ACTUATOR_MOTION_ENABLED", self.main)
        self.assertIn("mega_imu_set_delta", self.main)


if __name__ == "__main__":
    unittest.main(verbosity=2)
