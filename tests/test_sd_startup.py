"""Keep slow card negotiation outside the pre-scheduler startup path."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SdStartupTests(unittest.TestCase):
    def test_successful_logging_does_not_publish_a_warning(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        success = source.split("g_sd_ready = 1U;", 1)[1].split(
            "if (g_sd_ready == 0U)", 1)[0]
        self.assertNotIn("log_telemetry_string_asynchronous", success)
        self.assertNotIn("DAQ SD card is available; logging started", source)
        self.assertIn("DAQ SD card unavailable; acquisition continues without logging", source)

    def test_provisioning_sets_directory_label_before_marker(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        provision = source.split("static UINT sd_format_and_mark(void)", 1)[1].split(
            "static UINT sd_mount_or_provision(void)", 1)[0]
        self.assertLess(provision.index("fx_media_volume_set"),
                        provision.index("fx_file_create"))
        self.assertIn("SEDS_DAQ", provision)
        mounted = source.split("static UINT sd_mount_or_provision(void)", 1)[1].split(
            "static UINT sd_flush_pending", 1)[0]
        marked = mounted.split("sd_marker_exists() == FX_SUCCESS", 1)[1].split(
            "return status;", 1)[0]
        self.assertIn("fx_media_volume_set", marked)
        self.assertNotIn("sd_format_and_mark", marked)

    def test_card_negotiation_is_deferred_to_worker(self):
        main = (ROOT / "Core/Src/main.c").read_text()
        init = main.split("static void MX_SDMMC1_SD_Init(void)\n{", 1)[1].split(
            "/* USER CODE BEGIN SDMMC1_Init 2 */", 1)[0]
        self.assertNotIn("HAL_SD_Init(", init)
        self.assertIn("sd_card_set_hardware_ready(0U)", init)
        worker = (ROOT / "Core/Src/sd_card.c").read_text().split(
            "void sd_card_writer_thread_entry", 1)[1].split(
            "UINT create_sd_writer_thread", 1)[0]
        self.assertIn("HAL_SD_Init(&hsd1)", worker)
