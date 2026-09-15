"""Keep slow card negotiation outside the pre-scheduler startup path."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SdStartupTests(unittest.TestCase):
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
