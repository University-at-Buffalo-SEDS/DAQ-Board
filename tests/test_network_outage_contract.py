"""Network qualification must not silently disconnect soldered ADCs."""
import json
import unittest
from pathlib import Path


class NetworkOutageContract(unittest.TestCase):
    def test_onboard_adcs_remain_connected(self):
        root = Path(__file__).resolve().parents[1]
        layout = json.loads((root / "sim/board.json").read_text())
        adcs = [p for p in layout["peripherals"] if p["type"] == "adc"]
        self.assertEqual({p["name"] for p in adcs}, {"adc1", "adc4", "loadcell_adc"})
        for adc in adcs:
            with self.subTest(adc=adc["name"]):
                self.assertFalse(adc.get("failure_every", 0))
                self.assertFalse(adc.get("disconnect_after", 0))

    def test_acquisition_and_sd_must_keep_advancing(self):
        root = Path(__file__).resolve().parents[1]
        layout = json.loads((root / "sim/board.json").read_text())
        probes = {p["name"]: p for p in layout["execution"]["memory_probes"]}
        for name in ("loadcell_raw_samples", "sd_raw_records_written"):
            self.assertGreater(probes[name]["minimum_interval_gain"], 0)
