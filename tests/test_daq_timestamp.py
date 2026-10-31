import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class DaqTimestampTests(unittest.TestCase):
    def test_local_sync_and_outage_transitions(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = str(pathlib.Path(directory) / "timestamps")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(ROOT / "Core/Inc"),
                            str(ROOT / "tests/test_daq_timestamp.c"),
                            "-o", binary], check=True)
            subprocess.run([binary], check=True)

    def test_production_rows_use_fallback_and_identify_clock(self):
        source = (ROOT / "Core/Src/sd_card.c").read_text()
        self.assertIn("sd_format_csv_raw_record(line, sizeof(line),", source)
        self.assertIn("sample->network_unix_ms, captured_uptime, sensor", source)
        raw_formatter = (ROOT / "Core/Inc/sd_raw_record_format.h").read_text()
        self.assertIn("network_ms ? network_ms : uptime", raw_formatter)
        self.assertIn('network_ms ? "network" : "local"', raw_formatter)
        self.assertIn("sd_format_csv_record", source)
        header = (ROOT / "Core/Inc/sd_record_format.h").read_text()
        self.assertIn("calibrated_value,time_source", header)
        self.assertIn("uptime_ticks,uptime_frequency_hz", header)
        acquisition = (ROOT / "Core/Src/daq_thread.c").read_text()
        self.assertIn("daq_sample_network_ms(unix_now, mono_now, captured_uptime)", acquisition)
