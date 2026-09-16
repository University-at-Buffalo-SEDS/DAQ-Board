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
        self.assertIn("daq_timestamp_ms(sample->network_unix_ms, sample->monotonic_ms)", source)
        self.assertIn("daq_timestamp_ms(network_ms, timestamp_ms)", source)
        self.assertIn("daq_timestamp_ms(network_ms, local_ms)", source)
        self.assertIn("calibrated_value,time_source", source)
        self.assertIn("timestamp_ms,monotonic_ms", source)
        acquisition = (ROOT / "Core/Src/daq_thread.c").read_text()
        self.assertIn("daq_sample_network_ms(unix_now, mono_now, sample.monotonic_ms)", acquisition)
