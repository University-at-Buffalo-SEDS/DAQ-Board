"""Check live-buffer decoding and plot values without needing a DAQ attached."""
import importlib.util
from pathlib import Path
import struct
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('loadcell_plot', ROOT / 'tools/loadcell_plot.py')
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


class LoadcellPlotTests(unittest.TestCase):
    def test_existing_capture_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'capture.csv'
            path.write_text('existing recording\n')
            with self.assertRaises(FileExistsError):
                plot.Capture(path)
            self.assertEqual(path.read_text(), 'existing recording\n')

    def test_channel_tags_sign_extension_and_duplicate_filtering(self):
        ring = bytearray(plot.DEPTH * 8)
        struct.pack_into('<II', ring, 0, 0x10200000, 998)
        struct.pack_into('<II', ring, 8, 0x0FFFFFFF, 999)
        struct.pack_into('<II', ring, 16, 0x81234567, 999)
        seen = {}
        points = plot.decode_ring(ring, 3, 3, 1000, seen)
        self.assertEqual(points, [(2, 998, 1, 2097152), (1, 999, 0, -1)])
        self.assertEqual(plot.decode_ring(ring, 3, 3, 1001, seen), [])

    def test_inflight_slots_are_excluded_across_queue_wrap(self):
        ring = bytearray(plot.DEPTH * 8)
        for slot in (126, 127, 0, 1, 2):
            struct.pack_into('<II', ring, slot * 8, 0x10200000, 100)
        points = plot.decode_ring(ring, 126, 1, 101, {})
        self.assertEqual(points, [(1, 100, 1, 2097152)])

    def test_tick_wrap_and_stale_entries(self):
        ring = bytearray(plot.DEPTH * 8)
        struct.pack_into('<II', ring, 0, 0x10200000, 0xFFFFFFFE)
        struct.pack_into('<II', ring, 8, 0x10200000, 0xFFFFF000)
        self.assertEqual(plot.decode_ring(ring, 2, 2, 3, {}),
                         [(5, 0xFFFFFFFE, 1, 2097152)])

    def test_voltage_and_independent_calibration(self):
        calibration = [2., 3., 1., 0., 4., 5., 6., 0., 0., 0.01, 7.]
        volts, raw, kg0 = plot.convert(2097152, 0, calibration)
        self.assertEqual(volts, 0.6)
        self.assertAlmostEqual(raw, 0.0345375, places=8)
        _, raw1, kg1 = plot.convert(2097152, 1, calibration)
        self.assertEqual(raw, raw1)
        self.assertEqual(kg0, 2 * raw + 3)
        x = raw - .01
        self.assertAlmostEqual(kg1, 4 + 5*x + 6*x*x - 7)

    def test_capture_csv_and_incremental_browser_response(self):
        with tempfile.TemporaryDirectory() as folder:
            capture = plot.Capture(Path(folder) / 'capture.csv')
            try:
                capture.add([(1, 100, 1, 2097152), (0, 101, 0, 1000)],
                            [1., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0.], [50, 50])
                snapshot = capture.snapshot(1)
                self.assertEqual(len(snapshot['rows']), 1)
                self.assertEqual(snapshot['rows'][0][3], 0)
                self.assertEqual(snapshot['sequence'], 2)
                self.assertEqual(snapshot['published'], [50, 50])
                self.assertEqual(len(capture.filename.read_text().splitlines()), 3)
            finally:
                capture.file.close()

    def test_refuses_non_arm_elf(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'wrong.elf'
            path.write_bytes(b'not firmware')
            with self.assertRaisesRegex(ValueError, 'ELF32'):
                plot.load_elf(path)
