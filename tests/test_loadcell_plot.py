"""Check live-buffer decoding and plot values without needing a DAQ attached."""
import importlib.util
from pathlib import Path
import struct
import json
import math
import tempfile
import subprocess
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
        ring = bytearray(plot.DEPTH * plot.ENTRY_SIZE)
        struct.pack_into('<IIfi', ring, 0, 0x10200000, 998, 25.0, 123)
        struct.pack_into('<IIfi', ring, 16, 0x0FFFFFFF, 999, 25.0, 123)
        struct.pack_into('<IIfi', ring, 32, 0x81234567, 999, 25.0, 123)
        seen = {}
        points = plot.decode_ring(ring, 3, 3, 1000, seen)
        self.assertEqual(points, [(2, 998, 1, 2097152, 25.0), (1, 999, 0, -1, 25.0)])
        self.assertEqual(plot.decode_ring(ring, 3, 3, 1001, seen), [])

    def test_inflight_slots_are_excluded_across_queue_wrap(self):
        ring = bytearray(plot.DEPTH * plot.ENTRY_SIZE)
        for slot in (126, 127, 0, 1, 2):
            struct.pack_into('<IIfi', ring, slot * plot.ENTRY_SIZE, 0x10200000, 100, 25.0, 123)
        points = plot.decode_ring(ring, 126, 1, 101, {})
        self.assertEqual(points, [(1, 100, 1, 2097152, 25.0)])

    def test_tick_wrap_and_stale_entries(self):
        ring = bytearray(plot.DEPTH * plot.ENTRY_SIZE)
        struct.pack_into('<IIfi', ring, 0, 0x10200000, 0xFFFFFFFE, 25.0, 123)
        struct.pack_into('<IIfi', ring, 16, 0x10200000, 0xFFFFF000, 25.0, 123)
        self.assertEqual(plot.decode_ring(ring, 2, 2, 3, {}),
                         [(5, 0xFFFFFFFE, 1, 2097152, 25.0)])

    def test_voltage_and_independent_calibration(self):
        calibration = [2., 3., 1., 0., 4., 5., 6., 0., 0., 0.01, 7.]
        volts, raw, kg0 = plot.convert(2097152, 0, calibration)
        self.assertAlmostEqual(volts, 0.6, places=7)
        self.assertAlmostEqual(raw, 0.0345375, places=8)
        _, raw1, kg1 = plot.convert(2097152, 1, calibration)
        self.assertEqual(raw, raw1)
        self.assertAlmostEqual(kg0, 2 * raw + 3, places=6)
        x = raw - .01
        self.assertAlmostEqual(kg1, 4 + 5*x + 6*x*x - 7, places=6)

    def test_capture_csv_and_incremental_browser_response(self):
        with tempfile.TemporaryDirectory() as folder:
            capture = plot.Capture(Path(folder) / 'capture.csv')
            try:
                capture.add([(1, 100, 1, 2097152, 25.0), (0, 101, 0, 1000, math.nan)],
                            [1., 0., 1., 0., 0., 1., 0., 0., 0., 0., 0.], [50, 50])
                snapshot = capture.snapshot(1)
                self.assertEqual(len(snapshot['rows']), 1)
                self.assertEqual(snapshot['rows'][0][3], 0)
                self.assertEqual(snapshot['sequence'], 2)
                self.assertEqual(snapshot['published'], [50, 50])
                self.assertIsNone(snapshot['rows'][0][8])
                json.dumps(snapshot, allow_nan=False)
                self.assertEqual(len(capture.filename.read_text().splitlines()), 3)
            finally:
                capture.file.close()

    def test_refuses_non_arm_elf(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'wrong.elf'
            path.write_bytes(b'not firmware')
            with self.assertRaisesRegex(ValueError, 'ELF32'):
                plot.load_elf(path)

    def test_p6_and_pressure_have_distinct_scales_and_no_invented_mass(self):
        calibration = [1., 0., 1., 0., 99., 100., 0., 0., 0., 0., 0.] + [0.] * 6
        voltage, raw, mass = plot.convert(2097152, 9, calibration)
        self.assertAlmostEqual(voltage, .6 / 16, places=8)
        self.assertEqual(raw, voltage)
        self.assertIsNone(mass)  # Never reuse AMP2 calibration for P6.
        voltage, connector, psi = plot.convert(2097152, 6, calibration)
        self.assertAlmostEqual(connector, voltage * 11, places=6)
        self.assertAlmostEqual(psi, connector * 200, places=3)
        ring = bytearray(plot.DEPTH * plot.ENTRY_SIZE)
        struct.pack_into('<IIfi', ring, 0, 0x90200000, 40, 25., 123)
        struct.pack_into('<IIfi', ring, 16, 0x60200000, 41, 25., 123)
        self.assertEqual([p[2] for p in plot.decode_ring(ring, None, None, 42, {})], [9, 6])

    def test_chunked_double_read_omits_a_changing_entry(self):
        memory = bytearray(plot.DEPTH * plot.ENTRY_SIZE)
        struct.pack_into('<IIfi', memory, 0, 0x90200000, 40, 25., 123)
        struct.pack_into('<IIfi', memory, 16, 0x60200000, 41, 25., 123)
        class Probe:
            reads = 0
            def read(self, address, count):
                self.reads += 1
                offset = address - 0x2000 - plot.RING_OFFSET
                data = bytearray(memory[offset:offset + count * 4])
                if self.reads == 2:
                    data[0] ^= 1
                return bytes(data)
        probe = Probe()
        stable = plot.read_stable_ring(probe, 0x2000)
        self.assertEqual(probe.reads, 16)
        self.assertEqual([p[2] for p in plot.decode_ring(stable, None, None, 42, {})], [6])

    def test_verification_refuses_other_firmware_without_resetting(self):
        sections = [(name, address, bytes([index]) * 4) for index, (name, address) in enumerate(
            [('.isr_vector', 0x08008200), ('.text', 0x08008300), ('.rodata', 0x08008400)])]
        class Probe:
            mismatch = False
            commands = []
            def command(self, command):
                self.commands.append(command)
                return 'running'
            def word(self, address):
                return {0xE000ED00: 0xD210, 0xE000ED08: sections[0][1]}.get(address, 0)
            def read(self, address, count):
                expected = next(data for _, start, data in sections if start == address)
                return bytes([255]) * len(expected) if self.mismatch else expected
        probe = Probe()
        plot.verify_firmware(probe, {'g_mcp3564r_init_status': 0x2000}, sections, lambda _: None)
        probe.mismatch = True
        with self.assertRaisesRegex(RuntimeError, 'differs from the ELF'):
            plot.verify_firmware(probe, {'g_mcp3564r_init_status': 0x2000}, sections, lambda _: None)
        self.assertTrue(all('halt' not in cmd and 'reset' not in cmd for cmd in probe.commands))

    def test_debug_layout_matches_current_production_context(self):
        source = (ROOT / 'Core/Src/mcp3564r.c').read_text()
        declarations = source[source.index('typedef struct\n{\n  uint32_t raw32;'):
                              source.index('static mcp3564r_context_t g_mcp3564r')]
        # A host pointer is64-bit; target SPI pointer is32-bit. All other
        # members are explicit fixed-width types with matching alignment.
        declarations = declarations.replace('SPI_HandleTypeDef *spi;', 'uint32_t spi;')
        code = '#include <stdint.h>\n#include <stddef.h>\n#include <stdio.h>\n#define MCP3564R_SAMPLE_QUEUE_DEPTH 128U\n'
        code += declarations + '\nint main(void){printf("%zu %zu %zu",sizeof(mcp3564r_context_t),offsetof(mcp3564r_context_t,queue),sizeof(mcp3564r_sample_entry_t));}\n'
        with tempfile.TemporaryDirectory() as folder:
            binary = Path(folder) / 'layout'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Werror', '-x', 'c', '-', '-o', str(binary)],
                           input=code, text=True, check=True)
            result = subprocess.check_output([str(binary)], text=True)
        self.assertEqual(tuple(map(int, result.split())),
                         (plot.CONTEXT_SIZE, plot.RING_OFFSET, plot.ENTRY_SIZE))

    def test_sd_health_tracks_progress_stall_reset_and_missing_reads(self):
        state = plot.SDStatus()
        counters = dict.fromkeys(plot.SD_COUNTERS, 0)
        counters.update(ready=1, init_stage=5)
        state.update(counters, 1.)
        self.assertEqual(state.snapshot(1.)['state'], 'READY / IDLE')
        self.assertIsNone(state.snapshot(1.)['last_progress_age_s'])
        counters.update(raw_written=100, csv_written=20, raw_flushed=80)
        state.update(counters, 3.)
        live = state.snapshot(3.)
        self.assertEqual(live['state'], 'WRITING')
        self.assertEqual(live['rate_hz'], 60.)
        self.assertEqual(live['raw_flushed'], 80)
        state.update(counters, 5.)
        idle = state.snapshot(5.)
        self.assertEqual(idle['state'], 'READY / IDLE')
        self.assertEqual(idle['rate_hz'], 0)
        self.assertEqual(idle['last_progress_age_s'], 2.)
        self.assertEqual(state.snapshot(9.)['state'], 'UNKNOWN')
        counters.update(raw_written=0, csv_written=0)
        state.update(counters, 10.)
        self.assertTrue(state.snapshot(10.)['reset_detected'])
        self.assertIsNone(state.snapshot(10.)['rate_hz'])
        state.update({}, 11.)
        self.assertEqual(state.snapshot(11.)['state'], 'UNKNOWN')
        state.unavailable('Read failed')
        self.assertEqual(state.snapshot(12.)['reason'], 'Read failed')
        with self.assertRaisesRegex(ValueError, 'symbols absent'):
            plot.read_sd_counters(None, {})

    def test_sd_counter_wrap_is_progress_not_board_reset(self):
        state = plot.SDStatus()
        counters = dict.fromkeys(plot.SD_COUNTERS, 0)
        counters.update(ready=1, raw_written=0xFFFFFFFE)
        state.update(counters, 1.)
        counters['raw_written'] = 3
        state.update(counters, 2.)
        result = state.snapshot(2.)
        self.assertEqual(result['state'], 'WRITING')
        self.assertEqual(result['rate_hz'], 5)
        self.assertFalse(result['reset_detected'])
