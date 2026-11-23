"""SD logging throughput: offered load, 400 ms card stalls, unchanged file bytes.

tests/sd_throughput/harness.c runs the production writer (sd_card.c), the
production IDMA adapter (sd_storage_io.c) and the fetched FileX on the host
against a virtual-time card. The card's per-command program time is fitted to
the bench (6 ms reproduces ~1.7k records/s with the former 4 KiB writer), so the
target is also checked across a 2-15 ms range. This is a model of command
counts, sizes and waits, not a cycle-accurate or physical-card qualification.
"""
import hashlib
import os
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
HARNESS = ROOT / 'tests/sd_throughput/harness.c'
# Raw records/s offered by the 1 kHz frame schedule: two fast channels and
# one slot item per frame, plus the launch-window supply pair every 4 ms
# (DAQ_ADC_READ_RATE_HZ). The former schedule offered 2410/s on the bench.
OFFERED = 3500
MARGIN = OFFERED * 3 // 2  # 1.5x target

# Output of the pre-change writer (4 KiB unaligned batches, commit 1e04ec7) for
# the same records. Raw-only: whole file. Mixed: raw/header lines and loadcell
# lines each in their own order (the interleaving of the two streams follows
# writer scheduling and is not part of the format).
GOLDEN_RAW_ONLY = ('d62be12dadbb36fd3fdcadbf830f852a9ac0b891d356a82294b9f8ac8b0022a1', 5488013)
GOLDEN_MIXED = ('27f5cf5727ec7835ce3135985ab3c6bedb613ba791ded2d3cf1315b997b68810',
                'b626dcd49fca6257f3b31994b2d3e11142906a032283774f92fb2ef94ffcb164')
# Those goldens predate the <sensor>_<unit> names and the calibration line's
# loadcell_input/loadcell_boxcar_ms fields (loadcell_input left them stale).
# Undoing exactly those edits must give the old bytes, so the writer is
# otherwise unchanged. The warm-up row that opens the file was kg1000_network.
LEGACY_NAMES = ((b',kg1000_v,', b',mcp3564r_raw,'), (b',kg50_v,', b',kg50_raw,'),
                (b',kg50_p6_v,', b',mcp3564r_ch2_ch3_gain16_raw,'),
                (b',chamber_pressure_v,', b',chamber_pressure_raw_volts_psi,'),
                (b',mcp3564r_ch2_v,', b',mcp3564r_ch2_raw,'), (b',mcp3564r_ch3_v,', b',mcp3564r_ch3_raw,'),
                (b',valve_pressure_psi,', b',kg1000_network,'),
                (b',loadcell_input=adc_volts,loadcell_report=median_bin,kg1000_median=3,kg1000_bin_us=20000,'
                 b'kg50_median=5,kg50_bin_us=50000,adc_uptime_hz=1000000\r\n', b'\r\n'))


def legacy_bytes(data):
    for new, old in LEGACY_NAMES:
        data = data.replace(new, old)
    # MCP3564R rows now carry microsecond uptime_ticks with
    # uptime_frequency_hz=1000000; the old rows repeated monotonic_ms at 1000.
    # Raw rows also leave the temperature columns empty while unchanged.
    lines = data.split(b'\n')
    temperature = None
    for i, line in enumerate(lines):
        fields = line.split(b',')
        if len(fields) != 13 or b'"' in line:
            continue
        if fields[11] == b'1000000':
            fields[10], fields[11] = fields[1], b'1000'
        if fields[8] or fields[9]:
            temperature = fields[8:10]
        elif temperature is not None:
            fields[8:10] = temperature
        lines[i] = b','.join(fields)
    return b'\n'.join(lines)


def filex_source():
    explicit = os.environ.get('SDK_FREE_FILEX_SOURCE')
    if explicit:
        return pathlib.Path(explicit)
    candidates = sorted((ROOT / 'build').glob('*/_deps/filex-src/common/src/fx_file_write.c'))
    return candidates[0].parents[2] if candidates else None


class SdThroughputTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fx = filex_source()
        if fx is None:
            raise unittest.SkipTest('Configure a firmware build to fetch the pinned FileX source')
        cls.tmp = tempfile.TemporaryDirectory(prefix='daq-sd-throughput-')
        out = pathlib.Path(cls.tmp.name)
        cls.includes = ['-I', str(ROOT / 'tests/sd_throughput/stub'), '-I', str(ROOT / 'Core/Inc'),
                        '-I', str(ROOT / 'Core/Src'), '-I', str(fx / 'common/inc'),
                        '-I', str(fx / 'ports/generic/inc'),
                        '-DFX_STANDALONE_ENABLE', '-DFX_INCLUDE_USER_DEFINE_FILE']
        sources = sorted((fx / 'common/src').glob('*.c'))
        objects = []
        # At most four compilers at once: the host is shared with simulators.
        for start in range(0, len(sources), 4):
            batch = []
            for src in sources[start:start + 4]:
                obj = out / (src.stem + '.o')
                objects.append(obj)
                batch.append(subprocess.Popen(['cc', '-O1', '-w', *cls.includes, '-c', str(src),
                                               '-o', str(obj)]))
            for proc in batch:
                if proc.wait() != 0:
                    raise RuntimeError('FileX host build failed')
        cls.filex = out / 'libfx.a'
        subprocess.run(['ar', 'rcs', str(cls.filex), *map(str, objects)], check=True)
        cls.builds = 0

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_model(self, dump=None, **defines):
        type(self).builds += 1
        binary = pathlib.Path(self.tmp.name) / f'model-{self.builds}'
        flags = [f'-D{key}={value}U' for key, value in defines.items()]
        subprocess.run(['cc', '-std=gnu11', '-O1', '-Wall', '-Wno-unused-function', '-Werror',
                        *self.includes, *flags, str(HARNESS), str(self.filex), '-o', str(binary)],
                       check=True)
        env = dict(os.environ)
        if dump:
            env['SD_THROUGHPUT_DUMP'] = str(dump)
        text = subprocess.run([str(binary)], check=True, capture_output=True, text=True, env=env).stdout
        result = {}
        for token in text.split():
            key, _, value = token.partition('=')
            try:
                result[key] = float(value) if '.' in value else int(value)
            except ValueError:
                result[key] = value
        result['text'] = text
        return result

    def assert_no_loss(self, result):
        text = result['text']
        self.assertEqual(result['drained'], 1, text)
        self.assertEqual(result['raw_dropped'], 0, text)
        # Only optional 10 Hz snapshot rows may be shed during a 400 ms card
        # stall, a whole snapshot at a time; required rows never.
        self.assertEqual(result['essential_telemetry_dropped'], 0, text)
        self.assertEqual(result['line_dropped'] % 21, 0, text)
        self.assertLessEqual(result['line_dropped'], result['offered_scalar'] // 10, text)
        self.assertEqual(result['accepted_raw'], result['offered_raw'], text)
        self.assertEqual(result['write_errors'], 0, text)
        self.assertEqual(result['verify'], 0, text)
        self.assertEqual(result['file_bad'], 0, text)
        self.assertEqual(result['file_raw_rows'], result['accepted_raw'], text)

    def test_offered_rate_rides_out_400ms_card_stalls(self):
        result = self.run_model(OFFER_RAW_HZ=OFFERED, STALL_MS=400, STALL_PERIOD_MS=4000)
        self.assert_no_loss(result)
        self.assertGreaterEqual(result['stalls'], 4, result['text'])
        self.assertGreaterEqual(result['max_write_ms'], 400, result['text'])
        # Whole sectors leave in large in-place commands, not 4 KiB (8-sector)
        # pieces. 15.9 with the 10 Hz snapshot bursts; 16+ with the former
        # steady 500 rows/s of load-cell copies.
        self.assertGreaterEqual(result['sectors_per_write'], 14.0, result['text'])

    def test_one_and_a_half_times_offered_rate_across_card_latencies(self):
        # 1.5x the offered rate up to 10 ms per card command. The slowest
        # modelled card (15 ms per command) keeps 1.3x: the 1 kHz schedule
        # offers 45 % more records than the former 2410/s.
        for latency_us, rate in ((2000, MARGIN), (6000, MARGIN), (10000, MARGIN), (15000, OFFERED * 13 // 10)):
            with self.subTest(card_command_ms=latency_us / 1000):
                result = self.run_model(OFFER_RAW_HZ=rate, STALL_MS=400, STALL_PERIOD_MS=4000,
                                        PROG_FIXED_US=latency_us)
                self.assert_no_loss(result)

    def test_sustained_capacity_exceeds_margin(self):
        result = self.run_model(OFFER_RAW_HZ=12000, STALL_MS=0)
        print(f"SD model capacity at 6 ms/command: {result['written_raw_per_s']:.0f} raw records/s")
        self.assertGreaterEqual(result['written_raw_per_s'], MARGIN, result['text'])

    def test_file_bytes_match_pre_change_writer(self):
        out = pathlib.Path(self.tmp.name)
        raw = self.run_model(dump=out / 'raw.csv', OFFER_SCALAR_HZ=0, STALL_MS=0, PROG_FIXED_US=2000)
        self.assert_no_loss(raw)
        data = (out / 'raw.csv').read_bytes()
        self.assertNotIn(b'network,', data.replace(b',network,', b','))
        data = legacy_bytes(data)
        self.assertEqual((hashlib.sha256(data).hexdigest(), len(data)), GOLDEN_RAW_ONLY)
        mixed = self.run_model(dump=out / 'mixed.csv', STALL_MS=400, PROG_FIXED_US=2000)
        self.assert_no_loss(mixed)
        lines = (out / 'mixed.csv').read_bytes().split(b'\n')
        # Raw/header lines keep their exact old bytes; the scalar stream is now
        # the 10 Hz optional snapshot instead of 500 rows/s of load-cell copies.
        other = legacy_bytes(b'\n'.join(line for line in lines if b',"' not in line))
        self.assertEqual(hashlib.sha256(other).hexdigest(), GOLDEN_MIXED[0])
        scalar = [line for line in lines if b',"' in line]
        self.assertFalse([line for line in scalar if b'_network' in line or b'kg1000_v' in line])
        # kg1000_window_input_v duplicated kg1000_window_v once load cells
        # reported volts and is no longer written; nothing to undo above, since
        # the golden hash covers only the raw/header lines.
        self.assertFalse([line for line in scalar if b'kg1000_window_input_v' in line])
        self.assertTrue([line for line in scalar if b',kg1000_window_v,' in line])
        self.assertEqual(len(scalar), mixed['offered_scalar'] - mixed['line_dropped'] + 1)  # + warm-up row


if __name__ == '__main__':
    unittest.main()
