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
OFFERED = 2410            # bench offered raw records/s
MARGIN = OFFERED * 3 // 2  # 1.5x target

# Output of the pre-change writer (4 KiB unaligned batches, commit 1e04ec7) for
# the same records. Raw-only: whole file. Mixed: raw/header lines and loadcell
# lines each in their own order (the interleaving of the two streams follows
# writer scheduling and is not part of the format).
GOLDEN_RAW_ONLY = ('d62be12dadbb36fd3fdcadbf830f852a9ac0b891d356a82294b9f8ac8b0022a1', 5488013)
GOLDEN_MIXED = ('27f5cf5727ec7835ce3135985ab3c6bedb613ba791ded2d3cf1315b997b68810',
                'b626dcd49fca6257f3b31994b2d3e11142906a032283774f92fb2ef94ffcb164')


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
        self.assertEqual(result['line_dropped'], 0, text)
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
        # Whole sectors leave in large in-place commands, not 4 KiB pieces.
        self.assertGreaterEqual(result['sectors_per_write'], 16.0, result['text'])

    def test_one_and_a_half_times_offered_rate_across_card_latencies(self):
        for latency_us in (2000, 6000, 10000, 15000):
            with self.subTest(card_command_ms=latency_us / 1000):
                result = self.run_model(OFFER_RAW_HZ=MARGIN, STALL_MS=400, STALL_PERIOD_MS=4000,
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
        self.assertEqual((hashlib.sha256(data).hexdigest(), len(data)), GOLDEN_RAW_ONLY)
        mixed = self.run_model(dump=out / 'mixed.csv', STALL_MS=400, PROG_FIXED_US=2000)
        self.assert_no_loss(mixed)
        lines = (out / 'mixed.csv').read_bytes().split(b'\n')
        other = b'\n'.join(line for line in lines if b'_network,' not in line)
        network = b'\n'.join(line for line in lines if b'_network,' in line)
        self.assertEqual((hashlib.sha256(other).hexdigest(), hashlib.sha256(network).hexdigest()),
                         GOLDEN_MIXED)


if __name__ == '__main__':
    unittest.main()
