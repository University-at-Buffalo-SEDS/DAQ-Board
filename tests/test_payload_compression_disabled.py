"""Transmit never attempts zstd payload compression on the DAQ Board.

SEDSnet is built with its `compression` feature, so every payload of at least
PAYLOAD_COMPRESS_THRESHOLD (30 B here) went through ZSTD_compress. That puts a
3.8 KiB context on the calling thread's stack and then mallocs its workspace
(at least 18.9 KiB on a 32-bit target) from the newlib heap, which on this
board has 1,864 B free at link time. The allocation always failed, the payload
went out uncompressed anyway, and each attempt cost about 4.5 KiB of stack on
the router's side-TX path.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def _init_function(source: str) -> str:
    match = re.search(r'static SedsResult init_telemetry_router_locked\(void\)\s*\{', source)
    assert match, 'init_telemetry_router_locked not found'
    return source[match.start():source.index('\n}\n', match.start()) + 3]


class PayloadCompressionDisabledTests(unittest.TestCase):
    def test_router_tuning_disables_transmit_compression_before_router_creation(self):
        init = _init_function((ROOT / 'Core/Src/telemetry.c').read_text())
        get = init.find('seds_get_runtime_tuning_config(&tuning)')
        threshold = re.search(r'tuning\.payload_compress_threshold\s*=\s*SIZE_MAX\s*;', init)
        set_ = init.find('seds_set_runtime_tuning_config(&tuning)')
        create = init.find('seds_router_new')
        self.assertIsNotNone(threshold, 'transmit-side payload compression must be disabled')
        self.assertGreaterEqual(get, 0, 'tuning must start from the SDK defaults')
        self.assertTrue(get < threshold.start() < set_ < create,
                        'disable compression in the runtime tuning before the router exists')
        # The get/set results are checked like every other init step.
        self.assertRegex(init[set_:create], r'if \(result != SEDS_OK\)')

    def test_stdint_provides_size_max(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        self.assertIn('#include <stdint.h>', source)


if __name__ == '__main__':
    unittest.main()
