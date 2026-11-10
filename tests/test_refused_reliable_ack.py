"""A delivered reliable packet the DAQ refuses must still be acknowledged.

The SDK withholds the end-to-end ACK (and forgets the packet) when a receive
callback fails, so the sender retransmits a deterministic refusal for its
whole retry budget. The actual DAQ callbacks run under the real native SDK;
refused packets must be ACKed, counted and never applied, while transient
failures (busy writer, storage write) keep failing so the sender retries.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk
from test_pressure_sd_handoff import function

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    ('sd_retry_receive', 'Core/Src/telemetry.c'),
    ('calibration_valid', 'Core/Src/daq_calibration.c'),
    ('daq_calibration_current', 'Core/Src/daq_calibration.c'),
    ('refuse_calibration', 'Core/Src/daq_calibration.c'),
    ('apply_calibration', 'Core/Src/daq_calibration.c'),
    ('read_u64', 'Core/Src/daq_log_clock.c'),
    ('clock_update', 'Core/Src/daq_log_clock.c'),
    ('clear_managed_cache', 'Core/Src/flight_state_cache.c'),
    ('persist_pending', 'Core/Src/flight_state_cache.c'),
    ('persist_update', 'Core/Src/flight_state_cache.c'),
    ('authorized_primary', 'Core/Src/board_recovery_transport.c'),
    ('board_recovery_handle', 'Core/Src/board_recovery_transport.c'),
]


def definition(source, name):
    # Skip forward declarations: take the occurrence followed by a body.
    marker = name + '('
    at = 0
    while True:
        at = source.index(marker, at)
        end = source.index(')', at)
        if source[end + 1:].lstrip().startswith('{'):
            return function(source[source.rfind('\n', 0, at) + 1:], name)
        at = end


class RefusedReliableAckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='refused-reliable-ack-')
        path = Path(cls.tmp.name)
        code = (ROOT / 'tests/native/refused_reliable_ack.in.c').read_text()
        for name, file in SOURCES:
            marker = '/* SOURCE_FUNCTION:' + name + ' */'
            assert code.count(marker) == 1, marker
            code = code.replace(marker, definition((ROOT / file).read_text(), name))
        source = path / 'refused.c'
        source.write_text(code)
        cls.exe = path / 'refused'
        command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                   '-fsanitize=address,undefined', '-DTELEMETRY_ENABLED=1', '-I' + str(ROOT / 'Core/Inc'),
                   '-I' + str(Path(cls.sdk['header']).parent), str(source), cls.sdk['archive'],
                   '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        command += ['-o', str(cls.exe)]
        subprocess.run(command, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, mode):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), mode],
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS', result.stdout)

    def test_refused_sd_retry_request_is_acknowledged(self):
        self.run_case('sd_retry')

    def test_refused_calibration_is_acknowledged_and_not_cached(self):
        self.run_case('calibration')

    def test_refused_log_clock_is_acknowledged_and_not_cached(self):
        self.run_case('clock')

    def test_refused_flight_state_is_acknowledged_and_not_cached(self):
        self.run_case('fstate')

    def test_refused_recovery_request_is_acknowledged(self):
        self.run_case('recovery')


if __name__ == '__main__':
    unittest.main()
