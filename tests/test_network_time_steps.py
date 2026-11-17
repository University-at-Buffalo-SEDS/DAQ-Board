"""Station request freshness across real SEDSnet network time steps.

A real GS router leads network time for a real DAQ router; the GS clock steps
4 h back and 3.8 h forward. The actual board freshness and floor checks judge
the views the SDK delivers (time_epoch filled by the SDK): fresh requests are
admitted at once after either step, late copies from before a step and a
replay at the floor are refused, and a request in a newer epoch is fresh while
the DAQ re-anchors.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk
from test_refused_reliable_ack import definition

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    ('board_recovery_packet_stamp', 'Core/Src/board_recovery_transport.c'),
    ('board_recovery_stamp_fresh', 'Core/Src/board_recovery_transport.c'),
    ('board_recovery_command_is_stale', 'Core/Src/board_recovery_transport.c'),
]


class NetworkTimeStepTests(unittest.TestCase):
    def test_requests_across_4_h_back_and_3_8_h_forward_steps(self):
        sdk = prepare_native_sdk(ROOT)
        with tempfile.TemporaryDirectory(prefix='network-time-steps-') as tmp:
            path = Path(tmp)
            code = (ROOT / 'tests/native/network_time_steps.in.c').read_text()
            for name, file in SOURCES:
                marker = '/* SOURCE_FUNCTION:' + name + ' */'
                assert code.count(marker) == 1, marker
                code = code.replace(marker, definition((ROOT / file).read_text(), name))
            source = path / 'steps.c'
            source.write_text(code)
            exe = path / 'steps'
            command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                       '-fsanitize=address,undefined', '-DTELEMETRY_ENABLED=1', '-I' + str(ROOT / 'Core/Inc'),
                       '-I' + str(Path(sdk['header']).parent), str(source), sdk['archive'],
                       '-lpthread', '-lm']
            if sys.platform.startswith('linux'):
                command.append('-ldl')
            command += ['-o', str(exe)]
            subprocess.run(command, check=True)
            result = subprocess.run([str(exe), str(ROOT / 'config/sedsnet.json')],
                                    capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS', result.stdout)


if __name__ == '__main__':
    unittest.main()
