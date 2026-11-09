"""Every station 50 kg input selection the DAQ receives must be applied.

The station resends its selection while the DAQ reports another input. The
SDK silently drops (and still acknowledges) a managed value whose timestamp is
older than the one it cached from the same sender, so a selection stamped with
an older revision, from another station mode's calibration file or after a
station clock step, was never applied and the station resent it forever.
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
    ('refuse_calibration', 'Core/Src/daq_calibration.c'),
    ('apply_kg50_input', 'Core/Src/daq_calibration.c'),
]


class Kg50InputOrderTests(unittest.TestCase):
    def test_older_stamped_selection_from_the_station_is_applied(self):
        sdk = prepare_native_sdk(ROOT)
        with tempfile.TemporaryDirectory(prefix='kg50-input-order-') as tmp:
            path = Path(tmp)
            code = (ROOT / 'tests/native/kg50_input_order.in.c').read_text()
            for name, file in SOURCES:
                marker = '/* SOURCE_FUNCTION:' + name + ' */'
                assert code.count(marker) == 1, marker
                code = code.replace(marker, definition((ROOT / file).read_text(), name))
            source = path / 'order.c'
            source.write_text(code)
            exe = path / 'order'
            command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                       '-fsanitize=address,undefined', '-DTELEMETRY_ENABLED=1', '-I' + str(ROOT / 'Core/Inc'),
                       '-I' + str(Path(sdk['header']).parent), str(source), sdk['archive'],
                       '-lpthread', '-lm']
            if sys.platform.startswith('linux'):
                command.append('-ldl')
            command += ['-o', str(exe)]
            subprocess.run(command, check=True)
            result = subprocess.run([str(exe), str(ROOT / 'config/sedsnet.json')],
                                    capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('PASS', result.stdout)


if __name__ == '__main__':
    unittest.main()
