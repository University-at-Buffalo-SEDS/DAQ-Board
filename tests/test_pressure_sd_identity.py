"""Real SDK wire identity -> actual DAQ callback -> atomic SD handoff.

Each producer/receiver is a separate process: the std SDK's process-global wire
name cache would otherwise hide the compact aliases used on embedded builds.
Outputs/storage/ThreadX are inert bounded native fixtures, not hardware proof.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from native_sdk import prepare_native_sdk
from test_pressure_sd_handoff import function

ROOT = Path(__file__).resolve().parents[1]

class PressureSdIdentityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='pressure-sdk-identity-')
        cls.path = Path(cls.tmp.name)
        code = (ROOT/'tests/native/pressure_sd_identity.in.c').read_text()
        sd = (ROOT/'Core/Src/sd_card.c').read_text()
        telemetry = (ROOT/'Core/Src/telemetry.c').read_text()
        for name, source in [('sd_pressure_capture_route', sd),
                             ('sd_card_log_pressure_batch', sd),
                             ('pressure_sd_receive', telemetry)]:
            marker = '/* SOURCE_FUNCTION:' + name + ' */'
            assert code.count(marker) == 1, marker
            code = code.replace(marker, function(source, name))
        source = cls.path/'identity.c'
        source.write_text(code)
        cls.exe = cls.path/'identity'
        command = ['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                   '-fsanitize=address,undefined', '-I'+str(ROOT/'Core/Inc'),
                   '-I'+str(Path(cls.sdk['header']).parent), str(source), cls.sdk['archive'],
                   '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        command += ['-o', str(cls.exe)]
        subprocess.run(command, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, mode, wire):
        result = subprocess.run([str(self.exe), str(ROOT/'config/sedsnet.json'), mode, str(wire)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        return result.stdout

    def test_ambiguous_compact_collision_is_not_an_authorized_pressure_source(self):
        log = self.run_case('collision', self.path/'collision.bin')
        self.assertIn('ambiguous compact collision rejected', log)

    def test_real_receipt_waits_for_atomic_sd_admission_and_exact_retry(self):
        log = self.run_case('receipt', self.path/'receipt.bin')
        self.assertIn('exact packet retry succeeds after drain', log)

    def test_discovered_compact_source_reaches_atomic_sd_handoff(self):
        wire = self.path/'compact.bin'
        self.run_case('produce', wire)
        log = self.run_case('resolved', wire)
        self.assertIn('SDKresolveVB=0', log)
        self.assertIn('result=0 handoffs=2', log)

    def test_discovered_assigned_source_reaches_atomic_sd_handoff(self):
        wire = self.path/'assigned.bin'
        self.run_case('produce-assigned', wire)
        log = self.run_case('resolved', wire)
        self.assertIn('SDKresolveVB=0', log)
        self.assertIn('result=0 handoffs=2', log)

    def test_unknown_compact_source_cannot_claim_valveboard_identity(self):
        wire = self.path/'unknown-compact.bin'
        self.run_case('produce', wire)
        self.assertIn('last=-9', self.run_case('unknown', wire))

    def test_unknown_assigned_source_cannot_claim_valveboard_identity(self):
        wire = self.path/'unknown-assigned.bin'
        self.run_case('produce-assigned', wire)
        self.assertIn('last=-9', self.run_case('unknown', wire))

if __name__ == '__main__':
    unittest.main()
