"""Actual compact optional owners: bounded reserve, exact CSV and immutable routing."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest
from sd_cohort_fixture import ordinary_owner_fixture
from test_sd_raw_queue import function
ROOT = Path(__file__).resolve().parents[1]
class OptionalBatchTests(unittest.TestCase):
    def test_real_owners_keep_exact_rows_and_required_reserves(self):
        source = (ROOT / 'Core/Src/sd_card.c').read_text()
        code = ordinary_owner_fixture((ROOT / 'tests/sd_optional_batch_fixture.c.in').read_text(),source)
        for name in re.findall(r'/\* SOURCE_FUNCTION:(\w+) \*/', code):
            code = code.replace('/* SOURCE_FUNCTION:' + name + ' */', function(source, name))
        code = code.replace('/* SOURCE_TYPES */', source[source.index('typedef struct'):source.index('static TX_QUEUE')])
        start = source.index('static const char *const g_sd_scalar_sensors')
        code = code.replace('/* SOURCE_SENSORS */', source[start:source.index('/* The receive callback', start)])
        self.assertNotIn('/* SOURCE_', code)
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'owners'
            compiled = subprocess.run(['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror',
                '-Wno-unused-function', '-Wno-misleading-indentation', '-fsanitize=address,undefined',
                '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(exe)],
                input=code, text=True, capture_output=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            subprocess.run([str(exe)], check=True)
