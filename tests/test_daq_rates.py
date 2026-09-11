import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

class DaqRateTests(unittest.TestCase):
    def compile(self, code, defines=(), valid=True):
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / "test"
            cmd = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(ROOT / "Core/Inc")]
            cmd += ["-D" + value for value in defines]
            result = subprocess.run(cmd + ["-x", "c", "-", "-o", str(binary)],
                                    input=code, text=True, capture_output=True)
            if valid:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(binary)], check=True)
            else:
                self.assertNotEqual(result.returncode, 0)

    def test_default_and_fast_settings(self):
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ADC_OSR_BITS == 5); assert(DAQ_ADC_READ_INTERVAL_US == 271); assert(DAQ_BROADCAST_PERIOD_MS == 20);}')
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ADC_OSR_BITS == 3); assert(DAQ_ADC_READ_INTERVAL_US == 84); assert(DAQ_BROADCAST_PERIOD_MS == 100);}',
                     ['DAQ_ADC_OSR=256', 'DAQ_ADC_READ_RATE_HZ=12000', 'DAQ_ACQUISITION_PERIOD_MS=5', 'DAQ_BROADCAST_RATE_HZ=10'])

    def test_unsafe_configurations_fail_compilation(self):
        for defines in [['DAQ_ADC_READ_RATE_HZ=0'], ['DAQ_ADC_READ_RATE_HZ=12000'],
                        ['DAQ_ADC_OSR=256', 'DAQ_ADC_READ_RATE_HZ=12000'],
                        ['DAQ_BROADCAST_RATE_HZ=1000'], ['DAQ_ADC_OSR=123']]:
            with self.subTest(defines=defines):
                self.compile('#include "daq_rates.h"\nint main(void) {}', defines, False)

    def test_weighting_noise_rejection_and_clock_wrap(self):
        self.compile(r'''#include "daq_downsample.h"
#include <assert.h>
int main(void) {
  daq_downsample_t state = {0}; float out = 0;
  assert(!daq_downsample_add(&state, 2, 3, 0, 20, &out));
  assert(daq_downsample_add(&state, 10, 1, 20, 20, &out));
  assert(out == 4); /* Weighted samples, not an average of batch averages. */
  state = (daq_downsample_t){0};
  for (unsigned i=0; i<100; ++i)
    assert(!daq_downsample_add(&state, (i&1) ? 101 : 99, 1, i, 100, &out));
  assert(daq_downsample_add(&state, NAN, 1, 100, 100, &out));
  assert(out == 100 && state.count == 0); /* Noise averaged, NaN excluded. */
  state = (daq_downsample_t){0};
  assert(!daq_downsample_add(&state, 7, 1, UINT32_MAX-9, 20, &out));
  assert(daq_downsample_add(&state, 7, 1, 10, 20, &out));
  assert(out == 7);
}
''')
