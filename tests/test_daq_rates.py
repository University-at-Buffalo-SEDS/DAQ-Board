import pathlib
import subprocess
import tempfile
import unittest
from test_network_rate import function

ROOT = pathlib.Path(__file__).resolve().parents[1]

class DaqRateTests(unittest.TestCase):
    def test_can_report_is_uncalibrated(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        live = function(source, 'static void daq_offer_live_report(')
        self.assertIn('report->kg1000', live)
        self.assertNotIn('calibration', live)
        self.assertNotIn('sd_card_', live)
        self.assertIn('records[count].calibrated_value', source)

    def test_sd_uses_default_speed_clock_and_bounded_status_wait(self):
        import re
        main = (ROOT / 'Core/Src/main.c').read_text()
        ioc = (ROOT / 'DAQ-Board.ioc').read_text()
        divider = int(re.search(r'hsd1.Init.ClockDiv = (\d+);', main).group(1))
        clock = int(re.search(r'RCC.SDMMCFreq_Value=(\d+)', ioc).group(1))
        self.assertGreater(divider, 0)
        self.assertLessEqual(clock // (2 * divider), 25_000_000)
        self.assertIn(f'SDMMC1.ClockDiv={divider}', ioc)
        limits = (ROOT / 'Core/Inc/sd_hal_limits.h').read_text()
        self.assertIn('#define SDMMC_SWDATATIMEOUT 2000U', limits)
        self.assertIn('Core/Inc/sd_hal_limits.h', (ROOT / 'CMakeLists.txt').read_text())
        retry = (ROOT / 'Core/Src/sd_card.c').read_text().split('g_sd_retry_count++;', 1)[1]
        self.assertLess(retry.index('HAL_SD_DeInit'), retry.index('HAL_SD_Init'))
        self.assertLess(retry.index('HAL_SD_DeInit'), retry.index('__HAL_RCC_SDMMC1_FORCE_RESET'))
        self.assertLess(retry.index('__HAL_RCC_SDMMC1_FORCE_RESET'), retry.index('__HAL_RCC_SDMMC1_RELEASE_RESET'))
        self.assertLess(retry.index('__HAL_RCC_SDMMC1_RELEASE_RESET'), retry.index('HAL_SD_Init'))

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
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ADC_OSR_BITS == 3); assert(DAQ_ADC_READ_INTERVAL_US == 180); assert(DAQ_BROADCAST_PERIOD_MS == 4); assert(DAQ_ACQUISITION_PERIOD_MS == 2); assert(DAQ_SD_RAW_QUEUE_DEPTH == 401); assert(DAQ_RAW_BATCH_CAPACITY == 16);}')
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ADC_OSR_BITS == 5); assert(DAQ_ADC_READ_INTERVAL_US == 271); assert(DAQ_BROADCAST_PERIOD_MS == 4); assert(DAQ_ACQUISITION_PERIOD_MS == 2); assert(DAQ_SD_RAW_QUEUE_DEPTH == 401); assert(DAQ_RAW_BATCH_CAPACITY == 12);}', ['DAQ_CHAMBER_ADC_CHANNEL=-1'])
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ADC_OSR_BITS == 3); assert(DAQ_ADC_READ_INTERVAL_US == 84); assert(DAQ_BROADCAST_PERIOD_MS == 100);}',
                     ['DAQ_ADC_OSR=256', 'DAQ_ADC_READ_RATE_HZ=12000', 'DAQ_ACQUISITION_PERIOD_MS=5', 'DAQ_BROADCAST_RATE_HZ=10'])

    def test_unsafe_configurations_fail_compilation(self):
        for defines in [['DAQ_ADC_READ_RATE_HZ=0'], ['DAQ_ADC_READ_RATE_HZ=12000', 'DAQ_ADC_OSR=1024'],
                        ['DAQ_SD_RAW_BUFFER_MS=2000'],
                        ['DAQ_BROADCAST_RATE_HZ=1000'], ['DAQ_ADC_OSR=123']]:
            with self.subTest(defines=defines):
                self.compile('#include "daq_rates.h"\nint main(void) {}', defines, False)

    def test_report_phase_survives_scheduler_jitter(self):
        self.compile(r'''#include "daq_downsample.h"
#include <assert.h>
int main(void) {
  daq_downsample_t s = {0}; float out;
  assert(!daq_downsample_add(&s, 1, 1, 0, 20, &out));
  for (unsigned i = 1; i <= 50; ++i)
    assert(daq_downsample_add(&s, 1, 1, i*20 + (i%2), 20, &out));
  assert(daq_downsample_add(&s, 1, 1, 2000, 20, &out));
  assert(!daq_downsample_add(&s, 1, 1, 2001, 20, &out));
}''')

    def test_250_hz_reports_each_second_without_catchup_bursts(self):
        self.compile(r'''#include "daq_downsample.h"
#include "daq_rates.h"
#include <assert.h>
int main(void) {
  daq_downsample_t state = {0}; float out; unsigned reports = 0;
  for (unsigned ms=0; ms<=1000; ms+=DAQ_ACQUISITION_PERIOD_MS)
    reports += daq_downsample_add(&state, 1, 1, ms, DAQ_BROADCAST_PERIOD_MS, &out);
  assert(reports == 250);
  assert(daq_downsample_add(&state, 1, 1, 2000, DAQ_BROADCAST_PERIOD_MS, &out));
  assert(!daq_downsample_add(&state, 1, 1, 2001, DAQ_BROADCAST_PERIOD_MS, &out));
}''')

    def test_power_loss_does_not_throttle_reporting(self):
        source = (ROOT / "Core/Src/daq_thread.c").read_text()
        self.assertIn("sd_card_notify_power_loss()", source)
        self.assertNotIn("tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U)", source)
        self.assertIn("tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS - elapsed)", source)

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

    def test_supply_launch_period_defaults_and_bounds(self):
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_SUPPLY_LAUNCH_PERIOD_MS == 4);}')
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_SUPPLY_LAUNCH_PERIOD_MS == 5);}',
                     ['DAQ_ACQUISITION_PERIOD_MS=5', 'DAQ_BROADCAST_RATE_HZ=10'])
        for value in ('1', '11'):
            self.compile('#include "daq_rates.h"\nint main(void) {}', ['DAQ_SUPPLY_LAUNCH_PERIOD_MS=' + value], False)

    def test_loadcell_bins_are_fixed_on_uptime_and_close_on_the_next_bin(self):
        self.compile(r'''#include "daq_downsample.h"
#include <assert.h>
#include <string.h>
int main(void) {
  daq_bin_average_t s = {0}; float out = 0; uint64_t at = 0;
  /* Bin [8,12): 9, 9, 11 ms. The first sample of [12,16) closes it. */
  assert(!daq_bin_average_add(&s, 1.0f, 9, 4, &out, &at));
  assert(!daq_bin_average_add(&s, 2.0f, 9, 4, &out, &at));
  assert(!daq_bin_average_add(&s, NAN, 10, 4, &out, &at)); /* skipped */
  assert(!daq_bin_average_add(&s, 6.0f, 11, 4, &out, &at));
  assert(daq_bin_average_add(&s, 100.0f, 12, 4, &out, &at));
  assert(out == 3.0f && at == 11);
  /* A gap of several bins emits once, then starts the new bin. */
  assert(daq_bin_average_add(&s, 7.0f, 40, 4, &out, &at));
  assert(out == 100.0f && at == 12);
  /* Double sum in order, one binary32 rounding of sum/count. */
  s = (daq_bin_average_t){0};
  const float v[] = {0.1f, 0.2f, 0.30000001f};
  double sum = 0;
  for (unsigned i = 0; i < 3; ++i) { sum += (double)v[i]; assert(!daq_bin_average_add(&s, v[i], 100 + i, 4, &out, &at)); }
  assert(daq_bin_average_add(&s, 0, 104, 4, &out, &at));
  const float expected = (float)(sum / 3);
  assert(memcmp(&out, &expected, sizeof(out)) == 0 && at == 102);
  /* 64-bit uptime past the 32-bit HAL wrap keeps fixed bins. */
  s = (daq_bin_average_t){0};
  assert(!daq_bin_average_add(&s, 1, UINT64_C(0x100000000) - 1, 4, &out, &at));
  assert(daq_bin_average_add(&s, 1, UINT64_C(0x100000000), 4, &out, &at));
}''')

    def test_temperature_publication_uses_elapsed_time_not_loop_count(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        block = source[source.index('    const uint32_t report_ms ='):
                       source.index('    /* Pressure shares', source.index('    const uint32_t report_ms ='))]
        self.compile(r'''#include <stdint.h>
#include <assert.h>
#include "daq_rates.h"
#define SEDS_OK 0
#define SEDS_DT_DAQ_ADC_TEMPERATURE 140
static unsigned reports;
#define DAQ_REPORT_TEMPERATURE 4U
static struct {float temperature; unsigned flags;} report;
static unsigned last_temperature_report_ms;
static struct { uint64_t monotonic_ms; float ext_adc_temp_c; } snapshot;
static void publish(void) {
 report.flags=0;
''' + block + r'''
 if(report.flags & DAQ_REPORT_TEMPERATURE) { assert(report.temperature==snapshot.ext_adc_temp_c); reports++; }
}
int main(void) {
  for(unsigned ms=0;ms<=1000;ms+=20) { snapshot.monotonic_ms=ms; publish(); }
  assert(reports==10);
  snapshot.monotonic_ms=2500; publish(); publish();
  assert(reports==11); /* no catch-up burst */
  last_temperature_report_ms=UINT32_MAX-50;
  snapshot.monotonic_ms=49; publish();
  assert(reports==12);
}
''')
