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
        # 1 kHz frames: two fast conversions plus one slot item, plus the
        # launch-window supply pair every 4 ms (sizing only).
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ACQ_FRAME_HZ == 1000); assert(DAQ_ADC_READ_RATE_HZ == 3500); assert(DAQ_ACQUISITION_PERIOD_MS == 2); assert(DAQ_SD_RAW_QUEUE_DEPTH == 401); assert(DAQ_RAW_BATCH_CAPACITY == 11); assert(DAQ_FAST_REPORT_US == 20000); assert(DAQ_KG50_REPORT_US == 50000);}')
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_ACQ_FAST_CHANNELS == 1); assert(DAQ_ADC_READ_RATE_HZ == 2500);}', ['DAQ_CHAMBER_ADC_CHANNEL=-1'])

    def test_unsafe_configurations_fail_compilation(self):
        for defines in [['DAQ_SD_RAW_BUFFER_MS=2000'], ['DAQ_ACQUISITION_PERIOD_MS=0'],
                        ['DAQ_ACQUISITION_PERIOD_MS=21'], ['DAQ_ACQ_FRAME_US=900'], ['DAQ_ACQ_FRAME_US=1100']]:
            with self.subTest(defines=defines):
                self.compile('#include "daq_rates.h"\nint main(void) {}', defines, False)

    def test_report_bins_per_second_without_catchup_bursts(self):
        # 1 kHz conversions: 50 fast-pair bins and 20 KG50 bins per second, each
        # stamped at its bin centre; a gap produces one late bin, no burst.
        self.compile(r"""#include "daq_median.h"
#include "daq_rates.h"
#include <assert.h>
int main(void) {
  daq_bin_us_t fast = {0}, kg50 = {0}; daq_bin_out_t out; unsigned f = 0, k = 0;
  for (uint64_t us = 1000000; us < 2000000; us += 1000) {
    if (daq_bin_us_add(&fast, 1.0f, us + 288, DAQ_FAST_REPORT_US, &out)) { ++f; assert(out.centre_us % 20000 == 10000 && out.count == 20); }
    if (daq_bin_us_add(&kg50, 2.0f, us + 600, DAQ_KG50_REPORT_US, &out)) { ++k; assert(out.centre_us % 50000 == 25000 && out.count == 50); }
  }
  assert(f == 49 && k == 19); /* the last bins close at the next sample */
  assert(daq_bin_us_add(&fast, 3.0f, 5000000, DAQ_FAST_REPORT_US, &out) && out.mean == 1.0f);
  assert(!daq_bin_us_add(&fast, 3.0f, 5001000, DAQ_FAST_REPORT_US, &out));
}""")

    def test_power_loss_does_not_throttle_reporting(self):
        source = (ROOT / "Core/Src/daq_thread.c").read_text()
        self.assertIn("sd_card_notify_power_loss()", source)
        self.assertNotIn("tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 10U)", source)
        self.assertIn("tx_thread_sleep(DAQ_SAMPLE_PERIOD_TICKS - elapsed)", source)

    def test_supply_launch_period_defaults_and_bounds(self):
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_SUPPLY_LAUNCH_PERIOD_MS == 4);}')
        self.compile('#include "daq_rates.h"\n#include <assert.h>\nint main(void) {assert(DAQ_SUPPLY_LAUNCH_PERIOD_MS == 5);}',
                     ['DAQ_ACQUISITION_PERIOD_MS=5'])
        for value in ('1', '11'):
            self.compile('#include "daq_rates.h"\nint main(void) {}', ['DAQ_SUPPLY_LAUNCH_PERIOD_MS=' + value], False)

    def test_temperature_publication_uses_elapsed_time_not_loop_count(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        block = source[source.index('    const uint32_t report_ms ='):
                       source.index('    /* The chamber PT shares', source.index('    const uint32_t report_ms ='))]
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
