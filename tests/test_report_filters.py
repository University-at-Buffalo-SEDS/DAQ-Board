"""Report filters (daq_median.h), per-channel noise/drift (daq_channel_stats.h)
and the timing of the safety-relevant input faults at the new 1 kHz windows.

SD keeps every raw conversion; these filters only shape what is published.
The safety checks (daq_input_faults.h) run on raw 2 ms window means with
their existing time debounce, not on the median-filtered stream."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


def run_c(code):
    with tempfile.TemporaryDirectory() as tmp:
        exe = pathlib.Path(tmp) / 't'
        subprocess.run(['cc', '-std=c11', '-O1', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                        '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-lm', '-o', str(exe)],
                       input=code, text=True, check=True, capture_output=True)
        return subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout


class MedianAndBinTests(unittest.TestCase):
    def test_spikes_rejected_steps_pass_without_smearing(self):
        run_c(r'''
#include <assert.h>
#include "daq_median.h"
int main(void) {
  for (unsigned size = 3; size <= 5; size += 2) {
    daq_median_t m; daq_median_init(&m, (uint8_t)size);
    const unsigned delay = (size - 1) / 2;
    /* Baseline 1.0 with isolated spikes of up to delay consecutive samples:
     * none reaches the output. */
    for (unsigned i = 0; i < 200; ++i) {
      const int spike = (i % 20) >= 10 && (i % 20) < 10 + delay;
      const float out = daq_median_add(&m, spike ? ((i & 1) ? 1000.0f : -1000.0f) : 1.0f);
      assert(out == 1.0f);
    }
    /* A genuine step 1 -> 5 reaches the output after `delay` samples, with no
     * intermediate value (a median never averages across the step). */
    unsigned first = 0;
    for (unsigned i = 0; i < 10; ++i) {
      const float out = daq_median_add(&m, 5.0f);
      assert(out == 1.0f || out == 5.0f);
      if (out == 5.0f && !first) first = i + 1;
    }
    assert(first == delay + 1);
    /* Non-finite inputs are skipped. */
    assert(daq_median_add(&m, NAN) == 5.0f);
  }
  /* The first output is not delayed while the window fills. */
  daq_median_t m; daq_median_init(&m, 5);
  assert(daq_median_add(&m, 2.0f) == 2.0f);
  assert(isnan(daq_median_add(&(daq_median_t){0}, NAN)));
}
''')

    def test_fast_pipeline_step_reaches_report_within_one_bin(self):
        # Ignition-like step at 1 kHz: median-3 then 20 ms bins. The bin holding
        # the step carries a partial mean; the next bin carries the full value.
        out = run_c(r'''
#include <stdio.h>
#include "daq_median.h"
int main(void) {
  daq_median_t m; daq_median_init(&m, 3); daq_bin_us_t b = {0}; daq_bin_out_t out;
  for (uint64_t t = 0; t < 100000; t += 1000) {
    const float v = t >= 50288 ? 100.0f : 0.0f;
    if (daq_bin_us_add(&b, daq_median_add(&m, v), t + 288, 20000, &out))
      printf("%llu %.3f %.1f %.1f\n", (unsigned long long)out.centre_us, out.mean, out.min, out.max);
  }
}
''')
        rows = [line.split() for line in out.splitlines()]
        means = {int(r[0]): float(r[1]) for r in rows}
        self.assertEqual(means[30000], 0.0)
        self.assertAlmostEqual(means[50000], 100.0 * 8 / 20)   # step at 50.288 ms, 1 sample median delay
        self.assertEqual(means[70000], 100.0)


class ChannelStatsTests(unittest.TestCase):
    def test_noise_drift_and_health_word(self):
        run_c(r'''
#include <assert.h>
#include <stdlib.h>
#include "daq_channel_stats.h"
static double gauss(void) {
  double s = 0; for (int i = 0; i < 12; ++i) s += (double)rand() / RAND_MAX; return s - 6.0;
}
int main(void) {
  srand(7);
  daq_channel_stats_t s = {0};
  /* 1 kHz white noise of 50 uV on a slow ramp of 600 uV per minute. */
  for (uint32_t ms = 0; ms < 80000; ++ms)
    daq_channel_stats_add(&s, (float)(0.2 + 50e-6 * gauss() + 600e-6 * ms / 60000.0), ms);
  assert(s.noise_valid && fabsf(s.noise_v - 50e-6f) < 3e-6f);
  assert(s.drift_valid && fabsf(s.drift_v_per_min - 600e-6f) < 20e-6f);
  const uint32_t word = daq_channel_stats_word(&s);
  assert((word & 0xFFFF) >= 47 && (word & 0xFFFF) <= 53);
  assert((int16_t)(word >> 16) >= 580 && (int16_t)(word >> 16) <= 620);
  daq_channel_stats_t none = {0};
  assert(daq_channel_stats_word(&none) == 0x8000FFFFU);
  none.noise_valid = none.drift_valid = 1; none.noise_v = 1.0f; none.drift_v_per_min = -1.0f;
  assert(daq_channel_stats_word(&none) == ((uint32_t)(uint16_t)(int16_t)-32767 << 16 | 0xFFFEU));
}
''')


class SafetyTimingTests(unittest.TestCase):
    FAULTS = r'''
#include <assert.h>
#include <stdio.h>
#include "daq_input_faults.h"
/* One raw window mean per 2 ms acquisition cycle of 1 kHz conversions. */
static inline float window_mean(const float *samples, unsigned n) { float s = 0; for (unsigned i = 0; i < n; ++i) s += samples[i]; return s / n; }
'''

    def test_rail_clip_trips_on_time_and_single_spikes_do_not(self):
        run_c(self.FAULTS + r'''
int main(void) {
  daq_input_faults_t f; daq_input_faults_reset(&f);
  /* One conversion at the rail in a 2-sample window: the mean is far from the
   * rail, so it is not even an episode. */
  for (uint32_t ms = 0; ms < 1000; ms += 2) {
    const float s[2] = {ms == 500 ? 2.5f : 0.21f, 0.21f};
    daq_input_faults_kg1000(&f, window_mean(s, 2), ms);
  }
  assert(!daq_input_faults_active(&f) && f.fault[DAQ_FAULT_KG1000_HIGH_RAIL].episodes == 0);
  /* A whole 2 ms window at the rail is counted but does not activate. */
  daq_input_faults_kg1000(&f, 2.45f, 1000);
  for (uint32_t ms = 1002; ms < 2000; ms += 2) daq_input_faults_kg1000(&f, 0.21f, ms);
  assert(!daq_input_faults_active(&f) && f.fault[DAQ_FAULT_KG1000_HIGH_RAIL].episodes == 1);
  /* A real clip (supply dip) activates DAQ_FAULT_RAIL_SET_MS after onset. */
  uint32_t active_at = 0;
  for (uint32_t ms = 3000; ms < 3200; ms += 2) {
    daq_input_faults_kg1000(&f, 3.375f, ms);
    if (!active_at && (daq_input_faults_active(&f) & (1U << DAQ_FAULT_KG1000_HIGH_RAIL))) active_at = ms;
  }
  assert(active_at == 3000 + DAQ_FAULT_RAIL_SET_MS);
}
''')

    def test_chamber_no_signal_trips_in_burn_window_on_time(self):
        run_c(self.FAULTS + r'''
int main(void) {
  daq_input_faults_t f; daq_input_faults_reset(&f);
  uint32_t active_since_t0 = 0;
  /* Dead PT (0.01 V) all launch long, observed every 2 ms. */
  for (uint32_t t = 0; t < 6000; t += 2) {
    daq_input_faults_chamber(&f, 0.01f, 100000 + t, t);
    if (!active_since_t0 && (daq_input_faults_active(&f) & (1U << DAQ_FAULT_CHAMBER_NO_SIGNAL))) active_since_t0 = t;
  }
  assert(active_since_t0 == DAQ_FAULT_CHAMBER_BURN_START_MS + DAQ_FAULT_CHAMBER_SET_MS);
  /* A healthy PT with one-sample dropouts never trips. */
  daq_input_faults_reset(&f);
  for (uint32_t t = 0; t < 6000; t += 2)
    daq_input_faults_chamber(&f, (t % 500) == 0 ? 0.0f : 2.0f, 200000 + t, t);
  assert(!(daq_input_faults_active(&f) & (1U << DAQ_FAULT_CHAMBER_NO_SIGNAL)));
}
''')

    def test_supply_dip_trips_on_time(self):
        run_c(self.FAULTS + r'''
int main(void) {
  daq_input_faults_t f; daq_input_faults_reset(&f);
  /* Launch window: VMAIN every 4 ms. */
  uint32_t active_at = 0;
  for (uint32_t ms = 0; ms < 400; ms += 4) {
    daq_input_faults_supply(&f, ms >= 100 ? 11.0f : 14.2f, ms);
    if (!active_at && (daq_input_faults_active(&f) & 1U)) active_at = ms;
  }
  assert(active_at == 100 + DAQ_FAULT_SUPPLY_SET_MS);
}
''')

    def test_p6_leg_pairs_every_50_ms_keep_the_fault_latency(self):
        # The rotation converts CH2 then CH3 10 ms later, twice per 100 ms; the
        # former 10 Hz auxiliary pass gave one pair per 100 ms.
        run_c(self.FAULTS + r'''
int main(void) {
  daq_input_faults_t f; daq_input_faults_reset(&f);
  daq_input_faults_kg50_input(&f, 1);
  uint32_t onset = 1000, active_at = 0;
  for (uint32_t t = 10; t < 2000; t += 50) {
    const float plus = t >= onset ? 0.91f : 1.139f;  /* S+ lead drops out */
    daq_input_faults_p6_legs(&f, plus, 1.135f, t);
    if (!active_at && (daq_input_faults_active(&f) & (1U << DAQ_FAULT_P6_LEG_IMBALANCE))) active_at = t;
  }
  /* First pair after onset at 1010 ms; active one set time later. */
  assert(active_at == 1010 + DAQ_FAULT_P6_IMBALANCE_SET_MS);
}
''')


if __name__ == '__main__':
    unittest.main()
