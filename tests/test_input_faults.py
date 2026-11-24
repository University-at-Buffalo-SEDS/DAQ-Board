"""Live input fault detection (daq_input_faults.h) on 2026-11-16 test-fire data.

tests/fixtures/input_faults_fire_20261116.json holds DAQ sequences taken from
the GS recording 2026-11-17_00-59-46_065 (times in ms from each window's
first sample, as recorded by the GS):
  vmain_mv_*                DAQ_POWER_MONITOR[2], VMAIN mV
  ch6_connector_uv_0_20s    DAQ_SDADC_VOLTAGES[4], chamber CH6 connector uV
                            (launch accepted at 0.1 s, T0 9.3 s, burn 10.3-14.7 s;
                            should have read about 1-3+ V in the burn)
  p6_legs_uv_449_459s       DAQ_SDADC_VOLTAGES[0..1], P6/3 CH2 (S+) and CH3 uV
  kg1000_pin_uv_*           KG1000 legacy raw * 16000/921, ADC-pin uV
The GS recorded these thinned (VMAIN about 3 Hz); the DAQ observes the same
signals faster, which only adds observations of the same states.
"""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = json.loads((ROOT / 'tests/fixtures/input_faults_fire_20261116.json').read_text())

# Bits, in DAQ_INPUT_HEALTH order.
SUPPLY, KG1000_HIGH, KG1000_LOW, KG50_HIGH, KG50_LOW, P6_IMBALANCE, P6_COMMON, CHAMBER, VDDA_HIGH, VDDA_LOW = (
    1 << i for i in range(10))

DRIVER = r'''
#include "daq_input_faults.h"
#include <stdio.h>
#include <string.h>
int main(void) {
  daq_input_faults_t s; daq_input_faults_reset(&s);
  char op[4]; unsigned t; double a, b;
  while (scanf("%3s", op) == 1) {
    if (op[0] == 'R') {
      uint32_t w[DAQ_INPUT_HEALTH_WORDS]; daq_input_faults_report(&s, w);
      printf("R");
      for (unsigned i = 0; i < DAQ_INPUT_HEALTH_WORDS; ++i) printf(" %u", (unsigned)w[i]);
      printf("\n");
    } else if (op[0] == 'A') {
      printf("A %u %u\n", (unsigned)daq_input_faults_active(&s), (unsigned)s.onset);
    } else if (op[0] == 'I') {
      if (scanf("%lf", &a) != 1) return 2;
      daq_input_faults_kg50_input(&s, (uint8_t)a);
    } else if (op[0] == 'W') {
      /* W k (bit state count) x k: one window's conversions, in order */
      daq_ovr_window_t w; daq_ovr_window_reset(&w);
      unsigned k, bit, n; int state;
      if (scanf("%u", &k) != 1) return 2;
      for (unsigned j = 0; j < k; ++j) {
        if (scanf("%u %d %u", &bit, &state, &n) != 3) return 2;
        for (unsigned i = 0; i < n; ++i) daq_ovr_window_note(&w, bit, (int8_t)state);
      }
      daq_input_faults_over_range(&s, &w);
    } else if (op[0] == 'O') {
      if (scanf("%u %lf", &t, &b) != 2) return 2;
      daq_input_faults_chamber_over_range(&s, t, b < 0.0 ? DAQ_CHAMBER_NO_LAUNCH : (uint32_t)b);
    } else if (op[0] == 'P') {
      if (scanf("%u %lf %lf", &t, &a, &b) != 3) return 2;
      daq_input_faults_p6_legs(&s, (float)a, (float)b, t);
    } else {
      if (scanf("%u %lf", &t, &a) != 2) return 2;
      if (op[0] == 'S') daq_input_faults_supply(&s, (float)a, t);
      else if (op[0] == 'K') daq_input_faults_kg1000(&s, (float)a, t);
      else if (op[0] == 'F') daq_input_faults_kg50(&s, (float)a, t);
      else if (op[0] == 'V') daq_input_faults_vdda(&s, (float)a, t);
      else if (op[0] == 'C') {
        /* C t volts since_t0_ms; a negative since_t0_ms is no launch window. */
        if (scanf("%lf", &b) != 1) return 2;
        daq_input_faults_chamber(&s, (float)a, t, b < 0.0 ? DAQ_CHAMBER_NO_LAUNCH : (uint32_t)b);
      } else return 3;
    }
  }
  return 0;
}
'''


class Report:
    def __init__(self, words):
        self.words = words
        self.active = words[1] & 0xFFFF
        self.latched = words[1] >> 16
        self.episodes = [(words[2 + i // 2] >> (16 * (i & 1))) & 0xFFFF for i in range(8)]
        self.vmain_min_mv, self.vmain_mv = words[6] & 0xFFFF, words[6] >> 16
        self.chamber_min_mv, self.chamber_mv = words[7] & 0xFFFF, words[7] >> 16
        self.p6_imbalance_max_mv, self.p6_common_mv = words[8] & 0xFFFF, words[8] >> 16
        if len(words) >= 27:
            self.episodes += [words[26] & 0xFFFF, words[26] >> 16]
            self.ovr_now, self.ovr_latched = words[18] & 0xFFFF, words[18] >> 16
            self.ovr_low = words[19] & 0xFFFF
            self.ovr_count = [(words[20 + i // 2] >> (16 * (i & 1))) & 0xFFFF for i in range(10)]
            self.vdda_max_mv, self.vdda_mv = words[25] & 0xFFFF, words[25] >> 16


def build(binary, *defines):
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wconversion', '-Wno-sign-conversion',
                    *defines, '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary), '-lm'],
                   input=DRIVER, text=True, check=True)


class InputFaultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tmp.name) / 'faults'
        build(cls.binary)
        # A 1-5 V transducer: live zero 1.0 V, no-signal level 0.5 V, always checked.
        cls.live_zero_binary = Path(cls.tmp.name) / 'faults_live_zero'
        build(cls.live_zero_binary, '-DDAQ_CHAMBER_LIVE_ZERO_V=1.0f')

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_ops(self, ops, binary=None):
        out = subprocess.run([str(binary or self.binary)], input='\n'.join(ops) + '\n', text=True,
                             capture_output=True, check=True).stdout.split('\n')
        results = []
        for line in out:
            if line.startswith('R '):
                results.append(Report([int(x) for x in line.split()[1:]]))
            elif line.startswith('A '):
                results.append(tuple(int(x) for x in line.split()[1:]))
        return results

    # --- Supply ----------------------------------------------------------

    def test_fire_vmain_dips_are_each_counted_and_latched_between_reports(self):
        samples = FIXTURE['vmain_mv_320_460s']
        # Independent count: entries below 12.5 V, re-armed above 13.0 V.
        expected, low = 0, False
        for _, mv in samples:
            if not low and mv < 12500:
                expected, low = expected + 1, True
            elif low and mv >= 13000:
                low = False
        self.assertGreaterEqual(expected, 40, 'the 320-460 s window holds most of the 53 dips')
        # Report once a second, as the firmware does.
        ops, next_report = [], 1000
        for t, mv in samples:
            while t >= next_report:
                ops.append('R')
                next_report += 1000
            ops.append(f'S {t} {mv / 1000}')
        ops.append('R')
        reports = self.run_ops(ops)
        self.assertEqual(reports[-1].episodes[0], expected)
        # Every interval containing a dip latched it and reported its minimum.
        for index, report in enumerate(reports):
            start, end = index * 1000, (index + 1) * 1000
            window = [mv for t, mv in samples if start <= t < end]
            if any(mv < 12500 for mv in window):
                self.assertTrue(report.latched & SUPPLY, f'interval {index}')
                self.assertLessEqual(report.vmain_min_mv, min(window) + 1)
                self.assertGreater(report.vmain_min_mv, 10900)
            elif window and not any(mv < 13000 for t, mv in samples if start - 2100 <= t < end):
                # 2 s above 13.0 V: the fault had cleared before this interval.
                self.assertFalse(report.latched & SUPPLY, f'interval {index}')
        # The long 327.5-336 s dip (28 recorded samples) activates the fault.
        self.assertTrue(any(report.active & SUPPLY for report in reports[7:16]))
        self.assertEqual(reports[-1].words[9] & 0xFFFF, 12500, 'configured level reported in mV')

    def test_single_sample_dip_counts_but_needs_two_samples_to_activate(self):
        reports = self.run_ops([
            'S 0 14.2', 'S 100 11.06', 'A', 'S 200 14.18', 'R',
            'S 1000 14.2', 'R'])
        state, first, second = reports
        self.assertEqual(state, (0, SUPPLY), 'one 100 ms sample: counted, not active')
        self.assertEqual((first.active, first.latched, first.episodes[0]), (0, SUPPLY, 1))
        self.assertEqual(first.vmain_min_mv, 11060)
        self.assertEqual((second.latched, second.vmain_min_mv), (0, 14180), 'latch clears after it is reported; the interval starts from the last reading')

    def test_supply_hysteresis_holds_between_levels_and_clear_needs_two_seconds(self):
        reports = self.run_ops([
            'S 0 11.1', 'S 100 11.1', 'A',
            'S 200 12.8', 'S 300 12.4', 'S 400 12.9', 'A',   # between levels: same dip
            'S 500 13.1', 'A', 'S 2400 13.1', 'A', 'S 2500 13.1', 'A', 'R'])
        self.assertEqual(reports[0][0], SUPPLY)
        self.assertEqual(reports[1][0], SUPPLY)
        self.assertEqual(reports[2][0], SUPPLY, 'clear waits 2 s above 13.0 V')
        self.assertEqual(reports[3][0], SUPPLY)
        self.assertEqual(reports[4][0], 0)
        self.assertEqual(reports[5].episodes[0], 1, 'chatter between 12.5 and 13.0 V is one dip')

    def test_boot_at_the_low_supply_state_activates(self):
        ops = [f'S {t} {mv / 1000}' for t, mv in FIXTURE['vmain_mv_0_14s'] if t < 9000] + ['A']
        self.assertEqual(self.run_ops(ops)[0][0] & SUPPLY, SUPPLY, '0-9.2 s at 11.1 V')

    # --- Load-cell rails -------------------------------------------------

    def test_fire_kg1000_clip_and_low_rail(self):
        samples = FIXTURE['kg1000_pin_uv_325_336s']
        ops = []
        for t, uv in samples:
            ops += [f'K {t} {uv / 1e6}', 'A']
        results = self.run_ops(ops + ['R'])
        states, report = results[:-1], results[-1]
        clipped = [t for t, uv in samples if uv >= 3374000]
        self.assertTrue(clipped, 'clipped at code 0xB40000 (3.375 V)')
        active = [t for (t, _), state in zip(samples, states) if state[0] & KG1000_HIGH]
        self.assertTrue(active and active[0] <= clipped[0] + 100)
        self.assertFalse(states[-1][0] & KG1000_HIGH, 'back to zero load by 336 s')
        self.assertGreaterEqual(report.episodes[1], 2)
        self.assertEqual(report.episodes[2], 0)
        start = FIXTURE['kg1000_pin_uv_1177_1180s']
        state, report = self.run_ops([f'K {t} {uv / 1e6}' for t, uv in start] + ['A', 'R'])
        self.assertEqual(state[0], KG1000_LOW, 'amplifier floor after the reboot')
        self.assertEqual((report.episodes[1], report.episodes[2]), (0, 1))

    def test_kg1000_rail_levels_and_hysteresis_match_the_station(self):
        clip = 0xB40000 * 2.4 / 8388608
        reports = self.run_ops([
            'K 0 0.209', f'K 2 {clip}', 'A', 'K 22 2.4', 'A',
            'K 24 2.35', 'K 26 2.39', 'A',          # inside the 2.3-2.4 V band: still railed
            'K 28 2.2', 'K 527 2.2', 'A', 'K 528 2.2', 'A',
            'K 600 0.0434', 'K 602 0.05', 'K 622 0.05', 'A', 'K 630 0.0601', 'K 1200 0.209', 'A',
            'K 2000 2.39', 'K 2002 0.0435', 'A', 'R'])
        self.assertEqual(reports[0], (0, KG1000_HIGH))
        self.assertEqual(reports[1][0], KG1000_HIGH, '20 ms at the rail')
        self.assertEqual(reports[2][0], KG1000_HIGH)
        self.assertEqual(reports[3][0], KG1000_HIGH, 'clear waits 500 ms')
        self.assertEqual(reports[4][0], 0)
        self.assertEqual(reports[5][0], KG1000_LOW, '0.0434 V is the floor; 0.05 V holds it')
        self.assertEqual(reports[6][0], 0)
        self.assertEqual(reports[7][0], 0, '2.39 V and 0.0435 V are in range')
        self.assertEqual(reports[8].episodes[1:3], [1, 1])

    def test_kg50_rails_follow_the_selected_input(self):
        reports = self.run_ops([
            'I 0', 'F 0 0.2', 'F 2 2.4', 'F 30 2.4', 'A',
            'I 1', 'A',                              # switching input ends that fault
            'F 40 0.0042', 'F 42 0.2109375', 'F 70 0.2', 'A',
            'F 80 -0.15', 'F 110 -0.15', 'A', 'R'])
        self.assertEqual(reports[0][0], KG50_HIGH, 'AMP2 clips at 2.4 V')
        self.assertEqual(reports[1][0], 0)
        self.assertEqual(reports[2][0], KG50_HIGH, 'P6 x16 clips at 0.15 V')
        self.assertEqual(reports[3][0], KG50_HIGH | KG50_LOW)
        self.assertEqual(reports[4].episodes[3:5], [2, 1])

    # --- P6 legs ----------------------------------------------------------

    def test_fire_p6_s_plus_dropout_is_an_imbalance(self):
        legs = FIXTURE['p6_legs_uv_449_459s']
        ops = ['I 1']
        for t, plus, minus in legs:
            ops += [f'P {t} {plus / 1e6} {minus / 1e6}', 'A']
        ops.append('R')
        results = self.run_ops(ops)
        states, report = results[:-1], results[-1]
        dropped = [i for i, (_, plus, minus) in enumerate(legs) if abs(plus - minus) > 50000]
        self.assertTrue(dropped, 'CH2 1.139 -> 0.91 V while CH3 held 1.135 V')
        self.assertTrue(any(states[i][0] & P6_IMBALANCE for i in dropped))
        self.assertEqual(report.episodes[5], 1)
        self.assertGreater(report.p6_imbalance_max_mv, 220)
        self.assertEqual(report.episodes[6], 0, 'the legs never moved together')
        self.assertFalse(any(state[0] & P6_COMMON for state in states))

    def test_p6_checks_only_while_p6_is_selected_and_have_hysteresis(self):
        reports = self.run_ops([
            'I 0', 'P 0 1.139 0.0', 'P 200 1.139 0.0', 'A', 'R',
            'I 1', 'P 300 1.139 1.135', 'P 400 1.139 1.08', 'A', 'P 500 1.139 1.08', 'A',
            'P 600 1.139 1.10', 'P 1700 1.139 1.10', 'A',   # 39 mV: between levels
            'P 1800 1.139 1.125', 'P 2799 1.139 1.125', 'A', 'P 2800 1.139 1.125', 'A',
            'P 3000 0.3 0.3', 'P 3200 0.3 0.3', 'A', 'P 3300 0.65 0.65', 'P 5000 0.65 0.65', 'A',
            'P 5100 1.14 1.135', 'P 6200 1.14 1.135', 'A', 'R'])
        self.assertEqual(reports[0], (0, 0), 'AMP2 selected: P6 legs ignored')
        self.assertEqual(reports[1].words[8], 0xFFFFFFFF)
        self.assertEqual(reports[2][0], 0)
        self.assertEqual(reports[3][0], P6_IMBALANCE)
        self.assertEqual(reports[4][0], P6_IMBALANCE)
        self.assertEqual(reports[5][0], P6_IMBALANCE)
        self.assertEqual(reports[6][0], 0)
        self.assertEqual(reports[7][0], P6_COMMON, 'both legs at 0.3 V: excitation or both leads')
        self.assertEqual(reports[8][0], P6_COMMON, '0.65 V is inside the 0.1 V band')
        self.assertEqual(reports[9][0], 0)
        self.assertEqual(reports[10].p6_common_mv, 1138)

    # --- Chamber ----------------------------------------------------------
    # The installed 0-5 V PX309 reads about 0 V at 0 psig, so its check runs
    # only from T0 + 1.5 s to T0 + 5 s (DAQ_FAULT_CHAMBER_BURN_*_MS).

    FIRE_T0_MS = 9300

    def chamber_ops(self, samples, t0_ms=None, unit=1e6):
        """One 'C' op and one state query per sample; t0_ms None = no launch."""
        ops = []
        for t, value in samples:
            since = t - t0_ms if t0_ms is not None and t0_ms <= t < t0_ms + 20000 else -1
            ops += [f'C {t} {value / unit} {since}', 'A']
        return ops + ['R']

    def test_fire_chamber_reads_no_signal_in_the_burn_window(self):
        samples = FIXTURE['ch6_connector_uv_0_20s']
        self.assertLess(max(uv for _, uv in samples), 20000)
        results = self.run_ops(self.chamber_ops(samples, self.FIRE_T0_MS))
        states, report = results[:-1], results[-1]
        times = [t for t, _ in samples]
        active = [t for t, state in zip(times, states) if state[0] & CHAMBER]
        t0 = self.FIRE_T0_MS
        self.assertTrue(active, 'the dead input is caught in the burn')
        self.assertGreaterEqual(active[0], t0 + 3500, 'armed at T0 + 1.5 s, 2 s set time')
        self.assertLessEqual(active[0], t0 + 3500 + 300, 'within one recorded sample spacing')
        self.assertLess(active[-1], t0 + 5000, 'inactive after T0 + 5 s')
        self.assertFalse(any(state[1] & CHAMBER for t, state in zip(times, states) if t < t0 + 1500),
                         'no episode before the window')
        self.assertEqual(report.episodes[7], 1)
        self.assertEqual(report.active & CHAMBER, 0)
        self.assertLess(report.chamber_min_mv, 20)
        self.assertEqual(report.words[9] >> 16, 50)

    def test_fire_chamber_at_rest_and_without_a_launch_is_not_a_fault(self):
        samples = FIXTURE['ch6_connector_uv_0_20s']
        results = self.run_ops(self.chamber_ops(samples))
        states, report = results[:-1], results[-1]
        self.assertEqual([state for state in states if state != (0, 0)], [])
        self.assertEqual((report.active, report.latched, report.episodes[7]), (0, 0, 0))
        self.assertTrue(report.words[0] & 0x200, 'the reading is still reported')
        self.assertLess(report.chamber_min_mv, 20)

    def test_vented_zero_volts_at_rest_for_a_minute_is_not_a_fault(self):
        ops = [f'C {t} 0.0 -1' for t in range(0, 60000, 100)] + ['A', 'R']
        state, report = self.run_ops(ops)
        self.assertEqual(state, (0, 0))
        self.assertEqual((report.latched, report.episodes[7], report.chamber_mv), (0, 0, 0))
        # The countdown and the first 1.5 s after T0 (ignition delay) are not checked either.
        ops = [f'C {t} 0.0 {t - 20000 if t >= 20000 else -1}' for t in range(0, 21500, 100)] + ['A', 'R']
        state, report = self.run_ops(ops)
        self.assertEqual((state, report.episodes[7]), ((0, 0), 0))

    def test_healthy_rising_pressure_in_the_burn_window_is_not_a_fault(self):
        # PX309-1KG5V: 200 psig/V. 0 V until ignition at T0 + 1.0 s (as in the
        # fire), 400 psig (2 V) by T0 + 1.4 s, 300-600 psig through the burn.
        samples = []
        for t in range(0, 6000, 100):
            if t < 1000:
                v = 0.004
            elif t < 1400:
                v = 2.0 * (t - 1000) / 400
            else:
                v = 1.5 + 1.5 * ((t // 100) % 7) / 6
            samples.append((t, v))
        results = self.run_ops(self.chamber_ops(samples, 0, unit=1))
        states, report = results[:-1], results[-1]
        self.assertEqual([state for state in states if state != (0, 0)], [])
        self.assertEqual((report.latched, report.episodes[7]), (0, 0))

    def test_chamber_burn_window_boundaries(self):
        states = self.run_ops([
            'C 0 0.01 1499', 'A',                     # before T0 + 1.5 s: not checked
            'C 1 0.01 1500', 'A',                     # armed: an episode starts
            'C 2000 0.01 3499', 'A', 'C 2001 0.01 3500', 'A',   # 2 s set time
            'C 3500 0.01 4999', 'A', 'C 3501 0.01 5000', 'A',   # T0 + 5 s: inactive
            'C 3600 0.01 -1', 'A', 'R'])
        self.assertEqual(states[0], (0, 0))
        self.assertEqual(states[1], (0, CHAMBER))
        self.assertEqual(states[2][0], 0)
        self.assertEqual(states[3][0], CHAMBER)
        self.assertEqual(states[4][0], CHAMBER)
        self.assertEqual(states[5][0], 0)
        self.assertEqual(states[6][0], 0)
        self.assertEqual((states[7].latched, states[7].episodes[7]), (CHAMBER, 1),
                         'the in-window fault is latched into the next report and counted')

    def test_chamber_transient_in_the_window_does_not_activate_and_signal_clears(self):
        reports = self.run_ops([
            'C 0 1.2 1500', 'C 100 0.01 1600', 'C 1900 0.01 3400', 'A', 'C 2000 1.0 3500', 'A',
            'C 2100 0.01 3600', 'C 4100 0.01 4900', 'A', 'C 4200 0.08 4950', 'A', 'R'])
        self.assertEqual(reports[0][0], 0, '1.8 s low is a transient')
        self.assertEqual(reports[1][0], 0)
        self.assertEqual(reports[2][0], CHAMBER)
        self.assertEqual(reports[3][0], CHAMBER, '0.08 V is inside the 0.05-0.1 V band')
        self.assertEqual(reports[4].episodes[7], 2)

    def test_live_zero_transducer_is_checked_at_any_time_with_hysteresis(self):
        run = lambda ops: self.run_ops(ops, self.live_zero_binary)
        reports = run([
            'C 0 1.2 -1', 'C 100 0.01 -1', 'C 1900 0.01 -1', 'A', 'C 2000 1.1 -1', 'A',
            'C 3000 0.01 -1', 'C 5000 0.01 -1', 'A', 'C 5100 0.8 -1', 'C 7000 0.8 -1', 'A',
            'C 7100 1.0 -1', 'C 8099 1.0 -1', 'A', 'C 8100 1.0 -1', 'A', 'R'])
        self.assertEqual(reports[0][0], 0, '1.8 s low is a transient')
        self.assertEqual(reports[1][0], 0)
        self.assertEqual(reports[2][0], CHAMBER, 'no launch needed: 0 V is never valid')
        self.assertEqual(reports[3][0], CHAMBER, '0.8 V is inside the 0.5-1.0 V band')
        self.assertEqual(reports[4][0], CHAMBER)
        self.assertEqual(reports[5][0], 0)
        self.assertEqual(reports[6].episodes[7], 2)
        self.assertEqual(reports[6].words[9] >> 16, 500)
        state, report = run([f'C {t} {uv / 1e6} -1' for t, uv in FIXTURE['ch6_connector_uv_0_20s']] + ['A', 'R'])
        self.assertEqual((state[0], report.episodes[7]), (CHAMBER, 1), 'the fire input, at any time')

    # --- Reporting ----------------------------------------------------------

    def test_counters_saturate_and_unknown_readings_are_marked(self):
        reports = self.run_ops(['R'])
        self.assertEqual(reports[0].words[0], 3)
        self.assertEqual(len(reports[0].words), 28)
        self.assertEqual(reports[0].words[6:9], [0xFFFFFFFF] * 3)
        # v2: per-channel statistics unknown, no excitation reading.
        self.assertEqual(reports[0].words[10:14], [0x8000FFFF] * 4)
        self.assertEqual(reports[0].words[14], 0xFFFFFFFF)
        ops = []
        for i in range(70000):
            ops += [f'K {i * 4} 3.0', f'K {i * 4 + 2} 0.2']
        ops.append('R')
        self.assertEqual(self.run_ops(ops)[-1].episodes[1], 0xFFFF)

    def test_nan_readings_are_ignored(self):
        reports = self.run_ops(['S 0 nan', 'K 0 nan', 'C 0 nan 2000', 'I 1', 'P 0 nan 1.1', 'A', 'R'])
        self.assertEqual(reports[0], (0, 0))
        self.assertEqual(reports[1].words[6:9], [0xFFFFFFFF] * 3)


class OverRangeAndVddaTests(unittest.TestCase):
    """Version 3: ADC over-range bits and counters, +3V3A (VDDA) faults."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tmp.name) / 'faults'
        build(cls.binary)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    run_ops = InputFaultTests.run_ops

    def test_v3_report_is_v2_plus_ten_words_with_nothing_seen(self):
        report = self.run_ops(['R'])[0]
        self.assertEqual(len(report.words), 28)
        self.assertEqual((report.ovr_now, report.ovr_latched, report.ovr_low), (0, 0, 0))
        self.assertEqual(report.ovr_count, [0] * 10)
        self.assertEqual((report.vdda_max_mv, report.vdda_mv), (0xFFFF, 0xFFFF))
        self.assertEqual(report.episodes[8:], [0, 0])

    def test_over_range_now_latched_direction_and_counts(self):
        reports = self.run_ops([
            'W 1 0 1 20', 'W 1 9 -1 3', 'W 1 8 1 1', 'R',
            # KG1000's next conversion is in range: now clears, the latch shows it until reported.
            'W 2 0 1 5 0 0 1', 'R', 'R'])
        first, second, third = reports
        self.assertEqual(first.ovr_now, 1 | 1 << 8 | 1 << 9)
        self.assertEqual(first.ovr_latched, first.ovr_now)
        self.assertEqual(first.ovr_low, 1 << 9, 'P6 went below -150 mV')
        self.assertEqual(first.ovr_count[0], 20)
        self.assertEqual((first.ovr_count[8], first.ovr_count[9]), (1, 3))
        self.assertEqual(second.ovr_now, 1 << 8 | 1 << 9, 'KG1000 back in range')
        self.assertEqual(second.ovr_latched & 1, 1, 'but over range during this interval')
        self.assertEqual(second.ovr_count[0], 25)
        self.assertEqual(third.ovr_latched, third.ovr_now)

    def test_a_window_without_a_conversion_keeps_the_state(self):
        reports = self.run_ops(['W 1 3 1 1', 'W 1 0 0 1', 'R'])
        self.assertEqual(reports[0].ovr_now, 1 << 3, 'CH3 not converted again: still over range')

    def test_counters_wrap_modulo_65536(self):
        ops = ['W 1 6 1 65535', 'W 1 6 1 3', 'R']
        self.assertEqual(self.run_ops(ops)[0].ovr_count[6], 2)

    def test_vdda_fire_level_sets_the_high_fault_after_100_ms(self):
        # 4.19 V: 3.3 V x 14.1 / 11.1, the 2026-11-16 fire's implied +3V3A.
        reports = self.run_ops(['V 0 3.30', 'V 100 4.19', 'A', 'V 200 4.19', 'A', 'R',
                                'V 300 3.43', 'V 2300 3.43', 'A', 'V 2400 3.41', 'V 4399 3.41', 'A',
                                'V 4400 3.41', 'A', 'R', 'R'])
        self.assertEqual(reports[0][0], 0)
        self.assertEqual(reports[1][0], VDDA_HIGH)
        self.assertEqual((reports[2].vdda_max_mv, reports[2].vdda_mv), (4190, 4190))
        self.assertEqual(reports[2].episodes[8], 1)
        self.assertEqual(reports[3][0], VDDA_HIGH, '3.43 V is inside the 3.42-3.45 V hysteresis')
        self.assertEqual(reports[4][0], VDDA_HIGH, 'clears only after 2 s below 3.42 V')
        self.assertEqual(reports[5][0], 0)
        self.assertEqual(reports[6].vdda_max_mv, 4190, 'the interval starts at the last reading')
        self.assertEqual(reports[7].vdda_max_mv, 3410)

    def test_vdda_band_edges(self):
        cases = [(3.45, 0), (3.451, VDDA_HIGH), (3.15, 0), (3.149, VDDA_LOW), (3.3, 0)]
        for volts, expected in cases:
            state = self.run_ops([f'V 0 {volts}', f'V 100 {volts}', 'A'])[0]
            self.assertEqual(state[0], expected, volts)
        low = self.run_ops(['V 0 3.0', 'R'])[0]
        self.assertEqual(low.episodes[9], 1)
        self.assertEqual(low.latched, VDDA_LOW, 'one low reading is latched')

    def test_vdda_faults_do_not_move_the_v2_episode_words(self):
        report = self.run_ops(['V 0 4.0', 'V 100 4.0', 'R'])[0]
        self.assertEqual(report.words[2:6], [0, 0, 0, 0])
        self.assertEqual(report.words[26], 1)

    def test_chamber_over_range_in_the_burn_is_not_no_signal(self):
        # A live chamber goes over range in the burn window: no no-signal fault,
        # and a dead input that then goes over range clears it.
        ops = [f'O {t} {2000 + t}' for t in range(0, 3000, 2)] + ['A']
        self.assertEqual(self.run_ops(ops)[0][0], 0)
        ops = ([f'C {t} 0.01 {1500 + t}' for t in range(0, 2100, 100)] + ['A'] +
               [f'O {2100 + t} {3600 + t}' for t in range(0, 1100, 100)] + ['A', 'R'])
        dead, over, report = self.run_ops(ops)
        self.assertEqual(dead[0], CHAMBER)
        self.assertEqual(over[0], 0)
        self.assertTrue(report.words[0] & (1 << 9), 'the chamber input was seen')

    def test_over_range_rails_reach_the_rail_checks(self):
        # Acquisition passes an over-range window as +/-INFINITY.
        state = self.run_ops(['K 0 inf', 'K 20 inf', 'A'])[0]
        self.assertEqual(state[0], KG1000_HIGH)
        state = self.run_ops(['I 1', 'F 0 -inf', 'F 20 -inf', 'A'])[0]
        self.assertEqual(state[0], KG50_LOW)


class InputHealthWiringTests(unittest.TestCase):
    def test_schema_header_and_firmware_agree(self):
        definitions = json.loads((ROOT / 'config/sedsnet.json').read_text())
        entry = next(t for t in definitions['types'] if t['name'] == 'DAQ_INPUT_HEALTH')
        self.assertEqual(entry['element'], dict(kind='Static', data_type='UInt32', count=28))
        self.assertEqual((entry['reliable'], entry['endpoints']), ('Off', ['ActuatorGroundStation']))
        hashed = 0x811c9dc5 ^ 0x5459
        for byte in b'DAQ_INPUT_HEALTH':
            hashed = ((hashed ^ byte) * 0x01000193) & 0xffffffff
        header = (ROOT / 'Core/Inc/sedsnet_config.h').read_text()
        self.assertIn(f'#define SEDS_DT_DAQ_INPUT_HEALTH ((uint32_t){0x10000 | (hashed & 0x0fffffff)}U)', header)
        thread = (ROOT / 'Core/Src/daq_thread.c').read_text()
        self.assertIn('daq_observe_input_faults(&snapshot, &window, report.kg50_input, supply_fresh ? &supply : NULL, &ovr);',
                      thread)
        health = (ROOT / 'Core/Src/board_health.c').read_text()
        self.assertIn('SEDS_DT_DAQ_INPUT_HEALTH', health)

    def test_thresholds_mirror_the_station_rails(self):
        config = (ROOT / 'Core/Inc/daq_input_fault_config.h').read_text()
        self.assertIn('#define DAQ_FAULT_ADC_FULL_SCALE_V 2.4f', config)
        self.assertIn('#define DAQ_FAULT_KG1000_LOW_V ((float)(0.0025 * (16000.0 / 921.0)))', config)
        self.assertIn('(DAQ_FAULT_ADC_FULL_SCALE_V / 16.0f)', config)


if __name__ == '__main__':
    unittest.main()
