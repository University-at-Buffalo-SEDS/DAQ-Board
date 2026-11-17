"""Live input fault detection (daq_input_faults.h) on 2026-11-16 test-fire data.

tests/fixtures/input_faults_fire_20261116.json holds DAQ sequences taken from
the GS recording 2026-11-17_00-59-46_065 (times in ms from each window's
first sample, as recorded by the GS):
  vmain_mv_*                DAQ_POWER_MONITOR[2], VMAIN mV
  ch6_connector_uv_0_20s    DAQ_SDADC_VOLTAGES[4], chamber CH6 connector uV
                            (launch accepted at 0.1 s, T0 9.3 s, burn 10.3-14.7 s)
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
SUPPLY, KG1000_HIGH, KG1000_LOW, KG50_HIGH, KG50_LOW, P6_IMBALANCE, P6_COMMON, CHAMBER = (1 << i for i in range(8))

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
    } else if (op[0] == 'P') {
      if (scanf("%u %lf %lf", &t, &a, &b) != 3) return 2;
      daq_input_faults_p6_legs(&s, (float)a, (float)b, t);
    } else {
      if (scanf("%u %lf", &t, &a) != 2) return 2;
      if (op[0] == 'S') daq_input_faults_supply(&s, (float)a, t);
      else if (op[0] == 'K') daq_input_faults_kg1000(&s, (float)a, t);
      else if (op[0] == 'F') daq_input_faults_kg50(&s, (float)a, t);
      else if (op[0] == 'C') daq_input_faults_chamber(&s, (float)a, t);
      else return 3;
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


class InputFaultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tmp.name) / 'faults'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wconversion', '-Wno-sign-conversion',
                        '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(cls.binary), '-lm'],
                       input=DRIVER, text=True, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_ops(self, ops):
        out = subprocess.run([str(self.binary)], input='\n'.join(ops) + '\n', text=True,
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

    def test_fire_chamber_reads_no_signal_through_the_burn(self):
        samples = FIXTURE['ch6_connector_uv_0_20s']
        self.assertLess(max(uv for _, uv in samples), 20000)
        ops = []
        for t, uv in samples:
            ops += [f'C {t} {uv / 1e6}', 'A']
        ops.append('R')
        results = self.run_ops(ops)
        states, report = results[:-1], results[-1]
        times = [t for t, _ in samples]
        active = [t for t, state in zip(times, states) if state[0] & CHAMBER]
        self.assertTrue(active and active[0] <= 2500, 'active within 2 s of the first reading')
        burn = [state[0] & CHAMBER for t, state in zip(times, states) if 10300 <= t <= 14700]
        self.assertTrue(burn and all(burn))
        self.assertEqual(report.episodes[7], 1)
        self.assertLess(report.chamber_min_mv, 20)
        self.assertEqual(report.words[9] >> 16, 50)

    def test_chamber_transient_does_not_activate_and_signal_clears(self):
        reports = self.run_ops([
            'C 0 1.2', 'C 100 0.01', 'C 1900 0.01', 'A', 'C 2000 1.0', 'A',
            'C 3000 0.01', 'C 5000 0.01', 'A', 'C 5100 0.08', 'C 7000 0.08', 'A',
            'C 7100 0.1', 'C 8099 0.1', 'A', 'C 8100 0.1', 'A', 'R'])
        self.assertEqual(reports[0][0], 0, '1.8 s low is a transient')
        self.assertEqual(reports[1][0], 0)
        self.assertEqual(reports[2][0], CHAMBER)
        self.assertEqual(reports[3][0], CHAMBER, '0.08 V is inside the 0.05-0.1 V band')
        self.assertEqual(reports[4][0], CHAMBER)
        self.assertEqual(reports[5][0], 0)
        self.assertEqual(reports[6].episodes[7], 2)

    # --- Reporting ----------------------------------------------------------

    def test_counters_saturate_and_unknown_readings_are_marked(self):
        reports = self.run_ops(['R'])
        self.assertEqual(reports[0].words[0], 1)
        self.assertEqual(reports[0].words[6:9], [0xFFFFFFFF] * 3)
        ops = []
        for i in range(70000):
            ops += [f'K {i * 4} 3.0', f'K {i * 4 + 2} 0.2']
        ops.append('R')
        self.assertEqual(self.run_ops(ops)[-1].episodes[1], 0xFFFF)

    def test_nan_readings_are_ignored(self):
        reports = self.run_ops(['S 0 nan', 'K 0 nan', 'C 0 nan', 'I 1', 'P 0 nan 1.1', 'A', 'R'])
        self.assertEqual(reports[0], (0, 0))
        self.assertEqual(reports[1].words[6:9], [0xFFFFFFFF] * 3)


class InputHealthWiringTests(unittest.TestCase):
    def test_schema_header_and_firmware_agree(self):
        definitions = json.loads((ROOT / 'config/sedsnet.json').read_text())
        entry = next(t for t in definitions['types'] if t['name'] == 'DAQ_INPUT_HEALTH')
        self.assertEqual(entry['element'], dict(kind='Static', data_type='UInt32', count=10))
        self.assertEqual((entry['reliable'], entry['endpoints']), ('Off', ['ActuatorGroundStation']))
        hashed = 0x811c9dc5 ^ 0x5459
        for byte in b'DAQ_INPUT_HEALTH':
            hashed = ((hashed ^ byte) * 0x01000193) & 0xffffffff
        header = (ROOT / 'Core/Inc/sedsnet_config.h').read_text()
        self.assertIn(f'#define SEDS_DT_DAQ_INPUT_HEALTH ((uint32_t){0x10000 | (hashed & 0x0fffffff)}U)', header)
        thread = (ROOT / 'Core/Src/daq_thread.c').read_text()
        self.assertIn('daq_observe_input_faults(&snapshot, &window, report.kg50_input, supply_fresh ? &supply : NULL);', thread)
        health = (ROOT / 'Core/Src/board_health.c').read_text()
        self.assertIn('SEDS_DT_DAQ_INPUT_HEALTH', health)

    def test_thresholds_mirror_the_station_rails(self):
        config = (ROOT / 'Core/Inc/daq_input_fault_config.h').read_text()
        self.assertIn('#define DAQ_FAULT_ADC_FULL_SCALE_V 2.4f', config)
        self.assertIn('#define DAQ_FAULT_KG1000_LOW_V ((float)(0.0025 * (16000.0 / 921.0)))', config)
        self.assertIn('(DAQ_FAULT_ADC_FULL_SCALE_V / 16.0f)', config)


if __name__ == '__main__':
    unittest.main()
