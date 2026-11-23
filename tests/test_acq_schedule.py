"""MCP3564R frame schedule: the pure tables (daq_acq_schedule.h) and the
production frame engine (mcp3564r.c) on the modelled ADC/SPI/TIM2.

These verify determinism and starvation-freedom of the schedule as coded;
the conversion-time model and real interrupt latencies still need a bench
capture (DAQ_INPUT_HEALTH v2 words 15-17 report them on hardware)."""
import collections
import pathlib
import subprocess
import tempfile
import unittest

import acq_sim_support as sim

ROOT = pathlib.Path(__file__).resolve().parents[1]

TABLES = r'''
#include <assert.h>
#include <stdio.h>
#include "daq_acq_schedule.h"
int main(void) {
  /* Modelled scan durations: settled SINC3, AZ_MUX doubles, DMCLK = 4 MHz. */
  assert(DAQ_ACQ_FILL_FAST_US == 384 && DAQ_ACQ_LAUNCH_FAST_US == 768);
  assert(DAQ_ACQ_FILL_SLOW_US == 384 && DAQ_ACQ_LAUNCH_SLOW_US == 96 && DAQ_ACQ_TEMPERATURE_US == 384);
  printf("fill_slot %u launch_slot %u fill_end %u launch_end %u\n", (unsigned)DAQ_ACQ_FILL_SLOT_US,
         (unsigned)DAQ_ACQ_LAUNCH_SLOT_US,
         (unsigned)DAQ_ACQ_SLOT_END_US(DAQ_ACQ_FILL_SLOT_US, DAQ_ACQ_FILL_SLOW_MAX_US),
         (unsigned)DAQ_ACQ_SLOT_END_US(DAQ_ACQ_LAUNCH_SLOT_US, DAQ_ACQ_LAUNCH_SLOW_US));
  for (unsigned profile = 0; profile < 2; ++profile) {
    const daq_acq_scan_t a = daq_acq_fast_scan(profile);
    assert(a.scan == (1U | DAQ_CHAMBER_SCAN_MASK) && a.config2 == DAQ_ACQ_CONFIG2_GAIN1 && a.channels == 2);
    assert(((a.config1 >> 2) & 15) == (profile ? 3 : 2) && (a.config1 >> 6) == 0);
    for (unsigned kg50 = 0; kg50 < 2; ++kg50) {
      unsigned counts[16] = {0};
      uint8_t items[DAQ_ACQ_AUX_ROTATION];
      const unsigned n = daq_acq_aux_items(profile, kg50, items);
      assert(n == 10);
      for (unsigned i = 0; i < n; ++i) {
        assert(!daq_chamber_is_channel(items[i]));
        assert(!(kg50 == 0 && items[i] == 1));
        assert(items[i] != DAQ_ACQ_ITEM_TEMPERATURE && items[i] >= 1 && items[i] <= 7);
      }
      /* P6 legs open each half of the rotation, back to back. */
      assert(items[0] == 2 && items[1] == 3 && items[5] == 2 && items[6] == 3);
      for (uint32_t frame = 0; frame < 100000; ++frame) {
        const uint8_t item = daq_acq_slot_item(profile, kg50, frame);
        const daq_acq_scan_t b = daq_acq_slot_scan(profile, item);
        assert(daq_acq_slot_fits(&b, b.offset_us));
        ++counts[item];
        if (frame % 10 == 8 && profile == 0) { assert(item == DAQ_ACQ_ITEM_TEMPERATURE); continue; }
        if (frame % 10 != 9) { assert(item == (kg50 ? DAQ_ACQ_ITEM_P6 : 1)); continue; }
        assert(item == items[(frame / 10) % 10]);
      }
      /* Every auxiliary channel at least every 100 ms (10 per second of frames). */
      for (unsigned channel = 1; channel <= 7; ++channel) {
        if (channel == 1 && !kg50) continue; /* CH1 is then the KG50 item itself */
        if (daq_chamber_is_channel(channel)) { assert(counts[channel] == 0); continue; }
        assert(counts[channel] >= 1000);
      }
      assert(counts[2] == 2000 && counts[3] == 2000);
      assert(counts[kg50 ? DAQ_ACQ_ITEM_P6 : 1] == (profile ? 90000 : 80000));
      assert(counts[DAQ_ACQ_ITEM_TEMPERATURE] == (profile ? 0 : 10000));
    }
  }
  const daq_acq_scan_t p6 = daq_acq_slot_scan(DAQ_ACQ_PROFILE_FILL, DAQ_ACQ_ITEM_P6);
  assert(p6.config2 == DAQ_ACQ_CONFIG2_GAIN16 && p6.scan == (1U << 9) && ((p6.config1 >> 2) & 15) == 3);
  const daq_acq_scan_t t = daq_acq_slot_scan(DAQ_ACQ_PROFILE_FILL, DAQ_ACQ_ITEM_TEMPERATURE);
  assert(t.scan == (1U << 12) && (t.config1 >> 6) == 2 && ((t.config1 >> 2) & 15) == 1);
  uint8_t burst[DAQ_ACQ_CONFIG_BURST_BYTES];
  daq_acq_config_burst(&p6, burst);
  const uint8_t expected[] = {0x46, 0x82, 0x0c, 0xef, 0xb0, 0x06, 0x08, 0x00, 0x02, 0x00};
  for (unsigned i = 0; i < sizeof expected; ++i) assert(burst[i] == expected[i]);
  /* A late scan A can push the slot out of its frame: never past the end. */
  assert(!daq_acq_slot_fits(&p6, 600));
  return 0;
}
'''


class ScheduleTableTests(unittest.TestCase):
    def test_tables_budgets_and_rotation(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe = pathlib.Path(tmp) / 't'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'Core/Inc'),
                            '-x', 'c', '-', '-o', str(exe)], input=TABLES, text=True, check=True, capture_output=True)
            out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
        values = dict(zip(out.split()[::2], map(int, out.split()[1::2])))
        self.assertLessEqual(values['fill_end'], 1000)
        self.assertLessEqual(values['launch_end'], 1000)

    def test_frame_that_does_not_fit_fails_to_compile(self):
        code = '#include "daq_acq_schedule.h"\nint main(void){return 0;}\n'
        for define in ['DAQ_ACQ_FRAME_US=800', 'DAQ_ACQ_FRAME_US=900']:
            with self.subTest(define=define), tempfile.TemporaryDirectory() as tmp:
                result = subprocess.run(['cc', '-std=c11', '-D' + define, '-I', str(ROOT / 'Core/Inc'),
                                         '-x', 'c', '-', '-o', str(pathlib.Path(tmp) / 't')],
                                        input=code, text=True, capture_output=True)
                self.assertNotEqual(result.returncode, 0)


def fast_times(samples, channel):
    return [(s[1], s[3]) for s in samples if s[0] == channel]


# Conversion midpoint of each fast channel from the frame start (descending
# scan order: CH6 first, then CH0), per profile. The frame tick is strictly
# periodic; the midpoint moves once when the profile (and so the OSR) changes.
MIDPOINT = {6: {0: 96, 1: 192}, 0: {0: 288, 1: 576}}


class FrameEngineTests(unittest.TestCase):
    def assert_strict_kilohertz(self, times, channel, switch_tolerance_us=0):
        steps = collections.Counter()
        for (a, pa), (b, pb) in zip(times, times[1:]):
            expected = 1000 + MIDPOINT[channel][pb] - MIDPOINT[channel][pa]
            # Within a profile the spacing is exact; a switch moves the midpoint
            # by the modelled amount (plus the model error when the ADC is slower).
            tolerance = switch_tolerance_us if pa != pb else 0
            self.assertLessEqual(abs(b - a - expected), tolerance, (a, b, pa, pb))
            steps[b - a] += 1
        return len(times)

    def test_fast_pair_is_strictly_periodic_in_both_profiles(self):
        # 1 s FILL, 1.5 s LAUNCH, back to FILL: the fast pair never misses a frame
        # and keeps exactly 1000 us spacing through both profile switches.
        samples, diag = sim.run(seconds=4.0, launch=(1005, 2505), kg50=1)
        for channel in (0, 6):
            with self.subTest(channel=channel):
                count = self.assert_strict_kilohertz(fast_times(samples, channel), channel)
                self.assertGreaterEqual(count, 3995)
        for key in ('frames_skipped', 'slots_skipped', 'scans_timed_out', 'status_faults',
                    'unexpected', 'recoveries', 'overruns'):
            self.assertEqual(diag[key], 0, key)
        # Same-frame skew between the two fast channels is one conversion.
        by_frame = collections.defaultdict(dict)
        for channel, t, frame, profile, slot, code in samples:
            by_frame[frame][channel] = (t, profile)
        skews = {abs(v[0][0] - v[6][0]) for v in by_frame.values() if 0 in v and 6 in v}
        self.assertEqual(skews, {192, 384})  # FILL: 192 us per channel, LAUNCH: 384
        self.assertLess(diag['start_late_max_us'], 15)

    def test_slot_rotation_and_kg50_density(self):
        samples, diag = sim.run(seconds=2.0, kg50=1)
        slot = collections.Counter(s[0] for s in samples if s[4] == 1)
        self.assertGreaterEqual(slot[9], 1590)      # P6 KG50 in 8 of 10 FILL frames
        aux = {c: n for c, n in slot.items() if c != 9}
        self.assertEqual(set(aux), {1, 2, 3, 4, 5, 7})  # TEMP is consumed inside the driver
        # 10-slot rotation every 100 ms: legs and CH1/CH4 twice, CH5/CH7 once.
        self.assertEqual({c: round(n / 10) for c, n in aux.items()}, {2: 4, 3: 4, 1: 4, 4: 4, 5: 2, 7: 2})
        p6 = [s[1] for s in samples if s[0] == 9]
        steps = collections.Counter(b - a for a, b in zip(p6, p6[1:]))
        self.assertEqual(set(steps), {1000, 3000})  # temperature and aux frames in each 10 ms
        samples, diag = sim.run(seconds=1.0, kg50=0)
        slot = collections.Counter(s[0] for s in samples if s[4] == 1)
        self.assertNotIn(9, slot)
        self.assertGreaterEqual(slot[1], 795)  # AMP2 KG50 in 8 of 10 FILL frames

    def test_slow_conversions_drop_slots_never_fast_samples(self):
        # Conversions 12 % slower than modelled: LAUNCH slots no longer fit and
        # are skipped and counted; the fast pair stays strictly periodic.
        samples, diag = sim.run(seconds=1.5, slow_ppm=120000, launch=(0, 10_000), kg50=1)
        self.assertGreater(diag['slots_skipped'], 1000)
        for channel in (0, 6):
            self.assert_strict_kilohertz(fast_times(samples, channel), channel, switch_tolerance_us=80)
        self.assertEqual(diag['frames_skipped'], 0)

    def test_lost_drdy_edge_is_recovered(self):
        samples, diag = sim.run(seconds=2.0, drop_every=997, kg50=1)
        self.assertEqual(diag['recoveries'], 0)
        times = [t for t, _ in fast_times(samples, 0)]
        steps = collections.Counter(b - a for a, b in zip(times, times[1:]))
        # Lost edges cost at most the affected frame; spacing returns to 1000.
        self.assertGreater(steps[1000], 0.97 * len(times))
        self.assertTrue(all(step % 1000 < 30 or step % 1000 > 970 for step in steps), steps)

    def test_device_fault_recovers_and_fast_sampling_resumes(self):
        # An invalid STATUS byte: the thread re-initialises the device (blocking
        # SPI only with the engine stopped) and 1 kHz sampling resumes.
        samples, diag = sim.run(seconds=1.0, waveform='badstatus=500')
        self.assertEqual(diag['status_faults'], 1)
        self.assertEqual(diag['recoveries'], 1)
        times = [t for t, _ in fast_times(samples, 0)]
        late = [t for t in times if t > times[0] + 600_000]
        self.assertGreater(len(late), 390)
        self.assertEqual(set(b - a for a, b in zip(late, late[1:])), {1000})

    def test_lost_dma_completion_times_out_and_recovers(self):
        samples, diag = sim.run(seconds=1.0, waveform='stall=700')
        self.assertEqual(diag['dma_timeouts'], 1)
        self.assertEqual(diag['recoveries'], 1)
        self.assertGreater(diag['frames_skipped'], 15)  # ~20 ms until the timeout
        times = [t for t, _ in fast_times(samples, 6)]
        late = [t for t in times if t > times[0] + 600_000]
        self.assertEqual(set(b - a for a, b in zip(late, late[1:])), {1000})

    def test_spurious_ready_edges_are_ignored(self):
        samples, diag = sim.run(seconds=1.0, waveform='spurious=37')
        for key in ('status_faults', 'recoveries', 'unexpected', 'scans_timed_out', 'frames_skipped'):
            self.assertEqual(diag[key], 0, key)
        self.assert_strict_kilohertz(fast_times(samples, 0), 0)

    def test_analog_enable_failure_stops_before_any_spi(self):
        _, diag = sim.run(seconds=0.1, waveform='en12v_low')
        self.assertEqual(diag, {'init_failed': 1, 'spi_transfers': 0})

    def test_gain_and_filter_per_scan(self):
        # The fast pair always converts at unity gain; only the P6 slot uses x16.
        # TEMP alone uses the MCLK/4 clock. Every START follows a full burst.
        _, diag = sim.run(seconds=0.2, waveform='trace', kg50=1)
        seen = {}
        for scan, config1, config2 in diag['configs']:
            seen.setdefault(scan, set()).add((config1, config2))
        self.assertEqual(seen[0x41], {(0x08, 0xCF)})       # FILL: OSR128, gain 1
        self.assertEqual(seen[1 << 9], {(0x0C, 0xEF)})     # P6: OSR256, gain 16
        self.assertEqual(seen[1 << 12], {(0x84, 0xCF)})    # TEMP: MCLK/4, OSR64
        for channel in (1, 2, 3, 4, 5, 7):
            self.assertEqual(seen[1 << channel], {(0x0C, 0xCF)})
        _, diag = sim.run(seconds=0.2, waveform='trace', kg50=1, launch=(0, 1000))
        launch = {(scan, c1, c2) for scan, c1, c2 in diag['configs'][10:]}
        self.assertIn((0x41, 0x0C, 0xCF), launch)          # LAUNCH: fast pair OSR256
        self.assertIn((1 << 9, 0x04, 0xEF), launch)        # P6 at OSR64
        self.assertNotIn(1 << 12, {scan for scan, _, _ in launch})


if __name__ == '__main__':
    unittest.main()
