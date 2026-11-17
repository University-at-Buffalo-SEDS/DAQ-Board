"""Launch-window supply logging and the SD-only load-cell stream.

Runs the production drain (daq_drain_ext_adc), the acquisition loop's supply
scheduling block and the production raw CSV formatter over 3 s of simulated
5.5 kHz two-channel conversions with a launch window from 1 s to 2 s:

- the launch window holds input_voltage_v/input_current_a records every
  DAQ_SUPPLY_LAUNCH_PERIOD_MS (>= 100 Hz), in the load-cell record stream with
  consecutive record_sequence and the same uptime clock; outside it they keep
  the 10 Hz analog cadence;
- every KG1000/KG50 network value the drain emits is rebuilt bit for bit from
  the formatted SD raw records alone (independent Python boxcar).
"""
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def boxcar_from_csv(lines, sensors, period_ms):
    """Independent reading of SD rows: mean of each fixed uptime bin."""
    out, state = [], None
    for row in lines:
        fields = row.split(',')
        if fields[2] not in sensors:
            continue
        value = struct.unpack('<f', struct.pack('<f', float(fields[5])))[0]
        uptime = int(fields[10])
        bin_ = uptime // period_ms
        if state and state[0] != bin_:
            out.append((state[3], struct.unpack('<I', struct.pack('<f', state[1] / state[2]))[0]))
            state = None
        state = (bin_, (state[1] if state else 0.0) + value, (state[2] if state else 0) + 1, uptime)
    return out


class LaunchSupplyLogTests(unittest.TestCase):
    def run_model(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        drain = source[source.index('typedef struct\n'):source.index('volatile uint32_t g_daq_sd_snapshot_row_drop_count;')]
        start = source.index('#if (DISABLE_SD_CARD == 0U)\n    /* Launch window: VMAIN')
        supply_block = source[start:source.index('    stage_started = daq_profile_stage(0U', start)]
        program = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "daq_timestamp.h"
#include "board_uptime.h"
#include "daq_filter.h"
#include "daq_rates.h"
#include "daq_downsample.h"
#include "daq_calibration.h"
#include "mcp3564r_board_config.h"
#include "sd_raw_record_format.h"
#define DISABLE_SD_CARD 0U
#define TX_SUCCESS 0U
#define SD_RAW_CHANNEL_INPUT_VOLTAGE 10U
#define SD_RAW_CHANNEL_INPUT_CURRENT 11U
typedef unsigned UINT;
typedef struct { uint32_t monotonic_ms; float input_voltage_v, input_current_a, pin_v[2]; int32_t code[2]; } daq_supply_sample_t;
typedef struct {
  uint64_t monotonic_ms; uint8_t analog_sample_fresh;
  uint8_t ext_adc_sample_valid, ext_adc_channel;
  uint64_t ext_adc_monotonic_ms;
  int32_t ext_adc_code;
  float ext_adc_voltage_v, ext_adc_loadcell_kg1000, ext_adc_temp_c;
  int32_t ext_adc_temp_code;
} daq_snapshot_t;
typedef struct {
  uint8_t sample_valid, channel;
  uint64_t monotonic_ms;
  int32_t code;
  float voltage_v, raw_value, temperature_c;
  int32_t temperature_code;
} mcp3564r_sample_t;
typedef struct {
  uint64_t network_unix_ms;
  uint32_t monotonic_ms, record_sequence;
  int32_t raw_adc_code;
  float raw_value, calibrated_value, adc_temperature_c;
  int32_t adc_temperature_code;
  uint8_t channel;
} sd_raw_adc_record_t;
static unsigned g_daq_nonzero_raw_sample_count, g_daq_raw_samples_drained_count;
float daq_calibration_temperature_raw(const daq_calibration_t *c, unsigned ch, float raw, float t)
{ (void)c; (void)ch; (void)t; return raw; }
float daq_calibration_apply_kg50(const daq_calibration_t *c, float v) { (void)c; return v * 10.0f; }
/* 5.5 kHz conversions alternating CH0/CH1, queued by capture time. */
static uint64_t now_us, next_us;
static unsigned conversions;
static mcp3564r_sample_t make(void) {
  const unsigned i = conversions++;
  const int32_t code = (int32_t)((i * 2654435761u) >> 12) - 524288;
  const float v = (float)code * 2.4f / 8388608.0f;
  return (mcp3564r_sample_t){1, (uint8_t)(i & 1u), next_us / 1000u, code, v, v, 25.0f, 7};
}
static unsigned mcp3564r_pending_samples(void) { return next_us <= now_us; }
static UINT mcp3564r_get_sample(mcp3564r_sample_t *s) { *s = make(); next_us += 180; return TX_SUCCESS; }
static uint64_t daq_log_unix_ms_cached(void) { return 1800000000000ULL + now_us / 1000u; }
uint64_t board_uptime_ticks(void) { return now_us / 1000u; }
/* Board supply: a reading at each 100 ms analog pass or on request. */
static daq_supply_sample_t pending_supply; static uint8_t supply_fresh_flag; static unsigned supply_reads;
static void note_supply(void) {
  const uint32_t ms = (uint32_t)(now_us / 1000u);
  pending_supply = (daq_supply_sample_t){ms, 14.2f - (float)(ms % 7u) * 0.5f, 0.4f, {1.29f, 0.24f}, {6400 + (int32_t)ms, 1190}};
  supply_fresh_flag = 1;
}
static UINT daq_board_sample_supply(void) { ++supply_reads; note_supply(); return TX_SUCCESS; }
static uint8_t daq_board_take_supply(daq_supply_sample_t *s) {
  if (!supply_fresh_flag) return 0; *s = pending_supply; supply_fresh_flag = 0; return 1;
}
static uint8_t sd_card_launch_window_active(void) {
  const uint64_t ms = now_us / 1000u; return ms >= 500u && ms < 2000u;
}
''' + drain + r'''
int main(void) {
  daq_calibration_t cal = {.kg1000_slope = 2, .kg50 = {0, 1}};
  sd_raw_adc_record_t records[DAQ_RAW_BATCH_CAPACITY];
  sd_raw_format_cache_t cache = {0};
  uint32_t last_supply_ms = 0U, last_analog = 0U;
  uint8_t started = 0U;
  char line[256];
  for (now_us = 2000; now_us < 3000000; now_us += 2000) {
    daq_snapshot_t snapshot = {.monotonic_ms = now_us / 1000u};
    if (!started || (uint32_t)snapshot.monotonic_ms - last_analog >= DAQ_ANALOG_REPORT_PERIOD_MS) {
      started = 1U; last_analog = (uint32_t)snapshot.monotonic_ms; snapshot.analog_sample_fresh = 1U; note_supply();
    }
    if (mcp3564r_pending_samples()) {
      mcp3564r_sample_t first; (void)mcp3564r_get_sample(&first);
      snapshot.ext_adc_sample_valid = 1; snapshot.ext_adc_channel = first.channel;
      snapshot.ext_adc_monotonic_ms = first.monotonic_ms; snapshot.ext_adc_code = first.code;
      snapshot.ext_adc_voltage_v = first.voltage_v; snapshot.ext_adc_loadcell_kg1000 = first.raw_value;
      snapshot.ext_adc_temp_c = 25.0f; snapshot.ext_adc_temp_code = 7;
    }
''' + supply_block + r'''
    daq_loadcell_window_t window;
    daq_loadcell_bins_t bins;
    const uint16_t n = daq_drain_ext_adc(&snapshot, records, DAQ_RAW_BATCH_CAPACITY, &cal, &window,
                                         1U, &bins, supply_fresh ? &supply : NULL);
    for (uint16_t i = 0; i < n; ++i) {
      const sd_raw_adc_record_t *r = &records[i];
      const char *names[] = {"kg1000_v", "kg50_v"};
      const char *sensor = r->channel < 2u ? names[r->channel] : r->channel == SD_RAW_CHANNEL_INPUT_VOLTAGE ?
          "input_voltage_v" : r->channel == SD_RAW_CHANNEL_INPUT_CURRENT ? "input_current_a" : "other";
      assert(sd_format_csv_raw_record(line, sizeof line, r->network_unix_ms, r->monotonic_ms, sensor,
          r->raw_adc_code, r->raw_value, r->calibrated_value, r->adc_temperature_c,
          r->adc_temperature_code, r->record_sequence, &cache) > 0);
      fputs(line, stdout);
    }
    uint32_t bits;
    if (bins.kg1000.ready) { memcpy(&bits, &bins.kg1000.value, 4); printf("#bin,kg1000,%llu,%u\n", (unsigned long long)bins.kg1000.captured_ms, bits); }
    if (bins.kg50.ready) { memcpy(&bits, &bins.kg50.value, 4); printf("#bin,kg50,%llu,%u\n", (unsigned long long)bins.kg50.captured_ms, bits); }
  }
  printf("#supply_reads,%u\n", supply_reads);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'supply'
            build = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                                    '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-lm', '-o', str(exe)],
                                   input=program, text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            return subprocess.check_output([str(exe)], text=True).splitlines()

    def test_launch_window_logs_supply_aligned_with_load_cells(self):
        lines = self.run_model()
        rows = [line.rstrip('\r').split(',') for line in lines if not line.startswith('#')]
        sequences = [int(row[12]) for row in rows]
        # One record stream: supply and load-cell rows share consecutive sequence numbers.
        self.assertEqual(sequences, list(range(sequences[0], sequences[0] + len(sequences))))
        volts = [int(row[10]) for row in rows if row[2] == 'input_voltage_v']
        amps = [int(row[10]) for row in rows if row[2] == 'input_current_a']
        self.assertEqual(volts, amps)
        window = [t for t in volts if 1000 <= t < 2000]
        gaps = [b - a for a, b in zip(window, window[1:])]
        self.assertGreaterEqual(len(window), 1000 // 10, 'at least 100 Hz in the launch window')
        self.assertLessEqual(max(gaps), 4, gaps)
        outside = [t for t in volts if t < 450 or t >= 2100]
        self.assertTrue(all(b - a == 100 for a, b in zip(outside, outside[1:]) if b < 450 or a >= 2100), outside)
        # Same uptime clock: each supply row sits within one acquisition cycle of
        # the load-cell rows around it.
        for index, row in enumerate(rows):
            if row[2] != 'input_voltage_v' or not 1000 <= int(row[10]) < 2000:
                continue
            neighbours = [int(r[10]) for r in rows[max(0, index - 3):index + 4] if r[2].startswith('kg')]
            self.assertTrue(neighbours and all(abs(int(row[10]) - t) <= 2 for t in neighbours), (row, neighbours))
            self.assertEqual(row[7], 'network')
            f32 = lambda x: struct.unpack('<f', struct.pack('<f', x))[0]
            self.assertEqual(f32(float(row[6])), f32(f32(14.2) - (int(row[10]) % 7) * 0.5))

    def test_network_load_cell_values_rebuild_exactly_from_sd_rows(self):
        lines = self.run_model()
        csv_rows = [line.rstrip('\r') for line in lines if not line.startswith('#')]
        for sensor in ('kg1000', 'kg50'):
            emitted = [(int(t), int(bits)) for _, name, t, bits in
                       (line.split(',') for line in lines if line.startswith('#bin,'))
                       if name == sensor]
            rebuilt = boxcar_from_csv(csv_rows, {sensor + '_v'}, 4)
            self.assertGreater(len(emitted), 700)
            # The drain reports only the newest closed bin per cycle; every
            # reported value is one of the SD-rebuilt bins, bit for bit.
            self.assertTrue(set(emitted) <= set(rebuilt), sorted(set(emitted) - set(rebuilt))[:3])


if __name__ == '__main__':
    unittest.main()
