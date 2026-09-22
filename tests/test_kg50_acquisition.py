"""Execute the production channel drain and ADC decoder with interleaved samples."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class Kg50AcquisitionTests(unittest.TestCase):
    def test_interleaved_channels_preserve_raw_values_calibration_and_counts(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        drain = source[source.index('typedef struct\n'):source.index('static void daq_store_snapshot_csv')]
        adc = (ROOT / 'Core/Src/mcp3564r.c').read_text()
        decode = adc[adc.index('static int32_t mcp3564r_decode_code'):adc.index('static void mcp3564r_store_raw32')]
        calibration = (ROOT / 'Core/Src/daq_calibration.c').read_text().split('float daq_calibration_apply_kg50', 1)[1]
        harness = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "daq_timestamp.h"
#include "daq_filter.h"
#define TX_SUCCESS 0
static unsigned g_daq_nonzero_raw_sample_count, g_daq_raw_samples_drained_count;
typedef struct { float kg1000_slope, kg1000_intercept, iadc_slope, iadc_intercept, kg50[7], thermal[4], filter_tau_ms[2]; } daq_calibration_t;
typedef struct {
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
  uint8_t channel;
  uint32_t monotonic_ms;
  uint64_t network_unix_ms;
  int32_t raw_adc_code;
  float raw_value, calibrated_value, adc_temperature_c;
  int32_t adc_temperature_code;
} sd_raw_adc_record_t;
static mcp3564r_sample_t queued[8];
static unsigned next, queued_count;
static unsigned mcp3564r_pending_samples(void) { return queued_count - next; }
static unsigned mcp3564r_get_sample(mcp3564r_sample_t *out) { *out = queued[next++]; return 0; }
static uint64_t telemetry_unix_ms(void) { return 10000; }
static uint64_t telemetry_now_ms(void) { return 100; }
''' + 'float daq_calibration_apply_kg50' + calibration + decode + drain + r'''
int main(void) {
  /* Tagged 25-bit values: sign extension must not consume the channel nibble. */
  assert(mcp3564r_decode_code(0x10123456U) == 0x123456);
  assert(mcp3564r_decode_code(0x1FFFFFFFU) == -1);
  assert(mcp3564r_decode_code(0x01800000U) == -8388608);
  assert(mcp3564r_decode_code(0x00800000U) == 8388608);
  daq_calibration_t cal = { .kg1000_slope=2, .kg1000_intercept=1,
                           .kg50={4,3,0,0,0,0,10} };
  daq_snapshot_t snapshot = { .ext_adc_sample_valid=1, .ext_adc_channel=1,
      .ext_adc_monotonic_ms=90, .ext_adc_code=10,
      .ext_adc_voltage_v=1, .ext_adc_loadcell_kg1000=10 };
  queued[0]=(mcp3564r_sample_t){1,0,91,100,1,100,25,305656};
  queued[1]=(mcp3564r_sample_t){1,1,92,20,2,20,26,306695};
  queued[2]=(mcp3564r_sample_t){1,0,93,200,3,200,27,307734};
  queued_count=3;
  sd_raw_adc_record_t records[4];
  daq_loadcell_window_t window;
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window)==4);
  assert(window.count[0]==2 && window.count[1]==2);
  assert(window.sum[0]==300 && window.sum[1]==30);
  assert(window.voltage_sum[0]==4 && window.voltage_sum[1]==3);
  assert(snapshot.ext_adc_loadcell_kg1000==150);
  assert(snapshot.ext_adc_voltage_v==2);
  assert(records[0].channel==1 && records[0].raw_value==10);
  assert(records[0].calibrated_value==24 && records[0].network_unix_ms==9990);
  assert(records[1].channel==0 && records[1].calibrated_value==201);
  assert(records[1].adc_temperature_c==25 && records[1].adc_temperature_code==305656);
  assert(records[2].calibrated_value==54 && records[3].calibrated_value==401);
  assert(g_daq_raw_samples_drained_count==4);
  /* A KG50-only batch cannot become a KG1000 report. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=1,
      .ext_adc_voltage_v=0.6f, .ext_adc_loadcell_kg1000=7 };
  assert(daq_drain_ext_adc(&snapshot,NULL,4,&cal,&window)==1);
  assert(!snapshot.ext_adc_sample_valid && window.count[0]==0 && window.count[1]==1);
  assert(window.sum[1]==7 && window.voltage_sum[1]==0.6f);
  /* Invalid diagnostic channels are discarded, with bounded draining. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=15 };
  next=0; queued_count=3;
  assert(daq_drain_ext_adc(&snapshot,records,1,&cal,&window)==1);
  assert(next==1 && records[0].channel==0);
  cal.kg50[0]=1; cal.kg50[1]=2; cal.kg50[2]=3;
  cal.kg50[3]=4; cal.kg50[4]=5; cal.kg50[5]=1; cal.kg50[6]=6;
  assert(daq_calibration_apply_kg50(&cal,3)==123);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'kg50-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary)],
                                    input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

    def test_board_voltage_scale_preserves_existing_calibration_inputs(self):
        harness = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include <string.h>
#include "mcp3564r_board_config.h"
int main(void) {
  assert(MCP3564R_BOARD_SCAN == 3);
  assert(((MCP3564R_BOARD_CONFIG2 >> 3) & 7) == 1);
  assert(mcp3564r_code_to_voltage(0) == 0);
  /* AMP2's nominal 0.6 V bias is one quarter of the 2.4 V reference. */
  assert(mcp3564r_code_to_voltage(2097152) == 0.6f);
  assert(mcp3564r_code_to_voltage(8388608) == 2.4f);
  assert(mcp3564r_code_to_voltage(-8388608) == -2.4f);
  const int32_t codes[] = {0, 1, -1, 790001, 2097152, 8388607, -8388608, 16777215};
  for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
    const float historical = (((float)codes[i] * 2.2104f) / 16777216.0f) * 2.0f / 16.0f;
    const float current = mcp3564r_code_to_raw_value(codes[i]);
    assert(memcmp(&historical, &current, sizeof(float)) == 0);
  }
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'mcp3564r-scaling-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary)],
                                    input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
