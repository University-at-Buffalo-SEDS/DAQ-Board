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
        decode = adc[adc.index('static int32_t mcp3564r_decode_code'):adc.index('UINT mcp3564r_get_sample')]
        calibration = (ROOT / 'Core/Src/daq_calibration.c').read_text().split('float daq_calibration_apply_kg50', 1)[1]
        harness = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include "daq_timestamp.h"
#include "board_uptime.h"
#include "daq_filter.h"
#include "mcp3564r_board_config.h"
#include "daq_rates.h"
#include "daq_median.h"
#include "daq_channel_stats.h"
#include "daq_over_range.h"
static uint32_t primask;
static uint32_t __get_PRIMASK(void) { return primask; }
static void __disable_irq(void) { primask = 1U; }
static void __set_PRIMASK(uint32_t v) { primask = v; }
#define TX_SUCCESS 0
#define SD_RAW_CHANNEL_INPUT_VOLTAGE 10U
#define SD_RAW_CHANNEL_INPUT_CURRENT 11U
#define SD_RAW_CHANNEL_VDDA 12U
typedef struct { uint32_t monotonic_ms; float input_voltage_v, input_current_a, pin_v[2]; int32_t code[2];
                 float vdda_v; int32_t vrefint_code; } daq_supply_sample_t;
static unsigned g_daq_nonzero_raw_sample_count, g_daq_raw_samples_drained_count;
typedef struct { float kg1000_slope, kg1000_intercept, iadc_slope, iadc_intercept, kg50[7], thermal[4], filter_tau_ms[2]; } daq_calibration_t;
typedef struct {
  uint8_t ext_adc_sample_valid, ext_adc_channel;
  uint64_t ext_adc_sample_us;
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
  uint64_t sample_us;
} mcp3564r_sample_t;
typedef struct {
  uint8_t channel;
  uint32_t monotonic_ms, record_sequence;
  uint64_t network_unix_ms;
  int32_t raw_adc_code;
  float raw_value, calibrated_value, adc_temperature_c;
  int32_t adc_temperature_code;
  int8_t over_range;
  uint16_t sample_us;
} sd_raw_adc_record_t;
static mcp3564r_sample_t queued[8];
static unsigned next, queued_count;
static unsigned mcp3564r_pending_samples(void) { return queued_count - next; }
static unsigned mcp3564r_get_sample(mcp3564r_sample_t *out) { *out = queued[next++]; return 0; }
static uint64_t daq_log_unix_ms_cached(void) { return 10000; }
uint64_t board_uptime_ticks(void) { return 100; }
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
      .ext_adc_sample_us=90000, .ext_adc_code=10,
      .ext_adc_voltage_v=1, .ext_adc_loadcell_kg1000=10 };
  queued[0]=(mcp3564r_sample_t){1,0,91,100,1,100,25,305656,91000};
  queued[1]=(mcp3564r_sample_t){1,1,92,20,2,20,26,306695,92250};
  queued[2]=(mcp3564r_sample_t){1,0,93,200,3,200,27,307734,93500};
  queued_count=3;
  sd_raw_adc_record_t records[5];
  daq_loadcell_window_t window;
  daq_ovr_window_t ovr;
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window,1,NULL,NULL,&ovr)==4);
  assert(window.count[0]==2 && window.count[1]==2);
  assert(window.sum[0]==300 && window.sum[1]==30);
  assert(window.voltage_sum[0]==4 && window.voltage_sum[1]==3);
  assert(snapshot.ext_adc_loadcell_kg1000==150);
  assert(snapshot.ext_adc_voltage_v==2);
  assert(records[0].channel==1 && records[0].raw_value==10);
  assert(records[0].calibrated_value==24 && records[0].network_unix_ms==9990);
  assert(records[1].channel==0 && records[1].calibrated_value==201);
  assert(records[1].adc_temperature_c==25 && records[1].adc_temperature_code==305656);
  /* Microsecond capture time: the sub-millisecond part rides in the record. */
  assert(records[0].sample_us==0 && records[2].sample_us==250 && records[3].sample_us==500);
  assert(records[2].calibrated_value==54 && records[3].calibrated_value==401);
  assert(g_daq_raw_samples_drained_count==4);
  /* A KG50-only batch cannot become a KG1000 report. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=1,
      .ext_adc_voltage_v=0.6f, .ext_adc_loadcell_kg1000=7 };
  assert(daq_drain_ext_adc(&snapshot,NULL,4,&cal,&window,1,NULL,NULL,&ovr)==1);
  assert(!snapshot.ext_adc_sample_valid && window.count[0]==0 && window.count[1]==1);
  assert(window.sum[1]==7 && window.voltage_sum[1]==0.6f);
  /* Auxiliary samples retain ADC volts and divider-adjusted connector volts,
     with no load-cell calibration/filter or mass-channel count contamination. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=7,
      .ext_adc_voltage_v=0.6f, .ext_adc_loadcell_kg1000=0.6f };
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window,1,NULL,NULL,&ovr)==1);
  assert(window.count[0]==0 && window.count[1]==0 && window.count[7]==1);
  assert(!snapshot.ext_adc_sample_valid && records[0].channel==7);
  assert(fabsf(records[0].calibrated_value-6.6f)<1e-6f);
  assert(records[0].raw_value==0.6f);
  /* Default HVAIN1 pressure records retain connector volts plus psi, while
     pressure never enters either load-cell calibration/filter. */
  snapshot.ext_adc_channel=6; snapshot.ext_adc_sample_valid=1;
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window,1,NULL,NULL,&ovr)==1);
  assert(window.count[6]==1 && window.count[0]==0 && window.count[1]==0);
  assert(fabsf(records[0].raw_value-6.6f)<1e-6f);
  assert(fabsf(records[0].calibrated_value-1320.0f)<0.001f);
  /* P6 differential samples remain individual signed volts in SD and their
     own window; they never inherit AMP2's raw scale or calibration. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=9,
      .ext_adc_code=-100, .ext_adc_voltage_v=-0.002f, .ext_adc_loadcell_kg1000=-0.002f };
  assert(daq_drain_ext_adc(&snapshot,records,4,&cal,&window,1,NULL,NULL,&ovr)==1);
  assert(window.count[9]==1 && window.count[0]==0 && window.count[1]==0);
  assert(records[0].channel==9 && records[0].raw_adc_code==-100);
  assert(records[0].raw_value==-0.002f && records[0].calibrated_value==-0.002f);
  /* Invalid diagnostic channels are discarded, with bounded draining. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=15 };
  next=0; queued_count=3;
  assert(daq_drain_ext_adc(&snapshot,records,1,&cal,&window,1,NULL,NULL,&ovr)==1);
  assert(next==1 && records[0].channel==0);
  /* Supply reading: the last three records of the batch (VMON, IMON, VDDA),
     same clocks and sequence as the ADC records, never in a load-cell window. */
  daq_supply_sample_t supply={.monotonic_ms=95,.input_voltage_v=14.2f,.input_current_a=0.5f,
      .pin_v={1.29f,0.24f},.code={6400,1190},.vdda_v=4.19f,.vrefint_code=1181};
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=0,
      .ext_adc_sample_us=94000, .ext_adc_loadcell_kg1000=1 };
  next=0; queued_count=3;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,1,NULL,&supply,&ovr)==5);
  assert(records[0].channel==0 && records[1].channel==0 && next==1);
  assert(records[2].channel==SD_RAW_CHANNEL_INPUT_VOLTAGE && records[3].channel==SD_RAW_CHANNEL_INPUT_CURRENT);
  assert(records[2].record_sequence==records[1].record_sequence+1 && records[3].record_sequence==records[2].record_sequence+1);
  assert(records[2].monotonic_ms==95 && records[2].network_unix_ms==9995 && records[3].network_unix_ms==9995);
  assert(records[2].raw_adc_code==6400 && records[2].raw_value==1.29f && records[2].calibrated_value==14.2f);
  assert(records[3].raw_adc_code==1190 && records[3].raw_value==0.24f && records[3].calibrated_value==0.5f);
  assert(records[4].channel==SD_RAW_CHANNEL_VDDA && records[4].record_sequence==records[3].record_sequence+1);
  assert(records[4].raw_adc_code==1181 && records[4].raw_value==4.19f && records[4].calibrated_value==4.19f);
  assert(window.count[0]==2 && window.count[1]==0);
  /* VDDA is logged again only after a 5 mV move or 100 ms. */
  snapshot=(daq_snapshot_t){0}; next=queued_count=0;
  supply.monotonic_ms=99; supply.vdda_v=4.192f;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,1,NULL,&supply,&ovr)==2);
  supply.monotonic_ms=103; supply.vdda_v=4.199f;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,1,NULL,&supply,&ovr)==3 && records[2].raw_value==4.199f);
  supply.monotonic_ms=202;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,1,NULL,&supply,&ovr)==2);
  supply.monotonic_ms=203;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,1,NULL,&supply,&ovr)==3 && records[2].channel==SD_RAW_CHANNEL_VDDA);
  /* Over range (|code| > 2^23-1, daq_over_range.h): the SD record keeps the
     code, the linear value and the flag, with an infinite calibrated value;
     nothing enters a window sum, mean or filter; the window counts it. */
  snapshot=(daq_snapshot_t){ .ext_adc_sample_valid=1, .ext_adc_channel=0, .ext_adc_sample_us=96000,
      .ext_adc_code=0xB40000, .ext_adc_voltage_v=3.375f, .ext_adc_loadcell_kg1000=3.375f };
  queued[0]=(mcp3564r_sample_t){1,9,96,-8388609,-0.150000018f,-0.150000018f,25,305656,96250};
  queued[1]=(mcp3564r_sample_t){1,6,96,8388608,2.4f,2.4f,25,305656,96500};
  queued[2]=(mcp3564r_sample_t){1,0,96,8388607,2.39999986f,2.39999986f,25,305656,96750};
  queued[3]=(mcp3564r_sample_t){1,9,97,-8388607,-0.149999991f,-0.149999991f,25,305656,97000};
  next=0; queued_count=4;
  assert(daq_drain_ext_adc(&snapshot,records,5,&cal,&window,9,NULL,NULL,&ovr)==5);
  assert(records[0].over_range==1 && records[0].raw_adc_code==0xB40000 && records[0].raw_value==3.375f);
  assert(isinf(records[0].calibrated_value) && records[0].calibrated_value>0);
  assert(records[1].over_range==-1 && records[1].channel==9 && isinf(records[1].calibrated_value) && records[1].calibrated_value<0);
  assert(records[2].over_range==1 && records[2].channel==6 && fabsf(records[2].raw_value-26.4f)<1e-4f);
  assert(isinf(records[2].calibrated_value) && records[2].calibrated_value>0);
  /* Exactly full scale (+/-(2^23-1)) is in range. */
  assert(records[3].over_range==0 && records[4].over_range==0 && isfinite(records[3].calibrated_value));
  assert(window.count[0]==1 && window.sum[0]==2.39999986f && window.count[6]==0 && window.count[9]==1);
  assert(ovr.count[0]==1 && ovr.count[6]==1 && ovr.count[DAQ_OVR_BIT_P6]==1);
  assert(ovr.last[0]==0 && ovr.dir[0]==1 && ovr.last[6]==1 && ovr.last[DAQ_OVR_BIT_P6]==0 && ovr.dir[DAQ_OVR_BIT_P6]==-1);
  assert(ovr.last[1]==2 && ovr.count[1]==0);
  assert(snapshot.ext_adc_sample_valid && snapshot.ext_adc_code==8388607);
  /* Majority over range: the fault checks see the marker; otherwise the in-range mean. */
  assert(isinf(daq_window_value(&window,&ovr,6,1)) && daq_window_value(&window,&ovr,6,1)>0);
  assert(isinf(daq_window_value(&window,&ovr,0,0)));
  assert(daq_window_value(&window,&ovr,9,0)<0 && isinf(daq_window_value(&window,&ovr,9,0)));
  assert(isnan(daq_window_value(&window,&ovr,1,0)));
  cal.kg50[0]=1; cal.kg50[1]=2; cal.kg50[2]=3;
  cal.kg50[3]=4; cal.kg50[4]=5; cal.kg50[5]=1; cal.kg50[6]=6;
  assert(daq_calibration_apply_kg50(&cal,3)==123);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'kg50-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary), '-lm'],
                                    input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)

    def test_board_voltage_scale_and_legacy_raw_formula(self):
        harness = r'''
#include <assert.h>
#include <math.h>
#include <string.h>
#include <string.h>
#include "mcp3564r_board_config.h"
int main(void) {
  assert(DAQ_ACQ_FAST_SCAN == (1U | (1U<<6)));
  assert(((DAQ_ACQ_CONFIG2_GAIN1 >> 3) & 7) == 1 && ((DAQ_ACQ_CONFIG2_GAIN16 >> 3) & 7) == 5);
  assert(mcp3564r_code_to_voltage(0) == 0);
  /* AMP2's nominal 0.6 V bias is one quarter of the 2.4 V reference. */
  assert(mcp3564r_code_to_voltage(2097152) == 0.6f);
  assert(mcp3564r_code_to_voltage(8388608) == 2.4f);
  assert(mcp3564r_code_to_voltage(-8388608) == -2.4f);
  const int32_t codes[] = {0, 1, -1, 790001, 2097152, 8388607, -8388608, 16777215};
  for (unsigned i = 0; i < sizeof(codes) / sizeof(codes[0]); ++i) {
    /* The legacy formula is kept bit-exact for calibration migration. */
    const float historical = (((float)codes[i] * 2.2104f) / 16777216.0f) * 2.0f / 16.0f;
    const float current = mcp3564r_code_to_legacy_raw_value(codes[i]);
    assert(memcmp(&historical, &current, sizeof(float)) == 0);
  }
  /* The per-board reference setting scales every conversion: this board's
   * drifted ~2.2 V part. Full scale is VREF; the legacy formula is fixed. */
  const float t24 = mcp3564r_code_to_temperature(306000);
  g_mcp3564r_vref_v = 2.2f;
  assert(mcp3564r_code_to_voltage(8388608) == 2.2f && mcp3564r_code_to_voltage(-8388608) == -2.2f);
  assert(fabsf(mcp3564r_code_to_voltage(0xB40000) - 3.09375f) < 1e-6f); /* the fire plateau: 1.40625 FS */
  assert(fabsf((mcp3564r_code_to_temperature(306000) + 269.13f) / (t24 + 269.13f) - 2.2f / 2.4f) < 1e-6f);
  assert(mcp3564r_code_to_legacy_raw_value(8388608) == (((float)8388608 * 2.2104f) / 16777216.0f) * 2.0f / 16.0f);
  assert(MCP3564R_VREF_MIN_V == 1.9f && MCP3564R_VREF_MAX_V == 2.6f && MCP3564R_NOMINAL_VREF_V == 2.4f);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary = pathlib.Path(tmp) / 'mcp3564r-scaling-test'
            result = subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary)],
                                    input=harness, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(binary)], check=True)
