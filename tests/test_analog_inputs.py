"""Execute the production STM32 analog drivers against channel-selecting HAL mocks."""
import json
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]

HAL = r'''
#ifndef MOCK_HAL_H
#define MOCK_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK, HAL_ERROR, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef struct {uint32_t CHSELR, CR, ISR;} ADC_Instance;
typedef struct {ADC_Instance *Instance; uint32_t State;} ADC_HandleTypeDef;
typedef struct {uint32_t Channel, Rank, SamplingTime, SingleDiff, OffsetNumber, Offset,
 OffsetRightShift, OffsetSignedSaturation, OffsetSaturation, OffsetSign;} ADC_ChannelConfTypeDef;
#define ADC_SINGLE_ENDED 0
#define ADC_DIFFERENTIAL_ENDED 1
#define ADC_REGULAR_RANK_1 1
#define ADC4_RANK_CHANNEL_NUMBER 2
#define ADC_SAMPLETIME_391CYCLES 391
#define ADC4_SAMPLINGTIME_COMMON_1 0
#define ADC_OFFSET_NONE 0
#define ADC_OFFSET_SIGN_NEGATIVE 0
#define ADC_CALIB_OFFSET 0
#define DISABLE 0
#define WRITE_REG(r,v) ((r)=(v))
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *, ADC_ChannelConfTypeDef *);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *,uint32_t,uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
#endif
'''
HARNESS = r'''
#include <assert.h>
#include <math.h>
#include "daq_adc1.h"
#include "daq_adc4.h"
static ADC_Instance adc1,adc4;
ADC_HandleTypeDef hadc1={.Instance=&adc1},hadc4={.Instance=&adc4};
static unsigned selected, calls, fail_channel=99;
static uint32_t samples[18];
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *a,ADC_ChannelConfTypeDef *c) {
 assert(c->SingleDiff==ADC_SINGLE_ENDED);
 selected=c->Channel;
 if(a==&hadc4) {
   a->Instance->CHSELR |= 1U<<c->Channel;
   /* Actual ADC4 selects the lowest enabled channel first. */
   selected=__builtin_ctz(a->Instance->CHSELR);
 }
 calls++;
 return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *a) {(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *a) {(void)a;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *a,uint32_t t) {
 (void)a;(void)t;return selected==fail_channel ? HAL_TIMEOUT : HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *a,uint32_t o,uint32_t d) {
 (void)a;(void)o;assert(d==ADC_SINGLE_ENDED);return HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *a) {(void)a;return samples[selected];}
int main(void) {
 for(unsigned i=0;i<18;i++)samples[i]=i*100;
 samples[12]=8192; samples[11]=9000; samples[16]=1000; samples[17]=5500;
 assert(daq_adc1_init()==TX_SUCCESS && daq_adc4_init()==TX_SUCCESS);
 daq_adc1_sample_t a;
 assert(daq_adc1_sample(&a)==TX_SUCCESS && calls==8);
 for(unsigned i=0;i<4;i++)assert(fabsf(a.sar_inputs_v[i]-(i+5)*100*3.3f/16383)<1e-6f);
 assert(fabsf(a.current_sense_v[0]-8192*3.3f/16383)<1e-6f);
 assert(fabsf(a.current_sense_v[1]-9000*3.3f/16383)<1e-6f);
 assert(fabsf(a.current_sense_a[0])<0.003f);
 assert(fabsf(a.input_voltage_v-5500*3.3f/16383*11)<1e-5f);
 assert(fabsf(a.input_current_a-1000*3.3f/16383/(0.00007828f*6200))<1e-5f);
 fail_channel=6;
 assert(daq_adc1_sample(&a)==TX_NOT_DONE && isnan(a.sar_inputs_v[1]));
 assert(isfinite(a.sar_inputs_v[2]) && isfinite(a.input_voltage_v));
 fail_channel=99;
 daq_adc4_sample_t b;
 adc4.CHSELR=0x1e; /* CubeMX initially selects all four channels. */
 assert(daq_adc4_sample(&b)==TX_SUCCESS);
 for(unsigned i=0;i<4;i++)assert(fabsf(b.analog_inputs_v[i]-(i+1)*100*3.3f/4095)<1e-6f);
 fail_channel=2;
 assert(daq_adc4_sample(&b)==TX_NOT_DONE && isnan(b.analog_inputs_v[1]));
 assert(isfinite(b.analog_inputs_v[2]));
 assert(daq_adc1_sample(NULL)==TX_PTR_ERROR && daq_adc4_sample(NULL)==TX_PTR_ERROR);
}
'''


class AnalogInputTests(unittest.TestCase):
    def test_channel_selection_scaling_and_partial_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = pathlib.Path(tmp)
            (p/'stm32u5xx_hal.h').write_text(HAL + '\n'.join(f'#define ADC_CHANNEL_{i} {i}' for i in range(18)))
            (p/'main.h').write_text('#include "stm32u5xx_hal.h"\n')
            (p/'tx_api.h').write_text('typedef unsigned UINT;\n#define TX_SUCCESS 0\n#define TX_PTR_ERROR 1\n#define TX_NOT_DONE 2\n')
            (p/'test.c').write_text(HARNESS)
            run = subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(p),'-I',str(ROOT/'Core/Inc'),str(ROOT/'Core/Src/daq_adc1.c'),str(ROOT/'Core/Src/daq_adc4.c'),str(p/'test.c'),'-o',str(p/'test')],capture_output=True,text=True)
            self.assertEqual(run.returncode,0,run.stderr)
            subprocess.run([str(p/'test')],check=True)

    def test_schema_appends_fixed_channel_contracts(self):
        schema=json.loads((ROOT/'config/sedsnet.json').read_text())
        names=['DAQ_SAR_VOLTAGES','DAQ_SDADC_VOLTAGES','DAQ_CURRENT_SENSE','DAQ_POWER_MONITOR']
        for i,(name,count) in enumerate(zip(names,[8,6,4,4]),43):
            t=schema['types'][i]
            self.assertEqual(t['name'],name)
            self.assertEqual(t['element'],dict(kind='Static',data_type='Float32',count=count))
            self.assertFalse(t['reliable'])
            self.assertEqual(t['endpoints'],['ActuatorGroundStation'])

    def test_publication_preserves_units_and_expires_missing_auxiliary_inputs(self):
        source = (ROOT/'Core/Src/daq_thread.c').read_text()
        publish = source[source.index('volatile uint32_t g_daq_analog_publish_ok_count;'):
                         source.index('static void daq_publish_loadcell(')]
        harness = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "daq_rates.h"
#include "mcp3564r_board_config.h"
typedef unsigned SedsDataType;
#define SEDS_OK 0
#define SEDS_DT_DAQ_SAR_VOLTAGES 143
#define SEDS_DT_DAQ_SDADC_VOLTAGES 144
#define SEDS_DT_DAQ_CURRENT_SENSE 145
#define SEDS_DT_DAQ_POWER_MONITOR 146
typedef struct {
 uint64_t monotonic_ms;
 float analog_inputs_v[8], current_sense_v[2], current_sense_a[2], power_monitor_v[2];
 float input_voltage_v, input_current_a;
 unsigned analog_sample_fresh;
} daq_snapshot_t;
typedef struct {unsigned count[8];float voltage_sum[8];} daq_loadcell_window_t;
static float packets[4][8];
static unsigned counts[4], calls;
static int log_telemetry_asynchronous(SedsDataType type,const float *data,size_t count,size_t size) {
 assert(type>=143 && type<=146 && size==sizeof(float));
 counts[type-143]=count;memcpy(packets[type-143],data,count*size);calls++;return 0;
}
''' + publish + r'''
int main(void) {
 daq_snapshot_t snapshot={.monotonic_ms=100,.analog_sample_fresh=1,
   .current_sense_v={1,2},.current_sense_a={3,4},.power_monitor_v={5,6},
   .input_voltage_v=12,.input_current_a=0.4f};
 for(unsigned i=0;i<8;i++)snapshot.analog_inputs_v[i]=i*0.1f;
 daq_loadcell_window_t window={0};
 window.count[6]=2;window.voltage_sum[6]=1.2f;
 daq_publish_analog(&snapshot,&window);
 assert(calls==4 && counts[0]==8 && counts[1]==6 && counts[2]==4 && counts[3]==4);
 assert(isnan(packets[1][0]) && fabsf(packets[1][4]-6.6f)<1e-6f);
 assert(packets[0][7]==0.7f && packets[2][2]==3 && packets[3][2]==12);
 snapshot.monotonic_ms=150;snapshot.analog_sample_fresh=0;
 window=(daq_loadcell_window_t){0};daq_publish_analog(&snapshot,&window);assert(calls==4);
 snapshot.monotonic_ms=200;daq_publish_analog(&snapshot,&window);
 assert(calls==5 && isfinite(packets[1][4]));
 snapshot.monotonic_ms=401;daq_publish_analog(&snapshot,&window);
 assert(calls==6 && isnan(packets[1][4]));
 assert(g_daq_analog_publish_fail_count==0 && g_daq_analog_publish_ok_count==6);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            binary=pathlib.Path(tmp)/'publish-test'
            result=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=harness,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True)

    def test_snapshot_logging_spreads_work_and_retains_capture(self):
        source = (ROOT/'Core/Src/daq_thread.c').read_text()
        code = source[source.index('volatile uint32_t g_daq_sd_snapshot_row_drop_count;'):
                      source.index('volatile uint32_t g_daq_analog_publish_ok_count;')]
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define DISABLE_SD_CARD 0
#include "daq_board.h"
typedef struct {float value;} daq_calibration_t;
typedef struct {unsigned count[8];float voltage_sum[8];} daq_loadcell_window_t;
static unsigned calls;
static int sd_card_enqueue_csv_row(const char *name,uint64_t ms,float value,const daq_calibration_t *cal) {
 assert(ms==123 && cal->value==42);
 if(strcmp(name,"analog_in_8_v")==0)assert(value==8);
 if(strcmp(name,"mcp3564r_ch1_voltage_v")==0)assert(value==3);
 calls++;return 0;
}
''' + code + r'''
int main(void) {
 daq_snapshot_t s={.monotonic_ms=123,.analog_sample_fresh=1};
 s.analog_inputs_v[7]=8;
 daq_calibration_t c={42};daq_loadcell_window_t w={0};w.count[1]=2;w.voltage_sum[1]=6;
 daq_store_snapshot_csv(&s,&c,&w);assert(calls==1);
 s.analog_sample_fresh=0;s.monotonic_ms=999;s.analog_inputs_v[7]=99;c.value=99;
 for(unsigned i=1;i<24;i++){daq_store_snapshot_csv(&s,&c,&w);assert(calls==i+1);}
 daq_store_snapshot_csv(&s,&c,&w);assert(calls==24);
 assert(g_daq_sd_snapshot_row_drop_count==0);
 s.monotonic_ms=123;s.analog_sample_fresh=1;c.value=42;
 daq_store_snapshot_csv(&s,&c,&w);daq_store_snapshot_csv(&s,&c,&w);
 assert(g_daq_sd_snapshot_row_drop_count==23);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=pathlib.Path(tmp)
            (p/'tx_api.h').write_text('typedef unsigned UINT;')
            (p/'stm32u5xx_hal.h').write_text(HAL)
            binary=p/'snapshot-test'
            result=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(p),'-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=harness,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True)

    def test_worker_queue_copies_capture_and_counts_backpressure(self):
        source=(ROOT/'Core/Src/daq_thread.c').read_text()
        code=source[source.index('/* Bound slow telemetry work'):source.index('static void daq_analog_thread_entry(')]
        harness=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "daq_board.h"
typedef struct {float value;} daq_calibration_t;
typedef struct {unsigned count[8];float voltage_sum[8];} daq_loadcell_window_t;
typedef uintptr_t ULONG;
typedef void VOID;
typedef int TX_THREAD;
typedef int TX_QUEUE;
typedef int TX_BLOCK_POOL;
#define TX_SUCCESS 0
#define TX_NO_WAIT 0
static int tx_block_allocate(TX_BLOCK_POOL *,VOID **,unsigned);
static int tx_queue_send(TX_QUEUE *,VOID *,unsigned);
static int tx_block_release(VOID *);
'''+code+r'''
static daq_analog_work_t captured;
static int allocation_failure,queue_failure,released,sent;
static int tx_block_allocate(TX_BLOCK_POOL *pool,VOID **ptr,unsigned wait) {
 (void)pool;assert(wait==TX_NO_WAIT);*ptr=&captured;return allocation_failure;
}
static int tx_queue_send(TX_QUEUE *q,VOID *msg,unsigned wait) {
 (void)q;assert(wait==TX_NO_WAIT && *(ULONG *)msg==(uintptr_t)&captured);sent++;return queue_failure;
}
static int tx_block_release(VOID *ptr){assert(ptr==&captured);released++;return 0;}
int main(void) {
 (void)g_analog_thread;(void)g_analog_stack;(void)g_analog_queue_storage;(void)g_analog_pool_storage;
 daq_snapshot_t s={.monotonic_ms=50};daq_calibration_t c={42};daq_loadcell_window_t w={0};
 w.count[6]=1;w.voltage_sum[6]=0.2f;daq_enqueue_analog(&s,&c,&w);assert(sent==0);
 s.monotonic_ms=100;s.analog_sample_fresh=1;daq_enqueue_analog(&s,&c,&w);
 assert(sent==1 && captured.snapshot.monotonic_ms==100 && captured.calibration.value==42);
 assert(captured.window.count[6]==2 && captured.window.voltage_sum[6]==0.4f);
 s.monotonic_ms=200;c.value=99;assert(captured.snapshot.monotonic_ms==100 && captured.calibration.value==42);
 queue_failure=1;daq_enqueue_analog(&s,&c,&w);assert(released==1 && g_daq_analog_work_drop_count==1);
 allocation_failure=1;daq_enqueue_analog(&s,&c,&w);assert(sent==2 && g_daq_analog_work_drop_count==2);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            p=pathlib.Path(tmp);(p/'tx_api.h').write_text('typedef unsigned UINT;');(p/'stm32u5xx_hal.h').write_text(HAL)
            binary=p/'queue-test'
            result=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I',str(p),'-I',str(ROOT/'Core/Inc'),'-x','c','-','-o',str(binary)],input=harness,text=True,capture_output=True)
            self.assertEqual(result.returncode,0,result.stderr)
            subprocess.run([str(binary)],check=True)
