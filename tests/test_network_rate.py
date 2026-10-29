"""Production publishers and worker: one global weighted cap, untouched SD."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class NetworkRateTests(unittest.TestCase):
    def compile(self, code, defines=(), valid=True):
        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / 'test'
            args = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-DTELEMETRY_ENABLED=1',
                    '-I', str(ROOT / 'Core/Inc')]
            args += ['-D' + definition for definition in defines]
            p = subprocess.run(args + ['-x', 'c', '-', '-o', str(exe)],
                               input=code, text=True, capture_output=True)
            if valid:
                self.assertEqual(p.returncode, 0, p.stderr)
                subprocess.run([str(exe)], check=True)
            else:
                self.assertNotEqual(p.returncode, 0)

    def harness(self):
        source = (ROOT / 'Core/Src/daq_thread.c').read_text()
        temperature_start = source.index('daq_network_offer(DAQ_NETWORK_TEMPERATURE,')
        temperature = source[temperature_start:source.index(';', temperature_start) + 1]
        return r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "daq_network_schedule.h"
#define DISABLE_SD_CARD 0
#define SEDS_OK 0
#define SEDS_IO (-14)
#define SD_CARD_STATUS_OK 0
#define MCP3564R_AUX_INTERVAL_MS 100
#define SEDS_DT_DAQ_LOADCELL_CHAMBER 141
#define SEDS_DT_KG1000 0
#define SEDS_DT_KG50 1
#define SEDS_DT_DAQ_KG50_SELECTED 2
#define SEDS_DT_DAQ_ADC_TEMPERATURE 3
#define SEDS_DT_DAQ_SDADC_VOLTAGES 4
#define SEDS_DT_DAQ_SAR_VOLTAGES 5
#define SEDS_DT_DAQ_CURRENT_SENSE 6
#define SEDS_DT_DAQ_POWER_MONITOR 7
typedef int SedsResult;
typedef struct {float slope, intercept;} daq_calibration_t;
typedef struct {
 uint64_t monotonic_ms;
 unsigned ext_adc_sample_valid, analog_sample_fresh;
 float ext_adc_loadcell_kg1000, analog_inputs_v[8], current_sense_v[2], current_sense_a[2];
 float power_monitor_v[2], input_voltage_v, input_current_a;
} daq_snapshot_t;
typedef struct {unsigned count[8];float voltage_sum[8];} daq_loadcell_window_t;
static daq_network_schedule_t g_daq_network;
static uint32_t g_daq_network_offered[7],g_daq_network_coalesced[7],g_daq_network_sent[7],g_daq_network_failed[7],g_daq_network_invalid;
static unsigned g_daq_loadcell_publish_ok_count,g_daq_loadcell_publish_fail_count;
static unsigned g_daq_kg50_publish_ok_count,g_daq_kg50_publish_fail_count;
static unsigned g_daq_temperature_publish_ok_count,g_daq_temperature_publish_fail_count;
static unsigned g_daq_analog_publish_ok_count,g_daq_analog_publish_fail_count;
static unsigned g_daq_sd_network_row_ok_count,g_daq_sd_network_row_fail_count;
static unsigned sd_rows,sd_by_type[3],sent[7],buckets[10],fail_next,irq;
static uint32_t actual_ms,last_send_ms=UINT32_MAX;
static uint64_t expected_ms;
static float expected_value;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t value){irq=value;}
static float mcp3564r_connector_voltage(unsigned i,float value){(void)i;return value;}
static int sd_card_enqueue_csv_row(const char*name,uint64_t ms,float raw,const daq_calibration_t*c){
 assert(ms==expected_ms && raw==expected_value && c->slope==3 && c->intercept==-7);
 unsigned ty=!strcmp(name,"kg1000_network")?0:!strcmp(name,"kg50_network")?1:2;
 assert(ty!=2 || !strcmp(name,"kg50_p6_network_volts"));++sd_rows;++sd_by_type[ty];return 0;
}
static int log_telemetry_captured(unsigned type,const void*value,size_t n,size_t width,uint64_t captured){
 assert(!irq && width==sizeof(float) && actual_ms!=last_send_ms);
 last_send_ms=actual_ms;
 const unsigned flow=type==0?DAQ_NETWORK_KG1000:type<=2?DAQ_NETWORK_KG50:type-1;
 assert(flow<7);++sent[flow];if(actual_ms<10000)++buckets[actual_ms/1000];
 const float*p=value;
 const float marker[7]={200000,100000,300000,700000,400000,500000,600000};
 assert(captured<=10000+actual_ms);
 const float raw=flow==DAQ_NETWORK_KG50 && type==2?p[1]:p[0];
 assert(raw==marker[flow]+(float)captured);
 if(flow==DAQ_NETWORK_KG50){
   assert(n==(type==2?2:1));if(type==2)assert(p[0]==1);
   if(actual_ms<10000)assert(type==(actual_ms<5000?SEDS_DT_KG50:SEDS_DT_DAQ_KG50_SELECTED));
 }
 if(fail_next){--fail_next;return SEDS_IO;}return SEDS_OK;
}
''' + '\n'.join(function(source, marker) for marker in [
            'static void daq_network_offer(', 'static void daq_network_process_once(',
            'static void daq_publish_analog_packet(', 'static void daq_publish_analog(',
            'static void daq_publish_loadcell(', 'static void daq_publish_kg50(']) + r'''
static void publish_temperature(float value,uint64_t ms){
 struct {float temperature;uint64_t monotonic_ms;} record={value,ms};
 __typeof__(record)*work=&record;
''' + temperature + r'''
}
int main(void){
 assert(DAQ_ADC_READ_RATE_HZ==3700 && DAQ_ACQUISITION_PERIOD_MS==2 && DAQ_BROADCAST_PERIOD_MS==4);
 daq_calibration_t calibration={3,-7};daq_snapshot_t snapshot={.ext_adc_sample_valid=1};
 daq_loadcell_window_t window={.count={0,0,1}};
 const unsigned weights[7]={DAQ_NETWORK_WEIGHT_KG50,DAQ_NETWORK_WEIGHT_KG1000,DAQ_NETWORK_WEIGHT_TEMPERATURE,
   DAQ_NETWORK_WEIGHT_SDADC,DAQ_NETWORK_WEIGHT_SAR,DAQ_NETWORK_WEIGHT_CURRENT,DAQ_NETWORK_WEIGHT_POWER};
 for(actual_ms=0;actual_ms<10000;actual_ms++){
  if(actual_ms%4==0){
   expected_ms=10000+actual_ms;
   expected_value=100000+(float)expected_ms;snapshot.ext_adc_loadcell_kg1000=expected_value;snapshot.monotonic_ms=expected_ms;
   daq_publish_loadcell(&snapshot,&calibration,NULL);
   expected_value=200000+(float)expected_ms;
   daq_publish_kg50(expected_value,actual_ms>=5000,expected_ms,&calibration);
  }
  if(actual_ms%100==0){
   expected_ms=10000+actual_ms;publish_temperature(300000+(float)expected_ms,expected_ms);
   snapshot.monotonic_ms=expected_ms;snapshot.analog_sample_fresh=1;
   snapshot.analog_inputs_v[0]=400000+(float)expected_ms;
   snapshot.current_sense_v[0]=500000+(float)expected_ms;
   snapshot.power_monitor_v[0]=600000+(float)expected_ms;
   window.voltage_sum[2]=700000+(float)expected_ms;
   daq_publish_analog(&snapshot,&window);
  }
  daq_network_process_once(actual_ms);
 }
 const unsigned total=10*DAQ_NETWORK_RATE_HZ;
 unsigned sum=0;
 for(unsigned i=0;i<7;i++){
   sum+=sent[i];const float ideal=(float)(total*weights[i])/DAQ_NETWORK_WEIGHT_TOTAL;
   assert(fabsf((float)sent[i]-ideal)<=1.0f);
   assert(g_daq_network_sent[i]==sent[i] && !g_daq_network_failed[i]);
   assert(g_daq_network_offered[i]==(i<2?2500:100));
   assert(g_daq_network_coalesced[i]>=g_daq_network_offered[i]-sent[i]-1);
 }
 assert(sum==total && !g_daq_network_invalid);
 for(unsigned i=0;i<10;i++)assert(buckets[i]==DAQ_NETWORK_RATE_HZ);
 assert(sd_rows==5000 && sd_by_type[0]==2500 && sd_by_type[1]==1250 && sd_by_type[2]==1250);
 assert(g_daq_sd_network_row_ok_count==5000 && !g_daq_sd_network_row_fail_count);
 assert(!g_daq_loadcell_publish_fail_count && !g_daq_kg50_publish_fail_count && !g_daq_analog_publish_fail_count);
 /* A paused network worker can emit only one packet when it resumes. The
  * report producer still saves every SD row while replacing NETWORK work. */
 actual_ms=50000;
 for(unsigned i=0;i<20;i++){
  expected_ms=59000+i*4;expected_value=100000+(float)expected_ms;
  snapshot.monotonic_ms=expected_ms;snapshot.ext_adc_loadcell_kg1000=expected_value;
  daq_publish_loadcell(&snapshot,&calibration,NULL);
 }
 unsigned before=0;for(unsigned i=0;i<7;i++)before+=sent[i];
 fail_next=1;daq_network_process_once(actual_ms);
 for(unsigned i=0;i<100;i++)daq_network_process_once(actual_ms);
 unsigned after=0,failures=0;for(unsigned i=0;i<7;i++){after+=sent[i];failures+=g_daq_network_failed[i];}
 assert(after==before+1 && failures==1 && !fail_next && sd_rows==5020);
}
'''

    def test_combined_chamber_publisher_uses_same_slots_and_invalid_falls_back(self):
        code=self.harness()
        code=code.replace('type==0?DAQ_NETWORK_KG1000', '(type==0||type==141)?DAQ_NETWORK_KG1000')
        code=code.replace('const float*p=value;', 'const float*p=value; if(type==141){assert(n==3&&p[1]==500.0f&&p[2]==2.5f);}')
        code=code.replace('daq_calibration_t calibration={3,-7};','float pressure[2]={500.0f,2.5f};daq_calibration_t calibration={3,-7};')
        code=code.replace('if(actual_ms%4==0){','if(actual_ms%4==0){pressure[0]=actual_ms<5000?500.0f:NAN;')
        code=code.replace('daq_publish_loadcell(&snapshot,&calibration,NULL);','daq_publish_loadcell(&snapshot,&calibration,pressure);')
        self.compile(code)

    def test_global_cap_priority_weights_sd_capture_and_input_switch(self):
        for rate in (None, 83, 37):
            with self.subTest(rate=rate):
                self.compile(self.harness(), () if rate is None else (f'DAQ_NETWORK_RATE_HZ={rate}',))

    def test_fault_classifier_distinguishes_io_without_retrying_a_slot(self):
        code = self.harness().replace(
            'static unsigned sd_rows,',
            'static uint32_t g_sim_daq_publication_attempts, '
            'g_sim_daq_publication_io_refusals, g_sim_daq_publication_other_refusals;\n'
            'static unsigned sd_rows,')
        code = code.replace(
            'if(fail_next){--fail_next;return SEDS_IO;}',
            'if(fail_next){unsigned result=fail_next;fail_next=0;return result==2?-3:SEDS_IO;}')
        end = 'assert(after==before+1 && failures==1 && !fail_next && sd_rows==5020);'
        self.assertIn(end, code)
        code = code.replace(end, end + r'''
 assert(g_sim_daq_publication_attempts==after);
 assert(g_sim_daq_publication_io_refusals==1 && g_sim_daq_publication_other_refusals==0);
 actual_ms+=10;fail_next=2;daq_network_process_once(actual_ms);
 assert(g_sim_daq_publication_attempts==after+1);
 assert(g_sim_daq_publication_io_refusals==1 && g_sim_daq_publication_other_refusals==1);
''')
        self.compile(code, ('SEDS_FIRMWARE_SIM_TEST=1',))

    def test_header_weights_and_disabled_auxiliary_streams(self):
        definitions = ['DAQ_NETWORK_WEIGHT_KG50=3', 'DAQ_NETWORK_WEIGHT_KG1000=1']
        definitions += ['DAQ_NETWORK_WEIGHT_' + name + '=0' for name in ['TEMPERATURE','SDADC','SAR','CURRENT','POWER']]
        self.compile(self.harness(), definitions)

    def test_invalid_configurations(self):
        cases = [['DAQ_NETWORK_RATE_HZ=0'], ['DAQ_NETWORK_RATE_HZ=-1'], ['DAQ_NETWORK_RATE_HZ=1001'],
                 ['DAQ_NETWORK_WEIGHT_KG50=1001'], ['DAQ_NETWORK_WEIGHT_POWER=-1']]
        cases += [['DAQ_NETWORK_WEIGHT_'+n+'=0' for n in ['KG50','KG1000','TEMPERATURE','SDADC','SAR','CURRENT','POWER']]]
        for case in cases:
            self.compile('#include "daq_network_schedule.h"\nint main(void){}', case, False)

    def test_fractional_clock_wrap_empty_streams_and_invalid_samples(self):
        self.compile(r'''#include "daq_network_schedule.h"
#include <assert.h>
int main(void){
 daq_network_schedule_t s={0};unsigned sent=0,stream;daq_network_sample_t out;
 for(unsigned elapsed=0;elapsed<10000;elapsed++){
  for(unsigned i=0;i<7;i++){float value=(float)elapsed;assert(daq_network_schedule_offer(&s,i,i,&value,1,elapsed)>=0);}
  sent+=daq_network_schedule_take(&s,UINT32_MAX-500+elapsed,&stream,&out);
 }
 assert(sent==370);
 assert(daq_network_schedule_offer(&s,7,0,out.values,1,0)==-1);
 assert(daq_network_schedule_offer(&s,0,0,out.values,9,0)==-1);
 assert(daq_network_schedule_offer(&s,0,0,0,1,0)==-1);
 s=(daq_network_schedule_t){0};for(unsigned t=0;t<10000;t++)assert(!daq_network_schedule_take(&s,t,&stream,&out));
 assert(sizeof(s)<512);
}''', ['DAQ_NETWORK_RATE_HZ=37'])
