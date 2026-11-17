"""Production-path contracts: live values never wait on SD.

The virtual millisecond driver is not a ThreadX CPU/scheduling benchmark. It
checks capture freshness and cadence of acquisition's direct network offers;
there is no SD report queue beside them. Full-ELF SD timing needs Renode.
Set DAQ_ISOLATION_SOURCE_ROOT to a source snapshot to compare a pre-fix design.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = Path(os.environ.get('DAQ_ISOLATION_SOURCE_ROOT', ROOT))


def function(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def run_c(code):
    with tempfile.TemporaryDirectory(prefix='daq-live-isolation-') as directory:
        binary = Path(directory) / 'test'
        result = subprocess.run([
            'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
            '-Wno-unused-function', '-Wno-unused-variable',
            '-fsanitize=address,undefined', '-DTELEMETRY_ENABLED=1',
            '-I', str(ROOT / 'Core/Inc'), '-x', 'c', '-', '-o', str(binary),
        ], input=code, text=True, capture_output=True)
        if result.returncode:
            raise AssertionError(result.stderr)
        result = subprocess.run([str(binary)], text=True, capture_output=True, timeout=10)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        print(result.stdout, end='')


class SdLiveIsolationTests(unittest.TestCase):
    def test_acquisition_offers_before_any_sd_admission(self):
        source = (SOURCE / 'Core/Src/daq_thread.c').read_text()
        body = function(source, 'void daq_thread_entry(')
        self.assertTrue('daq_offer_live_report(&report)' in body,
                        'fresh network offers must not depend on the SD/report consumer')
        offer = body.index('daq_offer_live_report(&report)')
        self.assertLess(offer, body.index('sd_card_enqueue_raw_adc_samples_captured('))
        self.assertNotIn('daq_enqueue_report(', source, 'no SD report queue sits beside the live path')
        seam = function(source, 'static void daq_offer_live_report(')
        self.assertNotIn('sd_card_', seam)
        self.assertNotIn('tx_', seam)
        self.assertNotIn('log_telemetry', seam, 'SDK work belongs to the independent network task')

    def test_live_values_keep_cadence_and_never_replay(self):
        source = (SOURCE / 'Core/Src/daq_thread.c').read_text()
        definitions = source[source.index('#define DAQ_REPORT_KG1000'):source.index('/* Acquisition publishes only into fixed')]
        production = '\n'.join(function(source, marker) for marker in [
            'static void daq_network_offer(', 'static void daq_network_process_once(',
            'static void daq_offer_live_report('])
        run_c(r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "daq_network_schedule.h"
#define SEDS_OK 0
#define SEDS_DT_KG1000 100
#define SEDS_DT_KG50 101
#define SEDS_DT_DAQ_KG50_SELECTED 102
#define SEDS_DT_DAQ_LOADCELL_CHAMBER 103
#define SEDS_DT_DAQ_ADC_TEMPERATURE 104
typedef int SedsResult;
static daq_network_schedule_t g_daq_network;
static uint32_t g_daq_network_offered[7],g_daq_network_coalesced[7],g_daq_network_sent[7],g_daq_network_failed[7],g_daq_network_invalid;
static unsigned g_daq_loadcell_publish_ok_count,g_daq_loadcell_publish_fail_count;
static unsigned g_daq_kg50_publish_ok_count,g_daq_kg50_publish_fail_count;
static unsigned g_daq_temperature_publish_ok_count,g_daq_temperature_publish_fail_count;
static unsigned g_daq_analog_publish_ok_count,g_daq_analog_publish_fail_count;
static unsigned now_ms,irq,sent[7],last_send[7],max_gap[7],max_age,stale_replays;
static uint64_t last_capture[7];
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t value){irq=value;}
static int log_telemetry_captured(unsigned type,const void*data,size_t count,size_t width,uint64_t capture){
 assert(!irq&&width==sizeof(float));const float *values=data;
 unsigned stream=type==100||type==103?DAQ_NETWORK_KG1000:type==101||type==102?DAQ_NETWORK_KG50:DAQ_NETWORK_TEMPERATURE;
 assert(capture<=now_ms);
 if(stream<=DAQ_NETWORK_KG1000){
  assert(count==(type==103?3U:type==102?2U:1U));
  const float actual=stream==DAQ_NETWORK_KG50&&type==102?values[1]:values[0];
  assert(actual==(stream==DAQ_NETWORK_KG50?-(float)capture:(float)capture));
  if(sent[stream]&&now_ms-last_send[stream]>max_gap[stream])max_gap[stream]=now_ms-last_send[stream];
  if(capture<last_capture[stream])++stale_replays;
  if(now_ms-capture>max_age)max_age=now_ms-(unsigned)capture;
  last_capture[stream]=capture;last_send[stream]=now_ms;
 }
 ++sent[stream];return SEDS_OK;
}
'''+definitions+production+r'''
int main(void){
 const unsigned duration=3000;unsigned produced=0;
 for(now_ms=0;now_ms<duration;++now_ms){
  if(now_ms%4==3){
   /* A bin closes at its last conversion; acquisition offers it next cycle. */
   daq_report_t report={.monotonic_ms=now_ms,.kg1000_ms=now_ms-1,.kg50_ms=now_ms-1,
     .kg1000=(float)(now_ms-1),.kg50=-(float)(now_ms-1),.temperature=25,
     .chamber={100,0.5f},.chamber_valid=1,.flags=DAQ_REPORT_KG1000|DAQ_REPORT_KG50|DAQ_REPORT_TEMPERATURE,.kg50_input=1};
   ++produced;daq_offer_live_report(&report);
  }
  daq_network_process_once(now_ms);
 }
 printf("produced=%u live50=%u live1000=%u live_maxgap=%u/%u live_maxage=%u\n",produced,sent[0],sent[1],max_gap[0],max_gap[1],max_age);
 assert(g_daq_network_offered[DAQ_NETWORK_KG50]==produced && g_daq_network_offered[DAQ_NETWORK_KG1000]==produced);
 assert(!stale_replays&&max_age<=5&&max_gap[0]<=40&&max_gap[1]<=40);
 for(unsigned i=0;i<2;++i)assert(fabsf((float)sent[i]-(float)duration*40.0f/1000.0f)<=1.0f);
}
''')
