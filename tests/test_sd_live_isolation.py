"""Production-path contracts under stalled SD/report consumers.

The virtual millisecond driver is not a ThreadX CPU/scheduling benchmark. It
checks queue ownership, capture freshness, and that a consumer stall cannot
remove acquisition's direct network offers. Full-ELF SD timing needs Renode.
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
        self.assertLess(offer, body.index('sd_card_enqueue_raw_adc_samples('))
        self.assertLess(offer, body.index('daq_enqueue_report(&report)'))
        seam = function(source, 'static void daq_offer_live_report(')
        self.assertNotIn('sd_card_', seam)
        self.assertNotIn('tx_', seam)
        self.assertNotIn('log_telemetry', seam, 'SDK work belongs to the independent network task')

    def test_report_consumer_stalls_do_not_stall_or_replay_live_values(self):
        source = (SOURCE / 'Core/Src/daq_thread.c').read_text()
        report_start = source.index('#define DAQ_REPORT_DEPTH')
        definitions_end = source.index('static void daq_enqueue_report(', report_start)
        if 'static void daq_offer_live_report(' in source:
            definitions_end = min(definitions_end, source.index('static void daq_offer_live_report('))
        report_definitions = source[report_start:definitions_end]
        has_direct_offer = 'static void daq_offer_live_report(' in source
        # Absence models the old acquisition path honestly: it only enqueues a
        # report, and the paused consumer owns all load-cell network offers.
        direct_offer = function(source, 'static void daq_offer_live_report(') if has_direct_offer else ''
        preamble = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include "daq_network_schedule.h"
#define DISABLE_SD_CARD 0
#define SEDS_OK 0
#define SD_CARD_STATUS_OK 0
#define SEDS_DT_KG1000 100
#define SEDS_DT_KG50 101
#define SEDS_DT_DAQ_KG50_SELECTED 102
#define SEDS_DT_DAQ_LOADCELL_CHAMBER 103
#define SEDS_DT_DAQ_ADC_TEMPERATURE 104
#define SEDS_DT_DAQ_SDADC_VOLTAGES 105
#define SEDS_DT_DAQ_SAR_VOLTAGES 106
#define SEDS_DT_DAQ_CURRENT_SENSE 107
#define SEDS_DT_DAQ_POWER_MONITOR 108
#define TX_SUCCESS 0U
#define TX_NO_WAIT 0U
#define TX_WAIT_FOREVER 0xffffffffU
typedef unsigned UINT;
typedef uintptr_t ULONG;
typedef void VOID;
typedef int TX_THREAD;
typedef int TX_BLOCK_POOL;
typedef struct {ULONG messages[16];unsigned count,read,write;} TX_QUEUE;
typedef struct {float slope,intercept;} daq_calibration_t;
typedef struct {unsigned ext_adc_sample_valid;float ext_adc_loadcell_kg1000;uint64_t monotonic_ms;} daq_snapshot_t;
typedef int SedsResult;
static daq_network_schedule_t g_daq_network;
static uint32_t g_daq_network_offered[7],g_daq_network_coalesced[7],g_daq_network_sent[7],g_daq_network_failed[7],g_daq_network_invalid;
static unsigned g_daq_loadcell_publish_ok_count,g_daq_loadcell_publish_fail_count;
static unsigned g_daq_kg50_publish_ok_count,g_daq_kg50_publish_fail_count;
static unsigned g_daq_temperature_publish_ok_count,g_daq_temperature_publish_fail_count;
static unsigned g_daq_analog_publish_ok_count,g_daq_analog_publish_fail_count;
static unsigned g_daq_sd_network_row_ok_count,g_daq_sd_network_row_fail_count;
static unsigned now_ms,irq,sd_rows;
static unsigned sent[7],last_send[7],max_gap[7],max_age,stale_replays;
static uint64_t last_capture[7];
static unsigned allocations,releases,pool_used[16],queue_peak;
static _Alignas(max_align_t) unsigned char pool[16][512];
static jmp_buf consumer_idle;
static uint32_t __get_PRIMASK(void){return irq;}
static void __disable_irq(void){irq=1;}
static void __set_PRIMASK(uint32_t value){irq=value;}
static uint64_t daq_log_monotonic_ms(void){return now_ms;}
static int sd_card_enqueue_csv_row(const char*n,uint64_t t,float v,const daq_calibration_t*c){
 (void)n;(void)t;(void)v;(void)c;++sd_rows;return 1; /* full storage: explicit backpressure */
}
static int log_telemetry_captured(unsigned type,const void*data,size_t count,size_t width,uint64_t capture){
 assert(!irq&&width==sizeof(float));const float *values=data;
 unsigned stream=type==100||type==103?DAQ_NETWORK_KG1000:type==101||type==102?DAQ_NETWORK_KG50:type-102;
 assert(stream<7&&capture<=now_ms);
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
'''
        mocks = r'''
static UINT tx_block_allocate(TX_BLOCK_POOL*p,VOID**out,unsigned wait){
 (void)p;assert(wait==TX_NO_WAIT&&!irq);
 for(unsigned i=0;i<16;++i)if(!pool_used[i]){pool_used[i]=1;*out=pool[i];++allocations;return 0;}return 1;
}
static UINT tx_block_release(VOID*p){
 for(unsigned i=0;i<16;++i)if(p==pool[i]){assert(pool_used[i]);pool_used[i]=0;++releases;return 0;}assert(0);return 1;
}
static UINT tx_queue_send(TX_QUEUE*q,VOID*p,unsigned wait){
 assert(wait==TX_NO_WAIT);if(q->count==16)return 1;
 q->messages[q->write]=*(ULONG*)p;q->write=(q->write+1)%16;++q->count;
 if(q->count>queue_peak)queue_peak=q->count;return 0;
}
static UINT tx_queue_receive(TX_QUEUE*q,VOID*p,unsigned wait){
 assert(wait==TX_WAIT_FOREVER);if(!q->count)longjmp(consumer_idle,1);
 *(ULONG*)p=q->messages[q->read];q->read=(q->read+1)%16;--q->count;return 0;
}
'''
        production = '\n'.join(function(source, marker) for marker in [
            'static void daq_network_offer(', 'static void daq_network_process_once(',
            'static void daq_publish_loadcell(', 'static void daq_publish_kg50(',
            'static void daq_enqueue_report(',
        ]) + direct_offer + function(source, 'static void daq_report_thread_entry(')
        driver = r'''
static void run_case(unsigned pause_ms){
 memset(&g_daq_network,0,sizeof(g_daq_network));memset(&g_report_queue,0,sizeof(g_report_queue));
 memset(sent,0,sizeof(sent));memset(last_send,0,sizeof(last_send));memset(max_gap,0,sizeof(max_gap));memset(last_capture,0,sizeof(last_capture));
 memset(g_daq_network_offered,0,sizeof(g_daq_network_offered));memset(pool_used,0,sizeof(pool_used));
 allocations=releases=queue_peak=max_age=stale_replays=0;
 g_daq_report_enqueued_count=g_daq_report_completed_count=g_daq_report_drop_count=g_daq_report_max_age_ms=0;
 const unsigned duration=1000+pause_ms;unsigned produced=0;
 for(now_ms=0;now_ms<duration;++now_ms){
  if(now_ms%4==0){
   daq_report_t report={.monotonic_ms=now_ms,.calibration={1,0},.kg1000=(float)now_ms,.kg50=-(float)now_ms,
     .chamber={100,0.5f},.chamber_valid=1,.flags=DAQ_REPORT_KG1000|DAQ_REPORT_KG50|DAQ_REPORT_TEMPERATURE,.kg50_input=1};
   ++produced;
DIRECT_ACQUISITION_OFFER
   daq_enqueue_report(&report);
   /* The queued owner must retain its captured copy. */
   report.monotonic_ms=UINT64_MAX;report.kg1000=NAN;
   for(unsigned stream=DAQ_NETWORK_SDADC;stream<DAQ_NETWORK_STREAMS;++stream){
    const float v=(float)now_ms;daq_network_offer(stream,105+stream-DAQ_NETWORK_SDADC,&v,1,now_ms);
   }
  }
  if(!(now_ms>=500&&now_ms<500+pause_ms)&&g_report_queue.count){
   if(!setjmp(consumer_idle))daq_report_thread_entry(0);
  }
  daq_network_process_once(now_ms);
 }
 if(g_report_queue.count&&!setjmp(consumer_idle))daq_report_thread_entry(0);
 printf("pause=%u report_drops=%u report_maxage=%u queue_peak=%u live50=%u live1000=%u live_maxgap=%u/%u live_maxage=%u\n",
   pause_ms,g_daq_report_drop_count,g_daq_report_max_age_ms,queue_peak,sent[0],sent[1],max_gap[0],max_gap[1],max_age);
 assert(DAQ_NETWORK_RATE_HZ==100&&DAQ_NETWORK_WEIGHT_KG50==10&&DAQ_NETWORK_WEIGHT_KG1000==10&&DAQ_NETWORK_WEIGHT_TOTAL==25);
 assert(queue_peak==DAQ_REPORT_DEPTH&&g_daq_report_drop_count>0);
 assert(g_daq_report_completed_count+g_daq_report_drop_count==produced);
 assert(allocations==releases);for(unsigned i=0;i<16;++i)assert(!pool_used[i]);
 /* Real report backpressure remains visible without controlling live ownership. */
 assert(g_daq_network_offered[DAQ_NETWORK_KG50]==produced);
 assert(g_daq_network_offered[DAQ_NETWORK_KG1000]==produced);
 assert(!stale_replays&&max_age<=4);
 assert(max_gap[0]<=40&&max_gap[1]<=40);
 for(unsigned i=0;i<2;++i){
  const float ideal=(float)duration*40.0f/1000.0f;
  assert(fabsf((float)sent[i]-ideal)<=1.0f);
 }
 unsigned total=0;for(unsigned i=0;i<7;++i)total+=sent[i];
 assert(total<=(duration+9)/10+1); /* no backlog catch-up burst */
}
int main(void){run_case(180);run_case(500);run_case(2000);}
'''
        driver = driver.replace('DIRECT_ACQUISITION_OFFER',
                                '   daq_offer_live_report(&report);' if has_direct_offer else '')
        for pause_ms in (180, 500, 2000):
            with self.subTest(sd_consumer_pause_ms=pause_ms):
                case = driver.replace('int main(void){run_case(180);run_case(500);run_case(2000);}',
                                      f'int main(void){{run_case({pause_ms});}}')
                run_c(preamble + report_definitions + mocks + production + case)
