from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(source, name):
    start = source.index(name)
    brace = source.index("{", start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def run_c(code):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / "test.c").write_text(code)
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-Wno-unused-variable", "-I", str(ROOT / "Core/Inc"),
                        str(path / "test.c"), "-o", str(path / "test")], check=True)
        subprocess.run([str(path / "test")], check=True)


class ReportTests(unittest.TestCase):
    def test_nonblocking_handoff_preserves_capture_and_counts_rejection(self):
        source = (ROOT / "Core/Src/daq_thread.c").read_text()
        start = source.index("#define DAQ_REPORT_DEPTH")
        end = source.index("#if (DAQ_ENABLE_DUMMY_CAN_TELEMETRY", start)
        run_c(r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <setjmp.h>
#include "daq_rates.h"
typedef unsigned UINT;
typedef uintptr_t ULONG;
typedef void VOID;
typedef int TX_THREAD; typedef int TX_QUEUE; typedef int TX_BLOCK_POOL;
typedef struct {float slope;} daq_calibration_t;
typedef struct {unsigned ext_adc_sample_valid;float ext_adc_loadcell_kg1000;uint64_t monotonic_ms;} daq_snapshot_t;
#define TX_NO_WAIT 0
#define TX_WAIT_FOREVER 99
#define TX_SUCCESS 0
#define SEDS_OK 0
#define SEDS_DT_DAQ_ADC_TEMPERATURE 140
static unsigned g_daq_temperature_publish_ok_count,g_daq_temperature_publish_fail_count;
static unsigned fail_alloc,fail_send,released,published,queued;
static ULONG message;
static jmp_buf finished;
static UINT tx_block_allocate(TX_BLOCK_POOL*p,VOID**v,unsigned wait){(void)p;assert(wait==TX_NO_WAIT);if(fail_alloc)return 1;*v=malloc(256);return 0;}
static UINT tx_block_release(VOID*p){free(p);released++;return 0;}
static UINT tx_queue_send(TX_QUEUE*q,VOID*p,unsigned wait){(void)q;assert(wait==TX_NO_WAIT);if(fail_send)return 1;assert(!queued);message=*(ULONG*)p;queued=1;return 0;}
static UINT tx_queue_receive(TX_QUEUE*q,VOID*p,unsigned wait){(void)q;assert(wait==TX_WAIT_FOREVER);if(!queued)longjmp(finished,1);*(ULONG*)p=message;queued=0;return 0;}
static uint64_t telemetry_now_ms(void){return 12350;}
static void daq_publish_loadcell(const daq_snapshot_t*s,const daq_calibration_t*c){assert(s->monotonic_ms==12300 && s->ext_adc_loadcell_kg1000==1.25f && c->slope==2);published++;}
static void daq_publish_kg50(float value,uint64_t time,const daq_calibration_t*c){assert(value==2.5f && time==12300 && c->slope==2);published++;}
static int log_telemetry_captured(unsigned type,const void*data,size_t count,size_t size,uint64_t time){assert(type==140 && *(const float*)data==30 && count==1 && size==4 && time==12300);published++;return 0;}
''' + source[start:end] + r'''
int main(void){
 assert(DAQ_BROADCAST_RATE_HZ==250 && DAQ_BROADCAST_PERIOD_MS==4);
 daq_report_t report={.monotonic_ms=12300,.calibration={2},.kg1000=1.25,.kg50=2.5,.temperature=30,.flags=7};
 fail_alloc=1;daq_enqueue_report(&report);assert(g_daq_report_drop_count==1 && !published);
 fail_alloc=0;fail_send=1;daq_enqueue_report(&report);assert(g_daq_report_drop_count==2 && released==1);
 fail_send=0;daq_enqueue_report(&report);assert(!published && g_daq_report_enqueued_count==1);
 report.monotonic_ms=999;report.kg1000=999;report.calibration.slope=999;
 if(!setjmp(finished))daq_report_thread_entry(0);
 assert(published==3 && released==2 && g_daq_report_completed_count==1 && g_daq_report_max_age_ms==50);
}
''')

    def test_packet_time_subtracts_queue_age_and_handles_no_sync(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        run_c(r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#define TELEMETRY_ENABLED 1
#define SEDS_OK 0
#define SEDS_BAD_ARG 1
#define SEDS_ERR 2
typedef int SedsResult;typedef unsigned SedsDataType;
static struct {void*r;uint64_t start_time;} g_router={(void*)1,100};
static int synced=1;static uint64_t emitted;
static uint64_t telemetry_now_ms(void){return 1000;}
static SedsResult init_telemetry_router(void){return SEDS_OK;}
static int guess_kind_from_elem_size(size_t n){return (int)n;}
static int seds_router_get_network_time_ms(void*r,uint64_t*t){(void)r;*t=1000000;return synced?SEDS_OK:SEDS_ERR;}
static int seds_router_log_typed_ex(void*r,unsigned ty,const void*d,size_t n,size_t w,int k,const uint64_t*t,int queued){(void)r;(void)ty;(void)d;(void)n;(void)w;(void)k;assert(queued);emitted=*t;return SEDS_OK;}
''' + function(source, "SedsResult log_telemetry_captured(") + r'''
int main(void){float value=1;
 assert(log_telemetry_captured(118,&value,1,4,950)==SEDS_OK && emitted==999950);
 synced=0;assert(log_telemetry_captured(118,&value,1,4,950)==SEDS_OK && emitted==850);
 assert(log_telemetry_captured(118,&value,1,4,0)==SEDS_OK && emitted==0);
 assert(log_telemetry_captured(118,0,1,4,950)==SEDS_BAD_ARG);
}
''')


if __name__ == "__main__":
    unittest.main()
