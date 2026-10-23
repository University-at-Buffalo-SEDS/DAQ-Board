"""Capture the transient bootstrap refusal, rather than an idle later queue."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PublicationDiagnosticsTests(unittest.TestCase):
    def test_first_failure_snapshot_uses_copy_count_and_preserves_original_result(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        start = source.index('SedsResult log_telemetry_asynchronous(')
        end = source.index('\nSedsResult log_telemetry_string_asynchronous', start)
        harness = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#define TELEMETRY_ENABLED
typedef int32_t SedsResult;
typedef uint32_t SedsDataType;
typedef void SedsRouter;
enum {SEDS_OK=0, SEDS_BAD_ARG=-10, SEDS_ERR=-1};
struct {void *r;} g_router={(void *)1};
volatile uint32_t g_telemetry_alloc_failure_stack[128];
volatile uint32_t g_telemetry_alloc_failure_stack_words;
static int result, copies, copy_result=16;
int init_telemetry_router(void){return SEDS_ERR;}
int guess_kind_from_elem_size(size_t n){return (int)n;}
int seds_router_log_queue_typed(void *r,uint32_t t,const void *p,size_t n,size_t s,int k){
 (void)r;(void)t;(void)p;(void)n;(void)s;(void)k;return result;
}
int seds_router_copy_memory_counters(void *r,uint32_t *out,size_t capacity){
 assert(r && capacity==16);++copies;
 for(unsigned i=0;i<16;++i)out[i]=100+i;
 return copy_result;
}
int seds_router_copy_queue_counters(SedsRouter *r,uint32_t *out,size_t capacity){
 assert(r && capacity==16);for(unsigned i=0;i<16;++i)out[i]=1000+i;return 16;
}
uint32_t HAL_GetTick(void){return 123;}
size_t seds_memory_admission_rejected(void){return 7;}
''' + source[start:end] + r'''
int main(void){
 float values[6]={0};
 assert(log_telemetry_asynchronous(101,values,6,4)==0);assert(!copies);
 result=-14;assert(log_telemetry_asynchronous(101,values,6,4)==-14);
#ifdef SEDS_FIRMWARE_SIM_TEST
 assert(copies==1 && g_telemetry_alloc_failure_stack_words==113);
 assert(g_telemetry_alloc_failure_stack[0]==100);
 assert(g_telemetry_alloc_failure_stack[15]==115);
 assert((int32_t)g_telemetry_alloc_failure_stack[16]==-14);
 assert(g_telemetry_alloc_failure_stack[17]==101);
 assert(g_telemetry_alloc_failure_stack[18]==123);
 assert(g_telemetry_alloc_failure_stack[19]==6);
 assert(g_telemetry_alloc_failure_stack[20]==7);
 assert(g_telemetry_alloc_failure_stack[96]==1000 && g_telemetry_alloc_failure_stack[111]==1015 && g_telemetry_alloc_failure_stack[112]==123);
 result=-18;assert(log_telemetry_asynchronous(102,values,6,4)==-18);
 assert(copies==2 && (int32_t)g_telemetry_alloc_failure_stack[16]==-14);
 assert(g_telemetry_alloc_failure_stack_words==113);
 assert((int32_t)g_telemetry_alloc_failure_stack[80]==-18);
 assert(g_telemetry_alloc_failure_stack[81]==102);
 g_telemetry_alloc_failure_stack_words=128;
 assert(log_telemetry_asynchronous(103,values,6,4)==-18);
 assert(copies==2 && g_telemetry_alloc_failure_stack_words==128);
 g_telemetry_alloc_failure_stack_words=0;copy_result=-10;
 assert(log_telemetry_asynchronous(102,values,6,4)==-18);
 assert(copies==3 && g_telemetry_alloc_failure_stack_words==0);
#else
 assert(!copies && !g_telemetry_alloc_failure_stack_words);
#endif
 assert(log_telemetry_asynchronous(101,NULL,6,4)==SEDS_BAD_ARG);
 g_router.r=NULL;assert(log_telemetry_asynchronous(101,values,6,4)==SEDS_ERR);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            for simulated in [False, True]:
                exe = Path(tmp) / ('simulated' if simulated else 'production')
                flags = ['-DSEDS_FIRMWARE_SIM_TEST'] if simulated else []
                subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', *flags,
                                '-x', 'c', '-', '-o', str(exe)], input=harness,
                               text=True, check=True)
                subprocess.run([str(exe)], check=True)
