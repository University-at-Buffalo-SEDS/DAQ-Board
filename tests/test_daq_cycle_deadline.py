from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
class CycleDeadlineTests(unittest.TestCase):
    def test_actual_cycle_wait_boundary_late_cycle_and_tick_wrap(self):
        source=(ROOT/'Core/Src/daq_thread.c').read_text()
        start=source.index('    /* Include acquisition/publish work')
        end=source.index('\n  }\n}\n\n/* Bound slow',start)
        body=source[start:end]
        stub=r'''#include <stdint.h>
#include <assert.h>
typedef uint32_t ULONG;
#define DAQ_SAMPLE_PERIOD_TICKS 2U
static uint32_t tick,slept,yielded;
static uint32_t g_daq_cycle_elapsed_ticks,g_daq_cycle_max_ticks;
static uint32_t g_daq_first_overrun_ticks,g_daq_first_overrun_sample;
static uint32_t g_daq_sample_overrun_count,g_daq_sample_ok_count=7;
static ULONG tx_time_get(void){return tick;}
static void tx_thread_sleep(ULONG n){slept=n;}
static void tx_thread_relinquish(void){yielded++;}
static void finish(ULONG cycle_started){
'''
        main=r'''}
int main(void){
 tick=100; finish(100); assert(slept==2 && !g_daq_sample_overrun_count);
 slept=0; tick=101; finish(100); assert(slept==1 && !yielded);
 slept=0; tick=102; finish(100);
 assert(!slept && yielded==1 && !g_daq_sample_overrun_count);
 tick=103; finish(100);
 assert(slept==1 && g_daq_sample_overrun_count==1);
 assert(g_daq_first_overrun_ticks==3 && g_daq_first_overrun_sample==7);
 slept=0; tick=0; finish(UINT32_MAX-1);
 assert(!slept && yielded==2 && g_daq_sample_overrun_count==1);
 tick=1; finish(UINT32_MAX-1);
 assert(slept==1 && g_daq_sample_overrun_count==2);
 assert(g_daq_cycle_max_ticks==3);
}'''
        with tempfile.TemporaryDirectory() as directory:
            exe=Path(directory)/'test'
            subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=stub+body+main,text=True,check=True)
            subprocess.run([str(exe)],check=True,timeout=5)
