"""Execute the DAQ wait policy: drain backlog without starving SD/acquisition."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class BacklogWaitTests(unittest.TestCase):
    def test_busy_passes_yield_and_every_fifth_pass_waits_one_ms(self):
        source=(ROOT/'Core/Src/telemetry_thread.c').read_text()
        helper=source[source.index('static uint8_t immediate_work_passes;'):source.index('void telemetry_thread_entry(')]
        harness=r'''#include <stdint.h>
#include <stddef.h>
#include <assert.h>
typedef unsigned long ULONG;
static struct{void *r;}g_router={(void *)1};
static int pending;
static unsigned yields,sleeps;
static ULONG expected_ticks;
int seds_router_has_queued_work(void *r){assert(r);return pending;}
void tx_thread_relinquish(void){++yields;}
void tx_thread_sleep(ULONG ticks){assert(ticks==expected_ticks);++sleeps;}
''' + helper + r'''
int main(void){
 for(unsigned hz=1000;hz<=50000;hz+=49000){
  expected_ticks=(hz+999U)/1000U;immediate_work_passes=0;yields=sleeps=0;
  pending=1;
  for(unsigned i=0;i<10;++i)wait_for_telemetry_work(expected_ticks);
  assert(yields==8 && sleeps==2 && immediate_work_passes==0);
  wait_for_telemetry_work(expected_ticks);assert(immediate_work_passes==1);
  pending=0;wait_for_telemetry_work(expected_ticks);
  assert(sleeps==3 && immediate_work_passes==0);
  pending=-10;wait_for_telemetry_work(expected_ticks);assert(sleeps==4);
  pending=1;g_router.r=NULL;wait_for_telemetry_work(expected_ticks);
  assert(sleeps==5 && yields==9);g_router.r=(void *)1;
 }
 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            exe=Path(d)/'wait'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=harness,text=True,check=True)
            subprocess.run([str(exe)],check=True)
