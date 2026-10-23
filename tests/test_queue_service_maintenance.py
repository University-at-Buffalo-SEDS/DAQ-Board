"""Execute DAQ's queue service with instrumented SDK maintenance entry points."""
from pathlib import Path
import subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
class QueueServiceMaintenanceTests(unittest.TestCase):
    def test_explicit_board_poll_is_not_repeated_by_queue_drain(self):
        source=(ROOT/'Core/Src/telemetry.c').read_text()
        function=source[source.index('SedsResult process_all_queues_timeout('):source.index('\nstatic SedsResult log_error_impl')]
        harness=r'''
#include <stdint.h>
#include <assert.h>
#define TELEMETRY_ENABLED 1
typedef int SedsResult;
#define SEDS_OK 0
#define SEDS_ERR -1
struct{void *r;}g_router={(void *)1};
unsigned g_daq_queue_service_error_count, discovery_polls, timesync_polls, drains;
int32_t g_daq_queue_service_last_result;
int fail,init_fail;
SedsResult init_telemetry_router(void){return init_fail?SEDS_ERR:SEDS_OK;}
SedsResult seds_router_drain_queues_with_timeout(void *r,uint32_t timeout){assert(r && timeout==1);++drains;return fail;}
SedsResult seds_router_process_all_queues_with_timeout(void *r,uint32_t timeout){++discovery_polls;++timesync_polls;return seds_router_drain_queues_with_timeout(r,timeout);}
''' + function + r'''
int main(void){
 discovery_polls=timesync_polls=1; /* Explicit board poll once per worker pass. */
 assert(process_all_queues_timeout(1)==SEDS_OK);
 assert(discovery_polls==1 && timesync_polls==1 && drains==1);
 fail=-14;assert(process_all_queues_timeout(1)==-14);
 assert(g_daq_queue_service_error_count==1 && g_daq_queue_service_last_result==-14);
 g_router.r=0;init_fail=1;assert(process_all_queues_timeout(1)==SEDS_ERR && drains==2);
 return 0;
}'''
        with tempfile.TemporaryDirectory() as d:
            binary=Path(d)/'queue-service'
            subprocess.run(['cc','-std=c11','-O2','-Wall','-Wextra','-Werror','-x','c','-','-o',str(binary)],input=harness,text=True,check=True)
            subprocess.run([str(binary)],check=True)
