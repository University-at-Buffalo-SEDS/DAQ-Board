"""The router owner publishes SEDSnet network time and never restarts it.

SEDSnet 6 steps a follower to its source's newer time-base epoch by itself,
so the board no longer compares its network clock with a station clock built
from GS stamps or restarts time sync when they disagree. The actual
telemetry_update_network_health() and heartbeat handler run with modeled SDK
calls.
"""
from pathlib import Path
import re,subprocess,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
def function(text,signature):
 i=text.index(signature);j=text.index('{',i);depth=1;k=j+1
 while depth:depth+=(text[k]=='{')-(text[k]=='}');k+=1
 return text[i:k]
def compile_and_run(test,code):
 with tempfile.TemporaryDirectory() as d:
  exe=Path(d)/'probe';out=subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-x','c','-','-o',str(exe)],input=code,text=True,capture_output=True)
  test.assertEqual(out.returncode,0,out.stderr);subprocess.run([str(exe)],check=True,timeout=10)

class NetworkClockPublication(unittest.TestCase):
 def test_network_time_is_published_without_a_station_clock(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  for removed in ('board_recovery_network_clock_resync_needed','board_recovery_observe_station_time','g_network_clock_resyncs'):
   self.assertNotIn(removed,source)
  body=re.search(r'static void telemetry_update_network_health\(SedsRouter \*router\)\s*\{.*?\n\}',source,re.S).group()
  code=r'''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;
#define SEDS_OK 0
static SedsRouter router;static uint64_t net;static bool valid;static uint64_t last_observed=1;
static uint32_t g_telemetry_timesync_valid,g_telemetry_discovery_seen=1,g_telemetry_network_ready;
static SedsResult seds_router_get_network_time_ms(SedsRouter*r,uint64_t*out){assert(r==&router);if(!valid)return -1;*out=net;return SEDS_OK;}
typedef struct{uint64_t utc;uint32_t tick;}daq_clock_cache_t;static daq_clock_cache_t g_acquisition_clock;
static uint32_t HAL_GetTick(void){return 7;}
static uint32_t __get_PRIMASK(void){return 0;}static void __disable_irq(void){}static void __set_PRIMASK(uint32_t m){(void)m;}
static daq_clock_cache_t daq_clock_cache_make(uint64_t utc,uint32_t tick){last_observed=utc;return (daq_clock_cache_t){utc,tick};}
'''+body+r'''
int main(void){
 (void)g_acquisition_clock;
 valid=false;telemetry_update_network_health(&router);assert(!g_telemetry_timesync_valid&&!g_telemetry_network_ready&&last_observed==0);
 valid=true;net=1794183600000ULL;telemetry_update_network_health(&router);assert(g_telemetry_timesync_valid&&g_telemetry_network_ready&&last_observed==net);
 /* A clock 4 h behind (a step back) is published as it is. */
 net-=14400000ULL;telemetry_update_network_health(&router);assert(last_observed==net);
 return 0;}
'''
  compile_and_run(self,code)
 def test_heartbeat_handler_only_feeds_the_probe(self):
  source=(ROOT/'Core/Src/telemetry.c').read_text()
  body=function(source,'static SedsResult telemetry_heartbeat_handler(')
  code=r'''#include <assert.h>
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;typedef int SedsRouter;typedef struct{uint64_t timestamp;}SedsPacketView;
#define SEDS_OK 0
static unsigned probes;
static SedsResult sim_probe_heartbeat_handler(const SedsPacketView*p,void*u){(void)p;(void)u;probes++;return SEDS_OK;}
'''+body+r'''
int main(void){SedsPacketView p={.timestamp=1794183600000ULL};
 assert(telemetry_heartbeat_handler(&p,NULL)==SEDS_OK&&probes==1);
 assert(telemetry_heartbeat_handler(NULL,NULL)==SEDS_OK&&probes==2);return 0;}
'''
  compile_and_run(self,code)
if __name__=='__main__':unittest.main()
